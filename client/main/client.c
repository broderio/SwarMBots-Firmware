#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "client.h"

#define INCLUDE_vTaskDelay 1

#define GPIO_RECV 3
#define GPIO_SEND 2
#define GPIO_MOSI 11
#define GPIO_MISO 13
#define GPIO_SCLK 12
#define GPIO_CS 10

#define ESPNOW_MAXDELAY (size_t) 0xffffffff

static const char *TAG = "client";

static QueueHandle_t s_client_espnow_queue;
static QueueHandle_t packet_send_queue;
static uint8_t s_host_mac[ESP_NOW_ETH_ALEN] = {0xF4, 0x12, 0xFA, 0xFA, 0x11, 0xe1};

static void client_espnow_deinit(client_espnow_send_param_t *send_param);

/* WiFi should start before using ESPNOW */
static void client_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM)); // sets to use only ram for storage
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));    // AP
    ESP_ERROR_CHECK(esp_wifi_start());

    // NOTE: may need to add channel negotiation logic
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void client_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    client_espnow_event_t evt;
    client_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = CLIENT_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_client_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void client_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    client_espnow_event_t evt;
    client_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t *mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = CLIENT_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL)
    {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_client_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
// params 1 and 2 are raw data info, params 3-n are data fields to populate
int client_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *msg, int *len)
{
    comm_espnow_data_t *buf = (comm_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(comm_espnow_data_t))
    {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }
    if (data_len != sizeof(comm_espnow_data_t) + buf->len)
    {
        ESP_LOGE(TAG, "Receive ESPNOW data has wrong length, len:%d, expected len:%d", data_len, sizeof(comm_espnow_data_t) + buf->len);
        return -1;
    }

    *len = buf->len < ESPNOW_DATA_MAX_LEN ? buf->len : ESPNOW_DATA_MAX_LEN;
    memcpy(msg, buf->payload, *len);

    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc)
    {
        return 0;
    }

    return -1;
}

// Prepare ESPNOW data to be sent.
// data: data payload to be sent ; len: length of data in Bytes
void client_espnow_data_prepare(client_espnow_send_param_t *send_param, uint8_t *data, int len)
{
    comm_espnow_data_t *buf = (comm_espnow_data_t *)send_param->buffer;

    send_param->len = len + sizeof(comm_espnow_data_t);

    assert(len <= ESPNOW_DATA_MAX_LEN);

    buf->crc = 0;
    buf->len = len;

    /* Only fill payload if there is room to fit it */
    if (sizeof(comm_espnow_data_t) + buf->len <= send_param->len)
    {
        memcpy(buf->payload, data, len);
    }

    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len); // compute checksum to send with packet
}

static void client_espnow_task(void *pvParameter)
{
    // set important variables
    client_espnow_event_t evt;
    client_espnow_event_t dat;
    uint8_t recv_data[ESPNOW_DATA_MAX_LEN + 1];
    int recv_len;
    int ret;
    // bool hasRecv = false;
    recv_data[0] = '\0';
    // wait 5 seconds
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    // recover send param
    client_espnow_send_param_t *send_param = (client_espnow_send_param_t *)pvParameter;

    // send empty message to start comms
    client_espnow_data_prepare(send_param, NULL, 0);
    esp_err_t er = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
    if (er != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error: %d", er);
        client_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    // wait for response on repeat
    while (xQueueReceive(s_client_espnow_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        switch (evt.id)
        {
        // send was (or wasn't) received
        case CLIENT_ESPNOW_SEND_CB:
        {
            // client_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

            // if (!hasRecv) {

            //     ESP_LOGI(TAG, "Resending broadcast data");

            //     //send empty message to start comms
            //     esp_err_t er = esp_now_send(send_param->dest_mac, send_param->buffer, 1);
            //     if (er != ESP_OK) {
            //         ESP_LOGE(TAG, "Send error: %d", er);
            //         client_espnow_deinit(send_param);
            //         vTaskDelete(NULL);
            //     }
            // }
            // don't need to do anything after a successful send

            break;
        }
        case CLIENT_ESPNOW_RECV_CB:
        {
            client_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
            ret = client_espnow_data_parse(recv_cb->data, recv_cb->data_len, recv_data, &recv_len);

            free(recv_cb->data); // free data field allocated in receive callback function

            if (ret == 0)
            {
                /* If MAC address does not exist in peer list, add it to peer list and begin sending it messages*/
                if (esp_now_is_peer_exist(recv_cb->mac_addr) == false)
                {
                    // add peer
                    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                    if (peer == NULL)
                    {
                        ESP_LOGE(TAG, "Malloc peer information fail");
                        client_espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                    memset(peer, 0, sizeof(esp_now_peer_info_t));
                    peer->channel = ESPNOW_CHANNEL;
                    peer->ifidx = ESPNOW_WIFI_IF;
                    peer->encrypt = false;
                    memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                    ESP_ERROR_CHECK(esp_now_add_peer(peer));
                    free(peer);

                    ESP_LOGI(TAG, "Received comm from unknown device. Saving as a peer.");
                }

                // ================= Do Work With Received Data =================

                recv_data[recv_len] = '\0'; // add null terminator since we'll be interpreting this as a string
                dat.info.recv_cb.data = recv_data;
                ESP_LOGI(TAG, "Received data from Host: %s", (char *)recv_data);
                if (xQueueSend(packet_send_queue, &dat, ESPNOW_MAXDELAY) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Send queue fail");
                }

                client_espnow_data_prepare(send_param, (uint8_t *)"Ack", 3);

                esp_err_t er = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
                if (er != ESP_OK)
                {
                    ESP_LOGE(TAG, "Send error: %d", er);
                    client_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }

                // =================== End Received Data Work ===================
            }
            else
            {
                ESP_LOGI(TAG, "Receive error data from: " MACSTR "", MAC2STR(recv_cb->mac_addr));
            }
            break;
        }
        default:
            ESP_LOGE(TAG, "Callback type error: %d", evt.id);
            break;
        }
    }
}

static esp_err_t client_espnow_init(void)
{

    // queue semaphore of espnow requests to handle with task
    s_client_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(client_espnow_event_t));
    if (s_client_espnow_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(client_espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(client_espnow_recv_cb));

    /* Set primary master key. */
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)ESPNOW_PMK));

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_client_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_host_mac, ESP_NOW_ETH_ALEN); 
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    /* Initialize info about data to send. */
    client_espnow_send_param_t *send_param;

    send_param = malloc(sizeof(client_espnow_send_param_t));
    if (send_param == NULL)
    {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_client_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(client_espnow_send_param_t));
    send_param->len = 0;
    send_param->buffer = malloc(ESPNOW_DATA_MAX_LEN);
    if (send_param->buffer == NULL)
    {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_client_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_host_mac, ESP_NOW_ETH_ALEN);

    xTaskCreate(client_espnow_task, "client_espnow_task", 4096, send_param, 4, NULL);

    return ESP_OK;
}

// handles error by cleaning up param and deinitializing wifi
static void client_espnow_deinit(client_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_client_espnow_queue);
    esp_now_deinit();
}


// Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    if (trans->tx_buffer == NULL)
        gpio_set_level(GPIO_RECV, 1);
    else
        gpio_set_level(GPIO_SEND, 1);
}

// Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    if (trans->tx_buffer == NULL)
        gpio_set_level(GPIO_RECV, 0);
    else
        gpio_set_level(GPIO_SEND, 0);
}

void send_task(void *args)
{
    esp_err_t ret;
    int n = 0;
    spi_slave_transaction_t t;
    client_espnow_event_t dat;
    // TickType_t xLastWakeTime;

    while (1)
    {
        if (xQueueReceive(packet_send_queue, &dat, ESPNOW_MAXDELAY) != pdTRUE) 
        {
            printf("Error receiving from queue\n");
            continue;
        }
        // xLastWakeTime = xTaskGetTickCount();
        t.length = dat.info.recv_cb.data_len * 8;
        t.tx_buffer = dat.info.recv_cb.data;
        t.rx_buffer = NULL;
        printf("Received packet. Waiting for lock...\n");
        ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
        if (ret != ESP_OK)
        {
            printf("Error transmitting: 0x%x\n", ret);
            continue;
        }
        printf("Sent %zu bytes\n", t.trans_len / 8);
        ++n;
        // xTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
    }
}

void recv_task(void *args)
{
    esp_err_t ret;
    int n = 0;
    WORD_ALIGNED_ATTR uint8_t recvbuf[84];

    spi_slave_transaction_t t;
    // TickType_t xLastWakeTime;
    while (1)
    {
        // xLastWakeTime = xTaskGetTickCount();
        t.length = 84 * 8;
        t.tx_buffer = NULL;
        t.rx_buffer = recvbuf;

        printf("Waiting for packet...\n");
        ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
        if (ret != ESP_OK)
        {
            printf("Error transmitting: 0x%x\n", ret);
            continue;
        }

        if (t.trans_len > t.length)
            continue;

        // TODO: send t.tx_buffer over wifi

        ++n;
        // xTaskDelayUntil(&xLastWakeTime, 5 / portTICK_PERIOD_MS);
    }
}

int client_spi_init(void)
{
    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    // Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 6,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb};

    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    // Initialize SPI slave interface
    esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        printf("Error initializing SPI slave: %d\n", ret);
        return ret;
    }

    // Initialize GPIO handshake for sending messages to MBoard
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << GPIO_SEND);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        printf("Error configuring GPIO: %d\n", ret);
        return ret;
    }

    // Initialize GPIO handshake for receiving messages from MBoard
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << GPIO_RECV);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        printf("Error configuring GPIO: %d\n", ret);
        return ret;
    }
    // printf("Minimum free heap size: %lu bytes\n", esp_get_minimum_free_heap_size());
    return ret;
}

// Main application
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    printf("Initializing WiFi...\n");
    client_wifi_init();

    printf("Initializing ESP-NOW...\n");
    client_espnow_init();

    printf("Initializing SPI...\n");
    client_spi_init();

    // Mutex for SPI transmissions

    // Create tasks
    TaskHandle_t recv_task_handle, send_task_handle;

    packet_send_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(client_espnow_event_t));
    if (packet_send_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex.");
        return;
    }

    xTaskCreatePinnedToCore(recv_task, "recv_task", 2048 * 4, NULL, 5, &recv_task_handle, 0);
    xTaskCreatePinnedToCore(send_task, "send_task", 2048 * 4, NULL, 5, &send_task_handle, 1);

    // Give semaphore to start SPI transmissions
    // xSemaphoreGive(spi_mutex);
}