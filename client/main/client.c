#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include <esp_timer.h>

#include "client.h"
#include "wifi.h"
#include "lcm/mbot_lcm_msgs_serial.h"
#include "lcm/comms.h"

static void client_espnow_task(void *pvParameter)
{
    // set important variables
    espnow_event_t evt;
    espnow_event_t dat;
    uint8_t recv_data[ESPNOW_DATA_MAX_LEN + 1];
    int recv_len;
    int ret;
    // bool hasRecv = false;
    recv_data[0] = '\0';
    // wait 5 seconds
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    // recover send param
    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;

    // send empty message to start comms
    espnow_data_prepare(send_param, NULL, 0);
    esp_err_t er = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
    if (er != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error: %d", er);
        espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    // wait for response on repeat
    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        switch (evt.id)
        {
        // send was (or wasn't) received
        case ESPNOW_SEND_CB:
        {
            // espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

            // if (!hasRecv) {

            //     ESP_LOGI(TAG, "Resending broadcast data");

            //     //send empty message to start comms
            //     esp_err_t er = esp_now_send(send_param->dest_mac, send_param->buffer, 1);
            //     if (er != ESP_OK) {
            //         ESP_LOGE(TAG, "Send error: %d", er);
            //         espnow_deinit(send_param);
            //         vTaskDelete(NULL);
            //     }
            // }
            // don't need to do anything after a successful send

            break;
        }
        case ESPNOW_RECV_CB:
        {
            espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
            ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, recv_data, &recv_len);

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
                        espnow_deinit(send_param);
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
                dat.info.recv_cb.data_len = recv_len;
                ESP_LOGI(TAG, "Received data from Host: %s", (char *)recv_data);
                if (xQueueSend(spi_send_queue, &dat, ESPNOW_MAXDELAY) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Send queue fail");
                }

                espnow_data_prepare(send_param, (uint8_t *)"Ack", 3);

                esp_err_t er = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
                if (er != ESP_OK)
                {
                    ESP_LOGE(TAG, "Send error: %d", er);
                    espnow_deinit(send_param);
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
    spi_slave_transaction_t t;
    espnow_event_t dat;
    while (1)
    {
        if (xQueueReceive(spi_send_queue, &dat, ESPNOW_MAXDELAY) != pdTRUE) 
        {
            printf("Error receiving from queue\n");
            continue;
        }
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
    }
}

void recv_task(void *args)
{
    esp_err_t ret;
    spi_slave_transaction_t t;
    WORD_ALIGNED_ATTR uint8_t recvbuf[84];
    while (1)
    {
        t.length = 84 * 8;
        t.tx_buffer = NULL;
        t.rx_buffer = recvbuf;

        //printf("Waiting for packet...\n");
        ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
        if (ret != ESP_OK)
        {
            printf("Error transmitting: 0x%x\n", ret);
            continue;
        }

        if (t.trans_len > t.length)
            continue;

        // TODO: send t.tx_buffer over wifi
    }
}

int spi_init(void)
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
    wifi_init();

    printf("Initializing ESP-NOW...\n");

    espnow_send_param_t* send_param;
    send_param = espnow_init();
    if (send_param == NULL){
        ESP_LOGI(TAG, "Send param allocation failed");
        return;
    }

    printf("Initializing SPI...\n");
    spi_init();

    // Create tasks
    TaskHandle_t recv_task_handle, send_task_handle;

    spi_send_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (spi_send_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex.");
        return;
    }

    xTaskCreate(client_espnow_task, "host_espnow_task", 4096, (void*)send_param, 4, NULL);
    xTaskCreate(recv_task, "recv_task", 2048 * 4, NULL, 5, &recv_task_handle);
    xTaskCreate(send_task, "send_task", 2048 * 4, NULL, 5, &send_task_handle);
}