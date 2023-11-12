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
#include "spi.h"
#include "lcm/mbot_lcm_msgs_serial.h"
#include "lcm/comms.h"

SemaphoreHandle_t spi_mutex;
bool found_host = false;
uint8_t host_mac_addr[MAC_ADDR_LEN];
SemaphoreHandle_t wifi_ready;

static void espnow_recv_task(void *args)
{
    espnow_event_recv_t evt;
    uint8_t msg[ESPNOW_DATA_MAX_LEN];
    uint16_t data_len;
    int ret;

    uint8_t mac[MAC_ADDR_LEN];
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_AP, mac);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "Could not get mac address, error code %d", err);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Print client mac address
    ESP_LOGI(TAG, "Client MAC: " MACSTR, MAC2STR(mac));

    spi_slave_transaction_t t;
    while (xQueueReceive(espnow_recv_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        // Parse incoming packet
        ret = espnow_data_parse(evt.data, evt.data_len, msg, &data_len);
        free(evt.data);
        printf("receiving\n");
        // Check if data is invalid
        if (ret != 0) {
            ESP_LOGE(TAG, "Received invalid data");
            continue;
        }
        // Check if mac address is paired
        if (!found_host && !esp_now_is_peer_exist(evt.mac_addr))
        {
            // Allocate peer
            esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
            if (peer == NULL)
            {
                ESP_LOGE(TAG, "Malloc peer information fail");
            }
            memset(peer, 0, sizeof(esp_now_peer_info_t));
            peer->channel = ESPNOW_CHANNEL;
            peer->ifidx = ESPNOW_WIFI_IF;
            peer->encrypt = false;
            memcpy(peer->peer_addr, evt.mac_addr, MAC_ADDR_LEN);

            // Add peer
            ESP_ERROR_CHECK(esp_now_add_peer(peer));
            free(peer);
            found_host = true;
            memcpy(host_mac_addr, evt.mac_addr, MAC_ADDR_LEN);
            ESP_LOGI(TAG, "Found host (MAC: "MACSTR")", MAC2STR(evt.mac_addr));
            xSemaphoreGive(wifi_ready);
        }

        // Populate SPI packet
        t.length = data_len * 8;
        t.tx_buffer = msg;
        t.rx_buffer = NULL;
        t.trans_len = 0; //REMOVE LATER

        if (xSemaphoreTake(spi_mutex, portMAX_DELAY) == pdTRUE) {
            printf("esp receive took semaphore\n");
            ret = ESP_OK; 
            spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
            xSemaphoreGive(spi_mutex);
            if (ret != ESP_OK)
            {
                ESP_LOGI(TAG, "Error transmitting: 0x%x", ret);
                continue;
            }
        }
        printf("esp receive gave semaphore\n");
        ESP_LOGI(TAG, "Sent %zu bytes\n", t.trans_len / 8);
    }
}

void espnow_send_task(void *args)
{
    esp_err_t ret;
    spi_slave_transaction_t t;
    WORD_ALIGNED_ATTR uint8_t recvbuf[84];
    xSemaphoreTake(wifi_ready, portMAX_DELAY);
    while (1)
    {
        t.length = 84 * 8;
        t.tx_buffer = NULL;
        t.rx_buffer = recvbuf;
        t.trans_len = 0; //REMOVE LATEr

        if (xSemaphoreTake(spi_mutex, portMAX_DELAY) == pdTRUE) {
            printf("esp send took semaphore\n");
            ret = ESP_OK; 
            spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
            xSemaphoreGive(spi_mutex);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Error reading SPI transmission");
                continue;
            }
        }
        printf("esp send gave semaphore\n");

        if (t.trans_len && t.trans_len > t.length)
            continue;
        espnow_send_param_t send_param;
        //printf("sending to " MACSTR"\n", MAC2STR(send_param.dest_mac));
        memcpy(send_param.dest_mac, host_mac_addr, MAC_ADDR_LEN);
        espnow_data_prepare(&send_param, t.rx_buffer, t.trans_len / 8);
        esp_now_send(send_param.dest_mac, send_param.buffer, send_param.len);

        // Check to see if send was successful
        espnow_event_send_t *send_evt;
        if (xQueueReceive(espnow_send_queue, &send_evt, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Send failed");
        }
    }
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

    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init();

    ESP_LOGI(TAG, "Initializing ESP-NOW...");
    espnow_init();

    ESP_LOGI(TAG, "Initializing SPI...");
    spi_init();

    // Create mutex for SPI
    spi_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(spi_mutex);
    wifi_ready = xSemaphoreCreateBinary();

    // Create tasks
    TaskHandle_t recv_task_handle, send_task_handle;
    xTaskCreate(espnow_send_task, "espnow_send_task", 2048 * 4, NULL, 4, &send_task_handle);
    xTaskCreate(espnow_recv_task, "espnow_recv_task", 2048 * 4, NULL, 4, &recv_task_handle);
}