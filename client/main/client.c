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

const char *ESPNOW_SEND_TAG = "ESPNOW_SEND_TASK";
const char *ESPNOW_RECV_TAG = "ESPNOW_RECV_TASK";
const char *MAIN_TAG = "APP_MAIN";


SemaphoreHandle_t spi_mutex;
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
        ESP_LOGI(ESPNOW_RECV_TAG, "Could not get mac address, error code %d", err);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Print client mac address
    ESP_LOGI(ESPNOW_RECV_TAG, "Client MAC: " MACSTR, MAC2STR(mac));

    // Wait for first message from host
    ESP_LOGI(ESPNOW_RECV_TAG, "Waiting for host...");
    connect_to_host(host_mac_addr);
    ESP_LOGI(ESPNOW_RECV_TAG, "Connected to host.");
    xSemaphoreGive(wifi_ready);

    // Begin receiving messages and sending them over SPI
    spi_slave_transaction_t t;
    while (xQueueReceive(espnow_recv_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        // Parse incoming packet
        ret = espnow_data_parse(evt.data, evt.data_len, msg, &data_len);
        free(evt.data);
        ESP_LOGI(ESPNOW_RECV_TAG, "Received message.");

        // Check if data is invalid
        if (ret != 0) {
            ESP_LOGE(ESPNOW_RECV_TAG, "Received invalid data");
            continue;
        }

        // Populate SPI packet
        t.length = data_len * 8;
        t.tx_buffer = msg;
        t.rx_buffer = NULL;
        t.trans_len = 0;

        if (xSemaphoreTake(spi_mutex, portMAX_DELAY) == pdTRUE) {
            ret = ESP_OK; 
            spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
            xSemaphoreGive(spi_mutex);
            if (ret != ESP_OK)
            {
                ESP_LOGI(ESPNOW_RECV_TAG, "Error transmitting: 0x%x", ret);
                continue;
            }
        }
        ESP_LOGI(ESPNOW_RECV_TAG, "Sent %zu bytes\n", t.trans_len / 8);
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
            ret = ESP_OK; 
            spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
            xSemaphoreGive(spi_mutex);
            if (ret != ESP_OK)
            {
                ESP_LOGE(ESPNOW_SEND_TAG, "Error reading SPI transmission");
                continue;
            }
        }

        if (t.trans_len > t.length) {
            ESP_LOGE(ESPNOW_SEND_TAG, "Received more bytes than expected.");
            continue;
        }

        espnow_data_send(host_mac_addr, t.rx_buffer, t.trans_len / 8);
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

    ESP_LOGI(MAIN_TAG, "Initializing WiFi...");
    wifi_init();

    ESP_LOGI(MAIN_TAG, "Initializing ESP-NOW...");
    espnow_init();

    ESP_LOGI(MAIN_TAG, "Initializing SPI...");
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