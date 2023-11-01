#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_sleep.h"
#include "esp_mac.h"
#include "esp_random.h"

#define MY_ESPNOW_WIFI_MODE     WIFI_MODE_STA
#define ESP_NOW_ETH_ALEN        6
#define MY_ESPNOW_WIFI_IF       ESP_IF_WIFI_STA
#define ESPNOW_MAXDELAY         (size_t)0xffffffff
#define MY_RECEIVER_MAC         {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#define MY_ESPNOW_PMK           "pmk1234567890123"
#define MY_ESPNOW_CHANNEL       1

static const char *TAG = "CLIENT_TEST";

typedef struct __attribute__((packed)) {
    uint32_t random_value;
    uint32_t index;
} my_data_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    my_data_t data;
} recv_packet_t;

static QueueHandle_t recv_packet_queue;
static QueueHandle_t send_status_queue;

// This function is called after the data is sent
// It will set a bit in the event group to indicate success or failure
void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }
    if (xQueueSend(send_status_queue, &status, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Queue full, discarded");
        return;
    }
}

// This function is called after the data is received
// It will create a packet and store it in the receive queue
void recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    static recv_packet_t recv_packet;
    uint8_t *mac_addr = recv_info->src_addr;

    ESP_LOGI(TAG, "%d bytes incoming from " MACSTR, len, MAC2STR(mac_addr));
    
    if(len != sizeof(my_data_t))
    {
        ESP_LOGE(TAG, "Unexpected data length: %d != %u", len, sizeof(my_data_t));
        return;
    }

    memcpy(&recv_packet.mac_addr, mac_addr, sizeof(recv_packet.mac_addr));
    memcpy(&recv_packet.data, data, len);
    if (xQueueSend(recv_packet_queue, &recv_packet, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Queue full, discarded");
        return;
    }
}

void init_espnow(void)
{
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(MY_ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));
    ESP_ERROR_CHECK(esp_now_set_pmk((const uint8_t *)MY_ESPNOW_PMK));

    const esp_now_peer_info_t broadcast_destination = {
        .peer_addr = MY_RECEIVER_MAC,
        .channel = MY_ESPNOW_CHANNEL,
        .ifidx = MY_ESPNOW_WIFI_IF
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_destination));
}

void recv_task(void* args)
{
    static recv_packet_t recv_packet;

    ESP_LOGI(TAG, "Listening");
    while(1)
    {
        if(xQueueReceive(recv_packet_queue, &recv_packet, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        ESP_LOGI(TAG, "Data from "MACSTR": Random Value - %lu, Index - %lu",
                MAC2STR(recv_packet.mac_addr), 
                recv_packet.data.random_value, 
                recv_packet.data.index);
    }
}

void send_task(void* args)
{
    const uint8_t destination_mac[] = MY_RECEIVER_MAC;
    static my_data_t data;
    int i = 0;
    while (1) {
        // Populate the data to send
        data.random_value = esp_random();
        data.index = i++;

        // Send the data
        ESP_LOGI(TAG, "Sending %u bytes to " MACSTR, sizeof(data), MAC2STR(destination_mac));
        esp_err_t err = esp_now_send(destination_mac, (uint8_t*)&data, sizeof(data));
        if(err != ESP_OK)
        {
            ESP_LOGE(TAG, "Send error (%d)", err);
            continue;
        }

        // Wait for callback function to set status
        esp_now_send_status_t status;
        if(xQueueReceive(send_status_queue, &status, portMAX_DELAY) != pdTRUE)
        {
            ESP_LOGE(TAG, "Send timeout");
            continue;
        }
        if (status != ESP_NOW_SEND_SUCCESS) {
            ESP_LOGE(TAG, "Send failed");
            continue;
        }

        ESP_LOGI(TAG, "Sent!");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    recv_packet_queue = xQueueCreate(10, sizeof(recv_packet_t));
    send_status_queue = xQueueCreate(10, sizeof(esp_now_send_status_t));

    init_espnow();

    xTaskCreate(recv_task, "recv_task", 2048, NULL, 4, NULL);
    xTaskCreate(send_task, "send_task", 2048, NULL, 4, NULL);
}