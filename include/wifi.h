#ifndef WIFI_H
#define WIFI_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_now.h"

#define MBOT_PARAM_DEFS_H
// WIFI_MODE_AP
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF ESP_IF_WIFI_AP
#define ESPNOW_CHANNEL 1              // wifi channel - LEGAL: use only 1, 6, or 11 for FCC compliance & reliability
#define ESPNOW_PMK "pmk1234567890123" // primary master key
#define ESPNOW_DATA_MAX_LEN 250       // max size of packet payload
#define ESPNOW_MAXDELAY (size_t)0xffffffff
#define ESPNOW_QUEUE_SIZE 6
#define MAC_ADDR_LEN 6

// uint8_t s_host_mac[MAC_ADDR_LEN] = {0xF4, 0x12, 0xFA, 0xFA, 0x11, 0xe1};
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_host_mac, MAC_ADDR_LEN) == 0)

QueueHandle_t espnow_send_queue;
QueueHandle_t espnow_recv_queue;

const char *SEND_CB_TAG = "SEND_CB";
const char *RECV_CB_TAG = "RECV_CB";
const char *PARSE_TAG = "PARSE";
const char *PREPARE_TAG = "PREPARE";
const char *DATA_SEND_TAG = "DATA_SEND";

/* User defined field of ESPNOW data. */
// this should be the same between client and client
typedef struct
{
    uint8_t len;        // length of payload
    uint16_t crc;       // CRC16 value of ESPNOW data.
    uint8_t payload[0]; // Real payload of ESPNOW data.
} __attribute__((packed)) comm_espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct
{
    int len;                             // Length of ESPNOW data to be sent (buffer), unit: byte.
    uint8_t buffer[ESPNOW_DATA_MAX_LEN]; // Buffer pointing to ESPNOW data.
    uint8_t dest_mac[MAC_ADDR_LEN];      // MAC address of destination device.
} espnow_send_param_t;

typedef struct
{
    uint8_t mac_addr[MAC_ADDR_LEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_t;

typedef struct
{
    uint8_t mac_addr[MAC_ADDR_LEN];
    esp_now_send_status_t status;
} espnow_event_send_t;

void wifi_init(void);
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *msg, uint16_t *len);
int espnow_data_prepare(espnow_send_param_t *send_param, uint8_t *data, int len);
int espnow_data_send(uint8_t *mac_addr, uint8_t *data, int len);
void espnow_init();
void espnow_deinit();

/* WiFi should start before using ESPNOW */
void wifi_init(void)
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
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL)
    {
        ESP_LOGE(SEND_CB_TAG, "Send cb arg error");
        return;
    }

    espnow_event_send_t evt;
    memcpy(evt.mac_addr, mac_addr, MAC_ADDR_LEN);
    evt.status = status;

    // Publish send status to queue
    if (xQueueSend(espnow_send_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(SEND_CB_TAG, "Send send queue fail");
    }
}

void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info->src_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(RECV_CB_TAG, "Receive cb arg error");
        return;
    }

    // Copy source mac address
    espnow_event_recv_t evt;
    memcpy(evt.mac_addr, recv_info->src_addr, MAC_ADDR_LEN);
    evt.data = malloc(len);
    if (evt.data == NULL)
    {
        ESP_LOGE(RECV_CB_TAG, "Malloc receive data fail");
        return;
    }

    // Copy data
    memcpy(evt.data, data, len);
    evt.data_len = len;

    // ESP_LOGI(RECV_CB_TAG, "Received packet in callback with len: %d", len);

    // Publish to queue
    if (xQueueSend(espnow_recv_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(RECV_CB_TAG, "Send receive queue fail");
    }
}

/* Parse received ESPNOW data. */
// params 1 and 2 are raw data info, params 3-n are data fields to populate
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *msg, uint16_t *len)
{
    // Cast data byte array to buffer struct
    comm_espnow_data_t *buf = (comm_espnow_data_t *)data;

    // Check if data length is correct
    if (data_len != sizeof(comm_espnow_data_t) + buf->len)
    {
        ESP_LOGE(PARSE_TAG, "Receive ESPNOW data has wrong length, len:%d, expected len:%d", data_len, sizeof(comm_espnow_data_t) + buf->len);
        return -1;
    }

    // Check if CRC matches
    uint16_t crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf->payload, buf->len);
    if (crc_cal != buf->crc)
        return -1;

    // Copy payload to msg
    memcpy(msg, buf->payload, buf->len);
    *len = buf->len;

    return 0;
}

// Prepare ESPNOW data to be sent.
// data: data payload to be sent ; len: length of data in Bytes
int espnow_data_prepare(espnow_send_param_t *send_param, uint8_t *data, int len)
{
    // Check if length is valid
    if (len > ESPNOW_DATA_MAX_LEN)
    {
        ESP_LOGE(PREPARE_TAG, "Data too long! %d > %d", len, ESPNOW_DATA_MAX_LEN);
        return -1;
    }
    int send_param_len = len + sizeof(comm_espnow_data_t);
    send_param->len = send_param_len;

    // Set memory for sending
    memset(send_param->buffer, 0, send_param->len);

    // Cast buffer to struct
    comm_espnow_data_t *buf = (comm_espnow_data_t *)send_param->buffer;

    // Fill in fields
    buf->len = len;
    memcpy(buf->payload, data, len);
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf->payload, buf->len);
    return 0;
}

int espnow_data_send(uint8_t *mac_addr, uint8_t *data, int len)
{
    // Prepare data to send
    espnow_send_param_t send_param;
    memcpy(send_param.dest_mac, mac_addr, MAC_ADDR_LEN);
    int ret = espnow_data_prepare(&send_param, data, len);
    if (ret < 0) return -1;

    // Send data
    esp_now_send(send_param.dest_mac, send_param.buffer, send_param.len);

    // Check to see if send was successful
    espnow_event_send_t *send_evt;
    if (xQueueReceive(espnow_send_queue, &send_evt, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGE(DATA_SEND_TAG, "Send failed");
        return -1;
    }
    return 0;
}

void espnow_init()
{
    // Initialize ESPNOW and register sending and receiving callback function.
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // Create receive and send queues
    espnow_send_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_send_t));
    espnow_recv_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_recv_t));
    if (espnow_send_queue == NULL || espnow_recv_queue == NULL)
    {
        ESP_LOGE(PREPARE_TAG, "Create mutex fail");
        return;
    }

    // Set primary master key.
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)ESPNOW_PMK));

    return;
}

// handles error by cleaning up param and deinitializing wifi
// TODO: this function already exists in espnow.h...
void espnow_deinit()
{
    vSemaphoreDelete(espnow_send_queue);
    vSemaphoreDelete(espnow_recv_queue);
    esp_now_deinit();
}

#endif