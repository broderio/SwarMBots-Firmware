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
#define ESPNOW_DATA_MAX_LEN 100       // max size of packet payload
#define ESPNOW_MAXDELAY (size_t)0xffffffff
#define ESPNOW_QUEUE_SIZE 6
#define MAC_ADDR_LEN 6

static uint8_t s_host_mac[MAC_ADDR_LEN] = {0xF4, 0x12, 0xFA, 0xFA, 0x11, 0xe1};
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_host_mac, MAC_ADDR_LEN) == 0)

static esp_now_peer_info_t peers[8];
static int peer_num = 0;

const char *TAG = "WIFI";
static QueueHandle_t espnow_send_queue;
static QueueHandle_t espnow_recv_queue;

typedef struct
{
    uint8_t mac_addr[MAC_ADDR_LEN];
    esp_now_send_status_t status;
} espnow_event_send_t;

typedef struct
{
    uint8_t mac_addr[MAC_ADDR_LEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_t;

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
    int len;                        // Length of ESPNOW data to be sent (buffer), unit: byte.
    uint8_t *buffer;              // Buffer pointing to ESPNOW data.
    uint8_t dest_mac[MAC_ADDR_LEN]; // MAC address of destination device.
} espnow_send_param_t;

static void wifi_init(void);
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t **msg, uint16_t *len);
void espnow_data_prepare(espnow_send_param_t *send_param, uint8_t *data, int len);
static void espnow_init();
static void espnow_deinit();

/* WiFi should start before using ESPNOW */
static void wifi_init(void)
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
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    espnow_event_send_t evt;
    memcpy(evt.mac_addr, mac_addr, MAC_ADDR_LEN);
    evt.status = status;

    // Publish send status to queue
    if (xQueueSend(espnow_send_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info->src_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    // Copy source mac address
    espnow_event_recv_t *evt = (espnow_event_recv_t *)malloc(sizeof(espnow_event_recv_t));
    memcpy(evt->mac_addr, recv_info->src_addr, MAC_ADDR_LEN);
    evt->data = malloc(len);
    if (evt->data == NULL)
    {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }

    // Copy data
    memcpy(evt->data, data, len);
    evt->data_len = len;

    // Publish to queue
    if (xQueueSend(espnow_recv_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send receive queue fail");
    }
}

/* Parse received ESPNOW data. */
// params 1 and 2 are raw data info, params 3-n are data fields to populate
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t **msg, uint16_t *len)
{
    // Cast data byte array to buffer struct
    comm_espnow_data_t *buf = (comm_espnow_data_t *)data;

    // Check if data length is correct
    if (data_len != sizeof(comm_espnow_data_t) + buf->len)
    {
        ESP_LOGE(TAG, "Receive ESPNOW data has wrong length, len:%d, expected len:%d", data_len, sizeof(comm_espnow_data_t) + buf->len);
        return -1;
    }

    // Allocate message data
    *msg = malloc(buf->len);

    // Check if CRC matches
    uint16_t crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);
    if (crc_cal != buf->crc)
        return -1;

    // Copy payload to msg
    memcpy(*msg, buf->payload, buf->len);
    *len = buf->len;

    return 0;
}

// Prepare ESPNOW data to be sent.
// data: data payload to be sent ; len: length of data in Bytes
void espnow_data_prepare(espnow_send_param_t *send_param, uint8_t *data, int len)
{
    // Check if length is valid
    send_param->len = len + sizeof(comm_espnow_data_t);
    assert(send_param->len <= ESPNOW_DATA_MAX_LEN);

    // Allocate memory for sending
    send_param->buffer = malloc(send_param->len);
    if (send_param->buffer == NULL)
    {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        return;
    }
    memset(send_param->buffer, 0, send_param->len);

    // Cast buffer to struct
    comm_espnow_data_t *buf = (comm_espnow_data_t *)send_param->buffer;

    // Fill in fields
    buf->len = len;
    memcpy(buf->payload, data, len);
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void espnow_init()
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
        ESP_LOGE(TAG, "Create mutex fail");
        return;
    }

    // Set primary master key.
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)ESPNOW_PMK));

    return;
}

// handles error by cleaning up param and deinitializing wifi
static void espnow_deinit()
{
    vSemaphoreDelete(espnow_send_queue);
    vSemaphoreDelete(espnow_recv_queue);
    esp_now_deinit();
}

#endif