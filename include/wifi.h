#ifndef WIFI_H
#define WIFI_H

#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF ESP_IF_WIFI_AP
#define ESPNOW_CHANNEL 1              // wifi channel - LEGAL: use only 1, 6, or 11 for FCC compliance & reliability
#define ESPNOW_PMK "pmk1234567890123" // primary master key
#define ESPNOW_DATA_MAX_LEN 100       // max size of packet payload
#define ESPNOW_MAXDELAY (size_t)0xffffffff
#define ESPNOW_QUEUE_SIZE 6
#define ESP_NOW_ETH_ALEN 6 // length of MAC address

static uint8_t s_host_mac[ESP_NOW_ETH_ALEN] = {0xF4, 0x12, 0xFA, 0xFA, 0x11, 0xe1};
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_host_mac, ESP_NOW_ETH_ALEN) == 0)

const char *TAG = "WIFI";
static QueueHandle_t espnow_queue;

typedef enum
{
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct
{
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct
{
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union
{
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct
{
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

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
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];  // MAC address of destination device.
} espnow_send_param_t;

static void espnow_deinit(espnow_send_param_t *send_param);
static void wifi_init(void);
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *msg, int *len);
void espnow_data_prepare(espnow_send_param_t *send_param, uint8_t *data, int len);
static void espnow_init(espnow_send_param_t *send_param);
static void espnow_deinit(espnow_send_param_t *send_param);
int send_to_client(espnow_send_param_t *send_param, uint8_t *data, int len);

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
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t *mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL)
    {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }

    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
// params 1 and 2 are raw data info, params 3-n are data fields to populate
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *msg, int *len)
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
void espnow_data_prepare(espnow_send_param_t *send_param, uint8_t *data, int len)
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

static void espnow_init(espnow_send_param_t *send_param)
{
    // queue semaphore of espnow requests to handle with task
    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
        return;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* Set primary master key. */
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)ESPNOW_PMK));

    /* Add broadcast peer information to peer list. */
    // TODO: move this to client.c and host.c to specify which devices to connect to
    //       current implementation will try to connect the host to itself
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_host_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    memset(send_param, 0, sizeof(espnow_send_param_t));
    send_param->len = 0;
    memcpy(send_param->dest_mac, s_host_mac, ESP_NOW_ETH_ALEN);
    return;
}

// handles error by cleaning up param and deinitializing wifi
static void espnow_deinit(espnow_send_param_t *send_param)
{
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();
}

int send_to_client(espnow_send_param_t *send_param, uint8_t *data, int len)
{
    // if read data, forward to client
    printf("Sending to" MACSTR "\n", MAC2STR(send_param->dest_mac));
    if (len)
    {
        espnow_data_prepare(send_param, data, len);
        esp_err_t er = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
        if (er != ESP_OK)
        {
            ESP_LOGE(TAG, "Send error: %x", er);
            return -1;
        }
    }
    return 0;
}
#endif