#ifndef HOST_AP_H
#define HOST_AP_H

#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP

#define ESPNOW_CHANNEL          1                   //wifi channel - LEGAL: use only 1, 6, or 11 for FCC compliance & reliability
#define ESPNOW_PMK              "pmk1234567890123"  //primary master key
#define ESPNOW_DATA_MAX_LEN     100                 //max size of packet payload

#define ESPNOW_QUEUE_SIZE       6

#define UART_PORT_NUM           0

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_host_client_mac, ESP_NOW_ETH_ALEN) == 0)

typedef enum {
    HOST_ESPNOW_SEND_CB,
    HOST_ESPNOW_RECV_CB,
} host_espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} host_espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} host_espnow_event_recv_cb_t;

typedef union {
    host_espnow_event_send_cb_t send_cb;
    host_espnow_event_recv_cb_t recv_cb;
} host_espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    host_espnow_event_id_t id;
    host_espnow_event_info_t info;
} host_espnow_event_t;

/* User defined field of ESPNOW data. */
//this should be the same between host and client
typedef struct {
    uint8_t len;                         //true if the package contains info
    uint16_t crc;                         //CRC16 value of ESPNOW data.                             checksum
    uint8_t payload[0];                   //Real payload of ESPNOW data.
} __attribute__((packed)) comm_espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    int len;                              //Length of ESPNOW data to be sent (buffer), unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.                         must be >= sizeof(comm_espnow_data_t)
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
} host_espnow_send_param_t;

#endif
