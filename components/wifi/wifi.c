#include "wifi.h"

/* ==================================== FUNCTION DEFINITIONS ==================================== */

/**
 * @brief           Callback for sending via ESPNOW
 * 
 * @param mac_addr  MAC address of intended recipient
 * @param status    Status of send (success or failure)
 */
void
espnow_send_cb(const uint8_t* mac_addr, esp_now_send_status_t status) {
    espnow_event_send_t evt;

    if (mac_addr == NULL) {
        ESP_LOGE(SEND_CB_TAG, "Send cb arg error");
        return;
    }

    memcpy(evt.mac_addr, mac_addr, MAC_ADDR_LEN);
    evt.status = status;

    /* Publish send status to queue */
    if (xQueueSend(espnow_send_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(SEND_CB_TAG, "Send send queue fail");
    }
}

/**
 * @brief           Callback for receiving via ESPNOW
 * 
 * @param recv_info Contains information about message sender and recipient MAC addresses
 * @param data      Raw data bytes received
 * @param len       Length of \c data
 */
void
espnow_recv_cb(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len) {
    espnow_event_recv_t evt;

    if (recv_info->src_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(RECV_CB_TAG, "Receive cb arg error");
        return;
    }

    /* Copy source mac address */
    memcpy(evt.mac_addr, recv_info->src_addr, MAC_ADDR_LEN);
    evt.data = malloc(len);
    if (evt.data == NULL) {
        ESP_LOGE(RECV_CB_TAG, "Malloc receive data fail");
        return;
    }

    /* Copy data */
    memcpy(evt.data, data, len);
    evt.data_len = len;

    // ESP_LOGI(RECV_CB_TAG, "Received packet in callback with len: %d", len);

    /* Publish to queue */
    if (xQueueSend(espnow_recv_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(RECV_CB_TAG, "Send receive queue fail");
    }
}

/**
 * @brief           Parses received ESPNOW data and validates its crc
 * 
 * @param data      Raw data byes
 * @param data_len  Length of \c data
 * @param msg       Parsed output (value overwritten by function)
 * @param len       Length of \c msg (value overwritten by function)
 * @return          0 if parse was successful, else -1 (and logs an error)
 */
int
espnow_data_parse(uint8_t* data, uint16_t data_len, uint8_t* msg, uint16_t* len) {
    comm_espnow_data_t* buf;

    uint16_t crc_cal;

    buf = (comm_espnow_data_t*)data;            /* Cast data byte array to buffer struct */

    /* Check if data length is correct */
    if (data_len != sizeof(comm_espnow_data_t) + buf->len) {
        ESP_LOGE(PARSE_TAG, "Receive ESPNOW data has wrong length, len:%d, expected len:%d", data_len,
                 sizeof(comm_espnow_data_t) + buf->len);
        return -1;
    }

    /* Check if CRC matches */
    crc_cal = crc16_le(UINT16_MAX, (uint8_t const*)buf->payload, buf->len);
    if (crc_cal != buf->crc) {
        return -1;
    }

    /* Copy payload to msg */
    memcpy(msg, buf->payload, buf->len);
    *len = buf->len;

    return 0;
}

/**
 * @brief               Prepares ESPNOW data to be sent
 * 
 * @param send_param    Variable that will hold packet information until send occurs (value overwritten by function)
 * @param data          Raw data to be sent
 * @param len           Length of \c data
 * @return              0 if prepare was successful, else -1 (and logs an error)
 */
int
espnow_data_prepare(espnow_send_param_t* send_param, uint8_t* data, int len) {
    comm_espnow_data_t* buf;

    int32_t send_param_len;

    /* Check if length is valid */
    if (len > ESPNOW_DATA_MAX_LEN) {
        ESP_LOGE(PREPARE_TAG, "Data too long! %d > %d", len, ESPNOW_DATA_MAX_LEN);
        return -1;
    }
    send_param_len = len + sizeof(comm_espnow_data_t);
    send_param->len = send_param_len;

    /* Set memory for sending */
    memset(send_param->buffer, 0, send_param->len);

    /* Cast buffer to struct */
    buf = (comm_espnow_data_t*)send_param->buffer;

    /* Fill in fields */
    buf->len = len;
    memcpy(buf->payload, data, len);
    buf->crc = crc16_le(UINT16_MAX, (uint8_t const*)buf->payload, buf->len);
    return 0;
}

/**
 * @brief               Initiates an ESPNOW send transaction
 * 
 * @details             This function is not strictly necessary, but it eliminates possible errors
 *                      in parameter preparation and eliminates the need for the programmer to worry
 *                      about handling a send parameter.
 * 
 * @param mac_addr      MAC address of intended recipient
 * @param data          Raw data to be sent
 * @param len           Length of \c data
 * @return              0 if transaction was successful, else -1 (and logs an error)
 */
int
espnow_data_send(uint8_t* mac_addr, uint8_t* data, int len) {
    espnow_send_param_t send_param;
    espnow_event_send_t* send_evt;

    int32_t ret;

    /* Prepare data to send */
    memcpy(send_param.dest_mac, mac_addr, MAC_ADDR_LEN);
    ret = espnow_data_prepare(&send_param, data, len);
    if (ret < 0) {
        return -1;
    }

    /* Send data */
    esp_now_send(send_param.dest_mac, send_param.buffer, send_param.len);

    /* Check to see if send was successful */
    if (xQueueReceive(espnow_send_queue, &send_evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(DATA_SEND_TAG, "Send failed");
        return -1;
    }
    return 0;
}

/**
 * @brief           Initializes ESP Wi-Fi
 * 
 * @note            Call required before calling \c espnow_init()
 * @sa              espnow_init()
 */
void
wifi_init(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));    /* sets to use only ram for storage */
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));       /* AP */
    ESP_ERROR_CHECK(esp_wifi_start());

    /* NOTE: may need to add channel negotiation logic */
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

/**
 * @brief           Initializes ESPNOW
 * 
 * @note            Requires that Wi-Fi has been initialized by \c wifi_init()
 * @sa              wifi_init()
 */
void
espnow_init() {
    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* Create receive and send queues */
    espnow_send_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_send_t));
    espnow_recv_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_recv_t));
    if (espnow_send_queue == NULL || espnow_recv_queue == NULL) {
        ESP_LOGE(PREPARE_TAG, "Create mutex fail");
        return;
    }

    /* Set primary master key. */
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)ESPNOW_PMK));

    return;
}

/**
 * @brief           deletes semaphores and deinitializes ESPNOW
 * 
 * @note            It is usually not strictly required to call this function since there should
 *                  always be tasks using ESPNOW, otherwise the ESP should restart.
 */
void
espnow_deinit() {
    vSemaphoreDelete(espnow_send_queue);
    vSemaphoreDelete(espnow_recv_queue);
    esp_now_deinit();
}
