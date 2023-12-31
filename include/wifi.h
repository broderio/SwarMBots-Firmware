/**
 * @file            wifi.h
 * @author          The SwarMBots Team (EECS 473)
 * @brief           Header holding generic wifi functionalities (usable by host and client)
 * @version         0.1
 * @date            2023-11-30
 * 
 * @copyright       Copyright (c) 2023 The SwarMBots Team
 */

/*
 * Copyright (c) 2023 The SwarMBots Team (members below)
 * 
 * This project is released under the PolyForm Noncomercial License 1.0.0 and
 * owned collectively by the SwarMBots team members. Permission to replicate, 
 * modify, redistribute, sell, or otherwise make use of this software and associated 
 * documentation files is granted only insofar as is specified in the license text.
 * For license details, see LICENSE.MD or visit:
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 * 
 * SwarMBots team members:
 * Broderick Riopelle   ( broderio@umich.edu ),
 * Micah Williamson     ( micahwil@umich.edu ),
 * Jacob Gettig         ( jgettig@umich.edu ),
 * Gabriel Lounsbury    ( lounsbg@umich.edu ),
 * Daniel Benedict      ( benedan@umich.edu )
 */

#ifndef WIFI_H
#define WIFI_H

/* ==================================== INCLUDES ==================================== */

#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* ==================================== #DEFINED CONSTS ==================================== */

#define ESPNOW_WIFI_MODE        WIFI_MODE_AP        /**< sets wifi mode to softAP (see ESPNOW docs)*/
#define ESPNOW_WIFI_IF          ESP_IF_WIFI_AP      /**< sets ESPNOW to softAP interface (not used)*/

/* wifi channel - use only 1, 6, or 11 for FCC compliance & reliability */
#define ESPNOW_CHANNEL          11                  /**< Sets channel to be used by 2.4GHz wifi. FCC restrictions apply.*/
#define ESPNOW_PMK              "pmk1234567890123"  /**< Primary master key - used for packet encoding*/
#define ESPNOW_DATA_MAX_LEN     250                 /**< Physical maximum size of an ESPNOW payload*/
#define ESPNOW_MAXDELAY         (size_t)0xffffffff  /**< Maximum allowable number of FreeRTOS ticks to delay*/
#define ESPNOW_QUEUE_SIZE       6                   /**< Standard depth of FreeRTOS queues*/
#define MAC_ADDR_LEN            6                   /**< Length of a MAC address in Bytes - Redefinition of ESP_ETH_ALEN*/

#define SEND_CB_TAG             "SEND_CB"           /**< Tag for logging from \c espnow_send_cb() */
#define RECV_CB_TAG             "RECV_CB"           /**< Tag for logging from \c espnow_recv_cb() */
#define PARSE_TAG               "PARSE"             /**< Tag for logging from \c espnow_data_parse() */
#define PREPARE_TAG             "PREPARE"           /**< Tag for logging from \c espnow_data_prepare() */
#define DATA_SEND_TAG           "DATA_SEND"         /**< Tag for logging from \c espnow_data_send() */

#define DEBUG 1

/* ==================================== GLOBAL VARIABLES ==================================== */

extern QueueHandle_t espnow_send_queue;         /**< Queue of send events populated in \c espnow_send_cb()*/
extern QueueHandle_t espnow_recv_queue;         /**< Queue of send events populated in \c espnow_recv_cb()*/

/* ==================================== DATA STRUCTS ==================================== */

/**
 * @brief           Packet format to be sent over ESPNOW
 * 
 * @details         Stores data to transfer, plus 3 bytes overhead in the form of the length 
 *                  of the payload and a 16-bit crc.
 */
typedef struct comm_espnow_data{
    uint8_t len;                                /**< Length of \c payload */
    uint16_t crc;                               /**< CRC16 value of ESPNOW data. */
    uint8_t payload[0];                         /**< Real payload of ESPNOW data. */
} __attribute__((packed)) comm_espnow_data_t;

/**
 * @brief           Send parameter used to hold information about ESPNOW transaction
 * 
 * @details         Holds a packet to be sent over ESPNOW, the length of the packet, and
 *                  the packet's destination.
 */
typedef struct espnow_send_param{
    int len;                                    /**< Length of data to be sent (\c buffer ), unit: byte. */
    uint8_t buffer[ESPNOW_DATA_MAX_LEN];        /**< Buffer pointing to ESPNOW data - should be a \c comm_espnow_data_t */
    uint8_t dest_mac[MAC_ADDR_LEN];             /**< MAC address of destination device. */
} espnow_send_param_t;

/**
 * @brief           Holds information about a message received over ESPNOW
 */
typedef struct espnow_event_recv{
    uint8_t mac_addr[MAC_ADDR_LEN];             /**< MAC address of message sender */
    uint8_t* data;                              /**< raw data received - should be a \c comm_espnow_data_t*/
    int data_len;                               /**< length of \c data*/
} espnow_event_recv_t;

/**
 * @brief           Holds information about a message sent over ESPNOW
 */
typedef struct espnow_event_send{
    uint8_t mac_addr[MAC_ADDR_LEN];             /**< MAC address of intended recipient */
    esp_now_send_status_t status;               /**< Status of send (success or failure)*/
} espnow_event_send_t;

/* ==================================== FUNCTION PROTOTYPES ==================================== */

void espnow_send_cb(const uint8_t* mac_addr, esp_now_send_status_t status);
void espnow_recv_cb(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len);

int espnow_data_parse(uint8_t* data, uint16_t data_len, uint8_t* msg, uint16_t* len);
int espnow_data_prepare(espnow_send_param_t* send_param, uint8_t* data, int len);
int espnow_data_send(uint8_t* mac_addr, uint8_t* data, int len);

void wifi_init(void);
void espnow_init(void);
void espnow_deinit(void);

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
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const*)buf->payload, buf->len);
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
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const*)buf->payload, buf->len);
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

#endif
