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

#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_now.h"
#include "esp_log.h"
#include "rom/crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

#endif
