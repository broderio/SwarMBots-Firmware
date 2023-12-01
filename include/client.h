/**
 * @file            client.h
 * @author          The SwarMBots Team (EECS 473)
 * @brief           Header holding wifi functionalities specific to the client
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

#ifndef CLIENT_AP_H
#define CLIENT_AP_H

/* ==================================== INCLUDES ==================================== */

#include "lcm/comms.h"
#include "wifi.h"

/* ==================================== #DEFINED CONSTS ==================================== */

#define GPIO_RECV           2                   /**< GPIO Pin for SPI Handshake Receive wire */
#define GPIO_SEND           3                   /**< GPIO Pin for SPI Handshake Send wire */
#define GPIO_CS             10                  /**< GPIO Pin for SPI chip select wire */
#define GPIO_MOSI           11                  /**< GPIO Pin for SPI controller out/peripheral in */
#define GPIO_SCLK           12                  /**< GPIO Pin for SPI controller clock signal */
#define GPIO_MISO           13                  /**< GPIO Pin for SPI controller in/peripheral out */

#define CONNECT_TO_HOST_TAG "CONNECT_TO_HOST"   /**< Tag for logging from \c connect_to_host() */

/* ==================================== GLOBAL VARIABLES ==================================== */

/* ==================================== DATA STRUCTS ==================================== */

/* ==================================== FUNCTION PROTOTYPES ==================================== */

void connect_to_host(uint8_t* host_mac_addr);

/* ==================================== FUNCTION DEFINITIONS ==================================== */

/**
 * @brief                   Helper function for initial ESPNOW setup that finds the host and adds it as a peer
 * 
 * @details                 Waits for a message to be received, then logs the MAC address of the sender as the
 *                          host. All other information contained in the message is discarded.
 * 
 * @param host_mac_addr     The MAC address of the host to be returned (value overwritten by function)
 */
void
connect_to_host(uint8_t* host_mac_addr) {
    espnow_event_recv_t evt;

    uint16_t data_len;
    uint8_t msg[ESPNOW_DATA_MAX_LEN];

    /* Wait for first message to add host as peer */
    if (xQueueReceive(espnow_recv_queue, &evt, portMAX_DELAY) == pdTRUE) {
        esp_now_peer_info_t* peer;
        esp_now_rate_config_t rate_config = {
            .phymode = WIFI_PHY_MODE_HT20,
            .rate = WIFI_PHY_RATE_MCS7_SGI,
        };

        int32_t ret;

        ESP_LOGI(CONNECT_TO_HOST_TAG, "Received message.");

        /* Parse incoming packet */
        ret = espnow_data_parse(evt.data, evt.data_len, msg, &data_len);
        free(evt.data);

        /* Check if data is invalid */
        if (ret != 0) {
            ESP_LOGE(CONNECT_TO_HOST_TAG, "Received invalid data");
        }

        /* Allocate peer */
        peer = malloc(sizeof(esp_now_peer_info_t));
        if (peer == NULL) {
            ESP_LOGE(CONNECT_TO_HOST_TAG, "Malloc peer information fail");
        }
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        peer->channel = ESPNOW_CHANNEL;
        peer->ifidx = ESPNOW_WIFI_IF;
        peer->encrypt = false;
        memcpy(peer->peer_addr, evt.mac_addr, MAC_ADDR_LEN);

        /* Add peer */
        ESP_ERROR_CHECK(esp_now_add_peer(peer));
        free(peer);
        memcpy(host_mac_addr, evt.mac_addr, MAC_ADDR_LEN);
        esp_now_set_peer_rate_config(evt.mac_addr, &rate_config);
        ESP_LOGI(CONNECT_TO_HOST_TAG, "Found host (MAC: " MACSTR ")", MAC2STR(evt.mac_addr));
    }
}

#endif
