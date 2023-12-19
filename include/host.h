/**
 * @file            host.h
 * @author          The SwarMBots Team (EECS 473)
 * @brief           Header holding wifi functionalities specific to the host
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

#ifndef HOST_AP_H
#define HOST_AP_H

/* ==================================== INCLUDES ==================================== */

/* ==================================== #DEFINED CONSTS ==================================== */

#define JS_Y_PIN      4                         /**< Joystick Y pin on board (ADC1_CHANNEL3) */
#define JS_X_PIN      5                         /**< Joystick X pin on board (ADC1_CHANNEL4) */
#define SW_PIN        6                         /**< Switch pin on board (GPIO)*/
#define B5_PIN        7                         /**< Controller button 5 (Joystick) pin on board (GPIO)*/
#define B1_PIN        8                         /**< Controller button 1 (Up) pin on board (GPIO)*/
#define B2_PIN        9                         /**< Controller Button 2 (Right) pin on board (GPIO)*/
#define B3_PIN        10                        /**< Controller button 3 (Down) pin on board (GPIO)*/
#define B4_PIN        11                        /**< Controller button 4 (Left) pin on board (GPIO)*/
#define LED1_PIN      15                        /**< LED 1 pin on board (GPIO)*/
#define LED2_PIN      16                        /**< LED 2 pin on board (GPIO)*/

/* ==================================== GLOBAL VARIABLES ==================================== */

extern esp_now_peer_info_t peers[8];            /**< Array of peer info (to avoid using built-in ESPNOW functions)*/
extern int32_t peer_num;                        /**< The number of known peers*/

/* ==================================== DATA STRUCTS ==================================== */

/* ==================================== FUNCTION PROTOTYPES ==================================== */

void add_peer(uint8_t* mac_address);
uint8_t *create_timesync_packet(uint64_t time);

/* ==================================== FUNCTION DEFINITIONS ==================================== */

/**
 * @brief               Saves a new peer
 * 
 * @details             Adds a new peer with the given MAC address to the ESPNOW peer list as well
 *                      as the local one, and increments \c peer_num
 * 
 * @param mac_address   MAC address of new peer
 */
void
add_peer(uint8_t* mac_address) {
    esp_now_rate_config_t rate_config = {
        .phymode = WIFI_PHY_MODE_HT20,
        .rate = WIFI_PHY_RATE_MCS7_SGI,
    };

    memset(&peers[peer_num], 0, sizeof(esp_now_peer_info_t));
    peers[peer_num].channel = ESPNOW_CHANNEL;
    peers[peer_num].ifidx = ESPNOW_WIFI_IF;
    peers[peer_num].encrypt = false;
    memcpy(peers[peer_num].peer_addr, mac_address, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peers[peer_num]));
    esp_now_set_peer_rate_config(mac_address, &rate_config);
    ++peer_num;
}

/**
 * @brief           Creates a timesync packet
 * 
 * @param time      The current time in microseconds
 * @return          A Timestamp ROS packet as a byte array
 */
uint8_t*
create_timesync_packet(uint64_t time) {
    serial_timestamp_t msg = {
        .utime = time
    };

    // Initialize variables for packet
    size_t msg_len = sizeof(msg);
    uint8_t *msg_serialized = (uint8_t *)(malloc(msg_len));
    uint8_t *packet = (uint8_t *)(malloc(msg_len + ROS_PKG_LEN));

    // Serialize message and create packet
    timestamp_t_serialize(&msg, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_TIMESYNC, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);
    return packet;
}

#endif
