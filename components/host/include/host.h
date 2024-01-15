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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "esp_now.h"
#include "serial.h"

/* ==================================== #DEFINED CONSTS ==================================== */

#define JS_Y_PIN      4                         /**< Joystick Y pin on board (ADC1_CHANNEL3)*/
#define JS_X_PIN      5                         /**< Joystick X pin on board (ADC1_CHANNEL4)*/

#define PILOT_PIN     6                         /**< Pilot Button pin on board (GPIO)*/
#define PAIR_PIN      17                        /**< Pairing button pin on board (GPIO)*/

#define LED1_PIN      15                        /**< LED 1 pin on board (GPIO)*/
#define LED2_PIN      16                        /**< LED 2 pin on board (GPIO)*/

#define SDA_PIN       3                         /**< I2C SDA pin on board (GPIO)*/
#define SCL_PIN       2                         /**< I2C SCL pin on board (GPIO)*/

#define B1_PIN        10                        /**< Controller button 1 (Up) pin on board (GPIO)*/
#define B2_PIN        9                         /**< Controller Button 2 (Right) pin on board (GPIO)*/
#define B3_PIN        46                        /**< Controller button 3 (Down) pin on board (GPIO)*/
#define B4_PIN        11                        /**< Controller button 4 (Left) pin on board (GPIO)*/
#define B5_PIN        12                        /**< Controller button 5 (Joystick) pin on board (GPIO)*/

#define USB_BUF_SIZE  512                       /**< Size of buffer for USB communication */

/* ==================================== GLOBAL VARIABLES ==================================== */

extern esp_now_peer_info_t peers[8];            /**< Array of peer info (to avoid using built-in ESPNOW functions)*/
extern int32_t peer_num;                        /**< The number of known peers*/

/* ==================================== DATA STRUCTS ==================================== */

/* ==================================== FUNCTION PROTOTYPES ==================================== */

void add_peer(uint8_t* mac_address);
uint8_t *create_timesync_packet(uint64_t time);

#endif
