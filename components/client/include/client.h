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

#ifndef CLIENT_H
#define CLIENT_H

/* ==================================== INCLUDES ==================================== */

#include "serial.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "wifi.h"

/* ==================================== #DEFINED CONSTS ==================================== */

#define LOFI_HS_PIN             4                   /**< GPIO Pin for SPI Handshake Receive wire */
#define LIFO_HS_PIN             5                   /**< GPIO Pin for SPI Handshake Send wire */
#define LIFO_PIN                6                   /**< GPIO Pin for SPI controller in/peripheral out */
#define CS_PIN                  7                   /**< GPIO Pin for SPI chip select wire */
#define SCLK_PIN                15                  /**< GPIO Pin for SPI controller clock signal */
#define LOFI_PIN                16                  /**< GPIO Pin for SPI controller out/peripheral in */

#define MCLK_PIN                18                  /**< GPIO Pin for I2S master clock */
#define PCLK_PIN                8                   /**< GPIO Pin for I2S peripheral clock */
#define VSYNC_PIN               46                  /**< GPIO Pin for I2S vertical sync */
#define HSYNC_PIN               9                   /**< GPIO Pin for I2S horizontal sync */
#define D0_PIN                  14                  /**< GPIO Pin for I2S 0th bit data line */
#define D1_PIN                  47                  /**< GPIO Pin for I2S 1st bit data line */
#define D2_PIN                  48                  /**< GPIO Pin for I2S 2nd bit data line */
#define D3_PIN                  21                  /**< GPIO Pin for I2S 3rd bit data line */
#define D4_PIN                  13                  /**< GPIO Pin for I2S 4th bit data line */
#define D5_PIN                  12                  /**< GPIO Pin for I2S 5th bit data line */
#define D6_PIN                  11                  /**< GPIO Pin for I2S 6th bit data line */
#define D7_PIN                  10                  /**< GPIO Pin for I2S 7th bit data line */

#define SDA_PIN                 3                   /**< GPIO Pin for I2C data line */
#define SCL_PIN                 2                   /**< GPIO Pin for I2C clock line */

#define TX_PIN                  43                  /**< GPIO Pin for UART transmit line */
#define RX_PIN                  44                  /**< GPIO Pin for UART receive line */

#define PAIR_PIN                17                  /**< GPIO Pin for pairing button */

#define LIDAR_PWM_PIN           1                   /**< GPIO Pin for LIDAR PWM signal */

#define CONNECT_TO_HOST_TAG "CONNECT_TO_HOST"   /**< Tag for logging from \c connect_to_host() */

/* ==================================== GLOBAL VARIABLES ==================================== */

/* ==================================== DATA STRUCTS ==================================== */

/* ==================================== FUNCTION PROTOTYPES ==================================== */

void connect_to_host(uint8_t* host_mac_addr);

#endif
