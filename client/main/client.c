/**
 * @file            client.c
 * @author          The SwarMBots Team (EECS 473)
 * @brief           Program that handles the client's tasks and responsibilities
 * 
 * @details         The client is responsible for receiving data from the host wirelessly over
 *                  ESPNOW and forwarding it to the MBoard via SPI. Concurrently, the client 
 *                  receives sensor data fron the MBoard via SPI and forwards it back to the
 *                  host via ESPNOW.
 * 
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
 * For license details, visit:
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 * 
 * SwarMBots team members:
 * Broderick Riopelle   ( broderio@umich.edu ),
 * Micah Williamson     ( micahwil@umich.edu ),
 * Jacob Gettig         ( jgettig@umich.edu ),
 * Gabriel Lounsbury    ( lounsbg@umich.edu ),
 * Daniel Benedict      ( benedan@umich.edu )
 */

/* ==================================== INCLUDES ==================================== */

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include <esp_timer.h>
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_crc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "client.h"
#include "lcm/comms.h"
#include "lcm/mbot_lcm_msgs_serial.h"
#include "spi.h"
#include "wifi.h"

/* ==================================== #DEFINED CONSTS ==================================== */

#define ESPNOW_SEND_TAG "ESPNOW_SEND_TASK"      /**< Tag for logging from \c espnow_send_task() */
#define ESPNOW_RECV_TAG "ESPNOW_RECV_TASK"      /**< Tag for logging from \c espnow_recv_task() */
#define MAIN_TAG        "APP_MAIN"              /**< Tag for logging from \c app_main() */

/* ==================================== GLOBAL VARIABLES ==================================== */

QueueHandle_t espnow_send_queue;                /**< Queue of send events populated in \c espnow_send_cb() (\c wifi.h )*/
QueueHandle_t espnow_recv_queue;                /**< Queue of send events populated in \c espnow_recv_cb() (\c wifi.h )*/

SemaphoreHandle_t spi_mutex;                    /**< Mutex for mediating SPI - prevents simultaneous sends and receives*/
SemaphoreHandle_t wifi_ready;                   /**< Semaphore that delays \c espnow_send_task() until host found*/

uint8_t host_mac_addr[MAC_ADDR_LEN];            /**< Global storage of host MAC address*/

/* ==================================== FUNCTION DEFINITIONS ==================================== */

/**
 * @brief           Task that handles receiving over ESPNOW and sending over SPI
 * 
 * @details         Handles pairing such that the first communication received sets the host until restart.
 *                  After pairing, parses messages into data that can be sent to the MBoard over SPI and
 *                  sends it.
 * 
 * @note            The contents of the first message received are lost as it is exclusively used for pairing.
 * 
 * @param args      Ignores args. Parameter present for FreeRTOS compatibility.
 * @sa              espnow_send_task()
 */
static void
espnow_recv_task(void* args) {
    espnow_event_recv_t evt;
    esp_err_t err;
    spi_slave_transaction_t transaction;

    int32_t ret;
    uint16_t data_len;
    uint8_t msg[ESPNOW_DATA_MAX_LEN];
    uint8_t mac[MAC_ADDR_LEN];

    err = esp_wifi_get_mac(WIFI_IF_AP, mac);
    if (err != ESP_OK) {
        ESP_LOGI(ESPNOW_RECV_TAG, "Could not get mac address, error code %d", err);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /* Print client mac address */
    ESP_LOGI(ESPNOW_RECV_TAG, "Client MAC: " MACSTR, MAC2STR(mac));

    /* Wait for first message from host */
    ESP_LOGI(ESPNOW_RECV_TAG, "Waiting for host...");
    connect_to_host(host_mac_addr);
    ESP_LOGI(ESPNOW_RECV_TAG, "Connected to host.");
    xSemaphoreGive(wifi_ready);

    /* Begin receiving messages and sending them over SPI */
    while (xQueueReceive(espnow_recv_queue, &evt, portMAX_DELAY) == pdTRUE) {
        /* Parse incoming packet */
        ret = espnow_data_parse(evt.data, evt.data_len, msg, &data_len);
        free(evt.data);
        ESP_LOGI(ESPNOW_RECV_TAG, "Received message.");

        /* Check if data is invalid */
        if (ret != 0) {
            ESP_LOGE(ESPNOW_RECV_TAG, "Received invalid data");
            continue;
        }

        /* Populate SPI packet */
        transaction.length = data_len * 8;
        transaction.tx_buffer = msg;
        transaction.rx_buffer = NULL;
        transaction.trans_len = 0;

        ret = ESP_FAIL;
        if (xSemaphoreTake(spi_mutex, portMAX_DELAY) == pdTRUE) {
            ret = spi_slave_transmit(SPI2_HOST, &transaction, portMAX_DELAY);
            xSemaphoreGive(spi_mutex);
            if (ret != ESP_OK) {
                ESP_LOGE(ESPNOW_RECV_TAG, "SPI transmission failed.");
            }
        }
        ESP_LOGI(ESPNOW_RECV_TAG, "Sent %zu bytes over SPI", t.trans_len / 8);
    }
}

/**
 * @brief           Task that handles reading SPI and sending over ESPNOW
 * 
 * @details         Reads full sensor data (6 packets) from MBot over SPI, collates them into one packet,
 *                  and sends it to the host via ESPNOW. This task should run 25 times per second.
 * 
 * @param args      Ignores args. Parameter present for FreeRTOS compatibility.
 * @sa              espnow_recv_task()
 */
void
espnow_send_task(void* args) {
    esp_err_t ret;
    spi_slave_transaction_t transaction;

    size_t full_pkt_len = sizeof(packets_wrapper_t) + 1;
    WORD_ALIGNED_ATTR uint8_t recvbuf[84];
    uint8_t full_pkt[full_pkt_len];

    xSemaphoreTake(wifi_ready, portMAX_DELAY);
    while (1) {
        size_t pkt_idx = 0;

        for (size_t i = 0; i < 6; ++i) {
            size_t msg_len;
            uint8_t* msg_start;

            transaction.length = 84 * 8;
            transaction.tx_buffer = NULL;
            transaction.rx_buffer = recvbuf;
            transaction.trans_len = 0;

            ret = ESP_FAIL;
            if (xSemaphoreTake(spi_mutex, portMAX_DELAY) == pdTRUE) {
                ret = spi_slave_transmit(SPI2_HOST, &transaction, portMAX_DELAY);
                xSemaphoreGive(spi_mutex);
                if (ret != ESP_OK) {
                    ESP_LOGE(ESPNOW_SEND_TAG, "SPI transmission failed.");
                }
            }

            /* Copy data into packet (removes ROS header and footer) */
            msg_start = transaction.rx_buffer + 7;
            msg_len = transaction.trans_len / 8 - 8;

            // ESP_LOGI(ESPNOW_SEND_TAG, "Received SPI packet of len: %u.", msg_len);

            memcpy(full_pkt + pkt_idx, msg_start, msg_len);
            pkt_idx += msg_len;
        }

        if (pkt_idx != full_pkt_len - 1) {
            ESP_LOGE(ESPNOW_SEND_TAG, "Total packet length incorrect!");
            continue;
        }

        /* Add checksum to data for UART verification */
        full_pkt[full_pkt_len - 1] = checksum(full_pkt, full_pkt_len - 1);

        espnow_data_send(host_mac_addr, full_pkt, full_pkt_len);
    }
}

/**
 * @brief           Main function of the program. Initializes wifi, ESPNOW, and SPI. 
 *                  Launches tasks for receiving over SPI/sending over ESPNOW and for
 *                  receiving over ESPNOW/sending over SPI.
 */
void
app_main(void) {
    esp_err_t ret;
    TaskHandle_t recv_task_handle, send_task_handle;

    /* Initialize NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(MAIN_TAG, "Initializing WiFi...");
    wifi_init();

    ESP_LOGI(MAIN_TAG, "Initializing ESP-NOW...");
    espnow_init();

    ESP_LOGI(MAIN_TAG, "Initializing SPI...");
    spi_init();

    /* Create mutex for SPI */
    spi_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(spi_mutex);
    wifi_ready = xSemaphoreCreateBinary();

    /* Create tasks */
    xTaskCreate(espnow_send_task, "espnow_send_task", 2048 * 4, NULL, 4, &send_task_handle);
    xTaskCreate(espnow_recv_task, "espnow_recv_task", 2048 * 4, NULL, 4, &recv_task_handle);

    /* Silence logs if we are building release version */
#ifndef DEBUG
    ESP_LOGI("MAIN", "Silencing logs.");
    esp_log_level_set("*", ESP_LOG_NONE);
#endif
}