/**
 * @file            host.c
 * @author          The SwarMBots Team (EECS 473)
 * @brief           Program that handles the host's tasks and responsibilities
 * 
 * @details         The host is responsible for receiving data from the computer over UART
 *                  and forwarding it to the client wirelessly via ESPNOW. Additionally, the host
 *                  supports manual control of one robot via a built-in joystick and buttons.
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

/* ==================================== INCLUDES ==================================== */

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "driver/gpio.h"
#include "esp_crc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "lcm/comms.h"
#include "lcm/mbot_lcm_msgs_serial.h"
#include "mbot_params.h"
#include "nvs_flash.h"

#include "controller.h"
#include "host.h"
#include "wifi.h"

/* ==================================== #DEFINED CONSTS ==================================== */

#define PRINT_TAG       "PRINT_TASK"            /**< Tag for logging from \c print_task() */
#define SERIAL_TAG      "SERIAL_TASK"           /**< Tag for logging from \c serial_mode_task() */
#define PILOT_TAG       "PILOT_TASK"            /**< Tag for logging from \c pilot_mode_task() */
#define ESPNOW_RECV_TAG "ESPNOW_RECV_TASK"      /**< Tag for logging from \c espnow_recv_task() */

/* ==================================== GLOBAL VARIABLES ==================================== */

QueueHandle_t espnow_send_queue;                /**< Queue of send events populated in \c espnow_send_cb() (\c wifi.h ) */
QueueHandle_t espnow_recv_queue;                /**< Queue of send events populated in \c espnow_recv_cb() (\c wifi.h ) */

esp_now_peer_info_t peers[8];                   /**< Array of peer info (to avoid using built-in ESPNOW functions) */


SemaphoreHandle_t switch_sem;                   /**< Semaphore used to wait for mode switch */
TaskHandle_t serialMode;                        /**< Handle for task \c serial_mode_task() */
TaskHandle_t pilotMode;                         /**< Handle for task \c pilot_mode_task() */

int32_t peer_num = 0;                           /**< The number of known peers */
int joystick_x;                             /**< X value read from the joystick */
int joystick_y;                             /**< Y value read from the joystick */

bool doSerial = true;                           /**< Tracks whether the host is in serial or pilot mode */

/* ==================================== FUNCTION DEFINITIONS ==================================== */

/**
 * @brief           Task that prints debug information on current task mode.
 * 
 * @note            This task is only launched when the program is compiled in a debug format.
 * 
 * @param arg       Ignores arg. Parameter present for FreeRTOS compatibility.
 */
void
print_task(void* arg) {
    uint32_t io_num;

    for (;;) {

        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {

            if (io_num == B1_PIN) {
                ESP_LOGI(PRINT_TAG, "Button Up");
            } else if (io_num == B2_PIN) {
                ESP_LOGI(PRINT_TAG, "Button Down");
            } else {
                if (!doSerial) {
                    ESP_LOGI(PRINT_TAG, "Controller mode");
                } else {
                    ESP_LOGI(PRINT_TAG, "Serial Mode");
                }
            }
        }
    }
}

/**
 * @brief           Task that handles receiving data from clients via ESPNOW.
 * 
 * @details         When a new ESPNOW packet is received from a client, it is parsed, checked for validity, and 
 *                  forwarded to the computer over UART in the form of a BOTPKT (raw data plus checksum).
 * 
 * @param args      Ignores args. Parameter present for FreeRTOS compatibility.
 */
void
espnow_recv_task(void* args) {
    espnow_event_recv_t evt;
    esp_err_t err;

    uint16_t data_len;
    int16_t ret;
    uint8_t msg[ESPNOW_DATA_MAX_LEN];
    uint8_t packet[ESPNOW_DATA_MAX_LEN];
    uint8_t mac[MAC_ADDR_LEN];

    err = esp_wifi_get_mac(WIFI_IF_AP, mac);
    if (err != ESP_OK) {
        ESP_LOGI(ESPNOW_RECV_TAG, "Could not get mac address, error code %d", err);
    }

    /* Delay for 1 second */
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /* Print host mac address */
    ESP_LOGI(ESPNOW_RECV_TAG, "Host MAC: " MACSTR, MAC2STR(mac));

    while (xQueueReceive(espnow_recv_queue, &evt, portMAX_DELAY) == pdTRUE) {
        size_t packet_len;

        /* Parse incoming packet */
        ret = espnow_data_parse(evt.data, evt.data_len, msg, &data_len);
        free(evt.data);

        /* Check if data is invalid */
        if (ret < 0) {
            ESP_LOGE(ESPNOW_RECV_TAG, "Receive invalid data");
            continue;
        }

        /* If message not from known peer, ignore it */
        if (!esp_now_is_peer_exist(evt.mac_addr)) {
            ESP_LOGE(ESPNOW_RECV_TAG, "found a new peer unexpectedly: addr " MACSTR "", MAC2STR(evt.mac_addr));
            continue;
        }

        /* Create BOTPKT */
        packet_len = data_len + MAC_ADDR_LEN + 3;
        packet[0] = 0xff;

        /* Length of actual data (minus 1 for the checksum at the end, should always = 204) */
        packet[1] = (uint8_t)((data_len - 1) % 255);
        packet[2] = (uint8_t)((data_len - 1) >> 8);
        memcpy(packet + 3, evt.mac_addr, MAC_ADDR_LEN);
        memcpy(packet + MAC_ADDR_LEN + 3, msg, data_len);

        /* Send message to UART */
        uart_write_bytes(UART_PORT_NUM, (const char*)packet, packet_len);
    }

    ESP_LOGE(ESPNOW_RECV_TAG, "Exited task espnow_recv_task loop");
}

/**
 * @brief           Task that handles serial mode. Mutually exclusive with \c pilot_mode_task()
 * 
 * @details         Reads input from the computer over UART and sends it to the client specified by the MAC address 
 *                  in the packet. If the specified MAC address does not correspond to a peer, the task adds it as
 *                  a new peer and sends the packet.
 * 
 * @param arg       Ignores arg. Parameter present for FreeRTOS compatibility.
 * @sa              pilot_mode_task()
 */
void
serial_mode_task(void* arg) {
    uart_config_t uart_config = {               /* Configure UART */
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    TickType_t xLastWakeTime;

    int32_t intr_alloc_flags;
    uint8_t packet[ESPNOW_DATA_MAX_LEN];


    intr_alloc_flags = 0;
    ESP_ERROR_CHECK(uart_driver_install(0, 2 * ESPNOW_DATA_MAX_LEN, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    /* Suspend immediately if in controller mode */
    if (!doSerial) {
        vTaskSuspend(NULL);
    }
    
    ESP_LOGI(SERIAL_TAG, "Serial mode");

    while (1) {
        uint16_t pkt_len;
        uint8_t mac_address[MAC_ADDR_LEN];

        xLastWakeTime = xTaskGetTickCount();

        /* Read mac address and validate */
        ESP_LOGI(SERIAL_TAG, "Waiting for packet...");
        read_mac_address(mac_address, &pkt_len);
        ESP_LOGI(SERIAL_TAG, "Received packet for " MACSTR, MAC2STR(mac_address));

        /* Read the ROSPKT */
        read_packet(packet, pkt_len);

        /* Add peer if mac address is not already */
        if (!esp_now_is_peer_exist(mac_address)) {
            ESP_LOGI(SERIAL_TAG, "Adding peer ...");
            add_peer(mac_address);
        }

        espnow_data_send(mac_address, packet, pkt_len);

        xTaskDelayUntil(&xLastWakeTime, 1);
    }

    ESP_LOGE(SERIAL_TAG, "Exited task uart_in_task loop");
}

/**
 * @brief           Task that handles pilot mode. Mutually exclusive with \c serial_mode_task
 * 
 * @details         Reads controller input and parses it into a ROS packet as if it were sent by the computer,
 *                  then sends the packet to the client via ESPNOW. This task is
 * 
 * @note            This task does not yet support other client bots running serial commands while in pilot mode.
 * 
 * @param arg       Ignores arg. Parameter present for FreeRTOS compatibility.
 * @sa              serial_mode_task()
 */
static void
pilot_mode_task(void* arg) {
    esp_now_peer_info_t peer;
    espnow_send_param_t send_param;
    TickType_t xLastWakeTime;

    int vy_adc, vx_adc;

    float max;

    /* Suspend immediately if in serial mode or if we have no clients paired */
    if (doSerial) {
        vTaskSuspend(pilotMode);
    }

    ESP_LOGI(PILOT_TAG, "Pilot mode");

    peer = peers[curr_bot];
    memset(&send_param, 0, sizeof(espnow_send_param_t));
    send_param.len = 0;

    max = 1.5;
    while (1) {
        float vx, wz;

        xLastWakeTime = xTaskGetTickCount();

        adc_oneshot_get_calibrated_result(adc1_handle, JS_Y_cali, ADC_CHANNEL_3, &vy_adc);
        adc_oneshot_get_calibrated_result(adc1_handle, JS_X_cali, ADC_CHANNEL_4, &vx_adc);

        // ESP_LOGI(PILOT_TAG, "Vertical Voltage: %d", vy);
        // ESP_LOGI(PILOT_TAG, "Horizontal Voltage: %d", vx);

        /* max out at 5 m/s */
        vx = (abs(vy_adc - joystick_y) > 50) ? vy_adc * (2.0 * max) / 946.0 - max : 0;
        wz = (abs(vx_adc - joystick_x) > 50) ? -vx_adc * (6.0 * max) / 946.0 + 3 * max : 0;

        //ESP_LOGI(PILOT_TAG, "Forward Velocity: %f m/s", vx);
        //ESP_LOGI(PILOT_TAG, "Turn Velocity: %f m/s", wz);

        if (peer_num > 0) {
            uint16_t pkt_len;
            uint8_t* packet;

            peer = peers[curr_bot];
            ESP_LOGI(PILOT_TAG, "Sending to " MACSTR "", MAC2STR(peer.peer_addr));
            memcpy(send_param.dest_mac, peer.peer_addr, MAC_ADDR_LEN);
            packet = command_serializer(vx, 0, wz);
            pkt_len = sizeof(serial_twist2D_t) + ROS_PKG_LEN;
            espnow_data_send(peer.peer_addr, packet, pkt_len);
            free(packet);
        }

        //ESP_LOGI(PILOT_TAG, "GPIO17: %d\n", gpio_get_level(SW_PIN));
        //ESP_LOGI(PILOT_TAG, "Heap size (at end) %lu", esp_get_free_heap_size());

        vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief           Task that handles switching between serial mode and pilot mode and syncing with MBoard.
 * 
 * @param args      Ignores args. Parameter present for FreeRTOS compatibility.
 */
void
main_task(void* args) {
    TickType_t last_wake_time;
    while (1) {
        last_wake_time = xTaskGetTickCount();

        /* Wait for the semaphore, timeout so we can send timesync */
        if (xSemaphoreTake(switch_sem, 50 / portTICK_PERIOD_MS) == pdTRUE) {
            /* Check the switch state and suspend/resume tasks accordingly */
            if (doSerial) {
                ESP_LOGI("SWITCH", "Serial mode");

                vTaskSuspend(pilotMode);
                vTaskResume(serialMode);

                /* Remove ISR for buttons */
                gpio_isr_handler_remove(B1_PIN);
                gpio_isr_handler_remove(B2_PIN);
            } else {
                ESP_LOGI("SWITCH", "Pilot mode");

                vTaskSuspend(serialMode);
                vTaskResume(pilotMode);

                /* Add ISR for buttons */
                gpio_isr_handler_add(B1_PIN, buttons_isr_handler, (void*)B1_PIN);
                gpio_isr_handler_add(B2_PIN, buttons_isr_handler, (void*)B2_PIN);
            }
        }

        /* Create timesync message */
        uint64_t time = esp_timer_get_time();
        uint8_t *packet = create_timesync_packet(time);
        size_t pkt_len = sizeof(serial_timestamp_t) + ROS_PKG_LEN;

        /* Send timesync message to all peers */
        ESP_LOGI("TIMESYNC", "Sending timesync packet to all peers");
        for (int i = 0; i < peer_num; ++i) {
            ESP_LOGI("TIMESYNC", "Sending to " MACSTR "", MAC2STR(peers[i].peer_addr));
            espnow_data_send(peers[i].peer_addr, packet, pkt_len);
        }
        free(packet);

        // Run at 2 Hz
        vTaskDelayUntil(&last_wake_time, SYNC_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

/**          
 * @brief           Main function of the program. Initializes wifi, ESPNOW, and controller. 
 *                  Launches tasks for receiving over espnow, serial mode, pilot mode, and switching modes.
 */
void
app_main() {
    esp_err_t ret;

    /* Init NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Init wifi and espnow */
    wifi_init();
    espnow_init();

    /* Init controller */
    controller_init();
    calibrate_joystick(&joystick_x, &joystick_y, 1000);

    /* Set the mode given the switch state defined in controller.c */
    doSerial = !(bool)gpio_get_level(SW_PIN);
    
    /* Create tasks */
    xTaskCreate(espnow_recv_task, "espnow_recv_task", 4096, NULL, 4, NULL);
    xTaskCreate(main_task, "main_task", 4096, NULL, 4, NULL);
    xTaskCreate(serial_mode_task, "serial_mode_task", 4096, NULL, 3, &serialMode);
    xTaskCreate(pilot_mode_task, "pilot_mode_task", 4096, NULL, 3, &pilotMode);

    /* for debugging */
    // xTaskCreate(print_task, "print_task", 2048, NULL, 3, NULL);

/* Silence logs for UART communication if we are building release version */
#ifndef DEBUG
    ESP_LOGI("MAIN", "Silencing logs.");
    esp_log_level_set("*", ESP_LOG_NONE);
#endif
}