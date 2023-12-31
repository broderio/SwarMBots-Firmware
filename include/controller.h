/**
 * @file            controller.h
 * @author          The SwarMBots Team (EECS 473)
 * @brief           Header holding functionalities specific to the host's use as a controller
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

#ifndef CONTROLLER_H
#define CONTROLLER_H

/* ==================================== INCLUDES ==================================== */

#include "freertos/FreeRTOS.h"
#include "inttypes.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#include "esp_log.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "host.h"
#include "lcm/comms.h"
#include "lcm/mbot_lcm_msgs_serial.h"
#include "mbot_params.h"
#include "wifi.h"

/* ==================================== #DEFINED CONSTS ==================================== */
#define MAX_LIN_VEL 0.25 /**< Maximum linear velocity of the MBot in m/s */
#define MAX_ANG_VEL 1.5 /**< Maximum angular velocity of the MBot in rad/s */

/* ==================================== GLOBAL VARIABLES ==================================== */

static size_t curr_bot = 0; /**< Stores the index of the current robot being controlled */

static uint32_t last_button = 0; /**< Stores the port of the last button pressed */
static uint32_t last_press = 0;  /**< Stores the time of the last button press */
static uint32_t last_switch = 0; /**< Stores the time of the last switch toggle */

static QueueHandle_t gpio_evt_queue = NULL; /**< Queue for GPIO level change events*/

adc_oneshot_unit_handle_t adc1_handle; /**< Handle for referencing the joystick adc */
adc_cali_handle_t JS_Y_cali;    /**< Handle for storing joystick Y calibration data */
adc_cali_handle_t JS_X_cali;    /**< Handle for storing joystick X calibration data */
int joystick_y = 0;             /**< Stores the joystick Y rest value */
int joystick_x = 0;             /**< Stores the joystick X rest value */

extern bool doSerial;         /**< Tracks whether the host is in serial or pilot mode */
extern SemaphoreHandle_t switch_sem; /**< Semaphore used to wait for mode switch */

extern TaskHandle_t serialMode; /**< Handle for host's serial mode task */
extern TaskHandle_t pilotMode;  /**< Handle for host's pilot mode task */

/* ==================================== DATA STRUCTS ==================================== */

/* ==================================== FUNCTION PROTOTYPES ==================================== */

void get_vel_from_joystick(float* vx, float* wz);
uint8_t* command_serializer(float vx, float vy, float wz);

static void buttons_isr_handler(void* arg);
static void switch_isr_handler(void* arg);

void controller_init();
void calibrate_joystick(void);

/* ==================================== FUNCTION DEFINITIONS ==================================== */

void get_vel_from_joystick(float* vx, float* wz) {
    int vy_adc, vx_adc;
    adc_oneshot_get_calibrated_result(adc1_handle, JS_Y_cali, ADC_CHANNEL_3, &vy_adc);
    adc_oneshot_get_calibrated_result(adc1_handle, JS_X_cali, ADC_CHANNEL_4, &vx_adc);

    // ESP_LOGI("JOYSTICK", "Vertical Voltage: %d", vy_adc - joystick_y);
    // ESP_LOGI("JOYSTICK", "Horizontal Voltage: %d", vx_adc - joystick_x);

    /* max out at 1 m/s */
    int voltage_y = vy_adc - joystick_y;
    int voltage_x = vx_adc - joystick_x;
    if (abs(voltage_y) < 50) 
        *wz = 0;
    else
        *wz = MAX_ANG_VEL * (-0.00363636 * voltage_y + 0.0909091);

    if (abs(voltage_x) < 50)
        *vx = 0;
    else
        *vx = MAX_LIN_VEL * (-0.00347826 * voltage_x + 0.0434783);

    ESP_LOGI("JOYSTICK", "vy_adc: %d, vx_adc: %d, wz: %.3f, vx: %.3f", voltage_y, voltage_x, *wz, *vx);

    // *vx = (abs(vx_adc - joystick_x) > 50) ? -(float)vx_adc * (2.0 * MAX_VEL) / 946.0 + MAX_VEL : 0;
    // *wz = (abs(vy_adc - joystick_y) > 50) ? -(float)vy_adc * (6.0 * MAX_VEL) / 946.0 + 3 * MAX_VEL : 0;

    // ESP_LOGI("JOYSTICK", "Forward Velocity: %f m/s", *vx);
    // ESP_LOGI("JOYSTICK", "Turn Velocity: %f m/s", *wz);
}

/**
 * @brief           Turns data about horizontal velocities and rotation into a valid ROS Twist2D packet
 * 
 * @param vx        Desired horizontal velocity with forward as the positive direction
 * @param vy        Desired horizontal velocity with right as the positive direction (not usable by MBot)
 * @param wz        Desired counter-clockwise angular velocity
 * @return          A Twist2D ROS packet as a byte array
 */
uint8_t*
command_serializer(float vx, float vy, float wz) {
    serial_twist2D_t msg = {.vx = vx, .vy = vy, .wz = wz};

    /* Initialize variables for packet */
    size_t msg_len = sizeof(msg);
    uint8_t* msg_serialized = (uint8_t*)(malloc(msg_len));
    uint8_t* packet = (uint8_t*)(malloc(msg_len + ROS_PKG_LEN));

    /* Serialize message and create packet */
    twist2D_t_serialize(&msg, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_VEL_CMD, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);
    return packet;
}

/**
 * @brief           Interrupt service routine for button presses. Changes active bot.
 * 
 * @param arg       uint32_t: GPIO number corresponding to pressed button
 */
static void
buttons_isr_handler(void* arg) {
    uint32_t gpio_num;
    uint32_t ticks;

    gpio_num = (uint32_t)arg;
    ticks = xTaskGetTickCount();
    if ((gpio_num == last_button) && ((ticks - last_press) < 30)) {
        return;
    }
    last_button = gpio_num;
    last_press = ticks;

    if (peer_num == 0) return;
    switch (gpio_num) {
        case B1_PIN:
            curr_bot = (curr_bot + 1) % peer_num;
            break;
        case B2_PIN:
            curr_bot = (curr_bot + peer_num - 1) % peer_num;
            break;
    }
}

/* ISR for switch (change modes) */
/**
 * @brief           Interrupt service routine for switch toggles. Changes host mode.
 * 
 * @param arg       Unused (automatically contains GPIO number as a uint32_t)
 */
static void
switch_isr_handler(void* arg) {
    uint32_t ticks;

    ticks = xTaskGetTickCount();
    if ((ticks - last_switch) < 100) {
        return;
    }
    last_switch = ticks;

    doSerial = !doSerial;
    gpio_set_level(LED1_PIN, !doSerial);

    /* Give switch semaphore */
    xSemaphoreGiveFromISR(switch_sem, NULL);
}

/**
 * @brief           Initializes controller functionality, namely GPIO ports
 */
void
controller_init() {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        /* Configure the ADC */
        .unit_id = ADC_UNIT_1,
    };

    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_3,
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    gpio_config_t b_config = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (0b1 << B1_PIN) | (0b1 << B2_PIN) | (0b1 << B3_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
    };

    gpio_config_t sw_config = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .pin_bit_mask = (0b1 << SW_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
    };

    gpio_config_t led_config = {
        .pin_bit_mask = (0b1 << LED1_PIN) | (0b1 << LED2_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &adc_config));

    adc_cali_create_scheme_curve_fitting(&cali_config, &JS_Y_cali);
    cali_config.chan = ADC_CHANNEL_4;
    adc_cali_create_scheme_curve_fitting(&cali_config, &JS_X_cali);
    calibrate_joystick();

    /* Configure the GPIOs */
    gpio_config(&b_config);
    gpio_config(&sw_config);
    gpio_config(&led_config);

    /* Install gpio isr service */
    gpio_install_isr_service(0);
    
    /* Create a queue to handle gpio event from isr */
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    /* Initialize switch semaphore */
    switch_sem = xSemaphoreCreateBinary();

    /* Hook isr handler for specific gpio pin */
    gpio_isr_handler_add(B1_PIN, buttons_isr_handler, (void*)B1_PIN);
    gpio_isr_handler_add(B2_PIN, buttons_isr_handler, (void*)B2_PIN);
    // gpio_isr_handler_add(B3_PIN, buttons_isr_handler, (void*)B3_PIN);
    gpio_isr_handler_add(SW_PIN, switch_isr_handler, (void*)SW_PIN);
}

/**
 * @brief           Captures joystick default position to reduce drift effects
 * 
 * @param x         X rest value to be measured (value overwritten by function)
 * @param y         Y rest value to be measured (value overwritten by function)
 * @param n         number of samples to take
 */
void
calibrate_joystick(void) {
    int adc_val, x_tmp = 0, y_tmp = 0, n = 1000;
    for (size_t i = 0; i < n; ++i) {
        adc_oneshot_get_calibrated_result(adc1_handle, JS_X_cali, ADC_CHANNEL_4, &adc_val);
        x_tmp += adc_val;
        adc_oneshot_get_calibrated_result(adc1_handle, JS_Y_cali, ADC_CHANNEL_3, &adc_val);
        y_tmp += adc_val;
    }
    joystick_x = x_tmp / n;
    joystick_y = y_tmp / n;
}

#endif