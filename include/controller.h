#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "inttypes.h"
#include "freertos/FreeRTOS.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "lcm/mbot_lcm_msgs_serial.h"
#include "lcm/comms.h"
#include "mbot_params.h"
#include "host.h"
#include "esp_log.h"
#include "wifi.h"

static size_t curr_bot = 0;

static uint32_t last_button = 0;
static uint32_t last_press = 0;
static uint32_t last_switch = 0;

static QueueHandle_t gpio_evt_queue = NULL;
adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t JS_Y_cali;
static adc_cali_handle_t JS_X_cali;

static bool mode = 1;

TaskHandle_t serialMode;
TaskHandle_t controllerMode;

uint8_t *command_serializer(float vx, float vy, float wz)
{
    serial_twist2D_t msg = {
        .vx = vx,
        .vy = vy,
        .wz = wz};

    // Initialize variables for packet
    size_t msg_len = sizeof(msg);
    uint8_t *msg_serialized = (uint8_t *)(malloc(msg_len));
    uint8_t *packet = (uint8_t *)(malloc(msg_len + ROS_PKG_LEN));

    // Serialize message and create packet
    twist2D_t_serialize(&msg, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_VEL_CMD, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);
    return packet;
}

// ISR for a button press
static void buttons_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    uint32_t ticks = xTaskGetTickCount();
    if ((gpio_num == last_button) && ((ticks - last_press) < 30))
        return;
    last_button = gpio_num;
    last_press = ticks;
    if (gpio_num == B1_PIN) curr_bot = (curr_bot + 1) % peer_num;
    else curr_bot = (curr_bot == 0)? peer_num - 1: curr_bot - 1;
}

// ISR for switch (change modes)
static void switch_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    uint32_t ticks = xTaskGetTickCount();
    if ((ticks - last_switch) < 100)
        return;
    last_switch = ticks;
    if (mode)
    {
        gpio_isr_handler_add(B1_PIN, buttons_isr_handler, (void *)B1_PIN);
        gpio_isr_handler_add(B2_PIN, buttons_isr_handler, (void *)B2_PIN);
        vTaskSuspend(serialMode);
        vTaskResume(controllerMode);
    }
    else
    {
        gpio_isr_handler_remove(B1_PIN);
        gpio_isr_handler_remove(B2_PIN);
        vTaskSuspend(controllerMode);
        vTaskResume(serialMode);
    }
    mode = !mode;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void controller_init()
{
    // Configure the ADC
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_3,
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_cali_create_scheme_curve_fitting(&cali_config, &JS_Y_cali);
    cali_config.chan = ADC_CHANNEL_4;
    adc_cali_create_scheme_curve_fitting(&cali_config, &JS_X_cali);

    // Configure the GPIOs
    gpio_config_t b_config = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (0b1 << B1_PIN) | (0b1 << B2_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1
    };
    gpio_config(&b_config);

    gpio_config_t sw_config = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .pin_bit_mask = (0b1 << SW_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1
    };
    gpio_config(&sw_config);

    // Install gpio isr service
    gpio_install_isr_service(0);

    // Create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Hook isr handler for specific gpio pin
    gpio_isr_handler_add(B1_PIN, buttons_isr_handler, (void *)B1_PIN);
    gpio_isr_handler_add(B2_PIN, buttons_isr_handler, (void *)B2_PIN);
    gpio_isr_handler_add(SW_PIN, switch_isr_handler, (void *)SW_PIN);
}

void calibrate_joystick(int* x, int* y, size_t n) {
    int adc_val;
    int x_tmp = 0;
    int y_tmp = 0;
    for (size_t i = 0; i < n; ++i)
    {
        adc_oneshot_get_calibrated_result(adc1_handle, JS_X_cali, ADC_CHANNEL_4, &adc_val);
        x_tmp += adc_val;
        adc_oneshot_get_calibrated_result(adc1_handle, JS_Y_cali, ADC_CHANNEL_3, &adc_val);
        y_tmp += adc_val;
    }
    *x = x_tmp / n;
    *y = y_tmp / n;
}

#endif