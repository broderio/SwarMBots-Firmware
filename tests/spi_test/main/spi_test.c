#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include <esp_timer.h>

#include "client.h"
#include "lcm/mbot_lcm_msgs_serial.h"
#include "lcm/lcm_config.h"
#include "lcm/comms_common.h"

#define INCLUDE_vTaskDelay 1

#define GPIO_RECV 3
#define GPIO_SEND 2
#define GPIO_MOSI 11
#define GPIO_MISO 13
#define GPIO_SCLK 12
#define GPIO_CS 10

// Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    if (trans->tx_buffer == NULL)
        gpio_set_level(GPIO_RECV, 1);
    else
        gpio_set_level(GPIO_SEND, 1);
}

// Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    if (trans->tx_buffer == NULL)
        gpio_set_level(GPIO_RECV, 0);
    else
        gpio_set_level(GPIO_SEND, 0);
}

void send_task(void *args)
{
    esp_err_t ret;
    spi_slave_transaction_t t;
    TickType_t xLastWakeTime;
    serial_mbot_motor_vel_t motor_cmd = {
        .utime = esp_timer_get_time(),
        .velocity = {0, 0, 0}
    };
    int n = 0;
    float step = 0.1;
    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();
        size_t len = sizeof(serial_mbot_motor_vel_t) + ROS_PKG_LENGTH;
        uint8_t* packet = malloc(len);
        uint8_t* msg = malloc(sizeof(serial_mbot_motor_vel_t));
        mbot_motor_vel_t_serialize(&motor_cmd, msg);
        encode_msg(msg, sizeof(serial_mbot_motor_vel_t), MBOT_MOTOR_VEL_CMD, packet, len);
        free(msg);

        t.length = len * 8;
        t.tx_buffer = packet;
        t.rx_buffer = NULL;
        printf("Received packet. Waiting for lock...\n");
        ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
        free(packet);
        if (ret != ESP_OK)
        {
            printf("Error transmitting: 0x%x\n", ret);
            continue;
        }
        printf("Sent %zu bytes\n", t.trans_len / 8);
        motor_cmd.utime = esp_timer_get_time();
        if (n < 10) {
            motor_cmd.velocity[0] += step;
            motor_cmd.velocity[1] += step;
        }
        else if (n < 20) {
            motor_cmd.velocity[0] -= step;
            motor_cmd.velocity[1] -= step;
        }
        n = (n + 1) % 20;
        xTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
    }
}

void recv_task(void *args)
{
    esp_err_t ret;
    WORD_ALIGNED_ATTR uint8_t recvbuf[84];

    spi_slave_transaction_t t;
    while (1)
    {
        t.length = 84 * 8;
        t.tx_buffer = NULL;
        t.rx_buffer = recvbuf;

        printf("Waiting for packet...\n");
        ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
        if (ret != ESP_OK)
        {
            printf("Error transmitting: 0x%x\n", ret);
            continue;
        }

        if (t.trans_len > t.length) {
            printf("Invalid message.\n");
            continue;
        }
    }
}

int client_spi_init(void)
{
    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    // Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 6,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb};

    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    // Initialize SPI slave interface
    esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        printf("Error initializing SPI slave: %d\n", ret);
        return ret;
    }

    // Initialize GPIO handshake for sending messages to MBoard
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << GPIO_SEND);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        printf("Error configuring GPIO: %d\n", ret);
        return ret;
    }

    // Initialize GPIO handshake for receiving messages from MBoard
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << GPIO_RECV);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        printf("Error configuring GPIO: %d\n", ret);
        return ret;
    }
    return ret;
}

// Main application
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    printf("Initializing SPI...\n");
    client_spi_init();

    // Create tasks
    TaskHandle_t recv_task_handle, send_task_handle;

    xTaskCreate(recv_task, "recv_task", 2048 * 4, NULL, 5, &recv_task_handle);
    xTaskCreate(send_task, "send_task", 2048 * 4, NULL, 5, &send_task_handle);
}