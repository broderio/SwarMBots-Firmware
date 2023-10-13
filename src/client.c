/* SPI Slave example, receiver (uses SPI Slave driver to communicate with sender)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include "common/comms.h"
#include "common/mbot_lcm_msgs_serial.h"

#define GPIO_RECV 3
#define GPIO_SEND 2
#define GPIO_MOSI 11
#define GPIO_MISO 13
#define GPIO_SCLK 12
#define GPIO_CS 10

bool read_spi = false;
SemaphoreHandle_t read_semaphore;

//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans) {
    if (read_spi) gpio_set_level(GPIO_RECV, 1);
    else gpio_set_level(GPIO_SEND, 1);
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans) {
    if (read_spi) gpio_set_level(GPIO_RECV, 0);
    else gpio_set_level(GPIO_SEND, 0);
}

void send_task(void* args) {
    esp_err_t ret;
    int n=0;
    WORD_ALIGNED_ATTR uint8_t sendbuf[84];
    spi_slave_transaction_t t;

    // TODO: change while loop to spin until we recieve a message over wifi
    while(1) { 
        // TODO: fill sendbuf with received wifi data here

        t.length = 84 * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = NULL;

        if (xSemaphoreTake(read_semaphore, portMAX_DELAY) == pdTRUE) {
            read_spi = false;
            ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
            xSemaphoreGive(read_semaphore);
            if (ret != ESP_OK) {
                printf("Error transmitting: 0x%x\n", ret);
                continue;
            }
        }
        printf("Sent %zu bytes\n", t.trans_len / 8);
        ++n;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void recv_task(void* args) {
    esp_err_t ret;
    int n=0;
    WORD_ALIGNED_ATTR uint8_t recvbuf[84];

    spi_slave_transaction_t t;
    while(1) {
        t.length = 84 * 8;
        t.tx_buffer = NULL;
        t.rx_buffer = recvbuf;

        if (xSemaphoreTake(read_semaphore, portMAX_DELAY) == pdTRUE) {
            read_spi = true;
            ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
            xSemaphoreGive(read_semaphore);
            if (ret != ESP_OK) {
                printf("Error transmitting: 0x%x\n", ret);
                continue;
            }
        }

        if (t.trans_len > t.length) continue;

        // TODO: send t.tx_buffer over wifi

        ++n;
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

//Main application
void app_main(void)
{
    printf("Starting SPI test ...\n");
    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg={
        .mode=0,
        .spics_io_num=GPIO_CS,
        .queue_size=6,
        .flags=0,
        .post_setup_cb=my_post_setup_cb,
        .post_trans_cb=my_post_trans_cb
    };
    
    printf("Setting SPI pins ...\n");
    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    //Initialize SPI slave interface
    ret=spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    assert(ret==ESP_OK);

    // Initialize GPIO handshake for sending messages to MBoard
    gpio_config_t io_conf;
    io_conf.intr_type=GPIO_INTR_DISABLE;
    io_conf.mode=GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask=(1<<GPIO_SEND);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) printf("Error configuring GPIO: %d\n", ret);

    // Initialize GPIO handshake for receiving messages from MBoard
    io_conf.intr_type=GPIO_INTR_DISABLE;
    io_conf.mode=GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask=(1<<GPIO_RECV);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) printf("Error configuring GPIO: %d\n", ret);

    printf("Minimum free heap size: %lu bytes\n", esp_get_minimum_free_heap_size());
    printf("SPI Reciever ready.\n");

    // Semaphore for SPI transmissions
    read_semaphore = xSemaphoreCreateBinary();

    // Create tasks
    TaskHandle_t recv_task_handle, send_task_handle;

    // TODO: Check if its faster to not pin to a specific core
    xTaskCreatePinnedToCore(recv_task, "recv_task", 2048*4, NULL, 5, &recv_task_handle, 0);
    xTaskCreatePinnedToCore(send_task, "send_task", 2048*4, NULL, 5, &send_task_handle, 1);

    // Give semaphore to start SPI transmissions
    xSemaphoreGive(read_semaphore);

    while(1) {
        // Spin while tasks run
    }
}