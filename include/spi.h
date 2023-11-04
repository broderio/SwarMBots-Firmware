#ifndef SPI_H
#define SPI_H

#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "client.h"

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

int spi_init(void)
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
    // printf("Minimum free heap size: %lu bytes\n", esp_get_minimum_free_heap_size());
    return ret;
}
#endif