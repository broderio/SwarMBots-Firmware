/**
 * @file            spi.h
 * @author          The SwarMBots Team (EECS 473)
 * @brief           Header holding functionalities specific to the client's SPI communication
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

#ifndef SPI_H
#define SPI_H

/* ==================================== INCLUDES ==================================== */

#include "client.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"

/* ==================================== #DEFINED CONSTS ==================================== */

/* ==================================== GLOBAL VARIABLES ==================================== */

/* ==================================== DATA STRUCTS ==================================== */

/* ==================================== FUNCTION PROTOTYPES ==================================== */

void spi_post_setup_cb(const spi_slave_transaction_t* const trans);
void spi_post_trans_cb(const spi_slave_transaction_t* const trans);

int
spi_init(void)

    /* ==================================== FUNCTION DEFINITIONS ==================================== */

    /**
 * @brief           Callback for transaction queue. Sets handshake line high.
 * 
 * @details         Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
 * 
 * @param trans     Queued transaction information
 */
    void spi_post_setup_cb(spi_slave_transaction_t* trans) {
    if (trans->tx_buffer == NULL) {
        gpio_set_level(GPIO_RECV, 1);
    } else {
        gpio_set_level(GPIO_SEND, 1);
    }
}

/**
 * @brief           Callback for transaction send/receive. Sets handshake line low.
 * 
 * @details         Called after transaction is sent/received. We use this to set the handshake line low.
 * 
 * @param trans     Completed transaction information
 */
void
spi_post_trans_cb(spi_slave_transaction_t* trans) {
    if (trans->tx_buffer == NULL) {
        gpio_set_level(GPIO_RECV, 0);
    } else {
        gpio_set_level(GPIO_SEND, 0);
    }
}

/**
 * @brief           Initializes SPI GPIO ports
 * 
 * @return          ESPNOW error code based on setup result 
 */
int
spi_init(void) {
    spi_bus_config_t buscfg;
    spi_slave_interface_config_t slvcfg;
    esp_err_t ret;
    gpio_config_t io_conf;

    /* Configuration for the SPI bus */
    buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    /* Configuration for the SPI slave interface */
    slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 6,
        .flags = 0,
        .post_setup_cb = spi_post_setup_cb,
        .post_trans_cb = spi_post_trans_cb,
    };

    /* Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected. */
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    /* Initialize SPI slave interface */
    ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        printf("Error initializing SPI slave: %d\n", ret);
        return ret;
    }

    /* Initialize GPIO handshake for sending messages to MBoard */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << GPIO_SEND);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        printf("Error configuring GPIO: %d\n", ret);
        return ret;
    }

    /* Initialize GPIO handshake for receiving messages from MBoard */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << GPIO_RECV);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        printf("Error configuring GPIO: %d\n", ret);
        return ret;
    }
    // printf("Minimum free heap size: %lu bytes\n", esp_get_minimum_free_heap_size());
    return ret;
}
#endif