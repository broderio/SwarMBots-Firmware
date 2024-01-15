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

#ifndef SPI_H
#define SPI_H

/* ==================================== INCLUDES ==================================== */

#include "client.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"

/* ==================================== #DEFINED CONSTS ==================================== */

/* ==================================== GLOBAL VARIABLES ==================================== */

SemaphoreHandle_t spi_mutex;             /**< Mutex for SPI communication */

/* ==================================== DATA STRUCTS ==================================== */

/* ==================================== FUNCTION PROTOTYPES ==================================== */

void spi_post_setup_cb(spi_slave_transaction_t* trans);
void spi_post_trans_cb(spi_slave_transaction_t* trans);

int spi_init(void);
void spi_transaction(spi_slave_transaction_t *trans);

#endif /* SPI_H */