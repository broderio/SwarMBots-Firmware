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
#include "driver/uart.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "host.h"


static const char *TAG = "host";

const gpio_config_t* GPIO9;
const gpio_config_t* GPIO10;


void  app_main() {
    esp_err_t err;
    err = gpio_config(GPIO9);
    err = gpio_config(GPIO10);
    xTaskCreate(uart_in_task, "uart_in_task", 2048, send_param, 3, NULL);  
}
