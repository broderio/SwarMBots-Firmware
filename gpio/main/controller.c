#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "esp_adc/adc_continuous.h"

uint32_t last_button = 0;
uint32_t last_time = 0;

static QueueHandle_t gpio_evt_queue = NULL;

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if (io_num == 9) printf("Button Up\n");
            else printf("Button Down\n");
        }
    }
}

static void gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    uint32_t ticks = xTaskGetTickCount();
    if ((gpio_num == last_button) && ((ticks - last_time) < 30)) return;
    last_button = gpio_num;
    last_time = ticks;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void  app_main() {
    gpio_config_t GPIO = {};
     //interrupt of rising edge
    GPIO.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    GPIO.pin_bit_mask = (0b1 << 9) | (0b1 << 10);
    //set as input mode
    GPIO.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    GPIO.pull_up_en = 1;
    gpio_config(&GPIO);

    //install gpio isr service
    gpio_install_isr_service(0);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(9, gpio_isr_handler, (void*) 9);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(10, gpio_isr_handler, (void*) 10);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(9);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(9, gpio_isr_handler, (void*) 9);
}
