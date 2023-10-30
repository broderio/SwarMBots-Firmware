#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

static const char *TAG = "Controller";

uint32_t last_button = 0;
uint32_t last_time = 0;

static QueueHandle_t gpio_evt_queue = NULL;
static esp_adc_cal_characteristics_t adc1_chars;

//task to print the button causing the interrupt
static void print_task(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if (io_num == 9) printf("Button Up\n");
            else printf("Button Down\n");
        }
    }
}

//task to read the values of a joystick
void read_joystick(void* arg)
{
    uint32_t vertVoltage;
    uint32_t horizVoltage;

    while (1) 
    {
        vertVoltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_3), &adc1_chars);
        horizVoltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_4), &adc1_chars);
        ESP_LOGI(TAG, "Horizontal Value: %ld mV", horizVoltage); 
        ESP_LOGI(TAG, "Vertical Value: %ld mV", vertVoltage);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//ISR for a button press
static void gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    uint32_t ticks = xTaskGetTickCount();
    if ((gpio_num == last_button) && ((ticks - last_time) < 30)) return;
    last_button = gpio_num;
    last_time = ticks;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

//main
void  app_main() {
    //configure the ADC
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);

    //check for failures
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11));

    gpio_config_t GPIO = {};
     //interrupt of rising edge (release button)
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

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(9, gpio_isr_handler, (void*) 9);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(10, gpio_isr_handler, (void*) 10);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(9);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(9, gpio_isr_handler, (void*) 9);

    //start tasks
    //make the print preempt the adc since it happens rarely
    xTaskCreate(print_task, "print_task", 2048, NULL, 10, NULL);
    xTaskCreate(read_joystick, "read_joystick", 2048, NULL, 5, NULL);
}