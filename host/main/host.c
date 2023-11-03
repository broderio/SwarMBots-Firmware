// #include <stdio.h>
// #include <stdint.h>
// #include <stddef.h>
// #include <string.h>
// #include <stdlib.h>
// #include <time.h>
#include <string.h>
// #include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

// #include "driver/spi_slave.h"
// #include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"

#include "nvs_flash.h"
// #include "esp_random.h"
// #include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "lcm/mbot_lcm_msgs_serial.h"
#include "lcm/comms.h"
#include "mbot_params.h"
#include "driver/gpio.h"
//#include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"
// #include "freertos/timers.h"

#include "host.h"
#include "wifi.h"
#include "controller.h"

static int joystick_v;
static int joystick_h;


static int peerNum = 0;

static void host_espnow_task(void *pvParameter)
{
    //set important variables
    espnow_event_t evt;
    uint8_t recv_data[ESPNOW_DATA_MAX_LEN+1];
    int recv_len;
    int ret;

    uint8_t mac[ESP_NOW_ETH_ALEN];
    esp_err_t er = esp_wifi_get_mac(WIFI_IF_AP, mac);
    if (er != ESP_OK) {
        ESP_LOGI(TAG, "Could not get mac address, error code %d", er);
    }

    //wait 3 seconds
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Host MAC: "MACSTR"", MAC2STR(mac));
    ESP_LOGI(TAG, "Start sending broadcast data");

    //TODO: reformat to where host sends then waits a set time for a response
    //wait for response on repeat
    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            //send was (or wasn't) received
            case ESPNOW_SEND_CB:
            {
                //host_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                
                //don't need to do anything after a successful send

                break;
            }
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, recv_data, &recv_len);

                free(recv_cb->data); //free data field allocated in receive callback function

                if (ret == 0) {

                    /* If MAC address does not exist in peer list, add it to peer list and begin sending it messages*/
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        ESP_LOGI(TAG, "Received first data from Client: "MACSTR"", MAC2STR(recv_cb->mac_addr));

                        //add peer
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = false;
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);
                        memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        peerNum++;
                    }
                    else {
                        if (peerNum > 0 && mode) {
                            printf("changing peer\n");
                            esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                            if (peer == NULL) {
                                ESP_LOGE(TAG, "Malloc peer information fail");
                                espnow_deinit(send_param);
                                vTaskDelete(NULL);
                            }
                            memset(peer, 0, sizeof(esp_now_peer_info_t));
                            ESP_ERROR_CHECK( esp_now_fetch_peer(false, peer) );
                            memcpy(send_param->dest_mac, peer->peer_addr, ESP_NOW_ETH_ALEN);
                            free(peer);
                        }
                        //measured rtt around 9ms with host tick rate at 1000Hz and client at 100Hz
                        //forward to computer
                        recv_data[recv_len] = '\n';
                        uart_write_bytes(UART_PORT_NUM, (const char *) recv_data, recv_len+1);

                    }

                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }

    ESP_LOGE(TAG, "Exited task host_espnow_task loop");
}

//TODO: turn into 2 tasks with queue & interrupts for better mediation with computer
static void uart_in_task(void* arg) {
    //suspend immediately if in controller mode
    if(!mode) vTaskSuspend(NULL);
    gpio_isr_handler_remove(B1_PIN);
    gpio_isr_handler_remove(B2_PIN);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    //configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(0, 2 * ESPNOW_DATA_MAX_LEN, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));


    // Create a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(100);

    
    //wait 5 seconds
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    while (1) {
        if (send_param == NULL) {
            ESP_LOGE(TAG, "UART task send param pointer lost");
            free(data);
            vTaskDelete(NULL);
        }

        // Read data from the UART
        int len = uart_read_bytes(0, data, ESPNOW_DATA_MAX_LEN, 20 / portTICK_PERIOD_MS);
        
        if((len > 0) && send_to_client(send_param, data, len) == -1){
            free(data);
            vTaskDelete(NULL);
        }

        xTaskDelayUntil(&xLastWakeTime, 1);
    }

    ESP_LOGE(TAG, "Exited task uart_in_task loop");
}

//task to print the button causing the interrupt
static void print_task(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if (io_num == B1_PIN) printf("Button Up\n");
            else if(io_num == B2_PIN) printf("Button Down\n");
            else{
                if (!mode) printf("Controller mode\n");
                else printf("Serial Mode\n");
            }
        }
    }
}

//task to read the values of a joystick
void read_joystick_task(void* arg)
{
    //suspend immediately if in serial mode
    if (mode) vTaskSuspend(NULL);
    int vertVoltage;
    int horizVoltage;
    //ADJUSTABLE
    float max = 1.5;

    while (1) 
    {
        //uart_flush(0);
        if (send_param == NULL) {
            ESP_LOGE(TAG, "Joystick task send param pointer lost\n");
            vTaskDelete(NULL);
        }

        vertVoltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(JS_Y_PIN), &adc1_chars);
        horizVoltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(JS_X_PIN), &adc1_chars);

        
        //max out at 5 m/s
        float vx = (abs(vertVoltage - joystick_v) > 50)? vertVoltage*(2.0*max)/3120.0 - max:0;
        float wz = (abs(horizVoltage - joystick_h) > 50)? -horizVoltage*(6.0*max)/3120.0 + 3*max:0;
        //printf("Forward Velocity: %f m/s\n", vx); 
        //printf("Turn Velocity: %f m/s\n", wz);
        send_to_client(send_param, command_serializer(vx, 0 ,wz), sizeof(serial_twist2D_t) + ROS_PKG_LENGTH);
        //printf("GPIO17: %d\n", gpio_get_level(SW_PIN));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void  app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();
    espnow_init();
    xTaskCreate(host_espnow_task, "host_espnow_task", 4096, (void*)send_param, 4, NULL);
    controller_init();
  
    //set the mode given the switch state defined in controller.c
    mode = (bool)gpio_get_level(SW_PIN);
    mode = !mode;

    //find the center of the joystick
    for(int i = 0; i < 1000; ++i){
        joystick_v += esp_adc_cal_raw_to_voltage(adc1_get_raw(JS_Y_PIN), &adc1_chars);
        joystick_h += esp_adc_cal_raw_to_voltage(adc1_get_raw(JS_X_PIN), &adc1_chars);
    }
    //average of 1000 readings
    joystick_v = joystick_v/1000;
    joystick_h = joystick_h/1000;

    xTaskCreate(uart_in_task, "uart_in_task", 2048, NULL, 1, &serialMode);  
    //make the print preempt the adc since it happens rarely
    xTaskCreate(read_joystick_task, "read_joystick_task", 2048, NULL, 1, &controllerMode);

    //for debugging 
    //xTaskCreate(print_task, "print_task", 2048, NULL, 3, NULL);
}