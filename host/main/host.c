#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"

#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "esp_log.h"
#include "esp_adc_cal.h"

#include "nvs_flash.h"

#include "host.h"

#define ESPNOW_MAXDELAY (size_t) 0xffffffff

static const char *TAG = "host";

uint32_t switch_state;
uint32_t last_button = 0;
uint32_t last_press = 0;
uint32_t last_switch = 0;

bool mode = 1;
TaskHandle_t serialMode;
TaskHandle_t controllerMode;

static QueueHandle_t gpio_evt_queue = NULL;
static esp_adc_cal_characteristics_t adc1_chars;
static QueueHandle_t s_host_espnow_queue;

static void host_espnow_deinit(host_espnow_send_param_t *send_param);

//ISR for a button press
static void buttons_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    uint32_t ticks = xTaskGetTickCount();
    if ((gpio_num == last_button) && ((ticks - last_press) < 30)) return;
    last_button = gpio_num;
    last_press = ticks;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

//ISR for switch (change modes)
static void switch_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    uint32_t ticks = xTaskGetTickCount();
    if ((ticks - last_switch) < 100) return;
    last_switch = ticks;
    if(mode) {
        vTaskSuspend(serialMode);
        vTaskResume(controllerMode);
    }
    else{
        vTaskSuspend(controllerMode);
        vTaskResume(serialMode);
    }
    mode = !mode;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/* WiFi should start before using ESPNOW */
static void host_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) ); //sets to use only ram for storage
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) ); //AP
    ESP_ERROR_CHECK( esp_wifi_start());

    //NOTE: may need to add channel negotiation logic
    ESP_ERROR_CHECK( esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void host_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    host_espnow_event_t evt;
    host_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = HOST_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_host_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void host_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    host_espnow_event_t evt;
    host_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = HOST_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_host_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
//params 1 and 2 are raw data info, params 3-n are data fields to populate
int host_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t* msg, int* len)
{
    comm_espnow_data_t *buf = (comm_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(comm_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }
    if (data_len != sizeof(comm_espnow_data_t) + buf->len) {
        ESP_LOGE(TAG, "Receive ESPNOW data has wrong length, len:%d, expected len:%d", data_len, sizeof(comm_espnow_data_t) + buf->len);
        return -1;
    }
    
    *len = buf->len < ESPNOW_DATA_MAX_LEN ? buf->len : ESPNOW_DATA_MAX_LEN;
    memcpy(msg, buf->payload, *len);

    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return 0;
    }

    return -1;
}

/* Prepare ESPNOW data to be sent. */
void host_espnow_data_prepare(host_espnow_send_param_t *send_param, uint8_t* data, int len)
{
    comm_espnow_data_t *buf = (comm_espnow_data_t *)send_param->buffer;
    
    send_param->len = len + sizeof(comm_espnow_data_t);

    assert(len <= ESPNOW_DATA_MAX_LEN);

    buf->crc = 0;
    buf->len = len;
    
    /* Only fill payload if there is room to fit it */
    if (sizeof(comm_espnow_data_t) + buf->len <= send_param->len) {
        memcpy(buf->payload, data, len);
    }
    
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len); //compute checksum to send with packet
}

static void host_espnow_task(void *pvParameter)
{
    //set important variables
    host_espnow_event_t evt;
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

    //recover send param
    host_espnow_send_param_t *send_param = (host_espnow_send_param_t *)pvParameter;

    //wait for response on repeat
    while (xQueueReceive(s_host_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            //send was (or wasn't) received
            case HOST_ESPNOW_SEND_CB:
            {
                //host_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                
                //don't need to do anything after a successful send

                break;
            }
            case HOST_ESPNOW_RECV_CB:
            {
                host_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                ret = host_espnow_data_parse(recv_cb->data, recv_cb->data_len, recv_data, &recv_len);

                free(recv_cb->data); //free data field allocated in receive callback function

                if (ret == 0) {

                    /* If MAC address does not exist in peer list, add it to peer list and begin sending it messages*/
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        ESP_LOGI(TAG, "Received first data from Client");

                        //add peer
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            host_espnow_deinit(send_param);
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
                    }
                    else {

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

static host_espnow_send_param_t* host_espnow_init(void)
{

    //queue semaphore of espnow requests to handle with task
    s_host_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(host_espnow_event_t));
    if (s_host_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return NULL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(host_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(host_espnow_recv_cb) );

    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)ESPNOW_PMK) );

    /* Initialize info about data to send. */
    host_espnow_send_param_t *send_param;

    send_param = malloc(sizeof(host_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_host_espnow_queue);
        esp_now_deinit();
        return NULL;
    }
    memset(send_param, 0, sizeof(host_espnow_send_param_t));
    send_param->len = 0;
    send_param->buffer = malloc(ESPNOW_DATA_MAX_LEN + sizeof(comm_espnow_data_t));
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_host_espnow_queue);
        esp_now_deinit();
        return NULL;
    }

    xTaskCreate(host_espnow_task, "host_espnow_task", 4096, send_param, 4, NULL);

    return send_param;
}

//handles error by cleaning up param and deinitializing wifi
static void host_espnow_deinit(host_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_host_espnow_queue);
    esp_now_deinit();
}


//TODO: turn into 2 tasks with queue & interrupts for better mediation with computer
static void uart_in_task(void* arg) {

    //recover send param for forwarding uart -> wifi
    host_espnow_send_param_t *send_param = (host_espnow_send_param_t *)arg;

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
        
        //if read data, forward to client
        if (len) {
            host_espnow_data_prepare(send_param, data, len);

            esp_err_t er = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
            if (er != ESP_OK) {
                ESP_LOGE(TAG, "Send error: %x", er);
                free(data);
                vTaskDelete(NULL);
            }
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
            if (io_num == 9) printf("Button Up\n");
            else if(io_num == 17) printf("Switch\n");
            else printf("Button Down\n");
        }
    }
}

//task to read the values of a joystick
void read_joystick_task(void* arg)
{
    uint32_t vertVoltage;
    uint32_t horizVoltage;

    while (1) 
    {
        vertVoltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_3), &adc1_chars);
        horizVoltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_4), &adc1_chars);
        printf("Horizontal Value: %ld mV\n", horizVoltage); 
        printf("Vertical Value: %ld mV\n", vertVoltage);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void  app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    host_wifi_init();
    host_espnow_send_param_t* send_param = host_espnow_init();

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
    //configure switch interrupt
    GPIO.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO4/5 here
    GPIO.pin_bit_mask = (0b1 << 17);
    //set as input mode
    GPIO.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    GPIO.pull_down_en = 1;
    gpio_config(&GPIO);

    //install gpio isr service
    gpio_install_isr_service(0);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(9, buttons_isr_handler, (void*) 9);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(10, buttons_isr_handler, (void*) 10);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(17, switch_isr_handler, (void*) 17);

    //set the mode given the switch state
    mode = (bool)gpio_get_level(17);
    
    xTaskCreate(uart_in_task, "uart_in_task", 2048, send_param, 1, &serialMode);  
    //make the print preempt the adc since it happens rarely
    xTaskCreate(print_task, "print_task", 2048, NULL, 3, &controllerMode);
    xTaskCreate(read_joystick_task, "read_joystick_task", 2048, NULL, 2, &controllerMode);

    if(mode)  vTaskSuspend(serialMode);
    else vTaskSuspend(controllerMode);
    mode = !mode;
}


// serial_twist2D_t msg = {
//         .vx = 0.0,
//         .vy = 0.0,
//         .wz = 0.0
//     };

//     // Initialize variables for packet
//     size_t msg_len = sizeof(msg);
//     uint8_t* msg_serialized = malloc(msg_len);
//     uint8_t* packet = malloc(msg_len + ROS_PKG_LEN);

//     // Serialize message and create packet
//     twist2D_t_serialize(&msg, msg_serialized);
//     encode_msg(msg_serialized, msg_len, MBOT_VEL_CMD, packet, msg_len + ROS_PKG_LEN);
//     free(msg_serialized);
// /* THIS CODE IS FOR CONVERTING A PACKET INTO A MESSAGE */
//     uint8_t header[ROS_HEADER_LEN];
//     read_header(packet, header);
//     bool valid_header = validate_header(header);
//     if (!valid_header) {
//         printf("Invalid header\n");
//         return;
//     }

//     uint16_t message_len = ((uint16_t)header[3] << 8) + (uint16_t)header[2];
//     uint16_t topic_id = ((uint16_t)header[6] << 8) + (uint16_t)header[5];
//     uint8_t msg_data_serialized[message_len];
//     char topic_msg_data_checksum = 0;

//     read_message(packet, &msg_data_serialized[0], message_len, &topic_msg_data_checksum);
//     bool valid_message = validate_message(header, msg_data_serialized, message_len, topic_msg_data_checksum);
//     if (!valid_message) {
//         printf("Invalid message\n");
//         return;
//     }
//     twist2D_t_deserialize(msg_data_serialized, &msg);
