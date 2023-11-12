#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "esp_log.h"
#include "lcm/mbot_lcm_msgs_serial.h"
#include "lcm/comms.h"
#include "mbot_params.h"
#include "driver/gpio.h"

#include "host.h"
#include "wifi.h"
#include "controller.h"

const char *PRINT_TAG = "PRINT_TASK";
const char *SERIAL_TAG = "SERIAL_TASK";
const char *PILOT_TAG = "PILOT_TASK";
const char *ESPNOW_RECV_TAG = "ESPNOW_RECV_TASK";

int joystick_x;
int joystick_y;

// task to print the button causing the interrupt
static void print_task(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            if (io_num == B1_PIN)
                ESP_LOGI(PRINT_TAG, "Button Up");
            else if (io_num == B2_PIN)
                ESP_LOGI(PRINT_TAG, "Button Down");
            else
            {
                if (!doSerial)
                    ESP_LOGI(PRINT_TAG, "Controller mode");
                else
                    ESP_LOGI(PRINT_TAG, "Serial Mode");
            }
        }
    }
}

static void espnow_recv_task(void *args)
{
    espnow_event_recv_t evt;
    uint8_t msg[ESPNOW_DATA_MAX_LEN];
    uint16_t data_len;
    int ret;
    uint8_t mac[MAC_ADDR_LEN];
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_AP, mac);
    if (err != ESP_OK)
    {
        ESP_LOGI(ESPNOW_RECV_TAG, "Could not get mac address, error code %d", err);
    }

    // Delay for 1 second
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Print host mac address
    ESP_LOGI(ESPNOW_RECV_TAG, "Host MAC: " MACSTR, MAC2STR(mac));

    while (xQueueReceive(espnow_recv_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        // ESP_LOGI(ESPNOW_RECV_TAG, "receieved data!");
        // ESP_LOGI(ESPNOW_RECV_TAG, "Received data from: " MACSTR ", len: %d", MAC2STR(evt->mac_addr), evt->data_len);
        
        // Parse incoming packet
        ret = espnow_data_parse(evt.data, evt.data_len, msg, &data_len);
        free(evt.data);

        // Check if data is invalid
        if (ret < 0) {
            ESP_LOGE(ESPNOW_RECV_TAG, "Receive invalid data");
            continue;
        }

        // If message not from known peer, ignore it
        if (!esp_now_is_peer_exist(evt.mac_addr))
        {
            ESP_LOGE(ESPNOW_RECV_TAG, "found a new peer unexpectedly: addr "MACSTR"", MAC2STR(evt.mac_addr));
            continue;
        }

        // Create BOTPKT
        size_t packet_len = data_len + MAC_ADDR_LEN + 4;
        uint8_t* packet = malloc(packet_len);
        packet[0] = 0xff;
        packet[1] = (uint8_t) (data_len%255);
        packet[2] = (uint8_t) (data_len>>8);
        memcpy(packet + 3, evt.mac_addr, MAC_ADDR_LEN);
        msg[MAC_ADDR_LEN + 4] = checksum(evt.mac_addr, MAC_ADDR_LEN);
        memcpy(packet + MAC_ADDR_LEN + 4, msg, data_len);

        // Send message to UART
        uart_write_bytes(UART_PORT_NUM, (const char *)packet, packet_len);
    }
    ESP_LOGE(ESPNOW_RECV_TAG, "Exited task espnow_recv_task loop");
}

static void serial_mode_task(void *arg)
{
    // Suspend immediately if in controller mode
    if (!doSerial)
        vTaskSuspend(NULL);

    ESP_LOGI(SERIAL_TAG, "Serial mode");

    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;
    ESP_ERROR_CHECK(uart_driver_install(0, 2 * ESPNOW_DATA_MAX_LEN, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    uint8_t packet[ESPNOW_DATA_MAX_LEN];
    TickType_t xLastWakeTime;
    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        // Suspend task if we switch modes
        if (xSemaphoreTake(serial_sem, 1) == pdTRUE) {
            ESP_LOGI(SERIAL_TAG, "Switching to pilot mode");
            vTaskResume(pilotMode);
            vTaskSuspend(serialMode);
        }

        // Read mac address and validate
        uint8_t mac_address[MAC_ADDR_LEN];
        uint16_t pkt_len;
        ESP_LOGI(SERIAL_TAG, "Waiting for packet...");
        read_mac_address(mac_address, &pkt_len);
        ESP_LOGI(SERIAL_TAG, "Received packet for " MACSTR, MAC2STR(mac_address));
        
        // Read the ROSPKT
        read_packet(packet, pkt_len);

        // Add peer if mac address is not already
        if (!esp_now_is_peer_exist(mac_address)) {
            ESP_LOGI(SERIAL_TAG, "Adding peer ...");
            if (peer_num >= 8) {
                ESP_LOGE(SERIAL_TAG, "Reached max peer count");
                continue;
            }
            peer_num++;
            esp_now_peer_info_t *peer = peers + peer_num;
            peer->channel = ESPNOW_CHANNEL;
            peer->ifidx = ESPNOW_WIFI_IF;
            peer->encrypt = false;
            memcpy(peer->peer_addr, mac_address, MAC_ADDR_LEN);
            ESP_ERROR_CHECK(esp_now_add_peer(peer));
        }

        // Send packet to client
        espnow_send_param_t send_param;
        memcpy(send_param.dest_mac, mac_address, MAC_ADDR_LEN);
        espnow_data_prepare(&send_param, packet, pkt_len);
        esp_err_t err = esp_now_send(send_param.dest_mac, send_param.buffer, send_param.len);
        if (err != ESP_OK)
        {
            ESP_LOGE(SERIAL_TAG, "Send error %s", esp_err_to_name(err));
        }

        // Check to see if send was successful
        espnow_event_send_t *send_evt;
        xQueueReceive(espnow_send_queue, &send_evt, 1);
        if (xQueueReceive(espnow_send_queue, &send_evt, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(SERIAL_TAG, "Send failed");
        }
        
        xTaskDelayUntil(&xLastWakeTime, 1);
    }
    ESP_LOGE(SERIAL_TAG, "Exited task uart_in_task loop");
}

void pilot_mode_task(void *arg)
{
    // Suspend immediately if in serial mode or if we have no clients paired
    if (doSerial)
        vTaskSuspend(pilotMode);

    ESP_LOGI(PILOT_TAG, "Pilot mode");

    esp_now_peer_info_t peer = peers[curr_bot];
    espnow_send_param_t send_param;
    memset(&send_param, 0, sizeof(espnow_send_param_t));
    send_param.len = 0;
    
    TickType_t xLastWakeTime;

    int vyAdc, vxAdc;
    float max = 1.5;
    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        // Suspend task if we switch modes
        if (xSemaphoreTake(pilot_sem, 1) == pdTRUE) {
            ESP_LOGI(PILOT_TAG, "Switching to serial mode");
            vTaskResume(serialMode);
            vTaskSuspend(pilotMode);
        }

        adc_oneshot_get_calibrated_result(adc1_handle, JS_Y_cali, ADC_CHANNEL_3, &vyAdc);
        adc_oneshot_get_calibrated_result(adc1_handle, JS_X_cali, ADC_CHANNEL_4, &vxAdc);

        // ESP_LOGI(PILOT_TAG, "Vertical Voltage: %d", vy);
        // ESP_LOGI(PILOT_TAG, "Horizontal Voltage: %d", vx);

        // max out at 5 m/s
        float vx = (abs(vyAdc - joystick_y) > 50) ? vyAdc * (2.0 * max) / 946.0 - max : 0;
        float wz = (abs(vxAdc - joystick_x) > 50) ? -vxAdc * (6.0 * max) / 946.0 + 3 * max : 0;

        //ESP_LOGI(PILOT_TAG, "Forward Velocity: %f m/s", vx);
        //ESP_LOGI(PILOT_TAG, "Turn Velocity: %f m/s", wz);

        if (peer_num > 0) {
            peer = peers[curr_bot];
            ESP_LOGI(PILOT_TAG, "Sending to " MACSTR"", MAC2STR(peer.peer_addr));
            memcpy(send_param.dest_mac, peer.peer_addr, MAC_ADDR_LEN);
            uint8_t* packet = command_serializer(vx, 0, wz);
            espnow_data_prepare(&send_param, packet, sizeof(serial_twist2D_t) + ROS_PKG_LEN);
            free(packet);
            // ESP_LOGI(PILOT_TAG, "Heap size (at start) %lu", esp_get_free_heap_size());

            esp_err_t err = esp_now_send(send_param.dest_mac, send_param.buffer, send_param.len);
            if (err != ESP_OK)
            {
                ESP_LOGE(PILOT_TAG, "Send error %s", esp_err_to_name(err));
            }

            // Check to see if send was successful
            espnow_event_send_t *send_evt;
            if (xQueueReceive(espnow_send_queue, &send_evt, portMAX_DELAY) != pdTRUE) {
                ESP_LOGE(PILOT_TAG, "Send failed");
            }
        }
        // ESP_LOGI(PILOT_TAG, "GPIO17: %d\n", gpio_get_level(SW_PIN));
        // ESP_LOGI(PILOT_TAG, "Heap size (at end) %lu", esp_get_free_heap_size());

        vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Init wifi and espnow
    wifi_init();
    espnow_init();

    // Init controller
    controller_init();
    calibrate_joystick(&joystick_x, &joystick_y, 1000);

    // Set the mode given the switch state defined in controller.c
    doSerial = !(bool)gpio_get_level(SW_PIN);

    // uint8_t s_peer_mac[MAC_ADDR_LEN] = {0xEC, 0xDA, 0x3B, 0x8E, 0xD8, 0xDD};
    // add_peer(s_peer_mac);

    // uint8_t s_peer2_mac[MAC_ADDR_LEN] = {0x48, 0x27, 0xE2, 0xFD, 0x59, 0xF1};
    // add_peer(s_peer2_mac);
    
    // Create tasks
    xTaskCreate(espnow_recv_task, "espnow_recv_task", 4096, NULL, 4, NULL);
    xTaskCreate(serial_mode_task, "serial_mode_task", 4096, NULL, 3, &serialMode);
    xTaskCreate(pilot_mode_task, "pilot_mode_task", 4096, NULL, 3, &pilotMode);

    // for debugging
    // xTaskCreate(print_task, "print_task", 2048, NULL, 3, NULL);
}