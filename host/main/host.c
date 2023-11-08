#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "driver/uart.h"

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
                printf("Button Up\n");
            else if (io_num == B2_PIN)
                printf("Button Down\n");
            else
            {
                if (!mode)
                    printf("Controller mode\n");
                else
                    printf("Serial Mode\n");
            }
        }
    }
}

static void espnow_recv_task(void *args)
{
    espnow_event_recv_t *evt;
    uint8_t *msg;
    int data_len;
    int ret;

    uint8_t mac[MAC_ADDR_LEN];
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_AP, mac);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "Could not get mac address, error code %d", err);
    }

    // Print host mac address
    printf("Host MAC: " MACSTR "\n", MAC2STR(mac));

    while (xQueueReceive(espnow_recv_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        // Allocate memory for message
        int msg_len = data_len + MAC_ADDR_LEN + 1;
        msg = malloc(msg_len);
        if (msg == NULL)
        {
            ESP_LOGE(TAG, "Malloc msg fail");
            continue;
        }

        // Parse incoming packet
        ret = espnow_data_parse(evt->data, evt->data_len, msg, &data_len);
        free(evt->data);

        // Check if data is invalid
        if (ret < 0) {
            ESP_LOGE(TAG, "Receive invalid data");
            free(msg);
            continue;
        }

        // Check if mac address is paired
        if (esp_now_is_peer_exist(evt->mac_addr))
        {
            // Add mac address and checksum to end of message
            memcpy(msg + data_len, evt->mac_addr, MAC_ADDR_LEN);
            msg[data_len + MAC_ADDR_LEN] = checksum(evt->mac_addr, MAC_ADDR_LEN);

            // Send message to UART
            uart_write_bytes(UART_PORT_NUM, (const char *)msg, msg_len);
        }

        // Free message and event
        free(evt);
        free(msg);
    }
    ESP_LOGE(TAG, "Exited task espnow_recv_task loop");
}

static void serial_mode_task(void *arg)
{
    // Suspend immediately if in controller mode
    if (!mode)
        vTaskSuspend(NULL);

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

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // Find valid header
        uint8_t header_data[ROS_HEADER_LEN];
        read_header(header_data);
        if (!validate_header(header_data))
            continue; // continue if header is invalid
        
        // Read message data
        uint16_t msg_len = ((uint16_t)header_data[3] << 8) + (uint16_t)header_data[2];
        uint8_t msg_data_serialized[msg_len];
        char topic_msg_data_checksum = 0;
        read_message(msg_data_serialized, msg_len, &topic_msg_data_checksum);

        // Read mac address and validate
        uint8_t mac_address[MAC_ADDR_LEN];
        uint8_t checksum_val;
        read_mac_address(mac_address, &checksum_val);
        if (!validate_mac_address(mac_address, checksum_val))
            continue; // continue if mac address is invalid

        // Add peer if mac address is not already
        if (!esp_now_is_peer_exist(mac_address)) {
            if (peer_num >= 8) {
                ESP_LOGE(TAG, "Reached max peer count");
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

        // Reconstruct packet without mac address
        size_t data_len = msg_len + ROS_PKG_LEN;
        uint8_t* data = malloc(data_len);
        memcpy(data, header_data, ROS_HEADER_LEN);
        memcpy(data + ROS_HEADER_LEN, msg_data_serialized, msg_len);
        memcpy(data + ROS_HEADER_LEN + msg_len, &topic_msg_data_checksum, 1);

        // Send packet to client
        espnow_send_param_t send_param;
        memcpy(send_param.dest_mac, mac_address, MAC_ADDR_LEN);
        send_to_client(&send_param, data, data_len);

        // Free data
        free(data);

        xTaskDelayUntil(&xLastWakeTime, 1);
    }
    ESP_LOGE(TAG, "Exited task uart_in_task loop");
}

void pilot_mode_task(void *arg)
{
    // Suspend immediately if in serial mode
    if (mode)
        vTaskSuspend(NULL);
    int vy, vx;
    float max = 1.5;

    while (1)
    {
        adc_oneshot_get_calibrated_result(adc1_handle, JS_Y_cali, ADC_CHANNEL_3, &vy);
        adc_oneshot_get_calibrated_result(adc1_handle, JS_X_cali, ADC_CHANNEL_4, &vx);

        // printf("Vertical Voltage: %d\n", vy);
        // printf("Horizontal Voltage: %d\n", vx);

        // max out at 5 m/s
        float vx = (abs(vy - joystick_y) > 50) ? vy * (2.0 * max) / 946.0 - max : 0;
        float wz = (abs(vx - joystick_x) > 50) ? -vx * (6.0 * max) / 946.0 + 3 * max : 0;

        // printf("Forward Velocity: %f m/s\n", vx);
        // printf("Turn Velocity: %f m/s\n", wz);

        esp_now_peer_info_t peer = peers[curr_bot];
        espnow_send_param_t send_param;
        memcpy(send_param.dest_mac, peer.peer_addr, MAC_ADDR_LEN);
        send_to_client(&send_param, command_serializer(vx, 0, wz), sizeof(serial_twist2D_t) + ROS_PKG_LEN);
        
        // printf("GPIO17: %d\n", gpio_get_level(SW_PIN));
        
        vTaskDelay(pdMS_TO_TICKS(100));
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
    mode = !(bool)gpio_get_level(SW_PIN);

    // Create tasks
    xTaskCreate(espnow_recv_task, "host_espnow_task", 4096, NULL, 4, NULL);
    xTaskCreate(serial_mode_task, "uart_in_task", 2048, NULL, 1, &serialMode);
    xTaskCreate(pilot_mode_task, "read_joystick_task", 2048, NULL, 1, &controllerMode);

    // for debugging
    // xTaskCreate(print_task, "print_task", 2048, NULL, 3, NULL);
}