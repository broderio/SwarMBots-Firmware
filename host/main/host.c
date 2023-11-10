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
    uint8_t msg[ESPNOW_DATA_MAX_LEN];
    uint16_t data_len;
    int ret;
    uint32_t packet_count = 0;
    uint8_t mac[MAC_ADDR_LEN];
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_AP, mac);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "Could not get mac address, error code %d", err);
    }

    // Delay for 1 second
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Print host mac address
    ESP_LOGI(TAG, "Host MAC: " MACSTR, MAC2STR(mac));

    while (xQueueReceive(espnow_recv_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        printf("receieved data!\n");
        //ESP_LOGI(TAG, "Received data from: " MACSTR ", len: %d", MAC2STR(evt->mac_addr), evt->data_len);
        // Parse incoming packet
        ret = espnow_data_parse(evt->data, evt->data_len, msg, &data_len);

        // Check if data is invalid
        if (ret < 0) {
            ESP_LOGE(TAG, "Receive invalid data");
            continue;
        }

        // Check if mac address is paired TODO: how could this happen? Clients only talk to Host after host talks to them?
        if (esp_now_is_peer_exist(evt->mac_addr))
        {
            printf("found a new peer unexpectedly\n");
            // Create BOTPKT
            size_t packet_len = data_len + MAC_ADDR_LEN + 4;
            uint8_t* packet = malloc(packet_len);
            packet[0] = 0xff;
            packet[1] = (uint8_t) (data_len%255);
            packet[2] = (uint8_t) (data_len>>8);
            memcpy(packet + 3, evt->mac_addr, MAC_ADDR_LEN);
            msg[MAC_ADDR_LEN + 4] = checksum(evt->mac_addr, MAC_ADDR_LEN);
            memcpy(packet + MAC_ADDR_LEN + 4, msg, data_len);

            // Send message to UART
            uart_write_bytes(UART_PORT_NUM, (const char *)packet, packet_len);
        }
        ++packet_count;
        if (packet_count == 125){
            packet_count = 0;
            printf("received 125 packets\n");
        }
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
        .baud_rate = 921600,
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
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        printf("Serial Mode\n");
        // Read mac address and validate
        uint8_t mac_address[MAC_ADDR_LEN];
        uint8_t checksum_val;
        uint16_t pkt_len;
        read_mac_address(mac_address, &pkt_len, &checksum_val);
        if (!validate_mac_address(mac_address, pkt_len, checksum_val))
            continue; // continue if mac address is invalid
        
        // Read the ROSPKT
        read_packet(packet, pkt_len);

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

        // Send packet to client
        espnow_send_param_t send_param;
        memcpy(send_param.dest_mac, mac_address, MAC_ADDR_LEN);
        espnow_data_prepare(&send_param, packet, pkt_len);
        if (esp_now_send(send_param.dest_mac, send_param.buffer, send_param.len) != ESP_OK)
        {
            ESP_LOGE(TAG, "Send error");
        }

        // Check to see if send was successful
        espnow_event_send_t *send_evt;
        if (xQueueReceive(espnow_send_queue, &send_evt, 0) != pdTRUE) {
            ESP_LOGE(TAG, "Send failed");
        }
        
        xTaskDelayUntil(&xLastWakeTime, 1);
    }
    ESP_LOGE(TAG, "Exited task uart_in_task loop");
}

void pilot_mode_task(void *arg)
{
    // Suspend immediately if in serial mode
    if (mode)
        vTaskSuspend(NULL);
    int vyAdc, vxAdc;
    float max = 1.5;
    esp_now_peer_info_t peer = peers[curr_bot];
    espnow_send_param_t send_param;
    memset(&send_param, 0, sizeof(espnow_send_param_t));
    send_param.len = 0;
    while (1)
    {
        adc_oneshot_get_calibrated_result(adc1_handle, JS_Y_cali, ADC_CHANNEL_3, &vyAdc);
        adc_oneshot_get_calibrated_result(adc1_handle, JS_X_cali, ADC_CHANNEL_4, &vxAdc);

        // printf("Vertical Voltage: %d\n", vy);
        // printf("Horizontal Voltage: %d\n", vx);

        // max out at 5 m/s
        float vx = (abs(vyAdc - joystick_y) > 50) ? vyAdc * (2.0 * max) / 946.0 - max : 0;
        float wz = (abs(vxAdc - joystick_x) > 50) ? -vxAdc * (6.0 * max) / 946.0 + 3 * max : 0;

        //printf("Forward Velocity: %f m/s\n", vx);
        //printf("Turn Velocity: %f m/s\n", wz);

        peer = peers[curr_bot];
        ESP_LOGI(TAG, "sending to " MACSTR, MAC2STR(peer.peer_addr));
        memcpy(send_param.dest_mac, peer.peer_addr, MAC_ADDR_LEN);
        espnow_data_prepare(&send_param, command_serializer(vx, 0, wz), sizeof(serial_twist2D_t) + ROS_PKG_LEN);
        if (esp_now_send(send_param.dest_mac, send_param.buffer, send_param.len) != ESP_OK)
        {
            ESP_LOGE(TAG, "Send error");
        }
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

    //TODO:Remove later -------------------------------------------------------------
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        esp_now_deinit();
        return;
    }
    
    uint8_t s_peer_mac[MAC_ADDR_LEN] = {0xF4, 0x12, 0xFA, 0xFA, 0x07, 0x51};
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    peers[0] = *peer;

    uint8_t s_peer2_mac[MAC_ADDR_LEN] = {0x48, 0x27, 0xE2, 0xFD, 0x59, 0xF1};
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_peer2_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    peers[1] = *peer;
    peer_num = 2;
    curr_bot = 0;
    //--------------------------------------------------------------------------------------

    // Create tasks
    xTaskCreate(espnow_recv_task, "espnow_recv_task", 4096, NULL, 4, NULL);
    xTaskCreate(serial_mode_task, "serial_mode_task", 2048, NULL, 1, &serialMode);
    xTaskCreate(pilot_mode_task, "pilot_mode_task", 4096, NULL, 1, &controllerMode);

    // for debugging
    // xTaskCreate(print_task, "print_task", 2048, NULL, 3, NULL);
}