#ifndef HOST_AP_H
#define HOST_AP_H

#define UART_PORT_NUM 0
#define JS_Y_PIN 4 // ADC1_CHANNEL3
#define JS_X_PIN 5 // ADC1_CHANNEL4
#define B1_PIN 9
#define B2_PIN 10
#define SW_PIN 17

static esp_now_peer_info_t peers[8];
static int peer_num = 0;

void add_peer(uint8_t* mac_address) {
    memset(&peers[peer_num], 0, sizeof(esp_now_peer_info_t));
    peers[peer_num].channel = ESPNOW_CHANNEL;
    peers[peer_num].ifidx = ESPNOW_WIFI_IF;
    peers[peer_num].encrypt = false;
    memcpy(peers[peer_num].peer_addr, mac_address, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peers[peer_num]));
    esp_now_rate_config_t rate_config = {
            .phymode = WIFI_PHY_MODE_HT40,
            .rate = WIFI_PHY_RATE_MCS7_SGI,
        };
    esp_now_set_peer_rate_config(mac_address, &rate_config);
    peer_num++;
}

#endif
