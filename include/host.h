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

#endif
