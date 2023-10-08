#include <stdint.h>
#include <stddef.h>
#include <esp_now.h>
#include <driver/uart.h>

// The host_t board will receive messages from the client board over WiFi using the ESP-NOW protocol
// The host_t baord will send messages to the client board over WiFi
// The host_t will send messages to the user's laptop via USB serial
// An interrupt will handle receiving messages from the client board and sending them to the user's laptop
// A main task will handle reading messages from the user's laptop and sending them to the client board

typedef struct host_t {
    esp_now_peer_info_t client; // The client's peer info for ESP-NOW communication
    uart_config_t user; // The user's UART configuration for USB serial communication
    bool user_connected; // Whether the user is connected (determines controller mode or host mode)
} host_t;

// Initializes the host
int host_init(host_t *h);

// Sends data to the client
int host_send_client(host_t *h, uint8_t *data, size_t len);

// Sends data to the user
int host_send_user(host_t *h, uint8_t *data, size_t len);

// ISR for receiving data from the client (this will be deactived when the user is not connected)
void data_recv_client_isr(void *arg);

// Main task for receiving data from the user (this will read from joystick and buttons when user is not connected)
void data_recv_user_task(void *arg);