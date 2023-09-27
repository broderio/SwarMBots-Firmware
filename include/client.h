#include <stdint.h>
#include <stddef.h>

// Topics published by host
#define MBOT_TIMESYNC       201
#define MBOT_ODOMETRY_RESET 211
#define MBOT_ENCODERS_RESET 222
#define MBOT_MOTOR_PWM_CMD  230
#define MBOT_MOTOR_VEL_CMD  231
#define MBOT_VEL_CMD        214

// Topics published by client
#define MBOT_ODOMETRY       210
#define MBOT_IMU            220
#define MBOT_ENCODERS       221
#define MBOT_VEL            234
#define MBOT_MOTOR_VEL      232
#define MBOT_MOTOR_PWM      233

// There are 2 cores on the ESP32. Two interrupts will be used for communication with the host and the mbot.
// The host interrupt will be on core 0 and the mbot interrupt will be on core 1.
// The host interrupt will be triggered by the host sending data to the ESP32 and will be handled by the data_recv_host_isr function.
// The mbot interrupt will be triggered by the mbot sending data to the ESP32 and will be handled by the data_recv_mbot_isr function.
// Upon receiving data from the host, data_recv_host_isr will immediately send that data back to the mbot via SPI.
// Upon receiving data from the mbot, data_recv_mbot_isr will immediately send that data back to the host via Wi-Fi.

typedef struct client_t {
    esp_now_peer_info_t host; // The host's peer info for ESP-NOW communication
    spi_slave_transaction_t spi; // The SPI transaction for communication with the mbot
} client_t;

// Initializes the client
int client_init(client_t *c);

// Sends data to the host
int client_send_host(client_t *c, uint8_t topic_id, uint8_t *data, size_t len);

// Sends data to the mbot
int client_send_mbot(client_t *c, uint8_t topic_id, uint8_t *data, size_t len);

// ISR for receiving data from the host
void data_recv_host_isr(void *arg);

// ISR for receiving data from the mbot
void data_recv_mbot_isr(void *arg);