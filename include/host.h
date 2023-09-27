// The host_t board will receive messages from the client board over WiFi using the ESP-NOW protocol
// The host_t baord will send messages to the client board over WiFi
// The host_t will send messages to the user's laptop via USB serial
// An interrupt will handle receiving messages from the client board and sending them to the user's laptop
// A main task will handle reading messages from the user's laptop and sending them to the client board

typedef struct host_t {
    esp_now_peer_info_t client;
    uart_config_t user;
} host_t;

// Initializes the host
int host_init(host_t *h);

// Sends data to the client
int host_send_client(host_t *h, uint8_t *data, size_t len);

// Sends data to the user
int host_send_user(host_t *h, uint8_t *data, size_t len);

// ISR for receiving data from the client
void data_recv_client_isr(void *arg);

// Main task for receiving data from the user
void data_recv_user_task(void *arg);