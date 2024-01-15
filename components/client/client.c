#include "client.h"

/* ==================================== FUNCTION DEFINITIONS ==================================== */

/**
 * @brief                   Helper function for initial ESPNOW setup that finds the host and adds it as a peer
 * 
 * @details                 Waits for a message to be received, then logs the MAC address of the sender as the
 *                          host. All other information contained in the message is discarded.
 * 
 * @param host_mac_addr     The MAC address of the host to be returned (value overwritten by function)
 */
void
connect_to_host(uint8_t* host_mac_addr) {
    espnow_event_recv_t evt;

    uint16_t data_len;
    uint8_t msg[ESPNOW_DATA_MAX_LEN];

    /* Wait for first message to add host as peer */
    if (xQueueReceive(espnow_recv_queue, &evt, portMAX_DELAY) == pdTRUE) {
        esp_now_peer_info_t* peer;
        esp_now_rate_config_t rate_config = {
            .phymode = WIFI_PHY_MODE_HT20,
            .rate = WIFI_PHY_RATE_MCS7_SGI,
        };

        int32_t ret;

        ESP_LOGI(CONNECT_TO_HOST_TAG, "Received message.");

        /* Parse incoming packet */
        ret = espnow_data_parse(evt.data, evt.data_len, msg, &data_len);
        free(evt.data);

        /* Check if data is invalid */
        if (ret != 0) {
            ESP_LOGE(CONNECT_TO_HOST_TAG, "Received invalid data");
        }

        /* Allocate peer */
        peer = malloc(sizeof(esp_now_peer_info_t));
        if (peer == NULL) {
            ESP_LOGE(CONNECT_TO_HOST_TAG, "Malloc peer information fail");
        }
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        peer->channel = ESPNOW_CHANNEL;
        peer->ifidx = ESPNOW_WIFI_IF;
        peer->encrypt = false;
        memcpy(peer->peer_addr, evt.mac_addr, MAC_ADDR_LEN);

        /* Add peer */
        ESP_ERROR_CHECK(esp_now_add_peer(peer));
        free(peer);
        memcpy(host_mac_addr, evt.mac_addr, MAC_ADDR_LEN);
        esp_now_set_peer_rate_config(evt.mac_addr, &rate_config);
        ESP_LOGI(CONNECT_TO_HOST_TAG, "Found host (MAC: " MACSTR ")", MAC2STR(evt.mac_addr));
    }
}