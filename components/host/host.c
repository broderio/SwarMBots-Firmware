#include "host.h"

/* ==================================== FUNCTION DEFINITIONS ==================================== */

/**
 * @brief               Saves a new peer
 * 
 * @details             Adds a new peer with the given MAC address to the ESPNOW peer list as well
 *                      as the local one, and increments \c peer_num
 * 
 * @param mac_address   MAC address of new peer
 */
void
add_peer(uint8_t* mac_address) {
    esp_now_rate_config_t rate_config = {
        .phymode = WIFI_PHY_MODE_HT20,
        .rate = WIFI_PHY_RATE_MCS7_SGI,
    };

    memset(&peers[peer_num], 0, sizeof(esp_now_peer_info_t));
    peers[peer_num].channel = ESPNOW_CHANNEL;
    peers[peer_num].ifidx = ESPNOW_WIFI_IF;
    peers[peer_num].encrypt = false;
    memcpy(peers[peer_num].peer_addr, mac_address, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peers[peer_num]));
    esp_now_set_peer_rate_config(mac_address, &rate_config);
    ++peer_num;
}

/**
 * @brief           Creates a timesync packet
 * 
 * @param time      The current time in microseconds
 * @return          A Timestamp ROS packet as a byte array
 */
uint8_t*
create_timesync_packet(uint64_t time) {
    serial_timestamp_t msg = {
        .utime = time
    };

    // Initialize variables for packet
    size_t msg_len = sizeof(msg);
    uint8_t *msg_serialized = (uint8_t *)(malloc(msg_len));
    uint8_t *packet = (uint8_t *)(malloc(msg_len + ROS_PKG_LEN));

    // Serialize message and create packet
    timestamp_t_serialize(&msg, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_TIMESYNC, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);
    return packet;
}