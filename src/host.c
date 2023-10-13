#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include "common/comms.h"
#include "common/mbot_lcm_msgs_serial.h"

int main() {

    /* THIS CODE IS FOR CONVERTING A STRUCT INTO A PACKET */
    serial_twist2D_t msg = {
        .vx = 0.0,
        .vy = 0.0,
        .wz = 0.0
    };

    size_t msg_len = sizeof(msg);
    uint8_t* msg_serialized = malloc(msg_len);
    uint8_t* packet = malloc(msg_len + ROS_PKG_LEN);

    twist2D_t_serialize(&msg, msg_serialized);
    encode_msg(msg_serialized, msg_len, MBOT_VEL_CMD, packet, msg_len + ROS_PKG_LEN);
    free(msg_serialized);

    /* THIS CODE IS FOR CONVERTING A PACKET INTO A MESSAGE */
    uint8_t header[ROS_HEADER_LEN];
    read_header(packet, header);
    bool valid_header = validate_header(header);
    if (!valid_header) {
        printf("Invalid header\n");
        return;
    }

    uint16_t message_len = ((uint16_t)header[3] << 8) + (uint16_t)header[2];
    uint16_t topic_id = ((uint16_t)header[6] << 8) + (uint16_t)header[5];
    uint8_t msg_data_serialized[message_len];
    char topic_msg_data_checksum = 0;

    read_message(packet, &msg_data_serialized[0], message_len, &topic_msg_data_checksum);
    bool valid_message = validate_message(header, msg_data_serialized, message_len, topic_msg_data_checksum);
    if (!valid_message) {
        printf("Invalid message\n");
        return;
    }
    twist2D_t_deserialize(msg_data_serialized, &msg);
}