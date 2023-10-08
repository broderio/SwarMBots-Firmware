#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "driver/spi_slave.h"

#define ROS_HEADER_LEN 7

uint8_t checksum(uint8_t* addends, int len) {
    //takes in an array and sums the contents then checksums the array
    int sum = 0;
    for (int i = 0; i < len; i++) {
        sum += addends[i];
    }
    return 255 - ( ( sum ) % 256 );
}

// Function to read the header
int read_header(spi_slave_transaction_t* t, uint8_t* header_data) {
    header_data[0] = 0xff;
    memcpy(&header_data[0], &t->rx_buffer[0], ROS_HEADER_LEN);
    return 0;
}

// Function to validate the header
int validate_header(uint8_t* header_data) {
    bool valid_header = (header_data[1] == 0xfe);
    uint8_t cs1_addends[2] = {header_data[2], header_data[3]};
    uint8_t cs_msg_len = checksum(cs1_addends, 2);
    valid_header = valid_header && (cs_msg_len == header_data[4]);
    return valid_header;
}

// Function to read the message
int read_message(spi_slave_transaction_t* t, uint8_t* msg_data_serialized, uint16_t message_len, char* topic_msg_data_checksum) {
    memcpy(&msg_data_serialized[0], &t->rx_buffer[ROS_HEADER_LEN], message_len);
    return 0;
}

// Function to validate the message
int validate_message(uint8_t* header_data, uint8_t* msg_data_serialized, uint16_t message_len, char topic_msg_data_checksum) {
    uint8_t cs2_addends[message_len + 2]; 
    cs2_addends[0] = header_data[5];
    cs2_addends[1] = header_data[6];
    for (int i = 0; i < message_len; i++) {
        cs2_addends[i + 2] = msg_data_serialized[i];
    }
    uint8_t cs_topic_msg_data = checksum(cs2_addends, message_len + 2); 
    bool valid_message = (cs_topic_msg_data == topic_msg_data_checksum);
    return valid_message;
}