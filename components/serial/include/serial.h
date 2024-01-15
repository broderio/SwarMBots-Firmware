#include <memory.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#include "wifi.h"

#ifndef SERIAL_H
#define SERIAL_H

// definitions
#define SYNC_FLAG      0xff //beginning of packet sync flag
#define VERSION_FLAG   0xfe //version flag compatible with ROS2
#define SENSORS_TOPIC  101  //default message topic when a message is sent from the pico to the rpi
#define COMMANDS_TOPIC 102  //default message topic when a message is sent from the rpi to the pico
#define SYNC_HZ        2    //frequency of the sync signal in Hz
#define SYNC_PERIOD_MS (1000 / SYNC_HZ) //period of the sync signal in ms
#define MBOT_OUTPUT_HZ 25   //frequency of the mbot output in Hz
#define MBOT_OUTPUT_PERIOD_MS (1000 / MBOT_OUTPUT_HZ) //period of the mbot output in ms

#define PICO_IN_MSG    sizeof(data_pico) //equal to the size of the data_pico struct
#define RPI_IN_MSG     sizeof(data_rpi)  //equal to the size of the data_rpi struct
#define ROS_HEADER_LEN 7
#define ROS_FOOTER_LEN 1
#define ROS_PKG_LEN    (ROS_HEADER_LEN + ROS_FOOTER_LEN) //length (in bytes) of ros packaging (header and footer)
#define PICO_IN_BYTES                                                                                                  \
    (PICO_IN_MSG + ROS_PKG_LEN) //equal to the size of the data_pico struct data plus bytes for ros packaging
#define RPI_IN_BYTES                                                                                                   \
    (RPI_IN_MSG + ROS_PKG_LEN) //equal to the size of the data_rpi struct data plus bytes for ros packaging

#define TIMESYNC_PERIOD_US 1000000

typedef struct __attribute__((__packed__)) serial_pose2D_t {
    int64_t utime;
    float x;
    float y;
    float theta;
} serial_pose2D_t;

static inline int pose2D_t_deserialize(uint8_t* src, serial_pose2D_t* dest);
static inline int pose2D_t_serialize(serial_pose2D_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_mbot_motor_vel_t {
    int64_t utime;
    float velocity[3]; // [rad/s]
} serial_mbot_motor_vel_t;

static inline int mbot_motor_vel_t_deserialize(uint8_t* src, serial_mbot_motor_vel_t* dest);
static inline int mbot_motor_vel_t_serialize(serial_mbot_motor_vel_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_twist3D_t {
    int64_t utime;
    float vx;
    float vy;
    float vz;
    float wx;
    float wy;
    float wz;
} serial_twist3D_t;

static inline int twist3D_t_deserialize(uint8_t* src, serial_twist3D_t* dest);
static inline int twist3D_t_serialize(serial_twist3D_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_mbot_imu_t {
    int64_t utime;
    float gyro[3];
    float accel[3];
    float mag[3];
    float angles_rpy[3]; // roll (x), pitch (y), yaw, (z)
    float angles_quat[4]; // quaternion
    float temp;
} serial_mbot_imu_t;

static inline int mbot_imu_t_deserialize(uint8_t* src, serial_mbot_imu_t* dest);
static inline int mbot_imu_t_serialize(serial_mbot_imu_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_slam_status_t {
    int64_t utime;
    int32_t slam_mode; // mapping_only=0, action_only=1, localization_only=2, full_slam=3
    char map_path[256]; // Path to where the map is stored.
} serial_slam_status_t;

static inline int slam_status_t_deserialize(uint8_t* src, serial_slam_status_t* dest);
static inline int slam_status_t_serialize(serial_slam_status_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_mbot_motor_pwm_t {
    int64_t utime;
    float pwm[3]; // [-1.0..1.0]
} serial_mbot_motor_pwm_t;

static inline int mbot_motor_pwm_t_deserialize(uint8_t* src, serial_mbot_motor_pwm_t* dest);
static inline int mbot_motor_pwm_t_serialize(serial_mbot_motor_pwm_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_pose3D_t {
    int64_t utime;
    float x;
    float y;
    float z;
    float angles_rpy[3];
    float angles_quat[4];
} serial_pose3D_t;

static inline int pose3D_t_deserialize(uint8_t* src, serial_pose3D_t* dest);
static inline int pose3D_t_serialize(serial_pose3D_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_timestamp_t {
    int64_t utime;
} serial_timestamp_t;

static inline int timestamp_t_deserialize(uint8_t* src, serial_timestamp_t* dest);
static inline int timestamp_t_serialize(serial_timestamp_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_particle_t {
    serial_pose2D_t pose; // (x,y,theta) pose estimate
    serial_pose2D_t parent_pose; // (x,y,theta) of the prior pose the new estimate came from
    double weight; // normalized weight of the particle as computed by the sensor model
} serial_particle_t;

static inline int particle_t_deserialize(uint8_t* src, serial_particle_t* dest);
static inline int particle_t_serialize(serial_particle_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_twist2D_t {
    int64_t utime;
    float vx;
    float vy;
    float wz;
} serial_twist2D_t;

static inline int twist2D_t_deserialize(uint8_t* src, serial_twist2D_t* dest);
static inline int twist2D_t_serialize(serial_twist2D_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_mbot_encoders_t {
    int64_t utime;
    int64_t ticks[3]; // no units
    int32_t delta_ticks[3]; // no units
    int32_t delta_time; // [usec]
} serial_mbot_encoders_t;

static inline int mbot_encoders_t_deserialize(uint8_t* src, serial_mbot_encoders_t* dest);
static inline int mbot_encoders_t_serialize(serial_mbot_encoders_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_joy_t {
    int64_t timestamp;
    float left_analog_X;
    float left_analog_Y;
    float right_analog_X;
    float right_analog_Y;
    float right_trigger;
    float left_trigger;
    float dpad_X;
    float dpad_Y;
    int8_t button_A;
    int8_t button_B;
    int8_t button_2; // not used
    int8_t button_X;
    int8_t button_Y;
    int8_t button_5; // not used
    int8_t button_l1;
    int8_t button_r1;
    int8_t button_l2;
    int8_t button_r2;
    int8_t button_select;
    int8_t button_start;
    int8_t button_12; // not used
    int8_t button_left_analog;
    int8_t button_right_analog;
    int8_t button_15; //not used
} serial_joy_t;

static inline int joy_t_deserialize(uint8_t* src, serial_joy_t* dest);
static inline int joy_t_serialize(serial_joy_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_point3D_t {
    int64_t utime;
    float x;
    float y;
    float z;
} serial_point3D_t;

static inline int point3D_t_deserialize(uint8_t* src, serial_point3D_t* dest);
static inline int point3D_t_serialize(serial_point3D_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_mbot_message_received_t {
    int64_t utime; // Time of confirmation message creation
    int64_t creation_time; // time of message creation (assumption that this will be unique between messages)
    char channel[256]; //name of channel 
} serial_mbot_message_received_t;

static inline int mbot_message_received_t_deserialize(uint8_t* src, serial_mbot_message_received_t* dest);
static inline int mbot_message_received_t_serialize(serial_mbot_message_received_t* src, uint8_t* dest);


typedef struct __attribute__((__packed__)) serial_mbot_slam_reset_t {
    int64_t utime;
    int32_t slam_mode; // mapping_only=0, action_only=1, localization_only=2, full_slam=3
    char slam_map_location[256]; // only necessary when for localization-only and action_only modes
    bool retain_pose; // Whether to keep the pose when resetting.
} serial_mbot_slam_reset_t;

static inline int mbot_slam_reset_t_deserialize(uint8_t* src, serial_mbot_slam_reset_t* dest);
static inline int mbot_slam_reset_t_serialize(serial_mbot_slam_reset_t* src, uint8_t* dest);

enum message_topics {
    MBOT_TIMESYNC = 201,
    MBOT_ODOMETRY = 210,
    MBOT_ODOMETRY_RESET = 211,
    MBOT_VEL_CMD = 214,
    MBOT_IMU = 220,
    MBOT_ENCODERS = 221,
    MBOT_ENCODERS_RESET = 222,
    MBOT_MOTOR_PWM_CMD = 230,
    MBOT_MOTOR_VEL_CMD = 231,
    MBOT_MOTOR_VEL = 232,
    MBOT_MOTOR_PWM = 233,
    MBOT_VEL = 234
};

typedef struct __attribute__((__packed__)) packets_wrapper {
    serial_mbot_encoders_t encoders;
    serial_pose2D_t odom;
    serial_mbot_imu_t imu;
    serial_twist2D_t mbot_vel;
    serial_mbot_motor_vel_t motor_vel;
    serial_mbot_motor_pwm_t motor_pwm;
} packets_wrapper_t;


void write_usb(uint8_t* data, size_t len);
void read_usb(uint8_t* data, size_t len);
uint8_t checksum(uint8_t* addends, int len);
void read_mac_address(uint8_t* mac_address, uint16_t* pkt_len);
void read_header(uint8_t* header_data);
void read_message(uint8_t* msg_data_serialized, uint16_t message_len, char* topic_msg_data_checksum);
void read_packet(uint8_t* pkt_data, uint16_t pkt_len);
int validate_header(uint8_t* header_data);
int validate_message(uint8_t* header_data, uint8_t* msg_data_serialized, uint16_t message_len,
                     char topic_msg_data_checksum);
int encode_msg(uint8_t* MSG, int msg_len, uint16_t TOPIC, uint8_t* ROSPKT, int rospkt_len);

int pose2D_t_deserialize(uint8_t* src, serial_pose2D_t* dest) {
    memcpy(dest, src, sizeof(serial_pose2D_t));
    return 1;
}

int pose2D_t_serialize(serial_pose2D_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_pose2D_t));
    return 1;
}


int mbot_motor_vel_t_deserialize(uint8_t* src, serial_mbot_motor_vel_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_motor_vel_t));
    return 1;
}

int mbot_motor_vel_t_serialize(serial_mbot_motor_vel_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_motor_vel_t));
    return 1;
}


int twist3D_t_deserialize(uint8_t* src, serial_twist3D_t* dest) {
    memcpy(dest, src, sizeof(serial_twist3D_t));
    return 1;
}

int twist3D_t_serialize(serial_twist3D_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_twist3D_t));
    return 1;
}


int mbot_imu_t_deserialize(uint8_t* src, serial_mbot_imu_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_imu_t));
    return 1;
}

int mbot_imu_t_serialize(serial_mbot_imu_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_imu_t));
    return 1;
}


int slam_status_t_deserialize(uint8_t* src, serial_slam_status_t* dest) {
    memcpy(dest, src, sizeof(serial_slam_status_t));
    return 1;
}

int slam_status_t_serialize(serial_slam_status_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_slam_status_t));
    return 1;
}


int mbot_motor_pwm_t_deserialize(uint8_t* src, serial_mbot_motor_pwm_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_motor_pwm_t));
    return 1;
}

int mbot_motor_pwm_t_serialize(serial_mbot_motor_pwm_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_motor_pwm_t));
    return 1;
}


int pose3D_t_deserialize(uint8_t* src, serial_pose3D_t* dest) {
    memcpy(dest, src, sizeof(serial_pose3D_t));
    return 1;
}

int pose3D_t_serialize(serial_pose3D_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_pose3D_t));
    return 1;
}


int timestamp_t_deserialize(uint8_t* src, serial_timestamp_t* dest) {
    memcpy(dest, src, sizeof(serial_timestamp_t));
    return 1;
}

int timestamp_t_serialize(serial_timestamp_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_timestamp_t));
    return 1;
}


int particle_t_deserialize(uint8_t* src, serial_particle_t* dest) {
    memcpy(dest, src, sizeof(serial_particle_t));
    return 1;
}

int particle_t_serialize(serial_particle_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_particle_t));
    return 1;
}


int twist2D_t_deserialize(uint8_t* src, serial_twist2D_t* dest) {
    memcpy(dest, src, sizeof(serial_twist2D_t));
    return 1;
}

int twist2D_t_serialize(serial_twist2D_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_twist2D_t));
    return 1;
}


int mbot_encoders_t_deserialize(uint8_t* src, serial_mbot_encoders_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_encoders_t));
    return 1;
}

int mbot_encoders_t_serialize(serial_mbot_encoders_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_encoders_t));
    return 1;
}


int joy_t_deserialize(uint8_t* src, serial_joy_t* dest) {
    memcpy(dest, src, sizeof(serial_joy_t));
    return 1;
}

int joy_t_serialize(serial_joy_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_joy_t));
    return 1;
}


int point3D_t_deserialize(uint8_t* src, serial_point3D_t* dest) {
    memcpy(dest, src, sizeof(serial_point3D_t));
    return 1;
}

int point3D_t_serialize(serial_point3D_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_point3D_t));
    return 1;
}


int mbot_message_received_t_deserialize(uint8_t* src, serial_mbot_message_received_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_message_received_t));
    return 1;
}

int mbot_message_received_t_serialize(serial_mbot_message_received_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_message_received_t));
    return 1;
}


int mbot_slam_reset_t_deserialize(uint8_t* src, serial_mbot_slam_reset_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_slam_reset_t));
    return 1;
}

int mbot_slam_reset_t_serialize(serial_mbot_slam_reset_t* src, uint8_t* dest) {
    memcpy(dest, src, sizeof(serial_mbot_slam_reset_t));
    return 1;
}

#endif