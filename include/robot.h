#include <stdint.h>

enum drive_modes{
    MODE_MOTOR_PWM = 0,
    MODE_MOTOR_VEL_OL = 1,
    MODE_MOTOR_VEL_PID = 2, // not currently supported by MBot
    MODE_MBOT_VEL = 3
};

typedef struct robot_t {
    serial_pose2d_t odometry; // The robot's current odometry
    serial_twist2d_t velocity; // The robot's current velocity
    serial_mbot_imu_t imu; // The robot's current IMU values
    serial_mbot_encoders_t encoders; // The robot's current encoder values
    serial_mbot_motor_vel_t motor_vel_goal; // The robot's goal motor velocity
    serial_mbot_motor_vel_t motor_vel; // The robot's current motor velocity
    serial_mbot_motor_pwm_t motor_pwm; // The robot's current motor PWM
    int drive_mode; // The robot's current drive mode
    mbot_params_t params; // The robot's physical parameters
    uint8_t mac_address[6]; // The robot's MAC address for esp-now communication
} robot_t;

// Initializes the robot
int robot_init(robot_t *r);

// Resets the odometry
int robot_reset_odometry(robot_t *r);

// Resets the encoders
int robot_reset_encoders(robot_t *r);

// Sets the velocity
int robot_set_vel(robot_t *r, float linear, float angular);

// Sets the drive mode
int robot_set_drive_mode(robot_t *r, int mode);

// Sets the motor velocity if the drive mode is MODE_MOTOR_VEL_OL
int robot_set_motor_vel(robot_t *r, float left, float right);

// Sets the motor PWM if the drive mode is MODE_MOTOR_PWM
int robot_set_motor_pwm(robot_t *r, float left, float right);

// Callback function for reading data from the host via USB serial
void data_read_host_cb(void* args);