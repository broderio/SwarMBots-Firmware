enum drive_modes{
    MODE_MOTOR_PWM = 0,
    MODE_MOTOR_VEL_OL = 1,
    MODE_MOTOR_VEL_PID = 2, // not currently supported by MBot
    MODE_MBOT_VEL = 3
};

typedef struct robot_t {
    serial_pose2d_t odometry;
    serial_twist2d_t velocity;
    serial_mbot_imu_t imu;
    serial_mbot_encoders_t encoders;
    serial_mbot_motor_vel_t motor_vel;
    serial_mbot_motor_pwm_t motor_pwm;
    int drive_mode;
    mbot_params_t params;
    host_t host;
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