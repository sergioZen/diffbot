

/// Encoder pins
#define ENCODER_LEFT_H1 5
#define ENCODER_LEFT_H2 6
#define ENCODER_RIGHT_H1 7
#define ENCODER_RIGHT_H2 8

/*
#define ENCODER_LEFT_H1 26
#define ENCODER_LEFT_H2 25
#define ENCODER_RIGHT_H1 33
#define ENCODER_RIGHT_H2 32
*/

// Encoder resolution used for initialization 
// will be read from parameter server
#define ENCODER_RESOLUTION 1920

/// Motor L298N
#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2

/* PID diffbot orginal parameters */

#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant

/* PID ros_arduino_firmware project parameters */
/*
#define K_P 20 // P constant
#define K_I 0.3 // I constant
#define K_D 12 // D constant
*/

#define PWM_BITS 8  // PWM Resolution of the microcontroller


#define UPDATE_RATE_CONTROL 20
#define UPDATE_RATE_IMU 1
#define UPDATE_RATE_DEBUG 5

#define E_STOP_COMMAND_RECEIVED_DURATION 5 // Stop motors if no command was received after this amount of seconds



#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -(PWM_MAX)