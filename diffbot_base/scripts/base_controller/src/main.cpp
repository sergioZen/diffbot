//--------------------------------------------------------------------------------
// ESP32 keep rebooting when trying to use rosserial:
//
// Change in: ~/Arduino/libraries/ros_lib/ros.h
// #if defined(ESP8266) or defined(ESP32) or defined(ROSSERIAL_ARDUINO_TCP)
//   #include "ArduinoTcpHardware.h"
// #else
//   #include "ArduinoHardware.h"
// #endif
//
// Into:
// #if defined(ROSSERIAL_ARDUINO_TCP)
//   #include "ArduinoTcpHardware.h"
// #else
//   #include "ArduinoHardware.h"
// #endif
//
// RosSerial Error:
// Add to ArduinoHarwdware.h:
// #define SERIAL_CLASS HardwareSerial // Teensy HW Serial
// #define USE_TEENSY_HW_SERIAL
//
//--------------------------------------------------------------------------------
//#define ROSSERIAL_ARDUINO_TCP
//#define ESP32

/*
#define SERIAL_CLASS HardwareSerial // Teensy HW Serial
#define USE_TEENSY_HW_SERIAL
#define DEBUG
#define WAIT_FOR_DEBUGGER_SECONDS 120
#define WAIT_FOR_SERIAL_SECONDS 20
#ifdef DEBUG //Remove compiler optimizations for hardware debugging
#pragma GCC optimize ("O0") 
#include "Arduino.h"
#include "TeensyDebug.h"
#endif
*/

//#include <WiFi.h>

#include <ros.h>

#include "diffbot_base_config.h"
#include "base_controller.h"
#include "L298N_driver.h"

extern "C" int _gettimeofday(struct timeval *tv, void *ignore) {
  uint32_t hi1 = SNVS_HPRTCMR;
  uint32_t lo1 = SNVS_HPRTCLR;
  while (1) {
    uint32_t hi2 = SNVS_HPRTCMR;  // ref manual 20.3.3.1.3 page 1231
    uint32_t lo2 = SNVS_HPRTCLR;
    if (lo1 == lo2 && hi1 == hi2) {
      tv->tv_sec = (hi2 << 17) | (lo2 >> 15);
      tv->tv_usec = ((lo2 & 0x7FFF) * 15625) >> 9;
      return 0;
    }
    hi1 = hi2;
    lo1 = lo2;
  }
}
 
ros::NodeHandle nh;

using namespace diffbot;

L298NMotorController motor_controller_right = L298NMotorController(MOTOR_RIGHT);
L298NMotorController motor_controller_left = L298NMotorController(MOTOR_LEFT);

BaseController<L298NMotorController, L298N_MotorShield> base_controller(nh, &motor_controller_left, &motor_controller_right);


void setup()
{
    /*
    // START DEBUG
    // Debugger will use second USB Serial; this line is not need if using menu option
    debug.begin();
    delay(3000);
    Serial1.begin(9600);
    debug.begin(SerialUSB1);
    // END DEBUG
    */
 
    base_controller.setup();
    base_controller.init();

    nh.loginfo("Initialize DiffBot Motor Controllers");
    motor_controller_left.begin();
    motor_controller_right.begin();
    nh.loginfo("Setup finished");
}

void loop()
{
    //static bool imu_is_initialized;

    // The main control loop for the base_conroller.
    // This block drives the robot based on a defined control rate
    ros::Duration command_dt = nh.now() - base_controller.lastUpdateTime().control;
    if (command_dt.toSec() >= ros::Duration(1.0 / base_controller.publishRate().control_, 0).toSec())
    {
        base_controller.read();
        base_controller.write();
        base_controller.lastUpdateTime().control = nh.now();
    }

    // This block stops the motor when no wheel command is received
    // from the high level hardware_interface::RobotHW
    command_dt = nh.now() - base_controller.lastUpdateTime().command_received;
    if (command_dt.toSec() >= ros::Duration(E_STOP_COMMAND_RECEIVED_DURATION, 0).toSec())
    {
        nh.logwarn("Emergency STOP");
        base_controller.eStop();
    }

    // This block publishes the IMU data based on a defined imu rate
    /*
    ros::Duration imu_dt = nh.now() - base_controller.lastUpdateTime().imu;
    if (imu_dt.toSec() >= base_controller.publishRate().period().imu_)
    {
        // Sanity check if the IMU is connected
        if (!imu_is_initialized)
        {
            //imu_is_initialized = initIMU();
            if(imu_is_initialized)
                nh.loginfo("IMU Initialized");
            else
                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
        }
        else
        {
            //publishIMU();
        }
        base_controller.lastUpdateTime().imu = nh.now();
    }
    */

    // This block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(base_controller.debug())
    {
        ros::Duration debug_dt = nh.now() - base_controller.lastUpdateTime().debug;
        if (debug_dt.toSec() >= base_controller.publishRate().period().debug_)
        {
            base_controller.printDebug();
            base_controller.lastUpdateTime().debug = nh.now();
        }
    }

    // Call all the callbacks waiting to be called
    nh.spinOnce();
    delay(300);
}
