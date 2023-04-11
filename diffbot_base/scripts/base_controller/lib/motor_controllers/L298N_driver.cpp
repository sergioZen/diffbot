#include "L298N_driver.h"


/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/
diffbot::L298NMotorController::L298NMotorController(uint8_t motor_num)
{
   motor_driver_ = L298N_MotorShield();
   pMotor_ = motor_driver_.getMotor(motor_num);
}

void diffbot::L298NMotorController::begin()
{
   motor_driver_.begin();
}

void diffbot::L298NMotorController::setSpeed(int value)
{
    if (value > 0)
    {
        pMotor_->run(FORWARD);
    }
    else if (value < 0)
    {
        pMotor_->run(BACKWARD);
        // AdafruAdafruit_MotorShield requires uint8 values from 0 to 255
        // Make sure to convert the negative value to a positive one
        value = value * -1;
    }
    else // zero speed
    {
        // Cut power to the motor
        pMotor_->run(RELEASE);
    }

    pMotor_->setSpeed(value);
}
