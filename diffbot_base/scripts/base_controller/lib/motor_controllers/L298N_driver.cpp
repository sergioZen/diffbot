#include "L298N_driver.h"


/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/
L298NMotorController::L298NMotorController(uint8_t motor_num)
{
   motor_driver_ = L298N_MotorShield();
   pMotor_ = motor_driver_.getMotor(motor_num);
}

void L298NMotorController::begin()
{
   motor_driver_.begin();
}

void L298NMotorController::setSpeed(int spd)
{
   pMotor_->setSpeed(spd);
}
