#include "Arduino/L298N_driver.h"

/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

diffbot::L298NMotorController::L298NMotorController(uint8_t motor_num, uint8_t addrForward, uint8_t addrBackwards)
{
    pMotor_ = motor_driver_.getMotor(motor_num);
    pMotor_.setAddrForward() = addForward;
    pMotor_.setAddrBackwards() = addrBackwards;
}

void diffbot::L298NMotorController::begin(uint16_t freq)
{
    motor_driver_.begin(freq);
}

/*
void initMotorController() {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
}
*/

void diffbot::L298NMotorController::setSpeed(int spd)
{
   unsigned char reverse = 0;

   if (spd < 0)
   {
      spd = -spd;
      reverse = 1;
   }
   if (spd > 255)
      spd = 255;
    
   if (reverse == 0)
   { 
      analogWrite(pMotor_->addForward, spd); analogWrite(pMotor_->addrBackwards, 0);
   }
   else if (reverse == 1)
   { 
      analogWrite(pMotor_->addrBackwards, spd); analogWrite(pMotor_->addForward, 0); 
   }

}
