#ifndef _L298N_MotorShield_h_
#define _L298N_MotorShield_h_

#include <inttypes.h>

#define RIGHT_MOTOR_PWM      19
#define RIGHT_MOTOR_FORWARD  9
#define RIGHT_MOTOR_BACKWARD 10

#define LEFT_MOTOR_PWM       18
#define LEFT_MOTOR_FORWARD   3
#define LEFT_MOTOR_BACKWARD  4

#define RIGHT_MOTOR_ENABLE 0
#define LEFT_MOTOR_ENABLE 0

#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

class L298N_MotorShield;

/** Object that controls and keeps state for a single DC motor */
class L298N_DCMotor {
public:
  L298N_DCMotor();
  friend class L298N_MotorShield; ///< Let MotorShield create DCMotors
  void run(uint8_t);
  void setSpeed(int);
  void setSpeedFine(uint16_t speed);
  void setPinForward(uint8_t pin);
  void setPinBackwards(uint8_t pin);
  void getPinForward();
  void getPinBackwards();
  void fullOn();
  void fullOff();

private:
  uint8_t PWMpin, IN1pinForward, IN2pinBackwards;
  L298N_MotorShield *MC;
  uint8_t motornum;
};

/** Object that controls and keeps state for the whole motor shield.
    Use it to create DC and Stepper motor objects! */
class L298N_MotorShield {
public:
  L298N_MotorShield();

  bool begin();

  L298N_DCMotor *getMotor(uint8_t n);

  friend class L298N_DCMotor; ///< Let DCMotors control the Shield

  void setPWM(uint8_t pin, uint16_t val);
  void setPin(uint8_t pin, int val);  

private:

  L298N_DCMotor dcmotors[2];
};

#endif  // _L298N_MotorShield_h_