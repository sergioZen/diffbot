#ifndef _L298N_MotorShield_h_
#define _L298N_MotorShield_h_

#include <inttypes.h>

#define RIGHT_MOTOR_FORWARD  27
#define RIGHT_MOTOR_BACKWARD 14

#define LEFT_MOTOR_FORWARD   12
#define LEFT_MOTOR_BACKWARD  13

#define RIGHT_MOTOR_ENABLE 12
#define LEFT_MOTOR_ENABLE 13

class L298N_MotorShield;

/** Object that controls and keeps state for a single DC motor */
class L298N_DCMotor {
public:
  L298N_DCMotor();
  friend class L298N_MotorShield; ///< Let MotorShield create DCMotors
  void run(uint8_t);
  void setSpeed(uint8_t);
  void setSpeedFine(uint16_t speed);
  void setPinForward(uint8_t pin);
  void setPinBackwards(uint8_t pin);
  void getPinForward();
  void getPinBackwards();
  void fullOn();
  void fullOff();

private:
  uint8_t IN1pinForward, IN2pinBackwards;
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

private:

  L298N_DCMotor dcmotors[2];
};

#endif  // _L298N_MotorShield_h_