#include <inttypes.h>

class L298N_MotorShield;

/** Object that controls and keeps state for a single DC motor */
class L298N_DCMotor {
public:
  L298N_DCMotor(void);
  friend class L298N_MotorShield; ///< Let MotorShield create DCMotors
  void run(uint8_t);
  void setSpeed(uint8_t);
  void setSpeedFine(uint16_t speed);
  void fullOn();
  void fullOff();

private:
  uint8_t PWMpin, IN1pin, IN2pin;
  L298N_MotorShield *MC;
  uint8_t motornum;
};

/** Object that controls and keeps state for the whole motor shield.
    Use it to create DC and Stepper motor objects! */
class L298N_MotorShield {
public:
  L298N_MotorShield(uint8_t addr = 0x60);

  L298N_DCMotor *getMotor(uint8_t n);

  friend class L298N_DCMotor; ///< Let DCMotors control the Shield

  void setPWM(uint8_t pin, uint16_t val);
  void setPin(uint8_t pin, bool val);

private:
  uint8_t _addr;
  uint16_t _freq;

  L298N_DCMotor dcmotors[2];
};