
#include "Adafruit_MotorShield.h"
#include "Arduino.h"

#if (MICROSTEPS == 8)
///! A sinusoial microstepping curve for the PWM output (8-bit range) with 9
/// points - last one is start of next step.
static uint8_t microstepcurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
///! A sinusoial microstepping curve for the PWM output (8-bit range) with 17
/// points - last one is start of next step.
static uint8_t microstepcurve[] = {0,   25,  50,  74,  98,  120, 141, 162, 180,
                                   197, 212, 225, 236, 244, 250, 253, 255};
#endif

/**************************************************************************/
/*!
    @brief  Create the Motor Shield object at an I2C address, default is 0x60
    @param  addr Optional I2C address if you've changed it
*/
/**************************************************************************/
Adafruit_MotorShield::Adafruit_MotorShield(uint8_t addr) { _addr = addr; }

/**************************************************************************/
/*!
    @brief  Initialize the I2C hardware and PWM driver, then turn off all pins.
    @param    freq
    The PWM frequency for the driver, used for speed control and microstepping.
    By default we use 1600 Hz which is a little audible but efficient.
    @param    theWire
    A pointer to an optional I2C interface. If not provided, we use Wire or
   Wire1 (on Due)
    @returns true if successful, false otherwise
*/
/**************************************************************************/
bool Adafruit_MotorShield::begin(uint16_t freq, TwoWire *theWire) {
  // init PWM w/_freq
  _pwm = Adafruit_MS_PWMServoDriver(_addr);
  if (!_pwm.begin(theWire))
    return false;
  _freq = freq;
  _pwm.setPWMFreq(_freq); // This is the maximum PWM frequency
  for (uint8_t i = 0; i < 16; i++)
    _pwm.setPWM(i, 0, 0);
  return true;
}

/**************************************************************************/
/*!
    @brief  Helper that sets the PWM output on a pin and manages 'all on or off'
    @param  pin The PWM output on the driver that we want to control (0-15)
    @param  value The 12-bit PWM value we want to set (0-4095) - 4096 is a
   special 'all on' value
*/
/**************************************************************************/
void Adafruit_MotorShield::setPWM(uint8_t pin, uint16_t value) {
  if (value > 4095) {
    _pwm.setPWM(pin, 4096, 0);
  } else
    _pwm.setPWM(pin, 0, value);
}

/**************************************************************************/
/*!
    @brief  Helper that sets the PWM output on a pin as if it were a GPIO
    @param  pin The PWM output on the driver that we want to control (0-15)
    @param  value HIGH or LOW depending on the value you want!
*/
/**************************************************************************/
void Adafruit_MotorShield::setPin(uint8_t pin, boolean value) {
  if (value == LOW)
    _pwm.setPWM(pin, 0, 0);
  else
    _pwm.setPWM(pin, 4096, 0);
}

/**************************************************************************/
/*!
    @brief  Mini factory that will return a pointer to an already-allocated
    Adafruit_DCMotor object. Initializes the DC motor and turns off all pins
    @param  num The DC motor port we want to use: 1 thru 4 are valid
    @returns NULL if something went wrong, or a pointer to a Adafruit_DCMotor
*/
/**************************************************************************/
Adafruit_DCMotor *Adafruit_MotorShield::getMotor(uint8_t num) {
  if (num > 4)
    return NULL;

  num--;

  if (dcmotors[num].motornum == 0) {
    // not init'd yet!
    dcmotors[num].motornum = num;
    dcmotors[num].MC = this;
    uint8_t pwm, in1, in2;
    switch (num) {
    case 0:
      pwm = 8;
      in2 = 9;
      in1 = 10;
      break;
    case 1:
      pwm = 13;
      in2 = 12;
      in1 = 11;
      break;
    case 2:
      pwm = 2;
      in2 = 3;
      in1 = 4;
      break;
    default:
      pwm = 7;
      in2 = 6;
      in1 = 5;
      break;
    }
    dcmotors[num].PWMpin = pwm;
    dcmotors[num].IN1pin = in1;
    dcmotors[num].IN2pin = in2;
  }
  return &dcmotors[num];
}

/******************************************
               MOTORS
******************************************/

/**************************************************************************/
/*!
    @brief  Create a DCMotor object, un-initialized!
    You should never call this, instead have the {@link Adafruit_MotorShield}
    give you a DCMotor object with {@link Adafruit_MotorShield.getMotor}
*/
/**************************************************************************/
Adafruit_DCMotor::Adafruit_DCMotor(void) {
  MC = NULL;
  motornum = 0;
  PWMpin = IN1pin = IN2pin = 0;
}

/**************************************************************************/
/*!
    @brief  Control the DC Motor direction and action
    @param  cmd The action to perform, can be FORWARD, BACKWARD or RELEASE
*/
/**************************************************************************/
void Adafruit_DCMotor::run(uint8_t cmd) {
  switch (cmd) {
  case FORWARD:
    MC->setPin(IN2pin, LOW); // take low first to avoid 'break'
    MC->setPin(IN1pin, HIGH);
    break;
  case BACKWARD:
    MC->setPin(IN1pin, LOW); // take low first to avoid 'break'
    MC->setPin(IN2pin, HIGH);
    break;
  case RELEASE:
    MC->setPin(IN1pin, LOW);
    MC->setPin(IN2pin, LOW);
    break;
  }
}

/**************************************************************************/
/*!
    @brief  Control the DC Motor speed/throttle
    @param  speed The 8-bit PWM value, 0 is off, 255 is on
*/
/**************************************************************************/
void Adafruit_DCMotor::setSpeed(uint8_t speed) {
  MC->setPWM(PWMpin, speed * 16);
}

/**************************************************************************/
/*!
    @brief  Control the DC Motor speed/throttle with 12 bit resolution.
    @param  speed The 12-bit PWM value, 0 (full off) to 4095 (full on)
*/
/**************************************************************************/
void Adafruit_DCMotor::setSpeedFine(uint16_t speed) {
  MC->setPWM(PWMpin, speed > 4095 ? 4095 : speed);
}

/**************************************************************************/
/*!
    @brief  Set DC motor to full on.
*/
/**************************************************************************/
void Adafruit_DCMotor::fullOn() { MC->_pwm.setPWM(PWMpin, 4096, 0); }

/**************************************************************************/
/*!
    @brief  Set DC motor to full off.
*/
/**************************************************************************/
void Adafruit_DCMotor::fullOff() { MC->_pwm.setPWM(PWMpin, 0, 4096); }
