
#include "L298N_MotorShield.h"
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
L298N_MotorShield::L298N_MotorShield(){};

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
bool L298N_MotorShield::begin() {
  //digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
  //digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  return true;
}

/**************************************************************************/
/*!
    @brief  Mini factory that will return a pointer to an already-allocated
    Adafruit_DCMotor object. Initializes the DC motor and turns off all pins
    @param  num The DC motor port we want to use: 1 thru 4 are valid
    @returns NULL if something went wrong, or a pointer to a Adafruit_DCMotor
*/
/**************************************************************************/
L298N_DCMotor *L298N_MotorShield::getMotor(uint8_t num) {
  if (num > 2)
    return NULL;

  num--;

  if (dcmotors[num].motornum == 0) {
    // not init'd yet!
    dcmotors[num].motornum = num;
    dcmotors[num].MC = this;
    uint8_t in1, in2;
    switch (num) {
    case 0:
      in2 = LEFT_MOTOR_FORWARD;
      in1 = LEFT_MOTOR_BACKWARD;
      break;
    case 1:
      in2 = RIGHT_MOTOR_FORWARD;
      in1 = RIGHT_MOTOR_BACKWARD;
      break;
    default:
      break;
    }
    dcmotors[num].IN1pinForward = in1;
    dcmotors[num].IN2pinBackwards = in2;
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
L298N_DCMotor::L298N_DCMotor(void) {
  MC = NULL;
  motornum = 0;
  IN1pinForward = IN2pinBackwards = 0;
}

/**************************************************************************/
/*!
    @brief  Control the DC Motor speed/throttle
    @param  speed The 8-bit PWM value, 0 is off, 255 is on
*/
/**************************************************************************/
void L298N_DCMotor::setSpeed(uint8_t speed) {
   unsigned char reverse = 0;

   if (speed < 0)
   {
      speed = -speed;
      reverse = 1;
   }
   if (speed > 255)
      speed = 255;

   if (reverse == 0)
   {
      analogWrite(IN1pinForward, speed); analogWrite(IN2pinBackwards, 0);
   }
   else if (reverse == 1)
   {
      analogWrite(IN2pinBackwards, speed); analogWrite(IN1pinForward, 0); 
   }
}

void L298N_DCMotor::setPinForward(uint8_t pin) {
      IN1pinForward = pin;
}

void L298N_DCMotor::setPinBackwards(uint8_t pin) {
      IN2pinBackwards = pin;
}

/**************************************************************************/
/*!
    @brief  Set DC motor to full on.
*/
/**************************************************************************/
void L298N_DCMotor::fullOn() { setSpeed(255); }

/**************************************************************************/
/*!
    @brief  Set DC motor to full off.
*/
/**************************************************************************/
void L298N_DCMotor::fullOff() { setSpeed(0); }
