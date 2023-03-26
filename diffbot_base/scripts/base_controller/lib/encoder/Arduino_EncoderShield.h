#ifndef _Arduino_EncoderShield_h_
#define _Arduino_EncoderShield_h_

#define LEFT_ENCODER 1
#define RIGHT_ENCODER 2

//below can be changed, but should be PORTD pins; 
//otherwise additional changes in the code are required
#define LEFT_ENC_PIN_A PD2  //pin 2
#define LEFT_ENC_PIN_B PD3  //pin 3

//below can be changed, but should be PORTC pins
#define RIGHT_ENC_PIN_A PC4  //pin A4
#define RIGHT_ENC_PIN_B PC5   //pin A5

#include <inttypes.h>

class Arduino_EncoderShield;

/** Object that controls and keeps state for the whole motor shield.
    Use it to create DC and Stepper motor objects! */
class Arduino_EncoderShield {
public:
  Arduino_EncoderShield();

  bool begin();

  int read(int encoder);

};

#endif  // _Arduino_EncoderShield_h_