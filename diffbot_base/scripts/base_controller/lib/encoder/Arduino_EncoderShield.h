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
<<<<<<< HEAD
#include "encoder_diffbot.h"

volatile long global_left_enc_pos = 0L;
volatile long global_right_enc_pos = 0L;
static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
   
/* Interrupt routine for LEFT encoder, taking care of actual counting */
/*
ISR (PCINT2_vect){
   static uint8_t enc_last=0;
         
   enc_last <<=2; //shift previous state two places
   enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits

   global_left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}
*/

/* Interrupt routine for RIGHT encoder, taking care of actual counting */
/*
ISR (PCINT1_vect){
   static uint8_t enc_last=0;
         
   enc_last <<=2; //shift previous state two places
   enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits

   global_right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}
*/
=======
>>>>>>> 46842bceba844351a310bec0ca96c62418b4e34e

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