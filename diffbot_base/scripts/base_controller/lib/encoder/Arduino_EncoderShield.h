#include <inttypes.h>
#include "encoder_diffbot.h"

volatile long global_left_enc_pos = 0L;
volatile long global_right_enc_pos = 0L;
static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
   
/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR (PCINT2_vect){
   static uint8_t enc_last=0;
         
   enc_last <<=2; //shift previous state two places
   enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits

   global_left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

/* Interrupt routine for RIGHT encoder, taking care of actual counting */
ISR (PCINT1_vect){
   static uint8_t enc_last=0;
         
   enc_last <<=2; //shift previous state two places
   enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits

   global_right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

class Arduino_EncoderShield;

/** Object that controls and keeps state for the whole motor shield.
    Use it to create DC and Stepper motor objects! */
class Arduino_EncoderShield {
public:
  Arduino_EncoderShield();

  bool begin();

  friend class Encoder; ///< Let DCMotors control the Shield
};