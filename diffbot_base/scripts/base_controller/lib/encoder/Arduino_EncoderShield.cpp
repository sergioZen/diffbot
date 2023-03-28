
#include "Arduino_EncoderShield.h"
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

Arduino_EncoderShield::Arduino_EncoderShield() {};

bool Arduino_EncoderShield::begin() {
   //set as inputs
   /*
   DDRD &= ~(1<<LEFT_ENC_PIN_A);
   DDRD &= ~(1<<LEFT_ENC_PIN_B);
   DDRC &= ~(1<<RIGHT_ENC_PIN_A);
   DDRC &= ~(1<<RIGHT_ENC_PIN_B);
   
   //enable pull up resistors
   PORTD |= (1<<LEFT_ENC_PIN_A);
   PORTD |= (1<<LEFT_ENC_PIN_B);
   PORTC |= (1<<RIGHT_ENC_PIN_A);
   PORTC |= (1<<RIGHT_ENC_PIN_B);
   
   // tell pin change mask to listen to left encoder pins
   PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
   // tell pin change mask to listen to right encoder pins
   PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
   
   // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
   PCICR |= (1 << PCIE1) | (1 << PCIE2);
   */
   return true;
}

int Arduino_EncoderShield::read(int encoder) {
   if (encoder == LEFT_ENCODER) {
      return global_left_enc_pos;
   } else if (encoder == RIGHT_ENCODER) {
      return global_right_enc_pos;
   }
   return 0;
}
