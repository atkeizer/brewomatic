/*

$Header$

$Log$

*/

#include <avr/io.h>
#include "button.h"


void init_adc() {
 ADMUX |= _BV(REFS0);  // Vref AVcc
 ADCSRA |= _BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0)|_BV(ADEN);  //set prescaller to 128 and enable ADC 
}



uint8_t read_button(void) {   // button resistor ladder connected to PC0
   ADMUX &= 0xF0;          // channel 0
   ADCSRA |= _BV(ADSC);    // start single conversion
   while( ADCSRA & _BV(ADSC) );
   return ADC / 128;
}


