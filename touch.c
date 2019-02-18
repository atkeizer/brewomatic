/*

$Header: /home/atkeizer/brouwtomaat/RCS/touch.c,v 1.1 2016/10/07 22:00:21 atkeizer Exp $

$Log: touch.c,v $
Revision 1.1  2016/10/07 22:00:21  atkeizer
Initial revision


*/

#include "touch.h"

uint8_t touch_scan(uint8_t mask){
   register uint8_t count = 0;
   register uint8_t sreg=SREG;
   cli();
   DDRC &= ~mask;   // make sense pins inputs 
   //PORTC |= mask; // enable internal pull up
   // external pullup with a bit higher value ~82k works better
   while ( !(PINC & mask) ) count++;
   SREG=sreg;
   DDRC |= mask;   // set to output low to discharge
   PORTC &= ~mask; 
   return count;
}


uint8_t touch_scan_all(){  // return mask of buttons touched
   uint8_t mask=0;
   if (touch_scan(TOUCH_ENTER) > 5) mask+=TOUCH_ENTER;
   if (touch_scan(TOUCH_RIGHT) > 5) mask+=TOUCH_RIGHT;
   if (touch_scan(TOUCH_UP)    > 5) mask+=TOUCH_UP;
   if (touch_scan(TOUCH_DOWN)  > 5) mask+=TOUCH_DOWN; 
   if (touch_scan(TOUCH_LEFT)  > 5) mask+=TOUCH_LEFT;
   return mask;
}
   
uint8_t touch_scan_released(){ // return 1 if all below lower threshold
   uint8_t rel;
      rel = (touch_scan(TOUCH_ENTER) < 5) &&
            (touch_scan(TOUCH_RIGHT) < 5) &&
            (touch_scan(TOUCH_UP) < 5) &&
            (touch_scan(TOUCH_DOWN) < 5) &&
            (touch_scan(TOUCH_LEFT) < 5);
   return rel;
}






