/*

$Header: /home/atkeizer/brouwtomaat/RCS/button.h,v 1.1 2016/05/03 05:18:01 atkeizer Exp $

$Log: button.h,v $
Revision 1.1  2016/05/03 05:18:01  atkeizer
Initial revision


*/


// buttons - 5 least significant bits shifted out (/32)
#define BTN_RIGHT  0b000  
#define BTN_UP     0b001
#define BTN_DOWN   0b010
#define BTN_LEFT   0b011
#define BTN_ENTER  0b101
#define BTN_NONE   0b111


void init_adc(void);
uint8_t read_button(void); 


