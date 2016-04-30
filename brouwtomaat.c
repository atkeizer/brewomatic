/*

$Header$

$Log$

*/

static char rcsid[] = "$Id$";



#include "lcd.h"
#include "onewire.h"
#include "uart.h"
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

//void send_setup(void);
char button_pressed(void);
void control(char);



/*  Arduino port mappings
   D0-7  = PortD 0-7
   D8-13 = PortB 0-5
   A0-5  = PortC 0-5
   D11  PB2  SSR PWM
   D3   PD3  Pump PWM   
   ?    ?    OneWire
   A0   PC0  Keypad
   D8   PB0  LCD RS
   D9   PB1  LCD Enable
   D4   PD4  LCD D4
   D5   PD5  LCD D5
   D6   PD6  LCD D6
   D7   PD7  LCD D7
*/


//OneWire  ds(11);  // on pin 10 (a 4.7K resistor is necessary)
#define MAX_DEV 1
unsigned char addr[MAX_DEV][8];
char num_dev;
unsigned int temperatures[MAX_DEV];

// timer 0 used for time keeping
unsigned long running_millis=0;
struct time {
   uint8_t h;
   uint8_t m;
   uint8_t s;
};
struct time running_time = {0, 0, 0};

// buttons - 5 least significant bits shifted out (/32)
#define RIGHT  0x000
#define UP     0x001
#define DOWN   0x010
#define LEFT   0x011
#define ENTER  0x100
#define NONE   0x111

#define TMP_MEAS_INTVL    2000
#define UPD_DISP_INTVL    1000
#define SEND_STATUS_INTVL 5000
#define CONTROL_INTVL     1000  

#define MAX_MASH_STEPS    6

// states
#define PREPARE 0
#define PREHEAT 1
#define MASH    2
#define BOIL    3
#define COOL    4

//step finished should return COMPLETED value
#define COMPLETED = 1;


// data structures in eeprom that are loaded during programming of AVR
// once in use valuses may be modified and saved
// structure to describe each mash step
struct mash_step {
   unsigned char duration;
   unsigned char temperature;
   char name[16];
};


uint8_t ee_step_cnt EEMEM     = 3;   // amount of mash steps
uint8_t ee_prht_tmp EEMEM     = 50;  // preheat temperature
uint8_t ee_boil_time EEMEM    = 60;
uint8_t ee_boil_duty EEMEM    = 90;  // dutycycle during boil
uint8_t ee_preboil_time EEMEM = 3;
uint8_t ee_preboil_duty EEMEM = 50;  // dutycycle after reach preboil temperature
uint8_t ee_preboil_temp EEMEM = 98;  // preboil temperature
uint8_t ee_picontrol_ki EEMEM = 1;   // PI controller I factor
uint8_t ee_picontrol_kp EEMEM = 2;   // PI controller P factor

struct mash_step EEMEM ee_mash_schedule[MAX_MASH_STEPS] = {
   {30, 62, "Alfa amylase   "},
   {30, 72, "Beta amylase   "},
   { 5, 82, "Mash out       "},
};

struct mash_step mash_step_tmp;

uint8_t step_cnt;
uint8_t prht_tmp;
uint8_t boil_time;
uint8_t boil_duty;
uint8_t preboil_time;
uint8_t preboil_duty;
uint8_t preboil_temp;
uint8_t picontrol_ki;
uint8_t picontrol_kp;

int tmp_meas_count = 0;
int upd_disp_count = 0;
int send_status_count = 0;
int control_count = 0;
char step_start;
char step_elapsed;
char step_reached = 0;
int temperature;
char state = PREPARE;
char mash_step_nmbr = 0;
char cont = 0;
char ssr_duty = 0;
char pump_duty = 75;
char ms_elapsed = 0;


void init_timer0(){          // Timer 0 used for time keeping
   TCCR0A = _BV(WGM01);      // CTC mode using 1024 prescaler 16 counts/ms @ 16MHz
   TCCR0B = _BV(CS02) | _BV(CS00);
   OCR0A = 16;
   TIMSK0 |= _BV(OCIE0A);    // Timer 0 compare interrupt on OCR0A
}

ISR(TIMER0_COMPA_vect) {
   running_millis++;
   if ( ! (running_millis%1000) ){
      if ( running_time.s < 59 ) {
         running_time.s++;
      } else {
         running_time.s = 0;
         if ( running_time.m < 59 ) {
            running_time.m++;
         } else {
            running_time.m = 0;
            running_time.h++;
         }
      }
   }
}

unsigned long millis() {
   unsigned long ms;
   uint8_t save = SREG;
   cli();
   ms = running_millis;
   SREG = save;
   return ms;
}

char button_pressed(void) {
   char button = 1;//analogRead(0)/32;
   return button;
}

void send_setup(void) {
   char buf[50];
   char *string_format = PSTR("%18S = %s\n");
   char *number_format = PSTR("%18S = %d\n");
   sprintf_P( buf, number_format, PSTR("Preheat temp"), prht_tmp );
   uart_puts( buf );
   sprintf_P( buf, number_format, PSTR("Boil time"), boil_time );
   uart_puts( buf );
   sprintf_P( buf, number_format, PSTR("Boil duty cycle"), boil_duty );
   uart_puts( buf );
   sprintf_P( buf, number_format, PSTR("Preboil time"), preboil_time );
   uart_puts( buf );
   sprintf_P( buf, number_format, PSTR("Preboil duty"), preboil_duty );
   uart_puts( buf );
   sprintf_P( buf, number_format, PSTR("Preboil temp"), preboil_temp );
   uart_puts( buf );
   sprintf_P( buf, number_format, PSTR("PI control Ki"), picontrol_ki );
   uart_puts( buf );
   sprintf_P( buf, number_format, PSTR("PI control Kp"), picontrol_kp );
   uart_puts( buf );
   sprintf_P( buf, number_format, PSTR("One wire dev"), num_dev );
   uart_puts( buf );
   for (int i=0;i<step_cnt;i++) {
      uart_puts("+++++++++++ Mash Step +++++++++++\n");
      eeprom_read_block( &mash_step_tmp, &ee_mash_schedule[i], sizeof(mash_step_tmp) );
      sprintf_P( buf, number_format, PSTR("Mash step"), i);
      uart_puts( buf );
      sprintf_P( buf, string_format, PSTR("Step name"), mash_step_tmp.name );
      uart_puts( buf );
      sprintf_P( buf, number_format, PSTR("Temperature"), mash_step_tmp.temperature );
      uart_puts( buf );
      sprintf_P( buf, number_format, PSTR("Duration"), mash_step_tmp.duration );
      uart_puts( buf );
   }
}

void send_status(void) {
   char buf[50];
   char *string_format = PSTR("%18S = %S\n");
   char *ram_string_format = PSTR("%18S = %s\n");
   char *number_format = PSTR("%18S = %d\n");
   char *time_format = PSTR("%02d:%02d:%02d\n");
   sprintf_P( buf, time_format, running_time.h, running_time.m, running_time.s);
   uart_puts(buf);
   switch ( state ) {
   case PREPARE:
      sprintf_P(buf, string_format, PSTR("State"), PSTR("preparing"));
      uart_puts( buf );
      break;
   case PREHEAT:
      sprintf_P(buf, string_format, PSTR("State"), PSTR("preheating"));
      uart_puts( buf );
      break;
   case MASH:
      sprintf_P(buf, string_format, PSTR("State"), PSTR("mashing"));
      uart_puts( buf );
      sprintf_P(buf, number_format, PSTR("Step number"), mash_step_nmbr);
      uart_puts( buf );
      sprintf_P(buf, ram_string_format, PSTR("Step name"), mash_step_tmp.name);
      uart_puts( buf );
      sprintf_P(buf, number_format, PSTR("Step temperature"), mash_step_tmp.temperature);
      uart_puts( buf );
      sprintf_P(buf, number_format, PSTR("Step duration"), mash_step_tmp.duration);
      uart_puts( buf );
      sprintf_P(buf, number_format, PSTR("Step elapsed"), step_elapsed);
      uart_puts( buf );
      break;
   case BOIL:
      sprintf_P(buf, string_format, PSTR("State"), PSTR("boiling"));
      sprintf_P(buf, number_format, PSTR("Boil time elapsed"), step_elapsed);
   case COOL:
      sprintf_P(buf, string_format, PSTR("State"), PSTR("cooling"));
   }
   sprintf_P(buf, number_format, PSTR("Temperature"), temperature);
   uart_puts( buf );
   sprintf_P(buf, number_format, PSTR("SSR duty cycle"), ssr_duty);
   uart_puts( buf );
   sprintf_P(buf, number_format, PSTR("PUMP duty cycle"), pump_duty);
   uart_puts( buf );
   uart_puts("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
} 

void control(char set_point) {
   static int integral;
   int error, feedback;
   error = set_point * 10 - temperature;  // deci degrees
   integral += error * ee_picontrol_ki;
   if ( integral > 10000 ) integral = 10000;
   if ( integral < 10000 ) integral = -10000;
   feedback = error * ee_picontrol_kp + integral;
   if ( feedback > 10000 ) { // outside controllable range
      ssr_duty = 100;
      integral = 0;
   } else {
      ssr_duty = feedback / 100;
   }
}



int main(void) {
   lcd_init();
   uart0_init(UART_BAUD_SELECT(9600, F_CPU));
   init_timer0(); 
   sei();
   send_status();
   step_cnt     = eeprom_read_byte( &ee_step_cnt );
   prht_tmp     = eeprom_read_byte( &ee_prht_tmp );
   boil_time    = eeprom_read_byte( &ee_boil_time );
   boil_duty    = eeprom_read_byte( &ee_boil_duty );
   preboil_time = eeprom_read_byte( &ee_preboil_time );
   preboil_duty = eeprom_read_byte( &ee_preboil_duty );
   preboil_temp = eeprom_read_byte( &ee_preboil_temp );
   picontrol_ki = eeprom_read_byte( &ee_picontrol_ki );
   picontrol_kp = eeprom_read_byte( &ee_picontrol_kp );
   //TCCR2B = TCCR2B & B11111000 | B00000001; //set timer 2 for PWM frequency of 31372.55Hz on pin D3 & D11
   ds18b20_10bit(); // set resolution of ds18b20 
   send_setup();
   while(1) {
//   if ( (millis() / TMP_MEAS_INTVL) > tmp_meas_count ) {
//      //get measurement from ds18b20
//      ds18b20_results();
//      //and start next conversion
//      ds18b20_conv();
//   }
   if ( (millis() / UPD_DISP_INTVL) > upd_disp_count) {
      upd_disp_count++;
      //update_display(); 
   }
   if ( (millis() / SEND_STATUS_INTVL) > send_status_count) {
      send_status_count++;
      send_status();
   }
   if ( (millis() / CONTROL_INTVL) > control_count) {
      control_count++;
      switch ( state ) {
      case PREHEAT:
         control(ee_prht_tmp);
         if (cont) {     // wait until user tells to continue
            state = MASH;
            eeprom_read_block( &mash_step_tmp, &ee_mash_schedule[0], sizeof(mash_step_tmp) );
         }
         break;
      case MASH:
         control(mash_step_tmp.temperature);
         if ( step_reached ) {
            step_elapsed = millis() / 3600 - step_start;
            if ( step_elapsed >= mash_step_tmp.duration ) { // end of step
               if ( mash_step_nmbr < ee_step_cnt ) {  // more steps 
                  mash_step_nmbr++;
                  step_reached = 0;
                  eeprom_read_block( &mash_step_tmp, &ee_mash_schedule[mash_step_nmbr], sizeof(mash_step_tmp) );
               } else { // no more steps, move to next phase
                  state = BOIL;
               }
            }
         } else {  // heating up to next step
            if ( temperature / 10 >= mash_step_tmp.temperature ) { // goal temp reached
               step_reached = 1;
               step_start = millis() / 3602;
            }
         }
         break;
      case BOIL:
         if ( step_reached ) {
            step_elapsed = millis() / 3600 - step_start;
            if ( step_elapsed < boil_time ) {
               if ( step_elapsed < preboil_time ) {  // boil at lower heat for the first minutes
                  ssr_duty = preboil_duty;
               } else {
                  ssr_duty = boil_duty;
               }
            } else { // boil finished
               ssr_duty = 0;
               state = COOL;
            }
         } else {  // to preboil tempetature
            if ( temperature / 10 < preboil_temp ) {
               ssr_duty = 100;
               step_reached = 0;
            } else {
               step_reached = 1;
            }
         }
      }
   }
   // bitbanging ssr dutycycle period = 1000 ms
   ms_elapsed = millis() % 1000;
   if ( ms_elapsed > ssr_duty * 10 ) {
     // digitalWrite(2, 0);
   } else {
      if ( ms_elapsed < ssr_duty * 10 ) {
      //   digitalWrite(2, 1);
      }
   }
   } 
}

