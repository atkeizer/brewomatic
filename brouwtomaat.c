/*

$Header: /home/atkeizer/avr/brouwtomaat/RCS/brouwtomaat.c,v 1.3 2016/05/03 05:15:41 atkeizer Exp atkeizer $

$Log: brouwtomaat.c,v $
Revision 1.3  2016/05/03 05:15:41  atkeizer
buttons, lcd and uart functioning

Revision 1.2  2016/04/30 09:36:57  atkeizer
stdio stream output on stdout

Revision 1.1  2016/04/30 08:40:29  atkeizer
Initial revision


 Arduino port mappings
   D0-7  = PortD 0-7
   D8-13 = PortB 0-5
   A0-5  = PortC 0-5
   D11  PB3  Pump PWM   OC2A
   D12  PB4  SSR PWM    bitbanging
   D13  PB5  OneWire
   A0   PC0  Keypad
   D8   PB0  LCD RS
   D9   PB1  LCD Enable
   D4   PD4  LCD D4
   D5   PD5  LCD D5
   D6   PD6  LCD D6
   D7   PD7  LCD D7
*/


//OneWire  ds(11);  // on pin 10 (a 4.7K resistor is necessary)

#include "lcd.h"
#include "onewire.h"
#include "uart.h"
#include "button.h"

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>


void uart_putchar(char c, FILE *stream) {
   if (c == '\n') {
       uart_putchar('\r', stream);
   }
   uart_putc(c);
}

static FILE uart = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void lcd_putchar(char c, FILE *stream) {
   lcd_write_byte(c);
}

static FILE lcd = FDEV_SETUP_STREAM(lcd_putchar, NULL, _FDEV_SETUP_WRITE);


unsigned int temperatures[OW_MAX_DEV];

// timer 0 used for time keeping
unsigned long running_millis=0;
struct time {
   uint8_t h;
   uint8_t m;
   uint8_t s;
};
struct time running_time = {0, 0, 0};

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
uint8_t mash_step_nmbr = 0;
char cont = 0;
char ssr_duty = 0;
char pump_duty = 75;
char ms_elapsed = 0;


void init_timer0(){          // Timer 0 used for time keeping and SSR bitbanging
   TCCR0A = _BV(WGM01);      // CTC mode using 1024 prescaler 16 counts/ms @ 16MHz
   TCCR0B = _BV(CS02) | _BV(CS00);
   OCR0A = 15;
   TIMSK0 |= _BV(OCIE0A);    // Timer 0 compare interrupt on OCR0A
}

ISR(TIMER0_COMPA_vect) {
   uint8_t hundreds;
   static uint8_t hundreds_prev;
   running_millis++;
   hundreds = (running_millis / 10) % 100;
   if (hundreds != hundreds_prev ) {
      if ( ! hundreds ){
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
         PORTB |= _BV(PB4); // turn on SSR in beginning of period
      }
      if ( hundreds >= ssr_duty ) { // turn off at end of duty cycle
         PORTB &= ~_BV(PB4);
      }
      hundreds_prev = hundreds;
   }
}

void init_timer2(){     // use timer 2 for pump PWM
   OCR2A = 0;
   DDRB |= _BV(PB3);
   TCCR2A |= _BV(COM2A1) | _BV(WGM21) | _BV(WGM20) ; // clear OC2A on compare match, fast PWM
   TCCR2B = _BV(CS21);  // prescaler 8 gives  7.8KHz
}

void pump_pwm( uint8_t duty_cycle){  // 0-255
   OCR2A =  duty_cycle;
}


unsigned long millis() {
   unsigned long ms;
   uint8_t save = SREG;
   cli();
   ms = running_millis;
   SREG = save;
   return ms;
}

void send_setup(void) {
   char *string_format = PSTR("%18S = %s\n");
   char *number_format = PSTR("%18S = %d\n");
   fprintf_P(stderr, number_format, PSTR("Preheat temp"), prht_tmp );
   fprintf_P(stderr, number_format, PSTR("Boil time"), boil_time );
   fprintf_P(stderr, number_format, PSTR("Preboil time"), preboil_time );
   fprintf_P(stderr, number_format, PSTR("Preboil duty"), preboil_duty );
   fprintf_P(stderr, number_format, PSTR("Preboil temp"), preboil_temp );
   fprintf_P(stderr, number_format, PSTR("PI control Ki"), picontrol_ki );
   fprintf_P(stderr, number_format, PSTR("PI control Kp"), picontrol_kp );
   for (int i=0;i<step_cnt;i++) {
      uart_puts("+++++++++++ Mash Step +++++++++++\n");
      eeprom_read_block( &mash_step_tmp, &ee_mash_schedule[i], sizeof(mash_step_tmp) );
      fprintf_P(stderr, number_format, PSTR("Mash step"), i);
      fprintf_P(stderr, string_format, PSTR("Step name"), mash_step_tmp.name );
      fprintf_P(stderr, number_format, PSTR("Temperature"), mash_step_tmp.temperature );
      fprintf_P(stderr, number_format, PSTR("Duration"), mash_step_tmp.duration );
   }
}

void send_status(void) {
   char *string_format = PSTR("%18S = %S\n");
   char *ram_string_format = PSTR("%18S = %s\n");
   char *number_format = PSTR("%18S = %d\n");
   char *time_format = PSTR("%02d:%02d:%02d\n");
   fprintf_P(stderr, time_format, running_time.h, running_time.m, running_time.s);
   switch ( state ) {
   case PREPARE:
      fprintf_P(stderr, string_format, PSTR("State"), PSTR("preparing"));
      break;
   case PREHEAT:
      fprintf_P(stderr, string_format, PSTR("State"), PSTR("preheating"));
      break;
   case MASH:
      fprintf_P(stderr, string_format, PSTR("State"), PSTR("mashing"));
      fprintf_P(stderr, number_format, PSTR("Step number"), mash_step_nmbr);
      fprintf_P(stderr, ram_string_format, PSTR("Step name"), mash_step_tmp.name);
      fprintf_P(stderr, number_format, PSTR("Step temperature"), mash_step_tmp.temperature);
      fprintf_P(stderr, number_format, PSTR("Step duration"), mash_step_tmp.duration);
      fprintf_P(stderr, number_format, PSTR("Step elapsed"), step_elapsed);
      break;
   case BOIL:
      fprintf_P(stderr, string_format, PSTR("State"), PSTR("boiling"));
      fprintf_P(stderr, number_format, PSTR("Boil time elapsed"), step_elapsed);
   case COOL:
      fprintf_P(stderr, string_format, PSTR("State"), PSTR("cooling"));
   }
   fprintf_P(stderr, number_format, PSTR("Temperature"), temperature);
   fprintf_P(stderr, number_format, PSTR("SSR duty cycle"), ssr_duty);
   fprintf_P(stderr, number_format, PSTR("PUMP duty cycle"), pump_duty);
   fprintf_P(stderr,PSTR("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n"));
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
   uint8_t button = 0;
   stderr = &uart;
   stdout = &lcd;
   lcd_init();
   lcd_clear();
   uart0_init(UART_BAUD_SELECT(9600, F_CPU));
   init_adc();
   init_timer0(); 
   init_timer2(); 
   pump_pwm(0);
   DDRB |= _BV(PB4); // used for bitbanging SSR
   if ( ! ow_rom_search() ) {
      fprintf_P(&lcd,PSTR(" No temperature "));
      lcd_gotoxy(0,1);
      fprintf_P(&lcd,PSTR("sensor connected"));
   //   while(1);
   }
   sei();
   step_cnt     = eeprom_read_byte( &ee_step_cnt );
   prht_tmp     = eeprom_read_byte( &ee_prht_tmp );
   boil_time    = eeprom_read_byte( &ee_boil_time );
   boil_duty    = eeprom_read_byte( &ee_boil_duty );
   preboil_time = eeprom_read_byte( &ee_preboil_time );
   preboil_duty = eeprom_read_byte( &ee_preboil_duty );
   preboil_temp = eeprom_read_byte( &ee_preboil_temp );
   picontrol_ki = eeprom_read_byte( &ee_picontrol_ki );
   picontrol_kp = eeprom_read_byte( &ee_picontrol_kp );
//   ds18b20_10bit(); // set resolution of ds18b20 
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
      button = read_button();
      if (button != BTN_NONE ) {            //  don't waste any more time if no button pressed
         _delay_ms(2);
         if ( button == read_button() ) {   // debounce, readings should be the same
            lcd_gotoxy(0,0);
            switch ( button ) {
               case BTN_ENTER:
                  //lcd_puts("enter pressed   ");
                  printf("ENTER           ");
                  break;
               case BTN_RIGHT:
                  printf("RIGHT           ");
                  break;
               case BTN_LEFT:
                  printf("LEFT            ");
                  break;
               case BTN_UP:
                  printf("UP              ");
                  break;
               case BTN_DOWN:
                  printf("DOWN            ");
                  break;
            }
         }
      }
   }
}

