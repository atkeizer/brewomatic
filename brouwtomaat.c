/*

$Header: /home/atkeizer/brouwtomaat/RCS/brouwtomaat.c,v 1.8 2022/05/24 09:15:36 atkeizer Exp $

$Log: brouwtomaat.c,v $
Revision 1.8  2022/05/24 09:15:36  atkeizer
Some counter with larger datatype and faster baud rate

Revision 1.7  2017/10/08 08:50:54  atkeizer
simple manual control only

Revision 1.5  2016/05/11 16:07:37  atkeizer
PID using floats

Revision 1.4  2016/05/06 08:37:13  atkeizer
pump and SSR pwm implemented

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
   D10  PB2  OneWire
   D11  PB3  Pump PWM   OC2A
   D12  PB4  SSR PWM    bitbanging
   A0   PC0  Touchpad enter
   A1   PC1  Touchpad right
   A2   PC2  Touchpad up
   A3   PC3  Touchpad down
   A4   PC4  Touchpad left
   D8   PB0  LCD RS
   D9   PB1  LCD Enable
   D4   PD4  LCD D4
   D5   PD5  LCD D5
   D6   PD6  LCD D6
   D7   PD7  LCD D7
*/


//OneWire  ds(13);  // on pin 13 (a 4.7K resistor is necessary)

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "lcd.h"
#include "onewire.h"
#include "uart.h"
#include "menu.h"
#include "touch.h"


void uart_putchar(char c, FILE *stream) {
   if (c == '\n') 
      uart_putchar('\r', stream);
   uart_putc(c);
}

static FILE uart = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

unsigned long running_hundreds=0;

struct time {
   uint8_t h;
   uint8_t m;
   uint8_t s;
};
struct time running_time = {0, 0, 0};

#define TMP_MEAS_INTVL    200
#define UPD_DISP_INTVL    100
#define SEND_STATUS_INTVL 500

#define MAX_MASH_STEPS    6

// states
#define PREPARE 0
#define PREHEAT 1
#define MASH    2
#define BOIL    3
#define COOL    4

//step finished should return COMPLETED value
#define COMPLETED = 1;


// global variables for PID controller so they can be accessed outside ISR for tuning purposes
double pid_kp, pid_ki, pid_kd, pid_setpoint, pid_integral, pid_prev_error, pid_error, pid_feedback, pid_prop, pid_derivative;


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
double ee_pid_kp EEMEM = 75;   // PID controller P factor
double ee_pid_ki EEMEM = 0.5;   // PID controller I factor
double ee_pid_kd EEMEM = 100;   // PID controller D factor

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

unsigned long tmp_meas_count = 0;
unsigned long upd_disp_count = 0;
unsigned long send_status_count = 0;
unsigned long control_count = 0;
char step_start;
char step_elapsed;
char step_reached = 0;
double temperature = 46;
//char state = PREPARE;
char state = PREHEAT;
uint8_t mash_step_nmbr = 0;
char cont = 0;
char ssr_duty = 0;
char pump_duty = 0;


void init_timer1(){           // Timer 1 used for time keeping and SSR bitbanging (10mS interval)
   TCCR1B = _BV(WGM12) | _BV(CS12); // CTC mode with OCR1A as compare using 256 prescaler
   TCNT1 = 0;
   OCR1A = 624;               // 625 counts per hundred @ 16MHz
   TIMSK1 |= _BV(OCIE1A);     // Timer 0 compare interrupt on OCR1A
}

ISR(TIMER1_COMPA_vect) {  // called every 10 mS
   running_hundreds++;
   uint8_t sec_fract = running_hundreds  % 100; 
   if ( ! sec_fract ){                                // every second
      //pid_error = pid_setpoint - temperature;         // PID control temperature
      //pid_prop = pid_error * pid_kp;  
      //if (pid_prop > 100) pid_integral = 0;           // only integrate in controllable range
      //else pid_integral += pid_error * pid_ki;
      //pid_derivative = (pid_error - pid_prev_error) * pid_kd;
      //if ( pid_integral > 100 ) pid_integral = 100;
      //if ( pid_integral < 0 ) pid_integral = 0;
      //pid_feedback = pid_prop + pid_integral + pid_derivative;
      //if ( pid_feedback > 100 ) pid_feedback = 100;
      //if ( pid_feedback < 0 ) pid_feedback = 0;
      //ssr_duty = (uint8_t) pid_feedback;

      if ( running_time.s < 59 ) {  // update HMS clock
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
   if ( sec_fract == ssr_duty ) { // turn off at end of duty cycle
      PORTB &= ~_BV(PB4);
   }
}

void vars_from_eeprom(){  // get values from eeprom
   eeprom_read_block( &pid_kp, &ee_pid_kp, sizeof(pid_kp) );
   eeprom_read_block( &pid_ki, &ee_pid_ki, sizeof(pid_ki) );
   eeprom_read_block( &pid_kd, &ee_pid_kd, sizeof(pid_kd) );
   step_cnt     = eeprom_read_byte( &ee_step_cnt );
   prht_tmp     = eeprom_read_byte( &ee_prht_tmp );
   boil_time    = eeprom_read_byte( &ee_boil_time );
   boil_duty    = eeprom_read_byte( &ee_boil_duty );
   preboil_time = eeprom_read_byte( &ee_preboil_time );
   preboil_duty = eeprom_read_byte( &ee_preboil_duty );
   preboil_temp = eeprom_read_byte( &ee_preboil_temp );
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

void heater_duty( uint8_t duty_cycle){  // 0-100
   ssr_duty = duty_cycle;
}



unsigned long hundreds() {
   unsigned long hds;
   uint8_t save = SREG;
   cli();
   hds = running_hundreds;
   SREG = save;
   return hds;
}

void send_setup(void) {
   char *string_format = PSTR("%18S = %s\n");
   char *number_format = PSTR("%18S = %d\n");
   char *float_format = PSTR("%18S = %.2f\n");
   printf_P(number_format, PSTR("Preheat temp"), prht_tmp );
   printf_P(number_format, PSTR("Boil time"), boil_time );
   printf_P(number_format, PSTR("Preboil time"), preboil_time );
   printf_P(number_format, PSTR("Preboil duty"), preboil_duty );
   printf_P(number_format, PSTR("Preboil temp"), preboil_temp );
   printf_P(float_format, PSTR("PID control Kp"), pid_kp );
   printf_P(float_format, PSTR("PID control Ki"), pid_ki );
   printf_P(float_format, PSTR("PID control Kd"), pid_kd );
   for (int i=0;i<step_cnt;i++) {
      eeprom_read_block( &mash_step_tmp, &ee_mash_schedule[i], sizeof(mash_step_tmp) );
      printf_P(PSTR("+++++++++++ Mash Step +++++++++++\n"));
      printf_P(number_format, PSTR("Mash step"), i);
      printf_P(string_format, PSTR("Step name"), mash_step_tmp.name );
      printf_P(number_format, PSTR("Temperature"), mash_step_tmp.temperature );
      printf_P(number_format, PSTR("Duration"), mash_step_tmp.duration );
   }
}

void send_status(void) {
   char *string_format = PSTR("%18S = %S\n");
   char *ram_string_format = PSTR("%18S = %s\n");
   char *number_format = PSTR("%18S = %d\n");
   char *time_format = PSTR("%02d:%02d:%02d\n");
   char *float_format = PSTR("%18S = %.2f\n");
   printf_P(time_format, running_time.h, running_time.m, running_time.s);
   switch ( state ) {
   case PREPARE:
      printf_P(string_format, PSTR("State"), PSTR("preparing"));
      break;
   case PREHEAT:
      printf_P(string_format, PSTR("State"), PSTR("preheating"));
      break;
   case MASH:
      printf_P(string_format, PSTR("State"), PSTR("mashing"));
      printf_P(number_format, PSTR("Step number"), mash_step_nmbr);
      printf_P(ram_string_format, PSTR("Step name"), mash_step_tmp.name);
      printf_P(number_format, PSTR("Step temperature"), mash_step_tmp.temperature);
      printf_P(number_format, PSTR("Step duration"), mash_step_tmp.duration);
      printf_P(number_format, PSTR("Step elapsed"), step_elapsed);
      break;
   case BOIL:
      printf_P(string_format, PSTR("State"), PSTR("boiling"));
      printf_P(number_format, PSTR("Boil time elapsed"), step_elapsed);
   case COOL:
      printf_P(string_format, PSTR("State"), PSTR("cooling"));
   }
   printf_P(float_format, PSTR("Temperature"), temperature);
   printf_P(number_format, PSTR("SSR duty cycle"), ssr_duty);
   printf_P(number_format, PSTR("PUMP duty cycle"), pump_duty);
   fprintf_P(stderr,PSTR("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n"));
   // simulate kettle 
   //temperature = temperature + pid_feedback / 2000 - 0.01;
   //pid_prev_error = pid_error;
   //fprintf_P(&uart, PSTR("T=%.3f, S=%.1f, E=%.3f, P=%.1f, I=%.1f, D=%.1f, F=%.1f\n"), temperature, pid_setpoint, pid_error, pid_prop, pid_integral, pid_derivative, pid_feedback );
} 


void update_display() {
   char buf[21];
   lcd_gotoxy(0,2);
   sprintf_P(buf, PSTR("SSR = %d  Pump = %d"), ssr_duty, pump_duty);
   lcd_puts(buf);
   lcd_gotoxy(0,3);
   sprintf_P(buf, PSTR("%2d:%02d:%02d     %3.1f \337C"), 
             running_time.h, running_time.m, running_time.s, temperature);
   lcd_puts(buf);
}
   



void set_pid(double setpoint) {
   uint8_t sreg;
   sreg=SREG;
   cli();
   pid_setpoint = setpoint;
   SREG=sreg;
}

int main(void) {
   uint8_t button, btn_prev, btn_rpt=0, err;
   unsigned long btn_time = 0;
   char line_buf[20];
   stdout = &uart;
   _delay_ms(100);
   vars_from_eeprom();    // initialize variables / read from eeprom
   lcd_init();
   _delay_ms(100);
   lcd_clear();
   _delay_ms(100);
   uart0_init(UART_BAUD_SELECT(115200, F_CPU));
   //init_adc();
   //touch_init();
   init_timer1(); 
   init_timer2(); 
   pump_pwm(0);    // pump initially off
   sei();
   DDRB |= _BV(PB4); // used for bitbanging SSR
   if ( ! ow_rom_search() ) {
      printf_P(PSTR("No temperature sensor found\n"));
      lcd_puts_p(PSTR(" No temperature "));
      lcd_gotoxy(0,1);
      lcd_puts_p(PSTR("sensor connected"));
   } else {
     lcd_gotoxy(0,0);
     lcd_puts_p(PSTR("Temperature sensor found"));
   }
   ds18b20_10bit(); // set resolution of ds18b20 
   //send_setup();

// test SPI slave for wifi module
// to be moved to its own source file

// polled slave initialization
// MISO as output
// set SPE bit in SPCR register
// clear SPI interrupt by reading SPSR and SPDR




   while(1) {
      if ( (hundreds() / TMP_MEAS_INTVL) > tmp_meas_count ) {
         tmp_meas_count++;
         //get measurement from ds18x20
         lcd_gotoxy(0,3);
         err = read_ds18x20(romcodes[0], &temperature);
         if (err == 1 ) lcd_puts_p(PSTR("CRC error "));
         else if (err == 2 ) lcd_puts_p(PSTR("not a DS18x20 "));
         //and start next conversion
         start_conv_ds18x20(0); // id=0 means all
      }
      if ( (hundreds() / UPD_DISP_INTVL) > upd_disp_count) {
         upd_disp_count++;
         update_display(); 
      }
      if ( (hundreds() / SEND_STATUS_INTVL) > send_status_count) {
         send_status_count++;
         send_status();
      }
      // below is executed every cycle
      switch ( state ) {
         case PREHEAT:
         set_pid(prht_tmp);
         if (cont) {     // wait until user tells to continue
            state = MASH;
            eeprom_read_block( &mash_step_tmp, &ee_mash_schedule[0], sizeof(mash_step_tmp) );
         }
         break;
         case MASH:
         set_pid(mash_step_tmp.temperature);
         if ( step_reached ) {
            step_elapsed = hundreds() / 360 - step_start;
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
            if ( temperature >= mash_step_tmp.temperature ) { // goal temp reached
               step_reached = 1;
               step_start = hundreds() / 360;
            }
         }
         break;
         case BOIL:
         if ( step_reached ) {
            step_elapsed = hundreds() / 360 - step_start;
            if ( step_elapsed < boil_time ) {
               if ( step_elapsed < preboil_time ) {  // boil at lower heat for the first minutes
                  //ssr_duty = preboil_duty;
               } else {
                  //ssr_duty = boil_duty;
               }
            } else { // boil finished
               //ssr_duty = 0;
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
      // hysteresis on buttons
      if (touch_state == 0) {
         button = touch_scan_all();
         if (button) {
            touch_state = 1;
            process_menu(button);
         }
      }


      if ( touch_state == 1 ) {
         if (touch_scan_released()){
            touch_state = 0;
         }
      }
   }
}

