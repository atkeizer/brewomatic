/*

$Header: /home/atkeizer/avr/brouwtomaat/lcd.c,v 1.2 2016/04/30 08:52:25 atkeizer Exp atkeizer $

$Log: lcd.c,v $
Revision 1.2  2016/04/30 08:52:25  atkeizer
dos2unix conversion of source

Revision 1.1  2016/04/30 08:44:16  atkeizer
Initial revision


*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "lcd.h"


// Implements 4 bit communication to LCD display using pin assignments of arduino shield

#define RS PB0
#define EN PB1

// data on PORTD 4..7


void lcd_init(void){
   DDRB |= (1<<RS) | (1<<EN); //are outputs
   PORTB &= ~(1 << EN);  // initial state of enable is low
   DDRD |= 0xF0; // PD4..PD7 outputs
   lcd_command_mode();
   lcd_write_nibble(0x3);
   _delay_ms(5);
   lcd_write_nibble(0x3);
   _delay_ms(5);
   lcd_write_nibble(0x3);
   _delay_us(100);
   lcd_write_nibble(0x2); // 4 bit mode
   _delay_us(40);
   lcd_write_byte(0x28);  //  4 bit, 2 lines, 5x8 font 
   _delay_us(40);
   lcd_write_byte(0xC);  // display on, cursor and blink off
   _delay_us(40);
   lcd_write_byte(0x10);  // no display shift
   _delay_us(40);
   lcd_data_mode();
}

void lcd_char_gen(char * pixelrow){   
   uint8_t i;
   lcd_command_mode();
   lcd_write_byte(0x40);
   lcd_data_mode();
   for (i=0; i<8 ; i++){
      lcd_write_byte(pixelrow[i]);
   }
}

void lcd_puts(char *data){
   while ( *data ) {
      lcd_write_byte( *data++ );
   }
}

void lcd_puts_p(const char *prog_data){
   char c;
   while ( (c = pgm_read_byte(prog_data++)) ) {
      lcd_write_byte(c);
   }
}

void lcd_gotoxy(char x, char y){
   lcd_command_mode();
   lcd_write_byte( 0x80 + y * 0x40 + x);
   lcd_data_mode();
}

void lcd_clear(void){
   lcd_command_mode();
   lcd_write_byte(0x01);
   lcd_data_mode();
   _delay_ms(2);
}

void lcd_write_nibble(char byte){
   PORTD &= 0x0F;  // zero MS nibble
   PORTD |= (byte<<4) & 0xF0; // set MS nibble
   lcd_strobe();
}

void lcd_write_byte(char byte){
   _delay_us(40);
   lcd_write_nibble( (byte>>4) & 0x0F ); 
   lcd_write_nibble( byte & 0x0F );
}

void lcd_data_mode(void){
   PORTB |= (1 << RS);
}

void lcd_command_mode(void){
   PORTB &= ~(1 << RS);
}

void lcd_strobe(void) {
   PORTB |= (1 << EN);
   _delay_us(1);
   PORTB &= ~(1 << EN);
   _delay_us(1);
}

