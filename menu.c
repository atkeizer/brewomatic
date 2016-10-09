/*

$Header$

$Log$

*/

#include <avr/io.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "menu.h"
#include "touch.h"
#include "uart.h"
#include "lcd.h"


char line_buf[20];
uint8_t action;
static uint8_t ssr_duty =0;
static char pump_duty = 0;


typedef struct menu_item{
   const char * title;
   void (*action)(void);
   uint8_t items;
   struct menu_item *item_list;   // list of submenu items
} menu_item_t;


const char toplevel_tt[] PROGMEM = "Menu",
           continue_tt[] PROGMEM = "Continue",
               pump_tt[] PROGMEM = "Pump",
             heater_tt[] PROGMEM = "Heater",
           settings_tt[] PROGMEM = "Settings",
         mash_steps_tt[] PROGMEM = "Mash Steps",
         mash_sched_tt[] PROGMEM = "Mash Schedule",
                pid_tt[] PROGMEM = "PID Settings";

menu_item_t settings_menu[] =  
   { { mash_steps_tt, &mash_steps, 0, NULL },
     { mash_sched_tt, &mash_sched, 0, NULL },
     { pid_tt, &do_set_pid, 0, NULL } };

menu_item_t main_menu[] = {
   { .title = continue_tt, .action = &do_cont, .items = 0, .item_list = NULL },
   { .title = pump_tt, .action = &pump, .items = 0, .item_list = NULL },
   { .title = heater_tt, .action = &heater, .items = 0, .item_list = NULL },
   { .title = settings_tt, .action = &settings, .items = 3, .item_list = settings_menu }
};

menu_item_t toplevel = {
  .title = toplevel_tt,
  .action = &menu,
  .items = 3,
  .item_list = main_menu
};
  
  
void process_menu(uint8_t button) {
   static menu_item_t *current_item  = &toplevel;
   static int8_t menu_item=0;
   action = button;
   switch ( button ) {
            case TOUCH_ENTER:
               break;
            case TOUCH_RIGHT:
               if ( ++menu_item >= current_item->items ) menu_item=0;
               current_item->item_list[menu_item].action();
               break;
            case TOUCH_LEFT:
               if ( menu_item-- == 0 ) menu_item = current_item->items -1;
               current_item->item_list[menu_item].action();
               break;
            case TOUCH_UP:
               current_item->item_list[menu_item].action();
               break;
            case TOUCH_DOWN:
               current_item->item_list[menu_item].action();
               break;
   }
   //printf( "%S\n", (wchar_t *) current_item->title );
}


void menu() {
   lcd_gotoxy(0,0);
   lcd_puts_p(PSTR("Continue           "));
}

void do_cont() {
   lcd_gotoxy(0,0);
   lcd_puts_p(PSTR("Continue           "));
}

void pump() {
   uint8_t pump_power;
   lcd_gotoxy(0,0);
   lcd_puts_p(PSTR("Pump          %    "));
   if (action==TOUCH_UP) {
      if (pump_duty < 100) {
         pump_duty+=5;
      }
      if (pump_duty < 25 ) {
         pump_duty=25;
      }
   }
   if (action==TOUCH_DOWN ) {
      if (pump_duty > 25 ){
         pump_duty-=5;
         } else {
         pump_duty=0;
      }
   }
   if (pump_duty > 100) pump_duty=100;
   sprintf(line_buf, "%3d\0", pump_duty);
   lcd_gotoxy(10,0);
   lcd_puts(line_buf);
   pump_power = (uint8_t)(sqrt((double)pump_duty/100) * 255);
   pump_pwm(pump_power);
   sprintf(line_buf, "%3d\0", pump_power);
   lcd_gotoxy(10,1);
   lcd_puts(line_buf);
}

void heater() {
   lcd_gotoxy(0,0);
   lcd_puts_p(PSTR("Heater        %    "));
   if (action==TOUCH_UP && ssr_duty < 100) ssr_duty+=5;
   if (action==TOUCH_DOWN && ssr_duty > 0) ssr_duty-=5;
   if (ssr_duty > 100) ssr_duty=100;
   sprintf(line_buf, "%3d\0", ssr_duty); // % = 0010 0101
   lcd_gotoxy(10,0);
   lcd_puts(line_buf);
   heater_duty(ssr_duty);
}

void settings() {
   lcd_gotoxy(0,0);
   lcd_puts_p(PSTR("Settings           "));
}

void mash_steps() {
   lcd_gotoxy(0,0);
   lcd_puts_p(PSTR("Mash steps         "));
}

void mash_sched() {
   lcd_gotoxy(0,0);
   lcd_puts_p(PSTR("Mash Schedule      "));
}

void do_set_pid() {
   lcd_gotoxy(0,0);
   lcd_puts_p(PSTR("Set PID parameters  "));
}

