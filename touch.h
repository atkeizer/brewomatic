#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


uint8_t touch_scan(uint8_t mask);
uint8_t touch_scan_all();
uint8_t touch_scan_released();

uint8_t touch_state;

// mapping port pins
#define TOUCH_ENTER  1
#define TOUCH_RIGHT  2
#define TOUCH_UP     4
#define TOUCH_DOWN   8
#define TOUCH_LEFT   16

#define TOUCH_THRESHOLD 6

