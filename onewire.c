/*
        Access Dallas 1-Wire Device with ATMEL AVRs
                                                  
              Author: Peter Dannegger            
                      danni@specs.de            
                                                                
 modified by Martin Thomas <eversmith@heizung-thomas.de> 9/2004

 modified by Albert Keizer <atkeizer@gmail.com> 1/2012
     fixed rom search algorithm to allow more than 2 devices
     fixed byte order of rom code
     fixed ow_command for correct rom byte order
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include "onewire.h"



#define OW_GET_IN()   ( OW_IN & (1<<OW_PIN))
#define OW_OUT_LOW()  ( OW_OUT &= (~(1 << OW_PIN)) )
#define OW_OUT_HIGH() ( OW_OUT |= (1 << OW_PIN) )
#define OW_DIR_IN()   ( OW_DDR &= (~(1 << OW_PIN )) )
#define OW_DIR_OUT()  ( OW_DDR |= (1 << OW_PIN) )



uint8_t ow_rom_search()
{
   uint8_t byte, byte_bit, bit, branch=0, i, n=0, branches[OW_MAX_DEV];
   for ( i=0; i < OW_MAX_DEV; i++ )
      branches[i] = 0;
   do {
       if ( ow_reset() ) {
          return 0; // no sensors detected
       }
       ow_byte_wr( OW_SEARCH_ROM );
       bit = 0;
       do {
          byte =  7 - bit/8;
          byte_bit = (bit & 7);
          if ( ow_bit_io( 1 ) ) {     // if first bit is 1
             if ( ow_bit_io( 1 ) )    // and second bit is one, no devices participating in search
                return 0;
             else {                   // or second bit is 0, only devices with 1 in this bit position
                romcodes[n][byte] |= ( 1 << byte_bit );
                ow_bit_io( 1 );
             }
          }
          else {                      // first bit is 0
             if ( ow_bit_io( 1 ) ) {  // and second bit is 1, only devices with 0 in this bit position
                romcodes[n][byte] &= ~( 1 << byte_bit );
                ow_bit_io( 0 );
             }   
             else  {                 // both 1 and 0 in this bit position 
                if ( romcodes[n][byte] & ( 1 << byte_bit ) ) { // if bit was already set then continue
                                        // with the 1's branch, 0 branch was done in a previous iteration
                   ow_bit_io( 1 ); 
                   if ( branches[n] == bit ) { 
                      branch--; // only if current branch
                   }
                }
                else {
                   if ( branches[n] < bit ) {  // new branch 
                      branch++;
                      if ( n+branch < OW_MAX_DEV ) {  // avoid writing past reserved space
                         for ( i=byte ; i > 0 ; i-- )   // clone the rom id so far obtained
                            romcodes[n+branch][i] = romcodes[n][i];
                         romcodes[n+branch][byte] |= ( 1 << byte_bit ); // write 1 to the next romcode branch
                         romcodes[n][byte] &= ~( 1 << byte_bit ); // continue with 0 branch
                         branches[n] = bit;
                         branches[n+branch] = bit;
                      }
                   }
                   ow_bit_io( 0 );
                }
             }
          }      
       } while ( bit++ < 63 ) ;
    } while ( ( ++n < OW_MAX_DEV ) &&  branch ) ;
    return n;
}


uint8_t ow_input_pin_state()
{
    return OW_GET_IN();
}

void ow_parasite_enable(void)
{
    OW_OUT_HIGH();
    OW_DIR_OUT();
}

void ow_parasite_disable(void)
{
    OW_OUT_LOW();
    OW_DIR_IN();
}

uint8_t ow_reset(void)
{
    uint8_t err;
    uint8_t sreg;
    OW_OUT_LOW(); // disable internal pull-up (maybe on from parasite)
    OW_DIR_OUT(); // pull OW-Pin low for 480us
    _delay_us(480);
    sreg=SREG;
    cli();
    // set Pin as input - wait for clients to pull low
    OW_DIR_IN(); // input
    _delay_us(66);
    err = OW_GET_IN();        // no presence detect
    // nobody pulled to low, still high
    SREG=sreg; // sei()
    // after a delay the clients should release the line
    // and input-pin gets back to high due to pull-up-resistor
    _delay_us(480-66);
    if( OW_GET_IN() == 0 )        // short circuit
        err = 1;
    return err;
}

/* Timing issue when using runtime-bus-selection (!OW_ONE_BUS):
   The master should sample at the end of the 15-slot after initiating
   the read-time-slot. The variable bus-settings need more
   cycles than the constant ones so the delays had to be shortened 
   to achive a 15uS overall delay 
   Setting/clearing a bit in I/O Register needs 1 cyle in OW_ONE_BUS
   but around 14 cyles in configureable bus (us-Delay is 4 cyles per uS) */
uint8_t ow_bit_io( uint8_t b )
{
    uint8_t sreg;
    sreg=SREG;
    _delay_us(10); // Recovery
    //lcd_write_byte( b + 0x30);
    cli();
    OW_DIR_OUT(); // drive bus low
    _delay_us(1); // Recovery-Time wuffwuff was 1
    if ( b ) OW_DIR_IN(); // if bit is 1 set bus high (by ext. pull-up)
    // wuffwuff delay was 15uS-1 see comment above
    //_delay_us(15-1-OW_CONF_DELAYOFFSET);
    _delay_us(10);
    if( OW_GET_IN() == 0 ) b = 0;  // sample at end of read-timeslot
    _delay_us(60-15);
    OW_DIR_IN();
    SREG=sreg; // sei();
    return b;
}


uint8_t ow_byte_wr( uint8_t b )
{
    uint8_t i = 8, j;
    do {
        j = ow_bit_io( b & 1 );
        b >>= 1;
        if( j ) b |= 0x80;
    } while( --i );
    return b;
}


uint8_t ow_byte_rd( void )
{
  return ow_byte_wr( 0xFF ); 
}



void ow_command( uint8_t command, uint8_t *id )
{
    uint8_t i;
    ow_reset();
    if( id ) {
        ow_byte_wr( OW_MATCH_ROM );         // to a single device
        i=8;
        while (i > 0) ow_byte_wr( id[--i] );
    }   
    else {
      ow_byte_wr( OW_SKIP_ROM );            // to all devices
    } 
    ow_byte_wr( command );
}   



void start_conv_ds18x20( uint8_t *id )
{
   ow_command( 0x44, 0 ); // Start conversion for all
   ow_parasite_enable();
}


int8_t read_ds18x20( uint8_t *id, double *T )  // temp in hundreds degrees
{
   uint8_t i, scratch[9];
   int16_t raw;
//   if ( ! ( id[0] == 0x28 || id[0] == 0x10 )  ) {  // not ds18b20, ds1820 or ds18s20
//      return 2;
//   } else {
      ow_parasite_disable();
      ow_command(0xBE, id);  // Read scratchpad
      for ( i=0 ; i<9 ; i++ ) { 
          scratch[i]=ow_byte_rd();
      }
      if ( crc8( scratch, 9 ) )
         return 1;
      else {
         *T = (double)((scratch[1] << 8) | scratch[0]);  // 16 bit MSB, LSB
         switch (scratch[4] & 0x60) {           // configuration register
            case 0x00:
               *T /= 2;    // 9 bit resolution
               break;
            case 0x20:
               *T /= 4;    // 10 bit res
               break;
            case 0x40:
               *T /= 8;    // 11 bit res
               break;
            case 0x60:
               *T /= 16;   // 12 bit resolution
               break;
            case 0xff:        // DS18020 and DS18S20
               // byte 6 is Count Remain and byte 7 is Count per C (is hardwired to 16)
               *T = *T/2 - 0.25 + (16 - scratch[6])/16;
               break;
            }
         return 0;
      }
 //  }
}

void ds18b20_10bit(void) {
   uint8_t i;
   for (i=0;i < OW_MAX_DEV;i++){
      if (romcodes[i][0] == 0x28) { 
         ow_command(0x4E, romcodes[i]); // write scratchpad
         ow_byte_wr(0);            // byte 2 TL
         ow_byte_wr(0);            // byte 3 TH
         ow_byte_wr(0x20);         // byte 4 config register
         ow_command(0x48, romcodes[i]);   // copy Scratchpad
         ow_parasite_enable();
         _delay_ms(10);              // keep strong pullup 10ms while writing to eeprom
         ow_parasite_disable();
      }
   }
}


int8_t read_ds2406_status_mem( uint8_t *id )
{
   uint8_t i=0, output[8];
   uint16_t crc=0xffff;

   ow_command(0xF5,0);  
   output[i++]=ow_byte_wr(0x55);
   output[i++]=ow_byte_wr(0xFF);
   output[i]=ow_byte_rd();
   
   printf("%x %x %x %x %x\n", output[0], output[1], output[2], ow_byte_rd(), ow_byte_rd());
   crc =crc16(crc, output[2]);
   printf("CRC %x\n", crc);

   return output[2];
}


void set_ds2406_switch( uint8_t *id )
{
   ow_command(0x55,0);
   ow_byte_wr(0x07);
   ow_byte_wr(0x00);
   ow_byte_wr(0x7F); 
   ow_byte_wr(0xFF);
   ow_byte_wr(0xFF);
}

void clear_ds2406_switch( uint8_t *id )
{
   ow_command(0x55,0);
   ow_byte_wr(0x07);
   ow_byte_wr(0x00);
   ow_byte_wr(0x1F); 
   ow_byte_wr(0xFF);
   ow_byte_wr(0xFF);
}



uint8_t crc8 ( uint8_t *data_in, uint16_t len )
{
   uint8_t  crc=0, bit_counter, data, feedback_bit;
   uint16_t loop_count;
   for (loop_count = 0; loop_count != len; loop_count++)
   {
      data = data_in[loop_count];
      bit_counter = 8;
      do {
         feedback_bit = (crc ^ data) & 0x01;
         if ( feedback_bit == 0x01 ) {
            crc = crc ^ 0x18;           // X^8+X^5+X^4+X^0
         }
         crc = (crc >> 1) & 0x7F;
         if ( feedback_bit == 0x01 ) {
            crc = crc | 0x80;
         }
         data = data >> 1;
         bit_counter--;
      } while (bit_counter > 0);
   }
   return crc;
}


uint16_t crc16(uint16_t crc, uint8_t a)
{
   int i;
   crc ^= a;
   for (i = 0; i < 8; ++i)
   {
       if (crc & 1)
           crc = (crc >> 1) ^ 0xA001;
       else
           crc = (crc >> 1);
   }
   return crc;
}

