#include <stdint.h>

#define OW_PIN  PB2
#define OW_IN   PINB
#define OW_OUT  PORTB
#define OW_DDR  DDRB

#define OW_CONF_CYCLESPERACCESS 13
#define OW_CONF_DELAYOFFSET ( (uint16_t)( ((OW_CONF_CYCLESPERACCESS)*1000000L) / F_CPU  ) )
#define OW_MATCH_ROM	0x55
#define OW_SKIP_ROM     0xCC
#define	OW_SEARCH_ROM	0xF0
#define	OW_SEARCH_FIRST	0xFF		// start new search
#define	OW_PRESENCE_ERR	0xFF
#define	OW_DATA_ERR     0xFE
#define OW_LAST_DEVICE	0x00		// last device found
//			0x01 ... 0x40: continue searching

// rom-code size including CRC
#define OW_ROMCODE_SIZE 8

#define OW_MAX_DEV 2

uint8_t ow_reset(void);
uint8_t ow_bit_io( uint8_t b );
uint8_t ow_byte_wr( uint8_t b );
uint8_t ow_byte_rd( void );
uint8_t ow_rom_search( void );
void ow_command( uint8_t command, uint8_t *id );
void ow_parasite_enable(void);
void ow_parasite_disable(void);
uint8_t ow_input_pin_state(void);

void start_conv_ds18x20( uint8_t* );
int8_t read_ds18x20( uint8_t*, double*);
void ds18b20_10bit(void);
int8_t read_ds2406_status_mem( uint8_t*);
uint8_t	crc8(uint8_t* data, uint16_t len);
uint16_t crc16(uint16_t crc, uint8_t a);

uint8_t romcodes[OW_MAX_DEV][8];

