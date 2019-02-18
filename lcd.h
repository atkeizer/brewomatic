/*

$Header: /home/atkeizer/brouwtomaat/RCS/lcd.h,v 1.2 2016/05/16 08:50:14 atkeizer Exp $

$Log: lcd.h,v $
Revision 1.2  2016/05/16 08:50:14  atkeizer
lcd_puts_p added

Revision 1.1  2016/04/30 08:55:00  atkeizer
Initial revision


*/

void lcd_init(void);
void lcd_puts(char *data);
void lcd_puts_p(const char *prog_data);
void lcd_gotoxy(char x, char y);
void lcd_write_byte(char byte);
void lcd_write_nibble(char byte);
void lcd_char_gen(char * pixelrow);
void lcd_data_mode(void);
void lcd_command_mode(void);
void lcd_strobe(void);
void lcd_clear(void);
