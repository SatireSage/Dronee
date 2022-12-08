// This file is driver code for the hd44780 LCD
////more info on commands at https://mil.ufl.edu/3744/docs/lcdmanual/commands.html on how commands work

#ifndef _LCD_DISPLAY_H_
#define _LCD_DISPLAY_H_

// C library headers
#include "common.h"

// Define some device parameters
#define I2C_ADDR 0x27 // I2C device address

// Define some device constants
#define LCD_DATA_MODE 1 // Mode - Sending data
#define LCD_CMD_MODE 0  // Mode - Sending command

#define LINE1 0x80 // 1st line of LCDdisplay
#define LINE2 0xC0 // 2nd line of LCDdisplay

#define LCD_BACKLIGHT 0x08 // On 0x00 for off

#define ENABLE 0b00000100 // Enable bit
#define I2CDRV_LINUX_BUS0 "/dev/i2c-0"
#define I2CDRV_LINUX_BUS1 "/dev/i2c-1"
#define I2CDRV_LINUX_BUS2 "/dev/i2c-2"
#define ENABLE_I2C_P9_17 "config-pin P9_17 i2c"
#define ENABLE_I2C_P9_18 "config-pin P9_18 i2c"

// initilize the LCD display
void init_LCD(void);

// put cursor at line to start writing;  use by puting (line# + location on line)
// line locations are defined up above. EX.) LCD_cursor_location(Line1 + 1)
void LCD_cursor_location(int line);
void clear_LCD_screen(void);
void write_int_to_LCD(int input_int);
void write_float_to_LCD(float input_float);
void write_string_LCD(const char *input_string);
void write_char_LCD(char input_char);


#endif