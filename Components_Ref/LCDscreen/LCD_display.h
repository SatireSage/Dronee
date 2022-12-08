// This file is driver code for the hd44780 LCD
////more info on commands at https://mil.ufl.edu/3744/docs/lcdmanual/commands.html on how commands work

#ifndef _LCD_DISPLAY_H_
#define _LCD_DISPLAY_H_

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdint.h>

#define I2C_ADDR 0x27 // LCD device address

#define LCD_CHR 1 // Mode - Sending data
#define LCD_CMD 0 // Mode - Sending command

#define LINE1 0x80 // 1st line of LCDdisplay
#define LINE2 0xC0 // 2nd line of LCDdisplay

#define LCD_BACKLIGHT 0x08 // On
// LCD_BACKLIGHT = 0x00  # Off

#define ENABLE 0b00000100 // Enable bit
#define I2CDRV_LINUX_BUS0 "/dev/i2c-0"
#define I2CDRV_LINUX_BUS1 "/dev/i2c-1"
#define I2CDRV_LINUX_BUS2 "/dev/i2c-2"
#define ENABLE_I2C_P9_17 "config-pin P9_17 i2c"
#define ENABLE_I2C_P9_18 "config-pin P9_18 i2c"

int i2cFileDesc;
// initilize the LCD display
void init_LCD(void);
// change the bites used to control the LCD
void write_LCD_byte(int bits, int mode);
void enable_LCD(int bits);

void write_int_to_LCD(int input_int);
void write_float_to_LCD(float input_float);

// put cursor at line to start writing
// use by puting (line# + location on line)
// line locations are defined up above
// EX.) LCD_cursor_location(Line1 + 1)
void LCD_cursor_location(int line);
void clear_LCD_screen(void);

void write_string_LCD(const char *input_string);
void write_char_LCD(char input_char);
// given by Dr fraser for I2C
int initI2cBus(char *bus, int address);
void sleepForMs(long long delayInMs);

// adpated from <wiringPiI2C.h> library
static inline int i2c_smbus_access(int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data);
// run a linux command
void runCommand(char *command);
// adpated from <wiringPiI2C.h> library
int write_LCD_register(int i2cFileDesc, int reg);

#endif