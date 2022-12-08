
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
#include "LCD_display.h"

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

int main()
{

  i2cFileDesc = initI2cBus(I2CDRV_LINUX_BUS1, I2C_ADDR);

  runCommand(ENABLE_I2C_P9_17);
  runCommand(ENABLE_I2C_P9_18);

  init_LCD();

  int i = 0;
  printf("is running\n");
  while (1)
  {

    if (i >= 10)
    {
      LCD_cursor_location(LINE1);
      write_string_LCD("Andrew is wasted");

      LCD_cursor_location(LINE2);
      write_string_LCD("Drank ");
      write_int_to_LCD(i);
      write_string_LCD(" beers");
      i = 0;
      sleepForMs(3000);
      clear_LCD_screen();
      sleepForMs(3000);
      LCD_cursor_location(LINE1);
      write_string_LCD("The Next Day");
      sleepForMs(1000);
    }

    LCD_cursor_location(LINE1);
    write_string_LCD("Andrew drank ");
    write_int_to_LCD(i);
    LCD_cursor_location(LINE2);
    write_string_LCD("beers tonight");
    i++;
    sleepForMs(500);
  }

  return 0;
}

void runCommand(char *command)
{
  // Execute the shell command (output into pipe)
  FILE *pipe = popen(command, "r");
  // Ignore output of the command; but consume it
  // so we don't get an error when closing the pipe.
  char buffer[1024];
  while (!feof(pipe) && !ferror(pipe))
  {
    if (fgets(buffer, sizeof(buffer), pipe) == NULL)
      break;
    // printf("--> %s", buffer); // Uncomment for debugging
  }

  // Get the exit code from the pipe; non-zero is an error:
  int exitCode = WEXITSTATUS(pclose(pipe));
  if (exitCode != 0)
  {
    perror("Unable to execute command:");
    printf(" command: %s\n", command);
    printf(" exit code: %d\n", exitCode);
  }
}

int initI2cBus(char *bus, int address)
{
  int i2cFileDesc = open(bus, O_RDWR);
  int result = ioctl(i2cFileDesc, I2C_SLAVE, address);
  if (result < 0)
  {
    perror("I2C: Unable to set I2C device to slave address.");
    exit(1);
  }
  return i2cFileDesc;
}

// float to string
void write_float_to_LCD(float input_float)
{
  char buffer[20];
  sprintf(buffer, "%4.2f", input_float);
  write_string_LCD(buffer);
}

// int to string
void write_int_to_LCD(int input_int)
{
  char buffer[20];
  sprintf(buffer, "%d", input_int);
  write_string_LCD(buffer);
}

void clear_LCD_screen(void)
{
  write_LCD_byte(0x01, LCD_CMD_MODE);
  write_LCD_byte(0x02, LCD_CMD_MODE);
}

// go to location on LCD location=(line + offset)
void LCD_cursor_location(int location)
{
  write_LCD_byte(location, LCD_CMD_MODE);
}

// out char to LCD at current position
void write_char_LCD(char input_char)
{
  write_LCD_byte(input_char, LCD_DATA_MODE);
}

// this allows use of any size string
void write_string_LCD(const char *input_string)
{
  // while at valid charcter in string/not at the end
  // print character and and go to next position
  while (*input_string)
  {
    write_LCD_byte(*input_string, LCD_DATA_MODE);
    *(input_string++);
  }
}

void write_LCD_byte(int bits, int mode)
{

  // bits are the data beig sent
  // mode is 1 for writing data and 0 for sending a command
  int bits_high;
  int bits_low;
  // uses the two half byte writes to LCD
  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT;
  bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT;

  // High bits
  write_LCD_register(i2cFileDesc, bits_high);
  enable_LCD(bits_high);

  // Low bits
  write_LCD_register(i2cFileDesc, bits_low);
  enable_LCD(bits_low);
}

void enable_LCD(int bits)
{
  // Toggle enable pin on LCD display
  sleepForMs(1);
  write_LCD_register(i2cFileDesc, (bits | ENABLE));
  sleepForMs(1);
  write_LCD_register(i2cFileDesc, (bits & ~ENABLE));
  sleepForMs(1);
}

// more info at https://mil.ufl.edu/3744/docs/lcdmanual/commands.html on how commands work
void init_LCD()
{
  // Initialise display
  write_LCD_byte(0x33, LCD_CMD_MODE); // Initialise
  write_LCD_byte(0x32, LCD_CMD_MODE); // Initialise
  write_LCD_byte(0x06, LCD_CMD_MODE); // move cursor right, don’t shift display
  write_LCD_byte(0x0C, LCD_CMD_MODE); // display on, cursor Off
  write_LCD_byte(0x28, LCD_CMD_MODE); // Data length, number of lines, font size
  write_LCD_byte(0x01, LCD_CMD_MODE); // Clear display
  sleepForMs(1);
}

void sleepForMs(long long delayInMs)
{
  const long long NS_PER_MS = 1000 * 1000;
  const long long NS_PER_SECOND = 1000000000;
  long long delayNs = delayInMs * NS_PER_MS;
  int seconds = delayNs / NS_PER_SECOND;
  int nanoseconds = delayNs % NS_PER_SECOND;
  struct timespec reqDelay = {seconds, nanoseconds};
  nanosleep(&reqDelay, (struct timespec *)NULL);
}

// adpated from <wiringPiI2C.h> library
static inline int i2c_smbus_access(int i2cFileDesc, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args;

  args.read_write = rw;
  args.command = command;
  args.size = size;
  args.data = data;
  return ioctl(i2cFileDesc, I2C_SMBUS, &args);
}

// adpated from <wiringPiI2C.h> library
int write_LCD_register(int i2cFileDesc, int reg)
{
  union i2c_smbus_data data;

  if (i2c_smbus_access(i2cFileDesc, I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA, &data))
    return -1;
  else
    return data.byte & 0xFF;
}
