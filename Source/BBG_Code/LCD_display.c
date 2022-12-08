#include "LCD_display.h"

static int i2cFileDesc;

// given by Dr fraser for I2C
static int initI2cBus(char *bus, int address)
{
  int i2cFileDesc1 = open(bus, O_RDWR);
  int result = ioctl(i2cFileDesc1, I2C_SLAVE, address);
  if (result < 0)
  {
    perror("I2C: Unable to set I2C device to slave address.");
    exit(1);
  }
  return i2cFileDesc1;
}

// adpated from <wiringPiI2C.h> library
static inline int i2c_smbus_access(int i2cFileDesc1, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args;

  args.read_write = rw;
  args.command = command;
  args.size = size;
  args.data = data;
  return ioctl(i2cFileDesc1, I2C_SMBUS, &args);
}

// adpated from <wiringPiI2C.h> library
static int write_LCD_register(int i2cFileDesc1, int reg)
{
  union i2c_smbus_data data;

  if (i2c_smbus_access(i2cFileDesc1, I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA, &data))
    return -1;
  else
    return data.byte & 0xFF;
}

static void enable_LCD(int bits)
{
  // Toggle enable pin on LCD display
  sleepForMs(1);
  write_LCD_register(i2cFileDesc, (bits | ENABLE));
  sleepForMs(1);
  write_LCD_register(i2cFileDesc, (bits & ~ENABLE));
  sleepForMs(1);
}

// change the bites used to control the LCD
static void write_LCD_byte(int bits, int mode)
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
    write_LCD_byte(*input_string++, LCD_DATA_MODE);
  }
}

// more info at https://mil.ufl.edu/3744/docs/lcdmanual/commands.html on how commands work
void init_LCD()
{
  runCommand(ENABLE_I2C_P9_17);
  runCommand(ENABLE_I2C_P9_18);
  i2cFileDesc = initI2cBus(I2CDRV_LINUX_BUS1, I2C_ADDR);
  // Initialise display
  write_LCD_byte(0x33, LCD_CMD_MODE); // Initialise
  write_LCD_byte(0x32, LCD_CMD_MODE); // Initialise
  write_LCD_byte(0x06, LCD_CMD_MODE); // move cursor right, don’t shift display
  write_LCD_byte(0x0C, LCD_CMD_MODE); // display on, cursor Off
  write_LCD_byte(0x28, LCD_CMD_MODE); // Data length, number of lines, font size
  write_LCD_byte(0x01, LCD_CMD_MODE); // Clear display
  sleepForMs(1);
}