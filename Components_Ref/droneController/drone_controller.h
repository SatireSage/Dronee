#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#define _BSD_SOURCE
#define _DEFAULT_SOURCE

// C library headers
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <stdbool.h>

// DEFINES
#define A2D_FILE_VOLTAGE_DirectionX "/sys/bus/iio/devices/iio:device0/in_voltage2_raw"
#define A2D_FILE_VOLTAGE_DirectionY "/sys/bus/iio/devices/iio:device0/in_voltage3_raw"

#define A2D_FILE_VOLTAGE_ModeX "/sys/bus/iio/devices/iio:device0/in_voltage5_raw"
#define A2D_FILE_VOLTAGE_SpeedY "/sys/bus/iio/devices/iio:device0/in_voltage6_raw"

// Path to USER Button files
#define BUTTON_DIRECTION "/sys/class/gpio/gpio72/direction" // USER Button Direction
#define BUTTON_VALUE "/sys/class/gpio/gpio72/value"         // USER Button Value

#define A2D_VOLTAGE_REF_V 1.8
#define A2D_MAX_READING 4096
#define Upper_Threshold 3072
#define Lower_Threshold 1024
#define PWM_Max 255

#define Controller_Buffer_Size 2048
#define Buffer_Size 1024
#define Buff_Size 256

// FUNCTION PROTOTYPES
void runCommand(char *command);
void configPins();
int openPort();
char *writePort(char *command);
void configPort();
int getVoltage0Reading(char *file_value);
char *getDirection(int reading, char dir);
char *getSpeed(int reading);
void getMode(int reading);
char *createPacket();
void sendPacket(char *packet);
void sleepForMs(long long delayInMs);
void OpenCheck(FILE *fp, char *message);
void WrittenCheck(int status, char *message);
int readButton();
bool check_finish(int button_intterupt);

#endif