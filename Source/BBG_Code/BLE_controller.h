#ifndef BLE_CONTROLLER_H
#define BLE_CONTROLLER_H

#define _BSD_SOURCE
#define _DEFAULT_SOURCE

// C library headers
#include "common.h"

// DEFINES
// Right Joystick For Direction
#define A2D_FILE_VOLTAGE_DirectionX "/sys/bus/iio/devices/iio:device0/in_voltage5_raw"
#define A2D_FILE_VOLTAGE_DirectionY "/sys/bus/iio/devices/iio:device0/in_voltage6_raw"

// Left Joystick For Speed Control 
#define A2D_FILE_VOLTAGE_YawX "/sys/bus/iio/devices/iio:device0/in_voltage3_raw"
#define A2D_FILE_VOLTAGE_SpeedY "/sys/bus/iio/devices/iio:device0/in_voltage2_raw"

// Yellow Mode Button Direction / Value
#define BUTTON_DIRECTION_YELLOW_MODE "/sys/class/gpio/gpio44/direction"
#define BUTTON_VALUE_YELLOW_MODE "/sys/class/gpio/gpio44/value"

// Red Power Button Direction / Value
#define BUTTON_DIRECTION_RED_POWER "/sys/class/gpio/gpio26/direction"
#define BUTTON_VALUE_RED_POWER "/sys/class/gpio/gpio26/value"

#define A2D_VOLTAGE_REF_V 1.8
#define A2D_MAX_READING 4096
#define Upper_Threshold 3072
#define Lower_Threshold 1024
#define PWM_Max 255

#define Controller_Buffer_Size 2048
#define Buffer_Size 1024
#define Buff_Size 256

// EXTERNAL FUNCTION PROTOTYPES TO USE LIBRARY
void init_BLE();
void cleanup_BLE();

#endif