// Provides common libraries and functions used between multiple files
#ifndef _COMMON_H_
#define _COMMON_H_

// Common Linux/C library headers
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <stdbool.h>
#include <pthread.h>
#include <string.h>
#include <stdint.h>

// Other Linux/C headers
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions

// Function declarations
void runCommand(char *command);
void sleepForMs(long long delayInMs);
long long getTimeInNs(void);
int getReading(char *file_value);
void writeStringToFile(char *fileName, char *input);

#endif