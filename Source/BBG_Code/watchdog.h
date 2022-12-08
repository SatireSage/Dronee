// Used to interact with watchdog on beaglebone 
#ifndef _WATCHDOG
#define _WATCHDOG

// library headers
#include <linux/watchdog.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "common.h"

//hit the watchdog when given the watchdog file location
void hitWatchdog(int file);
//starts the watchdog and returns the watchdog file location
//input Timeout_time_in_s is how long it takes the watchdog
//to not be interacted with before it reboots 
int start_watchdog_timer(int Timeout_time_in_s);

void closeWatchdog(int file);

#endif