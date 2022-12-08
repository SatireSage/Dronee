#include "watchdog.h"

//hit the watchdog when given the watchdog file location
void hitWatchdog(int file)
{
	write(file, "w", 1);
}

//error out if unable to open watchdog file
static void dieOnError(_Bool successCondition, char *message)
{
	if (!successCondition) {
		fprintf(stderr, "ERROR: %s\n", message);
		fprintf(stderr, "Error string: %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}
}

//returns the watchdog file location
static int getWatchdogFile()
{
	int fd = open("/dev/watchdog", O_RDWR);
	dieOnError(fd != -1, "Unable to open WD.");
	return fd;
}

//change how long the watchdog timeout is
static void changeWatchdogTimeout(int timeout_s)
{
	printf("Setting watchdog timout to %ds\n", timeout_s);

	int fd = open("/dev/watchdog", O_RDWR);
	dieOnError(fd != -1, "Unable to open WD.");

	int result = ioctl(fd, WDIOC_SETTIMEOUT, &timeout_s);
	dieOnError(result == 0, "Unable to set watchdog timout.");

	close(fd);
}

//starts the watchdog and returns the watchdog file location
//input Timeout_time_in_s is how long it takes the watchdog
//to not be interacted with before it reboots 
int start_watchdog_timer(int Timeout_time_in_s)
{
	changeWatchdogTimeout(Timeout_time_in_s);
	int Watchfile = getWatchdogFile();
	hitWatchdog(Watchfile);

	return Watchfile;
}

//close watchdog with magic close to prevent broad reboot
void closeWatchdog(int file)
{
    write(file, "V", 1);
    close(file);
}