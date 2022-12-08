// Watchdog example
// by Brian Fraser
#include <linux/watchdog.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdbool.h>

void dieOnError(_Bool successCondition, char *message);
int readWatchdogTimeout();
void changeWatchdogTimeout(int timeout_s);
void hitWatchdogOnKeypress();
void *showTimeThread(void *notUsed);

static int timeSinceLastHit = 0;
static int timeout = 0;

int main(int argCount, char *args[])
{
    // Help
    printf("Watchdog demo app. Use:\n");
    printf("  %s - Run app\n", args[0]);
    printf("  %s [timeout [s]] - Set timeout and run app\n", args[0]);
    printf("\n");

    // Read current timeout.
    timeout = readWatchdogTimeout();
    printf("Current watchdog interval is %d seconds.\n", timeout);

    // Change and re-read current timeout.
    if (argCount > 1)
    {
        int newTimeout = atoi(args[1]);
        changeWatchdogTimeout(newTimeout);
        timeout = readWatchdogTimeout();
        printf("Updated watchdog interval is %d seconds.\n", timeout);
    }

    // Start background thread to show time ticking
    pthread_t id;
    pthread_create(&id, NULL, &showTimeThread, NULL);

    // Allow user to experiment with watchdog hitting.
    hitWatchdogOnKeypress();

    return 0;
}

int readWatchdogTimeout()
{
    int fd = open("/dev/watchdog", O_RDWR);
    dieOnError(fd != -1, "Unable to open WD.");

    int interval_s = 0;
    int result = ioctl(fd, WDIOC_GETTIMEOUT, &interval_s);
    dieOnError(result == 0, "Unable to read watchdog timeout.");

    close(fd);
    return interval_s;
}

void changeWatchdogTimeout(int timeout_s)
{
    printf("Setting watchdog timout to %ds\n", timeout_s);

    int fd = open("/dev/watchdog", O_RDWR);
    dieOnError(fd != -1, "Unable to open WD.");

    int result = ioctl(fd, WDIOC_SETTIMEOUT, &timeout_s);
    dieOnError(result == 0, "Unable to set watchdog timout.");

    close(fd);
}

void hitWatchdogOnKeypress()
{
    int fd = open("/dev/watchdog", O_RDWR);
    dieOnError(fd != -1, "Unable to open WD.");

    printf("Possible commands: (f)ile, (i)/o, or (q)uit.\n");
    while (1)
    {
        char buffer[1024];
        buffer[0] = 0;
        scanf("%s", buffer);
        char key = buffer[0];

        if (key == 'f')
        {
            printf("Hitting WD via the file...\n");
            // Can write any text to WD's device driver.
            write(fd, "w", 1);
            timeSinceLastHit = 0;
        }
        else if (key == 'i')
        {
            printf("Hitting WD via ioctl...\n");
            ioctl(fd, WDIOC_KEEPALIVE, NULL);
            timeSinceLastHit = 0;
        }

        else if (key == 'q')
        {
            printf("Exiting... (and try to stop watchdog)\n");
            // Write a 'V' to disable WD ("Magic Close" feature).
            // (Some drivers don't respond to Magic Close; some always close)
            write(fd, "V", 1);
            break;
        }

        else
        {
            printf("Invalid key (%c), use (f)ile, (i)/o, or (q)uit.\n", key);
        }
    }

    // Close file to disable watchdog (or exit program auto-closes)
    close(fd);
}

void dieOnError(_Bool successCondition, char *message)
{
    if (!successCondition)
    {
        fprintf(stderr, "ERROR: %s\n", message);
        fprintf(stderr, "Error string: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }
}

// Show the time on background thread
void *showTimeThread(void *notUsed)
{
    while (true)
    {
        sleep(1);
        timeSinceLastHit++;
        printf("%2ds since WD hit (timeout = %d).\n", timeSinceLastHit, timeout);
    }
}