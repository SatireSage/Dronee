#include "ultrasonic.h"

static bool running = false;

// assumed using pin 9_ 23 for Echo and pin 9_15 for Trig
int main()
{

    long double distance_in_cm = 0;

    init_beaglebone();
    printf("distance measurment using ultrasonic sensor \n");
    running = true;

    pthread_t pulse;
    pthread_create(&pulse, NULL, pulse_loop, &distance_in_cm);

    while (running)
    {
        distance_in_cm = get_distance_cm();
    }

    pthread_join(pulse, NULL);

    return 0;
}

static void runCommand(char *command)
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

static void sleepForMs(long long delayInMs)
{
    const long long NS_PER_MS = 1000 * 1000;
    const long long NS_PER_SECOND = 1000000000;
    long long delayNs = delayInMs * NS_PER_MS;
    int seconds = delayNs / NS_PER_SECOND;
    int nanoseconds = delayNs % NS_PER_SECOND;
    struct timespec reqDelay = {seconds, nanoseconds};
    nanosleep(&reqDelay, (struct timespec *)NULL);
}

static void writeStringToFile(char *fileName, char *input)
{
    sleepForMs(300);
    FILE *pFile = fopen(fileName, "w");
    if (pFile == NULL)
    {
        printf("ERROR: Unable to open export file.\n");
        exit(1);
    }
    // Write to data to the file using fprintf():
    fprintf(pFile, "%s", input);
    // Close the file using fclose():
    fclose(pFile);
    // Call nanosleep() to sleep for ~300ms before use.
}

static void readFromFileToScreen(char *fileName)
{
    FILE *pFile = fopen(fileName, "r");
    if (pFile == NULL)
    {
        printf("ERROR: Unable to open file (%s) for read\n", fileName);
        exit(-1);
    }
    // Read string (line)
    const int MAX_LENGTH = 1024;
    char buff[MAX_LENGTH];
    fgets(buff, MAX_LENGTH, pFile);
    // Close
    fclose(pFile);
    printf("Read: '%s'\n", buff);
}

static long long getTimeInNs(void)
{
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    long long seconds = spec.tv_sec;
    long long nanoSeconds = spec.tv_nsec;
    long long total_nanoSeconds = seconds * 1000000000 + nanoSeconds;
    return total_nanoSeconds;
}

static int getDataFromFile(char *fileName)
{
    FILE *pFile = fopen(fileName, "r");
    if (pFile == NULL)
    {
        printf("ERROR: Unable to open file (%s) for read\n", fileName);
        exit(-1);
    }
    // Read string (line)
    const int MAX_LENGTH = 1024;
    char buff[MAX_LENGTH];
    fgets(buff, MAX_LENGTH, pFile);
    // Close
    fclose(pFile);
    return buff[0];
}

static void writeIntToFile(char *fileName, int input)
{
    sleepForMs(300);
    FILE *pFile = fopen(fileName, "w");
    if (pFile == NULL)
    {
        printf("ERROR: Unable to open export file.\n");
        exit(1);
    }
    // Write to data to the file using fprintf():
    fprintf(pFile, "%d", input);
    // Close the file using fclose():
    fclose(pFile);
    // Call nanosleep() to sleep for ~300ms before use.
}

// used to create a thread that sends an impulse every 0.2s
static void *pulse_loop(void *input)
{
    while (running)
    {

        sleepForMs(200);
        writeIntToFile(Trig_value, 1);
        sleepForMs(0.01);
        writeIntToFile(Trig_value, 0);
    }
    return NULL;
}

// returns how far away the target is from the sensor in cm
static long double get_distance_cm()
{
    long double start_time = 0;
    long double end_time = 0;
    long double length_of_time = 0;
    long double distance_in_cm = 0;

    while (getDataFromFile(Echo_value) == 48) // ascii 0
    {
        start_time = getTimeInNs();
    }
    while (getDataFromFile(Echo_value) == 49) // ascii 1
    {
        end_time = getTimeInNs();
    }
    length_of_time = end_time - start_time;
    distance_in_cm = length_of_time * 0.000017150; // convert to cm
    if (distance_in_cm <= 0)
    {
        distance_in_cm = 0;
    }
    printf("distance: %.1Lf cm \n", distance_in_cm);

    return distance_in_cm;
}

// initialize all beaglebone pins
static void init_beaglebone()
{
    runCommand(allowInputEcho);
    runCommand(allowInputTrig);
    writeStringToFile(Echo_direction, "in");
    writeStringToFile(Trig_direction, "out");
    writeStringToFile(Echo_active_low, "0");
    readFromFileToScreen(Echo_direction);
    readFromFileToScreen(Trig_direction);
}