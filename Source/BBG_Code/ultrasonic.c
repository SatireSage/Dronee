#include "ultrasonic.h"

static bool running = false;
static pthread_t pulse;

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
        sleepForMs(100);
        writeIntToFile(Trig_value, 1);
        sleepForMs(0.01);
        writeIntToFile(Trig_value, 0);
    }
    return NULL;
}

// grabs the data from the ultrasonic sensor
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

// returns how far away the target is from the sensor in cm
long double get_distance_cm()
{
    long double start_time = 0;
    long double end_time = 0;
    long double length_of_time = 0;
    long double distance_in_cm = 0;

    while (getDataFromFile(Echo_value) == 48) // ascii 0
        start_time = getTimeInNs();
    while (getDataFromFile(Echo_value) == 49) // ascii 1
        end_time = getTimeInNs();
    length_of_time = end_time - start_time;
    distance_in_cm = length_of_time * 0.000017150; // convert to cm
    if (distance_in_cm <= 0)
        distance_in_cm = 0;

    return distance_in_cm;
}

// initialize sensor
void init_ultrasonic()
{
    runCommand(allowInputEcho);
    runCommand(allowInputTrig);
    writeStringToFile(Echo_direction, "in");
    writeStringToFile(Trig_direction, "out");
    writeStringToFile(Echo_active_low, "0");

    running = true;
    pthread_create(&pulse, NULL, pulse_loop, NULL);
}

// cleanup sensor
void cleanup_ultrasonic()
{
    running = false;
    pthread_join(pulse, NULL);
}