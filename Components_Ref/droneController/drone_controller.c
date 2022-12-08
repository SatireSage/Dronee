#include "drone_controller.h"

// VARIABLES
int fd;   // File descriptor for the port
int mode; // The display mode

// FUNCTION DEFINITIONS
void runCommand(char *command)
{
    // Execute the shell command (output into pipe)
    FILE *pipe = popen(command, "r");
    // Ignore output of the command; but consume it
    // so we don't get an error when closing the pipe.
    char buffer[Buffer_Size];
    while (!feof(pipe) && !ferror(pipe))
    {
        if (fgets(buffer, sizeof(buffer), pipe) == NULL)
            break;
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

void configPins()
{
    // Set the GPIO pins to the correct mode
    runCommand("config-pin P9_21 uart");
    runCommand("config-pin P9_22 uart");
    // configure user button to be gpio
    runCommand("config-pin p8.43 gpio");

    FILE *pButton = NULL;
    int charWritten = 0;

    // Set the direction of the button to input
    pButton = fopen(BUTTON_DIRECTION, "w");
    OpenCheck(pButton, "Failed to open button file.");
    charWritten = fprintf(pButton, "in");
    WrittenCheck(charWritten, "Failed to write to button file.");
    fclose(pButton);
}

int openPort()
{
    fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        // Could not open the port.
        perror("open_port: Unable to open /dev/ttyS2 - ");
    }
    else
        fcntl(fd, F_SETFL, 0);

    return (fd);
}

char *writePort(char *command)
{
    // Write to the port AT
    char *at = command;
    int n = write(fd, at, strlen(at));
    if (n < 0)
        fputs("write() of 2 bytes failed! - ", stderr);

    // Read back the response
    // use malloc to allocate memory for the response
    char *response = malloc(Buffer_Size);
    n = read(fd, response, sizeof(response));

    if (n < 0)
    {
        fputs("read failed! - ", stderr);
        free(response);
        response = NULL;
        close(fd);
        exit(1);
    }
    else
    {
        response[n] = '\0';
    }

    return response;
}

void configPort()
{
    // code to connect to the BLE device over UART on serial port /dev/ttyO2
    // set set Baud Rate:115200, Databits: 8, Stopbits: 1 and No Flow Control.
    configPins();
    fd = openPort();
    if (fd == -1)
    {
        printf("Error opening port /dev/ttyS2 - %s\n", strerror(errno));
        close(fd);
        exit(1);
    }

    //  Set the baud rate
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // Set the number of data bits and stop bits
    options.c_cflag &= ~CSIZE; // Mask the character size bits
    options.c_cflag |= CS8;    // Select 8 data bits
    options.c_cflag &= ~CSTOPB;

    // Set no flow control
    options.c_cflag &= ~CRTSCTS;

    // Make raw
    cfmakeraw(&options);

    // Flush port, then applies attributes
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("init_serialport: Couldn’t set term attributes - ");
        close(fd);
    }
}

int getVoltage0Reading(char *file_value)
{
    // Open file
    FILE *f = fopen(file_value, "r");
    if (!f)
    {
        printf("ERROR: Unable to open voltage input file. Cape loaded?\n");
        printf(" Check /boot/uEnv.txt for correct options.\n");
        exit(-1);
    }

    // Get reading
    int a2dReading = 0;
    int itemsRead = fscanf(f, "%d", &a2dReading);
    if (itemsRead <= 0)
    {
        printf("ERROR: Unable to read values from voltage input file.\n");
        exit(-1);
    }
    // Close file
    fclose(f);
    return a2dReading;
}

char *getDirection(int reading, char dir)
{
    if (dir == 'x')
    {
        if (reading > Upper_Threshold)
        {
            unsigned pwm = (reading - Upper_Threshold) * PWM_Max / Lower_Threshold;
            char *pwm_str = malloc(4);
            sprintf(pwm_str, "%d", pwm);
            return pwm_str;
            // return "D_Right";
        }
        else if (reading < Lower_Threshold)
        {
            int invertedReading = 0 - (1024 - reading);
            unsigned pwm = (invertedReading)*PWM_Max / Lower_Threshold;
            char *pwm_str = malloc(4);
            sprintf(pwm_str, "%d", pwm);
            return pwm_str;
            // return "D_Left";
        }
        else
        {
            return "D_Center";
        }
    }
    else if (dir == 'y')
    {
        if (reading > Upper_Threshold)
        {
            unsigned pwm = (reading - Upper_Threshold) * PWM_Max / Lower_Threshold;
            char *pwm_str = malloc(4);
            sprintf(pwm_str, "%d", pwm);
            return pwm_str;
            // return "D_Forward";
        }
        else if (reading < Lower_Threshold)
        {
            int invertedReading = 0 - (1024 - reading);
            unsigned pwm = (invertedReading)*PWM_Max / Lower_Threshold;
            char *pwm_str = malloc(4);
            sprintf(pwm_str, "%d", pwm);
            return pwm_str;
            // return "D_Backward";
        }
        else
        {
            return "D_Center";
        }
    }
    else
    {
        return "error";
    }
}

// convert the reading to a pwm value between 0 and 255
char *getSpeed(int reading)
{
    if (reading > Upper_Threshold)
    {
        unsigned pwm = (reading - Upper_Threshold) * PWM_Max / Lower_Threshold;
        char *pwm_str = malloc(4);
        sprintf(pwm_str, "%d", pwm);
        return pwm_str;
        // return "S_Up";
    }
    else if (reading < Lower_Threshold)
    {
        int invertedReading = 0 - (1024 - reading);
        unsigned pwm = (invertedReading)*PWM_Max / Lower_Threshold;
        char *pwm_str = malloc(4);
        sprintf(pwm_str, "%d", pwm);
        return pwm_str;
        // return "S_Down";
    }
    else
    {
        // return 0;
        return "S_Center";
    }
}

// convert the reading to setting the mode
void setMode(int reading)
{
    if (reading > Upper_Threshold)
    {
        mode++;
    }
    else if (reading < Lower_Threshold)
    {
        mode--;
    }
}

char *createPacket()
{
    // Store the reading from the X Axis
    int reading__direction_x = getVoltage0Reading(A2D_FILE_VOLTAGE_DirectionX);
    char *direction_x = getDirection(reading__direction_x, 'x');

    // Store the reading from the Y Axis
    int reading_y = getVoltage0Reading(A2D_FILE_VOLTAGE_DirectionY);
    char *direction_y = getDirection(reading_y, 'y');

    // Store the reading from the Z Axis
    int reading_z = getVoltage0Reading(A2D_FILE_VOLTAGE_SpeedY);
    char *speed = getSpeed(reading_z);

    char *packet = malloc(Buffer_Size);
    // check if all the values are: S_Center, D_Center
    if (strcmp(speed, "S_Center") == 0 && strcmp(direction_x, "D_Center") == 0 && strcmp(direction_y, "D_Center") == 0)
    {
        // return Center
        packet = "Center";
    }
    else
    {
        // concatenate the strings
        strcpy(packet, direction_x);
        strcat(packet, ",");
        strcat(packet, direction_y);
        strcat(packet, ",");
        strcat(packet, speed);
        // Print the string
        printf("Packet: %s\n", packet);
    }

    return packet;
}

void sendPacket(char *packet)
{
    char input[Buff_Size];
    strcpy(input, packet);
    int n = write(fd, packet, strlen(packet));
    if (n < 0)
        fputs("write() of controller packet failed! - ", stderr);
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

// Function to check if file was opened correctly
void OpenCheck(FILE *fp, char *message)
{
    // Check for null pointer (file not opened)
    if (fp == NULL)
    {
        printf("Error: %s\n", message);
        exit(-1);
    }
}

// Function to check if file was written to correctly
void WrittenCheck(int status, char *message)
{
    // Check for error (file not written to)
    if (status < 0)
    {
        printf("Error: %s\n", message);
        exit(-1);
    }
}

// Function to read the USER Button is pressed
int readButton()
{
    FILE *pButton = NULL;
    int buttonValue = 0;

    pButton = fopen(BUTTON_VALUE, "r");
    OpenCheck(pButton, "Failed to open button file.");
    fscanf(pButton, "%d", &buttonValue);
    fclose(pButton);
    buttonValue = !buttonValue;
    return buttonValue;
}

bool check_finish(int button_intterupt)
{
    if (button_intterupt == 1)
    {
        close(fd);
        return true;
    }
    else
    {
        return false;
    }
}