#include "BLE_controller.h"
#include "LCD_display.h"
#include "ultrasonic.h"
#include "watchdog.h"

// VARIABLES
static int fd;                      // File descriptor for the port
static int watchdog;                // For use with watchdog functions
static int mode = 0;                // Controll Modes
static int prevMode = 0;            // The previous button mode
static int updatedStat = 1;         // Flags when to update the mode dipslayed
static int updateSpeed = 1;         // Flags when to update the speed displayed
static int prevNonCenterPacket = 0; // Flags if the last packet was non all centered
static int PWMtrack = 0;
static int disconnect = 0;

static bool running = false;
static pthread_t BLE;
static pthread_t ConnectionCheck;
static pthread_t HitWatchDog;
static pthread_t LCD;

static pthread_mutex_t fd_mutex = PTHREAD_MUTEX_INITIALIZER;

// Check if the file is open
void OpenCheck(FILE *fp, char *message)
{
    // Check for null pointer (file not opened)
    if (fp == NULL)
    {
        printf("Error: %s\n", message);
        exit(-1);
    }
}

// Check if the file is not written
void WrittenCheck(int status, char *message)
{
    // Check for error (file not written to)
    if (status < 0)
    {
        printf("Error: %s\n", message);
        exit(-1);
    }
}

// Function sets the direction of the button
static void setButtonDirection(char *buttonDirectionLocation)
{
    FILE *pButton = NULL;
    int charWritten = 0;
    pButton = fopen(buttonDirectionLocation, "w");
    OpenCheck(pButton, "Failed to open button file.");
    charWritten = fprintf(pButton, "in");
    WrittenCheck(charWritten, "Failed to write to button file.");
    fclose(pButton);
}

// Function reads the button value
static int readbutton(char *buttonValueLocation)
{
    FILE *pButton = NULL;
    int buttonValue = 0;

    pButton = fopen(buttonValueLocation, "r");
    OpenCheck(pButton, "Failed to open button file.");
    fscanf(pButton, "%d", &buttonValue);
    fclose(pButton);
    buttonValue = !buttonValue;
    return buttonValue;
}

// Configures the pins of the buttons and bluetooth module
static void configPins()
{
    // Set the GPIO pins to the correct mode
    runCommand("config-pin P9_21 uart");
    runCommand("config-pin P9_22 uart");

    // set the GPIO pins to the correct mode for the two controller buttons
    runCommand("config-pin P8_14 gpio"); // Red button Power
    runCommand("config-pin P8_12 gpio"); // Yellow button Mode

    setButtonDirection(BUTTON_DIRECTION_RED_POWER);
    setButtonDirection(BUTTON_DIRECTION_YELLOW_MODE);
}

static int openPort()
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

// Configures the BLE ports
static void configPort()
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
    struct termios settings;
    tcgetattr(fd, &settings);
    cfsetispeed(&settings, B9600);
    cfsetospeed(&settings, B9600);

    // Set the number of data bits and stop bits
    settings.c_cflag &= ~CSIZE; // Mask the character size bits
    settings.c_cflag |= CS8;    // Select 8 data bits
    settings.c_cflag &= ~CSTOPB;

    // Set no flow control
    settings.c_cflag &= ~CRTSCTS;

    // Make raw
    cfmakeraw(&settings);

    // Flush port, then applies attributes
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &settings) != 0)
    {
        perror("init_serialport: Couldn’t set term attributes - ");
        close(fd);
    }
}

// Retrieves the direction in terms of a PWM value that is of a useful range
static void getDirection(int reading, char dir, char *dirStr)
{
    if (dir == 'x') // Rolling / Yawing from a PWM range of -28 to 28
    {
        if (reading > Upper_Threshold) // Roll / Yawing Rightwards
        {
            unsigned pwm = (((reading - Upper_Threshold) * PWM_Max / Lower_Threshold) / (19 / 2));
            sprintf(dirStr, "%d", pwm);
        }
        else if (reading < Lower_Threshold) // Rolling Leftwards
        {
            int invertedReading = 0 - (1024 - reading);
            unsigned pwm = (((invertedReading)*PWM_Max / Lower_Threshold) / (19 / 2));
            sprintf(dirStr, "%d", pwm);
        }
        else
        {
            strcpy(dirStr, "D_Center");
        }
    }
    else if (dir == 'y') // Pitching from a PWM range of -42 to 42
    {
        if (reading > Upper_Threshold) // Pitch Forwards
        {
            unsigned pwm = (((reading - Upper_Threshold) * PWM_Max / Lower_Threshold) / (12 / 2));
            sprintf(dirStr, "%d", pwm);
        }
        else if (reading < Lower_Threshold) // Pitch Backwards
        {
            int invertedReading = 0 - (1024 - reading);
            unsigned pwm = (((invertedReading)*PWM_Max / Lower_Threshold) / (12 / 2));
            sprintf(dirStr, "%d", pwm);
        }
        else
        {
            strcpy(dirStr, "D_Center");
        }
    }
    else
    {
        strcpy(dirStr, "error");
    }
}

// convert the reading to a pwm value to increase and decrease speed betwen the ranges of -10 to 10
static void getSpeed(int reading, char *speed)
{
    if (reading > Upper_Threshold) // Increment Speed Up
    {
        unsigned pwm = (((reading - Upper_Threshold) * PWM_Max / Lower_Threshold) / (51 / 2));
        PWMtrack = pwm;
        updateSpeed = 1;
        sprintf(speed, "%d", pwm);
    }
    else if (reading < Lower_Threshold) // Increment Speed Down
    {
        int invertedReading = 0 - (1024 - reading);
        unsigned pwm = (((invertedReading)*PWM_Max / Lower_Threshold) / (51 / 2));
        PWMtrack = pwm;
        updateSpeed = 1;
        sprintf(speed, "%d", pwm);
    }
    else
    {
        PWMtrack = 0;
        updateSpeed = 1;
        strcpy(speed, "S_Center");
    }
}

// convert the ultrasonic reading to a PWM value
static void ultrasonicToSpeed(long double reading, char *speed)
{
    if (reading >= 30.0)
    {
        updateSpeed = 1;
        PWMtrack = 0;
        strcpy(speed, "0");
    }
    else if ((reading > 5.0) && (reading < 30.0))
    {
        unsigned pwm = (30 - reading) * (PWM_Max / 30);
        updateSpeed = 1;
        PWMtrack = pwm;
        sprintf(speed, "%d", pwm);
    }
    else
    {
        updateSpeed = 1;
        PWMtrack = 255;
        strcpy(speed, "255");
    }
}

// take the mode currently selected and change to a string to be passed on to the drone
static void getMode(char *modeStr)
{
    if ((mode == 0))
        strcpy(modeStr, "Off");
    else if ((mode == 1))
        strcpy(modeStr, "Ultra");
    else if ((mode == 2))
        strcpy(modeStr, "On");
    else
        strcpy(modeStr, "Shutdown");
}

// Creates the packet being sent to the arduino
static void createPacket(char *packet)
{
    char *speed = malloc(Buff_Size * sizeof(char));
    char *direction_x = malloc(Buff_Size * sizeof(char));
    char *direction_y = malloc(Buff_Size * sizeof(char));
    char *direction_yaw_x = malloc(Buff_Size * sizeof(char));
    char *modeSet = malloc(Buff_Size * sizeof(char));

    // Store the reading from the X Axis right joystick
    int reading__direction_x = getReading(A2D_FILE_VOLTAGE_DirectionX);
    getDirection(reading__direction_x, 'x', direction_x);

    // Store the reading from the Y Axis right joystick
    int reading_y = getReading(A2D_FILE_VOLTAGE_DirectionY);
    getDirection(reading_y, 'y', direction_y);

    // Store the reading from the X axis of left joystick
    int reading_yaw_direction_x = getReading(A2D_FILE_VOLTAGE_YawX);
    getDirection(reading_yaw_direction_x, 'x', direction_yaw_x);

    // Basing the Z axis from the use of the joystick (default) (Autobalance)
    if ((mode == 0) || (mode == 2) || (mode == 3))
    {
        // Store the reading from the Z Axis
        int reading_z = getReading(A2D_FILE_VOLTAGE_SpeedY);
        getSpeed(reading_z, speed);
    }

    // Basing the Z axis from the use of the ultrasonic sensor (ultrasonic)
    if (mode == 1)
    {
        long double reading_z_ultrasonic = get_distance_cm();
        ultrasonicToSpeed(reading_z_ultrasonic, speed);
    }

    // Store the mode for autobalancing
    getMode(modeSet);

    // check if all the values are: S_Center, D_Center and the the previous packet was not all center
    // Purpose is to tell the drone when the user has stopped rolling and pitching the joystick
    if (prevNonCenterPacket == 0 && strcmp(speed, "S_Center") == 0 && strcmp(direction_x, "D_Center") == 0 && strcmp(direction_y, "D_Center") == 0 && strcmp(direction_yaw_x, "D_Center") == 0 && (prevMode == mode))
    {
        strcpy(packet, "Center");
    }
    else
    {
        // concatenate the strings
        strcpy(packet, modeSet);
        strcat(packet, ",");
        strcat(packet, direction_x);
        strcat(packet, ",");
        strcat(packet, direction_y);
        strcat(packet, ",");
        strcat(packet, direction_yaw_x);
        strcat(packet, ",");
        strcat(packet, speed);

        // Print the string
        printf("Packet: %s\n", packet);
        prevNonCenterPacket = 1;
    }
    if ((mode == 3) || (strcmp(speed, "S_Center") == 0 && strcmp(direction_x, "D_Center") == 0 && strcmp(direction_y, "D_Center") == 0 && strcmp(direction_yaw_x, "D_Center") == 0 && (prevMode == mode)))
    {
        prevNonCenterPacket = 0;
    }
    free(speed);
    speed = NULL;
    free(direction_x);
    direction_x = NULL;
    free(direction_y);
    direction_y = NULL;
    free(direction_yaw_x);
    direction_yaw_x = NULL;
    free(modeSet);
    modeSet = NULL;
}

// Sends the packet
static void sendPacket(char *packet)
{
    char input[Buff_Size];
    strcpy(input, packet);
    pthread_mutex_lock(&fd_mutex);
    int n = write(fd, packet, strlen(packet));
    pthread_mutex_unlock(&fd_mutex);
    if (n < 0)
        fputs("write() of controller packet failed! - ", stderr);
}

// Thread that runs the BLE, sending packets of information to the drone
static void *BLEId(void *_arg)
{
    sleepForMs(1);
    while (running)
    {
        char *controllerPacket = malloc(Controller_Buffer_Size * sizeof(char));
        createPacket(controllerPacket);

        // check if the packet is not Center
        if ((strcmp(controllerPacket, "Center") != 0))
        {
            sendPacket(controllerPacket);
            prevMode = mode;
        }
        free(controllerPacket);
        controllerPacket = NULL;

        sleepForMs(100);
    }

    return NULL;
}

// This function switches between the modes for control and cycle through the three modes
static void set_mode()
{
    prevMode = mode;
    if (mode <= 2)
        mode++;
    if (mode > 2)
        mode = 0;
    updatedStat = 1;
}

// Thread for displaying user readable information about the commands sent to drone
static void *LCDId(void *_arg)
{
    int prevPWM = 1;
    // Welcome startup message
    LCD_cursor_location(LINE1);
    write_string_LCD("Welcome");
    LCD_cursor_location(LINE2);
    write_string_LCD("Controller is on");
    sleepForMs(1000);
    while (running)
    {
        // Display controll mode
        if (updatedStat == 1)
        {
            LCD_cursor_location(LINE1);
            if (mode == 0)
            {
                clear_LCD_screen();
                write_string_LCD("Mode:");
                write_string_LCD("Default");
            }
            else if (mode == 1)
            {
                clear_LCD_screen();
                write_string_LCD("Mode:");
                write_string_LCD("Ultrasonic");
            }
            else if (mode == 2)
            {
                clear_LCD_screen();
                write_string_LCD("Mode:");
                write_string_LCD("Autobalance");
            }
            updateSpeed = 1;
        }
        // Display speed alteration for joystick
        if ((updateSpeed == 1) && ((mode == 0) || (mode == 2)))
        {
            if ((prevPWM != 0) || (updatedStat == 1))
            {
                LCD_cursor_location(LINE2);
                write_string_LCD("                 ");
                LCD_cursor_location(LINE2);
                write_string_LCD("Alter PWM by ");
                write_int_to_LCD(PWMtrack);
                prevPWM = PWMtrack;
            }
            prevPWM = PWMtrack;
            updatedStat = 0;
            updateSpeed = 0;
        }

        // Display speed control for ultrasonic
        if ((updateSpeed == 1) && (mode == 1))
        {
            if ((prevPWM != PWMtrack) || (updatedStat == 1))
            {
                LCD_cursor_location(LINE2);
                write_string_LCD("                ");
                LCD_cursor_location(LINE2);
                write_string_LCD("Speed: ");
                write_int_to_LCD(PWMtrack);
                prevPWM = PWMtrack;
            }
            prevPWM = PWMtrack;
            updatedStat = 0;
            updateSpeed = 0;
        }
    }
    // Closing message when shutting down controller
    clear_LCD_screen();
    LCD_cursor_location(LINE1);
    write_string_LCD("Shutting Down");
    sleepForMs(1000);

    return NULL;
}

// Thread for hitting the watchdog in the specified interval and does not when disconnection is detected from ConnectionCheck thread
static void *HitWatchDogID(void *_arg)
{
    watchdog = start_watchdog_timer(60);
    while (running)
    {
        if (disconnect == 0) // If disconnection flag is not active
        {
            hitWatchdog(watchdog);
            sleep(15);
        }
        else if (disconnect == 1) // If disconnection flag is active
        {
            runCommand("echo 0 > /sys/class/gpio/gpio67/value"); // Turn off LED
            clear_LCD_screen();                                  // Clear LCD screen
            LCD_cursor_location(LINE1);
            write_string_LCD("Rebooting ...");
            sleep(65);
        }
    }
    return NULL;
}

// Thread that checks that connection is still present
static void *ConnectionCheckID(void *_arg)
{
    char returnCode[4];
    sleep(30);
    while (running)
    {
        sendPacket("AT+RSSB?");
        char buf[Buff_Size];
        int n = read(fd, buf, sizeof(buf));
        if (n < 0)
        {
            fputs("read failed! - ", stderr);
        }
        else
        {
            buf[n] = '\0';
            strcpy(returnCode, buf + strlen(buf) - 4);
        }
        if (strcmp(returnCode, "9999") == 0)
        {
            printf("Not Connected\n");
            disconnect = 1;
        }
        sleep(15);
    }
    return NULL;
}

void init_BLE()
{
    // Initialize the watchdog
    bool polling = true;

    // Initialize the LCD that will be printing stats
    init_LCD();

    // Initializes the Ultrasonic sensor
    init_ultrasonic();

    // Open and configure the port
    configPort();

    running = true;
    pthread_create(&BLE, NULL, BLEId, NULL);
    pthread_create(&ConnectionCheck, NULL, ConnectionCheckID, NULL);
    pthread_create(&HitWatchDog, NULL, HitWatchDogID, NULL);
    pthread_create(&LCD, NULL, LCDId, NULL);

    // Configure the LED
    runCommand("config-pin P8_08 gpio");
    runCommand("echo out > /sys/class/gpio/gpio67/direction");
    runCommand("echo 1 > /sys/class/gpio/gpio67/value"); // Turn on LED

    int previousPress = 0;
    while (polling) // Polling buttons
    {
        if (readbutton(BUTTON_VALUE_YELLOW_MODE) == 0)
        {
            if (previousPress == 0)
            {
                set_mode();
                previousPress = 1;
            }
        }
        else if (readbutton(BUTTON_VALUE_RED_POWER) == 0)
        {
            mode = 3;
            sleepForMs(1000);
            break;
        }
        if (readbutton(BUTTON_VALUE_YELLOW_MODE) == 1) // Debouncing button press
        {
            if (previousPress != 0)
            {
                previousPress = 0;
            }
        }
        sleepForMs(100);
    }
}

// Function cleans up all running threads, closes watchdog, clears the screen, and fress the controllerPacket
void cleanup_BLE()
{
    running = false;
    pthread_join(HitWatchDog, NULL);
    closeWatchdog(watchdog);
    pthread_join(ConnectionCheck, NULL);
    pthread_join(BLE, NULL);
    pthread_join(LCD, NULL);
    cleanup_ultrasonic();
    LCD_cursor_location(LINE2);
    clear_LCD_screen();
    runCommand("echo 0 > /sys/class/gpio/gpio67/value"); // Turn off LED
}