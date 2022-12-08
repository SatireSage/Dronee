////// Drone BLE Controller Application //////
//  Authors:
//
//  Controller connects and sends control data in packets through BLE to be interpreted by the drone 
//
//  LCD DISPLAY:  
//  Displays to the user:
//          -   Current control mode
//          -   Current rate of PWM speed change for drone motors (Default and Autobalance Mode)
//          -   Current speed of the drone motors (Ultrasonic Mode)
//
//  CONTROLS:
//  Red Button: Shuts down powers off the motors for the drone as well as shutting down the controller program
//  Yellow Button: Switches through the following control modes for the drone
//     Default Mode:
//          -   Left Joystick: Controls the PWM speed of the drone from -10 to 10 PWM
//                  Up/Down: Linearly increase or decreases the PWM value by a rate between -10 to 10
//                  Left/Right: Linearly increase or decrease the Yaw by a value between -28 to 28
//
//          -   Right Joystick: Controls the Pitch and Roll of the drone
//                  Up/Down: Linearly increase or decrease the Pitch by a value between -42 to 42
//                  Left/Right: Linearly increase or decrease the Roll by a value between -28 to 28
//
//     Ultrasonic Mode:
//          -   Ultrasonic Sensor: Controls the PWM speed of the drone from 0 to 255 PWM
//
//     Autobalance Mode:
//          -   Left Joystick: Controls the PWM speed of the drone from -10 to 10 PWM
//                  Up/Down: Linearly increase or decreases the PWM value by a rate between -10 to 10


#include "BLE_controller.h"

int main()
{
    // Initilize the BLE which initializes the LCD to display statistics and buttons 
    init_BLE();
    
    // During this time, until the shutoff button has been pressed the program will remain running

    // Cleanup of threads and memory
    cleanup_BLE();
    return 0;
}
