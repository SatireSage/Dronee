# **【Ｄｒｏｎｅｅ】**

Authors:
[Sahaj Singh](https://github.com/SatireSage),
[Bryce Leung](https://github.com/Bryce-Leung),
[Andrew Speers](https://github.com/AndrewSpeers8),
[Sukha Lee](https://github.com/SukhaL)

# **Acknowledgment:**

We would like to thank lab technicians, Fredd and David. They were a great help to our project and gave us numerous tips and necessary parts. We would also like to thank Bryce's parents for driving Bryce to Lee's electronics to purchase parts, especially during the icy cold winter road. Furthermore, Dr. Brian for giving us the opportunity to take ENSC 351 this term. We wouldn't have been able to complete the course without Dr. Brian.

# **Dronee Overview:**

<img style="centre" width="400" height="350" alt="Drone" src="https://user-images.githubusercontent.com/49692422/206334491-c3aea398-3831-4ca6-84c7-75cddc995840.jpeg">

A drone and controller are the final product of this project. This project is coded in C and uses BeagleBone Green and Arduino Nano components that are soldered together with wires or onto the pins directly.

<img style="centre" width="350" height="350" alt="Frame_Drone" src="https://user-images.githubusercontent.com/49692422/206334472-d9877bae-f0b5-4bc1-b84b-65c90aa9fc4a.jpeg">

# Contents

- [Introduction](#Introduction)
- [BLE Module](#BLE-Module)
- [LCD Display](#LCD-Display)
- [Joystick & Buttons](#Joystick-&-Buttons)
- [Ultrasonic Sensor](#Ultrasonic-Sensor)
- [LED Connection Status Light](#LED-Connection-Status-Light)
- [Power Switch](#Power-Switch)
- [Watchdog](#Watchdog)
- [Systemd](#Systemd)

# **Dronee Documentations:**

### **Introduction**

Dronee uses a BeagleBone Green as the controller and an Arduino Nano 33 IOT as the drone. A BlueTooth module is attached to the BeagleBone which allows it to send packets between the BeagleBone Green (controller) to the Arduino (drone). Thus allowing the user to control the drone wirelessly with no cable connected between the two. There are three modes to control the drone. Default mode which allows the use of the joysticks to control speed, pitch, yaw, and roll.

Ultrasonic mode which allows the use of an ultrasonic sensor to control the speed of all the motors to go up and down. As well as Autobalance mode which has the drone attempt to self stabilize with a given speed given from the controller. The user can select the desired mode using the yellow button on the right which cycles through the modes upon pressing. MOSFETs are used to control each motor and the frame is 3D printed.

<img style="centre" width="600" height="500" alt="Controller" src="https://user-images.githubusercontent.com/49692422/206335368-2d841cb6-54fb-4b96-9995-91b3e0d05975.jpeg">

### **BLE Module**

<img width="171" alt="Screenshot 2022-12-07 175724" src="https://user-images.githubusercontent.com/49692422/206337788-31744c2c-20c5-4a2c-9b4c-d6ca63e7f403.png">

We use a UART BLE module to send encoded packets that contain information such as the PWM values for the motors for the Arduino module to process. The beaglebone is setup as a pripheral bluetooth device since the UART module can not write/read to specific BLE characteristics, hence the Arduino is setup as a central bluetooth device since it can read the base BLE characteristic where the UART module writes the encoded packets.

### **LCD Display**

<img width="425" alt="Screenshot 2022-12-07 173639" src="https://user-images.githubusercontent.com/49692422/206334649-110c8387-ed5e-4e6b-ba92-db097fb8c6eb.png">
<img width="425" alt="Screenshot 2022-12-07 173702" src="https://user-images.githubusercontent.com/49692422/206334656-15f68ca2-e63b-4266-a257-75471378a60a.png">

The LCD Display shows important and useful information to the user. It displays information such as the current mode of control, rate of which the PWM is being manipulated (Default and Autobalance Mode), or the current PWM (Ultrasonic Mode). The display also allows the user to see whether the controller is in the process of starting up or shutting down.

### **Joystick & Buttons**

<img width="562" alt="Screenshot 2022-12-07 173911" src="https://user-images.githubusercontent.com/49692422/206334812-290f7690-f34c-4ca4-bd11-c44836a973f3.png">

There are two joysticks and buttons on the controller for switching control mode, sending movement commands, and shutting off the controller and drone.

The left joystick controls the PWM and yaw of the drone. When the user moves the joystick up or down, the PWM value linearly changes between -10 to 10. When the user moves the joystick left or right, it linearly varies the yaw value between -28 to 28 PWM. Furthermore, the right joystick controls the Pitch and Roll of the drone. When the user moves the right joystick up or down, it linearly changes the Pitch between -42 to 42 PWM. As well as when the user moves the joystick left or right, it linearly varies the Roll between -28 to 28 PWM. Whenever the joysticks are centered, the drone no longer applies the commands previously given by the controller's joysticks.

The left red button enables the user to power off the drone and end the controller program. The right yellow button changes the control modes previously stated above. There are a total of three modes: default (joystick), ultrasonic sensor, and autobalance. Read the ultrasonic sensor section below for details on controlling the drone with the sensor.

### **Ultrasonic Sensor**

<img width="155" alt="Screenshot 2022-12-07 174120" src="https://user-images.githubusercontent.com/49692422/206335064-1ef46c0d-0603-46a4-a46a-32dc7d9571a4.png">

Read the "Set up guide for HC-SR04 Ultrasonic sensor" in the Source folder for setup guide and troubleshooting.

The ultrasonic sensor is applied to the project as a feature that allows the user to control the drone's height using their hand. With the distance readings from the ultrasonic sensor, the drone will raise or lower. The Ultrasonic Sensor is set as a controller only if the user selects this mode (by pressing the yellow button on the right side of the controller). To make the drone move upwards, the user must have their hand close to the sensor. To decrease the height of the drone, the user must move their hand further away from the sensor; fully stopping the drone by moving their hand further than 30 cm away from the sensor or out of the sensor's path.

### **LED Connection Status Light**

<img width="116" alt="Screenshot 2022-12-07 174207" src="https://user-images.githubusercontent.com/49692422/206335287-0c6e473f-802e-45c4-8eec-b74182f88719.png">

To provide an easy way to identify if the controller has connected to the drone, the LED is used to convey its status. When the LED is on it shows that the controller has successfully connected to the drone. When the LED is off it shows that the connection has been lost between the controller and drone. Thus allowing the user to know when their input commands are no longer being sent to the drone.

### **Power Switch**

<img width="125" alt="Screenshot 2022-12-07 174259" src="https://user-images.githubusercontent.com/49692422/206335266-3e278e3a-dd66-4a5e-b60c-1ff3445b46b2.png">

The power switch provides a physical way to no longer supply battery power to the controller. When the user is no longer using the controller and has shut down the controller and drone with the button, the power switch can be flipped off, no longer providing a source of power. Resulting in the complete shutoff of the controller hardware.

### **Watchdog**

Watchdog is utilized with two threads initialized in the controller program called: " **connectioncheck**" and " **Hitwatchdog**". When initialized and running, hitwatchdog starts the watchdog timer of 20 seconds. Based on the disconnect flag that is modified by connectioncheck thread, it'll hit the watchdog every 15 sec when there is no disconnection detected. This prevents the watchdog from restarting the processor as the timer will not reach the end of the 20 second timer duration. The way disconnect is detected is by sending an AT command through the connectioncheck thread. If the drone is connected, the packet won't return the code 9999 thus confirming it is not disconnected. If it does return a flag of 9999, the drone is determined to be disconnected and changes the disconnect flag to 1. With this flag set to 1, it changes the behaviour of the hitwatchdog thread. When this occurs, it'll not hit the watchdog and instead sleeps for 23s which is 3 seconds greater than the watchdog timer. This eventually causes the watchdog to restart the BeagleBone.

If the user is running the drone and when the user presses the shutoff button, it shuts off the watchdog threads and closes the watchdog. By closing the watchdog it stops the watchdog from performing an unexpected reboot after the program has ended.

### **Systemd**

To enable the controller to fully utilize the watchdog's ability to reboot the BeagleBone Green, systemd enables the BeagleBone Green to once again run the controller program upon restart. With systemd, this provides an easy way for the user to use the controller again without having to connect to the controller to manually run the program.
