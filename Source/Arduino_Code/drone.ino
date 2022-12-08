#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>

// constants that don't change (the variables are read-only)
// PWM Pin Outputs
#define PinBackLMotor 6  // pin for the motor PWM signal
#define PinFrontLMotor 5 // pin for the motor PWM signal
#define PinBackRMotor 3  // pin for the motor PWM signal
#define PinFrontRMotor 2 // pin for the motor PWM signal

// Operation Modes
#define AutobalanceOff 0
#define AutobalanceOn 2
#define Ultrasonic 1
#define Shutdown 3

#define buffer_size 256
#define gScale (8.75e-3 * (PI / 180))

// PID gain constants
// Understanding how to tune these constants referenced from https://oscarliang.com/pid/
// const float K[] = {0.001, 0.0007, 0.0009}; // K[0], K[1], K[2]
const float K[] = {0.9, 0.0, 13.0}; // K[0], K[1], K[2]

// variables that can change
// IMU Variables
float gyro_axis[] = {0.0, 0.0, 0.0};  // x, y, z
float accel_axis[] = {0.0, 0.0, 0.0}; // x, y, z
float gyroDrift[] = {0.0, 0.0, 0.0};  // x, y, z
float Roll = 0.0, Pitch = 0.0;
long previousInterval = 0, previousTimeStamp = 0, currentTimeStamp = 0;

// PID Variables
float Pitch_Error = 0.0, Roll_Error = 0.0;
float Previous_Pitch_Error = 0.0, Previous_Roll_Error = 0.0;
float Pitch_Rate_Error = 0.0, Roll_Rate_Error = 0.0;
float Pitch_Cumulative_Error = 0.0, Roll_Cumulative_Error = 0.0;
long Pitch_PIDPreviousTimeStamp = 0, Pitch_PIDTimeInterval = 0, Pitch_PIDCurrentTimeStamp = 0;
long Roll_PIDPreviousTimeStamp = 0, Roll_PIDTimeInterval = 0, Roll_PIDCurrentTimeStamp = 0;

// Controlling Variables
float Target_Pitch = 0.0, Target_Roll = 0.0;
float PWM_Change_Pitch = 0.0, PWM_Change_Roll = 0.0;
int Maximum_angle = 45;

// PWM Variables
float PWMBR_Offset_Target = 0, PWMFR_Offset_Target = 0, PWMBL_Offset_Target = 0;
int Target_PWM = 0, JoystickRoll = 0, JoystickPitch = 0, JoystickYaw = 0;
int PWM_FrontL = 0, PWM_FrontR = 0, PWM_BackL = 0, PWM_BackR = 0;
int ramp_time = 10;
int motor_offset = 10;

// Mode Change
int mode = 0;
int prevMode = 0;

// Setup Function
void setup()
{
  Serial.begin(9600);
  // initialize IMU
  if (!IMU.begin())
  {
    // Serial.println("starting IMU failed!");

    while (1)
      ;
  }
  IMUGyroCalibration();

  // Serial.println("IMU started");
  pinMode(LED_BUILTIN, OUTPUT);

  // begin initialization
  if (!BLE.begin())
  {
    // Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1)
      ;
  }

  // Serial.println("Bluetooth® Low Energy Central - Peripheral Explorer");
  //  start scanning for peripherals
  BLE.scan();
}

// Loop Function
void loop()
{
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral)
  {
    // discovered a peripheral, print out address, local name, and advertised service
    // Serial.print("Found ");
    // Serial.print(peripheral.address());
    // Serial.print(" '");
    // Serial.print(peripheral.localName());
    // Serial.print("' ");
    // Serial.print(peripheral.advertisedServiceUuid());
    // Serial.println();

    // see if peripheral is a HMSoft
    if (peripheral.localName() == "HMSoft")
    {
      // stop scanning
      BLE.stopScan();

      explorerPeripheral(peripheral);

      // peripheral is still connected
      while (1)
      {
        // write hello world to the characteristic every 2 seconds
        if (peripheral.connected())
        {
          // If the mode is 2 which is Autobalance mode
          // packetRead will still be used to help change the target PWM
          if (mode == AutobalanceOn)
          {
            BalancingPID();
          }

          // When shutdown has been activated the motors will turn off and will exit the void loop until the arduino is plugged in again
          if (mode == Shutdown)
          {
            Land();
            // Serial.println("Done");
            // Serial.println("Disconnecting ...");
            peripheral.disconnect();
            digitalWrite(LED_BUILTIN, LOW);
            // Serial.println("Disconnected");
            exit(0);
          }

          //  read the value of the characteristic and print it out
          if (peripheral.characteristic("FFE1").valueUpdated())
          {
            byte buffer[buffer_size] = {0};
            peripheral.characteristic("FFE1").readValue(buffer, buffer_size);
            packetRead(buffer);
          }
        }
        else
        {
          // peripheral disconnected
          // Serial.println("Peripheral disconnected");

          // start scanning again
          BLE.scan();

          break;
        }
      }
    }
  }
}

// Function explores peripherals
void explorerPeripheral(BLEDevice peripheral)
{
  // connect to the peripheral
  // Serial.println("Connecting ...");

  if (peripheral.connect())
  {
    // Serial.println("Connected");
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    // Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  // Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes())
  {
    // Serial.println("Attributes discovered");
  }
  else
  {
    // Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // read and print device name of peripheral
  // Serial.println();
  // Serial.print("Device name: ");
  // Serial.println(peripheral.deviceName());
  // Serial.print("Appearance: 0x");
  // Serial.println(peripheral.appearance(), HEX);
  // Serial.println();

  // loop the services of the peripheral and explore each
  for (int i = 0; i < peripheral.serviceCount(); i++)
  {
    BLEService service = peripheral.service(i);

    exploreService(service);
  }

  // Serial.println();
}

// Function explores the service
void exploreService(BLEService service)
{
  // print the UUID of the service
  // Serial.print("Service ");
  // Serial.println(service.uuid());

  // loop the characteristics of the service and explore each
  for (int i = 0; i < service.characteristicCount(); i++)
  {
    BLECharacteristic characteristic = service.characteristic(i);

    exploreCharacteristic(characteristic);
  }
}

// Function explores the device characteristic
void exploreCharacteristic(BLECharacteristic characteristic)
{
  // print the UUID and properties of the characteristic
  // Serial.print("\tCharacteristic ");
  // Serial.print(characteristic.uuid());
  // Serial.print(", properties 0x");
  // Serial.print(characteristic.properties(), HEX);

  // check if the characteristic is readable
  if (characteristic.canRead())
  {
    // read the characteristic value
    characteristic.read();

    if (characteristic.valueLength() > 0)
    {
      // print out the value of the characteristic
      // Serial.print(", value 0x");
      printData(characteristic.value(), characteristic.valueLength());
    }
  }
  // Serial.println();

  // loop the descriptors of the characteristic and explore each
  for (int i = 0; i < characteristic.descriptorCount(); i++)
  {
    BLEDescriptor descriptor = characteristic.descriptor(i);

    exploreDescriptor(descriptor);
  }
}

// Function explores the device descriptor
void exploreDescriptor(BLEDescriptor descriptor)
{
  // print the UUID of the descriptor
  // Serial.print("\t\tDescriptor ");
  // Serial.print(descriptor.uuid());

  // read the descriptor value
  descriptor.read();

  // print out the value of the descriptor
  // Serial.print(", value 0x");
  printData(descriptor.value(), descriptor.valueLength());

  // Serial.println();
}

// Function prints char string data
void printData(const unsigned char data[], int length)
{
  for (int i = 0; i < length; i++)
  {
    unsigned char b = data[i];

    if (b < 16)
    {
      // Serial.print("0");
    }

    // Serial.print(b, HEX);
  }
}

void packetRead(byte buffer[])
{
  // convert the byte array to a string
  String packet = "";
  for (int i = 0; i < buffer_size; i++)
  {
    packet += (char)buffer[i];
  }
  // token the string separated by commas and store in an array of strings
  String tokens[5] = {"", "", "", "", ""};
  // use strok to tokenize the string
  char *token = strtok((char *)packet.c_str(), ",");
  int i = 0;
  while (token != NULL)
  {
    tokens[i] = token;
    token = strtok(NULL, ",");
    i++;
  }
  // print out the tokens on new lines
  for (int i = 0; i < 5; i++)
  {
    // Serial.println(tokens[i]);
  }
  // Serial.println("");

  // Check the mode status from the packet sent
  if ((tokens[0] == "Off") && (prevMode != AutobalanceOff))
  {
    // Serial.println("Autobalance Off");
    mode = AutobalanceOff;
    prevMode = mode;
  }
  else if ((tokens[0] == "Ultra") && (prevMode != Ultrasonic))
  {
    // Serial.println("Ultrasonic reading");
    mode = Ultrasonic;
    prevMode = mode;
  }
  else if ((tokens[0] == "On") && (prevMode != AutobalanceOn))
  {
    // Serial.println("Autobalance On");
    mode = AutobalanceOn;
    prevMode = mode;
  }
  else if (tokens[0] == "Shutdown")
  {
    // Serial.println("Shutting down");
    mode = Shutdown;
  }

  // Check the speed status from the packet sent
  if ((tokens[4] != "S_Center") && (mode != Shutdown) && ((mode == AutobalanceOff) || (mode == AutobalanceOn) || (mode == Ultrasonic))) // If the speed is being manipulated
  {
    // Serial.println("SPEEDING ACTIVATED");
    if ((mode == AutobalanceOff) || (mode == AutobalanceOn)) // If the mode is Autobalance on/off the value from the packet modifies the current PWM incrementally
    {
      int incrementPWM = tokens[4].toInt();
      int updatedPWM = Target_PWM + incrementPWM;
      updatedPWM = PWMSpeedCheck(updatedPWM); // Checks if the new incremented value has reach the limits
      Target_PWM = updatedPWM;                // Sets the new target PWM
    }
    else if (mode == Ultrasonic) // If the mode is ultrasonic the PWM is completely replaced by the PWM sent from the packet
    {
      int incrementPWM = tokens[4].toInt();
      Target_PWM = incrementPWM; // Set the new PWM as the ultrasonic values sent at already made within the range of 0 to 255
    }
  }

  // Check the pitch status from the packet sent
  if ((tokens[2] != "D_Center") && (mode != Shutdown) && (mode == AutobalanceOff)) // If the pitch is being manipulated
  {
    int modifyPitchPWM = tokens[2].toInt();
    JoystickPitch = modifyPitchPWM;
    // Serial.println(modifyPitchPWM);
  }
  else if (tokens[2] == "D_Center") // If the joystick pitch has returned to the center
  {
    JoystickPitch = 0;
  }

  // Check the roll status from the packet sent
  if ((tokens[1] != "D_Center") && (mode != Shutdown) && (mode == AutobalanceOff)) // If the roll is being manipulated
  {
    int modifyRollPWM = tokens[1].toInt();
    JoystickRoll = modifyRollPWM;
    // Serial.println(modifyRollPWM);
  }
  else if (tokens[2] == "D_Center") // If the joystick roll has returned to the center
  {
    JoystickRoll = 0;
  }

  // Check the Yaw status from the packet sent
  if ((tokens[3] != "D_Center") && (mode != Shutdown) && (mode == AutobalanceOff)) // If the yaw is being manipulated
  {
    int modifyYawPWM = tokens[3].toInt();
    JoystickYaw = modifyYawPWM;
    // Serial.println(modifyYawPWM);
  }
  else if (tokens[2] == "D_Center") // If the joystick roll has returned to the center
  {
    JoystickYaw = 0;
  }

  if (mode != AutobalanceOn) // Allow the pitch and roll controls to be written when not in balance mode
  {
    SpeedWrite();
  }
}

// Function checks if the speed manipulated by the joystick has reached past the limits of the PWM 0 to 255
int PWMSpeedCheck(int valToCheck)
{
  if (valToCheck > 255)
  {
    valToCheck = 255;
  }
  else if (valToCheck < 0)
  {
    valToCheck = 0;
  }
  return valToCheck;
}

// Function write PWM to the motors based on the speed, pitch, and roll
void SpeedWrite()
{
  PWM_BackL = (Target_PWM + JoystickRoll + JoystickPitch - JoystickYaw);
  PWM_FrontL = (Target_PWM + JoystickRoll - JoystickPitch + JoystickYaw);
  PWM_BackR = (Target_PWM - JoystickRoll + JoystickPitch + JoystickYaw);
  PWM_FrontR = (Target_PWM - JoystickRoll - JoystickPitch - JoystickYaw);
  if (Target_PWM != 0)
  {
    PWM_FrontL += motor_offset;
    PWM_BackL += motor_offset;
  }
  MotorPWMCheck(); // Check if the final PWM values sent to the drone are within range
  analogWrite(PinBackLMotor, PWM_BackL);
  analogWrite(PinFrontLMotor, PWM_FrontL);
  analogWrite(PinBackRMotor, PWM_BackR);
  analogWrite(PinFrontRMotor, PWM_FrontR);
}

void Land()
{ // slowly ramp down motor speed to land safely during shutdown mode
  // Serial.println("Landing");
  while (Target_PWM > 0)
  {
    Target_PWM--;
    analogWrite(PinBackLMotor, Target_PWM);
    analogWrite(PinFrontLMotor, Target_PWM);
    analogWrite(PinBackRMotor, Target_PWM);
    analogWrite(PinFrontRMotor, Target_PWM);
    delay(ramp_time);
    // Serial.println(Target_PWM);
  }
}

// Formulas to appreach pitch and roll calcuations referenced from owenewno https://github.com/owennewo/Arduino_LSM6DS3/blob/master/examples/RollPitchYaw/RollPitchYaw.ino
// Also from Joop Brokking https://www.youtube.com/watch?v=4BoIE8YQwM8
void CalculatePitchRoll()
{
  if (GrabIMUValues())
  {
    currentTimeStamp = micros();
    previousInterval = currentTimeStamp - previousTimeStamp;
    previousTimeStamp = currentTimeStamp;
    float previousFrequency = 0;
    float Pitch_acceleration = 0, Roll_acceleration = 0;

    previousFrequency = (float)1000000.0 / previousInterval;
    Pitch_acceleration = ((atan2(-accel_axis[0], sqrt((accel_axis[1] * accel_axis[1]) + (accel_axis[2] * accel_axis[2]))) * 180) / PI);
    Roll_acceleration = ((atan2(accel_axis[1], accel_axis[2]) * 180) / PI);

    // Correct the initial reading from the gyro by removing the drift calculated from the initialization
    Pitch = Pitch + ((gyro_axis[0] - gyroDrift[0]) / previousFrequency);
    Roll = Roll + ((gyro_axis[1] - gyroDrift[1]) / previousFrequency);

    // Removes the drift that may affect the gyro readout values
    Pitch = ((0.98 * Pitch) + (0.02 * Pitch_acceleration));
    Roll = ((0.98 * Roll) + (0.02 * Roll_acceleration));
  }
}

// IMU Gyroscope drift callibration
void IMUGyroCalibration()
{
  int sampleSize = 0;
  int initializationStartTime = 0;
  int timeStamp = 0;
  int numberRan = 0;
  int done = 0;

  initializationStartTime = millis();
  timeStamp = millis();
  while (timeStamp < (initializationStartTime + 500))
  {
    if (GrabIMUValues() == true)
    {
      gyroDrift[0] += gyro_axis[0];
      gyroDrift[1] += gyro_axis[1];
      gyroDrift[2] += gyro_axis[2];
      sampleSize = sampleSize + 1;
    }
    timeStamp = millis();
    done = 1;
  }
  if (done == 1 && sampleSize > 0)
  {
    gyroDrift[0] = gyroDrift[0] / sampleSize;
    gyroDrift[1] = gyroDrift[1] / sampleSize;
    gyroDrift[2] = gyroDrift[2] / sampleSize;
  }
  // Used to have the read values leveled out at the start
  while (numberRan < 4500)
  {
    CalculatePitchRoll();
    numberRan++;
  }
}

// Function reads the IMU gyroscope and acclerometer values
bool GrabIMUValues()
{
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(gyro_axis[0], gyro_axis[1], gyro_axis[2]);
    IMU.readAcceleration(accel_axis[0], accel_axis[1], accel_axis[2]);
    gyro_axis[0] = -gyro_axis[0] * gScale;
    gyro_axis[1] = -gyro_axis[1] * gScale;
    gyro_axis[2] = -gyro_axis[2] * gScale;
    accel_axis[0] = -accel_axis[0];
    accel_axis[1] = -accel_axis[1];
    accel_axis[2] = -accel_axis[2];
    return true;
  }
  return false;
}

// Ensure that the PWM being sent to the motors will not be past the valid range of PWM signals (0 - 255)
void MotorPWMCheck()
{
  if (PWM_BackL < 0)
    PWM_BackL = 0;
  if (PWM_FrontL < 0)
    PWM_FrontL = 0;
  if (PWM_BackR < 0)
    PWM_BackR = 0;
  if (PWM_FrontR < 0)
    PWM_FrontR = 0;
  if (PWM_BackL > 255)
    PWM_BackL = 255;
  if (PWM_FrontL > 255)
    PWM_FrontL = 255;
  if (PWM_BackR > 255)
    PWM_BackR = 255;
  if (PWM_FrontR > 255)
    PWM_FrontR = 255;
}

// Function attemps to balance the drone as it is hovering at a user set target PWM
void BalancingPID()
{
  // Grab values again
  CalculatePitchRoll();

  // Pitch
  Pitch_PIDCurrentTimeStamp = millis();
  Pitch_PIDTimeInterval = Pitch_PIDCurrentTimeStamp - Pitch_PIDPreviousTimeStamp;
  Pitch_Error = Target_Pitch - Pitch;
  Pitch_Cumulative_Error = Pitch_Cumulative_Error + (Pitch_Error * Pitch_PIDTimeInterval);
  Pitch_Rate_Error = (Pitch_Error - Previous_Pitch_Error) / Pitch_PIDTimeInterval;
  Previous_Pitch_Error = Pitch_Error;
  Pitch_PIDPreviousTimeStamp = Pitch_PIDCurrentTimeStamp;
  PWM_Change_Pitch = (K[0] * Pitch_Error) + (K[1] * Pitch_Cumulative_Error) + (K[2] * Pitch_Rate_Error);

  // Roll
  Roll_PIDCurrentTimeStamp = millis();
  Roll_PIDTimeInterval = Roll_PIDCurrentTimeStamp - Roll_PIDPreviousTimeStamp;
  Roll_Error = Target_Roll - Roll;
  Roll_Cumulative_Error = Roll_Cumulative_Error + (Roll_Error * Roll_PIDTimeInterval);
  Roll_Rate_Error = (Roll_Error - Previous_Roll_Error) / Roll_PIDTimeInterval;
  Previous_Roll_Error = Roll_Error;
  Roll_PIDPreviousTimeStamp = Roll_PIDCurrentTimeStamp;
  PWM_Change_Roll = (K[0] * Roll_Error) + (K[1] * Roll_Cumulative_Error) + (K[2] * Roll_Rate_Error);

  // Applying all PWM Changes From Each Factor
  PWM_FrontL = Target_PWM - PWM_Change_Pitch - PWM_Change_Roll;
  PWM_FrontR = Target_PWM - PWM_Change_Pitch + PWM_Change_Roll;
  PWM_BackL = Target_PWM + PWM_Change_Pitch - PWM_Change_Roll;
  PWM_BackR = Target_PWM + PWM_Change_Pitch + PWM_Change_Roll;

  MotorPWMCheck();
  // WRITING TO MOTORS
  analogWrite(PinBackLMotor, PWM_BackL);
  analogWrite(PinFrontLMotor, PWM_FrontL);
  analogWrite(PinBackRMotor, PWM_BackR);
  analogWrite(PinFrontRMotor, PWM_FrontR);
}
