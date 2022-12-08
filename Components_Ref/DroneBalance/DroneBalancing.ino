#include <Arduino_LSM6DS3.h>

// Figure out the correct offsets for the pwm to have stable initial takeoff
// constants that don't change (the variables are read-only)
// PWM Pin Outputs
const int PinBackLMotor = 6;  // pin for the motor PWM signal
const int PinFrontLMotor = 5; // pin for the motor PWM signal
const int PinBackRMotor = 3;  // pin for the motor PWM signal
const int PinFrontRMotor = 2; // pin for the motor PWM signal

// PID gain constants
// Understanding how to tune these constants referenced from https://oscarliang.com/pid/
// const float K[] = {0.002, 0.0015, 0.0015}; // K[0], K[1], K[2]
 const float K[] = {0.001, 0.0007, 0.0009}; // K[0], K[1], K[2]
// const float K[] = {0.5, 0.05, 0.05}; // K[0], K[1], K[2]
//const float K[] = {0.0024, 0.0015, 0.47}; // K[0], K[1], K[2]
const float gScale = 8.75e-3 * (PI / 180);

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
int Takeoff_PWM = 160;
float PWMBR_Offset_Target = 0, PWMFR_Offset_Target = 0, PWMBL_Offset_Target = 0;
int Target_PWM = 0;
int PWM_FrontL = 0, PWM_FrontR = 0, PWM_BackL = 0, PWM_BackR = 0;

int i = 0;

void TakeOff()
{
    int PWM = 0;
    double PWMBRSet = 0;
    double PWMFRSet = 0;
    double PWMBLSet = 0;
    while (PWM < Takeoff_PWM)
    {
        BalancingPID();
        PWM += 1;
        Target_PWM = Takeoff_PWM;
        delay(5);
    }
    Target_PWM = Takeoff_PWM;
}

void setup()
{                       // setup code that only runs once
    Serial.begin(9600); // initialize serial communication, use for debugging if needed. Select Tools --> Serial Monitor to open window.
    IMU.begin();
    delay(1000);
    IMUGyroCalibration();
    previousTimeStamp = micros();
    delay(5000);
    TakeOff();
}

void loop()
{
    BalancingPID();
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

// Ensure that the PWM being sent to the motors will not be past the valid range of PWM signals (0 - Takeoff_PWM)
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
