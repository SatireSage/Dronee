#include <ArduinoBLE.h>

#define buffer_size 256

// constants that don't change (the variables are read-only)
const int PWMPin1 = 6; // pin for the motor PWM signal
const int PWMPin2 = 5; // pin for the motor PWM signal
const int PWMPin3 = 3; // pin for the motor PWM signal
const int PWMPin4 = 2; // pin for the motor PWM signal
// const int but_pin = 3;     // pin for start/stop buttongain
const int ramp_time = 1; // delay for motor ramp up/ramp down

// variables that can change
int but_status = 1; // button status
int PWM_offset = 0; // offset for PWM signal (between 0-255), roughly the value that makes the drone hover
int PWM_signal = 0; // PWM value

void setup()
{
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    while (!Serial)
        ;

    // begin initialization
    if (!BLE.begin())
    {
        Serial.println("starting Bluetooth® Low Energy module failed!");

        while (1)
            ;
    }

    Serial.println("Bluetooth® Low Energy Central - Peripheral Explorer");

    // start scanning for peripherals
    BLE.scan();
}

void loop()
{
    // check if a peripheral has been discovered
    BLEDevice peripheral = BLE.available();

    if (peripheral)
    {
        // discovered a peripheral, print out address, local name, and advertised service
        Serial.print("Found ");
        Serial.print(peripheral.address());
        Serial.print(" '");
        Serial.print(peripheral.localName());
        Serial.print("' ");
        Serial.print(peripheral.advertisedServiceUuid());
        Serial.println();

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
                    // read the value of the characteristic and print it out
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
                    Serial.println("Peripheral disconnected");

                    // start scanning again
                    BLE.scan();

                    break;
                }
            }
        }
    }
}

void explorerPeripheral(BLEDevice peripheral)
{
    // connect to the peripheral
    Serial.println("Connecting ...");

    if (peripheral.connect())
    {
        Serial.println("Connected");
    }
    else
    {
        Serial.println("Failed to connect!");
        return;
    }

    // discover peripheral attributes
    Serial.println("Discovering attributes ...");
    if (peripheral.discoverAttributes())
    {
        Serial.println("Attributes discovered");
    }
    else
    {
        Serial.println("Attribute discovery failed!");
        peripheral.disconnect();
        return;
    }

    // read and print device name of peripheral
    Serial.println();
    Serial.print("Device name: ");
    Serial.println(peripheral.deviceName());
    Serial.print("Appearance: 0x");
    Serial.println(peripheral.appearance(), HEX);
    Serial.println();

    // loop the services of the peripheral and explore each
    for (int i = 0; i < peripheral.serviceCount(); i++)
    {
        BLEService service = peripheral.service(i);

        exploreService(service);
    }

    Serial.println();

    // we are done exploring, disconnect
    // Serial.println("Disconnecting ...");
    // peripheral.disconnect();
    // Serial.println("Disconnected");
}

void exploreService(BLEService service)
{
    // print the UUID of the service
    Serial.print("Service ");
    Serial.println(service.uuid());

    // loop the characteristics of the service and explore each
    for (int i = 0; i < service.characteristicCount(); i++)
    {
        BLECharacteristic characteristic = service.characteristic(i);

        exploreCharacteristic(characteristic);
    }
}

void exploreCharacteristic(BLECharacteristic characteristic)
{
    // print the UUID and properties of the characteristic
    Serial.print("\tCharacteristic ");
    Serial.print(characteristic.uuid());
    Serial.print(", properties 0x");
    Serial.print(characteristic.properties(), HEX);

    // check if the characteristic is readable
    if (characteristic.canRead())
    {
        // read the characteristic value
        characteristic.read();

        if (characteristic.valueLength() > 0)
        {
            // print out the value of the characteristic
            Serial.print(", value 0x");
            printData(characteristic.value(), characteristic.valueLength());
        }
    }
    Serial.println();

    // loop the descriptors of the characteristic and explore each
    for (int i = 0; i < characteristic.descriptorCount(); i++)
    {
        BLEDescriptor descriptor = characteristic.descriptor(i);

        exploreDescriptor(descriptor);
    }
}

void exploreDescriptor(BLEDescriptor descriptor)
{
    // print the UUID of the descriptor
    Serial.print("\t\tDescriptor ");
    Serial.print(descriptor.uuid());

    // read the descriptor value
    descriptor.read();

    // print out the value of the descriptor
    Serial.print(", value 0x");
    printData(descriptor.value(), descriptor.valueLength());

    Serial.println();
}

void printData(const unsigned char data[], int length)
{
    for (int i = 0; i < length; i++)
    {
        unsigned char b = data[i];

        if (b < 16)
        {
            Serial.print("0");
        }

        Serial.print(b, HEX);
    }
}

void Liftoff()
{ // slowly ramp up motor speed to lift off
    Serial.println("Liftoff ");
    while (PWM_signal < PWM_offset)
    { // slowly ramp up motor speed for smooth takeoff
        analogWrite(PWMPin1, PWM_signal);
        analogWrite(PWMPin2, PWM_signal);
        analogWrite(PWMPin3, PWM_signal);
        analogWrite(PWMPin4, PWM_signal);
        PWM_signal++;
        delay(ramp_time);
    }
}

void Land()
{ // slowly ramp down motor speed to land safely
    Serial.println("Landing ");
    while (PWM_signal > 0)
    {
        PWM_signal--;
        analogWrite(PWMPin1, PWM_signal);
        analogWrite(PWMPin2, PWM_signal);
        analogWrite(PWMPin3, PWM_signal);
        analogWrite(PWMPin4, PWM_signal);
        delay(ramp_time);
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
    String tokens[3] = {"", "", ""};
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
    for (int i = 0; i < 3; i++)
    {
        Serial.println(tokens[i]);
    }

    if (tokens[2] != "S_Center")
    {
        int value = tokens[2].toInt();
        int new_PWM_offset = value + PWM_offset;
        if (new_PWM_offset > 255)
        {
            new_PWM_offset = 255;
        }
        else if (new_PWM_offset < 0)
        {
            new_PWM_offset = 0;
        }
        if (new_PWM_offset < PWM_offset)
        {
            PWM_offset = new_PWM_offset;
            Land();
        }
        else if (new_PWM_offset > PWM_offset)
        {
            PWM_offset = new_PWM_offset;
            Liftoff();
        }
    }
}