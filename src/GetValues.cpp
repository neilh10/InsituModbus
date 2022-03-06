/*****************************************************************************
GetValues.ino

Modified by Neil Hancock, from KellerhModbus/GetValues.ino
2020-Mar

For testing individual functions in InsituModbus library

*****************************************************************************/

// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <AltSoftSerial.h>
#include <SensorModbusMaster.h>
#include "InsituModbus.h"


// ---------------------------------------------------------------------------
// Set up the sensor specific information
//   ie, pin locations, addresses, calibrations and related settings
// ---------------------------------------------------------------------------

// Define the sensor type
InsituModel model = Leveltroll_InsituModel;

// Define the sensor's modbus address
byte modbusAddress = 0x01;  // The sensor's modbus address, or SlaveID
// Insitu defines the following:
//   Address 0 is reserved for broadcasting.
//   Addresses 1 (default) ...249 can be used for bus mode.
//   Address 250 is transparent and reserved for non-bus mode. Every device can be contacted with this address.
//   Addresses 251...255 are reserved for subsequent developments.

// Define pin number variables
const int PwrPin = 22;  // The pin sending power to the sensor *AND* RS485 adapter
const int DEREPin = -1;   // The pin controlling Recieve Enable and Driver Enable
                          // on the RS485 adapter, if applicable (else, -1)
                          // Setting HIGH enables the driver (arduino) to send text
                          // Setting LOW enables the receiver (sensor) to send text

// Construct software serial object for Modbus
AltSoftSerial modbusSerial;  // On Mayfly, requires connection D5 & D6

// Construct the modbus instance
modbusMaster modbus;

// Construct the Insitu modbus instance
Insitu sensor;
bool success;


// ---------------------------------------------------------------------------
// Working Functions
// ---------------------------------------------------------------------------

// Give values to variables;
// byte modbusSlaveID = modbusAddress;
// byte _slaveID = modbusSlaveID;


// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------
void setup()
{

    pinMode(PwrPin, OUTPUT);
    digitalWrite(PwrPin, HIGH);

    if (DEREPin > 0) pinMode(DEREPin, OUTPUT);

    Serial.begin(115200);  // Main serial port for debugging via USB Serial Monitor
    modbusSerial.begin(9600);  // The modbus serial stream - Baud rate MUST be 9600.

    // Start up the modbus sensor
    sensor.begin(model, modbusAddress, &modbusSerial, DEREPin);

    // Turn on debugging
    sensor.setDebugStream(&Serial);

    // Start up note
    Serial.println("Insitu LT500 (or other Series 30, Class 5, Group 20 sensor)");

    Serial.println("Waiting for sensor and adapter to be ready.");
    delay(500);

    //Serial.print("Device Aaddress, as integer: ");
    Serial.print("Device slaveId, as integer: ");
    //while (1) 
    {
        Serial.println(sensor.getSlaveID());
        //delay(1000);
    }

    Serial.print("Device slaveId, as frame: ");
    while (1) 
    {
        Serial.println(sensor.getSlaveIDFrame());
        delay(5000);
    }//*/
    Serial.print("Serial Number: ");
    while (1) 
    {
        Serial.println(sensor.getSerialNumber());
        delay(1000);
    }

    Serial.println("Starting sensor measurements");

    Serial.println("Allowing sensor to stabilize..");
    for (int i = 5; i > 0; i--)     // 4 second delay
    {
        Serial.print(i);
        delay (250);
        Serial.print(".");
        delay (250);
        Serial.print(".");
        delay (250);
        Serial.print(".");
        delay (250);
    }
    Serial.println("\n");

    Serial.print("Temp(°C)  ");
    Serial.print("Pressure(bar)  ");
    Serial.print("Depth (mWC)");
    Serial.println();

}

// Initialize variables
float waterPressureBar = -9999.0;
float waterTempertureC = -9999.0;
float waterDepthM = -9999.0;

// ---------------------------------------------------------------------------
// Main loop function
// ---------------------------------------------------------------------------
void loop()
{
    sensor.getValues(waterPressureBar, waterTempertureC);
    waterDepthM = sensor.calcWaterDepthM(waterPressureBar, waterTempertureC);  // float calcWaterDepthM(float waterPressureBar, float waterTempertureC)

    Serial.print(waterTempertureC);
    Serial.print("      ");
    Serial.print(waterPressureBar, 7);
    Serial.print("      ");
    Serial.print(waterDepthM, 6);
    Serial.println();

    delay(5000);

}
