/*****************************************************************************
GetValues.ino

Modified by Neil Hancock
2022-Mar

For testing individual functions in InsituModbus library
on a Mayfly with Modbus Wingboard (knh002rev7)
from KellerhModbus/GetValues.ino
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

// Define the sensor type for Modbus processing
InsituModel model = Leveltroll_InsituModel;

// Define the sensor's modbus address
byte modbusAddress = 0x01;  // The sensor's modbus address, or DeviceID
// In-situ defines the following:
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

int deviceId=0;

// ---------------------------------------------------------------------------
// Working Functions
// ---------------------------------------------------------------------------

// Give values to variables;
// byte modbusDeviceID = modbusAddress;
// byte _DeviceID = modbusDeviceID;


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
    //sensor.setDebugStream(&Serial);

    // Start up note
    Serial.println("Insitu LT500 220308-1443");

    delay(500);

    //Serial.print("Device Aaddress, as integer: ");
    Serial.print("DeviceId (");
    //while (1) 
    {
        deviceId=sensor.getDeviceID();
        Serial.print(deviceId);
        Serial.print("): ");
        //As per Appendix B - device ID
        switch (deviceId) {
            case 1:{Serial.println("LT500"); break;}
            case 2:{Serial.println("LT700"); break;}
            default:{Serial.println("unknown"); break;}
        }
        //delay(1000);
    }

    //Serial.println(sensor.getSlaveID());

    Serial.print("Serial Number: ");
    Serial.println(sensor.getSerialNumber());

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

    Serial.print(",Temp(°C),  ");
    Serial.print("Pressure(bar),  ");
    Serial.print("Depth (mWC)   ,");
    Serial.print("Depth (ft),");
    Serial.println();

}

// Initialize variables
float waterPressureBar = INSTU_MB_ERROR_RESULTS ;
float waterTempertureC = INSTU_MB_ERROR_RESULTS ;
float waterDepth1      = INSTU_MB_ERROR_RESULTS ;
float waterDepthM      = INSTU_MB_ERROR_RESULTS ;

// ---------------------------------------------------------------------------
// Main loop function
// ---------------------------------------------------------------------------
void loop()
{
    sensor.getValues(waterPressureBar, waterTempertureC,waterDepth1 );
    waterDepthM = sensor.calcWaterDepthM(waterPressureBar, waterTempertureC);  // float calcWaterDepthM(float waterPressureBar, float waterTempertureC)

    Serial.print(",");
    Serial.print(waterTempertureC);
    Serial.print(",      ");
    Serial.print(waterPressureBar, 7);
    Serial.print(",     ");
    Serial.print(waterDepthM, 6);
    Serial.print(",     ");
    Serial.print(waterDepth1, 6);
    Serial.println();

    delay(5000);

}
