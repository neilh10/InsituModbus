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
byte modbusAddress = 0x07;  // The sensor's modbus address, or DeviceID
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
Insitu sensorLT500;
bool success;

int deviceId=0;

//Working variables
float waterDepthFt     = INSTU_MB_ERROR_RESULTS ;
float waterTempertureC = INSTU_MB_ERROR_RESULTS ;
float waterPressureB   = INSTU_MB_ERROR_RESULTS ;

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

    // Start up the modbus sensorLT500
    sensorLT500.begin(model, modbusAddress, &modbusSerial, DEREPin);

    // Turn on debugging
    //sensorLT500.setDebugStream(&Serial);

    sensorLT500.setDevicePoll(IMDP_DEPTH_TEMPERATURE );
    Serial.println(F("Insitu LT500 220309-1648"));
    delay(1000);
    if (sensorLT500.readDeviceIdFrame()) {

        Serial.print(F("DeviceId ("));
        deviceId=sensorLT500.getDeviceHwID();
        Serial.print(deviceId);
        Serial.print(F("): "));
        //As per Appendix B - device ID
        switch (deviceId) 
        {
            case 1:{Serial.print(F("LT500")); break;}
            case 2:{Serial.print(F("LT700")); break;}
            default:{Serial.print(F("unknown")); break;}
        }
        Serial.print(F(" on Addr :"));
        Serial.println(modbusAddress);

        Serial.print(F("Serial Number: "));
        Serial.println(sensorLT500.getSerialNumber());

        Serial.print(F("Firmware Version: "));
        Serial.println(sensorLT500.getDeviceFwVer());
    }

    Serial.print(F("Sensor warmup "));
    for (int i = 5; i > 0; i--)     // 4 second delay
    {
        Serial.print(i);
        delay (250);
        Serial.print(F("."));
        delay (250);
        Serial.print(F("."));
        delay (250);
        Serial.print(F("."));
        delay (250);
    }
    Serial.println("\n");

    //Output formating make so easy to import into csv file
    Serial.print(F(",Temp(°C)    "));
    //Serial.print(F("Pressure(bar),  "));
    //Serial.print(F("Depth (mWC)   ,"));
    Serial.print(F(",Depth (ft)"));
    Serial.println();

}


// ---------------------------------------------------------------------------
// Main loop function
// ---------------------------------------------------------------------------
void loop()
{
    sensorLT500.getLtReadings(waterDepthFt,waterTempertureC,waterPressureB );
 
    Serial.print(F(","));
    Serial.print(waterTempertureC);
    Serial.print(F(",      "));
    //Serial.print(waterPressureBar, 7);
    //Serial.print(F(",     "));
    //Serial.print(waterDepthM, 6);
    //Serial.print(F(",     "));
    Serial.print(waterDepthFt, 6);
    Serial.println();

    delay(5000);
}
