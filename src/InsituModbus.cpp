/*
InsituModbus.cpp

Written by Neil Hancock

Tested with 
- a Insitu Series Level Troll series transducers
- https://in-situ.com/us/pub/media/support/documents/Modbus_Manual.pdf

*/

#include "InsituModbus.h"


//----------------------------------------------------------------------------
//                          PUBLIC SENSOR FUNCTIONS
//----------------------------------------------------------------------------


// This function sets up the communication
// It should be run during the arduino "setup" function.
// The "stream" device must be initialized and begun prior to running this.
bool Insitu::begin(InsituModel model,byte modbusDeviceID, Stream *stream, int enablePin)
{
    // Give values to variables;
    _deviceID = modbusDeviceID;
    _model = model;
    // Start up the modbus instance
    bool success = modbus.begin(modbusDeviceID, stream, enablePin);
    return success;
}
bool Insitu::begin(InsituModel model,byte modbusDeviceID, Stream &stream, int enablePin)
{return begin(model,modbusDeviceID, &stream, enablePin);}


// This gets the modbus Device ID.
// 2.3.1.8
byte Insitu::getDeviceID(void) //njh tested, gets something
{
    //2.3.1.8 Report (Slave) Device ID  special register 
    return modbus.byteFromRegister(MODBUS_READ_HREG, INSTU_MB_DEVICE_ID_REG, 3); 
    //return modbus.byteFromRegister(0x03, 0x11, 0); // byte byteFromRegister(byte regType, int regNum, int byteNum)
}

byte Insitu::getSlaveID(void)
{
    return getDeviceID();
}

#if 0
byte Insitu::getDeviceIDFrame(void) 
{
    //2.3.1.8 Report Device ID
    /*for (int i=0;i<0x11;i++) {
        modbus.byteFromRegister(0x03, i, 3); 
    }*/
    modbus.byteFromRegister(0x03, 0, 3);
    modbus.uint16FromRegister(0x03, 1);  
    modbus.byteFromRegister(0x03, 3,3);
    modbus.uint32FromRegister(0x03, 4);  //Time
    //modbus.uint32FromRegister(0x03, 8);  
    modbus.getRegisters(0x03,0,8);
    return 0;//modbus.byteFromRegister(0x03, 0x11, 3); 
    //return modbus.byteFromRegister(0x03, 0x11, 0); // byte byteFromRegister(byte regType, int regNum, int byteNum)
}
#endif //0

// This sets a new modbus device ID
// NOTE: NOT TESTED
// bool Insitu:::setDeviceID(byte newDeviceID)
// {
//     return modbus.byteToRegister(0x020D, 2, newDeviceID); //bool byteToRegister(int regNum, int byteNum, byte value, bool forceMultiple=false);
// }

/*byte Insitu::getDeviceID(void) //njh tested, gets something
{
    //#define IL_DEVICE_LEN 5
    //byte command[IL_DEVICE_LEN ];
    //comamnd 
    //sendCommand(byte command[], int commandLength);

    //2.3.1.8 Report Device ID
    return modbus.byteFromRegister(0x03, 0x9001, 3); 
    //return modbus.byteFromRegister(0x03, 0x11, 0); // byte byteFromRegister(byte regType, int regNum, int byteNum)
} */

// This gets the instrument serial number as a 16-bit unsigned integer (as specified by Insitu)
// The Serial number is in holding registers 0x2 or 9002 and occupies 2 registers (uint16)
long Insitu::getSerialNumber(void) //Nh Gets something 
{
    return modbus.uint16FromRegister(MODBUS_READ_HREG, INSTU_MB_DEVICE_SN_REG);  //Sect 7 Common registers
}


// This gets the hardware and software version of the sensor
// This data begins in holding register 0x020E (??) and continues for 2 registers
// bool Insitu::getVersion(float &ClassGroup, float &YearWeek)
// {
//     // Parse into version numbers, as a string "Class.Group-Year:Week"
//     // These aren't actually little endian responses.
//     // The first byte is the Class
//     // The second byte is the Group
//     if (modbus.getRegisters(0x03, 0x020E, 2))
//     {
//         ClassGroup = modbus.byteFromFrame(3) + (float)modbus.byteFromFrame(4) / 100;
//         YearWeek = modbus.byteFromFrame(5) + (float)modbus.byteFromFrame(6) / 100;
//         return true;
//     }
//     else return false;
// }

// This returns previously fetched value
bool Insitu::getValueLastTempC(float &value)
{
    value =  _LastTOB1;
    return true;
}
// This gets values back from the sensor
bool Insitu::getValues(float &valueP1, float &valueTOB1, float &valueDepth1)
{
    // Set values to -9999 and error flagged before asking for the result

    valueP1   = INSTU_MB_ERROR_RESULTS;  // Pressure (bar) for sensor1
    valueTOB1 = INSTU_MB_ERROR_RESULTS;  // Temperature (C) on board sensor 1
    valueDepth1 = INSTU_MB_ERROR_RESULTS;  // Depth (?) on board sensor 1

    switch(_model)
    {
        default:  // for all other sensors get two values in one message
        {
            if (modbus.getRegisters(MODBUS_READ_HREG, INSTU_MB_DEVICE_PARAM1_VALUE, MODBUS_RD_2REG))
            {
                valueP1 = modbus.float32FromFrame(bigEndian, MODBUS_FM_START);
                _LastPressure1 = valueP1;

            }
            if (modbus.getRegisters(MODBUS_READ_HREG, INSTU_MB_DEVICE_PARAM2_VALUE, MODBUS_RD_2REG))
            {
                valueTOB1 = modbus.float32FromFrame(bigEndian, MODBUS_FM_START);
                _LastTOB1 = valueTOB1;
            }
            if (modbus.getRegisters(MODBUS_READ_HREG, INSTU_MB_DEVICE_PARAM3_VALUE , MODBUS_RD_2REG))
            {
                valueDepth1 = modbus.float32FromFrame(bigEndian, MODBUS_FM_START);
                _LastDepth =valueDepth1;
            }            
        }
    }
    

    return true;
}

float Insitu::calcWaterDepthM(float &waterPressureBar, float &waterTempertureC)
{
    /// Initialize variables
    float waterPressurePa; // in Pascals (kg/m/s2)
    float waterDensity;    // in kmg/m2
    float waterDepthM;     // in m
    const float gravitationalConstant = 9.80665; // m/s2, meters per second squared

    if (waterPressureBar == -9999)
    {
        waterDepthM = -9999;  // error or sensor not connected
    }
    else
    {
        waterPressurePa = 1e5 * waterPressureBar;
        // Water density (kg/m3) from equation 6 from JonesHarris1992-NIST-DensityWater.pdf
        waterDensity =  + 999.84847
                        + 6.337563e-2 * waterTempertureC
                        - 8.523829e-3 * pow(waterTempertureC,2)
                        + 6.943248e-5 * pow(waterTempertureC,3)
                        - 3.821216e-7 * pow(waterTempertureC,4)
                        ;
        waterDepthM = waterPressurePa/(waterDensity * gravitationalConstant);  // from P = rho * g * h
    }
    #define M_TO_FEET 3.28084
    return waterDepthM/M_TO_FEET ;
}
