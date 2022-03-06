/*
InsituModbus.cpp

Written by Neil Hancpcl

Tested with Acculevel, Nanolevel (Neil Hancock)
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
bool Insitu::begin(InsituModel model,byte modbusSlaveID, Stream *stream, int enablePin)
{
    // Give values to variables;
    _slaveID = modbusSlaveID;
    _model = model;
    // Start up the modbus instance
    bool success = modbus.begin(modbusSlaveID, stream, enablePin);
    return success;
}
bool Insitu::begin(InsituModel model,byte modbusSlaveID, Stream &stream, int enablePin)
{return begin(model,modbusSlaveID, &stream, enablePin);}


// This gets the modbus slave ID.
// For Insitu, slaveID is function code 0x11register 0x020D (525), or regNum = 0x020D  njh
// regType = 0x03 for all Keller Modbus Register Read functions njh
// 2.3.1.8
byte Insitu::getSlaveID(void) //njh tested, gets something
{
    //2.3.1.8 Report Slave ID
    return modbus.byteFromRegister(0x03, 0x11, 3); 
    //return modbus.byteFromRegister(0x03, 0x11, 0); // byte byteFromRegister(byte regType, int regNum, int byteNum)
}

byte Insitu::getSlaveIDFrame(void) 
{
    //2.3.1.8 Report Slave ID
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

// This sets a new modbus slave ID
// For Keller, The slaveID is in register 0x020D (525), or regNum = 0x020D
// NOTE: NOT YET TESTED
// bool keller::setSlaveID(byte newSlaveID)
// {
//     return modbus.byteToRegister(0x020D, 2, newSlaveID); //bool byteToRegister(int regNum, int byteNum, byte value, bool forceMultiple=false);
// }

/*byte Insitu::getDeviceID(void) //njh tested, gets something
{
    //#define IL_SLAVE_LEN 5
    //byte command[IL_SLAVE_LEN ];
    //comamnd 
    //sendCommand(byte command[], int commandLength);

    //2.3.1.8 Report Slave ID
    return modbus.byteFromRegister(0x03, 0x9001, 3); 
    //return modbus.byteFromRegister(0x03, 0x11, 0); // byte byteFromRegister(byte regType, int regNum, int byteNum)
} */

// This gets the instrument serial number as a 16-bit unsigned integer (as specified by Insitu)
// The Serial number is in holding registers 0x2 or 9002 and occupies 2 registers (uint16)
long Insitu::getSerialNumber(void)
{
    #define LT_SENSOR_SERIAL_NUMBER_REG 1
    return modbus.uint16FromRegister(0x03, LT_SENSOR_SERIAL_NUMBER_REG);  //Sect 7 Common registers
    //return modbus.uint32FromRegister(0x03, 0x0202); // uint32_t uint32FromRegister(byte regType, int regNum, endianness endian=bigEndian);
}


// This gets the hardware and software version of the sensor
// This data begins in holding register 0x020E (??) and continues for 2 registers
// bool keller::getVersion(float &ClassGroup, float &YearWeek)
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
// Uses Keller Process Value Read Range (0x0100) 32bit floating point,
// which is Same as 0x0000 .. 0x000B but different mapping for accessing data in one cycle (e.g. P1 and TOB1)
// P1 is in register 0x0100 & TOB1 (Temperature of sensor1) is in 0x0102
bool Insitu::getValues(float &valueP1, float &valueTOB1)
{
    // Set values to -9999 and error flagged before asking for the result
    valueP1   = -9999;  // Pressure (bar) for sensor1
    valueTOB1 = -9999;  // Temperature (C) on board sensor 1

    switch(_model)
    {
        /*case Nanolevel_InsituModel:  // This gets two values, but as seperate messages
        {
            if (modbus.getRegisters(0x03, 0x0002, 2))
            {
                valueP1 = modbus.float32FromFrame(bigEndian, 3);
                if (modbus.getRegisters(0x03, 0x0006, 2))
                {
                   valueTOB1 = modbus.float32FromFrame(bigEndian, 3);
                   break;
                }
                else return false;
            }
            else return false;
        }*/
        default:  // for all other sensors get two values in one message
        {
            if (modbus.getRegisters(0x03, 0x0100, 4))
            {
                valueP1 = modbus.float32FromFrame(bigEndian, 3);
                valueTOB1 = modbus.float32FromFrame(bigEndian, 7);
                break;
            }
            else return false;
        }
    }
    
    _LastTOB1 = valueTOB1;
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

    return waterDepthM;
}
