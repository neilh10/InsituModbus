/*
InsituModbus.cpp

Written by Neil Hancock from KellerVersion

Tested with 
- a Insitu Series Level Troll series transducers LT500 - System Spec 1
Reference
In-Situ Modbus Communication Protocol Version 5.10
- https://in-situ.com/us/pub/media/support/documents/Modbus_Manual.pdf
 All In-Situ Registers are Holding Registers ie access as 0x03 MODBUS_READ_HREG


*/

#include "InsituModbus.h"


//----------------------------------------------------------------------------
//                          PUBLIC SENSOR FUNCTIONS
//----------------------------------------------------------------------------


// Start the communication
// The "stream" device must be initialized and begun prior to running this.
bool Insitu::begin(InsituModel model,byte modbusDeviceID, Stream *stream, int enablePin)
{
    // Give values to variables;
    _deviceID = modbusDeviceID;
    _model = model;
    // Start up the modbus instance
    bool success = modbus.begin(modbusDeviceID, stream, enablePin);
    //modbus.registerErrHandler(excpHandler);
    return success;
}

bool Insitu::begin(InsituModel model,byte modbusDeviceID, Stream &stream, int enablePin)
{
    return begin(model,modbusDeviceID, &stream, enablePin);
}


// This gets the modbus Device ID.
// 2.3.1.8
uint16_t Insitu::getDeviceHwID(void) //njh tested
{
    //2.3.1.8 Report (Slave) Device ID  special register 
    return modbus.uint16FromFrame(bigEndian, INSTU_MB_RDDEV_DEV_ID_IDX); 
}
float Insitu::getDeviceFwVer(void) //njh tested
{
    //2.3.1.8 Report (Slave) Device ID  special register 
    return (modbus.uint16FromFrame(bigEndian, INSTU_MB_RDDEV_FW_VER_IDX )/100); 
}

bool Insitu::readDeviceIdFrame(void) 
{   
    bool retRsp = false;
    //From 2.3.1.8 Report Slave Id - 
#define RDID_LEN 4
    byte command[RDID_LEN];

    command[0] = _deviceID;
    command[1] = MODBUS_REPORT_DEV_ID;
    //command 2 & 3 ~ crc

    // Send cmd with reTry
    int tries = 10;
    int16_t respSize = 0;
    do
    {
        // Send to instrument (this adds the CRC)
        respSize = modbus.sendCommand(command,RDID_LEN);
        if (INSTU_MB_RDDEV_LEN_VAL <= respSize) {
            retRsp = true;
            break;
        }
        //Serial.print("Retry: received ");
        //Serial.println(respSize);
        delay(25);
    } while (0 < --tries );

    return retRsp;//modbus.byteFromRegister(0x03, 0x11, 3); 
    //return modbus.byteFromRegister(0x03, 0x11, 0); // byte byteFromRegister(byte regType, int regNum, int byteNum)
}

// This gets the instrument serial number as a 32-bit unsigned integer (as specified by Insitu)
long Insitu::getSerialNumber(void) //Nh Gets something 
{
    return modbus.uint32FromFrame(bigEndian, INSTU_MB_RDDEV_SERIAL_NUM_IDX); 
}


// This returns previously fetched value
bool Insitu::getValueLastTempC(float &value)
{
    value =  _LastTOB1;
    return true;
}

bool Insitu::getLtReadings( float &valueDepth1,float &valueTOB1,float &valueP1)
{

    switch(_model)
    {
        default:  //Leveltroll_InsituModel  for water sensors 
        {
            if (IMDP_PRESSURE & _devToPoll) {
                if (modbus.getRegisters(MODBUS_READ_HREG, INSTU_MB_DEVICE_PARAM1_VALUE, INSTU_MB_PARAM_VALUE_SZ)) {
                    valueP1 = modbus.float32FromFrame(bigEndian, MODBUS_FM_START);
                } else {
                    valueP1 = INSTU_MB_ERROR_RESULTS;  // Temperature (C) on board sensor 1
                }
                _LastPressure1 = valueP1;                            
            } //IMDP_PRESSURE 
            if (IMDP_TEMPERATURE & _devToPoll) {
                if (modbus.getRegisters(MODBUS_READ_HREG, INSTU_MB_DEVICE_PARAM2_VALUE, INSTU_MB_PARAM_VALUE_SZ)) {
                    valueTOB1 = modbus.float32FromFrame(bigEndian, MODBUS_FM_START);
                }else {
                    valueTOB1 = INSTU_MB_ERROR_RESULTS;  // Temperature (C) on board sensor 1
                }
                _LastTOB1 = valueTOB1;
            }//IMDP_TEMPERATURE
            if (IMDP_DEPTH & _devToPoll) {
                if (modbus.getRegisters(MODBUS_READ_HREG, INSTU_MB_DEVICE_PARAM3_VALUE , INSTU_MB_PARAM_VALUE_SZ)) {
                    valueDepth1 = modbus.float32FromFrame(bigEndian, MODBUS_FM_START);
                }else {
                    valueDepth1 = INSTU_MB_ERROR_RESULTS;  // Depth (?) on board sensor 1
                }
                _LastDepth =valueDepth1;
            }//IMDP_DEPTH 
            #if 0
            if (IMDP_POWER_MV & _devToPoll) { //Only Leel TROLL 300 500 700 & Baro Troll 500
                if (modbus.getRegisters(MODBUS_READ_HREG, INSTU_MB_PARAM_POWER_MV_REG , INSTU_MB_PARAM_POWER_MV_SZ)) {
                    valueDepth1 = modbus.float32FromFrame(bigEndian, MODBUS_FM_START);
                }else {
                    valueDepth1 = INSTU_MB_ERROR_RESULTS;  // Depth (?) on board sensor 1
                }
                _LastDepth =valueDepth1;
            }//IMDP_POWER_MV 
            #endif //0
        }
    }
    

    return true;
}

bool Insitu::excpHandler(byte excpt) {
#if 0
    if (NULL == Insitu::_debugStream1) return false; //Not handled
    Stream *_debugStream = Insitu::_debugStream1;
    switch(excpt) {
        //In-situ Extended 
        case 0x80:{_debugStream->println(F("Field Mismatch")); break;}
        case 0x81:{_debugStream->println(F("Wr Only")); break;}
        case 0x82:{_debugStream->println(F("Rd Only")); break;}
        case 0x83:{_debugStream->println(F("Access Level")); break;}
        case 0x84:{_debugStream->println(F("Wr value")); break;}
        case 0x85:{_debugStream->println(F("Cmd Seq")); break;}
        case 0x86:{_debugStream->println(F("File Seq")); break;}
        case 0x87:{_debugStream->println(F("File Cmd")); break;}
        case 0x88:{_debugStream->println(F("File Number")); break;}
        case 0x89:{_debugStream->println(F("File Size")); break;}
        case 0x8A:{_debugStream->println(F("File Data")); break;}
        case 0x8B:{_debugStream->println(F("File Interval")); break;}
        case 0x90:{_debugStream->println(F("Gateway Err")); break;}
        case 0x91:{_debugStream->println(F("Sensor Seq")); break;}
        case 0x92:{_debugStream->println(F("Sensor Mode")); break;}
        case 0x93:{_debugStream->println(F("Sensor Config")); break;}
        case 0x94:{_debugStream->println(F("Sensor Missing")); break;}
        case 0x95:{_debugStream->println(F("Sensor Invalid")); break;}
        case 0x96:{_debugStream->println(F("Sensors Firmware")); break;}
        case 0xA0:{_debugStream->println(F("Data Log Reg")); break;}
        case 0xA1:{_debugStream->println(F("Data Log Mem")); break;}
        case 0xA2:{_debugStream->println(F("Data Log Dir")); break;}
        case 0xA3:{_debugStream->println(F("Data Log Edit")); break;}
        case 0xA4:{_debugStream->println(F("Data Log Seq")); break;}
        default: 
            return false;
            break;
    }
    return true;
#else 
    return false;
#endif //0
 }

#if 0
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
#endif
