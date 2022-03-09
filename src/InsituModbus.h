/*
InsituModbus.h

Written by Neil Hancock

Tested with In-Situ Level Troll 500
*/

#ifndef InsituModbus_h
#define InsituModbus_h

#include <Arduino.h>
#include <SensorModbusMaster.h>

// The various Insitu sensors.
// NOTE: not presently used but in place for future. See use in https://github.com/EnviroDIY/YosemitechModbus
typedef enum InsituModel
{
    //Acculevel_kellerModel = 0,
    //Nanolevel_kellerModel = 1,
    Leveltroll_InsituModel =2,
    OTHER=99   // Use if the sensor model is another model.
} InsituModel;

#define INSTU_MB_ERROR_RESULTS -9999

//Section 7 
#define INSTU_MB_DEVICE_MAP_BASE 0
#define INSTU_MB_DEVICE_ID_REG (0+INSTU_MB_DEVICE_MAP_BASE)
#define INSTU_MB_DEVICE_ID_SZ 1
#define INSTU_MB_DEVICE_SN_REG (01+INSTU_MB_DEVICE_MAP_BASE)
#define INSTU_MB_DEVICE_PARAM1_BASE  37
#define INSTU_MB_DEVICE_PARAM2_BASE  45
#define INSTU_MB_DEVICE_PARAM3_BASE  53
#define INSTU_MB_PARAM_VALUE    0
#define INSTU_MB_PARAM_ID       2
#define INSTU_MB_PARAM_UNITS_ID 3
#define INSTU_MB_PARAM_QLTY_ID  4
#define INSTU_MB_PARAM_SENT_VALUE  5
//
#define INSTU_MB_DEVICE_PARAM1_VALUE (INSTU_MB_DEVICE_PARAM1_BASE +INSTU_MB_PARAM_VALUE )
#define INSTU_MB_DEVICE_PARAM2_VALUE (INSTU_MB_DEVICE_PARAM2_BASE +INSTU_MB_PARAM_VALUE )
#define INSTU_MB_DEVICE_PARAM3_VALUE (INSTU_MB_DEVICE_PARAM3_BASE +INSTU_MB_PARAM_VALUE )

//Sect 6 Probe Common Registers  doesn't seem to support
//#define INSTU_MB_DEVICE_MAP9_BASE 9000

class Insitu
{

public:

    // This function sets up the communication
    // It should be run during the arduino "setup" function.
    // The "stream" device must be initialized prior to running this.
    bool begin(InsituModel model, byte modbusDeviceID, Stream *stream, int enablePin = -1);
    bool begin(InsituModel model, byte modbusDeviceID, Stream &stream, int enablePin = -1);

    // This gets the modbus Device ID.
    byte getDeviceID(void);
    byte getSlaveID(void); //Superseded getDiviceId

    // This sets a new modbus device ID
    // NOTE: NOT YET WORKING
//    bool setDeviceID(byte newDeviceID);

    // This gets the instrument serial number as a long integer
    long getSerialNumber(void);

    // This gets the hardware and software version of the sensor
    // The float variables for the hardware and software versions must be
    // initialized prior to calling this function.
    // The reference (&) is needed when declaring this function so that
    // the function is able to modify the actual input floats rather than
    // create and destroy copies of them.
    // There is no need to add the & when actually using the function.
    // NOTE: NOT YET WORKING
//   bool getVersion(float &hardwareVersion, float &softwareVersion);


    // This gets values back from the sensor
    bool getValues(float &valueP1, float &valueTOB1, float &valueDepth1);
    bool getValueLastTempC(float &value);
//    bool getValues(float &parmValue, float &tempValue, byte &errorCode);

    float calcWaterDepthM(float &waterPressureBar, float &waterTempertureC);

    // This sets a stream for debugging information to go to;
    void setDebugStream(Stream *stream){modbus.setDebugStream(stream);}
    void stopDebugging(void){modbus.stopDebugging();}


private:
    byte _model;
    byte _deviceID;
    float _LastPressure1;
    float _LastTOB1;
    float _LastDepth;

    modbusMaster modbus;
};

#endif
