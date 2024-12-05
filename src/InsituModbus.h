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
// See also  https://github.com/EnviroDIY/YosemitechModbus
typedef enum insituModel
{
    //Acculevel_kellerModel = 0,
    //Nanolevel_kellerModel = 1,
    Leveltroll_InsituModel =2,  //All models that have Param2 temperature and Param3=Depth
    OTHER=99   // Use if the sensor model is another model.
} InsituModel;

//#define INSTU_MB_ERROR_RESULTS -9999
/**
 * @anchor insitu_pressure
 * @name Pressure
 * The pressure variable from a Insitu modbus sensor
 */
/**@{*/
/// @brief Variable default Bar if no reading, can be overriden at cc. (readings are in mBar 1/1000th of this) 
#if !defined SNSRDEF_IP_WATERPRESSUREBAR 
#define SNSRDEF_IP_WATERPRESSUREBAR -0.0009876
#endif
/**@}*/

/**
 * @anchor insitu_temp
 * @name Temperature
 * The temperature variable from a Insitu modbus sensor
 */
/**@{*/
/// @brief Variable default if no reading, can be overriden at cc
#if !defined SNSRDEF_IP_WATERTEMPERATUREC
#define SNSRDEF_IP_WATERTEMPERATUREC -9.876
#endif
/**@}*/

/**
 * @anchor insitu_height
 * @name Height
 * The height variable from a Insitu modbus sensor
 */
/**@{*/
/// @brief Variable default if no reading, can be overriden at cc
#if !defined SNSRDEF_IP_WATERDEPTHM
#define SNSRDEF_IP_WATERDEPTHM -0.09876
#endif
/**@}*/

//Section 7 
#define INSTU_MB_DEVICE_MAP_BASE 0
#define INSTU_MB_DEVICE_ID_REG (0+INSTU_MB_DEVICE_MAP_BASE)
#define INSTU_MB_DEVICE_ID_SZ 1
#define INSTU_MB_DEVICE_SN_REG (01+INSTU_MB_DEVICE_MAP_BASE)
#define INSTU_MB_DEVICE_PARAM1_BASE  37
#define INSTU_MB_DEVICE_PARAM2_BASE  45
#define INSTU_MB_DEVICE_PARAM3_BASE  53

//Physical Measurement, 2 registers requested, 4byte float returned!
#define INSTU_MB_PARAM_VALUE_IDX    0
#define INSTU_MB_PARAM_VALUE_SZ     2

#define INSTU_MB_PARAM_ID_IDX       2
#define INSTU_MB_PARAM_ID_SZ        1
#define INSTU_MB_PARAM_UNITS_ID_IDX 3
#define INSTU_MB_PARAM_UNITS_ID_SZ  1
#define INSTU_MB_PARAM_QLTY_ID_IDX  4
#define INSTU_MB_PARAM_QLTY_ID_Sz   1
#define INSTU_MB_PARAM_SNTL_VALUE_IDX 5
#define INSTU_MB_PARAM_SNTL_VALUE_SZ  2
//
#define INSTU_MB_DEVICE_PARAM1_VALUE (INSTU_MB_DEVICE_PARAM1_BASE +INSTU_MB_PARAM_VALUE_IDX )
#define INSTU_MB_DEVICE_PARAM2_VALUE (INSTU_MB_DEVICE_PARAM2_BASE +INSTU_MB_PARAM_VALUE_IDX )
#define INSTU_MB_DEVICE_PARAM3_VALUE (INSTU_MB_DEVICE_PARAM3_BASE +INSTU_MB_PARAM_VALUE_IDX )

//Sect 6 Probe Common Registers  LT doesn't seem to support
//#define INSTU_MB_DEVICE_MAP9_BASE 9000

//Section 2.3.1.8 Report Slave ID Frame
#define INSTU_MB_RDDEV_LEN_IDX    2
#define INSTU_MB_RDDEV_LEN_VAL 0x17  
#define INSTU_MB_RDDEV_DEV_ID_IDX  8
#define INSTU_MB_RDDEV_FW_VER_IDX 10
#define INSTU_MB_RDDEV_BC_VER_IDX 12
#define INSTU_MB_RDDEV_HW_VER_IDX 14
#define INSTU_MB_RDDEV_TEMPLATE_ID_IDX 14
#define INSTU_MB_RDDEV_SERIAL_NUM_IDX  18
#define INSTU_MB_RDDEV_MAX_RSP_SZ_IDX  22

#define INSTU_MB_PARAM_POWER_MV_REG  108
#define INSTU_MB_PARAM_POWER_MV_SZ     1

typedef enum InsituMb_DevPoll
{
    IMDP_PRESSURE=0x01,
    IMDP_TEMPERATURE=0x02,
    IMDP_DEPTH=0x04,
    IMDP_POWER_MV=0x08,
    IMDP_DEPTH_TEMPERATURE = (IMDP_DEPTH | IMDP_TEMPERATURE),
    IMDP_ALL=(IMDP_DEPTH | IMDP_TEMPERATURE | IMDP_PRESSURE)
} InsituMb_DevPoll;

class insitu
{

public:

    // This function sets up the communication
    // The "stream" device must be initialized prior to running this.
    bool begin(InsituModel model, byte modbusDeviceID, Stream *stream, int enablePin = -1);
    bool begin(InsituModel model, byte modbusDeviceID, Stream &stream, int enablePin = -1);

    // Configure which sensors to poll
    void setDevicePoll(InsituMb_DevPoll devToPoll=IMDP_ALL) {_devToPoll = devToPoll;}

    // This gets information, model, hardware and software version of the sensor
    // The following is called first to get a frame of information, 
    // and then the following functions to parse that data, before the next frame is requested. 
    bool readDeviceIdFrame(void); //Called first
    // This gets the Insitu Device ID "Model Number"- LT5800 
    uint16_t getDeviceHwID(void);
    long getSerialNumber(void);
    // The FwVer is stored as an integer and returned (float) integer/100
    float getDeviceFwVer(void); 

    //Get LT specific readings from the sensor
    bool getLtReadings(float &valueDepth1, float &valueTOB1,float &vwaterPressureB );
    bool getValueLastTempC(float &value);

    // FUT SENSORMODBUSMASTER_DBG 
    // When debugging allow Insitu Specific Exceptions to be listed
    static bool excpHandler(byte excpt);
    // This sets a stream for debugging information to go to;
    void setDebugStream(Stream *stream){modbus.setDebugStream(stream);}
    void stopDebugging(void){modbus.stopDebugging();}
    //#endif //SENSORMODBUSMASTER_DBG 

private:
    byte _model;
    byte _deviceID;
    uint8_t _devToPoll=IMDP_ALL; 
    float _LastPressure1;
    float _LastTOB1;
    float _LastDepth;
    //excpInstuHandler_t _excpInstuHandler_fn = NULL;
    //static Stream *_debugStream1;  // The stream instance (serial port) for debugging

    modbusMaster modbus;
};

#endif
