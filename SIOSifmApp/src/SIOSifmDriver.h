#ifndef _H_SIOS_IFM
#define _H_SIOS_IFM

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>

#include <epicsExport.h>
#include "asynPortDriver.h"
#include "siosifmdef.h"
#include "siosifmdll.h"

/* Parameters: */
#define P_STR_Run                 "SIOS_RUN"                  /* Int32,    rw */
#define P_STR_Mode                "SIOS_MODE"                 /* Int32,    rw */ 
#define P_STR_Channel1            "SIOS_CHANNEL1"             /* Float64,  rw */ 
#define P_STR_Channel2            "SIOS_CHANNEL2"             /* Float64,  rw */ 
#define P_STR_Channel3            "SIOS_CHANNEL3"             /* Float64,  rw */ 
#define P_STR_Channel4            "SIOS_CHANNEL4"             /* Float64,  rw */ 
#define P_STR_MeasureLength       "SIOS_MEASURE_LENGTH"       /* Int32,    rw */
#define P_STR_MeasureSinCos       "SIOS_MEASURE_SIN_COS"      /* Int32,    rw */
#define P_STR_MeasureCircle       "SIOS_MEASURE_CIRCLE"       /* Int32,    rw */
#define P_STR_TestPattern         "SIOS_TEST_PATTERN"         /* Int32,    rw */
#define P_STR_ClockCounter        "SIOS_CLOCK_COUNTER"        /* Int32,    rw */
#define P_STR_SampleCounter       "SIOS_SAMPLE_COUNTER"       /* Int32,    rw */
#define P_STR_BeamBreak           "SIOS_BEAM_BREAK"           /* Int32,    rw */
#define P_STR_MeasureChannel1     "SIOS_MEASURE_CHANNEL1"     /* Int32,    rw */
#define P_STR_MeasureChannel2     "SIOS_MEASURE_CHANNEL2"     /* Int32,    rw */
#define P_STR_MeasureChannel3     "SIOS_MEASURE_CHANNEL3"     /* Int32,    rw */
#define P_STR_MeasureChannel4     "SIOS_MEASURE_CHANNEL4"     /* Int32,    rw */
#define P_STR_RawMode             "SIOS_RAW_MODE"             /* Int32,    rw */
#define P_STR_FilterUser          "SIOS_FILTER_USER"          /* Int32,    rw */
#define P_STR_OutputWordRate      "SIOS_OUTPUT_WORD_RATE"     /* Int32,    rw */
#define P_STR_Zero                "SIOS_ZERO"                 /* Int32,    rw */
#define P_STR_AverageAmount       "SIOS_AVERAGE_AMOUNT"       /* Int32,    rw */
#define P_STR_BeamBroken1         "SIOS_BEAM_BROKEN1"         /* Int32,    rw */
#define P_STR_BeamBroken2         "SIOS_BEAM_BROKEN2"         /* Int32,    rw */
#define P_STR_BeamBroken3         "SIOS_BEAM_BROKEN3"         /* Int32,    rw */
#define P_STR_BeamBroken4         "SIOS_BEAM_BROKEN4"         /* Int32,    rw */
#define P_STR_LaserUnstable1      "SIOS_LASER_UNSTABLE1"      /* Int32,    rw */
#define P_STR_LaserUnstable2      "SIOS_LASER_UNSTABLE2"      /* Int32,    rw */
#define P_STR_LaserUnstable3      "SIOS_LASER_UNSTABLE3"      /* Int32,    rw */
#define P_STR_LaserUnstable4      "SIOS_LASER_UNSTABLE4"      /* Int32,    rw */
#define P_STR_LostValues          "SIOS_LOST_VALUES"          /* Int32,    rw */
#define P_STR_SignalQuality1      "SIOS_SIGNAL_QUALITY1"      /* Int32,  rw */
#define P_STR_SignalQuality2      "SIOS_SIGNAL_QUALITY2"      /* Int32,  rw */
#define P_STR_SignalQuality3      "SIOS_SIGNAL_QUALITY3"      /* Int32,  rw */
#define P_STR_SignalQuality4      "SIOS_SIGNAL_QUALITY4"      /* Int32,  rw */
#define P_STR_AirPressure1        "SIOS_AIR_PRESSURE1"        /* Float64,  rw */
#define P_STR_AirPressure2        "SIOS_AIR_PRESSURE2"        /* Float64,  rw */
#define P_STR_AirPressure3        "SIOS_AIR_PRESSURE3"        /* Float64,  rw */
#define P_STR_AirPressure4        "SIOS_AIR_PRESSURE4"        /* Float64,  rw */
#define P_STR_AirRefraction1      "SIOS_AIR_REFRACTION1"      /* Float64,  rw */
#define P_STR_AirRefraction2      "SIOS_AIR_REFRACTION2"      /* Float64,  rw */
#define P_STR_AirRefraction3      "SIOS_AIR_REFRACTION3"      /* Float64,  rw */
#define P_STR_AirRefraction4      "SIOS_AIR_REFRACTION4"      /* Float64,  rw */
#define P_STR_Humidity1           "SIOS_HUMIDITY1"            /* Float64,  rw */
#define P_STR_Humidity2           "SIOS_HUMIDITY2"            /* Float64,  rw */
#define P_STR_Humidity3           "SIOS_HUMIDITY3"            /* Float64,  rw */
#define P_STR_Humidity4           "SIOS_HUMIDITY4"            /* Float64,  rw */
#define P_STR_Temperature1        "SIOS_TEMPERATURE1"         /* Float64,  rw */
#define P_STR_Temperature2        "SIOS_TEMPERATURE2"         /* Float64,  rw */
#define P_STR_Temperature3        "SIOS_TEMPERATURE3"         /* Float64,  rw */
#define P_STR_Temperature4        "SIOS_TEMPERATURE4"         /* Float64,  rw */
#define P_STR_VaporPressure1      "SIOS_VAPOR_PRESSURE1"      /* Float64,  rw */
#define P_STR_VaporPressure2      "SIOS_VAPOR_PRESSURE2"      /* Float64,  rw */
#define P_STR_VaporPressure3      "SIOS_VAPOR_PRESSURE3"      /* Float64,  rw */
#define P_STR_VaporPressure4      "SIOS_VAPOR_PRESSURE4"      /* Float64,  rw */
#define P_STR_Wavelength1         "SIOS_WAVELENGTH1"          /* Float64,  rw */
#define P_STR_Wavelength2         "SIOS_WAVELENGTH2"          /* Float64,  rw */
#define P_STR_Wavelength3         "SIOS_WAVELENGTH3"          /* Float64,  rw */
#define P_STR_Wavelength4         "SIOS_WAVELENGTH4"          /* Float64,  rw */
#define P_STR_StdDev1             "SIOS_STDDEV1"              /* Float64,  rw */
#define P_STR_StdDev2             "SIOS_STDDEV2"              /* Float64,  rw */
#define P_STR_StdDev3             "SIOS_STDDEV3"              /* Float64,  rw */
#define P_STR_StdDev4             "SIOS_STDDEV4"              /* Float64,  rw */
#define P_STR_DeadPath1           "SIOS_DEAD_PATH1"           /* Int32,    rw */
#define P_STR_DeadPath2           "SIOS_DEAD_PATH2"           /* Int32,    rw */
#define P_STR_DeadPath3           "SIOS_DEAD_PATH3"           /* Int32,    rw */
#define P_STR_DeadPath4           "SIOS_DEAD_PATH4"           /* Int32,    rw */

#define SIOS_MODE_CONTINUOUS   0
#define SIOS_MODE_BLOCK        1

#define SIOS_MAX_CHANNELS      4
/* functionality:
 * 
 * available through exported functions:
 *  Open and initialize via
 *    COM (string in linux, int in win32)
 *    USB (by serial or device #)
 *
 * available through parameters:
 *
 * mode switch (continuous/block)
 *
 * continuous mode:
 *  Start/stop acquisition
 *  Set measurement options
 *    Channels
 *    Filter
 *    Switch to block mode
 *  Notch filter frequency (rw)
 *  Filter coefficients (rw)
 *  Lengths (channels 0~3) (ro)
 *  Set to zero
 *  Offset from zero ('preset' in IFM)
 *  Trigger mode (a few options)
 *
 * block mode:
 *  (use waveform)
 *  Set block filter
 *  Read block
 *
 */

class SIOSifmDriver : public asynPortDriver {
public:
    SIOSifmDriver(const char *portName);
    ~SIOSifmDriver();
                 
    /* Overridden from asynPortDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    /* New functionality */
    void pollTask(void);

    asynStatus connectUSBbyDeviceNumber(int deviceNum);
    asynStatus connectUSBbySerialNumber(int serialNum);
    void setFlag(uint flag, bool value);

protected:
    /** Values used for pasynUser->reason, and indexes into the parameter library. */
    int P_Run;
    #define FIRST_SIOS_COMMAND P_Run
    int P_Mode;
    int P_Settings;
    int P_Channel[SIOS_MAX_CHANNELS];
    int P_MeasureLength;
    int P_MeasureSinCos;
    int P_MeasureCircle;
    int P_TestPattern;
    int P_ClockCounter;
    int P_SampleCounter;
    int P_BeamBreak;
    int P_MeasureChannel[SIOS_MAX_CHANNELS];
    int P_RawMode;
    int P_FilterUser;
    int P_OutputWordRate;
    int P_AverageAmount;

    int P_LostValues;
    int P_BeamBroken[SIOS_MAX_CHANNELS];
    int P_LaserUnstable[SIOS_MAX_CHANNELS];
    int P_SignalQuality[SIOS_MAX_CHANNELS];
    int P_AirPressure[SIOS_MAX_CHANNELS];
    int P_AirRefraction[SIOS_MAX_CHANNELS];
    int P_Humidity[SIOS_MAX_CHANNELS];
    int P_Temperature[SIOS_MAX_CHANNELS];
    int P_VaporPressure[SIOS_MAX_CHANNELS];
    int P_Wavelength[SIOS_MAX_CHANNELS];
    int P_DeadPath[SIOS_MAX_CHANNELS];
    int P_StdDev[SIOS_MAX_CHANNELS];
    int P_Zero;
    #define LAST_SIOS_COMMAND P_Zero
    #define NUM_SIOS_PARAMS (&LAST_SIOS_COMMAND - &FIRST_SIOS_COMMAND + 1)

    const uint 
    ReasonToFlag(int reason) {
        if (reason == P_MeasureLength) {
            return IFM_MEAS_LENGTH;
        } else if (reason == P_MeasureSinCos) {
            return IFM_MEAS_SINCOS;
        } else if (reason == P_MeasureCircle) {
            return IFM_MEAS_CIRCLE;
        } else if (reason == P_TestPattern) {
            return IFM_MEAS_PATTERN;
        } else if (reason == P_ClockCounter) {
            return IFM_MEAS_CLOCKCOUNT;
        } else if (reason == P_SampleCounter) {
            return IFM_MEAS_SAMPLECOUNT;
        } else if (reason == P_BeamBreak) {
            return IFM_MEAS_BEAMBREAK_OFF;
        } else if (reason == P_MeasureChannel[0]) {
            return IFM_MEAS_CH1;
        } else if (reason == P_MeasureChannel[1]) {
            return IFM_MEAS_CH2;
        } else if (reason == P_MeasureChannel[2]) {
            return IFM_MEAS_CH3;
        } else if (reason == P_MeasureChannel[3]) {
            return IFM_MEAS_CH4;
        } else if (reason == P_RawMode) {
            return IFM_MEAS_RAWMODE;
        } else if (reason == P_FilterUser) {
            return IFM_MEAS_FILTER_USER;
        } else {
            return 0;
        }
    }

    void updateEnvironment(void);

private:
    epicsEvent runEvent_;
    epicsEvent zeroEvent_;
    epicsEvent flagsUpdatedEvent_;
    bool connected_;
    int dev_;
    int flags_;
    bool running_;

};

/* Use the following structure and functions to manage multiple instances 
 * of the driver */
typedef struct SIOSifmNode {
    ELLNODE node;
    const char *portName;
    SIOSifmDriver *pDriver;
} SIOSifmNode;

bool addDriverToList(const char *portName, SIOSifmDriver *drv);
SIOSifmDriver* findDriverByPortName(const char *portName);
// --

const char* getSiosErrorString(int error) {
    switch (error) {
    case IFM_ERROR_NONE:
        return "IFM_ERROR_NONE";
    case IFM_ERROR_BAD_CHANNEL:
        return "IFM_ERROR_BAD_CHANNEL";
    case IFM_ERROR_BAD_DEVICETYPE:
        return "IFM_ERROR_BAD_DEVICETYPE";
    case IFM_ERROR_DATALEN:
        return "IFM_ERROR_DATALEN";
    case IFM_ERROR_BAD_DEVICE:
        return "IFM_ERROR_BAD_DEVICE or IFM_ERROR_DEVICE_INVALID";
    case IFM_ERROR_UNKNOWN:
        return "IFM_ERROR_UNKNOWN";
    case IFM_ERROR_DEVICECOUNT_OVERFLOW:
        return "IFM_ERROR_DEVICECOUNT_OVERFLOW";
    case IFM_ERROR_BAD_REQUESTTYPE:
        return "IFM_ERROR_BAD_REQUESTTYPE";
    case IFM_ERROR_INVALID_USB_ID:
        return "IFM_ERROR_INVALID_USB_ID";
    case IFM_ERROR_CREATE_HANDLE:
        return "IFM_ERROR_CREATE_HANDLE";
    case IFM_ERROR_NOT_IMPLEMENTED:
        return "IFM_ERROR_NOT_IMPLEMENTED";
    case IFM_ERROR_I2C_INUSE:
        return "IFM_ERROR_I2C_INUSE";
    case IFM_ERROR_I2C_WRITE:
        return "IFM_ERROR_I2C_WRITE";
    case IFM_ERROR_I2C_TIMEOUT:
        return "IFM_ERROR_I2C_TIMEOUT";
    case IFM_ERROR_OWR_TOOHIGHT:
        return "IFM_ERROR_OWR_TOOHIGHT";
    case IFM_ERROR_INFO_NOT_AVAILABLE:
        return "IFM_ERROR_INFO_NOT_AVAILABLE";
    case IFM_ERROR_BAD_SENSOR:
        return "IFM_ERROR_BAD_SENSOR";
    case IFM_ERROR_I2C_READ:
        return "IFM_ERROR_I2C_READ";
    default:
        return "Unknown error";
    }
}

#endif
