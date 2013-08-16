#include "SIOSifmDriver.h"

static ELLLIST SIOSifmDriverList;
static int SIOSifmDriverListInitialized = 0;

static const char *driverName="SIOSifmDriver";

void pollTask(void *drvPvt)
{
    SIOSifmDriver *pPvt = (SIOSifmDriver *)drvPvt;
    
    pPvt->pollTask();
}

void SIOSifmDriver::updateEnvironment() {
    bool updated=false;

    if (IfmNewSignalQualityAvailable(dev_)) {
        for (int i=0; i < SIOS_MAX_CHANNELS; i++) {
            setIntegerParam(P_SignalQuality[i], (unsigned int)(IfmSignalQuality(dev_, i, IFM_SIGNALQ_SUM)));
        }
        updated=true;
    }

    if (IfmNewEnvValuesAvailable(dev_)) {
        setIntegerParam(P_LostValues, IfmWasLostValues(dev_));

        for (int i=0; i < SIOS_MAX_CHANNELS; i++) {
            setDoubleParam(P_AirPressure[i],    IfmAirPressure(dev_, i));
            setDoubleParam(P_VaporPressure[i],  IfmWaterVapourPressure(dev_, i));
            setDoubleParam(P_AirRefraction[i],  IfmAirRefraction(dev_, i));
            setDoubleParam(P_Humidity[i],       IfmHumidity(dev_, i));
            setDoubleParam(P_Temperature[i],    IfmTemperature(dev_, i));
            setDoubleParam(P_Wavelength[i],     IfmWavelength(dev_, i));
            setIntegerParam(P_BeamBroken[i],    IfmWasBeamBreak(dev_, i));
            setIntegerParam(P_LaserUnstable[i], IfmWasLaserUnstable(dev_, i));
        }
        updated=true;
    }
    if (updated)
        callParamCallbacks();
}
void SIOSifmDriver::pollTask(void) {
    static const char *functionName = "pollTask";

    int run=0;
    int i=0;
    double rate = 0.10;
    int outputWordRate;
    int available=0;
    int err=0;
    int averageAmount=0;
    double length;
    bool zeroing=false;
    
    int allocated=0;
    double* data[SIOS_MAX_CHANNELS]={NULL, NULL, NULL, NULL};
    double offset[SIOS_MAX_CHANNELS]={0.0, 0.0, 0.0, 0.0};

    //lock();

    while (1) {
        //unlock();
        runEvent_.wait(rate);
        //lock();

        updateEnvironment();

        getIntegerParam(P_Run, &run);
        if (!run) continue;

        running_ = true;

        getIntegerParam(P_OutputWordRate, &outputWordRate);
        getIntegerParam(P_AverageAmount, &averageAmount);

        err = IfmSetMeasurement(dev_, flags_, outputWordRate);
        fprintf(stderr, "%s:%s Measurement flags SetMeasurement(flags=%x, rate=%d)\n", 
                driverName, functionName, flags_, outputWordRate);

        if (err) {
            fprintf(stderr, "%s:%s Unable to set measurement flags SetMeasurement(flags=%x, rate=%d) = %s\n", 
                driverName, functionName, flags_, outputWordRate, getSiosErrorString(err));
            setIntegerParam(P_Run, 0);
            continue;
        }

        fprintf(stderr, "Running\n");
        err = IfmStart(dev_);
        if (err) {
            fprintf(stderr, "%s:%s Unable to start measurement: %s\n", 
                driverName, functionName, getSiosErrorString(err));
            setIntegerParam(P_Run, 0);
            continue;
        }

        if (averageAmount < 0) {
            averageAmount = 0;
        }

        if (allocated != averageAmount) {
            if (allocated == 0) {
            } else if (allocated > 100 * averageAmount || allocated < averageAmount) {
                printf("Re-allocating samples (allocated=%d, new average amount=%d)\n", allocated, averageAmount);
                for (i=0; i < SIOS_MAX_CHANNELS; i++) {
                    delete [] data[i];
                    data[i] = NULL;
                }
                allocated = 0;
            }

            if (averageAmount > 1) {
                bool success=true;
                for (i=0; i < SIOS_MAX_CHANNELS; i++) {
                    data[i] = new double [averageAmount];
                    if (!data[i]) {
                        fprintf(stderr, "Allocating failed! %d\n", averageAmount);
                        success=false;
                        break;
                    }
                }

                if (success) {
                    allocated = averageAmount;
                } else {
                    fprintf(stderr, "Freeing allocated and no longer using averaging.\n");
                    for (int j=0; j < i; j++) {
                        delete [] data[j];
                        data[j] = NULL;
                    }
                    averageAmount = 0;
                }
            }
        }

        int sample_index=0;
        do {
            if ((available = IfmValueCount(dev_)) > 0) {
                //printf("Available: %d\n", available);
                while (available > 0) {
                    if (zeroEvent_.tryWait()) {
                        for (i=0; i < SIOS_MAX_CHANNELS; i++) {
                            offset[i] = 0.0;
                        }

                        if (averageAmount <= 1) {
                            fprintf(stderr, "%s:%s Zeroing\n", driverName, functionName);
                            err = IfmSetToZero(dev_, 0x0F);
                            if (err) {
                                fprintf(stderr, "%s:%s Unable to zero: %s\n", 
                                    driverName, functionName, getSiosErrorString(err));
                            }
                            IfmClearBuffers(dev_);
                            break;
                        } else {
                            sample_index = 0;
                            zeroing = true;
                        }
                    }

                    IfmGetValues(dev_);
                    if (averageAmount <= 1) {
                        for (i=0; i < SIOS_MAX_CHANNELS; i++) {
                            if (flags_ & (IFM_MEAS_CH1 << i)) {
                                length = IfmLengthValue(dev_, i) - offset[i];
                                setDoubleParam(P_Channel[i], length);
                            }
                        }
                    } else {
                        for (i=0; i < SIOS_MAX_CHANNELS; i++) {
                            if (flags_ & (IFM_MEAS_CH1 << i)) {
                                length = IfmLengthValue(dev_, i);
                                data[i][sample_index] = length;
                            }
                        }
                        
                        sample_index++;
                        if (sample_index == averageAmount) {
                            double mean, std_dev;
                            for (i=0; i < SIOS_MAX_CHANNELS; i++) {
                                if (flags_ & (IFM_MEAS_CH1 << i)) {
                                    //Update the mean and std dev
                                    mean = 0.0;
                                    std_dev = 0.0;
                                    for (int j=0; j < averageAmount; j++) {
                                        mean += (data[i][j] - mean) / (i+1);
                                    }

                                    for (int j=0; j < averageAmount; j++) {
                                        std_dev += (data[i][j] - mean) * (data[i][j] - mean);
                                    }
                                    std_dev = sqrtf(std_dev / (averageAmount - 1));
                                    
                                    // Then set the offset (if zeroing) or pass the length back through the channel PV
                                    length = mean - offset[i];
    
                                    if (zeroing) {
                                        offset[i] = length;
                                        printf("Channel %d zero position %g std dev: %g (%d samples)\n", i, length, std_dev, averageAmount);
                                    } else {
                                        setDoubleParam(P_Channel[i], length);
                                        setDoubleParam(P_StdDev[i], std_dev);
                                    }
                                }

                            }
                            if (zeroing) {
                                zeroing = false;
                            }
                            sample_index = 0;
                        }
                    }
                    callParamCallbacks();

                    available--;
                    epicsThreadSleepQuantum();

                    updateEnvironment();

                    if (runEvent_.tryWait()) {
                        getIntegerParam(P_Run, &run);
                        if (!run) {
                            IfmClearBuffers(dev_);
                            break;
                        }
                    }
                }
            } else {
                //unlock();
                epicsThreadSleep(rate);
                //lock();
            }

            if (runEvent_.tryWait()) {
                getIntegerParam(P_Run, &run);
            }
        } while (run);

        IfmStop(dev_);
        running_ = false;
        fprintf(stderr, "Stopping\n");

    }
}
void SIOSifmDriver::setFlag(uint flag, bool value) {
    // TODO: ensure valid combinations here
    switch (flag) {
    case IFM_MEAS_BEAMBREAK_OFF:
        value = !value;
        break;

    case IFM_MEAS_LENGTH:
    case IFM_MEAS_SINCOS:
    case IFM_MEAS_CIRCLE:
    case IFM_MEAS_PATTERN:
    case IFM_MEAS_CLOCKCOUNT:
    case IFM_MEAS_SAMPLECOUNT:
    case IFM_MEAS_FILTER_USER:
    default:
        break;
    }

    if (value)
        flags_ |= flag;
    else
        flags_ &= ~flag;

    flagsUpdatedEvent_.signal();
}

/** Called when asyn clients call pasynInt32->write().
  * For all parameters it sets the value in the parameter library and calls any registered callbacks.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus SIOSifmDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName = "(unset)";
    const char* functionName = "writeFloat64";

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    printf("%s updated = %g\n", paramName, value);
    /* Set the parameter in the parameter library. */
    status = setDoubleParam(function, value);
    
    /* Do callbacks so higher layers see any changes */
    status = callParamCallbacks();
    
    if (status) 
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%g", 
                  driverName, functionName, status, function, paramName, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%g\n", 
              driverName, functionName, function, paramName, value);
    return status;
}

/** Called when asyn clients call pasynInt32->write().
  * For all parameters it sets the value in the parameter library and calls any registered callbacks.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus SIOSifmDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName = "(unset)";
    const char* functionName = "writeInt32";

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    int flag=0;

    /* Set the parameter in the parameter library. */
    status = setIntegerParam(function, value);

    if (function == P_Run) {
        /* If run was set then wake up the simulation task */
        runEvent_.signal();
    } else if (function == P_Zero) {
        printf("Zero\n");
        zeroEvent_.signal();
        setIntegerParam(function, 0);
    } else if (function == P_Mode) {
        if (!running_) {
            // TODO setMode(value);
        }
    } else if ((flag = ReasonToFlag(function)) != 0) {
        setFlag(flag, (value != 0));
    }
    
    /* Do callbacks so higher layers see any changes */
    status = callParamCallbacks();
    
    if (status) 
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%d", 
                  driverName, functionName, status, function, paramName, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%d\n", 
              driverName, functionName, function, paramName, value);
    return status;
}

asynStatus SIOSifmDriver::connectUSBbyDeviceNumber(int deviceNum) {
    static const char* functionName = "connectUSBbyDeviceNumber";

    //asynPrint(pasynUser, ASYN_TRACE_FLOW, 
    //      "%s:%s: deviceNum=%d\n", 
    //      driverName, functionName, deviceNum);
    printf("%s:%s: deviceNum=%d\n", driverName, functionName, deviceNum);
    
    int cnt=IfmSearchUSBDevices();
    if (cnt <= 0) {
        fprintf(stderr, "%s:%s: IfmSearchUSBDevices: SIOS interferometer not found on USB bus\n", 
           driverName, functionName);
        return asynError;
    }
    
    dev_ = IfmOpenUSB(deviceNum);
    if (dev_ < 0) {
        fprintf(stderr, "%s:%s: IfmOpenUSB error %d = %s\n", 
           driverName, functionName, dev_, getSiosErrorString(dev_));
        return asynError;
    }

    printf("%s:%s: Connected (dev=%d)\n", driverName, functionName, deviceNum);
    connected_ = true;
    return asynSuccess;
}

asynStatus SIOSifmDriver::connectUSBbySerialNumber(int serialNum) {
    static const char* functionName = "connectUSBbySerialNumber";

    printf("%s:%s: Searching for serial %6.6d\n", driverName, functionName, serialNum);
    
    int cnt=IfmSearchUSBDevices();
    if (cnt <= 0) {
        fprintf(stderr, "%s:%s: IfmSearchUSBDevices: SIOS interferometer not found on USB bus\n", 
           driverName, functionName);
        return asynError;
    }
   
    dev_ = -1;

    for (int i=0; i < cnt; i++) {
        if (IfmUSBDeviceSerial(i) == serialNum) {
            dev_ = i;
            break;
        } else {
            fprintf(stderr, "%s:%s: Found non-matching serial %6.6d\n", 
               driverName, functionName, IfmUSBDeviceSerial(i));
        }
    }

    if (dev_ == -1) {
        fprintf(stderr, "%s:%s: Unable to find device with serial number: %6.6d\n", 
           driverName, functionName, serialNum);
        return asynError;
    }

    dev_ = IfmOpenUSB(dev_);
    if (dev_ < 0) {
        fprintf(stderr, "%s:%s: IfmOpenUSB error %d = %s\n", 
           driverName, functionName, dev_, getSiosErrorString(dev_));
        return asynError;
    }

    printf("%s:%s: Found serial %6.6d deviceNum=%d\n", driverName, functionName, serialNum, dev_);
    connected_ = true;
    return asynSuccess;
}

SIOSifmDriver::SIOSifmDriver(const char *portName) 
   : asynPortDriver(portName, 
                    1, /* maxAddr */ 
                    NUM_SIOS_PARAMS,
                    asynInt32Mask | asynFloat64Mask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags.  Single device, but it blocks during acquisition */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0) /* Default stack size*/    
{
    asynStatus status;
    const char *functionName = "SIOSifmDriver";

    if (!addDriverToList(portName, this)) {
        return;
    }

    connected_ = false;
    dev_ = -1;
    flags_ = 0;

    createParam(P_STR_Run,                asynParamInt32,         &P_Run);
    createParam(P_STR_Mode,               asynParamInt32,         &P_Mode);
    createParam(P_STR_Channel1,           asynParamFloat64,       &P_Channel[0]);
    createParam(P_STR_Channel2,           asynParamFloat64,       &P_Channel[1]);
    createParam(P_STR_Channel3,           asynParamFloat64,       &P_Channel[2]);
    createParam(P_STR_Channel4,           asynParamFloat64,       &P_Channel[3]);
    createParam(P_STR_MeasureLength,      asynParamInt32,         &P_MeasureLength);
    createParam(P_STR_MeasureSinCos,      asynParamInt32,         &P_MeasureSinCos);
    createParam(P_STR_MeasureCircle,      asynParamInt32,         &P_MeasureCircle);
    createParam(P_STR_TestPattern,        asynParamInt32,         &P_TestPattern);
    createParam(P_STR_ClockCounter,       asynParamInt32,         &P_ClockCounter);
    createParam(P_STR_SampleCounter,      asynParamInt32,         &P_SampleCounter);
    createParam(P_STR_BeamBreak,          asynParamInt32,         &P_BeamBreak);
    createParam(P_STR_MeasureChannel1,    asynParamInt32,         &P_MeasureChannel[0]);
    createParam(P_STR_MeasureChannel2,    asynParamInt32,         &P_MeasureChannel[1]);
    createParam(P_STR_MeasureChannel3,    asynParamInt32,         &P_MeasureChannel[2]);
    createParam(P_STR_MeasureChannel4,    asynParamInt32,         &P_MeasureChannel[3]);
    createParam(P_STR_RawMode,            asynParamInt32,         &P_RawMode);
    createParam(P_STR_FilterUser,         asynParamInt32,         &P_FilterUser);
    createParam(P_STR_OutputWordRate,     asynParamInt32,         &P_OutputWordRate);
    createParam(P_STR_Zero,               asynParamInt32,         &P_Zero);
    createParam(P_STR_AverageAmount,      asynParamInt32,         &P_AverageAmount);
    createParam(P_STR_BeamBroken1,        asynParamInt32,         &P_BeamBroken[0]);
    createParam(P_STR_BeamBroken2,        asynParamInt32,         &P_BeamBroken[1]);
    createParam(P_STR_BeamBroken3,        asynParamInt32,         &P_BeamBroken[2]);
    createParam(P_STR_BeamBroken4,        asynParamInt32,         &P_BeamBroken[3]);
    createParam(P_STR_LaserUnstable1,     asynParamInt32,         &P_LaserUnstable[0]);
    createParam(P_STR_LaserUnstable2,     asynParamInt32,         &P_LaserUnstable[1]);
    createParam(P_STR_LaserUnstable3,     asynParamInt32,         &P_LaserUnstable[2]);
    createParam(P_STR_LaserUnstable4,     asynParamInt32,         &P_LaserUnstable[3]);
    createParam(P_STR_LostValues,         asynParamInt32,         &P_LostValues);
    createParam(P_STR_SignalQuality1,     asynParamInt32,         &P_SignalQuality[0]);
    createParam(P_STR_SignalQuality2,     asynParamInt32,         &P_SignalQuality[1]);
    createParam(P_STR_SignalQuality3,     asynParamInt32,         &P_SignalQuality[2]);
    createParam(P_STR_SignalQuality4,     asynParamInt32,         &P_SignalQuality[3]);
    createParam(P_STR_AirPressure1,       asynParamFloat64,       &P_AirPressure[0]);
    createParam(P_STR_AirPressure2,       asynParamFloat64,       &P_AirPressure[1]);
    createParam(P_STR_AirPressure3,       asynParamFloat64,       &P_AirPressure[2]);
    createParam(P_STR_AirPressure4,       asynParamFloat64,       &P_AirPressure[3]);
    createParam(P_STR_AirRefraction1,     asynParamFloat64,       &P_AirRefraction[0]);
    createParam(P_STR_AirRefraction2,     asynParamFloat64,       &P_AirRefraction[1]);
    createParam(P_STR_AirRefraction3,     asynParamFloat64,       &P_AirRefraction[2]);
    createParam(P_STR_AirRefraction4,     asynParamFloat64,       &P_AirRefraction[3]);
    createParam(P_STR_Humidity1,          asynParamFloat64,       &P_Humidity[0]);
    createParam(P_STR_Humidity2,          asynParamFloat64,       &P_Humidity[1]);
    createParam(P_STR_Humidity3,          asynParamFloat64,       &P_Humidity[2]);
    createParam(P_STR_Humidity4,          asynParamFloat64,       &P_Humidity[3]);
    createParam(P_STR_Temperature1,       asynParamFloat64,       &P_Temperature[0]);
    createParam(P_STR_Temperature2,       asynParamFloat64,       &P_Temperature[1]);
    createParam(P_STR_Temperature3,       asynParamFloat64,       &P_Temperature[2]);
    createParam(P_STR_Temperature4,       asynParamFloat64,       &P_Temperature[3]);
    createParam(P_STR_VaporPressure1,     asynParamFloat64,       &P_VaporPressure[0]);
    createParam(P_STR_VaporPressure2,     asynParamFloat64,       &P_VaporPressure[1]);
    createParam(P_STR_VaporPressure3,     asynParamFloat64,       &P_VaporPressure[2]);
    createParam(P_STR_VaporPressure4,     asynParamFloat64,       &P_VaporPressure[3]);
    createParam(P_STR_Wavelength1,        asynParamFloat64,       &P_Wavelength[0]);
    createParam(P_STR_Wavelength2,        asynParamFloat64,       &P_Wavelength[1]);
    createParam(P_STR_Wavelength3,        asynParamFloat64,       &P_Wavelength[2]);
    createParam(P_STR_Wavelength4,        asynParamFloat64,       &P_Wavelength[3]);
    createParam(P_STR_DeadPath1,          asynParamInt32,         &P_DeadPath[0]);
    createParam(P_STR_DeadPath2,          asynParamInt32,         &P_DeadPath[1]);
    createParam(P_STR_DeadPath3,          asynParamInt32,         &P_DeadPath[2]);
    createParam(P_STR_DeadPath4,          asynParamInt32,         &P_DeadPath[3]);
    createParam(P_STR_StdDev1,            asynParamFloat64,       &P_StdDev[0]);
    createParam(P_STR_StdDev2,            asynParamFloat64,       &P_StdDev[1]);
    createParam(P_STR_StdDev3,            asynParamFloat64,       &P_StdDev[2]);
    createParam(P_STR_StdDev4,            asynParamFloat64,       &P_StdDev[3]);

    /* Set the initial values of some parameters */
    for (int i=0; i < SIOS_MAX_CHANNELS; i++) {
        setDoubleParam(P_Channel[i],        -1.0);
        setIntegerParam(P_MeasureChannel[i],   1);
        setIntegerParam(P_SignalQuality[i],    0);
        setDoubleParam(P_AirPressure[i],     0.0);
        setDoubleParam(P_AirRefraction[i],   0.0);
        setDoubleParam(P_Humidity[i],        0.0);
        setDoubleParam(P_Temperature[i],     0.0);
        setDoubleParam(P_VaporPressure[i],   0.0);
        setDoubleParam(P_Wavelength[i],      0.0);
        setDoubleParam(P_StdDev[i],          0.0);
        setIntegerParam(P_DeadPath[i],       0.0);
        setIntegerParam(P_BeamBroken[i],       0);
        setIntegerParam(P_LaserUnstable[i],    0);
    }

    setIntegerParam(P_Run,               0);
    setIntegerParam(P_Mode,              SIOS_MODE_CONTINUOUS);
    setIntegerParam(P_MeasureLength,     1);
    setIntegerParam(P_MeasureSinCos,     0);
    setIntegerParam(P_MeasureCircle,     0);
    setIntegerParam(P_TestPattern,       0);
    setIntegerParam(P_ClockCounter,      0);
    setIntegerParam(P_SampleCounter,     0);
    setIntegerParam(P_BeamBreak,         1); //NOTE: beambreak off internally (swapped in setflags) (swapped in setflags)
    setIntegerParam(P_RawMode,           0);
    setIntegerParam(P_FilterUser,        0);
    setIntegerParam(P_OutputWordRate,  100);
    setIntegerParam(P_Zero,              0);
    setIntegerParam(P_AverageAmount,    10);
    setIntegerParam(P_LostValues,        0);

    /* Create the thread that computes the waveforms in the background */
    status = (asynStatus)(epicsThreadCreate("SIOSifmDriverTask",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::pollTask,
                          this) == NULL);

    if (status) {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
}

SIOSifmDriver::~SIOSifmDriver() {
    //asynPrint(pasynUser, ASYN_TRACE_FLOW, 
    //      "%s:destructor called\n", 
    //      driverName);

    if (connected_) {
    //    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
    //          "%s:destructor: closing device %d\n", 
    //          driverName, dev_);
        IfmCloseDevice(dev_);
    }
}

bool addDriverToList(const char *portName, SIOSifmDriver *drv) {
    if (!SIOSifmDriverListInitialized) {
        SIOSifmDriverListInitialized = 1;
        ellInit(&SIOSifmDriverList);
    } else if (findDriverByPortName(portName) != NULL) {
        fprintf(stderr, "ERROR: Re-using portName=%s\n", portName);
        return false;
    }

    SIOSifmNode *pNode = (SIOSifmNode*)calloc(1, sizeof(SIOSifmNode));
    pNode->portName = epicsStrDup(portName);
    pNode->pDriver = drv;
    ellAdd(&SIOSifmDriverList, (ELLNODE*)pNode);
    return true;
}

SIOSifmDriver* findDriverByPortName(const char *portName) {
    SIOSifmNode *pNode;
    static const char *functionName = "findDriverByPortName";

    if (!portName)
        return NULL;

    // Find this Driver
    if (!SIOSifmDriverListInitialized) {
        printf("%s:%s: ERROR, Driver list not initialized\n",
            driverName, functionName);
        return NULL;
    }

    pNode = (SIOSifmNode*)ellFirst(&SIOSifmDriverList);
    while(pNode) {
        if (!strcmp(pNode->portName, portName)) {
            return pNode->pDriver;
        }
        pNode = (SIOSifmNode*)ellNext((ELLNODE*)pNode);
    }

    printf("%s:%s: Driver on port %s not found\n",
        driverName, functionName, portName);
    return NULL;
}

/* Configuration routine.  Called directly, or from the iocsh function below */
extern "C" {

/** EPICS iocsh callable function to call constructor for the SIOSifmDriver class.
  * \param[in] portName The name of the asyn port driver to be created.
  */
int SIOSifmDriverConfigure(const char *portName)
{
    static const char* functionName = "SIOSifmDriverConfigure";

    if (!portName) {
        fprintf(stderr, "Must specify port name\n");
        return asynError;
    }

    int error=0;
    IfmSetOption(IFM_OPTION_DEBUGFILES, false);

    error = IfmInit();
    if (error) {
        fprintf(stderr, "%s:%s: ERROR: Initializing port %s %d = %s\n", 
            driverName, functionName, portName, error, getSiosErrorString(error));
        return(asynError);
    }

    new SIOSifmDriver(portName);
    return(asynSuccess);
}

/** EPICS iocsh callable function to connect to the interferometer via USB
  * Either the device number or serial number must be set.
  *
  * \param[in] portName The name of the associated asyn port
  * \param[in] deviceNum The device number to connect to (-1 to ignore)
  * \param[in] serialNum The serial number to connect to (-1 to ignore)
  */
int SIOSifmConnectUSB(const char *portName, int deviceNum, int serialNum) {
    static const char *functionName = "SIOSifmConnectUSB";
    SIOSifmDriver *driver=NULL;

    if (!(driver = findDriverByPortName(portName))) {
        printf("Port name not created: %s\n", portName);
        return asynError;
    }

    if (deviceNum >= 0) {
        return driver->connectUSBbyDeviceNumber(deviceNum);
    } else if (serialNum >= 0) {
        return driver->connectUSBbySerialNumber(serialNum);
    } else {
        fprintf(stderr, "%s:%s: ERROR: %s must specify either deviceNum or serialNum (>= 0)\n", 
            driverName, functionName, portName);
        return asynError;
    }
}

/* EPICS iocsh shell commands */
// -- SIOSifmDriverConfigure(portName)
static const iocshArg initArg0 = { "portName",iocshArgString};
static const iocshArg * const initArgs[] = { &initArg0 };
                                            
static const iocshFuncDef initFuncDef = {"SIOSifmDriverConfigure",1,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    SIOSifmDriverConfigure(args[0].sval);
}

// -- SIOSifmConnectUSB(device_num, serial_num)
// Uses device_num or serial_num if non-negative
//
static const iocshArg ConnectUSBArg0 = { "portName", iocshArgString};
static const iocshArg ConnectUSBArg1 = { "device_num", iocshArgInt};
static const iocshArg ConnectUSBArg2 = { "serial_num", iocshArgInt};
static const iocshArg * const ConnectUSBArgs[] = { &ConnectUSBArg0, 
                                                   &ConnectUSBArg1,
                                                   &ConnectUSBArg2 };
                                            
static const iocshFuncDef ConnectUSBFuncDef = {"SIOSifmConnectUSB",3,ConnectUSBArgs};
static void ConnectUSBCallFunc(const iocshArgBuf *args)
{
    SIOSifmConnectUSB(args[0].sval, args[1].ival, args[2].ival);
}

// -- SIOSifmDriverRegister()
void SIOSifmDriverRegister(void)
{
    iocshRegister(&initFuncDef, initCallFunc);
    iocshRegister(&ConnectUSBFuncDef, ConnectUSBCallFunc);
}

epicsExportRegistrar(SIOSifmDriverRegister);

}

