
#ifndef SIOSIFMDLL_H
#define SIOSIFMDLL_H


#ifdef _MSC_VER
#    pragma pack( push, packing )
#    pragma pack( 1 )
#    define PACK_STRUCT
typedef __int64 int64;
typedef __uint64 uint64;
#elif defined( __GNUC__ )
#    define PACK_STRUCT    __attribute__((packed))
typedef long long int64;
typedef unsigned long long uint64;
#endif

#ifdef _WIN32

#ifdef DLLFUNC
#define DLLFUNC extern "C" __declspec(dllexport)
#else
#define DLLFUNC extern "C" __declspec(dllimport)
#endif

#else
#define DLLFUNC
#endif

#include <siosifmdef.h>



// Function for Initialization and closing
/*
The function IfmDLLVersion returns the version number
*/
DLLFUNC int IfmDLLVersion();

/*
The function IfmDLLVersionString returns a pointer to
a char field with the dll name and version number
*/
DLLFUNC const char *IfmDLLVersionString();


DLLFUNC void IfmSetOption(int option, int param1);

/* Initializes the library

*/
DLLFUNC int IfmInit();


/* Closes the library and gives free all resources
   Counterpart to IfmInit
*/
DLLFUNC void IfmClose();

// don't use !!!
// only if IFM_OPTION_POLLSELF=0 then call IfmPoll frequently
DLLFUNC int IfmPoll();

/* Open a device
   returns a handle for the device, the "devNumber"
*/
DLLFUNC int IfmOpenUSB(int uniqueId);

#ifdef _WIN32
DLLFUNC int IfmOpenCOM(int comNumber);
#else
DLLFUNC int IfmOpenCOM(const char *devName);
#endif

DLLFUNC int IfmOpenNetwork(const char *ipAddr, int port);
DLLFUNC int IfmOpenDemo(int channels);

/* Close a device which was opened with one of the IfmOpenXXX functions
*/
DLLFUNC void IfmCloseDevice(int devNumber);

/*
    The function IfmSearchUSBDevices finds the count of the devices on the USB-bus
    after a reset. This devices can be shown with the function IfmUSBDeviceSerial
*/
DLLFUNC int IfmSearchUSBDevices();

/*
 The function IfmUSBDeviceCount returns
 the count of devices on the USB-bus without a new search
*/
DLLFUNC int IfmUSBDeviceCount();

/*
  The function IfmUSBDeviceSerial returns the
 USB-serial number of the requested device; 0<=uniqueId<IfmUSBDeviceCount()
*/
DLLFUNC int IfmUSBDeviceSerial(int uniqueId);

/*
The function IfmMaxDeviceCount returns the count of maximum allowable devices
*/
DLLFUNC int IfmMaxDeviceCount();

/*
The function IfmDeviceCount returns the count of the open devices
*/
DLLFUNC int IfmDeviceCount();

// checks, if the devNumber is a valid device id
DLLFUNC int IfmDeviceValid(int devNumber);

// returns the version number of the firmware
DLLFUNC int IfmFirmwareVersion(int devNumber);
// returns the count of channels
DLLFUNC int IfmChannels(int devNumber);
// returns the type of the device, currently only RE10-cards are supported
DLLFUNC int IfmDeviceType(int devNumber);
// to which interface is the device connected ? RS232, USB od Network
DLLFUNC int IfmDeviceInterface(int devNumber);
// to ask for specific properties (defiened in siosifmdef.h)
DLLFUNC int IfmHasDeviceProperty(int devNumber, int porpertyNumber);

/*
IfmDeviceInfo returns required information, if it is available, otherwise it returns 0
Befor this function will be called, the device should be opened (IfmOpenCOM, IfmOpenUSB)
*/
DLLFUNC int IfmDeviceInfo(int devNumber, int requestedInfo);

//some of the device infos can also be set
DLLFUNC int IfmSetDeviceInfo(int devNumber, int requestedInfo, int newValue);

/*
IfmResetDevice causes the software reset of the according card
Alike after hardware reset, the device stops after this command all running processes and
breaks all connections. The USB-interface have to be reinitialized and
the device have to be reconfigured by user again.
*/
DLLFUNC void IfmResetDevice(int devNumber);


/*
The command IfmUpdateDevice causes the update-mode of the card.
Attention!!! After starting of the update-mode it is not possible without
the firmware - upgrade to set back the device to the run-mode.
A SIOS-bootloader software should be applied for the upgrading.
*/
DLLFUNC void IfmUpdateDevice(int devNumber);


/*
IfmSaveConfigDevice is command for the saving of the actual measuring settings into the flash.
After the next device reset the saved parameter will be applied
For an explanation of the measuring settings see the function IfmSetMeasurement
*/
DLLFUNC void IfmSaveConfigDevice(int devNumber);//Aktuelle Eunstellungen von Page0 in den Flash speichern

/*
IfmSetTrigger is command for the settings of trigger conditions.
This settings have to be made before the command IfmSetMeasurement
For the meaning of flags see the file siosifmdef.h
*/
DLLFUNC int IfmSetTrigger(int devNumber, unsigned int triggerMode);

/*
IfmFireTrigger requests one measuring value
*/
DLLFUNC int IfmFireTrigger(int devNumber);

/*
IfmSetFilter is command for the setting of the averaging filter one and two.
If the values avr1 or avr2 are equal 0, will be the corresponding filter deactivated.
Please note the settings of the mesurementFlags in the function IfmSetMeasurement.
For the settings of  filterFlags see siosifmdef.h
*/
DLLFUNC int IfmSetFilter(int devNumber, unsigned int filterFlags,int avg1, int avg2);


/*
IfmSetFilterCoeff is the function for the setting of the FIR-filter coefficients.
This command should be applied for every of available channels
*/
DLLFUNC int IfmSetFilterCoeff(int devNumber, int channel, double coeff);

/*
IfmGetFilterCoeff is the function for the requiring
of the FIR-filter coefficient for the corresponding device and channel.
The values will be activated by the IfmSetMeasurement function
*/
DLLFUNC double IfmGetFilterCoeff(int devNumber, int channel);

/*
Sets the notch frequency of the internal filter. A frequency of 0.0 means, that the
frequency of the reference mirror vibrator should be notched out.
This function is equivalent to IfmSetFilterCoeff but calculates the appropriate coeff based on the given frequency.
*/
DLLFUNC int IfmSetFilterNotchFrequency(int devNumber, int channel, double freq);

/*
Returns the notch frequency, which wre set with IfmSetFilterNotchFrequency.
The function delivers valid values AFTER IfmSetMeasurement was called
*/
DLLFUNC double IfmGetFilterNotchFrequency(int devNumber, int channel);


/*
Sets the "dead path" in mm for the environment correction.
The dead path is the distance from the sensor head to the zero position.
Because during this distance the laser beam is also influenced by changing in air refraction
it must be considered with the envirnonment correction even the distance plays no role in the
measurement itself.

ATTENTION!!! The dead path is taken over into the calculation with defining the Zero-Position (IfmSetToZero).
Therefore, IfmSetDeadPath must always be called before IfmSetToZero!

 */
DLLFUNC int IfmSetDeadPath(int devNumber,int channel, int deadPath);

/* IfmGetDeadpath returns the active dead path for the given channel in mm
   Because the dead path is set during IfmSetToZero and the dead path can be changed
   in the interferometer (for instance in LaserTracers) the returned value may be
   different from the value set with IfmSetDeadpath

   The dead path is always positive. A negative return value is an error number and indicates an error.

 */

DLLFUNC int IfmGetDeadPath(int devNumber, int channel);

/*
The function IfmSetMeasurementRawValues alike IfmSetMeasurement.
But the filter settings will be ignored here. It means the sampe frequency and the output word rate are equal
*/
//DLLFUNC int IfmSetMeasurementRawValues(int devNumber,unsigned int measurementFlags, int outputWordRate);

/*
The command IfmSetMeasurement is for the setting of measurement parameters like trigger conditions, filtering: on/off,
measuring value and output word rate.
For the settings of measurementFlags see siosifmdef.h
*/
DLLFUNC int IfmSetMeasurement(int devNumber,unsigned int measurementFlags, double outputWordRate);


/*
The function for the erasing of the PC-input buffer
*/
DLLFUNC int IfmClearBuffers(int devNumber);
/*
The command IfmStart is for the measurement start.
The measurement starts with the actual measurement parameter.
See commands IfmSaveConfigDevice and IfmSetMeasurement for the detailed information.
*/
DLLFUNC int IfmStart(int devNumber);

/*
The command IfmStop for the measurement stop.
*/
DLLFUNC int IfmStop(int devNumber);
/*
The function IfmValueCount return the count of values, which are in the input puffer available.
*/
DLLFUNC int IfmValueCount(int devNumber);
/*
The function IfmGetValues read out the mesuring values from the input buffer according to FIFO (first in first out) principle.
*/
DLLFUNC int IfmGetValues(int devNumber);
/*
The function IfmGetRecentValues reads out the (count-index-1) mesuring values from the input buffer.
*/
DLLFUNC int IfmGetRecentValues(int devNumber, int index);

//IfmDataStruct *IfmValue(int devNumber);

/*

*/
DLLFUNC int64 IfmRawValue(int devNumber,int channel);

/*
The function IfmLengthValue read out the length mesuring values.
It is to use directly after the function IfmGetValues
*/
DLLFUNC double IfmLengthValue(int devNumber,int channel);


DLLFUNC int IfmAuxValue(int devNumber,int channel,int valueType);


DLLFUNC unsigned int IfmStatus(int devNumber,int channel);

// Block mode
DLLFUNC int IfmSetBlockModeFilter(int devNumber, unsigned int filterFlags,int avg1, int avg2);
DLLFUNC int IfmSetBlockModeFilterCoeff(int devNumer, int channel, double coeff);
DLLFUNC int IfmSetBlockModeFilterNotchFrequency(int devNumer, int channel, double frequency);
DLLFUNC int IfmSetBlockMode(int devNumber,int measurementFlags,int triggerMode, int outputWordRate);
DLLFUNC int IfmStartBlock(int devNumber, int blockLen);
DLLFUNC int IfmIsBlockAvailable(int devNumber);
DLLFUNC int IfmCancelBlock(int devNumber);

// Functions for controling the interferometer
/*
Command IfmSetRefMirrorVibration for setting on the mirror vibration
*/
DLLFUNC int IfmSetRefMirrorVibration(int devNumber,int channel, int on);
DLLFUNC int IfmGetRefMirrorVibration(int devNumber,int channel);

DLLFUNC int IfmSetAGC(int devNumber,int channel, int on);
DLLFUNC int IfmGetAGC(int devNumber,int channel);


// enhanced functions for service tasks, do not use
DLLFUNC int IfmExtAGCStatus(int devNumber, int channel);
DLLFUNC int IfmSaveAGCStatus(int devNumber, int channel);


DLLFUNC int IfmSetToZero(int devNumber,int channelMask);

// detailierte Monitoring data in service Teil als struktur
DLLFUNC int IfmSignalQuality(int devNumber,int channel, int select);
DLLFUNC int IfmNewSignalQualityAvailable(int devNumber);

DLLFUNC int IfmWasBeamBreak(int devNumber,int channel);
DLLFUNC int IfmWasLostValues(int devNumber);
DLLFUNC int IfmWasLaserUnstable(int devNumber,int channel);

// Functions for environmental data and data depending from environmental values

// returns 1 if new (since the last call of this function) environment values are available
DLLFUNC int IfmNewEnvValuesAvailable(int devNumber);


// returns the connected/configured amount of sensors
DLLFUNC int IfmEnvSensorCount(int devNumber);

// returns a flag for the sensor on place "sensor" where sensor is 0.. IfmEnvSensorCount()-1
DLLFUNC unsigned int IfmSensorProperty(int devNumber, int sensor);

// return s the value of the sensor
DLLFUNC double IfmSensorValue(int devNumber, int sensor);

// returns the temperature which is assotiated with "channel"
DLLFUNC double IfmTemperature(int devNumber, int channel);
// returns a description, from where the value is
DLLFUNC int    IfmTemperatureFlags(int devNumber, int channel);

// returns the humidy which is assotiated with "channel"
DLLFUNC double IfmHumidity(int devNumber, int channel);
DLLFUNC int    IfmHumidityFlags(int devNumber, int channel);

// returns the air pressure which is assotiated with "channel"
DLLFUNC double IfmAirPressure(int devNumber, int channel);
DLLFUNC int    IfmAirPressureFlags(int devNumber, int channel);

// returns the water vapour pressure (calculated from humidy, temperature and air pressure) which is assotiated with "channel"
DLLFUNC double IfmWaterVapourPressure(int devNumber, int channel);

// returns the conversion coefficient (Edlen correction, calculated humidity, temperature and air pressure) which is assotiated with "channel"
DLLFUNC double IfmConversionCoeff(int devNumber, int channel);

// returns the conversion coefficient for the dead path correction
DLLFUNC double IfmDeadpathCoeff(int devNumber, int channel);

// returns the vacuum wavelength of the laser
DLLFUNC double IfmVacuumWavelength(int devNumber, int channel);

// returns the corrected wavelength
DLLFUNC double IfmWavelength(int devNumber, int channel);

// returns the air refraction index used for the environment correction of the channel
DLLFUNC double IfmAirRefraction(int devNumber, int channel);

// switch Edlen correction off, for example for measurements in vacuum or if you want to make your own correction
DLLFUNC void IfmEnableEdlenCorrection(int devNumber, int channel, int on);

// ask, if Edlen correction is enabled
DLLFUNC int IfmIsEdlenEnabled(int devNumber,int channel);

DLLFUNC void IfmSetTemperature(int devNumber, int channel, double value);
DLLFUNC void IfmSetHumidity(int devNumber, int channel, double value);
DLLFUNC void IfmSetAirPressure(int devNumber, int channel, double value);
DLLFUNC void IfmSetWaterVapourPressure(int devNumber, int channel, double value);
DLLFUNC void IfmSetConvertionCoeff(int devNumber, int channel, double value);
DLLFUNC void IfmSetVacuumWavelength(int devNumber, int channel, double value);

DLLFUNC int IfmEnvironmentFlags(int devNumber, int channel);
DLLFUNC int IfmManualEnvironment(int devNumber, int channel);  //depricated; use IfmEnvironmentFlags
DLLFUNC void IfmResetManualEnvironment(int devNumber,int channel, int mask);



// Special control commands


/*
section i2c-communication Special commands for accessing other cards

*/

/*
 Low level function for writing a block of data to a card in the interferometer
*/

DLLFUNC int IfmI2CRequestWrite(int devNumber, int i2cAddr, int ramAddr, int count, unsigned char* buffer);

/*
Check the last IfmI2CWrite operation

0 if successfull
IFM_ERROR_I2C_WRITE if an error has occured,
IFM_ERROR_I2C_INUSE if the system is yet waiting for the acknowledge
*/
DLLFUNC int IfmI2CStatus(int devNumber);

/*
  wie IfmI2CRequestWrite, blockiert aber so lange, bis R�ckmeldung von Firmware da oder Timeout
 */
DLLFUNC int IfmI2CWrite(int devNumber, int i2cAddr, int ramAddr, int count, unsigned char* buffer);



/*
 Low level function for requesting a read of a block of data from a card in the interferometer
*/
DLLFUNC int IfmI2CRequestRead(int devNumber, int i2cAddr, int ramAddr, int count);

/*
 Test whether a request to read was successfully.

\param devNumber the unique identifier for the device
\ret true if read has been successfully done, otherwise false

see IfmI2CRequestRead(int devNumber, int i2cAddr, int ramAddr, int count)
*/

DLLFUNC int IfmI2CReadReady(int devNumber);

/*! \brief Access to the internal read buffer

\param index the index in the buffer, valid from 0 to count-1
(see IfmI2CRequestRead(int devNumber, int i2cAddr, int ramAddr, int count))
*/
DLLFUNC unsigned char IfmI2CReadValue(int devNumber, int index);


/*
  like IfmI2CRequestRead but is blocking until read was successfully or an error has reported
*/
DLLFUNC int IfmI2CRead(int devNumber, int i2cAddr, int ramAddr, int count);

DLLFUNC unsigned char *IfmI2CReadBuffer(int devNumber);

// do not use; for internal operations only
DLLFUNC double IfmI2CReadOscFrequency(int devNumber, int card);
DLLFUNC int IfmI2CWriteOscFrequency(int devNumber, int card, double freq);

// Error Handling
DLLFUNC const char *IfmGetErrorString(int errorNumber);
DLLFUNC int IfmGetError(); //den ersten Fehler seit letztem Aufruf dieser Funktion

// Motorsteuerung; greift auf die I2C-Funktionen zur�ck

/*!
\brief Stops the motor and disables the power amplifiers

\param devNumber the unique identifier for the device
\param motorNumber the number of the motor
(each controller card controls two motors, 0 and 1 refers the motors of the first card, 2 and 3 the motors of a second controller card and so on)

\ret error value
*/
DLLFUNC int IfmMotorStop(int devNumber, int motorNumber);

/*!
\brief Moves the motor with a constant speed

\param devNumber the unique identifier for the device
\param motorNumber the number of the motor
(each controller card controls two motors, 0 and 1 refers the motors of the first card, 2 and 3 the motors of a second controller card and so on)
\param speed (in steps/s)

\ret error value
*/
DLLFUNC int IfmMotorMove(int devNumber,int motorNumber,int speed);


/*!
\brief Moves the motor to a new position relative to the current position

\param devNumber the unique identifier for the device
\param motorNumber the number of the motor
(each controller card controls two motors, 0 and 1 refers the motors of the first card, 2 and 3 the motors of a second controller card and so on)
\param position the amount of steps the motor should move
\param speed (in steps/s)

\ret error value
*/
DLLFUNC int IfmMotorMoveRel(int devNumber, int motorNumber,int position,int speed);

/*!
\brief Moves the motor to a new position

\param devNumber the unique identifier for the device
\param motorNumber the number of the motor
(each controller card controls two motors, 0 and 1 refers the motors of the first card, 2 and 3 the motors of a second controller card and so on)
\param position the new position
\param speed (in steps/s)

\ret error value
*/
DLLFUNC int IfmMotorMoveAbs(int devNumber, int motorNumber,int position, int speed);

/*!
\brief Sets the a position

\param devNumber the unique identifier for the device
\param motorNumber the number of the motor
(each controller card controls two motors, 0 and 1 refers the motors of the first card, 2 and 3 the motors of a second controller card and so on)
\param newPosition the new position

This function only updates the internal position counter. The motor will not move.

\ret error value
*/
DLLFUNC int IfmMotorSetPos(int devNumber, int motorNumber,int newPosition);

/*!
\brief Enables the motor

\param devNumber the unique identifier for the device
\param motorNumber the number of the motor
(each controller card controls two motors, 0 and 1 refers the motors of the first card, 2 and 3 the motors of a second controller card and so on)

\ret error value
*/
DLLFUNC int IfmMotorEnable(int devNumber, int motorNumber);

/*!
\brief Disables the motor

\param devNumber the unique identifier for the device
\param motorNumber the number of the motor
(each controller card controls two motors, 0 and 1 refers the motors of the first card, 2 and 3 the motors of a second controller card and so on)

\ret error value
*/
DLLFUNC int IfmMotorDisable(int devNumber, int motorNumber);

/*!
\brief Requests the status of the motor

\param devNumber the unique identifier for the device
\param motorNumber the number of the motor
(each controller card controls two motors, 0 and 1 refers the motors of the first card, 2 and 3 the motors of a second controller card and so on)

The transmission of the status takes up to appox. 100 milliseconds. This function requests the transmission and returns immediately.
The status itself can be read using IfmMotorStatus.

IfmMotorReadStatus requests the status and blocks until the data are transferred.

\ret error value
*/
DLLFUNC int IfmMotorRequestStatus(int devNumber, int motorNumber);

/*!
\brief Read the status of the motor

\param devNumber the unique identifier for the device
\param motorNumber the number of the motor
(each controller card controls two motors, 0 and 1 refers the motors of the first card, 2 and 3 the motors of a second controller card and so on)
\param status pointer to an integer an which the status word will be written
\param position pointer to an integer on which the current position will be written
\param application pointer to an integer on which an identifier of the motor will be written. This identifier stands for the specific application
 (gauging probe, tonometer test, NMM ...) the motor card is configured for

Before using this function the status must be requested using IfmMotorRequestStatus. If the requested status is not yet available,
the function returns with an error.

\ret error value
*/

DLLFUNC int IfmMotorStatus(int devNumber, int motorNumber,int *status, int* position, unsigned int *application);

/*!
\brief Read the status of the motor

\param devNumber the unique identifier for the device
\param motorNumber the number of the motor
(each controller card controls two motors, 0 and 1 refers the motors of the first card, 2 and 3 the motors of a second controller card and so on)
\param status pointer to an integer an which the status word will be written
\param position pointer to an integer on which the current position will be written
\param application pointer to an integer on which an identifier of the motor will be written. This identifier stands for the specific application
 (gauging probe, tonometer test, NMM ...) the motor card is configured for
\param timeout_ms the time in ms this function will wait for the status information. Should be typically above 200ms.

\ret error value
*/
DLLFUNC int IfmMotorReadStatus(int devNumber, int motorNumber,int *status, int* position, unsigned int *application,int timeout_ms);


// service functions; do not use

DLLFUNC int IfmMotorRequestControlStruct(int devNumber, int motorNumber);
DLLFUNC int IfmMotorGetControlStruct(int devNumber, int motorNumber,unsigned char *buffer);
DLLFUNC int IfmMotorSetControlStruct(int devNumber, int motorNumber,unsigned char *buffer);

DLLFUNC int IfmRequestFactoryCfg(int devNumber);
DLLFUNC int IfmRequestSystemCfg(int devNumber);
DLLFUNC int IfmGetSystemCfg(int devNumber,void *cfg);
DLLFUNC int IfmSetSystemCfg(int devNumber,void *cfg);
DLLFUNC int IfmMirrorSystemCfg(int devNumber);

DLLFUNC int IfmSendByteArray(int devNumber, void *Data);       // Bootloader
DLLFUNC int IfmSendBootAddr(int devNumber, void *Data);        // Bootloader
DLLFUNC int IfmGetAnswer(int devNumber);                       // Bootloader

DLLFUNC int IfmSpiCommand(int devNumber, int command, int val1, int val2);

// Human readable error message
DLLFUNC const char* IfmErrorMessage(int errorCode);

#endif
