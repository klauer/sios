#!../../bin/linux-x86/SIOSifm

< envPaths

## Register all support components
dbLoadDatabase("../../dbd/SIOSifm.dbd",0,0)
SIOSifm_registerRecordDeviceDriver(pdbbase) 

SIOSifmDriverConfigure("port1")
#SIOSifmConnectUSB("port1", 0, -1)
SIOSifmConnectUSB("port1", -1, 0)

## Load record instances
dbLoadRecords("../../db/SIOSifm.db","P=SIOS:,R=one:,PORT=port1,ADDR=0,TIMEOUT=1")

iocInit()

## Start any sequence programs
#seq sncSIOSifm,"user=nanopos"
