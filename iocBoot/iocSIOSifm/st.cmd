#!../../bin/linux-x86/SIOSifm

< envPaths

## Register all support components
dbLoadDatabase("../../dbd/SIOSifm.dbd",0,0)
SIOSifm_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet("P", "$(P=MLL:)")
epicsEnvSet("R", "$(R=SIOS:)")
epicsEnvSet("PORT", "$(PORT=SIOS)")

SIOSifmDriverConfigure("$(PORT)")

# SIOSifmConnectUSB portName device_num serial_num
#SIOSifmConnectUSB("$(PORT)", 0, -1)

SIOSifmConnectUSB("$(PORT)", -1, 0)

## Load record instances
dbLoadRecords("../../db/SIOSifm.db","P=$(P),R=$(R),PORT=$(PORT),ADDR=0,TIMEOUT=1")

iocInit()
