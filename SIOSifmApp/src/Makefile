TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = SIOSifm
# SIOSifm.dbd will be created and installed
DBD += SIOSifm.dbd

# SIOSifm.dbd will be made up from these files:
SIOSifm_DBD += base.dbd

# Include dbd files from all support applications:
SIOSifm_DBD += SIOSifmSupport.dbd asyn.dbd

# Add all the support libraries needed by this IOC
SIOSifm_LIBS += asyn siosifm

# SIOSifm_registerRecordDeviceDriver.cpp derives from SIOSifm.dbd
SIOSifm_SRCS += SIOSifm_registerRecordDeviceDriver.cpp
SIOSifm_SRCS += SIOSifmDriver.cpp

# Build the main IOC entry point on workstation OSs.
SIOSifm_SRCS_DEFAULT += SIOSifmMain.cpp
SIOSifm_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#SIOSifm_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
SIOSifm_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

