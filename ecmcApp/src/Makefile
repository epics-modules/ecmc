TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = ecmc
# ecmc.dbd will be created and installed
DBD += ecmc.dbd

# ecmc.dbd will be made up from these files:
ecmc_DBD += base.dbd

# Include dbd files from all support applications:
#ecmc_DBD += xxx.dbd
ecmc_DBD += asyn.dbd
ecmc_DBD += drvAsynIPPort.dbd
ecmc_DBD += motorSupport.dbd
ecmc_DBD += ecmcController.dbd

# Add source files
ecmc_SRCS += ecmcController.cpp
ecmc_SRCS += ecmcAxis.cpp
ecmc_SRCS += drvAsynECMCPort.cpp
ecmc_SRCS += cmd_EAT.c
ecmc_SRCS += hw_motor.c
ecmc_SRCS += cmd.c

# Add all the support libraries needed by this IOC
ecmc_LIBS += asyn
ecmc_LIBS += motor

# ecmc_registerRecordDeviceDriver.cpp derives from ecmc.dbd
ecmc_SRCS += ecmc_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
ecmc_SRCS_DEFAULT += ecmcMain.cpp
ecmc_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#ecmc_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
ecmc_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

