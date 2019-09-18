TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = ecmcIoc

# Ubuntu needs the following ldflags
USR_LDFLAGS_Linux += -Wl,--no-as-needed

# ecmcioc.dbd will be created and installed
DBD += ecmcIoc.dbd

# opcuaIoc.dbd will be made up from these files:
ecmcIoc_DBD += base.dbd
ecmcIoc_DBD += ecmcController.dbd

# Add all the support libraries needed by this IOC
ecmcIoc_LIBS += asyn
ecmcIoc_LIBS += ecmc
ecmcIoc_LIBS += exprtkSupport

ecmcIoc_SRCS += ecmcIoc_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
ecmcIoc_SRCS_DEFAULT += ecmcIocMain.cpp
ecmcIoc_SRCS_vxWorks += -nil-

# Finally link to the EPICS Base libraries
ecmcIoc_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================
include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

