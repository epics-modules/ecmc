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
USR_CXXFLAGS += -std=c++17

ifneq ($(filter linux-%,$(T_A)),)
ETHERLAB ?= /opt/etherlab
USR_INCLUDES += -I$(ETHERLAB)/include
USR_CFLAGS += -fPIC
USR_LDFLAGS += -L$(ETHERLAB)/lib
USR_LDFLAGS += -lethercat
USR_LDFLAGS += -Wl,-rpath=$(ETHERLAB)/lib
else
USR_INCLUDES += -I$(SDKTARGETSYSROOT)/usr/include/etherlab
USR_CFLAGS   += -fPIC
USR_LDFLAGS  += -L$(SDKTARGETSYSROOT)/usr/lib/etherlab
USR_LDFLAGS  += -lethercat
USR_LDFLAGS  += -Wl,-rpath=$(SDKTARGETSYSROOT)/usr/lib/etherlab
endif

USR_INCLUDES += -I$(RUCKIG)/include/ruckig
USR_LDFLAGS += -L$(RUCKIG)/build -lruckig
USR_LDFLAGS += -Wl,-rpath,'$(RUCKIG)/build'

# ecmcioc.dbd will be created and installed
DBD += ecmcIoc.dbd

# opcuaIoc.dbd will be made up from these files:
ecmcIoc_DBD += base.dbd
ecmcIoc_DBD += ecmcController.dbd
ecmcIoc_DBD += ecmcMotorRecordSupport.dbd
ecmcIoc_DBD += asyn.dbd
ecmcIoc_DBD += motorSupport.dbd

# Add all the support libraries needed by this IOC
ecmcIoc_LIBS += asyn
ecmcIoc_LIBS += ecmc
ecmcIoc_LIBS += motor
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
