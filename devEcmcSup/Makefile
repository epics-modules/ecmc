#************************************************************************
# Copyright (c) 2019 European Spallation Source ERIC
# ecmc is distributed subject to a Software License Agreement found
# in file LICENSE that is included with this distribution. 
#
# Author: Jeong Han Lee <jeonghan.lee@gmail.com>
#
#*************************************************************************

TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================


ECMC = $(TOP)/devEcmcSup

LIBRARY_IOC += ecmc

# Ubuntu needs the following ldflags
USR_LDFLAGS += -Wl,--no-as-needed
USR_LDFLAGS += -lstdc++

ifeq ($(T_A),linux-x86_64)
# Assume that the etherlab user library is done via
# https://github.com/icshwi/etherlabmaster
USR_INCLUDES += -I/opt/etherlab/include
USR_CFLAGS += -fPIC
USR_LDFLAGS += -L /opt/etherlab/lib
USR_LDFLAGS += -lethercat
USR_LDFLAGS += -Wl,-rpath=/opt/etherlab/lib
else
# Assume that the etherlab user library is done via
# Yocto ESS Linux bb recipe
USR_INCLUDES += -I$(SDKTARGETSYSROOT)/usr/include/etherlab
USR_CFLAGS   += -fPIC
USR_LDFLAGS  += -L $(SDKTARGETSYSROOT)/usr/lib/etherlab
USR_LDFLAGS  += -lethercat
USR_LDFLAGS  += -Wl,-rpath=$(SDKTARGETSYSROOT)/usr/lib/etherlab
endif


SRC_DIRS  += $(ECMC)/plc
ecmc_SRCS += ecmcPLC.cpp
ecmc_SRCS += ecmcPLCTask.cpp
ecmc_SRCS += ecmcPLCDataIF.cpp
ecmc_SRCS += ecmcPLCMain.cpp


SRC_DIRS  += $(ECMC)/misc
ecmc_SRCS += ecmcMisc.cpp 
ecmc_SRCS += ecmcEvent.cpp 
ecmc_SRCS += ecmcEventConsumer.cpp 
ecmc_SRCS += ecmcDataRecorder.cpp 
ecmc_SRCS += ecmcDataStorage.cpp 
ecmc_SRCS += ecmcCommandList.cpp 

SRC_DIRS  += $(ECMC)/main
ecmc_SRCS += ecmcGeneral.cpp 
ecmc_SRCS += ecmcError.cpp 
ecmc_SRCS += ecmcMainThread.cpp
ecmc_SRCS += gitversion.c



SRC_DIRS  += $(ECMC)/ethercat
ecmc_SRCS += ecmcEthercat.cpp 
ecmc_SRCS += ecmcEc.cpp 
ecmc_SRCS += ecmcEcEntry.cpp 
ecmc_SRCS += ecmcEcPdo.cpp 
ecmc_SRCS += ecmcEcSDO.cpp 
ecmc_SRCS += ecmcEcSlave.cpp 
ecmc_SRCS += ecmcEcSyncManager.cpp 
ecmc_SRCS += ecmcEcEntryLink.cpp 
ecmc_SRCS += ecmcAsynLink.cpp 
ecmc_SRCS += ecmcEcMemMap.cpp


SRC_DIRS  += $(ECMC)/com
DBD       += ecmcController.dbd
ecmc_SRCS += ecmcCom.cpp
ecmc_SRCS += ecmcOctetIF.c
ecmc_SRCS += ecmcCmdParser.c 
ecmc_SRCS += ecmcAsynPortDriver.cpp 
ecmc_SRCS += ecmcAsynPortDriverUtils.cpp 
ecmc_SRCS += ecmcAsynDataItem.cpp 

SRC_DIRS  += $(ECMC)/motion
ecmc_SRCS += ecmcMotion.cpp 
ecmc_SRCS += ecmcAxisBase.cpp 
ecmc_SRCS += ecmcAxisReal.cpp 
ecmc_SRCS += ecmcAxisVirt.cpp 
ecmc_SRCS += ecmcDriveBase.cpp 
ecmc_SRCS += ecmcDriveStepper.cpp 
ecmc_SRCS += ecmcDriveDS402.cpp 
ecmc_SRCS += ecmcEncoder.cpp 
ecmc_SRCS += ecmcFilter.cpp 
ecmc_SRCS += ecmcMonitor.cpp 
ecmc_SRCS += ecmcPIDController.cpp
ecmc_SRCS += ecmcAxisSequencer.cpp
ecmc_SRCS += ecmcTrajectoryTrapetz.cpp 
ecmc_SRCS += ecmcAxisData.cpp


ecmc_LIBS += exprtkSupport
ecmc_LIBS += $(EPICS_BASE_IOC_LIBS)



include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

gitversion.c:
	@$(RM) $@
	@sh $(TOP)/tools/gitversion.sh $@
