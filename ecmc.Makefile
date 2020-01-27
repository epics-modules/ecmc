#
#  Copyright (c) 2019            Jeong Han Lee
#  Copyright (c) 2017 - 2019     European Spallation Source ERIC
#
#  The program is free software: you can redistribute
#  it and/or modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation, either version 2 of the
#  License, or any newer version.
#
#  This program is distributed in the hope that it will be useful, but WITHOUT
#  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
#  more details.
#
#  You should have received a copy of the GNU General Public License along with
#  this program. If not, see https://www.gnu.org/licenses/gpl-2.0.txt
#
# 
# Author  : Jeong Han Lee
# email   : jeonghan.lee@gmail.com
# Date    : Tuesday, September 17 11:30:53 CEST 2019
# version : 0.0.7
#

where_am_I := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
include $(E3_REQUIRE_TOOLS)/driver.makefile
include $(E3_REQUIRE_CONFIG)/DECOUPLE_FLAGS

ifneq ($(strip $(ASYN_DEP_VERSION)),)
asyn_VERSION=$(ASYN_DEP_VERSION)
endif

ifneq ($(strip $(EXPRTK_DEP_VERSION)),)
exprtk_VERSION=$(EXPRTK_DEP_VERSION)
endif

ifneq ($(strip $(MOTOR_DEP_VERSION)),)
motor_VERSION=$(MOTOR_DEP_VERSION)
endif

EXCLUDE_ARCHS += linux-ppc64e6500 
EXCLUDE_ARCHS += linux-corei7-poky


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
USR_LDFLAGS  += -lstdc++
endif


APP:=devEcmcSup
APPSRC:=$(APP)

APPSRC_ETHERCAT:=$(APPSRC)/ethercat
APPSRC_MOTION:=$(APPSRC)/motion
APPSRC_COM:=$(APPSRC)/com
APPSRC_MAIN:=$(APPSRC)/main
APPSRC_PLC:=$(APPSRC)/plc
APPSRC_MISC:=$(APPSRC)/misc
APPSRC_MOTOR:=$(APPSRC)/motor

USR_INCLUDES += -I$(where_am_I)$(APPSRC)

SOURCES += $(APPSRC_MAIN)/ecmcGeneral.cpp 
SOURCES += $(APPSRC_MAIN)/ecmcError.cpp 
SOURCES += $(APPSRC_MAIN)/ecmcMainThread.cpp

SOURCES += $(APPSRC_COM)/ecmcCom.cpp
SOURCES += $(APPSRC_COM)/ecmcOctetIF.c
SOURCES += $(APPSRC_COM)/ecmcCmdParser.c 
SOURCES += $(APPSRC_COM)/ecmcAsynPortDriver.cpp 
SOURCES += $(APPSRC_COM)/ecmcAsynPortDriverUtils.cpp 
SOURCES += $(APPSRC_COM)/ecmcAsynDataItem.cpp 

SOURCES += $(APPSRC_MOTION)/ecmcMotion.cpp 
SOURCES += $(APPSRC_MOTION)/ecmcAxisBase.cpp 
SOURCES += $(APPSRC_MOTION)/ecmcAxisReal.cpp 
SOURCES += $(APPSRC_MOTION)/ecmcAxisVirt.cpp 
SOURCES += $(APPSRC_MOTION)/ecmcDriveBase.cpp 
SOURCES += $(APPSRC_MOTION)/ecmcDriveStepper.cpp 
SOURCES += $(APPSRC_MOTION)/ecmcDriveDS402.cpp 
SOURCES += $(APPSRC_MOTION)/ecmcEncoder.cpp 
SOURCES += $(APPSRC_MOTION)/ecmcFilter.cpp 
SOURCES += $(APPSRC_MOTION)/ecmcMonitor.cpp 
SOURCES += $(APPSRC_MOTION)/ecmcPIDController.cpp
SOURCES += $(APPSRC_MOTION)/ecmcAxisSequencer.cpp
SOURCES += $(APPSRC_MOTION)/ecmcTrajectoryTrapetz.cpp 
SOURCES += $(APPSRC_MOTION)/ecmcAxisData.cpp

SOURCES += $(APPSRC_ETHERCAT)/ecmcEthercat.cpp 
SOURCES += $(APPSRC_ETHERCAT)/ecmcEc.cpp 
SOURCES += $(APPSRC_ETHERCAT)/ecmcEcEntry.cpp 
SOURCES += $(APPSRC_ETHERCAT)/ecmcEcPdo.cpp 
SOURCES += $(APPSRC_ETHERCAT)/ecmcEcSDO.cpp 
SOURCES += $(APPSRC_ETHERCAT)/ecmcEcSlave.cpp 
SOURCES += $(APPSRC_ETHERCAT)/ecmcEcSyncManager.cpp 
SOURCES += $(APPSRC_ETHERCAT)/ecmcEcEntryLink.cpp 
SOURCES += $(APPSRC_ETHERCAT)/ecmcAsynLink.cpp 
SOURCES += $(APPSRC_ETHERCAT)/ecmcEcMemMap.cpp

SOURCES += $(APPSRC_MISC)/ecmcMisc.cpp 
SOURCES += $(APPSRC_MISC)/ecmcEvent.cpp 
SOURCES += $(APPSRC_MISC)/ecmcEventConsumer.cpp 
SOURCES += $(APPSRC_MISC)/ecmcDataRecorder.cpp 
SOURCES += $(APPSRC_MISC)/ecmcDataStorage.cpp 
SOURCES += $(APPSRC_MISC)/ecmcCommandList.cpp 

SOURCES += $(APPSRC_PLC)/ecmcPLC.cpp
SOURCES += $(APPSRC_PLC)/ecmcPLCTask.cpp
SOURCES += $(APPSRC_PLC)/ecmcPLCDataIF.cpp
SOURCES += $(APPSRC_PLC)/ecmcPLCMain.cpp

SOURCES += $(APPSRC_MOTOR)/ecmcMotorRecordController.cpp
SOURCES += $(APPSRC_MOTOR)/ecmcMotorRecordAxis.cpp

SOURCES += gitversion.c

DBDS    += $(APPSRC_COM)/ecmcController.dbd
DBDS    += $(APPSRC_MOTOR)/ecmcMotorRecordSupport.dbd

ecmcEcMemMap$(DEP): gitversion.c

gitversion.c: 
	$(QUIET)$(RM) $@
	$(QUIET)sh $(where_am_I)tools/gitversion.sh $@


.PHONY: vlibs db

db:

vlibs:
