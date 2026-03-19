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

USR_LDFLAGS += -L$(RUCKIG)/build/ -lruckig
USR_LDFLAGS += -Wl,-rpath,'$(RUCKIG)/build/'

SRC_DIRS += ${ECMC}/main
SRC_DIRS += ${ECMC}/ethercat
SRC_DIRS += ${ECMC}/com
SRC_DIRS += ${ECMC}/motion
SRC_DIRS += ${ECMC}/motor
SRC_DIRS += ${ECMC}/plc
SRC_DIRS += ${ECMC}/plugin
SRC_DIRS += ${ECMC}/misc

ecmc_SRCS += $(notdir $(foreach d,$(SRC_DIRS),$(wildcard $d/*.c)))
ecmc_SRCS += $(notdir $(foreach d,$(SRC_DIRS),$(wildcard $d/*.cpp)))
DBD += $(notdir $(foreach d,$(SRC_DIRS),$(wildcard $d/*.dbd)))

$(info ===== ECMC_SRCS =====)
$(info $(ecmc_SRCS))
$(info =====================)

ecmc_LIBS += exprtkSupport
ecmc_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

gitversion.c:
	@$(RM) $@
	@sh $(TOP)/tools/gitversion.sh $@
