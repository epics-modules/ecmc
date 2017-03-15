EXCLUDE_VERSIONS=3.14.12.5

include ${EPICS_ENV_PATH}/module.Makefile

USR_DEPENDENCIES = asyn,4.27.0

PROJECT=ecmc
# Temporally removed to speed up 
EXCLUDE_ARCHS += eldk

SOURCES = \
  ecmcApp/src/cmd.c \
  ecmcApp/src/cmd_EAT.c \
  ecmcApp/src/drvAsynECMCPort.cpp \
  ecmcApp/src/ecmcAxisBase.cpp \
  ecmcApp/src/ecmcAxisReal.cpp \
  ecmcApp/src/ecmcAxisVirt.cpp \
  ecmcApp/src/ecmcCommandTransform.cpp \
  ecmcApp/src/ecmcDriveBase.cpp \
  ecmcApp/src/ecmcDriveStepper.cpp \
  ecmcApp/src/ecmcDriveDS402.cpp \
  ecmcApp/src/ecmcEc.cpp \
  ecmcApp/src/ecmcEcEntry.cpp \
  ecmcApp/src/ecmcEcPdo.cpp \
  ecmcApp/src/ecmcEcSDO.cpp \
  ecmcApp/src/ecmcEcSlave.cpp \
  ecmcApp/src/ecmcEcSyncManager.cpp \
  ecmcApp/src/ecmcEcEntryLink.cpp \
  ecmcApp/src/ecmcEncoder.cpp \
  ecmcApp/src/ecmcError.cpp \
  ecmcApp/src/ecmcFilter.cpp \
  ecmcApp/src/ecmcMain.cpp \
  ecmcApp/src/ecmcMasterSlaveData.cpp \
  ecmcApp/src/ecmcMasterSlaveIF.cpp \
  ecmcApp/src/ecmcMonitor.cpp \
  ecmcApp/src/ecmcPIDController.cpp \
  ecmcApp/src/ecmcSequencer.cpp \
  ecmcApp/src/ecmcTrajectoryTrapetz.cpp \
  ecmcApp/src/ecmcEvent.cpp \
  ecmcApp/src/ecmcEventConsumer.cpp \
  ecmcApp/src/ecmcDataRecorder.cpp \
  ecmcApp/src/ecmcDataStorage.cpp \
  ecmcApp/src/ecmcCommandList.cpp \
  ecmcApp/src/ecmcAxisData.cpp \
  ecmcApp/src/hw_motor.cpp \

TEMPLATES = \
  ecmcApp/Db/ecmcGeneral.template \
  ecmcApp/Db/ecmcGeneral.db \
  ecmcApp/Db/elGenericAnalog.template \
  ecmcApp/Db/elGenericDigital.template \
  ecmcApp/Db/DUT_AxisStatus_v0_01.db \
  ecmcApp/Db/FB_DriveVirtual_v1_01.db \
  ecmcApp/Db/expression.db \
  ecmcApp/Db/expressionAR.db \
  ecmcApp/Db/dataStorage.db \
  ecmcApp/Db/dataRecorder.db \
  ecmcApp/Db/event.db \
  ecmcApp/Db/ethercat.db \








