include ${EPICS_ENV_PATH}/module.Makefile

PROJECT=ecmc
USR_DEPENDENCIES = asyn,4.27
USR_DEPENDENCIES += motor,6.10.5-ESS

# Temporally removed to speed up 
EXCLUDE_ARCHS += eldk

SOURCES = \
  ecmcApp/src/cmd.c \
  ecmcApp/src/cmd_EAT.c \
  ecmcApp/src/drvAsynECMCPort.cpp \
  ecmcApp/src/ecmcAxis.cpp \
  ecmcApp/src/ecmcController.cpp \
  ecmcApp/src/ecmcMain.cpp \
  ecmcApp/src/ecmcAxisBase.cpp \
  ecmcApp/src/ecmcAxisEncoder.cpp \
  ecmcApp/src/ecmcAxisReal.cpp \
  ecmcApp/src/ecmcAxisTraj.cpp \
  ecmcApp/src/ecmcAxisVirt.cpp \
  ecmcApp/src/ecmcCommandTransform.cpp \
  ecmcApp/src/ecmcDriveBase.cpp \
  ecmcApp/src/ecmcDriveStepper.cpp \
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
  ecmcApp/src/ecmcTrajectory.cpp \
  ecmcApp/src/ecmcTransform.cpp \
  ecmcApp/src/ecmcEvent.cpp \
  ecmcApp/src/ecmcEventConsumer.cpp \
  ecmcApp/src/ecmcDataRecorder.cpp \
  ecmcApp/src/ecmcDataStorage.cpp \
  ecmcApp/src/ecmcCommandList.cpp \
  ecmcApp/src/hw_motor.cpp \

TEMPLATES = \
  ecmcApp/Db/ecmcController.template \
  ecmcApp/Db/ecmc.template \
  ecmcApp/Db/ecmc-extra.template \
  ecmcApp/Db/ecmcGeneral.template \
  ecmcApp/Db/elGenericAnalog.template \
  ecmcApp/Db/elGenericDigital.template \
  ecmcApp/Db/DUT_AxisStatus_v0_01.db \
  ecmcApp/Db/FB_DriveVirtual_v1_01.db \
  ecmcApp/Db/expression.db \
  ecmcApp/Db/ethercat.db \






