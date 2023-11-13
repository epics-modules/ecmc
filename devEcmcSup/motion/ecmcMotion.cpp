/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcMotion.cpp
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#define __STDC_FORMAT_MACROS  // for printf uint_64_t
#include <inttypes.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string>

#include "ecmcMotion.h"
#include "ecmcOctetIF.h"        // Log Macros
#include "ecmcErrorsList.h"
#include "ecmcDefinitions.h"
#include "ecmcEthercat.h"
#include "ecmcPLC.h"
#include "ecmcAxisBase.h"      // Abstract class for all axis types
#include "ecmcAxisReal.h"      // Normal axis (cntr,drv, enc, traj, mon, seq)
#include "ecmcAxisVirt.h"      // Axis without drive and controller
#include "ecmcDriveBase.h"     // Abstract drive base class
#include "ecmcTrajectoryBase.h"
#include "ecmcPIDController.h"
#include "ecmcEncoder.h"
#include "ecmcMonitor.h"
#include "ecmcEc.h"
#include "ecmcEcSlave.h"
#include "ecmcEcEntry.h"

// TODO: REMOVE GLOBALS
#include "ecmcGlobalsExtern.h"


int moveAbsolutePosition(int    axisIndex,
                         double positionSet,
                         double velocitySet,
                         double accelerationSet,
                         double decelerationSet) {
  LOGINFO4(
    "%s/%s:%d axisIndex=%d, positionSet=%lf, velocitySet=%lf, accelerationSet=%lf, decelerationSet=%lf\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    axisIndex,
    positionSet,
    velocitySet,
    accelerationSet,
    decelerationSet);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->moveAbsolutePosition(positionSet,
                                               velocitySet,
                                               accelerationSet,
                                               decelerationSet);
}

int moveRelativePosition(int    axisIndex,
                         double positionSet,
                         double velocitySet,
                         double accelerationSet,
                         double decelerationSet) {
  LOGINFO4(
    "%s/%s:%d axisIndex=%d, positionSet=%lf, velocitySet=%lf, accelerationSet=%lf, decelerationSet=%lf\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    axisIndex,
    positionSet,
    velocitySet,
    accelerationSet,
    decelerationSet);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->moveRelativePosition(positionSet,
                                               velocitySet,
                                               accelerationSet,
                                               decelerationSet);
}

int moveVelocity(int    axisIndex,
                 double velocitySet,
                 double accelerationSet,
                 double decelerationSet) {
  LOGINFO4(
    "%s/%s:%d axisIndex=%d, velocitySet=%lf, accelerationSet=%lf, decelerationSet=%lf\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    axisIndex,
    velocitySet,
    accelerationSet,
    decelerationSet);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->moveVelocity(velocitySet,
                                       accelerationSet,
                                       decelerationSet);
}

int moveHome(int    axisIndex,
             int    nCmdData,
             double homePositionSet,
             double velocityTowardsCamSet,
             double velocityOffCamSet,
             double accelerationSet,
             double decelerationSet) {
  LOGINFO4(
    "%s/%s:%d axisIndex=%d, nCmdData=%d, homePositionSet=%lf, velocityTowardsCamSet=%lf, "
    "velocityOffCamSet=%lf, accelerationSet=%lf, decelerationSet=%lf\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    axisIndex,
    nCmdData,
    homePositionSet,
    velocityTowardsCamSet,
    velocityOffCamSet,
    accelerationSet,
    decelerationSet);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->moveHome(nCmdData,
                                   homePositionSet,
                                   velocityTowardsCamSet,
                                   velocityOffCamSet,
                                   accelerationSet,
                                   decelerationSet);
}

int setPosition(int    axisIndex,
                double homePositionSet) {
  LOGINFO4(
    "%s/%s:%d axisIndex=%d,homePositionSet=%lf\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    axisIndex,
    homePositionSet);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->setPosition(homePositionSet);
}

int stopMotion(int axisIndex, int killAmplifier) {
  LOGINFO4("%s/%s:%d axisIndex=%d, killAmplifier=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           killAmplifier);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->stopMotion(killAmplifier);
}

int getAxisError(int axisIndex) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getError();
}

int getAxisErrorID(int axisIndex) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getErrorID();
}

int getAxisCycleCounter(int axisIndex, int *counter) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  *counter = axes[axisIndex]->getCycleCounter();
  return 0;
}

int setAxisExecute(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  // Axis needs to be reset before new command is executed
  // (however allow execute=0)
  if (value && axes[axisIndex]->getError()) {
    return ERROR_MAIN_AXIS_ERROR_EXECUTE_INTERLOCKED;
  }

  return axes[axisIndex]->setExecute(value);
}

int setAxisCommand(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setCommand((motionCommandTypes)value);
  return 0;
}

int setAxisCmdData(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setCmdData(value);
  return 0;
}

int setAxisSeqTimeout(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setSequenceTimeout(value * mcuFrequency);
  return 0;
}

int setAxisHomePostMoveEnable(int axisIndex, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getConfigEnc()->setHomePostMoveEnable(enable);
  return 0;
}

int setAxisHomeSeqId(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getConfigEnc()->setHomeSeqId(value);
  return 0;
}

int setAxisHomeAcc(int axisIndex, double acc) {
  LOGINFO4("%s/%s:%d axisIndex=%d acc=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           acc);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getConfigEnc()->setHomeAcc(acc);
  return 0;
}

int setAxisHomeDec(int axisIndex, double dec) {
  LOGINFO4("%s/%s:%d axisIndex=%d acc=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           dec);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getConfigEnc()->setHomeDec(dec);
  return 0;
}

int setAxisHomePostMoveTargetPosition(int axisIndex, double targetPosition) {
  LOGINFO4("%s/%s:%d axisIndex=%d targetPosition=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           targetPosition);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getConfigEnc()->setHomePostMoveTargetPosition(targetPosition);
  return 0;
}

int getAxisCommand(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)
  * value = 0;
  *value  =  static_cast<int>(axes[axisIndex]->getSeq()->getCommand());
  return 0;
}

int getAxisCmdData(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  * value = axes[axisIndex]->getSeq()->getCmdData();
  return 0;
}

int getAxisDebugInfoData(int axisIndex, char *buffer, int bufferByteSize) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  int bytesUsed = 0;
  return axes[axisIndex]->getAxisDebugInfoData(buffer,
                                               bufferByteSize,
                                               &bytesUsed);
}

int getAxisStatusStructV2(int axisIndex, char *buffer, int bufferByteSize) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  ecmcAxisStatusType data;
  int error = axes[axisIndex]->getDebugInfoData(&data);

  if (error) {
    return error;
  }

  // (Ax,PosSet,PosAct,PosErr,PosTarg,DistLeft,CntrOut,VelFFSet,VelAct,VelFFRaw,VelRaw,CycleCounter,
  // Error,Co,CD,St,IL,TS,ES,En,Ena,Ex,Bu,Ta,L-,L+,Ho");
  int ret = snprintf(buffer,
                     bufferByteSize,
                     "Main.M%d.stAxisStatusV2=%g,%g,%" PRId64 ",%g,%g,%g,%g,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                     axisIndex,
                     data.onChangeData.positionTarget,
                     data.onChangeData.positionActual,
                     data.onChangeData.positionRaw,

                     // UINT64_t
                     data.onChangeData.velocitySetpoint,
                     data.onChangeData.velocityActual,
                     data.acceleration,
                     data.deceleration,
                     data.cycleCounter,
                     0,

                     // EtherCAT time low32 not available yet
                     0,

                     // EtherCAT time high32 not available yet
                     data.onChangeData.statusWd.enable,
                     data.onChangeData.statusWd.enabled,
                     data.onChangeData.statusWd.execute,
                     data.onChangeData.command,
                     data.onChangeData.cmdData,
                     data.onChangeData.statusWd.limitbwd,
                     data.onChangeData.statusWd.limitfwd,
                     data.onChangeData.statusWd.homeswitch,
                     data.onChangeData.error > 0,
                     data.onChangeData.error,
                     data.reset,
                     data.onChangeData.statusWd.homed,
                     data.onChangeData.statusWd.busy,
                     data.onChangeData.statusWd.attarget,
                     data.moving,
                     data.stall);

  if ((ret >= bufferByteSize) || (ret <= 0)) {
    return ERROR_MAIN_PRINT_TO_BUFFER_FAIL;
  }

  return 0;
}

int setAxisEnable(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  if (!value) {
    axes[axisIndex]->setExecute(value);
  }

  return axes[axisIndex]->setEnable(value);
}

int getAxisEnable(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  * value = 0;
  *value  = axes[axisIndex]->getEnable() > 0;
  return 0;
}

int setAxisEnableMotionFunctions(int axisIndex,
                                 int enablePos,
                                 int enableConstVel,
                                 int enableHome) {
  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  return axes[axisIndex]->setAllowMotionFunctions(enablePos,
                                                  enableConstVel,
                                                  enableHome);
}

int setAxisEnableAlarmAtHardLimits(int axisIndex, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  int error = axes[axisIndex]->getMon()->setEnableHardLimitBWDAlarm(enable);

  if (error) {
    return error;
  }

  error = axes[axisIndex]->getMon()->setEnableHardLimitFWDAlarm(enable);

  if (error) {
    return error;
  }
  return 0;
}

int setAxisBlockCom(int axisIndex, int block) {
  LOGINFO4("%s/%s:%d axisIndex=%d, block=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           block);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setBlockExtCom(block);
}

int getAxisBlockCom(int axisIndex, int *block) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  * block = 0;
  *block  = axes[axisIndex]->getBlockExtCom();
  return 0;
}

int getAxisEnableAlarmAtHardLimits(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)
  * value = 0;
  *value  = axes[axisIndex]->getMon()->getEnableAlarmAtHardLimit() > 0;
  return 0;
}

int getAxisEnabled(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  * value = 0;
  *value  = axes[axisIndex]->getEnabled() > 0;
  return 0;
}

int getAxisTrajSource(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex);

  *value =  static_cast<int>(axes[axisIndex]->
                             getTrajDataSourceType());
  return 0;
}

int getAxisEncSource(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  *value =  static_cast<int>(axes[axisIndex]->
                             getEncDataSourceType());
  return 0;
}

int getAxisEncPrimaryIndex(int axisIndex, int *index) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  *index =  axes[axisIndex]->getPrimaryEncoderIndex();
  return 0;
}

int getAxisEncConfigIndex(int axisIndex, int *index) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  *index =  axes[axisIndex]->getConfigEncoderIndex();
  return 0;
}

int getAxisEncHomeIndex(int axisIndex, int *index) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex);

  *index =  axes[axisIndex]->getHomeEncoderIndex();
  return 0;
}

int getAxisAllowCommandsFromPLC(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  * value = 0;
  *value  =  static_cast<int>(axes[axisIndex]->getAllowCmdFromPLC() > 0);
  return 0;
}

int getAxisPLCEnable(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_PLCS_RETURN_IF_ERROR();

  int enable = 0;
  int error  = plcs->getEnable(AXIS_PLC_ID_TO_PLC_ID(axisIndex), &enable);

  if (error) {
    return error;
  }

  *value = enable;
  return 0;
}

int getAxisType(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  * value = axes[axisIndex]->getAxisType();
  return 0;
}

int setAxisEnableSoftLimitBwd(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getMon()->setEnableSoftLimitBwd(value);
  return 0;
}

int setAxisEnableSoftLimitFwd(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getMon()->setEnableSoftLimitFwd(value);
  return 0;
}

int setAxisEnableAlarmAtSoftLimit(int axisIndex,
                                  int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getMon()->setEnableSoftLimitAlarm(value);
  return 0;
}

int setAxisSoftLimitPosBwd(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getMon()->setSoftLimitBwd(value);
  return 0;
}

int setAxisSoftLimitPosFwd(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getMon()->setSoftLimitFwd(value);
  return 0;
}

int getAxisSoftLimitPosBwd(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  * value = axes[axisIndex]->getMon()->getSoftLimitBwd();
  return 0;
}

int getAxisSoftLimitPosFwd(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)

  * value = axes[axisIndex]->getMon()->getSoftLimitFwd();
  return 0;
}

int getAxisEnableSoftLimitBwd(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)
  * value = 0;
  *value  = axes[axisIndex]->getMon()->getEnableSoftLimitBwd() > 0;
  return 0;
}

int getAxisEnableSoftLimitFwd(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)
  * value = 0;
  *value  = axes[axisIndex]->getMon()->getEnableSoftLimitFwd() > 0;
  return 0;
}

int getAxisBusy(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  * value = 0;
  *value  = axes[axisIndex]->getBusy() > 0;
  return 0;
}

int setAxisAcceleration(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setAcc(value);
  return 0;
}

int setAxisDeceleration(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setDec(value);
  return 0;
}

int setAxisEmergDeceleration(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setEmergDec(value);
  return 0;
}

int setAxisJerk(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  // TODO not implemented in trajectory generator
  axes[axisIndex]->getTraj()->setJerk(value);
  return 0;
}

int setAxisTargetPos(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setTargetPos(value);
  return 0;
}

int setAxisTargetVel(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->setTargetVel(value);

  return 0;
}

int setAxisJogVel(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setJogVel(value);
  return 0;
}

int setAxisExtSetPos(int    axisIndex,
                     double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->setExtSetPos(value);
  return 0;
}

int setAxisHomeVelTowardsCam(int axisIndex, double dVel) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           dVel);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getConfigEnc()->setHomeVelTowardsCam(dVel);
  return 0;
}

int setAxisHomeVelOffCam(int axisIndex, double dVel) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           dVel);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  axes[axisIndex]->getConfigEnc()->setHomeVelOffCam(dVel);
  return 0;
}

int setAxisHomePos(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getConfigEnc()->setHomePosition(value);
  return 0;
}

int setAxisEncHomeLatchCountOffset(int axisIndex, int count) {
  LOGINFO4("%s/%s:%d axisIndex=%d count=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           count);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getConfigEnc()->setHomeLatchCountOffset(count);
  return 0;
}

int setAxisAllowCommandsFromPLC(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  return axes[axisIndex]->setAllowCmdFromPLC(value);
}

int setAxisPLCEnable(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_PLCS_RETURN_IF_ERROR();

  return plcs->setEnable(AXIS_PLC_ID_TO_PLC_ID(axisIndex), value);
}

int axisErrorReset(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%i value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  axes[axisIndex]->setReset(value);
  return 0;
}

int setAxisEncScaleNum(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  double temp = 0;
  int errorCode = axes[axisIndex]->getEncScaleNum(&temp);

  if (errorCode) {
    return errorCode;
  }

  if (temp == value) {
    return 0;
  }

  // Change of encoder scale while axis enabled is not allowed
  if (axes[axisIndex]->getEnable()) {
    return ERROR_MAIN_ENC_SET_SCALE_FAIL_DRV_ENABLED;
  }
  axes[axisIndex]->setEncScaleNum(value);

  return 0;
}

int setAxisEncScaleDenom(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  double temp = 0;
  int errorCode = axes[axisIndex]->getEncScaleDenom(&temp);

  if (errorCode) {
    return errorCode;
  }

  if (temp == value) {
    return 0;
  }

  // Change of encoder scale while axis enabled is not allowed
  if (axes[axisIndex]->getEnable()) {
    return ERROR_MAIN_ENC_SET_SCALE_FAIL_DRV_ENABLED;
  }
  axes[axisIndex]->setEncScaleDenom(value);

  return 0;
}

int setAxisEncInvHwReady(int axisIndex, int invert) {
  LOGINFO4("%s/%s:%d axisIndex=%d invert=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           invert);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  return axes[axisIndex]->setEncInvHwReady(invert);
}

int appendAxisPLCExpr(int axisIndex, char *expr) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           expr);
  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_PLCS_RETURN_IF_ERROR();

  return plcs->appendExprLine(AXIS_PLC_ID_TO_PLC_ID(axisIndex), expr);
}

int compileAxisPLCExpr(int axisIndex) {
  LOGINFO4("%s/%s:%d index=%d\n", __FILE__, __FUNCTION__, __LINE__, axisIndex);
  CHECK_PLCS_RETURN_IF_ERROR();
  return plcs->compileExpr(AXIS_PLC_ID_TO_PLC_ID(axisIndex));
}

int setAxisPLCTrajVelFilterEnable(int axisIndex, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);
  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  return axes[axisIndex]->setEnableExtTrajVeloFilter(enable);
}

int setAxisPLCTrajVelFilterSize(int axisIndex,
                                int size) {
  LOGINFO4("%s/%s:%d axisIndex=%d size=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           size);
  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  if (size <= 0) {
    return ERROR_MAIN_FILTER_INVALID_SIZE;
  }

  return axes[axisIndex]->setExtTrajVeloFiltSize(size);
}

int setAxisPLCEncVelFilterEnable(int axisIndex, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  return axes[axisIndex]->setEnableExtEncVeloFilter(enable);
}

int setAxisPLCEncVelFilterSize(int axisIndex,
                               int size) {
  LOGINFO4("%s/%s:%d axisIndex=%d size=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           size);
  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  if (size <= 0) {
    return ERROR_MAIN_FILTER_INVALID_SIZE;
  }
  return axes[axisIndex]->setExtEncVeloFiltSize(size);
}

int setAxisEncVelFilterSize(int axisIndex,
                            int size) {
  LOGINFO4("%s/%s:%d axisIndex=%d size=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           size);
  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  if (size <= 0) {
    return ERROR_MAIN_FILTER_INVALID_SIZE;
  }

  return axes[axisIndex]->setEncVeloFiltSize(size);
}

int setAxisEncPosFilterSize(int axisIndex,
                            int size) {
  LOGINFO4("%s/%s:%d axisIndex=%d size=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           size);
  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  return axes[axisIndex]->setEncPosFiltSize(size);
}

int setAxisEncPosFilterEnable(int axisIndex,
                              int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);
  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  return axes[axisIndex]->setEncPosFiltEnable(enable);
}

const char* getAxisPLCExpr(int axisIndex, int *error) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  if ((axisIndex >= ECMC_MAX_AXES) || (axisIndex <= 0)) {
    LOGERR("ERROR: Axis index out of range.\n");
    *error = ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;
    return "";
  }

  if (axes[axisIndex] == NULL) {
    LOGERR("ERROR: Axis object NULL\n");
    *error = ERROR_MAIN_AXIS_OBJECT_NULL;
    return "";
  }

  int plcIndex = ECMC_MAX_PLCS + axisIndex;

  if ((plcIndex >= ECMC_MAX_PLCS + ECMC_MAX_AXES) ||
      (plcIndex < ECMC_MAX_PLCS)) {
    LOGERR("ERROR: PLC index out of range.\n");
    *error = ERROR_PLCS_INDEX_OUT_OF_RANGE;
    return "";
  }

  int errorLocal    = 0;
  std::string *expr = plcs->getExpr(plcIndex, &errorLocal);

  if (errorLocal) {
    *error = errorLocal;
    return "";
  }

  return expr->c_str();
}

int setAxisTrajSource(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setTrajDataSourceType((dataSource)value);
}

int setAxisEncSource(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_ENCODER_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setEncDataSourceType((dataSource)value);
}

int setAxisTrajStartPos(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getTraj()->setStartPos(value);
  return 0;
}

// *****GET*********
int getAxisAcceleration(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  * value = axes[axisIndex]->getTraj()->getAcc();
  return 0;
}

int getAxisDeceleration(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  * value = axes[axisIndex]->getTraj()->getDec();
  return 0;
}

int getAxisTargetPos(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  return axes[axisIndex]->getPosSet(value);
}

int getAxisTargetVel(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  * value = axes[axisIndex]->getSeq()->getTargetVel();
  return 0;
}

int getAxisDone(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)
  * value = 0;
  *value  = !axes[axisIndex]->getSeq()->getBusy() > 0;
  return 0;
}

int getAxisPosSet(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getPosSet(value);
}

int getAxisVelFF(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)

  * value = axes[axisIndex]->getTraj()->getVel();
  return 0;
}

int getAxisExecute(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  * value = 0;
  *value  = axes[axisIndex]->getExecute() > 0;
  return 0;
}

int getAxisReset(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  * value = 0;
  *value  = axes[axisIndex]->getReset() > 0;
  return 0;
}

int getAxisID(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%i\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  * value = axes[axisIndex]->getAxisID();
  return 0;
}

int getAxisAtHardFwd(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)
  * value = 0;
  *value  = axes[axisIndex]->getMon()->getHardLimitFwd() > 0;
  return 0;
}

// Just better name
int getAxisLimitSwitchFwd(int axisIndex, int *value) {
  return getAxisAtHardFwd(axisIndex, value);
}

int getAxisAtHardBwd(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex)
  * value = 0;
  *value  = axes[axisIndex]->getMon()->getHardLimitBwd() > 0;
  return 0;
}

// Just better name
int getAxisLimitSwitchBwd(int axisIndex, int *value) {
  return getAxisAtHardBwd(axisIndex, value);
}

int getAxisEncHomed(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  bool tempHomed = 0;
  int errorCode = axes[axisIndex]->getAxisHomed(&tempHomed);

  if (errorCode) {
    return errorCode;
  }
  *value = 0;
  *value = tempHomed > 0;
  return 0;
}

int getAxisEncPosAct(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if (int iRet = axes[axisIndex]->getPosAct(value)) {
    value = 0;
    return iRet;
  }
  return 0;
}

int getAxisEncVelAct(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if (int iRet = axes[axisIndex]->getVelAct(value)) {
    *value = 0;
    return iRet;
  }

  return 0;
}

int getAxisAtHome(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if (axes[axisIndex]->getMon() == NULL) {
    return ERROR_MAIN_MONITOR_OBJECT_NULL;
  }
  *value = 0;
  *value = axes[axisIndex]->getMon()->getHomeSwitch() > 0;
  return 0;
}

int getAxisCntrlError(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if (int iRet = axes[axisIndex]->getCntrlError(value)) {
    *value = 0;
    return iRet;
  }
  return 0;
}

int getAxisHomeVelOffCam(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getSeq()->getHomeVelOffCam();
  return 0;
}

int getAxisHomeVelTowardsCam(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getSeq()->getHomeVelTowardsCam();
  return 0;
}

int getAxisEncScaleNum(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  if (int iRet = axes[axisIndex]->getEncScaleNum(value)) {
    value = 0;
    return iRet;
  }
  return 0;
}

int getAxisEncScaleDenom(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  if (int iRet = axes[axisIndex]->getEncScaleDenom(value)) {
    *value = 0;
    return iRet;
  }
  return 0;
}

int getAxisEncPosRaw(int axisIndex, int64_t *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  if (int iRet = axes[axisIndex]->getEncPosRaw(value)) {
    *value = 0;
    return iRet;
  }
  return 0;
}

/****************************************************************************/

int setAxisCntrlKp(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setKp(value);
  return 0;
}

int setAxisCntrlKi(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setKi(value);
  return 0;
}

int setAxisCntrlKd(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setKd(value);
  return 0;
}

int setAxisCntrlKff(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setKff(value);
  return 0;
}

int setAxisCntrlInnerParams(int    axisIndex,
                            double kp,
                            double ki,
                            double kd,
                            double tol) {
  LOGINFO4("%s/%s:%d axisIndex=%d kp=%f,ki=%f,kd=%f,tol=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           kp, ki, kd, tol);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setInnerCtrlParams(kp, ki, kd, tol);
  return 0;
}

int setAxisCntrlOutHL(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setOutMax(value);
  return 0;
}

int setAxisCntrlOutLL(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setOutMin(value);
  return 0;
}

int setAxisCntrlIpartHL(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setIOutMax(value);
  return 0;
}

int setAxisCntrlIpartLL(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getCntrl()->setIOutMin(value);
  return 0;
}

int getAxisCntrlOutPpart(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getOutPPart();
  return 0;
}

int getAxisCntrlOutIpart(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getOutIPart();
  return 0;
}

int getAxisCntrlOutDpart(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getOutDPart();
  return 0;
}

int getAxisCntrlOutFFpart(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getOutFFPart();
  return 0;
}

int getAxisCntrlOutput(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getOutTot();
  return 0;
}

int setAxisEncOffset(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getConfigEnc()->setOffset(value);
}

int setAxisEncBits(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getConfigEnc()->setBits(value);
}

int setAxisEncAbsBits(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getConfigEnc()->setAbsBits(value);
}

int setAxisEncRawMask(int axisIndex, uint64_t rawMask) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           (uint)rawMask);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getConfigEnc()->setRawMask(rawMask);
}

int setAxisEncType(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getConfigEnc()->setType((encoderType)value);
}

int setAxisEncMaxDiffToPrimEnc(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getConfigEnc()->setMaxPosDiffToPrimEnc(value);
  return 0;
}

int addAxisEnc(int axisIndex) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);

  return axes[axisIndex]->addEncoder();
}

int selectAxisEncPrimary(int axisIndex, int index) {
  LOGINFO4("%s/%s:%d axisIndex=%d, primaryEncoder=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           index);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  return axes[axisIndex]->selectPrimaryEncoder(index);
}

int selectAxisEncConfig(int axisIndex, int index) {
  LOGINFO4("%s/%s:%d axisIndex=%d, configEncoder=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           index);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  return axes[axisIndex]->selectConfigEncoder(index);
}

int selectAxisEncHome(int axisIndex, int index) {
  LOGINFO4("%s/%s:%d axisIndex=%d, homeEncoder=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           index);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  return axes[axisIndex]->selectHomeEncoder(index);
}

int setAxisEncEnableRefAtHome(int axisIndex, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getConfigEnc()->setRefAtHoming(enable);
}

int setAxisEncRefToOtherEncAtStartup(int axisIndex, int encRef) {
  LOGINFO4("%s/%s:%d axisIndex=%d, encRef=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           encRef);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getConfigEnc()->setRefToOtherEncAtStartup(encRef);
}

/****************************************************************************/

// Drv GET
int getAxisDrvScaleNum(int axisIndex, double *value) {
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getDrv()->getScaleNum();
  return 0;
}

// Drv SET
int setAxisDrvScaleNum(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getDrv()->setScaleNum(value);
  return 0;
}

int setAxisDrvScaleDenom(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setScaleDenom(value);
}

int setAxisDrvVelSetOffsetRaw(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setVelSetOffsetRaw(value);
}

int setAxisDrvBrakeEnable(int axisIndex, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setEnableBrake(enable);
}

int setAxisDrvBrakeOpenDelayTime(int axisIndex, int delayTime) {
  LOGINFO4("%s/%s:%d axisIndex=%d delayTime=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           delayTime);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setBrakeOpenDelayTime(delayTime);
}

int setAxisDrvBrakeCloseAheadTime(int axisIndex, int aheadTime) {
  LOGINFO4("%s/%s:%d axisIndex=%d aheadTime=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           aheadTime);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setBrakeCloseAheadTime(aheadTime);
}

int setAxisDrvStateMachineTimeout(int    axisIndex,
                                  double seconds) {
  LOGINFO4("%s/%s:%d axisIndex=%d timeout=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           seconds);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setStateMachineTimeout(seconds);
}

int setAxisDrvReduceTorqueEnable(int axisIndex, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getDrv()->setEnableReduceTorque(enable);
}

// Drv GET
int getAxisDrvScale(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getDrv()->getScale();
  return 0;
}

int getAxisDrvEnable(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);
  *value = 0;
  *value = axes[axisIndex]->getDrv()->getEnable() > 0;
  return 0;
}

// Mon GET
int getAxisMonAtTargetTol(int axisIndex, double *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getAtTargetTol();
  return 0;
}

// Mon SET
int setAxisMonAtTargetTol(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setAtTargetTol(value);
}

int setAxisEnableCheckEncsDiff(int axisIndex, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setEnableCheckEncsDiff(enable);
}

int getAxisMonAtTargetTime(int axisIndex, int *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getAtTargetTime();
  return 0;
}

int setAxisMonAtTargetTime(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setAtTargetTime(value);
}

int getAxisMonEnableAtTargetMon(int axisIndex, int *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getEnableAtTargetMon();
  return 0;
}

int setAxisMonEnableAtTargetMon(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setEnableAtTargetMon(value);
  return 0;
}

int setAxisMonExtHWInterlockPolarity(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setHardwareInterlockPolarity(
    (ecmcSwitchPolarity)value);
}

int getAxisMonExtHWInterlockPolarity(int axisIndex, int *pol) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *pol = (int)axes[axisIndex]->getMon()->getHardwareInterlockPolarity();
  return 0;
}

int setAxisMonLimitBwdPolarity(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setHardLimitBwdPolarity(
    (ecmcSwitchPolarity)value);
}

int getAxisMonLimitBwdPolarity(int axisIndex, int *pol) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *pol = (int)axes[axisIndex]->getMon()->getHardLimitBwdPolarity();
  return 0;
}

int setAxisMonLimitFwdPolarity(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setHardLimitFwdPolarity(
    (ecmcSwitchPolarity)value);
}

int getAxisMonLimitFwdPolarity(int axisIndex, int *pol) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *pol = (int)axes[axisIndex]->getMon()->getHardLimitFwdPolarity();
  return 0;
}

int setAxisMonHomeSwitchPolarity(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setHomePolarity(
    (ecmcSwitchPolarity)value);
}

int getAxisMonHomeSwitchPolarity(int axisIndex, int *pol) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *pol = (int)axes[axisIndex]->getMon()->getHomePolarity();
  return 0;
}

int getAxisMonPosLagTol(int axisIndex, double *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getPosLagTol();
  return 0;
}

int setAxisMonPosLagTol(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setPosLagTol(value);
}

int getAxisMonPosLagTime(int axisIndex, int *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getPosLagTime();
  return 0;
}

int setAxisMonPosLagTime(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setPosLagTime(value);
}

int getAxisMonEnableLagMon(int axisIndex, int *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getMon()->getEnableLagMon();
  return 0;
}

int setAxisMonEnableLagMon(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setEnableLagMon(value);
  return 0;
}

int setAxisMonMaxVel(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setMaxVel(value);
}

int getAxisMonMaxVel(int axisIndex, double *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getMaxVel();
  return 0;
}

int setAxisMonEnableMaxVel(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setEnableMaxVelMon(value);
}

int getAxisMonEnableMaxVel(int axisIndex, int *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getEnableMaxVelMon();
  return 0;
}

int getAxisMonLatchLimit(int  axisIndex,
                         int *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getLatchAtLimit();
  return 0;
}

int setAxisMonMaxVelDriveILDelay(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setMaxVelDriveTime(value);
}

int setAxisMonMaxVelTrajILDelay(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setMaxVelTrajTime(value);
}

int setAxisMonLatchLimit(int axisIndex,
                         int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setLatchAtLimit(value);
}

int setAxisMonEnableExternalInterlock(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setEnableHardwareInterlock(value);
}

int setAxisMonEnableCntrlOutHLMon(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setEnableCntrlHLMon(value);
}

int setAxisMonEnableVelocityDiff(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setEnableVelocityDiffMon(value);
}

int setAxisMonVelDiffTol(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setVelDiffMaxDifference(value);
}

int setAxisMonVelDiffTrajILDelay(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setVelDiffTimeTraj(value);
}

int setAxisMonVelDiffDriveILDelay(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setVelDiffTimeDrive(value);
}

int setAxisMonCntrlOutHL(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setCntrlOutputHL(value);
}

// Mon GET
int getAxisMonAtTarget(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *value = 0;
  *value = axes[axisIndex]->getMon()->getAtTarget() > 0;
  return 0;
}

/****************************************************************************/

// Configuration procedures

int createAxis(int index, int type, int drvType, int trajType) {
  LOGINFO4("%s/%s:%d axisIndex=%d axisType=%d drvType=%d, trajType=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index,
           type,
           drvType,
           trajType);

  if ((index < 0) || (index >= ECMC_MAX_AXES)) {
    return ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;
  }

  // Do not allow create already created axis (must be deleted first)
  if (axes[index] != NULL) {
    return ERROR_MAIN_AXIS_ALREADY_CREATED;
  }

  try {
    switch ((axisType)type) {
    case ECMC_AXIS_TYPE_REAL:

      if (axes[index] != NULL) {
        delete axes[index];
      }

      // Sample rate fixed
      sampleRateChangeAllowed = 0;
      axes[index]             = new ecmcAxisReal(asynPort, index,
                                                 1 / mcuFrequency,
                                                 (ecmcDriveTypes)drvType,
                                                 (ecmcTrajTypes)trajType);
      break;

    case ECMC_AXIS_TYPE_VIRTUAL:

      // Drive type ignored (Virtual axis have no drive)
      if (axes[index] != NULL) {
        delete axes[index];
      }

      // Sample rate fixed
      sampleRateChangeAllowed = 0;
      axes[index]             = new ecmcAxisVirt(asynPort, index,
                                                 1 / mcuFrequency,
                                                 (ecmcTrajTypes)trajType);
      break;

    default:
      return ERROR_MAIN_AXIS_TYPE_UNKNOWN;
    }
  }
  catch (std::exception& e) {
    delete axes[index];
    axes[index] = NULL;
    LOGERR("%s/%s:%d: EXCEPTION %s WHEN ALLOCATE MEMORY FOR AXIS OBJECT.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           e.what());
    return ERROR_MAIN_EXCEPTION;
  }
  axisDiagIndex = index;  // Always printout last axis added

  int error = createPLC(AXIS_PLC_ID_TO_PLC_ID(index), mcuPeriod / 1e6, 1);

  if (error) {
    return error;
  }

  // axes[index]->setAxisMainPLCs(plcs);

  return axes[index]->getErrorID();
}

int linkEcEntryToAxisEnc(int   slaveIndex,
                         char *entryIDString,
                         int   axisIndex,
                         int   encoderEntryIndex,
                         int   bitIndex) {
  LOGINFO4(
    "%s/%s:%d slave_index=%d entry=%s encoder=%d encoder_entry=%d bit_index=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slaveIndex,
    entryIDString,
    axisIndex,
    encoderEntryIndex,
    bitIndex);

  if (!ec) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave = NULL;

  if (slaveIndex >= 0) {
    slave = ec->findSlave(slaveIndex);
  } else {    // simulation slave
    slave = ec->getSlave(slaveIndex);
  }

  if (slave == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID = entryIDString;

  ecmcEcEntry *entry = slave->findEntry(sEntryID);

  if (entry == NULL) return ERROR_MAIN_EC_ENTRY_NULL;

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  if ((encoderEntryIndex >= ECMC_EC_ENTRY_LINKS_MAX) ||
      (encoderEntryIndex <
       0)) return ERROR_MAIN_ENCODER_ENTRY_INDEX_OUT_OF_RANGE;

  return axes[axisIndex]->getConfigEnc()->setEntryAtIndex(entry,
                                                          encoderEntryIndex,
                                                          bitIndex);
}

int linkEcEntryToAxisDrv(int   slaveIndex,
                         char *entryIDString,
                         int   axisIndex,
                         int   driveEntryIndex,
                         int   bitIndex) {
  LOGINFO4(
    "%s/%s:%d slave_index=%d entry=%s drive=%d drive_entry=%d bit_index=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slaveIndex,
    entryIDString,
    axisIndex,
    driveEntryIndex,
    bitIndex);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  // Disable brake output with empty string
  if ((strlen(entryIDString) == 0) &&
      (driveEntryIndex == ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT)) {
    return axes[axisIndex]->getDrv()->setEnableBrake(0);
  }

  // Disable reduce torque output with empty string
  if ((strlen(entryIDString) == 0) &&
      (driveEntryIndex == ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT)) {
    return axes[axisIndex]->getDrv()->setEnableReduceTorque(0);
  }

  if (!ec) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave = NULL;

  if (slaveIndex >= 0) {
    slave = ec->findSlave(slaveIndex);
  } else {
    slave = ec->getSlave(slaveIndex);
  }

  if (slave == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID = entryIDString;

  ecmcEcEntry *entry = slave->findEntry(sEntryID);

  if (entry == NULL) return ERROR_MAIN_EC_ENTRY_NULL;

  if ((driveEntryIndex >= ECMC_EC_ENTRY_LINKS_MAX) || (driveEntryIndex < 0)) {
    return ERROR_MAIN_DRIVE_ENTRY_INDEX_OUT_OF_RANGE;
  }

  int ret = axes[axisIndex]->getDrv()->setEntryAtIndex(entry,
                                                       driveEntryIndex,
                                                       bitIndex);

  if (ret) {
    return ret;
  }

  // Auto enable break
  if (driveEntryIndex == ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT) {
    ret = axes[axisIndex]->getDrv()->setEnableBrake(1);

    if (ret) {
      return ret;
    }
  }

  // Auto enable reduce torque
  if (driveEntryIndex == ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT) {
    ret = axes[axisIndex]->getDrv()->setEnableReduceTorque(1);

    if (ret) {
      return ret;
    }
  }

  return 0;
}

int linkEcEntryToAxisMon(int   slaveIndex,
                         char *entryIDString,
                         int   axisIndex,
                         int   monitorEntryIndex,
                         int   bitIndex) {
  LOGINFO4(
    "%s/%s:%d slave_index=%d entry=%s monitor=%d monitor_entry=%d bit_index=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slaveIndex,
    entryIDString,
    axisIndex,
    monitorEntryIndex,
    bitIndex);

  if (!ec) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave = NULL;

  if (slaveIndex >= 0) {
    slave = ec->findSlave(slaveIndex);
  } else {
    slave = ec->getSlave(slaveIndex);
  }

  if (slave == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID = entryIDString;

  ecmcEcEntry *entry = slave->findEntry(sEntryID);

  if (entry == NULL) {
    return ERROR_MAIN_EC_ENTRY_NULL;
  }

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  if ((monitorEntryIndex >= ECMC_EC_ENTRY_LINKS_MAX) ||
      (monitorEntryIndex <
       0)) return ERROR_MAIN_MONITOR_ENTRY_INDEX_OUT_OF_RANGE;

  int errorCode = axes[axisIndex]->getMon()->setEntryAtIndex(entry,
                                                             monitorEntryIndex,
                                                             bitIndex);

  if (errorCode) {
    return errorCode;
  }

  if (monitorEntryIndex == ECMC_MON_ENTRY_INDEX_EXTINTERLOCK) {
    return setAxisMonEnableExternalInterlock(axisIndex, 1);
  }

  return 0;
}

int linkEcEntryToAxisStatusOutput(int   slaveIndex,
                                  char *entryIDString,
                                  int   axisIndex) {
  LOGINFO4("%s/%s:%d slave_index=%d entry=%s, axisId=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slaveIndex,
           entryIDString,
           axisIndex);

  if (!ec) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave = NULL;

  if (slaveIndex >= 0) {
    slave = ec->findSlave(slaveIndex);
  } else {
    slave = ec->getSlave(slaveIndex);
  }

  if (slave == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID = entryIDString;

  ecmcEcEntry *entry = slave->findEntry(sEntryID);

  if (entry == NULL) return ERROR_MAIN_EC_ENTRY_NULL;

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);

  return axes[axisIndex]->setEcStatusOutputEntry(entry);
}

int setDiagAxisIndex(int axisIndex) {
  LOGINFO4("%s/%s:%d axisIndex=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  axisDiagIndex = axisIndex;
  return 0;
}

int setDiagAxisFreq(int value) {
  LOGINFO4("%s/%s:%d frequency=%d \n", __FILE__, __FUNCTION__, __LINE__,
           value);

  if ((value < 1) ||
      (value > 500)) return ERROR_MAIN_DIAG_AXIS_FREQ_OUT_OF_RANGE;

  axisDiagFreq = value;
  return 0;
}

int setDiagAxisEnable(int value) {
  LOGINFO4("%s/%s:%d enable=%d \n", __FILE__, __FUNCTION__, __LINE__, value);

  WRITE_DIAG_BIT(FUNCTION_HW_MOTOR_AXIS_DIAGNOSTICS_BIT, value);

  return 0;
}

int getAxisModRange(int     axisIndex,
                    double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getModRange();
  return 0;
}

int setAxisModRange(int    axisIndex,
                    double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d, ModRange=%lf \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->setModRange(value);
}

int getAxisModType(int  axisIndex,
                   int *type) {
  LOGINFO4("%s/%s:%d axisIndex=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  *type = axes[axisIndex]->getModType();
  return 0;
}

void* getAxisPointer(int axisIndex) {
  LOGINFO4("%s/%s:%d axisIndex=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  if ((axisIndex >= ECMC_MAX_AXES) || (axisIndex <= 0)) {
    LOGERR("ERROR: Axis index out of range.\n");
    return NULL;
  }
  return (void *)axes[axisIndex];
}

int setAxisModType(int axisIndex,
                   int type) {
  LOGINFO4("%s/%s:%d axisIndex=%d, mod type=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           type);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->setModType(type);
}

int setAxisDisableAtErrorReset(int axisIndex,
                               int disable) {
  LOGINFO4("%s/%s:%d axisIndex=%d, disable=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           disable);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->setDisableAxisAtErrorReset(disable);
}

int setAxisAllowSourceChangeWhenEnabled(int axisIndex,
                                        int allow) {
  LOGINFO4("%s/%s:%d axisIndex=%d, allow=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           allow);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->setAllowSourceChangeWhenEnabled(allow);
}
