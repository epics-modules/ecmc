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

int setPVTControllerTrgDurMs(double durationMs) {

  LOGINFO4("%s/%s:%d duration=%lf\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    durationMs);

    if(!pvtCtrl_) return ERROR_PVT_CTRL_NULL;

  return pvtCtrl_->setTriggerDuration(durationMs/1000);
}

int setAxisExecute(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if (value && axes[axisIndex]->getBlockCom()) {
    return axes[axisIndex]->setExternalCommandBlockedError();
  }

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

int setAxisAutoEnableTimeout(int axisIndex, double timeS) {
  LOGINFO4("%s/%s:%d axisIndex=%d timeS=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           timeS);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setAutoEnableTimeout(timeS);
}

int setAxisEnableAutoEnable(int axisIndex, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setEnableAutoEnable(enable);
}

int setAxisAutoDisableAfterTime(int axisIndex, double timeS) {
  LOGINFO4("%s/%s:%d axisIndex=%d timeS=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           timeS);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setAutoDisableAfterTime(timeS);
}

int setAxisEnableAutoDisable(int axisIndex, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->setEnableAutoDisable(enable);
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

int getAxisHomePos(int axisIndex, double *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getConfigEnc()->getHomePosition();
  return 0;
}

int getAxisHomeSeqId(int axisIndex, int *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getConfigEnc()->getHomeSeqId();
  return 0;
}

int getAxisHomeAcc(int axisIndex, double *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getConfigEnc()->getHomeAcc();
  return 0;
}

int getAxisHomeDec(int axisIndex, double *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getConfigEnc()->getHomeDec();
  return 0;
}

int getAxisHomePostMoveEnable(int axisIndex, int *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getConfigEnc()->getHomePostMoveEnable();
  return 0;
}

int getAxisHomePostMoveTargetPosition(int axisIndex, double *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getConfigEnc()->getHomePostMoveTargetPosition();
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

int setAxisEnable(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)

  if (value && axes[axisIndex]->getBlockCom()) {
    return axes[axisIndex]->setExternalCommandBlockedError();
  }

  if (!value) {
    axes[axisIndex]->setExecute(value);
  }

  return axes[axisIndex]->setEnable(value);
}

int setAxisEnableAtStartup(int axisIndex, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  return axes[axisIndex]->setEnableAtStartup(enable);
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

int setAxisEnablePos(int axisIndex, int enablePos) {
  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  int enableConstVel = axes[axisIndex]->getAllowConstVelo();
  int enableHome     = axes[axisIndex]->getAllowHome();

  if (enableConstVel < 0) {
    return -enableConstVel;
  }
  if (enableHome < 0) {
    return -enableHome;
  }

  return axes[axisIndex]->setAllowMotionFunctions(enablePos, enableConstVel, enableHome);
}

int setAxisEnableConstVel(int axisIndex, int enableConstVel) {
  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  int enablePos  = axes[axisIndex]->getAllowPos();
  int enableHome = axes[axisIndex]->getAllowHome();

  if (enablePos < 0) {
    return -enablePos;
  }
  if (enableHome < 0) {
    return -enableHome;
  }

  return axes[axisIndex]->setAllowMotionFunctions(enablePos, enableConstVel, enableHome);
}

int setAxisEnableHome(int axisIndex, int enableHome) {
  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  int enablePos      = axes[axisIndex]->getAllowPos();
  int enableConstVel = axes[axisIndex]->getAllowConstVelo();

  if (enablePos < 0) {
    return -enablePos;
  }
  if (enableConstVel < 0) {
    return -enableConstVel;
  }

  return axes[axisIndex]->setAllowMotionFunctions(enablePos, enableConstVel, enableHome);
}

int getAxisEnablePos(int axisIndex, int *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value = 0;
  int ret = axes[axisIndex]->getAllowPos();
  if (ret < 0) {
    return -ret;
  }
  *value = ret;
  return 0;
}

int getAxisEnableConstVel(int axisIndex, int *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value = 0;
  int ret = axes[axisIndex]->getAllowConstVelo();
  if (ret < 0) {
    return -ret;
  }
  *value = ret;
  return 0;
}

int getAxisEnableHome(int axisIndex, int *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value = 0;
  int ret = axes[axisIndex]->getAllowHome();
  if (ret < 0) {
    return -ret;
  }
  *value = ret;
  return 0;
}

int getAxisAllowSourceChangeWhenEnabled(int axisIndex, int *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getAllowSourceChangeWhenEnabled();
  return 0;
}

int getAxisAutoEnableTimeout(int axisIndex, double *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getAutoEnableTimeout();
  return 0;
}

int getAxisAutoDisableAfterTime(int axisIndex, double *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getAutoDisableAfterTime();
  return 0;
}

int getAxisEnableAutoEnable(int axisIndex, int *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getEnableAutoEnable();
  return 0;
}

int getAxisEnableAutoDisable(int axisIndex, int *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getEnableAutoDisable();
  return 0;
}

int getAxisTweakDist(int axisIndex, double *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getTweakDist();
  return 0;
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

  return axes[axisIndex]->setBlockCom(block);
}

int getAxisBlockCom(int axisIndex, int *block) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  * block = 0;
  *block  = axes[axisIndex]->getBlockCom();
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

  return axes[axisIndex]->getMon()->setEnableSoftLimitBwd(value);
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

  return axes[axisIndex]->getMon()->setEnableSoftLimitFwd(value);
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

  return axes[axisIndex]->getMon()->setSoftLimitBwd(value);
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

  return axes[axisIndex]->getMon()->setSoftLimitFwd(value);
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
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setDefaultAcc(value);
  axes[axisIndex]->setAcc(value);
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
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->getSeq()->setDefaultDec(value);
  axes[axisIndex]->setDec(value);
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

  axes[axisIndex]->setTargetPosAndCmd(value);
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

int setAxisTweakDist(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)

  axes[axisIndex]->setTweakDist(value);

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

int setAxisErrorId(int axisIndex, int errorid) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           errorid);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  axes[axisIndex]->setErrorID(errorid);
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

int setAxisEncAllowOverUnderFlow(int axisIndex, int allow) {
  LOGINFO4("%s/%s:%d axisIndex=%d allow=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           allow);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)

  return axes[axisIndex]->getConfigEnc()->setAllowOverUnderFlow(allow);;
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

int loadAxisEncLookupTable(int axisIndex, const char* filename) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           filename);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  return axes[axisIndex]->loadEncLookupTable(filename);
}

int getAxisEncLookupTableEnable(int axisIndex, int *enable) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *enable = axes[axisIndex]->getConfigEnc()->getLookupTableEnable();
  return 0;
}

int setAxisEncLookupTableEnable(int axisIndex, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)

  return axes[axisIndex]->setEncLookupTableEnable(enable);
}

int setAxisEncLookupTableRange(int axisIndex, double range) {
  LOGINFO4("%s/%s:%d axisIndex=%d range=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           range);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getConfigEnc()->setLookupTableRange(range);
}

int getAxisEncLookupTableRange(int axisIndex, double *range) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  *range = axes[axisIndex]->getConfigEnc()->getLookupTableRange();
  return 0;
}

int setAxisEncLookupTableScale(int axisIndex, double scale) {
  LOGINFO4("%s/%s:%d axisIndex=%d scale=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           scale);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getConfigEnc()->setLookupTableScale(scale);
}

int getAxisEncLookupTableScale(int axisIndex, double *scale) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  *scale = axes[axisIndex]->getConfigEnc()->getLookupTableScale();
  return 0;
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

int getAxisPLCTrajVelFilterEnable(int axisIndex, int *enable) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *enable = axes[axisIndex]->getEnableExtTrajVeloFilter();
  return 0;
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

int getAxisPLCTrajVelFilterSize(int axisIndex, int *size) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *size = axes[axisIndex]->getExtTrajVeloFiltSize();
  return 0;
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

int getAxisPLCEncVelFilterEnable(int axisIndex, int *enable) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *enable = axes[axisIndex]->getEnableExtEncVeloFilter();
  return 0;
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

int getAxisPLCEncVelFilterSize(int axisIndex, int *size) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  *size = axes[axisIndex]->getExtEncVeloFiltSize();
  return 0;
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

int setAxisEncVelFilterEnable(int axisIndex,
                              int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);
  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex)
  return axes[axisIndex]->setEncVeloFiltEnable(enable);
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

int getAxisEncVelFilterSize(int axisIndex, int *size) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *size = axes[axisIndex]->getConfigEnc()->getVeloFilterSize();
  return 0;
}

int getAxisEncVelFilterEnable(int axisIndex, int *enable) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *enable = axes[axisIndex]->getConfigEnc()->getVelFilterEnable();
  return 0;
}

int getAxisEncPosFilterSize(int axisIndex, int *size) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *size = axes[axisIndex]->getConfigEnc()->getPosFilterSize();
  return 0;
}

int getAxisEncPosFilterEnable(int axisIndex, int *enable) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *enable = axes[axisIndex]->getConfigEnc()->getPosFilterEnable();
  return 0;
}

const char* getAxisPLCExpr(int axisIndex, int *error) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  if ((axisIndex >= ECMC_MAX_AXES) || (axisIndex < 0)) {
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

  if (!plcs) {
    LOGERR("ERROR: PLC object NULL.\n");
    *error = ERROR_MAIN_PLCS_NULL;
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

  return axes[axisIndex]->getSeq()->setTrajDataSourceType((dataSource)value);
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

int getAxisEmergDeceleration(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%i\n",
           __FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getTraj()->getEmergDec();
  return 0;
}

int getAxisJerk(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%i\n",
           __FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_TRAJ_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getTraj()->getJerk();
  return 0;
}

int getAxisJogVel(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%i\n",
           __FILE__, __FUNCTION__, __LINE__, axisIndex);

  CHECK_AXIS_SEQ_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getSeq()->getJogVel();
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

  return axes[axisIndex]->getVelSet(value);;
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
    *value = 0;
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
    *value = 0;
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

  return axes[axisIndex]->setCntrlKp(value);
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

  return axes[axisIndex]->setCntrlKi(value);
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

  return axes[axisIndex]->setCntrlKd(value);
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

  return axes[axisIndex]->setCntrlKff(value);
}

int setAxisCntrlDeadband(int    axisIndex,
                         double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setCtrlDeadband(value);
  return 0;
}

int setAxisCntrlDeadbandTime(int axisIndex,
                             int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setCtrlDeadbandTime(value);
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

int setAxisCntrlInnerKp(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  ecmcPIDController *cntrl = axes[axisIndex]->getCntrl();
  cntrl->setInnerCtrlParams(value,
                            cntrl->getInnerKi(),
                            cntrl->getInnerKd(),
                            cntrl->getInnerTol());
  return 0;
}

int setAxisCntrlInnerKi(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  ecmcPIDController *cntrl = axes[axisIndex]->getCntrl();
  cntrl->setInnerCtrlParams(cntrl->getInnerKp(),
                            value,
                            cntrl->getInnerKd(),
                            cntrl->getInnerTol());
  return 0;
}

int setAxisCntrlInnerKd(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  ecmcPIDController *cntrl = axes[axisIndex]->getCntrl();
  cntrl->setInnerCtrlParams(cntrl->getInnerKp(),
                            cntrl->getInnerKi(),
                            value,
                            cntrl->getInnerTol());
  return 0;
}

int setAxisCntrlInnerTol(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  ecmcPIDController *cntrl = axes[axisIndex]->getCntrl();
  cntrl->setInnerCtrlParams(cntrl->getInnerKp(),
                            cntrl->getInnerKi(),
                            cntrl->getInnerKd(),
                            value);
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

int getAxisCntrlKp(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getKp();
  return 0;
}

int getAxisCntrlKi(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getKi();
  return 0;
}

int getAxisCntrlKd(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getKd();
  return 0;
}

int getAxisCntrlKff(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getKff();
  return 0;
}

int getAxisCntrlDeadband(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getCtrlDeadband();
  return 0;
}

int getAxisCntrlDeadbandTime(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getCtrlDeadbandTime();
  return 0;
}

int getAxisCntrlIpartHL(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getIOutMax();
  return 0;
}

int getAxisCntrlIpartLL(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getIOutMin();
  return 0;
}

int getAxisCntrlOutHL(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getOutMax();
  return 0;
}

int getAxisCntrlOutLL(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getOutMin();
  return 0;
}

int getAxisCntrlInnerKp(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getInnerKp();
  return 0;
}

int getAxisCntrlInnerKi(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getInnerKi();
  return 0;
}

int getAxisCntrlInnerKd(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getInnerKd();
  return 0;
}

int getAxisCntrlInnerTol(int axisIndex, double *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_CONTROLLER_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getCntrl()->getInnerTol();
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

int getAxisEncOffset(int axisIndex, double *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *value = axes[axisIndex]->getConfigEnc()->getOffset();
  return 0;
}

int getAxisEncBits(int axisIndex, int *bits) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *bits = axes[axisIndex]->getConfigEnc()->getBits();
  return 0;
}

int getAxisEncAbsBits(int axisIndex, int *bits) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *bits = axes[axisIndex]->getConfigEnc()->getAbsBits();
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

int getAxisEncRawMask(int axisIndex, uint64_t *rawMask) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  *rawMask = axes[axisIndex]->getConfigEnc()->getRawMask();
  return 0;
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

int selectAxisEncCSPDrv(int axisIndex, int index) {
  LOGINFO4("%s/%s:%d axisIndex=%d, primaryEncoder=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           index);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  return axes[axisIndex]->selectCSPDriveEncoder(index);
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

int getAxisEncEnableRefAtHome(int axisIndex, int *enable) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex)
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex)
  *enable = axes[axisIndex]->getConfigEnc()->getRefAtHoming();
  return 0;
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

int setAxisEncHomeLatchArmControlWord(int axisIndex, uint64_t control, int bits) {
  LOGINFO4("%s/%s:%d axisIndex=%d control=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           bits);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getConfigEnc()->setHomeLatchArmControlWord(control, bits);
}

int setAxisEncDelayCompTime(int axisIndex, double cycles) {
  LOGINFO4("%s/%s:%d axisIndex=%d, cycles=%lf\n",
           __FILE__, __FUNCTION__, __LINE__, axisIndex, cycles);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  int enable = axes[axisIndex]->getConfigEnc()->getDelayCompEnable();
  return axes[axisIndex]->getConfigEnc()->setDelayCyclesAndEnable(cycles, enable);
}

int setAxisEncDelayCompEnable(int axisIndex, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d, enable=%d\n",
           __FILE__, __FUNCTION__, __LINE__, axisIndex, enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  double cycles = axes[axisIndex]->getConfigEnc()->getDelayCycles();
  return axes[axisIndex]->getConfigEnc()->setDelayCyclesAndEnable(cycles, enable);
}

int getAxisEncDelayCompTime(int axisIndex, double *cycles) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  *cycles = axes[axisIndex]->getConfigEnc()->getDelayCycles();
  return 0;
}

int getAxisEncDelayCompEnable(int axisIndex, int *enable) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  *enable = axes[axisIndex]->getConfigEnc()->getDelayCompEnable();
  return 0;
}

int setAxisEncDelayCyclesAndEnable(int axisIndex, double cycles, int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d, timeMs=%lf, enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           cycles,
           enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_ENCODER_CFG_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getConfigEnc()->setDelayCyclesAndEnable(cycles, enable);
}

/****************************************************************************/

int getAxisDrvBrakeEnable(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getDrv()->getEnableBrake();
  return 0;
}

int getAxisDrvBrakeOpenDelayTime(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getDrv()->getBrakeOpenDelayTime();
  return 0;
}

int getAxisDrvBrakeCloseAheadTime(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getDrv()->getBrakeCloseAheadTime();
  return 0;
}

int getAxisDrvReduceTorqueEnable(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getDrv()->getEnableReduceTorque();
  return 0;
}

// Drv GET
int getAxisDrvScaleNum(int axisIndex, double *value) {
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getDrv()->getScaleNum();
  return 0;
}

int getAxisDrvScaleDenom(int axisIndex, double *value) {
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getDrv()->getScaleDenom();
  return 0;
}

int getAxisDrvVelSetRaw(int axisIndex, int *value) {
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getDrv()->getVelSetRaw();
  return 0;
}

int getAxisDrvMode(int axisIndex, int *value) {
  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getDrvMode();

  // Differentiate CSP with ecmc outer position control enabled.
  if ((*value == ECMC_DRV_MODE_CSP) &&
      (axes[axisIndex]->getCSPDriveEncoderIndex() >= 0)) {
    *value = 2;  // CSP-PC
  }

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

int getAxisDrvEnabled(int axisIndex, int *value) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  CHECK_AXIS_DRIVE_RETURN_IF_ERROR(axisIndex);
  *value = 0;
  *value = axes[axisIndex]->getDrv()->getEnabled() > 0;
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

int setAxisLimitSwitchBwdPLCOverride(int axisIndex,
                                     int overrideSwitch) {
  LOGINFO4("%s/%s:%d axisIndex=%d overrideSwitch=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    axisIndex,
    overrideSwitch);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setLimitSwitchBwdPLCOverride(overrideSwitch);
}

int setAxisLimitSwitchFwdPLCOverride(int axisIndex,
                                     int overrideSwitch) {
  LOGINFO4("%s/%s:%d axisIndex=%d overrideSwitch=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    axisIndex,
    overrideSwitch);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  
  return axes[axisIndex]->getMon()->setLimitSwitchFwdPLCOverride(overrideSwitch);
}

int setAxisHomeSwitchPLCOverride(int axisIndex,
                                 int overrideSwitch) {
  LOGINFO4("%s/%s:%d axisIndex=%d overrideSwitch=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    axisIndex,
    overrideSwitch);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  
  return axes[axisIndex]->getMon()->setHomeSwitchPLCOverride(overrideSwitch);
}

int setAxisHomeSwitchEnable(int axisIndex,
                            int enable) {
LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
__FILE__,
__FUNCTION__,
__LINE__,
axisIndex,
enable);

CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

return axes[axisIndex]->getMon()->setHomeSwitchEnable(enable);
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

int getAxisMonEnableStallMon(int axisIndex,
                             int *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getEnableStallMon();
  return 0;
}

int setAxisMonEnableStallMon(int axisIndex,
                             int enable) {
  LOGINFO4("%s/%s:%d axisIndex=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           enable);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setEnableStallMon(enable);
  return 0;
}

int getAxisMonStallMinTimeOut(int axisIndex,
                              double *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getStallMinTimeOut();
  return 0;
}

int getAxisMonStallTime(int axisIndex,
                        double *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getStallTime();
  return 0;
}

int getAxisMonStallAtTime(int axisIndex,
                          double *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getStallAtTime();
  return 0;
}

int setAxisMonStallMinTimeOut(int axisIndex,
                             double timeCycles) {
  LOGINFO4("%s/%s:%d axisIndex=%d cycles=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           timeCycles);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setStallMinTimeOut(timeCycles);
  return 0;
}

int getAxisMonStallTimeFactor(int axisIndex,
                              double *value) {
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getStallTimeFactor();
  return 0;
}

int setAxisMonStallTimeFactor(int axisIndex,
                             double timeFactor) {
  LOGINFO4("%s/%s:%d axisIndex=%d factor=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           timeFactor);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  axes[axisIndex]->getMon()->setStallTimeFactor(timeFactor);
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

int setAxisMonAnalogInterlockPolarity(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setAnalogInterlockPolarity(
    (ecmcSwitchPolarity)value);
}

int getAxisMonAnalogInterlockPolarity(int axisIndex, int *pol) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *pol = (int)axes[axisIndex]->getMon()->getAnalogInterlockPolarity();
  return 0;
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

int getAxisMonStopAtAnyLimit(int  axisIndex,
                             int *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  *value = axes[axisIndex]->getMon()->getStopAtAnyLimit();
  return 0;
}

int getAxisMonMaxVelDriveILDelay(int axisIndex, int *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getMon()->getMaxVelDriveTime();
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

int getAxisMonMaxVelTrajILDelay(int axisIndex, int *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getMon()->getMaxVelTrajTime();
  return 0;
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

int setAxisMonStopAtAnyLimit(int axisIndex,
                             int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setStopAtAnyLimit(value);
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

int setAxisMonHomeSwitchEnable(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setHomeSwitchEnable(value);
}

int setAxisMonEnableAnalogInterlock(int axisIndex, int value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setEnableAnalogInterlock(value);
}

int getAxisMonEnableAnalogInterlock(int axisIndex, int *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getMon()->getEnableAnalogInterlock();
  return 0;
}

int setAxisMonAnalogInterlockRawLimit(int axisIndex, double value) {
  LOGINFO4("%s/%s:%d axisIndex=%d value=%f\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           value);

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getMon()->setAnalogRawLimit(value);
}

int getAxisMonAnalogInterlockRawLimit(int axisIndex, double *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getMon()->getAnalogRawLimit();
  return 0;
}

int getAxisMonAnalogInterlockRawValue(int axisIndex, double *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getMon()->getAnalogRawValue();
  return 0;
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

int getAxisMonEnableVelocityDiff(int axisIndex, int *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getMon()->getEnableVelocityDiffMon();
  return 0;
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

int getAxisMonVelDiffTol(int axisIndex, double *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getMon()->getVelDiffMaxDifference();
  return 0;
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

int getAxisMonVelDiffTrajILDelay(int axisIndex, int *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getMon()->getVelDiffTimeTraj();
  return 0;
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

int getAxisMonVelDiffDriveILDelay(int axisIndex, int *value) {
  CHECK_AXIS_MON_RETURN_IF_ERROR(axisIndex);
  *value = axes[axisIndex]->getMon()->getVelDiffTimeDrive();
  return 0;
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
    switch ((axisTypes)type) {
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

  return axes[index]->getErrorID();
}

int addAxisGroup(const char *name) {
  LOGINFO4("%s/%s:%d name=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           name);

  int index = axisGroupCounter;
  if ((index < 0) || (index >= ECMC_MAX_AXES)) {
    return ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;
  }

  for (int i = 0; i < index; i++) {
    if (axisGroups[i] && strcmp(axisGroups[i]->getName(), name) == 0) {
      return ERROR_AXISGRP_NAME_ALREADY_EXISTS;
    }
  }

  // Do not allow create already created axis (must be deleted first)
  if (axisGroups[index] != NULL) {
    return ERROR_MAIN_AXIS_ALREADY_CREATED;
  }

  try {
    axisGroups[index] = new ecmcAxisGroup(index, name);
    axisGroupCounter++;
  }
  catch (std::exception& e) {
    delete axisGroups[index];
    axisGroups[index] = NULL;
    LOGERR("%s/%s:%d: EXCEPTION %s WHEN ALLOCATE MEMORY FOR AXISGROUP OBJECT.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           e.what());
    return ERROR_MAIN_EXCEPTION;
  }
  
  return axisGroups[index]->getErrorID();
}

int getAxisGroupIndexByName(const char* grpName, int *index) {
  
  for(size_t i = 0; i < axisGroupCounter; i++ ) {
    if(!axisGroups[i]) {
      *index = -1;
      return ERROR_AXISGRP_NOT_FOUND;
    }
    if(strcmp(axisGroups[i]->getName(), grpName) == 0) {
      *index = i;
      return 0;
    }
  }
  *index = -1;
  return ERROR_AXISGRP_NOT_FOUND;
}

int getAxisGroupCount(int *count)
{
  *count = axisGroupCounter;
  return 0;
}

int addAxisToGroupByName(int axIndex, const char *grpName) {

  int index = -1;
  int error = getAxisGroupIndexByName(grpName, &index);
  if(error) {
    return error;
  }
  return addAxisToGroupByIndex(axIndex, index);
}

int addAxisToGroupByNameCreate(int axIndex, const char *grpName, int createGrp) {
  int index = -1;
  int error = getAxisGroupIndexByName(grpName, &index);
  if(error && createGrp > 0) {
    // group not found so create it
    error = addAxisGroup(grpName);
    if(error) {
      return error;
    }
  } else if (error){
    return error;
  }
  return addAxisToGroupByName(axIndex,grpName);
}

int addAxisToGroupByIndex(int axIndex, int grpIndex) {

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axIndex);

  if ((grpIndex < 0) ||
      (grpIndex >= static_cast<int>(axisGroupCounter)) ||
      !axisGroups[grpIndex]) {
    return ERROR_AXISGRP_NOT_FOUND;
  }
  
  if ((axIndex < 0) || (axIndex >= ECMC_MAX_AXES)) {
    return ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;
  }

  try {
    axisGroups[grpIndex]->addAxis(axes[axIndex]);    
  }
  catch (std::exception& e) {
    LOGERR("%s/%s:%d: EXCEPTION %s WHEN ADDING AXIS to GROUP.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           e.what());
    return ERROR_MAIN_EXCEPTION;
  }
  return 0;
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

  // Inform slave that it is used in motion axis (SDO settings check)
  slave->setEnableSDOCheck(1);

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

  // Inform slave that it is used in motion axis (SDO settings check)
  slave->setEnableSDOCheck(1);

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

  // Inform slave that it is used in motion axis (SDO settings check)
  slave->setEnableSDOCheck(1);

  // Auto enable fucntionalities
  if (monitorEntryIndex == ECMC_MON_ENTRY_INDEX_EXTINTERLOCK) {
    return setAxisMonEnableExternalInterlock(axisIndex, 1);
  }

  if (monitorEntryIndex == ECMC_MON_ENTRY_INDEX_HOMESENSOR) {
    return setAxisMonHomeSwitchEnable(axisIndex, 1);
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

  // Inform slave that it is used in motion axis (SDO settings check)
  slave->setEnableSDOCheck(1);

  return axes[axisIndex]->setEcStatusOutputEntry(entry);
}

int linkEcEntryToPVTController(int   slaveIndex,
                               char *entryIDString,
                               int   entryIndex,
                               int   bitIndex) {
  LOGINFO4("%s/%s:%d slave_index=%d entry=%s, entryIndex=%d, bitIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slaveIndex,
           entryIDString,
           entryIndex,
           bitIndex);

  if (!ec) return ERROR_MAIN_EC_NOT_INITIALIZED;

  if(!pvtCtrl_) return ERROR_PVT_CTRL_NULL;

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

  return pvtCtrl_->setEcEntry(entry, entryIndex, bitIndex);
}

int linkEcEntryToAxisSeqAutoModeSet(int   slaveIndex,
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

  // Inform slave that it is used in motion axis (SDO settings check)
  slave->setEnableSDOCheck(1);

  return axes[axisIndex]->getSeq()->setAutoModeSetEntry(entry);
}

int linkEcEntryToAxisSeqAutoModeAct(int   slaveIndex,
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

  // Inform slave that it is used in motion axis (SDO settings check)
  slave->setEnableSDOCheck(1);

  return axes[axisIndex]->getSeq()->setAutoModeActEntry(entry);
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

int setAxisAutoModeCmdMotion(int axisIndex,
                             int cmd) {
  LOGINFO4("%s/%s:%d axisIndex=%d, Mode cmd=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           cmd);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getSeq()->setAutoModeMotionCmd(cmd);
}

int setAxisAutoModeCmdHoming(int axisIndex,
                             int cmd) {
  LOGINFO4("%s/%s:%d axisIndex=%d, Mode cmd=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           cmd);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  return axes[axisIndex]->getSeq()->setAutoModeHomigCmd(cmd);
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

int getAxisValid(int axisIndex) {
  LOGINFO4("%s/%s:%d axisIndex=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);

  if ((axisIndex >= ECMC_MAX_AXES) || (axisIndex <= 0)) {
    return 0;
  }
  return axes[axisIndex] != NULL;
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

int setAxisAutoResetError(int axisIndex,
                       int autoResetError) {
  LOGINFO4("%s/%s:%d axisIndex=%d, autoResetError=%d \n",
  __FILE__,
  __FUNCTION__,
  __LINE__,
  axisIndex,
  autoResetError);

CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

return axes[axisIndex]->setAutoResetError(autoResetError);
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

int setAxisEmergencyStopInterlock(int axisIndex, int stop) {
  LOGINFO4("%s/%s:%d axisIndex=%d, stop=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           stop);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  axes[axisIndex]->setEmergencyStopInterlock(stop);
  return 0;
}

int setAxisExtMaxVelo(int axisIndex,                             
                      double veloLimit,
                      int active) {
  LOGINFO4("%s/%s:%d axisIndex=%d, veloLimit = %lf, active=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex,
           veloLimit,
           active);

  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  axes[axisIndex]->setExternalMaxVelo(veloLimit,
                                      active);
  return 0;
}

int getAxisEncVelo(int     axisIndex,
                   double *velo) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  *velo = axes[axisIndex]->getEncVelo();
  return 0;
}

/** \brief Get axis traj setpoint velo.\n
 *
 * The axis busy bit is high while an command is executed or while synchronizing to other axes.
 * \param[in] axisIndex  Axis index.\n
 * \param[out] velo Axis velocity.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 */
int getAxisTrajVelo(int     axisIndex,
                    double *velo) {
  LOGINFO4("%s/%s:%d axisIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);
  *velo = axes[axisIndex]->getTrajVelo();
  return 0;
}

void* getAxisPointer(int  axisIndex) {
  LOGINFO4("%s/%s:%d axisIndex=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           axisIndex);
  if (axisIndex >= ECMC_MAX_AXES || axisIndex <= 0) {
    LOGERR("ERROR: Axis index out of range.\n");
    return NULL;
  }
  return (void*)axes[axisIndex];  
}

int createMasterSlaveSM(int index,
                        const char *name,
                        const char *masterGrpName,
                        const char* slaveGrpName,
                        int autoDisableMasters,
                        int autoDisableSlaves) {
  LOGINFO4("%s/%s:%d index=%d ,name=%s, master=%s, slave=%s\n", 
           __FILE__, __FUNCTION__, __LINE__, 
           index, name, masterGrpName, slaveGrpName);
  CHECK_MST_SLBV_SM_IDNEX_RETURN_IF_ERROR(index)

  ecmcAxisGroup *masterGrp = NULL;
  ecmcAxisGroup *slaveGrp  = NULL;
  for(int i = 0;i < ECMC_MAX_AXES; i++) {
    if(axisGroups[i] != NULL) {
      LOGINFO4("%s/%s:%d comparing axis group %s with master=%s, slave=%s\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               axisGroups[i]->getName(),
               masterGrpName,
               slaveGrpName);
      if(strcmp(axisGroups[i]->getName(),masterGrpName) == 0) {
        masterGrp = axisGroups[i];
      }
      if(strcmp(axisGroups[i]->getName(),slaveGrpName) == 0) {
        slaveGrp = axisGroups[i];
      }
    }
  }

  if(masterGrp == NULL || slaveGrp == NULL) {
    return ERROR_GROUP_NULL;
  }

  masterSlaveSMs[index] = new ecmcMasterSlaveStateMachine(asynPort,
                                                          index,
                                                          name,
                                                          1.0 / mcuFrequency,
                                                          masterGrp,
                                                          slaveGrp,
                                                          autoDisableMasters,
                                                          autoDisableSlaves);
  return 0;
}
