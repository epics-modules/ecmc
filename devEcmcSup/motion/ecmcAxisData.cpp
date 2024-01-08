/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAxisData.cpp
*
*  Created on: Jan 24, 2017
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcAxisData.h"
#include "ecmcOctetIF.h"

ecmcAxisData::ecmcAxisData() {
  axisId_     = 0;
  sampleTime_ = 0;
  axisType_   = ECMC_AXIS_TYPE_BASE;
  memset(&command_,    0, sizeof(command_));
  memset(&status_,     0, sizeof(status_));
  memset(&interlocks_, 0, sizeof(interlocks_));
}

ecmcAxisData::~ecmcAxisData() {}

stopMode ecmcAxisData::refreshInterlocksInternal() {
  setSummaryInterlocks();

  // If no summary interlocks then no interlocks
  if (!interlocks_.driveSummaryInterlock &&
      !interlocks_.trajSummaryInterlockBWD  &&
      !interlocks_.trajSummaryInterlockFWD) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_NONE;
    interlocks_.currStopMode    = ECMC_STOP_MODE_RUN;
    return interlocks_.currStopMode;
  }

  // Emergency interlocks first
  if (interlocks_.axisErrorStateInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_AXIS_ERROR_STATE;
    interlocks_.currStopMode    = ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if (interlocks_.bothLimitsLowInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_BOTH_LIMITS;
    interlocks_.currStopMode    = ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if (interlocks_.unexpectedLimitSwitchBehaviourInterlock) {
    interlocks_.interlockStatus =
      ECMC_INTERLOCK_UNEXPECTED_LIMIT_SWITCH_BEHAVIOUR;
    interlocks_.currStopMode = ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if (interlocks_.velocityDiffDriveInterlock ||
      interlocks_.velocityDiffTrajInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_VELOCITY_DIFF;
    interlocks_.currStopMode    = ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if (interlocks_.cntrlOutputHLDriveInterlock ||
      interlocks_.cntrlOutputHLTrajInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_CONT_HIGH_LIMIT;
    interlocks_.currStopMode    = ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if (interlocks_.etherCatMasterInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_ETHERCAT_MASTER_NOT_OK;
    interlocks_.currStopMode    = ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if (interlocks_.hardwareInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_EXTERNAL;
    interlocks_.currStopMode    = ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if (interlocks_.lagDriveInterlock || interlocks_.lagTrajInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_POSITION_LAG;
    interlocks_.currStopMode    = ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if (interlocks_.bwdLimitInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_HARD_BWD;
    interlocks_.currStopMode    = ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if (interlocks_.encTransformInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_TRANSFORM;
    interlocks_.currStopMode    = ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if (interlocks_.fwdLimitInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_HARD_FWD;
    interlocks_.currStopMode    = ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if (interlocks_.bwdSoftLimitInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_SOFT_BWD;
    interlocks_.currStopMode    = ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if (interlocks_.fwdSoftLimitInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_SOFT_FWD;
    interlocks_.currStopMode    = ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if (interlocks_.maxVelocityDriveInterlock ||
      interlocks_.maxVelocityTrajInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_MAX_SPEED;
    interlocks_.currStopMode    = ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if (interlocks_.noExecuteInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_NO_EXECUTE;
    interlocks_.currStopMode    = ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if (interlocks_.trajTransformInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_TRANSFORM;
    interlocks_.currStopMode    = ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if (interlocks_.plcInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_PLC_NORMAL;
    interlocks_.currStopMode    = ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if (interlocks_.plcInterlockBWD) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_PLC_BWD;
    interlocks_.currStopMode    = ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if (interlocks_.plcInterlockFWD) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_PLC_FWD;
    interlocks_.currStopMode    = ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if (interlocks_.encDiffInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_ENC_DIFF;
    interlocks_.currStopMode    = ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  interlocks_.interlockStatus = ECMC_INTERLOCK_NONE;

  interlocks_.currStopMode = ECMC_STOP_MODE_RUN;

  return interlocks_.currStopMode;
}

stopMode ecmcAxisData::refreshInterlocks() {
  interlockTypes oldInterlock = interlocks_.interlockStatus;

  stopMode stop = refreshInterlocksInternal();

  // Latch latest active interlock
  if ((interlocks_.interlockStatus != ECMC_INTERLOCK_NONE) &&
      (interlocks_.interlockStatus != interlocks_.lastActiveInterlock)) {
    interlocks_.lastActiveInterlock = interlocks_.interlockStatus;
  }

  if (oldInterlock != interlocks_.interlockStatus) {
    if (interlocks_.interlockStatus) {
      LOGERR("%s/%s:%d: INFO (axis %d): Motion interlocked (type %d).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             axisId_,
             interlocks_.interlockStatus);
    } else {
      LOGERR("%s/%s:%d: INFO (axis %d): Motion interlock cleared.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             axisId_);
    }
  }
  return stop;
}

int ecmcAxisData::setSummaryInterlocks() {
  interlocks_.driveSummaryInterlock = interlocks_.bothLimitsLowInterlock
                                      || interlocks_.bothLimitsLowInterlock
                                      || interlocks_.
                                      cntrlOutputHLDriveInterlock
                                      || interlocks_.lagDriveInterlock
                                      || interlocks_.maxVelocityDriveInterlock
                                      || interlocks_.velocityDiffDriveInterlock
                                      || interlocks_.hardwareInterlock
                                      || interlocks_.etherCatMasterInterlock;

  interlocks_.trajSummaryInterlockBWD = interlocks_.driveSummaryInterlock
                                        || interlocks_.axisErrorStateInterlock
                                        || interlocks_.bwdLimitInterlock
                                        || interlocks_.bwdSoftLimitInterlock
                                        || interlocks_.
                                        cntrlOutputHLTrajInterlock
                                        || interlocks_.encTransformInterlock
                                        || interlocks_.lagTrajInterlock
                                        || interlocks_.maxVelocityTrajInterlock
                                        || interlocks_.noExecuteInterlock
                                        || interlocks_.trajTransformInterlock
                                        || interlocks_.
                                        unexpectedLimitSwitchBehaviourInterlock
                                        || interlocks_.
                                        velocityDiffTrajInterlock
                                        || interlocks_.plcInterlock
                                        || interlocks_.plcInterlockBWD
                                        || interlocks_.encDiffInterlock;

  interlocks_.trajSummaryInterlockFWD = interlocks_.driveSummaryInterlock
                                        || interlocks_.axisErrorStateInterlock
                                        || interlocks_.
                                        cntrlOutputHLTrajInterlock
                                        || interlocks_.encTransformInterlock
                                        || interlocks_.fwdLimitInterlock
                                        || interlocks_.fwdSoftLimitInterlock
                                        || interlocks_.lagTrajInterlock
                                        || interlocks_.maxVelocityTrajInterlock
                                        || interlocks_.noExecuteInterlock
                                        || interlocks_.trajTransformInterlock
                                        || interlocks_.
                                        unexpectedLimitSwitchBehaviourInterlock
                                        || interlocks_.
                                        velocityDiffTrajInterlock
                                        || interlocks_.plcInterlock
                                        || interlocks_.plcInterlockFWD
                                        || interlocks_.encDiffInterlock;

  return 0;
}

void ecmcAxisData::clearInterlocks() {
  // Clear and refresh struct
  memset(&interlocks_, 0, sizeof(interlocks_));
  refreshInterlocks();
}
