/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcTrajectoryBase.cpp
*
*  Created on: Nov 26, 2021
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcTrajectoryBase.h"
#include <stdio.h>

ecmcTrajectoryBase::ecmcTrajectoryBase(ecmcAxisData *axisData,
                                       double        sampleTime)
  : ecmcError(&(axisData->status_.errorCode),
              &(axisData->status_.warningCode)) {
  data_ = axisData;
  setExternalPtrs(&(data_->status_.errorCode), &(data_->status_.warningCode));
  initVars();

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
  sampleTime_ = sampleTime;
}

ecmcTrajectoryBase::~ecmcTrajectoryBase() {}

void ecmcTrajectoryBase::initVars() {
  errorReset();
  distToStop_                  = 0;
  targetVelocity_              = 0;
  targetAcceleration_          = 0;
  targetDeceleration_          = 0;
  targetDecelerationEmerg_     = 0;
  targetJerk_                  = 0;
  sampleTime_                  = 1;
  posSetMinus1_                = 0;
  targetPosition_              = 0;
  currentPositionSetpoint_     = 0;
  currentVelocitySetpoint_     = 0;
  currentAccelerationSetpoint_ = 0;
  index_                       = 0;
  execute_                     = 0;
  executeOld_                  = 0;
  startPosition_               = 0;
  enable_                      = false;
  enableOld_                   = false;
  busy_                        = false;
  motionMode_                  = ECMC_MOVE_MODE_POS;
  latchedStopMode_             = ECMC_STOP_MODE_RUN;
}

double ecmcTrajectoryBase::getCurrentPosSet() {
  return currentPositionSetpoint_;
}

bool ecmcTrajectoryBase::getBusy() {
  return busy_;
}

int ecmcTrajectoryBase::getIndex() {
  return index_;
}

double ecmcTrajectoryBase::getVel() {
  return currentVelocitySetpoint_;
}

double ecmcTrajectoryBase::getTargetVel() {
  return targetVelocity_;
}

double ecmcTrajectoryBase::getAcc() {
  return targetAcceleration_;
}

double ecmcTrajectoryBase::getDec() {
  return targetDeceleration_;
}

double ecmcTrajectoryBase::getJerk() {
  return targetJerk_;
}

double ecmcTrajectoryBase::getTargetPos() {
  return targetPosition_;
}

bool ecmcTrajectoryBase::getExecute() {
  return execute_;
}

void ecmcTrajectoryBase::setEnable(bool enable) {
  enableOld_               = enable_;
  enable_                  = enable;
  currentVelocitySetpoint_ = 0;

  if (!enableOld_ && enable_) {
    posSetMinus1_            = startPosition_;
    currentPositionSetpoint_ = startPosition_;
    currentVelocitySetpoint_ = 0;
    distToStop_              = 0;
  }

  if (!enable_) {
    busy_ = false;
  }
}

bool ecmcTrajectoryBase::getEnable() {
  return enable_;
}

int ecmcTrajectoryBase::setExecute(bool execute) {
  executeOld_ = execute_;
  execute_    = execute;

  if (!enable_ && execute_) {
    LOGERR("%s/%s:%d: ERROR: Trajectory not enabled (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_TRAJ_NOT_ENABLED);
    execute_                     = false;
    currentVelocitySetpoint_     = 0;
    currentAccelerationSetpoint_ = 0;
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_TRAJ_NOT_ENABLED);
  }

  if (!executeOld_ && execute_) {
    posSetMinus1_            = startPosition_;
    currentPositionSetpoint_ = startPosition_;

    if (!busy_) {
      posSetMinus1_            = currentPositionSetpoint_;
      currentVelocitySetpoint_ = 0;
    }
    initTraj();
    currentPositionSetpoint_              = startPosition_;
    busy_                                 = true; // Trigger new trajectory
    data_->interlocks_.noExecuteInterlock = false;
    data_->refreshInterlocks();
  }
  return 0;
}

void ecmcTrajectoryBase::initTraj() {
  // Default empty..
}

void ecmcTrajectoryBase::setStartPos(double pos) {
  startPosition_ = checkModuloPos(pos);
}

void ecmcTrajectoryBase::setMotionMode(motionMode mode) {
  motionMode_ = mode;
}

interlockTypes ecmcTrajectoryBase::getInterlockStatus() {
  return interlockStatus_;
}

double ecmcTrajectoryBase::getSampleTime() {
  return sampleTime_;
}

int ecmcTrajectoryBase::validate() {
  if (sampleTime_ <= 0) {
    LOGERR("%s/%s:%d: ERROR: Sample time out of range (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_TRAJ_INVALID_SAMPLE_TIME);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_TRAJ_INVALID_SAMPLE_TIME);
  }

  if (data_->command_.moduloRange < 0) {
    LOGERR("%s/%s:%d: ERROR: Modulo factor out of range (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_TRAJ_MOD_FACTOR_OUT_OF_RANGE);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_TRAJ_MOD_FACTOR_OUT_OF_RANGE);
  }

  return 0;
}

motionDirection ecmcTrajectoryBase::checkDirection(double oldPos,
                                                   double newPos) {
  double diff = newPos - oldPos;

  // No modulo or no overflow in modulo
  if ((data_->command_.moduloRange == 0) ||
      (std::abs(diff) <
       data_->command_.moduloRange * ECMC_OVER_UNDER_FLOW_FACTOR)) {
    if (newPos > oldPos) {
      return ECMC_DIR_FORWARD;
    } else if (newPos < oldPos) {
      return ECMC_DIR_BACKWARD;
    }
    return ECMC_DIR_STANDSTILL;
  }

  // Overflow in modulo
  if (diff > 0) {
    return ECMC_DIR_BACKWARD;
  } else if (diff < 0) {
    return ECMC_DIR_FORWARD;
  }

  return ECMC_DIR_STANDSTILL;
}

motionDirection ecmcTrajectoryBase::getCurrSetDir() {
  return targetVelocity_ > 0 ? ECMC_DIR_FORWARD : ECMC_DIR_BACKWARD;
}

/* Calculates distance between two points*/
double ecmcTrajectoryBase::dist(double from, double to) {
  return to - from;
}

/* Calculates distance between two points, takes modulo into account*/
double ecmcTrajectoryBase::distModulo(double          from,
                                      double          to,
                                      motionDirection direction) {
  // check if modulo enabled
  if (data_->command_.moduloRange == 0) {
    return dist(from, to);
  }

  // modulo
  switch (direction) {
  case ECMC_DIR_BACKWARD:

    if (from >= to) {
      return to - from;
    }
    return -from - (data_->command_.moduloRange - to);

    break;

  case ECMC_DIR_FORWARD:

    if (from <= to) {
      return to - from;
    }
    return to + (data_->command_.moduloRange - from);

    break;

  case ECMC_DIR_STANDSTILL:
    return 0;
  }
  return 0;
}

double ecmcTrajectoryBase::checkModuloPos(double pos) {
  // check modulo (allowed range 0..data_->command_.moduloRange)
  double posSetTemp = pos;

  if (data_->command_.moduloRange != 0) {
    if (posSetTemp >= 0) {
      posSetTemp = std::fmod(posSetTemp, data_->command_.moduloRange);
    } else {
      posSetTemp = std::fmod(posSetTemp, data_->command_.moduloRange) +
                   data_->command_.moduloRange;
    }
  }

  return posSetTemp;
}

void ecmcTrajectoryBase::setCurrentPosSet(double posSet) {
  currentPositionSetpoint_     = posSet;
  posSetMinus1_                = posSet;
  currentVelocitySetpoint_     = 0;
  currentAccelerationSetpoint_ = 0;
}

void ecmcTrajectoryBase::setTargetPos(double pos) {
  double distFWD = 0;
  double distBWD = 0;

  index_ = 0;


  // Modulo motion
  if (data_->command_.moduloRange > 0) {
    // Do not allow on the fly change for modulo motion
    if (busy_) {
      LOGERR(
        "%s/%s:%d: ERROR: Setpoint change while busy not allowed in modulo mode (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_TRAJ_MOD_POS_CHANGE_WHILE_BUSY_NOT_ALLOWED);
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_TRAJ_MOD_POS_CHANGE_WHILE_BUSY_NOT_ALLOWED);
      return;
    }

    pos = checkModuloPos(pos);

    switch (data_->command_.moduloType) {
    case ECMC_MOD_MOTION_BWD:

      if (currentPositionSetpoint_ < pos) {
        pos = pos - data_->command_.moduloRange;
      }

      break;

    case ECMC_MOD_MOTION_FWD:

      if (currentPositionSetpoint_ > pos) {
        pos = pos + data_->command_.moduloRange;
      }

      break;

    case ECMC_MOD_MOTION_NORMAL:

      break;

    case ECMC_MOD_MOTION_CLOSEST:

      distFWD =
        std::abs(distModulo(currentPositionSetpoint_, pos, ECMC_DIR_FORWARD));
      distBWD =
        std::abs(distModulo(currentPositionSetpoint_, pos, ECMC_DIR_BACKWARD));

      if (distBWD < distFWD) {
        if (currentPositionSetpoint_ < pos) {
          pos = pos - data_->command_.moduloRange;
        }
      } else {
        if (currentPositionSetpoint_ > pos) {
          pos = pos + data_->command_.moduloRange;
        }
      }
      break;

    default:
      LOGERR("%s/%s:%d: ERROR: Modulo type out of range (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             ERROR_TRAJ_MOD_TYPE_OUT_OF_RANGE);
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_TRAJ_MOD_TYPE_OUT_OF_RANGE);
      break;
    }
  }
  targetPosition_ = pos;
  setTargetPosLocal(pos);
}

void ecmcTrajectoryBase::setTargetVel(double velTarget) {
  targetVelocity_ = velTarget;
  initTraj();
}

void ecmcTrajectoryBase::setAcc(double acc) {
  targetAcceleration_ = acc;
  initTraj();
}

void ecmcTrajectoryBase::setEmergDec(double dec) {
  targetDecelerationEmerg_ = dec;
  initTraj();
}

void ecmcTrajectoryBase::setDec(double dec) {
  targetDeceleration_ = dec;

  if (targetDecelerationEmerg_ == 0) {
    targetDecelerationEmerg_ = targetDeceleration_ * 3;
  }
  initTraj();
}

// Not used
void ecmcTrajectoryBase::setJerk(double jerk) {
  targetJerk_ = jerk;
  initTraj();
}

int ecmcTrajectoryBase::initStopRamp(double currentPos,
                                     double currentVel,
                                     double currentAcc) {
  enable_                  = true;
  busy_                    = true;
  currentPositionSetpoint_ = currentPos;
  currentVelocitySetpoint_ = currentVel;
  return 0;
}

double ecmcTrajectoryBase::getNextPosSet() {
  // "Main" of trajectory generator. Needs to be called exactly once per cycle.
  // Updates trajectory setpoint

  // Some error occured. Axis lost enable.. abort
  if (busy_ && !data_->status_.enabled) {
    double tempPos = getCurrentPosSet();
    busy_ = false;
    updateSetpoint(tempPos, 0, 0, busy_);
  }

  if (!busy_ || !enable_) {
    if (execute_ && !enable_) {
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_TRAJ_EXECUTE_BUT_NO_ENABLE);
    }
    double tempPos = getCurrentPosSet();
    updateSetpoint(tempPos, 0, 0, busy_);

    // setCurrentPosSet(tempPos);
    return tempPos;
  }

  index_++;

  if (!execute_ && (data_->command_.trajSource == ECMC_DATA_SOURCE_INTERNAL)) {
    data_->interlocks_.noExecuteInterlock = true;
    data_->refreshInterlocks();
  }

  double nextSetpoint     = currentPositionSetpoint_;
  double nextVelocity     = currentVelocitySetpoint_;
  double nextAcceleration = currentAccelerationSetpoint_;

  // Execute derived class traj generator
  nextSetpoint = internalTraj(&nextVelocity, &nextAcceleration, &busy_);

  return updateSetpoint(nextSetpoint, nextVelocity, nextAcceleration, busy_);
}

double ecmcTrajectoryBase::updateSetpoint(double nextSetpoint,
                                          double nextVelocity,
                                          double nextAcceleration,
                                          bool   busy) {
  posSetMinus1_                = currentPositionSetpoint_;
  currentPositionSetpoint_     = checkModuloPos(nextSetpoint);
  currentVelocitySetpoint_     = nextVelocity;
  currentAccelerationSetpoint_ = nextAcceleration;
  busy_                        = busy;
  distToStop_                  = distToStop(currentVelocitySetpoint_);

  // In velocity mode always update taregetposition
  if (motionMode_ == ECMC_MOVE_MODE_VEL) {
    targetPosition_ = currentPositionSetpoint_;
  }
  return currentPositionSetpoint_;
}

double ecmcTrajectoryBase::getTargetPosMod() {
  return checkModuloPos(targetPosition_);
}
