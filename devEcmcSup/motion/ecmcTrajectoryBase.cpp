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
                                             double        sampleTime) :
  ecmcError() {
  data_ = axisData;
  initVars();

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
  sampleTime_ = sampleTime;
}

ecmcTrajectoryBase::ecmcTrajectoryBase(ecmcAxisData *axisData,
                                       double        velocityTarget,
                                       double        acceleration,
                                       double        deceleration,
                                       double        jerk,
                                       double        sampleTime) :
  ecmcError() {
  data_ = axisData;
  initVars();
  velocityTarget_        = velocityTarget;
  acceleration_          = acceleration;
  deceleration_          = deceleration;
  decelerationEmergency_ = deceleration;
  jerk_                  = jerk;
  sampleTime_            = sampleTime;  
}

ecmcTrajectoryBase::~ecmcTrajectoryBase()
{}

void ecmcTrajectoryBase::initVars() {
  errorReset();
  distToStop_              = 0;
  velocityTarget_          = 0;
  acceleration_            = 0;
  deceleration_            = 0;
  decelerationEmergency_   = 0;
  jerk_                    = 0;
  sampleTime_              = 1;
  posSetMinus1_            = 0;
  targetPosition_          = 0;
  currentPositionSetpoint_ = 0;
  velocity_                = 0;
  busy_                    = false;
  index_                   = 0;
  execute_                 = 0;
  executeOld_              = 0;
  startPosition_           = 0;
  enable_                  = false;
  enableOld_               = false;
  motionMode_              = ECMC_MOVE_MODE_POS;
  latchedStopMode_         = ECMC_STOP_MODE_RUN;
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
  return velocity_;
}

double ecmcTrajectoryBase::getTargetVel() {
  return velocityTarget_;
}

double ecmcTrajectoryBase::getAcc() {
  return acceleration_;
}

double ecmcTrajectoryBase::getDec() {
  return deceleration_;
}

double ecmcTrajectoryBase::getJerk() {
  return jerk_;
}

double ecmcTrajectoryBase::getTargetPos() {
  return targetPosition_;
}

bool ecmcTrajectoryBase::getExecute() {
  return execute_;
}

void ecmcTrajectoryBase::setEnable(bool enable) {
  enableOld_ = enable_;
  enable_    = enable;
  velocity_  = 0;

  if (!enableOld_ && enable_) {
    posSetMinus1_            = startPosition_;
    currentPositionSetpoint_ = startPosition_;   
    velocity_                = 0;
    distToStop_              = 0;
  }
}

bool ecmcTrajectoryBase::getEnable() {
  return enable_;
}

int ecmcTrajectoryBase::setExecute(bool execute) {
  double distFWD = 0;
  double distBWD = 0;
  executeOld_ = execute_;
  execute_    = execute;

  if (!enable_ && execute_) {
    LOGERR("%s/%s:%d: ERROR: Trajectory not enabled (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_TRAJ_NOT_ENABLED);    
    execute_  = false;
    velocity_ = 0;
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_TRAJ_NOT_ENABLED);;
  }

  if (!executeOld_ && execute_) {
    currentPositionSetpoint_ = startPosition_;

    switch (motionMode_) {
    case ECMC_MOVE_MODE_VEL:

      break;

    case ECMC_MOVE_MODE_POS:

      // Normal motion
      if(data_->command_.moduloRange==0) {
        if (targetPosition_ < currentPositionSetpoint_) {
          velocityTarget_= -std::abs(velocityTarget_);
        } else {
          velocityTarget_= std::abs(velocityTarget_);
        }
        break;
      } else { 
     
        if(data_->command_.moduloRange < 0) {
          LOGERR("%s/%s:%d: ERROR: Modulo factor out of range (0x%x).\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            ERROR_TRAJ_MOD_FACTOR_OUT_OF_RANGE);
          return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_TRAJ_MOD_FACTOR_OUT_OF_RANGE);
        }

        // Modulo motion
        switch (data_->command_.moduloType)
        {
        case ECMC_MOD_MOTION_BWD:

          velocityTarget_=-std::abs(velocityTarget_);
          break;

        case ECMC_MOD_MOTION_FWD:

          velocityTarget_= std::abs(velocityTarget_);
          break;

        case ECMC_MOD_MOTION_NORMAL:

          if (targetPosition_ < currentPositionSetpoint_) {
            velocityTarget_=-std::abs(velocityTarget_);
          } else {
            velocityTarget_= std::abs(velocityTarget_);
          }
          break;
        case ECMC_MOD_MOTION_CLOSEST:        

          distFWD = std::abs(dist(currentPositionSetpoint_,targetPosition_,ECMC_DIR_FORWARD));
          distBWD = std::abs(dist(currentPositionSetpoint_,targetPosition_,ECMC_DIR_BACKWARD));
          if(distBWD < distFWD) {
            velocityTarget_=-std::abs(velocityTarget_);
          } else {
            velocityTarget_= std::abs(velocityTarget_);
          }

          break;

        default:
          LOGERR("%s/%s:%d: ERROR: Modulo type out of range (0x%x).\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            ERROR_TRAJ_MOD_TYPE_OUT_OF_RANGE);
          return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_TRAJ_MOD_TYPE_OUT_OF_RANGE);
          break;
        }
      }
    }

    if (!busy_) {
      posSetMinus1_ = currentPositionSetpoint_;
      velocity_     = 0;
    }
    initTraj();
    currentPositionSetpoint_ = startPosition_;
    busy_                    = true;  // Trigger new trajectory
    data_->interlocks_.noExecuteInterlock = false;
    data_->refreshInterlocks();
  }
  return 0;
}

void ecmcTrajectoryBase::setStartPos(double pos) {
  startPosition_ = pos;
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
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_TRAJ_INVALID_SAMPLE_TIME);
  }
  return 0;
}

motionDirection ecmcTrajectoryBase::checkDirection(double oldPos,
                                                      double newPos) {
  double diff = newPos-oldPos;
  //No modulo or no overflow in modulo
  if(data_->command_.moduloRange==0 || (std::abs(diff) < data_->command_.moduloRange*ECMC_OVER_UNDER_FLOW_FACTOR)) {
    if (newPos > oldPos) {
      return ECMC_DIR_FORWARD;
    } 
    else if (newPos < oldPos) {
      return ECMC_DIR_BACKWARD;
    }
    return ECMC_DIR_STANDSTILL;
  }

  //Overflow in modulo
  if(diff > 0) {
    return ECMC_DIR_BACKWARD;
  }
  else if(diff < 0) {
   return ECMC_DIR_FORWARD;
  }

  return ECMC_DIR_STANDSTILL;
}

motionDirection ecmcTrajectoryBase::getCurrSetDir() {
  return velocityTarget_>0 ? ECMC_DIR_FORWARD:ECMC_DIR_BACKWARD;
}

/* Calculates distance between two points*/
double ecmcTrajectoryBase::dist(double from, double to, motionDirection direction) {
  //check if modulo enabled
  if(data_->command_.moduloRange==0){
    return to-from;
  }
  
  //modulo
  switch(direction){
    case ECMC_DIR_BACKWARD:
      if(from >= to){
        return to-from;
      }
      return -from-(data_->command_.moduloRange-to);
    break;
    case ECMC_DIR_FORWARD:
      if(from <= to){
        return to-from;
      }
      return to + (data_->command_.moduloRange-from);
    break;
    
    case ECMC_DIR_STANDSTILL:
      return 0;
  }
  return 0;
}

double ecmcTrajectoryBase::checkModuloPos(double pos,
                                             motionDirection direction) {
  //check modulo (allowed range 0..data_->command_.moduloRange)
  double posSetTemp = pos;
  if(data_->command_.moduloRange!=0) {
    if (direction == ECMC_DIR_FORWARD) {
      if(posSetTemp >= data_->command_.moduloRange) {
        posSetTemp = posSetTemp - data_->command_.moduloRange;
      }
    } else {
      if(posSetTemp < 0) {
        posSetTemp = data_->command_.moduloRange + posSetTemp;
      }
    }
  }
  
  return posSetTemp;
}
