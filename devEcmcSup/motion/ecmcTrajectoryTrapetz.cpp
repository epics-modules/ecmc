/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcTrajectoryTrapetz.cpp
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcTrajectoryTrapetz.h"
#include <stdio.h>

ecmcTrajectoryTrapetz::ecmcTrajectoryTrapetz(ecmcAxisData *axisData,
                                             double        sampleTime) :
  ecmcError() {
  data_ = axisData;
  initVars();

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
  sampleTime_ = sampleTime;
  initTraj();
}

ecmcTrajectoryTrapetz::ecmcTrajectoryTrapetz(ecmcAxisData *axisData,
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
  initTraj();
}

ecmcTrajectoryTrapetz::~ecmcTrajectoryTrapetz()
{}

void ecmcTrajectoryTrapetz::initVars() {
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
  stepACC_                 = 0;
  stepDEC_                 = 0;
  stepNOM_                 = 0;
  stepDECEmerg_            = 0;
  velocity_                = 0;
  busy_                    = false;
  index_                   = 0;
  execute_                 = 0;
  executeOld_              = 0;
  startPosition_           = 0;
  enable_                  = false;
  enableOld_               = false;
  motionMode_              = ECMC_MOVE_MODE_POS;
  prevStepSize_            = 0;
  setDirection_            = ECMC_DIR_FORWARD;
  actDirection_            = ECMC_DIR_FORWARD;
  latchedStopMode_         = ECMC_STOP_MODE_RUN;
}

void ecmcTrajectoryTrapetz::initTraj() {
  stepNOM_      = std::abs(velocityTarget_) * sampleTime_;
  // TODO check eq.
  stepACC_      =  /*0.5**/ acceleration_ * sampleTime_ * sampleTime_  /**2*/;
  stepDEC_      =  /*0.5**/ deceleration_ * sampleTime_ * sampleTime_  /**2*/;
  stepDECEmerg_ =  /*0.5**/ decelerationEmergency_ * sampleTime_ *
                  sampleTime_  /**2*/;
}

double ecmcTrajectoryTrapetz::getCurrentPosSet() {
  return currentPositionSetpoint_;
}

void ecmcTrajectoryTrapetz::setCurrentPosSet(double posSet) {
  currentPositionSetpoint_ = posSet;
  posSetMinus1_            = currentPositionSetpoint_;
  prevStepSize_            = 0;
  velocity_                = 0;
  setDirection_            = ECMC_DIR_STANDSTILL;
  actDirection_            = ECMC_DIR_STANDSTILL;
}

double ecmcTrajectoryTrapetz::getNextPosSet() {
  // "Main" of trajectory generator. Needs to be called exactly once per cycle.
  // Updates trajectory setpoint

  double nextSetpoint = currentPositionSetpoint_;
  double nextVelocity = velocity_;

  bool stopped = false;

  if (!busy_ || !enable_) {
    if (execute_ && !enable_) {
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_TRAJ_EXECUTE_BUT_NO_ENABLE);
    }
    posSetMinus1_ = currentPositionSetpoint_;
    prevStepSize_ = 0;
    velocity_     = 0;
    setDirection_ = ECMC_DIR_STANDSTILL;
    actDirection_ = ECMC_DIR_STANDSTILL;
    return currentPositionSetpoint_;
  }
  index_++;

  if (!execute_ && (data_->command_.trajSource == ECMC_DATA_SOURCE_INTERNAL)) {
    data_->interlocks_.noExecuteInterlock = true;
    data_->refreshInterlocks();
  }

  nextSetpoint = internalTraj(&nextVelocity);
  motionDirection nextDir = checkDirection(currentPositionSetpoint_,
                                           nextSetpoint);
  // Stop ramp when running external
  bool externalSourceStopTraj = data_->command_.trajSource !=
                                ECMC_DATA_SOURCE_INTERNAL;

  if (externalSourceStopTraj ||
      ((nextDir == ECMC_DIR_BACKWARD) &&
       data_->interlocks_.trajSummaryInterlockBWD) ||
      ((nextDir == ECMC_DIR_FORWARD) &&
       data_->interlocks_.trajSummaryInterlockFWD)) {
    nextSetpoint = moveStop(data_->interlocks_.currStopMode,
                            currentPositionSetpoint_,
                            velocity_,
                            velocityTarget_,
                            &stopped,
                            &nextVelocity);

    if (stopped) {
      setDirection_ = ECMC_DIR_STANDSTILL;
      actDirection_ = ECMC_DIR_STANDSTILL;
      busy_         = false;
      nextVelocity  = 0;
    }
  }
  actDirection_ = checkDirection(currentPositionSetpoint_,
                                 nextSetpoint);
  currentPositionSetpoint_ = updateSetpoint(nextSetpoint, nextVelocity);

  return currentPositionSetpoint_;
}

double ecmcTrajectoryTrapetz::updateSetpoint(double nextSetpoint,
                                             double nextVelocity) {
  posSetMinus1_            = currentPositionSetpoint_;
  currentPositionSetpoint_ = checkModuloPos(nextSetpoint,setDirection_); //=nextSetpoint;
  prevStepSize_            = dist(posSetMinus1_,currentPositionSetpoint_,setDirection_);
  velocity_                = nextVelocity;
  distToStop_              = distToStop(velocity_);
  return currentPositionSetpoint_;
}

double ecmcTrajectoryTrapetz::internalTraj(double *actVelocity) {
  double posSetTemp = currentPositionSetpoint_;

  switch (motionMode_) {
  case ECMC_MOVE_MODE_POS:
    posSetTemp = movePos(currentPositionSetpoint_,
                         targetPosition_,
                         distToStop_,
                         velocity_,
                         velocityTarget_,
                         &busy_);
    break;

  case ECMC_MOVE_MODE_VEL:
    posSetTemp = moveVel(currentPositionSetpoint_,
                         velocity_,
                         velocityTarget_,
                         &busy_);
    break;
  }
  *actVelocity = 0;

  if (busy_) { 

    *actVelocity = dist(currentPositionSetpoint_,posSetTemp,setDirection_) / sampleTime_;
  }
  return posSetTemp;
}

double ecmcTrajectoryTrapetz::moveVel(double currSetpoint,
                                      double currVelo,
                                      double targetVelo,
                                      bool  *trajBusy) {
  double positionStep = 0;
  double posSetTemp   = 0;

  *trajBusy = true;

  if(std::abs(prevStepSize_) > stepNOM_) {
    positionStep = std::abs(prevStepSize_) - stepDEC_;
    if(positionStep < stepNOM_) {      
      positionStep = stepNOM_;
    }
  } else {
    if(std::abs(prevStepSize_) < stepNOM_) {
      positionStep = std::abs(prevStepSize_) + stepACC_;
      if(positionStep > stepNOM_) {        
        positionStep = stepNOM_;
      }
    }
    else {
      positionStep = stepNOM_;
    }
  }

  if (setDirection_ == ECMC_DIR_FORWARD) {
    posSetTemp = currSetpoint + positionStep;
  } else {
    posSetTemp = currSetpoint - positionStep;
  }

  return posSetTemp;
}

double ecmcTrajectoryTrapetz::movePos(double currSetpoint,
                                      double targetSetpoint,
                                      double stopDistance,
                                      double currVelo,
                                      double targetVelo,
                                      bool  *trajBusy) {
  double positionStep = 0;
  double posSetTemp   = 0;
  bool   stopping   = false;
  //bool   changeDir    = false;

  *trajBusy = true;
  /*changeDir =
    ((targetSetpoint - currSetpoint) * currVelo < 0 && std::abs(currVelo)) > 0;*/
  
  double distToTargetOld = dist(currSetpoint,targetSetpoint,setDirection_);
  
  stopping = stopDistance > std::abs(distToTargetOld); /*||
               changeDir;*/

  if (!stopping) {
    if (std::abs(currVelo) < std::abs(targetVelo)) {
      positionStep = std::abs(prevStepSize_) + stepACC_;
    } else {
      positionStep = stepNOM_;
    }
  } else {  //Stopping
    positionStep = std::abs(prevStepSize_) - stepDEC_;
  }
  
  if (setDirection_ == ECMC_DIR_FORWARD) {
    if (currVelo >= 0) {
      posSetTemp = currSetpoint + positionStep;
    } else {
      posSetTemp = currSetpoint - positionStep;
    }

  } else {    
    // Negative Direction setpoint
    if (currVelo <= 0) {
      posSetTemp = currSetpoint - positionStep;
    } else {
      posSetTemp = currSetpoint + positionStep;
    }
  }

  posSetTemp = checkModuloPos(posSetTemp,setDirection_);
  double distToTargetnNew = dist(posSetTemp,targetSetpoint,setDirection_);
  
  // Tarjectory finished if passing target position
  if( std::abs(distToTargetOld) <= std::abs(distToTargetnNew) || 
      distToTargetnNew == 0 || distToTargetOld == 0) {
    posSetTemp      = targetSetpoint;
    // To allow for at target monitoring to go high (and then also bBusy)
    targetPosition_ = posSetTemp;
    *trajBusy       = false;
  }

  return posSetTemp; //checkModuloPos(posSetTemp);
}

double ecmcTrajectoryTrapetz::moveStop(stopMode stopMode,
                                       double   currSetpoint,
                                       double   currVelo,
                                       double   targetVelo,
                                       bool    *stopped,
                                       double  *velocity) {
  double positionStep;
  double posSetTemp = 0;

  *stopped = false;
  motionDirection nDir = ECMC_DIR_STANDSTILL;

  if (currVelo > 0) {
    nDir = ECMC_DIR_FORWARD;
  } else if (currVelo < 0) {
    nDir = ECMC_DIR_BACKWARD;
  }

  if (stopMode == ECMC_STOP_MODE_EMERGENCY) {
    positionStep = std::abs(prevStepSize_) - stepDECEmerg_;
  } else {
    positionStep = std::abs(prevStepSize_) - stepDEC_;
  }

  if (nDir == ECMC_DIR_FORWARD) {
    posSetTemp = currSetpoint + positionStep;
  } else if (nDir == ECMC_DIR_BACKWARD) {
    posSetTemp = currSetpoint - positionStep;
  }

  *velocity = (posSetTemp - currSetpoint) / sampleTime_;

  if ((nDir == ECMC_DIR_STANDSTILL) || (positionStep <= 0)) {
    *stopped        = true;
    *velocity       = 0;
    // To allow for at target monitoring to go high (and then also bBusy)
    targetPosition_ = currSetpoint;
    return currSetpoint;
  }

  return posSetTemp; //checkModuloPos(posSetTemp);
}

double ecmcTrajectoryTrapetz::distToStop(double vel) {
  return std::abs(0.5 * vel * vel / deceleration_)  /*+*/ - std::abs(
    vel * sampleTime_)  /*-4*stepDEC_*/;  // TODO check this equation
}

void ecmcTrajectoryTrapetz::setTargetPos(double pos) {
  targetPosition_ = pos;
  index_          = 0;
}

bool ecmcTrajectoryTrapetz::getBusy() {
  return busy_;
}

int ecmcTrajectoryTrapetz::getIndex() {
  return index_;
}

double ecmcTrajectoryTrapetz::getVel() {
  return velocity_;
}

void ecmcTrajectoryTrapetz::setTargetVel(double velTarget) {
  velocityTarget_ = velTarget;
  initTraj();
}

double ecmcTrajectoryTrapetz::getTargetVel() {
  return velocityTarget_;
}

void ecmcTrajectoryTrapetz::setAcc(double acc) {
  acceleration_ = acc;
  initTraj();
}

void ecmcTrajectoryTrapetz::setEmergDec(double dec) {
  decelerationEmergency_ = dec;
  initTraj();
}

void ecmcTrajectoryTrapetz::setDec(double dec) {
  deceleration_ = dec;

  if (decelerationEmergency_ == 0) {
    decelerationEmergency_ = deceleration_ * 3;
  }
  initTraj();
}

// Not used
void ecmcTrajectoryTrapetz::setJerk(double jerk) {
  jerk_ = jerk;
  initTraj();
}

double ecmcTrajectoryTrapetz::getAcc() {
  return acceleration_;
}

double ecmcTrajectoryTrapetz::getDec() {
  return deceleration_;
}

double ecmcTrajectoryTrapetz::getJerk() {
  return jerk_;  // Not used
}

double ecmcTrajectoryTrapetz::getTargetPos() {
  return targetPosition_;
}

bool ecmcTrajectoryTrapetz::getExecute() {
  return execute_;
}

void ecmcTrajectoryTrapetz::setEnable(bool enable) {
  enableOld_ = enable_;
  enable_    = enable;
  velocity_  = 0;

  if (!enableOld_ && enable_) {
    posSetMinus1_            = startPosition_;
    currentPositionSetpoint_ = startPosition_;
    prevStepSize_            = 0;
    velocity_                = 0;
    distToStop_              = 0;
  }
}

bool ecmcTrajectoryTrapetz::getEnable() {
  return enable_;
}

int ecmcTrajectoryTrapetz::setExecute(bool execute) {
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

      if (velocityTarget_ < 0) {
        setDirection_ = ECMC_DIR_BACKWARD;
      } else {
        setDirection_ = ECMC_DIR_FORWARD;
      }
      break;

    case ECMC_MOVE_MODE_POS:

      // Normal motion
      if(data_->command_.moduloRange==0) {
        if (targetPosition_ < currentPositionSetpoint_) {
          setDirection_ = ECMC_DIR_BACKWARD;
        } else {
          setDirection_ = ECMC_DIR_FORWARD;
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

          setDirection_ = ECMC_DIR_BACKWARD;
          break;

        case ECMC_MOD_MOTION_FWD:

          setDirection_ = ECMC_DIR_FORWARD;
          break;

        case ECMC_MOD_MOTION_NORMAL:

          if (targetPosition_ < currentPositionSetpoint_) {
            setDirection_ = ECMC_DIR_BACKWARD;
          } else {
            setDirection_ = ECMC_DIR_FORWARD;
          }
          break;
        case ECMC_MOD_MOTION_CLOSEST:        

          distFWD = std::abs(dist(currentPositionSetpoint_,targetPosition_,ECMC_DIR_FORWARD));
          distBWD = std::abs(dist(currentPositionSetpoint_,targetPosition_,ECMC_DIR_BACKWARD));
          if(distBWD < distFWD) {
            setDirection_ = ECMC_DIR_BACKWARD;
          } else {
            setDirection_ = ECMC_DIR_FORWARD;
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

void ecmcTrajectoryTrapetz::setStartPos(double pos) {
  startPosition_ = pos;
}

void ecmcTrajectoryTrapetz::setMotionMode(motionMode mode) {
  motionMode_ = mode;
}

interlockTypes ecmcTrajectoryTrapetz::getInterlockStatus() {
  return interlockStatus_;
}

double ecmcTrajectoryTrapetz::getSampleTime() {
  return sampleTime_;
}

int ecmcTrajectoryTrapetz::validate() {
  if (sampleTime_ <= 0) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_TRAJ_INVALID_SAMPLE_TIME);
  }
  return 0;
}

motionDirection ecmcTrajectoryTrapetz::checkDirection(double oldPos,
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

int ecmcTrajectoryTrapetz::initStopRamp(double currentPos,
                                        double currentVel,
                                        double currentAcc) {
  enable_ = 1;
  busy_                    = true;
  currentPositionSetpoint_ = currentPos;
  velocity_                = currentVel;
  prevStepSize_            = velocity_ * sampleTime_;
  return 0;
}

motionDirection ecmcTrajectoryTrapetz::getCurrSetDir() {
  return setDirection_;
}

/* Calculates distance between two points*/
double ecmcTrajectoryTrapetz::dist(double from, double to, motionDirection direction) {
  //check if modulo enabled
  if(data_->command_.moduloRange==0){
    return to-from;
  }
  
  //modulo
  switch(direction){
    case ECMC_DIR_BACKWARD:
      if(from > to){
        return to-from;
      }
      return -from-(data_->command_.moduloRange-to);
    break;
    case ECMC_DIR_FORWARD:
      if(from < to){
        return to-from;
      }
      return to + (data_->command_.moduloRange-from);
    break;
    
    case ECMC_DIR_STANDSTILL:
      return 0;
  }
  return 0;
}

double ecmcTrajectoryTrapetz::checkModuloPos(double pos,
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
