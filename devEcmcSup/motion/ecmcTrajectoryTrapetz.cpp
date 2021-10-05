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
  latchedStopMode_         = ECMC_STOP_MODE_RUN;
  switchTargetOnTheFly_    = false;
  stepStableTol_           = 0;
  thisStepSize_            = 0;
}

void ecmcTrajectoryTrapetz::initTraj() {
  stepNOM_      = velocityTarget_ * sampleTime_;
  
  // TODO check eq.
  stepACC_      =  acceleration_ * sampleTime_ * sampleTime_  /**2*/;
  stepDEC_      =  deceleration_ * sampleTime_ * sampleTime_  /**2*/;
  stepDECEmerg_ =  decelerationEmergency_ * sampleTime_ *
                  sampleTime_  /**2*/;

  // Avoid rounding errors when comparing doubles
  if(std::abs(stepACC_) > std::abs(stepDEC_)) {
    stepStableTol_ = std::abs(stepDEC_/10);
  } else {
    stepStableTol_ = std::abs(stepACC_/10);
  }
}

double ecmcTrajectoryTrapetz::getCurrentPosSet() {
  return currentPositionSetpoint_;
}

void ecmcTrajectoryTrapetz::setCurrentPosSet(double posSet) {
  currentPositionSetpoint_ = posSet;
  posSetMinus1_            = currentPositionSetpoint_;
  prevStepSize_            = 0;
  velocity_                = 0;
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
                            prevStepSize_,
                            &stopped,
                            &nextVelocity);

    if (stopped) {
      velocity_ = 0;
      busy_         = false;
      nextVelocity  = 0;
    }
  }

  currentPositionSetpoint_ = updateSetpoint(nextSetpoint, nextVelocity);

  return currentPositionSetpoint_;
}

double ecmcTrajectoryTrapetz::updateSetpoint(double nextSetpoint,
                                             double nextVelocity) {
  posSetMinus1_            = currentPositionSetpoint_;
  currentPositionSetpoint_ = checkModuloPos(nextSetpoint,velocityTarget_>0 ? ECMC_DIR_FORWARD:ECMC_DIR_BACKWARD); //=nextSetpoint;
  prevStepSize_            = thisStepSize_;
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
                          prevStepSize_,
                          stepNOM_,
                          &busy_);
    break;
  
  case ECMC_MOVE_MODE_VEL:
    posSetTemp = moveVel(currentPositionSetpoint_,
                         prevStepSize_,
                         stepNOM_,
                         &busy_);
    break;
  }
  *actVelocity = 0;

  if (busy_) { 

    *actVelocity = thisStepSize_ / sampleTime_; //dist(currentPositionSetpoint_,posSetTemp,velocityTarget_>0 ? ECMC_DIR_FORWARD:ECMC_DIR_BACKWARD) / sampleTime_;
  }
  return posSetTemp;
}

/*
  *  Method for moving at constant velocity (including ramps)
  * 
  * Supports on the fly changes of velocity target setpoint by update of stepNOM.
  * targetVelo and currVelo can be negative. 
  * - If decreasing abs(velo) then use decelerartion  (approaching zero velocity)
  * - If increasing abs(velo) velo then use acceleration
  * 
  * Note: 
  *   prevStepSize: correspons to current velocity
  *   stepNOM:      position step at velocityTarget 
  */
double ecmcTrajectoryTrapetz::moveVel(double currSetpoint,
                                      double prevStepSize,
                                      double stepNOM,
                                      bool  *trajBusy) {
  double positionStep = 0;

  *trajBusy = true;
  
   if( prevStepSize > 0) {  // currvelo
    if(stepNOM > prevStepSize) {
      // Acc
      positionStep = prevStepSize + stepACC_;
    } else if (stepNOM < prevStepSize) {
      // Dec      
      positionStep = prevStepSize - stepDEC_;
    }  else {
      positionStep = prevStepSize;
    }
  } else if ( prevStepSize < 0 ) {
    if(stepNOM > prevStepSize) {
      // Dec
      positionStep = prevStepSize + stepDEC_;
    } else if(stepNOM < prevStepSize) {
      // Acc
      positionStep = prevStepSize - stepACC_;
    } else {
      positionStep = prevStepSize;
    }
  } else {  // currvelo == 0
    if(stepNOM > prevStepSize) {
      // Acc
      positionStep = prevStepSize + stepACC_;
    } else if (stepNOM < prevStepSize) {
      // Acc
      positionStep = prevStepSize - stepACC_;
    }
  }

  // Avoid ripple due to rounding errors
  double diff = std::abs(stepNOM - positionStep);
  if( diff < stepStableTol_ ) {
    positionStep = stepNOM;
  }
  
  thisStepSize_ = positionStep;

  return currSetpoint + thisStepSize_;
}

/*
  *  Method for moving to a abs position (including ramps)
  * 
  * Supports on the fly changes of velocity target setpoint by update of stepNOM.
  * Supports on the fly changes of position target setpoint by update of targetSetpoint.

  * targetVelo and currVelo can be negative. 
  * - If decreasing abs(velo) then use decelerartion  (approaching zero velocity)
  * - If increasing abs(velo) velo then use acceleration
  * 
  * Note: 
  *   prevStepSize: correspons to current velocity
  *   stepNOM:      position step at velocityTarget 
  */ 
double ecmcTrajectoryTrapetz::movePos(double currSetpoint,
                                      double targetSetpoint,
                                      double stopDistance,
                                      double prevStepSize,
                                      double stepNom,
                                      bool  *trajBusy) {
  double posSetTemp   = 0;

  *trajBusy = true;

  // How long to target
  double distToTargetOld = 0;
  double distToTargetOldComp = 0;
  if (prevStepSize > 0) {
    distToTargetOld = dist(currSetpoint,targetSetpoint, ECMC_DIR_FORWARD);
    distToTargetOldComp = distToTargetOld - prevStepSize - stepDEC_;
  } else if (prevStepSize < 0) {
    distToTargetOld = dist(currSetpoint,targetSetpoint, ECMC_DIR_BACKWARD);
    distToTargetOldComp = distToTargetOld - prevStepSize + stepDEC_;
  } else {  // 0 velo, use target
    distToTargetOld = dist(currSetpoint,targetSetpoint, stepNom > 0 ? ECMC_DIR_FORWARD : ECMC_DIR_BACKWARD);  
    distToTargetOldComp = distToTargetOld;
  }

  
  double distToInitStop = distToTargetOldComp - stopDistance;
  double stepNomTemp = stepNom;

  if (std::abs(distToInitStop) <= std::abs(prevStepSize_)) {
    stepNomTemp = 0;  // targetVelo = 0
  } else if(distToInitStop > 0) {
    stepNomTemp = std::abs(stepNom);
  } else {
    stepNomTemp = -std::abs(stepNom);
  }

  posSetTemp = moveVel(currSetpoint,
                       prevStepSize,
                       stepNomTemp,
                       trajBusy);

  double distToTargetNew = dist(posSetTemp,targetSetpoint,prevStepSize >= 0 ? ECMC_DIR_FORWARD : ECMC_DIR_BACKWARD);
  printf("1 : distToTargetOldComp=%lf,distToTargetOld=%lf,stopDistance=%lf,distToTargetNew=%lf,distToInitStop=%lf,NOM=%lf,DEC=%lf,PREV=%lf\n",distToTargetOldComp,distToTargetOld,stopDistance,distToTargetNew,distToInitStop, stepNOM_,stepDEC_,prevStepSize);

  if( (std::abs(prevStepSize) <= std::abs(stepDEC_)) && 
      (std::abs(stopDistance) <= std::abs(stepDEC_)) && 
      (std::abs(distToTargetNew) <= std::abs(stepDEC_))) {
    posSetTemp      = targetSetpoint;
    targetPosition_ = posSetTemp;
    *trajBusy       = false;
  }

  return posSetTemp;
}

double ecmcTrajectoryTrapetz::moveStop(stopMode stopMode,
                                       double   currSetpoint,
                                       double   prevStepSize,                                       
                                       bool    *stopped,
                                       double  *velocity) {
  double positionStep;
  double posSetTemp = 0;
  
  *stopped = false;
  motionDirection nDir = ECMC_DIR_STANDSTILL;

  if (prevStepSize > 0) {
    nDir = ECMC_DIR_FORWARD;
  } else if (prevStepSize < 0) {
    nDir = ECMC_DIR_BACKWARD;
  }

  if (stopMode == ECMC_STOP_MODE_EMERGENCY) {
    positionStep = std::abs(prevStepSize) - stepDECEmerg_;
  } else {
    positionStep = std::abs(prevStepSize) - stepDEC_;
  }

  if (nDir == ECMC_DIR_FORWARD) {
    posSetTemp = currSetpoint + positionStep;
    thisStepSize_ = positionStep;
  } else if (nDir == ECMC_DIR_BACKWARD) {
    posSetTemp = currSetpoint - positionStep;
    thisStepSize_ = -positionStep;
  }

  *velocity = (posSetTemp - currSetpoint) / sampleTime_;

  if ((nDir == ECMC_DIR_STANDSTILL) || (positionStep <= 0)) {
    *stopped        = true;
    *velocity       = 0;
    // To allow for at target monitoring to go high (and then also bBusy)
    targetPosition_ = currSetpoint;
    return currSetpoint;
  }

  return posSetTemp;
}

double ecmcTrajectoryTrapetz::distToStop(double vel) {
  double d = 0.5 * vel * vel / deceleration_ + std::abs(vel * sampleTime_);
  return vel > 0 ? d : -d ;
}

void ecmcTrajectoryTrapetz::setTargetPos(double pos) {
  targetPosition_ = pos;
  index_          = 0;

  // Check if updated on the fly (copied from setExecute().. not so nice)
  if(busy_ && motionMode_ == ECMC_MOVE_MODE_POS){
    double dist2Stop=distToStop(velocity_);
    if(currentPositionSetpoint_ + dist2Stop > targetPosition_) {
      velocityTarget_= -std::abs(velocityTarget_);
    }
    else {
      velocityTarget_= std::abs(velocityTarget_);
    }
  }
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
  return velocityTarget_>0 ? ECMC_DIR_FORWARD:ECMC_DIR_BACKWARD;
}

/* Calculates distance between two points*/
double ecmcTrajectoryTrapetz::dist(double from, double to, motionDirection direction) {
  //check if modulo enabled
  if(data_->command_.moduloRange==0){
    return to-from;
  }
  printf("from %lf, to %lf, dir =%d\n",from,to,direction);
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
