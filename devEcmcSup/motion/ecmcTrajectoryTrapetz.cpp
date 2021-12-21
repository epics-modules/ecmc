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
                       ecmcTrajectoryBase(axisData, sampleTime) {
  initVars();              
  initTraj();
}

ecmcTrajectoryTrapetz::~ecmcTrajectoryTrapetz()
{}

void ecmcTrajectoryTrapetz::initVars() {
  stepACC_                 = 0;
  stepDEC_                 = 0;
  stepNOM_                 = 0;
  stepDECEmerg_            = 0;
  prevStepSize_            = 0;
  switchTargetOnTheFly_    = false;
  stepStableTol_           = 0;
  thisStepSize_            = 0;  
}

void ecmcTrajectoryTrapetz::initTraj() {
  printf("ecmcTrajectoryTrapetz::initTraj()\n");
  stepNOM_      = targetVelocity_ * sampleTime_;
  
  // TODO check eq.
  stepACC_      =  targetAcceleration_ * sampleTime_ * sampleTime_  /**2*/;
  stepDEC_      =  targetDeceleration_ * sampleTime_ * sampleTime_  /**2*/;
  stepDECEmerg_ =  targetDecelerationEmerg_ * sampleTime_ *
                  sampleTime_  /**2*/;

  // Avoid rounding errors when comparing doubles
  if(std::abs(stepACC_) > std::abs(stepDEC_)) {
    stepStableTol_ = std::abs(stepDEC_/10);
  } else {
    stepStableTol_ = std::abs(stepACC_/10);
  }
  //ecmcTrajectoryTrapetz::initTraj();
}

void ecmcTrajectoryTrapetz::setCurrentPosSet(double posSet) {
  currentPositionSetpoint_ = posSet;
  posSetMinus1_            = currentPositionSetpoint_;
  prevStepSize_            = 0;
  currentVelocitySetpoint_                = 0;
}

double ecmcTrajectoryTrapetz::getNextPosSet() {
  // "Main" of trajectory generator. Needs to be called exactly once per cycle.
  // Updates trajectory setpoint

  double nextSetpoint = currentPositionSetpoint_;
  double nextVelocity = currentVelocitySetpoint_;

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
    currentVelocitySetpoint_     = 0;
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
      currentVelocitySetpoint_ = 0;
      busy_         = false;
      nextVelocity  = 0;
    }
  }

  currentPositionSetpoint_ = updateSetpoint(nextSetpoint, nextVelocity);

  return currentPositionSetpoint_;
}

double ecmcTrajectoryTrapetz::updateSetpoint(double nextSetpoint,
                                             double nextVelocity) {
  posSetMinus1_                = currentPositionSetpoint_;
  currentPositionSetpoint_     = checkModuloPos(nextSetpoint,nextVelocity>0 ? ECMC_DIR_FORWARD:ECMC_DIR_BACKWARD); //=nextSetpoint;
  prevStepSize_                = thisStepSize_;
  currentVelocitySetpoint_     = nextVelocity;
  currentAccelerationSetpoint_ = (currentPositionSetpoint_ - posSetMinus1_) / sampleTime_;
  distToStop_                  = distToStop(currentVelocitySetpoint_);
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

    *actVelocity = thisStepSize_ / sampleTime_; //dist(currentPositionSetpoint_,posSetTemp,targetVelocity_>0 ? ECMC_DIR_FORWARD:ECMC_DIR_BACKWARD) / sampleTime_;
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

  //double distToTargetNew = dist(posSetTemp,targetSetpoint,prevStepSize >= 0 ? ECMC_DIR_FORWARD : ECMC_DIR_BACKWARD);

  //printf("1 : distToTargetOldComp=%lf,distToTargetOld=%lf,stopDistance=%lf,distToTargetNew=%lf,distToInitStop=%lf,NOM=%lf,DEC=%lf,PREV=%lf\n",distToTargetOldComp,distToTargetOld,stopDistance,distToTargetNew,distToInitStop, stepNOM_,stepDEC_,prevStepSize);

  if( (std::abs(prevStepSize) <= std::abs(stepDEC_)) &&   // The most important!
      (std::abs(stopDistance) <= 3*std::abs(stepDEC_)) && 
      (std::abs(distToTargetOld) <= 3*std::abs(stepDEC_))) {
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

/* dist to stop is the triangular area under 
 *  y = prevStepSize_
 *  x = std::abs(prevStepSize_/stepDEC_), the number of steps until
 *      zero speed (if deceleration)
 */
double ecmcTrajectoryTrapetz::distToStop(double vel) {
  //double d = 0.5 * vel * vel / targetDeceleration_ + std::abs(vel * sampleTime_);
  //return vel > 0 ? d : -d ;
  return prevStepSize_* std::abs(prevStepSize_/stepDEC_) / 2;
}

void ecmcTrajectoryTrapetz::setTargetPos(double pos) {
  targetPosition_ = pos;
  index_          = 0;

  // Check if updated on the fly (copied from setExecute().. not so nice)
  if(busy_ && motionMode_ == ECMC_MOVE_MODE_POS){
    double dist2Stop=distToStop(currentVelocitySetpoint_);
    if(currentPositionSetpoint_ + dist2Stop > targetPosition_) {
      targetVelocity_= -std::abs(targetVelocity_);
    }
    else {
      targetVelocity_= std::abs(targetVelocity_);
    }
  }
}

void ecmcTrajectoryTrapetz::setTargetVel(double velTarget) {
  targetVelocity_ = velTarget;
  initTraj();
}

void ecmcTrajectoryTrapetz::setAcc(double acc) {
  targetAcceleration_ = acc;
  initTraj();
}

void ecmcTrajectoryTrapetz::setEmergDec(double dec) {
  targetDecelerationEmerg_ = dec;
  initTraj();
}

void ecmcTrajectoryTrapetz::setDec(double dec) {
  targetDeceleration_ = dec;

  if (targetDecelerationEmerg_ == 0) {
    targetDecelerationEmerg_ = targetDeceleration_ * 3;
  }
  initTraj();
}

// Not used
void ecmcTrajectoryTrapetz::setJerk(double jerk) {
  targetJerk_ = jerk;
  initTraj();
}

void ecmcTrajectoryTrapetz::setEnable(bool enable) {
  ecmcTrajectoryBase::setEnable(enable);
  if (!enableOld_ && enable) {
    prevStepSize_            = 0;
  }
}

int ecmcTrajectoryTrapetz::initStopRamp(double currentPos,
                                        double currentVel,
                                        double currentAcc) {
  enable_ = 1;
  busy_                    = true;
  currentPositionSetpoint_ = currentPos;
  currentVelocitySetpoint_                = currentVel;
  prevStepSize_            = currentVelocitySetpoint_ * sampleTime_;
  return 0;
}
