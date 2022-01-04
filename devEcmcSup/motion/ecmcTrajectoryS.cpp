/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcTrajectoryS.cpp
*
*  Created on: Nov 26, 2021
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcTrajectoryS.h"
#include <stdio.h>

ecmcTrajectoryS::ecmcTrajectoryS(ecmcAxisData *axisData,
                                 double        sampleTime) : 
                 ecmcTrajectoryBase(axisData, sampleTime) {  
  initVars();
  initTraj();
}

ecmcTrajectoryS::~ecmcTrajectoryS()
{}

void ecmcTrajectoryS::initVars() {
  // Create ruckig params
  otg_                          = new Ruckig<DynamicDOFs>(1,sampleTime_);
  input_                        = new InputParameter<DynamicDOFs>(1);
  output_                       = new OutputParameter<DynamicDOFs>(1);
  stepNOM_                      = 0;
  localCurrentPositionSetpoint_ = 0;
  localBusy_                    = false;
}

void ecmcTrajectoryS::setCurrentPosSet(double posSet) {
  localCurrentPositionSetpoint_ = posSet;  
  ecmcTrajectoryBase::setCurrentPosSet(posSet);
}

// "Main" of trajectory generator. Needs to be called exactly once per cycle.
// Updates trajectory setpoint
double ecmcTrajectoryS::getNextPosSet() {

  if (!busy_ || !enable_) {
    if (execute_ && !enable_) {
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_TRAJ_EXECUTE_BUT_NO_ENABLE);
    }
    double tempPos = getCurrentPosSet();
    updateSetpoint(tempPos , 0, 0, localBusy_);
    setCurrentPosSet(tempPos);
    return tempPos;
  }

  index_++;

  if (!execute_ && (data_->command_.trajSource == ECMC_DATA_SOURCE_INTERNAL)) {
    data_->interlocks_.noExecuteInterlock = true;
    data_->refreshInterlocks();
  }

  double nextSetpoint     = localCurrentPositionSetpoint_;
  double nextVelocity     = currentVelocitySetpoint_;
  double nextAcceleration = currentAccelerationSetpoint_;  
  bool   stopped          = false;
  
  nextSetpoint = internalTraj(&nextVelocity,&nextAcceleration,&localBusy_);

  motionDirection nextDir = checkDirection(localCurrentPositionSetpoint_,
                                           nextSetpoint);
  // Stop ramp when running external
  bool externalSourceStopTraj = data_->command_.trajSource !=
                                ECMC_DATA_SOURCE_INTERNAL;
  // check interlocks if stop is needed
  if (externalSourceStopTraj ||
      ((nextDir == ECMC_DIR_BACKWARD) &&
       data_->interlocks_.trajSummaryInterlockBWD) ||
      ((nextDir == ECMC_DIR_FORWARD) &&
       data_->interlocks_.trajSummaryInterlockFWD)) {

    nextSetpoint = moveStop(data_->interlocks_.currStopMode,
                            &nextVelocity,
                            &nextAcceleration,
                            &stopped);
    localBusy_ = !stopped;
    if (stopped) {
      currentVelocitySetpoint_     = 0;
      currentAccelerationSetpoint_ = 0;
      localBusy_                   = false;
      nextVelocity                 = 0;
    }
  }
  
  localCurrentPositionSetpoint_ = nextSetpoint;
  output_->pass_to_input(*input_);
  return ecmcTrajectoryBase::updateSetpoint(nextSetpoint, nextVelocity, nextAcceleration, localBusy_);
}

double ecmcTrajectoryS::internalTraj(double *actVelocity, 
                                     double *actAcceleration, 
                                     bool   *trajBusy) {
  double posSetTemp = localCurrentPositionSetpoint_;
  
  switch (motionMode_) {
  case ECMC_MOVE_MODE_POS:
    posSetTemp = movePos(actVelocity, actAcceleration, trajBusy);
    break;
  
  case ECMC_MOVE_MODE_VEL:
    posSetTemp = moveVel(actVelocity, actAcceleration, trajBusy);
    break;

  default: 
    *actVelocity     = 0;
    *actAcceleration = 0;
    *trajBusy        = false;
    break;
  }

  return posSetTemp;
}

void ecmcTrajectoryS::initRuckig() {
  input_->current_position[0]     = currentPositionSetpoint_;
  input_->current_velocity[0]     = currentVelocitySetpoint_;
  input_->current_acceleration[0] = currentAccelerationSetpoint_;
  stepNOM_                        = std::abs(targetVelocity_ * sampleTime_);
}

bool ecmcTrajectoryS::updateRuckig() {

  Result res = otg_->update(*input_, *output_);
  
  if(res<0) {
    switch(res) {
      case Result::Error:
        setErrorID(__FILE__, __FUNCTION__, __LINE__,ERROR_TRAJ_RUCKIG_ERROR);
        break;
      case Result::ErrorInvalidInput:
        setErrorID(__FILE__, __FUNCTION__, __LINE__,ERROR_TRAJ_RUCKIG_INVALID_INPUT);
        break;
      case Result::ErrorTrajectoryDuration:
        setErrorID(__FILE__, __FUNCTION__, __LINE__,ERROR_TRAJ_RUCKIG_TRAJ_DURATION);
        break;
      case Result::ErrorPositionalLimits:
        setErrorID(__FILE__, __FUNCTION__, __LINE__,ERROR_TRAJ_RUCKIG_POS_LIMITS);
        break;
//      case Result::ErrorNoPhaseSynchronization:
//        setErrorID(__FILE__, __FUNCTION__, __LINE__,ERROR_TRAJ_RUCKIG_NO_PHASE_SYNC);
//        break;
      case Result::ErrorExecutionTimeCalculation:
        setErrorID(__FILE__, __FUNCTION__, __LINE__,ERROR_TRAJ_RUCKIG_EXE_TIME_CALC);
        break;
      case Result::ErrorSynchronizationCalculation:
        setErrorID(__FILE__, __FUNCTION__, __LINE__,ERROR_TRAJ_RUCKIG_SYNC_CALC);
        break;
      default:
        setErrorID(__FILE__, __FUNCTION__, __LINE__,ERROR_TRAJ_RUCKIG_ERROR);
        break;        
    }
    LOGERR("%s/%s:%d: ERROR: Ruckig error  %d (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        res,
        getErrorID());
  }
  
  return res == Result::Working;
}

double ecmcTrajectoryS::moveVel(double *actVelocity,
                                double *actAcceleration,
                                bool   *trajBusy){
  double positionSetpoint        = localCurrentPositionSetpoint_;
  bool ruckigBusy                = false;
  input_->control_interface      = ControlInterface::Velocity;

  if(execute_) {
    input_->target_velocity[0]   = targetVelocity_;
  } else {    
    input_->target_velocity[0]   = 0;    
  }

  input_->target_acceleration[0] = 0;
  input_->max_velocity[0]        = std::abs(targetVelocity_);
  input_->max_acceleration[0]    = std::abs(targetAcceleration_);
  input_->max_jerk[0]            = std::abs(targetJerk_);
  ruckigBusy                     = updateRuckig();
  *actVelocity                   = output_->new_velocity[0];
  *actAcceleration               = output_->new_acceleration[0];
  positionSetpoint               = output_->new_position[0];
  
  if(!ruckigBusy && *actVelocity != 0) {    
    // ramp up by ruckig is complete. Just continue in that velo
    *actVelocity                 = targetVelocity_;
    *actAcceleration             = 0;
    if(targetVelocity_ >= 0) {
      positionSetpoint = localCurrentPositionSetpoint_ + stepNOM_;
    } else {
      positionSetpoint = localCurrentPositionSetpoint_ - stepNOM_;
    }
  } 

  targetPosition_ = positionSetpoint;
  *trajBusy = *actVelocity !=0; 
  return positionSetpoint;
}

double ecmcTrajectoryS::movePos(double *actVelocity,
                                double *actAcceleration,
                                bool   *trajBusy){
  input_->control_interface      = ControlInterface::Position;
  input_->target_position[0]     = targetPosition_;
  input_->target_velocity[0]     = 0;
  input_->target_acceleration[0] = 0;
  input_->max_velocity[0]        = std::abs(targetVelocity_);
  input_->max_acceleration[0]    = std::abs(targetAcceleration_);
  input_->max_jerk[0]            = std::abs(targetJerk_);
  *trajBusy                      = updateRuckig();
  *actVelocity                   = output_->new_velocity[0];
  *actAcceleration               = output_->new_acceleration[0];  
  return output_->new_position[0];
}

double ecmcTrajectoryS::moveStop(stopMode stopMode,
                                 double *actVelocity, 
                                 double *actAcceleration,                                 
                                 bool   *stopped){
  input_->control_interface       = ControlInterface::Velocity;
  input_->target_velocity[0]      = 0;  // stop
  input_->target_acceleration[0]  = 0;
  input_->max_velocity[0]         = targetVelocity_;  
  if (stopMode == ECMC_STOP_MODE_EMERGENCY) {    
    input_->max_acceleration[0]  = targetDecelerationEmerg_;
  } else {
    input_->max_acceleration[0]  = targetDeceleration_;
  }

  input_->max_jerk[0]             = targetJerk_;
  *stopped                        = !updateRuckig();
  *actVelocity                    = output_->new_velocity[0];
  *actAcceleration                = output_->new_acceleration[0];
  targetPosition_ = output_->new_position[0];
  return output_->new_position[0];
}

double ecmcTrajectoryS::distToStop(double vel) {
  return 0;  // No nice way to calculate with ruckig
}

void ecmcTrajectoryS::setTargetPos(double pos) {
  ecmcTrajectoryBase::setTargetPos(pos);  
  input_->target_position[0] = pos;
}

void ecmcTrajectoryS::setTargetVel(double velTarget) {
  ecmcTrajectoryBase::setTargetVel(velTarget);
  stepNOM_ = std::abs(velTarget * sampleTime_);
}

void ecmcTrajectoryS::setAcc(double acc) {
  ecmcTrajectoryBase::setAcc(acc);
}

void ecmcTrajectoryS::setDec(double dec) {
  ecmcTrajectoryBase::setDec(dec);
}

void ecmcTrajectoryS::setEmergDec(double dec) {
  ecmcTrajectoryBase::setEmergDec(dec);       
}


void ecmcTrajectoryS::setJerk(double jerk) {
  ecmcTrajectoryBase::setJerk(jerk);
}

void ecmcTrajectoryS::setEnable(bool enable) {
  ecmcTrajectoryBase::setEnable(enable);
}

int ecmcTrajectoryS::initStopRamp(double currentPos,
                                  double currentVel,
                                  double currentAcc) {
  ecmcTrajectoryBase::initStopRamp(currentPos,currentVel,currentAcc);
  initRuckig();
  return 0;
}

int ecmcTrajectoryS::setExecute(bool execute) {
   if(execute && !executeOld_) {
     initRuckig();
   }
   if(!execute) {    
     initRuckig();
   }

   return ecmcTrajectoryBase::setExecute(execute);
}
