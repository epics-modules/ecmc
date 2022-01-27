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
{ 
  delete otg_;
  delete input_;
  delete output_;
}

void ecmcTrajectoryS::initVars() {
  // Create ruckig params
  otg_                          = new Ruckig<DynamicDOFs>(1,sampleTime_);
  input_                        = new InputParameter<DynamicDOFs>(1);
  output_                       = new OutputParameter<DynamicDOFs>(1);
  stepNOM_                      = 0;
  localCurrentPositionSetpoint_ = 0;
  targetPositionLocal_          = 0;
  localBusy_                    = false;
  targetVelocityLocal_          = 0;
}

void ecmcTrajectoryS::setCurrentPosSet(double posSet) {
  localCurrentPositionSetpoint_ = posSet;
  ecmcTrajectoryBase::setCurrentPosSet(posSet);
}

double ecmcTrajectoryS::internalTraj(double *actVelocity, 
                                     double *actAcceleration, 
                                     bool   *trajBusy) {
  double posSetTemp = localCurrentPositionSetpoint_;
  bool   stopped    = false;
  
  switch (motionMode_) {
  case ECMC_MOVE_MODE_POS:
    posSetTemp = movePos(actVelocity, actAcceleration, &localBusy_);
    break;
  
  case ECMC_MOVE_MODE_VEL:
    posSetTemp = moveVel(actVelocity, actAcceleration, &localBusy_);
    // in velo mode ensure local setpoint is not to high
    posSetTemp = checkModuloPos(posSetTemp);
    break;

  default: 
    *actVelocity     = 0;
    *actAcceleration = 0;
    localBusy_       = false;
    break;
  }
  
  motionDirection nextDir = checkDirection(localCurrentPositionSetpoint_,
                                           posSetTemp);
  // Stop ramp when running external
  bool externalSourceStopTraj = data_->command_.trajSource !=
                                ECMC_DATA_SOURCE_INTERNAL;
  // check interlocks if stop is needed
  if (externalSourceStopTraj ||
      ((nextDir == ECMC_DIR_BACKWARD) &&
       data_->interlocks_.trajSummaryInterlockBWD) ||
      ((nextDir == ECMC_DIR_FORWARD) &&
       data_->interlocks_.trajSummaryInterlockFWD)) {

    posSetTemp = moveStop(data_->interlocks_.currStopMode,
                          actVelocity,
                          actAcceleration,
                          &stopped);
    localBusy_ = !stopped;
    if (stopped) {
      *actVelocity     = 0;
      *actAcceleration = 0;
      localBusy_       = false;      
    }
  }

  //if(std::abs(output_->new_position[0]-localCurrentPositionSetpoint_) > stepNOM_*10) {
  //  printf("!!!!! SEVERE RUCKIG ERROR !!!!!\n");
  //  printf("Input pos:       %lf, vel  %lf, acc  %lf\n",input_->current_position[0],input_->current_velocity[0],input_->current_acceleration[0]);
  //  printf("Target pos:      %lf, vel  %lf, acc  %lf\n",input_->target_position[0],input_->target_velocity[0],input_->target_acceleration[0]);
  //  printf("Max vel:         %lf, acc  %lf, jerk %lf\n",input_->max_velocity[0],input_->max_acceleration[0],input_->max_jerk[0]);
  //  printf("Output pos:      %lf, vel  %lf, acc  %lf\n",output_->new_position[0],output_->new_velocity[0], output_->new_acceleration[0]);
  //  printf("Output prev pos: %lf, targ %lf\n",localCurrentPositionSetpoint_, targetPositionLocal_);
  //  printf("error:           0x%x, busy_ %d, localBusy %d\n",getErrorID(),busy_,localBusy_);
  //}

  *trajBusy                     = localBusy_;
  localCurrentPositionSetpoint_ = posSetTemp;
  output_->pass_to_input(*input_);

  return posSetTemp;
}

void ecmcTrajectoryS::initRuckig() {
  input_->current_position[0]     = localCurrentPositionSetpoint_;
  input_->current_velocity[0]     = currentVelocitySetpoint_;
  input_->current_acceleration[0] = currentAccelerationSetpoint_;
  // Position step for const velo
  stepNOM_                        = std::abs(targetVelocityLocal_ * sampleTime_);
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
        printf("Input pos %lf, vel %lf, acc %lf\n",input_->current_position[0],input_->current_velocity[0],input_->current_acceleration[0]);
        printf("Target pos %lf, vel %lf, acc %lf\n",input_->target_position[0],input_->target_velocity[0],input_->target_acceleration[0]);
        printf("Max vel %lf, acc %lf, jerk %lf\n",input_->max_velocity[0],input_->max_acceleration[0],input_->max_jerk[0]);
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
    //LOGERR("%s/%s:%d: ERROR: Ruckig error  %d (0x%x).\n",
    //    __FILE__,
    //    __FUNCTION__,
    //    __LINE__,
    //    res,
    //    getErrorID());
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
    input_->target_velocity[0]   = targetVelocityLocal_;
  } else {    
    input_->target_velocity[0]   = 0;    
  }

  input_->target_acceleration[0] = 0;
  input_->max_velocity[0]        = std::abs(targetVelocityLocal_);
  input_->max_acceleration[0]    = std::abs(targetAcceleration_);
  input_->max_jerk[0]            = std::abs(targetJerk_);
  ruckigBusy                     = updateRuckig();
  *actVelocity                   = output_->new_velocity[0];
  *actAcceleration               = output_->new_acceleration[0];
  positionSetpoint               = output_->new_position[0];
  
  if(!ruckigBusy && *actVelocity != 0) {    
    // ramp up by ruckig is complete. Just continue in that velo
    *actVelocity                 = targetVelocityLocal_;
    *actAcceleration             = 0;
    if(targetVelocityLocal_ >= 0) {
      positionSetpoint = localCurrentPositionSetpoint_ + stepNOM_;
    } else {
      positionSetpoint = localCurrentPositionSetpoint_ - stepNOM_;
    }
  } 

  targetPosition_      = positionSetpoint;
  targetPositionLocal_ = positionSetpoint;
  *trajBusy = *actVelocity !=0; 
  return positionSetpoint;
}

double ecmcTrajectoryS::movePos(double *actVelocity,
                                double *actAcceleration,
                                bool   *trajBusy){
  input_->control_interface      = ControlInterface::Position;
  input_->target_position[0]     = targetPositionLocal_;
  input_->target_velocity[0]     = 0;
  input_->target_acceleration[0] = 0;
  input_->max_velocity[0]        = std::abs(targetVelocityLocal_);
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
  input_->max_velocity[0]         = std::abs(targetVelocityLocal_);
  if (stopMode == ECMC_STOP_MODE_EMERGENCY) {    
    input_->max_acceleration[0]  = std::abs(targetDecelerationEmerg_);
  } else {
    input_->max_acceleration[0]  = std::abs(targetDeceleration_);
  }

  input_->max_jerk[0]             = targetJerk_;
  *stopped                        = !updateRuckig();
  *actVelocity                    = output_->new_velocity[0];
  *actAcceleration                = output_->new_acceleration[0];
  targetPosition_                 = output_->new_position[0];
  targetPositionLocal_            = output_->new_position[0];

  //printf("Input pos:       %lf, vel  %lf, acc  %lf\n",input_->current_position[0],input_->current_velocity[0],input_->current_acceleration[0]);
  //printf("Target pos:      %lf, vel  %lf, acc  %lf\n",input_->target_position[0],input_->target_velocity[0],input_->target_acceleration[0]);
  //printf("Max vel:         %lf, acc  %lf, jerk %lf\n",input_->max_velocity[0],input_->max_acceleration[0],input_->max_jerk[0]);
  //printf("Output pos:      %lf, vel  %lf, acc  %lf\n",output_->new_position[0],output_->new_velocity[0], output_->new_acceleration[0]);
  //printf("Output prev pos: %lf, targ %lf\n",localCurrentPositionSetpoint_, targetPositionLocal_);
  //printf("error:           0x%x, busy_ %d, stopped %d\n",getErrorID(),busy_,*stopped);
  
  return output_->new_position[0];
}

double ecmcTrajectoryS::distToStop(double vel) {
  return 0;  // No nice way to calculate with ruckig
}

void ecmcTrajectoryS::setTargetPosLocal(double pos) {
  localCurrentPositionSetpoint_ = currentPositionSetpoint_;
  targetPositionLocal_ = pos;
  input_->target_position[0] = pos;
}

void ecmcTrajectoryS::setTargetVel(double velTarget) {
  ecmcTrajectoryBase::setTargetVel(velTarget);
  stepNOM_ = std::abs(velTarget * sampleTime_);
}

int ecmcTrajectoryS::initStopRamp(double currentPos,
                                  double currentVel,
                                  double currentAcc) {
  ecmcTrajectoryBase::initStopRamp(currentPos,currentVel,currentAcc);

  localCurrentPositionSetpoint_ = currentPos;
  targetVelocityLocal_ = currentVel;
  initRuckig();
  return 0;
}

int ecmcTrajectoryS::setExecute(bool execute) {
   if(execute && !execute_) {
     initRuckig();
     targetVelocityLocal_ = targetVelocity_;
   }

   return ecmcTrajectoryBase::setExecute(execute);
}

int ecmcTrajectoryS::validate() {

  if(targetJerk_ == 0) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__,ERROR_TRAJ_RUCKIG_JERK_ZERO);
    LOGERR("%s/%s:%d: ERROR: Jerk zero (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        getErrorID());
    return ERROR_TRAJ_RUCKIG_JERK_ZERO;
  }
  return ecmcTrajectoryBase::validate();
}
