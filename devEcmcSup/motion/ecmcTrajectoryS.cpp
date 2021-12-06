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
  otg_    = new Ruckig<DynamicDOFs>(1,sampleTime_);
  input_  = new InputParameter<DynamicDOFs>(1);
  output_ = new OutputParameter<DynamicDOFs>(1);
  otgbusy_ = false;
}

void ecmcTrajectoryS::setCurrentPosSet(double posSet) {
  currentPositionSetpoint_     = posSet;
  posSetMinus1_                = posSet;  
  currentVelocitySetpoint_     = 0;
  currentAccelerationSetpoint_ = 0;
}

// "Main" of trajectory generator. Needs to be called exactly once per cycle.
// Updates trajectory setpoint
double ecmcTrajectoryS::getNextPosSet() {

  double nextSetpoint     = currentPositionSetpoint_;
  double nextVelocity     = currentVelocitySetpoint_;
  double nextAcceleration = currentAccelerationSetpoint_;
  bool   trajBusy         = 0;

  bool stopped = false;

  if (!busy_ || !enable_) {
    if (execute_ && !enable_) {
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_TRAJ_EXECUTE_BUT_NO_ENABLE);
    }
    posSetMinus1_ = currentPositionSetpoint_;
    currentVelocitySetpoint_     = 0;
    return currentPositionSetpoint_;
  }
  index_++;

  if (!execute_ && (data_->command_.trajSource == ECMC_DATA_SOURCE_INTERNAL)) {
    data_->interlocks_.noExecuteInterlock = true;
    data_->refreshInterlocks();
  }

  nextSetpoint = internalTraj(&nextVelocity,&nextAcceleration,&trajBusy);

  motionDirection nextDir = checkDirection(currentPositionSetpoint_,
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

    if (stopped) {
      currentVelocitySetpoint_     = 0;
      currentAccelerationSetpoint_ = 0;
      busy_                        = false;
      nextVelocity                 = 0;
    }    
  }

  // use the new setpoints!
  currentPositionSetpoint_ = updateSetpoint(nextSetpoint, nextVelocity, nextAcceleration);

  return currentPositionSetpoint_;
}

double ecmcTrajectoryS::updateSetpoint(double nextSetpoint,
                                       double nextVelocity,
                                       double nextAcceleration) {
  posSetMinus1_            = currentPositionSetpoint_;
  currentPositionSetpoint_ = checkModuloPos(nextSetpoint,targetVelocity_>0 ? ECMC_DIR_FORWARD:ECMC_DIR_BACKWARD);
  currentVelocitySetpoint_ = nextVelocity;
  currentAccelerationSetpoint_ = nextAcceleration;
  distToStop_              = distToStop(currentVelocitySetpoint_);
  return currentPositionSetpoint_;
}

double ecmcTrajectoryS::internalTraj(double *actVelocity, 
                                     double *actAcceleration, 
                                     bool   *trajBusy) {
  double posSetTemp = currentPositionSetpoint_;
  
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
  printf("ecmcTrajectoryS::initRuckig(),%lf , %lf , %lf \n", input_->current_position[0],input_->current_velocity[0],input_->current_acceleration[0]);
}

bool ecmcTrajectoryS::updateRuckig() {

  Result res = otg_->update(*input_, *output_);
  printf("Result %d\n",(int)res);
  //otgbusy_ = otg_->update(*input_, *output_) == ;
  if(res == Result::Working) {    
    output_->pass_to_input(*input_);
  }
  otgbusy_ = res == Result::Working;
  return otgbusy_;
}

double ecmcTrajectoryS::moveVel(double *actVelocity,
                                double *actAcceleration,
                                bool   *trajBusy){

  input_->control_interface      = ControlInterface::Velocity;
  input_->target_velocity[0]     = targetVelocity_;
  input_->target_acceleration[0] = 0;
  input_->max_velocity[0]        = targetVelocity_;
  input_->max_acceleration[0]    = targetAcceleration_;
  input_->max_jerk[0]            = targetJerk_;
  *trajBusy                      = updateRuckig();
  *actVelocity                   = output_->new_velocity[0];
  *actAcceleration               = output_->new_acceleration[0];
  printf("ecmcTrajectoryS::moveVel(),%lf , %lf , %lf \n", output_->new_position[0],output_->new_velocity[0],output_->new_acceleration[0]);

  return output_->new_position[0];
}

double ecmcTrajectoryS::movePos(double *actVelocity,
                                double *actAcceleration,
                                bool   *trajBusy){
  
  input_->control_interface      = ControlInterface::Position;
  input_->target_position[0]     = targetPosition_;
  input_->target_velocity[0]     = 0;
  input_->target_acceleration[0] = 0;
  input_->max_velocity[0]        = targetVelocity_;
  input_->max_acceleration[0]    = targetAcceleration_;
  input_->max_jerk[0]            = targetJerk_;
  *trajBusy                      = updateRuckig();
  *actVelocity                   = output_->new_velocity[0];
  *actAcceleration               = output_->new_acceleration[0];  
  printf("target: pos %lf, vel %lf, acc %lf, jerk %lf\n",targetPosition_,targetVelocity_,targetAcceleration_,targetJerk_);
  printf("Actual: pos %lf, vel %lf, acc %lf, busy %d\n",output_->new_position[0],actVelocity,actAcceleration, trajBusy);  
  return output_->new_position[0];
}

double ecmcTrajectoryS::moveStop(stopMode stopMode,
                                 double *actVelocity, 
                                 double *actAcceleration,                                 
                                 bool   *stopped){

  input_->control_interface       = ControlInterface::Velocity;
  input_->target_velocity[0]      = 0;  // stop
  
  if (stopMode == ECMC_STOP_MODE_EMERGENCY) {
    input_->target_acceleration[0]  = targetDecelerationEmerg_;
  } else {
    input_->target_acceleration[0]  = targetDeceleration_;
  }
  
  input_->max_velocity[0]         = targetVelocity_;
  input_->max_acceleration[0]     = targetAcceleration_;
  input_->max_jerk[0]             = targetJerk_;
  *stopped                        = updateRuckig();
  *actVelocity                    = output_->new_velocity[0];
  *actAcceleration                = output_->new_acceleration[0];
  //*stopped = output->new_velocity[0] == 0;
  return output_->new_position[0];
}

double ecmcTrajectoryS::distToStop(double vel) {
  return 0;
}

void ecmcTrajectoryS::setTargetPos(double pos) {
  targetPosition_ = pos;
  input_->target_position      = {targetPosition_};
}

void ecmcTrajectoryS::setTargetVel(double velTarget) {
  targetVelocity_ = velTarget;
  input_->target_velocity      = {targetVelocity_};
  input_->max_velocity         = {targetVelocity_};
}

void ecmcTrajectoryS::setAcc(double acc) {
  targetAcceleration_ = acc;
  input_->target_acceleration  = {targetAcceleration_};
  input_->max_acceleration     = {targetAcceleration_};
}

void ecmcTrajectoryS::setEmergDec(double dec) {
  targetDecelerationEmerg_ = dec;
}

void ecmcTrajectoryS::setDec(double dec) {
  targetDeceleration_ = dec;
  if (targetDecelerationEmerg_ == 0) {
    targetDecelerationEmerg_ = targetDeceleration_ * 3;
  }
}

void ecmcTrajectoryS::setJerk(double jerk) {
  targetJerk_ = jerk;
  input_->max_jerk             = {targetJerk_};
}

void ecmcTrajectoryS::setEnable(bool enable) {
  ecmcTrajectoryBase::setEnable(enable);
}

int ecmcTrajectoryS::initStopRamp(double currentPos,
                                  double currentVel,
                                  double currentAcc) {
  enable_                  = 1;
  busy_                    = true;
  currentPositionSetpoint_ = currentPos;
  currentVelocitySetpoint_ = currentVel;
  currentAccelerationSetpoint_ = currentAcc;
  initRuckig();
  return 0;
}

int ecmcTrajectoryS::setExecute(bool execute) {
   if(execute && !executeOld_) {
     initRuckig();
   }
   return ecmcTrajectoryBase::setExecute(execute);
}
