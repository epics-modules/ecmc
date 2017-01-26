/*
 * ecmcAxisData.cpp
 *
 *  Created on: Jan 24, 2017
 *      Author: anderssandstrom
 */

#include "ecmcAxisData.h"

ecmcAxisData::ecmcAxisData()
{
  axisId_=0;
  sampleTime_=0;
  axisType_=ECMC_AXIS_TYPE_BASE;
  memset(&command_,0,sizeof(command_));
  memset(&status_,0,sizeof(status_));
  memset(&interlocks_,0,sizeof(interlocks_));
}

ecmcAxisData::~ecmcAxisData()
{

}

stopMode ecmcAxisData::refreshInterlocks()
{
  setSummaryInterlocks();

  //Emergency interlocks first
  if(interlocks_.axisErrorStateInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_AXIS_ERROR_STATE;
    interlocks_.currStopMode=ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if(interlocks_.bothLimitsLowInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_BOTH_LIMITS;
    interlocks_.currStopMode=ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if(interlocks_.unexpectedLimitSwitchBehaviourInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_UNEXPECTED_LIMIT_SWITCH_BEHAVIOUR;
    interlocks_.currStopMode=ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if(interlocks_.velocityDiffDriveInterlock || interlocks_.velocityDiffTrajInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_VELOCITY_DIFF;
    interlocks_.currStopMode=ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if(interlocks_.cntrlOutputHLDriveInterlock || interlocks_.cntrlOutputHLTrajInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_CONT_HIGH_LIMIT;
    interlocks_.currStopMode=ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if(interlocks_.etherCatMasterInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_ETHERCAT_MASTER_NOT_OK;
    interlocks_.currStopMode=ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if(interlocks_.hardwareInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_EXTERNAL;
    interlocks_.currStopMode=ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if(interlocks_.lagDriveInterlock || interlocks_.lagTrajInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_POSITION_LAG;
    interlocks_.currStopMode=ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if(interlocks_.bwdLimitInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_HARD_BWD;
    interlocks_.currStopMode=ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if(interlocks_.encTransformInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_TRANSFORM;
    interlocks_.currStopMode=ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if(interlocks_.fwdLimitInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_HARD_FWD;
    interlocks_.currStopMode=ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if(interlocks_.bwdSoftLimitInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_SOFT_BWD;
    interlocks_.currStopMode=ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if(interlocks_.fwdSoftLimitInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_SOFT_FWD;
    interlocks_.currStopMode=ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if(interlocks_.maxVelocityDriveInterlock || interlocks_.maxVelocityTrajInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_MAX_SPEED;
    interlocks_.currStopMode=ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if(interlocks_.noExecuteInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_NO_EXECUTE;
    interlocks_.currStopMode=ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  if(interlocks_.trajTransformInterlock){
    interlocks_.interlockStatus=ECMC_INTERLOCK_TRANSFORM;
    interlocks_.currStopMode=ECMC_STOP_MODE_NORMAL;
    return interlocks_.currStopMode;
  }

  interlocks_.interlockStatus=ECMC_INTERLOCK_NONE;

  interlocks_.currStopMode=ECMC_STOP_MODE_RUN;

  return interlocks_.currStopMode;
}

int ecmcAxisData::setSummaryInterlocks()
{
  interlocks_.driveSummaryInterlock=interlocks_.bothLimitsLowInterlock
      || interlocks_.bothLimitsLowInterlock
      || interlocks_.cntrlOutputHLDriveInterlock
      || interlocks_.lagDriveInterlock
      || interlocks_.maxVelocityDriveInterlock
      || interlocks_.velocityDiffDriveInterlock
      || interlocks_.hardwareInterlock
      || interlocks_.etherCatMasterInterlock;

  interlocks_.trajSummaryInterlockOld=interlocks_.trajSummaryInterlock;

  interlocks_.trajSummaryInterlock=interlocks_.driveSummaryInterlock
      || interlocks_.axisErrorStateInterlock
      || interlocks_.bwdLimitInterlock
      || interlocks_.bwdSoftLimitInterlock
      || interlocks_.cntrlOutputHLTrajInterlock
      || interlocks_.encTransformInterlock
      || interlocks_.fwdLimitInterlock
      || interlocks_.fwdSoftLimitInterlock
      || interlocks_.lagTrajInterlock
      || interlocks_.maxVelocityTrajInterlock
      || interlocks_.noExecuteInterlock
      || interlocks_.trajTransformInterlock
      || interlocks_.unexpectedLimitSwitchBehaviourInterlock
      || interlocks_.velocityDiffTrajInterlock;


  return 0;
}
