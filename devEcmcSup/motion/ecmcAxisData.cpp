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
  memset(&control_,    0, sizeof(control_));
  memset(&controlOld_, 0, sizeof(controlOld_));
  memset(&status_,     0, sizeof(status_));
  memset(&statusOld_,  0, sizeof(statusOld_));
  memset(&interlocks_, 0, sizeof(interlocks_));
  memset(&interlocksOld_, 0, sizeof(interlocksOld_));
  status_.axisType = ECMC_AXIS_TYPE_BASE;
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
  if (interlocks_.safetyInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_SAFETY;
    interlocks_.currStopMode    = ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if (interlocks_.bothLimitsLowInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_BOTH_LIMITS;
    interlocks_.currStopMode    = ECMC_STOP_MODE_EMERGENCY;
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

  if (interlocks_.analogInterlock ) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_ANALOG;
    interlocks_.currStopMode    = ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if (interlocks_.stallInterlock ) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_STALL;
    interlocks_.currStopMode    = ECMC_STOP_MODE_EMERGENCY;
    return interlocks_.currStopMode;
  }

  if (interlocks_.bwdLimitInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_HARD_BWD;
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
  
  // General error
  if (interlocks_.axisErrorStateInterlock) {
    interlocks_.interlockStatus = ECMC_INTERLOCK_AXIS_ERROR_STATE;
    interlocks_.currStopMode    = ECMC_STOP_MODE_EMERGENCY;
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
             status_.axisId,
             interlocks_.interlockStatus);
    } else {
      LOGERR("%s/%s:%d: INFO (axis %d): Motion interlock cleared.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             status_.axisId);
    }
  }
  return stop;
}

int ecmcAxisData::setSummaryInterlocks() {
  interlocks_.driveSummaryInterlock =    interlocks_.bothLimitsLowInterlock
                                      || interlocks_.cntrlOutputHLDriveInterlock
                                      || interlocks_.lagDriveInterlock
                                      || interlocks_.maxVelocityDriveInterlock
                                      || interlocks_.velocityDiffDriveInterlock
                                      || interlocks_.hardwareInterlock
                                      || interlocks_.etherCatMasterInterlock
                                      || interlocks_.analogInterlock
                                      || interlocks_.stallInterlock;

  interlocks_.trajSummaryInterlockBWDEpics = interlocks_.driveSummaryInterlock
                                        || interlocks_.axisErrorStateInterlock
                                        || interlocks_.bwdLimitInterlock
                                        || interlocks_.bwdSoftLimitInterlock
                                        || interlocks_.cntrlOutputHLTrajInterlock
                                        || interlocks_.lagTrajInterlock
                                        || interlocks_.maxVelocityTrajInterlock
                                        || interlocks_.velocityDiffTrajInterlock
                                        || interlocks_.plcInterlock
                                        || interlocks_.plcInterlockBWD
                                        || interlocks_.encDiffInterlock
                                        || interlocks_.safetyInterlock;
  interlocks_.trajSummaryInterlockBWD = interlocks_.trajSummaryInterlockBWDEpics 
                                        || interlocks_.noExecuteInterlock;

  interlocks_.trajSummaryInterlockFWDEpics = interlocks_.driveSummaryInterlock
                                        || interlocks_.axisErrorStateInterlock
                                        || interlocks_.cntrlOutputHLTrajInterlock
                                        || interlocks_.fwdLimitInterlock
                                        || interlocks_.fwdSoftLimitInterlock
                                        || interlocks_.lagTrajInterlock
                                        || interlocks_.maxVelocityTrajInterlock
                                        || interlocks_.velocityDiffTrajInterlock
                                        || interlocks_.plcInterlock
                                        || interlocks_.plcInterlockFWD
                                        || interlocks_.encDiffInterlock
                                        || interlocks_.safetyInterlock;
  interlocks_.trajSummaryInterlockFWD = interlocks_.trajSummaryInterlockFWDEpics 
                                        || interlocks_.noExecuteInterlock;

  // printout interlock changes
  if(control_.controlWord_.enableDbgPrintout) {
    if(interlocksOld_.driveSummaryInterlock != interlocks_.driveSummaryInterlock) {
      printf("interlocks_.driveSummaryInterlock changed:\n");
      printf("interlocks_.bothLimitsLowInterlock= %d\n",interlocks_.bothLimitsLowInterlock);
      printf("interlocks_.bothLimitsLowInterlock= %d\n",interlocks_.bothLimitsLowInterlock);
      printf("interlocks_.cntrlOutputHLDriveInterlock= %d\n",interlocks_.cntrlOutputHLDriveInterlock);
      printf("interlocks_.lagDriveInterlock= %d\n",interlocks_.lagDriveInterlock);
      printf("interlocks_.maxVelocityDriveInterlock= %d\n",interlocks_.maxVelocityDriveInterlock);
      printf("interlocks_.velocityDiffDriveInterlock= %d\n",interlocks_.velocityDiffDriveInterlock);
      printf("interlocks_.hardwareInterlock= %d\n",interlocks_.hardwareInterlock);
      printf("interlocks_.etherCatMasterInterlock= %d\n",interlocks_.etherCatMasterInterlock);
      printf("interlocks_.analogInterlock= %d\n",interlocks_.analogInterlock);
      printf("interlocks_.stallInterlock= %d\n",interlocks_.stallInterlock);
    }

    if(interlocksOld_.trajSummaryInterlockBWD !=interlocks_.trajSummaryInterlockBWD) {
      printf("interlocks_.trajSummaryInterlockBWD changed:\n");
      printf("interlocks_.axisErrorStateInterlock= %d\n",interlocks_.axisErrorStateInterlock);
      printf("interlocks_.bwdLimitInterlock= %d\n",interlocks_.bwdLimitInterlock);
      printf("interlocks_.bwdSoftLimitInterlock= %d\n",interlocks_.bwdSoftLimitInterlock);
      printf("interlocks_.cntrlOutputHLTrajInterlock= %d\n",interlocks_.cntrlOutputHLTrajInterlock);
      printf("interlocks_.lagTrajInterlock= %d\n",interlocks_.lagTrajInterlock);
      printf("interlocks_.maxVelocityTrajInterlock= %d\n",interlocks_.maxVelocityTrajInterlock);
      printf("interlocks_.noExecuteInterlock= %d\n",interlocks_.noExecuteInterlock);
      printf("interlocks_.velocityDiffTrajInterlock= %d\n",interlocks_.velocityDiffTrajInterlock);
      printf("interlocks_.plcInterlock= %d\n",interlocks_.plcInterlock);
      printf("interlocks_.plcInterlockBWD= %d\n",interlocks_.plcInterlockBWD);
      printf("interlocks_.encDiffInterlock= %d\n",interlocks_.encDiffInterlock);
      printf("interlocks_.safetyInterlock= %d\n",interlocks_.safetyInterlock);
    }
  
    if(interlocksOld_.trajSummaryInterlockFWD !=interlocks_.trajSummaryInterlockFWD) {
      printf("interlocks_.trajSummaryInterlockFWD changed:\n");
      printf("interlocks_.axisErrorStateInterlock = %d \n",interlocks_.axisErrorStateInterlock);
      printf("interlocks_.cntrlOutputHLTrajInterlock = %d \n",interlocks_.cntrlOutputHLTrajInterlock);
      printf("interlocks_.fwdLimitInterlock = %d \n",interlocks_.fwdLimitInterlock);
      printf("interlocks_.fwdSoftLimitInterlock = %d \n",interlocks_.fwdSoftLimitInterlock);
      printf("interlocks_.lagTrajInterlock = %d \n",interlocks_.lagTrajInterlock);
      printf("interlocks_.maxVelocityTrajInterlock = %d \n",interlocks_.maxVelocityTrajInterlock);
      printf("interlocks_.noExecuteInterlock = %d \n",interlocks_.noExecuteInterlock);
      printf("interlocks_.velocityDiffTrajInterlock = %d \n",interlocks_.velocityDiffTrajInterlock);
      printf("interlocks_.plcInterlock = %d \n",interlocks_.plcInterlock);
      printf("interlocks_.plcInterlockFWD = %d \n",interlocks_.plcInterlockFWD);
      printf("interlocks_.encDiffInterlock = %d \n",interlocks_.encDiffInterlock);
      printf("interlocks_.safetyInterlock; = %d \n",interlocks_.safetyInterlock);
    }
  }

  interlocksOld_.driveSummaryInterlock   = interlocks_.driveSummaryInterlock;
  interlocksOld_.trajSummaryInterlockBWD = interlocks_.trajSummaryInterlockBWD;
  interlocksOld_.trajSummaryInterlockFWD = interlocks_.trajSummaryInterlockFWD;
  
  return 0;
}

void ecmcAxisData::clearInterlocks() {
  // Clear and refresh struct
  memset(&interlocks_, 0, sizeof(interlocks_));
  refreshInterlocks();
}
