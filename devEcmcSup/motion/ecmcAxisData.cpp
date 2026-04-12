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
#include "ecmcRtLogger.h"
#include "ecmcOctetIF.h"

namespace {
const char *interlockToString(interlockTypes interlock) {
  switch (interlock) {
  case ECMC_INTERLOCK_NONE:
    return "none";
  case ECMC_INTERLOCK_SOFT_BWD:
    return "soft limit backward";
  case ECMC_INTERLOCK_SOFT_FWD:
    return "soft limit forward";
  case ECMC_INTERLOCK_HARD_BWD:
    return "hard limit backward";
  case ECMC_INTERLOCK_HARD_FWD:
    return "hard limit forward";
  case ECMC_INTERLOCK_NO_EXECUTE:
    return "no execute";
  case ECMC_INTERLOCK_POSITION_LAG:
    return "position lag";
  case ECMC_INTERLOCK_BOTH_LIMITS:
    return "both limits";
  case ECMC_INTERLOCK_EXTERNAL:
    return "external interlock";
  case ECMC_INTERLOCK_TRANSFORM:
    return "transform interlock";
  case ECMC_INTERLOCK_MAX_SPEED:
    return "max speed";
  case ECMC_INTERLOCK_CONT_HIGH_LIMIT:
    return "controller high limit";
  case ECMC_INTERLOCK_CONT_OUT_INCREASE_AT_LIMIT_SWITCH:
    return "control output increase at limit";
  case ECMC_INTERLOCK_AXIS_ERROR_STATE:
    return "axis error state";
  case ECMC_INTERLOCK_UNEXPECTED_LIMIT_SWITCH_BEHAVIOUR:
    return "unexpected limit switch behaviour";
  case ECMC_INTERLOCK_VELOCITY_DIFF:
    return "velocity difference";
  case ECMC_INTERLOCK_ETHERCAT_MASTER_NOT_OK:
    return "EtherCAT master not OK";
  case ECMC_INTERLOCK_PLC_NORMAL:
    return "PLC interlock";
  case ECMC_INTERLOCK_PLC_BWD:
    return "PLC interlock backward";
  case ECMC_INTERLOCK_PLC_FWD:
    return "PLC interlock forward";
  case ECMC_INTERLOCK_ENC_DIFF:
    return "encoder difference";
  case ECMC_INTERLOCK_ANALOG:
    return "analog interlock";
  case ECMC_INTERLOCK_SAFETY:
    return "safety interlock";
  case ECMC_INTERLOCK_STALL:
    return "stall";
  default:
    return "unknown interlock";
  }
}

void appendActiveInterlock(char *buffer,
                           size_t bufferSize,
                           bool active,
                           const char *name) {
  if (!active) {
    return;
  }

  const size_t bufferLength = strlen(buffer);
  if (bufferLength >= bufferSize) {
    return;
  }

  snprintf(buffer + bufferLength,
           bufferSize - bufferLength,
           "%s%s",
           bufferLength ? "," : "",
           name);
}

void printInterlockSummaryChange(int axisId,
                                 const char *summaryName,
                                 bool active,
                                 const char *causes) {
  ECMC_RT_LOG_AXIS_DATA_DEBUG(axisId,
                              "%s/%s:%d: DEBUG: Axis[%d]: Interlock summary %s %s: active=%s.\n",
                              __FILE__,
                              __FUNCTION__,
                              __LINE__,
                              axisId,
                              summaryName,
                              active ? "active" : "cleared",
                              causes[0] ? causes : "none");
}
}

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

interlockTypes ecmcAxisData::getInterlockForMotion() const {
  if ((interlocks_.interlockStatus == ECMC_INTERLOCK_NONE) ||
      (interlocks_.interlockStatus == ECMC_INTERLOCK_NO_EXECUTE)) {
    return interlocks_.interlockStatus;
  }

  if (interlocks_.safetyInterlock) return ECMC_INTERLOCK_SAFETY;
  if (interlocks_.bothLimitsLowInterlock) return ECMC_INTERLOCK_BOTH_LIMITS;
  if (interlocks_.velocityDiffDriveInterlock ||
      interlocks_.velocityDiffTrajInterlock) {
    return ECMC_INTERLOCK_VELOCITY_DIFF;
  }
  if (interlocks_.cntrlOutputHLDriveInterlock ||
      interlocks_.cntrlOutputHLTrajInterlock) {
    return ECMC_INTERLOCK_CONT_HIGH_LIMIT;
  }
  if (interlocks_.etherCatMasterInterlock) {
    return ECMC_INTERLOCK_ETHERCAT_MASTER_NOT_OK;
  }
  if (interlocks_.hardwareInterlock) return ECMC_INTERLOCK_EXTERNAL;
  if (interlocks_.lagDriveInterlock || interlocks_.lagTrajInterlock) {
    return ECMC_INTERLOCK_POSITION_LAG;
  }
  if (interlocks_.analogInterlock) return ECMC_INTERLOCK_ANALOG;
  if (interlocks_.stallInterlock) return ECMC_INTERLOCK_STALL;

  const bool useVelocityDirection =
    (status_.command == ECMC_CMD_MOVEVEL) ||
    (status_.command == ECMC_CMD_JOG) ||
    (status_.command == ECMC_CMD_MOVEPVTABS) ||
    (status_.statusWord_.trajsource != ECMC_DATA_SOURCE_INTERNAL);
  const bool movingBwd =
    (status_.currentTargetPosition < status_.currentPositionSetpoint) ||
    (status_.currentPositionSetpoint < statusOld_.currentPositionSetpoint) ||
    (useVelocityDirection && (status_.currentVelocitySetpoint < 0));
  const bool movingFwd =
    (status_.currentTargetPosition > status_.currentPositionSetpoint) ||
    (status_.currentPositionSetpoint > statusOld_.currentPositionSetpoint) ||
    (useVelocityDirection && (status_.currentVelocitySetpoint > 0));

  if (movingBwd && !movingFwd) {
    if (interlocks_.bwdLimitInterlock) return ECMC_INTERLOCK_HARD_BWD;
    if (interlocks_.bwdSoftLimitInterlock) return ECMC_INTERLOCK_SOFT_BWD;
  } else if (movingFwd && !movingBwd) {
    if (interlocks_.fwdLimitInterlock) return ECMC_INTERLOCK_HARD_FWD;
    if (interlocks_.fwdSoftLimitInterlock) return ECMC_INTERLOCK_SOFT_FWD;
  } else {
    return interlocks_.interlockStatus;
  }

  if (interlocks_.maxVelocityDriveInterlock ||
      interlocks_.maxVelocityTrajInterlock) {
    return ECMC_INTERLOCK_MAX_SPEED;
  }
  if (interlocks_.plcInterlock) return ECMC_INTERLOCK_PLC_NORMAL;
  if (movingBwd && !movingFwd && interlocks_.plcInterlockBWD) {
    return ECMC_INTERLOCK_PLC_BWD;
  }
  if (movingFwd && !movingBwd && interlocks_.plcInterlockFWD) {
    return ECMC_INTERLOCK_PLC_FWD;
  }
  if (interlocks_.encDiffInterlock) return ECMC_INTERLOCK_ENC_DIFF;
  if (interlocks_.axisErrorStateInterlock) {
    return ECMC_INTERLOCK_AXIS_ERROR_STATE;
  }

  return ECMC_INTERLOCK_NONE;
}

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

  // Latch the first interlock seen for the current motion command.
  if ((interlocks_.interlockStatus != ECMC_INTERLOCK_NONE) &&
      (interlocks_.interlockStatus != ECMC_INTERLOCK_NO_EXECUTE) &&
      (interlocks_.lastActiveInterlock == ECMC_INTERLOCK_NONE)) {
    interlockTypes motionInterlock = getInterlockForMotion();
    if ((motionInterlock != ECMC_INTERLOCK_NONE) &&
        (motionInterlock != ECMC_INTERLOCK_NO_EXECUTE)) {
      interlocks_.lastActiveInterlock = motionInterlock;
    }
  }

  if ((oldInterlock != interlocks_.interlockStatus) &&
      !status_.statusWord_.instartup) {
    if (interlocks_.interlockStatus) {
      ECMC_RT_LOG_AXIS_DATA_INFO(status_.axisId,
                                 "%s/%s:%d: INFO: Axis[%d]: Motion interlocked: %s (type %d).\n",
                                 __FILE__,
                                 __FUNCTION__,
                                 __LINE__,
                                 status_.axisId,
                                 interlockToString(interlocks_.interlockStatus),
                                 interlocks_.interlockStatus);
    } else {
      ECMC_RT_LOG_AXIS_DATA_INFO(status_.axisId,
                                 "%s/%s:%d: INFO: Axis[%d]: Motion interlock cleared.\n",
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
    char activeInterlocks[512];
    if(interlocksOld_.driveSummaryInterlock != interlocks_.driveSummaryInterlock) {
      activeInterlocks[0] = '\0';
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.bothLimitsLowInterlock, "bothLimitsLow");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.cntrlOutputHLDriveInterlock, "driveCntrlHL");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.lagDriveInterlock, "driveLag");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.maxVelocityDriveInterlock, "driveMaxVelocity");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.velocityDiffDriveInterlock, "driveVelocityDiff");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.hardwareInterlock, "hardware");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.etherCatMasterInterlock, "etherCatMaster");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.analogInterlock, "analog");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.stallInterlock, "stall");
      printInterlockSummaryChange(status_.axisId,
                                  "drive",
                                  interlocks_.driveSummaryInterlock,
                                  activeInterlocks);
    }

    if(interlocksOld_.trajSummaryInterlockBWD !=interlocks_.trajSummaryInterlockBWD) {
      activeInterlocks[0] = '\0';
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.driveSummaryInterlock, "drive");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.axisErrorStateInterlock, "axisErrorState");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.bwdLimitInterlock, "bwdLimit");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.bwdSoftLimitInterlock, "bwdSoftLimit");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.cntrlOutputHLTrajInterlock, "trajCntrlHL");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.lagTrajInterlock, "trajLag");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.maxVelocityTrajInterlock, "trajMaxVelocity");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.noExecuteInterlock, "noExecute");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.velocityDiffTrajInterlock, "trajVelocityDiff");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.plcInterlock, "plc");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.plcInterlockBWD, "plcBwd");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.encDiffInterlock, "encDiff");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.safetyInterlock, "safety");
      printInterlockSummaryChange(status_.axisId,
                                  "trajectory backward",
                                  interlocks_.trajSummaryInterlockBWD,
                                  activeInterlocks);
    }
  
    if(interlocksOld_.trajSummaryInterlockFWD !=interlocks_.trajSummaryInterlockFWD) {
      activeInterlocks[0] = '\0';
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.driveSummaryInterlock, "drive");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.axisErrorStateInterlock, "axisErrorState");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.fwdLimitInterlock, "fwdLimit");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.fwdSoftLimitInterlock, "fwdSoftLimit");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.cntrlOutputHLTrajInterlock, "trajCntrlHL");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.lagTrajInterlock, "trajLag");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.maxVelocityTrajInterlock, "trajMaxVelocity");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.noExecuteInterlock, "noExecute");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.velocityDiffTrajInterlock, "trajVelocityDiff");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.plcInterlock, "plc");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.plcInterlockFWD, "plcFwd");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.encDiffInterlock, "encDiff");
      appendActiveInterlock(activeInterlocks, sizeof(activeInterlocks),
                            interlocks_.safetyInterlock, "safety");
      printInterlockSummaryChange(status_.axisId,
                                  "trajectory forward",
                                  interlocks_.trajSummaryInterlockFWD,
                                  activeInterlocks);
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

void ecmcAxisData::clearLatchedInterlock() {
  interlocks_.lastActiveInterlock = ECMC_INTERLOCK_NONE;
}
