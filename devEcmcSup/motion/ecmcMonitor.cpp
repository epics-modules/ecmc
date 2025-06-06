/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcMonitor.cpp
*
*  Created on: Jan 12, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcMonitor.h"
#include <stdio.h>

ecmcMonitor::ecmcMonitor(ecmcAxisData *axisData,
                         ecmcEncoder **encArray) : ecmcEcEntryLink(&(axisData->
                                                                     status_.
                                                                     errorCode),
                                                                   &(axisData->
                                                                     status_.
                                                                     warningCode))
{
  initVars();
  data_ = axisData;
  setExternalPtrs(&(data_->status_.errorCode), &(data_->status_.warningCode));
  encArray_ = encArray;

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
  errorReset();
}

void ecmcMonitor::initVars() {
  enable_             = false;
  atTargetTol_        = 0;
  atTargetTime_       = 0;
  enableAtTargetMon_  = true;
  posLagTol_          = 0;
  posLagTime_         = 0;
  enableLagMon_       = true;
  atTargetCounter_    = 0;
  lagMonCounter_      = 0;
  maxVel_             = 0;
  enableMaxVelMon_    = 0;
  maxVelCounterDrive_ = 0;
  maxVelCounterTraj_  = 0;
  maxVelDriveILDelay_ = 0;
  maxStallCounter_    = 0;
  stallLastMotionCmdCycles_  = 0;
  stallCheckAtTargetAtCycle_ = 0;
  stallMinTimeoutCycles_     = 0;
  stallTimeFactor_           = 10;
  enableStallMon_            = 0;

  // 200 cycles
  maxVelTrajILDelay_ = 200;

  // 400 cycles default
  maxVelDriveILDelay_        = maxVelTrajILDelay_ * 2;
  enableHardwareInterlock_   = false;
  cntrlOutputHL_             = 0;
  enableCntrlHLMon_          = false;
  enableVelocityDiffMon_     = false;
  velocityDiffCounter_       = 0;
  enableAlarmAtHardlimitBwd_ = false;
  enableAlarmAtHardlimitFwd_ = false;
  hardBwdOld_                = false;
  hardFwdOld_                = false;
  velDiffTimeTraj_           = 100;
  velDiffTimeDrive_          = 100;
  velDiffMaxDiff_            = 0;
  latchOnLimit_              = 1;
  switchFilterCounter_       = 0;
  memset(&limitFwdFilterBuffer_, 0, sizeof(limitFwdFilterBuffer_));
  memset(&limitBwdFilterBuffer_, 0, sizeof(limitBwdFilterBuffer_));
  memset(&homeFilterBuffer_,     0, sizeof(homeFilterBuffer_));
  interlockStatusOld_        = ECMC_INTERLOCK_NONE;
  hardwareInterlockPolarity_ = ECMC_POLARITY_NC;
  lowLimPolarity_            = ECMC_POLARITY_NC;
  highLimPolarity_           = ECMC_POLARITY_NC;
  homePolarity_              = ECMC_POLARITY_NC;
  encArray_                  = NULL;
  enableAlarmOnSofLimits_    = 1;
  enableDiffEncsMon_         = 1;  // If a tolerance is set then default check
  ctrlDeadbandTol_           = -1;
  ctrlDeadbandCounter_       = 0;
  ctrlDeadbandTime_          = -1;
  analogRawLimit_            = 0;
  enableAnalogInterlock_     = 0;
  analogPolarity_            = ECMC_POLARITY_NC; // Higher value than analogRawLimit_ is bad
  limitSwitchFwdPLCOverride_ = false;
  limitSwitchBwdPLCOverride_ = false;
  limitSwitchFwdPLCOverrideValue_ = false;
  limitSwitchBwdPLCOverrideValue_ = false;
  homeSwitchPLCOverride_          = false;
  homeSwitchPLCOverrideValue_     = false;
  enableHomeSensor_               = false;
  ctrlWinthinDBExternalTraj_      = false;
}

ecmcMonitor::~ecmcMonitor() {}

void ecmcMonitor::execute() {
  // Limits
  checkLimits();

  // At target
  checkAtTarget();

  // External interlock (on ethercat I/O)
  if (enableHardwareInterlock_ &&  data_->interlocks_.hardwareInterlock &&
      enable_) {
    setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_MON_EXTERNAL_HARDWARE_INTERLOCK,
               ECMC_SEVERITY_NORMAL);
  }

  // Lag error
  checkPositionLag();

  // Encoder diff
  checkEncoderDiff();

  // Max Vel
  checkMaxVelocity();

  checkStall();

  // Controller output HL
  checkCntrlMaxOutput();

  // Controller output increase at limit switches monitoring
  checkVelocityDiff();
  
  // refuse start when error code except axis disabled error (for backwards compatibility)
  if( data_->status_.errorCode != 0 && 
      data_->status_.enabled ) {
    data_->interlocks_.axisErrorStateInterlock = 1;
  }
  else {
    data_->interlocks_.axisErrorStateInterlock = 0;
  }

  data_->refreshInterlocks();
  interlockStatusOld_ = data_->interlocks_.interlockStatus;
}

bool ecmcMonitor::getAtTarget() {
  return data_->status_.atTarget;
}

bool ecmcMonitor::getCtrlInDeadband() {
  return data_->status_.ctrlWinthinDeadband;
}

bool ecmcMonitor::getHardLimitFwd() {
  return data_->status_.limitFwd;
}

bool ecmcMonitor::getHardLimitBwd() {
  return data_->status_.limitBwd;
}

int ecmcMonitor::setAtTargetTol(double tol) {
  if (tol < 0) {
    LOGERR("%s/%s:%d: At target tolerance invalid (tol. < 0) (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, ERROR_MON_TOL_OUT_OF_RANGE);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_MON_TOL_OUT_OF_RANGE);
  }

  atTargetTol_ = tol;

  // Default also for ctrl deadband
  if (ctrlDeadbandTol_ < 0) {
    ctrlDeadbandTol_ = tol;
  }

  return 0;
}

double ecmcMonitor::getAtTargetTol() {
  return atTargetTol_;
}

int ecmcMonitor::setAtTargetTime(int time) {
  if (time < 0) {
    LOGERR("%s/%s:%d: At target time invalid (time < 0) (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, ERROR_MON_TIME_OUT_OF_RANGE);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_MON_TIME_OUT_OF_RANGE);
  }

  atTargetTime_ = time;

  // Default also for ctrl deadband time
  if (ctrlDeadbandTime_ < 0) {
    ctrlDeadbandTime_ = time;
  }

  return 0;
}

int ecmcMonitor::getAtTargetTime() {
  return atTargetTime_;
}

void ecmcMonitor::setEnableAtTargetMon(bool enable) {
  enableAtTargetMon_ = enable;
}

bool ecmcMonitor::getEnableAtTargetMon() {
  return enableAtTargetMon_;
}

int ecmcMonitor::setPosLagTol(double tol) {
  if (tol < 0) {
    LOGERR("%s/%s:%d: Position lag tolerance invalid (tol. < 0) (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, ERROR_MON_TOL_OUT_OF_RANGE);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_MON_TOL_OUT_OF_RANGE);
  }

  posLagTol_ = tol;
  return 0;
}

double ecmcMonitor::getPosLagTol() {
  return posLagTol_;
}

int ecmcMonitor::setVelDiffTimeTraj(int time) {
  if (time < 0) {
    LOGERR("%s/%s:%d: Velocity diff traj time invalid (time < 0) (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, ERROR_MON_TIME_OUT_OF_RANGE);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_MON_TIME_OUT_OF_RANGE);
  }

  velDiffTimeTraj_ = time;
  return 0;
}

int ecmcMonitor::setVelDiffTimeDrive(int time) {
  if (time < 0) {
    LOGERR("%s/%s:%d: Velocity diff drive time invalid (time < 0) (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, ERROR_MON_TIME_OUT_OF_RANGE);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_MON_TIME_OUT_OF_RANGE);
  }

  velDiffTimeDrive_ = time;
  return 0;
}

int ecmcMonitor::setPosLagTime(int time) {
  if (time < 0) {
    LOGERR("%s/%s:%d: Position lag time invalid (time < 0) (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, ERROR_MON_TIME_OUT_OF_RANGE);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_MON_TIME_OUT_OF_RANGE);
  }

  posLagTime_ = time;
  return 0;
}

int ecmcMonitor::getPosLagTime() {
  return posLagTime_;
}

void ecmcMonitor::setEnableLagMon(bool enable) {
  if(enable && !enableLagMon_) {
    lagMonCounter_ = 0;
  }
  enableLagMon_ = enable;
}

bool ecmcMonitor::getEnableLagMon() {
  return enableLagMon_;
}

bool ecmcMonitor::getHomeSwitch() {
  return data_->status_.homeSwitch;
}

void ecmcMonitor::readEntries() {
  uint64_t tempRaw = 0;
  int errorCode = 0;
  // Hard limit BWD
  if(!limitSwitchBwdPLCOverride_) {
    errorCode = readEcEntryValue(ECMC_MON_ENTRY_INDEX_LOWLIM, &tempRaw);
  
    if (errorCode) {
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 errorCode,
                 ECMC_SEVERITY_EMERGENCY);
      return;
    }
  
    if (lowLimPolarity_ == ECMC_POLARITY_NO) {
      tempRaw = tempRaw == 0;
    }
  
    data_->status_.limitBwd = tempRaw > 0;

  } else {
    // Override limit switch from PLC code (when logic is needed)
    data_->status_.limitBwd = limitSwitchBwdPLCOverrideValue_;
  }

  // Hard limit FWD
  tempRaw = 0;
  if(!limitSwitchFwdPLCOverride_) {
    errorCode = readEcEntryValue(ECMC_MON_ENTRY_INDEX_HIGHLIM, &tempRaw);
  
    if (errorCode) {
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 errorCode,
                 ECMC_SEVERITY_NORMAL);
      return;
    }
  
    if (highLimPolarity_ == ECMC_POLARITY_NO) {
      tempRaw = tempRaw == 0;
    }
  
    data_->status_.limitFwd = tempRaw > 0;

  } else {
      // Override limit switch from PLC code (when logic is needed)
      data_->status_.limitFwd = limitSwitchFwdPLCOverrideValue_;
  }

  // Home
  tempRaw = 0;
  if(!homeSwitchPLCOverride_ && enableHomeSensor_) {
    errorCode = readEcEntryValue(ECMC_MON_ENTRY_INDEX_HOMESENSOR, &tempRaw);
  
    if (errorCode) {
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 errorCode,
                 ECMC_SEVERITY_NORMAL);
      return;
    }
  
    if (homePolarity_ == ECMC_POLARITY_NO) {
      tempRaw = tempRaw == 0;
    }
  
    data_->status_.homeSwitch = tempRaw > 0;
  } else {
    // Override limit switch from PLC code (when logic is needed)
    data_->status_.homeSwitch = homeSwitchPLCOverrideValue_;
  }

  // Refresh filtered switches
  filterSwitches();

  if (enableHardwareInterlock_) {
    tempRaw   = 0;
    errorCode = readEcEntryValue(ECMC_MON_ENTRY_INDEX_EXTINTERLOCK, &tempRaw);

    if (errorCode) {
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 errorCode,
                 ECMC_SEVERITY_NORMAL);
      return;
    }

    switch (hardwareInterlockPolarity_) {
    case ECMC_POLARITY_NC:
      data_->interlocks_.hardwareInterlock = tempRaw == 0;
      break;

    case ECMC_POLARITY_NO:
      data_->interlocks_.hardwareInterlock = tempRaw > 0;
      break;
    }
  }

  if (enableAnalogInterlock_) {
    double tempDouble = 0;
    errorCode = readEcEntryValueDouble(ECMC_MON_ENTRY_INDEX_ANALOG, &tempDouble);

    if (errorCode) {
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 errorCode,
                 ECMC_SEVERITY_NORMAL);
      return;
    }

    switch (analogPolarity_) {
    case ECMC_POLARITY_NC:
      data_->interlocks_.analogInterlock = tempDouble > analogRawLimit_;
      break;

    case ECMC_POLARITY_NO:
      data_->interlocks_.analogInterlock = tempDouble < analogRawLimit_;
      break;
    }
  }
}

void ecmcMonitor::setEnable(bool enable) {
  enable_ = enable;
}

bool ecmcMonitor::getEnable() {
  return enable_;
}

int ecmcMonitor::validate() {
  int errorCode = 0;
  if(!limitSwitchBwdPLCOverride_) {
    errorCode = validateEntryBit(ECMC_MON_ENTRY_INDEX_LOWLIM);
  
    if (errorCode) {  // Hard limit BWD
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_MON_ENTRY_HARD_BWD_NULL);
    }
  }

  if(!limitSwitchFwdPLCOverride_) {
    errorCode = validateEntryBit(ECMC_MON_ENTRY_INDEX_HIGHLIM);
  
    if (errorCode) {  // Hard limit FWD
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_MON_ENTRY_HARD_FWD_NULL);
    }
  }

  if(!homeSwitchPLCOverride_ && enableHomeSensor_) {
    errorCode = validateEntryBit(ECMC_MON_ENTRY_INDEX_HOMESENSOR);
  
    if (errorCode) {  // Home
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_MON_ENTRY_HOME_NULL);
    }
  }

  if (enableHardwareInterlock_) {
    errorCode = validateEntryBit(ECMC_MON_ENTRY_INDEX_EXTINTERLOCK);

    if (errorCode) {  // External interlock
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_MON_ENTRY_EXT_INTERLOCK_NULL);
    }
  }

  if (enableAnalogInterlock_) {
    errorCode = validateEntry(ECMC_MON_ENTRY_INDEX_ANALOG);

    if (errorCode) {  // Analog interlock
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_MON_ENTRY_ANALOG_INTERLOCK_NULL);
    }
  }

  return 0;
}

int ecmcMonitor::setMaxVel(double vel) {
  maxVel_ = vel;
  return 0;
}

double ecmcMonitor::getMaxVel(void) {
  return maxVel_;
}

int ecmcMonitor::setEnableMaxVelMon(bool enable) {
  enableMaxVelMon_ = enable;
  return 0;
}

bool ecmcMonitor::getEnableMaxVelMon() {
  return enableMaxVelMon_;
}

int ecmcMonitor::setMaxVelDriveTime(int time) {
  if (time < 0) {
    LOGERR("%s/%s:%d: Max velocity drive time invalid (time < 0) (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, ERROR_MON_TIME_OUT_OF_RANGE);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_MON_TIME_OUT_OF_RANGE);
  }

  maxVelDriveILDelay_ = time;
  return 0;
}

int ecmcMonitor::setMaxVelTrajTime(int time) {
  if (time < 0) {
    LOGERR("%s/%s:%d: Max velocity traj time invalid (time < 0) (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, ERROR_MON_TIME_OUT_OF_RANGE);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_MON_TIME_OUT_OF_RANGE);
  }

  maxVelTrajILDelay_ = time;
  return 0;
}

int ecmcMonitor::reset() {
  // data_->status_.atTarget = false;
  // atTargetCounter_        = 0;
  lagMonCounter_       = 0;
  maxVelCounterDrive_  = 0;
  maxVelCounterTraj_   = 0;
  velocityDiffCounter_ = 0;
  data_->clearInterlocks();
  return 0;
}

void ecmcMonitor::errorReset() {
  reset();
  ecmcEcEntryLink::errorReset();
  ecmcError::errorReset();
}

int ecmcMonitor::setEnableHardwareInterlock(bool enable) {
  if (enable) {
    int error = validateEntryBit(ECMC_MON_ENTRY_INDEX_EXTINTERLOCK);

    if (error) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_MON_ENTRY_HARDWARE_INTERLOCK_NULL);
    }
  }

  enableHardwareInterlock_ = enable;
  return 0;
}

int ecmcMonitor::setHomeSwitchEnable(bool enable) {
  if (enable) {
    int error = validateEntryBit(ECMC_MON_ENTRY_INDEX_HOMESENSOR);

    if (error) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_MON_ENTRY_HOME_NULL);
    }
  }

  enableHomeSensor_ = enable;
  return 0;
}

int ecmcMonitor::setEnableAnalogInterlock(bool enable) {
  if (enable) {
    int error = validateEntry(ECMC_MON_ENTRY_INDEX_ANALOG);

    if (error) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_MON_ENTRY_ANALOG_INTERLOCK_NULL);
    }
  }

  enableAnalogInterlock_ = enable;
  return 0;
}

int ecmcMonitor::setAnalogRawLimit(double analogLimit) {
  analogRawLimit_ = analogLimit;
  return 0;
}

int ecmcMonitor::setCntrlOutputHL(double outputHL) {
  cntrlOutputHL_ = outputHL;
  return 0;
}

int ecmcMonitor::setEnableCntrlHLMon(bool enable) {
  enableCntrlHLMon_ = enable;
  return 0;
}

bool ecmcMonitor::getEnableCntrlHLMon() {
  return enableCntrlHLMon_;
}

int ecmcMonitor::setEnableVelocityDiffMon(bool enable) {
  enableVelocityDiffMon_ = enable;
  return 0;
}

bool ecmcMonitor::getEnableVelocityDiffMon() {
  return enableVelocityDiffMon_;
}

int ecmcMonitor::setEnableHardLimitBWDAlarm(bool enable) {
  enableAlarmAtHardlimitBwd_ = enable;
  return 0;
}

int ecmcMonitor::setEnableHardLimitFWDAlarm(bool enable) {
  enableAlarmAtHardlimitFwd_ = enable;
  return 0;
}

int ecmcMonitor::setEnableSoftLimitBwd(bool enable) {
  data_->command_.enableSoftLimitBwd = enable;
  return 0;
}

int ecmcMonitor::setEnableSoftLimitFwd(bool enable) {
  data_->command_.enableSoftLimitFwd = enable;
  return 0;
}

int ecmcMonitor::setSoftLimitBwd(double limit) {
  data_->command_.softLimitBwd = limit;
  return 0;
}

int ecmcMonitor::setSoftLimitFwd(double limit) {
  data_->command_.softLimitFwd = limit;
  return 0;
}

int ecmcMonitor::checkLimits() {
  hardBwdOld_ = data_->status_.limitBwd;
  hardFwdOld_ = data_->status_.limitFwd;

  // Both limit switches

  data_->interlocks_.bothLimitsLowInterlock = !data_->status_.limitBwd &&
                                              !data_->status_.limitFwd;

  if (data_->interlocks_.bothLimitsLowInterlock) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MON_BOTH_LIMIT_INTERLOCK,
                      ECMC_SEVERITY_EMERGENCY);
  }

  // Bwd limit switch
  if (!data_->status_.limitBwd) {
    data_->interlocks_.bwdLimitInterlock = true;

    if (enableAlarmAtHardlimitBwd_) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_MON_HARD_LIMIT_BWD_INTERLOCK,
                        ECMC_SEVERITY_NORMAL);
    }
    setWarningID(WARNING_MON_HARD_LIMIT_BWD_INTERLOCK);
  } else {
    if (latchOnLimit_) {
      if (!data_->status_.moving ||
          (data_->status_.currentVelocityActual > 0)) {
        data_->interlocks_.bwdLimitInterlock = false;
      }
    } else {
      // Auto reset warning
      if (getWarningID() == WARNING_MON_HARD_LIMIT_BWD_INTERLOCK) {
        setWarningID(0);
      }
      data_->interlocks_.bwdLimitInterlock = false;
    }
  }

  // Fwd limit switch
  if (!data_->status_.limitFwd) {
    data_->interlocks_.fwdLimitInterlock = true;

    if (enableAlarmAtHardlimitFwd_) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_MON_HARD_LIMIT_FWD_INTERLOCK,
                        ECMC_SEVERITY_NORMAL);
    }
    setWarningID(WARNING_MON_HARD_LIMIT_FWD_INTERLOCK);
  } else {
    if (latchOnLimit_) {
      if (!data_->status_.moving ||
          (data_->status_.currentVelocityActual < 0)) {
        data_->interlocks_.fwdLimitInterlock = false;
      }
    } else {
      // Auto reset warning
      if (getWarningID() == WARNING_MON_HARD_LIMIT_FWD_INTERLOCK) {
        setWarningID(0);
      }
      data_->interlocks_.fwdLimitInterlock = false;
    }
  }

  // Bwd soft limit switch
  bool virtSoftlimitBwd =
    (data_->status_.currentPositionSetpoint < data_->command_.softLimitBwd) &&
    data_->status_.enabled && data_->status_.enabledOld; /*&&
    (data_->status_.currentPositionSetpoint <
    data_->status_.currentPositionSetpointOld); */// data_->command_.execute;

  if (virtSoftlimitBwd /*&& data_->status_.busy*/ &&
      data_->command_.enableSoftLimitBwd &&
      (data_->command_.command != ECMC_CMD_HOMING)) {
    data_->interlocks_.bwdSoftLimitInterlock = true;
    setWarningID(WARNING_MON_SOFT_LIMIT_BWD_INTERLOCK);

    // if(enableAlarmOnSofLimits_) {
    //  return setErrorID(__FILE__,
    //                    __FUNCTION__,
    //                    __LINE__,
    //                    ERROR_MON_SOFT_LIMIT_BWD_INTERLOCK);
    // }
  } else {
    // Auto reset this warning
    if (getWarningID() == WARNING_MON_SOFT_LIMIT_BWD_INTERLOCK) {
      setWarningID(0);
    }
    data_->interlocks_.bwdSoftLimitInterlock = false;
  }

  bool virtSoftlimitFwd =
    (data_->status_.currentPositionSetpoint > data_->command_.softLimitFwd) &&
    data_->status_.enabled && data_->status_.enabledOld; /*&&
    (data_->status_.currentPositionSetpoint >
      data_->status_.currentPositionSetpointOld);*/// && data_->command_.execute;

  // Fwd soft limit switch
  if (virtSoftlimitFwd /*&& data_->status_.busy*/ &&
      data_->command_.enableSoftLimitFwd &&
      (data_->command_.command != ECMC_CMD_HOMING)) {
    data_->interlocks_.fwdSoftLimitInterlock = true;
    setWarningID(WARNING_MON_SOFT_LIMIT_FWD_INTERLOCK);

    // if(enableAlarmOnSofLimits_) {
    //  return setErrorID(__FILE__,
    //                    __FUNCTION__,
    //                    __LINE__,
    //                    ERROR_MON_SOFT_LIMIT_FWD_INTERLOCK);
    // }
  } else {
    // Auto reset this warning
    if (getWarningID() == WARNING_MON_SOFT_LIMIT_FWD_INTERLOCK) {
      setWarningID(0);
    }
    data_->interlocks_.fwdSoftLimitInterlock = false;
  }
  return 0;
}

int ecmcMonitor::checkAtTarget() {
  bool atTarget      = false;
  bool ctrlWithinTol = false;

  if (enableAtTargetMon_) {
    /*if (std::abs(data_->status_.currentTargetPosition -
                 data_->status_.currentPositionActual) < atTargetTol_) {*/

    if (data_->status_.currentTargetPositionModulo ==
        data_->status_.currentPositionSetpoint) {
      if (std::abs(data_->status_.cntrlError) < atTargetTol_) {
        if (atTargetCounter_ <= atTargetTime_) {
          atTargetCounter_++;
        }

        if (atTargetCounter_ > atTargetTime_) {
          atTarget = true;
        }
      } else {
        atTargetCounter_ = 0;
      }

      // controller deadband
      if (std::abs(data_->status_.cntrlError) < ctrlDeadbandTol_) {
        if (ctrlDeadbandCounter_ <= ctrlDeadbandTime_) {
          ctrlDeadbandCounter_++;
        }

        if (ctrlDeadbandCounter_ > ctrlDeadbandTime_) {
          ctrlWithinTol = true;
        }
      } else {
        ctrlDeadbandCounter_ = 0;
      }
    }
  } else {
    atTarget = false; 
  }

  data_->status_.atTarget            = atTarget;
  
  if(data_->command_.trajSource == 0) {
    data_->status_.ctrlWinthinDeadband = ctrlWithinTol;
  } else {
    // external source used. No way for axis to know when atTarget/reduce torque. Make possible to write from PLC
    data_->status_.ctrlWinthinDeadband = ctrlWinthinDBExternalTraj_;
  }
  
  return 0;
}

// Only enabled when also atTarget monitoring is enabled
int  ecmcMonitor::checkStall() {

  // Do only check for stall when not busy (traj finished)
  if(!enableAtTargetMon_ || !enableStallMon_ ||
    !data_->status_.enabled || (data_->command_.trajSource != 0)) {
    data_->interlocks_.stallInterlock = false;
    maxStallCounter_ = 0;
    return 0;
  }
  
  // Measure time of last move, busy high to busy low.
  if(data_->status_.busy ) {
    stallLastMotionCmdCycles_++;
    if(!data_->status_.busyOld) {
      stallLastMotionCmdCycles_ = 0;
    }
    return 0;
  } else {
    if(data_->status_.busyOld) {
      stallCheckAtTargetAtCycle_ = stallLastMotionCmdCycles_ * stallTimeFactor_;
      // Ensure a minimum time window 
      if(stallCheckAtTargetAtCycle_ < stallMinTimeoutCycles_) {
        stallCheckAtTargetAtCycle_ = stallMinTimeoutCycles_;
      }
      if(data_->command_.enableDbgPrintout) {
        printf("Axis[%d]: Time to check stall after: %" PRIu64 "\n", data_->axisId_, stallCheckAtTargetAtCycle_);        
      }
    }
  }

  maxStallCounter_++;

  if(data_->status_.atTarget) {
    data_->interlocks_.stallInterlock = false;
    if(!data_->status_.atTargetOld) {
      if(data_->command_.enableDbgPrintout) {
        printf("Axis[%d]: No stall.. Brilliant!!\n",data_->axisId_);
      }
    }
    maxStallCounter_ = 0;
    return 0;
  }

  if((maxStallCounter_ > stallCheckAtTargetAtCycle_) && 
     (stallCheckAtTargetAtCycle_ > 0)) {
    if(data_->command_.enableDbgPrintout) {
      printf("Axis[%d]: Stall...\n",data_->axisId_);
    }
    stallCheckAtTargetAtCycle_ = 0;
    data_->interlocks_.stallInterlock = true;    
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MON_STALL,
                      ECMC_SEVERITY_EMERGENCY);
  }
  return 0;
}

int ecmcMonitor::checkPositionLag() {
  bool lagErrorTraj  = false;
  bool lagErrorDrive = false;

  if (enableLagMon_ && !lagErrorDrive && data_->command_.enable) {
    if ((std::abs(data_->status_.cntrlError) > posLagTol_) &&
        data_->status_.enabled &&
        data_->status_.enabledOld) {
      if (lagMonCounter_ <= posLagTime_ * 2) {
        lagMonCounter_++;
      }

      if (lagMonCounter_ > posLagTime_) {
        lagErrorTraj = true;
      }

      // interlock the drive in twice the time..
      if (lagMonCounter_ >= posLagTime_ * 2) {
        lagErrorDrive = true;
      }
    } else {
      lagMonCounter_ = 0;
    }
  } else {
    lagMonCounter_ = 0;
  }

  data_->interlocks_.lagDriveInterlock = lagErrorDrive;
  data_->interlocks_.lagTrajInterlock  = lagErrorTraj;

  if (lagErrorDrive || lagErrorTraj) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MON_MAX_POSITION_LAG_EXCEEDED,
                      ECMC_SEVERITY_NORMAL);
  }
  return 0;
}

int ecmcMonitor::checkEncoderDiff() {
  data_->interlocks_.encDiffInterlock = false;

  if (!data_->command_.enable) {
    return 0;
  }

  // Only one encoder configured
  if ((data_->status_.encoderCount <= 1) || !enableDiffEncsMon_) {
    return 0;
  }

  // Do not check if prim enc not homed
  if (!encArray_[data_->command_.primaryEncIndex]->getHomed()) {
    return 0;
  }

  // Multiple encoders configured so check pos diff vs primary
  double maxDiff       = 0;
  bool   encDiffILock  = false;
  double primEncActPos =
    encArray_[data_->command_.primaryEncIndex]->getActPos();

  for (int i = 0; i < data_->status_.encoderCount; i++) {
    // Do not check prim encoder vs itself or if this encoder is not homed
    if ((i == data_->command_.primaryEncIndex) || !encArray_[i]->getHomed()) {
      continue;
    }

    maxDiff = encArray_[i]->getMaxPosDiffToPrimEnc();

    // disable functionality if getMaxPosDiffToPrimEnc() == 0
    if (maxDiff == 0) {
      continue;
    }

    double diff = ecmcMotionUtils::getPosErrorModAbs(primEncActPos,
                                                     encArray_[i]->getActPos(),
                                                     data_->command_.moduloRange);

    encDiffILock = diff > maxDiff || encDiffILock;
  }

  data_->interlocks_.encDiffInterlock = encDiffILock;

  return 0;
}

int ecmcMonitor::checkVelocityDiff() {
  // cntrlOutput_/cntrlKff_;
  double currentSetVelocityToDrive = data_->status_.cntrlOutput;
  bool   velocityDiffErrorDrive    = false;
  bool   velocityDiffErrorTraj     = false;

  if (!enableVelocityDiffMon_ || !data_->status_.enabled ||
      !data_->status_.enabledOld) {
    velocityDiffCounter_ = 0;
  }

  if (std::abs(currentSetVelocityToDrive -
               data_->status_.currentVelocityActual) > velDiffMaxDiff_) {
    velocityDiffCounter_++;
  } else {
    velocityDiffCounter_ = 0;
  }

  if (velocityDiffCounter_ > velDiffTimeTraj_) {
    velocityDiffErrorTraj = true;
  }

  if (velocityDiffCounter_ > velDiffTimeDrive_) {
    velocityDiffErrorDrive = true;
  }

  data_->interlocks_.velocityDiffDriveInterlock = velocityDiffErrorDrive;
  data_->interlocks_.velocityDiffTrajInterlock  = velocityDiffErrorTraj;

  if (velocityDiffErrorDrive || velocityDiffErrorTraj) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MON_VELOCITY_DIFFERENCE_EXCEEDED,
                      ECMC_SEVERITY_NORMAL);
  }

  return 0;
}

int ecmcMonitor::checkMaxVelocity() {
  if (!data_->command_.enable || !data_->status_.enabled ||
      !data_->status_.enabledOld) {
    data_->interlocks_.maxVelocityTrajInterlock  = false;
    data_->interlocks_.maxVelocityDriveInterlock = false;
    maxVelCounterTraj_                           = 0;
    maxVelCounterDrive_                          = 0;
    return 0;
  }

  if (((std::abs(data_->status_.currentVelocityActual) > maxVel_) ||
       (std::abs(data_->status_.currentVelocitySetpoint) > maxVel_)) &&
      enableMaxVelMon_) {
    if (maxVelCounterTraj_ <= maxVelTrajILDelay_) {
      maxVelCounterTraj_++;
    }
  } else {
    maxVelCounterTraj_ = 0;
  }

  if (!data_->interlocks_.maxVelocityTrajInterlock) {
    data_->interlocks_.maxVelocityTrajInterlock = maxVelCounterTraj_ >=
                                                  maxVelTrajILDelay_;
  }

  if (data_->interlocks_.maxVelocityTrajInterlock &&
      (maxVelCounterDrive_ <= maxVelDriveILDelay_)) {
    maxVelCounterDrive_++;
  } else {
    maxVelCounterDrive_ = 0;
  }

  data_->interlocks_.maxVelocityDriveInterlock =
    data_->interlocks_.maxVelocityTrajInterlock &&
    maxVelCounterDrive_ >= maxVelDriveILDelay_;

  if (data_->interlocks_.maxVelocityDriveInterlock ||
      data_->interlocks_.maxVelocityTrajInterlock) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MON_MAX_VELOCITY_EXCEEDED,
                      ECMC_SEVERITY_NORMAL);
  }
  return 0;
}

int ecmcMonitor::checkCntrlMaxOutput() {
  if (!data_->status_.enabled || !data_->status_.enabledOld) {
    data_->interlocks_.cntrlOutputHLDriveInterlock = false;
    data_->interlocks_.cntrlOutputHLTrajInterlock  = false;
    return 0;
  }

  if (enableCntrlHLMon_ &&
      (std::abs(data_->status_.cntrlOutput) > cntrlOutputHL_)) {
    data_->interlocks_.cntrlOutputHLDriveInterlock = true;
    data_->interlocks_.cntrlOutputHLTrajInterlock  = true;
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MON_CNTRL_OUTPUT_EXCEED_LIMIT,
                      ECMC_SEVERITY_NORMAL);
  }
  return 0;
}

int ecmcMonitor::getEnableAlarmAtHardLimit() {
  return enableAlarmAtHardlimitBwd_ || enableAlarmAtHardlimitFwd_;
}

double ecmcMonitor::getSoftLimitBwd() {
  return data_->command_.softLimitBwd;
}

double ecmcMonitor::getSoftLimitFwd() {
  return data_->command_.softLimitFwd;
}

bool ecmcMonitor::getEnableSoftLimitBwd() {
  return data_->command_.enableSoftLimitBwd &&
         data_->command_.command != ECMC_CMD_HOMING;
}

bool ecmcMonitor::getEnableSoftLimitFwd() {
  return data_->command_.enableSoftLimitFwd &&
         data_->command_.command != ECMC_CMD_HOMING;
}

bool ecmcMonitor::getAtSoftLimitBwd() {
  return data_->command_.enableSoftLimitBwd &&
         data_->status_.currentPositionActual <= data_->command_.softLimitBwd;
}

bool ecmcMonitor::getAtSoftLimitFwd() {
  return data_->command_.enableSoftLimitFwd &&
         data_->status_.currentPositionActual >= data_->command_.softLimitFwd;
}

int ecmcMonitor::setVelDiffMaxDifference(double velo) {
  velDiffMaxDiff_ = std::abs(velo);
  return 0;
}

int ecmcMonitor::filterSwitches() {
  // Simple filtering of switches (average of last cycles)
  if (switchFilterCounter_ >= ECMC_MON_SWITCHES_FILTER_CYCLES) {
    switchFilterCounter_ = 0;
  }
  limitFwdFilterBuffer_[switchFilterCounter_] = data_->status_.limitFwd;
  limitBwdFilterBuffer_[switchFilterCounter_] = data_->status_.limitBwd;
  homeFilterBuffer_[switchFilterCounter_]     = data_->status_.homeSwitch;

  int limFwdSum  = 0;
  int limBwdSum  = 0;
  int limHomeSum = 0;

  for (int i = 0; i < ECMC_MON_SWITCHES_FILTER_CYCLES; i++) {
    limFwdSum  = limFwdSum + limitFwdFilterBuffer_[i];
    limBwdSum  = limBwdSum + limitBwdFilterBuffer_[i];
    limHomeSum = limHomeSum + homeFilterBuffer_[i];
  }
  data_->status_.limitFwdFiltered = limFwdSum >
                                    ECMC_MON_SWITCHES_FILTER_CYCLES / 2;
  data_->status_.limitBwdFiltered = limBwdSum >
                                    ECMC_MON_SWITCHES_FILTER_CYCLES / 2;
  data_->status_.homeSwitchFiltered = limHomeSum >
                                      ECMC_MON_SWITCHES_FILTER_CYCLES / 2;

  switchFilterCounter_++;
  return 0;
}

int ecmcMonitor::setHardwareInterlockPolarity(ecmcSwitchPolarity pol) {
  int errorCode = checkPolarity(pol);

  if (errorCode) {
    LOGERR("%s/%s:%d: Invalid polarity (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           errorCode);
    return errorCode;
  }

  if (hardwareInterlockPolarity_ != pol) {
    hardwareInterlockPolarity_ = pol;
  }
  return 0;
}

int ecmcMonitor::setAnalogInterlockPolarity(ecmcSwitchPolarity pol) {
  int errorCode = checkPolarity(pol);

  if (errorCode) {
    LOGERR("%s/%s:%d: Invalid polarity (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           errorCode);
    return errorCode;
  }

  if (analogPolarity_ != pol) {
    analogPolarity_ = pol;
  }
  return 0;
}

int ecmcMonitor::setPLCInterlock(bool ilock, plcInterlockTypes type) {
  switch (type) {
  case ECMC_PLC_INTERLOCK_DIR_BOTH:
    data_->interlocks_.plcInterlock = ilock;
    break;

  case ECMC_PLC_INTERLOCK_DIR_BWD:
    data_->interlocks_.plcInterlockBWD = ilock;
    break;

  case ECMC_PLC_INTERLOCK_DIR_FWD:
    data_->interlocks_.plcInterlockFWD = ilock;
    break;
  }
  return 0;
}

int ecmcMonitor::setHardLimitBwdPolarity(ecmcSwitchPolarity pol) {
  int errorCode = checkPolarity(pol);

  if (errorCode) {
    LOGERR("%s/%s:%d: Invalid polarity (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           errorCode);
    return errorCode;
  }

  lowLimPolarity_ = pol;
  return 0;
}

int ecmcMonitor::setHardLimitFwdPolarity(ecmcSwitchPolarity pol) {
  int errorCode = checkPolarity(pol);

  if (errorCode) {
    LOGERR("%s/%s:%d: Invalid polarity (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           errorCode);
    return errorCode;
  }

  highLimPolarity_ = pol;
  return 0;
}

int ecmcMonitor::setHomePolarity(ecmcSwitchPolarity pol) {
  int errorCode = checkPolarity(pol);

  if (errorCode) {
    LOGERR("%s/%s:%d: Invalid polarity (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           errorCode);
    return errorCode;
  }

  homePolarity_ = pol;
  return 0;
}

ecmcSwitchPolarity ecmcMonitor::getHardLimitBwdPolarity() {
  return lowLimPolarity_;
}

ecmcSwitchPolarity ecmcMonitor::getHardLimitFwdPolarity() {
  return highLimPolarity_;
}

ecmcSwitchPolarity ecmcMonitor::getHomePolarity() {
  return homePolarity_;
}

ecmcSwitchPolarity ecmcMonitor::getHardwareInterlockPolarity() {
  return hardwareInterlockPolarity_;
}

int ecmcMonitor::checkPolarity(ecmcSwitchPolarity pol) {
  if ((pol <  ECMC_POLARITY_NC) || (pol >  ECMC_POLARITY_NO)) {
    return ERROR_MON_POLARITY_OUT_OF_RANGE;
  }
  return 0;
}

int ecmcMonitor::setLatchAtLimit(bool latchOnLimit) {
  latchOnLimit_ = latchOnLimit;
  return 0;
}

int ecmcMonitor::getLatchAtLimit() {
  return latchOnLimit_;
}

int ecmcMonitor::setEnableSoftLimitAlarm(bool enable) {
  enableAlarmOnSofLimits_ = enable;
  return 0;
}

int ecmcMonitor::setEnableCheckEncsDiff(bool enable) {
  enableDiffEncsMon_ = enable;
  return 0;
}

int ecmcMonitor::setCtrlDeadband(double tol) {
  ctrlDeadbandTol_ = std::abs(tol);
  return 0;
}

int ecmcMonitor::setCtrlDeadbandTime(int cycles) {
  ctrlDeadbandTime_ = cycles;
  return 0;
}

void ecmcMonitor::setSafetyInterlock(int interlock) {
  data_->interlocks_.safetyInterlock = interlock > 0;
}

int ecmcMonitor::getSafetyInterlock() {
  return data_->interlocks_.safetyInterlock;
}

int ecmcMonitor::getSumInterlock() {
  return data_->interlocks_.driveSummaryInterlock ||
         data_->interlocks_.trajSummaryInterlockFWD ||
         data_->interlocks_.trajSummaryInterlockBWD;
}

void  ecmcMonitor::setStallTimeFactor(double timeFactor) {
  stallTimeFactor_ = timeFactor;
}

double  ecmcMonitor::getStallTimeFactor() {
  return stallTimeFactor_;
}

void ecmcMonitor::setEnableStallMon(bool enable) {
  enableStallMon_ = enable;
}

bool ecmcMonitor::getEnableStallMon() {
  return enableStallMon_;
}

void  ecmcMonitor::setStallMinTimeOut(double timeCycles) {
  stallMinTimeoutCycles_ = timeCycles;
}

int ecmcMonitor::setLimitSwitchFwdPLCOverride(bool overrideSwitch) {
  limitSwitchFwdPLCOverride_ = overrideSwitch;
  return 0;
}

int ecmcMonitor::setLimitSwitchBwdPLCOverride(bool overrideSwitch) {
  limitSwitchBwdPLCOverride_ = overrideSwitch;
  return 0;
}

void ecmcMonitor::setLimitSwitchFwdPLCOverrideValue(bool switchValue) {
  limitSwitchFwdPLCOverrideValue_ = switchValue;
}

void ecmcMonitor::setLimitSwitchBwdPLCOverrideValue(bool switchValue) {
  limitSwitchBwdPLCOverrideValue_ = switchValue;
}

int ecmcMonitor::setHomeSwitchPLCOverride(bool overrideSwitch) {
  homeSwitchPLCOverride_ = overrideSwitch;
  return 0;
}

void ecmcMonitor::setHomeSwitchPLCOverrideValue(bool switchValue) {
  homeSwitchPLCOverrideValue_ = switchValue;
}

void ecmcMonitor::setCtrlWithinDBExtTraj(bool within) {
  ctrlWinthinDBExternalTraj_ = within;
}
