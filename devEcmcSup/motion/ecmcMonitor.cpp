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
#include "ecmcErrorsList.h"

ecmcMonitor::ecmcMonitor(ecmcAxisData *axisData,
                         ecmcEncoder **encArray) : ecmcEcEntryLink(&(axisData->
                                                                     status_.
                                                                     errorCode),
                                                                   &(axisData->
                                                                     status_.
                                                                     warningCode))
{
  if (!axisData) {
    LOGERR("%s/%s:%d: ERROR: Axis data object is NULL.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    exit(EXIT_FAILURE);
  }
  data_ = axisData;
  setExternalPtrs(&(data_->status_.errorCode), &(data_->status_.warningCode));
  initVars();
  encArray_ = encArray;
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
  // Allow atleast 10 seconds before stall
  stallMinTimeoutCycles_     = 15 / data_->status_.sampleTime;
  // Allow stallMinTimeoutCycles_ + stallTimeFactor_*(last_move_time) befire stall
  stallTimeFactor_           = 10;
  enableStallMon_            = 1;

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
  velDiffTimeTraj_           = 100;
  velDiffTimeDrive_          = 100;
  velDiffMaxDiff_            = 0;
  latchOnLimit_              = 1;
  switchFilterCounter_       = 0;
  limitFwdFilterSum_         = 0;
  limitBwdFilterSum_         = 0;
  homeFilterSum_             = 0;
  memset(&limitFwdFilterBuffer_, 0, sizeof(limitFwdFilterBuffer_));
  memset(&limitBwdFilterBuffer_, 0, sizeof(limitBwdFilterBuffer_));
  memset(&homeFilterBuffer_,     0, sizeof(homeFilterBuffer_));
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
  analogRawValue_            = 0;
  enableAnalogInterlock_     = 0;
  analogPolarity_            = ECMC_POLARITY_NC; // Higher value than analogRawLimit_ is bad
  limitSwitchFwdPLCOverride_ = false;
  limitSwitchBwdPLCOverride_ = false;
  limitSwitchFwdPLCOverrideValue_ = false;
  limitSwitchBwdPLCOverrideValue_ = false;
  homeSwitchPLCOverride_          = false;
  homeSwitchPLCOverrideValue_     = false;
  enableHomeSensor_               = false;
  axisIsWithinCtrlDBExtTraj_      = false;
  stopAtAnyLimit_                 = false;
  softLimitReenablePending_       = false;
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
      data_->status_.statusWord_.enabled) {
    data_->interlocks_.axisErrorStateInterlock = 1;
  }
  else {
    data_->interlocks_.axisErrorStateInterlock = 0;
  }

  data_->refreshInterlocks();
}

bool ecmcMonitor::getAtTarget() {
  return data_->status_.statusWord_.attarget;
}

bool ecmcMonitor::getCtrlInDeadband() {
  return data_->status_.ctrlWithinDeadband;
}

bool ecmcMonitor::getHardLimitFwd() {
  return data_->status_.statusWord_.limitfwd;
}

bool ecmcMonitor::getHardLimitBwd() {
  return data_->status_.statusWord_.limitbwd;
}

int ecmcMonitor::setAtTargetTol(double tol) {
  if (tol < 0) {
    LOGERR("%s/%s:%d: ERROR: At-target tolerance %lf is invalid; expected >= 0 (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, tol, ERROR_MON_TOL_OUT_OF_RANGE);
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
    LOGERR("%s/%s:%d: ERROR: At-target time %d is invalid; expected >= 0 (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, time, ERROR_MON_TIME_OUT_OF_RANGE);
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
    LOGERR("%s/%s:%d: ERROR: Position-lag tolerance %lf is invalid; expected >= 0 (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, tol, ERROR_MON_TOL_OUT_OF_RANGE);
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
    LOGERR("%s/%s:%d: ERROR: Velocity-difference trajectory time %d is invalid; expected >= 0 (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, time, ERROR_MON_TIME_OUT_OF_RANGE);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_MON_TIME_OUT_OF_RANGE);
  }

  velDiffTimeTraj_ = time;
  return 0;
}

int ecmcMonitor::getVelDiffTimeTraj() {
  return velDiffTimeTraj_;
}

int ecmcMonitor::setVelDiffTimeDrive(int time) {
  if (time < 0) {
    LOGERR("%s/%s:%d: ERROR: Velocity-difference drive time %d is invalid; expected >= 0 (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, time, ERROR_MON_TIME_OUT_OF_RANGE);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_MON_TIME_OUT_OF_RANGE);
  }

  velDiffTimeDrive_ = time;
  return 0;
}

int ecmcMonitor::getVelDiffTimeDrive() {
  return velDiffTimeDrive_;
}

int ecmcMonitor::setPosLagTime(int time) {
  if (time < 0) {
    LOGERR("%s/%s:%d: ERROR: Position-lag time %d is invalid; expected >= 0 (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, time, ERROR_MON_TIME_OUT_OF_RANGE);
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
  return data_->status_.statusWord_.homeswitch;
}

void ecmcMonitor::readEntries() {
  uint64_t tempRaw = 0;
  int errorCode = 0;
  auto &statusWord = data_->status_.statusWord_;
  auto &interlocks = data_->interlocks_;
  // Hard limit BWD
  if(!limitSwitchBwdPLCOverride_) {
    errorCode = readEcEntryValue(ECMC_MON_ENTRY_INDEX_LOWLIM, &tempRaw);
  
    if (errorCode) {
      if (statusWord.instartup && (errorCode == ERROR_EC_ENTRY_EC_DOMAIN_ERROR)) {
        return;
      }
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
  
    statusWord.limitbwd = tempRaw > 0;

  } else {
    // Override limit switch from PLC code (when logic is needed)
    statusWord.limitbwd = limitSwitchBwdPLCOverrideValue_;
  }

  // Hard limit FWD
  tempRaw = 0;
  if(!limitSwitchFwdPLCOverride_) {
    errorCode = readEcEntryValue(ECMC_MON_ENTRY_INDEX_HIGHLIM, &tempRaw);
  
    if (errorCode) {
      if (statusWord.instartup && (errorCode == ERROR_EC_ENTRY_EC_DOMAIN_ERROR)) {
        return;
      }
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
  
    statusWord.limitfwd = tempRaw > 0;

  } else {
      // Override limit switch from PLC code (when logic is needed)
      statusWord.limitfwd = limitSwitchFwdPLCOverrideValue_;
  }

  // Home
  tempRaw = 0;
  if(!homeSwitchPLCOverride_ && enableHomeSensor_) {
    errorCode = readEcEntryValue(ECMC_MON_ENTRY_INDEX_HOMESENSOR, &tempRaw);
  
    if (errorCode) {
      if (statusWord.instartup && (errorCode == ERROR_EC_ENTRY_EC_DOMAIN_ERROR)) {
        return;
      }
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
  
    statusWord.homeswitch = tempRaw > 0;
  } else {
    // Override limit switch from PLC code (when logic is needed)
    statusWord.homeswitch = homeSwitchPLCOverrideValue_;
  }

  // Refresh filtered switches
  filterSwitches();

  if (enableHardwareInterlock_) {
    tempRaw   = 0;
    errorCode = readEcEntryValue(ECMC_MON_ENTRY_INDEX_EXTINTERLOCK, &tempRaw);

    if (errorCode) {
      if (statusWord.instartup && (errorCode == ERROR_EC_ENTRY_EC_DOMAIN_ERROR)) {
        return;
      }
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 errorCode,
                 ECMC_SEVERITY_NORMAL);
      return;
    }

    switch (hardwareInterlockPolarity_) {
    case ECMC_POLARITY_NC:
      interlocks.hardwareInterlock = tempRaw == 0;
      break;

    case ECMC_POLARITY_NO:
      interlocks.hardwareInterlock = tempRaw > 0;
      break;
    }
  }

  if (enableAnalogInterlock_) {
    double tempDouble = 0;
    errorCode = readEcEntryValueDouble(ECMC_MON_ENTRY_INDEX_ANALOG, &tempDouble);

    if (errorCode) {
      if (statusWord.instartup && (errorCode == ERROR_EC_ENTRY_EC_DOMAIN_ERROR)) {
        return;
      }
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 errorCode,
                 ECMC_SEVERITY_NORMAL);
      return;
    }
    analogRawValue_ = tempDouble;

    switch (analogPolarity_) {
    case ECMC_POLARITY_NC:
      interlocks.analogInterlock = tempDouble > analogRawLimit_;
      break;

    case ECMC_POLARITY_NO:
      interlocks.analogInterlock = tempDouble < analogRawLimit_;
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
    LOGERR("%s/%s:%d: ERROR: Max-velocity drive time %d is invalid; expected >= 0 (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, time, ERROR_MON_TIME_OUT_OF_RANGE);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_MON_TIME_OUT_OF_RANGE);
  }

  maxVelDriveILDelay_ = time;
  return 0;
}

int ecmcMonitor::getMaxVelDriveTime() {
  return maxVelDriveILDelay_;
}

int ecmcMonitor::setMaxVelTrajTime(int time) {
  if (time < 0) {
    LOGERR("%s/%s:%d: ERROR: Max-velocity trajectory time %d is invalid; expected >= 0 (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, time, ERROR_MON_TIME_OUT_OF_RANGE);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_MON_TIME_OUT_OF_RANGE);
  }

  maxVelTrajILDelay_ = time;
  return 0;
}

int ecmcMonitor::getMaxVelTrajTime() {
  return maxVelTrajILDelay_;
}

int ecmcMonitor::reset() {
  // data_->status_.statusWord_.attarget = false;
  // atTargetCounter_        = 0;
  const bool keepCurrentMoveStallTiming =
    data_ && data_->status_.statusWord_.localBusy;
  lagMonCounter_       = 0;
  maxVelCounterDrive_  = 0;
  maxVelCounterTraj_   = 0;
  maxStallCounter_     = 0;
  if (!keepCurrentMoveStallTiming) {
    stallLastMotionCmdCycles_ = 0;
  }
  stallCheckAtTargetAtCycle_ = 0;
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

double ecmcMonitor::getVelDiffMaxDifference() {
  return velDiffMaxDiff_;
}

int ecmcMonitor::setEnableSoftLimitBwd(bool enable) {  
  const bool oldEnable = data_->control_.controlWord_.enableSoftLimitBwd;
  data_->control_.controlWord_.enableSoftLimitBwd = enable;
  if (enable) {
    const int error = validateSoftLimitConfigForEnabledLimits();
    if (error) {
      data_->control_.controlWord_.enableSoftLimitBwd = oldEnable;
      return error;
    }
  }
  data_->status_.statusWord_.softlimbwdena = enable;
  softLimitReenablePending_ = false;
  refreshSoftLimitAsynParams();
  return 0;
}

int ecmcMonitor::setEnableSoftLimitFwd(bool enable) {
  const bool oldEnable = data_->control_.controlWord_.enableSoftLimitFwd;
  data_->control_.controlWord_.enableSoftLimitFwd = enable;
  if (enable) {
    const int error = validateSoftLimitConfigForEnabledLimits();
    if (error) {
      data_->control_.controlWord_.enableSoftLimitFwd = oldEnable;
      return error;
    }
  }
  data_->status_.statusWord_.softlimfwdena = enable;
  softLimitReenablePending_ = false;
  refreshSoftLimitAsynParams();
  return 0;
}

void ecmcMonitor::refreshSoftLimitAsynParams() {
  data_->axAsynParams_[ECMC_ASYN_AX_STATUS_ID]->refreshParamRT(1);
  data_->axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]->refreshParamRT(1);
}

bool ecmcMonitor::hasInvalidSoftLimitRange() const {
  return data_->control_.softLimitBwd >= data_->control_.softLimitFwd;
}

bool ecmcMonitor::hasInvalidSoftLimitConfigForEnabledLimits() const {
  return data_->control_.controlWord_.enableSoftLimitBwd &&
         data_->control_.controlWord_.enableSoftLimitFwd &&
         hasInvalidSoftLimitRange();
}

int ecmcMonitor::disableSoftLimitsForZeroRange() {
  data_->control_.controlWord_.enableSoftLimitBwd = false;
  data_->control_.controlWord_.enableSoftLimitFwd = false;
  data_->status_.statusWord_.softlimbwdena = false;
  data_->status_.statusWord_.softlimfwdena = false;
  softLimitReenablePending_ = false;
  refreshSoftLimitAsynParams();
  return 0;
}

int ecmcMonitor::setSoftLimitBwd(double limit) {
  data_->control_.softLimitBwd = limit;
  const bool bothEnabled = data_->control_.controlWord_.enableSoftLimitBwd &&
                           data_->control_.controlWord_.enableSoftLimitFwd;
  const bool bothDisabled = !data_->control_.controlWord_.enableSoftLimitBwd &&
                            !data_->control_.controlWord_.enableSoftLimitFwd;
  if (data_->control_.softLimitBwd == 0.0 &&
      data_->control_.softLimitFwd == 0.0) {
    return disableSoftLimitsForZeroRange();
  }
  if (hasInvalidSoftLimitRange() && (bothEnabled || bothDisabled)) {
    data_->control_.controlWord_.enableSoftLimitBwd = false;
    data_->control_.controlWord_.enableSoftLimitFwd = false;
    data_->status_.statusWord_.softlimbwdena = false;
    data_->status_.statusWord_.softlimfwdena = false;
    softLimitReenablePending_ = true;
  } else if (softLimitReenablePending_) {
    data_->control_.controlWord_.enableSoftLimitBwd = true;
    data_->control_.controlWord_.enableSoftLimitFwd = true;
    data_->status_.statusWord_.softlimbwdena = true;
    data_->status_.statusWord_.softlimfwdena = true;
    softLimitReenablePending_ = false;
  }
  refreshSoftLimitAsynParams();
  return 0;
}

int ecmcMonitor::setSoftLimitFwd(double limit) {
  data_->control_.softLimitFwd = limit;
  const bool bothEnabled = data_->control_.controlWord_.enableSoftLimitBwd &&
                           data_->control_.controlWord_.enableSoftLimitFwd;
  const bool bothDisabled = !data_->control_.controlWord_.enableSoftLimitBwd &&
                            !data_->control_.controlWord_.enableSoftLimitFwd;
  if (data_->control_.softLimitBwd == 0.0 &&
      data_->control_.softLimitFwd == 0.0) {
    return disableSoftLimitsForZeroRange();
  }
  if (hasInvalidSoftLimitRange() && (bothEnabled || bothDisabled)) {
    data_->control_.controlWord_.enableSoftLimitBwd = false;
    data_->control_.controlWord_.enableSoftLimitFwd = false;
    data_->status_.statusWord_.softlimbwdena = false;
    data_->status_.statusWord_.softlimfwdena = false;
    softLimitReenablePending_ = true;
  } else if (softLimitReenablePending_) {
    data_->control_.controlWord_.enableSoftLimitBwd = true;
    data_->control_.controlWord_.enableSoftLimitFwd = true;
    data_->status_.statusWord_.softlimbwdena = true;
    data_->status_.statusWord_.softlimfwdena = true;
    softLimitReenablePending_ = false;
  }
  refreshSoftLimitAsynParams();
  return 0;
}

int ecmcMonitor::validateSoftLimitConfigForEnabledLimits() {
  if (hasInvalidSoftLimitConfigForEnabledLimits()) {
    LOGERR("%s/%s:%d: ERROR: Invalid soft-limit configuration for axis %d. "
           "Backward soft limit (%lf) must be below forward soft limit (%lf).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_->status_.axisId,
           data_->control_.softLimitBwd,
           data_->control_.softLimitFwd);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MON_TOL_OUT_OF_RANGE);
  }
  return 0;
}

int ecmcMonitor::checkLimits() {
  auto &interlocks = data_->interlocks_;
  const auto &status = data_->status_;
  const auto &control = data_->control_;
  const auto &statusWord = status.statusWord_;
  const auto &statusOldWord = data_->statusOld_.statusWord_;
  int warningId = getWarningID();
  const bool limitBwdOk = statusWord.limitbwd;
  const bool limitFwdOk = statusWord.limitfwd;
  const bool moving = statusWord.moving;
  const bool axisEnabled = statusWord.enabled && statusOldWord.enabled;
  const bool isHomingCmd = status.command == ECMC_CMD_HOMING;

  if (statusWord.instartup) {
    interlocks.bothLimitsLowInterlock = false;
    interlocks.bwdLimitInterlock = false;
    interlocks.fwdLimitInterlock = false;
    return 0;
  }

  // Both limit switches
  interlocks.bothLimitsLowInterlock = !limitBwdOk && !limitFwdOk;

  // Stop at any limit (reuse same interlock as both limit)
  if (stopAtAnyLimit_) {
    interlocks.bothLimitsLowInterlock = !limitBwdOk || !limitFwdOk;
  }

  if (interlocks.bothLimitsLowInterlock) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MON_BOTH_LIMIT_INTERLOCK,
                      ECMC_SEVERITY_EMERGENCY);
  }

  // Bwd limit switch
  if (!limitBwdOk) {
    interlocks.bwdLimitInterlock = true;

    if (enableAlarmAtHardlimitBwd_) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_MON_HARD_LIMIT_BWD_INTERLOCK,
                        ECMC_SEVERITY_NORMAL);
    }
    if (warningId != WARNING_MON_HARD_LIMIT_BWD_INTERLOCK) {
      setWarningID(WARNING_MON_HARD_LIMIT_BWD_INTERLOCK);
      warningId = WARNING_MON_HARD_LIMIT_BWD_INTERLOCK;
    }
  } else {
    if (latchOnLimit_) {
      if (!moving || (status.currentVelocityActual > 0)) {
        interlocks.bwdLimitInterlock = false;
      }
    } else {
      // Auto reset warning
      if (warningId == WARNING_MON_HARD_LIMIT_BWD_INTERLOCK) {
        setWarningID(0);
        warningId = 0;
      }
      interlocks.bwdLimitInterlock = false;
    }
  }

  // Fwd limit switch
  if (!limitFwdOk) {
    interlocks.fwdLimitInterlock = true;

    if (enableAlarmAtHardlimitFwd_) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_MON_HARD_LIMIT_FWD_INTERLOCK,
                        ECMC_SEVERITY_NORMAL);
    }
    if (warningId != WARNING_MON_HARD_LIMIT_FWD_INTERLOCK) {
      setWarningID(WARNING_MON_HARD_LIMIT_FWD_INTERLOCK);
      warningId = WARNING_MON_HARD_LIMIT_FWD_INTERLOCK;
    }
  } else {
    if (latchOnLimit_) {
      if (!moving || (status.currentVelocityActual < 0)) {
        interlocks.fwdLimitInterlock = false;
      }
    } else {
      // Auto reset warning
      if (warningId == WARNING_MON_HARD_LIMIT_FWD_INTERLOCK) {
        setWarningID(0);
        warningId = 0;
      }
      interlocks.fwdLimitInterlock = false;
    }
  }

  const bool checkSoftLimits = axisEnabled && !isHomingCmd;
  const bool useVelocityDirection =
    (status.command == ECMC_CMD_MOVEVEL) ||
    (status.command == ECMC_CMD_JOG) ||
    (status.command == ECMC_CMD_MOVEPVTABS) ||
    (statusWord.trajsource != ECMC_DATA_SOURCE_INTERNAL);
  const bool movingBwd =
    (status.currentTargetPosition < status.currentPositionSetpoint) ||
    (useVelocityDirection && (status.currentVelocitySetpoint < 0));
  const bool movingFwd =
    (status.currentTargetPosition > status.currentPositionSetpoint) ||
    (useVelocityDirection && (status.currentVelocitySetpoint > 0));
  const bool movingAwayFromBwdLimit = movingFwd && !movingBwd;
  const bool movingAwayFromFwdLimit = movingBwd && !movingFwd;

  if (checkSoftLimits && statusWord.softlimbwdena) {
    const bool virtSoftlimitBwd =
      !movingAwayFromBwdLimit &&
      ((status.currentPositionSetpoint < control.softLimitBwd) ||
       (status.currentTargetPosition < control.softLimitBwd));
    if (virtSoftlimitBwd) {
      interlocks.bwdSoftLimitInterlock = true;
      if (warningId != WARNING_MON_SOFT_LIMIT_BWD_INTERLOCK) {
        setWarningID(WARNING_MON_SOFT_LIMIT_BWD_INTERLOCK);
        warningId = WARNING_MON_SOFT_LIMIT_BWD_INTERLOCK;
      }
    } else {
      if (warningId == WARNING_MON_SOFT_LIMIT_BWD_INTERLOCK) {
        setWarningID(0);
        warningId = 0;
      }
      interlocks.bwdSoftLimitInterlock = false;
    }
  } else {
    if (warningId == WARNING_MON_SOFT_LIMIT_BWD_INTERLOCK) {
      setWarningID(0);
      warningId = 0;
    }
    interlocks.bwdSoftLimitInterlock = false;
  }

  if (checkSoftLimits && statusWord.softlimfwdena) {
    const bool virtSoftlimitFwd =
      !movingAwayFromFwdLimit &&
      ((status.currentPositionSetpoint > control.softLimitFwd) ||
       (status.currentTargetPosition > control.softLimitFwd));
    if (virtSoftlimitFwd) {
      interlocks.fwdSoftLimitInterlock = true;
      if (warningId != WARNING_MON_SOFT_LIMIT_FWD_INTERLOCK) {
        setWarningID(WARNING_MON_SOFT_LIMIT_FWD_INTERLOCK);
        warningId = WARNING_MON_SOFT_LIMIT_FWD_INTERLOCK;
      }
    } else {
      if (warningId == WARNING_MON_SOFT_LIMIT_FWD_INTERLOCK) {
        setWarningID(0);
        warningId = 0;
      }
      interlocks.fwdSoftLimitInterlock = false;
    }
  } else {
    if (warningId == WARNING_MON_SOFT_LIMIT_FWD_INTERLOCK) {
      setWarningID(0);
      warningId = 0;
    }
    interlocks.fwdSoftLimitInterlock = false;
  }
  return 0;
}

int ecmcMonitor::checkAtTarget() {
  bool atTarget      = false;
  bool ctrlWithinTol = false;
  auto &status = data_->status_;
  auto &statusWord = status.statusWord_;
  const bool internalTrajSource = statusWord.trajsource == 0;

  if (enableAtTargetMon_) {
    const double targetSetDiffAbs = std::abs(status.currentTargetPositionModulo -
                                             status.currentPositionSetpoint);
    const double cntrlErrAbs      = std::abs(status.cntrlError);
    /*if (std::abs(data_->status_.currentTargetPosition -
                 data_->status_.currentPositionActual) < atTargetTol_) {*/
    if (targetSetDiffAbs < atTargetTol_ && !statusWord.localBusy) {
      if (cntrlErrAbs < atTargetTol_) {
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
      if (cntrlErrAbs < ctrlDeadbandTol_) {
        if (ctrlDeadbandCounter_ <= ctrlDeadbandTime_) {
          ctrlDeadbandCounter_++;
        }

        if (ctrlDeadbandCounter_ > ctrlDeadbandTime_) {
          ctrlWithinTol = true;
        }
      } else {
        ctrlDeadbandCounter_ = 0;
      }
    } else {
      atTargetCounter_ = 0;
      ctrlDeadbandCounter_ = 0;
    }
  } else {
    atTarget = false;
    atTargetCounter_ = 0;
    ctrlDeadbandCounter_ = 0;
  }

  statusWord.attarget = atTarget;
  
  if (internalTrajSource) {
    status.ctrlWithinDeadband = ctrlWithinTol;
  } else {
    // external source used. No way for axis to know when atTarget/reduce torque. Make possible to write from PLC
    status.ctrlWithinDeadband = axisIsWithinCtrlDBExtTraj_;
  }
  
  return 0;
}

// Only enabled when also atTarget monitoring is enabled
int  ecmcMonitor::checkStall() {
  const auto &statusWord = data_->status_.statusWord_;
  const auto &statusOldWord = data_->statusOld_.statusWord_;
  const bool dbgPrint = data_->control_.controlWord_.enableDbgPrintout;

  // Do only check for stall when not busy (traj finished)
  if (!enableAtTargetMon_ || !enableStallMon_ || !statusWord.enabled ||
      (statusWord.trajsource != 0) || (data_->status_.command == ECMC_CMD_MOVEPVTABS)) {
    data_->interlocks_.stallInterlock = false;
    maxStallCounter_ = 0;
    return 0;
  }
  
  // Measure time of last move, busy high to busy low.
  if (statusWord.localBusy) {
    // Stall monitoring only applies after the active move has finished.
    // Clear any post-move timeout state so a retriggered or retargeted move
    // cannot inherit elapsed stall time from an earlier settle window.
    maxStallCounter_ = 0;
    stallCheckAtTargetAtCycle_ = 0;
    stallLastMotionCmdCycles_++;
    if (!statusOldWord.localBusy) {
      stallLastMotionCmdCycles_ = 0;
    }
    return 0;
  } else {
    if (statusOldWord.localBusy) {
      stallCheckAtTargetAtCycle_ = stallLastMotionCmdCycles_ * stallTimeFactor_;
      // Ensure a minimum time window 
      if (stallCheckAtTargetAtCycle_ < stallMinTimeoutCycles_) {
        stallCheckAtTargetAtCycle_ = stallMinTimeoutCycles_;
      }
      if (dbgPrint) {
        LOGINFO("INFO: Axis[%d]: Stall check scheduled after %" PRIu64 " cycles (factor=%lf, min=%lf).\n",
                data_->status_.axisId,
                stallCheckAtTargetAtCycle_,
                stallTimeFactor_,
                stallMinTimeoutCycles_);
      }
    }
  }

  maxStallCounter_++;

  if (statusWord.attarget) {
    data_->interlocks_.stallInterlock = false;
    if (!statusOldWord.attarget) {
      if (dbgPrint) {
        LOGINFO("INFO: Axis[%d]: Stall check cleared: axis reached target.\n",
                data_->status_.axisId);
      }
    }
    stallLastMotionCmdCycles_ = 0;
    maxStallCounter_ = 0;
    return 0;
  }

  if ((maxStallCounter_ > stallCheckAtTargetAtCycle_) && 
     (stallCheckAtTargetAtCycle_ > 0)) {
    if (dbgPrint) {
      LOGINFO("INFO: Axis[%d]: Stall detected.\n", data_->status_.axisId);
    }
    stallLastMotionCmdCycles_ = 0;  
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
  auto &interlocks = data_->interlocks_;

  if (!enableLagMon_ || !data_->control_.controlWord_.enableCmd) {
    lagMonCounter_ = 0;
    interlocks.lagDriveInterlock = false;
    interlocks.lagTrajInterlock  = false;
    return 0;
  }

  const int posLagDriveTime = posLagTime_ * 2;
  const bool axisEnabled = data_->status_.statusWord_.enabled &&
                           data_->statusOld_.statusWord_.enabled;
  const double cntrlErrAbs = std::abs(data_->status_.cntrlError);

  if ((cntrlErrAbs > posLagTol_) && axisEnabled) {
    if (lagMonCounter_ <= posLagDriveTime) {
      lagMonCounter_++;
    }

    if (lagMonCounter_ > posLagTime_) {
      lagErrorTraj = true;
    }

    // interlock the drive in twice the time..
    if (lagMonCounter_ >= posLagDriveTime) {
      lagErrorDrive = true;
    }
  } else {
    lagMonCounter_ = 0;
  }

  interlocks.lagDriveInterlock = lagErrorDrive;
  interlocks.lagTrajInterlock  = lagErrorTraj;

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

  if (!data_->control_.controlWord_.enableCmd) {
    return 0;
  }

  // Only one encoder configured
  if ((data_->status_.encoderCount <= 1) || !enableDiffEncsMon_) {
    return 0;
  }

  const int primaryEncIndex = data_->control_.primaryEncIndex;
  const int encoderCount = data_->status_.encoderCount;
  const double moduloRange = data_->control_.moduloRange;
  ecmcEncoder *primaryEnc = encArray_[primaryEncIndex];

  // Do not check if prim enc not homed
  if (!primaryEnc->getHomed()) {
    return 0;
  }

  // Multiple encoders configured so check pos diff vs primary
  double maxDiff       = 0;
  bool   encDiffILock  = false;
  double primEncActPos = primaryEnc->getActPos();

  for (int i = 0; i < encoderCount; i++) {
    auto * const enc = encArray_[i];
    // Do not check prim encoder vs itself or if this encoder is not homed
    if ((i == primaryEncIndex) || !enc->getHomed()) {
      continue;
    }

    maxDiff = enc->getMaxPosDiffToPrimEnc();

    // disable functionality if getMaxPosDiffToPrimEnc() == 0
    if (maxDiff == 0) {
      continue;
    }

    double diff = ecmcMotionUtils::getPosErrorModAbs(primEncActPos,
                                                     enc->getActPos(),
                                                     moduloRange);

    if (diff > maxDiff) {
      encDiffILock = true;
      break;  // No need to keep scanning once interlock is active.
    }
  }

  data_->interlocks_.encDiffInterlock = encDiffILock;

  return 0;
}

int ecmcMonitor::checkVelocityDiff() {
  if (!enableVelocityDiffMon_ || !data_->status_.statusWord_.enabled||
      !data_->statusOld_.statusWord_.enabled) {
    velocityDiffCounter_ = 0;
    return 0;
  }

  bool velocityDiffErrorDrive = false;
  bool velocityDiffErrorTraj  = false;
  const double velocityDiffAbs = std::abs(data_->status_.cntrlOutput -
                                          data_->status_.currentVelocityActual);
  if (velocityDiffAbs > velDiffMaxDiff_) {
    velocityDiffCounter_++;
  } else {
    velocityDiffCounter_ = 0;
    return 0;
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
  if (!data_->control_.controlWord_.enableCmd || !data_->status_.statusWord_.enabled ||
      !data_->statusOld_.statusWord_.enabled) {
    data_->interlocks_.maxVelocityTrajInterlock  = false;
    data_->interlocks_.maxVelocityDriveInterlock = false;
    maxVelCounterTraj_                           = 0;
    maxVelCounterDrive_                          = 0;
    return 0;
  }

  if (enableMaxVelMon_) {
    const double currentVelocityActualAbs = std::abs(data_->status_.currentVelocityActual);
    const double currentVelocitySetAbs    = std::abs(data_->status_.currentVelocitySetpoint);
    if ((currentVelocityActualAbs > maxVel_) || (currentVelocitySetAbs > maxVel_)) {
      if (maxVelCounterTraj_ <= maxVelTrajILDelay_) {
        maxVelCounterTraj_++;
      }
    } else {
      maxVelCounterTraj_ = 0;
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
  if (!data_->status_.statusWord_.enabled || !data_->statusOld_.statusWord_.enabled) {
    data_->interlocks_.cntrlOutputHLDriveInterlock = false;
    data_->interlocks_.cntrlOutputHLTrajInterlock  = false;
    return 0;
  }

  if (enableCntrlHLMon_ && (std::abs(data_->status_.cntrlOutput) > cntrlOutputHL_)) {
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
  return data_->control_.softLimitBwd;
}

double ecmcMonitor::getSoftLimitFwd() {
  return data_->control_.softLimitFwd;
}

bool ecmcMonitor::getEnableSoftLimitBwd() {
  return data_->control_.controlWord_.enableSoftLimitBwd &&
         data_->control_.command != ECMC_CMD_HOMING;
}

bool ecmcMonitor::getEnableSoftLimitFwd() {
  return data_->control_.controlWord_.enableSoftLimitFwd &&
         data_->control_.command != ECMC_CMD_HOMING;
}

bool ecmcMonitor::getAtSoftLimitBwd() {
  return data_->control_.controlWord_.enableSoftLimitBwd &&
         data_->status_.currentPositionActual <= data_->control_.softLimitBwd;
}

bool ecmcMonitor::getAtSoftLimitFwd() {
  return data_->control_.controlWord_.enableSoftLimitFwd &&
         data_->status_.currentPositionActual >= data_->control_.softLimitFwd;
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

  const int idx = switchFilterCounter_;
  auto &statusWord = data_->status_.statusWord_;
  const bool limitFwd = statusWord.limitfwd;
  const bool limitBwd = statusWord.limitbwd;
  const bool homeSw = statusWord.homeswitch;

  // Rolling sum update: remove overwritten sample and add new sample.
  limitFwdFilterSum_ -= limitFwdFilterBuffer_[idx];
  limitBwdFilterSum_ -= limitBwdFilterBuffer_[idx];
  homeFilterSum_ -= homeFilterBuffer_[idx];

  limitFwdFilterBuffer_[idx] = limitFwd;
  limitBwdFilterBuffer_[idx] = limitBwd;
  homeFilterBuffer_[idx] = homeSw;

  limitFwdFilterSum_ += limitFwd;
  limitBwdFilterSum_ += limitBwd;
  homeFilterSum_ += homeSw;

  const int threshold = ECMC_MON_SWITCHES_FILTER_CYCLES / 2;
  data_->status_.limitFwdFiltered = limitFwdFilterSum_ > threshold;
  data_->status_.limitBwdFiltered = limitBwdFilterSum_ > threshold;
  data_->status_.homeSwitchFiltered = homeFilterSum_ > threshold;

  switchFilterCounter_ = idx + 1;
  return 0;
}

int ecmcMonitor::setHardwareInterlockPolarity(ecmcSwitchPolarity pol) {
  int errorCode = checkPolarity(pol);

  if (errorCode) {
    LOGERR("%s/%s:%d: ERROR: Hardware interlock polarity is invalid (0x%x).\n",
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
    LOGERR("%s/%s:%d: ERROR: Analog interlock polarity is invalid (0x%x).\n",
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
    LOGERR("%s/%s:%d: ERROR: Backward hard-limit polarity is invalid (0x%x).\n",
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
    LOGERR("%s/%s:%d: ERROR: Forward hard-limit polarity is invalid (0x%x).\n",
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
    LOGERR("%s/%s:%d: ERROR: Home switch polarity is invalid (0x%x).\n",
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

ecmcSwitchPolarity ecmcMonitor::getAnalogInterlockPolarity() {
  return analogPolarity_;
}

double ecmcMonitor::getAnalogRawLimit() {
  return analogRawLimit_;
}

double ecmcMonitor::getAnalogRawValue() {
  return analogRawValue_;
}

bool ecmcMonitor::getEnableAnalogInterlock() {
  return enableAnalogInterlock_;
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

int ecmcMonitor::setStopAtAnyLimit(bool stop) {
  stopAtAnyLimit_ = stop;
  return 0;
}

bool ecmcMonitor::getStopAtAnyLimit() {
  return stopAtAnyLimit_;
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

double ecmcMonitor::getCtrlDeadband() {
  return ctrlDeadbandTol_;
}

int ecmcMonitor::getCtrlDeadbandTime() {
  return ctrlDeadbandTime_;
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

double ecmcMonitor::getStallMinTimeOut() {
  return stallMinTimeoutCycles_;
}

double ecmcMonitor::getStallTime() {
  return maxStallCounter_;
}

double ecmcMonitor::getStallAtTime() {
  return stallCheckAtTargetAtCycle_;
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

void ecmcMonitor::setAxisIsWithinCtrlDBExtTraj(bool within) {
  axisIsWithinCtrlDBExtTraj_ = within;
}

bool ecmcMonitor::getAxisIsWithinCtrlDB() {
  return data_->status_.ctrlWithinDeadband;
}
