/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcDriveBase.cpp
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcDriveBase.h"
#include "../main/ecmcErrorsList.h"

ecmcDriveBase::ecmcDriveBase(ecmcAsynPortDriver *asynPortDriver, 
                             ecmcAxisData *axisData) {
  initVars();
  data_ = axisData;
  asynPortDriver_ = asynPortDriver;

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }

  initAsyn();

  stateMachineTimeoutCycles_ = ERROR_DRV_STATE_MACHINE_TIME_OUT_TIME / data_->sampleTime_;
}

void ecmcDriveBase::initVars() {
  errorReset();
  scale_                     = 0;
  scaleNum_                  = 0;
  scaleDenom_                = 0;
  velSet_                    = 0;
  enableBrake_               = 0;
  enableReduceTorque_        = 0;
  controlWord_               = 0;
  statusWord_                = 0;
  brakeOpenDelayTime_        = 0;
  brakeCloseAheadTime_       = 0;
  brakeOutputCmd_            = 0;
  brakeOutputCmdOld_         = 0;
  brakeState_                = ECMC_BRAKE_CLOSED;
  brakeCounter_              = 0;
  enableAmpCmd_              = false;
  enableAmpCmdOld_           = false;
  reduceTorqueOutputCmd_     = false;
  reduceTorqueOutputCmdOld_  = false;
  enableCmdOld_              = false;
  asynPortDriver_            = NULL;
  asynControlWd_             = NULL;
  asynStatusWd_              = NULL;
  cspPosSet_                 = 0;
  cspRawActPos_              = 0;
  cspActPos_                 = 0;
  cspRawPosOffset_           = 0;
  hwReset_                   = 0;
  hwErrorAlarm0_             = 0;
  hwErrorAlarm0Old_          = 0;
  hwErrorAlarm1_             = 0;
  hwErrorAlarm1Old_          = 0;
  hwErrorAlarm2_             = 0;
  hwErrorAlarm2Old_          = 0;
  hwWarning_                 = 0;
  hwWarningOld_              = false;
  hwResetDefined_            = false;
  hwErrorAlarm0Defined_      = false;
  hwErrorAlarm1Defined_      = false;
  hwErrorAlarm2Defined_      = false;
  hwWarningDefined_          = false;
  stateMachineTimeoutCycles_ = 0;
  cycleCounterBase_          = 0;
  localEnabledOld_           = 0;  
}

ecmcDriveBase::~ecmcDriveBase()
{}

int ecmcDriveBase::setVelSet(double vel) {
  if (!driveInterlocksOK()) {
    velSet_                                   = 0;
    data_->status_.currentVelocitySetpointRaw = 0;
    return 0;
  }
  velSet_                                   = vel;
  data_->status_.currentVelocitySetpointRaw = velSet_ / scale_;
  return 0;
}

int ecmcDriveBase::setCspPosSet(double posEng) {

  cspPosSet_ = posEng; // Engineering unit
  
  if (!driveInterlocksOK()) {        
    return 0;
  }

  if (data_->status_.enabled && data_->command_.enable) {
    data_->status_.currentPositionSetpointRaw = cspPosSet_ / scale_ + cspRawPosOffset_;
  }
  else {
    data_->status_.currentPositionSetpointRaw = cspRawActPos_;
  }

  // Calculate new offset
  if(data_->command_.enable && !enableCmdOld_) {
    setCspRecalcOffset(cspPosSet_);
    data_->status_.currentPositionSetpointRaw = cspPosSet_ / scale_ + cspRawPosOffset_;
  }
  
  return 0;
}

void ecmcDriveBase::setCspActPos(int64_t posRaw, double posAct){
  cspRawActPos_ = posRaw;
  cspActPos_    = posAct;
}

// For drv at homing
void ecmcDriveBase::setCspRef(int64_t posRaw, double posAct,  double posSet) {
  cspRawActPos_ = posRaw;
  cspActPos_    = posAct;
  cspRawPosOffset_ = cspRawActPos_- posSet / scale_;  // Raw
  setCspPosSet(posSet);
}

// Recalculate offset
int ecmcDriveBase::setCspRecalcOffset(double posEng) {
  cspRawPosOffset_ = cspRawActPos_- posEng / scale_;  // Raw
  return 0;
}

double ecmcDriveBase::getScaleNum(void) {
  return scaleNum_;
}

void ecmcDriveBase::setScaleNum(double scaleNum) {
  scaleNum_ = scaleNum;

  if (std::abs(scaleDenom_) > 0) {
    scale_ = scaleNum_ / scaleDenom_;
  }
}

int ecmcDriveBase::setScaleDenom(double scaleDenom) {
  scaleDenom_ = scaleDenom;

  if (scaleDenom_ == 0) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DRV_SCALE_DENOM_ZERO);
  }
  scale_ = scaleNum_ / scaleDenom_;
  return 0;
}

double ecmcDriveBase::getScale() {
  return scale_;
}

double ecmcDriveBase::getVelSet() {
  return velSet_;
}

int ecmcDriveBase::getVelSetRaw() {
  return data_->status_.currentVelocitySetpointRaw;
}

int ecmcDriveBase::setVelSetRaw(int vel) {
  data_->status_.currentVelocitySetpointRaw = vel;
  return 0;
}

int ecmcDriveBase::setEnableBrake(bool enable) {
  if (enable) {
    int errorCode = validateEntry(3);  // brake output

    if (errorCode) {
      enableBrake_ = false;
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_DRV_BRAKE_ENTRY_NULL);
    }
  }

  enableBrake_ = enable;
  return 0;
}

int ecmcDriveBase::setEnableReduceTorque(bool enable) {
  if (enable) {
    int errorCode = validateEntry(4);  // brake output

    if (errorCode) {
      enableReduceTorque_ = false;
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_DRV_REDUCE_TORQUE_ENTRY_NULL);
    }
  }

  enableReduceTorque_ = enable;

  return 0;
}

int ecmcDriveBase::getEnableBrake() {
  return enableBrake_;
}

int ecmcDriveBase::getEnableReduceTorque() {
  return enableReduceTorque_;
}

void ecmcDriveBase::writeEntries() {

  // Check if drive status OK
  if (!driveInterlocksOK() && data_->command_.enable) {
    data_->command_.enable = false;
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_DRV_DRIVE_INTERLOCKED);
  }

  // Update enable command
  if (enableBrake_) {
    // also wait for brakeOutputCmd_
    data_->status_.enabled = getEnabledLocal() && brakeOutputCmd_;
    updateBrakeState();
  } else {    
    // No brake
    data_->status_.enabled = getEnabledLocal();  
    enableAmpCmd_ = data_->command_.enable;
  }

  int errorCode = 0;
  // will only write the number of bits configured
  errorCode =
    writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD,
                      (uint64_t)controlWord_);
  if (errorCode) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  if(data_->command_.drvMode == ECMC_DRV_MODE_CSV) {
    errorCode =
      writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT,
                      (uint64_t)data_->status_.currentVelocitySetpointRaw);

    if (errorCode) {
      setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  } 
  else {
    // CSP
    errorCode =
      writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_POSITION_SETPOINT,
                      (uint64_t)data_->status_.currentPositionSetpointRaw);

    if (errorCode) {
      setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  }

  if (enableBrake_) {
    errorCode =
      writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT,
                        (uint64_t)brakeOutputCmd_);

    if (errorCode) {
      setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  }

  if (enableReduceTorque_) {
    reduceTorqueOutputCmd_    = data_->status_.atTarget;
    reduceTorqueOutputCmdOld_ = reduceTorqueOutputCmd_;
    errorCode                 = writeEcEntryValue(
      ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT,
      (uint64_t)reduceTorqueOutputCmd_);

    if (errorCode) {
      setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  }

  // write reset
  if (hwResetDefined_) {
    errorCode =
      writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_RESET,
                        (uint64_t)hwReset_);
    hwReset_ = 0;
    if (errorCode) {
      setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  }

  // Timeout?  
  if(!getEnabledLocal() && data_->command_.enable) {
    cycleCounterBase_++;
    if(cycleCounterBase_ > stateMachineTimeoutCycles_) {
      // Enable cmd timeout (not recived enable within time period)
      cycleCounterBase_ = 0;
      data_->command_.enable = 0;
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_DRV_STATE_MACHINE_TIME_OUT);
    }
  }  else {
    cycleCounterBase_ = 0;
  }

  // Enabled lost?
  if(!getEnabledLocal() && localEnabledOld_ && data_->command_.enable) {
      data_->command_.enable = 0;
      LOGERR("%s/%s:%d: WARNING (axis %d): Drive enabled lost while enable cmd is high.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_);
  }

  localEnabledOld_ = getEnabledLocal();
  
  // Enable command sent to amplfier
  // (if break is not used then enableAmpCmdOld_==enableCmdOld_)
  enableAmpCmdOld_ = enableAmpCmd_;
  enableCmdOld_    = data_->command_.enable;

  refreshAsyn();
}

void ecmcDriveBase::readEntries() {

  if (readEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD, &statusWord_)) {
    setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_DRV_ENABLED_READ_ENTRY_FAIL);
    statusWord_ = 0;
    return;
  }

  // Check warning link. Think about forwarding warning info to motor record somehow
  if (hwWarningDefined_) {
    if (readEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_WARNING, &hwWarning_)) {
      hwWarning_ = 0;
      hwWarningOld_ = 0;
      setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_DRV_WARNING_READ_ENTRY_FAIL);

      return;
    }

    if(hwWarning_ > 0 && hwWarningOld_ == 0) {
      LOGERR("%s/%s:%d: WARNING (axis %d): Drive hardware in warning state.\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          data_->axisId_);

    }
    if(hwWarning_ == 0 && hwWarningOld_ > 0) {
      LOGERR("%s/%s:%d: INFO (axis %d): Drive hardware warning state cleared.\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          data_->axisId_);
    }

    hwWarningOld_ = hwWarning_;
  }

  // check alarm 0
  if (hwErrorAlarm0Defined_) {
    if (readEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_ALARM_0, &hwErrorAlarm0_)) {
      hwErrorAlarm0_ = 0;
      hwErrorAlarm0Old_ = 0;
      setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_DRV_ALARM_READ_ENTRY_FAIL);

      return;
    }
    
    // Set Alarm
    if(hwErrorAlarm0_) {
      data_->command_.enable = 0;
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_DRV_HW_ALARM_0);
    }    
    hwErrorAlarm0Old_ = hwErrorAlarm0_;
  }

  // check alarm 1
  if (hwErrorAlarm1Defined_) {
    if (readEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_ALARM_1, &hwErrorAlarm1_)) {
      hwErrorAlarm1_ = 0;
      hwErrorAlarm1Old_ = 0;
      setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_DRV_ALARM_READ_ENTRY_FAIL);

      return;
    }
    
    // Set Alarm
    if(hwErrorAlarm1_) {
      data_->command_.enable = 0;
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_DRV_HW_ALARM_1);
    }    
    hwErrorAlarm1Old_ = hwErrorAlarm1_;
  }

  // check alarm 2
  if (hwErrorAlarm2Defined_) {
    if (readEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_ALARM_2, &hwErrorAlarm2_)) {
      hwErrorAlarm2_ = 0;
      hwErrorAlarm2Old_ = 0;
      setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_DRV_ALARM_READ_ENTRY_FAIL);

      return;
    }
    
    // Set Alarm
    if(hwErrorAlarm2_) {
      data_->command_.enable = 0;
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_DRV_HW_ALARM_2);
    }    
    hwErrorAlarm2Old_ = hwErrorAlarm2_;
  }
}

int ecmcDriveBase::validate() {
  // Enable entry output OR controlword
  int errorCode = validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }


  if(checkEntryExist(ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT)){
    // CSV
    int errorCodeVel = validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT);
    if (errorCodeVel) {    
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCodeVel);
    }
    data_->command_.drvMode = ECMC_DRV_MODE_CSV;
  } 
  else {
    // Must be CSP
    int errorCodePos = validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_POSITION_SETPOINT);
    if (errorCodePos) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCodePos);
    }
    data_->command_.drvMode = ECMC_DRV_MODE_CSP;
  }
  
  // Enabled entry input OR statusword
  errorCode = validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  // Brake output
  if (enableBrake_) {
    errorCode = validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT);

    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  }

  // reduce torque output
  if (enableReduceTorque_) {
    errorCode = validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT);

    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  }
  
  // Check reset link
  if (checkEntryExist(ECMC_DRIVEBASE_ENTRY_INDEX_RESET)) {
    errorCode = validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_RESET);
    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
    hwResetDefined_ = true;
  }

  // Check warning link
  if (checkEntryExist(ECMC_DRIVEBASE_ENTRY_INDEX_WARNING)) {
    errorCode = validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_WARNING);
    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
    hwWarningDefined_ = true;
  }

  // Check alarm link 0
  if (checkEntryExist(ECMC_DRIVEBASE_ENTRY_INDEX_ALARM_0)) {
    errorCode = validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_ALARM_0);
    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
    hwErrorAlarm0Defined_ = true;
  }

  // Check alarm link 1
  if (checkEntryExist(ECMC_DRIVEBASE_ENTRY_INDEX_ALARM_1)) {
    errorCode = validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_ALARM_1);
    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
    hwErrorAlarm1Defined_ = true;
  }

  // Check alarm link 2
  if (checkEntryExist(ECMC_DRIVEBASE_ENTRY_INDEX_ALARM_2)) {
    errorCode = validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_ALARM_2);
    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
    hwErrorAlarm2Defined_ = true;
  }

  if (scaleDenom_ == 0) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DRV_SCALE_DENOM_ZERO);
  }


  return 0;
}

bool ecmcDriveBase::getEnable() {
  return enableAmpCmd_;
}

bool ecmcDriveBase::getEnabled() {
  return data_->status_.enabled;
}

bool ecmcDriveBase::driveInterlocksOK() {
  return !(data_->interlocks_.driveSummaryInterlock ||
           data_->interlocks_.etherCatMasterInterlock);
}

int ecmcDriveBase::setBrakeOpenDelayTime(int delayTime) {
  if (delayTime < 0) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DRV_BRAKE_OPEN_DELAY_TIME_INVALID);
  }

  brakeOpenDelayTime_ = delayTime;
  return 0;
}

int ecmcDriveBase::setBrakeCloseAheadTime(int aheadTime) {
  if (aheadTime < 0) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DRV_BRAKE_CLOSE_AHEAD_TIME_INVALID);
  }

  brakeCloseAheadTime_ = aheadTime;
  return 0;
}

int ecmcDriveBase::updateBrakeState() {
  // General state transitions

  if (data_->command_.enable && !enableCmdOld_) {
    brakeState_   = ECMC_BRAKE_OPENING;
    brakeCounter_ = 0;
  }
  if (!data_->command_.enable && enableCmdOld_) {
    brakeState_   = ECMC_BRAKE_CLOSING;
    brakeCounter_ = 0;
  }

  switch (brakeState_) {
  case ECMC_BRAKE_CLOSED:
    brakeOutputCmd_ = 0;
    brakeCounter_   = 0;
    enableAmpCmd_   = 0;
    break;

  case ECMC_BRAKE_OPENING:

    // Purpose: Postpone opening of brake
    enableAmpCmd_ = 1;
    if (brakeCounter_ > brakeOpenDelayTime_) {
      
      if(!getEnabledLocal()) {
        data_->command_.enable = 0;
        brakeOutputCmd_ = 0;
        brakeCounter_   = 0;
        enableAmpCmd_   = 0;
        enableCmdOld_   = 0;  // to not trigger ECMC_BRAKE_CLOSING
        brakeState_     = ECMC_BRAKE_CLOSED;
      } else {
        brakeState_     = ECMC_BRAKE_OPEN;
        brakeOutputCmd_ = 1;
      }
    }

    // loose enabled while opening, go directlly to closed
    if(localEnabledOld_ && !getEnabledLocal()) {
      brakeState_     = ECMC_BRAKE_CLOSED;      
      enableAmpCmd_   = 0;
      brakeOutputCmd_ = 0;
    }

    // only start counting if enabled
    if(getEnabledLocal()) {
      brakeCounter_++;
    }

    break;

  case ECMC_BRAKE_OPEN:
    
    // enabled lost: apply brake directly without delay, goto stae BRAKE_CLOSED 
    if(!getEnabledLocal() && data_->command_.enable) {
      //data_->command_.enable = 0;  this is controlled separatelly
      brakeOutputCmd_ = 0;
      brakeCounter_   = 0;
      enableAmpCmd_   = 0;
      enableCmdOld_   = 0;  // to avoid to trigger ECMC_BRAKE_CLOSING state (see beginning of this function)
      brakeState_     = ECMC_BRAKE_CLOSED;
    } else {
      brakeOutputCmd_ = 1;
      brakeCounter_   = 0;
      enableAmpCmd_   = 1;
    }
    break;

  case ECMC_BRAKE_CLOSING:
    // Purpose: Postpone disable of amplifier

    brakeOutputCmd_ = 0;
    enableAmpCmd_   = 1;

    if (brakeCounter_ > brakeCloseAheadTime_ || !getEnabledLocal()) {
      brakeState_     = ECMC_BRAKE_CLOSED;
      enableAmpCmd_   = 0;
      brakeOutputCmd_ = 0;
    }

    brakeCounter_++;
    break;
  }

  return 0;
}

void ecmcDriveBase::errorReset() {
  
  // Reset hardware if needed
  if(hwResetDefined_) {
    hwReset_ = 1;
  }

  ecmcError::errorReset();
}

int ecmcDriveBase::initAsyn() {
  if (asynPortDriver_ == NULL) {
    LOGERR("%s/%s:%d: ERROR (axis %d): Drive AsynPortDriver object NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_->axisId_,
           ERROR_DRV_ASYN_PORT_OBJ_NULL);
    return ERROR_DRV_ASYN_PORT_OBJ_NULL;
  }

  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char *name = NULL;
  unsigned int charCount = 0;
  ecmcAsynDataItem *paramTemp = NULL;
  // Control word "ax<id>.drv.control"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_AX_STR "%d." ECMC_DRV_STR "." ECMC_DRV_ENABLE_STR,
                       data_->axisId_);
  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      ERROR_DRV_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_DRV_ASYN_PRINT_TO_BUFFER_FAIL;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamUInt32Digital,
                                         (uint8_t *)&(controlWord_),
                                         sizeof(controlWord_),
                                         ECMC_EC_U32,
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->addSupportedAsynType(asynParamInt32);
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->refreshParam(1);
  asynControlWd_ = paramTemp;


  // Status word "ax<id>.drv.status"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_AX_STR "%d." ECMC_DRV_STR "." ECMC_ASYN_AX_STATUS_NAME,
                       data_->axisId_);
  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      ERROR_DRV_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_DRV_ASYN_PRINT_TO_BUFFER_FAIL;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamUInt32Digital,
                                         (uint8_t *)&(statusWord_),
                                         sizeof(statusWord_),
                                         ECMC_EC_U32,
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->addSupportedAsynType(asynParamInt32);
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);    
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  asynStatusWd_ = paramTemp;
  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);

  return 0;
}

void ecmcDriveBase::refreshAsyn(){
  asynStatusWd_->refreshParamRT(0);
  asynControlWd_->refreshParamRT(0);
}

int ecmcDriveBase::setStateMachineTimeout(double seconds) {
  stateMachineTimeoutCycles_ = seconds / data_->sampleTime_;
  return 0;
}
