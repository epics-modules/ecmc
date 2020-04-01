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
  manualModeEnableAmpCmd_    = false;
  manualModeEnableAmpCmdOld_ = false;
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

int ecmcDriveBase::setEnable(bool enable) {
  // Only allowed in manual mode !!
  if (data_->command_.operationModeCmd != ECMC_MODE_OP_MAN) {
    manualModeEnableAmpCmd_ = false;
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DRV_COMMAND_NOT_ALLOWED_IN_AUTO_MODE);
  }

  manualModeEnableAmpCmdOld_ = manualModeEnableAmpCmd_;
  manualModeEnableAmpCmd_    = enable;

  return 0;
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
  if (!driveInterlocksOK() && data_->command_.enable) {
    data_->command_.enable = false;
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_DRV_DRIVE_INTERLOCKED);
  }

  int errorCode = 0;
  // will only write the number of bits configured
  errorCode =
    writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD,
                      (uint64_t)controlWord_);
  if (errorCode) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  errorCode =
    writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT,
                      (uint64_t)data_->status_.currentVelocitySetpointRaw);

  if (errorCode) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
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

  // Enable command sent to amplfier
  // (if break is not used then enableAmpCmdOld_==enableCmdOld_)
  enableAmpCmdOld_ = enableAmpCmd_;
  enableCmdOld_    = data_->command_.enable;

  refreshAsyn();
}

void ecmcDriveBase::readEntries() {
  // Update enable command
  if (enableBrake_) {
    updateBrakeState();
  } else {
    // No brake
    switch (data_->command_.operationModeCmd) {
    case ECMC_MODE_OP_AUTO:
      enableAmpCmd_ = data_->command_.enable;
      break;

    case ECMC_MODE_OP_MAN:
      enableAmpCmd_ = manualModeEnableAmpCmd_;
      break;
    }
  }

  if (readEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD, &statusWord_)) {
    setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_DRV_ENABLED_READ_ENTRY_FAIL);
    statusWord_ = 0;
    return;
  }
}

int ecmcDriveBase::validate() {
  // Enable entry output OR controlword
  int errorCode = validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  // Velocity Setpoint entry output
  errorCode = validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT);
  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
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
  switch (data_->command_.operationModeCmd) {
  case ECMC_MODE_OP_AUTO:

    if (data_->command_.enable && !enableCmdOld_) {
      brakeState_   = ECMC_BRAKE_OPENING;
      brakeCounter_ = 0;
    }

    if (!data_->command_.enable && enableCmdOld_) {
      brakeState_   = ECMC_BRAKE_CLOSING;
      brakeCounter_ = 0;
    }
    break;

  case ECMC_MODE_OP_MAN:

    if (manualModeEnableAmpCmd_ && !manualModeEnableAmpCmdOld_) {
      brakeState_   = ECMC_BRAKE_OPENING;
      brakeCounter_ = 0;
    }

    if (!manualModeEnableAmpCmd_ && manualModeEnableAmpCmdOld_) {
      brakeState_   = ECMC_BRAKE_CLOSING;
      brakeCounter_ = 0;
    }
    break;
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

    if (brakeCounter_ >= brakeOpenDelayTime_) {
      brakeState_     = ECMC_BRAKE_OPEN;
      brakeOutputCmd_ = 1;
    }
    brakeCounter_++;
    break;

  case ECMC_BRAKE_OPEN:
    brakeOutputCmd_ = 1;
    brakeCounter_   = 0;
    enableAmpCmd_   = 1;
    break;

  case ECMC_BRAKE_CLOSING:

    // Purpose: Postpone disable of amplifier
    brakeOutputCmd_ = 0;
    enableAmpCmd_   = 1;

    if (brakeCounter_ >= brakeCloseAheadTime_) {
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