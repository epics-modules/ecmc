/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcDriveDS402.cpp
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcDriveDS402.h"

ecmcDriveDS402::ecmcDriveDS402(ecmcAsynPortDriver *asynPortDriver,
                               ecmcAxisData       *axisData) :
  ecmcDriveBase(asynPortDriver, axisData) {
  data_ = axisData;

  initVars();

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
}

ecmcDriveDS402::ecmcDriveDS402(ecmcAsynPortDriver *asynPortDriver,
                               ecmcAxisData       *axisData,
                               double              scale) :
  ecmcDriveBase(asynPortDriver, axisData) {
  data_ = axisData;
  initVars();

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }

  scale_ = scale;
}

ecmcDriveDS402::~ecmcDriveDS402() {}

void ecmcDriveDS402::initVars() {
  enableStateMachine_    = ECMC_DS402_RESET_STATE;
  ds402WarningOld_       = false;
  enableStateMachineOld_ = enableStateMachine_;
  cycleCounter_          = 0;
  localEnabled_          = 0;
  localEnableAmpCmdOld_  = 0;
  startupFaultCleared_   = 0;
}

int ecmcDriveDS402::validate() {
  int errorCode = ecmcDriveBase::validate();

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  int bitCount = 0;  // DS402 must have 16 bit control word
  getEntryBitCount(ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD, &bitCount);

  if (bitCount != 16) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DRV_DS402_CONTROL_WORD_BIT_COUNT_ERROR);
  }

  int startBit = 0;  // DS402 must use all bits in word
  getEntryStartBit(ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD, &startBit);

  if (startBit != -1) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DRV_DS402_CONTROL_WORD_START_BIT_ERROR);
  }

  bitCount = 0;  // DS402 must have 16 bit status word
  getEntryBitCount(ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD, &bitCount);

  if (bitCount != 16) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DRV_DS402_STATUS_WORD_BIT_COUNT_ERROR);
  }

  startBit = 0;  // DS402 must use all bits in word
  getEntryStartBit(ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD, &startBit);

  if (startBit != -1) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DRV_DS402_STATUS_WORD_START_BIT_ERROR);
  }

  return 0;
}

void ecmcDriveDS402::writeEntries() {
  ecmcDriveBase::writeEntries();  // All not drive specific I/O
}

/*ECMC_DS402_READY_TO_SWITCH_ON_BIT 0
ECMC_DS402_SWITCHED_ON_BIT 1
ECMC_DS402_OPERATION_ENABLED_BIT 2
ECMC_DS402_FAULT_BIT 3
ECMC_DS402_SWITCH_ON_DISABLED_BIT 6
ECMC_DS402_SWITCH_ON_WARNING_BIT 7
ECMC_DS402_SWITCH_ON_INT_LIM 11*/
void ecmcDriveDS402::readEntries(bool masterOK) {
  ecmcDriveBase::readEntries(masterOK);

  if (cycleCounter_ > stateMachineTimeoutCycles_) {
    enableStateMachine_ = ECMC_DS402_FAULT_STATE;
    setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_DRV_DS402_STATE_MACHINE_TIME_OUT);
    return;
  }

  enableStateMachineOld_ = enableStateMachine_;

  cycleCounter_++;

  bool ds402Fault = BIT_CHECK(statusWord_, ECMC_DS402_FAULT_BIT);
  localEnabled_ = BIT_CHECK(statusWord_, ECMC_DS402_OPERATION_ENABLED_BIT);
  bool ds402Warning = BIT_CHECK(statusWord_, ECMC_DS402_SWITCH_ON_WARNING_BIT);

  if(!startupFaultCleared_) {
    if(masterOK) {
      //if(ds402Fault) {
        errorReset(); // Reset the DS402 fault at startup
      //}
    }
    if(!ds402Fault && masterOK){
      LOGERR("%s/%s:%d: DS402 Startup fault cleared.\n",
        __FILE__,
        __FUNCTION__,
        __LINE__);
        startupFaultCleared_ = 1;
        enableStateMachine_ = ECMC_DS402_RESET_STATE;
    }
  }
  
  // Printout warning.. Do not stop
  if (ds402Warning && !ds402WarningOld_) {
    LOGERR("%s/%s:%d: DS402 Warning bit high.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
  }
  ds402WarningOld_ = ds402Warning;

  switch (enableStateMachine_) {
  case ECMC_DS402_IDLE_STATE:
    controlWord_  = 0;
    cycleCounter_ = 0;

    if (enableAmpCmd_ && !localEnableAmpCmdOld_) {
      enableStateMachine_ = ECMC_DS402_STARTUP_RESET;
    }
    break;

  case ECMC_DS402_STARTUP_RESET:
    controlWord_        = 128;
    cycleCounter_       = 0;
    enableStateMachine_ = ECMC_DS402_SWITCH_ON_DISABLED_STATE;
    break;

  case ECMC_DS402_SWITCH_ON_DISABLED_STATE:
    controlWord_ = 6;

    if (BIT_CHECK(statusWord_, ECMC_DS402_READY_TO_SWITCH_ON_BIT)) {
      cycleCounter_       = 0;
      enableStateMachine_ = ECMC_DS402_READY_TO_SWITCH_ON_STATE;
    }

    if (!enableAmpCmd_) {
      enableStateMachine_ = ECMC_DS402_IDLE_STATE;
    }
    break;

  case ECMC_DS402_READY_TO_SWITCH_ON_STATE:
    controlWord_ = 7;

    if (ds402Fault) {
      cycleCounter_       = 0;
      enableStateMachine_ = ECMC_DS402_FAULT_STATE;
    }

    if (BIT_CHECK(statusWord_, ECMC_DS402_SWITCHED_ON_BIT)) {
      cycleCounter_       = 0;
      enableStateMachine_ = ECMC_DS402_SWITCHED_ON_STATE;
    }

    if (!enableAmpCmd_) {
      enableStateMachine_ = ECMC_DS402_IDLE_STATE;
    }
    break;

  case ECMC_DS402_SWITCHED_ON_STATE:
    controlWord_ = 15;

    if (ds402Fault) {
      cycleCounter_       = 0;
      enableStateMachine_ = ECMC_DS402_FAULT_STATE;
    }

    if (BIT_CHECK(statusWord_, ECMC_DS402_OPERATION_ENABLED_BIT)) {
      cycleCounter_       = 0;
      enableStateMachine_ = ECMC_DS402_OPERATION_ENABLED_STATE;
    }

    if (!enableAmpCmd_) {
      enableStateMachine_ = ECMC_DS402_IDLE_STATE;
    }
    break;

  case ECMC_DS402_OPERATION_ENABLED_STATE:

    if (ds402Fault) {
      cycleCounter_       = 0;
      enableStateMachine_ = ECMC_DS402_FAULT_STATE;
    }

    cycleCounter_ = 0;

    if (!enableAmpCmd_) {
      enableStateMachine_ = ECMC_DS402_IDLE_STATE;
    }
    break;

  case ECMC_DS402_FAULT_STATE:
    controlWord_           = 0;
    cycleCounter_          = 0;
    enableAmpCmd_          = false;
    data_->command_.controlWord_.enableCmd = false;
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_DRV_DS402_FAULT_STATE);
    break;

  case ECMC_DS402_RESET_STATE:
    controlWord_  = 128;
    cycleCounter_ = 0;

    if (!BIT_CHECK(statusWord_, ECMC_DS402_FAULT_BIT)) {
      enableStateMachine_    = ECMC_DS402_IDLE_STATE;
      data_->command_.controlWord_.enableCmd = false;
    }
    break;
  }

  localEnableAmpCmdOld_ = enableAmpCmd_;
}

void ecmcDriveDS402::errorReset() {
  // Reset error in drive (controlword=128)
  if (enableStateMachine_ == ECMC_DS402_FAULT_STATE) {
    enableStateMachine_ = ECMC_DS402_RESET_STATE;
    cycleCounter_       = 0;
  }
  ecmcDriveBase::errorReset();
}

bool ecmcDriveDS402::getEnabledLocal() {
  return localEnabled_;
}

int ecmcDriveDS402::hwReady() {
  // Consider also checking that CSP encoder is OK..   
  return !BIT_CHECK(statusWord_, ECMC_DS402_FAULT_BIT) &&
         masterOK_;
}
