/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAxisBase.cpp
*
*  Created on: Mar 10, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcAxisBase.h"
#include "ecmcRtLogger.h"
#include "ecmcRtLoggerPortDriver.h"
#include <inttypes.h>
#include <stdint.h>
#include <new>
#include <iostream>
#include <cmath>
#include "ecmcMotion.h"
#include "ecmcErrorsList.h"
#include "ecmcGlobalsExtern.h"
#include "ecmcMainThread.h"

#define ecmcRtLoggerLogInfo(...) \
  ECMC_RT_LOG_AXIS_BASE_INFO(data_.status_.axisId, __VA_ARGS__)
#define ecmcRtLoggerLogError(...) \
  ECMC_RT_LOG_AXIS_BASE_ERROR(data_.status_.axisId, __VA_ARGS__)
#define ecmcRtLoggerLogDebug(...) \
  ECMC_RT_LOG_AXIS_BASE_DEBUG(data_.status_.axisId, __VA_ARGS__)

namespace {
bool resolveAxisEncoderSampleTimeNs(ecmcEncoder *encoder,
                                    uint64_t *eventTimeNs) {
  if (!encoder || !eventTimeNs) return false;
  ecmcSignalTime time;
  if (!encoder->getSampleTiming(time)) return false;
  *eventTimeNs = time.sampleTimeNs;
  return true;
}

const char *axisCommandToString(motionCommandTypes command) {
  switch (command) {
  case ECMC_CMD_NOCMD:
    return "NO_CMD";
  case ECMC_CMD_JOG:
    return "JOG";
  case ECMC_CMD_MOVEVEL:
    return "MOVE_VEL";
  case ECMC_CMD_MOVEREL:
    return "MOVE_REL";
  case ECMC_CMD_MOVEABS:
    return "MOVE_ABS";
  case ECMC_CMD_MOVEMODULO:
    return "MOVE_MODULO";
  case ECMC_CMD_MOVEPVTABS:
    return "MOVE_PVT_ABS";
  case ECMC_CMD_HOMING:
    return "HOMING";
  case ECMC_CMD_SUPERIMP:
    return "SUPERIMP";
  case ECMC_CMD_GEAR:
    return "GEAR";
  default:
    return "UNKNOWN";
  }
}

void appendControlWordAction(char *message,
                             size_t messageSize,
                             const char *action) {
  size_t messageLength = strlen(message);
  if (messageLength >= messageSize) {
    return;
  }
  snprintf(message + messageLength,
           messageSize - messageLength,
           "%s%s",
           messageLength ? ", " : "",
           action);
}

void logRuntimeAxisDoubleChange(ecmcAxisBase *axis,
                                const char   *name,
                                double        value) {
  if (!axis || !axis->getRealTimeStarted()) {
    return;
  }

  ecmcLogBufferWrite(ECMC_LOG_BUFFER_INFO,
                       "Axis[%d]: runtime set %s=%.12g.",
                       axis->getAxisID(),
                       name,
                       value);
}

bool axisUsesEcmcPositionController(ecmcAxisBase *axis) {
  if (!axis) {
    return false;
  }

  const int drvMode = axis->getDrvMode();
  return drvMode == ECMC_DRV_MODE_CSV ||
         (drvMode == ECMC_DRV_MODE_CSP &&
          axis->getCSPDriveEncoderIndex() >= 0);
}

bool warnRuntimeControllerKpZero(ecmcAxisBase *axis,
                                 const char   *name,
                                 double        value) {
  if (!axis || !axis->getRealTimeStarted() || value != 0 ||
      !axisUsesEcmcPositionController(axis)) {
    return false;
  }

  ecmcLogBufferWrite(ECMC_LOG_BUFFER_WARNING,
                     "Axis[%d]: runtime %s is 0.",
                     axis->getAxisID(),
                     name);

  ECMC_RT_LOG_AXIS_BASE_WARNING(
    axis->getAxisID(),
    "%s/%s:%d: WARNING: Axis[%d]: runtime %s is 0.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    axis->getAxisID(),
    name);
  return true;
}
}

/**
 * Callback function for asynWrites (control word)
 * userObj = axis object
 *
 * */
asynStatus asynWriteCmd(void         *data,
                        size_t        bytes,
                        asynParamType asynParType,
                        void         *userObj) {
  if (!userObj) {
    return asynError;
  }
  return ((ecmcAxisBase *)userObj)->axisAsynWriteCmd(data, bytes, asynParType);
}

/**
 * Callback function for asynWrites (Target Velo)
 * userObj = axis object
 *
 * */
asynStatus asynWriteAcc(void         *data,
                        size_t        bytes,
                        asynParamType asynParType,
                        void         *userObj) {
  if (!userObj) {
    return asynError;
  }
  return ((ecmcAxisBase *)userObj)->axisAsynWriteAcc(data,
                                                     bytes,
                                                     asynParType);
}

/**
 * Callback function for asynWrites (Target Velo)
 * userObj = axis object
 *
 * */
asynStatus asynWriteDec(void         *data,
                        size_t        bytes,
                        asynParamType asynParType,
                        void         *userObj) {
  if (!userObj) {
    return asynError;
  }
  return ((ecmcAxisBase *)userObj)->axisAsynWriteDec(data,
                                                     bytes,
                                                     asynParType);
}

/**
 * Callback function for asynWrites (Target Velo)
 * userObj = axis object
 *
 * */
asynStatus asynWriteTargetVelo(void         *data,
                               size_t        bytes,
                               asynParamType asynParType,
                               void         *userObj) {
  if (!userObj) {
    return asynError;
  }
  return ((ecmcAxisBase *)userObj)->axisAsynWriteTargetVelo(data,
                                                            bytes,
                                                            asynParType);
}

/**
 * Callback function for asynWrites (set primary encoder)
 * userObj = axis object
 *
 * */
asynStatus asynWritePrimEncCtrlId(void         *data,
                                  size_t        bytes,
                                  asynParamType asynParType,
                                  void         *userObj) {
  if (!userObj) {
    return asynError;
  }
  return ((ecmcAxisBase *)userObj)->axisAsynWritePrimEncCtrlId(data,
                                                               bytes,
                                                               asynParType);
}

/**
 * Callback function for asynWrites (Target Pos)
 * userObj = axis object
 *
 * */
asynStatus asynWriteTargetPos(void         *data,
                              size_t        bytes,
                              asynParamType asynParType,
                              void         *userObj) {
  if (!userObj) {
    return asynError;
  }
  return ((ecmcAxisBase *)userObj)->axisAsynWriteTargetPos(data,
                                                           bytes,
                                                           asynParType);
}

/**
 * Callback function for asynWrites (Set Encoder Position)
 * userObj = axis object
 *
 * */
asynStatus asynWriteSetEncPos(void         *data,
                              size_t        bytes,
                              asynParamType asynParType,
                              void         *userObj) {
  if (!userObj) {
    return asynError;
  }
  return ((ecmcAxisBase *)userObj)->axisAsynWriteSetEncPos(data,
                                                           bytes,
                                                           asynParType);
}

/**
 * Callback function for asynWrites (Set Encoder Position)
 * userObj = axis object
 *
 * */
asynStatus asynWriteSetTweakValue(void         *data,
                                  size_t        bytes,
                                  asynParamType asynParType,
                                  void         *userObj) {
  if (!userObj) {
    return asynError;
  }
  return ((ecmcAxisBase *)userObj)->axisAsynWriteSetTweakValue(data,
                                                               bytes,
                                                               asynParType);
}

/**
 * Callback function for asynWrites (Command)
 * userObj = axis object
 *
 * */
asynStatus asynWriteCommand(void         *data,
                            size_t        bytes,
                            asynParamType asynParType,
                            void         *userObj) {
  if (!userObj) {
    return asynError;
  }
  return ((ecmcAxisBase *)userObj)->axisAsynWriteCommand(data,
                                                         bytes,
                                                         asynParType);
}

/**
 * Callback function for asynWrites (Cmddata)
 * userObj = axis object
 *
 * */
asynStatus asynWriteCmdData(void         *data,
                            size_t        bytes,
                            asynParamType asynParType,
                            void         *userObj) {
  if (!userObj) {
    return asynError;
  }
  return ((ecmcAxisBase *)userObj)->axisAsynWriteCmdData(data,
                                                         bytes,
                                                         asynParType);
}

ecmcAxisBase::ecmcAxisBase(ecmcAsynPortDriver *asynPortDriver,
                           int                 axisID,
                           double              sampleTime,
                           ecmcTrajTypes       trajType) {
  initVars();
  asynPortDriver_   = asynPortDriver;
  data_.status_.axisId     = axisID;
  data_.status_.sampleTime = sampleTime;
  invSampleTime_ = sampleTime > 0 ? 1.0 / sampleTime : 0.0;
  data_.control_.cspDrvEncIndex =  -1;   // used for control
  setExternalPtrs(&(data_.status_.errorCode), &(data_.status_.warningCode));
  try {
    data_.control_.primaryEncIndex = 0;
    int errorCode = addEncoder();
    if (errorCode) {
      setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
      return;
    }
    currentTrajType_ = trajType;

    if (currentTrajType_ == ECMC_S_CURVE) {
      traj_ = new ecmcTrajectoryS(data_,
                                  data_.status_.sampleTime);
    } else {
      traj_ = new ecmcTrajectoryTrapetz(data_,
                                        data_.status_.sampleTime);
    }

    mon_ = new ecmcMonitor(data_, encArray_);

    extTrajVeloFilter_ = new ecmcFilter(data_.status_.sampleTime);
    extEncVeloFilter_  = new ecmcFilter(data_.status_.sampleTime);
  } catch (std::bad_alloc& ex) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Memory allocation failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId);
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_MAIN_EXCEPTION);
    return;
  }
  seq_.init(sampleTime);
  seq_.setAxisDataRef(&data_);
  seq_.setTraj(traj_);
  seq_.setMon(mon_);
  seq_.setEnc(encArray_);
  int errorCode = initAsyn();
  if (errorCode) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }
}

ecmcAxisBase::~ecmcAxisBase() {
  invalidateDataSources();
  for (int i = 0; i < ECMC_MAX_ENCODERS; i++) {
    delete encArray_[i];
  }
  delete traj_;
  traj_ = NULL;
  delete mon_;
  mon_ = NULL;
  delete extTrajVeloFilter_;
  extTrajVeloFilter_ = NULL;
  delete extEncVeloFilter_;
  extEncVeloFilter_ = NULL;
}

void ecmcAxisBase::initVars() {
  // errorReset();  //THIS IS NOONOO.. KEEP THIS COMMENT!!!
  firstEnableDone_                       = 0;
  data_.status_.axisType                          = ECMC_AXIS_TYPE_BASE;
  data_.control_.controlWord_.resetCmd     = false;
  allowCmdFromOtherPLC_                    = true;
  data_.status_.statusWord_.instartup      = false;
  data_.status_.statusWord_.inrealtime     = false;
  data_.status_.externalTrajectoryPosition = 0;
  data_.status_.externalTrajectoryVelocity = 0;
  data_.status_.externalEncoderPosition    = 0;
  data_.status_.externalEncoderVelocity    = 0;
  data_.status_.currentPositionActual      = 0;
  data_.status_.currentPositionSetpoint    = 0;
  data_.status_.currentVelocityActual      = 0;
  data_.status_.currentVelocitySetpoint    = 0;
  data_.status_.sampleTime                 = 1 / 1000;
  data_.status_.cycleCounter               = 0;
  printHeaderCounter_      = 0;
  axisState_               = ECMC_AXIS_STATE_STARTUP;
  asynPortDriver_          = NULL;
  memset(&mrCmds_,    0, sizeof(mrCmds_));
  memset(&mrCmdsOld_,    0, sizeof(mrCmdsOld_));

  for (int i = 0; i < ECMC_ASYN_AX_PAR_COUNT; i++) {
    data_.axAsynParams_[i] = NULL;
  }

  statusOutputEntry_.reset();

  memset(diagBuffer_, 0, AX_MAX_DIAG_STRING_CHAR_LENGTH);
  extTrajVeloFilter_         = NULL;
  extEncVeloFilter_          = NULL;
  enableExtTrajVeloFilter_   = false;
  enableExtEncVeloFilter_    = false;
  disableAxisAtErrorReset_   = false;
  data_.control_.positionTarget = 0;
  data_.control_.command    = ECMC_CMD_MOVEABS;
  data_.control_.cmdData    = 0;
  data_.status_.encoderCount = 0;

  for (int i = 0; i < ECMC_MAX_ENCODERS; i++) {
    encArray_[i] = NULL;
  }

  data_.control_.cfgEncIndex    = 0;
  data_.control_.allowSourceChangeWhenEnabled = false;
  setEncoderPos_                = 0;
  encPrimIndexAsyn_             = 1;
  data_.status_.currentAccelerationSetpoint = 0;
  data_.status_.currentDecelerationSetpoint  = 0;
  hwReady_                      = 0;
  hwReadyOld_                   = 0;
  ignoreMRDisableStatusCheck_   = 0;
  autoEnableTimoutS_            = -1.0;
  autoDisableAfterS_            = -1.0;
  autoEnableRequest_            = false;
  encoderPowerEnablePending_    = false;
  encoderPowerInitPending_      = false;
  encoderPowerMoveCompleted_    = false;
  encoderMode_                  = 0;
  encoderPowerSourceIndex_      = -1;
  encoderPowerBackupIndex_      = -1;
  encoderPowerDiffTol_          = .01;
  encoderPowerBackupAligned_   = false;
  motionStartPermissive_         = nullptr;
  motionStartPending_            = false;
  autoEnableTimeCounter_        = 0.0;
  autoDisbleTimeCounter_        = 0.0;
  enableAutoEnable_             = 0;
  enableAutoDisable_            = 0;
  autoDisableAtTgtLtchEnable_   = false;
  autoDisableAtTargetLatched_   = false;
  positionTargetAsyn_           = 0;
  autoExecuteOnTargetPos_       = false;
  statusWord2Param_            = nullptr;
  invSampleTime_                = 1000.0;
  masterSlaveBlocked_           = false;
  data_.status_.statusWord_.blocked = 0;
  enableAutoResetError_         = 1;
  motionCommandMotorRecordRequestCounter_ = 0;
  motionCommandRequestCounter_  = 0;
  motionCommandExecuteCounter_  = 0;
}

void ecmcAxisBase::preExecute(bool masterOK) {
  if (getStartupCommandsHeld()) {
    // Keep drives disabled (including external trajectory mode) while motor
    // startup callbacks and position restoration run. Do not re-enter axis
    // startup here: the motor poller needs initialized axes to finish its work.
    setExecute(false, true);
    setEnable(false);
  } else if (startupEnablePending_) {
    startupEnablePending_ = false;
    setEnable(data_.control_.enableAtStartup);
    setMRCnen(data_.control_.enableAtStartup);
  }
  if (ecmcStartupMotionHeld()) {
    axisState_ = ECMC_AXIS_STATE_STARTUP;
    setInStartupPhase(true);
    setEnable(false);
  }
  auto &status = data_.status_;
  auto &statusWord = status.statusWord_;
  auto &interlocks = data_.interlocks_;
  const axisState axisStateOld = axisState_;
  hwReadyOld_ = hwReady_;

  //if(data_.status_.statusWord_.localBusy != data_.statusOld_.statusWord_.localBusy) {
  //  printf("ecmcAxisBase::preExecute(): localBusy changed state = %d\n",data_.status_.statusWord_.localBusy);
  //}

  //if(data_.status_.statusWord_.globalBusy != data_.statusOld_.statusWord_.globalBusy) {
  //  printf("ecmcAxisBase::preExecute(): globalBusy changed state = %d\n",data_.status_.statusWord_.globalBusy);
  //}
  
  interlocks.etherCatMasterInterlock = !masterOK;

  if (!masterOK && statusWord.enable) {
    setEnable(false);
  }
  data_.refreshInterlocks();

  statusWord.moving = statusWord.busy &&
                      std::abs(status.currentVelocityActual) > 0; /*|| getTraj()->getBusy();*/

  const int encoderCount = status.encoderCount;
  bool powerCoordinationNeeded = encoderMode_ != 0 ||
                                 encoderPowerBackupIndex_ >= 0 ||
                                 encoderPowerInitPending_;
  for (int i = 0; i < encoderCount; i++) {
    auto *const encoder = encArray_[i];
    const bool powerEnabled = encoder->getPowerEnable();
    if (powerEnabled) {
      bool requested = encoderPowerEnablePending_ || autoEnableRequest_ ||
        ((statusWord.enable || statusWord.enabled) && !encArray_[i]->getPowerIdleOff());
      // Keep absolute feedback powered until the post-cycle handover has read
      // BOTH encoders, aligned the counter and selected it for control.
      if (encoderPowerBackupIndex_ >= 0 && i == encoderPowerSourceIndex_ &&
          data_.control_.primaryEncIndex == i && encArray_[i]->getPowerReady() && !getErrorID())
        requested = true;
      if (encoderMode_ != 0) requested = i == encoderMode_-1 && !getErrorID();
      // Obtain real absolute feedback once before completing initial startup,
      // including when an operator selected the tracking counter explicitly.
      // Wait for valid process data before starting the power-up timeout.
      if (!status.startupFinsished && masterOK && encArray_[i]->checkAllDataValid())
        requested = true;
      encArray_[i]->requestPower(requested);
    }
    encoder->readEntries(masterOK);
    // Include one-shot synchronization left by a power cycle even if power
    // control has since been disabled. Membership is re-evaluated each cycle.
    powerCoordinationNeeded |= powerEnabled || encoder->powerPositionSyncPending();
    if (powerEnabled && encoder->getPowerError()) {
      setEnable(false);
      setErrorID(encArray_[i]->getPowerError());
    }
  }
  
  if (powerCoordinationNeeded) syncEncoderPowerPosition();
  hwReady_ = getHwReady();

  // Axis state machine
  switch (axisState_) {
  case ECMC_AXIS_STATE_STARTUP: {
    setInStartupPhase(true);
    if (statusWord.enable) {
      setEnable(false);
    }
    seq_.setGlobalBusy(true);
    statusWord.busy = true;
    status.distToStop = 0;

    bool powerStartupPending = false;
    if (!status.startupFinsished) {
      for (int i = 0; i < encoderCount; ++i)
        powerStartupPending |= encArray_[i]->getPowerEnable() &&
          !encArray_[i]->getPowerReady();
    }
    if (masterOK && !ecmcStartupMotionHeld()) {
      
      if (!hwReady_ && !powerStartupPending) {
        setErrorID(ERROR_AXIS_HW_NOT_READY);
      } else if (hwReady_) {  // auto reset error if ok
        if (getErrorID() == ERROR_AXIS_HW_NOT_READY && hwReadyOld_) {
          errorReset();
        }
      }
    }

    if (!ecmcStartupMotionHeld() && masterOK && hwReady_ && hwReadyOld_) {
      // Auto reset hardware error if starting up
      if ((getErrorID() == ERROR_AXIS_HARDWARE_STATUS_NOT_OK) &&
          data_.status_.statusWord_.instartup) {
        errorReset();
      }

      // bus ok and hw ok, startup finished
      setInStartupPhase(false);

      // only init encoders after first startup
      if (!status.startupFinsished) {
        initEncoders();
        status.currentPositionActual = getPrimEnc()->getActPos();
        setMRSync(true);
      }

      status.startupFinsished = true;
      data_.control_.positionTarget = status.currentPositionActual;
      status.currentTargetPosition = status.currentPositionActual;

      refreshAsynTargetValue();

      // start enable if needed
      if (getStartupCommandsHeld()) {
        startupEnablePending_ = true;
      } else {
        setEnable(data_.control_.enableAtStartup);
        setMRCnen(data_.control_.enableAtStartup);
      }
      axisState_ = ECMC_AXIS_STATE_DISABLED;
    }
    break;

  }
  case ECMC_AXIS_STATE_DISABLED:
    // Keep queued motion visible while enabling or waiting for the slave
    // group. Enable feedback can precede the transition out of DISABLED.
    if (!autoEnableRequest_ && !motionStartPending_) {
      seq_.setGlobalBusy(false); 
    }   
    statusWord.busy = false;
    status.distToStop = 0;

    if (data_.status_.statusWord_.enabled) {
      axisState_ = ECMC_AXIS_STATE_ENABLED;
    }

    if (!masterOK) {
      axisState_ = ECMC_AXIS_STATE_STARTUP;
    }
    break;

  case ECMC_AXIS_STATE_ENABLED:
    status.distToStop = traj_->distToStop(status.currentVelocitySetpoint);

    if (!statusWord.enabled) {
      axisState_ = ECMC_AXIS_STATE_DISABLED;
    }

    if (!masterOK) {
      axisState_ = ECMC_AXIS_STATE_STARTUP;
    }

    break;
  }
  const bool logAxisStateChange =
    (axisStateOld != axisState_) &&
    ((axisStateOld == ECMC_AXIS_STATE_STARTUP) ||
     (axisState_ == ECMC_AXIS_STATE_STARTUP));

  if (logAxisStateChange &&
      statusWord.inrealtime &&
      ecmcRtLoggerIsEnabled()) {
    if (axisState_ == ECMC_AXIS_STATE_STARTUP) {
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Axis[%d]: State change (%d->%d).\n",
                           __FILE__,
                           __FUNCTION__,
                           __LINE__,
                           status.axisId,
                           axisStateOld,
                           axisState_);
    } else {
      ecmcRtLoggerLogInfo("%s/%s:%d: INFO: Axis[%d]: State change (%d->%d).\n",
                          __FILE__,
                          __FUNCTION__,
                          __LINE__,
                          status.axisId,
                          axisStateOld,
                          axisState_);
    }
  } else if (logAxisStateChange && (axisState_ == ECMC_AXIS_STATE_STARTUP)) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Axis[%d]: State change (%d->%d).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           status.axisId,
           axisStateOld,
           axisState_);
  }

  mon_->readEntries();

  // Filter velocities from PLC source
  const double extTrajDelta =
    status.externalTrajectoryPosition -
    data_.statusOld_.externalTrajectoryPosition;
  const double extEncDelta =
    status.externalEncoderPosition -
    data_.statusOld_.externalEncoderPosition;

  // Traj
  if (enableExtTrajVeloFilter_ && extTrajVeloFilter_) {
    status.externalTrajectoryVelocity = extTrajVeloFilter_->getFiltVelo(extTrajDelta);
  } else {
    status.externalTrajectoryVelocity = extTrajDelta * invSampleTime_;
  }

  // Enc
  if (enableExtEncVeloFilter_ && extEncVeloFilter_) {
    status.externalEncoderVelocity = extEncVeloFilter_->getFiltVelo(extEncDelta);
  } else {
    status.externalEncoderVelocity = extEncDelta * invSampleTime_;
  }
}

void ecmcAxisBase::postExecute(bool masterOK) {
  auto &status = data_.status_;
  auto &statusWord = status.statusWord_;
  statusWord.busy = getBusy();
  encoderPowerIdleSM();
  if (encoderPowerEnablePending_) encoderPowerEnableSM();
  autoEnableSM();
  autoDisableSM();

  // status wordbit 9 homed
  bool homedtemp = 0;
  getAxisHomed(&homedtemp);
  statusWord.homed = homedtemp > 0;
  
  // Write encoder entries
  const int encoderCount = status.encoderCount;
  for (int i = 0; i < encoderCount; i++) {
    encArray_[i]->writeEntries();
  }

  for (int i = 0; i < encoderCount; i++) {
    auto *const encoder = encArray_[i];
    // Avoid the timing calculation when there is nothing
    // to publish. Use the same condition as refreshTouchProbeAsyn().
    if (!encoder->hasTouchProbeAsynParams()) {
      continue;
    }
    uint64_t encoderSampleTimeNs = 0;
    const bool sampleTimeValid =
      resolveAxisEncoderSampleTimeNs(encoder, &encoderSampleTimeNs);
    encoder->refreshTouchProbeAsyn(sampleTimeValid ?
                                        encoderSampleTimeNs : 0,
                                        false);
  }

  if (!getStartupCommandsHeld() && positionCompare_.isActive()) {
    ecmcEncoder *encoder = getPrimEnc();
    ecmcSignalTime signalTime;
    const bool sampleTimeValid = encoder && encoder->getSampleTiming(signalTime);
    const uint64_t encoderSampleTimeNs = signalTime.sampleTimeNs;
    const uint64_t controllerTimeNs = signalTime.controllerTimeNs;
    const double compareVelocity = status.currentVelocitySetpoint;
    double compareAcceleration = 0;
    ecmcAxisPVTSequence *pvt = seq_.getPVTObject();
    if (status.statusWord_.trajsource == ECMC_DATA_SOURCE_INTERNAL) {
      compareAcceleration =
        status.command == ECMC_CMD_MOVEPVTABS && pvt ?
          pvt->getCurrAcceleration() : traj_->getCurrentAcc();
    } else if (status.sampleTime > 0) {
      compareAcceleration =
        (status.currentVelocitySetpoint -
         data_.statusOld_.currentVelocitySetpoint) / status.sampleTime;
    }
    positionCompare_.execute(masterOK,
                             encoder->getActPosUncompensated(),
                             compareVelocity,
                             compareAcceleration,
                             encoderSampleTimeNs,
                             sampleTimeValid,
                             controllerTimeNs);
  }
  positionCompare_.refreshAsyn(false);
  
  status.cycleCounter++;

  // Update asyn parameters
  refreshStatusWd();
  data_.axAsynParams_[ECMC_ASYN_AX_SET_POS_ID]->refreshParamRT(0);
  data_.axAsynParams_[ECMC_ASYN_AX_ACT_POS_ID]->refreshParamRT(0);
  data_.axAsynParams_[ECMC_ASYN_AX_ACT_VEL_ID]->refreshParamRT(0);
  data_.axAsynParams_[ECMC_ASYN_AX_POS_ERR_ID]->refreshParamRT(0);
  data_.axAsynParams_[ECMC_ASYN_AX_STATUS_ID]->refreshParamRT(0);
  statusWord2Param_->refreshParamRT(0);
  data_.axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]->refreshParamRT(0);
  data_.axAsynParams_[ECMC_ASYN_AX_ERROR_ID]->refreshParamRT(0);
  data_.axAsynParams_[ECMC_ASYN_AX_WARNING_ID]->refreshParamRT(0);

  if (positionTargetAsyn_ != data_.control_.positionTarget) {
    refreshAsynTargetValue();
  }

  if(memcmp(&mrCmdsOld_,&mrCmds_, sizeof(mrCmdsOld_)) !=0) {
    data_.axAsynParams_[ECMC_ASYN_AX_MR_CMD_ID]->refreshParamRT(1);
  }
  memcpy(&mrCmdsOld_,&mrCmds_, sizeof(mrCmdsOld_));
  
  // Update status entry if linked
  if (statusOutputEntry_ && statusOutputEntry_->data) {
    statusOutputEntry_->writeValue(getErrorID() == 0);
  }

  // Data for last scan
  memcpy(&data_.statusOld_,&data_.status_,sizeof(data_.status_));
  memcpy(&data_.controlOld_,&data_.control_,sizeof(data_.control_));
}

axisTypes ecmcAxisBase::getAxisType() {
  return data_.status_.axisType;
}

int ecmcAxisBase::getAxisID() {
  return data_.status_.axisId;
}

void ecmcAxisBase::setReset(bool reset) {
  data_.control_.controlWord_.resetCmd = reset;
  // also reset auto enable/disable counters
  autoDisbleTimeCounter_ = 0;
  autoEnableTimeCounter_ = 0;

  if (reset) {
    if (motionStartPending_) {
      setExecute(false, true);
    }
    errorReset();

    if (getMon() != NULL) {
      getMon()->errorReset();
    }

    if (getEnc() != NULL) {
      getEnc()->errorReset();
    }

    if (getTraj() != NULL) {
      getTraj()->errorReset();
    }

    if (getSeq() != NULL) {
      getSeq()->errorReset();
    }

    if (getDrv() != NULL) {
      getDrv()->errorReset();
    }

    if (getCntrl() != NULL) {
      getCntrl()->errorReset();
    }
  }
  data_.control_.controlWord_.resetCmd = 0;
}

bool ecmcAxisBase::getReset() {
  return data_.control_.controlWord_.resetCmd;
}

int ecmcAxisBase::setAllowCmdFromPLC(bool enable) {
  allowCmdFromOtherPLC_        = enable;
  data_.control_.controlWord_.plcCmdsAllowCmd = enable;
  return 0;
}

bool ecmcAxisBase::getAllowCmdFromPLC() {
  return allowCmdFromOtherPLC_;
}

void ecmcAxisBase::setInStartupPhase(bool startup) {
  // When configuration is ready then defult to primary encoder for all encoder related calls
  if (!data_.status_.statusWord_.instartup && startup) {
    data_.control_.cfgEncIndex = data_.control_.primaryEncIndex;
  }
  data_.status_.statusWord_.instartup = startup;
}

bool ecmcAxisBase::getInStartupPhase() {
  return data_.status_.statusWord_.instartup || data_.statusOld_.statusWord_.instartup;
}

bool ecmcAxisBase::getStartupCommandsHeld() const {
  // The controller gate covers configuration/RT entry and the explicit opt-out.
  // An axis may finish its own handshake while unrelated axes are still waiting.
  return ecmcStartupCommandsHeld() &&
         !startupCommandsReleased_.load(std::memory_order_acquire);
}

const char *ecmcAxisBase::getStartupHoldReason() const {
  return getStartupCommandsHeld() ? startupHoldReason_.load(std::memory_order_acquire) : "released";
}

int ecmcAxisBase::getStartupBlockingAxis() const {
  return getStartupCommandsHeld() ? startupBlockingAxis_.load(std::memory_order_acquire) : -1;
}

int ecmcAxisBase::getStartupBlockingGroup() const {
  return getStartupCommandsHeld() ? startupBlockingGroup_.load(std::memory_order_acquire) : -1;
}

int ecmcAxisBase::setEncDataSourceType(dataSource refSource) {
  if ((encoderPowerBackupIndex_ >= 0 || encoderMode_ != 0) && refSource != ECMC_DATA_SOURCE_INTERNAL)
    return ERROR_ENC_POWER_CONFIG;
  if (refSource == data_.status_.statusWord_.encsource) return 0;

  if (!data_.control_.allowSourceChangeWhenEnabled) {
    if (getEnable() && (refSource != ECMC_DATA_SOURCE_INTERNAL)) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED);
    }
  }

  // If realtime: Ensure that ethercat enty for actual position is linked
  if ((refSource == ECMC_DATA_SOURCE_INTERNAL) && data_.status_.statusWord_.inrealtime) {
    int errorCode = getEnc()->validateData(
      ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION);

    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  }

  data_.status_.statusWord_.encsource = refSource;
  data_.control_.controlWord_.encSourceCmd = refSource;
  data_.axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]->refreshParamRT(1);
  data_.axAsynParams_[ECMC_ASYN_AX_STATUS_ID]->refreshParamRT(1);
  
  return 0;
}

int ecmcAxisBase::setRealTimeStarted(bool realtime) {
  if (realtime) {
    initControlWord();
  }
  data_.status_.statusWord_.inrealtime = realtime;
  return 0;
}

int ecmcAxisBase::getRealTimeStarted() {
  return data_.status_.statusWord_.inrealtime;
}

bool ecmcAxisBase::getError() {
  int error = ecmcAxisBase::getErrorID();

  if (error) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }
  return ecmcError::getError();
}

int ecmcAxisBase::getErrorID() {
  if (ecmcError::getError()) {
    return ecmcError::getErrorID();
  }

  // The below contains all errors from the "sub objects"
  if (data_.status_.errorCode) {
    return setErrorID(data_.status_.errorCode);
  }
  return 0;
}

int ecmcAxisBase::setErrorID(int errorID) {
  if (errorID &&
      data_.status_.statusWord_.inrealtime &&
      ecmcRtLoggerIsEnabled() &&
      (errorID != ecmcError::getErrorID())) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Axis[%d]: %s (0x%x).\n",
                         __FILE__,
                         __FUNCTION__,
                         __LINE__,
                         data_.status_.axisId,
                         ecmcError::convertErrorIdToString(errorID),
                         errorID);
  }

  return ecmcError::setErrorID(errorID);
}

int ecmcAxisBase::setErrorID(int               errorID,
                             ecmcAlarmSeverity severity) {
  if (errorID &&
      data_.status_.statusWord_.inrealtime &&
      ecmcRtLoggerIsEnabled() &&
      (errorID != ecmcError::getErrorID())) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Axis[%d]: %s (0x%x).\n",
                         __FILE__,
                         __FUNCTION__,
                         __LINE__,
                         data_.status_.axisId,
                         ecmcError::convertErrorIdToString(errorID),
                         errorID);
    return ecmcError::setErrorID(errorID, severity);
  }

  return ecmcError::setErrorID(errorID, severity);
}

int ecmcAxisBase::setErrorID(const char *fileName,
                             const char *functionName,
                             int         lineNumber,
                             int         errorID) {
  if (errorID &&
      data_.status_.statusWord_.inrealtime &&
      ecmcRtLoggerIsEnabled() &&
      (errorID != ecmcError::getErrorID())) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: %s (0x%x).\n",
                         fileName,
                         functionName,
                         lineNumber,
                         ecmcError::convertErrorIdToString(errorID),
                         errorID);
    return ecmcError::setErrorID(errorID);
  }

  return ecmcError::setErrorID(fileName, functionName, lineNumber, errorID);
}

int ecmcAxisBase::setErrorID(const char       *fileName,
                             const char       *functionName,
                             int               lineNumber,
                             int               errorID,
                             ecmcAlarmSeverity severity) {
  if (errorID &&
      data_.status_.statusWord_.inrealtime &&
      ecmcRtLoggerIsEnabled() &&
      (errorID != ecmcError::getErrorID())) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: %s (0x%x).\n",
                         fileName,
                         functionName,
                         lineNumber,
                         ecmcError::convertErrorIdToString(errorID),
                         errorID);
    return ecmcError::setErrorID(errorID, severity);
  }

  return ecmcError::setErrorID(fileName,
                               functionName,
                               lineNumber,
                               errorID,
                               severity);
}

bool ecmcAxisBase::shouldSyncSetpointToActual() {
  if (!mon_ || !mon_->getEnableAtTargetMon()) {
    return true;
  }

  const double positionError = std::abs(data_.status_.currentPositionActual -
                                        data_.status_.currentPositionSetpoint);
  return positionError > mon_->getAtTargetTol();
}

int ecmcAxisBase::setEnableLocal(bool enable) {
  if (enable && !data_.status_.statusWord_.enable) {
    // A new enable request starts a new idle period. Drive-enabled feedback
    // can remain high during brake closing; without this reset an expired
    // timer disables every retry before the drive can finish re-enabling.
    autoDisbleTimeCounter_ = 0;
    autoDisableAtTargetLatched_ = false;
    if(enableAutoResetError_) errorReset();
    extEncVeloFilter_->initFilter(0);  // init to 0 vel
    extTrajVeloFilter_->initFilter(0);  // init to 0 vel    
    if( (!data_.status_.statusWord_.attarget) || (!firstEnableDone_) || (!mon_->getEnableAtTargetMon())) {
      traj_->setStartPos(data_.status_.currentPositionActual);
      traj_->setCurrentPosSet(data_.status_.currentPositionActual);
      traj_->setTargetPos(data_.status_.currentPositionActual);
      //setTargetPos(data_.status_.currentPositionActual);      
      data_.status_.currentPositionSetpoint =
        data_.status_.currentPositionActual;
      data_.status_.currentVelocitySetpoint = 0;
      firstEnableDone_                      = true;
    }
  }
  if(!enable) {
    setMRStop(1);
  }
  //setMRCnen(enable);
  traj_->setEnable(enable);

  // reset axis error if ERROR_AXIS_NOT_ENABLED or ERROR_AXIS_SAFETY_IL_ACTIVE when try to enable
  int errid = getErrorID();
  if((errid == ERROR_AXIS_NOT_ENABLED  || 
      errid == ERROR_AXIS_SAFETY_IL_ACTIVE) && enable) {
    errorReset();
  }

  return 0;
}

void ecmcAxisBase::errorReset() {
  const bool syncInternalSetpointAfterPosLag =
    getErrorID() == ERROR_MON_MAX_POSITION_LAG_EXCEEDED &&
    data_.status_.statusWord_.trajsource == ECMC_DATA_SOURCE_INTERNAL &&
    !data_.status_.statusWord_.localBusy;

  if (disableAxisAtErrorReset_ && getError()) {
    setEnable(0);
  }

  // Monitor
  ecmcMonitor *mon = getMon();

  if (mon) {
    mon->errorReset();
  }

  // Encoders
  for (int i = 0; i < data_.status_.encoderCount; i++) {
    encArray_[i]->errorReset();
  }

  // Drive
  ecmcDriveBase *drv = getDrv();

  if (drv) {
    drv->errorReset();
  }

  // Trajectory
  ecmcTrajectoryBase *traj = getTraj();

  if (traj) {
    traj->errorReset();
  }

  // Controller
  ecmcPIDController *cntrl = getCntrl();

  if (cntrl) {
    cntrl->errorReset();
  }

  // Sequencer
  ecmcAxisSequencer *seq = getSeq();

  if (seq) {
    seq->errorReset();
  }

  if (syncInternalSetpointAfterPosLag && traj) {
    const double actualPosition = data_.status_.currentPositionActual;
    traj->setStartPos(actualPosition);
    traj->setCurrentPosSet(actualPosition);
    traj->setTargetPos(actualPosition);
    data_.status_.currentPositionSetpoint = actualPosition;
    data_.status_.currentTargetPosition = actualPosition;
    data_.status_.currentVelocitySetpoint = 0;
    data_.control_.positionTarget = actualPosition;
    refreshAsynTargetValue();
  }

  ecmcError::errorReset();
}

int ecmcAxisBase::validateBase() {
  if (encoderPowerBackupIndex_ >= 0 &&
      validateEncoderPowerBackup(encoderPowerSourceIndex_, encoderPowerBackupIndex_))
    return setErrorID(ERROR_ENC_POWER_CONFIG);
  for (int i = 0; i < data_.status_.encoderCount; ++i)
    if (encArray_[i]->getPowerEnable() &&
        (!mon_->getEnableAtTargetMon() || mon_->getAtTargetTol() <= 0))
      return setErrorID(ERROR_ENC_POWER_CONFIG);
  return 0;
}

int ecmcAxisBase::getPosAct(double *pos) {
  *pos = data_.status_.currentPositionActual;
  return 0;
}

int ecmcAxisBase::getVelAct(double *vel) {
  *vel = data_.status_.currentVelocityActual;
  return 0;
}

int ecmcAxisBase::getPosSet(double *pos) {
  *pos = data_.status_.currentPositionSetpoint;
  return 0;
}

int ecmcAxisBase::getVelSet(double *vel) {  
  *vel = data_.status_.currentVelocitySetpoint;
  return 0;
}

ecmcEncoder * ecmcAxisBase::getEnc() {
  return encArray_[data_.control_.primaryEncIndex];
}

ecmcEncoder * ecmcAxisBase::getEnc(int encIndex, int *error) {
  *error = 0;

  if (encIndex >= ECMC_MAX_ENCODERS || encIndex < 0) {
    *error = ERROR_AXIS_ENC_COUNT_OUT_OF_RANGE;
    return NULL;
  }

  if (encArray_[encIndex] == NULL) {
    *error = ERROR_AXIS_ENC_COUNT_OUT_OF_RANGE;
    return NULL;
  }

  return encArray_[encIndex];
}

ecmcEncoder * ecmcAxisBase::getConfigEnc() {
  return encArray_[data_.control_.cfgEncIndex];
}

// ecmcEncoder * ecmcAxisBase::getHomeEnc() {
//  return encArray_[data_.control_.homeEncIndex];
// }

ecmcEncoder * ecmcAxisBase::getPrimEnc() {
  return encArray_[data_.control_.primaryEncIndex];
}

ecmcEncoder * ecmcAxisBase::getCSPEnc() {
  if(data_.control_.cspDrvEncIndex < 0) {
    return encArray_[data_.control_.primaryEncIndex];
  }
  return encArray_[data_.control_.cspDrvEncIndex];
}

int ecmcAxisBase::setTouchProbeArm(int encoderIndex, bool arm) {
  int error = 0;
  ecmcEncoder *encoder = getEnc(encoderIndex, &error);
  if (!encoder) {
    return error;
  }
  // Configuration-time requests precede the normal runtime validation pass.
  if (appModeStat == ECMC_MODE_CONFIG) {
    error = encoder->validate();
    if (error) return error;
  }
  if (!encoder->getTouchProbeFuncEnabled() && !encoder->getLatchFuncEnabled()) {
    return ERROR_ENC_ENTRY_NULL;
  }
  if (arm) {
    encoder->setTouchProbeAutoRearm(true);
    if (encoder->getTouchProbeFuncEnabled()) {
      encoder->setTouchProbeControlEnabled(true);
      encoder->setArmTouchProbe(true);
    } else {
      encoder->setLatchControlEnabled(true);
      encoder->setArmLatch(true);
    }
  } else {
    encoder->setTouchProbeAutoRearm(false);
    if (encoder->getTouchProbeFuncEnabled()) {
      encoder->setTouchProbeControlEnabled(false);
    } else {
      encoder->setLatchControlEnabled(false);
    }
  }
  return 0;
}

ecmcEcTimedValue<double> ecmcAxisBase::getTouchProbeResult(
  int encoderIndex,
  uint64_t nearbyDcTimeNs,
  int *error) {
  ecmcEcTimedValue<double> result;
  if (!error) {
    return result;
  }
  ecmcEncoder *encoder = getEnc(encoderIndex, error);
  if (!encoder) {
    return result;
  }
  uint64_t resolvedNearbyDcTimeNs = nearbyDcTimeNs;
  uint64_t sampleTimeNs = 0;
  if (resolveAxisEncoderSampleTimeNs(encoder, &sampleTimeNs)) {
    resolvedNearbyDcTimeNs = sampleTimeNs;
  }
  return encoder->getTouchProbeTimedValue(resolvedNearbyDcTimeNs);
}

ecmcPositionCompare* ecmcAxisBase::getPositionCompare() {
  return &positionCompare_;
}

int ecmcAxisBase::createPositionCompareAsynParams() {
  return positionCompare_.createAsynParams(asynPortDriver_,
                                           data_.status_.axisId);
}

int ecmcAxisBase::armPositionCompare(double target,
                                     int direction,
                                     uint64_t outputValue) {
  return positionCompare_.arm(target, direction, outputValue);
}

int ecmcAxisBase::cancelPositionCompare() {
  return positionCompare_.cancel();
}

ecmcPositionCompareStatus ecmcAxisBase::getPositionCompareStatus() {
  return positionCompare_.getStatus();
}

ecmcTrajectoryBase * ecmcAxisBase::getTraj() {
  return traj_;
}

ecmcMonitor * ecmcAxisBase::getMon() {
  return mon_;
}

ecmcAxisSequencer * ecmcAxisBase::getSeq() {
  return &seq_;
}

int ecmcAxisBase::getAxisHomed(bool *homed) {
  if (data_.status_.statusWord_.encsource == ECMC_DATA_SOURCE_EXTERNAL) {
    *homed = 1;
  } else {
    *homed = encArray_[data_.control_.primaryEncIndex]->getHomed();
  }
  return 0;
}

int ecmcAxisBase::setAxisHomed(bool homed) {
  encArray_[data_.control_.primaryEncIndex]->setHomed(homed);
  return 0;
}

int ecmcAxisBase::getEncScaleNum(double *scale) {
  *scale = encArray_[data_.control_.cfgEncIndex]->getScaleNum();
  return 0;
}

int ecmcAxisBase::setEncScaleNum(double scale) {
  encArray_[data_.control_.cfgEncIndex]->setScaleNum(scale);
  return 0;
}

int ecmcAxisBase::getEncScaleDenom(double *scale) {
  *scale = encArray_[data_.control_.cfgEncIndex]->getScaleDenom();
  return 0;
}

int ecmcAxisBase::setEncScaleDenom(double scale) {
  encArray_[data_.control_.cfgEncIndex]->setScaleDenom(scale);
  return 0;
}

int ecmcAxisBase::getEncPosRaw(int64_t *rawPos) {
  *rawPos = encArray_[data_.control_.primaryEncIndex]->getRawPosMultiTurn();
  return 0;
}

int ecmcAxisBase::setCommand(motionCommandTypes command) {
  seq_.setCommand(command);
  data_.control_.command = command;
  data_.axAsynParams_[ECMC_ASYN_AX_COMMAND_ID]->refreshParamRT(1);
  return 0;
}

int ecmcAxisBase::setCmdData(int cmdData) {
  seq_.setCmdData(cmdData);
  data_.control_.cmdData = cmdData;
  data_.axAsynParams_[ECMC_ASYN_AX_CMDDATA_ID]->refreshParamRT(1);
  return 0;
}

unsigned int ecmcAxisBase::getMotionCommandMotorRecordRequestCounter() const {
  return motionCommandMotorRecordRequestCounter_;
}

unsigned int ecmcAxisBase::getMotionCommandRequestCounter() const {
  return motionCommandRequestCounter_;
}

unsigned int ecmcAxisBase::getMotionCommandExecuteCounter() const {
  return motionCommandExecuteCounter_;
}

void ecmcAxisBase::publishMotionCommandCounters() {
  ecmcRtLoggerPortDriverSetAxisCommandCounters(data_.status_.axisId,
                                               motionCommandMotorRecordRequestCounter_,
                                               motionCommandRequestCounter_,
                                               motionCommandExecuteCounter_);
}

void ecmcAxisBase::bumpMotionCommandMotorRecordRequestCounter() {
  ++motionCommandMotorRecordRequestCounter_;
  publishMotionCommandCounters();
}

void ecmcAxisBase::bumpMotionCommandRequestCounter() {
  ++motionCommandRequestCounter_;
  publishMotionCommandCounters();
}

void ecmcAxisBase::bumpMotionCommandExecuteCounter() {
  ++motionCommandExecuteCounter_;
  publishMotionCommandCounters();
}

motionCommandTypes ecmcAxisBase::getCommand() {
  return seq_.getCommand();
}

int ecmcAxisBase::getCmdData() {
  return seq_.getCmdData();
}

int ecmcAxisBase::slowExecute() {
  return 0;
}

void ecmcAxisBase::printAxisStatus() {
  if (memcmp(&data_.status_, &data_.statusOld_,
             sizeof(data_.statusOld_)) == 0) {
    return;  // Printout on change
  }

  // Only print header once per 25 status lines
  if (printHeaderCounter_ <= 0) {
    ecmcRtLoggerLogInfo(
      "ecmc::  Ax     PosSet     PosAct     PosErr    PosTarg   DistLeft    CntrOut   VelFFSet     VelAct   VelFFRaw VelRaw  Error Co CD St IL LI TS ES En Ex Bu Ta Hd L- L+ Ho\n");
    printHeaderCounter_ = 25;
  }
  printHeaderCounter_--;
  
  ecmcRtLoggerLogInfo(
    "ecmc:: %3d %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %6lf %6x %2d %2d %2d %2d %2d %2d %2d %1d%1d %2d %2d %2d %2d %2d %2d %2d\n",
    data_.status_.axisId,
    data_.status_.currentPositionSetpoint,
    data_.status_.currentPositionActual,
    data_.status_.cntrlError,
    data_.status_.currentTargetPosition,
    data_.status_.currentTargetPosition- data_.status_.currentPositionActual,
    data_.status_.cntrlOutput,
    data_.status_.currentVelocitySetpoint,
    data_.status_.currentVelocityActual,
    data_.status_.currentvelocityFFRaw,
    (double)data_.status_.currentVelocitySetpointRaw,
    getErrorID(),
    data_.status_.command,
    data_.status_.cmdData,
    data_.status_.statusWord2_.seqstate,
    (int)data_.interlocks_.interlockStatus,
    data_.status_.statusWord2_.lastilock,
    data_.status_.statusWord_.trajsource,
    data_.status_.statusWord_.encsource,
    data_.status_.statusWord_.enable,
    data_.status_.statusWord_.enabled,
    data_.status_.statusWord_.execute,
    data_.status_.statusWord_.busy,
    data_.status_.statusWord_.attarget,
    data_.status_.statusWord_.homed,
    data_.status_.statusWord_.limitbwd,
    data_.status_.statusWord_.limitfwd,
    data_.status_.statusWord_.homeswitch);
}

// Ignore busy so that PVT can run (PVT controller will check that traj is not busy)
int ecmcAxisBase::setExecute(bool execute) {
  // Reject before auto-enable can queue a move for later execution.
  if (execute && getStartupCommandsHeld()) {
    return ERROR_AXIS_BUSY;
  }
  // Auto enable if needed  (except for homing seq 15)
  //printf("ecmcAxisBase::setExecute(): 1 global busy %d, local busy %d\n", data_.status_.statusWord_.globalBusy, data_.status_.statusWord_.localBusy);
  if(!getEnable() && execute && autoEnableTimoutS_ > 0 && !((data_.control_.cmdData == ECMC_SEQ_HOME_SET_POS) &&
          (data_.control_.command == ECMC_CMD_HOMING))) {

    getSeq()->setGlobalBusy(true);
    autoEnableTimeCounter_ = 0;
    autoEnableRequest_ = true && enableAutoEnable_;
    // autoEnableSM will setExecute later when axis is enabled
    return 0;
  }
  //printf("ecmcAxisBase::setExecute(): 2 global busy %d, local busy %d\n", data_.status_.statusWord_.globalBusy, data_.status_.statusWord_.localBusy);
  return setExecute(execute, false);
}

int ecmcAxisBase::setExecute(bool execute, bool ignoreBusy) {
  if (execute && getStartupCommandsHeld()) {
    return ERROR_AXIS_BUSY;
  }
  // Keep coordination in command handling: do not start the sequencer or
  // trajectory until the state machine has prepared the slave group.
  // Encoder wake-up is independent of drive auto-enable: an amplifier may
  // already be holding torque while its absolute feedback is powered down.
  if (execute && getEnable() && !encoderPowerReady())
    return prepareEncoderPowerMotion();
  if (execute && motionStartPermissive_ && !*motionStartPermissive_ &&
      getEnable() && !getExecute() &&
      (ignoreBusy || !data_.status_.statusWord_.localBusy) &&
      !(data_.control_.command == ECMC_CMD_HOMING &&
        data_.control_.cmdData == ECMC_SEQ_HOME_SET_POS)) {
    motionStartPending_ = true;
    autoEnableTimeCounter_ = 0;
    getSeq()->setGlobalBusy(true);
    data_.control_.controlWord_.executeCmd = 0;
    data_.axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]->refreshParamRT(1);
    return 0;
  }
  if (execute && data_.control_.command != ECMC_CMD_NOCMD) {
    encoderPowerMoveCompleted_ = false;
    ignoredMotionMessage_ = ECMC_CMD_NOCMD;
    bumpMotionCommandExecuteCounter();
  }

  // Internal trajectory source
  if (data_.status_.statusWord_.trajsource == ECMC_DATA_SOURCE_INTERNAL) {
    // Allow direct homing without enable
    if (execute && !getEnable() &&
        !((data_.control_.cmdData == ECMC_SEQ_HOME_SET_POS) &&
          (data_.control_.command == ECMC_CMD_HOMING))) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_AXIS_NOT_ENABLED);
    }

    if(!ignoreBusy) {
      if (execute && !data_.statusOld_.statusWord_.execute && data_.status_.statusWord_.localBusy) {
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_AXIS_BUSY);
      }
    }

    const int warningCodeOld = data_.status_.warningCode;
    // PLC wrappers enter here directly, bypassing the motor-record command
    // update path. Clear only an obsolete soft-limit interlock for a valid
    // target before the sequencer checks whether its warning can be cleared.
    if (execute) {
      clearStaleSoftLimitInterlockForValidTarget();
    }
    int error = seq_.setExecute(execute);
    if (data_.status_.warningCode != warningCodeOld) {
      data_.axAsynParams_[ECMC_ASYN_AX_WARNING_ID]->refreshParamRT(1);
    }
    //printf("ecmcAxisBase::setExecute(): 3 global busy %d, local busy %d\n", data_.status_.statusWord_.globalBusy, data_.status_.statusWord_.localBusy);

    if (error) {      
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }
  }

  if(!execute) {
    if (autoEnableRequest_ && encoderPowerEnablePending_) {
      encoderPowerEnablePending_ = false;
      if (!getEnable()) data_.control_.controlWord_.enableCmd = false;
      getSeq()->setGlobalBusy(false);
    }
    autoEnableRequest_ = false;
    if (motionStartPending_) {
      motionStartPending_ = false;
      getSeq()->setGlobalBusy(false);
    }
  }

  // Always reset 
  data_.control_.controlWord_.executeCmd = 0;
  data_.axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]->refreshParamRT(1);
  return 0;
}

bool ecmcAxisBase::getExecute() {
  if (data_.status_.statusWord_.trajsource == ECMC_DATA_SOURCE_INTERNAL) {
    return seq_.getExecute();
  } else {
    return true;
  }
}

bool ecmcAxisBase::getBusy() {
  return seq_.getBusy();
}

bool ecmcAxisBase::getTrajBusy() {
  return getTraj()->getBusy();
}

int ecmcAxisBase::getCycleCounter() {
  /// Free-running 32-bit watchdog counter.
  return static_cast<int32_t>(data_.status_.cycleCounter);
}

bool ecmcAxisBase::getEnable() {
  return data_.status_.statusWord_.enable;
}

bool ecmcAxisBase::getEnabled() {
  return data_.status_.statusWord_.enabled && data_.status_.statusWord_.enable;
}

bool ecmcAxisBase::getEnabledOnly() {
  return data_.status_.statusWord_.enabled;
}

int ecmcAxisBase::initAsyn() {
  if (asynPortDriver_ == NULL) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Axis[%d]: AsynPortDriver object is NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_.status_.axisId,
           ERROR_AXIS_ASYN_PORT_OBJ_NULL);
    return ERROR_AXIS_ASYN_PORT_OBJ_NULL;
  }

  ecmcAsynDataItem *paramTemp = NULL;
  int errorCode               = 0;

  // Act pos
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_ACT_POS_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(data_.status_.currentPositionActual),
                              sizeof(data_.status_.currentPositionActual),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_ACT_POS_ID] = paramTemp;

  // Act vel
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_ENC_ACT_VEL_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(data_.status_.currentVelocityActual),
                              sizeof(data_.status_.currentVelocityActual),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_ACT_VEL_ID] = paramTemp;

  // Set encoder primary index for control
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_ENC_ID_CMD_NAME,
                              asynParamInt32,
                              ECMC_EC_S32,
                              (uint8_t *)&(encPrimIndexAsyn_),
                              sizeof(encPrimIndexAsyn_),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWritePrimEncCtrlId, this); // Access to this axis
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_ENC_ID_CMD_ID] = paramTemp;

  // Set pos
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_SET_POS_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(data_.status_.currentPositionSetpoint),
                              sizeof(data_.status_.currentPositionSetpoint),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_SET_POS_ID] = paramTemp;

  // Pos Error
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_POS_ERR_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(data_.status_.cntrlError),
                              sizeof(data_.status_.cntrlError),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_POS_ERR_ID] = paramTemp;

   // Status word
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_STATUS_NAME,
                              asynParamInt32,
                              ECMC_EC_U32,
                              (uint8_t *)&(data_.status_.statusWord_),
                              sizeof(data_.status_.statusWord_),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_STATUS_ID] = paramTemp;

  // Control structure binary
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_CONTROL_BIN_NAME,
                              asynParamInt32,
                              ECMC_EC_S32,
                              (uint8_t *)&(data_.control_.controlWord_),
                              sizeof(data_.control_.controlWord_),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteCmd, this); // Access to this axis
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID] = paramTemp;

  // Target Velo
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_TARG_VELO_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(data_.control_.velocityTarget),
                              sizeof(data_.control_.velocityTarget),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteTargetVelo, this); // Access to this axis
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_TARG_VELO_ID] = paramTemp;

  // Target Pos
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_TARG_POS_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(positionTargetAsyn_),
                              sizeof(positionTargetAsyn_),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteTargetPos, this); // Access to this axis
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_TARG_POS_ID] = paramTemp;

  errorCode = createAsynParam(ECMC_AX_STR "%d.status2",
                              asynParamInt32,
                              ECMC_EC_U32,
                              (uint8_t *)&data_.status_.statusWord2_,
                              sizeof(data_.status_.statusWord2_),
                              &statusWord2Param_);
  if (errorCode) return errorCode;
  statusWord2Param_->setAllowWriteToEcmc(false);
  statusWord2Param_->refreshParam(1);

  // Acceleration
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_ACC_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(data_.control_.accelerationTarget),
                              sizeof( data_.control_.accelerationTarget),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteAcc, this); // Access to this axis
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_ACC_ID] = paramTemp;

  // Deceleration
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_DEC_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(data_.control_.decelerationTarget),
                              sizeof(data_.control_.decelerationTarget),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteDec, this); // Access to this axis
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_DEC_ID] = paramTemp;

  // Set/reference encoder
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_SET_ENC_POS_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(setEncoderPos_),
                              sizeof(setEncoderPos_),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }

  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteSetEncPos, this); // Access to this axis
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_SET_ENC_POS_ID] = paramTemp;

  // Command
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_COMMAND_NAME,
                              asynParamInt32,
                              ECMC_EC_S32,
                              (uint8_t *)&(data_.control_.command),
                              sizeof(data_.control_.command),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteCommand, this); // Access to this axis
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_COMMAND_ID] = paramTemp;

  // CmdData
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_CMDDATA_NAME,
                              asynParamInt32,
                              ECMC_EC_S32,
                              (uint8_t *)&(data_.control_.cmdData),
                              sizeof(data_.control_.cmdData),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteCmdData, this); // Access to this axis
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_CMDDATA_ID] = paramTemp;

  // Error
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_ERROR_NAME,
                              asynParamInt32,
                              ECMC_EC_S32,
                              (uint8_t *)&(data_.status_.errorCode),
                              sizeof(data_.status_.errorCode),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }

  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_ERROR_ID] = paramTemp;

  // Warning
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_WARNING_NAME,
                              asynParamInt32,
                              ECMC_EC_S32,
                              (uint8_t *)&(data_.status_.warningCode),
                              sizeof(data_.status_.warningCode),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }

  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_WARNING_ID] = paramTemp;

  // MR commands
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_MR_CMD_NAME,
                              asynParamInt32,
                              ECMC_EC_U32,
                              (uint8_t *)&(mrCmds_),
                              sizeof(mrCmds_),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_MR_CMD_ID] = paramTemp;

  // Tweak value
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_TWEAK_VALUE_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(data_.control_.tweakValue),
                              sizeof(data_.control_.tweakValue),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteSetTweakValue, this); // Access to this axis
  paramTemp->refreshParam(1);
  data_.axAsynParams_[ECMC_ASYN_AX_TWEAK_VALUE_ID] = paramTemp;
  
  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST,
                                      ECMC_ASYN_DEFAULT_ADDR);
  return 0;
}

int ecmcAxisBase::getAxisDebugInfoData(char *buffer,
                                       int   bufferByteSize,
                                       int  *bytesUsed) {

  // (Ax,PosSet,PosAct,PosErr,PosTarg,DistLeft,CntrOut,VelFFSet,VelAct,VelFFRaw,VelRaw,CycleCounter,Error,Co,CD,St,IL,TS,ES,En,Ena,Ex,Bu,Ta,L-,L+,Ho");
  int ret = snprintf(buffer,
                     bufferByteSize,
                     "%d,%lf,%lf,%lf,%lf,%lf,%" PRId64",%lf,%lf,%lf,%lf,%" PRId64",%d,%x,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                     data_.status_.axisId,
                     data_.status_.currentPositionSetpoint,
                     data_.status_.currentPositionActual,
                     data_.status_.cntrlError,
                     data_.status_.currentTargetPosition,
                     data_.status_.currentTargetPosition - data_.status_.currentPositionActual,
                     data_.status_.currentPositionActualRaw,
                     data_.status_.cntrlOutput,
                     data_.status_.currentVelocitySetpoint,
                     data_.status_.currentVelocityActual,
                     data_.status_.currentvelocityFFRaw,
                     data_.status_.currentVelocitySetpointRaw,
                     static_cast<int32_t>(data_.status_.cycleCounter),
                     data_.status_.errorCode,
                     (int)data_.status_.command,
                     data_.status_.cmdData,
                     data_.status_.statusWord2_.seqstate,
                     (int)data_.interlocks_.interlockStatus,
                     data_.status_.statusWord2_.lastilock,
                     data_.status_.statusWord_.trajsource,
                     data_.status_.statusWord_.encsource,
                     data_.status_.statusWord_.enable,
                     data_.status_.statusWord_.enabled,
                     data_.status_.statusWord_.execute,
                     data_.status_.statusWord_.busy,
                     data_.status_.statusWord_.attarget,
                     data_.status_.statusWord_.homed,
                     data_.status_.statusWord_.limitbwd,
                     data_.status_.statusWord_.limitfwd,
                     data_.status_.statusWord_.homeswitch);

  if ((ret >= bufferByteSize) || (ret <= 0)) {
    *bytesUsed = 0;
    return ERROR_AXIS_PRINT_TO_BUFFER_FAIL;
  }
  *bytesUsed = ret;
  return 0;
}

int ecmcAxisBase::setEcStatusOutputEntry(ecmcEcEntry *entry) {
  return setStatusOutputData(entry ? entry->getDataAccess() : nullptr);
}

int ecmcAxisBase::setStatusOutputData(std::shared_ptr<ecmcDataAccess> entry) {
  statusOutputEntry_ = std::move(entry);
  return 0;
}

motionDirection ecmcAxisBase::getAxisSetDirection() {
  if (!data_.status_.statusWord_.enabled) {
    return ECMC_DIR_STANDSTILL;
  }

  // Transform or internal trajectory
  if (data_.status_.statusWord_.trajsource == ECMC_DATA_SOURCE_INTERNAL) {
    return getTraj()->getCurrSetDir();
  } else {
    if (data_.status_.currentPositionSetpoint <
        data_.statusOld_.currentPositionSetpoint) {
      return ECMC_DIR_BACKWARD;
    } else if (data_.status_.currentPositionSetpoint >
               data_.statusOld_.currentPositionSetpoint) {
      return ECMC_DIR_FORWARD;
    } else {
      return ECMC_DIR_STANDSTILL;
    }
  }
  return ECMC_DIR_STANDSTILL;
}

int ecmcAxisBase::getBlockCom() {
  return data_.control_.controlWord_.blockCom;
}

int ecmcAxisBase::setBlockCom(int block) {
  data_.control_.controlWord_.blockCom = block > 0;
  data_.status_.statusWord_.blocked    = getBlocked();
  if (data_.axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]) {
    data_.axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]->refreshParamRT(1);
  }
  return 0;
}

int ecmcAxisBase::setExternalCommandBlockedError() {
  ecmcRtLoggerLogError("%s/%s:%d: ERROR: Axis[%d]: External commands blocked (0x%x).\n",
         __FILE__,
         __FUNCTION__,
         __LINE__,
         data_.status_.axisId,
         ERROR_MAIN_AXIS_EXTERNAL_COM_DISABLED);
  return setErrorID(__FILE__,
                    __FUNCTION__,
                    __LINE__,
                    ERROR_MAIN_AXIS_EXTERNAL_COM_DISABLED);
}

int ecmcAxisBase::setMasterSlaveCommandBlockedError() {
  ecmcRtLoggerLogError(
    "%s/%s:%d: ERROR: Axis[%d]: Blocked (master/slave lock) (0x%x).\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.status_.axisId,
    ERROR_AXIS_BLOCKED);
  return ERROR_AXIS_BLOCKED;
}

int ecmcAxisBase::setModRange(double mod) {
  // Must be same mod factor in traj and enc
  if (mod < 0) {
    ecmcRtLoggerLogError(
      "ERROR: Axis[%d]: Modulo factor out of range. Must be a positive value (0x%x).\n",
      data_.status_.axisId,
      ERROR_AXIS_MODULO_OUT_OF_RANGE);

    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_MODULO_OUT_OF_RANGE);
  }
  data_.control_.moduloRange = mod;
  return 0;
}

double ecmcAxisBase::getModRange() {
  return data_.control_.moduloRange;
}

int ecmcAxisBase::setModType(int type) {
  if ((type < 0) || (type >= ECMC_MOD_MOTION_MAX)) {
    ecmcRtLoggerLogError(
      "ERROR: Axis[%d]: Modulo type out of range (0x%x).\n",
      data_.status_.axisId, ERROR_AXIS_MODULO_TYPE_OUT_OF_RANGE);

    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_MODULO_TYPE_OUT_OF_RANGE);
  }

  data_.control_.moduloType = (ecmcMotionModType)type;
  return 0;
}

int ecmcAxisBase::getModType() {
  return (int)data_.control_.moduloType;
}

int ecmcAxisBase::setAutoResetError(bool enable) {
  enableAutoResetError_ = enable;
  return 0;
}

int ecmcAxisBase::getCntrlError(double *error) {
  *error = data_.status_.cntrlError;
  return 0;
}

void ecmcAxisBase::refreshStatusWd() {
  if (ignoredMotionMessage_ != ECMC_CMD_NOCMD &&
      !motionCommandDisabled(ignoredMotionMessage_)) ignoredMotionMessage_ = ECMC_CMD_NOCMD;
  // No atomics, formatting, or logger calls on the unchanged RT path.
  if (!diagnosticTextPublished_ || diagnosticTextError_ != data_.status_.errorCode ||
      diagnosticTextWarning_ != data_.status_.warningCode ||
      diagnosticTextInterlock_ != data_.interlocks_.lastActiveInterlock ||
      diagnosticTextIgnored_ != ignoredMotionMessage_) {
    diagnosticTextError_ = data_.status_.errorCode;
    diagnosticTextWarning_ = data_.status_.warningCode;
    diagnosticTextInterlock_ = data_.interlocks_.lastActiveInterlock;
    diagnosticTextIgnored_ = ignoredMotionMessage_;
    ecmcRtLoggerPortDriverSetAxisStatusText(data_.status_.axisId,
      diagnosticTextError_, diagnosticTextWarning_, diagnosticTextInterlock_, diagnosticTextIgnored_);
    diagnosticTextPublished_ = true;
  }

  // bit 0 enable
  data_.status_.statusWord_.enable = getEnable() > 0;

  // bit 1 enabled
  data_.status_.statusWord_.enabled = getEnabled() > 0;

  // bit 2 execute
  data_.status_.statusWord_.execute = seq_.getExecute() > 0;

  // bit 9 homed
  bool homedtemp = 0;
  getAxisHomed(&homedtemp);
  data_.status_.statusWord_.homed = homedtemp > 0;
  
  // bit 13 Allow plc commands
  data_.status_.statusWord_.plccmdallowed = allowCmdFromOtherPLC_ > 0;

  // bit 14 softlimfwdena
  data_.status_.statusWord_.softlimfwdena =
    data_.control_.controlWord_.enableSoftLimitFwd > 0;

  // bit 15 softlimbwdena
  data_.status_.statusWord_.softlimbwdena =
    data_.control_.controlWord_.enableSoftLimitBwd > 0;

  // bit 16 inStartupPhase
  data_.status_.statusWord_.instartup = data_.status_.statusWord_.instartup >
                                                0;
  // bit 17 sumilockfwd (filter away execute IL)
  data_.status_.statusWord_.sumilockfwd =
    data_.interlocks_.trajSummaryInterlockFWDEpics;

  // bit 18 sumilockbwd (filter away execute IL)
  data_.status_.statusWord_.sumilockbwd =
    data_.interlocks_.trajSummaryInterlockBWDEpics;

  // bit 19 softlimilockfwd
  data_.status_.statusWord_.softlimilockfwd =
    data_.interlocks_.fwdSoftLimitInterlock;

  // bit 20 softlimilockbwd
  data_.status_.statusWord_.softlimilockbwd =
    data_.interlocks_.bwdSoftLimitInterlock;

  // bit 23 blocked
  data_.status_.statusWord_.blocked = getBlocked();

  // bit 24: native target writes trigger position moves.
  data_.status_.statusWord_.autoExecuteOnTargetPos = autoExecuteOnTargetPos_;
  data_.status_.statusWord_.encoderMode = encoderMode_;

  // The second status word carries sequence state and last interlock bytes.
  data_.status_.statusWord2_.lastilock =
    (unsigned char)data_.interlocks_.lastActiveInterlock;
}
int ecmcAxisBase::setEnable(bool enable) {
  if (enable && getStartupCommandsHeld()) {
    return ERROR_AXIS_BUSY;
  }
  if (!enable && encoderPowerEnablePending_) {
    encoderPowerEnablePending_ = false;
    data_.control_.controlWord_.enableCmd = false;
    setExecute(false);
  }
  if (data_.status_.statusWord_.enable == enable &&
      (!enable || encoderPowerReady())) return 0;

  if (ecmcRtLoggerPortDriverGetCountEnableCommands(data_.status_.axisId)) {
    bumpMotionCommandRequestCounter();
  }

  if (!enable) {  // Remove execute if enable is going down
    setExecute(false);
  }

  if (enable && validate()) {
    setExecute(false);
    return getErrorID();
  }

  if (enable && !encoderPowerReady()) {
    if (getErrorID()) return getErrorID();
    if (getBlockCom()) return setExternalCommandBlockedError();
    if (getBlocked()) return setMasterSlaveCommandBlockedError();
    encoderPowerEnablePending_ = true;
    data_.control_.controlWord_.enableCmd = true;
    for (int i = 0; i < data_.status_.encoderCount; ++i)
      if (encArray_[i]->getPowerEnable()) encArray_[i]->wakePower();
    data_.axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]->refreshParamRT(1);
    return 0;
  }

  if (enable && !getEnc()->hwReady()) {
    data_.control_.controlWord_.enableCmd = false;
    data_.status_.statusWord_.enable = false;
    return setErrorID(ERROR_ENC_NOT_READY);
  }

  if (getMon()->getSafetyInterlock() && enable) {
    data_.control_.controlWord_.enableCmd = false;
    data_.status_.statusWord_.enable = false;
    return setErrorID(ERROR_AXIS_SAFETY_IL_ACTIVE);
  }

  int error = setEnableLocal(enable);

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  data_.status_.statusWord_.enable = enable;
  data_.control_.controlWord_.enableCmd = enable;
  data_.axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]->refreshParamRT(1);

  return 0;
}

double ecmcAxisBase::getExtSetPos() {
  return data_.status_.externalTrajectoryPosition;
}

int ecmcAxisBase::setExtSetPos(double pos) {
  data_.status_.externalTrajectoryPosition = pos;
  return 0;
}

double ecmcAxisBase::getExtActPos() {
  return data_.status_.externalEncoderPosition;
}

int ecmcAxisBase::setExtActPos(double pos) {
  data_.status_.externalEncoderPosition = pos;
  return 0;
}

int ecmcAxisBase::setEnableExtTrajVeloFilter(bool enable) {
  if (!extTrajVeloFilter_) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_FILTER_OBJECT_NULL);
  }

  if (enable && !enableExtTrajVeloFilter_) {
    extTrajVeloFilter_->initFilter(0);  // init to 0 vel
    extTrajVeloFilter_->reset();
  }

  enableExtTrajVeloFilter_ = enable;

  return 0;
}

int ecmcAxisBase::setEnableExtEncVeloFilter(bool enable) {
  if (!extEncVeloFilter_) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_FILTER_OBJECT_NULL);
  }

  if (enable && !enableExtEncVeloFilter_) {
    extEncVeloFilter_->initFilter(0);  // init to 0 vel
    extEncVeloFilter_->reset();
  }

  enableExtEncVeloFilter_ = enable;

  return 0;
}

dataSource ecmcAxisBase::getTrajDataSourceType() {
  return (dataSource)data_.status_.statusWord_.trajsource;
}

dataSource ecmcAxisBase::getEncDataSourceType() {
  return (dataSource)data_.status_.statusWord_.encsource;
}

bool ecmcAxisBase::getEnableExtTrajVeloFilter() {
  return enableExtTrajVeloFilter_;
}

bool ecmcAxisBase::getEnableExtEncVeloFilter() {
  return enableExtEncVeloFilter_;
}

int ecmcAxisBase::setExtTrajVeloFiltSize(size_t size) {
  return extTrajVeloFilter_->setFilterSize(size);
}

int ecmcAxisBase::setExtEncVeloFiltSize(size_t size) {
  return extEncVeloFilter_->setFilterSize(size);
}

int ecmcAxisBase::getExtTrajVeloFiltSize() {
  return extTrajVeloFilter_ ? (int)extTrajVeloFilter_->getFilterSize() : 0;
}

int ecmcAxisBase::getExtEncVeloFiltSize() {
  return extEncVeloFilter_ ? (int)extEncVeloFilter_->getFilterSize() : 0;
}

int ecmcAxisBase::setEncVeloFiltSize(size_t size) {
  // Match the getters and other encoder configuration commands. Selecting an
  // encoder for configuration must not change the active feedback encoder.
  return encArray_[data_.control_.cfgEncIndex]->setVeloFilterSize(size);
}

int ecmcAxisBase::setEncPosFiltSize(size_t size) {
  return encArray_[data_.control_.cfgEncIndex]->setPosFilterSize(size);
}

int ecmcAxisBase::setEncPosFiltEnable(bool enable) {
  return encArray_[data_.control_.cfgEncIndex]->setPosFilterEnable(enable);
}

int ecmcAxisBase::setEncVeloFiltEnable(bool enable) {
  return encArray_[data_.control_.cfgEncIndex]->setVelFilterEnable(enable);
}

int ecmcAxisBase::setEncInvHwReady(int invert) {
  return encArray_[data_.control_.cfgEncIndex]->setInvHwReady(invert);
}

int ecmcAxisBase::createAsynParam(const char        *nameFormat,
                                  asynParamType      asynType,
                                  ecmcEcDataType     ecmcType,
                                  uint8_t           *data,
                                  size_t             bytes,
                                  ecmcAsynDataItem **asynParamOut) {
  if (asynPortDriver_ == NULL) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: AsynPortDriver object is NULL for %s (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      nameFormat,
      ERROR_AXIS_ASYN_PORT_OBJ_NULL);
    return ERROR_AXIS_ASYN_PORT_OBJ_NULL;
  }
  *asynParamOut = NULL;
  char  buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char *name                  = NULL;
  unsigned int charCount      = 0;
  ecmcAsynDataItem *paramTemp = NULL;

  charCount = snprintf(buffer,
                       sizeof(buffer),
                       nameFormat,
                       getAxisID());

  if (charCount >= sizeof(buffer) - 1) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Failed to generate (%s). Buffer too small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      nameFormat,
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }
  name      = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                                asynType,
                                                data,
                                                bytes,
                                                ecmcType,
                                                0);

  if (!paramTemp) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Failed to create default parameter for %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  *asynParamOut = paramTemp;
  return 0;
}

int ecmcAxisBase::movePVTAbs() {
  if (ignoreMotionCommand(ECMC_CMD_MOVEPVTABS)) return 0;
  return movePVTAbs(false);
}

int ecmcAxisBase::movePVTAbs(bool ignoreBusy) {
  if (ignoreMotionCommand(ECMC_CMD_MOVEPVTABS)) return 0;
  bumpMotionCommandRequestCounter();

  if(data_.control_.controlWord_.enableDbgPrintout) {
    ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Axis[%d]: movePVTAbs().\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.status_.axisId);
  }

  if (getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Move PVT failed since traj source is set to PLC (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL);

    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  // Allow pvt to be started even if globalBusy is set
  if(!ignoreBusy) {
    if(data_.status_.statusWord_.localBusy) {
      return ERROR_AXIS_BUSY;
    }
  }

  int errorCode = getErrorID();

  if (errorCode) {
    return errorCode;
  }

  errorCode = setExecute(0,ignoreBusy);

  if (errorCode) {
    return errorCode;
  }

  errorCode = setCommand(ECMC_CMD_MOVEPVTABS);

  if (errorCode) {
    return errorCode;
  }

  errorCode = setCmdData(0);

  if (errorCode) {
    return errorCode;
  }

  return setExecute(1,ignoreBusy);
}

// just a wrapper to the below
int ecmcAxisBase::moveAbsolutePosition(double positionSet) {
  if (ignoreMotionCommand(ECMC_CMD_MOVEABS)) return 0;
  return  moveAbsolutePosition(positionSet,data_.control_.velocityTarget, data_.status_.currentAccelerationSetpoint,data_.status_.currentDecelerationSetpoint);
}

int ecmcAxisBase::moveAbsolutePosition(
  double positionSet,
  double velocitySet,
  double accelerationSet,
  double decelerationSet) {
  if (ignoreMotionCommand(ECMC_CMD_MOVEABS)) return 0;
  bumpMotionCommandRequestCounter();

  if(data_.control_.controlWord_.enableDbgPrintout) {
    ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Axis[%d]: moveAbsolutePosition().\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.status_.axisId);
  }

  if(getBlocked()) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Blocked (master/slave lock) (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      ERROR_AXIS_BLOCKED);

    return ERROR_AXIS_BLOCKED;
  }

  if (getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Move to abs position failed since traj source is set to PLC (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL);

    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  if (getExecute() && (getCommand() == ECMC_CMD_MOVEABS) && data_.status_.statusWord_.localBusy) {
    setTargetVel(velocitySet);
    setAcc(accelerationSet);
    setDec(decelerationSet);
    data_.control_.positionTarget = positionSet;
    setTargetPos(positionSet);
    return 0;
  }

  int errorCode = getErrorID();

  if (errorCode) {
    return errorCode;
  }

  errorCode = setExecute(0);
  if (errorCode) {
    return errorCode;
  }

  errorCode = setCommand(ECMC_CMD_MOVEABS);

  if (errorCode) {
    return errorCode;
  }

  errorCode = setCmdData(0);

  if (errorCode) {
    return errorCode;
  }

  setTargetVel(velocitySet);
  setAcc(accelerationSet);
  setDec(decelerationSet);
  data_.control_.positionTarget = positionSet;
  setTargetPos(positionSet);
  errorCode = setExecute(1);
  if (errorCode) {
    return errorCode;
  }

  return 0;
}

int ecmcAxisBase::moveRelativePosition(
  double positionSet,
  double velocitySet,
  double accelerationSet,
  double decelerationSet) {
  if (ignoreMotionCommand(ECMC_CMD_MOVEREL)) return 0;
  bumpMotionCommandRequestCounter();
  
  if(data_.control_.controlWord_.enableDbgPrintout) {
    ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Axis[%d]: moveRelativePosition().\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.status_.axisId);
  }

  if(getBlocked()) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Blocked (master/slave lock) (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      ERROR_AXIS_BLOCKED);

    return ERROR_AXIS_BLOCKED;
  }

  if (getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Move to abs position failed since traj source is set to PLC (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL);

    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  if (getExecute() && (getCommand() == ECMC_CMD_MOVEREL) && data_.status_.statusWord_.localBusy) {
    setTargetVel(velocitySet);
    setAcc(accelerationSet);
    setDec(decelerationSet);
    data_.control_.positionTarget = positionSet;
    setTargetPos(positionSet);    
    return 0;
  }

  int errorCode = getErrorID();

  if (errorCode) {
    return errorCode;
  }

  errorCode = setExecute(0);

  if (errorCode) {
    return errorCode;
  }

  errorCode = setCommand(ECMC_CMD_MOVEREL);

  if (errorCode) {
    return errorCode;
  }

  errorCode = setCmdData(0);

  if (errorCode) {
    return errorCode;
  }

  setTargetVel(velocitySet);
  setAcc(accelerationSet);
  setDec(decelerationSet);
  data_.control_.positionTarget = positionSet;
  setTargetPos(positionSet);
  errorCode = setExecute(1);

  if (errorCode) {
    return errorCode;
  }
  return 0;
}

int ecmcAxisBase::moveVelocity(
  double velocitySet,
  double accelerationSet,
  double decelerationSet) {
  if (ignoreMotionCommand(ECMC_CMD_MOVEVEL)) return 0;
  bumpMotionCommandRequestCounter();
  
  if(getBlocked()) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Blocked (master/slave lock) (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      ERROR_AXIS_BLOCKED);

    return ERROR_AXIS_BLOCKED;
  }

  if (getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Move to abs position failed since traj source is set to PLC (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL);

    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  int errorCode = getErrorID();

  if (errorCode) {
    return errorCode;
  }

  // check if already moveVelo then just update vel and acc
  if (getExecute() && (getCommand() == ECMC_CMD_MOVEVEL) && data_.status_.statusWord_.localBusy) {
    setTargetVel(velocitySet);
    setAcc(accelerationSet);
    setDec(decelerationSet);
    return 0;
  }

  errorCode = setExecute(0);

  if (errorCode) {
    return errorCode;
  }

  errorCode = setCommand(ECMC_CMD_MOVEVEL);

  if (errorCode) {
    return errorCode;
  }

  errorCode = setCmdData(0);

  if (errorCode) {
    return errorCode;
  }

  setTargetVel(velocitySet);
  setAcc(accelerationSet);
  setDec(decelerationSet);

  errorCode = setExecute(1);

  if (errorCode) {
    return errorCode;
  }
  return 0;
}

int ecmcAxisBase::moveHome(int    nCmdData,
                           double homePositionSet,
                           double velocityTowardsCamSet,
                           double velocityOffCamSet,
                           double accelerationSet,
                           double decelerationSet
                           ) {
  if (ignoreMotionCommand(ECMC_CMD_HOMING)) return 0;
  bumpMotionCommandRequestCounter();

  if (getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Homing failed since traj source is set to PLC (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL);

    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  if (getBusy()) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Homing rejected: axis is busy (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId, ERROR_AXIS_BUSY);

    return setErrorID(ERROR_AXIS_BUSY);
  }

  int errorCode = getErrorID();

  if (errorCode) {
    return errorCode;
  }

  errorCode = setExecute(0);

  if (errorCode) {
    return errorCode;
  }

  errorCode = setCommand(ECMC_CMD_HOMING);

  if (errorCode) {
    return errorCode;
  }

  // if not valid then fallback on whats defined in encoder
  if (nCmdData <= 0) {
    nCmdData = getPrimEnc()->getHomeSeqId();
  }
  errorCode = setCmdData(nCmdData);

  if (errorCode) {
    return errorCode;
  }

  // All the below parameters might be overitten in sequencer if
  // cmdData (homing proc) is set to read cfg from encoder object
  // if not valid then fallback on whats defined in encoder
  if (velocityOffCamSet <= 0) {
    velocityOffCamSet = getPrimEnc()->getHomeVelOffCam();
  }
  getSeq()->setHomeVelOffCam(velocityOffCamSet);

  // if not valid then fallback on whats defined in encoder
  if (velocityTowardsCamSet <= 0) {
    velocityTowardsCamSet = getPrimEnc()->getHomeVelTowardsCam();
  }
  getSeq()->setHomeVelTowardsCam(velocityTowardsCamSet);

  // if not valid then fallback on whats defined in encoder
  if (accelerationSet <= 0) {
    accelerationSet = getPrimEnc()->getHomeAcc();
  }
  getSeq()->setHomeAcc(accelerationSet);
  

  // if not valid then fallback on whats defined in encoder
  if (decelerationSet <= 0) {
    decelerationSet = getPrimEnc()->getHomeDec();
  }
  getSeq()->setHomeDec(decelerationSet);

  getSeq()->setHomePosition(homePositionSet);

  errorCode = setExecute(1);

  if (errorCode) {
    return errorCode;
  }
  return 0;
}

// Homing with configs from encoder object
int ecmcAxisBase::moveHome() {
  if (ignoreMotionCommand(ECMC_CMD_HOMING)) return 0;
  bumpMotionCommandRequestCounter();

  if (getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Homing failed since traj source is set to PLC (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL);

    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  if (getBusy()) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Homing rejected: axis is busy (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId, ERROR_AXIS_BUSY);

    return setErrorID(ERROR_AXIS_BUSY);
  }

  int errorCode = getErrorID();

  if (errorCode) {
    return errorCode;
  }

  errorCode = setExecute(0);

  if (errorCode) {
    return errorCode;
  }

  errorCode = setCommand(ECMC_CMD_HOMING);

  if (errorCode) {
    return errorCode;
  }

  // Use cfgs in current home encoder
  errorCode = setCmdData(ECMC_SEQ_HOME_USE_ENC_CFGS);

  if (errorCode) {
    return errorCode;
  }

  errorCode = setExecute(1);

  if (errorCode) {
    return errorCode;
  }
  return 0;
}

int ecmcAxisBase::setPosition(double homePositionSet) {
  seq_.setNewPositionCtrlDrvTrajBumpless(homePositionSet);
  getPrimEnc()->setActPos(homePositionSet);
  setAxisHomed(1);  
  return 0;
}

int ecmcAxisBase::stopMotion(int killAmplifier) {
  if (encoderPowerEnablePending_) {
    encoderPowerEnablePending_ = false;
    if (!getEnable()) data_.control_.controlWord_.enableCmd = false;
  }
  int errorCode = setExecute(0);
  data_.interlocks_.noExecuteInterlock = true;
  data_.refreshInterlocks();
  autoEnableRequest_ = false;

  if (killAmplifier) {
    errorCode = setEnable(0);

    if (errorCode) {
      return errorCode;
    }
  }
  
  if (errorCode) {
    return errorCode;
  }

  return 0;
}

/**
 * Function to link to ecmcDataItem. Execute function instead of copy data.
 * This function implements commands execution of commands from motor record
 * or other (binary) asyn source.
 *
 * data_.control_.controlWord_.enableCmd  // Empty,enableCmd moved to dedicated asyn parameter
 * data_.control_.controlWord_.executeCmd
 * data_.control_.controlWord_.stopCmd
 * data_.control_.controlWord_.resetCmd
 * data_.control_.controlWord_.encSourceCmd
 * data_.control_.controlWord_.trajSourceCmd
 * data_.control_.controlWord_.plcEnableCmd
 * data_.control_.controlWord_.plcCmdsAllowCmd
 * data_.control_.controlWord_.enableSoftLimitBwd
 * data_.control_.controlWord_.enableSoftLimitFwd
 * data_.control_.controlWord_.enableILockChangePrintout
 * data_.control_.controlWord_.tweakBwdCmd
 * data_.control_.controlWord_.tweakFwdCmd
 * data_.control_.controlWord_.blockCom
 * 
*/
asynStatus ecmcAxisBase::axisAsynWriteCmd(void         *data,
                                          size_t        bytes,
                                          asynParamType asynParType) {
  asynStatus returnVal = asynSuccess;

  if (sizeof(data_.control_.controlWord_) != bytes) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Control word size mismatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }

  ecmcAsynAxisControlType controlWordNew;
  memcpy(&controlWordNew, data, sizeof(controlWordNew));
  const ecmcAsynAxisControlType controlWordCurrent = data_.control_.controlWord_;
  uint32_t controlWordRaw = 0;
  memcpy(&controlWordRaw,
         &controlWordNew,
         sizeof(controlWordNew) < sizeof(controlWordRaw) ?
         sizeof(controlWordNew) : sizeof(controlWordRaw));

  // Retired Force ON bit: reject old clients rather than silently accepting it.
  if (controlWordRaw & (1u << 18)) return asynError;

  // These are momentary commands that are auto-reset after this callback.
  // Each callback with a command bit set is a new request; the stored control
  // word is also used as state/readback and is not reliable edge history.
  //
  // STOP and disable must always remain effective. If a disabled motion
  // request arrives together with enableCmd=0, remove only its momentary
  // execute/tweak bits and continue so the disable is applied. With
  // enableCmd=1 retain the fast return: an ignored move must not re-enable an
  // axis that another command source is disabling during a transition.
  bool ignoredMotionRequest = false;
  if (!controlWordNew.stopCmd) {
    if (controlWordNew.executeCmd &&
        ignoreMotionCommand(data_.control_.command)) {
      controlWordNew.executeCmd = 0;
      ignoredMotionRequest = true;
    }
    if ((controlWordNew.tweakBwdCmd || controlWordNew.tweakFwdCmd) &&
        ignoreMotionCommand(ECMC_CMD_MOVEREL)) {
      controlWordNew.tweakBwdCmd = 0;
      controlWordNew.tweakFwdCmd = 0;
      ignoredMotionRequest = true;
    }
    if (ignoredMotionRequest && controlWordNew.enableCmd) return asynSuccess;
  }
  const bool executeRequest = controlWordNew.executeCmd;
  const bool tweakBwdRequest = controlWordNew.tweakBwdCmd;
  const bool tweakFwdRequest = controlWordNew.tweakFwdCmd;

  if (executeRequest) {
    bumpMotionCommandRequestCounter();
  }
  if (tweakBwdRequest) {
    bumpMotionCommandRequestCounter();
  }
  if (tweakFwdRequest) {
    bumpMotionCommandRequestCounter();
  }

  char controlWordActions[512] = "";
  char actionBuffer[128];
  if (controlWordNew.enableCmd != controlWordCurrent.enableCmd) {
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            controlWordNew.enableCmd ? "enable requested" :
                            "disable requested");
  }
  if (executeRequest) {
    snprintf(actionBuffer,
             sizeof(actionBuffer),
             "execute %s requested (target=%lf, velocity=%lf)",
             axisCommandToString(data_.control_.command),
             data_.control_.positionTarget,
             data_.control_.velocityTarget);
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            actionBuffer);
  }
  if (controlWordNew.stopCmd) {
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            controlWordNew.enableCmd ?
                            "stop requested" : "stop and disable requested");
  }
  if (controlWordNew.resetCmd && !controlWordCurrent.resetCmd) {
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            "reset requested");
  }
  if (tweakBwdRequest) {
    snprintf(actionBuffer,
             sizeof(actionBuffer),
             "tweak backward requested (step=%lf)",
             std::abs(data_.control_.tweakValue));
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            actionBuffer);
  }
  if (tweakFwdRequest) {
    snprintf(actionBuffer,
             sizeof(actionBuffer),
             "tweak forward requested (step=%lf)",
             std::abs(data_.control_.tweakValue));
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            actionBuffer);
  }
  if (controlWordNew.encSourceCmd != controlWordCurrent.encSourceCmd) {
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            controlWordNew.encSourceCmd ?
                            "encoder source external" :
                            "encoder source internal");
  }
  if (controlWordNew.trajSourceCmd != controlWordCurrent.trajSourceCmd) {
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            controlWordNew.trajSourceCmd ?
                            "trajectory source external" :
                            "trajectory source internal");
  }
  if (controlWordNew.plcEnableCmd != controlWordCurrent.plcEnableCmd) {
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            controlWordNew.plcEnableCmd ?
                            "axis PLC enabled" : "axis PLC disabled");
  }
  if (controlWordNew.plcCmdsAllowCmd != controlWordCurrent.plcCmdsAllowCmd) {
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            controlWordNew.plcCmdsAllowCmd ?
                            "PLC commands allowed" :
                            "PLC commands blocked");
  }
  if (controlWordNew.enableSoftLimitBwd !=
      controlWordCurrent.enableSoftLimitBwd) {
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            controlWordNew.enableSoftLimitBwd ?
                            "backward soft limit enabled" :
                            "backward soft limit disabled");
  }
  if (controlWordNew.enableSoftLimitFwd !=
      controlWordCurrent.enableSoftLimitFwd) {
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            controlWordNew.enableSoftLimitFwd ?
                            "forward soft limit enabled" :
                            "forward soft limit disabled");
  }
  if (controlWordNew.enableDbgPrintout !=
      controlWordCurrent.enableDbgPrintout) {
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            controlWordNew.enableDbgPrintout ?
                            "debug printout enabled" :
                            "debug printout disabled");
  }
  if (controlWordNew.blockCom != controlWordCurrent.blockCom) {
    appendControlWordAction(controlWordActions,
                            sizeof(controlWordActions),
                            controlWordNew.blockCom ?
                            "external commands blocked" :
                            "external commands unblocked");
  }
  if (controlWordActions[0]) {
    ecmcRtLoggerLogInfo(
      "%s/%s:%d: INFO: Axis[%d]: Control word request: %s (raw=0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      controlWordActions,
      controlWordRaw);
  }

  if (controlWordNew.blockCom != getBlockCom()) {
    setBlockCom(controlWordNew.blockCom);
  }

  if (getBlockCom()) {
    bool allowedCommand = false;
    bool disallowedCommandChange = false;
    int  errorCode      = 0;

    disallowedCommandChange =
      ((controlWordNew.enableCmd != controlWordCurrent.enableCmd) &&
       controlWordNew.enableCmd) ||
      (controlWordNew.executeCmd != controlWordCurrent.executeCmd) ||
      (controlWordNew.resetCmd != controlWordCurrent.resetCmd) ||
      (controlWordNew.encSourceCmd != controlWordCurrent.encSourceCmd) ||
      (controlWordNew.trajSourceCmd != controlWordCurrent.trajSourceCmd) ||
      (controlWordNew.plcEnableCmd != controlWordCurrent.plcEnableCmd) ||
      (controlWordNew.plcCmdsAllowCmd != controlWordCurrent.plcCmdsAllowCmd) ||
      (controlWordNew.enableSoftLimitBwd !=
       controlWordCurrent.enableSoftLimitBwd) ||
      (controlWordNew.enableSoftLimitFwd !=
       controlWordCurrent.enableSoftLimitFwd) ||
      (controlWordNew.enableDbgPrintout !=
       controlWordCurrent.enableDbgPrintout) ||
      (controlWordNew.tweakBwdCmd != controlWordCurrent.tweakBwdCmd) ||
      (controlWordNew.tweakFwdCmd != controlWordCurrent.tweakFwdCmd) ||
      (controlWordNew.encoderMode != controlWordCurrent.encoderMode);

    if (!controlWordNew.enableCmd) {
      errorCode = setEnable(0);
      if (!errorCode) {
        allowedCommand = true;
      }
    }

    if (controlWordNew.stopCmd) {
      int stopError = stopMotion(controlWordNew.enableCmd == 0);
      if (!errorCode) {
        errorCode = stopError;
      }
      if (!stopError) {
        allowedCommand = true;
      }
    }

    if (allowedCommand) {
      refreshStatusWd();
      return errorCode ? asynError : asynSuccess;
    }

    if (!disallowedCommandChange) {
      refreshStatusWd();
      return asynSuccess;
    }

    setExternalCommandBlockedError();
    return asynError;
  }

  if (masterSlaveBlocked_) {
    bool allowedCommand = false;
    bool disallowedCommandChange = false;
    int  errorCode      = 0;

    disallowedCommandChange =
      ((controlWordNew.enableCmd != controlWordCurrent.enableCmd) &&
       controlWordNew.enableCmd) ||
      (controlWordNew.executeCmd != controlWordCurrent.executeCmd) ||
      (controlWordNew.encSourceCmd != controlWordCurrent.encSourceCmd) ||
      (controlWordNew.trajSourceCmd != controlWordCurrent.trajSourceCmd) ||
      (controlWordNew.plcEnableCmd != controlWordCurrent.plcEnableCmd) ||
      (controlWordNew.plcCmdsAllowCmd != controlWordCurrent.plcCmdsAllowCmd) ||
      (controlWordNew.enableSoftLimitBwd !=
       controlWordCurrent.enableSoftLimitBwd) ||
      (controlWordNew.enableSoftLimitFwd !=
       controlWordCurrent.enableSoftLimitFwd) ||
      (controlWordNew.enableDbgPrintout !=
       controlWordCurrent.enableDbgPrintout) ||
      (controlWordNew.tweakBwdCmd != controlWordCurrent.tweakBwdCmd) ||
      (controlWordNew.tweakFwdCmd != controlWordCurrent.tweakFwdCmd) ||
      (controlWordNew.encoderMode != controlWordCurrent.encoderMode);

    if (!controlWordNew.enableCmd) {
      errorCode = setEnable(0);
      if (!errorCode) {
        allowedCommand = true;
      }
    }

    if (controlWordNew.stopCmd) {
      int stopError = stopMotion(controlWordNew.enableCmd == 0);
      if (!errorCode) {
        errorCode = stopError;
      }
      if (!stopError) {
        allowedCommand = true;
      }
    }

    if (controlWordNew.resetCmd && !controlWordCurrent.resetCmd) {
      setReset(1);
      allowedCommand = true;
    }

    if (disallowedCommandChange) {
      setMasterSlaveCommandBlockedError();
      return asynError;
    }

    if (allowedCommand) {
      refreshStatusWd();
      return errorCode ? asynError : asynSuccess;
    }

    refreshStatusWd();
    return asynSuccess;
  }

  // Validate mode before applying any enable/execute bits from this write.
  if (controlWordNew.encoderMode != encoderMode_ && setEncoderMode(controlWordNew.encoderMode))
    return asynError;
  memcpy(&data_.control_.controlWord_, &controlWordNew, sizeof(data_.control_.controlWord_));

  int errorCode = 0;

  errorCode = setEnable(data_.control_.controlWord_.enableCmd);
  
  if (errorCode) {
    returnVal = asynError;
  }

  if (data_.control_.controlWord_.stopCmd) {
    stopMotion(data_.control_.controlWord_.enableCmd == 0);
    if (errorCode) {
      returnVal = asynError;
    }
  }

  // Only trig new commands if source is internal
  if(data_.status_.statusWord_.trajsource == ECMC_DATA_SOURCE_INTERNAL) {
    // trigg new motion.
    if (!data_.control_.controlWord_.stopCmd && data_.control_.controlWord_.executeCmd) {
      if (getSeq() == NULL) {
        data_.control_.controlWord_.executeCmd = 0; // auto reset
        return asynError;
      }
      
      // Ensure valid command
      if(!commandValid(data_.control_.command)) {
        ecmcRtLoggerLogError(
          "%s/%s:%d: ERROR: Axis[%d]: Invalid command = 0x%x. Command aborted.\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          data_.status_.axisId,(int)data_.control_.command);
        return asynError;
      }

      // Only allow cmd change if not busy
      if (!getBusy()) {        
        setCommand(data_.control_.command);

        if (data_.control_.command == ECMC_CMD_HOMING) {
          // fallback on config encoder
          // if(cmdData_ <= 0 || cmdData_ == ECMC_SEQ_HOME_USE_ENC_CFGS ) {
          //  setCmdData(getPrimEnc()->getHomeSeqId());
          // } else {
          setCmdData(data_.control_.cmdData);

          // }

          // For homing velos, check if special homing velos,
          // otherwise fallback on data_.control_.velocityTarget
          double temp = getPrimEnc()->getHomeVelOffCam();

          if (temp <= 0) {
            temp = data_.control_.velocityTarget;
          }
          getSeq()->setHomeVelOffCam(temp);
          temp = getPrimEnc()->getHomeVelTowardsCam();

          if (temp <= 0) {
            temp = data_.control_.velocityTarget;
          }
          getSeq()->setHomeVelTowardsCam(temp);

          // Homing pos from encoder
          getSeq()->setHomePosition(getPrimEnc()->getHomePosition());
        } else {  // Not homing
          // cmddata for all other states
          setCmdData(data_.control_.cmdData);
        }
      }

      if (data_.control_.command != ECMC_CMD_HOMING) {   // Not homing!
        // allow on the fly updates of target velo and target pos
        setTargetVel(data_.control_.velocityTarget);        
        setTargetPos(data_.control_.positionTarget);
        clearStaleSoftLimitInterlockForValidTarget();
        setAcc(data_.control_.accelerationTarget);
        setDec(data_.control_.decelerationTarget);
      }

      // if not already moving then trigg new motion cmd
      if (!getBusy()) {
        errorCode = setExecute(0);

        if (errorCode) {
          returnVal = asynError;
        }
        errorCode = setExecute(1);
  
        if (errorCode) {
          returnVal = asynError;
        }
      } else {
        ecmcRtLoggerLogInfo("%s/%s:%d: INFO: Axis[%d]: Execute request ignored: axis is busy.\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               data_.status_.axisId);
      }

    }
  }
  
  data_.control_.controlWord_.executeCmd = 0; // auto reset

  if(data_.status_.statusWord_.trajsource == ECMC_DATA_SOURCE_INTERNAL) {
    // Tweak BWD
    if (!data_.control_.controlWord_.stopCmd && data_.control_.controlWord_.tweakBwdCmd) {
      if (!getBusy()) {
        setCommand(ECMC_CMD_MOVEABS);
        setCmdData(0);
        setTargetVel(data_.control_.velocityTarget);
        data_.control_.positionTarget = data_.status_.currentTargetPosition - std::abs(data_.control_.tweakValue);
        setTargetPos(data_.control_.positionTarget);  // Backward
        refreshAsynTargetValue();
        setAcc(data_.control_.accelerationTarget);
        setDec(data_.control_.decelerationTarget);

        errorCode = setExecute(0);
        if (errorCode) {
          returnVal = asynError;
        }

        errorCode = setExecute(1);
        if (errorCode) {
          returnVal = asynError;
        }
      }
    }
    
    // Tweak FWD
    if (!data_.control_.controlWord_.stopCmd && data_.control_.controlWord_.tweakFwdCmd) {
      if (!getBusy()) {
        setCommand(ECMC_CMD_MOVEABS);
        setCmdData(0);
        setTargetVel(data_.control_.velocityTarget);
        data_.control_.positionTarget = data_.status_.currentTargetPosition + std::abs(data_.control_.tweakValue);
        setTargetPos(data_.control_.positionTarget); // Forward
        refreshAsynTargetValue();
        setAcc(data_.control_.accelerationTarget);
        setDec(data_.control_.decelerationTarget);
        errorCode = setExecute(0);
        if (errorCode) {
          returnVal = asynError;
        }

        errorCode = setExecute(1);
        if (errorCode) {
          returnVal = asynError;
        }
      }
    }
  }

  // reset the commands
  data_.control_.controlWord_.tweakFwdCmd = 0;
  data_.control_.controlWord_.tweakBwdCmd = 0;
  data_.control_.controlWord_.stopCmd     = 0;
  setReset(data_.control_.controlWord_.resetCmd);  // resetCmd is reset in setReset()


  errorCode =
    setEncDataSourceType((data_.control_.controlWord_.encSourceCmd ? ECMC_DATA_SOURCE_EXTERNAL
  :
                          ECMC_DATA_SOURCE_INTERNAL));

  if (errorCode) {
    returnVal = asynError;
  }

  errorCode =
    seq_.setTrajDataSourceType((data_.control_.controlWord_.trajSourceCmd ?
                           ECMC_DATA_SOURCE_EXTERNAL
  : ECMC_DATA_SOURCE_INTERNAL));

  if (errorCode) {
    returnVal = asynError;
  }

  errorCode = setAllowCmdFromPLC(data_.control_.controlWord_.plcCmdsAllowCmd);

  if (errorCode) {
    returnVal = asynError;
  }

  errorCode = setAxisPLCEnable(data_.status_.axisId, data_.control_.controlWord_.plcEnableCmd);

  if (errorCode) {
    returnVal = asynError;
  }

  errorCode =
    getMon()->setEnableSoftLimitBwd(data_.control_.controlWord_.enableSoftLimitBwd);

  if (errorCode) {
    returnVal = asynError;
  }

  errorCode =
    getMon()->setEnableSoftLimitFwd(data_.control_.controlWord_.enableSoftLimitFwd);

  if (errorCode) {
    returnVal = asynError;
  }

  refreshStatusWd();
  data_.axAsynParams_[ECMC_ASYN_AX_STATUS_ID]->refreshParamRT(1);
  data_.axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]->refreshParamRT(1);
  return returnVal;
}

void ecmcAxisBase::initControlWord() {
  data_.control_.controlWord_.encoderMode = encoderMode_;
  // Fill all controlWord with actual data before rt
  int plcEnable = 0;

  getAxisPLCEnable(data_.status_.axisId, &plcEnable);
  data_.control_.controlWord_.plcEnableCmd = plcEnable;
  data_.control_.controlWord_.enableCmd    = getEnable();  
  data_.control_.controlWord_.executeCmd   = getExecute();
  data_.control_.controlWord_.resetCmd     = getReset();
  data_.control_.controlWord_.encSourceCmd = getEncDataSourceType() ==
                              ECMC_DATA_SOURCE_EXTERNAL;
  data_.control_.controlWord_.trajSourceCmd = getTrajDataSourceType() ==
                               ECMC_DATA_SOURCE_EXTERNAL;
  data_.control_.controlWord_.plcCmdsAllowCmd    = getAllowCmdFromPLC();
  data_.control_.controlWord_.blockCom           = getBlockCom();
  data_.control_.controlWord_.enableSoftLimitBwd = data_.control_.controlWord_.enableSoftLimitBwd;
  data_.control_.controlWord_.enableSoftLimitFwd = data_.control_.controlWord_.enableSoftLimitFwd;
  data_.control_.controlWord_.stopCmd  = false;  
  data_.control_.controlWord_.tweakBwdCmd = false;
  data_.control_.controlWord_.tweakFwdCmd = false;
}

asynStatus ecmcAxisBase::axisAsynWritePrimEncCtrlId(void         *data,
                                                    size_t        bytes,
                                                    asynParamType asynParType)
{
  if ((bytes != 4) || (asynParType != asynParamInt32)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Primary encoder index datatype mismatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  int index = 0;
  memcpy(&index, data, sizeof(index));

  int errorCode = selectPrimaryEncoder(index, 1);

  if (errorCode) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Set Primary encoder index failed (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId,
      errorCode);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }

  ecmcRtLoggerLogInfo(
    "%s/%s:%d: INFO: Axis[%d]: Primary encoder index set to %d.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.status_.axisId, index);

  return asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteTargetVelo(void         *data,
                                                 size_t        bytes,
                                                 asynParamType asynParType) {
  if ((bytes != 8) || (asynParType != asynParamFloat64)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Target velocity size or datatype mismatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  double velo = 0;
  memcpy(&velo, data, bytes);
  data_.control_.velocityTarget = velo;

  ecmcRtLoggerLogInfo(
    "%s/%s:%d: INFO: Axis[%d]: Target velocity set to %lf.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.status_.axisId, data_.control_.velocityTarget);

  // Write at next execute command

  return asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteAcc(void         *data,
                                          size_t        bytes,
                                          asynParamType asynParType) {
  if ((bytes != 8) || (asynParType != asynParamFloat64)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Acceleration size or datatype mismatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  double acc = 0;
  memcpy(&acc, data, bytes);

  data_.control_.accelerationTarget = acc;
  //setAcc(acc);

  ecmcRtLoggerLogInfo(
    "%s/%s:%d: INFO: Axis[%d]: Acceleration set to %lf.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.status_.axisId, data_.control_.accelerationTarget);

  return asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteDec(void         *data,
                                          size_t        bytes,
                                          asynParamType asynParType) {
  if ((bytes != 8) || (asynParType != asynParamFloat64)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Deceleration size or datatype mismatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  double dec = 0;
  memcpy(&dec, data, bytes);

  //setDec(dec);
  data_.control_.decelerationTarget = dec;

  ecmcRtLoggerLogInfo(
    "%s/%s:%d: INFO: Axis[%d]: Deceleration set to %lf.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.status_.axisId, data_.control_.decelerationTarget);

  // Write at next execute command

  return asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteSetTweakValue(void         *data,
                                                    size_t        bytes,
                                                    asynParamType asynParType) {
  if ((bytes != 8) || (asynParType != asynParamFloat64)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Tweak value size or datatype mismatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  double twk = 0;
  memcpy(&twk, data, bytes);

  data_.control_.tweakValue = twk;
  ecmcRtLoggerLogInfo(
    "%s/%s:%d: INFO: Axis[%d]: Tweak value set to %lf.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.status_.axisId, data_.control_.tweakValue);

  return asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteTargetPos(void         *data,
                                                size_t        bytes,
                                                asynParamType asynParType) {
  if ((bytes != 8) || (asynParType != asynParamFloat64)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Target position size or datatype mismatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  double pos = 0;
  memcpy(&pos, data, bytes);

  if (autoExecuteOnTargetPos_) {
    return moveOnTargetPosWrite(pos) ? asynError : asynSuccess;
  }

  data_.control_.positionTarget = pos;
  ecmcRtLoggerLogInfo(
    "%s/%s:%d: INFO: Axis[%d]: Target position set to %lf.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.status_.axisId, data_.control_.positionTarget);

  return asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteSetEncPos(void         *data,
                                                size_t        bytes,
                                                asynParamType asynParType) {
  if ((sizeof(double) != bytes) || (asynParType != asynParamFloat64)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Encoder position size or datatype mismatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }

  double pos = 0;
  memcpy(&pos, data, bytes);

  if (getBlockCom()) {
    setExternalCommandBlockedError();
    return asynError;
  }

  if (getBusy()) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Encoder position write rejected: axis is busy.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId);

    setWarningID(WARNING_AXIS_ASYN_CMD_WHILE_BUSY);
    return asynError;
  }

  ecmcRtLoggerLogInfo(
    "%s/%s:%d: INFO: Axis[%d]: Encoder position set to %lf.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.status_.axisId, pos);

  // Set position
  return setPosition(pos) ? asynError : asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteCommand(void         *data,
                                              size_t        bytes,
                                              asynParamType asynParType) {
  if ((bytes != 4) || (asynParType != asynParamInt32)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Command size or datatype mismatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  int command = 0;
  memcpy(&command, data, bytes);

  if (getBlockCom()) {
    setExternalCommandBlockedError();
    return asynError;
  }

  if (masterSlaveBlocked_) {
    setMasterSlaveCommandBlockedError();
    return asynError;
  }

  if(!commandValid((motionCommandTypes)command)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Invalid command 0x%x. Keeping old command %d.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId, command, (int)data_.control_.command);
    data_.axAsynParams_[ECMC_ASYN_AX_COMMAND_ID]->refreshParamRT(1);
    return asynError;
  }

  data_.control_.command = (motionCommandTypes)command;

  // Different target values if ABS or REL
  refreshAsynTargetValue();

  ecmcRtLoggerLogInfo(
    "%s/%s:%d: INFO: Axis[%d]: Command set to %s (%d).\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.status_.axisId,
    axisCommandToString(data_.control_.command),
    data_.control_.command);
  
  return asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteCmdData(void         *data,
                                              size_t        bytes,
                                              asynParamType asynParType) {
  if ((bytes != 4) || (asynParType != asynParamInt32)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Axis[%d]: Command data size or datatype mismatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.status_.axisId);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  int cmddata = 0;
  memcpy(&cmddata, data, bytes);

  if (getBlockCom()) {
    setExternalCommandBlockedError();
    return asynError;
  }
  data_.control_.cmdData = cmddata;
  //setCmdData(cmddata);

  ecmcRtLoggerLogInfo(
    "%s/%s:%d: INFO: Axis[%d]: Command data set to 0x%x.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.status_.axisId, data_.control_.cmdData);

  return asynSuccess;
}

int ecmcAxisBase::setDisableAxisAtErrorReset(bool disable) {
  disableAxisAtErrorReset_ = disable;
  return 0;
}

int ecmcAxisBase::setAllowMotionFunctions(bool enablePos,
                                          bool enableConstVel,
                                          bool enableHome) {
  if (getSeq() != NULL) {
    return getSeq()->setAllowMotionFunctions(enablePos,
                                             enableConstVel,
                                             enableHome);
  }
  return ERROR_AXIS_SEQ_OBJECT_NULL;
}

int ecmcAxisBase::getAllowPos() {
  if (getSeq() != NULL) {
    return getSeq()->getAllowPos();
  }
  return -ERROR_AXIS_SEQ_OBJECT_NULL;
}

int ecmcAxisBase::getAllowConstVelo() {
  if (getSeq() != NULL) {
    return getSeq()->getAllowConstVelo();
  }
  return -ERROR_AXIS_SEQ_OBJECT_NULL;
}

int ecmcAxisBase::getAllowHome() {
  if (getSeq() != NULL) {
    return getSeq()->getAllowHome();
  }
  return -ERROR_AXIS_SEQ_OBJECT_NULL;
}

int ecmcAxisBase::addEncoder() {
  if (data_.status_.encoderCount >= ECMC_MAX_ENCODERS) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_ENC_COUNT_OUT_OF_RANGE);
  }
  encArray_[data_.status_.encoderCount] = new ecmcEncoder(asynPortDriver_,
                                                          data_,
                                                          data_.status_.sampleTime,
                                                          data_.status_.encoderCount);
  if (!encArray_[data_.status_.encoderCount]) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_ENC_OBJECT_NULL);
  }
  int errorCode = encArray_[data_.status_.encoderCount]->getErrorID();
  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }
  data_.control_.cfgEncIndex = data_.status_.encoderCount; // Use current encoder index for cfg
  data_.status_.encoderCount++;

  return 0;
}

// Only for Configuration purpose
int ecmcAxisBase::selectCSPDriveEncoder(int index) {
  
  // Comamnd only makes sense if REAL axis
  if(data_.status_.axisType != ECMC_AXIS_TYPE_REAL) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Axis[%d]: Command only valid for axes of type REAL.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_.status_.axisId);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
  }

  if(data_.status_.statusWord_.inrealtime ) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Axis[%d]: Command not allowed in realtime.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_.status_.axisId);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_CMD_NOT_ALLOWED_IN_REALTIME);
  }

  // Do not change if less than 0 (allow ecmccfg to set -1 as default)
  int localIndex = index - 1;

  if (localIndex < 0) {
    return 0;
  }

  if ((localIndex >= ECMC_MAX_ENCODERS) ||
      (localIndex >= data_.status_.encoderCount)) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Axis[%d]: Encoder index out of range.\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.status_.axisId);
    
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_PRIMARY_ENC_ID_OUT_OF_RANGE);
  }


  if(data_.control_.controlWord_.enableDbgPrintout) {
    ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Axis[%d]: selectCSPDriveEncoder(): CSP encoder selected for drive control.\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.status_.axisId);
  }

  data_.control_.cspDrvEncIndex = localIndex;
  
  // CSP encoer is set to drv object in ecmcAxisReal::validate()

  data_.status_.currentCSPPositionSetpointOffset  = 0;
  return 0;
}

int ecmcAxisBase::selectPrimaryEncoder(int index) {
  return selectPrimaryEncoder(index, 0);
}

// Index 1 is the first encoder
int ecmcAxisBase::selectPrimaryEncoder(int index, int overrideError) {
  // Do not change if less than 0 (allow ecmccfg to set -1 as default)
  int localIndex = index - 1;

  if (localIndex < 0) {
    return 0;
  }

  if ((encoderPowerBackupIndex_ >= 0 || encoderMode_ != 0) && localIndex != data_.control_.primaryEncIndex)
    return ERROR_AXIS_BUSY; // Automatic handover owns primary selection.

  if ((localIndex >= ECMC_MAX_ENCODERS) ||
      (localIndex >= data_.status_.encoderCount)) {
    // Override error message to not stop axis if in motion
    if (!overrideError) {
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_AXIS_PRIMARY_ENC_ID_OUT_OF_RANGE);
    }
    return ERROR_AXIS_PRIMARY_ENC_ID_OUT_OF_RANGE;
  }

  if(!data_.status_.statusWord_.instartup) {  // Allow to switch if in startup phase (and busy)
    // Do not allow to switch encoder if busy and INTERNAL source
    if (getBusy() && (data_.status_.statusWord_.encsource ==
                      ECMC_DATA_SOURCE_INTERNAL)) {
      // Override error message to not stop axis if in motion
      if (!overrideError) {
        setErrorID(__FILE__,
                   __FUNCTION__,
                   __LINE__,
                   ERROR_AXIS_SWITCH_PRIMARY_ENC_NOT_ALLOWED_WHEN_BUSY);
      }
      return ERROR_AXIS_SWITCH_PRIMARY_ENC_NOT_ALLOWED_WHEN_BUSY;
    }
  }

  // Make sure the switch is bumpless
  if (localIndex != data_.control_.primaryEncIndex) {
    seq_.setNewPositionCtrlDrvTrajBumpless(encArray_[localIndex]->getActPos());
    refreshAsynTargetValue();
  }

  // This index is starting from 0
  data_.control_.primaryEncIndex = localIndex;

  // This index is starting from 1 (for asyn and external interface)
  encPrimIndexAsyn_ = index;
  data_.axAsynParams_[ECMC_ASYN_AX_ENC_ID_CMD_ID]->refreshParamRT(1);

  return 0;
}

// Index 1 is the first encoder
int ecmcAxisBase::selectConfigEncoder(int index) {
  // Do not change if less than 0 (allow ecmccfg to set -1 as default)
  int localIndex = index - 1;

  if (localIndex < 0) {
    return 0;
  }

  if ((localIndex >= ECMC_MAX_ENCODERS) ||
      (localIndex >= data_.status_.encoderCount)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_PRIMARY_ENC_ID_OUT_OF_RANGE);
  }

  // This index is starting from 0
  data_.control_.cfgEncIndex = localIndex;
  return 0;
}

// unfortunately starts from 1... why did I do like this..
int ecmcAxisBase::getConfigEncoderIndex() {
  return data_.control_.cfgEncIndex + 1;
}

int ecmcAxisBase::getPrimaryEncoderIndex() {
  return data_.control_.primaryEncIndex + 1;
}

int ecmcAxisBase::getDrvMode() {
  return static_cast<int>(data_.control_.drvMode);
}

int ecmcAxisBase::getCSPDriveEncoderIndex() {
  return data_.control_.cspDrvEncIndex;
}

bool ecmcAxisBase::getHwReady() {

  // Initial startup must verify every managed encoder, even if the tracking
  // counter is currently primary. Subsequent intentional idle power-off is OK.
  if (!data_.status_.startupFinsished) {
    for (int i = 0; i < data_.status_.encoderCount; ++i) {
      if (encArray_[i]->getPowerEnable() &&
          (!encArray_[i]->getPowerReady() || !encArray_[i]->hwReady()))
        return false;
    }
  }

  /* Only check prim encoder (allow encoders to be in error state 
  if they are not used) */
  auto * const primEnc = getPrimEnc();
  // An intentionally unpowered encoder does not prevent IOC/axis startup.
  // setEnable has a separate strict power/readiness gate before drive enable.
  bool ready = (primEnc->getPowerOff() && !encoderPowerEnablePending_) || primEnc->hwReady();
  if (!ready) {
    return false;
  }

  // Check drive if REAL axis
  auto * const drv = getDrv();
  if(drv) {
    ready = ready && drv->hwReady();
  }
  return ready;
}

void ecmcAxisBase::initEncoders() {
  encoderPowerInitPending_ = false;
  // set all relative encoders to 0
  for (int i = 0; i < data_.status_.encoderCount; i++) {
    encArray_[i]->setToZeroIfRelative();
  }

  // check if any encoders should be referenced to another encoder
  for (int i = 0; i < data_.status_.encoderCount; i++) {
    int encRefIndex =  encArray_[i]->getRefToOtherEncAtStartup();

    if ((encRefIndex < 0) || (encRefIndex >= data_.status_.encoderCount)) {
      continue;
    }

    if (encArray_[encRefIndex] == NULL) {
      continue;
    }
    if (encArray_[encRefIndex]->getPowerOff() || encArray_[i]->getPowerOff()) {
      encoderPowerInitPending_ = true;
      continue;
    }
    encArray_[i]->setActPos(encArray_[encRefIndex]->getActPos());
    encArray_[i]->setHomed(1);
  }
}

int ecmcAxisBase::setAllowSourceChangeWhenEnabled(bool allow) {
  data_.control_.allowSourceChangeWhenEnabled = allow;
  return 0;
}

int ecmcAxisBase::getAllowSourceChangeWhenEnabled() {
  return data_.control_.allowSourceChangeWhenEnabled;
}

void ecmcAxisBase::setAutoExecuteOnTargetPos(bool enable) {
  autoExecuteOnTargetPos_ = enable;
  data_.status_.statusWord_.autoExecuteOnTargetPos = enable;
  if (data_.axAsynParams_[ECMC_ASYN_AX_STATUS_ID]) {
    data_.axAsynParams_[ECMC_ASYN_AX_STATUS_ID]->refreshParamRT(1);
  }
}

bool ecmcAxisBase::getAutoExecuteOnTargetPos() const {
  return autoExecuteOnTargetPos_;
}

int ecmcAxisBase::moveOnTargetPosWrite(double posTarget) {
  if (ignoreMotionCommand(data_.control_.command)) return 0;
  // Only external native target writes enter here. Internal setpoint updates
  // must not retrigger moves (in particular relative moves) every RT scan.
  if (getBlockCom()) return setExternalCommandBlockedError();
  if (getBlocked()) return setMasterSlaveCommandBlockedError();
  if (getStartupCommandsHeld()) return ERROR_AXIS_BUSY;
  if (getErrorID()) return getErrorID();
  if (!std::isfinite(posTarget)) return ERROR_SEQ_TARGET_POS_OUT_OF_RANGE;
  const motionCommandTypes command = data_.control_.command;
  if (command != ECMC_CMD_MOVEABS && command != ECMC_CMD_MOVEREL) {
    return ERROR_SEQ_COMMAND_NOT_SUPPORTED;
  }
  if (getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }
  // Retarget only a position move of the same type; never interrupt homing,
  // jogging, or a pending enable/start sequence with a target write.
  if (getBusy() && !(getExecute() && getCommand() == command &&
                    data_.status_.statusWord_.localBusy)) {
    return ERROR_AXIS_BUSY;
  }
  if (!getEnable() && !(enableAutoEnable_ && autoEnableTimoutS_ > 0)) {
    return ERROR_AXIS_NOT_ENABLED;
  }
  if (command == ECMC_CMD_MOVEABS) {
    return moveAbsolutePosition(posTarget, data_.control_.velocityTarget,
                                data_.control_.accelerationTarget,
                                data_.control_.decelerationTarget);
  }
  return moveRelativePosition(posTarget, data_.control_.velocityTarget,
                              data_.control_.accelerationTarget,
                              data_.control_.decelerationTarget);
}

void ecmcAxisBase::setTargetPos(double posTarget) {
  const int warningCodeOld = data_.status_.warningCode;
  getSeq()->setTargetPos(posTarget);
  if (data_.status_.warningCode != warningCodeOld) {
    data_.axAsynParams_[ECMC_ASYN_AX_WARNING_ID]->refreshParamRT(1);
  }
  refreshAsynTargetValue();  
}

void ecmcAxisBase::setTargetPosAndCmd(double posTarget) {
  const int warningCodeOld = data_.status_.warningCode;
  getSeq()->setTargetPos(posTarget);
  if (data_.status_.warningCode != warningCodeOld) {
    data_.axAsynParams_[ECMC_ASYN_AX_WARNING_ID]->refreshParamRT(1);
  }
  data_.control_.positionTarget = getSeq()->getTargetPos();
  refreshAsynTargetValue();
}

void ecmcAxisBase::setTargetPosToCurrSetPos() {
  setTargetPos(data_.status_.currentPositionSetpoint);
}

void ecmcAxisBase::clearStaleSoftLimitInterlockForValidTarget() {
  auto &status = data_.status_;
  auto &control = data_.control_;
  auto &interlocks = data_.interlocks_;

  if ((control.command != ECMC_CMD_MOVEABS) &&
      (control.command != ECMC_CMD_MOVEREL)) {
    return;
  }

  if (control.cmdData != 0) {
    return;
  }

  const double target = status.currentTargetPosition;
  const double posSet = status.currentPositionSetpoint;
  bool refreshInterlocks = false;

  if (interlocks.fwdSoftLimitInterlock &&
      (posSet <= control.softLimitFwd) &&
      (target <= control.softLimitFwd) &&
      (target > posSet)) {
    interlocks.fwdSoftLimitInterlock = false;
    refreshInterlocks = true;
  }

  if (interlocks.bwdSoftLimitInterlock &&
      (posSet >= control.softLimitBwd) &&
      (target >= control.softLimitBwd) &&
      (target < posSet)) {
    interlocks.bwdSoftLimitInterlock = false;
    refreshInterlocks = true;
  }

  if (refreshInterlocks) {
    data_.refreshInterlocks();
  }
}

void ecmcAxisBase::setTargetVel(double velTarget) {
  getSeq()->setTargetVel(velTarget);

  // also set for ecmc interface
  data_.control_.velocityTarget = velTarget;
  data_.axAsynParams_[ECMC_ASYN_AX_TARG_VELO_ID]->refreshParamRT(1);
}

void ecmcAxisBase::setTweakDist(double dist) {
  data_.control_.tweakValue = std::abs(dist);
  refreshAsynTargetValue();

  data_.axAsynParams_[ECMC_ASYN_AX_TWEAK_VALUE_ID]->refreshParamRT(1);
}

double ecmcAxisBase::getTweakDist() {
  return data_.control_.tweakValue;
}

void ecmcAxisBase::setAcc(double acc) {
  getSeq()->setAcc(acc);

  // also set for ecmc interface
  data_.axAsynParams_[ECMC_ASYN_AX_ACC_ID]->refreshParamRT(1);
}

void ecmcAxisBase::setDec(double dec) {
  getSeq()->setDec(dec);

  // also set for ecmc interface
  data_.control_.decelerationTarget = dec;
  data_.axAsynParams_[ECMC_ASYN_AX_DEC_ID]->refreshParamRT(1);
}

void ecmcAxisBase::setEmergencyStopInterlock(int stop) {
  getMon()->setSafetyInterlock(stop);

  // Switch to internal source
  if ((data_.status_.statusWord_.trajsource != ECMC_DATA_SOURCE_INTERNAL) && stop) {
    seq_.setTrajDataSourceTypeInternal(ECMC_DATA_SOURCE_INTERNAL, 1);
  }
}

void ecmcAxisBase::setExternalMaxVelo(double veloLimit,int active) {
  getTraj()->setExternalMaxVelo(veloLimit, active);
}

double ecmcAxisBase::getEncVelo() {
  return data_.status_.currentVelocityActual;
}

double ecmcAxisBase::getTrajVelo() {
  return data_.status_.currentVelocitySetpoint;
}

int ecmcAxisBase::setSlavedAxisInError() {
  setErrorID(ERROR_AXIS_SLAVED_AXIS_IN_ERROR);
  return 0;
}

int ecmcAxisBase::setSlavedAxisInterlock() {
  setErrorID(ERROR_AXIS_SLAVED_AXIS_INTERLOCK);
  return 0;
}

int ecmcAxisBase::setSlavedAxisTrajSourceChanged() {
  setErrorID(ERROR_AXIS_SLAVED_AXIS_TRAJ_SOURCE_CHANGED);
  return 0;
}

// motor.SYNC (set to act)
void ecmcAxisBase::setMRSync(bool sync) {
  // Ensure to only trigg command once per cycle
  if(mrCmds_.syncMRCmdTgl == mrCmdsOld_.syncMRCmdTgl) {
    mrCmds_.syncMRCmdTgl += 1;
  }
  // However always update the value
  mrCmds_.syncMRVal = sync;
  // param updated in ::postExecute

  // Sync ecmc if not enabled (only works in internal mode)
  if(!data_.status_.statusWord_.enabled && sync) {
    getTraj()->setCurrentPosSet(data_.status_.currentPositionActual);
  }
}

// motor.STOP
void ecmcAxisBase::setMRStop(bool stop) {
  // Ensure to only trigg command once per cycle
  if(mrCmds_.stopMRCmdTgl == mrCmdsOld_.stopMRCmdTgl) {
    mrCmds_.stopMRCmdTgl += 1;
  }
  // However always update the value
  mrCmds_.stopMRVal = stop;
  // param updated in ::postExecute
}

// motor.CNEN
void ecmcAxisBase::setMRCnen(bool cnen) {
  // Ensure to only trigg command once per cycle
  if(mrCmds_.cnenMRCmdTgl == mrCmdsOld_.cnenMRCmdTgl) {
    mrCmds_.cnenMRCmdTgl += 1;
  }
  // However always update the value
  mrCmds_.cnenMRVal = cnen;
  // param updated in ::postExecute
}

int ecmcAxisBase::loadEncLookupTable(const char* filename) {
  return encArray_[data_.control_.cfgEncIndex]->loadLookupTable(filename);
}

int  ecmcAxisBase::setEncLookupTableEnable(int enable) {
  return encArray_[data_.control_.cfgEncIndex]->setLookupTableEnable(enable);
}

int ecmcAxisBase::getSumInterlock() {
  return (data_.interlocks_.interlockStatus > 0) && 
         (data_.interlocks_.interlockStatus != ECMC_INTERLOCK_NO_EXECUTE);
}

int ecmcAxisBase::getSumInterlockOrStop() {
  return (data_.interlocks_.interlockStatus > 0);
}

int ecmcAxisBase::getSoftLimitInterlock() {
  return data_.interlocks_.fwdSoftLimitInterlock ||
         data_.interlocks_.bwdSoftLimitInterlock;
}

int ecmcAxisBase::getPrintDbg() {
  return data_.control_.controlWord_.enableDbgPrintout;
}

ecmcAxisPVTSequence* ecmcAxisBase::getPVTObject() {
  return seq_.getPVTObject();
}

double ecmcAxisBase::getCurrentPositionSetpoint() {
  return data_.status_.currentPositionSetpoint;
}


void ecmcAxisBase::setMRIgnoreDisableStatusCheck(bool ignore) {
  ignoreMRDisableStatusCheck_ = ignore;
}

bool ecmcAxisBase::getMRIgnoreDisableStatusCheck() {
 return ignoreMRDisableStatusCheck_;
}

// At switch if 0
bool ecmcAxisBase::getLimitBwd() {
  return data_.status_.statusWord_.limitbwd;
}

// At switch if 0
bool ecmcAxisBase::getLimitFwd() {
  return data_.status_.statusWord_.limitfwd;
}

// Operator controls share the existing axis control/status words. A feedback
// change is an explicit disabled-axis operation, never an implicit move.
int ecmcAxisBase::getEncoderForceOn() const {
  const int index = encoderPowerSourceIndex_ >= 0 ? encoderPowerSourceIndex_ : data_.control_.primaryEncIndex;
  return encArray_[index] ? encArray_[index]->getPowerControl() : 0;
}

int ecmcAxisBase::setEncoderForceOn(int force) {
  if (force < 0 || force > 1) return ERROR_ENC_POWER_CONFIG;
  if (getBlockCom() || masterSlaveBlocked_) return ERROR_AXIS_BLOCKED;
  const int index = encoderPowerSourceIndex_ >= 0 ? encoderPowerSourceIndex_ : data_.control_.primaryEncIndex;
  if (!encArray_[index]) return ERROR_ENC_POWER_CONFIG;
  return encArray_[index]->setPowerControl(force);
}

int ecmcAxisBase::setEncoderMode(int mode) {
  if (mode < 0 || mode > data_.status_.encoderCount) return ERROR_ENC_POWER_CONFIG;
  if (getBlockCom() || masterSlaveBlocked_) return ERROR_AXIS_BLOCKED;
  if (mode == encoderMode_) return 0;
  if (getEnable() || getEnabled() || getBusy() || encoderPowerEnablePending_ ||
      autoEnableRequest_ || motionStartPending_) return ERROR_AXIS_BUSY;
  if (data_.status_.statusWord_.encsource != ECMC_DATA_SOURCE_INTERNAL ||
      (getDrv() && data_.control_.drvMode != ECMC_DRV_MODE_CSV))
    return ERROR_ENC_POWER_CONFIG;
  // Without a configured power source, the diagnostic hold belongs to the
  // primary encoder. Release it before changing the primary owner.
  if (encoderPowerSourceIndex_ < 0 && getEncoderForceOn()) return ERROR_AXIS_BUSY;
  if (mode) {
    auto *encoder = encArray_[mode-1];
    if (!getInStartupPhase() && !encoder->getPowerEnable() &&
        (!encoder->hwReady() || !encoder->checkAllDataValid() || encoder->getErrorID()))
      return ERROR_ENC_POWER_CONFIG;
    // Keep each encoder's own reference. Explicit selection is not a homing
    // or reference command. Powered feedback is selected by the RT path only
    // after its power-up delay and optional ready input have completed.
    if (encoder->getPowerReady()) switchEncoderPowerFeedback(mode-1);
    if (encoder->getPowerEnable()) encoder->wakePower();
  } else {
    encoderPowerMoveCompleted_ = true;
    for (int i=0; i<data_.status_.encoderCount; ++i) encArray_[i]->updatePowerIdle(false);
  }
  encoderMode_ = mode;
  data_.control_.controlWord_.encoderMode = mode;
  data_.status_.statusWord_.encoderMode = mode;
  return 0;
}

// Indices here are zero-based. This validator is called after the real drive's
// validate() has established CSV/CSP, and before startup releases motion.
int ecmcAxisBase::validateEncoderPowerBackup(int source, int backup) {
  if (source < 0 || backup < 0 || source >= data_.status_.encoderCount ||
      backup >= data_.status_.encoderCount || source == backup ||
      !encArray_[source] || !encArray_[backup]) {
    ecmcRtLoggerLogError("Axis[%d]: Encoder power source %d and backup %d must be distinct configured encoders (count=%d).\n",
      data_.status_.axisId, source+1, backup+1, data_.status_.encoderCount);
    return ERROR_ENC_POWER_CONFIG;
  }
  if (!encArray_[source]->getPowerEnable() || encArray_[backup]->getPowerEnable() ||
      encArray_[source]->getType() != ECMC_ENCODER_TYPE_ABSOLUTE ||
      encArray_[backup]->getType() != ECMC_ENCODER_TYPE_INCREMENTAL) {
    ecmcRtLoggerLogError("Axis[%d]: Encoder power handover requires a power-controlled absolute source and an always-powered incremental backup.\n",
      data_.status_.axisId);
    return ERROR_ENC_POWER_CONFIG;
  }
  if (data_.control_.moduloRange != 0 ||
      data_.status_.statusWord_.encsource != ECMC_DATA_SOURCE_INTERNAL ||
      (getDrv() && data_.control_.drvMode != ECMC_DRV_MODE_CSV)) {
    ecmcRtLoggerLogError("Axis[%d]: Encoder power handover requires internal encoder feedback, no modulo and a validated CSV drive.\n",
      data_.status_.axisId);
    return ERROR_ENC_POWER_CONFIG;
  }
  return 0;
}

int ecmcAxisBase::setEncoderPowerBackup(int source, int backup, double tolerance) {
  if (getEnable() || getEnabled() || getBusy()) return ERROR_AXIS_BUSY;
  if (!std::isfinite(tolerance) || tolerance <= 0) return ERROR_ENC_POWER_CONFIG;
  if (backup == 0) {
    if (getEncoderForceOn()) return ERROR_AXIS_BUSY;
    encoderMode_ = 0;
    data_.control_.controlWord_.encoderMode = 0;
    data_.status_.statusWord_.encoderMode = 0;
    encoderPowerDiffTol_ = tolerance;
    encoderPowerSourceIndex_ = encoderPowerBackupIndex_ = -1;
    encoderPowerBackupAligned_ = false;
    return 0;
  }
  // During CONFIG, subsequent YAML files may still create/configure either
  // encoder and the drive has not necessarily classified its setpoint mode.
  // Store bounded, distinct indices; final axis validation checks the pair.
  if (source < 1 || backup < 1 || source > ECMC_MAX_ENCODERS ||
      backup > ECMC_MAX_ENCODERS || source == backup)
    return ERROR_ENC_POWER_CONFIG;
  // Once realtime has started, a change must be valid immediately. Never let
  // a runtime request install missing pointers or an unsupported drive mode.
  if (getRealTimeStarted()) {
    if (validateEncoderPowerBackup(source-1, backup-1)) return ERROR_ENC_POWER_CONFIG;
    if (encArray_[source-1]->getPowerState() == 3) return ERROR_ENC_POWER_CONFIG;
  }
  encoderPowerSourceIndex_ = source-1;
  encoderPowerBackupIndex_ = backup-1;
  encoderPowerDiffTol_ = tolerance;
  encoderPowerBackupAligned_ = false;
  return 0;
}

bool ecmcAxisBase::encoderPowerBackupSelected() const {
  return encoderPowerBackupIndex_ >= 0 &&
    data_.control_.primaryEncIndex == encoderPowerBackupIndex_;
}

bool ecmcAxisBase::encoderPowerFeedbackReady() {
  if (encoderMode_ != 0) return encoderPowerReady() && !encoderPowerEnablePending_;
  if (encoderPowerBackupSelected()) {
    auto *backup = encArray_[encoderPowerBackupIndex_];
    return !backup->getPowerEnable() && !backup->getErrorID() &&
      backup->checkAllDataValid() && backup->hwReady();
  }
  if (encoderPowerSourceIndex_ >= 0 &&
      !encArray_[encoderPowerSourceIndex_]->checkAllDataValid()) return false;
  return encoderPowerReady() && !encoderPowerEnablePending_;
}

void ecmcAxisBase::switchEncoderPowerFeedback(int index) {
  if (data_.control_.primaryEncIndex == index) return;
  // Called with the axis disabled for manual selection, or from the RT
  // handover with new trajectory starts blocked.
  // Preserve staged commands: the normal bumpless helper rewrites that target.
  const double request = data_.control_.positionTarget;
  data_.control_.primaryEncIndex = index;
  encPrimIndexAsyn_ = index+1;
  seq_.setNewPositionCtrlDrvTrajBumpless(encArray_[index]->getActPos());
  data_.control_.positionTarget = request;
  data_.axAsynParams_[ECMC_ASYN_AX_ENC_ID_CMD_ID]->refreshParamRT(1);
}

void ecmcAxisBase::encoderPowerBackupSM() {
  if (encoderPowerBackupIndex_ < 0 || encoderMode_ != 0) return;
  auto *source = encArray_[encoderPowerSourceIndex_];
  auto *backup = encArray_[encoderPowerBackupIndex_];
  const auto &word = data_.status_.statusWord_;
  if (getInStartupPhase()) return;
  if (!source->getPowerEnable() || backup->getPowerEnable()) {
    setEnable(false);
    setErrorID(ERROR_ENC_POWER_CONFIG);
    return;
  }
  if (backup->getErrorID() || !backup->checkAllDataValid() || !backup->hwReady()) {
    const int error = backup->getErrorID() ? backup->getErrorID() : ERROR_ENC_NOT_READY;
    setEnable(false);
    setErrorID(error);
    return;
  }
  if (source->getPowerReady() && !word.busy &&
      source->getPowerIdleOff() &&
      data_.control_.primaryEncIndex == encoderPowerSourceIndex_) {
    backup->referencePowerBackup(source->getActPos(), source->getHomed());
    encoderPowerBackupAligned_ = true;
    switchEncoderPowerFeedback(encoderPowerBackupIndex_);
  }
}

void ecmcAxisBase::syncEncoderPowerPosition() {
  if (encoderMode_ != 0) {
    auto *encoder = encArray_[encoderMode_-1];
    if (!getInStartupPhase() && (encoder->getErrorID() ||
        (encoder->getPowerReady() && (!encoder->checkAllDataValid() || !encoder->hwReady())))) {
      setEnable(false);
      setErrorID(encoder->getErrorID() ? encoder->getErrorID() : ERROR_ENC_NOT_READY);
      return;
    }
    if (encoder->getPowerReady() && encoder->checkAllDataValid() && encoder->hwReady() &&
        !encoder->getErrorID()) {
      const bool fresh = encoder->consumePowerPositionSync();
      const bool alreadySelected = data_.control_.primaryEncIndex == encoderMode_-1;
      switchEncoderPowerFeedback(encoderMode_-1);
      if (fresh && alreadySelected) {
        const double request = data_.control_.positionTarget;
        seq_.setNewPositionCtrlDrvTrajBumpless(encoder->getActPos());
        data_.control_.positionTarget = request;
      }
      if (fresh && autoEnableRequest_ && data_.control_.command != ECMC_CMD_HOMING)
        setTargetPos(data_.control_.positionTarget);
    }
    return;
  }
  if (encoderPowerBackupIndex_ >= 0) {
    auto *source = encArray_[encoderPowerSourceIndex_];
    auto *backup = encArray_[encoderPowerBackupIndex_];
    if (!getInStartupPhase() && (!backup->checkAllDataValid() || !backup->hwReady() ||
        backup->getErrorID() || (source->getPowerReady() && !source->checkAllDataValid()))) {
      setEnable(false);
      setErrorID(backup->getErrorID() ? backup->getErrorID() : ERROR_ENC_NOT_READY);
      return;
    }
    if (!source->getPowerReady()) {
      // Startup has no valid absolute position to reference from yet. The
      // counter is only used for disabled/holding feedback; motion still waits.
      if (backup->hwReady() && (encoderPowerBackupAligned_ || getInStartupPhase() ||
          (!getEnable() && !getEnabled() && !getBusy())))
        switchEncoderPowerFeedback(encoderPowerBackupIndex_);
      return;
    }
    if (source->consumePowerPositionSync() || (encoderPowerBackupSelected() &&
        (autoEnableRequest_ || encoderPowerEnablePending_))) {
      if (encoderMode_ == 0 && encoderPowerBackupAligned_ &&
          (!std::isfinite(source->getActPos()) || !std::isfinite(backup->getActPos()) ||
           std::abs(source->getActPos()-backup->getActPos()) > encoderPowerDiffTol_)) {
        setEnable(false);
        setErrorID(ERROR_ENC_POWER_MISMATCH);
        return; // Retain counter feedback; never correct toward stale targets.
      }
      switchEncoderPowerFeedback(encoderPowerSourceIndex_);
      if (!autoEnableRequest_ && !motionStartPending_) encoderPowerMoveCompleted_ = true;
      if (autoEnableRequest_ && data_.control_.command != ECMC_CMD_HOMING)
        setTargetPos(data_.control_.positionTarget);
    }
  }
  if (!encoderPowerReady()) return;
  if (encoderPowerInitPending_) initEncoders();
  if (getPrimEnc()->consumePowerPositionSync()) {
    // Rebase on the fresh absolute sample before drive enable. A REL request
    // must use this position, not the last sample retained while powered off.
    const double position = getPrimEnc()->getActPos();
    data_.status_.currentPositionActual = position;
    data_.status_.currentPositionSetpoint = position;
    data_.status_.currentTargetPosition = position;
    data_.status_.currentVelocitySetpoint = 0;
    traj_->setStartPos(position);
    traj_->setCurrentPosSet(position);
    traj_->setTargetPos(position);
    if (autoEnableRequest_ && data_.control_.command != ECMC_CMD_HOMING)
      setTargetPos(data_.control_.positionTarget);
  }
}

int ecmcAxisBase::prepareEncoderPowerMotion() {
  if (getErrorID()) return getErrorID();
  if (getBlockCom()) return setExternalCommandBlockedError();
  if (getBlocked()) return setMasterSlaveCommandBlockedError();
  encoderPowerMoveCompleted_ = false;
  encoderPowerEnablePending_ = true;
  autoEnableRequest_ = true;
  motionStartPending_ = true;
  autoEnableTimeCounter_ = 0;
  getSeq()->setGlobalBusy(true);
  for (int i = 0; i < data_.status_.encoderCount; ++i)
    if (encArray_[i]->getPowerEnable()) encArray_[i]->wakePower();
  return 0;
}

void ecmcAxisBase::encoderPowerIdleSM() {
  const auto &word = data_.status_.statusWord_;
  if (data_.statusOld_.statusWord_.localBusy && !word.localBusy)
    encoderPowerMoveCompleted_ = true;
  if (!word.enable && !word.enabled && encoderPowerBackupIndex_ < 0)
    encoderPowerMoveCompleted_ = false;
  // The ordinary encoder path has no idle-power qualification to calculate.
  // Reuse this scan as the start of the existing powered-encoder loop.
  const int encoderCount = data_.status_.encoderCount;
  int firstPoweredEncoder = 0;
  while (firstPoweredEncoder < encoderCount &&
         !encArray_[firstPoweredEncoder]->getPowerEnable()) {
    ++firstPoweredEncoder;
  }
  if (firstPoweredEncoder == encoderCount) {
    if (encoderPowerBackupIndex_ >= 0) encoderPowerBackupSM();
    return;
  }
  const double deadband = mon_->getCtrlDeadband();
  const bool idleQualified = !word.busy &&
    !autoEnableRequest_ && !motionStartPending_ && !encoderPowerEnablePending_ &&
    !getErrorID() && word.attarget && mon_->getEnableAtTargetMon() &&
    mon_->getAtTargetTol() > 0 &&
    (deadband <= 0 || deadband >= mon_->getAtTargetTol() || mon_->getCtrlInDeadband()) &&
    word.trajsource == ECMC_DATA_SOURCE_INTERNAL;
  for (int i = firstPoweredEncoder; i < encoderCount; ++i) {
    if (!encArray_[i]->getPowerEnable()) continue;
    if (!mon_->getEnableAtTargetMon() || mon_->getAtTargetTol() <= 0) {
      setEnable(false);
      setErrorID(ERROR_ENC_POWER_CONFIG);
      return;
    }
    encArray_[i]->updatePowerIdle(!(encoderMode_ == i+1) && idleQualified &&
      (encoderPowerMoveCompleted_ || encArray_[i]->getPowerReleasePending()));
  }
  encoderPowerBackupSM();
}

bool ecmcAxisBase::encoderPowerReady() {
  if (encoderMode_ != 0) {
    auto *encoder = encArray_[encoderMode_-1];
    return data_.control_.primaryEncIndex == encoderMode_-1 && encoder->getPowerReady() &&
      encoder->hwReady() && encoder->checkAllDataValid() && !encoder->getErrorID();
  }
  if (encoderPowerBackupSelected()) return false;
  for (int i = 0; i < data_.status_.encoderCount; ++i) {
    if (encArray_[i]->getPowerEnable() && !encArray_[i]->getPowerReady()) return false;
  }
  return true;
}

void ecmcAxisBase::encoderPowerEnableSM() {
  int error = getErrorID();
  for (int i = 0; i < data_.status_.encoderCount && !error; ++i)
    error = encArray_[i]->getPowerError();
  if (error || getStartupCommandsHeld() || getBlockCom() || getBlocked() ||
      !data_.control_.controlWord_.enableCmd) {
    setEnable(false);
    if (error) setErrorID(error);
    return;
  }
  if (!encoderPowerReady()) return;

  encoderPowerEnablePending_ = false;
  const int enableError = setEnable(true);
  if (enableError) setErrorID(enableError);
}

void ecmcAxisBase::autoEnableSM() {
  if (encoderPowerEnablePending_) return;
  if (motionStartPending_ &&
      (!getEnable() || getError() || getBlocked() || getStartupCommandsHeld())) {
    motionStartPending_ = false;
    getSeq()->setGlobalBusy(false);
    return;
  }
  if( !enableAutoEnable_ && !motionStartPending_) {
    return;
  }

  if(!autoEnableRequest_ && !motionStartPending_) {
    return;
  }
  
  if(!data_.status_.statusWord_.enable) {
    if(data_.control_.controlWord_.enableDbgPrintout) {
      ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Axis[%d]: autoEnableSM(): enabling axis.\n",
                           __FILE__,
                           __FUNCTION__,
                           __LINE__,
                           data_.status_.axisId);
    }

    setEnable(true);
    // The encoder has its own power timeout. Start the normal drive-enable
    // timer only after fresh encoder feedback permits enabling the amplifier.
    if (encoderPowerEnablePending_) return;
  }

  if(autoEnableTimoutS_ > 0 && autoEnableTimeCounter_ > autoEnableTimoutS_) {
    // Capture the wait condition before disabling clears pending/busy state.
    ecmcRtLoggerLogError(
      "Axis[%d]: auto-enable timeout: enable=%d enabled=%d previousEnabled=%d "
      "autoRequest=%d startPending=%d startAllowed=%d globalBusy=%d localBusy=%d "
      "blocked=%d error=0x%x elapsed=%g limit=%g.\n",
      data_.status_.axisId, data_.status_.statusWord_.enable,
      data_.status_.statusWord_.enabled, data_.statusOld_.statusWord_.enabled,
      autoEnableRequest_, motionStartPending_,
      !motionStartPermissive_ || *motionStartPermissive_,
      data_.status_.statusWord_.globalBusy, data_.status_.statusWord_.localBusy,
      getBlocked(), getErrorID(), autoEnableTimeCounter_, autoEnableTimoutS_);
    setEnable(false);
    autoEnableRequest_= false;
    autoEnableTimeCounter_ = 0;
    getSeq()->setGlobalBusy(false);
    setErrorID(ERROR_AXIS_AUTO_ENABLE_TIMEOUT);
    return;
  } else {
    autoEnableTimeCounter_ += data_.status_.sampleTime;
  }

  if(data_.status_.statusWord_.enabled && data_.status_.statusWord_.enable &&
     data_.statusOld_.statusWord_.enabled &&
     (!motionStartPermissive_ || *motionStartPermissive_)) {
    autoEnableTimeCounter_ = 0;
    autoEnableRequest_= false;
    motionStartPending_ = false;
    if(data_.control_.controlWord_.enableDbgPrintout) {
      ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Axis[%d]: autoEnableSM(): axis enabled, triggering motion.\n",
                           __FILE__,
                           __FUNCTION__,
                           __LINE__,
                           data_.status_.axisId);
    }
    
    setExecute(0,1); // need ignore busy
    setExecute(1,1); // need ignore busy 
    getSeq()->setGlobalBusy(false); // Need to clear the global busy bit. Now seq->busy is used
    return;
  }
}

void ecmcAxisBase::autoDisableSM() {
  if(autoDisableAfterS_ < 0 || !enableAutoDisable_ ) {
    return;
  }

  if(data_.status_.statusWord_.busy) {
    autoDisbleTimeCounter_ = 0;
    autoDisableAtTargetLatched_ = false;
    return;
  }

  if(!data_.status_.statusWord_.enabled) {
    autoDisbleTimeCounter_ = 0;
    autoDisableAtTargetLatched_ = false;
    return;
  }

  if(getMon()->getEnableAtTargetMon()) {
    if(data_.status_.statusWord_.attarget) {
      autoDisableAtTargetLatched_ = true;
    }

    if(autoDisableAtTgtLtchEnable_) {
      if(!autoDisableAtTargetLatched_) {
        autoDisbleTimeCounter_ = 0;
        return;
      }
    } else if(!data_.status_.statusWord_.attarget) {
      autoDisbleTimeCounter_ = 0;
      return;
    }
  }

  if(autoDisbleTimeCounter_ > autoDisableAfterS_ && data_.control_.controlWord_.enableCmd) {
    if(data_.control_.controlWord_.enableDbgPrintout) {
      ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Axis[%d]: autoDisableSM(): auto-disabling axis after %lf s idle.\n",
                           __FILE__,
                           __FUNCTION__,
                           __LINE__,
                           data_.status_.axisId,
                           autoDisableAfterS_);
    }

    setEnable(0);
  } else {
    autoDisbleTimeCounter_+= data_.status_.sampleTime;
  }
}

int ecmcAxisBase::setAutoEnableTimeout(double timeS) {
  autoEnableTimoutS_ = timeS;
  enableAutoEnable_ = timeS > 0;
  return 0;
}

double ecmcAxisBase::getAutoEnableTimeout() {
  return autoEnableTimoutS_;
}

int ecmcAxisBase::setEnableAutoEnable(bool enable) {
  if(enableAutoEnable_ != enable) {
    autoDisbleTimeCounter_ = 0;
  }
  enableAutoEnable_ = enable;
  return 0;
}

int ecmcAxisBase::getEnableAutoEnable() {
  return static_cast<int>(enableAutoEnable_);
}

int ecmcAxisBase::setAutoDisableAfterTime(double timeS) {
  autoDisableAfterS_ = timeS;
  enableAutoDisable_ = timeS > 0;
  return 0;
}

double ecmcAxisBase::getAutoDisableAfterTime() {
  return autoDisableAfterS_;
}

int ecmcAxisBase::setEnableAutoDisable(bool enable) {
  if(enableAutoDisable_ != enable) {
    autoDisbleTimeCounter_ = 0;
    autoDisableAtTargetLatched_ = enable &&
                                  autoDisableAtTgtLtchEnable_ &&
                                  data_.status_.statusWord_.attarget;
  }
  
  enableAutoDisable_ = enable;
  return 0;
}

int ecmcAxisBase::getEnableAutoDisable() {
  return static_cast<int>(enableAutoDisable_);
}

int ecmcAxisBase::setAutoDisableAtTgtLtch(bool enable) {
  if(autoDisableAtTgtLtchEnable_ != enable) {
    autoDisbleTimeCounter_ = 0;
    autoDisableAtTargetLatched_ = false;
  }

  autoDisableAtTgtLtchEnable_ = enable;
  return 0;
}

int ecmcAxisBase::getAutoDisableAtTgtLtch() {
  return static_cast<int>(autoDisableAtTgtLtchEnable_);
}

ecmcAxisDataStatus* ecmcAxisBase::getAxisStatusDataPtr() {
  return &data_.status_;
}

void ecmcAxisBase::refreshAsynTargetValue() {
  positionTargetAsyn_ = data_.control_.positionTarget;
  data_.axAsynParams_[ECMC_ASYN_AX_TARG_POS_ID]->refreshParamRT(1);  
}

int ecmcAxisBase::setCntrlKp(double kp) {
  ecmcPIDController *cntrl = getCntrl();
  if(!cntrl) {
    return ERROR_AXIS_CNTRL_OBJECT_NULL;
  }
  getCntrl()->setKp(kp);
  if (!warnRuntimeControllerKpZero(this, "controller.kp", kp)) {
    logRuntimeAxisDoubleChange(this, "controller.kp", kp);
  }
  return 0;
}

int ecmcAxisBase::setCntrlKi(double ki) {
  ecmcPIDController *cntrl = getCntrl();
  if(!cntrl) {
    return ERROR_AXIS_CNTRL_OBJECT_NULL;
  }
  getCntrl()->setKi(ki);
  logRuntimeAxisDoubleChange(this, "controller.ki", ki);
  return 0;
}

int ecmcAxisBase::setCntrlKd(double kd) {
  ecmcPIDController *cntrl = getCntrl();
  if(!cntrl) {
    return ERROR_AXIS_CNTRL_OBJECT_NULL;
  }
  getCntrl()->setKd(kd);
  logRuntimeAxisDoubleChange(this, "controller.kd", kd);
  return 0;
}

int ecmcAxisBase::setCntrlKff(double kff) {
  ecmcPIDController *cntrl = getCntrl();
  if(!cntrl) {
    return ERROR_AXIS_CNTRL_OBJECT_NULL;
  }
  getCntrl()->setKff(kff);
  logRuntimeAxisDoubleChange(this, "controller.kff", kff);
  return 0;
}

bool ecmcAxisBase::commandValid(motionCommandTypes command) {

  bool valid = false;
  switch((motionCommandTypes)command) {
    case ECMC_CMD_NOCMD:
      valid = true;
      break;
    case ECMC_CMD_MOVEVEL:
      valid = true;
      break;
    case ECMC_CMD_MOVEREL:
      valid = true;
      break;
    case ECMC_CMD_MOVEABS:
      valid = true;
      break;
    case ECMC_CMD_MOVEMODULO:
      valid = false;   // NOT valid!!
      break;
    case ECMC_CMD_MOVEPVTABS:
      ecmcRtLoggerLogError(
        "%s/%s:%d: ERROR: Axis[%d]: PVT motion can only be executed from the dedicated \"Profile move\" pvs.\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        data_.status_.axisId);
      valid = false;
      break;
    case ECMC_CMD_HOMING:
      valid = true;
      break;
    default:
      valid = false;
      break;
  }
  return valid;
}

ecmcAxisDataStatus* ecmcAxisBase::getAxisStatusStruct() {
  return &data_.status_;
}

int ecmcAxisBase::setEnableAtStartup(bool enable) {
  data_.control_.enableAtStartup = enable;
  return 0;
}

bool ecmcAxisBase::getGlobalBusy() {
  return data_.status_.statusWord_.globalBusy;
}

bool ecmcAxisBase::getLocalBusy() {
  return data_.status_.statusWord_.localBusy;
}

// set Group blocked
void ecmcAxisBase::setBlocked(bool blocked) {
  const bool oldBlocked = masterSlaveBlocked_;
  masterSlaveBlocked_               = blocked;
  data_.status_.statusWord_.blocked = getBlocked();
  if (oldBlocked != masterSlaveBlocked_) {
    ecmcRtLoggerPortDriverSetAxisMasterSlaveBlock(data_.status_.axisId,
                                                  masterSlaveBlocked_ ? 1 : 0,
                                                  getCycleCounter());
  }
}

// get Group blocked
bool ecmcAxisBase::getBlocked() {
  return masterSlaveBlocked_ || getBlockCom();
}

ecmcAxisDataType ecmcAxisBase::parseDataSource(const char *path, int *isBoolean) {
  if (isBoolean) *isBoolean = 0;
  if (!path || !strstr(path, ECMC_AX_STR)) return ECMC_AXIS_DATA_NONE;
  const char *name = strchr(path, '.');
  if (!name) return ECMC_AXIS_DATA_NONE;
  ++name;
  struct Name { const char *name; ecmcAxisDataType type; int boolean; };
  static const Name names[] = {
    {ECMC_AXIS_DATA_STR_AXIS_ID, ECMC_AXIS_DATA_AXIS_ID, 0},
    {ECMC_AXIS_DATA_STR_POS_SET, ECMC_AXIS_DATA_POS_SET, 0},
    {ECMC_AXIS_DATA_STR_POS_ACT, ECMC_AXIS_DATA_POS_ACT, 0},
    {ECMC_AXIS_DATA_STR_POS_TARGET, ECMC_AXIS_DATA_POS_TARGET, 0},
    {ECMC_AXIS_DATA_STR_POS_ERROR, ECMC_AXIS_DATA_POS_ERROR, 0},
    {ECMC_AXIS_DATA_STR_POS_RAW, ECMC_AXIS_DATA_POS_RAW, 0},
    {ECMC_AXIS_DATA_STR_CNTRL_OUT, ECMC_AXIS_DATA_CNTRL_OUT, 0},
    {ECMC_AXIS_DATA_STR_VEL_SET, ECMC_AXIS_DATA_VEL_SET, 0},
    {ECMC_AXIS_DATA_STR_VEL_ACT, ECMC_AXIS_DATA_VEL_ACT, 0},
    {ECMC_AXIS_DATA_STR_VEL_SET_FF_RAW, ECMC_AXIS_DATA_VEL_SET_FF_RAW, 0},
    {ECMC_AXIS_DATA_STR_VEL_SET_RAW, ECMC_AXIS_DATA_VEL_SET_RAW, 0},
    {ECMC_AXIS_DATA_STR_CYCLE_COUNTER, ECMC_AXIS_DATA_CYCLE_COUNTER, 0},
    {ECMC_AXIS_DATA_STR_ERROR, ECMC_AXIS_DATA_ERROR, 0},
    {ECMC_AXIS_DATA_STR_COMMAND, ECMC_AXIS_DATA_COMMAND, 0},
    {ECMC_AXIS_DATA_STR_CMD_DATA, ECMC_AXIS_DATA_CMD_DATA, 0},
    {ECMC_AXIS_DATA_STR_SEQ_STATE, ECMC_AXIS_DATA_SEQ_STATE, 0},
    {ECMC_AXIS_DATA_STR_INTERLOCK_TYPE, ECMC_AXIS_DATA_INTERLOCK_TYPE, 0},
    {ECMC_AXIS_DATA_STR_TRAJ_SOURCE, ECMC_AXIS_DATA_TRAJ_SOURCE, 1},
    {ECMC_AXIS_DATA_STR_ENC_SOURCE, ECMC_AXIS_DATA_ENC_SOURCE, 1},
    {ECMC_AXIS_DATA_STR_ENABLE, ECMC_AXIS_DATA_ENABLE, 1},
    {ECMC_AXIS_DATA_STR_ENABLED, ECMC_AXIS_DATA_ENABLED, 1},
    {ECMC_AXIS_DATA_STR_EXECUTE, ECMC_AXIS_DATA_EXECUTE, 1},
    {ECMC_AXIS_DATA_STR_BUSY, ECMC_AXIS_DATA_BUSY, 1},
    {ECMC_AXIS_DATA_STR_AT_TARGET, ECMC_AXIS_DATA_AT_TARGET, 1},
    {ECMC_AXIS_DATA_STR_HOMED, ECMC_AXIS_DATA_HOMED, 1},
    {ECMC_AXIS_DATA_STR_LIMIT_BWD, ECMC_AXIS_DATA_LIMIT_BWD, 1},
    {ECMC_AXIS_DATA_STR_LIMIT_FWD, ECMC_AXIS_DATA_LIMIT_FWD, 1},
    {ECMC_AXIS_DATA_STR_HOME_SWITCH, ECMC_AXIS_DATA_HOME_SWITCH, 1},
    {ECMC_AXIS_DATA_STR_RESET, ECMC_AXIS_DATA_RESET, 1},
    {ECMC_AXIS_DATA_STR_VEL_TARGET_SET, ECMC_AXIS_DATA_VEL_TARGET_SET, 0},
    {ECMC_AXIS_DATA_STR_ACC_TARGET_SET, ECMC_AXIS_DATA_ACC_TARGET_SET, 0},
    {ECMC_AXIS_DATA_STR_DEC_TARGET_SET, ECMC_AXIS_DATA_DEC_TARGET_SET, 0},
    {ECMC_AXIS_DATA_STR_SOFT_LIMIT_BWD, ECMC_AXIS_DATA_SOFT_LIMIT_BWD, 0},
    {ECMC_AXIS_DATA_STR_SOFT_LIMIT_FWD, ECMC_AXIS_DATA_SOFT_LIMIT_FWD, 0},
    {ECMC_AXIS_DATA_STR_SOFT_LIMIT_BWD_ENABLE, ECMC_AXIS_DATA_SOFT_LIMIT_BWD_ENABLE, 1},
    {ECMC_AXIS_DATA_STR_SOFT_LIMIT_FWD_ENABLE, ECMC_AXIS_DATA_SOFT_LIMIT_FWD_ENABLE, 1},
    {ECMC_AXIS_DATA_STR_TRAJ_DIRECTION, ECMC_AXIS_DATA_TRAJ_DIRECTION, 0},
    {ECMC_AXIS_DATA_STR_ENC_HOMEPOS, ECMC_AXIS_DATA_ENC_HOMEPOS, 0},
    {ECMC_AXIS_DATA_STR_BLOCK_COM, ECMC_AXIS_DATA_BLOCK_COM, 1},
    {ECMC_AXIS_DATA_STR_INTERLOCK_FWD_TYPE, ECMC_AXIS_DATA_INTERLOCK_FWD_TYPE, 1},
    {ECMC_AXIS_DATA_STR_INTERLOCK_BWD_TYPE, ECMC_AXIS_DATA_INTERLOCK_BWD_TYPE, 1},
    {ECMC_AXIS_DATA_STR_ALLOW_PLC_CMD, ECMC_AXIS_DATA_ALLOW_PLC_WRITE, 1},
    {ECMC_AXIS_DATA_STR_POS_SET_EXTERNAL, ECMC_AXIS_DATA_POS_SET_EXTERNAL, 0},
    {ECMC_AXIS_DATA_STR_POS_ACT_EXTERNAL, ECMC_AXIS_DATA_POS_ACT_EXTERNAL, 0},
    {ECMC_AXIS_DATA_STR_CTRL_KP_CMD, ECMC_AXIS_DATA_CTRL_KP, 0},
    {ECMC_AXIS_DATA_STR_CTRL_KI_CMD, ECMC_AXIS_DATA_CTRL_KI, 0},
    {ECMC_AXIS_DATA_STR_CTRL_KD_CMD, ECMC_AXIS_DATA_CTRL_KD, 0},
    {ECMC_AXIS_DATA_STR_CTRL_KFF_CMD, ECMC_AXIS_DATA_CTRL_KFF, 0},
  };
  for (const auto &entry : names) {
    if (strcmp(name, entry.name)) continue;
    if (isBoolean) *isBoolean = entry.boolean;
    return entry.type;
  }
  return ECMC_AXIS_DATA_NONE;
}

int ecmcAxisBase::readDataSource(ecmcAxisDataType source, double *value) {
  auto *axis = this;
  if (!value || !getTraj()) return ERROR_AXIS_TRAJ_OBJECT_NULL;
  ecmcAxisDataStatus * const axisData = axis->getAxisStatusDataPtr();
  // Shared reader for bit fields and values computed by motion objects.
  auto * const mon  = axis->getMon();
  auto * const seq  = axis->getSeq();
  auto * const traj = axis->getTraj();
  auto * const cntrl = axis->getCntrl();

  switch (source) {
  case ECMC_AXIS_DATA_NONE:
    *value = 0;
    break;

  case ECMC_AXIS_DATA_AXIS_ID:
    *value = static_cast<double>(axisData->axisId);
    break;

  case ECMC_AXIS_DATA_POS_SET:
    *value = axisData->currentPositionSetpoint;
    break;

  case ECMC_AXIS_DATA_POS_ACT:
    *value = axisData->currentPositionActual;
    break;

  case ECMC_AXIS_DATA_CNTRL_ERROR:
    *value = axisData->cntrlError;
    break;

  case ECMC_AXIS_DATA_POS_TARGET:
    *value = axisData->currentTargetPosition;
    break;

  case ECMC_AXIS_DATA_POS_ERROR:
    *value = axisData->currentPositionSetpoint - axisData->currentPositionActual;
    break;

  case ECMC_AXIS_DATA_POS_RAW:
    // Risc of data loss
    *value = static_cast<double>(axisData->currentPositionActualRaw);
    break;

  case ECMC_AXIS_DATA_CNTRL_OUT:
    *value = axisData->cntrlOutput;
    break;

  case ECMC_AXIS_DATA_VEL_SET:
    *value = axisData->currentVelocitySetpoint;
    break;

  case ECMC_AXIS_DATA_VEL_ACT:
    *value = axisData->currentVelocityActual;
    break;

  case ECMC_AXIS_DATA_VEL_SET_FF_RAW:
    *value = axisData->currentvelocityFFRaw;
    break;

  case ECMC_AXIS_DATA_VEL_SET_RAW:
    *value = static_cast<double>(axisData->currentVelocitySetpointRaw);
    break;

  case ECMC_AXIS_DATA_CYCLE_COUNTER:
    *value = static_cast<double>(axisData->cycleCounter);
    break;

  case ECMC_AXIS_DATA_ERROR:
    *value = static_cast<double>(axisData->errorCode);
    break;

  case ECMC_AXIS_DATA_COMMAND:
    *value = static_cast<double>(axisData->command);
    break;

  case ECMC_AXIS_DATA_CMD_DATA:
    *value = static_cast<double>(axisData->cmdData);
    break;

  case ECMC_AXIS_DATA_SEQ_STATE:
    *value = static_cast<double>(axisData->statusWord2_.seqstate);
    break;

  case ECMC_AXIS_DATA_INTERLOCK_TYPE:
    *value = static_cast<double>(axis->getSumInterlock() == 0);
    break;

  case ECMC_AXIS_DATA_TRAJ_SOURCE:
    *value = static_cast<double>(axisData->statusWord_.trajsource);
    break;

  case ECMC_AXIS_DATA_ENC_SOURCE:
    *value = static_cast<double>(axisData->statusWord_.encsource);
    break;

  case ECMC_AXIS_DATA_ENABLE:

    *value = static_cast<double>(axisData->statusWord_.enable);
    break;

  case ECMC_AXIS_DATA_ENABLED:
    *value = static_cast<double>(axisData->statusWord_.enabled);
    break;

  case ECMC_AXIS_DATA_EXECUTE:
    *value = static_cast<double>(axisData->statusWord_.execute);
    break;

  case ECMC_AXIS_DATA_BUSY:
    *value = static_cast<double>(axisData->statusWord_.busy);
    break;

  case ECMC_AXIS_DATA_AT_TARGET:
    *value = static_cast<double>(axisData->statusWord_.attarget);
    break;

  case ECMC_AXIS_DATA_HOMED:
    *value = static_cast<double>(axisData->statusWord_.homed);
    break;

  case ECMC_AXIS_DATA_LIMIT_BWD:
    *value = static_cast<double>(axisData->statusWord_.limitbwd);
    break;

  case ECMC_AXIS_DATA_LIMIT_FWD:
    *value = static_cast<double>(axisData->statusWord_.limitfwd);
    break;

  case ECMC_AXIS_DATA_HOME_SWITCH:
    *value = static_cast<double>(axisData->statusWord_.homeswitch);
    break;

  case ECMC_AXIS_DATA_RESET:
    *value = 0;
    break;

  case ECMC_AXIS_DATA_VEL_TARGET_SET:
    *value = seq->getTargetVel();
    break;

  case ECMC_AXIS_DATA_ACC_TARGET_SET:
    *value = traj->getAcc();
    break;

  case ECMC_AXIS_DATA_DEC_TARGET_SET:
    *value = traj->getDec();
    break;

  case ECMC_AXIS_DATA_SOFT_LIMIT_BWD:
    *value = mon->getSoftLimitBwd();
    break;

  case ECMC_AXIS_DATA_SOFT_LIMIT_FWD:
    *value = mon->getSoftLimitFwd();
    break;

  case ECMC_AXIS_DATA_SOFT_LIMIT_BWD_ENABLE:
    *value = static_cast<double>(mon->getEnableSoftLimitBwd());
    break;

  case ECMC_AXIS_DATA_SOFT_LIMIT_FWD_ENABLE:
    *value = static_cast<double>(mon->getEnableSoftLimitFwd());
    break;

  case ECMC_AXIS_DATA_TRAJ_DIRECTION:

    switch (axis->getAxisSetDirection()) {
    case ECMC_DIR_BACKWARD:
      *value = -1;
      break;

    case ECMC_DIR_FORWARD:
      *value = 1;
      break;

    case ECMC_DIR_STANDSTILL:
      *value = 0;
      break;
    }
    break;

  case ECMC_AXIS_DATA_ENC_HOMEPOS:
    *value = seq->getHomePosition();
    break;

  case ECMC_AXIS_DATA_BLOCK_COM:
    *value = axis->getBlockCom();
    break;

  case ECMC_AXIS_DATA_INTERLOCK_BWD_TYPE:
    *value =
      static_cast<double>(axisData->statusWord_.sumilockbwd == 0);
    break;

  case ECMC_AXIS_DATA_INTERLOCK_FWD_TYPE:
    *value =
      static_cast<double>(axisData->statusWord_.sumilockfwd == 0);
    break;

  case ECMC_AXIS_DATA_ALLOW_PLC_WRITE:
    *value = static_cast<double>(axis->getAllowCmdFromPLC());
    break;

  case ECMC_AXIS_DATA_POS_SET_EXTERNAL:
    *value = axis->getExtSetPos();
    break;

  case ECMC_AXIS_DATA_POS_ACT_EXTERNAL:
    *value = axis->getExtActPos();
    break;

  case ECMC_AXIS_DATA_CTRL_KP:

    if (cntrl) {
      *value = cntrl->getKp();
    } else {
      *value = 0;
    }
    break;

  case ECMC_AXIS_DATA_CTRL_KI:

    if (cntrl) {
      *value = cntrl->getKi();
    } else {
      *value = 0;
    }
    break;

  case ECMC_AXIS_DATA_CTRL_KD:

    if (cntrl) {
      *value = cntrl->getKd();
    } else {
      *value = 0;
    }
    break;

  case ECMC_AXIS_DATA_CTRL_KFF:

    if (cntrl) {
      *value = cntrl->getKff();
    } else {
      *value = 0;
    }
    break;

  default:
    return ERROR_AXIS_SOURCE_TYPE_NOT_DEFINED;

    break;
  }

  return 0;
}
std::shared_ptr<ecmcDataAccess> ecmcAxisBase::getDataSource(const char *path) {
  if (!path || strncmp(path, ECMC_AX_STR, strlen(ECMC_AX_STR))) return nullptr;
  const char *index = path + strlen(ECMC_AX_STR);
  if (*index < '0' || *index > '9') return nullptr;
  int axisId = 0;
  while (*index >= '0' && *index <= '9') {
    axisId = axisId * 10 + (*index++ - '0');
    if (axisId >= ECMC_MAX_AXES) return nullptr;
  }
  if (*index != '.' || axisId != data_.status_.axisId) return nullptr;
  int boolean = 0;
  const auto source = parseDataSource(path, &boolean);
  if (source == ECMC_AXIS_DATA_NONE) return nullptr;
  for (const auto &entry : dataSources_)
    if (entry->selector == source) return entry;
  auto view = std::make_shared<ecmcDataAccess>();
  view->identity = this;
  view->selector = source;
  view->data = reinterpret_cast<uint8_t *>(this); // Lifetime sentinel for computed sources.
  view->type = ECMC_EC_F64;
  view->bits = 64;
  view->positiveBoolean = boolean;
  view->numberReader = [](const ecmcDataAccess &access, double *value) {
    return static_cast<ecmcAxisBase *>(access.identity)->readDataSource(
      static_cast<ecmcAxisDataType>(access.selector), value);
  };
  switch (source) {
    case ECMC_AXIS_DATA_AXIS_ID:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.axisId);
      view->type = ECMC_EC_S32; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_POS_SET:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.currentPositionSetpoint);
      view->type = ECMC_EC_F64; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_POS_ACT:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.currentPositionActual);
      view->type = ECMC_EC_F64; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_CNTRL_ERROR:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.cntrlError);
      view->type = ECMC_EC_F64; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_POS_TARGET:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.currentTargetPosition);
      view->type = ECMC_EC_F64; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_POS_RAW:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.currentPositionActualRaw);
      view->type = ECMC_EC_S64; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_CNTRL_OUT:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.cntrlOutput);
      view->type = ECMC_EC_F64; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_VEL_SET:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.currentVelocitySetpoint);
      view->type = ECMC_EC_F64; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_VEL_ACT:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.currentVelocityActual);
      view->type = ECMC_EC_F64; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_VEL_SET_FF_RAW:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.currentvelocityFFRaw);
      view->type = ECMC_EC_F64; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_VEL_SET_RAW:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.currentVelocitySetpointRaw);
      view->type = ECMC_EC_S64; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_CYCLE_COUNTER:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.cycleCounter);
      view->type = ECMC_EC_U32; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_ERROR:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.errorCode);
      view->type = ECMC_EC_S32; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_COMMAND:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.command);
      view->type = ECMC_EC_S32; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_CMD_DATA:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.cmdData);
      view->type = ECMC_EC_S32; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_POS_SET_EXTERNAL:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.externalTrajectoryPosition);
      view->type = ECMC_EC_F64; view->numberReader = nullptr; break;
    case ECMC_AXIS_DATA_POS_ACT_EXTERNAL:
      view->data = reinterpret_cast<uint8_t *>(&data_.status_.externalEncoderPosition);
      view->type = ECMC_EC_F64; view->numberReader = nullptr; break;
    default: break;
  }
  view->bits = static_cast<int>(ecmcDataAccess::sizeOf(view->type) * 8);
  view->finiteRequired = view->type == ECMC_EC_F32 || view->type == ECMC_EC_F64;
  dataSources_.push_back(view); // Configuration only; consumers keep a stable descriptor.
  return view;
}

void ecmcAxisBase::invalidateDataSources() {
  for (const auto &view : dataSources_) {
    view->data = nullptr;
    view->numberReader = nullptr;
  }
  dataSources_.clear();
}

// Admission check: callers must use this before changing the active motion.
bool ecmcAxisBase::motionCommandDisabled(motionCommandTypes command) {
  if (!ignoreDisabledMotionCmds_) return false;
  switch (command) {
  case ECMC_CMD_MOVEABS:
  case ECMC_CMD_MOVEREL:
  case ECMC_CMD_MOVEPVTABS: return !getAllowPos();
  case ECMC_CMD_MOVEVEL:
  case ECMC_CMD_JOG: return !getAllowConstVelo();
  case ECMC_CMD_HOMING: return !getAllowHome();
  default: return false;
  }
}

bool ecmcAxisBase::ignoreMotionCommand(motionCommandTypes command) {
  if (!motionCommandDisabled(command)) return false;
  ++ignoredMotionCommandCount_;
  lastIgnoredMotionCommand_ = ignoredMotionMessage_ = command;
  return true;
}
