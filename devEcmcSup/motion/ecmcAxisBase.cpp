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
#include <inttypes.h>
#include <stdint.h>
#include <string>
#include <new>
#include <iostream>
#include "ecmcMotion.h"
#include "ecmcErrorsList.h"

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
  data_.axisId_     = axisID;
  data_.sampleTime_ = sampleTime;
  data_.control_.cspDrvEncIndex =  -1;   // used for control
  setExternalPtrs(&(data_.status_.errorCode), &(data_.status_.warningCode));
  try {
    data_.control_.primaryEncIndex = 0;
    addEncoder();
    currentTrajType_ = trajType;

    if (currentTrajType_ == ECMC_S_CURVE) {
      traj_ = new ecmcTrajectoryS(&data_,
                                  data_.sampleTime_);
    } else {
      traj_ = new ecmcTrajectoryTrapetz(&data_,
                                        data_.sampleTime_);
    }

    mon_ = new ecmcMonitor(&data_, encArray_);

    extTrajVeloFilter_ = new ecmcFilter(data_.sampleTime_);
    extEncVeloFilter_  = new ecmcFilter(data_.sampleTime_);
  } catch (std::bad_alloc& ex) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Mem alloc error.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_);

    exit(1);
  }
  seq_.init(sampleTime);
  seq_.setAxisDataRef(&data_);
  seq_.setTraj(traj_);
  seq_.setMon(mon_);
  seq_.setEnc(encArray_);

  initAsyn();
}

ecmcAxisBase::~ecmcAxisBase() {
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
  // errorReset();  //THIS IS NONO..
  firstEnableDone_                       = 0;
  data_.axisType_                          = ECMC_AXIS_TYPE_BASE;
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
  data_.sampleTime_                        = 1 / 1000;
  printHeaderCounter_      = 0;
  cycleCounter_            = 0;
  axisState_               = ECMC_AXIS_STATE_STARTUP;
  oldPositionAct_          = 0;
  oldPositionSet_          = 0;
  asynPortDriver_          = NULL;
  memset(&mrCmds_,    0, sizeof(mrCmds_));
  memset(&mrCmdsOld_,    0, sizeof(mrCmdsOld_));

  for (int i = 0; i < ECMC_ASYN_AX_PAR_COUNT; i++) {
    axAsynParams_[i] = NULL;
  }

  statusOutputEntry_ = 0;

  blockExtCom_ = 0;
  memset(diagBuffer_, 0, AX_MAX_DIAG_STRING_CHAR_LENGTH);
  extTrajVeloFilter_         = NULL;
  extEncVeloFilter_          = NULL;
  enableExtTrajVeloFilter_   = false;
  enableExtEncVeloFilter_    = false;
  disableAxisAtErrorReset_   = false;
  positionTarget_            = 0;
  velocityTarget_            = 0;
  command_                   = ECMC_CMD_MOVEABS;
  cmdData_                   = 0;
  data_.status_.encoderCount = 0;

  for (int i = 0; i < ECMC_MAX_ENCODERS; i++) {
    encArray_[i] = NULL;
  }
  data_.control_.cfgEncIndex    = 0;
  allowSourceChangeWhenEnabled_ = false;
  setEncoderPos_                = 0;
  encPrimIndexAsyn_             = 1;
  acceleration_                 = 0;
  deceleration_                 = 0;
  hwReady_                      = 0;
  hwReadyOld_                   = 0;
  globalBusy_                   = 0;
  ignoreMRDisableStatusCheck_   = 0;
  autoEnableTimoutS_            = -1.0;
  autoDisableAfterS_            = -1.0;
  autoEnableRequest_            = false;
  autoEnableTimeCounter_        = 0.0;
  autoDisbleTimeCounter_        = 0.0;
  enableCmd_                    = 0;
  enableAutoEnable_             = 0;
  enableAutoDisable_            = 0;
}

void ecmcAxisBase::preExecute(bool masterOK) {
  hwReadyOld_ = hwReady_;

  data_.interlocks_.etherCatMasterInterlock = !masterOK;

  if (!masterOK) {
    setEnable(false);
  }
  data_.refreshInterlocks();

  data_.status_.statusWord_.moving = data_.status_.statusWord_.busy && std::abs(
    data_.status_.currentVelocityActual) > 0; /*|| getTraj()->getBusy();*/

  for (int i = 0; i < data_.status_.encoderCount; i++) {
    if (encArray_[i] == NULL) {
      break;
    }

    encArray_[i]->readEntries(masterOK);
  }
  
  hwReady_ = getHwReady();

  // Axis state machine
  switch (axisState_) {
  case ECMC_AXIS_STATE_STARTUP:
    setEnable(false);
    data_.status_.statusWord_.busy       = true;
    data_.status_.distToStop = 0;

    if (masterOK) {
      
      if (!hwReady_) {
        setErrorID(ERROR_AXIS_HW_NOT_READY);
      } else {  // auto reset error if ok
        if (getErrorID() == ERROR_AXIS_HW_NOT_READY && hwReadyOld_) {
          errorReset();
        }
      }
    }

    if (masterOK && hwReady_ && hwReadyOld_) {
      // Auto reset hardware error if starting up
      if ((getErrorID() == ERROR_AXIS_HARDWARE_STATUS_NOT_OK) &&
          data_.status_.statusWord_.instartup) {
        errorReset();
      }

      // bus ok and hw ok, startup finished
      setInStartupPhase(false);

      // only init encoders after first startup
      if (!data_.status_.startupFinsished) {
        initEncoders();
      }

      data_.status_.startupFinsished = true;


      axisState_ = ECMC_AXIS_STATE_DISABLED;
    }
    break;

  case ECMC_AXIS_STATE_DISABLED:
    data_.status_.statusWord_.busy       = false;
    data_.status_.distToStop = 0;

    if (data_.status_.statusWord_.enabled) {
      axisState_ = ECMC_AXIS_STATE_ENABLED;
    }

    if (!masterOK) {
      LOGERR(
        "%s/%s:%d: ERROR (axis %d): State change (ECMC_AXIS_STATE_DISABLED->ECMC_AXIS_STATE_STARTUP).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        data_.axisId_);
      axisState_ = ECMC_AXIS_STATE_STARTUP;
    }
    break;

  case ECMC_AXIS_STATE_ENABLED:
    data_.status_.distToStop = traj_->distToStop(
      data_.status_.currentVelocitySetpoint);

    if (!data_.status_.statusWord_.enabled) {
      axisState_ = ECMC_AXIS_STATE_DISABLED;
    }

    if (!masterOK) {
      LOGERR(
        "%s/%s:%d: ERROR (axis %d): State change (ECMC_AXIS_STATE_ENABLED->ECMC_AXIS_STATE_STARTUP).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        data_.axisId_);
      axisState_ = ECMC_AXIS_STATE_STARTUP;
    }

    break;
  }
  mon_->readEntries();

  // Filter velocities from PLC source
  // Traj
  if (enableExtTrajVeloFilter_ && extTrajVeloFilter_) {
    data_.status_.externalTrajectoryVelocity = extTrajVeloFilter_->getFiltVelo(
      data_.status_.externalTrajectoryPosition -
      data_.status_.externalTrajectoryPositionOld);
  } else {
    data_.status_.externalTrajectoryVelocity =
      (data_.status_.externalTrajectoryPosition -
       data_.status_.
       externalTrajectoryPositionOld) / data_.sampleTime_;
  }

  // Enc
  if (enableExtEncVeloFilter_ && extEncVeloFilter_) {
    data_.status_.externalEncoderVelocity = extEncVeloFilter_->getFiltVelo(
      data_.status_.externalEncoderPosition -
      data_.status_.externalEncoderPositionOld);
  } else {
    data_.status_.externalEncoderVelocity =
      (data_.status_.externalEncoderPosition -
       data_.status_.
       externalEncoderPositionOld) / data_.sampleTime_;
  }
}

void ecmcAxisBase::postExecute(bool masterOK) {

  autoEnableSM();
  autoDisableSM();

  data_.status_.externalTrajectoryPositionOld =
    data_.status_.externalTrajectoryPosition;
  data_.status_.externalEncoderPositionOld =
    data_.status_.externalEncoderPosition;

  // Write encoder entries
  for (int i = 0; i < data_.status_.encoderCount; i++) {
    encArray_[i]->writeEntries();
  }
  
  cycleCounter_++;

  // Update asyn parameters
  axAsynParams_[ECMC_ASYN_AX_SET_POS_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_ACT_POS_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_ACT_VEL_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_POS_ERR_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_STATUS_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_ERROR_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_WARNING_ID]->refreshParamRT(0);

  if(memcmp(&mrCmdsOld_,&mrCmds_, sizeof(mrCmdsOld_)) !=0) {
    axAsynParams_[ECMC_ASYN_AX_MR_CMD_ID]->refreshParamRT(1);
  }
  memcpy(&mrCmdsOld_,&mrCmds_, sizeof(mrCmdsOld_));
  
  // Update status entry if linked
  if (statusOutputEntry_) {
    statusOutputEntry_->writeValue(getErrorID() == 0);
  }

  // Data for last scan
  memcpy(&data_.statusOld_,&data_.status_,sizeof(data_.status_));
  memcpy(&data_.controlOld_,&data_.control_,sizeof(data_.control_));
}

axisType ecmcAxisBase::getAxisType() {
  return data_.axisType_;
}

int ecmcAxisBase::getAxisID() {
  return data_.axisId_;
}

void ecmcAxisBase::setReset(bool reset) {
  data_.control_.controlWord_.resetCmd = reset;
  // also reset auto enable/disable counters
  autoDisbleTimeCounter_ = 0;
  autoEnableTimeCounter_ = 0;

  if (reset) {
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

int ecmcAxisBase::setTrajDataSourceType(dataSource refSource) {
  return setTrajDataSourceTypeInternal(refSource,
                                       allowSourceChangeWhenEnabled_);
}

int ecmcAxisBase::setTrajDataSourceTypeInternal(dataSource refSource,
                                                int        force) {
  if (refSource == data_.control_.trajSource) return 0;

  if (!force) {
    if (getEnable() && (refSource != ECMC_DATA_SOURCE_INTERNAL)) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED);
    }
  }

  if ((refSource != ECMC_DATA_SOURCE_INTERNAL) &&
      getMon()->getSafetyInterlock()) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_TRAJ_SRC_CHANGE_NOT_ALLOWED_WHEN_SAFETY_IL);
  }

  if (refSource != ECMC_DATA_SOURCE_INTERNAL) {
    data_.interlocks_.noExecuteInterlock = false;
    data_.refreshInterlocks();
    data_.status_.statusWord_.busy         = true;
    data_.control_.controlWord_.executeCmd = true;
  } else {
    traj_->setStartPos(data_.status_.currentPositionActual);
    traj_->initStopRamp(data_.status_.currentPositionActual,
                        data_.status_.currentVelocityActual,
                        0);
    data_.status_.statusWord_.busy = traj_->getBusy();
    getSeq()->setTargetPos(data_.status_.currentPositionSetpoint);

    if (!getEnable()) {
      data_.status_.statusWord_.busy         = false;
      data_.control_.controlWord_.executeCmd = false;
    }
  }

  data_.control_.trajSource  = refSource;
  data_.control_.controlWord_.trajSourceCmd =  data_.control_.trajSource ==
                               ECMC_DATA_SOURCE_EXTERNAL;
  return 0;
}

int ecmcAxisBase::setEncDataSourceType(dataSource refSource) {
  if (refSource == data_.control_.encSource) return 0;

  if (!allowSourceChangeWhenEnabled_) {
    if (getEnable() && (refSource != ECMC_DATA_SOURCE_INTERNAL)) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED);
    }
  }

  // If realtime: Ensure that ethercat enty for actual position is linked
  if ((refSource == ECMC_DATA_SOURCE_INTERNAL) && data_.status_.statusWord_.inrealtime) {
    int errorCode = getEnc()->validateEntry(
      ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION);

    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  }

  data_.control_.encSource  = refSource;
  data_.control_.controlWord_.encSourceCmd =  data_.control_.encSource ==
                              ECMC_DATA_SOURCE_EXTERNAL;
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

int ecmcAxisBase::setEnableLocal(bool enable) {
  if (enable && !data_.control_.controlWord_.enableCmd) {
    errorReset();
    extEncVeloFilter_->initFilter(0);  // init to 0 vel
    extTrajVeloFilter_->initFilter(0);  // init to 0 vel    
    if(!data_.status_.statusWord_.attarget || !firstEnableDone_ || !mon_->getEnableAtTargetMon()) {      
      traj_->setStartPos(data_.status_.currentPositionActual);
      traj_->setCurrentPosSet(data_.status_.currentPositionActual);
      traj_->setTargetPos(data_.status_.currentPositionActual);
    
      data_.status_.currentTargetPosition =
        data_.status_.currentPositionActual;
      data_.status_.currentPositionSetpoint =
        data_.status_.currentPositionActual;
      data_.status_.currentPositionSetpointOld =
        data_.status_.currentPositionSetpoint;
      data_.status_.currentVelocitySetpoint = 0;
      firstEnableDone_                      = true;
    }
  }
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

  ecmcError::errorReset();
}

int ecmcAxisBase::validateBase() {
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
  if (data_.control_.encSource == ECMC_DATA_SOURCE_EXTERNAL) {
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
  return 0;
}

int ecmcAxisBase::setCmdData(int cmdData) {
  seq_.setCmdData(cmdData);
  return 0;
}

motionCommandTypes ecmcAxisBase::getCommand() {
  return seq_.getCommand();
}

int ecmcAxisBase::getCmdData() {
  return seq_.getCmdData();
}

int ecmcAxisBase::slowExecute() {
  oldPositionAct_         = data_.status_.currentPositionActual;
  oldPositionSet_         = data_.status_.currentPositionSetpoint;
  return 0;
}

void ecmcAxisBase::printAxisStatus() {
  if (memcmp(&data_.status_, &data_.statusOld_,
             sizeof(data_.statusOld_)) == 0) {
    return;  // Printout on change
  }

  // Only print header once per 25 status lines
  if (printHeaderCounter_ <= 0) {
    LOGINFO(
      "ecmc::  Ax     PosSet     PosAct     PosErr    PosTarg   DistLeft    CntrOut   VelFFSet     VelAct   VelFFRaw VelRaw  Error Co CD St IL LI TS ES En Ex Bu Ta Hd L- L+ Ho\n");
    printHeaderCounter_ = 25;
  }
  printHeaderCounter_--;
  
  LOGINFO(
    "ecmc:: %3d %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %6lf %6x %2d %2d %2d %2d %2d %2d %2d %1d%1d %2d %2d %2d %2d %2d %2d %2d\n",
    data_.axisId_,
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
    data_.status_.statusWord_.seqstate,
    (int)data_.interlocks_.interlockStatus,
    data_.status_.statusWord_.lastilock,
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
  // Auto enable if needed  (except for homing seq 15)
  if(!data_.status_.statusWord_.enabled and execute and autoEnableTimoutS_ > 0 and !((data_.control_.cmdData == ECMC_SEQ_HOME_SET_POS) &&
          (data_.control_.command == ECMC_CMD_HOMING))) {

    setGlobalBusy(true);
    autoEnableTimeCounter_ = 0;
    autoEnableRequest_ = true && enableAutoEnable_;
    // autoEnableSM will setExecute later when axis is enabled
    return 0;
  }

  if(!execute) {
    autoEnableRequest_ = false;
  }
  
  return setExecute(execute, false);
}

int ecmcAxisBase::setExecute(bool execute, bool ignoreBusy) {
  // Internal trajectory source
  if (data_.control_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
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
      if (execute && !data_.statusOld_.statusWord_.execute && data_.status_.statusWord_.busy) {
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_AXIS_BUSY);
      }
    }

    int error = seq_.setExecute(execute);

    if (error) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }
  }
  
  // Always reset 
  data_.control_.controlWord_.executeCmd = 0;

  return 0;
}

bool ecmcAxisBase::getExecute() {
  if (data_.control_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
    return seq_.getExecute();
  } else {
    return true;
  }
}

bool ecmcAxisBase::getBusy() {
  return data_.status_.statusWord_.busy || globalBusy_;
}

bool ecmcAxisBase::getTrajBusy() {
  return getTraj()->getBusy();
}

int ecmcAxisBase::getCycleCounter() {
  /// Use for watchdog purpose (will overflow)
  return cycleCounter_;
}

bool ecmcAxisBase::getEnable() {
  return data_.control_.controlWord_.enableCmd;
}

bool ecmcAxisBase::getEnabled() {
  return data_.status_.statusWord_.enabled && data_.control_.controlWord_.enableCmd;
}

bool ecmcAxisBase::getEnabledOnly() {
  return data_.status_.statusWord_.enabled;
}

int ecmcAxisBase::initAsyn() {
  if (asynPortDriver_ == NULL) {
    LOGERR("%s/%s:%d: ERROR (axis %d): AsynPortDriver object NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_.axisId_,
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
  axAsynParams_[ECMC_ASYN_AX_ACT_POS_ID] = paramTemp;

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
  axAsynParams_[ECMC_ASYN_AX_ACT_VEL_ID] = paramTemp;

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
  axAsynParams_[ECMC_ASYN_AX_ENC_ID_CMD_ID] = paramTemp;

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
  axAsynParams_[ECMC_ASYN_AX_SET_POS_ID] = paramTemp;

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
  axAsynParams_[ECMC_ASYN_AX_POS_ERR_ID] = paramTemp;

  // Diagnostic string (array)
  //errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_DIAG_NAME,
  //                            asynParamInt8Array,
  //                            ECMC_EC_S8,
  //                            (uint8_t *)diagBuffer_,
  //                            AX_MAX_DIAG_STRING_CHAR_LENGTH,
  //                            &paramTemp);
//
  //if (errorCode) {
  //  return errorCode;
  //}
//
  //paramTemp->setAllowWriteToEcmc(false);
  //paramTemp->refreshParam(1);
  //axAsynParams_[ECMC_ASYN_AX_DIAG_ID] = paramTemp;

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
  axAsynParams_[ECMC_ASYN_AX_STATUS_ID] = paramTemp;

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
  axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID] = paramTemp;

  // Target Velo
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_TARG_VELO_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(velocityTarget_),
                              sizeof(velocityTarget_),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteTargetVelo, this); // Access to this axis
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_TARG_VELO_ID] = paramTemp;

  // Target Pos
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_TARG_POS_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(positionTarget_),
                              sizeof(positionTarget_),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteTargetPos, this); // Access to this axis
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_TARG_POS_ID] = paramTemp;

  // Acceleration
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_ACC_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(acceleration_),
                              sizeof(acceleration_),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteAcc, this); // Access to this axis
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_ACC_ID] = paramTemp;

  // Deceleration
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_DEC_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(deceleration_),
                              sizeof(deceleration_),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteDec, this); // Access to this axis
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_DEC_ID] = paramTemp;

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
  axAsynParams_[ECMC_ASYN_AX_SET_ENC_POS_ID] = paramTemp;

  // Command
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_COMMAND_NAME,
                              asynParamInt32,
                              ECMC_EC_S32,
                              (uint8_t *)&(command_),
                              sizeof(command_),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteCommand, this); // Access to this axis
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_COMMAND_ID] = paramTemp;

  // CmdData
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_CMDDATA_NAME,
                              asynParamInt32,
                              ECMC_EC_S32,
                              (uint8_t *)&(cmdData_),
                              sizeof(cmdData_),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteCmdData, this); // Access to this axis
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_CMDDATA_ID] = paramTemp;

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
  axAsynParams_[ECMC_ASYN_AX_ERROR_ID] = paramTemp;

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
  axAsynParams_[ECMC_ASYN_AX_WARNING_ID] = paramTemp;

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
  axAsynParams_[ECMC_ASYN_AX_MR_CMD_ID] = paramTemp;

  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST,
                                      ECMC_ASYN_DEFAULT_ADDR);
  return 0;
}

//int ecmcAxisBase::getAxisDebugInfoData(char *buffer,
//                                       int   bufferByteSize,
//                                       int  *bytesUsed) {
//
//  // (Ax,PosSet,PosAct,PosErr,PosTarg,DistLeft,CntrOut,VelFFSet,VelAct,VelFFRaw,VelRaw,CycleCounter,Error,Co,CD,St,IL,TS,ES,En,Ena,Ex,Bu,Ta,L-,L+,Ho");
//  int ret = snprintf(buffer,
//                     bufferByteSize,
//                     "%d,%lf,%lf,%lf,%lf,%lf,%" PRId64 ",%lf,%lf,%lf,%lf,%d,%d,%x,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
//                     data_.axisId_,
//                     data_.status_.currentPositionSetpoint,
//                     data_.status_.currentPositionActual,
//                     data_.status_.cntrlError,
//                     data_.status_.positionTarget,
//                     data_.status_.currentTargetPosition - data_.status_.currentPositionActual,
//                     data_.status_.positionRaw,
//                     data_.status_.cntrlOutput,
//                     data_.status_.currentVelocitySetpoint,
//                     data_.status_.currentVelocityActual,
//                     data_.status_.currentvelocityFFRaw,
//                     data_.status_.currentVelocitySetpointRaw,
//                     cycleCounter_,
//                     data_.status_.errorCode,
//                     data_.status_.command,
//                     data_.status_.cmdData,
//                     data_.status_.statusWord_.seqstate,
//                     data_.status_.trajInterlock,
//                     data_.status_.statusWord_.lastilock,
//                     data_.status_.statusWord_.trajsource,
//                     data_.status_.statusWord_.encsource,
//                     data_.status_.statusWord_.enable,
//                     data_.status_.statusWord_.enabled,
//                     data_.status_.statusWord_.execute,
//                     data_.status_.statusWord_.busy,
//                     data_.status_.statusWord_.attarget,
//                     data_.status_.statusWord_.homed,
//                     data_.status_.statusWord_.limitbwd,
//                     data_.status_.statusWord_.limitfwd,
//                     data_.status_.statusWord_.homeswitch);
//
//  if ((ret >= bufferByteSize) || (ret <= 0)) {
//    *bytesUsed = 0;
//    return ERROR_AXIS_PRINT_TO_BUFFER_FAIL;
//  }
//  *bytesUsed = ret;
//  return 0;
//}

int ecmcAxisBase::setEcStatusOutputEntry(ecmcEcEntry *entry) {
  statusOutputEntry_ = entry;
  return 0;
}

motionDirection ecmcAxisBase::getAxisSetDirection() {
  if (!data_.status_.statusWord_.enabled) {
    return ECMC_DIR_STANDSTILL;
  }

  // Transform or internal trajectory
  if (data_.control_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
    return getTraj()->getCurrSetDir();
  } else {
    if (data_.status_.currentPositionSetpoint <
        data_.status_.currentPositionSetpointOld) {
      return ECMC_DIR_BACKWARD;
    } else if (data_.status_.currentPositionSetpoint >
               data_.status_.currentPositionSetpointOld) {
      return ECMC_DIR_FORWARD;
    } else {
      return ECMC_DIR_STANDSTILL;
    }
  }
  return ECMC_DIR_STANDSTILL;
}

int ecmcAxisBase::getBlockExtCom() {
  return blockExtCom_;
}

int ecmcAxisBase::setBlockExtCom(int block) {
  blockExtCom_ = block;
  return 0;
}

int ecmcAxisBase::setModRange(double mod) {
  // Must be same mod factor in traj and enc
  if (mod < 0) {
    LOGERR(
      "ERROR (axis %d): Modulo factor out of range. Must be a positive value (0x%x).\n",
      data_.axisId_,
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
    LOGERR(
      "ERROR (axis %d): Modulo type out of range (0x%x).\n",
      data_.axisId_, ERROR_AXIS_MODULO_TYPE_OUT_OF_RANGE);

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

int ecmcAxisBase::getCntrlError(double *error) {
  *error = data_.status_.cntrlError;
  return 0;
}

void ecmcAxisBase::refreshStatusWd() {
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
    data_.control_.controlWord_.enableSoftLimitFwd> 0;

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

  // bit 21 axis type
  data_.status_.statusWord_.axisType = data_.axisType_ ==
                                               ECMC_AXIS_TYPE_VIRTUAL;
  // bit 26..31 lastActiveInterlock type
  data_.status_.statusWord_.lastilock =
    (unsigned char)data_.interlocks_.lastActiveInterlock;
}
int ecmcAxisBase::setEnable(bool enable) {

  if (data_.status_.statusWord_.enable == enable) return 0;

  if (!enable) {  // Remove execute if enable is going down
    setExecute(false);
  }

  if (enable && validate()) {
    setExecute(false);
    return getErrorID();
  }

  if (!getEnc()->hwReady()) {
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
  enableCmd_ = enable;
  axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]->refreshParamRT(1);

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
  return data_.control_.trajSource;
}

dataSource ecmcAxisBase::getEncDataSourceType() {
  return data_.control_.encSource;
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

int ecmcAxisBase::setEncVeloFiltSize(size_t size) {
  return encArray_[data_.control_.primaryEncIndex]->setVeloFilterSize(size);
}

int ecmcAxisBase::setEncPosFiltSize(size_t size) {
  return encArray_[data_.control_.primaryEncIndex]->setPosFilterSize(size);
}

int ecmcAxisBase::setEncPosFiltEnable(bool enable) {
  return encArray_[data_.control_.primaryEncIndex]->setPosFilterEnable(enable);
}

int ecmcAxisBase::setEncVeloFiltEnable(bool enable) {
  return encArray_[data_.control_.primaryEncIndex]->setVelFilterEnable(enable);
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
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): AsynPortDriver object NULL (%s) (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
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
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate (%s). Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
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
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  *asynParamOut = paramTemp;
  return 0;
}

int ecmcAxisBase::movePVTAbs() { 
  return movePVTAbs(false);
}

int ecmcAxisBase::movePVTAbs(bool ignoreBusy) {
  if (getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Move PVT failed since traj source is set to PLC (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL);

    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  // Allow pvt to be started even if globalBusy is set
  if(!ignoreBusy) {
    if(getBusy()) {
      return ERROR_AXIS_BUSY;
    }
  }

  int errorCode = getErrorID();

  if (errorCode) {
    return errorCode;
  }

  errorCode = setExecute(0);

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
  return  moveAbsolutePosition(positionSet,data_.control_.velocityTarget,acceleration_,deceleration_);
}

int ecmcAxisBase::moveAbsolutePosition(
  double positionSet,
  double velocitySet,
  double accelerationSet,
  double decelerationSet) {
  if (getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Move to abs position failed since traj source is set to PLC (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL);

    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  if (getExecute() && (getCommand() == ECMC_CMD_MOVEABS) && getBusy()) {
    getSeq()->setTargetVel(velocitySet);
    setAcc(accelerationSet);
    setDec(decelerationSet);
    getSeq()->setTargetPos(positionSet);
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
  getSeq()->setTargetVel(velocitySet);
  setAcc(accelerationSet);
  setDec(decelerationSet);
  getSeq()->setTargetPos(positionSet);

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
  if (getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Move to abs position failed since traj source is set to PLC (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL);

    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  if (getExecute() && (getCommand() == ECMC_CMD_MOVEREL) && getBusy()) {
    getSeq()->setTargetVel(velocitySet);
    setAcc(accelerationSet);
    setDec(decelerationSet);
    getSeq()->setTargetPos(positionSet);
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

  getSeq()->setTargetVel(velocitySet);
  setAcc(accelerationSet);
  setDec(decelerationSet);
  getSeq()->setTargetPos(positionSet);

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
  if (getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Move to abs position failed since traj source is set to PLC (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL);

    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  int errorCode = getErrorID();

  if (errorCode) {
    return errorCode;
  }

  // check if already moveVelo then just update vel and acc
  if (getExecute() && (getCommand() == ECMC_CMD_MOVEVEL) && getBusy()) {
    getSeq()->setTargetVel(velocitySet);
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

  getSeq()->setTargetVel(velocitySet);
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
  if (getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Homing failed since traj source is set to PLC (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL);

    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  if (getBusy()) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Axis Busy, homing not possible (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_, ERROR_AXIS_BUSY);

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

  // if not valid the fallback on whats defined in encoder
  if (nCmdData <= 0) {
    nCmdData = getPrimEnc()->getHomeSeqId();
  }
  errorCode = setCmdData(nCmdData);

  if (errorCode) {
    return errorCode;
  }

  // if not valid the fallback on whats defined in encoder
  if (velocityOffCamSet < 0) {
    velocityOffCamSet = getPrimEnc()->getHomeVelOffCam();
  }
  getSeq()->setHomeVelOffCam(velocityOffCamSet);

  // if not valid the fallback on whats defined in encoder
  if (velocityTowardsCamSet < 0) {
    velocityTowardsCamSet = getPrimEnc()->getHomeVelTowardsCam();
  }
  getSeq()->setHomeVelTowardsCam(velocityTowardsCamSet);

  // if not valid the fallback on whats defined in encoder
  if (accelerationSet < 0) {
    accelerationSet = getPrimEnc()->getHomeAcc();
  }
  setAcc(accelerationSet);

  // if not valid the fallback on whats defined in encoder
  if (decelerationSet < 0) {
    decelerationSet = getPrimEnc()->getHomeDec();
  }

  setDec(decelerationSet);

  getSeq()->setHomePosition(homePositionSet);

  errorCode = setExecute(1);

  if (errorCode) {
    return errorCode;
  }
  return 0;
}

// Homing with configs from encoder object
int ecmcAxisBase::moveHome() {
  if (getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Homing failed since traj source is set to PLC (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL);

    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  if (getBusy()) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Axis Busy, homing not possible (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_, ERROR_AXIS_BUSY);

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
  getPrimEnc()->setHomed(1);
  return 0;
}

int ecmcAxisBase::stopMotion(int killAmplifier) {
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
 * Function to link to ecmcDataItem. Execute function instead of copoy data.
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
 * 
*/
asynStatus ecmcAxisBase::axisAsynWriteCmd(void         *data,
                                          size_t        bytes,
                                          asynParamType asynParType) {
  asynStatus returnVal = asynSuccess;

  if (sizeof(data_.control_.controlWord_) != bytes) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Control word size missmatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }

  memcpy(&data_.control_.controlWord_, data, sizeof(data_.control_.controlWord_));
  uint32_t *tmpcontrolWordPtr = (uint32_t *)&data_.control_.controlWord_;
  LOGERR(
    "%s/%s:%d: INFO (axis %d): Write : Control Word = 0x%x.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.axisId_, *tmpcontrolWordPtr);


  // Check if com is blocked but allow stop cmd
  if (blockExtCom_) {
    bool refreshNeeded = false;
    
    // allow to stop and disable but still go to error state
    if (!data_.control_.controlWord_.enableCmd) {
      setEnable(data_.control_.controlWord_.enableCmd);
      refreshNeeded = true;
    }

    if (data_.control_.controlWord_.stopCmd) {
      stopMotion(enableCmd_ == 0);    
      //errorCode            = setExecute(0);
      data_.control_.controlWord_.stopCmd = 0;  // auto reset
      data_.control_.controlWord_.executeCmd = 0;
      refreshNeeded        = true;
    }

    if (refreshNeeded) {
      refreshStatusWd();
    }

    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Communication blocked (stopCmd==1 and enableCmd==0 still can be used) (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      ERROR_MAIN_AXIS_EXTERNAL_COM_DISABLED);
    return asynError;
  }

  int errorCode = 0;

  errorCode = setEnable(data_.control_.controlWord_.enableCmd);
  
  if (errorCode) {
    returnVal = asynError;
  }

  if (data_.control_.controlWord_.stopCmd) {
    stopMotion(enableCmd_ == 0);    
    //errorCode            = setExecute(0);
    data_.control_.controlWord_.stopCmd = 0;  // auto reset
    data_.control_.controlWord_.executeCmd = 0;
    if (errorCode) {
      returnVal = asynError;
    }
  }

  // Only trig new commands if source is internal
  if(data_.control_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
    // trigg new motion.
    if (!data_.control_.controlWord_.stopCmd && data_.control_.controlWord_.executeCmd) {
      if (getSeq() == NULL) {
        data_.control_.controlWord_.executeCmd = 0; // auto reset
        return asynError;
      }

      // Only allow cmd change if not busy
      if (!getBusy()) {
        setCommand(command_);

        if (command_ == ECMC_CMD_HOMING) {
          // fallback on config encoder
          // if(cmdData_ <= 0 || cmdData_ == ECMC_SEQ_HOME_USE_ENC_CFGS ) {
          //  setCmdData(getPrimEnc()->getHomeSeqId());
          // } else {
          setCmdData(cmdData_);

          // }

          // For homing velos, check if special homing velos,
          // otherwise fallback on velocityTarget_
          double temp = getPrimEnc()->getHomeVelOffCam();

          if (temp <= 0) {
            temp = velocityTarget_;
          }
          getSeq()->setHomeVelOffCam(temp);
          temp = getPrimEnc()->getHomeVelTowardsCam();

          if (temp <= 0) {
            temp = velocityTarget_;
          }
          getSeq()->setHomeVelTowardsCam(temp);

          // Homing pos from encoder
          getSeq()->setHomePosition(getPrimEnc()->getHomePosition());
        } else {  // Not homing
          // cmddata for all other states
          setCmdData(cmdData_);
        }
      }

      if (command_ != ECMC_CMD_HOMING) {   // Not homing!
        // allow on the fly updates of target velo and target pos
        getSeq()->setTargetVel(velocityTarget_);
        getSeq()->setTargetPos(positionTarget_);
        getSeq()->setAcc(acceleration_);
        getSeq()->setDec(deceleration_);
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
      }
    }
  }

  data_.control_.controlWord_.executeCmd = 0; // auto reset

  if(data_.control_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
    // Tweak BWD
    if (!data_.control_.controlWord_.stopCmd && data_.control_.controlWord_.tweakBwdCmd && !getBusy()) {
      setCommand(ECMC_CMD_MOVEREL);
      getSeq()->setTargetVel(velocityTarget_);
      getSeq()->setTargetPos(-std::abs(positionTarget_));  // Backward
      getSeq()->setAcc(acceleration_);
      getSeq()->setDec(deceleration_);
      errorCode = setExecute(0);
      if (errorCode) {
        returnVal = asynError;
      }
      errorCode = setExecute(1);
  
      if (errorCode) {
        returnVal = asynError;
      }   
    }
    
    // Tweak FWD
    if (!data_.control_.controlWord_.stopCmd && data_.control_.controlWord_.tweakFwdCmd && !getBusy()) {
      setCommand(ECMC_CMD_MOVEREL);
      getSeq()->setTargetVel(velocityTarget_);
      getSeq()->setTargetPos(std::abs(positionTarget_)); // Forward
      getSeq()->setAcc(acceleration_);
      getSeq()->setDec(deceleration_);
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

  // reset the commands
  data_.control_.controlWord_.tweakFwdCmd = 0;
  data_.control_.controlWord_.tweakBwdCmd = 0;

  setReset(data_.control_.controlWord_.resetCmd);  // resetCmd is reset in setReset()


  errorCode =
    setEncDataSourceType((data_.control_.controlWord_.encSourceCmd ? ECMC_DATA_SOURCE_EXTERNAL
  :
                          ECMC_DATA_SOURCE_INTERNAL));

  if (errorCode) {
    returnVal = asynError;
  }

  errorCode =
    setTrajDataSourceType((data_.control_.controlWord_.trajSourceCmd ?
                           ECMC_DATA_SOURCE_EXTERNAL
  : ECMC_DATA_SOURCE_INTERNAL));

  if (errorCode) {
    returnVal = asynError;
  }

  errorCode = setAllowCmdFromPLC(data_.control_.controlWord_.plcCmdsAllowCmd);

  if (errorCode) {
    returnVal = asynError;
  }

  errorCode = setAxisPLCEnable(data_.axisId_, data_.control_.controlWord_.plcEnableCmd);

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
  axAsynParams_[ECMC_ASYN_AX_STATUS_ID]->refreshParamRT(1);
  axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]->refreshParamRT(1);
  return returnVal;
}

void ecmcAxisBase::initControlWord() {
  // Fill all controlWord with actual data before rt
  int plcEnable = 0;

  getAxisPLCEnable(data_.axisId_, &plcEnable);
  data_.control_.controlWord_.plcEnableCmd = plcEnable;
  data_.control_.controlWord_.enableCmd = getEnable();  
  data_.control_.controlWord_.executeCmd    = getExecute();
  data_.control_.controlWord_.resetCmd      = getReset();
  data_.control_.controlWord_.encSourceCmd  = getEncDataSourceType() ==
                              ECMC_DATA_SOURCE_EXTERNAL;
  data_.control_.controlWord_.trajSourceCmd = getTrajDataSourceType() ==
                               ECMC_DATA_SOURCE_EXTERNAL;
  data_.control_.controlWord_.plcCmdsAllowCmd    = getAllowCmdFromPLC();
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
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Primary encoder index datatype missmatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  int index = 0;
  memcpy(&index, data, sizeof(index));

  int errorCode = selectPrimaryEncoder(index, 1);

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Set Primary encoder index failed (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      errorCode);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }

  LOGERR(
    "%s/%s:%d: INFO (axis %d): Write: Prim encoder = %d.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.axisId_, index);

  return asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteTargetVelo(void         *data,
                                                 size_t        bytes,
                                                 asynParamType asynParType) {
  if ((bytes != 8) || (asynParType != asynParamFloat64)) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Target Velo size or datatype missmatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  double velo = 0;
  memcpy(&velo, data, bytes);
  velocityTarget_ = velo;

  LOGERR(
    "%s/%s:%d: INFO (axis %d): Write: Target Velo = %lf.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.axisId_, velocityTarget_);

  // Write at next execute command

  return asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteAcc(void         *data,
                                          size_t        bytes,
                                          asynParamType asynParType) {
  if ((bytes != 8) || (asynParType != asynParamFloat64)) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Target Velo size or datatype missmatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  double acc = 0;
  memcpy(&acc, data, bytes);

  LOGERR(
    "%s/%s:%d: INFO (axis %d): Write: Acceleration = %lf.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.axisId_, acceleration_);

  acceleration_ = acc;

  // Write at next execute command

  return asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteDec(void         *data,
                                          size_t        bytes,
                                          asynParamType asynParType) {
  if ((bytes != 8) || (asynParType != asynParamFloat64)) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Write deceleration size or datatype missmatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  double dec = 0;
  memcpy(&dec, data, bytes);
  deceleration_ = dec;

  LOGERR(
    "%s/%s:%d: INFO (axis %d): Write: Deceleration = %lf.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.axisId_, deceleration_);

  // Write at next execute command

  return asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteTargetPos(void         *data,
                                                size_t        bytes,
                                                asynParamType asynParType) {
  if ((bytes != 8) || (asynParType != asynParamFloat64)) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Target Pos size or datatype missmatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  double pos = 0;
  memcpy(&pos, data, bytes);
  positionTarget_ = pos;

  LOGERR(
    "%s/%s:%d: INFO (axis %d): Write: Target Pos = %lf.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.axisId_, positionTarget_);

  return asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteSetEncPos(void         *data,
                                                size_t        bytes,
                                                asynParamType asynParType) {
  if ((sizeof(double) != bytes) || (asynParType != asynParamFloat64)) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Encoder Pos size or datatype missmatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }

  double pos = 0;
  memcpy(&pos, data, bytes);

  if (getBusy()) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Axis Busy, homing not possible.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_);

    setWarningID(WARNING_AXIS_ASYN_CMD_WHILE_BUSY);
    return asynError;
  }

  LOGERR(
    "%s/%s:%d: INFO (axis %d): Write: Encoder position = %lf.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.axisId_, pos);

  // Set position
  return setPosition(pos) ? asynError : asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteCommand(void         *data,
                                              size_t        bytes,
                                              asynParamType asynParType) {
  if ((bytes != 4) || (asynParType != asynParamInt32)) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Command size or datatype missmatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  int command = 0;
  memcpy(&command, data, bytes);
  command_ = (motionCommandTypes)command;

  LOGERR(
    "%s/%s:%d: INFO (axis %d): Write: Command = 0x%x.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.axisId_, command_);

  return asynSuccess;
}

asynStatus ecmcAxisBase::axisAsynWriteCmdData(void         *data,
                                              size_t        bytes,
                                              asynParamType asynParType) {
  if ((bytes != 4) || (asynParType != asynParamInt32)) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): CmdData size or datatype missmatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }
  int cmddata = 0;
  memcpy(&cmddata, data, bytes);
  cmdData_ = cmddata;

  LOGERR(
    "%s/%s:%d: INFO (axis %d): Write: Command Data = 0x%x.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    data_.axisId_, cmdData_);

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
                                                          &data_,
                                                          data_.sampleTime_,
                                                          data_.status_.encoderCount);
  data_.control_.cfgEncIndex = data_.status_.encoderCount; // Use current encoder index for cfg
  data_.status_.encoderCount++;

  return 0;
}

// Only for Configuration purpose
int ecmcAxisBase::selectCSPDriveEncoder(int index) {
  
  // Comamnd only makes sense if REAL axis
  if(data_.axisType_ != ECMC_AXIS_TYPE_REAL) {
    LOGERR("%s/%s:%d: ERROR (axis %d): Command only valid for axes of type REAL.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_.axisId_);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
  }

  if(data_.status_.statusWord_.inrealtime ) {
    LOGERR("%s/%s:%d: ERROR (axis %d): Command not allowed in realtime.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_.axisId_);
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
    LOGERR("%s/%s:%d: ERROR (axis %d): Encoder index out of range.\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_);
    
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_PRIMARY_ENC_ID_OUT_OF_RANGE);
  }
  printf("CSP with control enabled\n");
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
    if (getBusy() && (data_.control_.encSource ==
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
  }

  // This index is starting from 0
  data_.control_.primaryEncIndex = localIndex;

  // This index is starting from 1 (for asyn and external interface)
  encPrimIndexAsyn_ = index;
  axAsynParams_[ECMC_ASYN_AX_ENC_ID_CMD_ID]->refreshParamRT(1);

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

// unfortenatelly starts from 1... why did I do like this..
int ecmcAxisBase::getConfigEncoderIndex() {
  return data_.control_.cfgEncIndex + 1;
}

int ecmcAxisBase::getPrimaryEncoderIndex() {
  return data_.control_.primaryEncIndex + 1;
}

bool ecmcAxisBase::getHwReady() {

  /* Only check prim encoder (allow encoders to be in error state 
  if they are not used) */
  bool ready = getPrimEnc()->hwReady();

  // Check drive if REAL axis
  if(getDrv()) {
    ready = ready && getDrv()->hwReady();
  }
  return ready;
}

void ecmcAxisBase::initEncoders() {
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
    encArray_[i]->setActPos(encArray_[encRefIndex]->getActPos());
    encArray_[i]->setHomed(1);
  }
}

int ecmcAxisBase::setAllowSourceChangeWhenEnabled(bool allow) {
  allowSourceChangeWhenEnabled_ = allow;
  return 0;
}

void ecmcAxisBase::setTargetVel(double velTarget) {
  getSeq()->setTargetVel(velTarget);

  // also set for ecmc interface
  velocityTarget_ = velTarget;
  axAsynParams_[ECMC_ASYN_AX_TARG_VELO_ID]->refreshParamRT(1);
}

void ecmcAxisBase::setAcc(double acc) {
  getSeq()->setAcc(acc);

  // also set for ecmc interface
  acceleration_ = acc;
  axAsynParams_[ECMC_ASYN_AX_ACC_ID]->refreshParamRT(1);
}

void ecmcAxisBase::setDec(double dec) {
  getSeq()->setDec(dec);

  // also set for ecmc interface
  deceleration_ = dec;
  axAsynParams_[ECMC_ASYN_AX_DEC_ID]->refreshParamRT(1);
}

void ecmcAxisBase::setEmergencyStopInterlock(int stop) {
  getMon()->setSafetyInterlock(stop);

  // Switch to internal source
  if ((data_.control_.trajSource != ECMC_DATA_SOURCE_INTERNAL) && stop) {
    setTrajDataSourceTypeInternal(ECMC_DATA_SOURCE_INTERNAL, 1);
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

int ecmcAxisBase::getPrintDbg() {
  return data_.control_.controlWord_.enableDbgPrintout;
}

ecmcAxisPVTSequence* ecmcAxisBase::getPVTObject() {
  return seq_.getPVTObject();
}

double ecmcAxisBase::getCurrentPositionSetpoint() {
  return data_.status_.currentPositionSetpoint;
}

int ecmcAxisBase::setGlobalBusy(bool busy) {  // Allow for sequences
  globalBusy_ = busy;
  return 0;
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

void ecmcAxisBase::autoEnableSM() {
  if( !enableAutoEnable_) {
    return;
  }

  if(!autoEnableRequest_) {
    return;
  }
  
  if(!data_.control_.controlWord_.enableCmd) {
    printf("HEPP\n");
    setEnable(true);
  }

  if(autoEnableTimeCounter_ > autoEnableTimoutS_) {
    setEnable(false);
    autoEnableRequest_= false;
    autoEnableTimeCounter_ = 0;    
    setGlobalBusy(false);
    setErrorID(ERROR_AUTO_ENABLE_TIMEOUT);
    return;
  } else {
    autoEnableTimeCounter_ += data_.sampleTime_;
  }

  if(data_.status_.statusWord_.enabled) {
    autoEnableTimeCounter_ = 0;
    autoEnableRequest_= false;
    printf("Axis[%d]: Auto enable axis and triggering motion\n", data_.axisId_);
    setExecute(1,1); // need ignore busy 
    setGlobalBusy(false); // Need to clear the global busy bit. Now seq->busy is used
    return;
  }
}

void ecmcAxisBase::autoDisableSM() {
  if(autoDisableAfterS_ < 0 || !enableAutoDisable_ ) {
    return;
  }

  if(data_.status_.statusWord_.busy) {
    autoDisbleTimeCounter_ = 0;
    return;
  }

  if(!data_.status_.statusWord_.enabled) {
    autoDisbleTimeCounter_ = 0;
    return;
  }
  
  if(autoDisbleTimeCounter_ > autoDisableAfterS_ && data_.control_.controlWord_.enableCmd) {
    printf("Axis[%d]: Auto disable axis (axis idle for %lfs)\n", data_.axisId_,autoDisableAfterS_);
    setEnable(0);
  } else {
    autoDisbleTimeCounter_+= data_.sampleTime_;
  }
}

int ecmcAxisBase::setAutoEnableTimeout(double timeS) {
  autoEnableTimoutS_ = timeS;
  enableAutoEnable_ = timeS > 0;
  return 0;
}

int ecmcAxisBase::setEnableAutoEnable(bool enable) {
  if(enableAutoEnable_ != enable) {
    autoDisbleTimeCounter_ = 0;
  }
  enableAutoEnable_ = enable;
  return 0;
}

int ecmcAxisBase::setAutoDisableAfterTime(double timeS) {
  autoDisableAfterS_ = timeS;
  enableAutoDisable_ = timeS > 0;
  return 0;
}

int ecmcAxisBase::setEnableAutoDisable(bool enable) {
  if(enableAutoDisable_ != enable) {
    autoDisbleTimeCounter_ = 0;
  }
  
  enableAutoDisable_ = enable;
  return 0;
}
