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
  setExternalPtrs(&(data_.status_.errorCode), &(data_.status_.warningCode));
  try {
    data_.command_.primaryEncIndex = 0;
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

  //  free(plcExpr_);
}

void ecmcAxisBase::initVars() {
  // errorReset();  //THIS IS NONO..
  firstEnableDone_                       = 0;
  data_.axisType_                          = ECMC_AXIS_TYPE_BASE;
  data_.command_.reset                     = false;
  allowCmdFromOtherPLC_                    = true;
  data_.status_.inStartupPhase             = false;
  data_.status_.inRealtime                 = false;
  data_.status_.externalTrajectoryPosition = 0;
  data_.status_.externalTrajectoryVelocity = 0;
  data_.status_.externalEncoderPosition    = 0;
  data_.status_.externalEncoderVelocity    = 0;
  data_.status_.currentPositionActual      = 0;
  data_.status_.currentPositionSetpoint    = 0;
  data_.status_.currentVelocityActual      = 0;
  data_.status_.currentVelocitySetpoint    = 0;
  data_.sampleTime_                        = 1 / 1000;
  memset(&statusData_,    0, sizeof(statusData_));
  memset(&statusDataOld_, 0, sizeof(statusDataOld_));
  memset(&controlWord_,   0, sizeof(controlWord_));
  printHeaderCounter_      = 0;
  data_.status_.enabledOld = false;
  data_.status_.enableOld  = false;
  data_.status_.executeOld = false;
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
  data_.command_.cfgEncIndex    = 0;
  allowSourceChangeWhenEnabled_ = false;
  setEncoderPos_                = 0;
  encPrimIndexAsyn_             = 1;
  acceleration_                 = 0;
  deceleration_                 = 0;
  hwReady_                      = 0;
  hwReadyOld_                   = 0;
}

void ecmcAxisBase::preExecute(bool masterOK) {
  hwReadyOld_ = hwReady_;

  data_.interlocks_.etherCatMasterInterlock = !masterOK;

  if (!masterOK) {
    setEnable(false);
  }
  data_.refreshInterlocks();

  data_.status_.moving = std::abs(
    data_.status_.currentVelocityActual) > 0 || getTraj()->getBusy();

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
    data_.status_.busy       = true;
    data_.status_.distToStop = 0;

    if (masterOK) {
      
      if (!hwReady_) {
        setErrorID(ERROR_ENC_NOT_READY);
      } else {  // auto reset error if ok
        if (getErrorID() == ERROR_ENC_NOT_READY && hwReadyOld_) {
          errorReset();
        }
      }
    }

    if (masterOK && hwReady_ && hwReadyOld_) {
      // Auto reset hardware error if starting up
      if ((getErrorID() == ERROR_AXIS_HARDWARE_STATUS_NOT_OK) &&
          data_.status_.inStartupPhase) {
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
    data_.status_.busy       = false;
    data_.status_.distToStop = 0;

    if (data_.status_.enabled) {
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

    if (!data_.status_.enabled) {
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
  data_.status_.externalTrajectoryPositionOld =
    data_.status_.externalTrajectoryPosition;
  data_.status_.externalEncoderPositionOld =
    data_.status_.externalEncoderPosition;

  // Write encoder entries
  for (int i = 0; i < data_.status_.encoderCount; i++) {
    encArray_[i]->writeEntries();
  }
  data_.status_.busyOld                    = data_.status_.busy;
  data_.status_.enabledOld                 = data_.status_.enabled;
  data_.status_.executeOld                 = getExecute();
  data_.status_.atTargetOld                = data_.status_.atTarget;
  data_.status_.currentPositionSetpointOld =
    data_.status_.currentPositionSetpoint;
  data_.status_.cntrlOutputOld = data_.status_.cntrlOutput;

  cycleCounter_++;
  refreshDebugInfoStruct();

  // Update asyn parameters
  axAsynParams_[ECMC_ASYN_AX_SET_POS_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_ACT_POS_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_ACT_VEL_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_POS_ERR_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_STATUS_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID]->refreshParamRT(0);

  // axAsynParams_[ECMC_ASYN_AX_TARG_VELO_ID]->refreshParamRT(0);
  // axAsynParams_[ECMC_ASYN_AX_TARG_POS_ID]->refreshParamRT(0);
  // axAsynParams_[ECMC_ASYN_AX_COMMAND_ID]->refreshParamRT(0);
  // axAsynParams_[ECMC_ASYN_AX_CMDDATA_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_ERROR_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_WARNING_ID]->refreshParamRT(0);

  if(memcmp(&mrCmdsOld_,&mrCmds_, sizeof(mrCmdsOld_)) !=0) {
    axAsynParams_[ECMC_ASYN_AX_MR_CMD_ID]->refreshParamRT(1);
  }
  memcpy(&mrCmdsOld_,&mrCmds_, sizeof(mrCmdsOld_));
  
  if (axAsynParams_[ECMC_ASYN_AX_DIAG_ID]->willRefreshNext() &&
      axAsynParams_[ECMC_ASYN_AX_DIAG_ID]->linkedToAsynClient()) {
    int bytesUsed = 0;
    int error     = getAxisDebugInfoData(&diagBuffer_[0],
                                         AX_MAX_DIAG_STRING_CHAR_LENGTH,
                                         &bytesUsed);

    if (error) {
      LOGERR(
        "%s/%s:%d: ERROR (axis %d): Fail to update asyn par axis<id>.diag. Buffer to small.\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        data_.axisId_);
    } else {
      axAsynParams_[ECMC_ASYN_AX_DIAG_ID]->refreshParamRT(1,
                                                          (uint8_t *)diagBuffer_,
                                                          bytesUsed);
    }
  } else {
    // Just to count up for correct freq
    axAsynParams_[ECMC_ASYN_AX_DIAG_ID]->refreshParamRT(0);
  }

  // Update status entry if linked
  if (statusOutputEntry_) {
    statusOutputEntry_->writeValue(getErrorID() == 0);
  }
}

axisType ecmcAxisBase::getAxisType() {
  return data_.axisType_;
}

int ecmcAxisBase::getAxisID() {
  return data_.axisId_;
}

void ecmcAxisBase::setReset(bool reset) {
  data_.command_.reset = reset;

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
  controlWord_.resetCmd = 0;
}

bool ecmcAxisBase::getReset() {
  return data_.command_.reset;
}

int ecmcAxisBase::setAllowCmdFromPLC(bool enable) {
  allowCmdFromOtherPLC_        = enable;
  controlWord_.plcCmdsAllowCmd = enable;
  return 0;
}

bool ecmcAxisBase::getAllowCmdFromPLC() {
  return allowCmdFromOtherPLC_;
}

void ecmcAxisBase::setInStartupPhase(bool startup) {
  // When configuration is ready then defult to primary encoder for all encoder related calls
  if (!data_.status_.inStartupPhase && startup) {
    data_.command_.cfgEncIndex = data_.command_.primaryEncIndex;
  }

  data_.status_.inStartupPhase = startup;
}

int ecmcAxisBase::setTrajDataSourceType(dataSource refSource) {
  return setTrajDataSourceTypeInternal(refSource,
                                       allowSourceChangeWhenEnabled_);
}

int ecmcAxisBase::setTrajDataSourceTypeInternal(dataSource refSource,
                                                int        force) {
  if (refSource == data_.command_.trajSource) return 0;

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
    data_.status_.busy     = true;
    data_.command_.execute = true;
  } else {
    traj_->setStartPos(data_.status_.currentPositionActual);
    traj_->initStopRamp(data_.status_.currentPositionActual,
                        data_.status_.currentVelocityActual,
                        0);
    getSeq()->setTargetPos(data_.status_.currentPositionSetpoint);

    if (!getEnable()) {
      data_.status_.busy     = false;
      data_.command_.execute = false;
    }
  }

  data_.command_.trajSource  = refSource;
  controlWord_.trajSourceCmd =  data_.command_.trajSource ==
                               ECMC_DATA_SOURCE_EXTERNAL;
  return 0;
}

int ecmcAxisBase::setEncDataSourceType(dataSource refSource) {
  if (refSource == data_.command_.encSource) return 0;

  if (!allowSourceChangeWhenEnabled_) {
    if (getEnable() && (refSource != ECMC_DATA_SOURCE_INTERNAL)) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED);
    }
  }

  // If realtime: Ensure that ethercat enty for actual position is linked
  if ((refSource == ECMC_DATA_SOURCE_INTERNAL) && data_.status_.inRealtime) {
    int errorCode = getEnc()->validateEntry(
      ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION);

    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  }

  data_.command_.encSource  = refSource;
  controlWord_.encSourceCmd =  data_.command_.encSource ==
                              ECMC_DATA_SOURCE_EXTERNAL;
  return 0;
}

int ecmcAxisBase::setRealTimeStarted(bool realtime) {
  if (realtime) {
    initControlWord();
  }
  data_.status_.inRealtime = realtime;
  return 0;
}

int ecmcAxisBase::getRealTimeStarted() {
  return data_.status_.inRealtime;
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
  if (enable && !data_.command_.enable) {
    errorReset();
    extEncVeloFilter_->initFilter(0);  // init to 0 vel
    extTrajVeloFilter_->initFilter(0);  // init to 0 vel    
    if(!data_.status_.atTarget || !firstEnableDone_ || !mon_->getEnableAtTargetMon()) {      
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

  data_.status_.enableOld = data_.command_.enable;
  data_.command_.enable   = enable;

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
  return encArray_[data_.command_.primaryEncIndex];
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
  return encArray_[data_.command_.cfgEncIndex];
}

// ecmcEncoder * ecmcAxisBase::getHomeEnc() {
//  return encArray_[data_.command_.homeEncIndex];
// }

ecmcEncoder * ecmcAxisBase::getPrimEnc() {
  return encArray_[data_.command_.primaryEncIndex];
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
  if (data_.command_.encSource == ECMC_DATA_SOURCE_EXTERNAL) {
    *homed = 1;
  } else {
    *homed = encArray_[data_.command_.primaryEncIndex]->getHomed();
  }
  return 0;
}

int ecmcAxisBase::setAxisHomed(bool homed) {
  encArray_[data_.command_.primaryEncIndex]->setHomed(homed);
  return 0;
}

int ecmcAxisBase::getEncScaleNum(double *scale) {
  *scale = encArray_[data_.command_.cfgEncIndex]->getScaleNum();
  return 0;
}

int ecmcAxisBase::setEncScaleNum(double scale) {
  encArray_[data_.command_.cfgEncIndex]->setScaleNum(scale);
  return 0;
}

int ecmcAxisBase::getEncScaleDenom(double *scale) {
  *scale = encArray_[data_.command_.cfgEncIndex]->getScaleDenom();
  return 0;
}

int ecmcAxisBase::setEncScaleDenom(double scale) {
  encArray_[data_.command_.cfgEncIndex]->setScaleDenom(scale);
  return 0;
}

int ecmcAxisBase::getEncPosRaw(int64_t *rawPos) {
  *rawPos = encArray_[data_.command_.primaryEncIndex]->getRawPosMultiTurn();
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
  data_.status_.movingOld = data_.status_.moving;
  return 0;
}

void ecmcAxisBase::printAxisStatus() {
  if (memcmp(&statusDataOld_.onChangeData, &statusData_.onChangeData,
             sizeof(statusData_.onChangeData)) == 0) {
    return;  // Printout on change
  }

  statusDataOld_ = statusData_;

  // Only print header once per 25 status lines
  if (printHeaderCounter_ <= 0) {
    LOGINFO(
      "ecmc::  Ax     PosSet     PosAct     PosErr    PosTarg   DistLeft    CntrOut   VelFFSet     VelAct   VelFFRaw VelRaw  Error Co CD St IL LI TS ES En Ex Bu Ta Hd L- L+ Ho\n");
    printHeaderCounter_ = 25;
  }
  printHeaderCounter_--;

  LOGINFO(
    "ecmc:: %3d %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %6i %6x %2d %2d %2d %2d %2d %2d %2d %1d%1d %2d %2d %2d %2d %2d %2d %2d\n",
    statusData_.axisID,
    statusData_.onChangeData.positionSetpoint,
    statusData_.onChangeData.positionActual,
    statusData_.onChangeData.positionError,
    statusData_.onChangeData.positionTarget,
    statusData_.onChangeData.positionTarget - statusData_.onChangeData.positionActual,
    statusData_.onChangeData.cntrlOutput,
    statusData_.onChangeData.velocitySetpoint,
    statusData_.onChangeData.velocityActual,
    statusData_.onChangeData.velocityFFRaw,
    statusData_.onChangeData.velocitySetpointRaw,
    statusData_.onChangeData.error,
    statusData_.onChangeData.command,
    statusData_.onChangeData.cmdData,
    statusData_.onChangeData.seqState,
    statusData_.onChangeData.trajInterlock,
    statusData_.onChangeData.statusWd.lastilock,
    statusData_.onChangeData.statusWd.trajsource,
    statusData_.onChangeData.statusWd.encsource,
    statusData_.onChangeData.statusWd.enable,
    statusData_.onChangeData.statusWd.enabled,
    statusData_.onChangeData.statusWd.execute,
    statusData_.onChangeData.statusWd.busy,
    statusData_.onChangeData.statusWd.attarget,
    statusData_.onChangeData.statusWd.homed,
    statusData_.onChangeData.statusWd.limitbwd,
    statusData_.onChangeData.statusWd.limitfwd,
    statusData_.onChangeData.statusWd.homeswitch);
}

int ecmcAxisBase::setExecute(bool execute) {
  // Internal trajectory source
  if (data_.command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
    // Allow direct homing without enable
    if (execute && !getEnable() &&
        !((data_.command_.cmdData == ECMC_SEQ_HOME_SET_POS) &&
          (data_.command_.command == ECMC_CMD_HOMING))) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_AXIS_NOT_ENABLED);
    }

    if (execute && !data_.status_.executeOld && data_.status_.busy) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_AXIS_BUSY);
    }

    int error = seq_.setExecute(execute);

    if (error) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }
  }
  
  controlWord_.executeCmd = execute;

  return 0;
}

bool ecmcAxisBase::getExecute() {
  if (data_.command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
    return seq_.getExecute();
  } else {
    return true;
  }
}

bool ecmcAxisBase::getBusy() {
  return data_.status_.busy;
}

int ecmcAxisBase::getDebugInfoData(ecmcAxisStatusType *data) {
  if (data == NULL) {
    return ERROR_AXIS_DATA_POINTER_NULL;
  }

  memcpy(data, &statusData_, sizeof(*data));
  return 0;
}

ecmcAxisStatusType * ecmcAxisBase::getDebugInfoDataPointer() {
  return &statusData_;
}

int ecmcAxisBase::getCycleCounter() {
  /// Use for watchdog purpose (will overflow)
  return cycleCounter_;
}

bool ecmcAxisBase::getEnable() {
  return data_.command_.enable;
}

bool ecmcAxisBase::getEnabled() {
  return data_.status_.enabled && data_.command_.enable;
}

bool ecmcAxisBase::getEnabledOnly() {
  return data_.status_.enabled;
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
                              (uint8_t *)&(statusData_.onChangeData.
                                           positionSetpoint),
                              sizeof(statusData_.onChangeData.positionSetpoint),
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
                              (uint8_t *)&(statusData_.onChangeData.cntrlError),
                              sizeof(statusData_.onChangeData.cntrlError),
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_POS_ERR_ID] = paramTemp;

  // Diagnostic string (array)
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_DIAG_NAME,
                              asynParamInt8Array,
                              ECMC_EC_S8,
                              (uint8_t *)diagBuffer_,
                              AX_MAX_DIAG_STRING_CHAR_LENGTH,
                              &paramTemp);

  if (errorCode) {
    return errorCode;
  }

  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_DIAG_ID] = paramTemp;

  // Status word
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_STATUS_NAME,
                              asynParamInt32,
                              ECMC_EC_U32,
                              (uint8_t *)&(statusData_.onChangeData.statusWd),
                              sizeof(statusData_.onChangeData.statusWd),
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
                              (uint8_t *)&(controlWord_),
                              sizeof(controlWord_),
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
                              (uint8_t *)&(statusData_.onChangeData.error),
                              sizeof(statusData_.onChangeData.error),
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

int ecmcAxisBase::getAxisDebugInfoData(char *buffer,
                                       int   bufferByteSize,
                                       int  *bytesUsed) {
  ecmcAxisStatusType data;
  int error = getDebugInfoData(&data);

  if (error) {
    return error;
  }

  // (Ax,PosSet,PosAct,PosErr,PosTarg,DistLeft,CntrOut,VelFFSet,VelAct,VelFFRaw,VelRaw,CycleCounter,Error,Co,CD,St,IL,TS,ES,En,Ena,Ex,Bu,Ta,L-,L+,Ho");
  int ret = snprintf(buffer,
                     bufferByteSize,
                     "%d,%lf,%lf,%lf,%lf,%lf,%" PRId64 ",%lf,%lf,%lf,%lf,%d,%d,%x,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                     data.axisID,
                     data.onChangeData.positionSetpoint,
                     data.onChangeData.positionActual,
                     data.onChangeData.positionError,
                     data.onChangeData.positionTarget,
                     data.onChangeData.positionTarget - data.onChangeData.positionActual,
                     data.onChangeData.positionRaw,
                     data.onChangeData.cntrlOutput,
                     data.onChangeData.velocitySetpoint,
                     data.onChangeData.velocityActual,
                     data.onChangeData.velocityFFRaw,
                     data.onChangeData.velocitySetpointRaw,
                     data.cycleCounter,
                     data.onChangeData.error,
                     data.onChangeData.command,
                     data.onChangeData.cmdData,
                     data.onChangeData.statusWd.seqstate,
                     data.onChangeData.trajInterlock,
                     data.onChangeData.statusWd.lastilock,
                     data.onChangeData.statusWd.trajsource,
                     data.onChangeData.statusWd.encsource,
                     data.onChangeData.statusWd.enable,
                     data.onChangeData.statusWd.enabled,
                     data.onChangeData.statusWd.execute,
                     data.onChangeData.statusWd.busy,
                     data.onChangeData.statusWd.attarget,
                     data.onChangeData.statusWd.homed,
                     data.onChangeData.statusWd.limitbwd,
                     data.onChangeData.statusWd.limitfwd,
                     data.onChangeData.statusWd.homeswitch);

  if ((ret >= bufferByteSize) || (ret <= 0)) {
    *bytesUsed = 0;
    return ERROR_AXIS_PRINT_TO_BUFFER_FAIL;
  }
  *bytesUsed = ret;
  return 0;
}

int ecmcAxisBase::setEcStatusOutputEntry(ecmcEcEntry *entry) {
  statusOutputEntry_ = entry;
  return 0;
}

motionDirection ecmcAxisBase::getAxisSetDirection() {
  if (!data_.status_.enabled) {
    return ECMC_DIR_STANDSTILL;
  }

  // Transform or internal trajectory
  if (data_.command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
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
  data_.command_.moduloRange = mod;
  return 0;
}

double ecmcAxisBase::getModRange() {
  return data_.command_.moduloRange;
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

  data_.command_.moduloType = (ecmcMotionModType)type;
  return 0;
}

int ecmcAxisBase::getModType() {
  return (int)data_.command_.moduloType;
}

//
// double ecmcAxisBase::getPosErrorMod() {
//
//  double normalCaseError = data_.status_.currentPositionSetpoint-data_.status_.currentPositionActual;
//  if(data_.command_.moduloRange==0) {
//    return normalCaseError;
//  }
//
//  // Modulo
//  if(std::abs(normalCaseError)<data_.command_.moduloRange*ECMC_OVER_UNDER_FLOW_FACTOR) {
//    // No overflows
//    return normalCaseError;
//  } else {
//    //Overflow has happended in either encoder or setpoint
//    double overUnderFlowError = data_.command_.moduloRange-std::abs(normalCaseError);
//    double  setDiff = data_.status_.currentPositionSetpoint - data_.status_.currentPositionSetpointOld;
//    //Moving forward (overflow)
//    if(setDiff>0 || setDiff < -data_.command_.moduloRange*ECMC_OVER_UNDER_FLOW_FACTOR) {
//      //Actual lagging setpoint  ACT SET
//      if(data_.status_.currentPositionActual>data_.status_.currentPositionSetpoint) {
//        return overUnderFlowError;
//      }
//      else { //Actual before setpoint SET ACT
//        return -overUnderFlowError;
//      }
//    }
//    else {
//      //Moving backward (underflow)
//      //Actual lagging setpoint SET ACT
//      if(data_.status_.currentPositionActual<data_.status_.currentPositionSetpoint) {
//        return -overUnderFlowError;
//      }
//      else { //Actual before setpoint ACT SET
//        return overUnderFlowError;
//      }
//    }
//  }
//  LOGERR(
//        "%s/%s:%d: WARNING (axis %d): Modulo controller error calc failed...\n",
//        __FILE__,
//        __FUNCTION__,
//        __LINE__,
//        data_.axisId_);
//
//  return 0;
// }
//
int ecmcAxisBase::getCntrlError(double *error) {
  *error = data_.status_.cntrlError;
  return 0;
}

void ecmcAxisBase::refreshStatusWd() {
  // bit 0 enable
  statusData_.onChangeData.statusWd.enable = getEnable() > 0;

  // bit 1 enabled
  statusData_.onChangeData.statusWd.enabled = getEnabled() > 0;

  // bit 2 execute
  statusData_.onChangeData.statusWd.execute = seq_.getExecute() > 0;

  // bit 3 busy
  statusData_.onChangeData.statusWd.busy = data_.status_.busy > 0;

  // bit 4 at target
  statusData_.onChangeData.statusWd.attarget = data_.status_.atTarget > 0;

  // bit 5 moving
  statusData_.onChangeData.statusWd.moving = data_.status_.moving > 0;

  // bit 6 limit fwd
  statusData_.onChangeData.statusWd.limitfwd = data_.status_.limitFwd > 0;

  // bit 7 limit bwd
  statusData_.onChangeData.statusWd.limitbwd = data_.status_.limitBwd > 0;

  // bit 8 homeswitch
  statusData_.onChangeData.statusWd.homeswitch = data_.status_.homeSwitch > 0;

  // bit 9 homed
  bool homedtemp = 0;
  getAxisHomed(&homedtemp);
  statusData_.onChangeData.statusWd.homed = homedtemp > 0;

  // bit 10 inRealtime
  statusData_.onChangeData.statusWd.inrealtime = data_.status_.inRealtime > 0;

  // bit 11 traj source
  statusData_.onChangeData.statusWd.trajsource = data_.command_.trajSource > 0;

  // bit 12 enc source
  statusData_.onChangeData.statusWd.encsource = data_.command_.encSource > 0;

  // bit 13 Allow plc commands
  statusData_.onChangeData.statusWd.plccmdallowed = allowCmdFromOtherPLC_ > 0;

  // bit 14 softlimfwdena
  statusData_.onChangeData.statusWd.softlimfwdena =
    data_.command_.enableSoftLimitFwd > 0;

  // bit 15 softlimbwdena
  statusData_.onChangeData.statusWd.softlimbwdena =
    data_.command_.enableSoftLimitBwd > 0;

  // bit 16 inStartupPhase
  statusData_.onChangeData.statusWd.instartup = data_.status_.inStartupPhase >
                                                0;

  // bit 17 sumilockfwd (filter away execute IL)
  statusData_.onChangeData.statusWd.sumilockfwd =
    data_.interlocks_.trajSummaryInterlockFWDEpics;

  // bit 18 sumilockbwd (filter away execute IL)
  statusData_.onChangeData.statusWd.sumilockbwd =
    data_.interlocks_.trajSummaryInterlockBWDEpics;

  // bit 19 axis type
  statusData_.onChangeData.statusWd.axisType = data_.axisType_ ==
                                               ECMC_AXIS_TYPE_VIRTUAL;

  // bit 20..23 seq state
  statusData_.onChangeData.statusWd.seqstate =
    (unsigned char)data_.status_.seqState;

  // bit 24..31 lastActiveInterlock type
  statusData_.onChangeData.statusWd.lastilock =
    (unsigned char)data_.interlocks_.lastActiveInterlock;
}

void ecmcAxisBase::refreshDebugInfoStruct() {
  refreshStatusWd();

  statusData_.axisID                   = data_.axisId_;
  statusData_.cycleCounter             = cycleCounter_;
  statusData_.onChangeData.cntrlError  = data_.status_.cntrlError;
  statusData_.onChangeData.cntrlOutput = data_.status_.cntrlOutput;

  statusData_.onChangeData.error          = getErrorID();
  statusData_.onChangeData.positionActual =
    data_.status_.currentPositionActual;
  statusData_.onChangeData.positionError =
    ecmcMotionUtils::getPosErrorModWithSign(
      data_.status_.currentPositionSetpoint,
      data_.status_.currentPositionSetpointOld,
      data_.status_.currentPositionActual,
      data_.command_.moduloRange);
  statusData_.onChangeData.positionSetpoint =
    data_.status_.currentPositionSetpoint;
  statusData_.onChangeData.positionTarget =
    data_.status_.currentTargetPosition;
  statusData_.onChangeData.seqState      = seq_.getSeqState();
  statusData_.onChangeData.trajInterlock =
    data_.interlocks_.interlockStatus;
  statusData_.onChangeData.velocityActual =
    data_.status_.currentVelocityActual;
  statusData_.onChangeData.velocitySetpoint =
    data_.status_.currentVelocitySetpoint;
  statusData_.onChangeData.velocitySetpointRaw =
    data_.status_.currentVelocitySetpointRaw;
  statusData_.onChangeData.velocityFFRaw =
    data_.status_.currentvelocityFFRaw;
  statusData_.onChangeData.cmdData     = data_.command_.cmdData;
  statusData_.onChangeData.command     = data_.command_.command;
  statusData_.onChangeData.positionRaw =
    encArray_[data_.command_.primaryEncIndex]->getRawPosRegister();
  statusData_.onChangeData.homeposition = seq_.getHomePosition();
  statusData_.acceleration              = traj_->getAcc();
  statusData_.deceleration              = traj_->getDec();
  statusData_.reset                     = data_.command_.reset;
  statusData_.moving                    = data_.status_.moving;
  statusData_.stall                     =
    data_.interlocks_.lagTrajInterlock
    || data_.interlocks_.
    lagDriveInterlock
    || data_.interlocks_.
    velocityDiffTrajInterlock
    || data_.interlocks_.
    velocityDiffDriveInterlock;
}

int ecmcAxisBase::setEnable(bool enable) {
  if (data_.command_.enable == enable)return 0;

  if (!enable) {  // Remove execute if enable is going down
    setExecute(false);
  }

  if (enable && validate()) {
    setExecute(false);
    return getErrorID();
  }

  if (!getEnc()->hwReady()) {
    data_.command_.enable = false;
    return setErrorID(ERROR_ENC_NOT_READY);
  }

  if (getMon()->getSafetyInterlock() && enable) {
    data_.command_.enable = false;
    return setErrorID(ERROR_AXIS_SAFETY_IL_ACTIVE);
  }

  int error = setEnableLocal(enable);

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  controlWord_.enableCmd = enable;

  // Cascade commands via command transformation
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
  return data_.command_.trajSource;
}

dataSource ecmcAxisBase::getEncDataSourceType() {
  return data_.command_.encSource;
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
  return encArray_[data_.command_.primaryEncIndex]->setVeloFilterSize(size);
}

int ecmcAxisBase::setEncPosFiltSize(size_t size) {
  return encArray_[data_.command_.primaryEncIndex]->setPosFilterSize(size);
}

int ecmcAxisBase::setEncPosFiltEnable(bool enable) {
  return encArray_[data_.command_.primaryEncIndex]->setPosFilterEnable(enable);
}

int ecmcAxisBase::setEncVeloFiltEnable(bool enable) {
  return encArray_[data_.command_.primaryEncIndex]->setVelFilterEnable(enable);
}

int ecmcAxisBase::setEncInvHwReady(int invert) {
  return encArray_[data_.command_.cfgEncIndex]->setInvHwReady(invert);
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

  if(getBusy()) {
    return ERROR_AXIS_BUSY;
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

  errorCode = setExecute(1);
  return 0;

}

int ecmcAxisBase::movePVTRel() {
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

  if(getBusy()) {
    return ERROR_AXIS_BUSY;
  }

  int errorCode = getErrorID();

  if (errorCode) {
    return errorCode;
  }

  errorCode = setExecute(0);

  if (errorCode) {
    return errorCode;
  }

  errorCode = setCommand(ECMC_CMD_MOVEPVTREL);

  if (errorCode) {
    return errorCode;
  }

  errorCode = setCmdData(0);

  if (errorCode) {
    return errorCode;
  }

  errorCode = setExecute(1);
  return 0;

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
 * controlWord_.enableCmd
 * controlWord_.executeCmd
 * controlWord_.stopCmd
 * controlWord_.resetCmd
 * controlWord_.encSourceCmd
 * controlWord_.trajSourceCmd
 * controlWord_.plcEnableCmd
 * controlWord_.plcCmdsAllowCmd
 * controlWord_.enableSoftLimitBwd
 * controlWord_.enableSoftLimitFwd
 * controlWord_.enableILockChangePrintout
 *
*/
asynStatus ecmcAxisBase::axisAsynWriteCmd(void         *data,
                                          size_t        bytes,
                                          asynParamType asynParType) {
  asynStatus returnVal = asynSuccess;

  if (sizeof(controlWord_) != bytes) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Control word size missmatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_);

    setWarningID(WARNING_AXIS_ASYN_CMD_DATA_ERROR);
    return asynError;
  }

  memcpy(&controlWord_, data, sizeof(controlWord_));
  uint32_t *tmpcontrolWordPtr = (uint32_t *)&controlWord_;
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
    if (!controlWord_.enableCmd) {
      setEnable(controlWord_.enableCmd);
      refreshNeeded = true;
    }

    if (controlWord_.stopCmd) {
      setExecute(0);
      controlWord_.stopCmd = 0;  // auto reset
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

  // make printouts when interlock state changes
  data_.command_.enableDbgPrintout = controlWord_.enableDbgPrintout;

  int errorCode = 0;

  errorCode = setEnable(controlWord_.enableCmd);

  if (errorCode) {
    returnVal = asynError;
  }

  if (controlWord_.stopCmd) {
    errorCode            = setExecute(0);
    controlWord_.stopCmd = 0;  // auto reset

    if (errorCode) {
      returnVal = asynError;
    }
  }

  // trigg new motion.
  if (!controlWord_.stopCmd && controlWord_.executeCmd) {
    if (getSeq() == NULL) {
      controlWord_.executeCmd = 0; // auto reset
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
  controlWord_.executeCmd = 0; // auto reset

  setReset(controlWord_.resetCmd);  // resetCmd is reset in setReset()


  errorCode =
    setEncDataSourceType((controlWord_.encSourceCmd ? ECMC_DATA_SOURCE_EXTERNAL
  :
                          ECMC_DATA_SOURCE_INTERNAL));

  if (errorCode) {
    returnVal = asynError;
  }

  errorCode =
    setTrajDataSourceType((controlWord_.trajSourceCmd ?
                           ECMC_DATA_SOURCE_EXTERNAL
  : ECMC_DATA_SOURCE_INTERNAL));

  if (errorCode) {
    returnVal = asynError;
  }

  errorCode = setAllowCmdFromPLC(controlWord_.plcCmdsAllowCmd);

  if (errorCode) {
    returnVal = asynError;
  }

  errorCode = setAxisPLCEnable(data_.axisId_, controlWord_.plcEnableCmd);

  if (errorCode) {
    returnVal = asynError;
  }

  errorCode =
    getMon()->setEnableSoftLimitBwd(controlWord_.enableSoftLimitBwd);

  if (errorCode) {
    returnVal = asynError;
  }

  errorCode =
    getMon()->setEnableSoftLimitFwd(controlWord_.enableSoftLimitFwd);

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
  controlWord_.plcEnableCmd = plcEnable;
  controlWord_.enableCmd    = getEnable();
  controlWord_.executeCmd   = getExecute();
  controlWord_.resetCmd     = getReset();
  controlWord_.encSourceCmd = getEncDataSourceType() ==
                              ECMC_DATA_SOURCE_EXTERNAL;
  controlWord_.trajSourceCmd = getTrajDataSourceType() ==
                               ECMC_DATA_SOURCE_EXTERNAL;
  controlWord_.plcCmdsAllowCmd    = getAllowCmdFromPLC();
  controlWord_.enableSoftLimitBwd = data_.command_.enableSoftLimitBwd;
  controlWord_.enableSoftLimitFwd = data_.command_.enableSoftLimitFwd;
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
  data_.command_.cfgEncIndex = data_.status_.encoderCount; // Use current encoder index for cfg
  data_.status_.encoderCount++;

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

  if(!data_.status_.inStartupPhase) {  // Allow to switch if in startup phase (and busy)
    // Do not allow to switch encoder if busy and INTERNAL source
    if (getBusy() && (data_.command_.encSource ==
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
  if (localIndex != data_.command_.primaryEncIndex) {
    seq_.setNewPositionCtrlDrvTrajBumpless(encArray_[localIndex]->getActPos());
  }

  // This index is starting from 0
  data_.command_.primaryEncIndex = localIndex;

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
  data_.command_.cfgEncIndex = localIndex;
  return 0;
}

// unfortenatelly starts from 1... why did I do like this..
int ecmcAxisBase::getConfigEncoderIndex() {
  return data_.command_.cfgEncIndex + 1;
}

int ecmcAxisBase::getPrimaryEncoderIndex() {
  return data_.command_.primaryEncIndex + 1;
}

bool ecmcAxisBase::getHwReady() {

  // TODO Check drives TODO

  /* Only check prim encoder (allow encoders to be in error state 
  if they are not used) */
  return getPrimEnc()->hwReady();
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
  if ((data_.command_.trajSource != ECMC_DATA_SOURCE_INTERNAL) && stop) {
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
  if(!data_.status_.enabled && sync) {
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
  return encArray_[data_.command_.cfgEncIndex]->loadLookupTable(filename);
}

int  ecmcAxisBase::setEncLookupTableEnable(int enable) {
  return encArray_[data_.command_.cfgEncIndex]->setLookupTableEnable(enable);
}

int ecmcAxisBase::getSumInterlock() {
  return (data_.interlocks_.interlockStatus > 0) && 
         (data_.interlocks_.interlockStatus != ECMC_INTERLOCK_NO_EXECUTE);
}

int ecmcAxisBase::getPrintDbg() {
  return controlWord_.enableDbgPrintout;
}
