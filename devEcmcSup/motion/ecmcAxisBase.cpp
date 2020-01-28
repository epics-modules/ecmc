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


/**
 * 
 * Callback function for asynWrites (commands)
 * userObj = axis object
 * 
 * */ 
/*asynStatus asynWriteCmd(void* data, size_t bytes, asynParamType asynParType,void *userObj) {
  if (!userObj) {
    return asynError;
  }
  return ((ecmcAxisBase*)userObj)->axisAsynWriteCmd(data, bytes, asynParType);
}*/

ecmcAxisBase::ecmcAxisBase(ecmcAsynPortDriver *asynPortDriver,
                           int axisID, 
                           double sampleTime) {
  initVars();

  asynPortDriver_                 = asynPortDriver;
  data_.axisId_                   = axisID;
  data_.sampleTime_               = sampleTime;
  data_.command_.operationModeCmd = ECMC_MODE_OP_AUTO;

  try {
    enc_  = new ecmcEncoder(&data_, data_.sampleTime_);
    traj_ = new ecmcTrajectoryTrapetz(&data_,
                                    data_.sampleTime_);
    mon_ = new ecmcMonitor(&data_);

    extTrajVeloFilter_ = new ecmcFilter(data_.sampleTime_);    
    extEncVeloFilter_ = new ecmcFilter(data_.sampleTime_);    

  } catch(std::bad_alloc& ex) {
      LOGERR(
        "%s/%s:%d: ERROR (axis %d): Mem alloc error.\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        data_.axisId_);

        exit(1);
  }

  seq_.setAxisDataRef(&data_);
  seq_.setTraj(traj_);
  seq_.setMon(mon_);
  seq_.setEnc(enc_);

  initAsyn();
}

ecmcAxisBase::~ecmcAxisBase() {
  delete enc_;
  enc_ = NULL;
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
  data_.axisType_              = ECMC_AXIS_TYPE_BASE;
  data_.command_.reset         = false;
  allowCmdFromOtherPLC_        = false;
  plcEnable_                   = false;
  data_.status_.inStartupPhase = false;
  data_.status_.inRealtime = false;
  data_.status_.externalTrajectoryPosition  = 0;
  data_.status_.externalTrajectoryVelocity  = 0;
  data_.status_.externalEncoderPosition  = 0;
  data_.status_.externalEncoderVelocity  = 0;
  data_.status_.currentPositionActual   = 0;
  data_.status_.currentPositionSetpoint = 0;
  data_.status_.currentVelocityActual   = 0;
  data_.status_.currentVelocitySetpoint = 0;
  data_.sampleTime_ = 1 / 1000;
  memset(&statusData_,    0, sizeof(statusData_));
  memset(&statusDataOld_, 0, sizeof(statusDataOld_));
  printHeaderCounter_      = 0;
  data_.status_.enabledOld = false;
  data_.status_.enableOld  = false;
  data_.status_.executeOld = false;
  cycleCounter_            = 0;
  axisState_               = ECMC_AXIS_STATE_STARTUP;
  oldPositionAct_          = 0;
  oldPositionSet_          = 0;
  asynPortDriver_          = NULL;
  for (int i = 0; i < ECMC_ASYN_AX_PAR_COUNT; i++) {
    axAsynParams_[i] = NULL;
  }
  statusOutputEntry_          = 0;
  blockExtCom_                = 0;
  //memset(&controlData_,0,sizeof(controlData_));
  //memset(&controlDataOld_,0,sizeof(controlDataOld_));
  memset(diagBuffer_,0,AX_MAX_DIAG_STRING_CHAR_LENGTH);
  extTrajVeloFilter_ = NULL;
  extEncVeloFilter_ = NULL;
  enableExtTrajVeloFilter_ = false;
  enableExtEncVeloFilter_ = false;
}

void ecmcAxisBase::preExecute(bool masterOK) {

  data_.interlocks_.etherCatMasterInterlock = !masterOK;
  data_.refreshInterlocks();

  statusData_.onChangeData.statusWd.trajsource = data_.command_.trajSource;
  statusData_.onChangeData.statusWd.encsource = data_.command_.encSource;
  data_.status_.moving = std::abs(
    data_.status_.currentVelocityActual) > 0;

  if (data_.command_.encSource == ECMC_DATA_SOURCE_INTERNAL) {
    enc_->readEntries();
  }

  // Axis state machine
  switch (axisState_) {
  case ECMC_AXIS_STATE_STARTUP:
    setEnable(false);
    data_.status_.busy       = false;
    data_.status_.distToStop = 0;

    if (masterOK) {
      // Auto reset hardware error if starting up
      if ((getErrorID() == ERROR_AXIS_HARDWARE_STATUS_NOT_OK) &&
          data_.status_.inStartupPhase) {
        errorReset();
        setInStartupPhase(false);
        enc_->setToZeroIfRelative();
      }
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

    if (data_.command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
      data_.status_.currentTargetPosition = traj_->getTargetPos();
    } else {  // Synchronized to other axis
      data_.status_.currentTargetPosition =
        data_.status_.currentPositionSetpoint;
    }

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
  if(enableExtTrajVeloFilter_ && extTrajVeloFilter_) {
    data_.status_.externalTrajectoryVelocity = extTrajVeloFilter_->getFiltVelo(
      data_.status_.externalTrajectoryPosition -
      data_.status_.externalTrajectoryPositionOld);
  }
  else {
    data_.status_.externalTrajectoryVelocity = (data_.status_.externalTrajectoryPosition -
      data_.status_.externalTrajectoryPositionOld) / data_.sampleTime_;      
  }

  // Enc
  if(enableExtEncVeloFilter_ && extEncVeloFilter_) {
    data_.status_.externalEncoderVelocity = extEncVeloFilter_->getFiltVelo(
      data_.status_.externalEncoderPosition -
      data_.status_.externalEncoderPositionOld);
  } else {
    data_.status_.externalEncoderVelocity = (data_.status_.externalEncoderPosition -
      data_.status_.externalEncoderPositionOld) / data_.sampleTime_;
  }
}

void ecmcAxisBase::postExecute(bool masterOK) {  

  data_.status_.externalTrajectoryPositionOld = 
            data_.status_.externalTrajectoryPosition;
  data_.status_.externalEncoderPositionOld = 
            data_.status_.externalEncoderPosition;

  // Write encoder entries
  enc_->writeEntries();
  data_.status_.busyOld                    = data_.status_.busy;
  data_.status_.enabledOld                 = data_.status_.enabled;
  data_.status_.executeOld                 = getExecute();
  data_.status_.currentPositionSetpointOld =
    data_.status_.currentPositionSetpoint;
  data_.status_.cntrlOutputOld             = data_.status_.cntrlOutput;

  cycleCounter_++;
  refreshDebugInfoStruct();
  
  // Update asyn parameters  
  axAsynParams_[ECMC_ASYN_AX_ACT_POS_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_SET_POS_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_POS_ERR_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_STATUS_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_STATUS_BIN_ID]->refreshParamRT(0);
  
  if(axAsynParams_[ECMC_ASYN_AX_DIAG_ID]->willRefreshNext() && axAsynParams_[ECMC_ASYN_AX_DIAG_ID]->initialized() ) {    
    int  bytesUsed = 0;
    int  error = getAxisDebugInfoData(&diagBuffer_[0],
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
      axAsynParams_[ECMC_ASYN_AX_DIAG_ID]->refreshParamRT(1,(uint8_t*)diagBuffer_,bytesUsed);
    }
  }
  else {
    //Just to count up for correct freq
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
}

bool ecmcAxisBase::getReset() {
  return data_.command_.reset;
}

int ecmcAxisBase::setAllowCmdFromPLC(bool enable) {
  allowCmdFromOtherPLC_ = enable;
  return 0;
}

bool ecmcAxisBase::getAllowCmdFromPLC() {
  return allowCmdFromOtherPLC_;
}

void ecmcAxisBase::setInStartupPhase(bool startup) {
  data_.status_.inStartupPhase = startup;
}

int ecmcAxisBase::setDriveType(ecmcDriveTypes driveType) {
  return setErrorID(__FILE__,
                    __FUNCTION__,
                    __LINE__,
                    ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

int ecmcAxisBase::setTrajDataSourceType(dataSource refSource) {
  
  if (getEnable() && (refSource != ECMC_DATA_SOURCE_INTERNAL)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED);
  }
  
  if (refSource != ECMC_DATA_SOURCE_INTERNAL) {
    data_.interlocks_.noExecuteInterlock = false;
    data_.refreshInterlocks();
    data_.status_.busy = true;
  }

  data_.command_.trajSource = refSource;
  return 0;
}

int ecmcAxisBase::setEncDataSourceType(dataSource refSource) {

  if (getEnable() && (refSource != ECMC_DATA_SOURCE_INTERNAL)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED);
  }

  // If realtime: Ensure that ethercat enty for actual position is linked
  if ((refSource == ECMC_DATA_SOURCE_INTERNAL) && data_.status_.inRealtime) {
    
    int errorCode = getEnc()->validateEntry(
      ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION);

    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  }
  
  data_.command_.encSource = refSource;
  return 0;
}

int ecmcAxisBase::setRealTimeStarted(bool realtime) {
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
  // GeneralsetErrorID
  if (ecmcError::getError()) {
    return ecmcError::getErrorID();
  }

  // Monitor
  ecmcMonitor *mon = getMon();

  if (mon) {
    if (mon->getError()) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, mon->getErrorID());
    }
  }

  // Encoder
  ecmcEncoder *enc = getEnc();

  if (enc) {
    if (enc->getError()) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, enc->getErrorID());
    }
  }

  // Drive
  ecmcDriveBase *drv = getDrv();

  if (drv) {
    if (drv->getError()) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, drv->getErrorID());
    }
  }

  // Trajectory
  ecmcTrajectoryTrapetz *traj = getTraj();

  if (traj) {
    if (traj->getError()) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, traj->getErrorID());
    }
  }

  // Controller
  ecmcPIDController *cntrl = getCntrl();

  if (cntrl) {
    if (cntrl->getError()) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, cntrl->getErrorID());
    }
  }

  // Sequencer
  ecmcAxisSequencer *seq = getSeq();

  if (seq) {
    if (seq->getErrorID()) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, seq->getErrorID());
    }
  }

  return ecmcError::getErrorID();
}

int ecmcAxisBase::setEnableLocal(bool enable) {
  if (enable && !data_.command_.enable) {
    traj_->setStartPos(data_.status_.currentPositionActual);
    traj_->setCurrentPosSet(data_.status_.currentPositionActual);
    traj_->setTargetPos(data_.status_.currentPositionActual);
    data_.status_.currentTargetPosition = data_.status_.currentPositionActual;
  }
  traj_->setEnable(enable);

  data_.status_.enableOld = data_.command_.enable;
  data_.command_.enable   = enable;
  return 0;
}

void ecmcAxisBase::errorReset() {
  // Monitor
  ecmcMonitor *mon = getMon();

  if (mon) {
    mon->errorReset();
  }

  // Encoder
  ecmcEncoder *enc = getEnc();

  if (enc) {
    enc->errorReset();
  }

  // Drive
  ecmcDriveBase *drv = getDrv();

  if (drv) {
    drv->errorReset();
  }

  // Trajectory
  ecmcTrajectoryTrapetz *traj = getTraj();

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
  if ((data_.command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) && getSeq()) {
    *pos = data_.command_.positionTarget;
  } else {
    *pos = data_.status_.currentPositionSetpoint;
  }

  return 0;
}

ecmcEncoder * ecmcAxisBase::getEnc() {
  return enc_;
}

ecmcTrajectoryTrapetz * ecmcAxisBase::getTraj() {
  return traj_;
}

ecmcMonitor * ecmcAxisBase::getMon() {
  return mon_;
}

ecmcAxisSequencer * ecmcAxisBase::getSeq() {
  return &seq_;
}

int ecmcAxisBase::getAxisHomed(bool *homed) {
  *homed = enc_->getHomed();
  return 0;
}

int ecmcAxisBase::setAxisHomed(bool homed) {
  enc_->setHomed(homed);
  return 0;
}

int ecmcAxisBase::getEncScaleNum(double *scale) {
  *scale = enc_->getScaleNum();
  return 0;
}

int ecmcAxisBase::setEncScaleNum(double scale) {
  enc_->setScaleNum(scale);
  return 0;
}

int ecmcAxisBase::getEncScaleDenom(double *scale) {
  *scale = enc_->getScaleDenom();
  return 0;
}

int ecmcAxisBase::setEncScaleDenom(double scale) {
  enc_->setScaleDenom(scale);
  return 0;
}

int ecmcAxisBase::getEncPosRaw(int64_t *rawPos) {
  *rawPos = enc_->getRawPosMultiTurn();
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
  oldPositionAct_ = data_.status_.currentPositionActual;
  oldPositionSet_ = data_.status_.currentPositionSetpoint;
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
    statusData_.onChangeData.cntrlError,
    statusData_.onChangeData.positionTarget,
    statusData_.onChangeData.positionError,
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
        !((data_.command_.cmdData == ECMC_SEQ_HOME_SET_POS) && (data_.command_.command == ECMC_CMD_HOMING))) {
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

ecmcAxisStatusType *ecmcAxisBase::getDebugInfoDataPointer() {
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
  int errorCode = 0;
 
  // Act pos
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_ACT_POS_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(statusData_.onChangeData.positionActual),
                              sizeof(statusData_.onChangeData.positionActual),
                              &paramTemp);
  if(errorCode) {
    return errorCode;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_ACT_POS_ID] = paramTemp;

  // Set pos
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_SET_POS_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(statusData_.onChangeData.positionSetpoint),
                              sizeof(statusData_.onChangeData.positionSetpoint),
                              &paramTemp);
  if(errorCode) {
    return errorCode;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_SET_POS_ID] = paramTemp;

  // Pos Error
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_POS_ERR_NAME,
                              asynParamFloat64,
                              ECMC_EC_F64,
                              (uint8_t *)&(statusData_.onChangeData.cntrlError),
                              sizeof(statusData_.onChangeData.cntrlError),
                              &paramTemp);
  if(errorCode) {
    return errorCode;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_POS_ERR_ID] = paramTemp;

  // Diagnostic string (array)
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_DIAG_NAME,
                              asynParamInt8Array,
                              ECMC_EC_S8,
                              (uint8_t *)diagBuffer_,
                              AX_MAX_DIAG_STRING_CHAR_LENGTH,
                              &paramTemp);
  if(errorCode) {
    return errorCode;
  }

  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_DIAG_ID] = paramTemp;

  // Status binary struct (for motor record)
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_STATUS_BIN_NAME,
                              asynParamInt8Array,
                              ECMC_EC_S8,
                              (uint8_t *)&(statusData_),
                              sizeof(ecmcAxisStatusType),
                              &paramTemp);
  if(errorCode) {
    return errorCode;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_STATUS_BIN_ID] = paramTemp;
 
  // Status word
  errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_STATUS_NAME,
                              asynParamInt32,
                              ECMC_EC_U32,
                              (uint8_t*)&(statusData_.onChangeData.statusWd),
                              sizeof(statusData_.onChangeData.statusWd),
                              &paramTemp);
  if(errorCode) {
    return errorCode;
  }
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);    
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_STATUS_ID] = paramTemp;

  // Control structure binary (for motor record)
  /*errorCode = createAsynParam(ECMC_AX_STR "%d." ECMC_ASYN_AX_CONTROL_BIN_NAME,
                              asynParamInt8Array,
                              ECMC_EC_S8,
                              (uint8_t*)&(controlData_),
                              sizeof(controlData_),
                              &paramTemp);
  if(errorCode) {
    return errorCode;
  }
  paramTemp->allowWriteToEcmc(true);
  paramTemp->setExeCmdFunctPtr(asynWriteCmd,this); // Access to this axis
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID] = paramTemp;
*/

  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);
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
                     data.onChangeData.cntrlError,
                     data.onChangeData.positionTarget,
                     data.onChangeData.positionError,
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
  //Must be same mod factor in traj and enc
  if(mod<0) {
    LOGERR(
       "ERROR (axis %d): Modulo factor out of range. Must be a positive value (0x%x).\n",
        data_.axisId_,ERROR_AXIS_MODULO_OUT_OF_RANGE);

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
  
  if(type < 0 || type >= ECMC_MOD_MOTION_MAX) {
    LOGERR(
       "ERROR (axis %d): Modulo type out of range (0x%x).\n",
        data_.axisId_,ERROR_AXIS_MODULO_TYPE_OUT_OF_RANGE);

    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_MODULO_TYPE_OUT_OF_RANGE);
  }
  
  data_.command_.moduloType = (ecmcMotionModType) type;
  return 0;
}

int ecmcAxisBase::getModType() {
  return (int)data_.command_.moduloType;
}

double ecmcAxisBase::getPosErrorMod() {

  double normalCaseError = data_.status_.currentPositionSetpoint-data_.status_.currentPositionActual;
  if(data_.command_.moduloRange==0) {
    return normalCaseError;
  }
  
  // Modulo
  if(std::abs(normalCaseError)<data_.command_.moduloRange*ECMC_OVER_UNDER_FLOW_FACTOR) {
    // No overflows 
    return normalCaseError;
  } else {
    //Overflow has happended in either encoder or setpoint
    double overUnderFlowError = data_.command_.moduloRange-std::abs(normalCaseError);
    double  setDiff = data_.status_.currentPositionSetpoint - data_.status_.currentPositionSetpointOld;
    //Moving forward (overflow)
    if(setDiff>0 || setDiff < -data_.command_.moduloRange*ECMC_OVER_UNDER_FLOW_FACTOR) {
      //Actual lagging setpoint  ACT SET
      if(data_.status_.currentPositionActual>data_.status_.currentPositionSetpoint) {         
        return overUnderFlowError;
      } 
      else { //Actual before setpoint SET ACT
        return -overUnderFlowError;
      }
    }
    else {
      //Moving backward (underflow)    
      //Actual lagging setpoint SET ACT
      if(data_.status_.currentPositionActual<data_.status_.currentPositionSetpoint) {             
        return -overUnderFlowError;
      } 
      else { //Actual before setpoint ACT SET
        return overUnderFlowError;
      }       
    }         
  }
  LOGERR(
        "%s/%s:%d: WARNING (axis %d): Modulo controller error calc failed...\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        data_.axisId_);

  return 0;
}

int ecmcAxisBase::getCntrlError(double *error) {
  *error = data_.status_.cntrlError;
  return 0;
}

void ecmcAxisBase::refreshDebugInfoStruct() {

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
  // bit 9 inStartupPhase
  statusData_.onChangeData.statusWd.instartup = data_.status_.inStartupPhase > 0;
  // bit 10 inRealtime  
  statusData_.onChangeData.statusWd.inrealtime = data_.status_.inRealtime > 0;
  // bit 11 traj source  
  statusData_.onChangeData.statusWd.trajsource = data_.command_.trajSource > 0;
  // bit 12 enc source  
  statusData_.onChangeData.statusWd.encsource = data_.command_.encSource > 0;
  // bit 13 Allow plc commands  
  statusData_.onChangeData.statusWd.plccmdallowed = allowCmdFromOtherPLC_ > 0;
  // bit 14 softlimfwdena
  statusData_.onChangeData.statusWd.softlimfwdena = data_.command_.enableSoftLimitFwd > 0;
  // bit 15 softlimbwdena  
  statusData_.onChangeData.statusWd.softlimfwdena = data_.command_.enableSoftLimitBwd > 0;
  // bit 16 homed
  bool homedtemp = 0;
  getAxisHomed(&homedtemp);
  statusData_.onChangeData.statusWd.homed = homedtemp > 0;
  // bit 17..19 unused
  // bit 20..23 seq state
  statusData_.onChangeData.statusWd.seqstate = (unsigned char)data_.status_.seqState;
  // bit 24..31 lastActiveInterlock type
  statusData_.onChangeData.statusWd.lastilock = (unsigned char)data_.interlocks_.lastActiveInterlock;

  statusData_.axisID                          = data_.axisId_;
  statusData_.cycleCounter                    = cycleCounter_;
  statusData_.onChangeData.cntrlError         = data_.status_.cntrlError;
  statusData_.onChangeData.cntrlOutput        = data_.status_.cntrlOutput;

  statusData_.onChangeData.error              = getErrorID(); 
  statusData_.onChangeData.positionActual     =
    data_.status_.currentPositionActual;
  statusData_.onChangeData.positionError      = getPosErrorMod();    
  statusData_.onChangeData.positionSetpoint   =
    data_.status_.currentPositionSetpoint;
  statusData_.onChangeData.positionTarget     =
    data_.status_.currentTargetPosition;
  statusData_.onChangeData.seqState           = seq_.getSeqState();
  statusData_.onChangeData.trajInterlock      =
    data_.interlocks_.interlockStatus;
  statusData_.onChangeData.statusWd.sumilockbwd = 
    data_.interlocks_.trajSummaryInterlockBWD;
  statusData_.onChangeData.statusWd.sumilockfwd = 
    data_.interlocks_.trajSummaryInterlockFWD;
  statusData_.onChangeData.velocityActual =
    data_.status_.currentVelocityActual;
  statusData_.onChangeData.velocitySetpoint =
    data_.status_.currentVelocitySetpoint;
  statusData_.onChangeData.velocitySetpointRaw =
    data_.status_.currentVelocitySetpointRaw;
  statusData_.onChangeData.velocityFFRaw =
    data_.status_.currentvelocityFFRaw;
  statusData_.onChangeData.cmdData        = data_.command_.cmdData;
  statusData_.onChangeData.command        = data_.command_.command;
  statusData_.onChangeData.positionRaw    = enc_->getRawPosRegister();
  statusData_.onChangeData.homeposition   = seq_.getHomePosition();
  statusData_.acceleration                = traj_->getAcc();
  statusData_.deceleration                = traj_->getDec();
  statusData_.reset                       = data_.command_.reset;
  statusData_.moving                      = data_.status_.moving;
  statusData_.stall                       =
    data_.interlocks_.lagTrajInterlock
    || data_.interlocks_.
    lagDriveInterlock
    || data_.interlocks_.
    velocityDiffTrajInterlock
    || data_.interlocks_.
    velocityDiffDriveInterlock;
}

int ecmcAxisBase::setEnable(bool enable) {
  if (!enable) {  // Remove execute if enable is going down
    setExecute(false);
  }
  
  if (enable && validate()) {
    setExecute(false);
    return getErrorID();
  }

  int error = setEnableLocal(enable);

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  // Cascade commands via command transformation
  return 0;
}


int ecmcAxisBase::setExtSetPos(double pos) {
  data_.status_.externalTrajectoryPosition = pos;
  return 0;
}

int ecmcAxisBase::setExtActPos(double pos) {
  data_.status_.externalEncoderPosition = pos;
  return 0;
}

int ecmcAxisBase::setEnableExtTrajVeloFilter(bool enable) {

  if(!extTrajVeloFilter_) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_FILTER_OBJECT_NULL);    
  }

  if (enable && !enableExtTrajVeloFilter_) {
    extTrajVeloFilter_->reset();
  }

  enableExtTrajVeloFilter_ = enable;

  return 0;
}

int ecmcAxisBase::setEnableExtEncVeloFilter(bool enable) {

  if(!extEncVeloFilter_) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_FILTER_OBJECT_NULL);    
  }

  if (enable && !enableExtEncVeloFilter_) {
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

int ecmcAxisBase::setExtTrajVeloFiltSize(size_t size){
  return extTrajVeloFilter_->setFilterSize(size);
}

int ecmcAxisBase::setExtEncVeloFiltSize(size_t size) {
  return extEncVeloFilter_->setFilterSize(size);
}

int ecmcAxisBase::setEncVeloFiltSize(size_t size) {
  return enc_->setVeloFilterSize(size);
}

int ecmcAxisBase::createAsynParam(const char       *nameFormat,
                                  asynParamType     asynType, 
                                  ecmcEcDataType    ecmcType,
                                  uint8_t*          data, 
                                  size_t            bytes,                                  
                                  ecmcAsynDataItem **asynParamOut) {
  if (asynPortDriver_ == NULL) {
    LOGERR("%s/%s:%d: ERROR (axis %d): AsynPortDriver object NULL (%s) (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_.axisId_,
           nameFormat,
           ERROR_AXIS_ASYN_PORT_OBJ_NULL);
    return ERROR_AXIS_ASYN_PORT_OBJ_NULL;
  }
  *asynParamOut = NULL;
  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char *name = NULL;
  unsigned int charCount = 0;
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
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                                asynType,
                                                data,
                                                bytes,
                                                ecmcType,
                                                0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  *asynParamOut = paramTemp;
  return 0;
}

int ecmcAxisBase::moveAbsolutePosition(                            
                            double positionSet,
                            double velocitySet,
                            double accelerationSet,
                            double decelerationSet) {
 
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
  getSeq()->setTargetPos(positionSet);
  getSeq()->setTargetVel(velocitySet);
  getTraj()->setAcc(accelerationSet);
  getTraj()->setDec(decelerationSet);
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

  getSeq()->setTargetPos(positionSet);
  getSeq()->setTargetVel(velocitySet);
  getTraj()->setAcc(accelerationSet);
  getTraj()->setDec(decelerationSet);

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

  int errorCode = getErrorID();
  if (errorCode) {
    return errorCode;
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
  getTraj()->setAcc(accelerationSet);
  getTraj()->setAcc(decelerationSet);
  
  errorCode = setExecute(1);
  if (errorCode) {
    return errorCode;
  }
  return 0;
}

int ecmcAxisBase::moveHome(int    nCmdData,
                           double homePositionSet,
                           double velocityTwordsCamSet,
                           double velocityOffCamSet,                            
                           double accelerationSet,
                           double decelerationSet
                           ) {
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
  errorCode = setCmdData(nCmdData);

  if (errorCode) {
    return errorCode;
  }
  getSeq()->setHomePosition(homePositionSet);
  getSeq()->setHomeVelOffCam(velocityOffCamSet);
  getSeq()->setHomeVelTwordsCam(velocityTwordsCamSet);
  getTraj()->setAcc(accelerationSet);
  getTraj()->setDec(decelerationSet);
  errorCode = setExecute(1);

  if (errorCode) {
    return errorCode;
  }
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
*/
/*asynStatus ecmcAxisBase::axisAsynWriteCmd(void* data, size_t bytes, asynParamType asynParType) 
{
  if(bytes != sizeof(ecmcAsynClinetCmdType)){
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Asyn command data size mssmatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_);
    return asynError;
  }

  memcpy(&controlData_,data,bytes);
  //printf("axisAsynWriteCmd CMD = %d!!!\n", (int)controlData_.cmd);
  int errorCode = 0;
  int cmddata   = 0;
  switch(controlData_.cmd) {
    case ECMC_CMD_NOCMD:
      return asynError;
      break;

    case ECMC_CMD_JOG:
      return asynError;
      break;

    case ECMC_CMD_MOVEVEL:      
      // cmd, vel,acc
      if (getBusy() || getErrorID()) {
        return asynError;
      }
      errorCode = setExecute(0);
      if (errorCode) {
        return asynError;
      }
      errorCode = setCommand(ECMC_CMD_MOVEVEL);
      if (errorCode) {
        return asynError;
      }
      errorCode = setCmdData(0);
      if (errorCode) {
        return asynError;
      }
      seq_.setTargetVel(controlData_.val0);
      traj_->setAcc(controlData_.val1);
      traj_->setDec(controlData_.val1); //same as acc
      errorCode = setExecute(1);
      if (errorCode) {
        return asynError;
      }      
      break;

    case ECMC_CMD_MOVEREL:
      // cmd, pos,vel,acc
      if (getBusy() || getErrorID()) {
        return asynError;
      }
      errorCode = setExecute(0);
      if (errorCode) {
        return asynError;
      }
      errorCode = setCommand(ECMC_CMD_MOVEREL);
      if (errorCode) {
        return asynError;
      }
      errorCode = setCmdData(0);
      if (errorCode) {
        return asynError;
      }
      seq_.setTargetPos(controlData_.val0);
      seq_.setTargetVel(controlData_.val1);
      traj_->setAcc(controlData_.val2);
      traj_->setDec(controlData_.val2); //same as acc
      errorCode = setExecute(1);
      if (errorCode) {
        return asynError;
      }
      break;

    case ECMC_CMD_MOVEABS:
      // cmd, pos,vel,acc
      if (getBusy() || getErrorID()) {
        return asynError;
      }
      errorCode = setExecute(0);
      if (errorCode) {
        return asynError;
      }
      errorCode = setCommand(ECMC_CMD_MOVEABS);
      if (errorCode) {
        return asynError;
      }
      errorCode = setCmdData(0);
      if (errorCode) {
        return asynError;
      }
      seq_.setTargetPos(controlData_.val0);
      seq_.setTargetVel(controlData_.val1);
      traj_->setAcc(controlData_.val2);
      traj_->setDec(controlData_.val2); //same as acc
      errorCode = setExecute(1);
      if (errorCode) {
        return asynError;
      }
      break;

    case ECMC_CMD_MOVEMODULO:
      return asynError;
      break;

    case ECMC_CMD_HOMING:

      // cmd, seqnbr,homepos,velhigh,vellow,acc
      if (getBusy() || getErrorID()) {
        return asynError;
      }
      errorCode = setExecute(0);
      if (errorCode) {
        return asynError;
      }
      errorCode = setCommand(ECMC_CMD_HOMING);
      if (errorCode) {
        return asynError;
      }
      cmddata = (int)controlData_.val0;
      errorCode = setCmdData(cmddata);
      if (errorCode) {
        return asynError;
      }
      seq_.setHomePosition(controlData_.val1);

      // Velo and acc not needed for set position
      if(cmddata != ECMC_SEQ_HOME_SET_POS) {
        seq_.setHomeVelTwordsCam(controlData_.val2);
        seq_.setHomeVelOffCam(controlData_.val3);
        traj_->setAcc(controlData_.val4);
        traj_->setDec(controlData_.val4); //same as acc
      }
      errorCode = setExecute(1);
      if (errorCode) {
        return asynError;
      }
      break;

    case ECMC_CMD_SUPERIMP:
      return asynError;
      break;

    case ECMC_CMD_GEAR:
      return asynError;
      break;

    case ECMC_CMD_STOP:
      errorCode = setExecute(0);
      if(errorCode) {
        return asynError;
      }
      break;

    case ECMC_CMD_SET_ENABLE:
      // cmd,  enable
      errorCode = setEnable(controlData_.val0>0);
      if(errorCode) {
        return asynError;
      }= 15
      break;

    case ECMC_CMD_SET_SOFTLIMBWD:
      errorCode = mon_->setSoftLimitBwd(controlData_.val0);
      if(errorCode) {
        return asynError;
      }
      break;

    case ECMC_CMD_SET_SOFTLIMFWD:
      errorCode = mon_->setSoftLimitFwd(controlData_.val0);
      if(errorCode) {
        return asynError;
      }

      break;

    case ECMC_CMD_SET_SOFTLIMBWD_ENA:
      errorCode = mon_->setEnableSoftLimitBwd(controlData_.val0>0);
      if(errorCode) {
        return asynError;
      }
      break;

    case ECMC_CMD_SET_SOFTLIMFWD_ENA:
      errorCode = mon_->setEnableSoftLimitFwd(controlData_.val0>0);
      if(errorCode) {
        return asynError;
      }
      break;
    
    case ECMC_CMD_SET_RESET:
      setReset(1);
      setReset(0);
      break;

    default:
      return asynError;
      break;

  }

  return asynSuccess;
}*/

// enum motionCommandTypes {             // Data order for motor record communications
//   ECMC_CMD_NOCMD              = -1,   
//   ECMC_CMD_JOG                = 0,    
//   ECMC_CMD_MOVEVEL            = 1,    // cmd, vel, acc
//   ECMC_CMD_MOVEREL            = 2,    // cmd, pos,vel,acc
//   ECMC_CMD_MOVEABS            = 3,    // cmd, pos,vel,acc
//   ECMC_CMD_MOVEMODULO         = 4,    
//   ECMC_CMD_HOMING             = 10,   // cmd, seqnbr,homepos,velhigh,vellow,acc
//   ECMC_CMD_SUPERIMP           = 20,   
//   ECMC_CMD_GEAR               = 30,   
//   ECMC_CMD_STOP               = 100,  // cmd, (Should have been 0 instead of 100)
//   ECMC_CMD_SET_ENABLE         = 101,  // cmd,  enable
//   ECMC_CMD_SET_SOFTLIMBWD     = 102,  // cmd,  soflimbwd
//   ECMC_CMD_SET_SOFTLIMFWD     = 103,  // cmd,  soflimfwd
//   ECMC_CMD_SET_SOFTLIMBWD_ENA = 104,  // cmd,  soflimbwdena
//   ECMC_CMD_SET_SOFTLIMFWD_ENA = 105,  // cmd,  soflimfwdena
// };