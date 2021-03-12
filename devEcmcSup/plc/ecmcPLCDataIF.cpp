/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPLCDataIF.cpp
*
*  Created on: Oct 4, 2018
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcPLCDataIF.h"

ecmcPLCDataIF::ecmcPLCDataIF(int plcIndex,
                             double plcSampleRateMs,
                             ecmcAxisBase *axis,
                             char *axisVarName,
                             ecmcAsynPortDriver *asynPortDriver) {
  errorReset();
  initVars();
  plcIndex_        = plcIndex;
  sampleRateMs_    = plcSampleRateMs; 
  axis_            = axis;
  varName_         = axisVarName;
  exprTkVarName_   = axisVarName;
  asynPortDriver_  = asynPortDriver;
  source_          = ECMC_RECORDER_SOURCE_AXIS;
  dataSourceAxis_  = parseAxisDataSource(axisVarName);

  if (dataSourceAxis_ == ECMC_AXIS_DATA_NONE) {
    LOGERR("%s/%s:%d: ERROR: Axis data Source Undefined  (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_PLC_AXIS_DATA_TYPE_ERROR);
  }
  asynWriteAllow_ = 0; 
  initAsyn();
}

ecmcPLCDataIF::ecmcPLCDataIF(int plcIndex,
                             double plcSampleRateMs,
                             ecmcDataStorage *ds,
                             char *dsVarName,
                             ecmcAsynPortDriver *asynPortDriver) {
  errorReset();
  initVars();
  plcIndex_        = plcIndex;
  sampleRateMs_    = plcSampleRateMs; 
  ds_              = ds;
  varName_         = dsVarName;
  exprTkVarName_   = dsVarName;
  asynPortDriver_  = asynPortDriver;
  source_          = ECMC_RECORDER_SOURCE_DATA_STORAGE;
  dataSourceDs_    = parseDataStorageDataSource(dsVarName);

  if (dataSourceDs_ == ECMC_DATA_STORAGE_DATA_NONE) {
    LOGERR("%s/%s:%d: ERROR: Axis data Source Undefined  (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_PLC_AXIS_DATA_TYPE_ERROR);
  }
  asynWriteAllow_ = 0; 
  initAsyn();
}

ecmcPLCDataIF::ecmcPLCDataIF(int plcIndex,
                             double plcSampleRateMs,
                             ecmcEc *ec,
                             char *ecVarName,
                             ecmcAsynPortDriver *asynPortDriver) {
  errorReset();
  initVars();
  plcIndex_        = plcIndex;
  sampleRateMs_    = plcSampleRateMs; 
  ec_              = ec;
  varName_         = ecVarName;
  exprTkVarName_   = ecVarName;
  asynPortDriver_  = asynPortDriver;
  source_          = ECMC_RECORDER_SOURCE_ETHERCAT;
  parseAndLinkEcDataSource(ecVarName);
  asynWriteAllow_  = 0; 
  initAsyn();
}

ecmcPLCDataIF::ecmcPLCDataIF(int plcIndex,
                             double plcSampleRateMs,
                             char *varName,
                             ecmcDataSourceType dataSource,
                             ecmcAsynPortDriver *asynPortDriver) {
  errorReset();
  initVars();  
  plcIndex_        = plcIndex; 
  sampleRateMs_    = plcSampleRateMs; 
  varName_         = varName;
  exprTkVarName_   = varName;
  asynPortDriver_  = asynPortDriver;
  source_          = dataSource;
  asynWriteAllow_  = 1; 
  initAsyn();  // Only static and global variables accessible from PLC
}

ecmcPLCDataIF::~ecmcPLCDataIF()
{}

void ecmcPLCDataIF::initVars() {
  plcIndex_       = 0;
  axis_           = 0;
  ds_             = 0;
  ec_             = 0;
  data_           = 0;
  varName_        = "";
  exprTkVarName_  = "";
  dataSourceAxis_ = ECMC_AXIS_DATA_NONE;
  dataSourceDs_   = ECMC_DATA_STORAGE_DATA_NONE;
  source_         = ECMC_RECORDER_SOURCE_NONE;
  readOnly_       = 0;
  asynPortDriver_ = 0;
  asynDataItem_   = 0;
  asynWriteAllow_ = 0; 
  isBool_         = 0;
  sampleRateMs_   = 0;
}

double& ecmcPLCDataIF::getDataRef() {
  return data_;
}

int ecmcPLCDataIF::read() {
  int errorCode = 0;

  switch (source_) {
  case ECMC_RECORDER_SOURCE_NONE:
    errorCode = ERROR_PLC_SOURCE_INVALID;
    break;

  case ECMC_RECORDER_SOURCE_ETHERCAT:
    errorCode = readEc();
    break;

  case ECMC_RECORDER_SOURCE_AXIS:
    errorCode = readAxis();
    break;

  case ECMC_RECORDER_SOURCE_STATIC_VAR:
    return 0;

    break;

  case ECMC_RECORDER_SOURCE_GLOBAL_VAR:
    return 0;

    break;

  case ECMC_RECORDER_SOURCE_DATA_STORAGE:
    errorCode = readDs();
    break;
  }

  dataRead_ = data_;
  return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
}

int ecmcPLCDataIF::write() {  
  
  // Only write if data changed between read and write
  if ((data_ == dataRead_) || readOnly_ || (isBool_ && ((data_>0) == (dataRead_>0)))) {
    return 0;
  }

  switch (source_) {
  case ECMC_RECORDER_SOURCE_NONE:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_SOURCE_INVALID);

    break;

  case ECMC_RECORDER_SOURCE_ETHERCAT:
    return writeEc();

    break;

  case ECMC_RECORDER_SOURCE_AXIS:
    return writeAxis();

    break;

  case ECMC_RECORDER_SOURCE_STATIC_VAR:
    return 0;

    break;

  case ECMC_RECORDER_SOURCE_GLOBAL_VAR:
    return 0;

    break;

  case ECMC_RECORDER_SOURCE_DATA_STORAGE:
    return writeDs();

    break;
  }

  return ERROR_PLC_SOURCE_INVALID;
}

int ecmcPLCDataIF::readEc() {
  double tempData = 0;
  int errorCode = readEcEntryValueDouble(ECMC_PLC_EC_ENTRY_INDEX, &tempData);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  data_ = tempData;

  return 0;
}

int ecmcPLCDataIF::writeEc() {
  
  int errorCode = writeEcEntryValueDouble(ECMC_PLC_EC_ENTRY_INDEX, data_);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }
  return 0;
}

int ecmcPLCDataIF::readDs() {
  if (ds_ == NULL) {
    return ERROR_PLC_DATA_STORAGE_NULL;
  }

  int errorCode   = 0;
  double tempData = 0;

  switch (dataSourceDs_) {
  case ECMC_DATA_STORAGE_DATA_NONE:
    data_ = 0;
    break;

  case ECMC_DATA_STORAGE_DATA_APPEND:
    data_ = NAN;
    break;

  case ECMC_DATA_STORAGE_DATA_SIZE:
    data_ = ds_->getSize();
    break;

  case ECMC_DATA_STORAGE_DATA_INDEX:
    data_ = ds_->getCurrentIndex();
    break;

  case ECMC_DATA_STORAGE_DATA_ERROR:
    data_ = ds_->getErrorID();
    break;

  case ECMC_DATA_STORAGE_DATA_DATA:
    errorCode = ds_->getDataElement(&tempData);

    if (!errorCode) {
      data_ = tempData;
    }
    break;

  case ECMC_DATA_STORAGE_DATA_CLEAR:
    data_ = NAN;
    break;

  case ECMC_DATA_STORAGE_DATA_FULL:
    data_ = static_cast<double>(ds_->isStorageFull());
    break;

  default:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_DATA_STORGAE_DATA_TYPE_ERROR);

    break;
  }

  return 0;
}

int ecmcPLCDataIF::writeDs() {
  if (ds_ == NULL) {
    return ERROR_PLC_DATA_STORAGE_NULL;
  }

  switch (dataSourceDs_) {
  case ECMC_DATA_STORAGE_DATA_NONE:
    data_ = 0;
    break;

  case ECMC_DATA_STORAGE_DATA_APPEND:
    // Check for NAN (only append new value)
    if (data_ == data_) {
      ds_->appendData(data_);
    }
    break;

  case ECMC_DATA_STORAGE_DATA_SIZE:
    ds_->setBufferSize(data_);
    break;

  case ECMC_DATA_STORAGE_DATA_INDEX:
    ds_->setCurrentPosition(data_);
    break;

  case ECMC_DATA_STORAGE_DATA_ERROR:
    ;
    break;

  case ECMC_DATA_STORAGE_DATA_DATA:
    ds_->setDataElement(data_);
    break;

  case ECMC_DATA_STORAGE_DATA_CLEAR:
    // Check for NAN (only append new value)
    if ((data_ == data_) && (data_ > 0)) {
      ds_->clearBuffer();
    }
    break;

  case ECMC_DATA_STORAGE_DATA_FULL:
    ;
    break;

  default:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_DATA_STORGAE_DATA_TYPE_ERROR);

    break;
  }
  return 0;
}

int ecmcPLCDataIF::readAxis() {
  if (axis_ == NULL) {
    return ERROR_PLC_AXIS_NULL;
  }

  if (axis_->getTraj() == NULL) {
    return ERROR_PLC_TRAJ_NULL;
  }

  ecmcAxisStatusType *axisData = axis_->getDebugInfoDataPointer();

  switch (dataSourceAxis_) {
  case ECMC_AXIS_DATA_NONE:
    data_ = 0;
    break;

  case ECMC_AXIS_DATA_AXIS_ID:
    data_ = static_cast<double>(axisData->axisID);
    break;

  case ECMC_AXIS_DATA_POS_SET:
    data_ = axisData->onChangeData.positionSetpoint;
    break;

  case ECMC_AXIS_DATA_POS_ACT:
    data_ = axisData->onChangeData.positionActual;
    break;

  case ECMC_AXIS_DATA_CNTRL_ERROR:
    data_ = axisData->onChangeData.cntrlError;
    break;

  case ECMC_AXIS_DATA_POS_TARGET:
    data_ = axisData->onChangeData.positionTarget;
    break;

  case ECMC_AXIS_DATA_POS_ERROR:
    data_ = axisData->onChangeData.positionError;
    break;

  case ECMC_AXIS_DATA_POS_RAW:
    // Risc of data loss
    data_ = static_cast<double>(axisData->onChangeData.positionRaw);
    break;

  case ECMC_AXIS_DATA_CNTRL_OUT:
    data_ = axisData->onChangeData.cntrlOutput;
    break;

  case ECMC_AXIS_DATA_VEL_SET:
    data_ = axisData->onChangeData.velocitySetpoint;
    break;

  case ECMC_AXIS_DATA_VEL_ACT:
    data_ = axisData->onChangeData.velocityActual;
    break;

  case ECMC_AXIS_DATA_VEL_SET_FF_RAW:
    data_ = axisData->onChangeData.velocityFFRaw;
    break;

  case ECMC_AXIS_DATA_VEL_SET_RAW:
    data_ = static_cast<double>(axisData->onChangeData.velocitySetpointRaw);
    break;

  case ECMC_AXIS_DATA_CYCLE_COUNTER:
    data_ = static_cast<double>(axisData->cycleCounter);
    break;

  case ECMC_AXIS_DATA_ERROR:
    data_ = static_cast<double>(axisData->onChangeData.error);
    break;

  case ECMC_AXIS_DATA_COMMAND:
    data_ = static_cast<double>(axisData->onChangeData.command);
    break;

  case ECMC_AXIS_DATA_CMD_DATA:
    data_ = static_cast<double>(axisData->onChangeData.cmdData);
    break;

  case ECMC_AXIS_DATA_SEQ_STATE:
    data_ = static_cast<double>(axisData->onChangeData.seqState);
    break;

  case ECMC_AXIS_DATA_INTERLOCK_TYPE:
    data_ = static_cast<double>(axisData->onChangeData.trajInterlock == 0);
    break;

  case ECMC_AXIS_DATA_TRAJ_SOURCE:
    data_ = static_cast<double>(axisData->onChangeData.statusWd.trajsource);
    break;

  case ECMC_AXIS_DATA_ENC_SOURCE:
    data_ = static_cast<double>(axisData->onChangeData.statusWd.encsource);
    break;

  case ECMC_AXIS_DATA_ENABLE:

    data_ = static_cast<double>(axisData->onChangeData.statusWd.enable);
    break;

  case ECMC_AXIS_DATA_ENABLED:
    data_ = static_cast<double>(axisData->onChangeData.statusWd.enabled);
    break;

  case ECMC_AXIS_DATA_EXECUTE:
    data_ = static_cast<double>(axisData->onChangeData.statusWd.execute);
    break;

  case ECMC_AXIS_DATA_BUSY:
    data_ = static_cast<double>(axisData->onChangeData.statusWd.busy);
    break;

  case ECMC_AXIS_DATA_AT_TARGET:
    data_ = static_cast<double>(axisData->onChangeData.statusWd.attarget);
    break;

  case ECMC_AXIS_DATA_HOMED:
    data_ = static_cast<double>(axisData->onChangeData.statusWd.homed);
    break;

  case ECMC_AXIS_DATA_LIMIT_BWD:
    data_ = static_cast<double>(axisData->onChangeData.statusWd.limitbwd);
    break;

  case ECMC_AXIS_DATA_LIMIT_FWD:
    data_ = static_cast<double>(axisData->onChangeData.statusWd.limitfwd);
    break;

  case ECMC_AXIS_DATA_HOME_SWITCH:
    data_ = static_cast<double>(axisData->onChangeData.statusWd.homeswitch);
    break;

  case ECMC_AXIS_DATA_RESET:
    data_ = 0;
    break;

  case ECMC_AXIS_DATA_VEL_TARGET_SET:
    data_ = axis_->getSeq()->getTargetVel();
    break;

  case ECMC_AXIS_DATA_ACC_TARGET_SET:
    data_ = axis_->getTraj()->getAcc();
    break;

  case ECMC_AXIS_DATA_DEC_TARGET_SET:
    data_ = axis_->getTraj()->getDec();
    break;

  case ECMC_AXIS_DATA_SOFT_LIMIT_BWD:
    data_ = axis_->getMon()->getSoftLimitBwd();
    break;

  case ECMC_AXIS_DATA_SOFT_LIMIT_FWD:
    data_ = axis_->getMon()->getSoftLimitFwd();
    break;

  case ECMC_AXIS_DATA_SOFT_LIMIT_BWD_ENABLE:
    data_ = static_cast<double>(axis_->getMon()->getEnableSoftLimitBwd());
    break;

  case ECMC_AXIS_DATA_SOFT_LIMIT_FWD_ENABLE:
    data_ = static_cast<double>(axis_->getMon()->getEnableSoftLimitFwd());
    break;

  case ECMC_AXIS_DATA_TRAJ_DIRECTION:

    switch (axis_->getAxisSetDirection()) {
    case ECMC_DIR_BACKWARD:
      data_ = -1;
      break;

    case ECMC_DIR_FORWARD:
      data_ = 1;
      break;

    case ECMC_DIR_STANDSTILL:
      data_ = 0;
      break;
    }
    break;

  case ECMC_AXIS_DATA_ENC_HOMEPOS:
    data_ = axis_->getSeq()->getHomePosition();
    break;

  case ECMC_AXIS_DATA_BLOCK_COM:
    data_ = axis_->getBlockExtCom();
    break;

  case ECMC_AXIS_DATA_INTERLOCK_BWD_TYPE:
    data_ = static_cast<double>(axisData->onChangeData.statusWd.sumilockbwd == 0);
    break;

  case ECMC_AXIS_DATA_INTERLOCK_FWD_TYPE:
    data_ = static_cast<double>(axisData->onChangeData.statusWd.sumilockfwd == 0);
    break;

  case ECMC_AXIS_DATA_ALLOW_PLC_WRITE:
    data_ = static_cast<double>(axis_->getAllowCmdFromPLC());
    break;

  case ECMC_AXIS_DATA_POS_SET_EXTERNAL:
    data_ = axis_->getExtSetPos();
    break;

  case ECMC_AXIS_DATA_POS_ACT_EXTERNAL:
    data_ = axis_->getExtActPos();
    break;

  default:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_AXIS_DATA_TYPE_ERROR);

    break;
  }

  return 0;
}

int ecmcPLCDataIF::writeAxis() {
  
  if (axis_ == NULL) {
    return ERROR_PLC_AXIS_NULL;
  }

  // Write from PLC to Axis allowed?
  if(!axis_->getAllowCmdFromPLC() && 
     (dataSourceAxis_ != ECMC_AXIS_DATA_ALLOW_PLC_WRITE)) {
    return 0;
  } 

  if (axis_->getTraj() == NULL) {
    return ERROR_PLC_TRAJ_NULL;
  }

  if (axis_->getMon() == NULL) {
    return ERROR_PLC_MON_NULL;
  }

  // Only write commands if changed
  switch (dataSourceAxis_) {
  case ECMC_AXIS_DATA_NONE:
    return 0;

    break;

  case ECMC_AXIS_DATA_AXIS_ID:
    return 0;

    break;

  case ECMC_AXIS_DATA_POS_SET:    
    return axis_->setExtSetPos(data_);;

    break;

  case ECMC_AXIS_DATA_POS_ACT:
    return axis_->setExtActPos(data_);

    break;

  case ECMC_AXIS_DATA_CNTRL_ERROR:
    return 0;

    break;

  case ECMC_AXIS_DATA_POS_TARGET:
    axis_->getSeq()->setTargetPos(data_);
    return 0;

    break;

  case ECMC_AXIS_DATA_POS_ERROR:
    return 0;

    break;

  case ECMC_AXIS_DATA_POS_RAW:
    return 0;

    break;

  case ECMC_AXIS_DATA_CNTRL_OUT:
    return 0;

    break;

  case ECMC_AXIS_DATA_VEL_SET:
    return 0;

    break;

  case ECMC_AXIS_DATA_VEL_ACT:
    return 0;

    break;

  case ECMC_AXIS_DATA_VEL_SET_FF_RAW:
    return 0;

    break;

  case ECMC_AXIS_DATA_VEL_SET_RAW:
    return 0;

    break;

  case ECMC_AXIS_DATA_CYCLE_COUNTER:
    return 0;

    break;

  case ECMC_AXIS_DATA_ERROR:
    return 0;

    break;

  case ECMC_AXIS_DATA_COMMAND:
    return axis_->setCommand((motionCommandTypes)data_);

    break;

  case ECMC_AXIS_DATA_CMD_DATA:
    return axis_->setCmdData(static_cast<int>(data_));

    break;

  case ECMC_AXIS_DATA_SEQ_STATE:
    return 0;

    break;

  case ECMC_AXIS_DATA_INTERLOCK_TYPE:
    return axis_->getMon()->setPLCInterlock(data_ == 0,ECMC_PLC_INTERLOCK_DIR_BOTH);

    break;

  case ECMC_AXIS_DATA_TRAJ_SOURCE:
    if (data_>=1) {
      return axis_->setTrajDataSourceType(ECMC_DATA_SOURCE_EXTERNAL);
    }
    else {
      return axis_->setTrajDataSourceType(ECMC_DATA_SOURCE_INTERNAL);
    }

    break;

  case ECMC_AXIS_DATA_ENC_SOURCE:
    if (data_>=1) {
      return axis_->setEncDataSourceType(ECMC_DATA_SOURCE_EXTERNAL);
    }
    else {
      return axis_->setEncDataSourceType(ECMC_DATA_SOURCE_INTERNAL);
    }

    break;

  case ECMC_AXIS_DATA_ENABLE:
    return axis_->setEnable(static_cast<bool>(data_));

    break;

  case ECMC_AXIS_DATA_ENABLED:
    return 0;

    break;

  case ECMC_AXIS_DATA_EXECUTE:
    return axis_->setExecute(static_cast<bool>(data_));

    break;

  case ECMC_AXIS_DATA_BUSY:
    return 0;

    break;

  case ECMC_AXIS_DATA_AT_TARGET:
    return 0;

    break;

  case ECMC_AXIS_DATA_HOMED:
    axis_->setAxisHomed(static_cast<bool>(data_));
    return 0;

    break;

  case ECMC_AXIS_DATA_LIMIT_BWD:
    return 0;

    break;

  case ECMC_AXIS_DATA_LIMIT_FWD:
    return 0;

    break;

  case ECMC_AXIS_DATA_HOME_SWITCH:
    return 0;

    break;

  case ECMC_AXIS_DATA_RESET:

    if (data_) {
      axis_->errorReset();
    }
    return 0;

    break;

  case ECMC_AXIS_DATA_VEL_TARGET_SET:
    axis_->getSeq()->setTargetVel(data_);
    break;

  case ECMC_AXIS_DATA_ACC_TARGET_SET:
    axis_->getTraj()->setAcc(data_);
    break;

  case ECMC_AXIS_DATA_DEC_TARGET_SET:
    axis_->getTraj()->setDec(data_);
    break;

  case ECMC_AXIS_DATA_SOFT_LIMIT_BWD:
    axis_->getMon()->setSoftLimitBwd(data_);
    break;

  case ECMC_AXIS_DATA_SOFT_LIMIT_FWD:
    axis_->getMon()->setSoftLimitFwd(data_);
    break;

  case ECMC_AXIS_DATA_SOFT_LIMIT_BWD_ENABLE:
    axis_->getMon()->setEnableSoftLimitBwd(static_cast<bool>(data_));
    break;

  case ECMC_AXIS_DATA_SOFT_LIMIT_FWD_ENABLE:
    axis_->getMon()->setEnableSoftLimitFwd(static_cast<bool>(data_));
    break;

  case ECMC_AXIS_DATA_TRAJ_DIRECTION:
    return 0;

    break;

  case ECMC_AXIS_DATA_ENC_HOMEPOS:
    axis_->getSeq()->setHomePosition(data_);
    break;

  case ECMC_AXIS_DATA_BLOCK_COM:
    axis_->setBlockExtCom(data_);
    break;

  case ECMC_AXIS_DATA_INTERLOCK_BWD_TYPE:
    return axis_->getMon()->setPLCInterlock(data_ == 0,ECMC_PLC_INTERLOCK_DIR_BWD);
    break;

  case ECMC_AXIS_DATA_INTERLOCK_FWD_TYPE:
    return axis_->getMon()->setPLCInterlock(data_ == 0,ECMC_PLC_INTERLOCK_DIR_FWD);
    break;

  case ECMC_AXIS_DATA_ALLOW_PLC_WRITE:
    return axis_->setAllowCmdFromPLC(data_>=1);
    break;

  case ECMC_AXIS_DATA_POS_SET_EXTERNAL:
    return axis_->setExtSetPos(data_);
    break;

  case ECMC_AXIS_DATA_POS_ACT_EXTERNAL:
    return axis_->setExtActPos(data_);
    break;

  default:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_AXIS_DATA_TYPE_ERROR);

    break;
  }
  return 0;
}

ecmcDataStorageType ecmcPLCDataIF::parseDataStorageDataSource(
  char *axisDataSource) {
  char *varName;

  varName = strstr(axisDataSource, ECMC_PLC_DATA_STORAGE_STR);

  if (!varName) {
    return ECMC_DATA_STORAGE_DATA_NONE;
  }
  varName = strstr(axisDataSource, ".");

  if (!varName) {
    return ECMC_DATA_STORAGE_DATA_NONE;
  }
  varName++;

  int npos = strcmp(varName, ECMC_DATA_STORAGE_DATA_APPEND_STR);

  if (npos == 0) {
    return ECMC_DATA_STORAGE_DATA_APPEND;
  }

  npos = strcmp(varName, ECMC_DATA_STORAGE_DATA_INDEX_STR);

  if (npos == 0) {
    return ECMC_DATA_STORAGE_DATA_INDEX;
  }

  npos = strcmp(varName, ECMC_DATA_STORAGE_DATA_ERROR_STR);

  if (npos == 0) {
    return ECMC_DATA_STORAGE_DATA_ERROR;
  }

  npos = strcmp(varName, ECMC_DATA_STORAGE_DATA_SIZE_STR);

  if (npos == 0) {
    return ECMC_DATA_STORAGE_DATA_SIZE;
  }

  npos = strcmp(varName, ECMC_DATA_STORAGE_DATA_CLEAR_STR);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_DATA_STORAGE_DATA_CLEAR;
  }

  npos = strcmp(varName, ECMC_DATA_STORAGE_DATA_DATA_STR);

  if (npos == 0) {
    return ECMC_DATA_STORAGE_DATA_DATA;
  }

  npos = strcmp(varName, ECMC_DATA_STORAGE_DATA_FULL_STR);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_DATA_STORAGE_DATA_FULL;
  }

  return ECMC_DATA_STORAGE_DATA_NONE;
}

ecmcAxisDataType ecmcPLCDataIF::parseAxisDataSource(char *axisDataSource) {
  char *varName;

  varName = strstr(axisDataSource, ECMC_AX_STR);

  if (!varName) {
    return ECMC_AXIS_DATA_NONE;
  }
  varName = strstr(axisDataSource, ".");

  if (!varName) {
    return ECMC_AXIS_DATA_NONE;
  }
  varName++;

  int npos = strcmp(varName, ECMC_AXIS_DATA_STR_AXIS_ID);

  if (npos == 0) {
    return ECMC_AXIS_DATA_AXIS_ID;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_POS_SET);

  if (npos == 0) {
    return ECMC_AXIS_DATA_POS_SET;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_POS_ACT);

  if (npos == 0) {
    return ECMC_AXIS_DATA_POS_ACT;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_POS_TARGET);

  if (npos == 0) {
    return ECMC_AXIS_DATA_POS_TARGET;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_POS_ERROR);

  if (npos == 0) {
    return ECMC_AXIS_DATA_POS_ERROR;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_POS_RAW);

  if (npos == 0) {
    return ECMC_AXIS_DATA_POS_RAW;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_CNTRL_OUT);

  if (npos == 0) {
    return ECMC_AXIS_DATA_CNTRL_OUT;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_VEL_SET);

  if (npos == 0) {
    return ECMC_AXIS_DATA_VEL_SET;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_VEL_ACT);

  if (npos == 0) {
    return ECMC_AXIS_DATA_VEL_ACT;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_VEL_SET_FF_RAW);

  if (npos == 0) {
    return ECMC_AXIS_DATA_VEL_SET_FF_RAW;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_VEL_SET_RAW);

  if (npos == 0) {
    return ECMC_AXIS_DATA_VEL_SET_RAW;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_CYCLE_COUNTER);

  if (npos == 0) {
    return ECMC_AXIS_DATA_CYCLE_COUNTER;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_ERROR);

  if (npos == 0) {
    return ECMC_AXIS_DATA_ERROR;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_COMMAND);

  if (npos == 0) {
    return ECMC_AXIS_DATA_COMMAND;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_CMD_DATA);

  if (npos == 0) {
    return ECMC_AXIS_DATA_CMD_DATA;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_SEQ_STATE);

  if (npos == 0) {
    return ECMC_AXIS_DATA_SEQ_STATE;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_INTERLOCK_TYPE);

  if (npos == 0) {
    return ECMC_AXIS_DATA_INTERLOCK_TYPE;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_TRAJ_SOURCE);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_TRAJ_SOURCE;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_ENC_SOURCE);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_ENC_SOURCE;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_ENABLE);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_ENABLE;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_ENABLED);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_ENABLED;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_EXECUTE);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_EXECUTE;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_BUSY);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_BUSY;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_AT_TARGET);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_AT_TARGET;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_HOMED);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_HOMED;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_LIMIT_BWD);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_LIMIT_BWD;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_LIMIT_FWD);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_LIMIT_FWD;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_HOME_SWITCH);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_HOME_SWITCH;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_RESET);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_RESET;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_VEL_TARGET_SET);

  if (npos == 0) {    
    return ECMC_AXIS_DATA_VEL_TARGET_SET;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_ACC_TARGET_SET);

  if (npos == 0) {
    return ECMC_AXIS_DATA_ACC_TARGET_SET;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_DEC_TARGET_SET);

  if (npos == 0) {
    return ECMC_AXIS_DATA_DEC_TARGET_SET;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_SOFT_LIMIT_BWD);

  if (npos == 0) {
    return ECMC_AXIS_DATA_SOFT_LIMIT_BWD;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_SOFT_LIMIT_FWD);

  if (npos == 0) {
    return ECMC_AXIS_DATA_SOFT_LIMIT_FWD;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_SOFT_LIMIT_BWD_ENABLE);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_SOFT_LIMIT_BWD_ENABLE;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_SOFT_LIMIT_FWD_ENABLE);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_SOFT_LIMIT_FWD_ENABLE;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_TRAJ_DIRECTION);

  if (npos == 0) {
    return ECMC_AXIS_DATA_TRAJ_DIRECTION;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_ENC_HOMEPOS);

  if (npos == 0) {
    return ECMC_AXIS_DATA_ENC_HOMEPOS;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_BLOCK_COM);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_BLOCK_COM;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_INTERLOCK_FWD_TYPE);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_INTERLOCK_FWD_TYPE;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_INTERLOCK_BWD_TYPE);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_INTERLOCK_BWD_TYPE;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_ALLOW_PLC_CMD);

  if (npos == 0) {
    isBool_ = 1;
    return ECMC_AXIS_DATA_ALLOW_PLC_WRITE;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_POS_SET_EXTERNAL);

  if (npos == 0) {    
    return ECMC_AXIS_DATA_POS_SET_EXTERNAL;
  }

  npos = strcmp(varName, ECMC_AXIS_DATA_STR_POS_ACT_EXTERNAL);

  if (npos == 0) {    
    return ECMC_AXIS_DATA_POS_ACT_EXTERNAL;
  }

  return ECMC_AXIS_DATA_NONE;
}

int ecmcPLCDataIF::parseAndLinkEcDataSource(char *ecDataSource) {
  int  masterId = 0;
  int  slaveId  = 0;
  int  bitId    = 0;
  char alias[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  errorCode =
    parseEcPath(ecDataSource, &masterId, &slaveId, alias, &bitId);

  if (errorCode) {
    LOGERR("%s/%s:%d: ERROR: Parse %s failed (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ecDataSource,
           errorCode);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  if (ec_->getMasterIndex() != masterId) {
    LOGERR("%s/%s:%d: ERROR: Master %s not configured (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ecDataSource,
           ERROR_PLC_EC_MASTER_INVALID);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_EC_MASTER_INVALID);
  }

  ecmcEcSlave *slave = NULL;

  if (slaveId >= 0) {
    slave = ec_->findSlave(slaveId);
  } else {
    // Change exprtk var name ('-' not allowed in var name)
    std::stringstream ss;

    if (bitId >= 0) {
      ss << ECMC_EC_STR << masterId << "." ECMC_DUMMY_SLAVE_STR << -slaveId <<
      "." << alias << "." << bitId;
    } else {
      ss << ECMC_EC_STR << masterId << "." ECMC_DUMMY_SLAVE_STR << -slaveId <<
      "." << alias;
    }
    exprTkVarName_ = ss.str();
    slave          = ec_->getSlave(slaveId);
  }

  if (slave == NULL) {
    LOGERR("%s/%s:%d: ERROR: Slave %s not configured (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ecDataSource,
           ERROR_PLC_EC_SLAVE_NULL);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_PLC_EC_SLAVE_NULL);
  }

  std::string sEntryID = alias;

  ecmcEcEntry *entry = slave->findEntry(sEntryID);

  if (entry == NULL) {
    LOGERR("%s/%s:%d: ERROR: Entry %s not configured (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ecDataSource,
           ERROR_PLC_EC_ENTRY_NULL);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_PLC_EC_ENTRY_NULL);
  }

  errorCode = setEntryAtIndex(entry, ECMC_PLC_EC_ENTRY_INDEX, bitId);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  errorCode = validateEntry(ECMC_PLC_EC_ENTRY_INDEX);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  // Print warning if data type is "NONE" , "U64" or "S64"
  ecmcEcDataType dt=getEntryDataType(ECMC_PLC_EC_ENTRY_INDEX);

  if(dt==ECMC_EC_NONE || dt==ECMC_EC_U64 || dt==ECMC_EC_S64 ) {
    LOGERR("%s/%s:%d: WARNING: Entry %s is of type S64, U64 or undefined. PLC values are doubles and might not be able to represent the ethercat value correct.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ecDataSource);
  }
  
  return 0;
}

int ecmcPLCDataIF::parseEcPath(char *ecPath,
                               int  *master,
                               int  *slave,
                               char *alias,
                               int  *bit) {
  int masterId = 0;
  int slaveId  = 0;
  int bitId    = 0;
  int nvals    = 0;

  nvals = sscanf(ecPath,
                 ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d." ECMC_PLC_EC_ALIAS_FORMAT ".%d",
                 &masterId,
                 &slaveId,
                 alias,
                 &bitId);

  if (nvals == 4) {
    *master = masterId;
    *slave  = slaveId;
    *bit    = bitId;
    return 0;
  }

  nvals = sscanf(ecPath,
                 ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d." ECMC_PLC_EC_ALIAS_FORMAT,
                 &masterId,
                 &slaveId,
                 alias);

  if (nvals == 3) {
    *master = masterId;
    *slave  = slaveId;
    *bit    = -1;
    return 0;
  }
  return ERROR_PLC_EC_VAR_NAME_INVALID;
}

const char * ecmcPLCDataIF::getVarName() {
  return varName_.c_str();
}

const char * ecmcPLCDataIF::getExprTkVarName() {
  return exprTkVarName_.c_str();
}

int ecmcPLCDataIF::validate() {
  if (getErrorID()) {
    return getErrorID();
  }

  switch (source_) {
  case ECMC_RECORDER_SOURCE_NONE:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_SOURCE_INVALID);

    break;

  case ECMC_RECORDER_SOURCE_ETHERCAT:

    if (!ec_) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_PLC_EC_NULL);
    } else {
      return 0;
    }
    break;

  case ECMC_RECORDER_SOURCE_AXIS:

    if (!axis_) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_PLC_AXIS_NULL);
    } else {
      return 0;
    }
    break;

  case ECMC_RECORDER_SOURCE_STATIC_VAR:
    return 0;

    break;

  case ECMC_RECORDER_SOURCE_GLOBAL_VAR:
    return 0;

    break;

  case ECMC_RECORDER_SOURCE_DATA_STORAGE:

    if (!ds_) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_PLC_DATA_STORAGE_NULL);
    } else {
      return 0;
    }
    break;
  }
  return ERROR_PLC_SOURCE_INVALID;
}

double ecmcPLCDataIF::getData() {
  return data_;
}

void ecmcPLCDataIF::setData(double data) {
  data_ = data;  
}

int ecmcPLCDataIF::setReadOnly(int readOnly) {
  readOnly_ = readOnly;
  return 0;
}

int ecmcPLCDataIF::initAsyn() {
  
  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];  
  char *name = buffer;
  unsigned int charCount = 0;
  if(plcIndex_>=0) { //local variable (plc index)
    // "plc%d.%s"
    charCount = snprintf(buffer,
                         sizeof(buffer),
                         ECMC_PLCS_DATA_STR "." ECMC_PLC_DATA_STR "%d.%s",
                         plcIndex_,
                         varName_.c_str());
  } else { //global variable (no plc index)
    // "%s"
    charCount = snprintf(buffer,
                         sizeof(buffer),
                         ECMC_PLCS_DATA_STR ".%s",
                         varName_.c_str());
  }
 
  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_ASYN_DATA_BUFFER_TO_SMALL);
      return setErrorID(ERROR_ASYN_DATA_BUFFER_TO_SMALL);
  }

  name = buffer;
  asynDataItem_ = asynPortDriver_->addNewAvailParam(name,
                                                    asynParamFloat64,
                                                    (uint8_t *)&data_,
                                                    sizeof(data_),
                                                    ECMC_EC_F64,
                                                    sampleRateMs_,
                                                    0);
    if(!asynDataItem_) {
      LOGERR(
        "%s/%s:%d: ERROR: Add create default parameter for %s failed (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        name,
        ERROR_ASYN_CREATE_PARAM_FAIL);
        return setErrorID(ERROR_ASYN_CREATE_PARAM_FAIL);
    }

  asynDataItem_->setAllowWriteToEcmc(asynWriteAllow_);    
  updateAsyn(1);
  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);
  return 0;
}

int ecmcPLCDataIF::updateAsyn(int force) {  
  if(asynDataItem_) {
    asynDataItem_->refreshParamRT(force);    
  }
  return 0;
}
  
