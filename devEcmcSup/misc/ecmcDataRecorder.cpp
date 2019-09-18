/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcDataRecorder.cpp
*
*  Created on: Jun 1, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcDataRecorder.h"

ecmcDataRecorder::ecmcDataRecorder(int index) : ecmcEcEntryLink() {
  index_ = index;
  PRINT_ERROR_PATH("dataRecorder[%d].error", index_);
  initVars();
  setInStartupPhase(1);
  printCurrentState();
}

ecmcDataRecorder::~ecmcDataRecorder()
{}

void ecmcDataRecorder::printCurrentState() {
  LOGINFO11("%s/%s:%d: dataRecorder[%d]=new;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            index_);
  LOGINFO11("%s/%s:%d: dataRecorder[%d].enable=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            index_,
            enable_);
  LOGINFO11("%s/%s:%d: dataRecorder[%d].buffer=null;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            index_);
  printDataSource();
  printAxisDataSource();
}

void ecmcDataRecorder::printDataSource() {
  switch (dataSource_) {
  case  ECMC_RECORDER_SOURCE_NONE:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].source=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_RECORDER_SOURCE_NONE");
    break;

  case ECMC_RECORDER_SOURCE_ETHERCAT:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].source=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_RECORDER_SOURCE_ETHERCAT");
    break;

  case ECMC_RECORDER_SOURCE_AXIS:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].source=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_RECORDER_SOURCE_AXIS");
    break;

  default:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].source=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              dataSource_);
    break;
  }
}

void ecmcDataRecorder::printAxisDataSource() {
  switch (axisDataTypeToRecord_) {
  case ECMC_AXIS_DATA_NONE:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_NONE");
    break;

  case ECMC_AXIS_DATA_AXIS_ID:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_AXIS_ID");
    break;

  case ECMC_AXIS_DATA_POS_SET:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_POS_SET");
    break;

  case ECMC_AXIS_DATA_POS_ACT:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_POS_ACT");
    break;

  case ECMC_AXIS_DATA_CNTRL_ERROR:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_CNTRL_ERROR");
    break;

  case ECMC_AXIS_DATA_POS_TARGET:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_POS_TARGET");
    break;

  case ECMC_AXIS_DATA_POS_ERROR:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_POS_ERROR");
    break;

  case ECMC_AXIS_DATA_POS_RAW:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_POS_RAW");
    break;

  case ECMC_AXIS_DATA_CNTRL_OUT:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_CNTRL_OUT");
    break;

  case ECMC_AXIS_DATA_VEL_SET:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_VEL_SET");
    break;

  case ECMC_AXIS_DATA_VEL_ACT:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_VEL_ACT");
    break;

  case ECMC_AXIS_DATA_VEL_SET_FF_RAW:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_VEL_SET_FF_RAW");
    break;

  case ECMC_AXIS_DATA_VEL_SET_RAW:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_VEL_SET_RAW");
    break;

  case ECMC_AXIS_DATA_CYCLE_COUNTER:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_CYCLE_COUNTER");
    break;

  case ECMC_AXIS_DATA_ERROR:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_ERROR");
    break;

  case ECMC_AXIS_DATA_COMMAND:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_COMMAND");
    break;

  case ECMC_AXIS_DATA_CMD_DATA:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_CMD_DATA");
    break;

  case ECMC_AXIS_DATA_SEQ_STATE:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_SEQ_STATE");
    break;

  case ECMC_AXIS_DATA_INTERLOCK_TYPE:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_INTERLOCK_TYPE");
    break;

  case ECMC_AXIS_DATA_TRAJ_SOURCE:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_TRAJ_SOURCE");
    break;

  case ECMC_AXIS_DATA_ENC_SOURCE:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_ENC_SOURCE");
    break;

  case ECMC_AXIS_DATA_ENABLE:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_ENABLE");
    break;

  case ECMC_AXIS_DATA_ENABLED:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_ENABLED");
    break;

  case ECMC_AXIS_DATA_EXECUTE:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_EXECUTE");
    break;

  case ECMC_AXIS_DATA_BUSY:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_BUSY");
    break;

  case ECMC_AXIS_DATA_AT_TARGET:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_AT_TARGET");
    break;

  case ECMC_AXIS_DATA_HOMED:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_HOMED");
    break;

  case ECMC_AXIS_DATA_LIMIT_BWD:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_LIMIT_BWD");
    break;

  case ECMC_AXIS_DATA_LIMIT_FWD:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_LIMIT_FWD");
    break;

  case ECMC_AXIS_DATA_HOME_SWITCH:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              "ECMC_AXIS_DATA_HOME_SWITCH");
    break;

  default:
    LOGINFO11("%s/%s:%d: dataRecorder[%d].axisDataType=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              axisDataTypeToRecord_);
    break;
  }
}

void ecmcDataRecorder::initVars() {
  errorReset();
  dataBuffer_           = NULL;
  data_                 = 0;
  inStartupPhase_       = 1;
  enable_               = false;
  axisData_             = NULL;
  axisDataTypeToRecord_ = ECMC_AXIS_DATA_NONE;
  dataSource_           = ECMC_RECORDER_SOURCE_NONE;
}

int ecmcDataRecorder::setDataStorage(ecmcDataStorage *buffer) {
  dataBuffer_ = buffer;

  if (!dataBuffer_) {
    LOGINFO11("%s/%s:%d: dataRecorder[%d].buffer=dataStorage[%d];\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              dataBuffer_->getIndex());
  }
  return 0;
}

int ecmcDataRecorder::validate() {
  switch (dataSource_) {
  case  ECMC_RECORDER_SOURCE_NONE:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_RECORDER_NO_DATA_SOURCE_CHOOSEN);

    break;

  case ECMC_RECORDER_SOURCE_ETHERCAT:

    if (validateEntry(0)) {  // Data
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_DATA_RECORDER_DATA_ECENTRY_NULL);
    }
    break;

  case ECMC_RECORDER_SOURCE_AXIS:

    if (axisDataTypeToRecord_ == ECMC_AXIS_DATA_NONE) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_DATA_RECORDER_AXIS_DATA_TYPE_NOT_CHOOSEN);
    }
    break;

  default:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_RECORDER_NO_DATA_SOURCE_CHOOSEN);

    break;
  }

  if (dataBuffer_ == NULL) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_RECORDER_BUFFER_NULL);
  }

  return setErrorID(0);
}

int ecmcDataRecorder::setEnable(int enable) {
  if (enable_ != enable) {
    LOGINFO11("%s/%s:%d: dataRecorder[%d].enable=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              index_,
              enable);
  }
  enable_ = enable;
  validate();
  return 0;
}

int ecmcDataRecorder::getEnabled(int *enabled) {
  *enabled = enable_;
  return 0;
}

void ecmcDataRecorder::printStatus() {
  LOGINFO11("%s/%s:%d: INFO: Data Recorder %d. Data: %lf, Error: 0x%x.\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            index_,
            static_cast<double>(data_),
            getErrorID());
}

void ecmcDataRecorder::setInStartupPhase(bool startup) {
  inStartupPhase_ = startup;
}

int ecmcDataRecorder::executeEvent(int masterOK) {
  if (!masterOK) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_RECORDER_HARDWARE_STATUS_NOT_OK);
  }

  if (inStartupPhase_) {
    // Auto reset hardware error
    if (getErrorID() == ERROR_DATA_RECORDER_HARDWARE_STATUS_NOT_OK) {
      setErrorID(0);
    }
    setInStartupPhase(false);
  }

  if (getError() || !enable_) {
    return getErrorID();
  }

  int errorCode = getData(&data_);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  errorCode = dataBuffer_->appendData(data_);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  printStatus();

  return 0;
}

int ecmcDataRecorder::setAxisDataSource(ecmcAxisStatusType *axisData,
                                        ecmcAxisDataType    dataToStore) {
  axisData_             = axisData;
  axisDataTypeToRecord_ = dataToStore;
  printAxisDataSource();
  return 0;
}

int ecmcDataRecorder::setDataSourceType(ecmcDataSourceType type) {
  dataSource_ = type;
  printDataSource();

  return 0;
}

int ecmcDataRecorder::getData(double *data) {
  int error = 0;

  switch (dataSource_) {
  case  ECMC_RECORDER_SOURCE_NONE:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_RECORDER_NO_DATA_SOURCE_CHOOSEN);

    break;

  case ECMC_RECORDER_SOURCE_ETHERCAT:
    error = getEtherCATData(data);

    if (error) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }
    break;

  case ECMC_RECORDER_SOURCE_AXIS:
    error = getAxisData(data);

    if (error) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }
    break;

  default:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_RECORDER_NO_DATA_SOURCE_CHOOSEN);

    break;
  }
  return 0;
}

int ecmcDataRecorder::getEtherCATData(double *data) {
  uint64_t tempRaw = 0;

  if (readEcEntryValue(0, &tempRaw)) {  // Data
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EVENT_ECENTRY_READ_FAIL);
  }
  *data = static_cast<double>(tempRaw);
  return 0;
}

int ecmcDataRecorder::getAxisData(double *data) {
  if (!axisData_) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_RECORDER_AXIS_DATA_NULL);
  }

  switch (axisDataTypeToRecord_) {
  case ECMC_AXIS_DATA_NONE:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_RECORDER_AXIS_DATA_TYPE_NOT_CHOOSEN);

    break;

  case ECMC_AXIS_DATA_AXIS_ID:
    *data = static_cast<double>(axisData_->axisID);
    break;

  case ECMC_AXIS_DATA_POS_SET:
    *data = axisData_->onChangeData.positionSetpoint;
    break;

  case ECMC_AXIS_DATA_POS_ACT:
    *data = axisData_->onChangeData.positionActual;
    break;

  case ECMC_AXIS_DATA_CNTRL_ERROR:
    *data = axisData_->onChangeData.cntrlError;
    break;

  case ECMC_AXIS_DATA_POS_TARGET:
    *data = axisData_->onChangeData.positionTarget;
    break;

  case ECMC_AXIS_DATA_POS_ERROR:
    *data = axisData_->onChangeData.positionError;
    break;

  case ECMC_AXIS_DATA_POS_RAW:
    *data = static_cast<double>(axisData_->onChangeData.positionRaw);
    break;

  case ECMC_AXIS_DATA_CNTRL_OUT:
    *data = axisData_->onChangeData.cntrlOutput;
    break;

  case ECMC_AXIS_DATA_VEL_SET:
    *data = axisData_->onChangeData.velocitySetpoint;
    break;

  case ECMC_AXIS_DATA_VEL_ACT:
    *data = axisData_->onChangeData.velocityActual;
    break;

  case ECMC_AXIS_DATA_VEL_SET_FF_RAW:
    *data = axisData_->onChangeData.velocityFFRaw;
    break;

  case ECMC_AXIS_DATA_VEL_SET_RAW:
    *data = static_cast<double>(axisData_->onChangeData.velocitySetpointRaw);
    break;

  case ECMC_AXIS_DATA_CYCLE_COUNTER:
    *data = static_cast<double>(axisData_->cycleCounter);
    break;

  case ECMC_AXIS_DATA_ERROR:
    *data = static_cast<double>(axisData_->onChangeData.error);
    break;

  case ECMC_AXIS_DATA_COMMAND:
    *data = static_cast<double>(axisData_->onChangeData.command);
    break;

  case ECMC_AXIS_DATA_CMD_DATA:
    *data = static_cast<double>(axisData_->onChangeData.cmdData);
    break;

  case ECMC_AXIS_DATA_SEQ_STATE:
    *data = static_cast<double>(axisData_->onChangeData.seqState);
    break;

  case ECMC_AXIS_DATA_INTERLOCK_TYPE:
    *data = static_cast<double>(axisData_->onChangeData.trajInterlock);
    break;

  case ECMC_AXIS_DATA_TRAJ_SOURCE:
    *data = static_cast<double>(axisData_->onChangeData.trajSource);
    break;

  case ECMC_AXIS_DATA_ENC_SOURCE:
    *data = static_cast<double>(axisData_->onChangeData.encSource);
    break;

  case ECMC_AXIS_DATA_ENABLE:
    *data = static_cast<double>(axisData_->onChangeData.enable);
    break;

  case ECMC_AXIS_DATA_ENABLED:
    *data = static_cast<double>(axisData_->onChangeData.enabled);
    break;

  case ECMC_AXIS_DATA_EXECUTE:
    *data = static_cast<double>(axisData_->onChangeData.execute);
    break;

  case ECMC_AXIS_DATA_BUSY:
    *data = static_cast<double>(axisData_->onChangeData.busy);
    break;

  case ECMC_AXIS_DATA_AT_TARGET:
    *data = static_cast<double>(axisData_->onChangeData.atTarget);
    break;

  case ECMC_AXIS_DATA_HOMED:
    *data = static_cast<double>(axisData_->onChangeData.homed);
    break;

  case ECMC_AXIS_DATA_LIMIT_BWD:
    *data = static_cast<double>(axisData_->onChangeData.limitBwd);
    break;

  case ECMC_AXIS_DATA_LIMIT_FWD:
    *data = static_cast<double>(axisData_->onChangeData.limitFwd);
    break;

  case ECMC_AXIS_DATA_HOME_SWITCH:
    *data = static_cast<double>(axisData_->onChangeData.homeSwitch);
    break;

  default:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_RECORDER_AXIS_DATA_TYPE_NOT_CHOOSEN);

    break;
  }
  return 0;
}
