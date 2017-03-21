/*
 * ecmcDataRecorder.cpp
 *
 *  Created on: Jun 1, 2016
 *      Author: anderssandstrom
 */

#include "ecmcDataRecorder.h"

ecmcDataRecorder::ecmcDataRecorder (int index):ecmcEcEntryLink()
{
  initVars();
  setInStartupPhase(1);
  index_=index;
  LOGINFO11("%s/%s:%d: dataRecorder[%d]=new;\n",__FILE__, __FUNCTION__, __LINE__,index);
}

ecmcDataRecorder::~ecmcDataRecorder ()
{

}

void ecmcDataRecorder::initVars()
{
  dataBuffer_=NULL;
  index_=0;
  data_=0;
  inStartupPhase_=1;
  enable_=false;
  axisData_=NULL;
  axisDataTypeToRecord_=ECMC_RECORDER_AXIS_DATA_NONE;
  dataSource_=ECMC_RECORDER_SOURCE_NONE;
}

int ecmcDataRecorder::setDataStorage(ecmcDataStorage* buffer)
{
  dataBuffer_=buffer;
  return 0;
}

int ecmcDataRecorder::validate()
{
   switch(dataSource_){
    case  ECMC_RECORDER_SOURCE_NONE:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_RECORDER_NO_DATA_SOURCE_CHOOSEN);
      break;
    case ECMC_RECORDER_SOURCE_ETHERCAT:
      if(validateEntry(0)){ //Data
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_RECORDER_DATA_ECENTRY_NULL);
      }
      break;
    case ECMC_RECORDER_SOURCE_AXIS:
      if(axisDataTypeToRecord_==ECMC_RECORDER_AXIS_DATA_NONE){
	return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_RECORDER_AXIS_DATA_TYPE_NOT_CHOOSEN);
      }
      break;
    default:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_RECORDER_NO_DATA_SOURCE_CHOOSEN);
      break;
  }

  if(dataBuffer_==NULL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_RECORDER_BUFFER_NULL);
  }

  return setErrorID(0);
}

int ecmcDataRecorder::setEnable(int enable)
{
  if(enable_!=enable){
      LOGINFO11("%s/%s:%d: dataRecorder[%d].enable=%d;\n",__FILE__, __FUNCTION__, __LINE__,index_,enable);
  }

  enable_=enable;
  //LOGINFO11("%s/%s:%d: INFO: Data Recorder %d. Enable set to %d.\n",__FILE__, __FUNCTION__, __LINE__,index_,enable_);
  validate();
  return 0;
}

int ecmcDataRecorder::getEnabled(int *enabled)
{
  *enabled=enable_;
  return 0;
}


void ecmcDataRecorder::printStatus()
{
  LOGINFO11("%s/%s:%d: INFO: Data Recorder %d. Data: %lf, Error: 0x%x.\n",__FILE__,__FUNCTION__,__LINE__,index_,(double)data_,getErrorID());
}

void ecmcDataRecorder::setInStartupPhase(bool startup)
{
  inStartupPhase_=startup;
}

int ecmcDataRecorder::executeEvent(int masterOK)
{
  if(!masterOK){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_RECORDER_HARDWARE_STATUS_NOT_OK);
  }

  if(inStartupPhase_){
    //Auto reset hardware error
    if(getErrorID()==ERROR_DATA_RECORDER_HARDWARE_STATUS_NOT_OK){
      setErrorID(0);
    }
    setInStartupPhase(false);
  }

  if(getError() || !enable_){
    return getErrorID();
  }

  int errorCode=getData(&data_);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  errorCode=dataBuffer_->appendData(data_);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  printStatus();

  return 0;
}

int ecmcDataRecorder::setAxisDataSource(ecmcAxisStatusType *axisData,ecmcAxisDataRecordType dataToStore)
{
  axisData_=axisData;
  axisDataTypeToRecord_=dataToStore;
  LOGINFO11("%s/%s:%d: INFO: Data Recorder %d. Axis data type set to %d.\n",__FILE__, __FUNCTION__, __LINE__,index_,axisDataTypeToRecord_);
  return 0;
}

int ecmcDataRecorder::setDataSourceType(ecmcDataSourceType type)
{
  dataSource_=type;
  LOGINFO11("%s/%s:%d: INFO: Data Recorder %d. Data source set to %d.\n",__FILE__, __FUNCTION__, __LINE__,index_,dataSource_);
  return 0;
}

int ecmcDataRecorder::getData(double *data)
{
  int error=0;
  switch(dataSource_){
    case  ECMC_RECORDER_SOURCE_NONE:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_RECORDER_NO_DATA_SOURCE_CHOOSEN);
      break;
    case ECMC_RECORDER_SOURCE_ETHERCAT:
      error=getEtherCATData(data);
      if(error){
	return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
      }
      break;
    case ECMC_RECORDER_SOURCE_AXIS:
      error=getAxisData(data);
      if(error){
	return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
      }
      break;
    default:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_RECORDER_NO_DATA_SOURCE_CHOOSEN);
      break;
  }
  return 0;
}

int ecmcDataRecorder::getEtherCATData(double *data)
{
  uint64_t tempRaw=0;
  if(readEcEntryValue(0,&tempRaw)){//Data
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EVENT_ECENTRY_READ_FAIL);
  }
  *data=(double)tempRaw;
  return 0;
}

int ecmcDataRecorder::getAxisData(double *data)
{
  if(!axisData_){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_RECORDER_AXIS_DATA_NULL);
  }

  switch(axisDataTypeToRecord_){
    case ECMC_RECORDER_AXIS_DATA_NONE:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_RECORDER_AXIS_DATA_TYPE_NOT_CHOOSEN);
      break;
    case ECMC_RECORDER_AXIS_DATA_AXIS_ID:
      *data=(double)axisData_->axisID;
      break;
    case ECMC_RECORDER_AXIS_DATA_POS_SET:
      *data=axisData_->onChangeData.positionSetpoint;
      break;
    case ECMC_RECORDER_AXIS_DATA_POS_ACT:
      *data=axisData_->onChangeData.positionActual;
      break;
    case ECMC_RECORDER_AXIS_DATA_CNTRL_ERROR:
      *data=axisData_->onChangeData.cntrlError;
      break;
    case ECMC_RECORDER_AXIS_DATA_POS_TARGET:
      *data=axisData_->onChangeData.positionTarget;
      break;
    case ECMC_RECORDER_AXIS_DATA_POS_ERROR:
      *data=axisData_->onChangeData.positionError;
      break;
    case ECMC_RECORDER_AXIS_DATA_POS_RAW:
      *data=(double)axisData_->onChangeData.positionRaw;
      break;
    case ECMC_RECORDER_AXIS_DATA_CNTRL_OUT:
      *data=axisData_->onChangeData.cntrlOutput;
      break;
    case ECMC_RECORDER_AXIS_DATA_VEL_SET:
      *data=axisData_->onChangeData.velocitySetpoint;
      break;
    case ECMC_RECORDER_AXIS_DATA_VEL_ACT:
      *data=axisData_->onChangeData.velocityActual;
      break;
    case ECMC_RECORDER_AXIS_DATA_VEL_SET_FF_RAW:
      *data=axisData_->onChangeData.velocityFFRaw;
      break;
    case ECMC_RECORDER_AXIS_DATA_VEL_SET_RAW:
      *data=(double)axisData_->onChangeData.velocitySetpointRaw;
      break;
    case ECMC_RECORDER_AXIS_DATA_CYCLE_COUNTER:
      *data=(double)axisData_->cycleCounter;
      break;
    case ECMC_RECORDER_AXIS_DATA_ERROR:
      *data=(double)axisData_->onChangeData.error;
      break;
    case ECMC_RECORDER_AXIS_DATA_COMMAND:
      *data=(double)axisData_->onChangeData.command;
      break;
    case ECMC_RECORDER_AXIS_DATA_CMD_DATA:
      *data=(double)axisData_->onChangeData.cmdData;
      break;
    case ECMC_RECORDER_AXIS_DATA_SEQ_STATE:
      *data=(double)axisData_->onChangeData.seqState;
      break;
    case ECMC_RECORDER_AXIS_DATA_INTERLOCK_TYPE:
      *data=(double)axisData_->onChangeData.trajInterlock;
      break;
    case ECMC_RECORDER_AXIS_DATA_TRAJ_SOURCE:
      *data=(double)axisData_->onChangeData.trajSource;
      break;
    case ECMC_RECORDER_AXIS_DATA_ENC_SOURCE:
      *data=(double)axisData_->onChangeData.encSource;
      break;
    case ECMC_RECORDER_AXIS_DATA_ENABLE:
      *data=(double)axisData_->onChangeData.enable;
      break;
    case ECMC_RECORDER_AXIS_DATA_ENABLED:
      *data=(double)axisData_->onChangeData.enabled;
      break;
    case ECMC_RECORDER_AXIS_DATA_EXECUTE:
      *data=(double)axisData_->onChangeData.execute;
      break;
    case ECMC_RECORDER_AXIS_DATA_BUSY:
      *data=(double)axisData_->onChangeData.busy;
      break;
    case ECMC_RECORDER_AXIS_DATA_AT_TARGET:
      *data=(double)axisData_->onChangeData.atTarget;
      break;
    case ECMC_RECORDER_AXIS_DATA_HOMED:
      *data=(double)axisData_->onChangeData.homed;
      break;
    case ECMC_RECORDER_AXIS_DATA_LIMIT_BWD:
      *data=(double)axisData_->onChangeData.limitBwd;
      break;
    case ECMC_RECORDER_AXIS_DATA_LIMIT_FWD:
      *data=(double)axisData_->onChangeData.limitFwd;
      break;
    case ECMC_RECORDER_AXIS_DATA_HOME_SWITCH:
      *data=(double)axisData_->onChangeData.homeSwitch;
      break;
    default:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_RECORDER_AXIS_DATA_TYPE_NOT_CHOOSEN);
      break;
  }
  return 0;
}
