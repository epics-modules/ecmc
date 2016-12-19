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
  execute_=false;
}

int ecmcDataRecorder::setDataStorage(ecmcDataStorage* buffer)
{
  dataBuffer_=buffer;
  validate();
  return 0;
}

int ecmcDataRecorder::validate()
{
  if(dataBuffer_==NULL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_RECORDER_BUFFER_NULL);
  }

  if(validateEntry(0)){ //Data
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_RECORDER_DATA_ECENTRY_NULL);
  }

  return setErrorID(0);
}

int ecmcDataRecorder::setExecute(int execute)
{
  execute_=execute;
  validate();
  return 0;
}

void ecmcDataRecorder::printStatus()
{
  LOGINFO11("%s/%s:%d: INFO: Data Recorder %d. Data: %lf, Error: 0x%x.\n",index_,(double)data_,getErrorID());
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

  if(getError() || !execute_){
    return getErrorID();
  }

  uint64_t tempRaw=0;
  if(readEcEntryValue(0,&tempRaw)){//Data
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EVENT_ECENTRY_READ_FAIL);
  }

  data_=tempRaw;
  int errorCode=dataBuffer_->appendData((double)data_);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  printStatus();

  return 0;
}
