#include "../com/ecmcAsynDataItem.h"
#include "../com/ecmcAsynPortDriver.h"
#include <stdio.h>
#include <string.h>

ecmcAsynDataItem::ecmcAsynDataItem (ecmcAsynPortDriver *asynPortDriver)
{
  asynPortDriver_=asynPortDriver;  
  data_=0;  
  asynUpdateCycleCounter_=0;
  paramInfo_= new ecmcParamInfo();
  memset(paramInfo_,0,sizeof(ecmcParamInfo));
  paramInfo_->asynType=asynParamNotDefined;
}

ecmcAsynDataItem::~ecmcAsynDataItem ()
{
  delete paramInfo_->recordName;
  delete paramInfo_->recordType;
  delete paramInfo_->scan;
  delete paramInfo_->dtyp;
  delete paramInfo_->inp;
  delete paramInfo_->out;
  delete paramInfo_->drvInfo;
  delete paramInfo_->asynTypeStr;
  delete paramInfo_->name;
  delete paramInfo_;
}

int ecmcAsynDataItem::setEcmcDataPointer(uint8_t *data,size_t bytes)
{
  data_=data;
  paramInfo_->ecmcSize=bytes;
  paramInfo_->ecmcDataPointerValid =  data && bytes>0;
  return 0;
}

int ecmcAsynDataItem::refreshParamRT(int force)
{
  if(!asynPortDriver_->getAllowRtThreadCom()){
    return 0;
  }
  return refreshParam(force,data_,paramInfo_->ecmcSize);
}

int ecmcAsynDataItem::refreshParam(int force)
{
  return refreshParam(force,data_,paramInfo_->ecmcSize);
}

int ecmcAsynDataItem::refreshParamRT(int force, size_t bytes)
{
  if(!asynPortDriver_->getAllowRtThreadCom()){
    return 0;
  }
  return refreshParam(force,data_,bytes);
}

int ecmcAsynDataItem::refreshParam(int force, size_t bytes)
{
  return refreshParam(force,data_,bytes);
}

int ecmcAsynDataItem::refreshParamRT(int force,uint8_t *data, size_t bytes)
{
  if(!asynPortDriver_->getAllowRtThreadCom()){
    return 0;
  }
  return refreshParam(force,data,bytes);
}
int ecmcAsynDataItem::refreshParam(int force,uint8_t *data, size_t bytes)
{

  if(asynUpdateCycleCounter_<paramInfo_->sampleTimeCycles && !force){ //Only update at desired samplerate
    asynUpdateCycleCounter_++;
    return 0;
  }

  if(data==0 || bytes<0){
    return ERROR_ASYN_DATA_NULL;
  }

  if(asynPortDriver_==0){
    return ERROR_ASYN_PORT_NULL;
  }

  data_=data;
  paramInfo_->ecmcSize=bytes;

  switch(paramInfo_->asynType){
    case asynParamUInt32Digital:
      asynPortDriver_->setUIntDigitalParam(paramInfo_->index,*((epicsInt32*)data),0xFFFFFFFF);
      break;
    case asynParamInt32:
      asynPortDriver_->setIntegerParam(paramInfo_->index,*((epicsInt32*)data));
      break;
    case asynParamFloat64:
      asynPortDriver_->setDoubleParam(paramInfo_->index,*((epicsFloat64*)data));
      break;
    case asynParamInt8Array:
      asynPortDriver_->doCallbacksInt8Array((epicsInt8*)data,bytes, paramInfo_->index, 0);
      break;
    case asynParamInt16Array:
      asynPortDriver_->doCallbacksInt16Array((epicsInt16*)data,bytes/sizeof(epicsInt16), paramInfo_->index, 0);
      break;
    case asynParamInt32Array:
      asynPortDriver_->doCallbacksInt32Array((epicsInt32*)data,bytes/sizeof(epicsInt32), paramInfo_->index, 0);
      break;
    case asynParamFloat32Array:
      asynPortDriver_->doCallbacksFloat32Array((epicsFloat32*)data,bytes/sizeof(epicsFloat32), paramInfo_->index, 0);
      break;
    case asynParamFloat64Array:
      asynPortDriver_->doCallbacksFloat64Array((epicsFloat64*)data,bytes/sizeof(epicsFloat64), paramInfo_->index, 0);
      break;
    default:
      return ERROR_ASYN_DATA_TYPE_NOT_SUPPORTED;
      break;
  }
  asynUpdateCycleCounter_=0;
  return 0;
}

int ecmcAsynDataItem::createParam()
{  
  return createParam(paramInfo_->name,paramInfo_->asynType);
}

int ecmcAsynDataItem::createParam(const char *paramName,asynParamType asynParType,uint8_t *data,size_t bytes)
{
  setEcmcDataPointer(data,bytes);
  return createParam(paramName,asynParType);
}

int ecmcAsynDataItem::createParam(const char *paramName,asynParamType asynParType)
{
  if(asynPortDriver_==0){
    return ERROR_ASYN_PORT_NULL;
  }
  paramInfo_->name=strdup(paramName);  
  paramInfo_->asynType=asynParType;
  asynStatus status = asynPortDriver_->createParam(paramName,paramInfo_->asynType,&paramInfo_->index);
  return (status==asynSuccess) ? 0 : ERROR_ASYN_CREATE_PARAM_FAIL;
}

int ecmcAsynDataItem::getAsynParameterIndex()
{
  return paramInfo_->index;
}

int ecmcAsynDataItem::setAsynParameterType(asynParamType parType)
{
  paramInfo_->asynType=parType;
  return 0;
}

int ecmcAsynDataItem::getAsynParameterType()
{
  return paramInfo_->asynType;
}

int ecmcAsynDataItem::setAsynPortDriver(ecmcAsynPortDriver *asynPortDriver)
{
  asynPortDriver_=asynPortDriver;
  return 0;
}

int ecmcAsynDataItem::setAsynParSampleTimeMS(double sampleTime)
{
  paramInfo_->sampleTimeMS=sampleTime;
  paramInfo_->sampleTimeCycles=sampleTime/MCU_FREQUENCY-1;
  return 0;
}

ecmcParamInfo *ecmcAsynDataItem::getParamInfo()
{
  return paramInfo_;
}