/*
 * ecmcEcMemMap.cpp
 *
 *  Created on: Dec 2, 2015
 *      Author: anderssandstrom
 */

#include "ecmcEcMemMap.h"

ecmcEcMemMap::ecmcEcMemMap(ecmcEcEntry *startEntry,size_t byteSize,int type,ec_direction_t nDirection, std::string id)
{
  initVars();
  startEntry_=startEntry;
  byteSize_=byteSize;
  direction_=nDirection;
  idString_=id;
  buffer_=new uint8_t[byteSize_];
  type_=type;
}

void ecmcEcMemMap::initVars()
{
  errorReset();
  direction_=EC_DIR_INVALID;
  idString_="";
  startEntry_=NULL;
  byteSize_=0;
  buffer_=NULL;
  type_=0;
  domainSize_=0;
  validationDone_=0;
}

ecmcEcMemMap::~ecmcEcMemMap()
{
  delete buffer_;
}

int ecmcEcMemMap::write(uint8_t *values, size_t byteToWrite, size_t *bytesWritten)
{
  size_t bytesToCopy=byteToWrite;
  if(byteToWrite>byteSize_){
    bytesToCopy=byteSize_;
  }
  memcpy(buffer_,values,bytesToCopy);
  *bytesWritten=bytesToCopy;
  return 0;
}

int ecmcEcMemMap::read(uint8_t *values, size_t bytesToRead, size_t *bytesRead)
{
  size_t bytesToCopy=bytesToRead;
  if(bytesToRead>byteSize_){
    bytesToCopy=byteSize_;
  }

  memcpy(values,buffer_,bytesToCopy);
  *bytesRead=bytesToCopy;
  return 0;
}

int ecmcEcMemMap::updateInputProcessImage()
{

  if(direction_!=EC_DIR_INPUT){
    return 0;
  }

  if(!validationDone_){
    if(validate()!=0){
      return 0;
    }
    validationDone_=1;
  }

  memcpy(buffer_,(domainAdr_+byteOffset_),byteSize_);
  updateAsyn(0);
  return 0;
}

int ecmcEcMemMap::updateOutProcessImage()
{
  if(direction_!=EC_DIR_OUTPUT){
    return 0;
  }

  if(!validationDone_){
    if(validate()!=0){
      return 0;
    }
    validationDone_=1;
  }

  memcpy((domainAdr_+byteOffset_),buffer_,byteSize_);
  return 0;
}

std::string ecmcEcMemMap::getIdentificationName()
{
  return idString_;
}

int ecmcEcMemMap::setDomainSize(size_t size)
{
  domainSize_=size;
  return 0;
}

int ecmcEcMemMap::updateAsyn(bool force)
{
  //I/O intr to EPICS
  if(!asynPortDriver_ || asynParameterIndex_<0){
   return 0;
  }

  if(asynUpdateCycleCounter_>=asynUpdateCycles_ || force){ //Only update at desired samplerate

    switch(asynParameterType_){
      case asynParamInt8Array:
        asynPortDriver_->doCallbacksInt8Array((epicsInt8*)buffer_,byteSize_, asynParameterIndex_, 0);
	asynUpdateCycleCounter_=0;
        break;
      case asynParamInt16Array:
        asynPortDriver_->doCallbacksInt16Array((epicsInt16*)buffer_,byteSize_, asynParameterIndex_, 0);
	asynUpdateCycleCounter_=0;
        break;
      case asynParamInt32Array:
        asynPortDriver_->doCallbacksInt32Array((epicsInt32*)buffer_,byteSize_, asynParameterIndex_, 0);
	asynUpdateCycleCounter_=0;
        break;
      case asynParamFloat32Array:
        asynPortDriver_->doCallbacksFloat32Array((epicsFloat32*)buffer_,byteSize_, asynParameterIndex_, 0);
	asynUpdateCycleCounter_=0;
        break;
      case asynParamFloat64Array:
        asynPortDriver_->doCallbacksFloat64Array((epicsFloat64*)buffer_,byteSize_, asynParameterIndex_, 0);
	asynUpdateCycleCounter_=0;
        break;
      default:
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ENTRY_ASYN_TYPE_NOT_SUPPORTED);
        break;
    }
  }
  else{
    asynUpdateCycleCounter_++;
  }
  return 0;
}


int ecmcEcMemMap::validate()
{
  byteOffset_=startEntry_->getByteOffset();
  domainAdr_=startEntry_->getDomainAdr();

  if(byteOffset_<0 ){
    LOGERR("%s/%s:%d: ERROR: MemMap %s: Invalid data offset (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,idString_.c_str(),ERROR_EC_ENTRY_INVALID_OFFSET);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ENTRY_INVALID_OFFSET);
  }

  if(domainAdr_<0 || domainAdr_==NULL){
    LOGERR("%s/%s:%d: ERROR: MemMap %s: Invalid domain address (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,idString_.c_str(),ERROR_EC_ENTRY_INVALID_DOMAIN_ADR);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ENTRY_INVALID_DOMAIN_ADR);
  }

  if(byteOffset_+byteSize_>domainSize_){
    LOGERR("%s/%s:%d: ERROR: MemMap %s: Byte size, including offset, exceeds domain size (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,idString_.c_str(),ERROR_MEM_MAP_SIZE_OUT_OF_RANGE);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_MEM_MAP_SIZE_OUT_OF_RANGE);
  }
  return 0;
}
