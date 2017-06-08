/*
 * ecmcEcEntryArray.cpp
 *
 *  Created on: Dec 2, 2015
 *      Author: anderssandstrom
 */

#include "ecmcEcEntryArray.h"

ecmcEcEntryArray::ecmcEcEntryArray(ecmcEcEntry *startEntry,size_t byteSize,int type,ec_direction_t nDirection, std::string id)
{
  initVars();
  startEntry_=startEntry;
  byteSize_=byteSize;
  direction_=nDirection;
  idString_=id;
  buffer_=new uint8_t[byteSize_];
  type_=type;
}

void ecmcEcEntryArray::initVars()
{
  errorReset();
  direction_=EC_DIR_INVALID;
  idString_="";
  startEntry_=NULL;
  byteSize_=0;
  buffer_=NULL;
  type_=0;
}

ecmcEcEntryArray::~ecmcEcEntryArray()
{
  delete buffer_;
}

int ecmcEcEntryArray::write(uint8_t *values, size_t byteToWrite, size_t *bytesWritten)
{
  size_t bytesToCopy=byteToWrite;
  if(byteToWrite>byteSize_){
    bytesToCopy=byteSize_;
  }
  memcpy(buffer_,values,bytesToCopy);
  *bytesWritten=bytesToCopy;
  return 0;
}

int ecmcEcEntryArray::read(uint8_t *values, size_t bytesToRead, size_t *bytesRead)
{
  size_t bytesToCopy=bytesToRead;
  if(bytesToRead>byteSize_){
    bytesToCopy=byteSize_;
  }

  memcpy(values,buffer_,bytesToCopy);
  *bytesRead=bytesToCopy;
  return 0;
}

int ecmcEcEntryArray::updateInputProcessImage()
{
  if(direction_!=EC_DIR_INPUT){
    return 0;
  }

  byteOffset_=startEntry_->getByteOffset();
  domainAdr_=startEntry_->getDomainAdr();

  if(byteOffset_<0 ){
    LOGERR("%s/%s:%d: ERROR: EntryArray %s: Invalid data offset (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,idString_.c_str(),ERROR_EC_ENTRY_INVALID_OFFSET);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ENTRY_INVALID_OFFSET);
  }

  if(domainAdr_<0 || domainAdr_==NULL){
    LOGERR("%s/%s:%d: ERROR: EntryArray %s: Invalid domain address (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,idString_.c_str(),ERROR_EC_ENTRY_INVALID_DOMAIN_ADR);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ENTRY_INVALID_DOMAIN_ADR);
  }

  memcpy(buffer_,(domainAdr_+byteOffset_),byteSize_);

  //updateAsyn(0);
  return 0;
}

int ecmcEcEntryArray::updateOutProcessImage()
{
  if(direction_!=EC_DIR_OUTPUT){
    return 0;
  }

  byteOffset_=startEntry_->getByteOffset();
  domainAdr_=startEntry_->getDomainAdr();

  if(byteOffset_<0 ){
    LOGERR("%s/%s:%d: ERROR: EntryArray %s: Invalid data offset (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,idString_.c_str(),ERROR_EC_ENTRY_INVALID_OFFSET);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ENTRY_INVALID_OFFSET);
  }

  if(domainAdr_<0 || domainAdr_==NULL){
    LOGERR("%s/%s:%d: ERROR: EntryArray %s: Invalid domain address (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,idString_.c_str(),ERROR_EC_ENTRY_INVALID_DOMAIN_ADR);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ENTRY_INVALID_DOMAIN_ADR);
  }

  memcpy((domainAdr_+byteOffset_),buffer_,byteSize_);

  return 0;
}

std::string ecmcEcEntryArray::getIdentificationName()
{
  return idString_;
}

/*int ecmcEcEntryArray::updateAsyn(bool force)
{
  //I/O intr to EPICS
  if(!asynPortDriver_ || asynParameterIndex_<0){
   return 0;
  }

  if(asynUpdateCycleCounter_>=asynUpdateCycles_ || force){ //Only update at desired samplerate
    asynUpdateCycleCounter_=0;
    switch(asynParameterType_){
      case asynParamInt32:
        asynPortDriver_-> setIntegerParam(asynParameterIndex_,static_cast<int32_t>(value_));
        break;
      case asynParamFloat64:
        asynPortDriver_-> setDoubleParam(asynParameterIndex_,static_cast<double>(value_));
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
}*/
