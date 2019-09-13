/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEcMemMap.cpp
*
*  Created on: Dec 2, 2015
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcEcMemMap.h"
#include <stdlib.h> 

ecmcEcMemMap::ecmcEcMemMap(ecmcAsynPortDriver *asynPortDriver,
                           int masterId,
                           ecmcEcEntry   *startEntry,
                           size_t         byteSize,
                           int            type,
                           ec_direction_t nDirection,
                           std::string    id) {
  initVars();
  asynPortDriver_ = asynPortDriver;
  masterId_=masterId;
  startEntry_ = startEntry;
  byteSize_   = byteSize;
  direction_  = nDirection;
  idString_   = id;
  idStringChar_  = strdup(idString_.c_str());
  buffer_     = new uint8_t[byteSize_];
  type_       = type;
  initAsyn();
}

void ecmcEcMemMap::initVars() {
  errorReset();
  asynPortDriver_ = NULL;
  masterId_       = -1;
  direction_      = EC_DIR_INVALID;
  idString_       = "";
  startEntry_     = NULL;
  byteSize_       = 0;
  buffer_         = NULL;
  type_           = 0;
  domainSize_     = 0;
  adr_            = 0;
  memMapAsynParam_ = NULL;
}

ecmcEcMemMap::~ecmcEcMemMap() {
  delete buffer_;
  buffer_ = NULL;
  free(idStringChar_);
  idStringChar_ = NULL;
  delete memMapAsynParam_;
  memMapAsynParam_=NULL;
}

int ecmcEcMemMap::write(uint8_t *values,
                        size_t   byteToWrite,
                        size_t  *bytesWritten) {
  size_t bytesToCopy = byteToWrite;

  if (byteToWrite > byteSize_) {
    bytesToCopy = byteSize_;
  }
  memcpy(buffer_, values, bytesToCopy);
  *bytesWritten = bytesToCopy;
  return 0;
}

int ecmcEcMemMap::read(uint8_t *values, size_t bytesToRead,
                       size_t *bytesRead) {
  size_t bytesToCopy = bytesToRead;

  if (bytesToRead > byteSize_) {
    bytesToCopy = byteSize_;
  }

  memcpy(values, buffer_, bytesToCopy);
  *bytesRead = bytesToCopy;
  return 0;
}

int ecmcEcMemMap::updateInputProcessImage() {
  if (direction_ != EC_DIR_INPUT) {
    return 0;
  }

  memcpy(buffer_, adr_, byteSize_);
  updateAsyn(0);
  return 0;
}

int ecmcEcMemMap::updateOutProcessImage() {
  if (direction_ != EC_DIR_OUTPUT) {
    return 0;
  }

  memcpy(adr_, buffer_, byteSize_);
  return 0;
}

std::string ecmcEcMemMap::getIdentificationName() {
  return idString_;
}

int ecmcEcMemMap::setDomainSize(size_t size) {
  domainSize_ = size;
  return 0;
}

int ecmcEcMemMap::updateAsyn(bool force) {
  memMapAsynParam_->refreshParamRT(force);
  return 0;
}

int ecmcEcMemMap::validate() {
  byteOffset_ = startEntry_->getByteOffset();
  domainAdr_  = startEntry_->getDomainAdr();

  if (byteOffset_ < 0) {
    if (getErrorID() != ERROR_EC_ENTRY_INVALID_OFFSET) {
      LOGERR("%s/%s:%d: ERROR: MemMap %s: Invalid data offset (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             idString_.c_str(),
             ERROR_EC_ENTRY_INVALID_OFFSET);
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_INVALID_OFFSET);
  }

  if ((domainAdr_ < 0) || (domainAdr_ == NULL)) {
    if (getErrorID() != ERROR_EC_ENTRY_INVALID_DOMAIN_ADR) {
      LOGERR("%s/%s:%d: ERROR: MemMap %s: Invalid domain address (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             idString_.c_str(),
             ERROR_EC_ENTRY_INVALID_DOMAIN_ADR);
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_INVALID_DOMAIN_ADR);
  }

  if (byteOffset_ + byteSize_ > domainSize_) {
    if (getErrorID() != ERROR_MEM_MAP_SIZE_OUT_OF_RANGE) {
      LOGERR(
        "%s/%s:%d: ERROR: MemMap %s: Byte size, including offset, exceeds domain size (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        idString_.c_str(),
        ERROR_MEM_MAP_SIZE_OUT_OF_RANGE);
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MEM_MAP_SIZE_OUT_OF_RANGE);
  }
  adr_ = domainAdr_ + byteOffset_;
  return 0;
}

int ecmcEcMemMap::initAsyn() {
  
  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];  
  char *name = buffer;

  // "ec%d.mm.alias"
  unsigned int charCount = snprintf(buffer,
                                    sizeof(buffer),
                                    ECMC_EC_STR "%d." ECMC_MEMMAP_STR".%s",
                                    masterId_,
                                    idStringChar_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_MEM_ASYN_VAR_BUFFER_OUT_OF_RANGE);
    return ERROR_MEM_ASYN_VAR_BUFFER_OUT_OF_RANGE;
  }
  name = buffer;
  memMapAsynParam_ = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt8Array,  //default type
                                         buffer_,
                                         byteSize_,
                                         0);
  if(!memMapAsynParam_) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }

  memMapAsynParam_->addSupportedAsynType(asynParamInt8Array);
  memMapAsynParam_->addSupportedAsynType(asynParamInt16Array);
  memMapAsynParam_->addSupportedAsynType(asynParamInt32Array);
  memMapAsynParam_->addSupportedAsynType(asynParamFloat32Array);
  memMapAsynParam_->addSupportedAsynType(asynParamFloat64Array);
  memMapAsynParam_->allowWriteToEcmc(direction_ == EC_DIR_OUTPUT);
  memMapAsynParam_->refreshParam(1);
  asynPortDriver_->callParamCallbacks();

  return 0;
}