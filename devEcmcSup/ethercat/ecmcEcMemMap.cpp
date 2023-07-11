/*************************************************************************\
* Copyright (c) 2019 European spallation Source ERIC
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
#include "ecmcErrorsList.h"

ecmcEcMemMap::ecmcEcMemMap(ecmcAsynPortDriver *asynPortDriver,
                           int masterId,
                           int slaveId,
                           ecmcEcEntry   *startEntry,
                           size_t         byteSize,
                           ec_direction_t nDirection,
                           ecmcEcDataType dt,
                           std::string    id) {
  initVars();
  asynPortDriver_  = asynPortDriver;
  masterId_        = masterId;
  startEntry_      = startEntry;
  byteSize_        = byteSize;
  direction_       = nDirection;
  idString_        = id;
  idStringChar_    = strdup(idString_.c_str());
  buffer_          = new uint8_t[byteSize_];
  slaveId_         = slaveId;
  dataType_        = dt;
  bytesPerElement_ = getEcDataTypeByteSize(dataType_);
  if(bytesPerElement_>0) {
    elements_        = byteSize_ / bytesPerElement_;
  }
  initAsyn();
}

void ecmcEcMemMap::initVars() {
  errorReset();
  asynPortDriver_  = NULL;
  masterId_        = -1;
  direction_       = EC_DIR_INVALID;
  idString_        = "";
  startEntry_      = NULL;
  byteSize_        = 0;
  buffer_          = NULL;
  domainSize_      = 0;
  adr_             = 0;
  memMapAsynParam_ = NULL;
  slaveId_         = 0;
  dataType_        = ECMC_EC_NONE;
  elements_        = 0;
  bytesPerElement_ = 0;
  int8Ptr_         = (int8_t*)&buffer_;
  uint8Ptr_        = (uint8_t*)&buffer_;
  int16Ptr_        = (int16_t*)&buffer_;
  uint16Ptr_       = (uint16_t*)&buffer_;
  int32Ptr_        = (int32_t*)&buffer_;
  uint32Ptr_       = (uint32_t*)&buffer_;
  int64Ptr_        = (int64_t*)&buffer_;
  uint64Ptr_       = (uint64_t*)&buffer_;
  float32Ptr_      = (float*)&buffer_;
  float64Ptr_      = (double*)&buffer_;
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

  if (domainAdr_ == NULL) {
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

  unsigned int charCount = snprintf(buffer,
                                    sizeof(buffer),
                                    "%s",
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
                                         dataType_,
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
  memMapAsynParam_->setAllowWriteToEcmc(direction_ == EC_DIR_OUTPUT);
  memMapAsynParam_->refreshParam(1);
  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);

  return 0;
}

int ecmcEcMemMap::getByteSize() {
  return byteSize_;
}

uint8_t* ecmcEcMemMap::getBufferPointer() {
  return buffer_;
}

ecmcEcDataType ecmcEcMemMap::getDataType() {
  return dataType_;
}

int ecmcEcMemMap::getDoubleDataAtIndex(size_t index, double *data) {
  if(index >= elements_) {
    return ERROR_MEM_INDEX_OUT_OF_RANGE;
  }
      
  switch(dataType_) {
  case ECMC_EC_U8:
    uint8Ptr_ = (uint8_t*)&buffer_[index*bytesPerElement_];
    *data = (double)*uint8Ptr_;
    break;

  case ECMC_EC_S8:
    int8Ptr_ = (int8_t*)&buffer_[index*bytesPerElement_];
    *data = (double)*int8Ptr_;
    break;

  case ECMC_EC_U16:
    uint16Ptr_ = (uint16_t*)&buffer_[index*bytesPerElement_];
    *data = (double)*uint16Ptr_;
    break;

  case ECMC_EC_S16:
    int16Ptr_ = (int16_t*)&buffer_[index*bytesPerElement_];
    *data = (double)*int16Ptr_;
    break;

  case ECMC_EC_U32:
    uint32Ptr_ = (uint32_t*)&buffer_[index*bytesPerElement_];
    *data = (double)*uint32Ptr_;
    break;

  case ECMC_EC_S32:
    int32Ptr_ = (int32_t*)&buffer_[index*bytesPerElement_];
    *data = (double)*int32Ptr_;
    break;

  case ECMC_EC_U64:
    uint64Ptr_ = (uint64_t*)&buffer_[index*bytesPerElement_];
    *data = (double)*uint64Ptr_;
    break;

  case ECMC_EC_S64:
    int64Ptr_ = (int64_t*)&buffer_[index*bytesPerElement_];
    *data = (double)*int64Ptr_;
    break;

  case ECMC_EC_F32:
    float32Ptr_ = (float*)&buffer_[index*bytesPerElement_];
    *data = (double)*float32Ptr_;
    break;

  case ECMC_EC_F64:
    float64Ptr_ = (double*)&buffer_[index*bytesPerElement_];
    *data = (double)*float64Ptr_;
    break;

  default:
    *data = 0;
    return ERROR_MEM_INVALID_DATA_TYPE;
    break;
  }

  return 0;
}

int ecmcEcMemMap::setDoubleDataAtIndex(size_t index, double data) {
  if(index >= elements_) {
    return ERROR_MEM_INDEX_OUT_OF_RANGE;
  }
  
  switch(dataType_) {
  case ECMC_EC_U8:
    uint8Ptr_ = (uint8_t*)&buffer_[index*bytesPerElement_];
    *uint8Ptr_ = (uint8_t)data;
    break;

  case ECMC_EC_S8:
    int8Ptr_ = (int8_t*)&buffer_[index*bytesPerElement_];
    *int8Ptr_ = (int8_t)data;
    break;

  case ECMC_EC_U16:
    uint16Ptr_ = (uint16_t*)&buffer_[index*bytesPerElement_];
    *uint16Ptr_ = (uint16_t)data;
    break;

  case ECMC_EC_S16:
    int16Ptr_ = (int16_t*)&buffer_[index*bytesPerElement_];
    *int16Ptr_ = (int16_t)data;
    break;

  case ECMC_EC_U32:
    uint32Ptr_ = (uint32_t*)&buffer_[index*bytesPerElement_];
    *uint32Ptr_ = (uint32_t)data;
    break;

  case ECMC_EC_S32:
    int32Ptr_ = (int32_t*)&buffer_[index*bytesPerElement_];
    *int32Ptr_ = (int32_t)data;
    break;

  case ECMC_EC_U64:
    uint64Ptr_ = (uint64_t*)&buffer_[index*bytesPerElement_];
    *uint64Ptr_ = (uint64_t)data;
    break;

  case ECMC_EC_S64:
    int64Ptr_ = (int64_t*)&buffer_[index*bytesPerElement_];
    *int64Ptr_ = (int64_t)data;
    break;

  case ECMC_EC_F32:
    float32Ptr_ = (float*)&buffer_[index*bytesPerElement_];
    *float32Ptr_ = (float)data;

    break;

  case ECMC_EC_F64:
    float64Ptr_ = (double*)&buffer_[index*bytesPerElement_];
    *float64Ptr_ = (double)data;
    break;

  default:    
    return ERROR_MEM_INVALID_DATA_TYPE;
    break;
  }

  return 0;
}

size_t ecmcEcMemMap::getElementCount() {
  return  elements_;
}

size_t ecmcEcMemMap::getBytesPerElement() {
  return  bytesPerElement_;
}
