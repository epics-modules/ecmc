/*************************************************************************\
* Copyright (c) 2019 European spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEcData.cpp
*
*  Created on: Dec 2, 2015
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcEcData.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ecmcErrorsList.h"

ecmcEcData::ecmcEcData(ecmcAsynPortDriver *asynPortDriver,
                               int masterId,
                               int slaveId,
                               ecmcEcDomain  *domain,
                               ecmcEcEntry   *startEntry,
                               ec_direction_t nDirection,
                               ecmcEcDataType dt,
                               std::string    id) {
  initVars();
  asynPortDriver_  = asynPortDriver;
  masterId_        = masterId;
  startEntry_      = startEntry;
  domain_          = domain;
  dataType_        = dt;
  byteSize_        = getEcDataTypeByteSize(dataType_);
  direction_       = nDirection;
  idString_        = id;
  idStringChar_    = strdup(idString_.c_str());
  buffer_          = new uint8_t[byteSize_];
  slaveId_         = slaveId;  
  initAsyn();
}

void ecmcEcData::initVars() {
  errorReset();
  asynPortDriver_  = NULL;
  domain_          = NULL;
  masterId_        = -1;
  direction_       = EC_DIR_INVALID;
  idString_        = "";
  startEntry_      = NULL;
  byteSize_        = 0;
  buffer_          = NULL;
  domainSize_      = 0;
  adr_             = 0;
  dataAsynParam_   = NULL;
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

ecmcEcData::~ecmcEcData() {
  delete buffer_;  
  free(idStringChar_);
  idStringChar_ = NULL;
  delete dataAsynParam_;
  dataAsynParam_=NULL;
}

int ecmcEcData::write(uint8_t *values,
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

int ecmcEcData::read(uint8_t *values, size_t bytesToRead,
                       size_t *bytesRead) {
  size_t bytesToCopy = bytesToRead;

  if (bytesToRead > byteSize_) {
    bytesToCopy = byteSize_;
  }

  memcpy(values, buffer_, bytesToCopy);
  *bytesRead = bytesToCopy;
  return 0;
}

int ecmcEcData::updateInputProcessImage() {
  if (direction_ != EC_DIR_INPUT) {
    return 0;
  }

  //memcpy(buffer_, adr_, byteSize_);
  
  // Read data from ethercat memory area
  switch(dataType_) {
    case ECMC_EC_B1:
      *uint8Ptr_ = Read1Bit(adr_, byteOffset_, bitOffset_);
      break;

    case ECMC_EC_B2:
      *uint8Ptr_ = Read2Bit(adr_, byteOffset_, bitOffset_);
      break;

    case ECMC_EC_B3:
      *uint8Ptr_ = Read3Bit(adr_, byteOffset_, bitOffset_);
      break;

    case ECMC_EC_B4:
      *uint8Ptr_ = Read4Bit(adr_, byteOffset_, bitOffset_);
      break;

    case ECMC_EC_U8:
      *uint8Ptr_ = ReadUInt8(adr_, byteOffset_, bitOffset_);
      break;

    case ECMC_EC_S8:
      *int8Ptr_ = ReadInt8(adr_, byteOffset_, bitOffset_);
      break;

    case ECMC_EC_U16:
      *uint16Ptr_ = ReadUInt16(adr_, byteOffset_, bitOffset_);
      break;
  
    case ECMC_EC_S16:
      *int16Ptr_ = ReadInt16(adr_, byteOffset_, bitOffset_);
      break;
  
    case ECMC_EC_U32:
      *uint32Ptr_ = ReadUInt32(adr_, byteOffset_, bitOffset_);
      break;
  
    case ECMC_EC_S32:
      *int32Ptr_ = ReadInt32(adr_, byteOffset_, bitOffset_);
      break;
  
    case ECMC_EC_U64:
      *uint64Ptr_ = ReadUInt64(adr_, byteOffset_, bitOffset_);
      break;
  
    case ECMC_EC_S64:
      *int64Ptr_ = ReadInt64(adr_, byteOffset_, bitOffset_);
      break;
  
    case ECMC_EC_F32:
      *float32Ptr_ = ReadFloat(adr_, byteOffset_, bitOffset_);
      break;
  
    case ECMC_EC_F64:
      *float64Ptr_ = ReadDouble(adr_, byteOffset_, bitOffset_);
      break;  
  }

  updateAsyn(0);
  return 0;
}

int ecmcEcData::updateOutProcessImage() {
  if (direction_ != EC_DIR_OUTPUT) {
    return 0;
  }

  //memcpy(adr_, buffer_, byteSize_);

  // Write data to ethercat memory area
  // No endians check...
  switch(dataType_) {
    case ECMC_EC_B1:
      Write1Bit(adr_, byteOffset_, bitOffset_,*uint8Ptr_);
      break;

    case ECMC_EC_B2:
      Write2Bit(adr_, byteOffset_, bitOffset_,*uint8Ptr_);
      break;

    case ECMC_EC_B3:
      Write3Bit(adr_, byteOffset_, bitOffset_,*uint8Ptr_);
      break;

    case ECMC_EC_B4:
      Write4Bit(adr_, byteOffset_, bitOffset_,*uint8Ptr_);
      break;

    case ECMC_EC_U8:
      WriteUInt8(adr_, byteOffset_, bitOffset_,*uint8Ptr_);
      break;

    case ECMC_EC_S8:
      WriteInt8(adr_, byteOffset_, bitOffset_,*int8Ptr_);
      break;

    case ECMC_EC_U16:
      WriteUInt16(adr_, byteOffset_, bitOffset_,*uint16Ptr_);
      break;
  
    case ECMC_EC_S16:
      WriteInt16(adr_, byteOffset_, bitOffset_,*int16Ptr_);
      break;
  
    case ECMC_EC_U32:
      WriteUInt32(adr_, byteOffset_, bitOffset_,*uint32Ptr_);
      break;
  
    case ECMC_EC_S32:
      WriteInt32(adr_, byteOffset_, bitOffset_,*int32Ptr_);
      break;
  
    case ECMC_EC_U64:
      WriteUInt64(adr_, byteOffset_, bitOffset_,*uint64Ptr_);
      break;
  
    case ECMC_EC_S64:
      WriteInt64(adr_, byteOffset_, bitOffset_,*int64Ptr_);
      break;
  
    case ECMC_EC_F32:
      WriteFloat(adr_, byteOffset_, bitOffset_,*float32Ptr_);
      break;
  
    case ECMC_EC_F64:
      WriteDouble(adr_, byteOffset_, bitOffset_*float64Ptr_);
      break;  
  }

  return 0;
}

std::string ecmcEcData::getIdentificationName() {
  return idString_;
}

int ecmcEcData::setDomainSize() {
  domainSize_ = domain_->getSize();
  return 0;
}

int ecmcEcData::updateAsyn(bool force) {
  dataAsynParam_->refreshParamRT(force);
  return 0;
}

int ecmcEcData::validate() {
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
    if (getErrorID() != ERROR_ECDATAITEM_MAP_SIZE_OUT_OF_RANGE) {
      LOGERR(
        "%s/%s:%d: ERROR: MemMap %s: Byte size, including offset, exceeds domain size (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        idString_.c_str(),
        ERROR_ECDATAITEM_MAP_SIZE_OUT_OF_RANGE);
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_ECDATAITEM_MAP_SIZE_OUT_OF_RANGE);
  }
  adr_ = domainAdr_ + byteOffset_;
  return 0;
}

int ecmcEcData::initAsyn() {
  
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
      ERROR_ECDATAITEM_ASYN_VAR_BUFFER_OUT_OF_RANGE);
    return ERROR_ECDATAITEM_ASYN_VAR_BUFFER_OUT_OF_RANGE;
  }

  // Set default asyn type
  asynParamType asynDefaultType = asynParamInt32;
  switch(dataType_) {

    case ECMC_EC_NONE:
      asynDefaultType = asynParamNotDefined;
      break;
#ifdef ECMC_ASYN_ASYNPARAMINT64
    case ECMC_EC_U64:
      asynDefaultType = asynParamInt64;
      break;
#endif

#ifdef ECMC_ASYN_ASYNPARAMINT64
    case ECMC_EC_S64:
      asynDefaultType = asynParamInt64;
      break;
#endif
    case ECMC_EC_F32:
      asynDefaultType = asynParamFloat64;
      break;

    case ECMC_EC_F64:
      asynDefaultType = asynParamFloat64;
      break;

  default:
    asynDefaultType = asynParamInt32;
    break;
  }

  if(asynDefaultType == asynParamNotDefined) {
    LOGERR(
      "%s/%s:%d: ERROR: Asyn data type error.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }

  name = buffer;
  dataAsynParam_ = asynPortDriver_->addNewAvailParam(name,
                                                     asynDefaultType,  //default type
                                                     buffer_,
                                                     byteSize_,
                                                     dataType_,
                                                     0);
  if(!dataAsynParam_) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }

  //Add more supported types
  switch(dataType_) {
    case ECMC_EC_NONE:      
      break;

    default:
      dataAsynParam_->addSupportedAsynType(asynParamInt32);
      dataAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
      dataAsynParam_->addSupportedAsynType(asynParamFloat64);

    case ECMC_EC_U64:
      dataAsynParam_->addSupportedAsynType(asynParamInt32);
      dataAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
      dataAsynParam_->addSupportedAsynType(asynParamFloat64);
#ifdef ECMC_ASYN_ASYNPARAMINT64
      dataAsynParam_->addSupportedAsynType(asynParamInt64);
#endif //ECMC_ASYN_ASYNPARAMINT64
      break;

    case ECMC_EC_S64:
      dataAsynParam_->addSupportedAsynType(asynParamInt32);
      dataAsynParam_->addSupportedAsynType(asynParamUInt32Digital);      
      dataAsynParam_->addSupportedAsynType(asynParamFloat64);
#ifdef ECMC_ASYN_ASYNPARAMINT64
      dataAsynParam_->addSupportedAsynType(asynParamInt64);
#endif //ECMC_ASYN_ASYNPARAMINT64
      break;

    case ECMC_EC_F32:
      dataAsynParam_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 4;
      break;

    case ECMC_EC_F64:
      dataAsynParam_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 8;      
      break;
  }
  dataAsynParam_->refreshParam(1);
  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);

  return 0;
}

int ecmcEcData::getByteSize() {
  return byteSize_;
}

uint8_t* ecmcEcData::getBufferPointer() {
  return buffer_;
}

ecmcEcDataType ecmcEcData::getDataType() {
  return dataType_;
}

int ecmcEcData::getDoubleDataAtIndex(size_t index, double *data) {
  if(index >= elements_) {
    return ERROR_ECDATAITEM_INDEX_OUT_OF_RANGE;
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
    return ERROR_ECDATAITEM_INVALID_DATA_TYPE;
    break;
  }

  return 0;
}

int ecmcEcData::setDoubleDataAtIndex(size_t index, double data) {
  if(index >= elements_) {
    return ERROR_ECDATAITEM_INDEX_OUT_OF_RANGE;
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
    return ERROR_ECDATAITEM_INVALID_DATA_TYPE;
    break;
  }

  return 0;
}

size_t ecmcEcData::getElementCount() {
  return  elements_;
}

size_t ecmcEcData::getBytesPerElement() {
  return  bytesPerElement_;
}

// New
uint8_t ecmcEcData::Read1Bit(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    uint8_t* dataPtr = buffer + byteOffset;
    return (*dataPtr >> bitOffset) & 0x01;
}

void ecmcEcData::Write1Bit(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value) {
    uint8_t* dataPtr = buffer + byteOffset;
    *dataPtr = (*dataPtr & ~(0x01 << bitOffset)) | ((value & 0x01) << bitOffset);
}

uint8_t ecmcEcData::Read2Bits(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    uint8_t* dataPtr = buffer + byteOffset;
    return (*dataPtr >> bitOffset) & 0x03;
}

void ecmcEcData::Write2Bits(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value) {
    uint8_t* dataPtr = buffer + byteOffset;
    *dataPtr = (*dataPtr & ~(0x03 << bitOffset)) | ((value & 0x03) << bitOffset);
}

uint8_t ecmcEcData::Read3Bits(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    uint8_t* dataPtr = buffer + byteOffset;
    return (*dataPtr >> bitOffset) & 0x07;
}

void ecmcEcData::Write3Bits(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value) {
    uint8_t* dataPtr = buffer + byteOffset;
    *dataPtr = (*dataPtr & ~(0x07 << bitOffset)) | ((value & 0x07) << bitOffset);
}

uint8_t ecmcEcData::Read4Bits(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    uint8_t* dataPtr = buffer + byteOffset;
    return (*dataPtr >> bitOffset) & 0x0F;
}

void ecmcEcData::Write4Bits(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value) {
    uint8_t* dataPtr = buffer + byteOffset;
    *dataPtr = (*dataPtr & ~(0x0F << bitOffset)) | ((value & 0x0F) << bitOffset);
}

uint8_t ecmcEcData::ReadUInt8(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    uint8_t* dataPtr = buffer + byteOffset;
    return (*dataPtr >> bitOffset) & 0xFF;
}

void ecmcEcData::WriteUInt8(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value) {
    uint8_t* dataPtr = buffer + byteOffset;
    *dataPtr = (*dataPtr & ~(0xFF << bitOffset)) | ((value & 0xFF) << bitOffset);
}

int8_t ecmcEcData::ReadInt8(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    return (int8_t)ReadUInt8(buffer, byteOffset, bitOffset);
}

void ecmcEcData::WriteInt8(uint8_t* buffer, size_t byteOffset, int bitOffset, int8_t value) {
    WriteUInt8(buffer, byteOffset, bitOffset, (uint8_t)value);
}

uint16_t ecmcEcData::ReadUInt16(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    uint16_t* dataPtr = (uint16_t*)(buffer + byteOffset);
    return (*dataPtr >> bitOffset) & 0xFFFF;
}

void ecmcEcData::WriteUInt16(uint8_t* buffer, size_t byteOffset, int bitOffset, uint16_t value) {
    uint16_t* dataPtr = (uint16_t*)(buffer + byteOffset);
    *dataPtr = (*dataPtr & ~(0xFFFF << bitOffset)) | ((value & 0xFFFF) << bitOffset);
}

int16_t ecmcEcData::ReadInt16(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    return (int16_t)ReadUInt16(buffer, byteOffset, bitOffset);
}

void ecmcEcData::WriteInt16(uint8_t* buffer, size_t byteOffset, int bitOffset, int16_t value) {
    WriteUInt16(buffer, byteOffset, bitOffset, (uint16_t)value);
}

uint32_t ecmcEcData::ReadUInt32(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    uint32_t* dataPtr = (uint32_t*)(buffer + byteOffset);
    return (*dataPtr >> bitOffset) & 0xFFFFFFFF;
}

void ecmcEcData::WriteUInt32(uint8_t* buffer, size_t byteOffset, int bitOffset, uint32_t value) {
    uint32_t* dataPtr = (uint32_t*)(buffer + byteOffset);
    *dataPtr = (*dataPtr & ~(0xFFFFFFFF << bitOffset)) | ((value & 0xFFFFFFFF) << bitOffset);
}

int32_t ecmcEcData::ReadInt32(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    return (int32_t)ReadUInt32(buffer, byteOffset, bitOffset);
}

void ecmcEcData::WriteInt32(uint8_t* buffer, size_t byteOffset, int bitOffset, int32_t value) {
    WriteUInt32(buffer, byteOffset, bitOffset, (uint32_t)value);
}

uint64_t ecmcEcData::ReadUInt64(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    uint64_t* dataPtr = (uint64_t*)(buffer + byteOffset);
    return (*dataPtr >> bitOffset) & 0xFFFFFFFFFFFFFFFF;
}

void ecmcEcData::WriteUInt64(uint8_t* buffer, size_t byteOffset, int bitOffset, uint64_t value) {
    uint64_t* dataPtr = (uint64_t*)(buffer + byteOffset);
    *dataPtr = (*dataPtr & ~(0xFFFFFFFFFFFFFFFFULL << bitOffset)) | ((value & 0xFFFFFFFFFFFFFFFFULL) << bitOffset);
}

int64_t ecmcEcData::ReadInt64(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    return (int64_t)ReadUInt64(buffer, byteOffset, bitOffset);
}

void ecmcEcData::WriteInt64(uint8_t* buffer, size_t byteOffset, int bitOffset, int64_t value) {
    WriteUInt64(buffer, byteOffset, bitOffset, (uint64_t)value);
}

float ecmcEcData::ReadFloat(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    uint32_t bits = ReadUInt32(buffer, byteOffset, bitOffset);
    return *(float*)&bits;
}

void ecmcEcData::WriteFloat(uint8_t* buffer, size_t byteOffset, int bitOffset, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(float));
    WriteUInt32(buffer, byteOffset, bitOffset, bits);
}

double ecmcEcData::ReadDouble(uint8_t* buffer, size_t byteOffset, int bitOffset) {
    uint64_t bits = ReadUInt64(buffer, byteOffset, bitOffset);
    return *(double*)&bits;
}

void ecmcEcData::WriteDouble(uint8_t* buffer, size_t byteOffset, int bitOffset, double value) {
    uint64_t bits;
    memcpy(&bits, &value, sizeof(double));
    WriteUInt64(buffer, byteOffset, bitOffset, bits);
}
