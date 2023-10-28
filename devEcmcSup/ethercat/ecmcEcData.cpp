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
                       ecmcEcEntry *startEntry,
                       size_t entryByteOffset,
                       size_t entryBitOffset,
                       ec_direction_t nDirection,
                       ecmcEcDataType dt,
                       std::string id) : 
            ecmcEcEntry(asynPortDriver,
                        masterId,
                        slaveId,                                                  
                        NULL,
                        dt,
                        id) {
  initVars();
  startEntry_      = startEntry;
  direction_       = nDirection;
  entryByteOffset_ = 0;
  entryBitOffset_  = 0;
}

void ecmcEcData::initVars() {
  startEntry_      = NULL;
  entryByteOffset_ = 0;
  entryBitOffset_  = 0;
  direction_       = EC_DIR_INVALID;
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
}

int ecmcEcData::updateInputProcessImage() {
  if (direction_ != EC_DIR_INPUT) {
    return 0;
  }
  
  // Read data from ethercat memory area
  switch(dataType_) {
    case ECMC_EC_B1:
      *uint8Ptr_ = Read1Bit(adr_, 0, bitOffset_);
      break;

    case ECMC_EC_B2:
      *uint8Ptr_ = Read2Bit(adr_, 0, bitOffset_);
      break;

    case ECMC_EC_B3:
      *uint8Ptr_ = Read3Bit(adr_, 0, bitOffset_);
      break;

    case ECMC_EC_B4:
      *uint8Ptr_ = Read4Bit(adr_, 0, bitOffset_);
      break;

    case ECMC_EC_U8:
      *uint8Ptr_ = ReadUInt8(adr_, 0, bitOffset_);
      break;

    case ECMC_EC_S8:
      *int8Ptr_ = ReadInt8(adr_, 0, bitOffset_);
      break;

    case ECMC_EC_U16:
      *uint16Ptr_ = ReadUInt16(adr_, 0, bitOffset_);
      break;
  
    case ECMC_EC_S16:
      *int16Ptr_ = ReadInt16(adr_, 0, bitOffset_);
      break;
  
    case ECMC_EC_U32:
      *uint32Ptr_ = ReadUInt32(adr_, 0, bitOffset_);
      break;
  
    case ECMC_EC_S32:
      *int32Ptr_ = ReadInt32(adr_, 0, bitOffset_);
      break;
  
    case ECMC_EC_U64:
      *uint64Ptr_ = ReadUInt64(adr_, 0, bitOffset_);
      break;
  
    case ECMC_EC_S64:
      *int64Ptr_ = ReadInt64(adr_, 0, bitOffset_);
      break;
  
    case ECMC_EC_F32:
      *float32Ptr_ = ReadFloat(adr_, 0, bitOffset_);
      break;
  
    case ECMC_EC_F64:
      *float64Ptr_ = ReadDouble(adr_, 0, bitOffset_);
      break;  
  }

  updateAsyn(0);
  return 0;
}

int ecmcEcData::updateOutProcessImage() {
  if (direction_ != EC_DIR_OUTPUT) {
    return 0;
  }

  // Write data to ethercat memory area
  // No endians check...
  switch(dataType_) {
    case ECMC_EC_B1:
      Write1Bit(adr_, 0, bitOffset_,*uint8Ptr_);
      break;

    case ECMC_EC_B2:
      Write2Bit(adr_, 0, bitOffset_,*uint8Ptr_);
      break;

    case ECMC_EC_B3:
      Write3Bit(adr_, 0, bitOffset_,*uint8Ptr_);
      break;

    case ECMC_EC_B4:
      Write4Bit(adr_, 0, bitOffset_,*uint8Ptr_);
      break;

    case ECMC_EC_U8:
      WriteUInt8(adr_, 0, bitOffset_,*uint8Ptr_);
      break;

    case ECMC_EC_S8:
      WriteInt8(adr_, 0, bitOffset_,*int8Ptr_);
      break;

    case ECMC_EC_U16:
      WriteUInt16(adr_, 0, bitOffset_,*uint16Ptr_);
      break;
  
    case ECMC_EC_S16:
      WriteInt16(adr_, 0, bitOffset_,*int16Ptr_);
      break;
  
    case ECMC_EC_U32:
      WriteUInt32(adr_, 0, bitOffset_,*uint32Ptr_);
      break;
  
    case ECMC_EC_S32:
      WriteInt32(adr_, 0, bitOffset_,*int32Ptr_);
      break;
  
    case ECMC_EC_U64:
      WriteUInt64(adr_, 0, bitOffset_,*uint64Ptr_);
      break;
  
    case ECMC_EC_S64:
      WriteInt64(adr_, 0, bitOffset_,*int64Ptr_);
      break;
  
    case ECMC_EC_F32:
      WriteFloat(adr_, 0, bitOffset_,*float32Ptr_);
      break;
  
    case ECMC_EC_F64:
      WriteDouble(adr_, 0, bitOffset_,*float64Ptr_);
      break;  
  }

  return 0;
}

int ecmcEcData::validate() {
  // offset to data from start of domain
  byteOffset_ = startEntry_->getByteOffset() + entryByteOffset_;
  domainAdr_  = startEntry_->getDomainAdr();
 
  if (byteOffset_ < 0) {
    if (getErrorID() != ERROR_EC_ENTRY_INVALID_OFFSET) {
      LOGERR("%s/%s:%d: ERROR: EcDataItem %s: Invalid data byte offset (0x%x).\n",
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

 if (entryBitOffset_ < 0 || entryBitOffset_  > 7) {
    if (getErrorID() != ERROR_EC_ENTRY_INVALID_OFFSET) {
      LOGERR("%s/%s:%d: ERROR: EcDataItem %s: Invalid data bit offset (0x%x).\n",
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
      LOGERR("%s/%s:%d: ERROR: EcDataItem %s: Invalid domain address (0x%x).\n",
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

  if (byteOffset_ + byteSize_ + entryBitOffset_ / 8 > domainSize_) {
    if (getErrorID() != ERROR_MEM_MAP_SIZE_OUT_OF_RANGE) {
      LOGERR(
        "%s/%s:%d: ERROR: EcDataItem %s: Byte size, including byte and bit offsets, exceeds domain size (0x%x).\n",
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

// Utility functions
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
