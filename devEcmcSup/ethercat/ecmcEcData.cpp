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
                       int                 masterId,
                       int                 slaveId,
                       ecmcEcEntry        *startEntry,
                       size_t              entryByteOffset,
                       size_t              entryBitOffset,
                       ec_direction_t      nDirection,
                       ecmcEcDataType      dt,
                       std::string         id) :
  ecmcEcEntry(asynPortDriver,
              masterId,
              slaveId,
              NULL,
              dt,
              id) {
  initVars();
  startEntry_      = startEntry;
  domain_          = startEntry_->getDomain();
  direction_       = nDirection;
  dataType_        = dt;
  entryByteOffset_ = entryByteOffset;
  entryBitOffset_  = entryBitOffset;
  byteSize_        = getEcDataTypeByteSize(dataType_);
}

void ecmcEcData::initVars() {
  startEntry_      = NULL;
  entryByteOffset_ = 0;
  entryBitOffset_  = 0;
  byteSize_        = 0;
  direction_       = EC_DIR_INVALID;
}

ecmcEcData::~ecmcEcData() {}

int ecmcEcData::updateInputProcessImage() {
  if (direction_ != EC_DIR_INPUT) {
    return 0;
  }

  buffer_ = 0;

  // Read data from ethercat memory area
  switch (dataType_) {
  case ECMC_EC_B1:
    *uint8Ptr_ = ecmcEcData::Read1Bit(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_B2:
    *uint8Ptr_ = ecmcEcData::Read2Bits(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_B3:
    *uint8Ptr_ = ecmcEcData::Read3Bits(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_B4:
    *uint8Ptr_ = ecmcEcData::Read4Bits(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_U8:
    *uint8Ptr_ = ecmcEcData::ReadUInt8(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_S8:
    *int8Ptr_ = ecmcEcData::ReadInt8(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_U16:
    *uint16Ptr_ = ecmcEcData::ReadUInt16(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_S16:
    *int16Ptr_ = ecmcEcData::ReadInt16(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_U32:
    *uint32Ptr_ = ecmcEcData::ReadUInt32(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_S32:
    *int32Ptr_ = ecmcEcData::ReadInt32(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_U64:
    *uint64Ptr_ = ecmcEcData::ReadUInt64(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_S64:
    *int64Ptr_ = ecmcEcData::ReadInt64(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_F32:
    *float32Ptr_ = ecmcEcData::ReadFloat(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_F64:
    *float64Ptr_ = ecmcEcData::ReadDouble(adr_, 0, entryBitOffset_);
    break;

  default:
    return ERROR_EC_ENTRY_DATATYPE_INVALID;
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
  switch (dataType_) {
  case ECMC_EC_B1:
    ecmcEcData::Write1Bit(adr_, 0, entryBitOffset_, *uint8Ptr_);
    break;

  case ECMC_EC_B2:
    ecmcEcData::Write2Bits(adr_, 0, entryBitOffset_, *uint8Ptr_);
    break;

  case ECMC_EC_B3:
    ecmcEcData::Write3Bits(adr_, 0, entryBitOffset_, *uint8Ptr_);
    break;

  case ECMC_EC_B4:
    ecmcEcData::Write4Bits(adr_, 0, entryBitOffset_, *uint8Ptr_);
    break;

  case ECMC_EC_U8:
    ecmcEcData::WriteUInt8(adr_, 0, entryBitOffset_, *uint8Ptr_);
    break;

  case ECMC_EC_S8:    
    ecmcEcData::WriteInt8(adr_, 0, entryBitOffset_, *int8Ptr_);
    break;

  case ECMC_EC_U16:
    ecmcEcData::WriteUInt16(adr_, 0, entryBitOffset_, *uint16Ptr_);
    break;

  case ECMC_EC_S16:
    ecmcEcData::WriteInt16(adr_, 0, entryBitOffset_, *int16Ptr_);
    break;

  case ECMC_EC_U32:
    ecmcEcData::WriteUInt32(adr_, 0, entryBitOffset_, *uint32Ptr_);
    break;

  case ECMC_EC_S32:
    ecmcEcData::WriteInt32(adr_, 0, entryBitOffset_, *int32Ptr_);
    break;

  case ECMC_EC_U64:
    ecmcEcData::WriteUInt64(adr_, 0, entryBitOffset_, *uint64Ptr_);
    break;

  case ECMC_EC_S64:
    ecmcEcData::WriteInt64(adr_, 0, entryBitOffset_, *int64Ptr_);
    break;

  case ECMC_EC_F32:
    ecmcEcData::WriteFloat(adr_, 0, entryBitOffset_, *float32Ptr_);
    break;

  case ECMC_EC_F64:
    ecmcEcData::WriteDouble(adr_, 0, entryBitOffset_, *float64Ptr_);
    break;

  default:
    return ERROR_EC_ENTRY_DATATYPE_INVALID;
  }

  updateAsyn(0);

  return 0;
}

int ecmcEcData::validate() {
  // offset to data from start of domain
  if (startEntry_ == NULL) {
    if (getErrorID() != ERROR_EC_ENTRY_INVALID_DOMAIN_ADR) {
      LOGERR("%s/%s:%d: ERROR: EcDataItem %s: Start entry invalid (0x%x).\n",
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

  if (startEntry_->getDirection() != direction_) {
    LOGERR(
      "%s/%s:%d: WARNING: EcDataItem %s: Data item direction is not same as for start ec-entry (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      idString_.c_str(),
      WARNING_DATA_ITEM_EC_ENTRY_DIR_MISSMATCH);
    setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               WARNING_DATA_ITEM_EC_ENTRY_DIR_MISSMATCH);
  }

  domainAdr_ = startEntry_->getDomainAdr();

  if (domainAdr_ == NULL) {
    if (getErrorID() != ERROR_EC_ENTRY_INVALID_DOMAIN_ADR) {
      LOGERR(
        "%s/%s:%d: ERROR: EcDataItem %s: Invalid domain address (0x%x).\n",
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

  // Set default to sim entry size
  size_t domainSize = 8;
  if(!startEntry_->getSimEntry()) {
    // never use getDomain for sim entry (NULL)
    domainSize = startEntry_->getDomain()->getSize();
  }
  byteOffset_ = startEntry_->getByteOffset() + entryByteOffset_;

  if (entryBitOffset_  > 7) {
    if (getErrorID() != ERROR_EC_ENTRY_INVALID_OFFSET) {
      LOGERR(
        "%s/%s:%d: ERROR: EcDataItem %s: Invalid data bit offset (0x%x).\n",
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
      LOGERR(
        "%s/%s:%d: ERROR: EcDataItem %s: Invalid domain address (0x%x).\n",
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

  // if bit-offset > 0 then one extra byte is needed
  int extraByteNeeded = entryBitOffset_ > 0;

  if (byteOffset_ + byteSize_ +  extraByteNeeded > domainSize) {
    if (getErrorID() != ERROR_EC_ENTRY_SIZE_OUT_OF_RANGE) {
      LOGERR(
        "%s/%s:%d: ERROR: EcDataItem %s: Byte size, including byte and bit offsets, exceeds domain size (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        idString_.c_str(),
        ERROR_EC_ENTRY_SIZE_OUT_OF_RANGE);
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_SIZE_OUT_OF_RANGE);
  }

  adr_ = domainAdr_ + byteOffset_;
  return 0;
}

// Utility functions
uint8_t ecmcEcData::Read1Bit(uint8_t *buffer, size_t byteOffset,
                             int bitOffset) {
  uint8_t *dataPtr = buffer + byteOffset;

  return (*dataPtr >> bitOffset) & 0x01;
}

void ecmcEcData::Write1Bit(uint8_t *buffer,
                           size_t   byteOffset,
                           int      bitOffset,
                           uint8_t  value) {
  uint8_t *dataPtr = buffer + byteOffset;

  *dataPtr = (*dataPtr & ~(0x01 << bitOffset)) | ((value & 0x01) << bitOffset);
}

uint8_t ecmcEcData::Read2Bits(uint8_t *buffer, size_t byteOffset,
                              int bitOffset) {
  uint8_t *dataPtr = buffer + byteOffset;

  return (*dataPtr >> bitOffset) & 0x03;
}

void ecmcEcData::Write2Bits(uint8_t *buffer,
                            size_t   byteOffset,
                            int      bitOffset,
                            uint8_t  value) {
  uint8_t *dataPtr = buffer + byteOffset;

  *dataPtr = (*dataPtr & ~(0x03 << bitOffset)) | ((value & 0x03) << bitOffset);
}

uint8_t ecmcEcData::Read3Bits(uint8_t *buffer, size_t byteOffset,
                              int bitOffset) {
  uint8_t *dataPtr = buffer + byteOffset;

  return (*dataPtr >> bitOffset) & 0x07;
}

void ecmcEcData::Write3Bits(uint8_t *buffer,
                            size_t   byteOffset,
                            int      bitOffset,
                            uint8_t  value) {
  uint8_t *dataPtr = buffer + byteOffset;

  *dataPtr = (*dataPtr & ~(0x07 << bitOffset)) | ((value & 0x07) << bitOffset);
}

uint8_t ecmcEcData::Read4Bits(uint8_t *buffer, size_t byteOffset,
                              int bitOffset) {
  uint8_t *dataPtr = buffer + byteOffset;

  return (*dataPtr >> bitOffset) & 0x0F;
}

void ecmcEcData::Write4Bits(uint8_t *buffer,
                            size_t   byteOffset,
                            int      bitOffset,
                            uint8_t  value) {
  uint8_t *dataPtr = buffer + byteOffset;

  *dataPtr = (*dataPtr & ~(0x0F << bitOffset)) | ((value & 0x0F) << bitOffset);
}

uint8_t ecmcEcData::ReadUInt8(uint8_t *buffer, size_t byteOffset,
                              int bitOffset) {
  uint8_t *dataPtr = buffer + byteOffset;

  return (*dataPtr >> bitOffset) & 0xFF;
}

void ecmcEcData::WriteUInt8(uint8_t *buffer,
                            size_t   byteOffset,
                            int      bitOffset,
                            uint8_t  value) {
  uint8_t *dataPtr = buffer + byteOffset;

  *dataPtr = (*dataPtr & ~(0xFF << bitOffset)) | ((value & 0xFF) << bitOffset);
}

int8_t ecmcEcData::ReadInt8(uint8_t *buffer, size_t byteOffset,
                            int bitOffset) {
  return (int8_t)ReadUInt8(buffer, byteOffset, bitOffset);
}

void ecmcEcData::WriteInt8(uint8_t *buffer,
                           size_t   byteOffset,
                           int      bitOffset,
                           int8_t   value) {
  WriteUInt8(buffer, byteOffset, bitOffset, (uint8_t)value);
}

uint16_t ecmcEcData::ReadUInt16(uint8_t *buffer,
                                size_t   byteOffset,
                                int      bitOffset) {
  uint16_t *dataPtr = (uint16_t *)(buffer + byteOffset);

  return (*dataPtr >> bitOffset) & 0xFFFF;
}

void ecmcEcData::WriteUInt16(uint8_t *buffer,
                             size_t   byteOffset,
                             int      bitOffset,
                             uint16_t value) {
  uint16_t *dataPtr = (uint16_t *)(buffer + byteOffset);

  *dataPtr =
    (*dataPtr & ~(0xFFFF << bitOffset)) | ((value & 0xFFFF) << bitOffset);
}

int16_t ecmcEcData::ReadInt16(uint8_t *buffer, size_t byteOffset,
                              int bitOffset) {
  return (int16_t)ReadUInt16(buffer, byteOffset, bitOffset);
}

void ecmcEcData::WriteInt16(uint8_t *buffer,
                            size_t   byteOffset,
                            int      bitOffset,
                            int16_t  value) {
  WriteUInt16(buffer, byteOffset, bitOffset, (uint16_t)value);
}

uint32_t ecmcEcData::ReadUInt32(uint8_t *buffer,
                                size_t   byteOffset,
                                int      bitOffset) {
  uint32_t *dataPtr = (uint32_t *)(buffer + byteOffset);

  return (*dataPtr >> bitOffset) & 0xFFFFFFFF;
}

void ecmcEcData::WriteUInt32(uint8_t *buffer,
                             size_t   byteOffset,
                             int      bitOffset,
                             uint32_t value) {
  uint32_t *dataPtr = (uint32_t *)(buffer + byteOffset);

  *dataPtr =
    (*dataPtr &
     ~(0xFFFFFFFF << bitOffset)) | ((value & 0xFFFFFFFF) << bitOffset);
}

int32_t ecmcEcData::ReadInt32(uint8_t *buffer, size_t byteOffset,
                              int bitOffset) {
  return (int32_t)ReadUInt32(buffer, byteOffset, bitOffset);
}

void ecmcEcData::WriteInt32(uint8_t *buffer,
                            size_t   byteOffset,
                            int      bitOffset,
                            int32_t  value) {
  WriteUInt32(buffer, byteOffset, bitOffset, (uint32_t)value);
}

uint64_t ecmcEcData::ReadUInt64(uint8_t *buffer,
                                size_t   byteOffset,
                                int      bitOffset) {
  uint64_t *dataPtr = (uint64_t *)(buffer + byteOffset);

  return (*dataPtr >> bitOffset) & 0xFFFFFFFFFFFFFFFF;
}

void ecmcEcData::WriteUInt64(uint8_t *buffer,
                             size_t   byteOffset,
                             int      bitOffset,
                             uint64_t value) {
  uint64_t *dataPtr = (uint64_t *)(buffer + byteOffset);

  *dataPtr =
    (*dataPtr &
     ~(0xFFFFFFFFFFFFFFFFULL <<
  bitOffset)) | ((value & 0xFFFFFFFFFFFFFFFFULL) << bitOffset);
}

int64_t ecmcEcData::ReadInt64(uint8_t *buffer, size_t byteOffset,
                              int bitOffset) {
  return (int64_t)ReadUInt64(buffer, byteOffset, bitOffset);
}

void ecmcEcData::WriteInt64(uint8_t *buffer,
                            size_t   byteOffset,
                            int      bitOffset,
                            int64_t  value) {
  WriteUInt64(buffer, byteOffset, bitOffset, (uint64_t)value);
}

float ecmcEcData::ReadFloat(uint8_t *buffer, size_t byteOffset,
                            int bitOffset) {
  uint32_t bits = ReadUInt32(buffer, byteOffset, bitOffset);

  return *(float *)&bits;
}

void ecmcEcData::WriteFloat(uint8_t *buffer,
                            size_t   byteOffset,
                            int      bitOffset,
                            float    value) {
  uint32_t bits;

  memcpy(&bits, &value, sizeof(float));
  WriteUInt32(buffer, byteOffset, bitOffset, bits);
}

double ecmcEcData::ReadDouble(uint8_t *buffer, size_t byteOffset,
                              int bitOffset) {
  uint64_t bits = ReadUInt64(buffer, byteOffset, bitOffset);

  return *(double *)&bits;
}

void ecmcEcData::WriteDouble(uint8_t *buffer,
                             size_t   byteOffset,
                             int      bitOffset,
                             double   value) {
  uint64_t bits;

  memcpy(&bits, &value, sizeof(double));
  WriteUInt64(buffer, byteOffset, bitOffset, bits);
}
