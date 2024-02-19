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
    *uint8Ptr_ = ecmcEcData::read_1_bit_offset(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_B2:
    *uint8Ptr_ = ecmcEcData::read_2_bit_offset(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_B3:
    *uint8Ptr_ = ecmcEcData::read_3_bit_offset(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_B4:
    *uint8Ptr_ = ecmcEcData::read_4_bit_offset(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_U8:
    *uint8Ptr_ = ecmcEcData::read_uint8_offset(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_S8:
    *int8Ptr_ = ecmcEcData::read_int8_offset(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_U16:
    *uint16Ptr_ = ecmcEcData::read_uint16_offset(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_S16:
    *int16Ptr_ = ecmcEcData::read_int16_offset(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_U32:
    *uint32Ptr_ = ecmcEcData::read_uint32_offset(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_S32:
    *int32Ptr_ = ecmcEcData::read_int32_offset(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_U64:
    *uint64Ptr_ = ecmcEcData::read_uint64_offset(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_S64:
    *int64Ptr_ = ecmcEcData::read_int64_offset(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_F32:
    *float32Ptr_ = ecmcEcData::read_float_offset(adr_, 0, entryBitOffset_);
    break;

  case ECMC_EC_F64:
    *float64Ptr_ = ecmcEcData::read_double_offset(adr_, 0, entryBitOffset_);
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
    ecmcEcData::write_1_bit_offset(adr_, 0, entryBitOffset_, *uint8Ptr_);
    break;

  case ECMC_EC_B2:
    ecmcEcData::write_2_bit_offset(adr_, 0, entryBitOffset_, *uint8Ptr_);
    break;

  case ECMC_EC_B3:
    ecmcEcData::write_3_bit_offset(adr_, 0, entryBitOffset_, *uint8Ptr_);
    break;

  case ECMC_EC_B4:
    ecmcEcData::write_4_bit_offset(adr_, 0, entryBitOffset_, *uint8Ptr_);
    break;

  case ECMC_EC_U8:
    ecmcEcData::write_uint8_offset(adr_, 0, entryBitOffset_, *uint8Ptr_);
    break;

  case ECMC_EC_S8:
    ecmcEcData::write_int8_offset(adr_, 0, entryBitOffset_, *int8Ptr_);
    break;

  case ECMC_EC_U16:
    ecmcEcData::write_uint16_offset(adr_, 0, entryBitOffset_, *uint16Ptr_);
    break;

  case ECMC_EC_S16:
    ecmcEcData::write_int16_offset(adr_, 0, entryBitOffset_, *int16Ptr_);
    break;

  case ECMC_EC_U32:
    ecmcEcData::write_uint32_offset(adr_, 0, entryBitOffset_, *uint32Ptr_);
    break;

  case ECMC_EC_S32:
    ecmcEcData::write_int32_offset(adr_, 0, entryBitOffset_, *int32Ptr_);
    break;

  case ECMC_EC_U64:
    ecmcEcData::write_uint64_offset(adr_, 0, entryBitOffset_, *uint64Ptr_);
    break;

  case ECMC_EC_S64:
    ecmcEcData::write_int64_offset(adr_, 0, entryBitOffset_, *int64Ptr_);
    break;

  case ECMC_EC_F32:
    ecmcEcData::write_float_offset(adr_, 0, entryBitOffset_, *float32Ptr_);
    break;

  case ECMC_EC_F64:
    ecmcEcData::write_double_offset(adr_, 0, entryBitOffset_, *float64Ptr_);
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

  if (!startEntry_->getSimEntry()) {
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

// Read 1 bit from the buffer at a specific byte and bit offset
uint8_t ecmcEcData::read_1_bit_offset(uint8_t *buffer,
                                      int      byteOffset,
                                      int      bitOffset) {
  return (buffer[byteOffset] >> bitOffset) & 0x01;
}

// Write 1 bit to the buffer at a specific byte and bit offset
void ecmcEcData::write_1_bit_offset(uint8_t *buffer,
                                    int      byteOffset,
                                    int      bitOffset,
                                    uint8_t  value) {
  value             &= 0x01;
  buffer[byteOffset] =
    (buffer[byteOffset] & ~(0x01 << bitOffset)) | (value << bitOffset);
}

// Read 2 bits from the buffer at a specific byte and bit offset
uint8_t ecmcEcData::read_2_bit_offset(uint8_t *buffer,
                                      int      byteOffset,
                                      int      bitOffset) {
  uint8_t result = 0;

  if (bitOffset <= 6) {
    result = (buffer[byteOffset] >> bitOffset) & 0x03;
  } else {
    result =
      (buffer[byteOffset] >>
       bitOffset) | (buffer[byteOffset + 1] << (8 - bitOffset));
    result &= 0x03;
  }
  return result;
}

// Write 2 bits to the buffer at a specific byte and bit offset
void ecmcEcData::write_2_bit_offset(uint8_t *buffer,
                                    int      byteOffset,
                                    int      bitOffset,
                                    uint8_t  value) {
  value &= 0x03;

  if (bitOffset <= 6) {
    buffer[byteOffset] =
      (buffer[byteOffset] &
       ~((0x03 << bitOffset) & 0xFF)) | (value << bitOffset);
  } else {
    buffer[byteOffset] =
      (buffer[byteOffset] & ~(0xFF << bitOffset)) | (value << bitOffset);
    buffer[byteOffset +
           1] =
      (buffer[byteOffset + 1] &
       (0xFF << (2 - (8 - bitOffset)))) | (value >> (8 - bitOffset));
  }
}

// Read 3 bits from the buffer at a specific byte and bit offset
uint8_t ecmcEcData::read_3_bit_offset(uint8_t *buffer,
                                      int      byteOffset,
                                      int      bitOffset) {
  uint8_t result = 0;

  if (bitOffset <= 5) {
    result = (buffer[byteOffset] >> bitOffset) & 0x07;
  } else {
    result =
      (buffer[byteOffset] >>
       bitOffset) | (buffer[byteOffset + 1] << (8 - bitOffset));
    result &= 0x07;
  }
  return result;
}

// Write 3 bits to the buffer at a specific byte and bit offset
void ecmcEcData::write_3_bit_offset(uint8_t *buffer,
                                    int      byteOffset,
                                    int      bitOffset,
                                    uint8_t  value) {
  value &= 0x07;

  if (bitOffset <= 5) {
    buffer[byteOffset] =
      (buffer[byteOffset] &
       ~((0x07 << bitOffset) & 0xFF)) | (value << bitOffset);
  } else {
    buffer[byteOffset] =
      (buffer[byteOffset] & ~(0xFF << bitOffset)) | (value << bitOffset);
    buffer[byteOffset +
           1] =
      (buffer[byteOffset + 1] &
       (0xFF << (3 - (8 - bitOffset)))) | (value >> (8 - bitOffset));
  }
}

// Read 4 bits from the buffer at a specific byte and bit offset
uint8_t ecmcEcData::read_4_bit_offset(uint8_t *buffer,
                                      int      byteOffset,
                                      int      bitOffset) {
  uint8_t result = 0;

  if (bitOffset <= 4) {
    result = (buffer[byteOffset] >> bitOffset) & 0x0F;
  } else {
    result =
      (buffer[byteOffset] >>
       bitOffset) | (buffer[byteOffset + 1] << (8 - bitOffset));
    result &= 0x0F;
  }
  return result;
}

// Write 4 bits to the buffer at a specific byte and bit offset
void ecmcEcData::write_4_bit_offset(uint8_t *buffer,
                                    int      byteOffset,
                                    int      bitOffset,
                                    uint8_t  value) {
  value &= 0x0F;

  if (bitOffset <= 4) {
    buffer[byteOffset] =
      (buffer[byteOffset] &
       ~((0x0F << bitOffset) & 0xFF)) | (value << bitOffset);
  } else {
    buffer[byteOffset] =
      (buffer[byteOffset] & ~(0xFF << bitOffset)) | (value << bitOffset);
    buffer[byteOffset +
           1] =
      (buffer[byteOffset + 1] &
       (0xFF << (4 - (8 - bitOffset)))) | (value >> (8 - bitOffset));
  }
}

// Read a uint8_t from the buffer at a specific byte and bit offset
uint8_t ecmcEcData::read_uint8_offset(uint8_t *buffer,
                                      int      byteOffset,
                                      int      bitOffset) {
  if (bitOffset == 0) {
    return buffer[byteOffset];
  } else {
    uint8_t result =
      (buffer[byteOffset] >>
       bitOffset) | (buffer[byteOffset + 1] << (8 - bitOffset));
    return result;
  }
}

// Write a uint8_t to the buffer at a specific byte and bit offset
void ecmcEcData::write_uint8_offset(uint8_t *buffer,
                                    int      byteOffset,
                                    int      bitOffset,
                                    uint8_t  value) {
  if (bitOffset == 0) {
    buffer[byteOffset] = value;
  } else {
    buffer[byteOffset] =
      (buffer[byteOffset] & (0xFF >> (8 - bitOffset))) | (value << bitOffset);
    buffer[byteOffset +
           1] =
      (buffer[byteOffset + 1] &
       (0xFF << bitOffset)) | (value >> (8 - bitOffset));
  }
}

// Read an int8_t from the buffer at a specific byte and bit offset
int8_t ecmcEcData::read_int8_offset(uint8_t *buffer,
                                    int      byteOffset,
                                    int      bitOffset) {
  int8_t value = (int8_t)read_uint8_offset(buffer, byteOffset, bitOffset);

  return value;
}

// Write an int8_t to the buffer at a specific byte and bit offset
void ecmcEcData::write_int8_offset(uint8_t *buffer,
                                   int      byteOffset,
                                   int      bitOffset,
                                   int8_t   value) {
  write_uint8_offset(buffer, byteOffset, bitOffset, (uint8_t)value);
}

// Read a uint16_t from the buffer at a specific byte and bit offset
uint16_t ecmcEcData::read_uint16_offset(uint8_t *buffer,
                                        int      byteOffset,
                                        int      bitOffset) {
  if (bitOffset == 0) {
    uint16_t result;
    memcpy(&result, &buffer[byteOffset], 2);
    return result;
  } else {
    uint16_t result =
      (buffer[byteOffset] >>
       bitOffset) | (buffer[byteOffset + 1] << (8 - bitOffset));
    return result;
  }
}

// Write a uint16_t to the buffer at a specific byte and bit offset
void ecmcEcData::write_uint16_offset(uint8_t *buffer,
                                     int      byteOffset,
                                     int      bitOffset,
                                     uint16_t value) {
  if (bitOffset == 0) {
    memcpy(&buffer[byteOffset], &value, 2);
  } else {
    buffer[byteOffset] =
      (buffer[byteOffset] & (0xFF >> (8 - bitOffset))) | (value << bitOffset);
    buffer[byteOffset +
           1] =
      (buffer[byteOffset + 1] &
       (0xFF << bitOffset)) | (value >> (8 - bitOffset));
  }
}

// Read an int16_t from the buffer at a specific byte and bit offset
int16_t ecmcEcData::read_int16_offset(uint8_t *buffer,
                                      int      byteOffset,
                                      int      bitOffset) {
  int16_t value = (int16_t)read_uint16_offset(buffer, byteOffset, bitOffset);

  return value;
}

// Write an int16_t to the buffer at a specific byte and bit offset
void ecmcEcData::write_int16_offset(uint8_t *buffer,
                                    int      byteOffset,
                                    int      bitOffset,
                                    int16_t  value) {
  write_uint16_offset(buffer, byteOffset, bitOffset, (uint16_t)value);
}

// Read a uint32_t from the buffer at a specific byte and bit offset
uint32_t ecmcEcData::read_uint32_offset(uint8_t *buffer,
                                        int      byteOffset,
                                        int      bitOffset) {
  if (bitOffset == 0) {
    uint32_t result;
    memcpy(&result, &buffer[byteOffset], 4);
    return result;
  } else {
    uint32_t result =
      (buffer[byteOffset] >>
       bitOffset) |
      (buffer[byteOffset + 1] <<
        (8 - bitOffset)) |
      (buffer[byteOffset + 2] <<
        (16 - bitOffset)) | (buffer[byteOffset + 3] << (24 - bitOffset));
    return result;
  }
}

// Write a uint32_t to the buffer at a specific byte and bit offset
void ecmcEcData::write_uint32_offset(uint8_t *buffer,
                                     int      byteOffset,
                                     int      bitOffset,
                                     uint32_t value) {
  if (bitOffset == 0) {
    memcpy(&buffer[byteOffset], &value, 4);
  } else {
    buffer[byteOffset] =
      (buffer[byteOffset] & (0xFF >> (8 - bitOffset))) | (value << bitOffset);
    buffer[byteOffset +
           1] =
      (buffer[byteOffset + 1] &
       (0xFF << bitOffset)) | (value >> (8 - bitOffset));
    buffer[byteOffset +
           2] =
      (buffer[byteOffset + 2] &
       (0xFF << bitOffset)) | (value >> (16 - bitOffset));
    buffer[byteOffset +
           3] =
      (buffer[byteOffset + 3] &
       (0xFF << bitOffset)) | (value >> (24 - bitOffset));
  }
}

// Read an int32_t from the buffer at a specific byte and bit offset
int32_t ecmcEcData::read_int32_offset(uint8_t *buffer,
                                      int      byteOffset,
                                      int      bitOffset) {
  int32_t value = (int32_t)read_uint32_offset(buffer, byteOffset, bitOffset);

  return value;
}

// Write an int32_t to the buffer at a specific byte and bit offset
void ecmcEcData::write_int32_offset(uint8_t *buffer,
                                    int      byteOffset,
                                    int      bitOffset,
                                    int32_t  value) {
  write_uint32_offset(buffer, byteOffset, bitOffset, (uint32_t)value);
}

// Read a uint64_t from the buffer at a specific byte and bit offset
uint64_t ecmcEcData::read_uint64_offset(uint8_t *buffer,
                                        int      byteOffset,
                                        int      bitOffset) {
  if (bitOffset == 0) {
    uint64_t result;
    memcpy(&result, &buffer[byteOffset], 8);
    return result;
  } else {
    uint64_t result = (uint64_t)read_uint32_offset(buffer,
                                                   byteOffset,
                                                   bitOffset) |
                      ((uint64_t)read_uint32_offset(buffer, byteOffset + 4,
                                                    bitOffset) << 32);
    return result;
  }
}

// Write a uint64_t to the buffer at a specific byte and bit offset
void ecmcEcData::write_uint64_offset(uint8_t *buffer,
                                     int      byteOffset,
                                     int      bitOffset,
                                     uint64_t value) {
  if (bitOffset == 0) {
    memcpy(&buffer[byteOffset], &value, 8);
  } else {
    write_uint32_offset(buffer, byteOffset,     bitOffset, (uint32_t)value);
    write_uint32_offset(buffer, byteOffset + 4, bitOffset,
                        (uint32_t)(value >> 32));
  }
}

// Read an int64_t from the buffer at a specific byte and bit offset
int64_t ecmcEcData::read_int64_offset(uint8_t *buffer,
                                      int      byteOffset,
                                      int      bitOffset) {
  int64_t value = (int64_t)read_uint64_offset(buffer, byteOffset, bitOffset);

  return value;
}

// Write an int64_t to the buffer at a specific byte and bit offset
void ecmcEcData::write_int64_offset(uint8_t *buffer,
                                    int      byteOffset,
                                    int      bitOffset,
                                    int64_t  value) {
  write_uint64_offset(buffer, byteOffset, bitOffset, (uint64_t)value);
}

// Read a float from the buffer at a specific byte and bit offset
float ecmcEcData::read_float_offset(uint8_t *buffer,
                                    int      byteOffset,
                                    int      bitOffset) {
  uint32_t value = read_uint32_offset(buffer, byteOffset, bitOffset);

  return *((float *)&value);
}

// Write a float to the buffer at a specific byte and bit offset
void ecmcEcData::write_float_offset(uint8_t *buffer,
                                    int      byteOffset,
                                    int      bitOffset,
                                    float    value) {
  uint32_t intValue = *((uint32_t *)&value);

  write_uint32_offset(buffer, byteOffset, bitOffset, intValue);
}

// Read a double from the buffer at a specific byte and bit offset
double ecmcEcData::read_double_offset(uint8_t *buffer,
                                      int      byteOffset,
                                      int      bitOffset) {
  uint64_t value = read_uint64_offset(buffer, byteOffset, bitOffset);

  return *((double *)&value);
}

// Write a double to the buffer at a specific byte and bit offset
void ecmcEcData::write_double_offset(uint8_t *buffer,
                                     int      byteOffset,
                                     int      bitOffset,
                                     double   value) {
  uint64_t longValue = *((uint64_t *)&value);

  write_uint64_offset(buffer, byteOffset, bitOffset, longValue);
}
