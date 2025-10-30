/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcEcEntry.cpp
*
*  Created on: Dec 2, 2015
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcEcEntry.h"
#include <stdlib.h>
#include "ecmcErrorsList.h"

#define EC_MASK_B2 0x03
#define EC_MASK_B3 0x07
#define EC_MASK_B4 0x0F

// Read 2 bits (lsb)
#define EC_READ_B2(DATA, POS) ((*((uint8_t *)(DATA)) >> (POS))&EC_MASK_B2)

// Read 3 bits (lsb)
#define EC_READ_B3(DATA, POS) ((*((uint8_t *)(DATA)) >> (POS))&EC_MASK_B3)

// Read 4 bits (lsb)
#define EC_READ_B4(DATA, POS) ((*((uint8_t *)(DATA)) >> (POS))&EC_MASK_B4)

// Write 2 bits (lsb)
#define EC_WRITE_B2(DATA, VAL)\
        do {\
          *((uint8_t *)(DATA)) &= ~EC_MASK_B2;\
          *((uint8_t *)(DATA)) |= (VAL &EC_MASK_B2);\
        } while (0)

#define EC_WRITE_B3(DATA, VAL)\
        do {\
          *((uint8_t *)(DATA)) &= ~EC_MASK_B3;\
          *((uint8_t *)(DATA)) |= (VAL &EC_MASK_B3);\
        } while (0)

#define EC_WRITE_B4(DATA, VAL)\
        do {\
          *((uint8_t *)(DATA)) &= ~EC_MASK_B4;\
          *((uint8_t *)(DATA)) |= (VAL &EC_MASK_B4);\
        } while (0)

ecmcEcEntry::ecmcEcEntry(ecmcAsynPortDriver *asynPortDriver,
                         int                 masterId,
                         int                 slaveId,
                         ecmcEcDomain       *domain,
                         ec_slave_config_t  *slave,
                         uint16_t            pdoIndex,
                         uint16_t            entryIndex,
                         uint8_t             entrySubIndex,
                         ec_direction_t      direction,
                         ecmcEcDataType      dt,
                         std::string         id,
                         int                 useInRealtime) {
  initVars();
  asynPortDriver_   = asynPortDriver;
  masterId_         = masterId;
  slaveId_          = slaveId;
  entryIndex_       = entryIndex;
  entrySubIndex_    = entrySubIndex;
  bitLength_        = getEcDataTypeBits(dt);
  direction_        = direction;
  sim_              = false;
  idString_         = id;
  idStringChar_     = strdup(idString_.c_str());
  domain_           = domain;
  pdoIndex_         = pdoIndex;
  slave_            = slave;
  dataType_         = dt;
  updateInRealTime_ = useInRealtime;
  int errorCode = ecrt_slave_config_pdo_mapping_add(slave,
                                                    pdoIndex_,
                                                    entryIndex_,
                                                    entrySubIndex_,
                                                    bitLength_);

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: ecrt_slave_config_pdo_mapping_add() failed with error code %d (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode,
      ERROR_EC_ENTRY_ASSIGN_ADD_FAIL);
    setErrorID(__FILE__, __FUNCTION__, __LINE__,
               ERROR_EC_ENTRY_ASSIGN_ADD_FAIL);
  }

  LOGINFO5(
    "INFO: Entry %s added: pdoIndex 0x%x, entryIndex 0x%x, entrySubIndex 0x%x, direction %d, bits %d.\n",
    idString_.c_str(),
    pdoIndex_,
    entryIndex_,
    entrySubIndex_,
    direction_,
    bitLength_);

  if (useInRealtime) {
    initAsyn();
  }
}

// Used for pure simulation entries
ecmcEcEntry::ecmcEcEntry(ecmcAsynPortDriver *asynPortDriver,
                         int                 masterId,
                         int                 slaveId,
                         uint8_t            *domainAdr,
                         ecmcEcDataType      dt,
                         std::string         id) {
  initVars();
  asynPortDriver_ = asynPortDriver;
  masterId_       = masterId;
  slaveId_        = slaveId;
  domainAdr_      = domainAdr;
  sim_            = true;
  direction_      = EC_DIR_OUTPUT;
  idString_       = id;
  idStringChar_   = strdup(idString_.c_str());
  adr_            = 0;
  dataType_       = dt;
  bitLength_      = getEcDataTypeBits(dt);
  initAsyn();
}

void ecmcEcEntry::initVars() {
  errorReset();
  asynPortDriver_   = NULL;
  masterId_         = -1;
  slaveId_          = -1;
  entryAsynParam_   = NULL;
  domainAdr_        = NULL;
  bitOffset_        = 0;
  byteOffset_       = 0;
  entryIndex_       = 0;
  entrySubIndex_    = 0;
  direction_        = EC_DIR_INVALID;
  sim_              = false;
  idString_         = "";
  idStringChar_     = NULL;
  asynPortDriver_   = NULL;
  updateInRealTime_ = 1;
  domain_           = NULL;
  pdoIndex_         = 0;
  slave_            = NULL;
  buffer_           = 0;
  dataType_         = ECMC_EC_NONE;
  bitLength_        = 0;
  int8Ptr_          = (int8_t *)&buffer_;
  uint8Ptr_         = (uint8_t *)&buffer_;
  int16Ptr_         = (int16_t *)&buffer_;
  uint16Ptr_        = (uint16_t *)&buffer_;
  int32Ptr_         = (int32_t *)&buffer_;
  uint32Ptr_        = (uint32_t *)&buffer_;
  int64Ptr_         = (int64_t *)&buffer_;
  uint64Ptr_        = (uint64_t *)&buffer_;
  float32Ptr_       = (float *)&buffer_;
  float64Ptr_       = (double *)&buffer_;
  usedSizeBytes_    = 0;
}

ecmcEcEntry::~ecmcEcEntry() {
  free(idStringChar_);
  idStringChar_ = NULL;
  delete entryAsynParam_;
  entryAsynParam_ = NULL;
}

void ecmcEcEntry::setDomainAdr() {
  domainAdr_ = domain_->getDataPtr();
}

uint16_t ecmcEcEntry::getEntryIndex() {
  return entryIndex_;
}

uint8_t ecmcEcEntry::getEntrySubIndex() {
  return entrySubIndex_;
}

int ecmcEcEntry::getEntryInfo(ec_pdo_entry_info_t *info) {
  if (info == NULL) {
    LOGERR(
      "%s/%s:%d: ERROR: Entry (0x%x:0x%x): output parameter pointer NULL (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      entryIndex_,
      entrySubIndex_,
      ERROR_EC_ENTRY_DATA_POINTER_NULL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_DATA_POINTER_NULL);
  }
  info->bit_length = bitLength_;
  info->index      = entryIndex_;
  info->subindex   = entrySubIndex_;
  return 0;
}

int ecmcEcEntry::getBits() {
  return bitLength_;
}

int ecmcEcEntry::writeValue(uint64_t value) {
  buffer_ = value;
  return updateAsyn(0);
}

int ecmcEcEntry::writeDouble(double value) {
  switch (dataType_) {
  case ECMC_EC_S8:
    *int8Ptr_ = (int8_t)value;
    break;

  case ECMC_EC_S16:
    *int16Ptr_ = (int16_t)value;
    break;

  case ECMC_EC_S32:
    *int32Ptr_ = (int32_t)value;
    break;

  case ECMC_EC_S64:
    *int64Ptr_ = (int64_t)value;
    break;

  case ECMC_EC_F32:
    *float32Ptr_ = (float)value;
    break;

  case ECMC_EC_F64:
    *float64Ptr_ = value;
    break;

  default:
    // All unsigned and bits
    buffer_ = (uint64_t)value;
    break;
  }

  return updateAsyn(0);
}

int ecmcEcEntry::writeValueForce(uint64_t value) {
  buffer_ = value;
  return updateAsyn(1);
}

int ecmcEcEntry::writeBitForce(int bitNumber, uint64_t value) {
  writeBit(bitNumber, value);
  return updateAsyn(1);
}

int ecmcEcEntry::writeBit(int bitNumber, uint64_t value) {
  if (value) {
    BIT_SET(buffer_, bitNumber);
  } else {
    BIT_CLEAR(buffer_, bitNumber);
  }

  return 0;
}

int ecmcEcEntry::readValue(uint64_t *value) {
  *value = buffer_;
  return 0;
}

int ecmcEcEntry::readDouble(double *value) {
  switch (dataType_) {
  case ECMC_EC_S8:
    *value = (double)(*int8Ptr_);
    break;

  case ECMC_EC_S16:
    *value = (double)(*int16Ptr_);
    break;

  case ECMC_EC_S32:
    *value = (double)(*int32Ptr_);
    break;

  case ECMC_EC_S64:
    *value = (double)(*int64Ptr_);
    break;

  case ECMC_EC_F32:

    *value = (double)(*float32Ptr_);
    break;

  case ECMC_EC_F64:
    *value = *float64Ptr_;
    break;

  default:
    // All unsigned and bits
    *value = (double)(*uint64Ptr_);
    break;
  }

  return 0;
}

int ecmcEcEntry::readBit(int bitNumber, uint64_t *value) {
  *value = BIT_CHECK(buffer_, bitNumber) > 0;
  return 0;
}

int ecmcEcEntry::updateInputProcessImage() {
  if (!updateInRealTime_) {
    return 0;
  }

  if (direction_ != EC_DIR_INPUT) {
    return 0;
  }

  buffer_ = 0;

  switch (dataType_) {
  case ECMC_EC_NONE:
    buffer_ = 0;
    break;

  case ECMC_EC_B1:
    buffer_ = (uint64_t)EC_READ_BIT(adr_, bitOffset_);
    break;

  case ECMC_EC_B2:
    buffer_ = (uint64_t)EC_READ_B2(adr_, bitOffset_);
    break;

  case ECMC_EC_B3:
    buffer_ = (uint64_t)EC_READ_B3(adr_, bitOffset_);
    break;

  case ECMC_EC_B4:
    buffer_ = (uint64_t)EC_READ_B4(adr_, bitOffset_);
    break;

  case ECMC_EC_U8:
    buffer_ = (uint64_t)EC_READ_U8(adr_);
    break;

  case ECMC_EC_S8:
    buffer_ = (uint64_t)EC_READ_S8(adr_);
    break;

  case ECMC_EC_U16:
    buffer_ = (uint64_t)EC_READ_U16(adr_);
    break;

  case ECMC_EC_S16:
    buffer_ = (uint64_t)EC_READ_S16(adr_);
    break;

  case ECMC_EC_U32:
    buffer_ = (uint64_t)EC_READ_U32(adr_);
    break;

  case ECMC_EC_S32:
    buffer_ = (uint64_t)EC_READ_S32(adr_);
    break;

#ifdef EC_READ_U64
  case ECMC_EC_U64:
    buffer_ = (uint64_t)EC_READ_U64(adr_);
    break;
#endif // ifdef EC_READ_U64

#ifdef EC_READ_S64
  case ECMC_EC_S64:
    buffer_ = (uint64_t)EC_READ_S64(adr_);
    break;
#endif // ifdef EC_READ_S64

#ifdef EC_READ_REAL
  case ECMC_EC_F32:
    *float32Ptr_ = EC_READ_REAL(adr_);
    break;
#endif // ifdef EC_READ_REAL

#ifdef EC_READ_LREAL
  case ECMC_EC_F64:
    *float64Ptr_ = EC_READ_LREAL(adr_);
    break;

#endif // ifdef EC_READ_LREAL
  default:
    buffer_ = 0;
    break;
  }

  updateAsyn(0);
  return 0;
}

int ecmcEcEntry::updateOutProcessImage() {
  if (!updateInRealTime_) {
    return 0;
  }

  if ((direction_ != EC_DIR_OUTPUT) && !sim_) {
    return 0;
  }

  switch (dataType_) {
  case ECMC_EC_NONE:
    buffer_ = 0;
    break;

  case ECMC_EC_B1:
    EC_WRITE_BIT(adr_, bitOffset_, buffer_);
    break;

  case ECMC_EC_B2:
    EC_WRITE_B2(adr_, buffer_);
    break;

  case ECMC_EC_B3:
    EC_WRITE_B3(adr_, buffer_);
    break;

  case ECMC_EC_B4:
    EC_WRITE_B4(adr_, buffer_);
    break;

  case ECMC_EC_U8:
    EC_WRITE_U8(adr_, buffer_);
    break;

  case ECMC_EC_S8:
    EC_WRITE_S8(adr_, buffer_);
    break;

  case ECMC_EC_U16:
    EC_WRITE_U16(adr_, buffer_);
    break;

  case ECMC_EC_S16:
    EC_WRITE_S16(adr_, buffer_);
    break;

  case ECMC_EC_U32:
    EC_WRITE_U32(adr_, buffer_);
    break;

  case ECMC_EC_S32:
    EC_WRITE_S32(adr_, buffer_);
    break;

#ifdef EC_WRITE_U64
  case ECMC_EC_U64:
    EC_WRITE_U64(adr_, buffer_);
    break;
#endif // ifdef EC_WRITE_U64

#ifdef EC_WRITE_S64
  case ECMC_EC_S64:
    EC_WRITE_S64(adr_, buffer_);
    break;
#endif // ifdef EC_WRITE_S64

#ifdef EC_WRITE_REAL
  case ECMC_EC_F32:
    EC_WRITE_REAL(adr_, *float32Ptr_);
    break;
#endif // ifdef EC_WRITE_REAL

#ifdef EC_WRITE_LREAL
  case ECMC_EC_F64:
    EC_WRITE_LREAL(adr_, *float64Ptr_);
    break;
#endif // ifdef EC_WRITE_LREAL

  default:
    buffer_ = 0;
    break;
  }

  updateAsyn(0);
  return 0;
}

std::string ecmcEcEntry::getIdentificationName() {
  return idString_;
}

int ecmcEcEntry::updateAsyn(bool force) {
  if (!entryAsynParam_) {
    return 0;
  }

  switch (entryAsynParam_->getAsynParameterType()) {
  case asynParamInt32:

    entryAsynParam_->refreshParamRT(force, (uint8_t *)&buffer_,
                                    usedSizeBytes_);
    break;

  case asynParamUInt32Digital:

    entryAsynParam_->refreshParamRT(force, (uint8_t *)&buffer_,
                                    usedSizeBytes_);
    break;

  case asynParamFloat64:

    entryAsynParam_->refreshParamRT(force, (uint8_t *)&buffer_,
                                    usedSizeBytes_);
    break;

#ifdef ECMC_ASYN_ASYNPARAMINT64

  case asynParamInt64:

    entryAsynParam_->refreshParamRT(force, (uint8_t *)&buffer_,
                                    usedSizeBytes_);
    break;
#endif //ECMC_ASYN_ASYNPARAMINT64

  default:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_ASYN_TYPE_NOT_SUPPORTED);

    break;
  }
  return 0;
}

int ecmcEcEntry::getByteOffset() {
  return byteOffset_;
}

uint8_t * ecmcEcEntry::getDomainAdr() {
  return domainAdr_;
}

int ecmcEcEntry::setUpdateInRealtime(int update) {
  updateInRealTime_ = update;
  return 0;
}

int ecmcEcEntry::getUpdateInRealtime() {
  return updateInRealTime_;
}

int ecmcEcEntry::compileRegInfo() {
  byteOffset_ = ecrt_slave_config_reg_pdo_entry(slave_,
                                                entryIndex_,
                                                entrySubIndex_,
                                                domain_->getDomain(),
                                                &bitOffset_);

  if (byteOffset_ < 0) {
    int errorCode = -byteOffset_;
    LOGERR(
      "%s/%s:%d: ERROR: ecrt_slave_config_reg_pdo_entry() failed with error code %d (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode,
      ERROR_EC_ENTRY_REGISTER_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_REGISTER_FAIL);
  }
  LOGINFO5(
    "%s/%s:%d: INFO: Entry %s registered in domain: bits %d, byteOffset %d, bitOffset %d.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    idString_.c_str(),
    bitLength_,
    byteOffset_,
    bitOffset_);
  return 0;
}

bool ecmcEcEntry::getSimEntry() {
  return sim_;
}

int ecmcEcEntry::validate() {
  if (byteOffset_ < 0) {
    LOGERR("%s/%s:%d: ERROR: Entry (0x%x:0x%x): Invalid data offset (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           entryIndex_,
           entrySubIndex_,
           ERROR_EC_ENTRY_INVALID_OFFSET);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_INVALID_OFFSET);
  }

  if (domainAdr_ == NULL) {
    LOGERR(
      "%s/%s:%d: ERROR: Entry (0x%x:0x%x): Invalid domain address (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      entryIndex_,
      entrySubIndex_,
      ERROR_EC_ENTRY_INVALID_DOMAIN_ADR);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_INVALID_DOMAIN_ADR);
  }

  if (dataType_ == ECMC_EC_NONE) {
    LOGERR(
      "%s/%s:%d: ERROR: Entry (0x%x:0x%x): Invalid data type (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      entryIndex_,
      entrySubIndex_,
      ERROR_EC_ENTRY_INVALID_BIT_LENGTH);

    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_INVALID_BIT_LENGTH);
  }

  // Calculate final address
  adr_ = domainAdr_ + byteOffset_;

  return 0;
}

int ecmcEcEntry::initAsyn() {
  char  buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char *name = buffer;

  // Simulation slave
  //if (slaveId_ < 0) {
  //  return 0;
  //}

  // "ec%d.s%d.alias"
  unsigned int charCount = snprintf(buffer,
                                    sizeof(buffer),
                                    ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d.%s",
                                    masterId_,
                                    slaveId_,
                                    idStringChar_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_ENTRY_REGISTER_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_REGISTER_FAIL);
  }
  name            = buffer;
  entryAsynParam_ = asynPortDriver_->addNewAvailParam(name,
                                                      asynParamInt32, // default type
                                                      (uint8_t *)&(buffer_),
                                                      sizeof(buffer_),
                                                      dataType_,
                                                      0);

  if (!entryAsynParam_) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MAIN_ASYN_CREATE_PARAM_FAIL);
  }

  // Add supported types
  switch (dataType_) {
  case ECMC_EC_NONE:
    return 0;

    break;

  case ECMC_EC_B1:
    entryAsynParam_->addSupportedAsynType(asynParamInt32);
    entryAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);
    usedSizeBytes_ = 1;
    break;

  case ECMC_EC_B2:
    entryAsynParam_->addSupportedAsynType(asynParamInt32);
    entryAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);
    usedSizeBytes_ = 1;
    break;

  case ECMC_EC_B3:
    entryAsynParam_->addSupportedAsynType(asynParamInt32);
    entryAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);
    usedSizeBytes_ = 1;
    break;

  case ECMC_EC_B4:
    entryAsynParam_->addSupportedAsynType(asynParamInt32);
    entryAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);
    usedSizeBytes_ = 1;
    break;

  case ECMC_EC_U8:
    entryAsynParam_->addSupportedAsynType(asynParamInt32);
    entryAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);
    usedSizeBytes_ = 1;
    break;

  case ECMC_EC_S8:
    entryAsynParam_->addSupportedAsynType(asynParamInt32);
    entryAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);
    usedSizeBytes_ = 1;
    break;

  case ECMC_EC_U16:
    entryAsynParam_->addSupportedAsynType(asynParamInt32);
    entryAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);
    usedSizeBytes_ = 2;
    break;

  case ECMC_EC_S16:
    entryAsynParam_->addSupportedAsynType(asynParamInt32);
    entryAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);
    usedSizeBytes_ = 2;
    break;

  case ECMC_EC_U32:
    entryAsynParam_->addSupportedAsynType(asynParamInt32);
    entryAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);
    usedSizeBytes_ = 4;
    break;

  case ECMC_EC_S32:
    entryAsynParam_->addSupportedAsynType(asynParamInt32);
    entryAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);
    usedSizeBytes_ = 4;
    break;

  case ECMC_EC_U64:
    entryAsynParam_->addSupportedAsynType(asynParamInt32);
    entryAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);

#ifdef ECMC_ASYN_ASYNPARAMINT64
    entryAsynParam_->addSupportedAsynType(asynParamInt64);
#endif //ECMC_ASYN_ASYNPARAMINT64

    usedSizeBytes_ = 8;
    break;

  case ECMC_EC_S64:
    entryAsynParam_->addSupportedAsynType(asynParamInt32);
    entryAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);

#ifdef ECMC_ASYN_ASYNPARAMINT64
    entryAsynParam_->addSupportedAsynType(asynParamInt64);
#endif //ECMC_ASYN_ASYNPARAMINT64

    usedSizeBytes_ = 8;
    break;

  case ECMC_EC_F32:
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);
    usedSizeBytes_ = 4;
    break;

  case ECMC_EC_F64:
    entryAsynParam_->addSupportedAsynType(asynParamFloat64);
    usedSizeBytes_ = 8;
    break;
  }
  entryAsynParam_->setEcmcDataSize(usedSizeBytes_);

  entryAsynParam_->setAllowWriteToEcmc(direction_ == EC_DIR_OUTPUT || sim_);
  entryAsynParam_->setEcmcBitCount(bitLength_);
  entryAsynParam_->setEcmcMinValueInt(getEcDataTypeMinVal(dataType_));
  entryAsynParam_->setEcmcMaxValueInt(getEcDataTypeMaxVal(dataType_));

  entryAsynParam_->refreshParam(1);
  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST,
                                      ECMC_ASYN_DEFAULT_ADDR);
  return 0;
}

int ecmcEcEntry::setComAlarm(bool alarm) {
  asynStatus stat;

  if (entryAsynParam_ == NULL) {
    return 0;
  }

  if (alarm) {
    stat = entryAsynParam_->setAlarmParam(COMM_ALARM, INVALID_ALARM);
  } else {
    stat = entryAsynParam_->setAlarmParam(NO_ALARM, NO_ALARM);
  }

  if (stat != asynSuccess) {
    return ERROR_EC_ENTRY_SET_ALARM_STATE_FAIL;
  }

  return 0;
}

ecmcEcDataType ecmcEcEntry::getDataType() {
  return dataType_;
}

int ecmcEcEntry::getSlaveId() {
  return slaveId_;
}

int ecmcEcEntry::activate() {
  setDomainAdr();
  return 0;
}

int ecmcEcEntry::getDomainOK() {
  // Simulation entries
  if (!domain_) {
    return 1;
  }
  return domain_->getOK();
}

ecmcEcDomain * ecmcEcEntry::getDomain() {
  return domain_;
}

ec_direction_t ecmcEcEntry::getDirection() {
  return direction_;
}

int ecmcEcEntry::writeBits(int startBitNumber, int bits,
                           uint64_t valueToWrite) {

  if (bits <= 0) return 0;

  if (startBitNumber < 0 || startBitNumber >= 64 || bits > 64 ||
      startBitNumber + bits > 64) {
    return 0;  // maybe consider throw error
  }
 
  // Create a mask with 'bits' ones (careful when bits == 64)
  const uint64_t lowMask = (bits == 64) ? ~uint64_t(0) : ((uint64_t(1) << bits) - 1u);
 
  // Keep only the relevant part of valueToWrite
  const uint64_t writePart = static_cast<uint64_t>(valueToWrite) & lowMask;
 
  // Shift mask to target position
  const uint64_t fieldMask = lowMask << static_cast<uint64_t>(startBitNumber);
 
  // Clear target field and OR in the new bits
  const uint64_t cleared   = static_cast<uint64_t>(buffer_) & ~fieldMask;
  const uint64_t result    = cleared | (writePart << static_cast<uint64_t>(startBitNumber));
  buffer_ =static_cast<uint64_t>(result);
  return 0;
}

int ecmcEcEntry::readBits(int startBitNumber, int bits, uint64_t *result) {
 
  if (bits <= 0) return 0;
 
  // Range check
  if (startBitNumber < 0 || startBitNumber >= 64 || bits > 64 ||
      startBitNumber + bits > 64) {
    // Handle invalid input: return 0 or assert/throw
    return 0;
  }
 
  // Mask with 'bits' ones
  const uint64_t lowMask = (bits == 64) ? ~uint64_t(0) : ((uint64_t(1) << bits) - 1u);
 
  // Shift down and mask
  *result = (buffer_ >> static_cast<uint64_t>(startBitNumber)) & lowMask;
 
  return 0;
}
