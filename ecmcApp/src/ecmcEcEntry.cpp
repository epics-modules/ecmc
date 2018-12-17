/*
 * ecmcEcEntry.cpp
 *
 *  Created on: Dec 2, 2015
 *      Author: anderssandstrom
 */

#include "ecmcEcEntry.h"

#define EC_READ_2_BITS(DATA, POS) ((*((uint8_t *) (DATA)) >> (POS)) & 0x03)
#define EC_READ_3_BITS(DATA, POS) ((*((uint8_t *) (DATA)) >> (POS)) & 0x07)
#define EC_READ_4_BITS(DATA, POS) ((*((uint8_t *) (DATA)) >> (POS)) & 0x0F)

ecmcEcEntry::ecmcEcEntry(ec_domain_t       *domain,
                         ec_slave_config_t *slave,
                         uint16_t           pdoIndex,
                         uint16_t           entryIndex,
                         uint8_t            entrySubIndex,
                         uint8_t            bits,
                         ec_direction_t     direction,
                         std::string        id) {
  initVars();
  entryIndex_    = entryIndex;
  entrySubIndex_ = entrySubIndex;
  bitLength_     = bits;
  direction_     = direction;
  sim_           = false;
  idString_      = id;
  domain_        = domain;
  pdoIndex_      = pdoIndex;
  slave_         = slave;
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
}

ecmcEcEntry::ecmcEcEntry(ec_domain_t       *domain,
                         ec_slave_config_t *slave,
                         uint16_t           pdoIndex,
                         uint16_t           entryIndex,
                         uint8_t            entrySubIndex,
                         uint8_t            bits,
                         ec_direction_t     direction,
                         std::string        id,
                         int                signedValue) {
  initVars();
  entryIndex_    = entryIndex;
  entrySubIndex_ = entrySubIndex;
  bitLength_     = bits;
  direction_     = direction;
  sim_           = false;
  idString_      = id;
  domain_        = domain;
  pdoIndex_      = pdoIndex;
  slave_         = slave;
  signedValue_   = signedValue;
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
}

ecmcEcEntry::ecmcEcEntry(uint8_t bits, uint8_t *domainAdr, std::string id) {
  initVars();
  domainAdr_ = domainAdr;
  bitLength_ = bits;
  sim_       = true;
  idString_  = id;
}

void ecmcEcEntry::initVars() {
  errorReset();
  domainAdr_              = NULL;
  bitLength_              = 0;
  bitOffset_              = 0;
  byteOffset_             = 0;
  entryIndex_             = 0;
  entrySubIndex_          = 0;
  value_                  = 0;
  direction_              = EC_DIR_INVALID;
  sim_                    = false;
  idString_               = "";
  asynParameterIndex_     = -1;
  asynParameterType_      = asynParamFloat64;
  asynPortDriver_         = NULL;
  asynUpdateCycles_       = 0;
  asynUpdateCycleCounter_ = 0;
  updateInRealTime_       = 1;
  domain_                 = NULL;
  pdoIndex_               = 0;
  slave_                  = NULL;
  signedValue_            = 0;
}

ecmcEcEntry::~ecmcEcEntry()
{}

void ecmcEcEntry::setDomainAdr(uint8_t *domainAdr) {
  domainAdr_ = domainAdr;
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
  value_ = value;
  return updateAsyn(0);
}

int ecmcEcEntry::writeValueForce(uint64_t value) {
  value_ = value;
  return updateAsyn(1);
}

int ecmcEcEntry::writeBit(int bitNumber, uint64_t value) {
  if (value) {
    BIT_SET(value_, bitNumber);
  } else {
    BIT_CLEAR(value_, bitNumber);
  }

  return 0;
}

int ecmcEcEntry::readValue(uint64_t *value) {
  *value = value_;
  return 0;
}

int ecmcEcEntry::readBit(int bitNumber, uint64_t *value) {
  *value = BIT_CHECK(value_, bitNumber) > 0;
  return 0;
}

int ecmcEcEntry::updateInputProcessImage() {
  if (!updateInRealTime_) {
    return 0;
  }

  if (direction_ != EC_DIR_INPUT  /*&& !sim_*/) {
    return 0;
  }

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

  if ((domainAdr_ < 0) || (domainAdr_ == NULL)) {
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

  switch (bitLength_) {
  case 1:
    value_ = (uint64_t)EC_READ_BIT(domainAdr_ + byteOffset_, bitOffset_);
    break;

  case 2:
    value_ = (uint64_t)EC_READ_2_BITS(domainAdr_ + byteOffset_, bitOffset_);
    break;

  case 3:
    value_ = (uint64_t)EC_READ_3_BITS(domainAdr_ + byteOffset_, bitOffset_);
    break;

  case 4:
    value_ = (uint64_t)EC_READ_4_BITS(domainAdr_ + byteOffset_, bitOffset_);
    break;

  case 8:
    value_ = (uint64_t)EC_READ_U8(domainAdr_ + byteOffset_);
    break;

  case 16:
    value_ = (uint64_t)EC_READ_U16(domainAdr_ + byteOffset_);
    break;

  case 32:
    value_ = (uint64_t)EC_READ_U32(domainAdr_ + byteOffset_);
    break;

  case 64:
    value_ = (uint64_t)EC_READ_U64(domainAdr_ + byteOffset_);
    break;

  default:

    if (ERROR_EC_ENTRY_INVALID_BIT_LENGTH != getErrorID()) {
      LOGERR(
        "%s/%s:%d: ERROR: Entry (0x%x:0x%x): Invalid bit length (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        entryIndex_,
        entrySubIndex_,
        ERROR_EC_ENTRY_INVALID_BIT_LENGTH);
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_INVALID_BIT_LENGTH);

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

  if ((domainAdr_ < 0) || (domainAdr_ == NULL)) {
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

  switch (bitLength_) {
  case 1:
    EC_WRITE_BIT(domainAdr_ + byteOffset_, bitOffset_, value_);
    break;

  case 8:
    EC_WRITE_U8(domainAdr_ + byteOffset_, value_);
    break;

  case 16:
    EC_WRITE_U16(domainAdr_ + byteOffset_, value_);
    break;

  case 32:
    EC_WRITE_U32(domainAdr_ + byteOffset_, value_);
    break;

  case 64:
    EC_WRITE_U64(domainAdr_ + byteOffset_, value_);
    break;

  default:

    if (ERROR_EC_ENTRY_INVALID_BIT_LENGTH != getErrorID()) {
      LOGERR(
        "%s/%s:%d: ERROR: Entry (0x%x:0x%x): Invalid bit length (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        entryIndex_,
        entrySubIndex_,
        ERROR_EC_ENTRY_INVALID_BIT_LENGTH);
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_INVALID_BIT_LENGTH);

    break;
  }
  return 0;
}

std::string ecmcEcEntry::getIdentificationName() {
  return idString_;
}

int ecmcEcEntry::updateAsyn(bool force) {
  // I/O intr to EPICS
  if (!asynPortDriver_ || (asynParameterIndex_ < 0)) {
    return 0;
  }

  // Only update at desired samplerate
  if (asynPortDriver_->getAllowRtThreadCom() &&
      ((asynUpdateCycleCounter_ >= asynUpdateCycles_) || force)) {
    asynUpdateCycleCounter_ = 0;
    double *tempDouble64 = 0;

    switch (asynParameterType_) {
    case asynParamInt32:
      asynPortDriver_->setIntegerParam(asynParameterIndex_, ecValue2Int32());
      break;

    case asynParamUInt32Digital:
      asynPortDriver_->setUIntDigitalParam(asynParameterIndex_,
                                           ecValue2Int32(),
                                           0xFFFFFFFF);
      break;

    case asynParamFloat64:
      tempDouble64 = reinterpret_cast<double *>(&value_);
      asynPortDriver_->setDoubleParam(asynParameterIndex_, *tempDouble64);
      break;

    default:
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_EC_ENTRY_ASYN_TYPE_NOT_SUPPORTED);

      break;
    }
  } else {
    asynUpdateCycleCounter_++;
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

int ecmcEcEntry::registerInDomain() {
  byteOffset_ = ecrt_slave_config_reg_pdo_entry(slave_,
                                                entryIndex_,
                                                entrySubIndex_,
                                                domain_,
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

int32_t ecmcEcEntry::ecValue2Int32() {
  int32_t tempInt32 = (int32_t)value_;

  if (signedValue_) {
    switch (bitLength_) {
    case 8:
      tempInt32 = (int32_t)((int8_t)value_);
      break;

    case 16:
      tempInt32 = (int32_t)((int16_t)value_);
      break;
    }
  }
  return tempInt32;
}

bool ecmcEcEntry::getSimEntry() {
  return sim_;
}
