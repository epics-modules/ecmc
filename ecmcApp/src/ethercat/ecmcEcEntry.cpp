/*
 * ecmcEcEntry.cpp
 *
 *  Created on: Dec 2, 2015
 *      Author: anderssandstrom
 */

#include "ecmcEcEntry.h"
#include <stdlib.h> 

#define EC_READ_2_BITS(DATA, POS) ((*((uint8_t *) (DATA)) >> (POS)) & 0x03)
#define EC_READ_3_BITS(DATA, POS) ((*((uint8_t *) (DATA)) >> (POS)) & 0x07)
#define EC_READ_4_BITS(DATA, POS) ((*((uint8_t *) (DATA)) >> (POS)) & 0x0F)

ecmcEcEntry::ecmcEcEntry(ecmcAsynPortDriver *asynPortDriver,
                         int masterId,
                         int slaveId,                         
                         ec_domain_t       *domain,
                         ec_slave_config_t *slave,
                         uint16_t           pdoIndex,
                         uint16_t           entryIndex,
                         uint8_t            entrySubIndex,
                         uint8_t            bits,
                         ec_direction_t     direction,
                         std::string        id) {
  initVars();
  asynPortDriver_ = asynPortDriver;
  masterId_=masterId;
  slaveId_=slaveId;
  entryIndex_    = entryIndex;
  entrySubIndex_ = entrySubIndex;
  bitLength_     = bits;
  direction_     = direction;
  sim_           = false;
  idString_      = id;
  idStringChar_  = strdup(idString_.c_str());
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
    initAsyn();
}

ecmcEcEntry::ecmcEcEntry(ecmcAsynPortDriver *asynPortDriver,
                        int masterId,
                        int slaveId,
                        ec_domain_t       *domain,
                        ec_slave_config_t *slave,
                        uint16_t           pdoIndex,
                        uint16_t           entryIndex,
                        uint8_t            entrySubIndex,
                        uint8_t            bits,
                        ec_direction_t     direction,
                        std::string        id,
                        int                signedValue) {
  initVars();
  asynPortDriver_ = asynPortDriver;
  masterId_=masterId;
  slaveId_=slaveId;
  entryIndex_    = entryIndex;
  entrySubIndex_ = entrySubIndex;
  bitLength_     = bits;
  direction_     = direction;
  sim_           = false;
  idString_      = id;
  idStringChar_  = strdup(idString_.c_str());
  domain_        = domain;
  pdoIndex_      = pdoIndex;
  slave_         = slave;
  signed_        = signedValue;
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

    initAsyn();
}

ecmcEcEntry::ecmcEcEntry(ecmcAsynPortDriver *asynPortDriver,
                         int masterId,
                         int slaveId,                         
                         uint8_t bits,
                         uint8_t *domainAdr,
                         std::string id) {
  initVars();
  asynPortDriver_ = asynPortDriver;
  masterId_=masterId;
  slaveId_=slaveId;
  domainAdr_ = domainAdr;
  bitLength_ = bits;
  sim_       = true;
  direction_ = EC_DIR_OUTPUT;
  idString_  = id;
  idStringChar_  = strdup(idString_.c_str());
  adr_       = 0;
  initAsyn();
}

void ecmcEcEntry::initVars() {
  errorReset();
  asynPortDriver_         = NULL;
  masterId_               = -1;
  slaveId_                = -1;
  entryAsynParam_         = NULL;
  domainAdr_              = NULL;
  bitLength_              = 0;
  bitOffset_              = 0;
  byteOffset_             = 0;
  entryIndex_             = 0;
  entrySubIndex_          = 0;  
  direction_              = EC_DIR_INVALID;
  sim_                    = false;
  idString_               = "";
  idStringChar_           = NULL;
  asynPortDriver_         = NULL;
  updateInRealTime_       = 1;
  domain_                 = NULL;
  pdoIndex_               = 0;
  slave_                  = NULL;
  signed_                 = 0;
  value_                  = 0;
}

ecmcEcEntry::~ecmcEcEntry()
{
  free(idStringChar_);
  idStringChar_ = NULL;
  delete entryAsynParam_;
  entryAsynParam_ = NULL;
}

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

  if (direction_ != EC_DIR_INPUT ) {
    return 0;
  }

  value_ = 0;

  switch (bitLength_) {
  case 1:          
    value_ = (uint64_t)EC_READ_BIT(adr_, bitOffset_);
    break;

  case 2:    
    value_ = (uint64_t)EC_READ_2_BITS(adr_, bitOffset_);
    break;

  case 3:    
    value_ = (uint64_t)EC_READ_3_BITS(adr_, bitOffset_);
    break;

  case 4:
    value_ = (uint64_t)EC_READ_4_BITS(adr_, bitOffset_);
    break;

  case 8:
    if(signed_){
      value_ = (uint64_t)EC_READ_S8(adr_);
    } else {
      value_ = (uint64_t)EC_READ_U8(adr_);
    }
    break;

  case 16:
    if(signed_){
      value_ = (uint64_t)EC_READ_S16(adr_);
    } else {
      value_ = (uint64_t)EC_READ_U16(adr_);
    }
    break;

  case 32:
    if(signed_){
      value_ = (uint64_t)EC_READ_S32(adr_);
    } else {
      value_ = (uint64_t)EC_READ_U32(adr_);
    }
    break;

  case 64:
    if(signed_){
      value_ = (uint64_t)EC_READ_S64(adr_);
    } else {
      value_ = (uint64_t)EC_READ_U64(adr_);
    }
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

  switch (bitLength_) {
  case 1:
    EC_WRITE_BIT(adr_, bitOffset_, value_);
    break;

  case 8:
    if(signed_){
      EC_WRITE_S8(adr_, value_);
    } else {
      EC_WRITE_U8(adr_, value_);
    }
    break;

  case 16:
    if(signed_){
      EC_WRITE_S16(adr_, value_);
    } else {
      EC_WRITE_U16(adr_, value_);
    }
    break;

  case 32:
    if(signed_){
      EC_WRITE_S32(adr_, value_);
    } else {
      EC_WRITE_U32(adr_, value_);
    }
    break;

  case 64:
    if(signed_){
      EC_WRITE_S64(adr_, value_);
    } else {
      EC_WRITE_U64(adr_, value_);
    }
    break;
  }
  return 0;
}

std::string ecmcEcEntry::getIdentificationName() {
  return idString_;
}

int ecmcEcEntry::updateAsyn(bool force) {

  if(!entryAsynParam_) {
    return 0;
  }

  switch (entryAsynParam_->getAsynParameterType()) {
    case asynParamInt32:
      
      //tempInt32=ecValue2Int32();
      //entryAsynParam_->refreshParamRT(force, (uint8_t *)&tempInt32, sizeof(tempInt32));
      entryAsynParam_->refreshParamRT(force, (uint8_t *)&value_, sizeof(value_));
      break;
    case asynParamUInt32Digital:
      //tempInt32=ecValue2Int32();
      entryAsynParam_->refreshParamRT(force, (uint8_t *)&value_, sizeof(value_));      
      break;

    case asynParamFloat64:
      //tempDouble64 = reinterpret_cast<double *>(&value_);
      //entryAsynParam_->refreshParamRT(force, (uint8_t *)tempDouble64, sizeof(double));            
      entryAsynParam_->refreshParamRT(force, (uint8_t *)&value_, sizeof(double));            
      break;

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

/*int32_t ecmcEcEntry::ecValue2Int32() {
  int32_t tempInt32 = (int32_t)value_;

  if (signed_) {
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
}*/

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
  adr_ = domainAdr_ + byteOffset_;

  switch (bitLength_) {
  case 1:
    break;

  case 2:      
    break;

  case 3:    
    break;

  case 4:    
    break;

  case 8:
    break;

  case 16:    
    break;

  case 32:    
    break;

  case 64:    
    break;

  default:    
    LOGERR(
      "%s/%s:%d: ERROR: Entry (0x%x:0x%x): Invalid bit length (0x%x).\n",
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

    break;
  }

  return 0;
}

int ecmcEcEntry::initAsyn() {
  
  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];  
  char *name = buffer;
  
  //Simulation slave
  if(slaveId_ < 0) {
    return 0;    
  }
  // "ec%d.s%d.alias"
  unsigned int charCount = snprintf(buffer,
                                    sizeof(buffer),
                                    ECMC_EC_STR"%d." ECMC_SLAVE_CHAR "%d.%s",
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
    return ERROR_EC_ENTRY_REGISTER_FAIL;
  }
  name = buffer;
  entryAsynParam_ = asynPortDriver_->addNewAvailParam(name,
                                    asynParamInt32,  //default type
                                    (uint8_t *)&(value_),
                                    sizeof(value_),
                                    0);
  if(!entryAsynParam_) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }

  //Add supported types  
  entryAsynParam_->addSupportedAsynType(asynParamInt32);
  entryAsynParam_->addSupportedAsynType(asynParamUInt32Digital);
  entryAsynParam_->addSupportedAsynType(asynParamFloat64);
  if(sim_) {
    entryAsynParam_->allowWriteToEcmc(1);
  }
  else {
    entryAsynParam_->allowWriteToEcmc(direction_ == EC_DIR_OUTPUT);
  }
  entryAsynParam_->setEcmcBitCount(bitLength_);
  if( signed_ ) {
    entryAsynParam_->setEcmcMinValueInt(-pow(2,bitLength_-1));
    entryAsynParam_->setEcmcMaxValueInt(pow(2,bitLength_-1)-1);
  } else {
    entryAsynParam_->setEcmcMinValueInt(0);
    entryAsynParam_->setEcmcMaxValueInt(pow(2,bitLength_)-1);
  }
  entryAsynParam_->refreshParam(1);
  asynPortDriver_->callParamCallbacks();
  return 0;
}