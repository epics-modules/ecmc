/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEc.cpp
*
*  Created on: Dec 1, 2015
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcEc.h"
#include <cmath>
#include <time.h>
#include <string>

ecmcEc::ecmcEc() {
  initVars();  
  setErrorID(ERROR_EC_STATUS_NOT_OK);
}

void ecmcEc::initVars() {
  errorReset();
  slaveCounter_                     = 0;
  initDone_                         = false;
  diag_                             = true;
  simSlave_                         = NULL;
  master_                           = NULL;
  domain_                           = NULL;
  domainPd_                         = 0;
  slavesOK_                         = 0;
  masterOK_                         = 0;
  domainOK_                         = 0;
  domainNotOKCounter_               = 0;
  domainNotOKCounterTotal_          = 0;
  domainNotOKCyclesLimit_           = 0;
  domainNotOKCounterMax_            = 0;
  masterAlStates_                   = 0;
  masterLinkUp_                     = 0;
  statusWordMaster_                 = 0;
  statusWordDomain_                 = 0;

  for (int i = 0; i < EC_MAX_SLAVES; i++) {
    slaveArray_[i] = NULL;
  }

  for (int i = 0; i < EC_MAX_ENTRIES; i++) {
    pdoByteOffsetArray_[i]           = 0;
    pdoBitOffsetArray_[i]            = 0;
    slaveEntriesReg_[i].alias        = 0;
    slaveEntriesReg_[i].bit_position = 0;
    slaveEntriesReg_[i].index        = 0;
    slaveEntriesReg_[i].offset       = 0;
    slaveEntriesReg_[i].position     = 0;
    slaveEntriesReg_[i].product_code = 0;
    slaveEntriesReg_[i].subindex     = 0;
    slaveEntriesReg_[i].vendor_id    = 0;
  }
  
  memset(&domainState_, 0, sizeof(domainState_));
  memset(&masterState_, 0, sizeof(masterState_));
  memset(&domainStateOld_,0,sizeof(domainStateOld_));
  memset(&masterStateOld_,0,sizeof(masterStateOld_));
  
  inStartupPhase_ = true;
  asynPortDriver_ = NULL;

  ecMemMapArrayCounter_ = 0;

  for (int i = 0; i < EC_MAX_MEM_MAPS; i++) {
    ecMemMapArray_[i] = NULL;
  }
  domainSize_        = 0;
  statusOutputEntry_ = NULL;
  masterIndex_       = -1;
  entryCounter_      = 0;

  for (int i = 0; i < ECMC_ASYN_EC_PAR_COUNT; i++) {
    ecAsynParams_[i]=NULL;
  }
  memset(&timeOffset_,0,sizeof(timeOffset_)); 
}

int ecmcEc::init(int nMasterIndex) {
  master_ = ecrt_request_master(nMasterIndex);

  if (!master_) {
    LOGERR("%s/%s:%d: ERROR: EtherCAT master request failed (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_EC_MAIN_REQUEST_FAILED);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_MAIN_REQUEST_FAILED);
  }

  domain_ = ecrt_master_create_domain(master_);

  if (!domain_) {
    LOGERR("%s/%s:%d: ERROR: EtherCAT create domain failed (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_EC_MAIN_CREATE_DOMAIN_FAILED);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_MAIN_CREATE_DOMAIN_FAILED);
  }
  initDone_    = true;
  masterIndex_ = nMasterIndex;

  return initAsyn(asynPortDriver_);
}

ecmcEc::~ecmcEc() {
  LOGINFO5("%s/%s:%d: INFO: Deleting Ec.\n", __FILE__, __FUNCTION__, __LINE__);

  for (int i = 0; i < slaveCounter_; i++) {
    delete slaveArray_[i];
    slaveArray_[i] = NULL;
  }

  if (simSlave_ != NULL) {
    delete simSlave_;
    simSlave_ = NULL;
  }

  for (int i = 0; i < EC_MAX_MEM_MAPS; i++) {
    delete ecMemMapArray_[i];
    ecMemMapArray_[i] = NULL;
  }

  for (int i = 0; i < ECMC_ASYN_EC_PAR_COUNT; i++) {
    delete ecAsynParams_[i];
    ecAsynParams_[i] = NULL;
  }  
}

bool ecmcEc::getInitDone() {
  return initDone_;
}

ec_domain_t * ecmcEc::getDomain() {
  return domain_;
}

ec_master_t * ecmcEc::getMaster() {
  return master_;
}

int ecmcEc::getMasterIndex() {
  return masterIndex_;
}

int ecmcEc::addSlave(
  uint16_t alias,  /**< Slave alias. */
  uint16_t position,  /**< Slave position. */
  uint32_t vendorId,  /**< Expected vendor ID. */
  uint32_t productCode  /**< Expected product code. */) {
  LOGINFO5(
    "%s/%s:%d: INFO: Adding EtherCAT slave (alias=%d, position=%d, vendorId=%x, productCode=%x).\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    alias,
    position,
    vendorId,
    productCode);

  if (slaveCounter_ < EC_MAX_SLAVES - 1) {
    slaveArray_[slaveCounter_] = new ecmcEcSlave(asynPortDriver_,
                                                 masterIndex_,
                                                 master_,
                                                 domain_,
                                                 alias,
                                                 position,
                                                 vendorId,
                                                 productCode);
    slaveCounter_++;

    ecAsynParams_[ECMC_ASYN_EC_PAR_SLAVE_COUNT_ID]->refreshParam(1);
    asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);

    return slaveCounter_ - 1;
  } else {
    LOGERR("%s/%s:%d: ERROR: Slave Array full (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_EC_MAIN_SLAVE_ARRAY_FULL);
    return -setErrorID(__FILE__,
                       __FUNCTION__,
                       __LINE__,
                       ERROR_EC_MAIN_SLAVE_ARRAY_FULL);
  }
}

ecmcEcSlave * ecmcEc::getSlave(int slaveIndex) {
  if ((slaveIndex >= EC_MAX_SLAVES) || (slaveIndex < -1) ||
      (slaveIndex >= slaveCounter_)) {
    LOGERR("%s/%s:%d: ERROR: Invalid slave index (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_EC_MAIN_INVALID_SLAVE_INDEX);
    setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_EC_MAIN_INVALID_SLAVE_INDEX);
    return NULL;
  }

  if (slaveIndex == -1) {
    return simSlave_;
  }

  if (slaveArray_[slaveIndex] == NULL) {
    LOGERR("%s/%s:%d: ERROR: Slave NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_EC_MAIN_SLAVE_NULL);
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_EC_MAIN_SLAVE_NULL);
    return NULL;
  }

  return slaveArray_[slaveIndex];
}

int ecmcEc::activate() {
  LOGINFO5("%s/%s:%d: INFO: Activating master...\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);

  if (ecrt_master_activate(master_)) {
    LOGERR("%s/%s:%d: ERROR: ecrt_master_activate() failed (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_EC_MAIN_MASTER_ACTIVATE_FAILED);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_MAIN_MASTER_ACTIVATE_FAILED);
  }

  if (!(domainPd_ = ecrt_domain_data(domain_))) {
    LOGERR("%s/%s:%d: ERROR: ecrt_domain_data() failed (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_EC_MAIN_DOMAIN_DATA_FAILED);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_MAIN_DOMAIN_DATA_FAILED);
  }

  LOGINFO5("%s/%s:%d: INFO: Writing process data offsets to entries.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);

  for (int slaveIndex = 0; slaveIndex < slaveCounter_; slaveIndex++) {
    if (slaveArray_[slaveIndex] == NULL) {
      LOGERR("%s/%s:%d: ERROR: Slave NULL (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             ERROR_EC_MAIN_SLAVE_NULL);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_EC_MAIN_SLAVE_NULL);
    }
    int nEntryCount = slaveArray_[slaveIndex]->getEntryCount();

    for (int entryIndex = 0; entryIndex < nEntryCount; entryIndex++) {
      ecmcEcEntry *tempEntry = slaveArray_[slaveIndex]->getEntry(entryIndex);

      if (tempEntry == NULL) {
        LOGERR("%s/%s:%d: ERROR: Entry NULL (0x%x).\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_EC_MAIN_ENTRY_NULL);
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_EC_MAIN_ENTRY_NULL);
      }

      if (!tempEntry->getSimEntry()) {
        tempEntry->setDomainAdr(domainPd_);
        LOGINFO5("%s/%s:%d: INFO: Entry %s (index = %d): domainAdr: %p.\n",
                 __FILE__,
                 __FUNCTION__,
                 __LINE__,
                 tempEntry->getIdentificationName().c_str(),
                 entryIndex,
                 domainPd_);
      }
    }
  }

  return validate();
}

int ecmcEc::compileRegInfo() {
  int entryCounter = 0;

  // Write offstes to entries
  for (int slaveIndex = 0; slaveIndex < slaveCounter_; slaveIndex++) {
    if (slaveArray_[slaveIndex] == NULL) {
      LOGERR("%s/%s:%d: ERROR: Slave NULL (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             ERROR_EC_MAIN_SLAVE_NULL);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_EC_MAIN_SLAVE_NULL);
    }
    int entryCountInSlave = slaveArray_[slaveIndex]->getEntryCount();

    for (int entryIndex = 0; entryIndex < entryCountInSlave; entryIndex++) {
      ecmcEcEntry *tempEntry = slaveArray_[slaveIndex]->getEntry(entryIndex);

      if (tempEntry == NULL) {
        LOGERR("%s/%s:%d: ERROR: Entry NULL (0x%x).\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_EC_MAIN_ENTRY_NULL);
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_EC_MAIN_ENTRY_NULL);
      }

      if (!tempEntry->getSimEntry()) {
        int ret = tempEntry->registerInDomain();

        if (ret) {
          LOGERR("%s/%s:%d: ERROR: register entry in domain failed (0x%x).\n",
                 __FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ret);
          return setErrorID(__FILE__, __FUNCTION__, __LINE__, ret);
        }
      }
      entryCounter++;
    }
  }

  // Set domain size to MemMap objects to avoid write outside memarea
  domainSize_ = ecrt_domain_size(domain_);

  for (int i = 0; i < ecMemMapArrayCounter_; i++) {
    if (ecMemMapArray_[i]) {
      ecMemMapArray_[i]->setDomainSize(domainSize_);
    }
  }

  return 0;
}

void ecmcEc::checkDomainState(void) {
  if (!diag_) {
    domainOK_ = true;
  }

  ecrt_domain_state(domain_, &domainState_);

  // filter domainOK_ for some cycles
  if (domainState_.wc_state != EC_WC_COMPLETE) {
    if (domainNotOKCounter_ <= domainNotOKCyclesLimit_) {
      domainNotOKCounter_++;
    }

    if (domainNotOKCounter_ > domainNotOKCounterMax_) {
      domainNotOKCounterMax_ = domainNotOKCounter_;
    }
    domainNotOKCounterTotal_++;
  } else {
    domainNotOKCounter_ = 0;
  }
  domainOK_ = domainNotOKCounter_ <= domainNotOKCyclesLimit_;
}

bool ecmcEc::checkSlavesConfState() {

  if (!diag_) {
    slavesOK_ = true;
    return slavesOK_;
  }
  
  bool localSlavesOK = true;
  int retVal = 0;

  for (int i = 0; i < slaveCounter_; i++) {
    retVal = checkSlaveConfState(i);
    if (retVal && !getErrorID()) {
      LOGERR(
        "%s/%s:%d: ERROR: Slave with bus position %d reports error (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        slaveArray_[i]->getSlaveBusPosition(),
        retVal);      
      setErrorID(__FILE__, __FUNCTION__, __LINE__, retVal);
    }
    if(retVal){
      localSlavesOK = false;
    }
  }

  if(localSlavesOK) {
    slavesOK_ = true;
  }

  return slavesOK_;
}

int ecmcEc::checkSlaveConfState(int slaveIndex) {
  if (!diag_) {
    return 0;
  }

  if ((slaveIndex >= EC_MAX_SLAVES) || (slaveIndex >= slaveCounter_)) {
    LOGERR("%s/%s:%d: ERROR: Invalid slave index (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_EC_MAIN_INVALID_SLAVE_INDEX);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_MAIN_INVALID_SLAVE_INDEX);
  }

  if (slaveArray_[slaveIndex] == NULL) {
    LOGERR("%s/%s:%d: ERROR: Slave NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_EC_MAIN_SLAVE_NULL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_MAIN_SLAVE_NULL);
  }

  return slaveArray_[slaveIndex]->checkConfigState();
}

bool ecmcEc::checkState(void) {
  if (!diag_) {
    masterOK_ = true;
    return true;
  }

  bool slavesUp = checkSlavesConfState();

  if (!slavesUp) {
    return slavesUp;
  }

  ecrt_master_state(master_, &masterState_);
 
  //Build master status word
  statusWordMaster_ = 0;
  // bit 0
  statusWordMaster_ = statusWordMaster_ + masterState_.link_up;
  // bit 1..4
  statusWordMaster_ = statusWordMaster_ + (masterState_.al_states << 1);
  // 16..31 
  statusWordMaster_ = statusWordMaster_ + ((uint16_t)(masterState_.slaves_responding) << 16);
  

  //Build domain status word
  statusWordDomain_ = 0;
  // bit 0
  statusWordDomain_ = statusWordDomain_ + (domainState_.redundancy_active > 0);
  // bit 1
  statusWordDomain_ = statusWordDomain_ + ((domainState_.wc_state ==  EC_WC_ZERO) << 1);
  // bit 2
  statusWordDomain_ = statusWordDomain_ + ((domainState_.wc_state ==  EC_WC_INCOMPLETE) << 2);
  // bit 3
  statusWordDomain_ = statusWordDomain_ + ((domainState_.wc_state ==  EC_WC_COMPLETE) << 3);
  // bit 16..31
  statusWordDomain_ = statusWordDomain_ + ((uint16_t)(domainState_.working_counter) << 16);
    
  if (masterState_.slaves_responding != masterStateOld_.slaves_responding) {
    LOGINFO5("%s/%s:%d: INFO: %u slave(s) responding.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             masterState_.slaves_responding);
  }

  if (masterState_.link_up != masterStateOld_.link_up) {
    LOGINFO5("%s/%s:%d: INFO: Master link is %s.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             masterState_.link_up ? "up" : "down");
  }
  masterLinkUp_=masterState_.link_up;

  if (masterState_.al_states != masterStateOld_.al_states) {
    LOGINFO5("%s/%s:%d: INFO: Application Layer state: 0x%x.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             masterState_.al_states);
  }
  masterAlStates_=masterState_.al_states;
  
  masterStateOld_ = masterState_;

  if (!masterState_.link_up) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_EC_LINK_DOWN);
    masterOK_ = false;
    return false;
  }

  if (static_cast<int>(masterState_.slaves_responding) < slaveCounter_) {
    if(getErrorID() != ERROR_EC_RESPOND_VS_CONFIG_SLAVES_MISSMATCH) {
      LOGERR(
        "%s/%s:%d: ERROR: Respondig slave count VS configures slave count missmatch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_EC_RESPOND_VS_CONFIG_SLAVES_MISSMATCH);
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_EC_RESPOND_VS_CONFIG_SLAVES_MISSMATCH);
    }
    masterOK_ = false;
    return false;
  }

  if (masterState_.al_states != EC_AL_STATE_OP) {
    switch (masterState_.al_states) {
    case EC_AL_STATE_INIT:
      setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_EC_AL_STATE_INIT);
      masterOK_ = false;
      return false;

      break;

    case EC_AL_STATE_PREOP:
      setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_EC_AL_STATE_PREOP);
      masterOK_ = false;
      return false;

      break;

    case EC_AL_STATE_SAFEOP:
      setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_EC_AL_STATE_SAFEOP);
      masterOK_ = false;
      return false;

      break;
    }
  }

  masterOK_ = true;

  return masterOK_;
}

void ecmcEc::receive() {
  ecrt_master_receive(master_);
  ecrt_domain_process(domain_);
  updateInputProcessImage();
}

void ecmcEc::send(timespec timeOffset) {
  struct timespec timeRel, timeAbs;
  timeOffset_=timeOffset;
  // Write status hardware status to output
  if (statusOutputEntry_) {
    statusOutputEntry_->writeValue((uint64_t)(getErrorID() == 0));
  }

  updateOutProcessImage();


  clock_gettime(CLOCK_MONOTONIC, &timeRel);
  timeAbs = timespecAdd(timeRel, timeOffset);

  ecrt_master_application_time(master_, TIMESPEC2NS(timeAbs));
  ecrt_master_sync_reference_clock(master_);
  ecrt_master_sync_slave_clocks(master_);
  
  ecrt_domain_queue(domain_);
  ecrt_master_send(master_);
}

int ecmcEc::setDiagnostics(bool bDiag) {
  diag_ = bDiag;
  return 0;
}

int ecmcEc::addSDOWrite(uint16_t slavePosition,
                        uint16_t sdoIndex,
                        uint8_t  sdoSubIndex,
                        uint32_t writeValue,
                        int      byteSize) {
  ecmcEcSlave *slave = findSlave(slavePosition);

  if (!slave) {
    LOGERR("%s/%s:%d: ERROR: Slave object NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_EC_MAIN_SLAVE_NULL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_MAIN_SLAVE_NULL);
  }

  return slave->addSDOWrite(sdoIndex, sdoSubIndex, writeValue, byteSize);
}

int ecmcEc::writeSDO(uint16_t slavePosition,
                     uint16_t sdoIndex,
                     uint8_t  sdoSubIndex,
                     uint32_t value,
                     int      byteSize) {
  return ecmcEcSDO::write(master_,
                          slavePosition,
                          sdoIndex,
                          sdoSubIndex,
                          value,
                          (size_t)byteSize);
}

int ecmcEc::writeSDOComplete(uint16_t slavePosition,
                             uint16_t sdoIndex,
                             uint32_t value,
                             int      byteSize) {
  return ecmcEcSDO::writeComplete(master_, slavePosition, sdoIndex, value,
                                  (size_t)byteSize);
}

int ecmcEc::readSDO(uint16_t  slavePosition,
                    uint16_t  sdoIndex,
                    uint8_t   sdoSubIndex,
                    int       byteSize,
                    uint32_t *value) {
  // uint32_t value=0;
  size_t bytesRead = 0;
  int    errorCode = ecmcEcSDO::read(master_,
                                     slavePosition,
                                     sdoIndex,
                                     sdoSubIndex,
                                     value,
                                     &bytesRead);

  if (errorCode) {
    return errorCode;
  }
  return 0;
}

int ecmcEc::readSoE(uint16_t  slavePosition, /**< Slave position. */
                    uint8_t   driveNo, /**< Drive number. */
                    uint16_t  idn, /**< SoE IDN (see ecrt_slave_config_idn()). */
                    size_t    byteSize, /**< Size of data to write. */
                    uint8_t  *value /**< Pointer to data to write. */
                   ){
    
  size_t bytesRead = 0;
  uint16_t soeError = 0;

  int errorCode = ecrt_master_read_idn(
        master_, /**< EtherCAT master. */
        slavePosition, /**< Slave position. */
        driveNo, /**< Drive number. */
        idn, /**< SoE IDN (see ecrt_slave_config_idn()). */
        value, /**< Pointer to memory where the read data can be stored. */
        byteSize, /**< Size of the memory \a target points to. */
        &bytesRead, /**< Actual size of the received data. */
        &soeError /**< Pointer to variable, where an SoE error code
                               can be stored. */
        );

  if(errorCode) {
    LOGERR("%s/%s:%d: ERROR: SoE read failed with SoE error code 0x%x (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        soeError,
        errorCode);
   }

   return errorCode;
}

int ecmcEc::writeSoE(uint16_t  slavePosition, /**< Slave position. */
                     uint8_t   driveNo, /**< Drive number. */
                     uint16_t  idn, /**< SoE IDN (see ecrt_slave_config_idn()). */
                     size_t    byteSize, /**< Size of data to write. */
                     uint8_t  *value /**< Pointer to data to write. */
                     ){

  uint16_t soeError = 0;

  int errorCode = ecrt_master_write_idn(
        master_, /**< EtherCAT master. */
        slavePosition, /**< Slave position. */
        driveNo, /**< Drive number. */
        idn, /**< SoE IDN (see ecrt_slave_config_idn()). */
        value, /**< Pointer to memory where the read data can be read. */
        byteSize, /**< Size of the memory \a target points to. */        
        &soeError /**< Pointer to variable, where an SoE error code
                               can be stored. */
        );

  if(errorCode) {
    LOGERR("%s/%s:%d: ERROR: SoE write failed with SoE error code 0x%x (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        soeError,
        errorCode);
   }

   return errorCode;
}

int ecmcEc::updateInputProcessImage() {
  for (int i = 0; i < slaveCounter_; i++) {
    if (slaveArray_[i] != NULL) {
      slaveArray_[i]->updateInputProcessImage();
    }
  }

  for (int i = 0; i < ecMemMapArrayCounter_; i++) {
    if (ecMemMapArray_[i] != NULL) {
      ecMemMapArray_[i]->updateInputProcessImage();
    }
  }

  return 0;
}

int ecmcEc::updateOutProcessImage() {
  for (int i = 0; i < slaveCounter_; i++) {
    if (slaveArray_[i] != NULL) {
      slaveArray_[i]->updateOutProcessImage();
    }
  }

  for (int i = 0; i < ecMemMapArrayCounter_; i++) {
    if (ecMemMapArray_[i] != NULL) {
      ecMemMapArray_[i]->updateOutProcessImage();
    }
  }

  // I/O intr to EPCIS.
  if (asynPortDriver_) {    
    ecAsynParams_[ECMC_ASYN_EC_PAR_SLAVE_COUNT_ID]->refreshParamRT(0);
    ecAsynParams_[ECMC_ASYN_EC_PAR_MASTER_STAT_ID]->refreshParamRT(0);
    ecAsynParams_[ECMC_ASYN_EC_PAR_DOMAIN_STAT_ID]->refreshParamRT(0);    
    ecAsynParams_[ECMC_ASYN_EC_PAR_DOMAIN_FAIL_COUNTER_TOT_ID]->refreshParamRT(0);    
    ecAsynParams_[ECMC_ASYN_EC_PAR_ENTRY_COUNT_ID]->refreshParamRT(0);    
    ecAsynParams_[ECMC_ASYN_EC_PAR_MEMMAP_COUNT_ID]->refreshParamRT(0);      
  }
  return 0;
}

int ecmcEc::addEntry(
  uint16_t       position,     // Slave position.
  uint32_t       vendorId,     // Expected vendor ID.
  uint32_t       productCode,  // Expected product code.
  ec_direction_t direction,
  uint8_t        syncMangerIndex,
  uint16_t       pdoIndex,
  uint16_t       entryIndex,
  uint8_t        entrySubIndex, 
  ecmcEcDataType dt,
  std::string    id,
  int            useInRealTime) 
  {

   // Ensure master can support datatype
   if(!validEntryType(dt)){
    LOGERR("%s/%s:%d: ERROR: Data type is not supported for current installed master. Please upgrade to newer ethercat master version (0x%x).\n",
         __FILE__,
         __FUNCTION__,
         __LINE__,
         ERROR_EC_DATATYPE_NOT_VALID);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                    ERROR_EC_DATATYPE_NOT_VALID);
  }

  ecmcEcSlave *slave = findSlave(position);

  if (slave == NULL) {
    int slaveIndex = addSlave(0, position, vendorId, productCode);

    if (slaveIndex < 0) {  // Error
      return -slaveIndex;
    }
    slave = slaveArray_[slaveIndex];  // last added slave
  }

  int errorCode = slave->addEntry(direction,
                                  syncMangerIndex,
                                  pdoIndex,
                                  entryIndex,
                                  entrySubIndex,
                                  dt,
                                  id,
                                  useInRealTime);

  if (errorCode) {
    return errorCode;
  }
  entryCounter_++;
  
  ecAsynParams_[ECMC_ASYN_EC_PAR_ENTRY_COUNT_ID]->refreshParam(1);
  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);

  return 0;
}

/*
* Check if valid entry type for the installed master
*
*/
bool ecmcEc::validEntryType(ecmcEcDataType dt) {
 switch(dt) {
    case ECMC_EC_NONE:
      return 1;
      break;

    case ECMC_EC_B1:
      return 1;
      break;

    case ECMC_EC_B2:
      return 1;
      break;

    case ECMC_EC_B3:
      return 1;
      break;

    case ECMC_EC_B4:
      return 1;
      break;

    case ECMC_EC_U8:
      return 1;
      break;

    case ECMC_EC_S8:
      return 1;
      break;

    case ECMC_EC_U16:
      return 1;
      break;

    case ECMC_EC_S16:
      return 1;
      break;

    case ECMC_EC_U32:
      return 1;
      break;

    case ECMC_EC_S32:
      return 1;
      break;
    case ECMC_EC_U64:
#ifdef EC_READ_U64
      return 1;
#else
      return 0;
#endif
      break;

    case ECMC_EC_S64:
#ifdef EC_READ_S64
      return 1;
#else
      return 0;
#endif
      break;

    case ECMC_EC_F32:
#ifdef EC_READ_F32
      return 1;
#else
      return 0;
#endif
      break;

    case ECMC_EC_F64:
#ifdef EC_READ_F64
      return 1;
#else
      return 0;
#endif
      break;

    default:
      return 0; 
      break;
  }
  return 0;
}

ecmcEcSlave * ecmcEc::findSlave(int busPosition) {
  if (busPosition == -1) {
    return simSlave_;
  }

  for (int i = 0; i < slaveCounter_; i++) {
    if (slaveArray_[i] != NULL) {
      if (slaveArray_[i]->getSlaveBusPosition() == busPosition) {
        return slaveArray_[i];
      }
    }
  }
  return NULL;
}

int ecmcEc::findSlaveIndex(int busPosition, int *slaveIndex) {
  if (busPosition == -1) {
    *slaveIndex = busPosition;
    return 0;
  }

  for (int i = 0; i < slaveCounter_; i++) {
    if (slaveArray_[i] != NULL) {
      if (slaveArray_[i]->getSlaveBusPosition() == busPosition) {
        *slaveIndex = i;
        return 0;
      }
    }
  }
  LOGERR("%s/%s:%d: ERROR: Slave NULL (0x%x).\n",
         __FILE__,
         __FUNCTION__,
         __LINE__,
         ERROR_EC_MAIN_SLAVE_NULL);
  return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                    ERROR_EC_MAIN_SLAVE_NULL);
}

int ecmcEc::statusOK() {
  if (!diag_) {
    return 1;
  }

  // Auto reset error at startup
  if (inStartupPhase_ && slavesOK_ && domainOK_ && masterOK_) {
    inStartupPhase_ = false;
    errorReset();
  }
  return slavesOK_ && domainOK_ && masterOK_;
}

int ecmcEc::setDomainFailedCyclesLimitInterlock(int cycles) {
  domainNotOKCyclesLimit_ = cycles;
  return 0;
}

void ecmcEc::slowExecute() {
  LOGINFO5(
    "%s/%s:%d: INFO: MasterOK: %d, SlavesOK: %d, DomainOK: %d, DomainNotOKCounter: %d, DomainNotOKLimit: %d, Error Code:0x%x .\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    masterOK_,
    slavesOK_,
    domainOK_,
    domainNotOKCounterMax_,
    domainNotOKCyclesLimit_,
    getErrorID());

  checkState();

  domainNotOKCounterMax_ = 0;
}

int ecmcEc::reset() {
  ecrt_master_reset(master_);
  return 0;
}

timespec ecmcEc::timespecAdd(timespec time1, timespec time2) {
  timespec result;

  if ((time1.tv_nsec + time2.tv_nsec) >= MCU_NSEC_PER_SEC) {
    result.tv_sec  = time1.tv_sec + time2.tv_sec + 1;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec - MCU_NSEC_PER_SEC;
  } else {
    result.tv_sec  = time1.tv_sec + time2.tv_sec;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
  }
  return result;
}

int ecmcEc::addMemMap(uint16_t       startEntryBusPosition,
                      std::string    startEntryIDString,
                      int            byteSize,                      
                      ec_direction_t direction,
                      ecmcEcDataType dt,
                      std::string    memMapIDString) {
  ecmcEcSlave *slave = findSlave(startEntryBusPosition);

  if (!slave) {
    LOGERR("%s/%s:%d: ERROR: Slave with busposition %d not found (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           startEntryBusPosition,
           ERROR_EC_MAIN_SLAVE_NULL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_MAIN_SLAVE_NULL);
  }

  if (ecMemMapArrayCounter_ >= EC_MAX_MEM_MAPS) {
    LOGERR("%s/%s:%d: ERROR: Adding ecMemMap failed. Array full (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_EC_MEM_MAP_INDEX_OUT_OF_RANGE);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_MEM_MAP_INDEX_OUT_OF_RANGE);
  }

  ecmcEcEntry *entry = slave->findEntry(startEntryIDString);

  if (!entry) {
    LOGERR(
      "%s/%s:%d: ERROR: Adding ecMemMap failed. Start entry not found (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_MEM_MAP_START_ENTRY_NULL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_MEM_MAP_START_ENTRY_NULL);
  }

  char alias[1024];
  std::string aliasString;
  int masterIndex = 0;
  int dummySlaveIndex = 0;
  int nvals       = sscanf(memMapIDString.c_str(),
                           "ec%d.s%d.mm.%s",
                           &masterIndex,
                           &dummySlaveIndex,
                           alias);

  if (nvals != 3) {
    LOGERR("%s/%s:%d: ERROR: Alias not found in idString %s (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           memMapIDString.c_str(),
           ERROR_EC_ASYN_ALIAS_NOT_VALID);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ASYN_ALIAS_NOT_VALID);
  }
  aliasString                           = alias;
  ecMemMapArray_[ecMemMapArrayCounter_] = new ecmcEcMemMap(asynPortDriver_,
                                                           masterIndex_,
                                                           startEntryBusPosition,                                                        
                                                           entry,
                                                           byteSize,
                                                           direction,
                                                           dt,
                                                           aliasString);

  if (!ecMemMapArray_[ecMemMapArrayCounter_]) {
    LOGERR(
      "%s/%s:%d: ERROR: Adding ecMemMap failed. New ecmcEcMemMap fail (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_MEM_MAP_NULL);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_EC_MEM_MAP_NULL);
  }

  ecMemMapArrayCounter_++;
  ecAsynParams_[ECMC_ASYN_EC_PAR_MEMMAP_COUNT_ID]->refreshParam(1);
  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);  // also for memmap and ecEntry

  return ecMemMapArray_[ecMemMapArrayCounter_-1]->getErrorID();
}

ecmcEcMemMap * ecmcEc::findMemMap(std::string id) {
  
  ecmcEcMemMap *temp = NULL;
  for (int i = 0; i < ecMemMapArrayCounter_; i++) {
    if (ecMemMapArray_[i]) {
      if (ecMemMapArray_[i]->getIdentificationName().compare(id) == 0) {
        temp = ecMemMapArray_[i];
      }
    }
  }
  return temp;
}

ecmcEcMemMap * ecmcEc::getMemMap(int index) {

  if(index<0 || index >= ecMemMapArrayCounter_) {
    return NULL;
  }
  
  return ecMemMapArray_[index];
}

int ecmcEc::setEcStatusOutputEntry(ecmcEcEntry *entry) {
  statusOutputEntry_ = entry;
  return 0;
}

int ecmcEc::setAsynPortDriver(ecmcAsynPortDriver *asynPortDriver) {
  asynPortDriver_ = asynPortDriver;
  simSlave_ = new ecmcEcSlave(asynPortDriver_,0 ,NULL, NULL,0, -1, 0, 0);  
  return 0;
}

int ecmcEc::initAsyn(ecmcAsynPortDriver *asynPortDriver) {
  
  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];

  // Master status
  unsigned int charCount = snprintf(buffer,
                                    sizeof(buffer),
                                    ECMC_EC_STR "%d." ECMC_ASYN_EC_PAR_MASTER_STAT_NAME,
                                    masterIndex_);
  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }
  char *name = buffer;
  ecmcAsynDataItem *paramTemp=NULL;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt32,
                                         (uint8_t *)&(statusWordMaster_),
                                         sizeof(statusWordMaster_),
                                         ECMC_EC_S32,
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->addSupportedAsynType(asynParamInt32);
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);    
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  ecAsynParams_[ECMC_ASYN_EC_PAR_MASTER_STAT_ID] = paramTemp;

  // Status word domain
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_ASYN_EC_PAR_DOMAIN_STAT_NAME,
                       masterIndex_);
  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt32,
                                         (uint8_t *)&(statusWordDomain_),
                                         sizeof(statusWordDomain_),
                                         ECMC_EC_U32,
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->addSupportedAsynType(asynParamInt32);
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);    
  paramTemp->allowWriteToEcmc(false);  
  paramTemp->refreshParam(1);
  ecAsynParams_[ECMC_ASYN_EC_PAR_DOMAIN_STAT_ID] = paramTemp;

  // Slave Counter
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_ASYN_EC_PAR_SLAVE_COUNT_NAME,
                       masterIndex_);
  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt32,
                                         (uint8_t *)&(slaveCounter_),
                                         sizeof(slaveCounter_),
                                         ECMC_EC_S32,
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->refreshParam(1);
  ecAsynParams_[ECMC_ASYN_EC_PAR_SLAVE_COUNT_ID] = paramTemp;

  // Mem map counter
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_ASYN_EC_PAR_MEMMAP_COUNT_NAME,
                       masterIndex_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt32,
                                         (uint8_t *)&(ecMemMapArrayCounter_),
                                         sizeof(ecMemMapArrayCounter_),
                                         ECMC_EC_S32,
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  ecAsynParams_[ECMC_ASYN_EC_PAR_MEMMAP_COUNT_ID] = paramTemp;

  // Domain fail counter total
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_ASYN_EC_PAR_DOMAIN_FAIL_COUNTER_TOT_NAME,
                       masterIndex_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt32,
                                         (uint8_t *)&(domainNotOKCounterTotal_),
                                         sizeof(domainNotOKCounterTotal_),
                                         ECMC_EC_S32,
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  ecAsynParams_[ECMC_ASYN_EC_PAR_DOMAIN_FAIL_COUNTER_TOT_ID] = paramTemp;

  // Entry counter
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_ASYN_EC_PAR_ENTRY_COUNT_NAME,
                       masterIndex_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt32,
                                         (uint8_t *)&(entryCounter_),
                                         sizeof(entryCounter_),
                                         ECMC_EC_S32,
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  ecAsynParams_[ECMC_ASYN_EC_PAR_ENTRY_COUNT_ID] = paramTemp;

  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);

  return 0;
}

int ecmcEc::printAllConfig() {
  ec_master_info_t masterInfo;
  ec_slave_info_t  slaveInfo;
  ec_sync_info_t   syncInfo;
  ec_pdo_info_t    pdoInfo;
  ec_pdo_entry_info_t pdoEntryInfo;
  int slaveLoopIndex   = 0;
  int syncManLoopIndex = 0;
  int pdoLoopIndex     = 0;
  int entryLoopIndex   = 0;
  int slaveCount       = 0;
  int syncManCount     = 0;
  int pdoCount         = 0;
  int entryCount       = 0;

  if (!master_) {
    LOGINFO("%s/%s:%d: INFO: No EtherCAT master selected.\n",
            __FILE__,
            __FUNCTION__,
            __LINE__);
    return 0;
  }

  int errorCode = ecrt_master(master_, &masterInfo);

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: Error: Function ecrt_master() failed with error code 0x%x.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode);
    return 0;
  }

  LOGINFO("ec%d.slaveCount=%d\n", masterIndex_, masterInfo.slave_count);

  // Slave loop
  slaveCount = masterInfo.slave_count;

  for (slaveLoopIndex = 0; slaveLoopIndex < slaveCount; slaveLoopIndex++) {
    errorCode = ecrt_master_get_slave(master_, slaveLoopIndex, &slaveInfo);

    if (errorCode) {
      LOGERR(
        "%s/%s:%d: Error: Function ecrt_master_get_slave() failed with error code 0x%x.\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        errorCode);
      return 0;
    }
    LOGINFO("ec%d.s%d.position=%d\n",
            masterIndex_,
            slaveLoopIndex,
            slaveInfo.position);
    LOGINFO("ec%d.s%d.syncManCount=%d\n",
            masterIndex_,
            slaveLoopIndex,
            slaveInfo.sync_count);
    LOGINFO("ec%d.s%d.alias=%d\n",
            masterIndex_,
            slaveLoopIndex,
            slaveInfo.alias);
    LOGINFO("ec%d.s%d.vendorId=0x%x\n",
            masterIndex_,
            slaveLoopIndex,
            slaveInfo.vendor_id);
    LOGINFO("ec%d.s%d.revisionNum=0x%x\n",
            masterIndex_,
            slaveLoopIndex,
            slaveInfo.revision_number);
    LOGINFO("ec%d.s%d.productCode=0x%x\n",
            masterIndex_,
            slaveLoopIndex,
            slaveInfo.product_code);
    LOGINFO("ec%d.s%d.serial_number=0x%x\n",
            masterIndex_,
            slaveLoopIndex,
            slaveInfo.serial_number);

    // Sync manager loop
    syncManCount = slaveInfo.sync_count;

    for (syncManLoopIndex = 0;
         syncManLoopIndex < syncManCount;
         syncManLoopIndex++) {
      errorCode = ecrt_master_get_sync_manager(master_,
                                               slaveLoopIndex,
                                               syncManLoopIndex,
                                               &syncInfo);

      if (errorCode) {
        LOGERR(
          "%s/%s:%d: Error: Function ecrt_master_get_sync_manager() failed with error code 0x%x.\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          errorCode);
        return 0;
      }
      LOGINFO("ec%d.s%d.sm%d.pdoCount=%d\n",
              masterIndex_,
              slaveLoopIndex,
              syncManLoopIndex,
              syncInfo.n_pdos);
      LOGINFO("ec%d.s%d.sm%d.direction=%s\n",
              masterIndex_,
              slaveLoopIndex,
              syncManLoopIndex,
              syncInfo.dir == 1 ? "Output" : "Input");
      LOGINFO("ec%d.s%d.sm%d.watchDogMode=%s\n",
              masterIndex_,
              slaveLoopIndex,
              syncManLoopIndex,
              syncInfo.watchdog_mode ==
              0 ? "Default" : (syncInfo.watchdog_mode ==
                               1 ? "Enabled" : "Disabled"));

      // Pdo loop
      pdoCount = syncInfo.n_pdos;

      for (pdoLoopIndex = 0; pdoLoopIndex < pdoCount; pdoLoopIndex++) {
        errorCode = ecrt_master_get_pdo(master_,
                                        slaveLoopIndex,
                                        syncManLoopIndex,
                                        pdoLoopIndex,
                                        &pdoInfo);

        if (errorCode) {
          LOGERR(
            "%s/%s:%d: Error: Function ecrt_master_get_pdo() failed with error code 0x%x.\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            errorCode);
          return 0;
        }
        LOGINFO("ec%d.s%d.sm%d.pdo%d.index=0x%x\n",
                masterIndex_,
                slaveLoopIndex,
                syncManLoopIndex,
                pdoLoopIndex,
                pdoInfo.index);
        LOGINFO("ec%d.s%d.sm%d.pdo%d.entryCount=%d\n",
                masterIndex_,
                slaveLoopIndex,
                syncManLoopIndex,
                pdoLoopIndex,
                pdoInfo.n_entries);

        // Entry loop
        entryCount = pdoInfo.n_entries;

        for (entryLoopIndex = 0; entryLoopIndex < entryCount;
             entryLoopIndex++) {
          errorCode = ecrt_master_get_pdo_entry(master_,
                                                slaveLoopIndex,
                                                syncManLoopIndex,
                                                pdoLoopIndex,
                                                entryLoopIndex,
                                                &pdoEntryInfo);

          if (errorCode) {
            LOGERR(
              "%s/%s:%d: Error: Function ecrt_master_get_pdo_entry() failed with error code 0x%x.\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              errorCode);
            return 0;
          }
          LOGINFO("ec%d.s%d.sm%d.pdo%d.e%d.index=0x%x\n",
                  masterIndex_,
                  slaveLoopIndex,
                  syncManLoopIndex,
                  pdoLoopIndex,
                  entryLoopIndex,
                  pdoEntryInfo.index);
          LOGINFO("ec%d.s%d.sm%d.pdo%d.e%d.subIndex=0x%x\n",
                  masterIndex_,
                  slaveLoopIndex,
                  syncManLoopIndex,
                  pdoLoopIndex,
                  entryLoopIndex,
                  pdoEntryInfo.subindex);
          LOGINFO("ec%d.s%d.sm%d.pdo%d.e%d.bitLength=0x%x\n",
                  masterIndex_,
                  slaveLoopIndex,
                  syncManLoopIndex,
                  pdoLoopIndex,
                  entryLoopIndex,
                  pdoEntryInfo.bit_length);
        }
      }
    }
  }

  return 0;
}

int ecmcEc::printSlaveConfig(int slaveIndex) {
  ec_master_info_t masterInfo;
  ec_slave_info_t  slaveInfo;
  ec_sync_info_t   syncInfo;
  ec_pdo_info_t    pdoInfo;
  ec_pdo_entry_info_t pdoEntryInfo;
  int syncManLoopIndex = 0;
  int pdoLoopIndex     = 0;
  int entryLoopIndex   = 0;
  int slaveCount       = 0;
  int syncManCount     = 0;
  int pdoCount         = 0;
  int entryCount       = 0;

  if (!master_) {
    LOGINFO("%s/%s:%d: INFO: No EtherCAT master selected.\n",
            __FILE__,
            __FUNCTION__,
            __LINE__);
    return 0;
  }

  int errorCode = ecrt_master(master_, &masterInfo);

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: Error: Function ecrt_master() failed with error code 0x%x.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode);
    return 0;
  }

  LOGINFO("ec%d.slaveCount=%d\n", masterIndex_, masterInfo.slave_count);

  // Slave loop
  slaveCount = masterInfo.slave_count;

  if (slaveIndex >= slaveCount) {
    LOGINFO("%s/%s:%d: INFO: Slave index out of range.\n",
            __FILE__,
            __FUNCTION__,
            __LINE__);
    return 0;
  }

  errorCode = ecrt_master_get_slave(master_, slaveIndex, &slaveInfo);

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: Error: Function ecrt_master_get_slave() failed with error code 0x%x.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode);
    return 0;
  }

  printf("#############################################\n");
  printf("# Auto generated configuration file\n");
  printf("# Slave information:\n");
  printf("#   position = %d \n",       slaveInfo.position);
  printf("#   alias = %d \n",          slaveInfo.alias);
  printf("#   vendor ID = %d \n",      slaveInfo.vendor_id);
  printf("#   product code = %d \n",   slaveInfo.product_code);
  printf("#   revison number = %d \n", slaveInfo.revision_number);
  printf("#   serial number = %d \n",  slaveInfo.serial_number);
  printf("#############################################\n");

  // Sync manager loop
  syncManCount = slaveInfo.sync_count;

  for (syncManLoopIndex = 0;
       syncManLoopIndex < syncManCount;
       syncManLoopIndex++) {
    errorCode = ecrt_master_get_sync_manager(master_,
                                             slaveIndex,
                                             syncManLoopIndex,
                                             &syncInfo);

    if (errorCode) {
      LOGERR(
        "%s/%s:%d: Error: Function ecrt_master_get_sync_manager() failed with error code 0x%x.\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        errorCode);
      return 0;
    }

    printf("# Configuration for sync manager %d\n", syncManLoopIndex);

    // Pdo loop
    pdoCount = syncInfo.n_pdos;

    for (pdoLoopIndex = 0; pdoLoopIndex < pdoCount; pdoLoopIndex++) {
      errorCode = ecrt_master_get_pdo(master_,
                                      slaveIndex,
                                      syncManLoopIndex,
                                      pdoLoopIndex,
                                      &pdoInfo);

      if (errorCode) {
        LOGERR(
          "%s/%s:%d: Error: Function ecrt_master_get_pdo() failed with error code 0x%x.\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          errorCode);
        return 0;
      }
      printf("# Configuration for pdoIndex 0x%x\n", pdoInfo.index);

      // Entry loop
      entryCount = pdoInfo.n_entries;

      for (entryLoopIndex = 0; entryLoopIndex < entryCount; entryLoopIndex++) {
        errorCode = ecrt_master_get_pdo_entry(master_,
                                              slaveIndex,
                                              syncManLoopIndex,
                                              pdoLoopIndex,
                                              entryLoopIndex,
                                              &pdoEntryInfo);

        if (errorCode) {
          LOGERR(
            "%s/%s:%d: Error: Function ecrt_master_get_pdo_entry() failed with error code 0x%x.\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            errorCode);
          return 0;
        }

        printf(ECMC_IOCSH_CFG_CMD" ");
        printf(
          "\"Cfg.EcAddEntryComplete(%d,0x%x,0x%x,%d,%d,0x%x,0x%x,0x%x,%d,sm%d.p%d.e%d)\"\n",
          slaveIndex,
          slaveInfo.vendor_id,
          slaveInfo.product_code,
          syncInfo.dir,
          syncManLoopIndex,
          pdoInfo.index,
          pdoEntryInfo.index,
          pdoEntryInfo.subindex,
          pdoEntryInfo.bit_length,
          syncManLoopIndex,
          pdoLoopIndex,
          entryLoopIndex);
      }
    }
    printf("#############################################\n");
  }
  return 0;
}

int ecmcEc::validate() {
  int errorCode = 0;
  
  // All real slaves
  for (int i = 0; i < slaveCounter_; i++) {
    if(slaveArray_[i]) {
      errorCode = slaveArray_[i]->validate();
      if (errorCode) {
        return errorCode;
      }
    }
  }

  // Simulation slave
  if (simSlave_) {
    errorCode = simSlave_->validate();
    if (errorCode) {
      return errorCode;
    }
  }

  // Mem maps
  for (int i = 0; i < ecMemMapArrayCounter_; i++) {
    if(ecMemMapArray_[i]) {
      errorCode = ecMemMapArray_[i]->validate();
      if (errorCode) {
        return errorCode;
      }
    }
  }

  return 0;
}

int ecmcEc::verifySlave(uint16_t alias,  /**< Slave alias. */
                        uint16_t slavePos,   /**< Slave position. */
                        uint32_t vendorId,   /**< Expected vendor ID. */
                        uint32_t productCode  /**< Exp)*/) {

  ec_master_info_t masterInfo;
  ec_slave_info_t  slaveInfo;
  int slaveCount       = 0;

  if (!master_) {
    LOGERR("%s/%s:%d: INFO: No EtherCAT master selected (0x%x).\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            ERROR_EC_MASTER_NULL);
    return setErrorID(ERROR_EC_MASTER_NULL);
  }

  int errorCode = ecrt_master(master_, &masterInfo);

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: Error: Function ecrt_master() failed with error code 0x%x.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode);
    return setErrorID(ERROR_EC_MASTER_NULL);
  }  

  // Slave loop
  slaveCount = masterInfo.slave_count;
  if (slavePos >= slaveCount) {
    LOGERR("%s/%s:%d: INFO: Slave index out of range (0x%x).\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            ERROR_EC_MAIN_INVALID_SLAVE_INDEX);
    return setErrorID(ERROR_EC_MAIN_INVALID_SLAVE_INDEX);
  }

  errorCode = ecrt_master_get_slave(master_, slavePos, &slaveInfo);

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: Error: Function ecrt_master_get_slave() failed with error code 0x%x (ecmc error 0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode,
      ERROR_EC_MAIN_GET_SLAVE_INFO_FAILED);
    return setErrorID(ERROR_EC_MAIN_GET_SLAVE_INFO_FAILED);
  }

  if(slaveInfo.position != slavePos) {
    LOGERR(
      "%s/%s:%d: Error: Slave verification for busposition %d failed. Bus position %d != %d (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePos,
      slavePos,
      slaveInfo.position,
      ERROR_EC_SLAVE_VERIFICATION_FAIL);
      return setErrorID(ERROR_EC_SLAVE_VERIFICATION_FAIL);
  }

  if(slaveInfo.alias != alias) {
    LOGERR(
      "%s/%s:%d: Error: Slave verification for busposition %d failed. Alias %d != %d (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePos,
      alias,
      slaveInfo.alias,
      ERROR_EC_SLAVE_VERIFICATION_FAIL);
      return setErrorID(ERROR_EC_SLAVE_VERIFICATION_FAIL);
  }

  if(slaveInfo.vendor_id != vendorId) {
    LOGERR(
      "%s/%s:%d: Error: Slave verification for busposition %d failed. Vendor Id 0x%x != 0x%x (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePos,
      vendorId,
      slaveInfo.vendor_id,
      ERROR_EC_SLAVE_VERIFICATION_FAIL);
      return setErrorID(ERROR_EC_SLAVE_VERIFICATION_FAIL);
  }

  if(slaveInfo.product_code != productCode) {
    LOGERR(
      "%s/%s:%d: Error: Slave verification for busposition %d failed. Product Code 0x%x != 0x%x (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePos,
      productCode,
      slaveInfo.product_code,
      ERROR_EC_SLAVE_VERIFICATION_FAIL);
      return setErrorID(ERROR_EC_SLAVE_VERIFICATION_FAIL);
  }

  return 0;
}

int ecmcEc::checkReadyForRuntime() {
  
  if(slaveCounter_ == 0 || entryCounter_ == 0){
    LOGERR(
      "%s/%s:%d: Error: No valid EtherCAT configuration (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SLAVE_VERIFICATION_FAIL);
    return setErrorID(ERROR_EC_NO_VALID_CONFIG);
  }
  
  return 0;
}

uint64_t ecmcEc::getTimeNs() {
  struct timespec timeRel, timeAbs; 
  clock_gettime(CLOCK_MONOTONIC, &timeRel);
  timeAbs = timespecAdd(timeRel, timeOffset_);
  return TIMESPEC2NS(timeAbs);
}
