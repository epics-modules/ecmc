/*
 * ecmcEcSlave.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: anderssandstrom
 */

#include "ecmcEcSlave.h"

ecmcEcSlave::ecmcEcSlave(
  ecmcAsynPortDriver* asynPortDriver,  /** Asyn port driver*/
  int masterId,
  ec_master_t *master,  /**< EtherCAT master */
  ec_domain_t *domain,  /** <Domain> */
  uint16_t     alias, /**< Slave alias. */
  uint16_t     position, /**< Slave position. */
  uint32_t     vendorId, /**< Expected vendor ID. */
  uint32_t     productCode  /**< Expected product code. */) {
  
  initVars();
  asynPortDriver_ = asynPortDriver;
  masterId_       = masterId;
  master_         = master;
  alias_          = alias; /**< Slave alias. */
  slavePosition_  = position;  /**< Slave position. */
  vendorId_       = vendorId; /**< Expected vendor ID. */
  productCode_    = productCode; /**< Expected product code. */
  
  // Simulation entries
  simEntries_[0] = new ecmcEcEntry(asynPortDriver_,
                                   masterId_,
                                   slavePosition_,
                                   (uint8_t)32,
                                   &simBuffer_[0],
                                    "ZERO");
  simEntries_[1] = new ecmcEcEntry(asynPortDriver_,
                                   masterId_,
                                   position,
                                   (uint8_t)32,
                                   &simBuffer_[8],
                                   "ONE");
  simEntries_[0]->writeValue(0);  // Default 0
  simEntries_[1]->writeValue(0x7FFFFFFF);  // Default 1 (31 bits (not sign bit for int))

  if ((alias == 0) && (position == 0) && (vendorId == 0) &&
      (productCode == 0)) {
    simSlave_ = true;
    return;
  }

  // Add simulation entries as first two entries
  entryList_[entryCounter_] = simEntries_[0];
  entryCounter_++;
  entryList_[entryCounter_] = simEntries_[1];
  entryCounter_++;

  domain_ = domain;

  if (!(slaveConfig_ =
          ecrt_master_slave_config(master_, alias_, slavePosition_, vendorId_,
                                   productCode_))) {
    LOGERR(
      "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Failed to get slave configuration (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      vendorId_,
      productCode_,
      ERROR_EC_SLAVE_CONFIG_FAILED);
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_EC_SLAVE_CONFIG_FAILED);
  }
  LOGINFO5(
    "%s/%s:%d: INFO: Slave %d created: alias %d, vendorId 0x%x, productCode 0x%x.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slavePosition_,
    alias_,
    vendorId_,
    productCode_);

    initAsyn();
}

void ecmcEcSlave::initVars() {
  errorReset();
  masterId_          = -1;
  simSlave_          = false;
  master_            = NULL;
  alias_             = 0;  // Slave alias.
  slavePosition_     = 0;  // Slave position.
  vendorId_          = 0;  // Expected vendor ID.
  productCode_       = 0;  // Expected product code.
  slaveConfig_       = NULL;
  syncManCounter_    = 0;
  entryCounter_      = 0;
  pdosArrayIndex_    = 0;
  syncManArrayIndex_ = 0;

  for (int i = 0; i < EC_MAX_SYNC_MANAGERS; i++) {
    syncManagerArray_[i] = NULL;
  }

  for (int i = 0; i < SIMULATION_ENTRIES; i++) {
    simEntries_[i] = NULL;
  }

  for (int i = 0; i < SIMULATION_ENTRIES * 8; i++) {
    simBuffer_[i] = 0;
  }

  for (int i = 0; i < EC_MAX_ENTRIES; i++) {
    entryList_[i] = NULL;
  }

  for (int i = 0; i < ECMC_ASYN_EC_SLAVE_PAR_COUNT; i++) {
    slaveAsynParams_[i] = NULL;
  }

  domain_ = NULL;
  memset(&slaveState_,    0, sizeof(slaveState_));
  memset(&slaveStateOld_, 0, sizeof(slaveStateOld_));

  asynPortDriver_  = NULL;
  online_          = 0;
  operational_     = 0;
  alState_         = 0;
}

ecmcEcSlave::~ecmcEcSlave() {
  for (int i = 0; i < EC_MAX_SYNC_MANAGERS; i++) {
    if (syncManagerArray_[i] != NULL) {
      delete syncManagerArray_[i];
    }
    syncManagerArray_[i] = NULL;
  }

  for (int i = 0; i < SIMULATION_ENTRIES; i++) {
    if (simEntries_[i] != NULL) {
      delete simEntries_[i];
    }
    simEntries_[i] = NULL;
  }
}

int ecmcEcSlave::getEntryCount() {
  return entryCounter_;
}

int ecmcEcSlave::addSyncManager(ec_direction_t direction,
                                uint8_t        syncMangerIndex) {
  if (simSlave_) {
    LOGERR(
      "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Simulation slave: Functionality not supported (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      vendorId_,
      productCode_,
      ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
  }

  if (syncManCounter_ >= EC_MAX_SYNC_MANAGERS) {
    LOGERR(
      "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Sync manager array full (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      vendorId_,
      productCode_,
      ERROR_EC_SLAVE_SM_ARRAY_FULL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_SM_ARRAY_FULL);
  }

  syncManagerArray_[syncManCounter_] = new ecmcEcSyncManager(asynPortDriver_,
                                                             masterId_,
                                                             slavePosition_,
                                                             domain_,
                                                             slaveConfig_,
                                                             direction,
                                                             syncMangerIndex);
  syncManCounter_++;
  return 0;
}

ecmcEcSyncManager * ecmcEcSlave::getSyncManager(int syncManagerIndex) {
  if (simSlave_) {
    LOGERR(
      "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Simulation slave: Functionality not supported (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      vendorId_,
      productCode_,
      ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return NULL;
  }

  if (syncManagerIndex >= EC_MAX_SYNC_MANAGERS) {
    LOGERR(
      "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Sync manager array index out of range (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      vendorId_,
      productCode_,
      ERROR_EC_SLAVE_SM_INDEX_OUT_OF_RANGE);
    setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_EC_SLAVE_SM_INDEX_OUT_OF_RANGE);
    return NULL;
  }
  return syncManagerArray_[syncManagerIndex];
}

int ecmcEcSlave::getSlaveInfo(mcu_ec_slave_info_light *info) {
  if (info == NULL) {
    LOGERR(
      "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Slave Info structure NULL (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      vendorId_,
      productCode_,
      ERROR_EC_SLAVE_SLAVE_INFO_STRUCT_NULL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_SLAVE_INFO_STRUCT_NULL);
  }
  info->alias        = alias_;
  info->position     = slavePosition_;
  info->product_code = productCode_;
  info->vendor_id    = vendorId_;
  return 0;
}

int ecmcEcSlave::checkConfigState(void) {
  if (simSlave_) {
    LOGERR(
      "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Simulation slave: Functionality not supported (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      vendorId_,
      productCode_,
      ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
  }

  memset(&slaveState_, 0, sizeof(slaveState_));
  ecrt_slave_config_state(slaveConfig_, &slaveState_);

  if (slaveState_.al_state != slaveStateOld_.al_state) {
    LOGINFO5("%s/%s:%d: INFO: Slave position: %d. State 0x%x.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             slaveState_.al_state);
  }
  alState_ = slaveState_.al_state;

  if (slaveState_.online != slaveStateOld_.online) {
    LOGINFO5("%s/%s:%d: INFO: Slave position: %d %s.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             slaveState_.online ? "Online" : "Offline");
  }
  online_ = slaveState_.online;

  if (slaveState_.operational != slaveStateOld_.operational) {
    LOGINFO5("%s/%s:%d: INFO: Slave position: %d %s operational.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             slaveState_.operational ? "" : "Not ");
  }
  operational_ = slaveState_.operational;  
  slaveStateOld_ = slaveState_;

  if (!slaveState_.online) {
    if (getErrorID() != ERROR_EC_SLAVE_NOT_ONLINE) {
      LOGERR("%s/%s:%d: ERROR: Slave %d: Not online (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             ERROR_EC_SLAVE_NOT_ONLINE);
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_NOT_ONLINE);
  }

  if (!slaveState_.operational) {
    if (getErrorID() != ERROR_EC_SLAVE_NOT_OPERATIONAL) {
      LOGERR("%s/%s:%d: ERROR: Slave %d: Not operational (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             ERROR_EC_SLAVE_NOT_OPERATIONAL);
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_NOT_OPERATIONAL);
  }

  switch (slaveState_.al_state) {
  case 1:

    if (getErrorID() != ERROR_EC_SLAVE_STATE_INIT) {
      LOGERR("%s/%s:%d: ERROR: Slave %d: State INIT (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             ERROR_EC_SLAVE_STATE_INIT);
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_STATE_INIT);

    break;

  case 2:

    if (getErrorID() != ERROR_EC_SLAVE_STATE_PREOP) {
      LOGERR("%s/%s:%d: ERROR: Slave %d: State PREOP (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             ERROR_EC_SLAVE_STATE_PREOP);
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_STATE_PREOP);

    break;

  case 4:

    if (getErrorID() != ERROR_EC_SLAVE_STATE_SAFEOP) {
      LOGERR("%s/%s:%d: ERROR: Slave %d: State SAFEOP (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             ERROR_EC_SLAVE_STATE_SAFEOP);
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_STATE_SAFEOP);

    break;

  case 8:

    // OK
    return 0;

    break;

  default:

    if (getErrorID() != ERROR_EC_SLAVE_STATE_UNDEFINED) {
      LOGERR("%s/%s:%d: ERROR: Slave %d: State UNDEFINED (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             ERROR_EC_SLAVE_STATE_UNDEFINED);
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_STATE_UNDEFINED);

    break;
  }

  return 0;
}

ecmcEcEntry * ecmcEcSlave::getEntry(int entryIndex) {
  if (!simSlave_) {
    if (entryIndex >= EC_MAX_ENTRIES) {
      LOGERR(
        "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry index out of range (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        slavePosition_,
        vendorId_,
        productCode_,
        ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE);
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE);
      return NULL;
    }

    if (entryList_[entryIndex] == NULL) {
      LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry NULL (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             vendorId_,
             productCode_,
             ERROR_EC_SLAVE_ENTRY_NULL);
      setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_EC_SLAVE_ENTRY_NULL);
      return NULL;
    }
    return entryList_[entryIndex];
  } else {
    if (entryIndex >= SIMULATION_ENTRIES) {
      LOGERR(
        "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry index out of range (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        slavePosition_,
        vendorId_,
        productCode_,
        ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE);
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE);
      return NULL;
    }

    if (simEntries_[entryIndex] == NULL) {
      LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry NULL (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             vendorId_,
             productCode_,
             ERROR_EC_SLAVE_ENTRY_NULL);
      setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_EC_SLAVE_ENTRY_NULL);
      return NULL;
    }
    return simEntries_[entryIndex];
  }
}

int ecmcEcSlave::updateInputProcessImage() {
  for (int i = 0; i < entryCounter_; i++) {
    if (entryList_[i] != NULL) {
      entryList_[i]->updateInputProcessImage();
    }
  }

  return 0;
}

int ecmcEcSlave::updateOutProcessImage() {
  for (int i = 0; i < entryCounter_; i++) {
    if (entryList_[i] != NULL) {
      entryList_[i]->updateOutProcessImage();
    }
  }

  // I/O intr to EPCIS.
  if (asynPortDriver_) {
    // Only update at desired samplerate
    slaveAsynParams_[ECMC_ASYN_EC_SLAVE_PAR_ONLINE_ID]->refreshParamRT(0);
    slaveAsynParams_[ECMC_ASYN_EC_SLAVE_PAR_AL_STATE_ID]->refreshParamRT(0);
    slaveAsynParams_[ECMC_ASYN_EC_SLAVE_PAR_OPERA_ID]->refreshParamRT(0);
    slaveAsynParams_[ECMC_ASYN_EC_SLAVE_PAR_ENTRY_COUNT_ID]->refreshParamRT(0);
  }
  return 0;
}

int ecmcEcSlave::getSlaveBusPosition() {
  return slavePosition_;
}

int ecmcEcSlave::addEntry(
  ec_direction_t direction,
  uint8_t        syncMangerIndex,
  uint16_t       pdoIndex,
  uint16_t       entryIndex,
  uint8_t        entrySubIndex,
  uint8_t        bits,
  std::string    id,
  int            signedValue
  ) {
  int err                        = 0;
  ecmcEcSyncManager *syncManager = findSyncMan(syncMangerIndex);

  if (syncManager == NULL) {
    err = addSyncManager(direction, syncMangerIndex);

    if (err) {
      LOGERR(
        "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Add sync manager failed (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        slavePosition_,
        vendorId_,
        productCode_,
        err);
      return err;
    }
    syncManager = syncManagerArray_[syncManCounter_ - 1];  // last added sync manager
  }

  ecmcEcEntry *entry = syncManager->addEntry(pdoIndex,
                                             entryIndex,
                                             entrySubIndex,
                                             bits,
                                             id,
                                             signedValue,
                                             &err);

  if (!entry) {
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Add entry failed (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slavePosition_,
           vendorId_,
           productCode_,
           err);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, err);
  }

  entryList_[entryCounter_] = entry;
  entryCounter_++;
  slaveAsynParams_[ECMC_ASYN_EC_SLAVE_PAR_ENTRY_COUNT_ID]->refreshParam(1);
  asynPortDriver_->callParamCallbacks();
  return 0;
}

ecmcEcSyncManager * ecmcEcSlave::findSyncMan(uint8_t syncMangerIndex) {
  for (int i = 0; i < syncManCounter_; i++) {
    if (syncManagerArray_[i] != NULL) {
      if (syncManagerArray_[i]->getSyncMangerIndex() == syncMangerIndex) {
        return syncManagerArray_[i];
      }
    }
  }
  return NULL;
}

int ecmcEcSlave::configDC(
  uint16_t assignActivate,     /**< AssignActivate word. */
  uint32_t sync0Cycle,     /**< SYNC0 cycle time [ns]. */
  int32_t  sync0Shift,    /**< SYNC0 shift time [ns]. */
  uint32_t sync1Cycle,     /**< SYNC1 cycle time [ns]. */
  int32_t  sync1Shift  /**< SYNC1 shift time [ns]. */) {
  if (slaveConfig_ == 0) {
    LOGERR(
      "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Slave Config NULL (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      vendorId_,
      productCode_,
      ERROR_EC_SLAVE_CONFIG_NULL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_CONFIG_NULL);
  }

  ecrt_slave_config_dc(slaveConfig_,
                       assignActivate,
                       sync0Cycle,
                       sync0Shift,
                       sync1Cycle,
                       sync1Shift);
  return 0;
}

ecmcEcEntry * ecmcEcSlave::findEntry(std::string id) {
  ecmcEcEntry *temp = NULL;

  for (int i = 0; i < syncManCounter_; i++) {
    temp = syncManagerArray_[i]->findEntry(id);

    if (temp) {
      return temp;
    }
  }

  // Simulation entries
  for (int i = 0; i < SIMULATION_ENTRIES; i++) {
    if (simEntries_[i] != NULL) {
      if (simEntries_[i]->getIdentificationName().compare(id) == 0) {
        return simEntries_[i];
      }
    }
  }
  return NULL;
}

int ecmcEcSlave::findEntryIndex(std::string id) {
  // Real entries
  for (int i = 0; i < EC_MAX_ENTRIES; i++) {
    if (entryList_[i] != NULL) {
      if (entryList_[i]->getIdentificationName().compare(id) == 0) {
        return i;
      }
    }
  }

  // Simulation entries
  for (int i = 0; i < SIMULATION_ENTRIES; i++) {
    if (simEntries_[i] != NULL) {
      if (simEntries_[i]->getIdentificationName().compare(id) == 0) {
        return i;
      }
    }
  }
  LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry not found (0x%x).\n",
         __FILE__,
         __FUNCTION__,
         __LINE__,
         slavePosition_,
         vendorId_,
         productCode_,
         ERROR_EC_SLAVE_ENTRY_NULL);
  return -ERROR_EC_SLAVE_ENTRY_NULL;
}

int ecmcEcSlave::selectAsReferenceDC() {
  return ecrt_master_select_reference_clock(master_, slaveConfig_);
}

int ecmcEcSlave::setWatchDogConfig(
  // Number of 40 ns intervals. Used as a base unit for all slave watchdogs.
  // If set to zero, the value is not written, so the default is used.
  uint16_t watchdogDivider,
  // Number of base intervals for process data watchdog. If set to zero,
  // the value is not written, so the default is used.
  uint16_t watchdogIntervals) {
  if (!slaveConfig_) {
    LOGERR(
      "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Slave Config NULL (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      vendorId_,
      productCode_,
      ERROR_EC_SLAVE_CONFIG_NULL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_CONFIG_NULL);
  }
  ecrt_slave_config_watchdog(slaveConfig_, watchdogDivider, watchdogIntervals);
  return 0;
}

int ecmcEcSlave::addSDOWrite(uint16_t sdoIndex,
                             uint8_t  sdoSubIndex,
                             uint32_t writeValue,
                             int      byteSize) {
  if (!slaveConfig_) {
    LOGERR(
      "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Slave Config NULL (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      vendorId_,
      productCode_,
      ERROR_EC_SLAVE_CONFIG_NULL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_CONFIG_NULL);
  }

  return ecmcEcSDO::addSdoConfig(slaveConfig_,
                                 slavePosition_,
                                 sdoIndex,
                                 sdoSubIndex,
                                 writeValue,
                                 byteSize);
}

int ecmcEcSlave::getSlaveState(ec_slave_config_state_t *state) {
  state = &slaveState_;
  return 0;
}

int ecmcEcSlave::initAsyn() {
  
  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];  
  char *name = buffer;
  ecmcAsynDataItem *paramTemp=NULL;

  // "ec%d.s%d.online"
  unsigned int charCount = snprintf(buffer,
                                    sizeof(buffer),
                                    ECMC_EC_STR"%d." ECMC_SLAVE_CHAR "%d."ECMC_ASYN_EC_SLAVE_PAR_ONLINE_NAME,
                                    masterId_,
                                    slavePosition_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SLAVE_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_SLAVE_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt32,
                                         (uint8_t *)&(online_),
                                         sizeof(online_),
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
  slaveAsynParams_[ECMC_ASYN_EC_SLAVE_PAR_ONLINE_ID] = paramTemp;

  // "ec%d.s%d.alstate"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR"%d." ECMC_SLAVE_CHAR "%d."ECMC_ASYN_EC_SLAVE_PAR_AL_STATE_NAME,
                       masterId_,
                       slavePosition_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SLAVE_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_SLAVE_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt32,
                                         (uint8_t *)&(alState_),
                                         sizeof(alState_),
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
  slaveAsynParams_[ECMC_ASYN_EC_SLAVE_PAR_AL_STATE_ID] = paramTemp;

  // "ec%d.s%d.operational"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR"%d." ECMC_SLAVE_CHAR "%d."ECMC_ASYN_EC_SLAVE_PAR_OPER_NAME,
                       masterId_,
                       slavePosition_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SLAVE_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_SLAVE_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt32,
                                         (uint8_t *)&(operational_),
                                         sizeof(operational_),
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
  slaveAsynParams_[ECMC_ASYN_EC_SLAVE_PAR_OPERA_ID] = paramTemp;


  // "ec%d.s%d.entrycounter"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR"%d." ECMC_SLAVE_CHAR "%d."ECMC_ASYN_EC_SLAVE_PAR_ENTRY_COUNT_NAME,
                       masterId_,
                       slavePosition_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SLAVE_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_SLAVE_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt32,
                                         (uint8_t *)&(entryCounter_),
                                         sizeof(entryCounter_),
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
  slaveAsynParams_[ECMC_ASYN_EC_SLAVE_PAR_ENTRY_COUNT_ID] = paramTemp;
  asynPortDriver_->callParamCallbacks();

  return 0;
}

int ecmcEcSlave::validate() {
  int errorCode=0;
  for (int i = 0; i < entryCounter_; i++) {
    if (entryList_[i]) {
      errorCode=entryList_[i]->validate();
      if (errorCode) {
        return errorCode;
      }
    }
  }
  return 0;
}