/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcEcSlave.cpp
*
*  Created on: Nov 30, 2015
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcEcSlave.h"
#include "ecmcErrorsList.h"

ecmcEcSlave::ecmcEcSlave(
  ecmcAsynPortDriver *asynPortDriver,  /** Asyn port driver*/
  int                 masterId,
  ec_master_t        *master, /**< EtherCAT master */
  ecmcEcDomain       *domain, /** <Domain> */
  uint16_t            alias, /**< Slave alias. */
  int32_t             position, /**< Slave position. */
  uint32_t            vendorId, /**< Expected vendor ID. */
  uint32_t            productCode /**< Expected product code. */) {
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
                                   &simBuffer_[0],
                                   ECMC_EC_U32,
                                   "ZERO");
  simEntries_[1] = new ecmcEcEntry(asynPortDriver_,
                                   masterId_,
                                   slavePosition_,
                                   &simBuffer_[8],
                                   ECMC_EC_U32,
                                   "ONE");
  simEntries_[0]->writeValue(0);  // Default 0
  simEntries_[1]->writeValue(0xFFFFFFFF);  // Default 1 (32 bits)

  if ((alias == 0) && (position == -1) && (vendorId == 0) &&
      (productCode == 0)) {
    simSlave_ = true;
    return;
  }

  // Add simulation entries as first two entries
  appendEntryToList(simEntries_[0], 1);
  appendEntryToList(simEntries_[1], 1);

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
  entryCounterInUse_ = 0;
  pdosArrayIndex_    = 0;
  syncManArrayIndex_ = 0;
  statusWord_        = 0;
  statusWordOld_     = 0;
  asyncSDOCounter_   = 0;

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
    entryList_[i]      = NULL;
    entryListInUse_[i] = NULL;
  }

  for (int i = 0; i < ECMC_ASYN_EC_SLAVE_PAR_COUNT; i++) {
    slaveAsynParams_[i] = NULL;
  }

  domain_ = NULL;
  memset(&slaveState_,    0, sizeof(slaveState_));
  memset(&slaveStateOld_, 0, sizeof(slaveStateOld_));
  asynPortDriver_ = NULL;
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

  // Clear pointers
  for (int i = 0; i < EC_MAX_ENTRIES; i++) {
    entryList_[i]      = NULL; // deleted in ecmcEcPdo()
    entryListInUse_[i] = NULL; // deleted in ecmcEcPdo()
  }

  for (int i = 0; i < ECMC_ASYN_EC_SLAVE_PAR_COUNT; i++) {
    delete slaveAsynParams_[i];
    slaveAsynParams_[i] = NULL;
  }

  for (int i = 0; i < asyncSDOCounter_; i++) {
    delete asyncSDOvector_[i];
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

  // Update status word
  //  lower 16  : status bits
  //  higher 16 : entrycounter

  statusWord_ = 0;
  statusWord_ = statusWord_ + (slaveState_.online);
  statusWord_ = statusWord_ + (slaveState_.operational << 1);
  statusWord_ = statusWord_ + (slaveState_.al_state << 2);
  statusWord_ = statusWord_ + (entryCounter_ << 16);

  if (statusWord_ != statusWordOld_) {
    slaveAsynParams_[ECMC_ASYN_EC_SLAVE_PAR_STATUS_ID]->refreshParamRT(1);
  }
  statusWordOld_ = statusWord_;

  if (slaveState_.al_state != slaveStateOld_.al_state) {
    LOGINFO5("%s/%s:%d: INFO: Slave position: %d. State 0x%x.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             slaveState_.al_state);
  }

  bool updateAlarmState = false;

  if (slaveState_.online != slaveStateOld_.online) {
    LOGINFO5("%s/%s:%d: INFO: Slave position: %d %s.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             slaveState_.online ? "Online" : "Offline");

    // Status changed.. Update alarm status
    updateAlarmState = true;
  }

  if (slaveState_.operational != slaveStateOld_.operational) {
    LOGINFO5("%s/%s:%d: INFO: Slave position: %d %s operational.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             slaveState_.operational ? "" : "Not ");

    // Status changed.. Update alarm status
    updateAlarmState = true;
  }

  // Alarm state
  if (updateAlarmState) {
    for (uint i = 0; i < entryCounter_; i++) {
      if (entryList_[i] != NULL) {
        entryList_[i]->setComAlarm((!slaveState_.online ||
                                    !slaveState_.operational));
      }
    }
  }

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
  for (uint i = 0; i < entryCounterInUse_; i++) {
    if (entryListInUse_[i] != NULL) {
      entryListInUse_[i]->updateInputProcessImage();
    }
  }

  // Execute async SDOs
  for (int i = 0; i < asyncSDOCounter_; i++) {
    asyncSDOvector_[i]->execute();
  }

  return 0;
}

int ecmcEcSlave::updateOutProcessImage() {
  for (uint i = 0; i < entryCounterInUse_; i++) {
    if (entryListInUse_[i] != NULL) {
      entryListInUse_[i]->updateOutProcessImage();
    }
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
  ecmcEcDataType dt,
  std::string    id,
  int            useInRealTime) {
  if (entryCounter_ >= EC_MAX_ENTRIES) {
    return ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE;
  }

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
                                             dt,
                                             id,
                                             useInRealTime,
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

  if (entry->getError()) {
    return entry->getErrorID();
  }

  return appendEntryToList(entry, useInRealTime);
}

int ecmcEcSlave::addDataItem(ecmcEcEntry   *startEntry,
                             size_t         entryByteOffset,
                             size_t         entryBitOffset,
                             ec_direction_t direction,
                             ecmcEcDataType dt,
                             std::string    id) {
  if (entryCounter_ >= EC_MAX_ENTRIES) {
    return ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE;
  }

  // Do not add this to sync manager and pdo since no ethercat configs are done.
  // This is just pure mem access of alreday configured entry/processimage
  ecmcEcEntry *entry = new ecmcEcData(asynPortDriver_,
                                      masterId_,
                                      slavePosition_,
                                      startEntry,
                                      entryByteOffset,
                                      entryBitOffset,
                                      direction,
                                      dt,
                                      id);

  if (!entry) {
    LOGERR(
      "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Add data item failed (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      vendorId_,
      productCode_,
      ERROR_EC_SLAVE_ADD_DATA_ITEM_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_ADD_DATA_ITEM_FAIL);
  }

  if (entry->getError()) {
    return entry->getErrorID();
  }

  return appendEntryToList(entry, 1);
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
  int32_t  sync1Shift /**< SYNC1 shift time [ns]. */) {
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
  for (uint i = 0; i < entryCounter_; i++) {
    if (entryList_[i]) {
      if (entryList_[i]->getIdentificationName().compare(id) == 0) {
        return entryList_[i];
      }
    }
  }

  // for (int i = 0; i < syncManCounter_; i++) {
  //  temp = syncManagerArray_[i]->findEntry(id);
  //
  //  if (temp) {
  //    return temp;
  //  }
  // }

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

int ecmcEcSlave::addSDOWriteDT(uint16_t       sdoIndex,
                               uint8_t        sdoSubIndex,
                               const char    *value,
                               ecmcEcDataType dt) {
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

  return ecmcEcSDO::addSdoConfigDT(slaveConfig_,
                                   slavePosition_,
                                   sdoIndex,
                                   sdoSubIndex,
                                   value,
                                   dt);
}

int ecmcEcSlave::getSlaveState(ec_slave_config_state_t *state) {
  state = &slaveState_;
  return 0;
}

int ecmcEcSlave::initAsyn() {
  char  buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char *name                  = buffer;
  ecmcAsynDataItem *paramTemp = NULL;

  // "ec%d.s%d.status"
  unsigned int charCount = snprintf(buffer,
                                    sizeof(buffer),
                                    ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d." ECMC_ASYN_EC_SLAVE_PAR_STATUS_NAME,
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
  name      = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                                asynParamUInt32Digital,
                                                (uint8_t *)&(statusWord_),
                                                sizeof(statusWord_),
                                                ECMC_EC_U32,
                                                0);

  if (!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  paramTemp->addSupportedAsynType(asynParamInt32);
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);
  slaveAsynParams_[ECMC_ASYN_EC_SLAVE_PAR_STATUS_ID] = paramTemp;
  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST,
                                      ECMC_ASYN_DEFAULT_ADDR);
  return 0;
}

int ecmcEcSlave::validate() {
  int errorCode = 0;

  for (uint i = 0; i < entryCounter_; i++) {
    if (entryList_[i]) {
      errorCode = entryList_[i]->validate();

      if (errorCode) {
        return errorCode;
      }
    }
  }
  return 0;
}

int ecmcEcSlave::appendEntryToList(ecmcEcEntry *entry, bool useInRealTime) {
  if (entryCounter_ >= EC_MAX_ENTRIES) {
    return ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE;
  }
  entryList_[entryCounter_] = entry;
  entryCounter_++;

  // Only update if in realtime
  if (useInRealTime) {
    entryListInUse_[entryCounterInUse_] = entry;
    entryCounterInUse_++;
  }
  return 0;
}

int ecmcEcSlave::addSDOWriteComplete(uint16_t    sdoIndex,
                                     const char *dataBuffer,
                                     int         byteSize) {
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

  return ecmcEcSDO::addWriteComplete(slaveConfig_,
                                     sdoIndex,
                                     dataBuffer,
                                     (size_t)byteSize);
}

int ecmcEcSlave::addSDOWriteBuffer(uint16_t    sdoIndex,
                                   uint8_t     sdoSubIndex,
                                   const char *dataBuffer,
                                   int         byteSize) {
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

  return ecmcEcSDO::addSdoConfigBuffer(slaveConfig_,
                                       sdoIndex,
                                       sdoSubIndex,
                                       dataBuffer,
                                       (size_t)byteSize);
}

int ecmcEcSlave::addSDOAsync(uint16_t       sdoIndex, /**< SDO index. */
                             uint8_t        sdoSubIndex, /**< SDO subindex. */
                             ecmcEcDataType dt,
                             std::string    alias) {
  try {
    ecmcEcAsyncSDO *temp = new ecmcEcAsyncSDO(asynPortDriver_,
                                              masterId_,
                                              slavePosition_,
                                              slaveConfig_,
                                              sdoIndex,
                                              sdoSubIndex,
                                              dt,
                                              alias);
    asyncSDOvector_.push_back(temp);
  }
  catch (std::exception& e) {
    LOGERR(
      "%s/%s:%d: ERROR: Slave %d: Failed to create async SDO object (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      ERROR_EC_SLAVE_SDO_ASYNC_CREATE_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_SDO_ASYNC_CREATE_FAIL);
  }
  asyncSDOCounter_++;
  return 0;
}

int ecmcEcSlave::activate() {
  int ret = 0;

  for (uint entryIndex = 0; entryIndex < entryCounter_; entryIndex++) {
    ecmcEcEntry *tempEntry = getEntry(entryIndex);

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
      ret = tempEntry->activate();

      if (ret) {
        return ret;
      }
    }
  }

  return 0;
}

int ecmcEcSlave::compileRegInfo() {
  int ret = 0;

  for (uint entryIndex = 0; entryIndex < entryCounter_; entryIndex++) {
    ecmcEcEntry *tempEntry = getEntry(entryIndex);

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
      ret = tempEntry->compileRegInfo();

      if (ret) {
        return ret;
      }
    }
  }

  return 0;
}

int ecmcEcSlave::getAllowOffline() {
  if (domain_) {
    return domain_->getAllowOffline();
  }

  // No domain attached the simulation slave
  return 1;
}
