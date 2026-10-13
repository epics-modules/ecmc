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
#include <stddef.h>
#include "ecmcErrorsList.h"
#include "ecmcRtLogger.h"
#include "ecmcGeneral.h"

#define ECMC_EC_SM_OUTPUT_TIMING_INDEX 0x1C32
#define ECMC_EC_SM_INPUT_TIMING_INDEX  0x1C33

extern app_mode_type appModeStat;

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
  
  // Simulation entries, two 32 bit entries
  addSimEntry("ZERO",ECMC_EC_U32,0);
  addSimEntry("ONE",ECMC_EC_U32,0xFFFFFFFF);

  if ((alias == 0) && (position == -1) && (vendorId == 0) &&
      (productCode == 0)) {
    simSlave_ = true;
    int errorCode = initAsyn();
    if (errorCode) {
      setErrorID(errorCode);
    }
    return;
  }

  domain_ = domain;

  if (!(slaveConfig_ =
          ecrt_master_slave_config(master_, alias_, slavePosition_, vendorId_,
                                   productCode_))) {
    ecmcRtLoggerLogError(
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
  ECMC_RT_LOGINFO5(
    "%s/%s:%d: INFO: Slave %d created: alias %d, vendorId 0x%x, productCode 0x%x.\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slavePosition_,
    alias_,
    vendorId_,
    productCode_);

  int errorCode = initAsyn();
  if (errorCode) {
    setErrorID(errorCode);
  }
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
  portWiringCheckEnabled_ = false;
  slaveConfig_       = NULL;
  syncManCounter_    = 0;
  entryCounter_      = 0;
  entryCounterRtInput_  = 0;
  entryCounterRtOutput_ = 0;
  pdosArrayIndex_    = 0;
  syncManArrayIndex_ = 0;
  statusWord_        = 0;
  statusWordOld_     = 0;
  asyncSDOCounter_   = 0;
  enableSDOCheck_    = 0;
  inputSmTiming_     = ecmcEcSmTiming(ECMC_EC_SM_INPUT_TIMING_INDEX);
  outputSmTiming_    = ecmcEcSmTiming(ECMC_EC_SM_OUTPUT_TIMING_INDEX);
  inputTimestampEntry_ = NULL;
  outputTimestampEntry_ = NULL;
  smTimingRequestCount_ = 0;
  smTimingDiscoveryComplete_ = false;
  dcScheduleRequest_ = NULL;
  dcScheduleStage_ = 0;
  dcScheduleRequestStarted_ = false;
  nominalTimingCycleNs_ = 0;
  hasProcessDataInput_ = false;
  hasProcessDataOutput_ = false;
  memset(&inputTimingAsynData_, 0, sizeof(inputTimingAsynData_));
  memset(&outputTimingAsynData_, 0, sizeof(outputTimingAsynData_));
  timingAsynDirty_ = true;
  for (size_t i = 0; i < sizeof(smTimingRequests_) / sizeof(smTimingRequests_[0]); ++i) {
    smTimingRequests_[i] = {NULL, NULL, NULL, 0, false, false};
  }
  for (int i = 0; i < EC_MAX_SYNC_MANAGERS; i++) {
    syncManagerArray_[i] = NULL;
  }

  for (int i = 0; i < EC_MAX_ENTRIES; i++) {
    entryList_[i]         = NULL;
    entryListRtInput_[i]  = NULL;
    entryListRtOutput_[i] = NULL;
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

  for (size_t i = 0; i < simEntries_.size(); i++) {
    delete simEntries_[i];
    simEntries_[i] = NULL;
  }
  simEntries_.clear();

  for (size_t i = 0; i < simBuffer_.size(); i++) {
    delete simBuffer_[i];
    simBuffer_[i] = NULL;
  }
  simBuffer_.clear();

  // Clear pointers
  for (int i = 0; i < EC_MAX_ENTRIES; i++) {
    entryList_[i]         = NULL; // deleted in ecmcEcPdo()
    entryListRtInput_[i]  = NULL; // deleted in ecmcEcPdo()
    entryListRtOutput_[i] = NULL; // deleted in ecmcEcPdo()
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
    ecmcRtLoggerLogError(
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
    ecmcRtLoggerLogError(
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

  ecmcEcSyncManager *syncManager = new ecmcEcSyncManager(asynPortDriver_,
                                                         masterId_,
                                                         slavePosition_,
                                                         domain_,
                                                         slaveConfig_,
                                                         direction,
                                                         syncMangerIndex);
  if (!syncManager) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MAIN_EXCEPTION);
  }

  int errorCode = syncManager->getErrorID();
  if (errorCode) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Sync manager creation failed: %s (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slavePosition_,
           vendorId_,
           productCode_,
           getErrorString(errorCode),
           errorCode);
    delete syncManager;
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  syncManagerArray_[syncManCounter_] = syncManager;
  syncManCounter_++;
  return 0;
}

ecmcEcSyncManager * ecmcEcSlave::getSyncManager(int syncManagerIndex) {
  if (simSlave_) {
    ecmcRtLoggerLogError(
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
    ecmcRtLoggerLogError(
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
    ecmcRtLoggerLogError(
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
    ecmcRtLoggerLogError(
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
    ECMC_RT_LOGINFO5("%s/%s:%d: INFO: Slave position: %d. State 0x%x.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             slaveState_.al_state);
  }

  bool updateAlarmState = false;

  if (slaveState_.online != slaveStateOld_.online) {
    ECMC_RT_LOGINFO5("%s/%s:%d: INFO: Slave position: %d %s.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             slaveState_.online ? "Online" : "Offline");

    // Status changed.. Update alarm status
    updateAlarmState = true;
  }

  if (slaveState_.operational != slaveStateOld_.operational) {
    ECMC_RT_LOGINFO5("%s/%s:%d: INFO: Slave position: %d %s operational.\n",
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
    if ((appModeStat != ECMC_MODE_STARTUP) &&
        (getErrorID() != ERROR_EC_SLAVE_NOT_ONLINE)) {
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Slave %d: Not online (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             ERROR_EC_SLAVE_NOT_ONLINE);
    }

    if (appModeStat == ECMC_MODE_STARTUP) {
      return ERROR_EC_SLAVE_NOT_ONLINE;
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_NOT_ONLINE);
  }

  if (!slaveState_.operational) {
    if ((appModeStat != ECMC_MODE_STARTUP) &&
        (getErrorID() != ERROR_EC_SLAVE_NOT_OPERATIONAL)) {
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Slave %d: Not operational (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             slavePosition_,
             ERROR_EC_SLAVE_NOT_OPERATIONAL);
    }

    if (appModeStat == ECMC_MODE_STARTUP) {
      return ERROR_EC_SLAVE_NOT_OPERATIONAL;
    }
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_NOT_OPERATIONAL);
  }


  switch (slaveState_.al_state) {
  case 1:

    if (getErrorID() != ERROR_EC_SLAVE_STATE_INIT) {
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Slave %d: State INIT (0x%x).\n",
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
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Slave %d: State PREOP (0x%x).\n",
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
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Slave %d: State SAFEOP (0x%x).\n",
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
    if (getErrorID()) {
      errorReset();
    }
    return 0;

    break;

  default:

    if (getErrorID() != ERROR_EC_SLAVE_STATE_UNDEFINED) {
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Slave %d: State UNDEFINED (0x%x).\n",
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
      ecmcRtLoggerLogError(
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
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry NULL (0x%x).\n",
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
    if ((entryIndex < 0) || (static_cast<size_t>(entryIndex) >= simEntries_.size())) {
      ecmcRtLoggerLogError(
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
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry NULL (0x%x).\n",
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
  const uint entryCountInUse = entryCounterRtInput_;
  for (uint i = 0; i < entryCountInUse; i++) {
    entryListRtInput_[i]->updateInputProcessImage();
  }

  // Execute async SDOs
  const int asyncSdoCount = asyncSDOCounter_;
  for (int i = 0; i < asyncSdoCount; i++) {
    asyncSDOvector_[i]->execute();
  }
  executeSmTimingDiscovery();
  executeDcScheduleDiscovery();

  // Publish the final startup snapshot even when slow diagnostics are disabled.
  if (!timingStartupReady_.load(std::memory_order_relaxed)) {
    bool complete = !dcSchedule_.discoveryAttempted ||
                    dcSchedule_.discoveryComplete;
    for (uint8_t i = 0; i < smTimingRequestCount_; ++i) {
      complete &= smTimingRequests_[i].finished;
    }
    if (complete) {
      refreshTimingAsyn();
      timingStartupReady_.store(true, std::memory_order_release);
    }
  }

  return 0;
}

int ecmcEcSlave::updateOutProcessImage() {
  const uint entryCountInUse = entryCounterRtOutput_;
  for (uint i = 0; i < entryCountInUse; i++) {
    entryListRtOutput_[i]->updateOutProcessImage();
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
  int            useInRealTime,
  bool           useExistingMapping,
  bool           registerByPosition,
  unsigned int   entryPosition) {
  if (entryCounter_ >= EC_MAX_ENTRIES) {
    return ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE;
  }

  int err                        = 0;
  ecmcEcSyncManager *syncManager = findSyncMan(syncMangerIndex);

  if (syncManager == NULL) {
    err = addSyncManager(direction, syncMangerIndex);

    if (err) {
      ecmcRtLoggerLogError(
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
                                             useExistingMapping,
                                             registerByPosition,
                                             entryPosition,
                                             &err);

  if (!entry) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Add entry failed (0x%x).\n",
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
    ecmcRtLoggerLogError(
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
    ecmcRtLoggerLogError(
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
  dcConfig_.configured = true;
  dcConfig_.assignActivate = assignActivate;
  dcConfig_.sync0CycleNs = sync0Cycle;
  dcConfig_.sync0ShiftNs = sync0Shift;
  dcConfig_.sync1OffsetNs = sync1Cycle;
  dcConfig_.sync1ShiftNs = sync1Shift;
  timingAsynDirty_ = true;
  return 0;
}

const ecmcEcDcConfig& ecmcEcSlave::getDcConfig() const {
  return dcConfig_;
}

const ecmcEcDcSchedule& ecmcEcSlave::getDcSchedule() const {
  return dcSchedule_;
}

void ecmcEcSlave::setNominalTimingCycleNs(uint32_t cycleTimeNs) {
  nominalTimingCycleNs_ = cycleTimeNs;
  timingAsynDirty_ = true;
}

void ecmcEcSlave::prepareSmTimingValue(uint16_t objectIndex,
                                       uint8_t subIndex,
                                       uint8_t byteSize,
                                       ecmcEcSmTiming *timing,
                                       ecmcEcSmTimingValue *value) {
  if (!value || !timing || !slaveConfig_ ||
      smTimingRequestCount_ >= sizeof(smTimingRequests_) /
                               sizeof(smTimingRequests_[0])) {
    return;
  }
  smTimingRequest& item = smTimingRequests_[smTimingRequestCount_++];
  item.request = ecrt_slave_config_create_sdo_request(slaveConfig_,
                                                       objectIndex,
                                                       subIndex,
                                                       byteSize);
  item.value = value;
  item.timing = timing;
  item.byteSize = byteSize;
  if (!item.request) {
    item.finished = true;
    timingAsynDirty_ = true;
    value->requestError = ERROR_EC_SLAVE_SDO_ASYNC_CREATE_FAIL;
  } else {
    ecrt_sdo_request_timeout(item.request, DEFAULT_SDO_ASYNC_TIMOUT_MS);
  }
}

void ecmcEcSlave::prepareSmTiming(uint16_t objectIndex,
                                  ecmcEcSmTiming *timing) {
  if (!timing) {
    return;
  }
  *timing = ecmcEcSmTiming(objectIndex);
  timing->discoveryAttempted = true;
  prepareSmTimingValue(objectIndex, 0x01, 2, timing, &timing->syncType);
  prepareSmTimingValue(objectIndex, 0x02, 4, timing, &timing->cycleTimeNs);
  prepareSmTimingValue(objectIndex, 0x03, 4, timing, &timing->shiftTimeNs);
  prepareSmTimingValue(objectIndex, 0x04, 2, timing,
                       &timing->syncTypesSupported);
  prepareSmTimingValue(objectIndex, 0x05, 4, timing,
                       &timing->minimumCycleTimeNs);
  prepareSmTimingValue(objectIndex, 0x06, 4, timing,
                       &timing->calculationCopyTimeNs);
  prepareSmTimingValue(objectIndex, 0x07, 4, timing,
                       &timing->minimumDelayTimeNs);
  prepareSmTimingValue(objectIndex, 0x08, 2, timing, &timing->command);
  prepareSmTimingValue(objectIndex, 0x09, 4, timing,
                       &timing->maximumDelayTimeNs);
  prepareSmTimingValue(objectIndex, 0x20, 1, timing,
                       &timing->synchronizationError);
}

void ecmcEcSlave::discoverSmTiming() {
  timingStartupReady_.store(false, std::memory_order_release);
  if (simSlave_ || smTimingRequestCount_ != 0) {
    return;
  }
  for (uint32_t i = 0; i < entryCounter_; ++i) {
    if (!entryList_[i] || entryList_[i]->getSimEntry()) {
      continue;
    }
    hasProcessDataInput_ |= entryList_[i]->getDirection() == EC_DIR_INPUT;
    hasProcessDataOutput_ |= entryList_[i]->getDirection() == EC_DIR_OUTPUT;
  }
  // Non-DC endpoints are still usable against the receive/application cycle
  // anchors, but have no DC SyncManager schedule to discover.
  if (!dcConfig_.configured) {
    resolveEndpointTiming(outputSmTiming_, outputTimingOverride_,
                          outputTimestampConfig_, EC_DIR_OUTPUT,
                          &outputTiming_);
    resolveEndpointTiming(inputSmTiming_, inputTimingOverride_,
                          inputTimestampConfig_, EC_DIR_INPUT, &inputTiming_);
    return;
  }
  prepareDcScheduleDiscovery();
  // Only request timing objects for process-data directions actually used by
  // this slave. For example, an EL5042 has no output SyncManager timing.
  if (hasProcessDataOutput_) {
    prepareSmTiming(ECMC_EC_SM_OUTPUT_TIMING_INDEX, &outputSmTiming_);
  }
  if (hasProcessDataInput_) {
    prepareSmTiming(ECMC_EC_SM_INPUT_TIMING_INDEX, &inputSmTiming_);
  }

  // Publish the configured DC state and the correct direction-specific cycle
  // references immediately. SDO discovery will refine these values after OP.
  resolveEndpointTiming(outputSmTiming_, outputTimingOverride_,
                        outputTimestampConfig_, EC_DIR_OUTPUT, &outputTiming_);
  resolveEndpointTiming(inputSmTiming_, inputTimingOverride_,
                        inputTimestampConfig_, EC_DIR_INPUT, &inputTiming_);
  timingAsynDirty_ = true;
}

void ecmcEcSlave::prepareDcScheduleDiscovery() {
  if (dcSchedule_.discoveryAttempted || !dcConfig_.configured || simSlave_) {
    return;
  }
  dcSchedule_.discoveryAttempted = true;
#ifdef EC_HAVE_REG_ACCESS
  dcScheduleRequest_ =
    ecrt_slave_config_create_reg_request(slaveConfig_, sizeof(uint64_t));
  if (!dcScheduleRequest_) {
    dcSchedule_.discoveryComplete = true;
    dcSchedule_.requestError = ERROR_EC_SLAVE_CONFIG_FAILED;
  }
#else
  dcSchedule_.discoveryComplete = true;
  dcSchedule_.requestError = ERROR_EC_SLAVE_CONFIG_FAILED;
#endif
}

void ecmcEcSlave::executeDcScheduleDiscovery() {
#ifdef EC_HAVE_REG_ACCESS
  if (!dcScheduleRequest_ || dcSchedule_.discoveryComplete) {
    return;
  }

  ec_slave_config_state_t state;
  memset(&state, 0, sizeof(state));
  ecrt_slave_config_state(slaveConfig_, &state);
  if (!state.operational) {
    return;
  }

  static const uint16_t addresses[] = {0x0990, 0x09A0, 0x09A4};
  static const size_t sizes[] = {sizeof(uint64_t), sizeof(uint32_t),
                                 sizeof(uint32_t)};
  if (!dcScheduleRequestStarted_) {
    if (dcScheduleStage_ == 0) {
      ecmcRtLoggerLogInfo(
        "EtherCAT master %d slave %d: reading DC timing registers "
        "0x0990, 0x09A0, 0x09A4.\n", masterId_, slavePosition_);
    }
    // EtherLab releases differ here: some declare this function void and
    // newer releases return an error code. Ignoring the return is compatible
    // with both; completion/error is reported by ecrt_reg_request_state().
    ecrt_reg_request_read(dcScheduleRequest_,
                          addresses[dcScheduleStage_],
                          sizes[dcScheduleStage_]);
    dcScheduleRequestStarted_ = true;
    return;
  }

  const ec_request_state_t requestState =
    ecrt_reg_request_state(dcScheduleRequest_);
  if (requestState == EC_REQUEST_BUSY || requestState == EC_REQUEST_UNUSED) {
    return;
  }
  if (requestState != EC_REQUEST_SUCCESS) {
    dcSchedule_.requestError = ERROR_EC_SLAVE_CONFIG_FAILED;
    dcSchedule_.discoveryComplete = true;
    return;
  }

  const uint8_t *data = ecrt_reg_request_data(dcScheduleRequest_);
  if (dcScheduleStage_ == 0) {
    dcSchedule_.startTimeNs = EC_READ_U64(data);
  } else if (dcScheduleStage_ == 1) {
    dcSchedule_.sync0CycleNs = EC_READ_U32(data);
  } else {
    dcSchedule_.sync1CycleNs = EC_READ_U32(data);
  }
  dcScheduleRequestStarted_ = false;
  ++dcScheduleStage_;
  if (dcScheduleStage_ >= sizeof(addresses) / sizeof(addresses[0])) {
    dcSchedule_.discoveryComplete = true;
  }
#endif
}

void ecmcEcSlave::executeSmTimingDiscovery() {
  if (smTimingRequestCount_ == 0 || smTimingDiscoveryComplete_) {
    return;
  }

  bool allFinished = true;
  for (uint8_t i = 0; i < smTimingRequestCount_; ++i) {
    allFinished &= smTimingRequests_[i].finished;
  }
  if (allFinished) {
    smTimingDiscoveryComplete_ = true;
    return;
  }

  // Requests are prepared before master activation but must not be triggered
  // until the configured slave has reached OP. Query the state directly here:
  // slaveState_ is maintained by optional diagnostics and remains zero when
  // diagnostics are disabled.
  ec_slave_config_state_t timingState;
  memset(&timingState, 0, sizeof(timingState));
  ecrt_slave_config_state(slaveConfig_, &timingState);
  if (!timingState.operational) {
    return;
  }

  // Serialize mailbox traffic. Some slaves reject or starve a burst of SDO
  // requests, and timing discovery is a one-shot startup operation.
  for (uint8_t i = 0; i < smTimingRequestCount_; ++i) {
    smTimingRequest& item = smTimingRequests_[i];
    if (item.finished || !item.request) {
      continue;
    }
    if (!item.started) {
      if (!smTimingReadLogged_) {
        ecmcRtLoggerLogInfo(
          "EtherCAT master %d slave %d: reading SyncManager timing "
          "objects (0x1C32 output: %s, 0x1C33 input: %s).\n",
          masterId_, slavePosition_,
          outputSmTiming_.discoveryAttempted ? "requested" : "not requested",
          inputSmTiming_.discoveryAttempted ? "requested" : "not requested");
        smTimingReadLogged_ = true;
      }
      ecrt_sdo_request_read(item.request);
      item.started = true;
      return;
    }
    const ec_request_state_t state = ecrt_sdo_request_state(item.request);
    if (state == EC_REQUEST_BUSY || state == EC_REQUEST_UNUSED) {
      return;
    }
    item.finished = true;
    if (state == EC_REQUEST_SUCCESS) {
      const uint8_t *data = ecrt_sdo_request_data(item.request);
      item.value->value = 0;
      for (uint8_t byte = 0; byte < item.byteSize; ++byte) {
        item.value->value |= static_cast<uint32_t>(data[byte]) << (8u * byte);
      }
      item.value->byteSize = item.byteSize;
      item.value->available = true;
    } else {
      item.value->requestError = ERROR_EC_SDO_ASYNC_ERROR;
    }
    timingAsynDirty_ = true;
    // Start or inspect at most one request per cycle.
    break;
  }

  outputSmTiming_.discoveryComplete = outputSmTiming_.discoveryAttempted;
  inputSmTiming_.discoveryComplete = inputSmTiming_.discoveryAttempted;
  for (uint8_t i = 0; i < smTimingRequestCount_; ++i) {
    if (!smTimingRequests_[i].finished) {
      smTimingRequests_[i].timing->discoveryComplete = false;
    }
  }
  smTimingDiscoveryComplete_ = outputSmTiming_.discoveryComplete &&
                               inputSmTiming_.discoveryComplete;
  resolveEndpointTiming(outputSmTiming_, outputTimingOverride_,
                        outputTimestampConfig_, EC_DIR_OUTPUT, &outputTiming_);
  resolveEndpointTiming(inputSmTiming_, inputTimingOverride_,
                        inputTimestampConfig_, EC_DIR_INPUT, &inputTiming_);
}

const ecmcEcSmTiming& ecmcEcSlave::getInputSmTiming() const {
  return inputSmTiming_;
}

const ecmcEcSmTiming& ecmcEcSlave::getOutputSmTiming() const {
  return outputSmTiming_;
}

int ecmcEcSlave::setTimingOverride(ec_direction_t direction,
                                   int32_t cycleOffset,
                                   int32_t eventOffsetNs,
                                   uint32_t uncertaintyNs) {
  ecmcEcTimingOverride *timingOverride = NULL;
  ecmcEcEndpointTiming *endpoint = NULL;
  const ecmcEcSmTiming *smTiming = NULL;
  if (direction == EC_DIR_INPUT) {
    timingOverride = &inputTimingOverride_;
    endpoint = &inputTiming_;
    smTiming = &inputSmTiming_;
  } else if (direction == EC_DIR_OUTPUT) {
    timingOverride = &outputTimingOverride_;
    endpoint = &outputTiming_;
    smTiming = &outputSmTiming_;
  } else {
    return ERROR_EC_SLAVE_ENTRY_INFO_STRUCT_NULL;
  }
  timingOverride->configured = true;
  timingOverride->cycleOffset = cycleOffset;
  timingOverride->eventOffsetNs = eventOffsetNs;
  timingOverride->uncertaintyNs = uncertaintyNs;
  const ecmcEcTimestampConfig& timestampConfig =
    direction == EC_DIR_INPUT ? inputTimestampConfig_ : outputTimestampConfig_;
  resolveEndpointTiming(*smTiming, *timingOverride, timestampConfig, direction,
                        endpoint);
  timingAsynDirty_ = true;
  return 0;
}

int ecmcEcSlave::setTimingSource(ec_direction_t direction,
                                 ecmcEcTimingSource source) {
  if (source != ecmcEcTimingSource::CYCLE_ONLY &&
      source != ecmcEcTimingSource::SYNC0_DERIVED &&
      source != ecmcEcTimingSource::SYNC1_DERIVED) {
    return ERROR_EC_SLAVE_ENTRY_INFO_STRUCT_NULL;
  }
  if (source != ecmcEcTimingSource::CYCLE_ONLY && !dcConfig_.configured) {
    return ERROR_EC_SLAVE_ENTRY_INFO_STRUCT_NULL;
  }

  ecmcEcTimingOverride *timingOverride = NULL;
  const ecmcEcSmTiming *smTiming = NULL;
  ecmcEcEndpointTiming *endpoint = NULL;
  if (direction == EC_DIR_INPUT) {
    timingOverride = &inputTimingOverride_;
    smTiming = &inputSmTiming_;
    endpoint = &inputTiming_;
  } else if (direction == EC_DIR_OUTPUT) {
    timingOverride = &outputTimingOverride_;
    smTiming = &outputSmTiming_;
    endpoint = &outputTiming_;
  } else {
    return ERROR_EC_SLAVE_ENTRY_INFO_STRUCT_NULL;
  }

  timingOverride->sourceConfigured = true;
  timingOverride->source = source;
  const ecmcEcTimestampConfig& timestampConfig = direction == EC_DIR_INPUT ?
    inputTimestampConfig_ : outputTimestampConfig_;
  resolveEndpointTiming(*smTiming, *timingOverride, timestampConfig, direction,
                        endpoint);
  timingAsynDirty_ = true;
  return 0;
}

int ecmcEcSlave::setTimingUpdateDivisor(ec_direction_t direction,
                                        uint32_t updateDivisor) {
  if (!updateDivisor) {
    return ERROR_EC_SLAVE_ENTRY_INFO_STRUCT_NULL;
  }
  ecmcEcTimingOverride *timingOverride = NULL;
  ecmcEcEndpointTiming *endpoint = NULL;
  const ecmcEcSmTiming *smTiming = NULL;
  if (direction == EC_DIR_INPUT) {
    timingOverride = &inputTimingOverride_;
    endpoint = &inputTiming_;
    smTiming = &inputSmTiming_;
  } else if (direction == EC_DIR_OUTPUT) {
    timingOverride = &outputTimingOverride_;
    endpoint = &outputTiming_;
    smTiming = &outputSmTiming_;
  } else {
    return ERROR_EC_SLAVE_ENTRY_INFO_STRUCT_NULL;
  }
  timingOverride->updateDivisorConfigured = true;
  timingOverride->updateDivisor = updateDivisor;
  const ecmcEcTimestampConfig& timestampConfig =
    direction == EC_DIR_INPUT ? inputTimestampConfig_ : outputTimestampConfig_;
  resolveEndpointTiming(*smTiming, *timingOverride, timestampConfig, direction,
                        endpoint);
  timingAsynDirty_ = true;
  return 0;
}

int ecmcEcSlave::linkTimingTimestamp(ec_direction_t direction,
                                     const std::string& entryId,
                                     uint8_t bits,
                                     int32_t correctionNs) {
  if ((direction != EC_DIR_INPUT && direction != EC_DIR_OUTPUT) ||
      (bits != 32 && bits != 64)) {
    return ERROR_EC_SLAVE_ENTRY_INFO_STRUCT_NULL;
  }
  ecmcEcEntry *entry = findEntry(entryId);
  if (!entry || entry->getDirection() != EC_DIR_INPUT) {
    return ERROR_EC_SLAVE_ENTRY_INFO_STRUCT_NULL;
  }
  ecmcEcTimestampConfig *config = direction == EC_DIR_INPUT ?
    &inputTimestampConfig_ : &outputTimestampConfig_;
  ecmcEcEntry **timestampEntry = direction == EC_DIR_INPUT ?
    &inputTimestampEntry_ : &outputTimestampEntry_;
  config->configured = true;
  config->bits = bits;
  config->correctionNs = correctionNs;
  *timestampEntry = entry;
  resolveEndpointTiming(direction == EC_DIR_INPUT ? inputSmTiming_ :
                        outputSmTiming_,
                        direction == EC_DIR_INPUT ? inputTimingOverride_ :
                        outputTimingOverride_, *config,
                        direction,
                        direction == EC_DIR_INPUT ? &inputTiming_ :
                        &outputTiming_);
  timingAsynDirty_ = true;
  return 0;
}

bool ecmcEcSlave::readTimingTimestamp(ec_direction_t direction,
                                      uint64_t nearbyDcTimeNs,
                                      uint64_t *eventTimeNs) const {
  if (!eventTimeNs) {
    return false;
  }
  const ecmcEcTimestampConfig& config = direction == EC_DIR_INPUT ?
    inputTimestampConfig_ : outputTimestampConfig_;
  ecmcEcEntry *entry = direction == EC_DIR_INPUT ? inputTimestampEntry_ :
                                                    outputTimestampEntry_;
  if ((direction != EC_DIR_INPUT && direction != EC_DIR_OUTPUT) ||
      !config.configured || !entry) {
    return false;
  }
  uint64_t raw = 0;
  if (entry->readValue(&raw)) {
    return false;
  }
  uint64_t timestamp = config.bits == 32 ?
    ecmcEcExtendDcTimestamp32(static_cast<uint32_t>(raw), nearbyDcTimeNs) : raw;
  if (config.correctionNs < 0) {
    const uint64_t correction =
      static_cast<uint64_t>(-static_cast<int64_t>(config.correctionNs));
    if (correction > timestamp) {
      return false;
    }
    timestamp -= correction;
  } else {
    if (timestamp > std::numeric_limits<uint64_t>::max() -
                    static_cast<uint32_t>(config.correctionNs)) {
      return false;
    }
    timestamp += static_cast<uint32_t>(config.correctionNs);
  }
  *eventTimeNs = timestamp;
  return true;
}

void ecmcEcSlave::resolveEndpointTiming(
  const ecmcEcSmTiming& smTiming,
  const ecmcEcTimingOverride& timingOverride,
  const ecmcEcTimestampConfig& timestampConfig,
  ec_direction_t direction,
  ecmcEcEndpointTiming *endpoint) {
  if (!endpoint) {
    return;
  }
  *endpoint = ecmcEcEndpointTiming();
  endpoint->reference = direction == EC_DIR_OUTPUT ?
    ecmcEcTimingReference::APPLICATION : ecmcEcTimingReference::RECEIVE;
  endpoint->uncertaintyNs = smTiming.cycleTimeNs.available ?
    smTiming.cycleTimeNs.value : (dcConfig_.sync0CycleNs ?
    dcConfig_.sync0CycleNs : nominalTimingCycleNs_);
  endpoint->valid = direction == EC_DIR_OUTPUT ? hasProcessDataOutput_ :
                                                 hasProcessDataInput_;
  if (!dcConfig_.configured) {
    endpoint->source = ecmcEcTimingSource::CYCLE_ONLY;
    endpoint->cycleOffsetKnown = true;
    endpoint->eventOffsetKnown = true;
  }

  if (smTiming.calculationCopyTimeNs.available) {
    endpoint->calculationCopyTimeAvailable = true;
    endpoint->calculationCopyTimeNs = smTiming.calculationCopyTimeNs.value;
  }

  // ETG SyncManager synchronization types: 2=DC SYNC0, 3=DC SYNC1.
  // Free-run and SM-event endpoints remain cycle-only because they have no
  // phase that can be related generically to the common DC clock.
  if (dcConfig_.configured && smTiming.syncType.available) {
    if (smTiming.syncType.value == 2) {
      endpoint->source = ecmcEcTimingSource::SYNC0_DERIVED;
      endpoint->reference = ecmcEcTimingReference::SYNC;
      endpoint->valid = true;
    } else if (smTiming.syncType.value == 3) {
      endpoint->source = ecmcEcTimingSource::SYNC1_DERIVED;
      endpoint->reference = ecmcEcTimingReference::SYNC;
      endpoint->valid = true;
    }
  } else if (smTiming.syncType.available) {
    endpoint->source = ecmcEcTimingSource::CYCLE_ONLY;
    endpoint->valid = true;
  }

  if (timingOverride.sourceConfigured) {
    endpoint->source = timingOverride.source;
    endpoint->reference = timingOverride.source ==
      ecmcEcTimingSource::CYCLE_ONLY ?
      (direction == EC_DIR_OUTPUT ? ecmcEcTimingReference::APPLICATION :
                                    ecmcEcTimingReference::RECEIVE) :
      ecmcEcTimingReference::SYNC;
    endpoint->valid = true;
    endpoint->overrideApplied = true;
  }

  // A DC synchronization type establishes the nominal event at the selected
  // SYNC. Keep 0x1C32/0x1C33:03 as raw diagnostics: its physical meaning is
  // terminal-specific and it must not be added generically. Hardware config
  // can supply a documented correction through the timing override.
  if (endpoint->source == ecmcEcTimingSource::SYNC0_DERIVED ||
      endpoint->source == ecmcEcTimingSource::SYNC1_DERIVED) {
    endpoint->eventOffsetNs = 0;
    endpoint->eventOffsetKnown = true;
  }

  if (timingOverride.configured) {
    endpoint->valid = true;
    endpoint->cycleOffset = timingOverride.cycleOffset;
    endpoint->eventOffsetNs += timingOverride.eventOffsetNs;
    endpoint->uncertaintyNs = timingOverride.uncertaintyNs;
    endpoint->cycleOffsetKnown = true;
    endpoint->eventOffsetKnown = true;
    endpoint->overrideApplied = true;
  }
  if (timingOverride.updateDivisorConfigured) {
    endpoint->updateDivisor = timingOverride.updateDivisor;
    endpoint->overrideApplied = true;
  }
  if (timestampConfig.configured) {
    endpoint->source = ecmcEcTimingSource::HARDWARE_TIMESTAMP;
    endpoint->valid = true;
    endpoint->timestampLinked = true;
    endpoint->timestampBits = timestampConfig.bits;
    endpoint->timestampCorrectionNs = timestampConfig.correctionNs;
  }
}

const ecmcEcEndpointTiming& ecmcEcSlave::getInputTiming() const {
  return inputTiming_;
}

const ecmcEcEndpointTiming& ecmcEcSlave::getOutputTiming() const {
  return outputTiming_;
}

void ecmcEcSlave::printSmTiming(const char *name,
                                const ecmcEcSmTiming& timing) const {
  printf("# DC %s timing object 0x%04x%s\n",
         name,
         timing.objectIndex,
         !timing.discoveryAttempted ? " (not requested)" :
         (timing.discoveryComplete ? "" : " (pending)"));
  if (!timing.discoveryAttempted) {
    return;
  }

  const uint8_t subIndices[] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x20
  };
  const char *labels[] = {
    "syncType", "cycleTimeNs", "shiftTimeNs", "syncTypesSupported",
    "minimumCycleTimeNs", "calculationCopyTimeNs", "minimumDelayTimeNs",
    "command", "maximumDelayTimeNs", "synchronizationError"
  };
  const ecmcEcSmTimingValue *values[] = {
    &timing.syncType, &timing.cycleTimeNs, &timing.shiftTimeNs,
    &timing.syncTypesSupported, &timing.minimumCycleTimeNs,
    &timing.calculationCopyTimeNs, &timing.minimumDelayTimeNs,
    &timing.command, &timing.maximumDelayTimeNs,
    &timing.synchronizationError
  };
  for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
    if (values[i]->available) {
      printf("#   0x%04x:%02x %-23s = %u (0x%x, %u bytes)\n",
             timing.objectIndex, subIndices[i], labels[i], values[i]->value,
             values[i]->value, values[i]->byteSize);
    } else {
      printf("#   0x%04x:%02x %-23s = unavailable"
             " (request=%d, abort=0x%x)\n",
             timing.objectIndex, subIndices[i], labels[i],
             values[i]->requestError, values[i]->abortCode);
    }
  }
}

void ecmcEcSlave::printDcTiming() const {
  printf("# DC configured=%d assignActivate=0x%x sync0CycleNs=%u "
         "sync0ShiftNs=%d sync1OffsetNs=%u sync1ShiftNs=%d\n",
         dcConfig_.configured, dcConfig_.assignActivate,
         dcConfig_.sync0CycleNs, dcConfig_.sync0ShiftNs,
         dcConfig_.sync1OffsetNs, dcConfig_.sync1ShiftNs);
  printf("# DC ESC schedule: attempted=%d complete=%d startTimeNs=%llu "
         "sync0CycleNs=%u sync1CycleNs=%u error=%d\n",
         dcSchedule_.discoveryAttempted, dcSchedule_.discoveryComplete,
         static_cast<unsigned long long>(dcSchedule_.startTimeNs),
         dcSchedule_.sync0CycleNs, dcSchedule_.sync1CycleNs,
         dcSchedule_.requestError);
  printSmTiming("output", outputSmTiming_);
  printSmTiming("input", inputSmTiming_);
  printEndpointTiming("output", outputTiming_);
  printEndpointTiming("input", inputTiming_);
}

void ecmcEcSlave::printEndpointTiming(
  const char *name,
  const ecmcEcEndpointTiming& timing) const {
  const char *source = "cycle-only";
  if (timing.source == ecmcEcTimingSource::SYNC0_DERIVED) {
    source = "SYNC0";
  } else if (timing.source == ecmcEcTimingSource::SYNC1_DERIVED) {
    source = "SYNC1";
  } else if (timing.source == ecmcEcTimingSource::HARDWARE_TIMESTAMP) {
    source = "timestamp";
  }
  const char *reference = "receive";
  if (timing.reference == ecmcEcTimingReference::APPLICATION) {
    reference = "application";
  } else if (timing.reference == ecmcEcTimingReference::SYNC) {
    reference = "SYNC";
  }
  printf("# Resolved %s timing: valid=%d source=%s reference=%s "
         "pdoCycleOffset=%d%s "
         "eventOffsetNs=%d%s updateDivisor=%u uncertaintyNs=%u override=%d",
         name, timing.valid, source, reference, timing.cycleOffset,
         timing.cycleOffsetKnown ? "" : " (unknown)", timing.eventOffsetNs,
         timing.eventOffsetKnown ? "" : " (unknown)", timing.updateDivisor,
         timing.uncertaintyNs, timing.overrideApplied);
  if (timing.timestampLinked) {
    printf(" timestampBits=%u timestampCorrectionNs=%d",
           timing.timestampBits, timing.timestampCorrectionNs);
  }
  if (timing.calculationCopyTimeAvailable) {
    printf(" calculationCopyTimeNs=%u (reported, not automatically additive)",
           timing.calculationCopyTimeNs);
  }
  printf("\n");
}

ecmcEcEntry * ecmcEcSlave::findEntry(std::string id) {
  for (uint i = 0; i < entryCounter_; i++) {
    if (entryList_[i]) {
      if (entryList_[i]->matchesIdentificationName(id)) {
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
  for (size_t i = 0; i < simEntries_.size(); i++) {
    if (simEntries_[i] != NULL) {
      if (simEntries_[i]->matchesIdentificationName(id)) {
        return simEntries_[i];
      }
    }
  }
  return NULL;
}

int ecmcEcSlave::findEntryIndex(std::string id) {
  // Real entries
  const int realEntryCount = entryCounter_;
  for (int i = 0; i < realEntryCount; i++) {
    if (entryList_[i] != NULL) {
      if (entryList_[i]->matchesIdentificationName(id)) {
        return i;
      }
    }
  }

  // Simulation entries
  for (size_t i = 0; i < simEntries_.size(); i++) {
    if (simEntries_[i] != NULL) {
      if (simEntries_[i]->matchesIdentificationName(id)) {
        return static_cast<int>(i);
      }
    }
  }
  ecmcRtLoggerLogError("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry not found (0x%x).\n",
         __FILE__,
         __FUNCTION__,
         __LINE__,
         slavePosition_,
         vendorId_,
         productCode_,
         ERROR_EC_SLAVE_ENTRY_NULL);
  return -ERROR_EC_SLAVE_ENTRY_NULL;
}

int ecmcEcSlave::addEntryAlias(std::string entryId,
                               std::string alias) {
  ecmcEcEntry *entry = findEntry(entryId);

  if (!entry) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry %s not found (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      vendorId_,
      productCode_,
      entryId.c_str(),
      ERROR_EC_SLAVE_ENTRY_NULL);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_EC_SLAVE_ENTRY_NULL);
  }

  ecmcEcEntry *aliasEntry = findEntry(alias);
  if (aliasEntry && (aliasEntry != entry)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry alias %s already maps to another entry (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      vendorId_,
      productCode_,
      alias.c_str(),
      ERROR_EC_SLAVE_ENTRY_ALIAS_EXISTS);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SLAVE_ENTRY_ALIAS_EXISTS);
  }

  return entry->addAlias(alias);
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
    ecmcRtLoggerLogError(
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
    ecmcRtLoggerLogError(
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
    ecmcRtLoggerLogError(
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
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Failed to generate alias. Buffer too small (0x%x).\n",
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
    ecmcRtLoggerLogError(
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

void ecmcEcSlave::updateTimingAsynData() {
  const ecmcEcEndpointTiming *endpoint[] = {&inputTiming_, &outputTiming_};
  const ecmcEcSmTiming *sm[] = {&inputSmTiming_, &outputSmTiming_};
  ecmcEcTimingDiag *data[] = {&inputTimingAsynData_, &outputTimingAsynData_};
  for (size_t i = 0; i < 2; ++i) {
    uint32_t status = 0;
    status |= endpoint[i]->valid ? 1u << 0 : 0;
    status |= dcConfig_.configured ? 1u << 1 : 0;
    status |= sm[i]->discoveryComplete ? 1u << 2 : 0;
    status |= endpoint[i]->overrideApplied ? 1u << 3 : 0;
    status |= endpoint[i]->timestampLinked ? 1u << 4 : 0;
    status |= endpoint[i]->calculationCopyTimeAvailable ? 1u << 5 : 0;
    status |= endpoint[i]->cycleOffsetKnown ? 1u << 6 : 0;
    status |= endpoint[i]->eventOffsetKnown ? 1u << 7 : 0;
    data[i]->status = static_cast<int32_t>(status);
    data[i]->source = static_cast<int32_t>(endpoint[i]->source);
    data[i]->reference = static_cast<int32_t>(endpoint[i]->reference);
    data[i]->syncType = sm[i]->syncType.available ?
      static_cast<int32_t>(sm[i]->syncType.value) : -1;
    data[i]->cycleOffset = endpoint[i]->cycleOffset;
    data[i]->updateDivisor = endpoint[i]->updateDivisor;
    data[i]->timestampBits = endpoint[i]->timestampBits;
    data[i]->cycleTimeNs = sm[i]->cycleTimeNs.available ?
      sm[i]->cycleTimeNs.value : (dcConfig_.sync0CycleNs ?
      dcConfig_.sync0CycleNs : nominalTimingCycleNs_);
    data[i]->shiftTimeNs = sm[i]->shiftTimeNs.available ?
      static_cast<int32_t>(sm[i]->shiftTimeNs.value) : 0;
    data[i]->calculationCopyTimeNs =
      endpoint[i]->calculationCopyTimeAvailable ?
      endpoint[i]->calculationCopyTimeNs : 0;
    data[i]->eventOffsetNs = endpoint[i]->eventOffsetNs;
    data[i]->uncertaintyNs = endpoint[i]->uncertaintyNs;
    data[i]->timestampCorrectionNs = endpoint[i]->timestampCorrectionNs;
  }
}

void ecmcEcSlave::refreshTimingAsyn() {
  if (!timingAsynDirty_) {
    return;
  }
  updateTimingAsynData();
  ecmcRtLoggerPortDriverSetEcTiming(slavePosition_, 0,
                                    &inputTimingAsynData_);
  ecmcRtLoggerPortDriverSetEcTiming(slavePosition_, 1,
                                    &outputTimingAsynData_);
  timingAsynDirty_ = false;
}

bool ecmcEcSlave::timingStartupReady() const {
  return timingStartupReady_.load(std::memory_order_acquire) &&
         ecmcRtLoggerPortDriverEcTimingPublished(slavePosition_);
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

  // Check that SDO settings are made if needed for channels (if used in motion axis)
  if(enableSDOCheck_) {
    for(uint ch = 0; ch < sdoChVerify_.size(); ch++) {
      if(sdoChVerify_[ch].needSdo && !sdoChVerify_[ch].sdosDone) {
        ecmcRtLoggerLogError(
          "%s/%s:%d: ERROR: Important SDO settings, i.e. max current or other, missing for slave %d, ch %d.\n"
          "Use \"ecmcConfigOrDie \"Cfg.EcSetSlaveSDOSettingsDone(<slave_id>,<ch_id>,1)\"\" after addSlave.cmd to override\n",
          __FILE__,
          __FUNCTION__,
          __LINE__, slavePosition_, ch + 1);
        return ERROR_EC_SLAVE_SDO_SETTINGS_MISSING;
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

  // Only add to real-time lists if in real-time.
  if (useInRealTime) {
    if (entry->getDirection() == EC_DIR_INPUT) {
      if (entryCounterRtInput_ >= EC_MAX_ENTRIES) {
        return ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE;
      }
      entryListRtInput_[entryCounterRtInput_] = entry;
      entryCounterRtInput_++;
    } else if (entry->getDirection() == EC_DIR_OUTPUT || entry->getSimEntry()) {
      if (entryCounterRtOutput_ >= EC_MAX_ENTRIES) {
        return ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE;
      }
      entryListRtOutput_[entryCounterRtOutput_] = entry;
      entryCounterRtOutput_++;
    }
  }
  return 0;
}

int ecmcEcSlave::addSDOWriteComplete(uint16_t    sdoIndex,
                                     const char *dataBuffer,
                                     int         byteSize) {
  if (!slaveConfig_) {
    ecmcRtLoggerLogError(
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
    ecmcRtLoggerLogError(
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
  if ((dt == ECMC_EC_NONE) ||
      (dt == ECMC_EC_B1) ||
      (dt == ECMC_EC_B2) ||
      (dt == ECMC_EC_B3) ||
      (dt == ECMC_EC_B4)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Slave %d: Async SDO 0x%x:%x datatype invalid (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      slavePosition_,
      sdoIndex,
      sdoSubIndex,
      ERROR_EC_SDO_DATATYPE_ERROR);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SDO_DATATYPE_ERROR);
  }

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
    ecmcRtLoggerLogError(
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
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Entry NULL (0x%x).\n",
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
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Entry NULL (0x%x).\n",
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

/* The two commands, setNeedSDOSettings and setSDOSettingsDone, is
   typically set from the configuration frame work:
  * setNeedSDOSettings from addSlave.cmd for certain hardware
  * setSDOSettingsDone from ecmccomp applyComponent.cmd, configureSlave.cmd or applySkaveConfig.cmd.
  Note: chId start count at 1
*/ 
int ecmcEcSlave::setNeedSDOSettings(int chId, int need) {
  
  if(chId <= 0 || chId > SDO_SETTINGS_MAX_CH) {
    return ERROR_EC_SLAVE_SDO_CH_ID_OUT_OF_RANGE;
  }

  sdoVerifyChX temp;
  temp.needSdo = 0; 
  temp.sdosDone = 0;

  // Ensure vector is long enough..
  while(sdoChVerify_.size() < (uint)chId) {
    sdoChVerify_.push_back(temp);
  }
  
  // Zero based index
  sdoChVerify_[chId-1].needSdo = need;
  return 0;
}

// chId start count at 1
int ecmcEcSlave::setSDOSettingsDone(int chId, int done) {

  if(chId <= 0 || chId > SDO_SETTINGS_MAX_CH) {
    return ERROR_EC_SLAVE_SDO_CH_ID_OUT_OF_RANGE; 
  }

  sdoVerifyChX temp;
  temp.needSdo = 0; 
  temp.sdosDone = 0;

  // Ensure vector is long enough..
  while(sdoChVerify_.size() < (uint)chId) {
    sdoChVerify_.push_back(temp);
  }

  // Zero based index
  sdoChVerify_[chId-1].sdosDone = done;

  return 0;
}

int ecmcEcSlave::setEnableSDOCheck(int enable) {
  enableSDOCheck_ = enable;
  return 0;
}

int ecmcEcSlave::setPortWiringCheckEnable(int enable) {
  portWiringCheckEnabled_ = enable != 0;
  return 0;
}

bool ecmcEcSlave::getPortWiringCheckEnable() const {
  return portWiringCheckEnabled_;
}

int ecmcEcSlave::addSimEntry(std::string    id,
                             ecmcEcDataType dt,
                             uint64_t value) {
  uint64_t *simBuffer = new uint64_t;

  if (!simBuffer) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MAIN_EXCEPTION);
  }

  *simBuffer = value;
  
  // Entry
  ecmcEcEntry *entry = new ecmcEcEntry(asynPortDriver_,
                                       masterId_,
                                       slavePosition_,
                                       (uint8_t*)simBuffer,
                                       dt,
                                       id);
  if (!entry) {
    delete simBuffer;
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MAIN_EXCEPTION);
  }

  int errorCode = entry->getErrorID();
  if (errorCode) {
    delete entry;
    delete simBuffer;
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  entry->writeValue(value);

  errorCode = appendEntryToList(entry, 1);
  if (errorCode) {
    delete entry;
    delete simBuffer;
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  simBuffer_.push_back(simBuffer);
  simEntries_.push_back(entry);
  return 0;
}
