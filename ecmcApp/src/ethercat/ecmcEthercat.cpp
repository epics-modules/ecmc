#include "ecmcEthercat.h"
#include "ecmcEc.h"
#include "ecmcEcPdo.h"
#include "ecmcEcSlave.h"
#include "ecmcEcSyncManager.h"
#include "ecmcEcEntry.h"

#include "ecmcGlobalsExtern.h"

int ecSetMaster(int masterIndex) {
  LOGINFO4("%s/%s:%d masterIndex=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           masterIndex);
  int errorCode = ec->setAsynPortDriver(asynPort);
  if(errorCode) {
    return errorCode;
  }

  errorCode = ec->init(masterIndex);
  if(errorCode) {
    return errorCode;
  }
  return 0;
}

int ecResetMaster(int masterIndex) {
  LOGINFO4("%s/%s:%d master index=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           masterIndex);

  /// todo  master index not used. Only there for future use.
  return ec->reset();
}

int ecResetError() {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);

  ec->errorReset();
  return 0;
}

int ecAddSlave(uint16_t alias,
               uint16_t position,
               uint32_t vendorId,
               uint32_t productCode) {
  LOGINFO4("%s/%s:%d alias=%d position=%d vendor_id=%d product_code=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           alias,
           position,
           vendorId,
           productCode);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ec->addSlave(alias, position, vendorId, productCode);
  return 0;
}

int ecSlaveConfigDC(
  int      slaveBusPosition,
  uint16_t assignActivate,   /**< AssignActivate word. */
  uint32_t sync0Cycle,   /**< SYNC0 cycle time [ns]. */
  int32_t  sync0Shift,  /**< SYNC0 shift time [ns]. */
  uint32_t sync1Cycle,   /**< SYNC1 cycle time [ns]. */
  int32_t  sync1Shift  /**< SYNC1 shift time [ns]. */) {
  LOGINFO4(
    "%s/%s:%d position=%d assign_active=%x sync0_cycle=%d sync0_shift=%d sync1_cycle=%d sync1_shift=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slaveBusPosition,
    assignActivate,
    sync0Cycle,
    sync0Shift,
    sync1Cycle,
    sync1Shift);

  ecmcEcSlave *slave = ec->findSlave(slaveBusPosition);

  if (slave == NULL) {
    return ERROR_EC_MAIN_SLAVE_NULL;
  }

  return slave->configDC(assignActivate,
                         sync0Cycle,
                         sync0Shift,
                         sync1Cycle,
                         sync1Shift);
}

int ecSelectReferenceDC(int masterIndex, int slaveBusPosition) {
  LOGINFO4("%s/%s:%d master=%d position=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           masterIndex,
           slaveBusPosition);

  ecmcEcSlave *slave = ec->findSlave(slaveBusPosition);

  if (slave == NULL) {
    return ERROR_EC_MAIN_SLAVE_NULL;
  }

  return slave->selectAsReferenceDC();
}

int ecAddEntryComplete(
  uint16_t position,
  uint32_t vendorId,
  uint32_t productCode,
  int      direction,
  uint8_t  syncMangerIndex,
  uint16_t pdoIndex,
  uint16_t entryIndex,
  uint8_t  entrySubIndex,
  uint8_t  bits,
  char    *entryIDString,
  int      signedValue
  ) {
  std::string id = entryIDString;

  LOGINFO4(
    "%s/%s:%d slave=%d vendor=%d productcode=%d direction=%d sm=%d pdoindex=%d entry_index=%d entry_subindex=%d bits=%d id=%s\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    position,
    vendorId,
    productCode,
    direction,
    syncMangerIndex,
    pdoIndex,
    entryIndex,
    entrySubIndex,
    bits,
    entryIDString);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec->addEntry(position,
                     vendorId,
                     productCode,
                     (ec_direction_t)direction,
                     syncMangerIndex,
                     pdoIndex,
                     entryIndex,
                     entrySubIndex,
                     bits,
                     id,
                     signedValue);
}

int ecSetEntryUpdateInRealtime(
  uint16_t slavePosition,
  char    *entryIDString,
  int      updateInRealtime
  ) {
  LOGINFO4("%s/%s:%d slave=%d id=%s updateInRealtime=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slavePosition,
           entryIDString,
           updateInRealtime);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave = NULL;

  if (slavePosition >= 0) {
    slave = ec->findSlave(slavePosition);
  } else {    // simulation slave
    slave = ec->getSlave(slavePosition);
  }

  if (slave == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID = entryIDString;

  ecmcEcEntry *entry = slave->findEntry(sEntryID);

  if (entry == NULL) return ERROR_MAIN_EC_ENTRY_NULL;

  return entry->setUpdateInRealtime(updateInRealtime);
}

int ecAddMemMap(
  uint16_t startEntryBusPosition,
  char    *startEntryIDString,
  size_t   byteSize,
  int      direction,
  char    *memMapIDString
  ) {
  std::string memMapId     = memMapIDString;
  std::string startEntryId = startEntryIDString;

  LOGINFO4(
    "%s/%s:%d startEntryBusPosition=%d, startEntryID=%s byteSize=%lu, direction=%d entryId=%s\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    startEntryBusPosition,
    startEntryIDString,
    byteSize,
    direction,
    memMapIDString);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec->addMemMap(startEntryBusPosition, startEntryId, byteSize, 0,
                      (ec_direction_t)direction, memMapId);
}

int ecAddPdo(int slaveIndex, int syncManager, uint16_t pdoIndex) {
  LOGINFO4("%s/%s:%d slave=%d sm=%d pdo_index=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slaveIndex,
           syncManager,
           pdoIndex);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  if (ec->getSlave(slaveIndex) == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  if (ec->getSlave(slaveIndex)->getSyncManager(syncManager) ==
      NULL) return ERROR_MAIN_EC_SM_NULL;

  return ec->getSlave(slaveIndex)->getSyncManager(syncManager)->addPdo(pdoIndex);
}

int ecAddSyncManager(int slaveIndex, int direction, uint8_t syncMangerIndex) {
  LOGINFO4("%s/%s:%d slave=%d direction=%d sm_index=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slaveIndex,
           direction,
           syncMangerIndex);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  if (ec->getSlave(slaveIndex) == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  return ec->getSlave(slaveIndex)->addSyncManager((ec_direction_t)direction,
                                                 syncMangerIndex);
}

int ecAddSdo(uint16_t slavePosition,
             uint16_t sdoIndex,
             uint8_t  sdoSubIndex,
             uint32_t value,
             int      byteSize) {
  LOGINFO4(
    "%s/%s:%d slave_position=%d sdo_index=%d sdo_subindex=%d value=%d bytesize=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slavePosition,
    sdoIndex,
    sdoSubIndex,
    value,
    byteSize);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return  ec->addSDOWrite(slavePosition,
                         sdoIndex,
                         sdoSubIndex,
                         value,
                         byteSize);
}

int ecWriteSdo(uint16_t slavePosition,
               uint16_t sdoIndex,
               uint8_t  sdoSubIndex,
               uint32_t value,
               int      byteSize) {
  LOGINFO4(
    "%s/%s:%d slave_position=%d sdo_index=%d sdo_subindex=%d value=%d bytesize=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slavePosition,
    sdoIndex,
    sdoSubIndex,
    value,
    byteSize);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec->writeSDO(slavePosition, sdoIndex, sdoSubIndex, value, byteSize);
}

int ecWriteSdoComplete(uint16_t slavePosition,
                       uint16_t sdoIndex,
                       uint32_t value,
                       int      byteSize) {
  LOGINFO4("%s/%s:%d slave_position=%d sdo_index=%d value=%d bytesize=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slavePosition,
           sdoIndex,
           value,
           byteSize);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec->writeSDOComplete(slavePosition, sdoIndex, value, byteSize);
}

uint32_t ecReadSdo(uint16_t  slavePosition,
                   uint16_t  sdoIndex,
                   uint8_t   sdoSubIndex,
                   int       byteSize,
                   uint32_t *value) {
  LOGINFO4(
    "%s/%s:%d slave_position=%d sdo_index=%d sdo_subindex=%d bytesize=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slavePosition,
    sdoIndex,
    sdoSubIndex,
    byteSize);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec->readSDO(slavePosition, sdoIndex, sdoSubIndex, byteSize, value);
}

int ecSlaveConfigWatchDog(int slaveBusPosition,
                          int watchdogDivider,
                          int watchdogIntervals) {
  LOGINFO4("%s/%s:%d position=%d, watchdogDivider=%d watchdogIntervals=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slaveBusPosition,
           watchdogDivider,
           watchdogIntervals);

  ecmcEcSlave *slave = ec->findSlave(slaveBusPosition);

  if (slave == NULL) {
    return ERROR_EC_MAIN_SLAVE_NULL;
  }

  return slave->setWatchDogConfig(watchdogDivider, watchdogIntervals);
}


int readEcMemMap(const char *memMapIDString,
                 uint8_t    *data,
                 size_t      bytesToRead,
                 size_t     *bytesRead) {
  LOGINFO4("%s/%s:%d alias=%s bytesToRead=%lu\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           memMapIDString,
           bytesToRead);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcMemMap *memMap = ec->findMemMap(memMapIDString);

  if (!memMap) {
    return ERROR_MAIN_MEM_MAP_NULL;
  }

  return memMap->read(data, bytesToRead, bytesRead);
}

int writeEcEntry(int slaveIndex, int entryIndex, uint64_t value) {
  LOGINFO4("%s/%s:%d slave_index=%d entry=%d value=%" PRIu64 "\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slaveIndex,
           entryIndex,
           value);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  if (ec->getSlave(slaveIndex) == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  if (ec->getSlave(slaveIndex)->getEntry(entryIndex) ==
      NULL) return ERROR_MAIN_EC_ENTRY_NULL;

  return ec->getSlave(slaveIndex)->getEntry(entryIndex)->writeValueForce(value);
}

int writeEcEntryIDString(int slavePosition, char *entryIDString,
                         uint64_t value) {
  LOGINFO4("%s/%s:%d slave_position=%d entry=%s value=%" PRIu64 "\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slavePosition,
           entryIDString,
           value);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave = NULL;

  if (slavePosition >= 0) {
    slave = ec->findSlave(slavePosition);
  } else {    // simulation slave
    slave = ec->getSlave(slavePosition);
  }

  if (slave == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID = entryIDString;

  ecmcEcEntry *entry = slave->findEntry(sEntryID);

  if (entry == NULL) return ERROR_MAIN_EC_ENTRY_NULL;

  return entry->writeValueForce(value);
}

int readEcEntry(int slaveIndex, int entryIndex, uint64_t *value) {
  LOGINFO4("%s/%s:%d slave_index=%d entry=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slaveIndex,
           entryIndex);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  if (ec->getSlave(slaveIndex) == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  if (ec->getSlave(slaveIndex)->getEntry(entryIndex) ==
      NULL) return ERROR_MAIN_EC_ENTRY_NULL;

  return ec->getSlave(slaveIndex)->getEntry(entryIndex)->readValue(value);
}

int readEcEntryIDString(int slavePosition, char *entryIDString,
                        uint64_t *value) {
  LOGINFO4("%s/%s:%d slave_index=%d entry=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slavePosition,
           entryIDString);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave = NULL;

  if (slavePosition >= 0) {
    slave = ec->findSlave(slavePosition);
  } else {    // simulation slave
    slave = ec->getSlave(slavePosition);
  }

  if (slave == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  std::string tempEntryIDString = entryIDString;

  ecmcEcEntry *entry = slave->findEntry(tempEntryIDString);

  if (entry == NULL) return ERROR_MAIN_EC_ENTRY_NULL;

  return entry->readValue(value);
}

int readEcEntryIndexIDString(int slavePosition, char *entryIDString,
                             int *value) {
  LOGINFO4("%s/%s:%d slave_index=%d entry=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slavePosition,
           entryIDString);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave = NULL;

  if (slavePosition >= 0) {
    slave = ec->findSlave(slavePosition);
  } else {    // simulation slave
    slave = ec->getSlave(slavePosition);
  }

  if (slave == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  std::string tempEntryIDString = entryIDString;

  int entryIndex = slave->findEntryIndex(tempEntryIDString);

  if (entryIndex < 0) return ERROR_MAIN_EC_ENTRY_NULL;

  *value = entryIndex;
  return 0;
}

int readEcSlaveIndex(int slavePosition, int *value) {
  LOGINFO4("%s/%s:%d slave_index=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slavePosition);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec->findSlaveIndex(slavePosition, value);
}

int ecApplyConfig(int masterIndex) {
  // Master index not used right now..
  LOGINFO4("%s/%s:%d value=%d\n", __FILE__, __FUNCTION__, __LINE__,
           masterIndex);

  int errorCode = 0;

  if ((errorCode = ec->compileRegInfo())) {
    LOGERR("ERROR:\tCompileRegInfo failed\n");
    return errorCode;
  }
  return 0;
}

int ecSetDiagnostics(int value) {  // Set diagnostics mode
  LOGINFO4("%s/%s:%d value=%d\n", __FILE__, __FUNCTION__, __LINE__, value);

  return ec->setDiagnostics(value);
}

int ecSetDomainFailedCyclesLimit(int value) {
  LOGINFO4("%s/%s:%d value=%d\n", __FILE__, __FUNCTION__, __LINE__, value);

  return ec->setDomainFailedCyclesLimitInterlock(value);
}

int ecEnablePrintouts(int value) {
  LOGINFO4("%s/%s:%d value=%d\n", __FILE__, __FUNCTION__, __LINE__, value);

  WRITE_DIAG_BIT(FUNCTION_ETHERCAT_DIAGNOSTICS_BIT, value);

  return 0;
}

int ecPrintAllHardware() {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
  return ec->printAllConfig();
}

int ecPrintSlaveConfig(int slaveIndex) {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
  return ec->printSlaveConfig(slaveIndex);
}

int linkEcEntryToEcStatusOutput(int slaveIndex, char *entryIDString) {
  LOGINFO4("%s/%s:%d slave_index=%d entry=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slaveIndex,
           entryIDString);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave = NULL;

  if (slaveIndex >= 0) {
    slave = ec->findSlave(slaveIndex);
  } else {
    slave = ec->getSlave(slaveIndex);
  }

  if (slave == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID = entryIDString;

  ecmcEcEntry *entry = slave->findEntry(sEntryID);

  if (entry == NULL) return ERROR_MAIN_EC_ENTRY_NULL;


  if (entry == NULL) return ERROR_MAIN_EC_ENTRY_NULL;

  return ec->setEcStatusOutputEntry(entry);
}

int ecVerifySlave(uint16_t alias,  /**< Slave alias. */
                  uint16_t slavePos,   /**< Slave position. */
                  uint32_t vendorId,   /**< Expected vendor ID. */
                  uint32_t productCode  /**< Exp)*/) {
  LOGINFO4("%s/%s:%d alias=%d slavePos=%d, vendorId=0x%x, productCode=0x%x\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           alias,
           slavePos,
           vendorId,
           productCode
           );

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec->verifySlave(alias,slavePos,vendorId,productCode);
}
