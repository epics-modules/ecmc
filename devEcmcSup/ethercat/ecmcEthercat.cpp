/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEthercat.cpp
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#include "../com/ecmcOctetIF.h"        // Log Macros
#include "../main/ecmcErrorsList.h"
#include "../main/ecmcDefinitions.h"

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
  
  // Sample rate fixed
  sampleRateChangeAllowed = 0;
  int errorCode = ec->init(masterIndex);
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
  if(ec->getMasterIndex() != masterIndex) {
    return ERROR_MAIN_EC_INDEX_OUT_OF_RANGE;
  }

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

// Old syntax still supported
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
    "%s/%s:%d slave=%d vendor=%d productcode=%d direction=%d sm=%d pdoindex=%d entry_index=%d entry_subindex=%d bits=%d id=%s,signed=%d\n",
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
    entryIDString,
    signedValue);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  // Old syntax only vaid for integers use "Cfg.EcAddEntry()" for double, real
  ecmcEcDataType dataType = getEcDataType(bits,signedValue);

  return ec->addEntry(position,
                     vendorId,
                     productCode,
                     (ec_direction_t)direction,
                     syncMangerIndex,
                     pdoIndex,
                     entryIndex,
                     entrySubIndex,
                     dataType,
                     id,
                     1); //Update in realtime as default
}

// New syntax
int ecAddEntry(
  uint16_t position,
  uint32_t vendorId,
  uint32_t productCode,
  int      direction,
  uint8_t  syncMangerIndex,
  uint16_t pdoIndex,
  uint16_t entryIndex,
  uint8_t  entrySubIndex,
  char    *datatype,  
  char    *entryIDString,
  int      updateInRealtime  
  ) {
  std::string id = entryIDString;

  LOGINFO4(
    "%s/%s:%d slave=%d vendor=%d productcode=%d direction=%d sm=%d pdoindex=%d entry_index=%d entry_subindex=%d datatype=%s id=%s rt=%d\n",
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
    entryIDString,
    datatype,
    updateInRealtime);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcDataType dt = getEcDataTypeFromStr(datatype);

  return ec->addEntry(position,
                     vendorId,
                     productCode,
                     (ec_direction_t)direction,
                     syncMangerIndex,
                     pdoIndex,
                     entryIndex,
                     entrySubIndex,
                     dt,
                     id,
                     updateInRealtime);
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

// New syntax with datatype
int ecAddMemMapDT(  
  char    *ecPath,
  size_t   byteSize,
  int      direction,
  char    *dataType, 
  char    *memMapIDString
  ) {

  LOGINFO4(
    "%s/%s:%d startEntryID=%s byteSize=%zu, direction=%d dataType=%s entryId=%s\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    ecPath,
    byteSize,
    direction,
    dataType,
    memMapIDString);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  int  masterId   = -1;
  int  slaveIndex = -1;
  char alias[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  bitIndex = -1;

  int errorCode = parseEcPath(ecPath, &masterId, &slaveIndex, alias, &bitIndex);

  if (errorCode) {
    return errorCode;
  }
  
  std::string memMapId     = memMapIDString;
  std::string startEntryId = alias;
  ecmcEcDataType dt = getEcDataTypeFromStr(dataType);

  return ec->addMemMap(slaveIndex, startEntryId, byteSize,
                      (ec_direction_t)direction, dt, memMapId);
}

//Legacy syntax support
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
    "%s/%s:%d startEntryBusPosition=%d, startEntryID=%s byteSize=%zu, direction=%d entryId=%s\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    startEntryBusPosition,
    startEntryIDString,
    byteSize,
    direction,    
    memMapIDString);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;
  
    return ec->addMemMap(startEntryBusPosition, startEntryId, byteSize,
                      (ec_direction_t)direction, ECMC_EC_NONE, memMapId);
}

int  ecGetMemMapId(char* memMapIDString, int *id) {
  
  std::string memMapId = memMapIDString;

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;
  
  int idLocal = ec->findMemMapId(memMapId);
  
  *id =  idLocal;

  return 0;
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
    "%s/%s:%d slave_position=%d sdo_index=%d sdo_subindex=%d value=0x%x bytesize=%d\n",
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


int ecAddSdoComplete(uint16_t    slavePosition,
                     uint16_t    sdoIndex,                     
                     const char* valueBuffer,
                     int         byteSize) {
  LOGINFO4(
    "%s/%s:%d slave_position=%d sdo_index=%d value=%s bytesize=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slavePosition,
    sdoIndex,    
    valueBuffer,
    byteSize);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return  ec->addSDOWriteComplete(slavePosition,
                                  sdoIndex,
                                  valueBuffer,
                                  byteSize);
}

int ecAddSdoBuffer(uint16_t    slavePosition,
                   uint16_t    sdoIndex,
                   uint8_t     sdoSubIndex,
                   const char* valueBuffer,
                   int         byteSize) {
  LOGINFO4(
    "%s/%s:%d slave_position=%d sdo_index=%d sdo_sub_index=%d value=%s bytesize=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slavePosition,
    sdoIndex,
    sdoSubIndex,
    valueBuffer,
    byteSize);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return  ec->addSDOWriteBuffer(slavePosition,
                                sdoIndex,
                                sdoSubIndex,
                                valueBuffer,
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

/*int ecWriteSdoComplete(uint16_t slavePosition,
                       uint16_t sdoIndex,
                       uint32_t value,
                       int      byteSize) {
  LOGINFO4("%s/%s:%d slave_position=%d sdo_index=%d value=0x%x bytesize=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           slavePosition,
           sdoIndex,
           value,
           byteSize);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec->writeSDOComplete(slavePosition, sdoIndex, value, byteSize);
}*/

int ecReadSdo(uint16_t  slavePosition,
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

int ecVerifySdo(uint16_t  slavePosition,
                uint16_t  sdoIndex,
                uint8_t   sdoSubIndex,
                uint32_t  verValue,
                int       byteSize
                ) {
  LOGINFO4(
    "%s/%s:%d slave_position=%d sdo_index=%d sdo_subindex=%d bytesize=%d verValue=0x%x\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slavePosition,
    sdoIndex,
    sdoSubIndex,
    byteSize,
    verValue);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;
  
  uint32_t readValue = 0;
  int errorCode=ec->readSDO(slavePosition, sdoIndex, sdoSubIndex, byteSize, &readValue);
  if(errorCode) {
    return errorCode;
  }

  if(readValue != verValue) {
    LOGERR("%s/%s:%d: ERROR: Verification of SDO failed (%u != %u) (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           readValue,
           verValue,
           ERROR_MAIN_EC_SDO_VERIFICATION_FAIL);
    return ERROR_MAIN_EC_SDO_VERIFICATION_FAIL;
  }
  
  return 0;
}

int ecReadSoE(uint16_t  slavePosition, /**< Slave position. */
                   uint8_t   driveNo, /**< Drive number. */
                   uint16_t  idn, /**< SoE IDN (see ecrt_slave_config_idn()). */
                   size_t    byteSize, /**< Size of data to write. */
                   uint8_t  *value /**< Pointer to data to write. */
                   ){
  LOGINFO4(
    "%s/%s:%d slave_position=%d drive no=%d idn=%u bytesize=%zu\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slavePosition,
    driveNo,
    idn,
    byteSize);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec->readSoE(slavePosition, driveNo, idn, byteSize, value);
}

int ecWriteSoE(uint16_t  slavePosition, /**< Slave position. */
                   uint8_t   driveNo, /**< Drive number. */
                   uint16_t  idn, /**< SoE IDN (see ecrt_slave_config_idn()). */
                   size_t    byteSize, /**< Size of data to write. */
                   uint8_t  *value /**< Pointer to data to write. */
                   ){
  LOGINFO4(
    "%s/%s:%d slave_position=%d drive no=%d idn=%u bytesize=%zu\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    slavePosition,
    driveNo,
    idn,
    byteSize);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec->writeSoE(slavePosition, driveNo, idn, byteSize, value);
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
  LOGINFO4("%s/%s:%d alias=%s bytesToRead=%zu\n",
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
                  uint32_t productCode,  /**< Expected product code. */
                  uint32_t revisionNum  /**< Revision number*/) {

  LOGINFO4("%s/%s:%d alias=%d slavePos=%d, vendorId=0x%x, productCode=0x%x, revisionNum=0x%x\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           alias,
           slavePos,
           vendorId,
           productCode,
           revisionNum);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  return ec->verifySlave(alias,slavePos,vendorId,productCode, revisionNum);
}

int ecGetSlaveVendorId(uint16_t alias,  /**< Slave alias. */
                            uint16_t slavePos,   /**< Slave position. */
                            uint32_t *result) {
  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;
  
  *result = ec->getSlaveVendorId(alias, slavePos);
  return 0;
}

int ecGetSlaveProductCode(uint16_t alias,  /**< Slave alias. */
                               uint16_t slavePos,   /**< Slave position. */
                               uint32_t *result) {
  if (!ec->getInitDone()) return 0;
  
  *result = ec->getSlaveProductCode(alias, slavePos);
  return 0;
}

int ecGetSlaveRevisionNum(uint16_t alias,  /**< Slave alias. */
                               uint16_t slavePos,   /**< Slave position. */
                               uint32_t *result) {
  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;
  
  *result = ec->getSlaveRevisionNum(alias, slavePos);
  return 0;
}

int ecGetSlaveSerialNum(uint16_t alias,  /**< Slave alias. */
                             uint16_t slavePos,   /**< Slave position. */
                             uint32_t *result) {
  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;
  
  *result = ec->getSlaveSerialNum(alias, slavePos);
  return 0;
}

int ecUseClockRealtime(int useClkRT) {
  LOGINFO4("%s/%s:%d useClkRT=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           useClkRT);
  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;
  
  return ec->useClockRealtime(useClkRT);
}
