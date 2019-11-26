/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEc.h
*
*  Created on: Dec 1, 2015
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCEC_H_
#define ECMCEC_H_

#include "stdio.h"
#include "ecrt.h"
#include "../main/ecmcDefinitions.h"
#include "../main/ecmcError.h"
#include "../com/ecmcOctetIF.h"  // Logging macros
#include "../com/ecmcAsynPortDriver.h"
#include "ecmcEcEntry.h"
#include "ecmcEcSDO.h"
#include "ecmcEcSlave.h"
#include "ecmcEcMemMap.h"

// EC ERRORS
#define ERROR_EC_MAIN_REQUEST_FAILED 0x26000
#define ERROR_EC_MAIN_CREATE_DOMAIN_FAILED 0x26001
#define ERROR_EC_MAIN_INVALID_SLAVE_INDEX 0x26002
#define ERROR_EC_MAIN_MASTER_ACTIVATE_FAILED 0x26003
#define ERROR_EC_MAIN_SLAVE_NULL 0x26004
#define ERROR_EC_MAIN_GET_SLAVE_INFO_FAILED 0x26005
#define ERROR_EC_MAIN_ENTRY_NULL 0x26006
#define ERROR_EC_MAIN_GET_ENTRY_INFO_FAILED 0x26007
#define ERROR_EC_MAIN_DOM_REG_PDO_ENTRY_LIST_FAILED 0x26008
#define ERROR_EC_MAIN_SDO_ARRAY_FULL 0x26009
#define ERROR_EC_MAIN_SDO_ENTRY_NULL 0x2600A
#define ERROR_EC_MAIN_SDO_READ_FAILED 0x2600B
#define ERROR_EC_MAIN_DOMAIN_DATA_FAILED 0x2600C
#define ERROR_EC_MAIN_SLAVE_ARRAY_FULL 0x2600D
#define ERROR_EC_AL_STATE_INIT 0x2600E
#define ERROR_EC_AL_STATE_PREOP 0x2600F
#define ERROR_EC_AL_STATE_SAFEOP 0x26010
#define ERROR_EC_LINK_DOWN 0x26011
#define ERROR_EC_RESPOND_VS_CONFIG_SLAVES_MISSMATCH 0x26012
#define ERROR_EC_STATUS_NOT_OK 0x26013
#define ERROR_EC_ALIAS_TO_LONG 0x26014
#define ERROR_EC_ASYN_PORT_OBJ_NULL 0x26015
#define ERROR_EC_ASYN_PORT_CREATE_PARAM_FAIL 0x26016
#define ERROR_EC_ASYN_SKIP_CYCLES_INVALID 0x26017
#define ERROR_EC_MEM_MAP_INDEX_OUT_OF_RANGE 0x26018
#define ERROR_EC_MEM_MAP_START_ENTRY_NULL 0x26019
#define ERROR_EC_MEM_MAP_NULL 0x2601A
#define ERROR_EC_ASYN_ALIAS_NOT_VALID 0x2601B
#define ERROR_EC_AUTO_CONFIG_BUFFER_OVERFLOW 0x2601C
#define ERROR_EC_AUTO_CONFIG_MASTER_INFO_FAIL 0x2601D
#define ERROR_EC_AUTO_CONFIG_SLAVE_INFO_FAIL 0x2601E
#define ERROR_EC_AUTO_CONFIG_SM_INFO_FAIL 0x2601F
#define ERROR_EC_AUTO_CONFIG_PDO_INFO_FAIL 0x2601F
#define ERROR_EC_AUTO_CONFIG_ENTRY_INFO_FAIL 0x26020
#define ERROR_EC_AUTO_CONFIG_MASTER_NOT_SELECTED_FAIL 0x26021
#define ERROR_EC_AUTO_CONFIG_SLAVE_INDEX_OUT_OF_RANGE 0x26022
#define ERROR_EC_AUTO_CONFIG_DIRECTION_INVALID 0x26023
#define ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW 0x26024
#define ERROR_EC_MASTER_NULL 0x26025
#define ERROR_EC_SLAVE_VERIFICATION_FAIL 0x26026
#define ERROR_EC_NO_VALID_CONFIG 0x26027

class ecmcEc : public ecmcError {
 public:
  ecmcEc();
  ~ecmcEc();
  int init(int nMasterIndex);

  // void setMaster(ec_master_t *master);
  int addSlave(
    uint16_t alias,   /**< Slave alias. */
    uint16_t position,   /**< Slave position. */
    uint32_t vendorId,   /**< Expected vendor ID. */
    uint32_t productCode  /**< Expected product code. */);
  ecmcEcSlave* getSlave(int slave);  // NOTE: index not bus position
  ec_domain_t* getDomain();
  ec_master_t* getMaster();
  int          getMasterIndex();
  bool         getInitDone();
  void         receive();
  void         send(timespec timeOffset);
  int          compileRegInfo();
  void         checkDomainState();
  int          checkSlaveConfState(int slave);
  bool         checkSlavesConfState();
  bool         checkState();
  int          activate();
  int          setDiagnostics(bool diag);
  int          addSDOWrite(uint16_t slavePosition,
                           uint16_t sdoIndex,
                           uint8_t  sdoSubIndex,
                           uint32_t value,
                           int      byteSize);
  int writeAndVerifySDOs();
  int readSDO(uint16_t  slavePosition,
              uint16_t  sdoIndex,
              uint8_t   sdoSubIndex,
              int       byteSize,
              uint32_t *value);

  int writeSDO(uint16_t slavePosition,
               uint16_t sdoIndex,
               uint8_t  sdoSubIndex,
               uint32_t value,
               int      byteSize);
  int writeSDOComplete(uint16_t slavePosition,
                       uint16_t sdoIndex,
                       uint32_t value,
                       int      byteSize);
  int readSoE(uint16_t  slavePosition, /**< Slave position. */
              uint8_t   driveNo, /**< Drive number. */
              uint16_t  idn, /**< SoE IDN (see ecrt_slave_config_idn()). */
              size_t    byteSize, /**< Size of data to read. */
              uint8_t  *value /**< Pointer to data to read. */
              );
  int writeSoE(uint16_t  slavePosition, /**< Slave position. */
               uint8_t   driveNo, /**< Drive number. */
               uint16_t  idn, /**< SoE IDN (see ecrt_slave_config_idn()). */
               size_t    byteSize, /**< Size of data to write. */
               uint8_t  *value /**< Pointer to data to write. */
              );
  int addEntry(
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
              int            useInRealTime);
  int addMemMap(uint16_t       startEntryBusPosition,
                std::string    startEntryIDString,
                int            byteSize,
                int            type,
                ec_direction_t direction,
                ecmcEcDataType dt,
                std::string    memMapIDString);
  ecmcEcMemMap* findMemMap(std::string id);
  ecmcEcMemMap* getMemMap(int index);
  ecmcEcSlave * findSlave(int busPosition);

  int           findSlaveIndex(int  busPosition,
                               int *slaveIndex);
  int           updateTime();
  int           printTimingInformation();
  int           statusOK();
  int           setDomainFailedCyclesLimitInterlock(int cycles);
  void          slowExecute();
  int           reset();
  int           setEcStatusOutputEntry(ecmcEcEntry *entry);
  int           initAsyn(ecmcAsynPortDriver *asynPortDriver);
  int           setAsynPortDriver(ecmcAsynPortDriver *asynPortDriver);
  int           printAllConfig();
  int           printSlaveConfig(int slaveIndex);
  int           validate();
  int           verifySlave(uint16_t alias,  /**< Slave alias. */
                        uint16_t slavePos,   /**< Slave position. */
                        uint32_t vendorId,   /**< Expected vendor ID. */
                        uint32_t productCode  /**< Exp)*/);
  int           checkReadyForRuntime();
  uint64_t      getTimeNs();

private:
  void     initVars();
  int      updateInputProcessImage();
  int      updateOutProcessImage();
  timespec timespecAdd(timespec time1,
                       timespec time2);
  ec_master_t *master_;
  ec_domain_t *domain_;
  ec_domain_state_t domainStateOld_;
  ec_domain_state_t domainState_;
  ec_master_state_t masterStateOld_;
  ec_master_state_t masterState_;
  uint8_t *domainPd_;
  int slaveCounter_;
  int entryCounter_;
  ecmcEcSlave *slaveArray_[EC_MAX_SLAVES];
  ec_pdo_entry_reg_t slaveEntriesReg_[EC_MAX_ENTRIES];
  unsigned int pdoByteOffsetArray_[EC_MAX_ENTRIES];
  unsigned int pdoBitOffsetArray_[EC_MAX_ENTRIES];
  bool initDone_;
  bool diag_;
  ecmcEcSlave *simSlave_;
  int slavesOK_;
  int masterOK_;
  int domainOK_;
  int domainNotOKCounter_;
  int domainNotOKCounterTotal_;
  int domainNotOKCounterMax_;
  int domainNotOKCyclesLimit_;
  bool inStartupPhase_;

  ecmcEcMemMap *ecMemMapArray_[EC_MAX_MEM_MAPS];
  int ecMemMapArrayCounter_;
  size_t domainSize_;
  ecmcEcEntry *statusOutputEntry_;
  int masterIndex_;
  int masterAlStates_;
  int masterLinkUp_;
  uint32_t statusWordMaster_;
  uint32_t statusWordDomain_;

  ecmcAsynPortDriver *asynPortDriver_;
  ecmcAsynDataItem  *ecAsynParams_[ECMC_ASYN_EC_PAR_COUNT];
  timespec timeOffset_;
};
#endif  /* ECMCEC_H_ */
