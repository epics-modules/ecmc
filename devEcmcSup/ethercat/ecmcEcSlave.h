/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcEcSlave.h
*
*  Created on: Nov 30, 2015
*      Author: anderssandstrom
*
\*************************************************************************/


#ifndef ECMCECSLAVE_H_
#define ECMCECSLAVE_H_

#include <string>
#include <atomic>
#include <vector>
#include "stdio.h"
#include "ecrt.h"
#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "ecmcOctetIF.h"  // Logging macros
#include "ecmcAsynPortDriver.h"
#include "ecmcAsynDataItem.h"
#include "ecmcEcEntry.h"
#include "ecmcEcMemMap.h"
#include "ecmcEcSyncManager.h"
#include "ecmcEcSDO.h"
#include "ecmcEcAsyncSDO.h"
#include "ecmcEcDomain.h"
#include "ecmcEcData.h"
#include "ecmcEcTiming.h"
#include "ecmcRtLoggerPortDriver.h"

// ECSLAVE ERRORS
#define ERROR_EC_SLAVE_CONFIG_FAILED 0x24000
#define ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE 0x24001
#define ERROR_EC_SLAVE_SM_ARRAY_FULL 0x24002
#define ERROR_EC_SLAVE_SM_INDEX_OUT_OF_RANGE 0x24003
#define ERROR_EC_SLAVE_ENTRY_INFO_STRUCT_NULL 0x24004
#define ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE 0x24005
#define ERROR_EC_SLAVE_SLAVE_INFO_STRUCT_NULL 0x24006
#define ERROR_EC_SLAVE_CONFIG_PDOS_FAILED 0x24007
#define ERROR_EC_SLAVE_ENTRY_NULL 0x24008
#define ERROR_EC_SLAVE_STATE_CHANGED 0x24009
#define ERROR_EC_SLAVE_ONLINE_OFFLINE_CHANGED 0x2400A
#define ERROR_EC_SLAVE_OPERATIONAL_CHANGED 0x2400B
#define ERROR_EC_SLAVE_CONFIG_NULL 0x2400C
#define ERROR_EC_SLAVE_STATE_INIT 0x2400D
#define ERROR_EC_SLAVE_STATE_PREOP 0x2400E
#define ERROR_EC_SLAVE_STATE_SAFEOP 0x2400F
#define ERROR_EC_SLAVE_STATE_UNDEFINED 0x24010
#define ERROR_EC_SLAVE_NOT_OPERATIONAL 0x24011
#define ERROR_EC_SLAVE_NOT_ONLINE 0x24012
#define ERROR_EC_SLAVE_REG_ASYN_PAR_BUFFER_OVERFLOW 0x24013
#define ERROR_EC_SLAVE_SDO_ASYNC_CREATE_FAIL 0x24014
#define ERROR_EC_SLAVE_ADD_DATA_ITEM_FAIL 0x24015
#define ERROR_EC_SLAVE_SDO_SETTINGS_MISSING 0x24016
#define ERROR_EC_SLAVE_SDO_CH_ID_OUT_OF_RANGE 0x24017
#define ERROR_EC_SLAVE_ENTRY_ALIAS_EXISTS 0x24018

typedef struct {
  int needSdo; 
  int sdosDone;  
} sdoVerifyChX;

#define SDO_SETTINGS_MAX_CH 256

typedef struct {
  uint16_t position;   /**< Offset of the slave in the ring. */
  uint32_t vendor_id;   /**< Vendor-ID stored on the slave. */
  uint32_t product_code;   /**< Product-Code stored on the slave. */
  uint16_t alias;   /**< The slaves alias if not equal to 0. */
} mcu_ec_slave_info_light;

class ecmcEcSlave : public ecmcError {
public:
  ecmcEcSlave(
    ecmcAsynPortDriver *asynPortDriver,  /** Asyn port driver*/
    int                 masterId,
    ec_master_t        *master, /**< EtherCAT master */
    ecmcEcDomain       *domain,
    uint16_t            alias, /**< Slave alias. */
    int32_t             position, /**< Slave position. */
    uint32_t            vendorId, /**< Expected vendor ID. */
    uint32_t            productCode /**< Expected product code. */);
  ~ecmcEcSlave();
  int                addSyncManager(ec_direction_t direction,
                                    uint8_t        syncMangerIndex);
  ecmcEcSyncManager* getSyncManager(int syncManagerIndex);
  int                getSlaveInfo(mcu_ec_slave_info_light *info);
  int                getEntryCount();
  ecmcEcEntry*       getEntry(int entryIndex);
  int                checkConfigState(void);
  void               setDomainBaseAdr(uint8_t *domainAdr);
  int                updateInputProcessImage();
  int                updateOutProcessImage();
  int                getSlaveBusPosition();
  int                addEntry(
    ec_direction_t direction,
    uint8_t        syncMangerIndex,
    uint16_t       pdoIndex,
    uint16_t       entryIndex,
    uint8_t        entrySubIndex,
    ecmcEcDataType dt,
    std::string    id,
    int            useInRealTime,
    bool           useExistingMapping = false,
    bool           registerByPosition = false,
    unsigned int   entryPosition = 0);
  int addSimEntry(std::string    id,
                  ecmcEcDataType dt,
                  uint64_t       value);
  int addDataItem(
    ecmcEcEntry   *startEntry,
    size_t         entryByteOffset,
    size_t         entryBitOffset,
    ec_direction_t direction,
    ecmcEcDataType dt,
    std::string    id);
  int configDC(

    // AssignActivate word.
    uint16_t assignActivate,

    // SYNC0 cycle time [ns].
    uint32_t sync0Cycle,

    // SYNC0 shift time [ns].
    int32_t sync0Shift,

    // SYNC1 cycle time [ns].
    uint32_t sync1Ccycle,

    // SYNC1 shift time [ns].
    int32_t sync1Shift);
  const ecmcEcDcConfig& getDcConfig() const;
  const ecmcEcDcSchedule& getDcSchedule() const;
  void setNominalTimingCycleNs(uint32_t cycleTimeNs);
  void discoverSmTiming();
  const ecmcEcSmTiming& getInputSmTiming() const;
  const ecmcEcSmTiming& getOutputSmTiming() const;
  int setTimingOverride(ec_direction_t direction,
                        int32_t cycleOffset,
                        int32_t eventOffsetNs,
                        uint32_t uncertaintyNs);
  int setTimingSource(ec_direction_t direction,
                      ecmcEcTimingSource source);
  int setTimingUpdateDivisor(ec_direction_t direction,
                             uint32_t updateDivisor);
  int linkTimingTimestamp(ec_direction_t direction,
                          const std::string& entryId,
                          uint8_t bits,
                          int32_t correctionNs);
  bool readTimingTimestamp(ec_direction_t direction,
                           uint64_t nearbyDcTimeNs,
                           uint64_t *eventTimeNs) const;
  const ecmcEcEndpointTiming& getInputTiming() const;
  const ecmcEcEndpointTiming& getOutputTiming() const;
  void refreshTimingAsyn();
  bool timingStartupReady() const;
  void printDcTiming() const;
  ecmcEcEntry* findEntry(std::string id);
  int          findEntryIndex(std::string id);
  int          addEntryAlias(std::string entryId,
                             std::string alias);
  int          selectAsReferenceDC();
  int          setWatchDogConfig(

    // Number of 40 ns intervals. Used as a base unit for all slave watchdogs.
    // If set to zero, the value is not written, so the default is used.
    uint16_t watchdogDivider,

    // Number of base intervals for process  data watchdog.
    // If set to zero, the value is not written, so the default is used.
    uint16_t watchdogIntervals);

  // accepts up to 4 bytes ints
  int addSDOWrite(uint16_t sdoIndex,
                  uint8_t  sdoSubIndex,
                  uint32_t writeValue,
                  int      byteSize);

  // accepts up to 8 bytes
  int addSDOWriteDT(uint16_t       sdoIndex,
                    uint8_t        sdoSubIndex,
                    const char    *value,
                    ecmcEcDataType dt);
  int addSDOWriteComplete(uint16_t    sdoIndex,
                          const char *dataBuffer,
                          int         byteSize);

  int addSDOWriteBuffer(uint16_t    sdoIndex,
                        uint8_t     sdoSubIndex,
                        const char *dataBuffer,
                        int         byteSize);
  int getSlaveState(ec_slave_config_state_t *state);
  int activate();
  int compileRegInfo();
  int validate();
  int addSDOAsync(uint16_t       sdoIndex, /**< SDO index. */
                  uint8_t        sdoSubIndex, /**< SDO subindex. */
                  ecmcEcDataType dt,
                  std::string    alias);
  int getAllowOffline();

/* 
 *  The below functions are used for preventing accidental usage of a drive slaves
 *  without setting important SDO parameters like Max current. The hardware snippet
 *  for drive terminals called by ecmccfg.addSlave.cmd calls "setNeedSDOSettings()"
 *  to tell ecmc that SDO settings are needed for this slave. The ecmccfg functions
 *  configureSlave.cmd, applySlaveConfig.cmd and ecmccomp.applyComponent.cmd then calls
 *  "setSDOSettingsDone()" after successful sdo configuration to tell ecmc that
 *  configuration for a certain channel has been performed. 
 *  At validation, before going to runtime, ecmc checks that the slave SDO settings 
 *  have been performed, if not, ecmc exits. IMPORTANT: the check is only applied if 
 *  the drive terminal is used in an ecmc motion axis ().
 */
  int setNeedSDOSettings(int chid, int need);
  int setSDOSettingsDone(int chid, int done);
  int setEnableSDOCheck(int enable);
  int setPortWiringCheckEnable(int enable);
  bool getPortWiringCheckEnable() const;

private:
  void               initVars();
  int                initAsyn();
  int                appendEntryToList(ecmcEcEntry *entry,
                                       bool         useInRealTime);
  ecmcEcSyncManager* findSyncMan(uint8_t syncMangerIndex);
  struct smTimingRequest {
    ec_sdo_request_t *request;
    ecmcEcSmTimingValue *value;
    ecmcEcSmTiming *timing;
    uint8_t byteSize;
    bool started;
    bool finished;
  };
  void prepareSmTiming(uint16_t objectIndex, ecmcEcSmTiming *timing);
  void prepareSmTimingValue(uint16_t objectIndex,
                            uint8_t subIndex,
                            uint8_t byteSize,
                            ecmcEcSmTiming *timing,
                            ecmcEcSmTimingValue *value);
  void executeSmTimingDiscovery();
  void prepareDcScheduleDiscovery();
  void executeDcScheduleDiscovery();
  void resolveEndpointTiming(const ecmcEcSmTiming& smTiming,
                             const ecmcEcTimingOverride& timingOverride,
                             const ecmcEcTimestampConfig& timestampConfig,
                             ec_direction_t direction,
                             ecmcEcEndpointTiming *endpoint);
  void printSmTiming(const char *name, const ecmcEcSmTiming& timing) const;
  void printEndpointTiming(const char *name,
                           const ecmcEcEndpointTiming& timing) const;
  void updateTimingAsynData();
  ec_master_t *master_;     // EtherCAT master
  uint16_t alias_;          // Slave alias.
  int32_t slavePosition_;   // Slave position.
  uint32_t vendorId_;       // Expected vendor ID.
  uint32_t productCode_;    // Expected product code.
  bool portWiringCheckEnabled_;
  ec_slave_config_t *slaveConfig_;
  ecmcEcSyncManager *syncManagerArray_[EC_MAX_SYNC_MANAGERS];
  ecmcEcEntry *entryList_[EC_MAX_ENTRIES];
  uint32_t entryCounter_;
  ecmcEcEntry *entryListRtInput_[EC_MAX_ENTRIES];
  ecmcEcEntry *entryListRtOutput_[EC_MAX_ENTRIES];
  uint32_t entryCounterRtInput_;
  uint32_t entryCounterRtOutput_;
  int pdosArrayIndex_;
  int syncManArrayIndex_;
  int syncManCounter_;
  int pdosInSMCount_;
  ec_slave_config_state_t slaveState_;
  ec_slave_config_state_t slaveStateOld_;

  // used to simulate endswitches
  bool simSlave_;
  std::vector<uint64_t *>simBuffer_;
  std::vector<ecmcEcEntry *>simEntries_;
  ecmcEcDomain *domain_;
  ecmcAsynPortDriver *asynPortDriver_;
  ecmcAsynDataItem *slaveAsynParams_[ECMC_ASYN_EC_SLAVE_PAR_COUNT];
  int masterId_;

  // bit 0 online          : The slave is online.
  // bit 1 int operational : The slave was brought into  OP state
  // bit 2..5  al_state    :  The application-layer state of the slave.
  //                             - 1: \a INIT
  //                             - 2: \a PREOP
  //                             - 4: \a SAFEOP
  //                             - 8: \a OP
  // bit 16..31            : entry counter
  uint32_t statusWord_;
  uint32_t statusWordOld_;

  std::vector<ecmcEcAsyncSDO *>asyncSDOvector_;
  int asyncSDOCounter_;
  
  // Ensure important SDO settings like max current are set.
  std::vector<sdoVerifyChX> sdoChVerify_;
  bool enableSDOCheck_;
  ecmcEcDcConfig dcConfig_;
  ecmcEcSmTiming inputSmTiming_;
  ecmcEcSmTiming outputSmTiming_;
  ecmcEcTimingOverride inputTimingOverride_;
  ecmcEcTimingOverride outputTimingOverride_;
  ecmcEcTimestampConfig inputTimestampConfig_;
  ecmcEcTimestampConfig outputTimestampConfig_;
  ecmcEcEntry *inputTimestampEntry_;
  ecmcEcEntry *outputTimestampEntry_;
  ecmcEcEndpointTiming inputTiming_;
  ecmcEcEndpointTiming outputTiming_;
  smTimingRequest smTimingRequests_[20];
  uint8_t smTimingRequestCount_;
  bool smTimingDiscoveryComplete_;
  bool smTimingReadLogged_ = false;
  ec_reg_request_t *dcScheduleRequest_;
  ecmcEcDcSchedule dcSchedule_;
  uint8_t dcScheduleStage_;
  bool dcScheduleRequestStarted_;
  uint32_t nominalTimingCycleNs_;
  bool hasProcessDataInput_;
  bool hasProcessDataOutput_;
  ecmcEcTimingDiag inputTimingAsynData_;
  ecmcEcTimingDiag outputTimingAsynData_;
  bool timingAsynDirty_;
  std::atomic<bool> timingStartupReady_{false};
};
#endif  /* ECMCECSLAVE_H_ */
