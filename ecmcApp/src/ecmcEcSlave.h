/*
 * ecmcEcSlave.h
 *
 *  Created on: Nov 30, 2015
 *      Author: anderssandstrom
 */

#ifndef ECMCECSLAVE_H_
#define ECMCECSLAVE_H_

#include "ecrt.h"
#include "stdio.h"
#include <string>

#include "ecmcDefinitions.h"
#include "ecmcEcEntry.h"
#include "ecmcEcSyncManager.h"
#include "ecmcError.h"
#include "cmd.h" //Logging macros

#define SIMULATION_ENTRIES 2

//ECSLAVE ERRORS
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

typedef struct
{
    uint16_t position; /**< Offset of the slave in the ring. */
    uint32_t vendor_id; /**< Vendor-ID stored on the slave. */
    uint32_t product_code; /**< Product-Code stored on the slave. */
    uint16_t alias; /**< The slaves alias if not equal to 0. */
} mcu_ec_slave_info_light;

class ecmcEcSlave : public ecmcError
{
public:
  ecmcEcSlave(
    ec_master_t *master, /**< EtherCAT master */
    uint16_t alias, /**< Slave alias. */
    uint16_t position, /**< Slave position. */
    uint32_t vendorId, /**< Expected vendor ID. */
    uint32_t productCode /**< Expected product code. */);
  ~ecmcEcSlave();
  int addSyncManager(ec_direction_t direction,uint8_t syncMangerIndex); //Step 3 Iterate if needed
  ecmcEcSyncManager *getSyncManager(int syncManagerIndex);
  int getSlaveInfo(mcu_ec_slave_info_light *info);
  int getEntryCount();
  int getEntryInfo(int entryIndex, ec_pdo_entry_info_t *info);
  ecmcEcEntry *getEntry(int entryIndex);
  int configPdos( ec_domain_t *domain); //Step 4  //For all entries in slave!!
  int checkConfigState(void);
  void setDomainBaseAdr(uint8_t * domainAdr);
  int updateInputProcessImage();
  int updateOutProcessImage();
  int getSlaveBusPosition();
  int addEntry(
      ec_direction_t direction,
      uint8_t        syncMangerIndex,
      uint16_t       pdoIndex,
      uint16_t       entryIndex,
      uint8_t        entrySubIndex,
      uint8_t        bits,
      std::string    id);
  int configDC(
      uint16_t assignActivate, /**< AssignActivate word. */
      uint32_t sync0Cycle, /**< SYNC0 cycle time [ns]. */
      int32_t sync0Shift, /**< SYNC0 shift time [ns]. */
      uint32_t sync1Ccycle, /**< SYNC1 cycle time [ns]. */
      int32_t sync1Shift /**< SYNC1 shift time [ns]. */);
  ecmcEcEntry *findEntry(std::string id);
  int findEntryIndex(std::string id);
  int selectAsReferenceDC();
  int setWatchDogConfig(
      uint16_t watchdogDivider, /**< Number of 40 ns intervals. Used as a
                                       base unit for all slave watchdogs. If set
                                       to zero, the value is not written, so the
                                       default is used. */
      uint16_t watchdogIntervals /**< Number of base intervals for process
                                      data watchdog. If set to zero, the value
                                      is not written, so the default is used.
                                     */
      );

private:
  void initVars();
  ecmcEcSyncManager *findSyncMan(uint8_t syncMangerIndex);
  int configSlave() ;
  ec_master_t *master_; /**< EtherCAT master */
  uint16_t alias_; /**< Slave alias. */
  uint16_t slavePosition_; /**< Slave position. */
  uint32_t vendorId_; /**< Expected vendor ID. */
  uint32_t productCode_; /**< Expected product code. */
  ec_slave_config_t *slaveConfig_;
  ecmcEcSyncManager *syncManagerArray_[EC_MAX_SYNC_MANAGERS];
  ec_pdo_entry_info_t slavePdoEntries_[EC_MAX_ENTRIES];
  ec_pdo_info_t slavePdos_[EC_MAX_PDOS];
  ec_sync_info_t slaveSyncs_[EC_MAX_SYNC_MANAGERS];
  ecmcEcEntry *entryList_[EC_MAX_ENTRIES];
  int entriesArrayIndex_;
  int pdosArrayIndex_;
  int syncManArrayIndex_;
  int syncManCounter_;
  void writePdoStruct();
  void writeSyncsStruct();
  void writeEntriesStruct();
  int pdosInSMCount_;
  ec_slave_config_state_t slaveStateOld_;
  bool simSlave_;  //used to simulate endswitches Consider make derived simulation class insteaed
  uint8_t simBuffer_[8*SIMULATION_ENTRIES]; //used to simulate endswitches
  ecmcEcEntry *simEntries_[SIMULATION_ENTRIES]; //used to simulate endswitches
};
#endif /* ECMCECSLAVE_H_ */
