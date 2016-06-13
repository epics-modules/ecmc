/*
 * ecmcEcSyncManager.h
 *
 *  Created on: Dec 11, 2015
 *      Author: anderssandstrom
 */

#ifndef ECMCECSYNCMANAGER_H_
#define ECMCECSYNCMANAGER_H_
#include "ecrt.h"
#include "inttypes.h"
#include <string>

#include "ecmcDefinitions.h"
#include "ecmcEcEntry.h"
#include "ecmcEcPdo.h"
#include "ecmcError.h"

//ECSYNCMANAGER ERRORS
#define ERROR_EC_SM_PDO_ARRAY_FULL 0x25000
#define ERROR_EC_SM_PDO_INDEX_OUT_OF_RANGE 0x25001
#define ERROR_EC_SM_ENTRY_INFO_STRUCT_NULL 0x25002

class ecmcEcSyncManager : public ecmcError
{
public:
  ecmcEcSyncManager(ec_direction_t direction,uint8_t syncMangerIndex);
  ~ecmcEcSyncManager();
  int addPdo(uint16_t pdoIndex);
  ecmcEcPdo *getPdo(int index);
  int getPdoCount();
  int getInfo(ec_sync_info_t* info);
  ec_direction_t getDirection();
  uint8_t getSyncMangerIndex();
  int addEntry(
      uint16_t       pdoIndex,
      uint16_t       entryIndex,
      uint8_t        entrySubIndex,
      uint8_t        bits,
      std::string    id);
private:
  void initVars();
  ecmcEcPdo *findPdo(uint16_t pdoIndex);
  ecmcEcPdo *pdoArray_[EC_MAX_PDOS];
  ec_direction_t direction_;
  uint8_t syncMangerIndex_;
  int pdoCounter_;
};

#endif /* ECMCECSYNCMANAGER_H_ */
