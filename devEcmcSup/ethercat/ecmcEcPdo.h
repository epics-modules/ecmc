/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcEcPdo.h
*
*  Created on: Nov 30, 2015
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCECPDO_H_
#define ECMCECPDO_H_

#include <stdio.h>
#include <string>
#include "ecrt.h"
#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "ecmcOctetIF.h"  // Logging macros
#include "ecmcEcEntry.h"

// Errors
#define ERROR_EC_PDO_ENTRY_ARRAY_FULL 0x22000
#define ERROR_EC_PDO_ADD_FAIL 0x22001
#define ERROR_EC_PDO_CLEAR_ENTRIES_FAIL 0x22002

class ecmcEcPdo : public ecmcError {
public:
  ecmcEcPdo(ecmcAsynPortDriver *asynPortDriver,
            int                 masterId,
            int                 slaveId,
            ecmcEcDomain       *domain,
            ec_slave_config_t  *slave,
            uint8_t             syncMangerIndex,
            uint16_t            pdoIndex,
            ec_direction_t      direction);
  ~ecmcEcPdo();
  ecmcEcEntry* addEntry(uint16_t       entryIndex,
                        uint8_t        entrySubIndex,
                        ecmcEcDataType dt,
                        std::string    id,
                        int            useInRealTime,
                        int           *errorCode);
  ecmcEcEntry* getEntry(int index);
  ecmcEcEntry* findEntry(std::string id);
  int          getEntryCount();
  uint16_t     getPdoIndex();

private:
  void         initVars();
  uint16_t pdoIndex_;
  ecmcEcEntry *entryArray_[EC_MAX_ENTRIES];
  int entryCounter_;
  ec_direction_t direction_;
  ecmcEcDomain *domain_;
  ec_slave_config_t *slave_;
  int masterId_;
  int slaveId_;
  ecmcAsynPortDriver *asynPortDriver_;
};
#endif  // ifndef ECMCECPDO_H_
