/*
 * ecmcEcPdo.h
 *
 *  Created on: Nov 30, 2015
 *      Author: anderssandstrom
 */

#ifndef ECMCECPDO_H_
#define ECMCECPDO_H_

#include <stdio.h>
#include <string>
#include "ecrt.h"
#include "../main/ecmcDefinitions.h"
#include "../main/ecmcError.h"
#include "../com/ecmcOctetIF.h"  // Logging macros
#include "ecmcEcEntry.h"

// Errors
#define ERROR_EC_PDO_ENTRY_ARRAY_FULL 0x22000
#define ERROR_EC_PDO_ADD_FAIL 0x22001
#define ERROR_EC_PDO_CLEAR_ENTRIES_FAIL 0x22002

class ecmcEcPdo : public ecmcError {
 public:
  ecmcEcPdo(ec_domain_t       *domain,
            ec_slave_config_t *slave,
            uint8_t            syncMangerIndex,
            uint16_t           pdoIndex,
            ec_direction_t     direction);
  ~ecmcEcPdo();
  ecmcEcEntry* addEntry(uint16_t    entryIndex,
                        uint8_t     entrySubIndex,
                        uint8_t     bits,
                        std::string id,
                        int         signedValue,
                        int        *errorCode);
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
  ec_domain_t *domain_;
  ec_slave_config_t *slave_;
};
#endif  // ifndef ECMCECPDO_H_
