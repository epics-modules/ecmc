/*
 * ecmcEcPdo.h
 *
 *  Created on: Nov 30, 2015
 *      Author: anderssandstrom
 */

#ifndef ECMCECPDO_H_
#define ECMCECPDO_H_
#include "ecrt.h"
#include <stdio.h>
#include <string>

#include "ecmcDefinitions.h"
#include "ecmcEcEntry.h"
#include "ecmcError.h"
#include "cmd.h" //Logging macros

//ECPDO
#define ERROR_EC_PDO_ENTRY_ARRAY_FULL 0x22000

class ecmcEcPdo : public ecmcError
{
public:
  ecmcEcPdo(uint16_t pdoIndex,ec_direction_t direction);
  ~ecmcEcPdo();
  int addEntry( uint16_t entryIndex,uint8_t  entrySubIndex, uint8_t bits,std::string id);
  ecmcEcEntry *getEntry(int index);
  int getEntryCount();
  uint16_t  getPdoIndex();
private:
  void initVars();
  uint16_t pdoIndex_;
  ecmcEcEntry *entryArray_[EC_MAX_ENTRIES];
  int entryCounter_;
  ec_direction_t direction_;
  };
#endif
