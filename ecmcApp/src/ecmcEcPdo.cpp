/*
 * ecmcEcPdo.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: anderssandstrom
 */

#include "ecmcEcPdo.h"

ecmcEcPdo::ecmcEcPdo(uint16_t pdoIndex,ec_direction_t direction)
{
  initVars();
  pdoIndex_=pdoIndex;
  direction_=direction;
}
void ecmcEcPdo::initVars()
{
  for(int i=0;i<EC_MAX_ENTRIES;i++){
    entryArray_[i]=NULL;
  }
  errorReset();
  entryCounter_=0;
  pdoIndex_=0;
  direction_=EC_DIR_INVALID;
}

ecmcEcPdo::~ecmcEcPdo()
{
  for(int i=0;i<EC_MAX_ENTRIES;i++){
    if(entryArray_[i]!=NULL){
      delete entryArray_[i];
    }
    entryArray_[i]=NULL;
  }
}

int ecmcEcPdo::addEntry(uint16_t entryIndex,uint8_t  entrySubIndex, uint8_t bits, std::string id)
{
  if(entryCounter_>=(EC_MAX_ENTRIES)){
    return setErrorID(ERROR_EC_PDO_ENTRY_ARRAY_FULL);
  }

  entryArray_[entryCounter_]=new ecmcEcEntry(entryIndex,entrySubIndex,bits, direction_,id);
  entryCounter_++;
  return 0;
}

ecmcEcEntry *ecmcEcPdo::getEntry(int index)
{
  if(index>=EC_MAX_ENTRIES-1){
    return NULL;
  }
  return entryArray_[index];
}

int ecmcEcPdo::getEntryCount()
{
  return entryCounter_;
}

uint16_t  ecmcEcPdo::getPdoIndex()
{
  return pdoIndex_;
}
