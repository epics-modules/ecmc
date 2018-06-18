/*
 * ecmcEcPdo.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: anderssandstrom
 */

#include "ecmcEcPdo.h"

ecmcEcPdo::ecmcEcPdo(ec_domain_t *domain,ec_slave_config_t *slave,uint8_t syncMangerIndex,uint16_t pdoIndex,ec_direction_t direction)
{
  initVars();
  pdoIndex_=pdoIndex;
  direction_=direction;
  domain_=domain;
  slave_=slave;
  int errorCode=ecrt_slave_config_pdo_assign_add(slave_,syncMangerIndex,pdoIndex_);
  if(errorCode){
    LOGERR("%s/%s:%d: ERROR: ecrt_slave_config_pdo_assign_add() failed with error code %d (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,errorCode,ERROR_EC_PDO_ADD_FAIL);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_PDO_ADD_FAIL);
  }
  ecrt_slave_config_pdo_mapping_clear(slave_,pdoIndex_);
  LOGINFO5("%s/%s:%d: INFO: Pdo 0x%x created: syncManger %d, direction %d.\n",__FILE__, __FUNCTION__, __LINE__,pdoIndex_,syncMangerIndex,direction_);
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
  domain_=NULL;
  slave_=NULL;
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

ecmcEcEntry *ecmcEcPdo::addEntry(uint16_t entryIndex,uint8_t  entrySubIndex, uint8_t bits, std::string id,int signedValue,int *errorCode)
{
  if(entryCounter_>=(EC_MAX_ENTRIES)){
    LOGERR("%s/%s:%d: ERROR: Entries array full (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_PDO_ENTRY_ARRAY_FULL);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_PDO_ENTRY_ARRAY_FULL);
    *errorCode=ERROR_EC_PDO_ENTRY_ARRAY_FULL;
    return NULL;
  }
  ecmcEcEntry * entry=new ecmcEcEntry(domain_,slave_,pdoIndex_,entryIndex,entrySubIndex,bits, direction_,id,signedValue);
  entryArray_[entryCounter_]=entry;
  entryCounter_++;
  *errorCode=0;
  return entry;
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

ecmcEcEntry *ecmcEcPdo::findEntry(std::string id)
{
  ecmcEcEntry *temp=NULL;
  for(int i=0;i<entryCounter_;i++){
    if(entryArray_[i]){
      if(entryArray_[i]->getIdentificationName().compare(id)==0){
	  temp=entryArray_[i];
      }
    }
  }
  return temp;
}
