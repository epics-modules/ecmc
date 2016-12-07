/*
 * ecmcEcSyncManager.cpp
 *
 *  Created on: Dec 11, 2015
 *      Author: anderssandstrom
 */

#include "ecmcEcSyncManager.h"

ecmcEcSyncManager::ecmcEcSyncManager(ec_direction_t direction,uint8_t syncMangerIndex)
{
  initVars();
  syncMangerIndex_=syncMangerIndex;
  direction_=direction;
}

void ecmcEcSyncManager::initVars()
{
  errorReset();
  for(int i=0;i<EC_MAX_PDOS;i++){
    pdoArray_[i]=NULL;
  }
  direction_=EC_DIR_INPUT;
  syncMangerIndex_=0;
  pdoCounter_=0;
}

ecmcEcSyncManager::~ecmcEcSyncManager()
{
  for(int i=0;i<EC_MAX_PDOS;i++){
    if(pdoArray_[i]!=NULL){
      delete pdoArray_[i];
    }
    pdoArray_[i]=NULL;
  }
}

int ecmcEcSyncManager::addPdo(uint16_t pdoIndex)
{
  if(pdoCounter_>=EC_MAX_PDOS-1){
    LOGERR("%s/%s:%d: ERROR: PDO array full (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_SM_PDO_ARRAY_FULL);
    return setErrorID(ERROR_EC_SM_PDO_ARRAY_FULL);
  }
  pdoArray_[pdoCounter_]=new ecmcEcPdo( pdoIndex, direction_);
  pdoCounter_++;
  return 0;
}

ecmcEcPdo *ecmcEcSyncManager::getPdo(int index)
{
  if(index>=EC_MAX_PDOS){
    LOGERR("%s/%s:%d: ERROR: PDO index out of range (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_SM_PDO_INDEX_OUT_OF_RANGE);
    setErrorID(ERROR_EC_SM_PDO_INDEX_OUT_OF_RANGE);
    return NULL;
  }
  return pdoArray_[index];
}

int ecmcEcSyncManager::getPdoCount()
{
  return pdoCounter_;
}


int ecmcEcSyncManager::getInfo(ec_sync_info_t* info)
{
  if(info==NULL){
    LOGERR("%s/%s:%d: ERROR: Output parameter pointer NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_SM_ENTRY_INFO_STRUCT_NULL);
    return setErrorID(ERROR_EC_SM_ENTRY_INFO_STRUCT_NULL);
  }

  info->dir=direction_;
  info->index=syncMangerIndex_;
  info->n_pdos=pdoCounter_;
  info->pdos=NULL;
  info->watchdog_mode=EC_WD_DEFAULT;
  return 0;
}


ec_direction_t ecmcEcSyncManager::getDirection()
{
  return direction_;
}

uint8_t ecmcEcSyncManager::getSyncMangerIndex()
{
  return syncMangerIndex_;
}

int ecmcEcSyncManager::addEntry(
    uint16_t       pdoIndex,
    uint16_t       entryIndex,
    uint8_t        entrySubIndex,
    uint8_t        bits,
    std::string    id
    )
{

ecmcEcPdo *pdo=findPdo(pdoIndex);
 if(pdo==NULL){
   int error=addPdo(pdoIndex);
   if(error){
     return error;
   }
   pdo=pdoArray_[pdoCounter_-1]; //last added sync manager
 }
 pdo->addEntry(entryIndex,entrySubIndex,bits,id);
 return 0;
}

ecmcEcPdo *ecmcEcSyncManager::findPdo(uint16_t pdoIndex)
{
 for(int i=0;i< pdoCounter_;i++){
   if(pdoArray_[i]!=NULL){
     if(pdoArray_[i]->getPdoIndex()==pdoIndex){
       return pdoArray_[i];
     }
   }
 }
 return NULL;
}
