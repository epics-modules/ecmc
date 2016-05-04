/*
 * ecmcEcSlave.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: anderssandstrom
 */

#include "ecmcEcSlave.h"

ecmcEcSlave::ecmcEcSlave(
  ec_master_t *master, /**< EtherCAT master */
  uint16_t alias, /**< Slave alias. */
  uint16_t position, /**< Slave position. */
  uint32_t vendorId, /**< Expected vendor ID. */
  uint32_t productCode /**< Expected product code. */)
{
  initVars();
  master_=master;
  alias_=alias; /**< Slave alias. */
  slavePosition_=position; /**< Slave position. */
  vendorId_=vendorId; /**< Expected vendor ID. */
  productCode_=productCode; /**< Expected product code. */

  //Simulation entries
  simEntries_[0]=new ecmcEcEntry((uint8_t)1,&simBuffer_[0],"ZERO");
  simEntries_[1]=new ecmcEcEntry((uint8_t)1,&simBuffer_[8],"ONE");
  simEntries_[0]->writeValue(0); //ALways 0
  simEntries_[1]->writeValue(1); //Always 1

  if(alias==0 && position==0 && vendorId==0 && productCode==0){
    simSlave_=true;
    return;
  }
  configSlave();
}

void ecmcEcSlave::initVars()
{
  errorReset();
  simSlave_=false;
  master_=0;
  alias_=0; /**< Slave alias. */
  slavePosition_=0; /**< Slave position. */
  vendorId_=0; /**< Expected vendor ID. */
  productCode_=0; /**< Expected product code. */
  slaveConfig_=0;
  syncManCounter_=0;
  entriesArrayIndex_=0;
  pdosArrayIndex_=0;
  syncManArrayIndex_=0;
  for(int i=0;i<EC_MAX_SYNC_MANAGERS;i++){
    syncManagerArray_[i]=NULL;
  }
  for(int i=0; i < SIMULATION_ENTRIES; i++){
    simEntries_[i]=NULL;
  }

  for(int i=0; i < SIMULATION_ENTRIES*8; i++){
    simBuffer_[i]=0;
  }

  for(int i=0;i<EC_MAX_ENTRIES;i++){
    entryList_[i]=NULL;
  }
}

ecmcEcSlave::~ecmcEcSlave()
{
  for(int i=0;i<EC_MAX_SYNC_MANAGERS;i++){
    if(syncManagerArray_[i]!=NULL){
      delete syncManagerArray_[i];
    }
    syncManagerArray_[i]=NULL;
  }

  for(int i=0;i<SIMULATION_ENTRIES;i++){
    if(simEntries_[i]!=NULL){
      delete simEntries_[i];
    }
    simEntries_[i]=NULL;
  }
}

int ecmcEcSlave::configSlave()
{
  if (!(slaveConfig_ = ecrt_master_slave_config(master_, alias_,slavePosition_, vendorId_,productCode_))) {
    printf("ERROR:\tFailed to get slave configuration.\n");
    return setErrorID(ERROR_EC_SLAVE_CONFIG_FAILED);
  }
  else
    return 0;
}

int ecmcEcSlave::getEntryCount()
{
  return entriesArrayIndex_;
}

int ecmcEcSlave::addSyncManager(ec_direction_t direction,uint8_t syncMangerIndex)
{
  if(simSlave_){
    return setErrorID(ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
  }

  if(syncManCounter_>=EC_MAX_SYNC_MANAGERS){
    return setErrorID(ERROR_EC_SLAVE_SM_ARRAY_FULL);
  }

  syncManagerArray_[syncManCounter_]=new ecmcEcSyncManager(direction, syncMangerIndex);
  syncManCounter_++;
  return 0;
}

ecmcEcSyncManager *ecmcEcSlave::getSyncManager(int syncManagerIndex)
{
  if(simSlave_){
    setErrorID(ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return NULL;
  }

  if(syncManagerIndex>=EC_MAX_SYNC_MANAGERS){
    setErrorID(ERROR_EC_SLAVE_SM_INDEX_OUT_OF_RANGE);
    return NULL;
  }
  return syncManagerArray_[syncManagerIndex];
}

int ecmcEcSlave::getEntryInfo(int entryIndex, ec_pdo_entry_info_t *info)
{
  if(simSlave_){
    return setErrorID(ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
  }

  if(info==NULL){
    return setErrorID(ERROR_EC_SLAVE_ENTRY_INFO_STRUCT_NULL);
  }

  if(entryIndex>=EC_MAX_ENTRIES || entryIndex>=entriesArrayIndex_){
    return setErrorID(ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE);
  }

  info->bit_length=slavePdoEntries_[entryIndex].bit_length;
  info->index=slavePdoEntries_[entryIndex].index;
  info->subindex=slavePdoEntries_[entryIndex].subindex;

  return 0;
}

int ecmcEcSlave::getSlaveInfo( mcu_ec_slave_info_light *info)
{
  if(info==NULL){
    return setErrorID(ERROR_EC_SLAVE_SLAVE_INFO_STRUCT_NULL);
  }
  info->alias=alias_;
  info->position=slavePosition_;
  info->product_code=productCode_;
  info->vendor_id=vendorId_;
  return 0;
}


int ecmcEcSlave::configPdos( ec_domain_t *domain)
{
  if(simSlave_){
    return setErrorID(ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
  }

  printf("INFO:\t\tConfiguring slave %d.\n", slavePosition_);
  ecmcEcSyncManager *sm=NULL;
  ecmcEcPdo *pdo=NULL;
  ecmcEcEntry *entry=NULL;
  entriesArrayIndex_=0;
  pdosArrayIndex_=0;
  syncManArrayIndex_=0;
  for(int smIndex=0;smIndex<syncManCounter_;smIndex++){
    sm=syncManagerArray_[smIndex];
    int pdoCount=sm->getPdoCount();
    sm->getInfo(&slaveSyncs_[syncManArrayIndex_]);
    slaveSyncs_[syncManArrayIndex_].pdos=&slavePdos_[pdosArrayIndex_];
    slaveSyncs_[syncManArrayIndex_].n_pdos=pdoCount;
    syncManArrayIndex_++;
    for(int pdoIndex=0;pdoIndex<pdoCount;pdoIndex++){
      pdo=sm->getPdo(pdoIndex);
      int entryCount=pdo->getEntryCount();
      slavePdos_[pdosArrayIndex_].index=pdo->getPdoIndex(); //nPdoInde 0x1600;
      slavePdos_[pdosArrayIndex_].n_entries=entryCount;
      slavePdos_[pdosArrayIndex_].entries=&slavePdoEntries_[entriesArrayIndex_];
      pdosArrayIndex_++;
      for(int entryIndex=0;entryIndex<entryCount;entryIndex++){
        entry=pdo->getEntry(entryIndex);
        entryList_[entriesArrayIndex_]=entry;
        entry->getEntryInfo(&slavePdoEntries_[entriesArrayIndex_]);
        entriesArrayIndex_++;
      }
    }
  }
  slaveSyncs_[syncManArrayIndex_].index=0xff; //Terminate structure for ecrt_slave_config_pdos()

  if(entriesArrayIndex_==0){
    printf("WARNING:\tNo Pdo:s to configure for this slave..\n");
    return 0;
  }

  writeEntriesStruct();
  writePdoStruct();
  writeSyncsStruct();

  if (ecrt_slave_config_pdos(slaveConfig_, EC_END, slaveSyncs_)){
    printf("Error:\tPDO configuration failed!\n");
    return setErrorID(ERROR_EC_SLAVE_CONFIG_PDOS_FAILED);
  }

  printf("INFO:\t\tConfiguration done successfully for slave %d.\n", slavePosition_);

  return 0;
}

void ecmcEcSlave::writeEntriesStruct()
{
  if(simSlave_){
    setErrorID(ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return;
  }
  printf("INFO:\t\tWriting: slave_pdo_entries: \n");
  printf("\t\tIndex\tSubIndex\tBitLength \n");
  for(int i=0;i<entriesArrayIndex_;i++){
    printf("\t\t{%x\t%x\t%x}\n",slavePdoEntries_[i].index,slavePdoEntries_[i].subindex,slavePdoEntries_[i].bit_length);
  }
}
void ecmcEcSlave::writePdoStruct()
{
  if(simSlave_){
    setErrorID(ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return;
  }
  printf("INFO:\t\tWriting: slave_pdos: \n");
  printf("\t\tIndex\tEntryCount\n");
  for(int i=0;i<pdosArrayIndex_;i++){
    printf("\t\t{%x\t%x}\n",slavePdos_[i].index,slavePdos_[i].n_entries/*,slave_pdos[i].entries*/);
  }
}

void ecmcEcSlave::writeSyncsStruct()
{
  if(simSlave_){
    setErrorID(ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return;
  }
  printf("INFO:\t\tWriting: slave_syncs: \n");
  printf("\t\tIndex\tDirection\tPdoCount\tWatchDog\n");
  for(int i=0;i<syncManArrayIndex_;i++){
    printf("\t\t{%x\t%x\t%x\t%x}\n",slaveSyncs_[i].index,slaveSyncs_[i].dir,slaveSyncs_[i].n_pdos/*,slave_syncs[i].pdos*/,slaveSyncs_[i].watchdog_mode);
  }
}

int ecmcEcSlave::checkConfigState(void)
{
  if(simSlave_){
    return setErrorID(ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
  }
  ec_slave_config_state_t s;
  ecrt_slave_config_state(slaveConfig_, &s);

  if (s.al_state != sOld_.al_state){
    printf("INFO:\t\tSlave position: %d. State 0x%02X.\n",slavePosition_ ,s.al_state);
    setErrorID(ERROR_EC_SLAVE_STATE_CHANGED);
  }
  if (s.online != sOld_.online){
    printf("INFO:\t\tSlave position: %d. %s.\n",slavePosition_ ,s.online ? "online" : "offline");
    setErrorID(ERROR_EC_SLAVE_ONLINE_OFFLINE_CHANGED);
  }
  if (s.operational != sOld_.operational){
    printf("INFO:\t\t\tSlave position: %d. AnaIn: %soperational.\n",slavePosition_,s.operational ? "" : "Not ");
    setErrorID(ERROR_EC_SLAVE_OPERATIONAL_CHANGED);
  }

  sOld_ = s;
  return getErrorID();
}

ecmcEcEntry *ecmcEcSlave::getEntry(int entryIndex)
{
  if(!simSlave_){
    if(entryIndex>=EC_MAX_ENTRIES){
      printf("Error:\tEntry index out of range\n");
      setErrorID(ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE);
      return NULL;
    }

    if(entryList_[entryIndex]==NULL ){
      printf("Error:\tEntry ==NULL\n");
      setErrorID(ERROR_EC_SLAVE_ENTRY_NULL);
      return NULL;
    }
    return entryList_[entryIndex];
  }
  else{
    if(entryIndex>=SIMULATION_ENTRIES){
      printf("Error:\tEntry index out of range\n");
      setErrorID(ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE);
      return NULL;
    }

    if(simEntries_[entryIndex]==NULL){
      printf("Error:\tEntry ==NULL\n");
      setErrorID(ERROR_EC_SLAVE_ENTRY_NULL);
      return NULL;
    }
    return simEntries_[entryIndex];
  }
}

int ecmcEcSlave::updateInputProcessImage()
{
  for(int i=0;i<entriesArrayIndex_;i++){
    if(entryList_[i]!=NULL){
      entryList_[i]->updateInputProcessImage();
    }
  }
  return 0;
}

int ecmcEcSlave::updateOutProcessImage()
{
  for(int i=0;i<entriesArrayIndex_;i++){
    if(entryList_[i]!=NULL){
      entryList_[i]->updateOutProcessImage();
    }
  }
  return 0;
}

int ecmcEcSlave::getSlaveBusPosition()
{
  return slavePosition_;
}


int ecmcEcSlave::addEntry(
    ec_direction_t direction,
    uint8_t        syncMangerIndex,
    uint16_t       pdoIndex,
    uint16_t       entryIndex,
    uint8_t        entrySubIndex,
    uint8_t        bits,
    std::string    id
    )
{

ecmcEcSyncManager *sync_manager=findSyncMan(syncMangerIndex);
 if(sync_manager==NULL){
   int error=addSyncManager(direction,syncMangerIndex);
   if(error){
     printf("Error addSync..");
     return error;
   }
   sync_manager=syncManagerArray_[syncManCounter_-1]; //last added sync manager
 }
 sync_manager->addEntry(pdoIndex,entryIndex,entrySubIndex,bits,id);

 return 0;
}

ecmcEcSyncManager *ecmcEcSlave::findSyncMan(uint8_t syncMangerIndex)
{
 for(int i=0;i< syncManCounter_;i++){
   if(syncManagerArray_[i]!=NULL){
     if(syncManagerArray_[i]->getSyncMangerIndex()==syncMangerIndex){
       return syncManagerArray_[i];
     }
   }
 }
 return NULL;
}

int ecmcEcSlave::configDC(
      uint16_t assignActivate, /**< AssignActivate word. */
      uint32_t sync0Cycle, /**< SYNC0 cycle time [ns]. */
      int32_t sync0Shift, /**< SYNC0 shift time [ns]. */
      uint32_t sync1Cycle, /**< SYNC1 cycle time [ns]. */
      int32_t sync1Shift /**< SYNC1 shift time [ns]. */)
{
  if(slaveConfig_==0){
    return setErrorID(ERROR_EC_SLAVE_CONFIG_NULL);
  }

  ecrt_slave_config_dc(slaveConfig_,assignActivate,sync0Cycle,sync0Shift,sync1Cycle,sync1Shift);
  return 0;
}

ecmcEcEntry *ecmcEcSlave::findEntry(std::string id)
{
  //Real entries
  for(int i=0; i<EC_MAX_ENTRIES;i++){
    if(entryList_[i]!=NULL){
      if(entryList_[i]->getIdentificationName().compare(id)==0){
        return entryList_[i];
      }
    }
  }

  //Simulation entries
  for(int i=0; i<SIMULATION_ENTRIES;i++){
    if(simEntries_[i]!=NULL){
      if(simEntries_[i]->getIdentificationName().compare(id)==0){
        return simEntries_[i];
      }
    }
  }
  return NULL;
}

int ecmcEcSlave::findEntryIndex(std::string id)
{
  //Real entries
  for(int i=0; i<EC_MAX_ENTRIES;i++){
    if(entryList_[i]!=NULL){
      if(entryList_[i]->getIdentificationName().compare(id)==0){
        return i;
      }
    }
  }

  //Simulation entries
  for(int i=0; i<SIMULATION_ENTRIES;i++){
    if(simEntries_[i]!=NULL){
      if(simEntries_[i]->getIdentificationName().compare(id)==0){
        return i;
      }
    }
  }
  return -ERROR_EC_SLAVE_ENTRY_NULL;
}

