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
  master_=NULL;
  alias_=0; /**< Slave alias. */
  slavePosition_=0; /**< Slave position. */
  vendorId_=0; /**< Expected vendor ID. */
  productCode_=0; /**< Expected product code. */
  slaveConfig_=NULL;
  syncManCounter_=0;
  entryCounter_=0;
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

  memset(&slaveSyncs_,0,sizeof(slaveSyncs_));
  memset(&slavePdoEntries_,0,sizeof(slavePdoEntries_));
  memset(&slavePdos_,0,sizeof(slavePdos_));
  memset(&slaveStateOld_,0,sizeof(slaveStateOld_));
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
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Failed to get slave configuration (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_CONFIG_FAILED);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_CONFIG_FAILED);
  }
  else
    return 0;
}

int ecmcEcSlave::getEntryCount()
{
  return entryCounter_;
}

int ecmcEcSlave::addSyncManager(ec_direction_t direction,uint8_t syncMangerIndex)
{
  if(simSlave_){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Simulation slave: Functionality not supported (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
  }

  if(syncManCounter_>=EC_MAX_SYNC_MANAGERS){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Sync manager array full (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_SM_ARRAY_FULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_SM_ARRAY_FULL);
  }

  syncManagerArray_[syncManCounter_]=new ecmcEcSyncManager(direction, syncMangerIndex);
  syncManCounter_++;
  return 0;
}

ecmcEcSyncManager *ecmcEcSlave::getSyncManager(int syncManagerIndex)
{
  if(simSlave_){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Simulation slave: Functionality not supported (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return NULL;
  }

  if(syncManagerIndex>=EC_MAX_SYNC_MANAGERS){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Sync manager array index out of range (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_SM_INDEX_OUT_OF_RANGE);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_SM_INDEX_OUT_OF_RANGE);
    return NULL;
  }
  return syncManagerArray_[syncManagerIndex];
}

int ecmcEcSlave::getEntryInfo(int entryIndex, ec_pdo_entry_info_t *info)
{
  if(simSlave_){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Simulation slave: Functionality not supported (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
  }

  if(info==NULL){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry Info structure NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_ENTRY_INFO_STRUCT_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_ENTRY_INFO_STRUCT_NULL);
  }

  if(entryIndex>=EC_MAX_ENTRIES || entryIndex>=entryCounter_){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry index out of range (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE);
  }

  info->bit_length=slavePdoEntries_[entryIndex].bit_length;
  info->index=slavePdoEntries_[entryIndex].index;
  info->subindex=slavePdoEntries_[entryIndex].subindex;

  return 0;
}

int ecmcEcSlave::getSlaveInfo( mcu_ec_slave_info_light *info)
{
  if(info==NULL){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Slave Info structure NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_SLAVE_INFO_STRUCT_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_SLAVE_INFO_STRUCT_NULL);
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
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Simulation slave: Functionality not supported (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
  }
  LOGINFO5("%s/%s:%d: INFO: Configuring slave %d.\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_);
  ecmcEcSyncManager *sm=NULL;
  ecmcEcPdo *pdo=NULL;
  ecmcEcEntry *entry=NULL;
  entryCounter_=0;
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
      slavePdos_[pdosArrayIndex_].entries=&slavePdoEntries_[entryCounter_];
      pdosArrayIndex_++;
      for(int entryIndex=0;entryIndex<entryCount;entryIndex++){
        entry=pdo->getEntry(entryIndex);
        entryList_[entryCounter_]=entry;
        entry->getEntryInfo(&slavePdoEntries_[entryCounter_]);
        entryCounter_++;
      }
    }
  }
  slaveSyncs_[syncManArrayIndex_].index=0xff; //Terminate structure for ecrt_slave_config_pdos()

  if(entryCounter_==0){
    LOGINFO5("%s/%s:%d: WARNING: No Pdo:s to configured for slave %d.\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_);
    return 0;
  }

  writeEntriesStruct();
  writePdoStruct();
  writeSyncsStruct();

  if (ecrt_slave_config_pdos(slaveConfig_, EC_END, slaveSyncs_)){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): PDO configuration failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_CONFIG_PDOS_FAILED);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_CONFIG_PDOS_FAILED);
  }
  LOGINFO5("%s/%s:%d: INFO: Configuration done successfully for slave %d.\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_);

  return 0;
}

void ecmcEcSlave::writeEntriesStruct()
{
  if(simSlave_){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): PDO configuration failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_CONFIG_PDOS_FAILED);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return;
  }
  LOGINFO5("%s/%s:%d: INFO: Writing slave_pdo_entries.\n",__FILE__, __FUNCTION__, __LINE__);
  LOGINFO5("\t\tIndex\tSubIndex\tBitLength \n");
  for(int i=0;i<entryCounter_;i++){
    LOGINFO5("\t\t{%x\t%x\t%d}\n",slavePdoEntries_[i].index,slavePdoEntries_[i].subindex,slavePdoEntries_[i].bit_length);
  }
}
void ecmcEcSlave::writePdoStruct()
{
  if(simSlave_){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Simulation slave: Functionality not supported (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return;
  }
  LOGINFO5("%s/%s:%d: INFO: Writing slave_pdos.\n",__FILE__, __FUNCTION__, __LINE__);
  LOGINFO5("\t\tIndex\tEntryCount\n");
  for(int i=0;i<pdosArrayIndex_;i++){
    LOGINFO5("\t\t{%x\t%x}\n",slavePdos_[i].index,slavePdos_[i].n_entries/*,slave_pdos[i].entries*/);
  }
}

void ecmcEcSlave::writeSyncsStruct()
{
  if(simSlave_){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Simulation slave: Functionality not supported (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return;
  }
  LOGINFO5("%s/%s:%d: INFO: Writing slave_syncs.\n",__FILE__, __FUNCTION__, __LINE__);
  LOGINFO5("\t\tIndex\tDirection\tPdoCount\tWatchDog\n");
  for(int i=0;i<syncManArrayIndex_;i++){
    LOGINFO5("\t\t{%x\t%x\t%x\t%x}\n",slaveSyncs_[i].index,slaveSyncs_[i].dir,slaveSyncs_[i].n_pdos/*,slave_syncs[i].pdos*/,slaveSyncs_[i].watchdog_mode);
  }
}

int ecmcEcSlave::checkConfigState(void)
{
  if(simSlave_){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Simulation slave: Functionality not supported (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE);
  }
  ec_slave_config_state_t slaveState;
  memset(&slaveState,0,sizeof(slaveState));
  ecrt_slave_config_state(slaveConfig_, &slaveState);
  if (slaveState.al_state != slaveStateOld_.al_state){
    LOGINFO5("%s/%s:%d: INFO: Slave position: %d. State 0x%x.\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_ ,slaveState.al_state);
  }
  if (slaveState.online != slaveStateOld_.online){
    LOGINFO5("%s/%s:%d: INFO: Slave position: %d %s.\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_ ,slaveState.online ? "Online" : "Offline");
  }
  if (slaveState.operational != slaveStateOld_.operational){
    LOGINFO5("%s/%s:%d: INFO: Slave position: %d %s operational.\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,slaveState.operational ? "" : "Not ");
  }
  slaveStateOld_ = slaveState;

  if(!slaveState.online){
    if(getErrorID()!=ERROR_EC_SLAVE_NOT_ONLINE){
     LOGERR("%s/%s:%d: ERROR: Slave %d: Not online (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,ERROR_EC_SLAVE_NOT_ONLINE);
    }
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_NOT_ONLINE);
  }
  if(!slaveState.operational){
    if(getErrorID()!=ERROR_EC_SLAVE_NOT_OPERATIONAL){
      LOGERR("%s/%s:%d: ERROR: Slave %d: Not operational (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,ERROR_EC_SLAVE_NOT_OPERATIONAL);
    }
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_NOT_OPERATIONAL);
  }

  switch(slaveState.al_state){
    case 1:
      if(getErrorID()!=ERROR_EC_SLAVE_STATE_INIT){
        LOGERR("%s/%s:%d: ERROR: Slave %d: State INIT (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,ERROR_EC_SLAVE_STATE_INIT);
      }
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_STATE_INIT);
      break;
    case 2:
      if(getErrorID()!=ERROR_EC_SLAVE_STATE_PREOP){
        LOGERR("%s/%s:%d: ERROR: Slave %d: State PREOP (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,ERROR_EC_SLAVE_STATE_PREOP);
      }
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_STATE_PREOP);
      break;
    case 4:
      if(getErrorID()!=ERROR_EC_SLAVE_STATE_SAFEOP){
        LOGERR("%s/%s:%d: ERROR: Slave %d: State SAFEOP (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,ERROR_EC_SLAVE_STATE_SAFEOP);
      }
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_STATE_SAFEOP);
      break;
    case 8:
      //OK
      return 0;
      break;
    default:
      if(getErrorID()!=ERROR_EC_SLAVE_STATE_UNDEFINED){
        LOGERR("%s/%s:%d: ERROR: Slave %d: State UNDEFINED (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,ERROR_EC_SLAVE_STATE_UNDEFINED);
      }
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_STATE_UNDEFINED);
      break;
  }

  return 0;
}

ecmcEcEntry *ecmcEcSlave::getEntry(int entryIndex)
{
  if(!simSlave_){
    if(entryIndex>=EC_MAX_ENTRIES){
      LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry index out of range (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE);
      setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE);
      return NULL;
    }

    if(entryList_[entryIndex]==NULL ){
      LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_ENTRY_NULL);
      setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_ENTRY_NULL);
      return NULL;
    }
    return entryList_[entryIndex];
  }
  else{
    if(entryIndex>=SIMULATION_ENTRIES){
      LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry index out of range (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE);
      setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE);
      return NULL;
    }

    if(simEntries_[entryIndex]==NULL){
      LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_ENTRY_NULL);
      setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_ENTRY_NULL);
      return NULL;
    }
    return simEntries_[entryIndex];
  }
}

int ecmcEcSlave::updateInputProcessImage()
{
  for(int i=0;i<entryCounter_;i++){
    if(entryList_[i]!=NULL){
      entryList_[i]->updateInputProcessImage();
    }
  }

  return 0;
}

int ecmcEcSlave::updateOutProcessImage()
{
  for(int i=0;i<entryCounter_;i++){
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
  ecmcEcSyncManager *syncManager=findSyncMan(syncMangerIndex);
  if(syncManager==NULL){
    int error=addSyncManager(direction,syncMangerIndex);
    if(error){
      LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Add sync manager failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,error);
      return error;
    }
    syncManager=syncManagerArray_[syncManCounter_-1]; //last added sync manager
  }
  syncManager->addEntry(pdoIndex,entryIndex,entrySubIndex,bits,id);

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
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Slave Config NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_CONFIG_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_CONFIG_NULL);
  }

  ecrt_slave_config_dc(slaveConfig_,assignActivate,sync0Cycle,sync0Shift,sync1Cycle,sync1Shift);
  return 0;
}

ecmcEcEntry *ecmcEcSlave::findEntry(std::string id)
{
  //Real entries
  /*for(int i=0; i<EC_MAX_ENTRIES;i++){
    if(entryList_[i]!=NULL){
      if(entryList_[i]->getIdentificationName().compare(id)==0){
        return entryList_[i];
      }
    }
  }*/

  ecmcEcEntry *temp=NULL;
  for(int i=0;i<syncManCounter_;i++){
    temp=syncManagerArray_[i]->findEntry(id);
    if(temp){
      return temp;
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
  LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Entry not found (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_ENTRY_NULL);
  return -ERROR_EC_SLAVE_ENTRY_NULL;
}

int ecmcEcSlave::selectAsReferenceDC()
{
  return ecrt_master_select_reference_clock(master_,slaveConfig_);
}

int ecmcEcSlave::setWatchDogConfig(
        uint16_t watchdogDivider, /**< Number of 40 ns intervals. Used as a
                                     base unit for all slave watchdogs. If set
                                     to zero, the value is not written, so the
                                     default is used. */
        uint16_t watchdogIntervals /**< Number of base intervals for process
                                    data watchdog. If set to zero, the value
                                    is not written, so the default is used.
                                   */
    )
{
  if(!slaveConfig_){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Slave Config NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_CONFIG_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_CONFIG_NULL);
  }
  ecrt_slave_config_watchdog(slaveConfig_,watchdogDivider,watchdogIntervals);
  return 0;
}

int ecmcEcSlave::addSDOWrite(uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t writeValue, int byteSize)
{
  if(!slaveConfig_){
    LOGERR("%s/%s:%d: ERROR: Slave %d (0x%x,0x%x): Slave Config NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slavePosition_,vendorId_,productCode_,ERROR_EC_SLAVE_CONFIG_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_SLAVE_CONFIG_NULL);
  }

  return ecmcEcSDO::addSdoConfig(slaveConfig_,slavePosition_,sdoIndex,sdoSubIndex,writeValue,byteSize);;
}
