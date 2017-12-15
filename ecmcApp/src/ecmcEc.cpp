/*
 * ecmcEc.cpp
 *
 *  Created on: Dec 1, 2015
 *      Author: anderssandstrom
 */

#include "ecmcEc.h"

ecmcEc::ecmcEc()
{
  initVars();
  simSlave_=new ecmcEcSlave(NULL,NULL,0,0,0,0);
  setErrorID(ERROR_EC_STATUS_NOT_OK);
}

void ecmcEc::initVars()
{
  errorReset();
  slaveCounter_=0;
  initDone_=false;
  diag_=true;
  simSlave_=NULL;
  master_=NULL;
  domain_=NULL;
  domainStateOld_.redundancy_active=0;
  domainStateOld_.wc_state=EC_WC_ZERO;
  domainStateOld_.working_counter=0;
  masterStateOld_.slaves_responding=0;
  masterStateOld_.link_up=0;
  masterStateOld_.al_states=0;
  domainPd_=0;
  slavesOK_=0;
  masterOK_=0;
  domainOK_=0;
  domainNotOKCounter_=0;
  domainNotOKCounterTotal_=0;
  domainNotOKCyclesLimit_=0;
  domainNotOKCounterMax_=0;
  for(int i=1; i < EC_MAX_SLAVES; i++){
    slaveArray_[i]=NULL;
  }
  for(int i=0; i < EC_MAX_ENTRIES; i++){
    pdoByteOffsetArray_[i]=0;
    pdoBitOffsetArray_[i]=0;
    slaveEntriesReg_[i].alias=0;
    slaveEntriesReg_[i].bit_position=0;
    slaveEntriesReg_[i].index=0;
    slaveEntriesReg_[i].offset=0;
    slaveEntriesReg_[i].position=0;
    slaveEntriesReg_[i].product_code=0;
    slaveEntriesReg_[i].subindex=0;
    slaveEntriesReg_[i].vendor_id=0;
  }
  memset(&domainState_,0,sizeof(domainState_));
  memset(&masterState_,0,sizeof(masterState_));
  inStartupPhase_=true;
  asynPortDriver_=NULL;

  ecMemMapArrayCounter_=0;
  for(int i=0;i<EC_MAX_MEM_MAPS;i++){
    ecMemMapArray_[i]=NULL;
  }
  domainSize_=0;
  statusOutputEntry_=NULL;
  masterIndex_=-1;

  updateDefAsynParams_=0;
  asynParIdSlaveCounter_=0;
  asynParIdMemMapCounter_=0;
  asynParIdSlavesStatus_=0;
  asynParIdDomianStatus_=0;
  asynParIdDomianFailCounter_=0;
  asynParIdAlState=0;
  entryCounter_=0;
  asynParIdDomianFailCounterTotal_=0;

  asynUpdateCycleCounter_=0;
  asynUpdateCycles_=0;
  asynParIdMasterLink_=0;
}

int ecmcEc::init(int nMasterIndex)
{
  master_ = ecrt_request_master(nMasterIndex);

  if (!master_){
    LOGERR("%s/%s:%d: ERROR: EtherCAT master request failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_REQUEST_FAILED);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_REQUEST_FAILED);
  }

  domain_ = ecrt_master_create_domain(master_);
  if (!domain_){
    LOGERR("%s/%s:%d: ERROR: EtherCAT create domain failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_CREATE_DOMAIN_FAILED);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_CREATE_DOMAIN_FAILED);
  }
  initDone_=true;
  masterIndex_=nMasterIndex;
  return 0;
}

ecmcEc::~ecmcEc()
{
  LOGINFO5("%s/%s:%d: INFO: Deleting Ec.\n",__FILE__, __FUNCTION__, __LINE__);
  for(int i=0; i < slaveCounter_; i++){
    delete slaveArray_[i];
    slaveArray_[i]=NULL;
  }

  if(simSlave_!=NULL){
    delete simSlave_;
    simSlave_=NULL;
  }

  for(int i=0;i<EC_MAX_MEM_MAPS;i++){
    delete ecMemMapArray_[i];
    ecMemMapArray_[i]=NULL;
  }
}

bool ecmcEc::getInitDone()
{
  return initDone_;
}

ec_domain_t *ecmcEc::getDomain()
{
  return domain_;
}

ec_master_t *ecmcEc::getMaster()
{
  return master_;
}

int ecmcEc::getMasterIndex()
{
  return masterIndex_;
}

//Step 1
int ecmcEc::addSlave(
  uint16_t alias, /**< Slave alias. */
  uint16_t position, /**< Slave position. */
  uint32_t vendorId, /**< Expected vendor ID. */
  uint32_t productCode /**< Expected product code. */)
{
  LOGINFO5("%s/%s:%d: INFO: Adding EtherCAT slave (alias=%d, position=%d, vendorId=%x, productCode=%x).\n",__FILE__, __FUNCTION__, __LINE__,alias,position,vendorId,productCode);
  if(slaveCounter_<EC_MAX_SLAVES-1){
    slaveArray_[slaveCounter_]=new ecmcEcSlave(master_,domain_,alias,position,vendorId,productCode);
    slaveCounter_++;
    if(updateDefAsynParams_){
      asynPortDriver_-> setIntegerParam(asynParIdSlaveCounter_,slaveCounter_);
      asynPortDriver_-> callParamCallbacks();
    }
    return slaveCounter_-1;
  }
  else{
    LOGERR("%s/%s:%d: ERROR: Slave Array full (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SLAVE_ARRAY_FULL);
    return -setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_ARRAY_FULL);
  }
}

ecmcEcSlave *ecmcEc::getSlave(int slaveIndex)
{
  if(slaveIndex>=EC_MAX_SLAVES || slaveIndex<-1 || slaveIndex>=slaveCounter_){
    LOGERR("%s/%s:%d: ERROR: Invalid slave index (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_INVALID_SLAVE_INDEX);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_INVALID_SLAVE_INDEX);
    return NULL;
  }

  if(slaveIndex==-1){
    return simSlave_;
  }

  if(slaveArray_[slaveIndex]==NULL){
    LOGERR("%s/%s:%d: ERROR: Slave NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SLAVE_NULL);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_NULL);
    return NULL;
  }

  return slaveArray_[slaveIndex];
}

int ecmcEc::activate()
{
  LOGINFO5("%s/%s:%d: INFO: Activating master...\n",__FILE__, __FUNCTION__, __LINE__);

  if (ecrt_master_activate(master_)){
    LOGERR("%s/%s:%d: ERROR: ecrt_master_activate() failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_MASTER_ACTIVATE_FAILED);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_MASTER_ACTIVATE_FAILED);
  }

  if (!(domainPd_ = ecrt_domain_data(domain_))) {
    LOGERR("%s/%s:%d: ERROR: ecrt_domain_data() failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_DOMAIN_DATA_FAILED);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_DOMAIN_DATA_FAILED);
  }

  LOGINFO5("%s/%s:%d: INFO: Writing process data offsets to entries.\n",__FILE__, __FUNCTION__, __LINE__);
  for(int slaveIndex=0; slaveIndex<slaveCounter_;slaveIndex++){
    if(slaveArray_[slaveIndex]==NULL){
      LOGERR("%s/%s:%d: ERROR: Slave NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SLAVE_NULL);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_NULL);
    }
    int nEntryCount=slaveArray_[slaveIndex]->getEntryCount();
    for(int entryIndex=0;entryIndex<nEntryCount;entryIndex++){
      ecmcEcEntry *tempEntry=slaveArray_[slaveIndex]->getEntry(entryIndex);
      if(tempEntry==NULL){
	LOGERR("%s/%s:%d: ERROR: Entry NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_ENTRY_NULL);
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_ENTRY_NULL);
      }
      tempEntry->setDomainAdr(domainPd_);
      LOGINFO5("%s/%s:%d: INFO: Entry %s (index = %d): domainAdr: %p.\n",__FILE__, __FUNCTION__, __LINE__,tempEntry->getIdentificationName().c_str(),entryIndex,domainPd_);
    }
  }

  return 0;
}

int ecmcEc::compileRegInfo()
{
  int entryCounter=0;
  //Write offstes to entries
  for(int slaveIndex=0; slaveIndex<slaveCounter_;slaveIndex++){
    if(slaveArray_[slaveIndex]==NULL){
      LOGERR("%s/%s:%d: ERROR: Slave NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SLAVE_NULL);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_NULL);
    }
    int entryCountInSlave=slaveArray_[slaveIndex]->getEntryCount();
    for(int entryIndex=0;entryIndex<entryCountInSlave;entryIndex++){
      ecmcEcEntry *tempEntry=slaveArray_[slaveIndex]->getEntry(entryIndex);
      if(tempEntry==NULL){
        LOGERR("%s/%s:%d: ERROR: Entry NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_ENTRY_NULL);
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_ENTRY_NULL);
      }
      int ret=tempEntry->registerInDomain();
      if(ret){
         LOGERR("%s/%s:%d: ERROR: register entry in domain failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ret);
	 return setErrorID(__FILE__,__FUNCTION__,__LINE__,ret);
      }
      entryCounter++;
    }
  }

  //Set domain size to MemMap objects to avoid write outside memarea
  domainSize_=ecrt_domain_size(domain_);
  for(int i=0;i<ecMemMapArrayCounter_;i++){
    if(ecMemMapArray_[i]){
      ecMemMapArray_[i]->setDomainSize(domainSize_);
    }
  }

  //LOGINFO5("%s/%s:%d: INFO: Leaving ecmcEc::compileRegInfo. Entries registered: %d.\n",__FILE__, __FUNCTION__, __LINE__,entryCounter);
  return 0;
}

void ecmcEc::checkDomainState(void)
{
  if(!diag_){
    domainOK_=true;
  }

  ecrt_domain_state(domain_, &domainState_);

  //filter domainOK_ for some cycles
  if( domainState_.wc_state!=EC_WC_COMPLETE){
    if(domainNotOKCounter_<=domainNotOKCyclesLimit_){
      domainNotOKCounter_++;
    }
    if(domainNotOKCounter_> domainNotOKCounterMax_){
      domainNotOKCounterMax_=domainNotOKCounter_;
    }
    domainNotOKCounterTotal_++;
  }
  else{
    domainNotOKCounter_=0;
  }
  domainOK_=domainNotOKCounter_<=domainNotOKCyclesLimit_;
}

bool ecmcEc::checkSlavesConfState()
{
  if(!diag_){
    slavesOK_=true;
    return slavesOK_;
  }

  int retVal=0;
  for(int i=0;i<slaveCounter_;i++){
    retVal=checkSlaveConfState(i);
    if(retVal){
      LOGINFO5("%s/%s:%d: INFO: Slave with bus position %d reports error (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,slaveArray_[i]->getSlaveBusPosition(),retVal);
      slavesOK_=false;
      setErrorID(__FILE__,__FUNCTION__,__LINE__,retVal);
      return slavesOK_;
    }
  }

  slavesOK_=true;
  return slavesOK_;
}

int ecmcEc::checkSlaveConfState(int slaveIndex)
{
  if(!diag_){
    return 0;
  }

  if(slaveIndex>=EC_MAX_SLAVES || slaveIndex >=slaveCounter_){
    LOGERR("%s/%s:%d: ERROR: Invalid slave index (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_INVALID_SLAVE_INDEX);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_INVALID_SLAVE_INDEX);
  }

  if(slaveArray_[slaveIndex]==NULL){
    LOGERR("%s/%s:%d: ERROR: Slave NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SLAVE_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_NULL);
  }

  return slaveArray_[slaveIndex]->checkConfigState();
}

bool ecmcEc::checkState(void)
{
  if(!diag_){
    masterOK_=true;
    return true;
  }

  bool slavesUp=checkSlavesConfState();
  if(!slavesUp){
    return slavesUp;
  }

  ecrt_master_state(master_, &masterState_);

  if (masterState_.slaves_responding != masterStateOld_.slaves_responding){
    LOGINFO5("%s/%s:%d: INFO: %u slave(s) responding.\n",__FILE__, __FUNCTION__, __LINE__,masterState_.slaves_responding);
  }
  if (masterState_.link_up != masterStateOld_.link_up){
    LOGINFO5("%s/%s:%d: INFO: Master link is %s.\n",__FILE__, __FUNCTION__, __LINE__,masterState_.link_up ? "up" : "down");
  }
  if (masterState_.al_states != masterStateOld_.al_states){
    LOGINFO5("%s/%s:%d: INFO: Application Layer state: 0x%x.\n",__FILE__, __FUNCTION__, __LINE__,masterState_.al_states);
  }
  masterStateOld_ = masterState_;

  if((int)masterState_.slaves_responding<slaveCounter_){
      LOGERR("%s/%s:%d: ERROR: Respondig slave count VS configures slave count missmatch (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_RESPOND_VS_CONFIG_SLAVES_MISSMATCH);
      setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_RESPOND_VS_CONFIG_SLAVES_MISSMATCH);
      masterOK_=false;
      return false;
  }

  if(masterState_.al_states!=EC_AL_STATE_OP){
    switch(masterState_.al_states){
      case EC_AL_STATE_INIT:
        setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_AL_STATE_INIT);
        masterOK_=false;
        return false;
        break;
      case EC_AL_STATE_PREOP:
        setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_AL_STATE_PREOP);
        masterOK_=false;
        return false;
        break;
      case EC_AL_STATE_SAFEOP:
        setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_AL_STATE_SAFEOP);
        masterOK_=false;
        return false;
        break;
    }
  }

  if(!masterState_.link_up){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_LINK_DOWN);
    masterOK_=false;
    return false;
  }

  masterOK_=true;

  return masterOK_;
}

void ecmcEc::receive()
{
  ecrt_master_receive(master_);
  ecrt_domain_process(domain_);
  updateInputProcessImage();
}

void ecmcEc::send(timespec timeOffset)
{
  struct timespec timeRel,timeAbs;

  // Write status hardware status to output
  if(statusOutputEntry_){
    statusOutputEntry_->writeValue(getErrorID()==0);
  }

  updateOutProcessImage();
  ecrt_domain_queue(domain_);

  clock_gettime(CLOCK_MONOTONIC, &timeRel);
  timeAbs=timespecAdd(timeRel,timeOffset);

  ecrt_master_application_time(master_, TIMESPEC2NS(timeAbs));
  ecrt_master_sync_reference_clock(master_);
  ecrt_master_sync_slave_clocks(master_);
  ecrt_master_send(master_);
}

int ecmcEc::setDiagnostics(bool bDiag)
{
  diag_=bDiag;
  return 0;
}

int ecmcEc::addSDOWrite(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t writeValue, int byteSize)
{
  ecmcEcSlave *slave= findSlave(slavePosition);
  if(!slave){
    LOGERR("%s/%s:%d: ERROR: Slave object NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SLAVE_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_NULL);
  }

  return slave->addSDOWrite(sdoIndex,sdoSubIndex,writeValue,byteSize);
}

int ecmcEc::writeSDO(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t value, int byteSize)
{
  return ecmcEcSDO::write(master_,slavePosition,sdoIndex,sdoSubIndex,value,(size_t)byteSize);;
}

int ecmcEc::writeSDOComplete(uint16_t slavePosition,uint16_t sdoIndex,uint32_t value, int byteSize)
{
  return ecmcEcSDO::writeComplete(master_,slavePosition,sdoIndex,value,(size_t)byteSize);;
}

int ecmcEc::readSDO(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,int byteSize,uint32_t *value)
{
  //uint32_t value=0;
  size_t bytesRead=0;
  int errorCode=ecmcEcSDO::read(master_,slavePosition,sdoIndex,sdoSubIndex,value,&bytesRead);
  if(errorCode){
    return errorCode;
  }
  return 0;
}

int ecmcEc::updateInputProcessImage()
{
  for(int i=0;i<slaveCounter_;i++){
    if(slaveArray_[i]!=NULL){
      slaveArray_[i]->updateInputProcessImage();
    }
  }

  for(int i=0;i<ecMemMapArrayCounter_;i++){
    if(ecMemMapArray_[i]!=NULL){
      ecMemMapArray_[i]->updateInputProcessImage();
    }
  }

  return 0;
}

int ecmcEc::updateOutProcessImage()
{
  for(int i=0;i<slaveCounter_;i++){
    if(slaveArray_[i]!=NULL){
      slaveArray_[i]->updateOutProcessImage();
    }
  }

  for(int i=0;i<ecMemMapArrayCounter_;i++){
    if(ecMemMapArray_[i]!=NULL){
	ecMemMapArray_[i]->updateOutProcessImage();
    }
  }

  //I/O intr to EPCIS.
  if(asynPortDriver_){
    if(updateDefAsynParams_){
      if(asynUpdateCycleCounter_>=asynUpdateCycles_ && asynPortDriver_!=NULL){ //Only update at desired samplerate
	asynPortDriver_-> setIntegerParam(asynParIdSlaveCounter_,masterState_.slaves_responding );
	asynPortDriver_-> setIntegerParam(asynParIdAlState,masterState_.al_states);
	asynPortDriver_-> setIntegerParam(asynParIdMasterLink_,masterState_.link_up);
        asynPortDriver_-> setIntegerParam(asynParIdSlavesStatus_,slavesOK_);
        asynPortDriver_-> setIntegerParam(asynParIdDomianStatus_,domainOK_);
        asynPortDriver_-> setIntegerParam(asynParIdDomianFailCounter_,domainNotOKCounterMax_);
        asynPortDriver_-> setIntegerParam(asynParIdDomianFailCounterTotal_,domainNotOKCounterTotal_);
        asynPortDriver_-> setIntegerParam(asynParIdEntryCounter_,entryCounter_);
        asynPortDriver_-> setIntegerParam(asynParIdMemMapCounter_,ecMemMapArrayCounter_);
      }
      else{
        asynUpdateCycleCounter_++;
      }
    }
  }

  return 0;
}

int ecmcEc::addEntry(
    uint16_t       position, /**< Slave position. */
    uint32_t       vendorId, /**< Expected vendor ID. */
    uint32_t       productCode, /**< Expected product code. */
    ec_direction_t direction,
    uint8_t        syncMangerIndex,
    uint16_t       pdoIndex,
    uint16_t       entryIndex,
    uint8_t        entrySubIndex,
    uint8_t        bits,
    std::string    id
    )
{
  ecmcEcSlave *slave=findSlave(position);
  if(slave==NULL){
    int slaveIndex=addSlave(0,position,vendorId,productCode);
    if(slaveIndex<0){ //Error
      return -slaveIndex;
    }
    slave=slaveArray_[slaveIndex]; //last added slave
  }

  int errorCode=slave->addEntry(direction,syncMangerIndex,pdoIndex,entryIndex,entrySubIndex,bits,id);
  if(errorCode){
    return errorCode;
  }
  entryCounter_++;
  if(updateDefAsynParams_){
    asynPortDriver_-> setIntegerParam(asynParIdEntryCounter_,entryCounter_);
    asynPortDriver_-> callParamCallbacks();
  }

  return 0;
}

ecmcEcSlave *ecmcEc::findSlave(int busPosition)
{
  if(busPosition==-1){
     return simSlave_;
  }

  for(int i=0;i< slaveCounter_;i++){
    if(slaveArray_[i]!=NULL){
      if(slaveArray_[i]->getSlaveBusPosition()==busPosition){
        return slaveArray_[i];
      }
    }
  }
  return NULL;
}

int ecmcEc::findSlaveIndex(int busPosition,int *slaveIndex)
{
  if(busPosition==-1){
    *slaveIndex=busPosition;
    return 0;
  }

  for(int i=0;i< slaveCounter_;i++){
    if(slaveArray_[i]!=NULL){
      if(slaveArray_[i]->getSlaveBusPosition()==busPosition){
        *slaveIndex=i;
        return 0;
      }
    }
  }
  LOGERR("%s/%s:%d: ERROR: Slave NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SLAVE_NULL);
  return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_NULL);
}

int ecmcEc::statusOK()
{
  if(!diag_){
    return 1;
  }

  //Auto reset error at startup
  if(inStartupPhase_ && slavesOK_ && domainOK_ && masterOK_){
    inStartupPhase_=false;
    errorReset();
  }
  return slavesOK_ && domainOK_ && masterOK_;
}

int ecmcEc::setDomainFailedCyclesLimitInterlock(int cycles)
{
  domainNotOKCyclesLimit_=cycles;
  return 0;
}

void ecmcEc::slowExecute()
{
  LOGINFO5("%s/%s:%d: INFO: MasterOK: %d, SlavesOK: %d, DomainOK: %d, DomainNotOKCounter: %d, DomainNotOKLimit: %d, Error Code:0x%x .\n",__FILE__, __FUNCTION__, __LINE__,masterOK_,slavesOK_,domainOK_,domainNotOKCounterMax_,domainNotOKCyclesLimit_,getErrorID());

  checkState();
  checkSlavesConfState();

  domainNotOKCounterMax_=0;
}

int ecmcEc::reset()
{
  ecrt_master_reset(master_);
  return 0;
}

timespec ecmcEc::timespecAdd(timespec time1, timespec time2)
{
  timespec result;

  if ((time1.tv_nsec + time2.tv_nsec) >= MCU_NSEC_PER_SEC) {
    result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec - MCU_NSEC_PER_SEC;
  } else {
    result.tv_sec = time1.tv_sec + time2.tv_sec;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
  }
  return result;
}

int ecmcEc::linkEcEntryToAsynParameter(void* asynPortObject, const char *entryIDString, int asynParType, int skipCycles)
{
  char alias[1024];
  int slaveNumber=0;
  int masterNumber=0;
  int nvals = sscanf(entryIDString,"ec%d.s%d.%s",&masterNumber ,&slaveNumber,alias);
  if (nvals != 3) {
    LOGERR("%s/%s:%d: ERROR: Master index, slave number or alias not found %s ,(0x%x).\n",__FILE__, __FUNCTION__, __LINE__,entryIDString,ERROR_EC_MAIN_SLAVE_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_NULL);
  }

  ecmcEcSlave *slave=NULL;
  if(slaveNumber>=0){
    slave=findSlave(slaveNumber);
  }
  else{ //simulation slave
    slave=getSlave(slaveNumber);
  }

  if(slave==NULL){
    LOGERR("%s/%s:%d: ERROR: Slave not found ,(0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_ENTRY_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_NULL);
  }

  std::string sID=alias;
  //std::string sID=entryIDString;

  ecmcEcEntry *entry=slave->findEntry(sID);
  if(entry==NULL){
    LOGERR("%s/%s:%d: ERROR: Entry not found ,(0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_ENTRY_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_ENTRY_NULL);
  }

  if(skipCycles<0){
    LOGERR("%s/%s:%d: ERROR: Skip cycles value invalid (must >=0),(0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_ASYN_SKIP_CYCLES_INVALID);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ASYN_SKIP_CYCLES_INVALID);
  }

  int index=-1;

  asynPortDriver_=(ecmcAsynPortDriver*)asynPortObject;
  if(asynPortDriver_==NULL){
    LOGERR("%s/%s:%d: ERROR: Asyn port driver object NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_ASYN_PORT_OBJ_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ASYN_PORT_OBJ_NULL);
  }

  asynStatus status = asynPortDriver_->createParam(entryIDString,(asynParamType)asynParType,&index);

  if(index<0 || status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Asyn port driver: Create parameter failed with asyncode %d (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,status,ERROR_EC_ASYN_PORT_CREATE_PARAM_FAIL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ASYN_PORT_CREATE_PARAM_FAIL);
  }

  entry->setAsynParameterIndex(index);
  entry->setAsynParameterType((asynParamType)asynParType);
  entry->setAsynPortDriver(asynPortDriver_);
  entry->setAsynParameterSkipCycles(skipCycles);

  return 0;
}

int ecmcEc::linkEcMemMapToAsynParameter(void* asynPortObject, const char *memMapIDString, int asynParType,int skipCycles)
{

  char alias[1024];
  int masterIndex=0;
  int nvals = sscanf(memMapIDString,"ec%d.mm.%s",&masterIndex, alias);
  if (nvals != 2) {
    LOGERR("%s/%s:%d: ERROR: Alias not found in idString %s (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,memMapIDString,ERROR_EC_ASYN_ALIAS_NOT_VALID);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ASYN_ALIAS_NOT_VALID);
  }

  std::string sID=alias;

  ecmcEcMemMap *memMap=findMemMap(sID);
  if(memMap==NULL){
    LOGERR("%s/%s:%d: ERROR: Mem map not found ,(0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MEM_MAP_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MEM_MAP_NULL);
  }

  if(skipCycles<0){
    LOGERR("%s/%s:%d: ERROR: Skip cycles value invalid (must >=0),(0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_ASYN_SKIP_CYCLES_INVALID);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ASYN_SKIP_CYCLES_INVALID);
  }

  int index=-1;

  asynPortDriver_=(ecmcAsynPortDriver*)asynPortObject;

  if(asynPortDriver_==NULL){
    LOGERR("%s/%s:%d: ERROR: Asyn port driver object NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_ASYN_PORT_OBJ_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ASYN_PORT_OBJ_NULL);
  }

  asynStatus status = asynPortDriver_->createParam(memMapIDString,(asynParamType)asynParType,&index);

  if(index<0 || status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Asyn port driver: Create parameter failed with asyncode %d (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,status,ERROR_EC_ASYN_PORT_CREATE_PARAM_FAIL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ASYN_PORT_CREATE_PARAM_FAIL);
  }

  memMap->setAsynParameterIndex(index);
  memMap->setAsynParameterType((asynParamType)asynParType);
  memMap->setAsynPortDriver(asynPortDriver_);
  memMap->setAsynParameterSkipCycles(skipCycles);

  return 0;
}

int ecmcEc::addMemMap(uint16_t startEntryBusPosition,
		    std::string startEntryIDString,
		    int byteSize,
		    int type,
		    ec_direction_t direction,
		    std::string memMapIDString)
{
  ecmcEcSlave* slave=findSlave(startEntryBusPosition);
  if(!slave){
    LOGERR("%s/%s:%d: ERROR: Slave with busposition %d noy found (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,startEntryBusPosition,ERROR_EC_MAIN_SLAVE_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_NULL);
  }

  if(ecMemMapArrayCounter_>=EC_MAX_MEM_MAPS){
    LOGERR("%s/%s:%d: ERROR: Adding ecMemMap failed. Array full (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MEM_MAP_INDEX_OUT_OF_RANGE);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MEM_MAP_INDEX_OUT_OF_RANGE);
  }

  ecmcEcEntry *entry=slave->findEntry(startEntryIDString);
  if(!entry){
    LOGERR("%s/%s:%d: ERROR: Adding ecMemMap failed. Start entry not found (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MEM_MAP_START_ENTRY_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MEM_MAP_START_ENTRY_NULL);
  }

  char alias[1024];
  std::string aliasString;
  int masterIndex=0;
  int nvals = sscanf(memMapIDString.c_str(),"ec%d.mm.%s",&masterIndex, alias);
  if (nvals != 2) {
    LOGERR("%s/%s:%d: ERROR: Alias not found in idString %s (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,memMapIDString.c_str(),ERROR_EC_ASYN_ALIAS_NOT_VALID);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ASYN_ALIAS_NOT_VALID);
  }
  aliasString=alias;
  ecMemMapArray_[ecMemMapArrayCounter_]=new ecmcEcMemMap(entry,byteSize,type,direction,aliasString);

  if(!ecMemMapArray_[ecMemMapArrayCounter_]){
    LOGERR("%s/%s:%d: ERROR: Adding ecMemMap failed. New ecmcEcMemMap fail (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MEM_MAP_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MEM_MAP_NULL );
  }

  ecMemMapArrayCounter_++;

  if(updateDefAsynParams_){
    asynPortDriver_-> setIntegerParam(asynParIdMemMapCounter_,ecMemMapArrayCounter_);
    asynPortDriver_-> callParamCallbacks();  //also for memmap and ecEntry
  }

  return 0;
}

ecmcEcMemMap *ecmcEc::findMemMap(std::string id)
{
  ecmcEcMemMap *temp=NULL;
  for(int i=0;i<ecMemMapArrayCounter_;i++){
    if(ecMemMapArray_[i]){
      if(ecMemMapArray_[i]->getIdentificationName().compare(id)==0){
	temp=ecMemMapArray_[i];
      }
    }
  }
  return temp;
}

int ecmcEc::setEcStatusOutputEntry(ecmcEcEntry *entry)
{
  statusOutputEntry_=entry;
  return 0;
}

int ecmcEc::initAsyn(ecmcAsynPortDriver* asynPortDriver,bool regAsynParams,int skipCycles)
{
  asynPortDriver_=asynPortDriver;
  updateDefAsynParams_=regAsynParams;
  asynUpdateCycles_=skipCycles;

  if(!regAsynParams){
    return 0;
  }

  char buffer[128];

  unsigned int charCount=snprintf(buffer,sizeof(buffer),"ec%d.alstates",masterIndex_);
  if(charCount>=sizeof(buffer)-1){
    LOGERR("%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }

  asynStatus status = asynPortDriver_->createParam(buffer,asynParamInt32,&asynParIdAlState);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter %s failed.\n",__FILE__,__FUNCTION__,__LINE__,buffer);
    return asynError;
  }
  asynPortDriver_-> setIntegerParam(asynParIdAlState,0);

  charCount=snprintf(buffer,sizeof(buffer),"ec%d.link",masterIndex_);
  if(charCount>=sizeof(buffer)-1){
    LOGERR("%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }

  status = asynPortDriver_->createParam(buffer,asynParamInt32,&asynParIdMasterLink_);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter %s failed.\n",__FILE__,__FUNCTION__,__LINE__,buffer);
    return asynError;
  }
  asynPortDriver_-> setIntegerParam(asynParIdMasterLink_,0);

  charCount=snprintf(buffer,sizeof(buffer),"ec%d.slavecounter",masterIndex_);
  if(charCount>=sizeof(buffer)-1){
    LOGERR("%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }

  status = asynPortDriver_->createParam(buffer,asynParamInt32,&asynParIdSlaveCounter_);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter %s failed.\n",__FILE__,__FUNCTION__,__LINE__,buffer);
    return asynError;
  }
  asynPortDriver_-> setIntegerParam(asynParIdSlaveCounter_,0);

  charCount=snprintf(buffer,sizeof(buffer),"ec%d.slavesstatus",masterIndex_);
  if(charCount>=sizeof(buffer)-1){
    LOGERR("%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }

  status = asynPortDriver_->createParam(buffer,asynParamInt32,&asynParIdSlavesStatus_);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter %s failed.\n",__FILE__,__FUNCTION__,__LINE__,buffer);
    return asynError;
  }
  asynPortDriver_-> setIntegerParam(asynParIdSlavesStatus_,0);

  charCount=snprintf(buffer,sizeof(buffer),"ec%d.memmapcounter",masterIndex_);
  if(charCount>=sizeof(buffer)-1){
    LOGERR("%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }

  status = asynPortDriver_->createParam(buffer,asynParamInt32,&asynParIdMemMapCounter_);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter %s failed.\n",__FILE__,__FUNCTION__,__LINE__,buffer);
    return asynError;
  }
  asynPortDriver_-> setIntegerParam(asynParIdMemMapCounter_,0);

  charCount=snprintf(buffer,sizeof(buffer),"ec%d.domainstatus",masterIndex_);
  if(charCount>=sizeof(buffer)-1){
    LOGERR("%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }

  status = asynPortDriver_->createParam(buffer,asynParamInt32,&asynParIdDomianStatus_);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter %s failed.\n",__FILE__,__FUNCTION__,__LINE__,buffer);
    return asynError;
  }
  asynPortDriver_-> setIntegerParam(asynParIdDomianStatus_,0);

  charCount=snprintf(buffer,sizeof(buffer),"ec%d.domainfailcounter",masterIndex_);
  if(charCount>=sizeof(buffer)-1){
    LOGERR("%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }

  status = asynPortDriver_->createParam(buffer,asynParamInt32,&asynParIdDomianFailCounter_);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter %s failed.\n",__FILE__,__FUNCTION__,__LINE__,buffer);
    return asynError;
  }
  asynPortDriver_-> setIntegerParam(asynParIdDomianFailCounter_,0);

  charCount=snprintf(buffer,sizeof(buffer),"ec%d.domainfailcountertotal",masterIndex_);
  if(charCount>=sizeof(buffer)-1){
    LOGERR("%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }

  status = asynPortDriver_->createParam(buffer,asynParamInt32,&asynParIdDomianFailCounterTotal_);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter %s failed.\n",__FILE__,__FUNCTION__,__LINE__,buffer);
    return asynError;
  }
  asynPortDriver_-> setIntegerParam(asynParIdDomianFailCounterTotal_,0);

  charCount=snprintf(buffer,sizeof(buffer),"ec%d.entrycounter",masterIndex_);
  if(charCount>=sizeof(buffer)-1){
    LOGERR("%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW);
    return ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW;
  }

  status = asynPortDriver_->createParam(buffer,asynParamInt32,&asynParIdEntryCounter_);
  if(status!=asynSuccess){
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter %s failed.\n",__FILE__,__FUNCTION__,__LINE__,buffer);
    return asynError;
  }
  asynPortDriver_-> setIntegerParam(asynParIdEntryCounter_,0);


  asynPortDriver_-> callParamCallbacks();

  return 0;
}


int ecmcEc::printAllConfig()
{
  ec_master_info_t masterInfo;
  ec_slave_info_t slaveInfo;
  ec_sync_info_t syncInfo;
  ec_pdo_info_t pdoInfo;
  ec_pdo_entry_info_t  pdoEntryInfo;
  int slaveLoopIndex=0;
  int syncManLoopIndex=0;
  int pdoLoopIndex=0;
  int entryLoopIndex=0;
  int slaveCount=0;
  int syncManCount=0;
  int pdoCount=0;
  int entryCount=0;

  if(!master_){
    LOGINFO("%s/%s:%d: INFO: No EtherCAT master selected.\n",__FILE__, __FUNCTION__, __LINE__);
    return 0;
  }

  int errorCode=ecrt_master(master_,&masterInfo);
  if(errorCode){
    LOGERR("%s/%s:%d: Error: Function ecrt_master() failed with error code 0x%x.\n",__FILE__, __FUNCTION__, __LINE__,errorCode);
    return 0;
  }

  LOGINFO("ec%d.slaveCount=%d\n",masterIndex_,masterInfo.slave_count);

  // Slave loop
  slaveCount=masterInfo.slave_count;
  for(slaveLoopIndex=0;slaveLoopIndex<slaveCount; slaveLoopIndex++){
    errorCode=ecrt_master_get_slave( master_, slaveLoopIndex, &slaveInfo);
    if(errorCode){
      LOGERR("%s/%s:%d: Error: Function ecrt_master_get_slave() failed with error code 0x%x.\n",__FILE__, __FUNCTION__, __LINE__,errorCode);
      return 0;
    }
    LOGINFO("ec%d.s%d.position=%d\n",masterIndex_,slaveLoopIndex,slaveInfo.position);
    LOGINFO("ec%d.s%d.syncManCount=%d\n",masterIndex_,slaveLoopIndex,slaveInfo.sync_count);
    LOGINFO("ec%d.s%d.alias=%d\n",masterIndex_,slaveLoopIndex,slaveInfo.alias);
    LOGINFO("ec%d.s%d.vendorId=0x%x\n",masterIndex_,slaveLoopIndex,slaveInfo.vendor_id);
    LOGINFO("ec%d.s%d.revisionNum=0x%x\n",masterIndex_,slaveLoopIndex,slaveInfo.revision_number);
    LOGINFO("ec%d.s%d.productCode=0x%x\n",masterIndex_,slaveLoopIndex,slaveInfo.product_code);
    LOGINFO("ec%d.s%d.serial_number=0x%x\n",masterIndex_,slaveLoopIndex,slaveInfo.serial_number);

    // Sync manager loop
    syncManCount=slaveInfo.sync_count;
    for(syncManLoopIndex=0; syncManLoopIndex < syncManCount; syncManLoopIndex++){
      errorCode=ecrt_master_get_sync_manager(master_, slaveLoopIndex,syncManLoopIndex,&syncInfo);
      if(errorCode){
        LOGERR("%s/%s:%d: Error: Function ecrt_master_get_sync_manager() failed with error code 0x%x.\n",__FILE__, __FUNCTION__, __LINE__,errorCode);
        return 0;
      }
      LOGINFO("ec%d.s%d.sm%d.pdoCount=%d\n",masterIndex_,slaveLoopIndex,syncManLoopIndex,syncInfo.n_pdos);
      LOGINFO("ec%d.s%d.sm%d.direction=%s\n",masterIndex_,slaveLoopIndex,syncManLoopIndex,syncInfo.dir == 1 ? "Output" : "Input");
      LOGINFO("ec%d.s%d.sm%d.watchDogMode=%s\n",masterIndex_,slaveLoopIndex,syncManLoopIndex,syncInfo.watchdog_mode == 0 ? "Default" : (syncInfo.watchdog_mode == 1 ? "Enabled" : "Disabled"));

      // Pdo loop
      pdoCount=syncInfo.n_pdos;
      for( pdoLoopIndex = 0; pdoLoopIndex < pdoCount; pdoLoopIndex++ ){
        errorCode=ecrt_master_get_pdo( master_, slaveLoopIndex, syncManLoopIndex, pdoLoopIndex, &pdoInfo);
        if(errorCode){
          LOGERR("%s/%s:%d: Error: Function ecrt_master_get_pdo() failed with error code 0x%x.\n",__FILE__, __FUNCTION__, __LINE__,errorCode);
          return 0;
        }
        LOGINFO("ec%d.s%d.sm%d.pdo%d.index=0x%x\n",masterIndex_,slaveLoopIndex,syncManLoopIndex,pdoLoopIndex,pdoInfo.index);
        LOGINFO("ec%d.s%d.sm%d.pdo%d.entryCount=%d\n",masterIndex_,slaveLoopIndex,syncManLoopIndex,pdoLoopIndex,pdoInfo.n_entries);

        // Entry loop
        entryCount=pdoInfo.n_entries;
        for( entryLoopIndex = 0; entryLoopIndex < entryCount; entryLoopIndex++ ){
          errorCode=ecrt_master_get_pdo_entry( master_, slaveLoopIndex, syncManLoopIndex, pdoLoopIndex,entryLoopIndex,&pdoEntryInfo);

          if(errorCode){
            LOGERR("%s/%s:%d: Error: Function ecrt_master_get_pdo_entry() failed with error code 0x%x.\n",__FILE__, __FUNCTION__, __LINE__,errorCode);
            return 0;
          }
          LOGINFO("ec%d.s%d.sm%d.pdo%d.e%d.index=0x%x\n",masterIndex_,slaveLoopIndex,syncManLoopIndex,pdoLoopIndex,entryLoopIndex,pdoEntryInfo.index);
          LOGINFO("ec%d.s%d.sm%d.pdo%d.e%d.subIndex=0x%x\n",masterIndex_,slaveLoopIndex,syncManLoopIndex,pdoLoopIndex,entryLoopIndex,pdoEntryInfo.subindex);
          LOGINFO("ec%d.s%d.sm%d.pdo%d.e%d.bitLength=0x%x\n",masterIndex_,slaveLoopIndex,syncManLoopIndex,pdoLoopIndex,entryLoopIndex,pdoEntryInfo.bit_length);
        }
      }
    }
  }

  return 0;
}

int ecmcEc::printSlaveConfig(int slaveIndex)
{
  ec_master_info_t masterInfo;
  ec_slave_info_t slaveInfo;
  ec_sync_info_t syncInfo;
  ec_pdo_info_t pdoInfo;
  ec_pdo_entry_info_t  pdoEntryInfo;
  int syncManLoopIndex=0;
  int pdoLoopIndex=0;
  int entryLoopIndex=0;
  int slaveCount=0;
  int syncManCount=0;
  int pdoCount=0;
  int entryCount=0;
  if(!master_){
    LOGINFO("%s/%s:%d: INFO: No EtherCAT master selected.\n",__FILE__, __FUNCTION__, __LINE__);
    return 0;
  }

  int errorCode=ecrt_master(master_,&masterInfo);
  if(errorCode){
    LOGERR("%s/%s:%d: Error: Function ecrt_master() failed with error code 0x%x.\n",__FILE__, __FUNCTION__, __LINE__,errorCode);
    return 0;
  }

  LOGINFO("ec%d.slaveCount=%d\n",masterIndex_,masterInfo.slave_count);

  // Slave loop
  slaveCount=masterInfo.slave_count;

  if(slaveIndex>=slaveCount){
    LOGINFO("%s/%s:%d: INFO: Slave index out of range.\n",__FILE__, __FUNCTION__, __LINE__);
    return 0;
  }

  errorCode=ecrt_master_get_slave( master_, slaveIndex, &slaveInfo);
  if(errorCode){
    LOGERR("%s/%s:%d: Error: Function ecrt_master_geasynParIdMasterLink_t_slave() failed with error code 0x%x.\n",__FILE__, __FUNCTION__, __LINE__,errorCode);
    return 0;
  }

  printf("#############################################\n");
  printf("# Auto generated configuration file\n");
  printf("# Slave information:\n");
  printf("#   position = %d \n",slaveInfo.position);
  printf("#   alias = %d \n",slaveInfo.alias);
  printf("#   vendor ID = %d \n",slaveInfo.vendor_id);
  printf("#   product code = %d \n",slaveInfo.product_code);
  printf("#   revison number = %d \n",slaveInfo.revision_number);
  printf("#   serial number = %d \n",slaveInfo.serial_number);
  printf("#############################################\n");

  // Sync manager loop
  syncManCount=slaveInfo.sync_count;
  for(syncManLoopIndex=0; syncManLoopIndex < syncManCount; syncManLoopIndex++){
    errorCode=ecrt_master_get_sync_manager(master_, slaveIndex,syncManLoopIndex,&syncInfo);
    if(errorCode){
      LOGERR("%s/%s:%d: Error: Function ecrt_master_get_sync_manager() failed with error code 0x%x.\n",__FILE__, __FUNCTION__, __LINE__,errorCode);
      return 0;
    }

    printf("# Configuration for sync manager %d\n",syncManLoopIndex);

    // Pdo loop
    pdoCount=syncInfo.n_pdos;
    for( pdoLoopIndex = 0; pdoLoopIndex < pdoCount; pdoLoopIndex++ ){
      errorCode=ecrt_master_get_pdo( master_, slaveIndex, syncManLoopIndex, pdoLoopIndex, &pdoInfo);
      if(errorCode){
        LOGERR("%s/%s:%d: Error: Function ecrt_master_get_pdo() failed with error code 0x%x.\n",__FILE__, __FUNCTION__, __LINE__,errorCode);
        return 0;
      }
      printf("# Configuration for pdoIndex 0x%x\n",pdoInfo.index);

      // Entry loop
      entryCount=pdoInfo.n_entries;
      for( entryLoopIndex = 0; entryLoopIndex < entryCount; entryLoopIndex++ ){
        errorCode=ecrt_master_get_pdo_entry( master_, slaveIndex, syncManLoopIndex, pdoLoopIndex,entryLoopIndex,&pdoEntryInfo);
        if(errorCode){
          LOGERR("%s/%s:%d: Error: Function ecrt_master_get_pdo_entry() failed with error code 0x%x.\n",__FILE__, __FUNCTION__, __LINE__,errorCode);
          return 0;
        }

        printf("EthercatMCConfigController ${ECMC_MOTOR_PORT},");
        printf("\"Cfg.EcAddEntryComplete(%d,0x%x,0x%x,%d,%d,0x%x,0x%x,0x%x,%d,sm%d.p%d.e%d)\"\n",slaveIndex,slaveInfo.vendor_id,
                      slaveInfo.product_code,syncInfo.dir,syncManLoopIndex,pdoInfo.index,pdoEntryInfo.index,pdoEntryInfo.subindex,pdoEntryInfo.bit_length,
	              syncManLoopIndex,pdoLoopIndex,entryLoopIndex);
      }
    }
    printf("#############################################\n");
  }
  return 0;
}

int ecmcEc::autoConfigSlave(int slaveIndex,int addAsAsynParams)
{
  ec_master_info_t masterInfo;
  ec_slave_info_t slaveInfo;
  ec_sync_info_t syncInfo;
  ec_pdo_info_t pdoInfo;
  ec_pdo_entry_info_t  pdoEntryInfo;
  int syncManLoopIndex=0;
  int pdoLoopIndex=0;
  int entryLoopIndex=0;
  int slaveCount=0;
  int syncManCount=0;
  int pdoCount=0;
  int entryCount=0;
  char entryAliasBuffer[128];
  char *entryAlias;
  char entryAliasPathBuffer[128];
  char *entryAliasPath;
  entryAlias=&entryAliasBuffer[0];
  entryAliasPath=&entryAliasPathBuffer[0];

  if(!master_){
    LOGERR("%s/%s:%d: INFO: No EtherCAT master selected (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_AUTO_CONFIG_MASTER_NOT_SELECTED_FAIL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_AUTO_CONFIG_MASTER_NOT_SELECTED_FAIL);
  }

  int errorCode=ecrt_master(master_,&masterInfo);
  if(errorCode){
    LOGERR("%s/%s:%d: Error: Function ecrt_master() failed with error code 0x%x (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,errorCode,ERROR_EC_AUTO_CONFIG_MASTER_INFO_FAIL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_AUTO_CONFIG_MASTER_INFO_FAIL);
  }

  // Slave loop
  slaveCount=masterInfo.slave_count;

  if(slaveIndex>=slaveCount){
    LOGERR("%s/%s:%d: INFO: Slave index out of range (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_AUTO_CONFIG_SLAVE_INDEX_OUT_OF_RANGE);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_AUTO_CONFIG_SLAVE_INDEX_OUT_OF_RANGE);
  }

  errorCode=ecrt_master_get_slave( master_, slaveIndex, &slaveInfo);
  if(errorCode){
    LOGERR("%s/%s:%d: Error: Function ecrt_master_get_slave() failed with error code 0x%x (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,errorCode,ERROR_EC_AUTO_CONFIG_SLAVE_INFO_FAIL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_AUTO_CONFIG_SLAVE_INFO_FAIL);
  }

  // Sync manager loop
  syncManCount=slaveInfo.sync_count;
  for(syncManLoopIndex=0; syncManLoopIndex < syncManCount; syncManLoopIndex++){
    errorCode=ecrt_master_get_sync_manager(master_, slaveIndex,syncManLoopIndex,&syncInfo);
    if(errorCode){
      LOGERR("%s/%s:%d: Error: Function ecrt_master_get_sync_manager() failed with error code 0x%x (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,errorCode,ERROR_EC_AUTO_CONFIG_SM_INFO_FAIL);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_AUTO_CONFIG_SM_INFO_FAIL);
    }

    // Pdo loop
    pdoCount=syncInfo.n_pdos;
    for( pdoLoopIndex = 0; pdoLoopIndex < pdoCount; pdoLoopIndex++ ){
      errorCode=ecrt_master_get_pdo( master_, slaveIndex, syncManLoopIndex, pdoLoopIndex, &pdoInfo);
      if(errorCode){
        LOGERR("%s/%s:%d: Error: Function ecrt_master_get_pdo() failed with error code 0x%x (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,errorCode,ERROR_EC_AUTO_CONFIG_PDO_INFO_FAIL);
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_AUTO_CONFIG_PDO_INFO_FAIL);
      }

      // Entry loop
      entryCount=pdoInfo.n_entries;
      for( entryLoopIndex = 0; entryLoopIndex < entryCount; entryLoopIndex++ ){
        errorCode=ecrt_master_get_pdo_entry( master_, slaveIndex, syncManLoopIndex, pdoLoopIndex,entryLoopIndex,&pdoEntryInfo);
        if(errorCode){
          LOGERR("%s/%s:%d: Error: Function ecrt_master_get_pdo_entry() failed with error code 0x%x (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,errorCode,ERROR_EC_AUTO_CONFIG_ENTRY_INFO_FAIL);
          return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_AUTO_CONFIG_ENTRY_INFO_FAIL);
        }

        unsigned int charCount=snprintf(entryAliasBuffer,sizeof(entryAliasBuffer),"sm%d.p%d.e%d",syncManLoopIndex,pdoLoopIndex,entryLoopIndex);
        if(charCount>=sizeof(entryAliasBuffer)-1){
          LOGERR("%s/%s:%d: Error: Autoconfig. Failed to generate alias. Buffer to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_AUTO_CONFIG_BUFFER_OVERFLOW);
          return ERROR_EC_AUTO_CONFIG_BUFFER_OVERFLOW;
        }

        //Add the entry!
        errorCode=addEntry(slaveInfo.position,slaveInfo.vendor_id,slaveInfo.product_code,syncInfo.dir,syncManLoopIndex,pdoInfo.index,pdoEntryInfo.index,pdoEntryInfo.subindex,pdoEntryInfo.bit_length,entryAlias);
        if(errorCode){
          LOGERR("%s/%s:%d: Error: Autoconfig. Add entry, alias %s, failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,entryAlias,errorCode);
          return errorCode;
        }

        if(!addAsAsynParams){
          return 0;
        }

        charCount=snprintf(entryAliasPathBuffer,sizeof(entryAliasPathBuffer),"ec%d.s%d.sm%d.p%d.e%d",masterIndex_,slaveIndex,syncManLoopIndex,pdoLoopIndex,entryLoopIndex);
        if(charCount>=sizeof(entryAliasPathBuffer)-1){
          LOGERR("%s/%s:%d: Error: Autoconfig. Failed to generate alias. Buffer to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_AUTO_CONFIG_BUFFER_OVERFLOW);
          return ERROR_EC_AUTO_CONFIG_BUFFER_OVERFLOW;
        }
        //Link to asyn parameter (default int32)
        errorCode=linkEcEntryToAsynParameter(asynPortDriver_,entryAliasPath,asynParamInt32,0);
        if(errorCode){
          LOGERR("%s/%s:%d: Error: Autoconfig. Link entry , alias %s, to Asyn parameter failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,entryAlias,errorCode);
          return errorCode;
        }

        //redo since '.' is not allowed in epics record name..
        charCount=snprintf(entryAliasBuffer,sizeof(entryAliasBuffer),"sm%d-p%d-e%d",syncManLoopIndex,pdoLoopIndex,entryLoopIndex);
        if(charCount>=sizeof(entryAliasBuffer)-1){
          LOGERR("%s/%s:%d: Error: Autoconfig. Failed to generate alias. Buffer to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_AUTO_CONFIG_BUFFER_OVERFLOW);
          return ERROR_EC_AUTO_CONFIG_BUFFER_OVERFLOW;
        }

        //Generate epics records
        char dbLoadBuffer[512];
        // Output
        switch(syncInfo.dir){
          case 1:
            charCount=snprintf(dbLoadBuffer,sizeof(dbLoadBuffer),"dbLoadRecords(\"ecmcAsynGenericAnalogOutput.db\",\"P=$(ECMC_PREFIX),PORT=$(ECMC_ASYN_PORT),ADDR=0,TIMEOUT=1,SLAVE_POS=%d,HWTYPE=%s,NAME=%s,REC_SUFFIX=data,TYPE=asynInt32\")",slaveIndex,entryAliasBuffer,entryAliasPathBuffer);
            break;
          case 2:
            charCount=snprintf(dbLoadBuffer,sizeof(dbLoadBuffer),"dbLoadRecords(\"ecmcAsynGenericAnalogInput.db\",\"P=$(ECMC_PREFIX),PORT=$(ECMC_ASYN_PORT),ADDR=0,TIMEOUT=1,SLAVE_POS=%d,HWTYPE=%s,NAME=%s,REC_SUFFIX=data,TYPE=asynInt32\")",slaveIndex,entryAliasBuffer,entryAliasPathBuffer);
            break;
          default:
            LOGERR("%s/%s:%d: Error: Autoconfig. Direction not input or output (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_AUTO_CONFIG_DIRECTION_INVALID);
            return ERROR_EC_AUTO_CONFIG_DIRECTION_INVALID;
        }
        if(charCount>=sizeof(dbLoadBuffer)-1){
          LOGERR("%s/%s:%d: Error: Autoconfig. Failed to generate dbLoadRecord string. Buffer to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_AUTO_CONFIG_BUFFER_OVERFLOW);
          return ERROR_EC_AUTO_CONFIG_BUFFER_OVERFLOW;
       }
        printf("%s\n",dbLoadBuffer);
        errorCode=iocshCmd(dbLoadBuffer);
        if(errorCode){
          LOGERR("%s/%s:%d: Error: Autoconfig. Failed to generate records. iocshCmd() returned error (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,errorCode);
          return errorCode;
        }
      }
    }
  }
  return 0;
}
