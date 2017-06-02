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
  simSlave_=new ecmcEcSlave(NULL,0,0,0,0);
  setErrorID(ERROR_EC_STATUS_NOT_OK);
}

void ecmcEc::initVars()
{
  errorReset();
  slaveCounter_=0;
  initDone_=false;
  diag_=true;
  sdoCounter_=0;
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
  domainNotOKCyclesLimit_=0;
  domainNotOKCounterMax_=0;
  //enableDiagnosticPrintouts_=1;
  for(int i=1; i < EC_MAX_SLAVES; i++){
    slaveArray_[i]=NULL;
  }
  for(int i=0; i < EC_MAX_ENTRIES; i++){
    sdoArray_[i]=NULL;
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
  return 0;
}

ecmcEc::~ecmcEc()
{
  LOGINFO5("%s/%s:%d: INFO: Deleting Ec.\n",__FILE__, __FUNCTION__, __LINE__);
  for(int i=0; i < slaveCounter_; i++){
    delete slaveArray_[i];
    slaveArray_[i]=NULL;
  }

  for(int i=0; i < sdoCounter_; i++){
    delete sdoArray_[i];
    sdoArray_[i]=NULL;
  }

  if(simSlave_!=NULL){
    delete simSlave_;
    simSlave_=NULL;
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

//Step 1
int ecmcEc::addSlave(
  uint16_t alias, /**< Slave alias. */
  uint16_t position, /**< Slave position. */
  uint32_t vendorId, /**< Expected vendor ID. */
  uint32_t productCode /**< Expected product code. */)
{
  LOGINFO5("%s/%s:%d: INFO: Adding EtherCAT slave (alias=%d, position=%d, vendorId=%x, productCode=%x).\n",__FILE__, __FUNCTION__, __LINE__,alias,position,vendorId,productCode);
  if(slaveCounter_<EC_MAX_SLAVES-1){
    slaveArray_[slaveCounter_]=new ecmcEcSlave(master_,alias,position,vendorId,productCode);
    slaveCounter_++;
    return slaveCounter_-1;
  }
  else{
    LOGERR("%s/%s:%d: ERROR: Slave Array full (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SLAVE_ARRAY_FULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_ARRAY_FULL);
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
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_MASTER_ACTIVATE_FAILED);
  }

  if (!(domainPd_ = ecrt_domain_data(domain_))) {
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_DOMAIN_DATA_FAILED);
  }

  mcu_ec_slave_info_light slaveinfo;
  LOGINFO5("%s/%s:%d: INFO: Writing process data offsets to entries.\n",__FILE__, __FUNCTION__, __LINE__);
  for(int slaveIndex=0; slaveIndex<slaveCounter_;slaveIndex++){
    if(slaveArray_[slaveIndex]==NULL){
      LOGERR("%s/%s:%d: ERROR: Slave NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SLAVE_NULL);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_NULL);
    }
    if(slaveArray_[slaveIndex]->getSlaveInfo(&slaveinfo)){  //Get slave info
      LOGERR("%s/%s:%d: ERROR: Get slave information failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_GET_SLAVE_INFO_FAILED);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_GET_SLAVE_INFO_FAILED);
    }
    int nEntryCount=slaveArray_[slaveIndex]->getEntryCount();
    for(int entryIndex=0;entryIndex<nEntryCount;entryIndex++){
      ecmcEcEntry *tempEntry=slaveArray_[slaveIndex]->getEntry(entryIndex);
      if(tempEntry==NULL){
	LOGERR("%s/%s:%d: ERROR: Entry NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_ENTRY_NULL);
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_ENTRY_NULL);
      }
      tempEntry->setDomainAdr(domainPd_);
    }
  }
  return 0;
}

int ecmcEc::compileRegInfo()
{
  LOGINFO5("%s/%s:%d: INFO: Compiling registration info.\n",__FILE__, __FUNCTION__, __LINE__);
  mcu_ec_slave_info_light slaveinfo;
  ec_pdo_entry_info_t entryinfo;
  int entryCounter=0;
  for(int slaveIndex=0; slaveIndex<slaveCounter_;slaveIndex++){
    if(slaveArray_[slaveIndex]==NULL){
      LOGERR("%s/%s:%d: ERROR: Slave NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SLAVE_NULL);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_NULL);
    }
    if(slaveArray_[slaveIndex]->getSlaveInfo(&slaveinfo)){  //Get slave info
      LOGERR("%s/%s:%d: ERROR: Get slave information failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_GET_SLAVE_INFO_FAILED);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_GET_SLAVE_INFO_FAILED);
    }
    slaveArray_[slaveIndex]->configPdos(domain_);
    int entryCountInSlave=slaveArray_[slaveIndex]->getEntryCount();
    for(int entryIndex=0;entryIndex<entryCountInSlave;entryIndex++){
      if(slaveArray_[slaveIndex]->getEntryInfo(entryIndex, &entryinfo)){ //Get entry info
	LOGERR("%s/%s:%d: ERROR: Get entry information failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_GET_ENTRY_INFO_FAILED);
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_GET_ENTRY_INFO_FAILED);
      }
      slaveEntriesReg_[entryCounter].alias=slaveinfo.alias;
      slaveEntriesReg_[entryCounter].position=  slaveinfo.position;
      slaveEntriesReg_[entryCounter].vendor_id=slaveinfo.vendor_id;;
      slaveEntriesReg_[entryCounter].product_code=slaveinfo.product_code;;
      slaveEntriesReg_[entryCounter].index=entryinfo.index; //0x7000
      slaveEntriesReg_[entryCounter].subindex=entryinfo.subindex;//0x0
      slaveEntriesReg_[entryCounter].offset=&pdoByteOffsetArray_[entryCounter]; //Output byte address offset
      slaveEntriesReg_[entryCounter].bit_position=&pdoBitOffsetArray_[entryCounter]; //Output address bit offset
      entryCounter++;
    }
  }
  // slave_entries_reg filled
  LOGINFO5("%s/%s:%d: INFO: Slave register entries structure filled.\n",__FILE__, __FUNCTION__, __LINE__);
  for(int i=0;i<entryCounter;i++){
    LOGINFO5("\t\t{%d\t%d\t%x\t%x\t%x\t%x}\n",slaveEntriesReg_[i].alias,slaveEntriesReg_[i].position,slaveEntriesReg_[i].vendor_id,slaveEntriesReg_[i].product_code,slaveEntriesReg_[i].index,slaveEntriesReg_[i].subindex);
  }
  //Register to get offsets!
  if (ecrt_domain_reg_pdo_entry_list(domain_, slaveEntriesReg_)) {  //Now offsets are stored in slave_enties reg
    LOGERR("%s/%s:%d: ERROR: Entry list registartion failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_DOM_REG_PDO_ENTRY_LIST_FAILED);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_DOM_REG_PDO_ENTRY_LIST_FAILED);
  }
  LOGINFO5("%s/%s:%d: INFO: Writing address offsets to entry objects.\n",__FILE__, __FUNCTION__, __LINE__);

  entryCounter=0;
  //Write offstes to entries
  for(int slaveIndex=0; slaveIndex<slaveCounter_;slaveIndex++){
    if(slaveArray_[slaveIndex]==NULL){
      LOGERR("%s/%s:%d: ERROR: Slave NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SLAVE_NULL);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SLAVE_NULL);
    }
    if(slaveArray_[slaveIndex]->getSlaveInfo(&slaveinfo)){  //Get slave info
      LOGERR("%s/%s:%d: ERROR: Get slave information failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_GET_SLAVE_INFO_FAILED);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_GET_SLAVE_INFO_FAILED);
    }
    int entryCountInSlave=slaveArray_[slaveIndex]->getEntryCount();
    for(int entryIndex=0;entryIndex<entryCountInSlave;entryIndex++){
      ecmcEcEntry *tempEntry=slaveArray_[slaveIndex]->getEntry(entryIndex);
      if(tempEntry==NULL){
        LOGERR("%s/%s:%d: ERROR: Entry NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_ENTRY_NULL);
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_ENTRY_NULL);
      }
      tempEntry->setAdrOffsets(pdoByteOffsetArray_[entryCounter],pdoBitOffsetArray_[entryCounter]);
      entryCounter++;
    }
  }
  LOGINFO5("%s/%s:%d: INFO: Leaving ecmcEc::compileRegInfo. Entries registered: %d.\n",__FILE__, __FUNCTION__, __LINE__,entryCounter);
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
        //LOGERR("%s/%s:%d: ERROR: Application layer state INIT (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_AL_STATE_INIT);
        setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_AL_STATE_INIT);
        masterOK_=false;
        return false;
        break;
      case EC_AL_STATE_PREOP:
        //LOGERR("%s/%s:%d: ERROR: Application layer state PREOP (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_AL_STATE_PREOP);
        setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_AL_STATE_PREOP);
        masterOK_=false;
        return false;
        break;
      case EC_AL_STATE_SAFEOP:
        //LOGERR("%s/%s:%d: ERROR: Application layer state PREOP (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_AL_STATE_SAFEOP);
        setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_AL_STATE_SAFEOP);
        masterOK_=false;
        return false;
        break;
    }
  }

  if(!masterState_.link_up){
    //LOGERR("%s/%s:%d: ERROR: Master link down (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_LINK_DOWN);
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
  if(sdoCounter_>=EC_MAX_ENTRIES-1){
    LOGERR("%s/%s:%d: ERROR: SDO object array full (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SDO_ARRAY_FULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SDO_ARRAY_FULL);
  }
  sdoArray_[sdoCounter_]=new ecmcEcSDO(master_,slavePosition,sdoIndex,sdoSubIndex,writeValue, byteSize);
  sdoCounter_++;
  return 0;
}

int ecmcEc::writeAndVerifySDOs()
{
  for(int i=0;i<sdoCounter_;i++){
    if(sdoArray_[i]==NULL){
      LOGERR("%s/%s:%d: ERROR: SDO object NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SDO_ENTRY_NULL);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SDO_ENTRY_NULL);
    }
    int iRet;
    if((iRet=sdoArray_[i]->writeAndVerify())){
      LOGERR("%s/%s:%d: ERROR: SDO write and verify failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,iRet);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,iRet);
    }
  }
  return 0;
}

int ecmcEc::writeSDO(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t value, int byteSize)
{
  ecmcEcSDO * sdo=new ecmcEcSDO(master_,slavePosition,sdoIndex,sdoSubIndex, byteSize);
  int nRet=sdo->write(value);
  delete sdo;
  return nRet;
}

uint32_t ecmcEc::readSDO(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex, int byteSize)
{
  ecmcEcSDO * sdo=new ecmcEcSDO(master_,slavePosition,sdoIndex,sdoSubIndex, byteSize);
  if(sdo->read()){
    LOGERR("%s/%s:%d: ERROR: SDO read failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_MAIN_SDO_READ_FAILED);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_MAIN_SDO_READ_FAILED);
    delete sdo;
    return 0;
  }
  uint32_t nRet=sdo->getReadValue();
  delete sdo;
  return nRet;
}

int ecmcEc::updateInputProcessImage()
{
  for(int i=0;i<slaveCounter_;i++){
    if(slaveArray_[i]!=NULL){
      slaveArray_[i]->updateInputProcessImage();
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

  //I/O intr to EPCIS.
  if(asynPortDriver_ ){
    asynPortDriver_-> callParamCallbacks();
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
    slave=slaveArray_[slaveIndex]; //last added slave
  }
  slave->addEntry(direction,syncMangerIndex,pdoIndex,entryIndex,entrySubIndex,bits,id);

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

void ecmcEc::printStatus()
{
  LOGINFO5("%s/%s:%d: INFO: MasterOK: %d, SlavesOK: %d, DomainOK: %d, DomainNotOKCounter: %d, DomainNotOKLimit: %d, Error Code:0x%x .\n",__FILE__, __FUNCTION__, __LINE__,masterOK_,slavesOK_,domainOK_,domainNotOKCounterMax_,domainNotOKCyclesLimit_,getErrorID());
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

int ecmcEc::linkEcEntryToAsynParameter(void* asynPortObject, int slaveNumber, const char *entryIDString, int asynParType, int skipCycles){

  ecmcEcSlave *slave=NULL;
  if(slaveNumber>=0){
    slave=findSlave(slaveNumber);
  }
  else{ //simulation slave
    slave=getSlave(slaveNumber);
  }

  if(slave==NULL){
    return ERROR_EC_MAIN_SLAVE_NULL;
  }

  std::string sEntryID=entryIDString;

  ecmcEcEntry *entry=slave->findEntry(sEntryID);

  if(entry==NULL){
    return ERROR_EC_MAIN_ENTRY_NULL;
  }

  if(skipCycles<0){
    LOGERR("%s/%s:%d: ERROR: Skip cycles value invalid (must >=0),(0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_ASYN_SKIP_CYCLES_INVALID);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ASYN_SKIP_CYCLES_INVALID);
  }

  char buffer[1024];
  uint bytesWritten=snprintf ( buffer, sizeof(buffer), "EC%d_%s",slaveNumber,entryIDString);
  if(bytesWritten>=sizeof(buffer)-1){
    LOGERR("%s/%s:%d: ERROR: Alias to long (0x%x). Buffer size is %d.\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_ALIAS_TO_LONG,(int)sizeof(buffer));
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ALIAS_TO_LONG);
  }

  int index=-1;

  asynPortDriver_=(ecmcAsynPortDriver*)asynPortObject;
  if(asynPortDriver_==NULL){
    LOGERR("%s/%s:%d: ERROR: Asyn port driver object NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_EC_ASYN_PORT_OBJ_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ASYN_PORT_OBJ_NULL);
  }

  asynStatus status = asynPortDriver_->createParam(buffer,(asynParamType)asynParType,&index);

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
