/*************************************************************************\
* Copyright (c) 2024 Paul Scherrer Institut
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcMasterSlaveStateMachine.cpp
*
*  Created on: Jun 09, 2025
*      Author: anderssandstrom
*
\*************************************************************************/
#include "ecmcMasterSlaveStateMachine.h"

ecmcMasterSlaveStateMachine::ecmcMasterSlaveStateMachine(ecmcAsynPortDriver *asynPortDriver,
                                                         int index,
                                                         const char *name,
                                                         double sampleTimeS,
                                                         ecmcAxisGroup *masterGrp,
                                                         ecmcAxisGroup *slaveGrp){
  asynPortDriver_           = asynPortDriver;
  asynEnable_               = NULL;
  asynState_                = NULL;
  asynStatus_               = NULL;
  index_                    = index;
  name_                     = name;
  sampleTimeS_              = sampleTimeS;
  timeCounter_              = 0;
  masterGrp_                = masterGrp;
  slaveGrp_                 = slaveGrp;
  validationOK_             = false;
  asynInitOk_               = false;
  optionAutoDisableMasters_ = false;
  enable_                   = 1;  // default true
  status_                   = 0;
  state_                    = ECMC_MST_SLV_STATE_IDLE;
  printf("ecmcMasterSlaveStateMachine: %s:  Created master slave state machine [%d]\n",name_.c_str(),index_);
  initAsyn();
};

ecmcMasterSlaveStateMachine::~ecmcMasterSlaveStateMachine(){
};

const char* ecmcMasterSlaveStateMachine::getName(){
  return name_.c_str();
};

void ecmcMasterSlaveStateMachine::execute(){

  //always update
  refreshAsyn();

  if(!enable_) {
    return;
  }

  if(!validationOK_) {
    return;
  }
  
  if(timeCounter_ < MST_SLV_START_DELAY_S) {
    timeCounter_+=sampleTimeS_;
    return;
  }

  switch(state_) {
    case ECMC_MST_SLV_STATE_IDLE:
      stateIdle();
      break;
    case ECMC_MST_SLV_STATE_SLAVES:
      stateSlave();
      break;
    case ECMC_MST_SLV_STATE_MASTERS:
      stateMaster();
      break;
  };
};

int ecmcMasterSlaveStateMachine::stateIdle(){

  if( slaveGrp_->getAnyBusy() && 
     !slaveGrp_->getTrajSrcAnyExt() && 
     !masterGrp_->getAnyEnabled() && 
     !masterGrp_->getAnyEnable()) {

     state_ = ECMC_MST_SLV_STATE_SLAVES;
     printf("ecmcMasterSlaveStateMachine: %s: State change, IDLE -> SLAVE_GRP_IN_CHARGE\n",name_.c_str());
  } else if(masterGrp_->getAnyEnabled() &&
            slaveGrp_->getAnyErrorId() == 0 ) {
    if(slaveGrp_->getAnyEnabled() && masterGrp_->getAnyEnabled() && !slaveGrp_->getAnyBusy()){
      if(masterGrp_->getAnyBusy()) {
        slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_EXTERNAL);
        state_ = ECMC_MST_SLV_STATE_MASTERS;
        printf("ecmcMasterSlaveStateMachine: %s: State change, IDLE -> MASTER\n", name_.c_str());
      }
    } else {
      int errorSlave = slaveGrp_->setEnable(1);
      int errorMaster = masterGrp_->setEnable(1);
      if(errorSlave || errorMaster) {
        slaveGrp_->setEnable(0);
        masterGrp_->setEnable(0);
        slaveGrp_->setMRCnen(0);
        masterGrp_->setMRCnen(0);
        slaveGrp_->setMRSync(1);
        masterGrp_->setMRSync(1);
        if(errorSlave) {
          masterGrp_->setSlavedAxisInError();
        }
        printf("ecmcMasterSlaveStateMachine: %s: Error enabling axes\n", name_.c_str());
        printf("ecmcMasterSlaveStateMachine: %s: State change, IDLE -> IDLE\n", name_.c_str());

        state_ = ECMC_MST_SLV_STATE_IDLE;
      }
      slaveGrp_->halt();
    }
  } else if(slaveGrp_->getAnyErrorId() > 0){
    masterGrp_->setSlavedAxisInError();
  }
  return 0;
}

int ecmcMasterSlaveStateMachine::stateSlave(){

  if(!slaveGrp_->getAnyBusy()) {
    slaveGrp_->setEnable(0);
    masterGrp_->setEnable(0);
    slaveGrp_->setMRCnen(0);
    masterGrp_->setMRCnen(0);
    slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);

    // Sync the master axes
    masterGrp_->setMRSync(1);
    masterGrp_->setMRStop(1);

    state_ = ECMC_MST_SLV_STATE_IDLE;
    printf("ecmcMasterSlaveStateMachine: %s: State change, SLAVE -> IDLE\n", name_.c_str());
  }
  if(masterGrp_->getAnyEnabled() && masterGrp_->getAnyEnable()) {
    masterGrp_->setEnable(0);
    masterGrp_->setMRCnen(0);
  }
  return 0;
}

int ecmcMasterSlaveStateMachine::stateMaster(){
  if(slaveGrp_->getAnyErrorId() > 0) {
    masterGrp_->setSlavedAxisInError();
  }

  if(!masterGrp_->getAnyBusy() && masterGrp_->getAnyEnabled()) {
    if(optionAutoDisableMasters_) {
      slaveGrp_->setEnable(0);
      masterGrp_->setEnable(0);
      slaveGrp_->setMRCnen(0);
      masterGrp_->setMRCnen(0);
    }
  }

  // postpone disable until all master axes are done
  masterGrp_->setEnableAutoDisable(masterGrp_->getAnyBusy() == 0);
  // ensure attarget/reduced current of slave axes
  slaveGrp_->setAxisIsWithinCtrlDBExtTraj(masterGrp_->getAxisIsWithinCtrlDB());
  // sync enable.. Not sure this should eb here.. Better in state transision maybe
  slaveGrp_->setEnable(masterGrp_->getEnable());

  // Change this to any interlock?!
  if(slaveGrp_->getAnyAtLimit()){
    state_ = ECMC_MST_SLV_STATE_SLAVES;
    printf("ecmcMasterSlaveStateMachine: %s: Slaved axis at limit..\n", name_.c_str());
    printf("ecmcMasterSlaveStateMachine: %s: State change, MASTER -> SLAVE\n", name_.c_str());
    slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);
    slaveGrp_->setMRSync(1);
    slaveGrp_->setMRStop(1);
    slaveGrp_->halt();
    masterGrp_->setSlavedAxisIlocked();
  }

  // Done?
  if(!masterGrp_->getEnabled() && !slaveGrp_->getAnyEnabled()) {
    slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);
    slaveGrp_->setErrorReset();
    state_ = ECMC_MST_SLV_STATE_IDLE;
    printf("ecmcMasterSlaveStateMachine: %s: State change, MASTER -> IDLE\n", name_.c_str());
    slaveGrp_->setMRSync(1);
    slaveGrp_->setMRStop(1);    
  }
  return 0;
}

int ecmcMasterSlaveStateMachine::validate(){
  if( masterGrp_ == NULL || slaveGrp_ == NULL){
    return ERROR_MST_SLV_SM_GRP_NULL;
  };

  if( !asynInitOk_){
    return ERROR_MST_SLV_SM_GRP_INIT_ASYN_FAILED;
  };


  validationOK_ = true;
  return 0;
};



int ecmcMasterSlaveStateMachine::initAsyn() {
  if (asynPortDriver_ == NULL) {
    LOGERR("%s/%s:%d: ERROR (master/slave state-machine %d): AsynPortDriver object NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index_,
           ERROR_AXIS_ASYN_PORT_OBJ_NULL);
    return ERROR_AXIS_ASYN_PORT_OBJ_NULL;
  }

  ecmcAsynDataItem *paramTemp = NULL;
  int errorCode               = 0;

  // Enable
  errorCode = createAsynParam(ECMC_MST_SLV_OBJ_STR "%d." ECMC_ASYN_AX_ENABLE_CMD_NAME,
                              asynParamInt32,
                              ECMC_EC_U32,
                              (uint8_t *)&(enable_),
                              sizeof(enable_),
                              &paramTemp);

  if (errorCode) {
    printf("Error creating asyn parameter for enable\n");
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->refreshParam(1);
  asynEnable_ = paramTemp;


  // State
  errorCode = createAsynParam(ECMC_MST_SLV_OBJ_STR "%d." ECMC_MST_SLVS_STR_STATE,
                              asynParamInt32,
                              ECMC_EC_U32,
                              (uint8_t *)&(state_),
                              sizeof(state_),
                              &paramTemp);

  if (errorCode) {
    printf("Error creating asyn parameter for state\n");
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->refreshParam(1);
  asynState_ = paramTemp;

  // Status
  errorCode = createAsynParam(ECMC_MST_SLV_OBJ_STR "%d." ECMC_MST_SLVS_STR_STATUS,
                              asynParamInt32,
                              ECMC_EC_U32,
                              (uint8_t *)&(status_),
                              sizeof(status_),
                              &paramTemp);

  if (errorCode) {
    printf("Error creating asyn parameter for status\n");
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  asynStatus_ = paramTemp;

  // asyn init fine!
  asynInitOk_ = true;
  return 0;
}

int ecmcMasterSlaveStateMachine::createAsynParam(const char        *nameFormat,
                                  asynParamType      asynType,
                                  ecmcEcDataType     ecmcType,
                                  uint8_t           *data,
                                  size_t             bytes,
                                  ecmcAsynDataItem **asynParamOut) {
  if (asynPortDriver_ == NULL) {
    LOGERR(
      "%s/%s:%d: ERROR (master/slave state-machine %d): AsynPortDriver object NULL (%s) (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index_,
      nameFormat,
      ERROR_AXIS_ASYN_PORT_OBJ_NULL);
    return ERROR_AXIS_ASYN_PORT_OBJ_NULL;
  }
  *asynParamOut = NULL;
  char  buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char *name                  = NULL;
  unsigned int charCount      = 0;
  ecmcAsynDataItem *paramTemp = NULL;

  charCount = snprintf(buffer,
                       sizeof(buffer),
                       nameFormat,
                       index_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (master/slave state-machine %d): Failed to generate (%s). Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index_,
      nameFormat,
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }
  name      = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                                asynType,
                                                data,
                                                bytes,
                                                ecmcType,
                                                0);

  if (!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (master/slave state-machine %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  *asynParamOut = paramTemp;
  return 0;
}

void ecmcMasterSlaveStateMachine::refreshAsyn() {
  asynStatus_->refreshParamRT(0);
  asynEnable_->refreshParamRT(0);
  asynState_->refreshParamRT(0);
}
