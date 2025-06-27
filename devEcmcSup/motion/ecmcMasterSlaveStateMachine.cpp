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
                                                         ecmcAxisGroup *slaveGrp,
                                                         int autoDisbleMasters,
                                                         int autoDisbleSlaves){
  asynPortDriver_           = asynPortDriver;
  asynControl_              = NULL;
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
  status_                   = 0;
  state_                    = ECMC_MST_SLV_STATE_IDLE;    

  memset(&control_,0,sizeof(control_));

  control_.enable             = 1;
  control_.autoDisableMasters = autoDisbleMasters;
  control_.autoDisableSlaves  = autoDisbleSlaves;
  control_.enableDbgPrintouts = false;
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

  if(!control_.enable) {
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
    case ECMC_MST_SLV_STATE_RESET:
      stateReset();
      break;

  };
};

int ecmcMasterSlaveStateMachine::stateIdle(){
  // optimize
  bool anySlaveBusy     = slaveGrp_->getAnyBusy();
  bool anySlaveErrorId  = slaveGrp_->getAnyErrorId();
  bool anyMasterEnabled = masterGrp_->getAnyEnabled();

  // State transision to SLAVE
  if( anySlaveBusy && 
    !slaveGrp_->getTrajSrcAnyExt() && 
    !anyMasterEnabled && 
    !masterGrp_->getAnyEnable()) {

    state_ = ECMC_MST_SLV_STATE_SLAVES;
    if(control_.enableDbgPrintouts) {
      printf("ecmcMasterSlaveStateMachine: %s: State change, IDLE -> SLAVE\n",name_.c_str());
    }

  // State transision to MASTER
  } else if(anyMasterEnabled &&
            anySlaveErrorId == 0 ) {

    if(slaveGrp_->getEnabled() && anyMasterEnabled && !anySlaveBusy){

      if(masterGrp_->getAnyBusy()) {
        slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_EXTERNAL);
        state_ = ECMC_MST_SLV_STATE_MASTERS;
        if(control_.enableDbgPrintouts) {
          printf("ecmcMasterSlaveStateMachine: %s: State change, IDLE -> MASTER\n", name_.c_str());
        }
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
        if(control_.enableDbgPrintouts) {
          printf("ecmcMasterSlaveStateMachine: %s: Error enabling axes\n", name_.c_str());
          printf("ecmcMasterSlaveStateMachine: %s: State change, IDLE -> IDLE\n", name_.c_str());
        }
        state_ = ECMC_MST_SLV_STATE_IDLE;
      }
      if(anySlaveBusy) {
        slaveGrp_->halt();
      }
    }
  } else if(anySlaveErrorId > 0){
    masterGrp_->setSlavedAxisInError();
  }

  return 0;
}

int ecmcMasterSlaveStateMachine::stateSlave(){

  if(!slaveGrp_->getAnyBusy()) {

    if(control_.autoDisableSlaves) {
      slaveGrp_->setEnable(0);
      slaveGrp_->setMRCnen(0);
    }
    
    if(control_.autoDisableMasters) {      
      masterGrp_->setEnable(0);
      masterGrp_->setMRCnen(0);
    }

    slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);

    // Sync the master axes
    masterGrp_->setMRSync(1);
    masterGrp_->setMRStop(1);

    state_ = ECMC_MST_SLV_STATE_IDLE;
    if(control_.enableDbgPrintouts) {
      printf("ecmcMasterSlaveStateMachine: %s: State change, SLAVE -> IDLE\n", name_.c_str());
    }
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
    if(control_.autoDisableMasters) {
      slaveGrp_->setEnable(0);
      masterGrp_->setEnable(0);
      slaveGrp_->setMRCnen(0);
      masterGrp_->setMRCnen(0);
    }
  }

  // One master axis gets killed during motion then kill all and goto IDLE
  if( (masterGrp_->getAnyEnabled() && !masterGrp_->getEnabled()) || 
      (slaveGrp_->getAnyEnabled() && !slaveGrp_->getEnabled())) {   
    state_ = ECMC_MST_SLV_STATE_RESET;
    if(control_.enableDbgPrintouts) {
      printf("ecmcMasterSlaveStateMachine: %s: At least one axis lost enable during motion. Disable all axes..\n", name_.c_str());    
      printf("ecmcMasterSlaveStateMachine: %s: State change, MASTER -> RESET\n", name_.c_str());
    }
    stateReset(); // A bit nasty but ....
    return 0;
  }

  // postpone disable until all master axes are done
  masterGrp_->setEnableAutoDisable(masterGrp_->getAnyBusy() == 0);
  // ensure attarget/reduced current of slave axes
  slaveGrp_->setAxisIsWithinCtrlDBExtTraj(masterGrp_->getAxisIsWithinCtrlDB());
  
  // Ilock or if any slaved axis is changing to internal source
  if(slaveGrp_->getAnyIlocked() || !slaveGrp_->getTrajSrcExt()){
    slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);
    slaveGrp_->setMRSync(1);
    slaveGrp_->setMRStop(1);
    slaveGrp_->halt();
    masterGrp_->setSlavedAxisIlocked();
    masterGrp_->setEnableAutoDisable(1);
    state_ = ECMC_MST_SLV_STATE_SLAVES;
    if(control_.enableDbgPrintouts) {
      printf("ecmcMasterSlaveStateMachine: %s: Slaved axis interlock (or traj source change)\n", name_.c_str());
      printf("ecmcMasterSlaveStateMachine: %s: State change, MASTER -> SLAVE\n", name_.c_str());
    }
  }
  
  // Done?
  if(!masterGrp_->getAnyEnabled()) {
    slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);
    slaveGrp_->setErrorReset();
    slaveGrp_->setEnable(0);
    slaveGrp_->setMRCnen(0);
    masterGrp_->setEnableAutoDisable(1);
    state_ = ECMC_MST_SLV_STATE_IDLE;
    if(control_.enableDbgPrintouts) {
      printf("ecmcMasterSlaveStateMachine: %s: State change, MASTER -> IDLE\n", name_.c_str());
    }
  }
  return 0;
}

int ecmcMasterSlaveStateMachine::stateReset() {
  slaveGrp_->setEnable(0);
  masterGrp_->setEnable(0);
  slaveGrp_->setMRCnen(0);
  masterGrp_->setMRCnen(0);
  slaveGrp_->setErrorReset();
  masterGrp_->setErrorReset();
  slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);
  masterGrp_->setEnableAutoDisable(1);
  state_ = ECMC_MST_SLV_STATE_IDLE;
  if(control_.enableDbgPrintouts) {
    printf("ecmcMasterSlaveStateMachine: %s: State change, RESET -> IDLE\n", name_.c_str());
  }
  return  0;
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

  // Control
  errorCode = createAsynParam(ECMC_MST_SLV_OBJ_STR "%d.control",
                              asynParamInt32,
                              ECMC_EC_U32,
                              (uint8_t *)&(control_),
                              sizeof(control_),
                              &paramTemp);

  if (errorCode) {
    printf("Error creating asyn parameter for control word\n");
    return errorCode;
  }

  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);
  paramTemp->refreshParam(1);
  asynControl_ = paramTemp;

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
  asynControl_->refreshParamRT(0);
  asynState_->refreshParamRT(0);
}
