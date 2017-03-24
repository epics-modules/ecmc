#include "ecmcDriveDS402.hpp"

ecmcDriveDS402::ecmcDriveDS402(ecmcAxisData *axisData) : ecmcDriveBase(axisData)
{
  PRINT_ERROR_PATH("axis[%d].drive.error",axisData->axisId_);
  data_=axisData;

  initVars();
  if(!data_){
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n",__FILE__,__FUNCTION__,__LINE__);
    exit(EXIT_FAILURE);
  }

  LOGINFO15("%s/%s:%d: axis[%d].drive=new;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_);
  printCurrentState();
}

ecmcDriveDS402::ecmcDriveDS402(ecmcAxisData *axisData,double scale) : ecmcDriveBase(axisData)
{
  PRINT_ERROR_PATH("axis[%d].drive.error",axisData->axisId_);
  data_=axisData;
  initVars();
  if(!data_){
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n",__FILE__,__FUNCTION__,__LINE__);
    exit(EXIT_FAILURE);
  }

  scale_=scale;
  LOGINFO15("%s/%s:%d: axis[%d].drive=new;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_);
  printCurrentState();
}
ecmcDriveDS402::~ecmcDriveDS402()
{
  ;
}

void ecmcDriveDS402::printCurrentState()
{
  LOGINFO15("%s/%s:%d: axis[%d].drive.type=ECMC_DS402;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_);
  LOGINFO15("%s/%s:%d: axis[%d].drive.scale=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,scale_);
  LOGINFO15("%s/%s:%d: axis[%d].drive.enabled=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,data_->status_.enabled>0);

  switch(enableStateMachine_){
    case ECMC_DS402_RESET_STATE:
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_RESET_STATE");
      break;
    case ECMC_DS402_SWITCH_ON_DISABLED_STATE:
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_SWITCH_ON_DISABLED_STATE");
      break;
    case ECMC_DS402_READY_TO_SWITCH_ON_STATE:
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_READY_TO_SWITCH_ON_STATE");
      break;
    case ECMC_DS402_SWITCHED_ON_STATE:
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_SWITCHED_ON_STATE");
      break;
    case ECMC_DS402_OPERATION_ENABLED_STATE:
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_OPERATION_ENABLED_STATE");
      break;
    default:
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,enableStateMachine_);
      break;
   }
   enableCmdOld_=data_->command_.enable;
   return;
}


void ecmcDriveDS402::initVars()
{
  //ecmcDriveBase::initVars();
  driveState_=ECMC_DS402_INVALID_STATE_STATUS;
  LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_INVALID_STATE_STATUS");
  enableCmdOld_=false;
  enableSequenceRunning_=false;
  enableStateMachine_=ECMC_DS402_RESET_STATE;
}

int ecmcDriveDS402::validate()
{

  int errorCode=ecmcDriveBase::validate();
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  int bitCount=0;  //DS402 must have 16 bit control word
  getEntryBitCount(ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD,&bitCount);
  if(bitCount!=16){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DRV_DS402_CONTROL_WORD_BIT_COUNT_ERROR);
  }

  int startBit=0;  //DS402 must use all bits in word
  getEntryStartBit(ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD,&startBit);
  if(startBit!=-1){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DRV_DS402_CONTROL_WORD_START_BIT_ERROR);
  }

  bitCount=0;  //DS402 must have 16 bit status word
  getEntryBitCount(ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD,&bitCount);
  if(bitCount!=16){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DRV_DS402_STATUS_WORD_BIT_COUNT_ERROR);
  }

  startBit=0;  //DS402 must use all bits in word
  getEntryStartBit(ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD,&startBit);
  if(startBit!=-1){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DRV_DS402_STATUS_WORD_START_BIT_ERROR);
  }

  return 0;
}

void ecmcDriveDS402::writeEntries()
{
  ecmcDriveBase::writeEntries();  //All not drive specific I/O
  return;
}

void ecmcDriveDS402::readEntries()
{
  ecmcDriveBase::readEntries();

  checkDS402State();

  switch(data_->command_.operationModeCmd){
    case ECMC_MODE_OP_AUTO:
      if(getError() || !data_->command_.enable){
	controlWord_=0;
        enableCmdOld_=data_->command_.enable;
	return;
      }
      break;
    case ECMC_MODE_OP_MAN:
      if(getError() || !manualModeEnable_){
	controlWord_=0;
        enableCmdOld_=manualModeEnable_;
	return;
      }
      break;
  }

  if(cycleCounter>ERROR_DRV_DS402_STATE_MACHINE_TIME_OUT_TIME)
  {
    enableSequenceRunning_=false;
    controlWord_=0;
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DRV_DS402_STATE_MACHINE_TIME_OUT);
    return;
  }

  driveStateOld_=driveState_;
  enableStateMachineOld_=enableStateMachine_;

  bool inManualMode=data_->command_.operationModeCmd==ECMC_MODE_OP_MAN;

  if((data_->command_.enable || (manualModeEnable_ && inManualMode)) && !enableCmdOld_){ //Trigger new power on sequence
    enableSequenceRunning_=true;
    enableStateMachine_=ECMC_DS402_RESET_STATE;
    controlWord_=128;
    cycleCounter=0;
  }

  if(enableSequenceRunning_)
  {
    switch(enableStateMachine_)
    {
      cycleCounter++;
      case ECMC_DS402_RESET_STATE:
 	controlWord_=128;
        if(driveState_==ECMC_DS402_SWITCH_ON_DISABLED_STATUS){
          cycleCounter=0;
          enableStateMachine_=ECMC_DS402_SWITCH_ON_DISABLED_STATE;
        }
 	break;
      case ECMC_DS402_SWITCH_ON_DISABLED_STATE:
        controlWord_=6;
        if(driveState_==ECMC_DS402_READY_TO_SWITCH_ON_STATUS){
          cycleCounter=0;
          enableStateMachine_=ECMC_DS402_READY_TO_SWITCH_ON_STATE;
        }
 	break;
      case ECMC_DS402_READY_TO_SWITCH_ON_STATE:
 	controlWord_=7;
        if(driveState_==ECMC_DS402_SWITCHED_ON_STATUS){
          cycleCounter=0;
          enableStateMachine_=ECMC_DS402_SWITCHED_ON_STATE;
        }
 	break;
      case ECMC_DS402_SWITCHED_ON_STATE:
 	controlWord_=15;
        if(driveState_==ECMC_DS402_OPERATION_ENABLED_STATUS){
          cycleCounter=0;
          enableStateMachine_=ECMC_DS402_OPERATION_ENABLED_STATE;
        }
 	break;
      case ECMC_DS402_OPERATION_ENABLED_STATE:
        cycleCounter=0;
        enableSequenceRunning_=false; //startup sequence ready
	break;
     }
   }

   enableCmdOld_=data_->command_.enable;
  return;
}

int ecmcDriveDS402::checkDS402State()
{
  int driveStateOld=driveState_;
  driveState_=ECMC_DS402_INVALID_STATE_STATUS;

  bool enabledOld=data_->status_.enabled;
  if((statusWord_ & ECMC_DS402_STATUS_MASK_1)==ECMC_DS402_NOT_READY_TO_SWITCH_ON_STATUS)
  {
    driveState_=ECMC_DS402_NOT_READY_TO_SWITCH_ON_STATUS;
    data_->status_.enabled=false;
    if(enabledOld!=data_->status_.enabled){
      LOGINFO15("%s/%s:%d: axis[%d].drive.enabled=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,data_->status_.enabled>0);
    }
    if(driveStateOld!=driveState_){
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_NOT_READY_TO_SWITCH_ON_STATUS");
    }
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_1)==ECMC_DS402_SWITCH_ON_DISABLED_STATUS)
  {
    driveState_=ECMC_DS402_SWITCH_ON_DISABLED_STATUS;
    data_->status_.enabled=false;
    if(enabledOld!=data_->status_.enabled){
      LOGINFO15("%s/%s:%d: axis[%d].drive.enabled=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,data_->status_.enabled>0);
    }
    if(driveStateOld!=driveState_){
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_SWITCH_ON_DISABLED_STATUS");
    }
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_2)==ECMC_DS402_READY_TO_SWITCH_ON_STATUS)
  {
    driveState_=ECMC_DS402_READY_TO_SWITCH_ON_STATUS;
    data_->status_.enabled=false;
    if(enabledOld!=data_->status_.enabled){
      LOGINFO15("%s/%s:%d: axis[%d].drive.enabled=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,data_->status_.enabled>0);
    }
    if(driveStateOld!=driveState_){
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_READY_TO_SWITCH_ON_STATUS");
    }
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_2)==ECMC_DS402_SWITCHED_ON_STATUS)
  {
    driveState_=ECMC_DS402_SWITCHED_ON_STATUS;
    data_->status_.enabled=false;
    if(enabledOld!=data_->status_.enabled){
      LOGINFO15("%s/%s:%d: axis[%d].drive.enabled=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,data_->status_.enabled>0);
    }
    if(driveStateOld!=driveState_){
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_SWITCHED_ON_STATUS");
    }
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_2)==ECMC_DS402_OPERATION_ENABLED_STATUS)
  {
    driveState_=ECMC_DS402_OPERATION_ENABLED_STATUS;
    data_->status_.enabled=true;
    if(enabledOld!=data_->status_.enabled){
      LOGINFO15("%s/%s:%d: axis[%d].drive.enabled=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,data_->status_.enabled>0);
    }
    if(driveStateOld!=driveState_){
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_OPERATION_ENABLED_STATUS");
    }
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_2)==ECMC_DS402_QUICK_STOP_ACTIVE_STATUS)
  {
    driveState_=ECMC_DS402_QUICK_STOP_ACTIVE_STATUS;
    data_->status_.enabled=false;
    if(enabledOld!=data_->status_.enabled){
      LOGINFO15("%s/%s:%d: axis[%d].drive.enabled=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,data_->status_.enabled>0);
    }
    if(driveStateOld!=driveState_){
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_QUICK_STOP_ACTIVE_STATUS");
    }
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_2)==ECMC_DS402_FAULT_REACTION_ACTIVE_STATUS)
  {
    driveState_=ECMC_DS402_FAULT_REACTION_ACTIVE_STATUS;
    data_->status_.enabled=false;
    if(enabledOld!=data_->status_.enabled){
      LOGINFO15("%s/%s:%d: axis[%d].drive.enabled=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,data_->status_.enabled>0);
    }
    if(driveStateOld!=driveState_){
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_FAULT_REACTION_ACTIVE_STATUS");
    }
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_1)==ECMC_DS402_FAULT_REACTION_ACTIVE_STATUS)
  {
    driveState_=ECMC_DS402_FAULT_REACTION_ACTIVE_STATUS;
    data_->status_.enabled=false;
    if(enabledOld!=data_->status_.enabled){
      LOGINFO15("%s/%s:%d: axis[%d].drive.enabled=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,data_->status_.enabled>0);
    }
    if(driveStateOld!=driveState_){
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_FAULT_REACTION_ACTIVE_STATUS");
    }
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_1)==ECMC_DS402_FAULT_STATUS)
  {
    driveState_=ECMC_DS402_FAULT_STATUS;
    data_->status_.enabled=false;
    if(enabledOld!=data_->status_.enabled){
      LOGINFO15("%s/%s:%d: axis[%d].drive.enabled=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,data_->status_.enabled>0);
    }
    if(driveStateOld!=driveState_){
      LOGINFO15("%s/%s:%d: axis[%d].drive.state=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_DS402_FAULT_STATUS");
    }
    return 0;
  }
  return 0;
}
