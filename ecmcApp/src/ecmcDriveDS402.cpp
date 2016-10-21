#include "ecmcDriveDS402.hpp"

ecmcDriveDS402::ecmcDriveDS402()
{
  initVars();
}

ecmcDriveDS402::ecmcDriveDS402(double scale)
{
  initVars();
  scale_=scale;
}
ecmcDriveDS402::~ecmcDriveDS402()
{
  ;
}

void ecmcDriveDS402::initVars()
{
  ecmcDriveBase::initVars();
  driveState_=ECMC_DS402_INVALID_STATE_STATUS;
  enableCmdOld_=false;
  enableSequenceRunning_=false;
  enableStateMachine_=ECMC_DS402_RESET_STATE;
}

int ecmcDriveDS402::setEnable(bool enable)
{
  if(interlock_ && enable){
    enableCmd_=false;
    return setErrorID(ERROR_DRV_DRIVE_INTERLOCKED);
  }

  if(enableBrake_){
    if(!enabledStatus_ ){
      brakeOutput_=0;  //brake locked when 0 . TODO: Apply brake some cycles before enable is low
    }
    else{
      brakeOutput_=1;  //brake open when 1
    }
  }

  enableCmd_=enable;
  return 0;
}

int ecmcDriveDS402::validate()
{
  int errorCode=ecmcDriveBase::validate();
  if(errorCode){
    return setErrorID(errorCode);
  }

  int bitCount=0;  //DS402 must have 16 bit control word
  getEntryBitCount(ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD,&bitCount);
  if(bitCount!=16){
    return setErrorID(ERROR_DRV_DS402_CONTROL_WORD_BIT_COUNT_ERROR);
  }

  bitCount=0;  //DS402 must have 16 bit status word
  getEntryBitCount(ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD,&bitCount);
  if(bitCount!=16){
    return setErrorID(ERROR_DRV_DS402_STATUS_WORD_BIT_COUNT_ERROR);
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
  if(enableCmd_ && ! enableCmdOld_){ //Trigger new power on sequence
    enableSequenceRunning_=true;
    enableStateMachine_=ECMC_DS402_RESET_STATE;
  }

  if(enableSequenceRunning_)
  {
    switch(enableStateMachine_)
    {
      case ECMC_DS402_RESET_STATE:
 	controlWord_=128;
        if(statusWord_==ECMC_DS402_SWITCH_ON_DISABLED_STATUS){
          enableStateMachine_=ECMC_DS402_SWITCH_ON_DISABLED_STATE;
        }
 	break;
      case ECMC_DS402_SWITCH_ON_DISABLED_STATE:
        controlWord_=6;
        if(statusWord_==ECMC_DS402_READY_TO_SWITCH_ON_STATUS){
          enableStateMachine_=ECMC_DS402_READY_TO_SWITCH_ON_STATE;
        }
 	break;
      case ECMC_DS402_READY_TO_SWITCH_ON_STATE:
 	controlWord_=7;
        if(statusWord_==ECMC_DS402_SWITCHED_ON_STATE){
          enableStateMachine_=ECMC_DS402_SWITCHED_ON_STATE;
        }
 	break;
      case ECMC_DS402_SWITCHED_ON_STATE:
 	controlWord_=15;
        if(statusWord_==ECMC_DS402_OPERATION_ENABLED_STATE){
          enableStateMachine_=ECMC_DS402_OPERATION_ENABLED_STATE;
        }
 	break;
      case ECMC_DS402_OPERATION_ENABLED_STATE:
        enableSequenceRunning_=false; //startup sequence ready
	break;
     }
   }

   enableCmdOld_=enableCmd_;
  return;
}

int ecmcDriveDS402::checkDS402State()
{
  driveState_=ECMC_DS402_INVALID_STATE_STATUS;
  enabledStatus_=false;

  if((statusWord_ & ECMC_DS402_STATUS_MASK_1)==ECMC_DS402_NOT_READY_TO_SWITCH_ON_STATUS)
  {
    driveState_=ECMC_DS402_NOT_READY_TO_SWITCH_ON_STATUS;
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_1)==ECMC_DS402_SWITCH_ON_DISABLED_STATUS)
  {
    driveState_=ECMC_DS402_SWITCH_ON_DISABLED_STATUS;
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_2)==ECMC_DS402_READY_TO_SWITCH_ON_STATUS)
  {
    driveState_=ECMC_DS402_READY_TO_SWITCH_ON_STATUS;
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_2)==ECMC_DS402_SWITCHED_ON_STATUS)
  {
    driveState_=ECMC_DS402_SWITCHED_ON_STATUS;
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_2)==ECMC_DS402_OPERATION_ENABLED_STATUS)
  {
    driveState_=ECMC_DS402_OPERATION_ENABLED_STATUS;
    enabledStatus_=true;
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_2)==ECMC_DS402_QUICK_STOP_ACTIVE_STATUS)
  {
    driveState_=ECMC_DS402_QUICK_STOP_ACTIVE_STATUS;
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_2)==ECMC_DS402_FAULT_REACTION_ACTIVE_STATUS)
  {
    driveState_=ECMC_DS402_FAULT_REACTION_ACTIVE_STATUS;
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_1)==ECMC_DS402_FAULT_REACTION_ACTIVE_STATUS)
  {
    driveState_=ECMC_DS402_FAULT_REACTION_ACTIVE_STATUS;
    return 0;
  }

  if((statusWord_ & ECMC_DS402_STATUS_MASK_1)==ECMC_DS402_FAULT_STATUS)
  {
    driveState_=ECMC_DS402_FAULT_STATUS;
    return 0;
  }

  return 0;
}
