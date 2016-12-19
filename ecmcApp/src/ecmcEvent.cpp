/*
 * ecmcDataRecorder.cpp
 *
 *  Created on: May 27, 2016
 *      Author: anderssandstrom
 */

#include "ecmcEvent.h"

ecmcEvent::ecmcEvent(double sampleTime, int index): ecmcEcEntryLink()
{
  initVars();
  sampleTime_=sampleTime;
  index_=index;
}

ecmcEvent::~ecmcEvent()
{

}

void ecmcEvent::initVars()
{
  execute_=false;
  eventType_=ECMC_SAMPLED;
  sampleTime_=1;
  triggerOld_=0;
  trigger_=0;
  dataSampleTime_=1;
  triggerEdge_=ECMC_POSITIVE_EDGE;
  dataSampleTimeCounter_=0;
  enableArmSequence_=0;
  armState_=0;
  armStateCounter_=0;
  setInStartupPhase(1);
  armed_=0;
  eventTriggered_=0;
  for(int i=0;i<ECMC_MAX_EVENT_CONSUMERS;i++){
    consumers_[i]=NULL;
  }
  reArm_=getError();
}

int ecmcEvent::setEventType(eventType type)
{
  eventType_=type;
  int errorCode=validate();
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  return 0;
}

int ecmcEvent::setTriggerEdge(triggerEdgeType triggerEdge)
{
  triggerEdge_=triggerEdge;
  return 0;
}

int ecmcEvent::setExecute(int execute)
{
  if(!execute_  && execute){
    int errorCode=validate();
    if(errorCode){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
    }
  }
  execute_=execute;
  return 0;
}

int ecmcEvent::setDataSampleTime(int sampleTime)
{
  if(sampleTime<=0){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EVENT_INVALID_SAMPLE_TIME);
  }
  dataSampleTime_=sampleTime;
  return 0;
}

int ecmcEvent::execute(int masterOK)
{
  if(!masterOK){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EVENT_HARDWARE_STATUS_NOT_OK);
  }

  if(inStartupPhase_){
    //Auto reset hardware error
    if(getErrorID()==ERROR_EVENT_HARDWARE_STATUS_NOT_OK){
      setErrorID(0);
    }
    armSequence();
    setInStartupPhase(false);
  }

  if(getError()){
    reArm_=true;  //re-arm if error is reset
    return getErrorID();
  }

  if(!execute_){
    return 0;
  }

  if ((eventTriggered_ || armState_ || reArm_) && enableArmSequence_ ){
    reArm_=false;
    int errorCode=armSequence();
    if(errorCode>0){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
    }
  }

  eventTriggered_=0;

  if(eventType_==ECMC_SAMPLED){  //trigger entry NOT needed
    dataSampleTimeCounter_++;
    if(dataSampleTimeCounter_==dataSampleTime_){
      dataSampleTimeCounter_=0;
      eventTriggered_=true;
      int errorCode=callConsumers(masterOK);
      if(errorCode){
	return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
      }
      printStatus();
    }
  }

  if(eventType_!=ECMC_SAMPLED){  //trigger entry needed
    uint64_t tempRaw=0;
    if(readEcEntryValue(0,&tempRaw)){//Trigger
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EVENT_ECENTRY_READ_FAIL);
    }
    triggerOld_=trigger_;
    trigger_=tempRaw;

    switch(triggerEdge_){
      case ECMC_POSITIVE_EDGE:
	if(trigger_>triggerOld_){
          eventTriggered_=true;
          int errorCode=callConsumers(masterOK);
	  if(errorCode){
	    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
          }
	  printStatus();
	}
	break;
      case ECMC_NEGATIVE_EDGE:
	if(trigger_<triggerOld_){
          eventTriggered_=true;
          int errorCode=callConsumers(masterOK);
          if(errorCode){
            return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
          }
          printStatus();
	}
	break;
      case ECMC_ON_CHANGE:
	if(trigger_!=triggerOld_){
          eventTriggered_=true;
          int errorCode=callConsumers(masterOK);
          if(errorCode){
            return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
          }
          printStatus();
	}
	break;
    }
  }

  return 0;
}

int ecmcEvent::validate()
{
  if(validateEntry(0)){ //Trigger entry
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EVENT_TRIGGER_ECENTRY_NULL);
  }

  if(enableArmSequence_){
    if(validateEntry(1)){ //Arm entry
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EVENT_ARM_ECENTRY_NULL);
    }
  }
  return setErrorID(0);
}

int ecmcEvent::armSequence()
{
  armed_=0;
  int errorCode=0;
  //Used for I/O that need toogle in order to rearm latch
  switch(armState_){
    case 0:
      armState_=1;
      return -armState_;
      break;
    case 1:
      LOGINFO10("%s/%s:%d: INFO: Event %d. Arming sequence in state %d.\n",__FILE__, __FUNCTION__, __LINE__,index_,armState_);
      errorCode=writeEcEntryValue(1,armed_); //Write 0
      if(errorCode){
	  return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
      }
      armStateCounter_++;
      if(armStateCounter_>1){
        armState_=2;
        armStateCounter_=0;
      }
      return -armState_;
      break;
    case 2:
      LOGINFO10("%s/%s:%d: INFO: Event %d. Arming sequence in state %d.\n",__FILE__, __FUNCTION__, __LINE__,index_,armState_);
      armed_=1;
      errorCode=writeEcEntryValue(1,armed_); //Write 1
      if(errorCode){
	  return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
      }

      armState_=0;
      return -armState_;
      break;
  }

  return 0;
}

int ecmcEvent::setEnableArmSequence(int enable)
{
  enableArmSequence_=enable;
  validate();
  return 0;
}

void ecmcEvent::setInStartupPhase(bool startup)
{
  inStartupPhase_=startup;
}

void ecmcEvent::printStatus()
{
  printf("New event triggered! Index: %d, Armed: %d, enableArmSequence_: %d, Error: %x\n",index_,armed_,enableArmSequence_,getErrorID());
}

int ecmcEvent::setEnablePrintOuts(bool enable)
{
  enableDiagnosticPrintouts_=enable;
  return 0;
}

int ecmcEvent::callConsumers(int masterOK)
{
  int errorCode=0;
  for(int i=0;i<ECMC_MAX_EVENT_CONSUMERS;i++){
    if(consumers_[i]!=NULL){
      LOGINFO10("%s/%s:%d: INFO: Event %d. Calling consumer %d.\n",__FILE__, __FUNCTION__, __LINE__,index_,i);
      errorCode=consumers_[i]->executeEvent(masterOK);
      if(errorCode){
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
      }
    }
  }
  return 0;
}

int ecmcEvent::linkEventConsumer(ecmcEventConsumer *consumer,int index)
{
  if(index<0 || index>=ECMC_MAX_EVENT_CONSUMERS){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EVENT_CONSUMER_INDEX_OUT_OF_RANGE);
  }

  consumers_[index]=consumer;
  return 0;
}

int ecmcEvent::triggerEvent(int masterOK)
{
  return callConsumers(masterOK);
}

int ecmcEvent::arm()
{
  if(!enableArmSequence_){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EVENT_ARM_NOT_ENABLED);
  }
  armSequence();
  return 0;
}
