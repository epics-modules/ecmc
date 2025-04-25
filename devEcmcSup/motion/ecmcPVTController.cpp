/*************************************************************************\
* Copyright (c) 2024 Paul Scherrer Institut
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcPVTController.cpp
*
*  Created on: September 04, 2024
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcPVTController.h"

ecmcPVTController::ecmcPVTController(double sampleTime) {
  sampleTime_     = sampleTime;
  nextTime_       = 0;
  accTime_        = 0;
  endTime_        = 0;
  executeOld_     = 0;
  execute_        = 0;
  state_          = ECMC_PVT_IDLE;
  busy_           = 0;
  startPositions_.reserve(ECMC_MAX_AXES);
  axes_.reserve(ECMC_MAX_AXES);
  triggerDefined_ = 0;
  triggerEcEntryIndex_ = 0;
  triggerStartPoint_   = 0;
  triggerEndPoint_     = 0;
  triggerCount_        = 0;
  triggerStartTime_    = 0;
  triggerEndTime_      = 0;
  triggerTimeBetween_  = 0;
  triggerValidatedOK_  = false;
  triggerCurrentId_    = 0;
  setTriggerDuration(0.1);
  clearPVTAxes();
}

ecmcPVTController::~ecmcPVTController() {
}

void ecmcPVTController::addAxis(ecmcAxisBase* axis) {
  axes_.push_back(axis);
  if(axis) {
    printf("ecmcPVTController::addAxis(%d)\n",axis->getAxisID());
  } else {
    printf("ecmcPVTController::addAxis(NaN)\n");
  }
}

void ecmcPVTController::clearPVTAxes() {
  // make sure global PVT busy is not high
  setAxesBusy(false);
  axes_.clear();
  startPositions_.clear();
}

size_t ecmcPVTController::getCurrentPointId() {
  return 0;  
}

size_t ecmcPVTController::getCurrentTriggerId() {
  return triggerCurrentId_;
}

double ecmcPVTController::getCurrentTime() {
  return nextTime_;
}

void ecmcPVTController::checkIfTimeToTrigger() {

}

void ecmcPVTController::setExecute(bool execute) {
  executeOld_ = execute_;
  execute_ = execute;
  int error = 0;
  if(execute_ && !executeOld_ && axes_[0]) {
    error=validate();
    if(error) {
      setErrorID(error);
      execute_ = 0;
      executeOld_ = 0;
      return;
    }
    state_ = ECMC_PVT_ENABLE_AXES;
    busy_ = 1;
    setAxesBusy(true);
  }
  if(!execute) {
    if(busy_) {
      abortPVT();
    }
  }
}

// Execute by ecmc RT
void ecmcPVTController::execute() {
  int error = 0;
  int axesAtStartPosition = 0;
  bool seqDone = 0;
  int enabledState = 0;
  switch(state_) {
    case  ECMC_PVT_IDLE:
      busy_ = 0;
      setAxesBusy(false);
      break;
    case ECMC_PVT_ENABLE_AXES:
      
      if(setEnable(1) < 0) {
        state_ = ECMC_PVT_ERROR;
        printf("ecmcPVTController::execute(): Error: Enabling axes failed\n");
      }
      enabledState = checkEnabledState(1);
      if(enabledState < 0) {
        state_ = ECMC_PVT_ERROR;
        printf("ecmcPVTController::execute(): Error: Enabling axes failed\n");
      } else if(enabledState == 1) {
        printf("ecmcPVTController::execute(): All axes enabled.\n");
        state_ = ECMC_PVT_TRIGG_MOVE_AXES_TO_START;
      }
      break;
    case ECMC_PVT_TRIGG_MOVE_AXES_TO_START:
      initPVT();  // prepare pvt objects
      error = triggMoveAxesToStart();
      if(error) {
        setErrorID(error);
        state_ = ECMC_PVT_ERROR;
        printf("ecmcPVTController::execute(): Error: Axis in error state when trigger move to start position\n");
      }
      setAxesBusy(true);
      state_ = ECMC_PVT_WAIT_FOR_AXES_TO_REACH_START;
      break;

    case  ECMC_PVT_WAIT_FOR_AXES_TO_REACH_START:
      axesAtStartPosition = axesAtStart();
      if(axesAtStartPosition < 0) {
        setErrorID(-axesAtStartPosition);
        state_ = ECMC_PVT_ERROR;
        printf("ecmcPVTController::execute(): Error: Axis in error state when moving to start position\n");
      }
      if(axesAtStartPosition > 0 ) {
        state_ = ECMC_PVT_TRIGG_PVT;
        nextTime_ = -sampleTime_; // Start at -1 sample
        // Set time to 0 in all PVT objects
        for(uint i = 0; i < axes_.size(); i++ ) {      
          axes_[i]->getPVTObject()->setNextTime(0);
          axes_[i]->getPVTObject()->setCurrTime(0);
        }    
        // get end time from first axis
        endTime_ = axes_[0]->getPVTObject()->endTime();
        // get acc time from first axis
        accTime_ = axes_[0]->getPVTObject()->getSegDuration(0);
        //printf("ecmcPVTController: All axes in position, trigger PVT sequence\n");
      }
      break;

    case ECMC_PVT_TRIGG_PVT:
      error =  triggPVT();  
      if(error){
        setErrorID(error);
        state_ = ECMC_PVT_ERROR;
        printf("ecmcPVTController::execute(): Error: Triggering of PVT objects failed\n");
      }
      //printf("ecmcPVTController: Executing PVT sequence\n");
      state_ = ECMC_PVT_EXECUTE_PVT;      
      break;

    case ECMC_PVT_EXECUTE_PVT:

      if(anyAxisInterlocked()) {
        abortPVT();
      }
      // Increase time
      nextTime_ = nextTime_ + sampleTime_;
  
      if(nextTime_> endTime_) {    
        nextTime_ = endTime_;
        seqDone = true;
      }

      for(uint i = 0; i < axes_.size(); i++ ) {
        if(axes_[i]->getPVTObject()->getBusy()) {
          axes_[i]->getPVTObject()->setNextTime(nextTime_);          
          if(seqDone) {
            axes_[i]->getPVTObject()->setBusy(false);
            state_ =  ECMC_PVT_IDLE;            
          }
        }
      }

      break;

    case ECMC_PVT_ABORT:

      // Wait for axes to stop  
      if(axisNotBusy()) {
        //printf("ecmcPVTController: All axes stopped\n");
        state_ =  ECMC_PVT_IDLE;
      }

      for(uint i = 0; i < axes_.size(); i++ ) {
        axes_[i]->getPVTObject()->setBusy(false);
        axes_[i]->setGlobalBusy(0);
        state_ =  ECMC_PVT_IDLE;        
      }

      break;
    case ECMC_PVT_ERROR:
      busy_ = 0;
      for(uint i = 0; i < axes_.size(); i++ ) {
        axes_[i]->getPVTObject()->setBusy(false);
        state_ =  ECMC_PVT_IDLE;        
      }
      setAxesBusy(false);
      break;
  }
  
  // Trigger Control
  if(!triggerValidatedOK_) {  // Triggers not in use    
    return;
  }

  if(nextTime_ > triggerEndTime_+ triggerDuration_) {
    writeEcEntryValue(triggerEcEntryIndex_,0);
    return; // Done
  }

  double nextTriggerTime = triggerStartTime_+ triggerCurrentId_ * triggerTimeBetween_;

  if(triggerCurrentId_ < triggerCount_) {
    if(nextTime_ > (nextTriggerTime + triggerTimeBetween_)) {
      triggerCurrentId_ = triggerCurrentId_ + 1;
    }
  }

  if( nextTime_ >= nextTriggerTime && nextTime_ <= (nextTriggerTime + triggerDuration_)) {
    writeEcEntryValue(triggerEcEntryIndex_,1);
  } else {
    writeEcEntryValue(triggerEcEntryIndex_,0);
  }
}

bool  ecmcPVTController::getBusy() {
  return busy_;
}

int ecmcPVTController::triggMoveAxesToStart() {
  startPositions_.clear();
  double startPosition = 0;
  int error = 0;
  startPositions_.clear();
  for(uint i = 0; i < axes_.size(); i++ ) {
    axes_[i]->getPVTObject()->startPosition(&startPosition);        
    if(axes_[i]->getPVTObject()->getRelMode()) {      
      startPosition = axes_[i]->getCurrentPositionSetpoint() + startPosition;
      axes_[i]->getPVTObject()->setPositionOffset(axes_[i]->getCurrentPositionSetpoint());
    } else {    
      axes_[i]->getPVTObject()->setPositionOffset(0);
    }
    error = axes_[i]->moveAbsolutePosition(startPosition);
    if(error) {
      return error;
    }

    // Save all startpositions to be able to verify that the axes arrived
    startPositions_.push_back(startPosition);
  }
  return 0;
}

// negative error
int ecmcPVTController::axesAtStart() {
  int error = 0;
  for(uint i = 0; i < axes_.size(); i++ ) {
    error = axes_[i]->getErrorID();
    if(error) {
      return -error;
    }
    if(axes_[i]->getTrajBusy()) {
      return 0;
    }
    if(axes_[i]->getCurrentPositionSetpoint() != startPositions_[i]) {
      //printf("ecmcPVTController:: target set %lf, actual set %lf\n",axes_[i]->getCurrentPositionSetpoint(), startPositions_[i]);
      return 0;
    }
  }
  // All axes in correct position to start
  return 1;
}

// negative error
int ecmcPVTController::triggPVT() {
  int error = 0;
  for(uint i = 0; i < axes_.size(); i++ ) {    
    error = axes_[i]->getErrorID();    
    if(error) {
      return -error;
    }
    
    // Relative are handled with PVTobject with an offset
    error = axes_[i]->movePVTAbs(true);  // ignore busy.. have checked before
    if(error) {
      return error;
    }
  }
  // All axes in correct position to start
  return 0;
}

void ecmcPVTController::errorReset() {
  ecmcError::errorReset();
  if(state_ == ECMC_PVT_ERROR) {
    state_ = ECMC_PVT_IDLE;
  }
}

int ecmcPVTController::validate() {
  if(axes_.size()== 0) {
    printf("ecmcPVTController::validate(): Error axis count zero\n");
    return setErrorID(ERROR_PVT_CTRL_AXIS_COUNT_ZERO);
  }

  // trigger
  triggerValidatedOK_ = false;
  if(!triggerDefined_ || triggerCount_ <= 0 ) {
    return 0;   
  }

  triggerValidatedOK_ = true;
  return 0; 
}

int ecmcPVTController::anyAxisInterlocked() {
  
  for(uint i = 0; i < axes_.size(); i++ ) {    
    if(axes_[i]->getSumInterlock() || axes_[i]->getErrorID()>0) {
      return 1;
    }
  }
  return 0;
}

int ecmcPVTController::abortPVT() {
  for(uint i = 0; i < axes_.size(); i++ ) {    
    axes_[i]->stopMotion(0);
    axes_[i]->getPVTObject()->setBusy(false);
  }
  state_ = ECMC_PVT_ABORT;
  // All axes in correct position to start
  return 0;
}

int ecmcPVTController::axisNotBusy() {
  int axesFree = 1;
  for(uint i = 0; i < axes_.size(); i++ ) {    
    axesFree = axesFree && !axes_[i]->getTrajBusy();
  }
  // All axes in correct position to start
  return axesFree;
}

void ecmcPVTController::initPVT() {
  for(uint i = 0; i < axes_.size(); i++ ) {    
    axes_[i]->getPVTObject()->setExecute(0);
  }
  triggerCurrentId_ = 0;
}

int ecmcPVTController::setEcEntry(ecmcEcEntry *entry,int entryIndex, int bitIndex) {
  
  int error = setEntryAtIndex(entry,
                              entryIndex,
                              bitIndex);
  if(error) {
    triggerDefined_ = 0;
    return error;
  }
  
  error = validateEntryBit(triggerEcEntryIndex_);
  if(error) {
    return setErrorID(error);
  }
  triggerEcEntryIndex_ = entryIndex;
  triggerDefined_ = 1;
  printf("ecmcPVTController::setEcEntry(): Trigger defined.\n");
  return 0;
}

int ecmcPVTController::checkTriggerTiming() {
  if(axes_.size() <= 0) {
    return setErrorID(ERROR_PVT_CTRL_AXIS_COUNT_ZERO);
  }

  // use timing of first PVT object to calcu trigger info
  ecmcAxisPVTSequence *pvt = axes_[0]->getPVTObject();
  
  if(pvt == NULL) {
    printf("ecmcPVTController::checkTriggerTiming(): PVT obj NULL\n");
    return setErrorID(ERROR_PVT_CTRL_TRIGG_CFG_INVALID);
  }
  
  size_t segCount = pvt->getSegCount();
  // Note first segment is acc and last dec (must be 3 segments)
  if(triggerEndPoint_> (segCount - 1)) {
    triggerEndPoint_ = segCount - 1;
  }
  if((segCount < 3) || (triggerEndPoint_ > (segCount - 1)) || ((triggerStartPoint_ >= triggerEndPoint_) && (triggerCount_ > 1))) {
    printf("ecmcPVTController::checkTriggerTiming(): Invalid trigger params\n");
    return setErrorID(ERROR_PVT_CTRL_TRIGG_CFG_INVALID);
  }

  // find first trigger times
  double timeSum = 0;
  for(size_t i = 0; i <= triggerEndPoint_; i++) {
    timeSum +=pvt->getSegDuration(i);
    if(i == (triggerStartPoint_-1)) {
      triggerStartTime_ = timeSum;
    }
    if(i == (triggerEndPoint_-1)) {
      triggerEndTime_ = timeSum;
    }
  }

  if(triggerCount_<=1) {
    triggerTimeBetween_ = 0;  // Only one trigger
    triggerEndTime_ = triggerStartTime_;
  } else {
    triggerTimeBetween_ = (triggerEndTime_-triggerStartTime_) / (triggerCount_-1);
  }
  printf("ecmcPVTController::checkTriggerTiming(): Triggers %lf:%lf:%lf (with duration %lf)\n",
          triggerStartTime_,triggerTimeBetween_,triggerEndTime_,triggerDuration_);
  return 0;
}

int ecmcPVTController::setTriggerInfo(size_t startPointId, size_t endPointId, size_t count) { 
  triggerStartPoint_ = startPointId;  // start from 1 (1==first point, however first point is acceleration so compensate fro that)
  triggerEndPoint_   = endPointId;    // start from 1
  triggerCount_      = count;
  return checkTriggerTiming();
}

int ecmcPVTController::setTriggerDuration(double durationS) {
  triggerDuration_ = durationS + sampleTime_;
  return 0;
}

int ecmcPVTController::setAxesBusy(bool busy) {  
  for(uint i = 0; i < axes_.size(); i++ ) {
    if(axes_[i]!=NULL) {axes_[i]->setGlobalBusy(busy);};
  }
  return 0;
}

ecmcPVTSMType  ecmcPVTController::getSMState() {
  return state_;
}

int ecmcPVTController::setEnable(bool enable) {  
  for(uint i = 0; i < axes_.size(); i++ ) {
    if(axes_[i]!=NULL) {
      // Always disable
      if(!enable) {
        axes_[i]->setEnable(enable);        
        break;
      }
      // do not enable if error
      if(axes_[i]->getError()) {
        return -1;
      }      
      if(!axes_[i]->getEnable()) {
        axes_[i]->setEnable(enable);
      }
    };
  }
  return 0;
}

int ecmcPVTController::checkEnabledState(bool enabled) {
  bool state = 1;
  for(uint i = 0; i < axes_.size(); i++ ) {
    if(axes_[i]!=NULL) {
      state = state && axes_[i]->getEnabled();
    } else {
      return -1;
    }
  }
  return state;
}
