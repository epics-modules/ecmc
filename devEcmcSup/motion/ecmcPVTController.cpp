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
  sampleTime_ = sampleTime;
  nextTime_   = 0;
  accTime_    = 0;
  endTime_    = 0;
  executeOld_ = 0;
  execute_    = 0;
  state_      = ECMC_PVT_IDLE;
  busy_       = 0;
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
  axes_.clear();
  startPositions_.clear();
  printf("ecmcPVTController::clearPVTAxes()\n");

}

size_t ecmcPVTController::getCurrentPointId() {
  return 0;  
}

size_t ecmcPVTController::getCurrentTriggerId() {
  return 0;
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
    state_ = ECMC_PVT_TRIGG_MOVE_AXES_TO_START;
    busy_ = 1;
    printf("ecmcPVTController::setExecute(%d): Start PVT seq.. Move axis to startpositions\n",execute);
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
  switch(state_) {
    case  ECMC_PVT_IDLE:
      busy_ = 0;
      return;
    break;

    case ECMC_PVT_TRIGG_MOVE_AXES_TO_START:
      initPVT();  // prepare pvt objects
      error = triggMoveAxesToStart();
      if(error) {
        setErrorID(error);
        state_ = ECMC_PVT_ERROR;
        printf("ecmcPVTController: Error: Axis in error state when trigger move to start position\n");
      }      
      state_ = ECMC_PVT_WAIT_FOR_AXES_TO_REACH_START;
      break;

    case  ECMC_PVT_WAIT_FOR_AXES_TO_REACH_START:
      axesAtStartPosition = axesAtStart();
      if(axesAtStartPosition < 0) {
        setErrorID(-axesAtStartPosition);
        state_ = ECMC_PVT_ERROR;
        printf("ecmcPVTController: Error: Axis in error state when moving to start position\n");
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
        printf("ecmcPVTController: All axes in position, trigger PVT sequence\n");
      }
      break;

    case ECMC_PVT_TRIGG_PVT:
      error =  triggPVT();  
      if(error){
        setErrorID(error);
        state_ = ECMC_PVT_ERROR;
        printf("ecmcPVTController: Error: Triggering of PVT objects failed\n");
      }
      printf("ecmcPVTController: Executing PVT sequence\n");
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
      if(axisFree()) {
        printf("ecmcPVTController: All axes stopped\n");
        state_ =  ECMC_PVT_IDLE;
      }
      break;
    case ECMC_PVT_ERROR:
      busy_ = 0;
      break;
  }     
}

bool  ecmcPVTController::getBusy() {
  return busy_;
}

int ecmcPVTController::triggMoveAxesToStart() {
  startPositions_.clear();
  double startPosition = 0;
  int error = 0;
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
    if(axes_[i]->getBusy()) {
      return 0;
    }
    if(axes_[i]->getCurrentPositionSetpoint() != startPositions_[i]) {
      printf("ecmcPVTController:: target set %lf, actual set %lf\n",axes_[i]->getCurrentPositionSetpoint(), startPositions_[i]);
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
    error = axes_[i]->movePVTAbs();
    if(error) {
      return error;
    }
  }
  // All axes in correct position to start
  return 0;
}

void ecmcPVTController::errorReset() {
  state_ = ECMC_PVT_IDLE;
  ecmcError::errorReset();
}

int ecmcPVTController::validate() {
  if(axes_.size()== 0) {
    printf("ecmcPVTController::validate(): Error no axis linked\n");
    return 12;

  }
  return 0; 
}

int ecmcPVTController::anyAxisInterlocked() {
  
  for(uint i = 0; i < axes_.size(); i++ ) {    
    if(axes_[i]->getSumInterlock()) {
      return 1;
    }
  }
  return 0;
}

int ecmcPVTController::abortPVT() {
  state_ = ECMC_PVT_ABORT;
  for(uint i = 0; i < axes_.size(); i++ ) {    
    axes_[i]->stopMotion(0);
    axes_[i]->getPVTObject()->setBusy(false);    
  }
  // All axes in correct position to start
  return 0;
}

int ecmcPVTController::axisFree() {
  int axesFree = 1;
  for(uint i = 0; i < axes_.size(); i++ ) {    
    axesFree = axesFree && !axes_[i]->getBusy();
  }
  // All axes in correct position to start
  return axesFree;
}

void ecmcPVTController::initPVT() {
  for(uint i = 0; i < axes_.size(); i++ ) {    
    axes_[i]->getPVTObject()->setExecute(0);
  }
}
