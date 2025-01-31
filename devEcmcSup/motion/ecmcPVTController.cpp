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
  clearPVTAxes();
}

ecmcPVTController::~ecmcPVTController() {
}

void ecmcPVTController::addPVTAxis(ecmcAxisPVTSequence* axis) {
  pvt_.push_back(axis);
}

void ecmcPVTController::clearPVTAxes() {
  pvt_.clear();  
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
  if(!pvt_[0]) {
    printf("ecmcPVTController::setExecute(%d): Error no axis linked\n",execute);
    return;
  }
  if(!executeOld_ && execute_ && pvt_[0]) {
    nextTime_ = -sampleTime_; // Start at -1 sample
    // Set time to 0 in all PVT objects
    for(uint i = 0; i < pvt_.size(); i++ ) {
      pvt_[i]->setNextTime(nextTime_);
    }
    // get end time from first axis
    endTime_ = pvt_[0]->endTime();
    // get acc time from first axis
    accTime_ = pvt_[0]->getSegDuration(0);
  }
}

// Execute by ecmc RT
void ecmcPVTController::execute() {
  if(!execute_ || nextTime_ >= endTime_) {    
    return;
  }
  
  bool seqDone = false;

  // Increase time
  nextTime_ = nextTime_ + sampleTime_;

  if(nextTime_> endTime_) {    
    nextTime_ = endTime_;
    seqDone = true;
  }

  for(uint i = 0; i < pvt_.size(); i++ ) {
    if(pvt_[i]->getBusy() || nextTime_ == 0)   {
      pvt_[i]->setNextTime(nextTime_);
      if(seqDone) {
        pvt_[i]->setBusy(false);
      }
    }
  }
}

bool  ecmcPVTController::getBusy() {
  if(pvt_.size() > 0) {
    return pvt_[0]->getBusy();
  }
  return false;
}
