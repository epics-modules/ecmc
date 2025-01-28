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
  printf("ecmcPVTController::ecmcPVTController() created\n");
  currTime_ = 0;
  offsetTime_ = 0;
}

ecmcPVTController::~ecmcPVTController() {
}

//void ecmcPVTController::addPVTAxis(ecmcAxisBase* axis) {
//  pvtAxes_.push_back(axis);
//}

//void ecmcPVTController::clearPVTAxes() {
//  pvtAxes_.clear();
//}

size_t ecmcPVTController::getCurrentPointId() {
  return 0;  
}

size_t ecmcPVTController::getCurrentTriggerId() {
  return 0;
}


double ecmcPVTController::getCurrentTime() {
  return currTime_;
}

// Time is set by ALL PVT active axis in use. TODO not optimal
void ecmcPVTController::setCurrentTime(double time) {
  currTime_ = time;
  checkIfTimeToTrigger();
}

void ecmcPVTController::checkIfTimeToTrigger() {
  return;
}

// Offset for start period
void ecmcPVTController::initNewSeq(double offsetTime) {
  offsetTime_ = offsetTime;
}
