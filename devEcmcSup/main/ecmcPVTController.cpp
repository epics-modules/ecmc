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

ecmcPVTController::ecmcPVTController() {
  //sampleTime_ = sampleTime;
  printf("ecmcPVTController::ecmcPVTController() created\n");
  clearPVTAxes();
}

ecmcPVTController::~ecmcPVTController() {
}

void ecmcPVTController::addPVTAxis(ecmcAxisBase* axis) {  
  pvtAxes_.push_back(axis);
}

void ecmcPVTController::clearPVTAxes() {
  pvtAxes_.clear();
}

size_t ecmcPVTController::getCurrentPointId() {
  return 0;  
}
size_t ecmcPVTController::getCurrentTriggerId() {
  return 0;
}
