/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcDriveStepper.cpp
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcDriveStepper.h"

ecmcDriveStepper::ecmcDriveStepper(ecmcAsynPortDriver *asynPortDriver,
                                   ecmcAxisData *axisData) : 
                                   ecmcDriveBase(asynPortDriver,axisData) {
  initVars();
  data_ = axisData;

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
}

ecmcDriveStepper::ecmcDriveStepper(ecmcAsynPortDriver *asynPortDriver,
                                   ecmcAxisData *axisData,
                                   double        scale) : 
                                   ecmcDriveBase(asynPortDriver,axisData) {
  initVars();
  data_ = axisData;

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }

  scale_ = scale;
}

ecmcDriveStepper::~ecmcDriveStepper()
{}

void ecmcDriveStepper::initVars()
{}

int  ecmcDriveStepper::validate() {
  int errorCode = ecmcDriveBase::validate();

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }
  return 0;
}

void ecmcDriveStepper::writeEntries() {
  controlWord_ = !getError() && (uint64_t)enableAmpCmd_;

  if (getError()) {
    enableAmpCmd_                             = false;
    controlWord_                              = 0;
    data_->status_.currentVelocitySetpointRaw = 0;
  }
  ecmcDriveBase::writeEntries();
}

void ecmcDriveStepper::readEntries() {
  ecmcDriveBase::readEntries();
  data_->status_.enabled = statusWord_ > 0;
}
