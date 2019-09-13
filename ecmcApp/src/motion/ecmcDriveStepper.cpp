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
  PRINT_ERROR_PATH("axis[%d].drive.error", axisData->axisId_);
  initVars();
  data_ = axisData;

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
  LOGINFO15("%s/%s:%d: axis[%d].drive=new;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_);
  printCurrentState();
}

ecmcDriveStepper::ecmcDriveStepper(ecmcAsynPortDriver *asynPortDriver,
                                   ecmcAxisData *axisData,
                                   double        scale) : 
                                   ecmcDriveBase(asynPortDriver,axisData) {
  PRINT_ERROR_PATH("axis[%d].drive.error", axisData->axisId_);
  initVars();
  data_ = axisData;

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }

  scale_ = scale;
  LOGINFO15("%s/%s:%d: axis[%d].drive=new;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_);
  printCurrentState();
}

ecmcDriveStepper::~ecmcDriveStepper()
{}

void ecmcDriveStepper::printCurrentState() {
  ecmcDriveBase::printCurrentState();
  LOGINFO15("%s/%s:%d: axis[%d].drive.type=ECMC_STEPPER;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_);
  LOGINFO15("%s/%s:%d: axis[%d].drive.scale=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            scale_);
  LOGINFO15("%s/%s:%d: axis[%d].drive.enabled=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            data_->status_.enabled > 0);
}

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
  bool enabledOld = data_->status_.enabled;
  data_->status_.enabled = statusWord_ > 0;

  if (enabledOld != data_->status_.enabled) {
    LOGINFO15("%s/%s:%d: axis[%d].drive.enabled=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              data_->status_.enabled > 0);
  }
}
