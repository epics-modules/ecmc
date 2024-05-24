/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAxisVirt.cpp
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcAxisVirt.h"

ecmcAxisVirt::ecmcAxisVirt(ecmcAsynPortDriver *asynPortDriver,
                           int                 axisID,
                           double              sampleTime,
                           ecmcTrajTypes       trajType) :
  ecmcAxisBase(asynPortDriver,
               axisID,
               sampleTime,
               trajType) {
  initVars();
  data_.axisId_   = axisID;
  data_.axisType_ = ECMC_AXIS_TYPE_VIRTUAL;
  seq_.setCntrl(NULL);
  data_.sampleTime_ = sampleTime;
}

ecmcAxisVirt::~ecmcAxisVirt() {}

void ecmcAxisVirt::initVars() {
}

void ecmcAxisVirt::execute(bool masterOK) {
  ecmcAxisBase::preExecute(masterOK);

  // update setpoinmt and actual values
  seq_.execute();
  
  if (getEnabled() && masterOK && !getError()) {
    mon_->setEnable(true);
    data_.status_.cntrlError = ecmcMotionUtils::getPosErrorModWithSign(
      data_.status_.currentPositionSetpoint,
      data_.status_.currentPositionSetpointOld,
      data_.status_.currentPositionActual,
      data_.command_.moduloRange);
  } else {
    mon_->setEnable(false);

    if (getExecute()) {
      setExecute(false);
    }

    if (!beforeFirstEnable_ && masterOK) {
      data_.status_.currentPositionSetpoint =
        data_.status_.currentPositionActual;
      traj_->setStartPos(data_.status_.currentPositionSetpoint);
    }
  }

  if (!masterOK) {
    if (getEnabled() || getEnable()) {
      setEnable(false);
    }
    setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_AXIS_HARDWARE_STATUS_NOT_OK);
  }

  // No drive object so update needed variables
  data_.status_.currentvelocityFFRaw = 0;
  data_.status_.enabled              = data_.command_.enable;

  ecmcAxisBase::postExecute(masterOK);
}

ecmcPIDController * ecmcAxisVirt::getCntrl() {
  return NULL;
}

ecmcDriveBase * ecmcAxisVirt::getDrv() {
  return NULL;
}

int ecmcAxisVirt::validate() {
  int error = 0;

  if (data_.command_.primaryEncIndex >= data_.status_.encoderCount) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_ENC_OBJECT_NULL);
  }

  for (int i = 0; i < data_.status_.encoderCount; i++) {
    if (encArray_[i] == NULL) {
      LOGERR("%s/%s:%d: ax%d.enc%d NULL (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             data_.axisId_,
             i,
             ERROR_AXIS_ENC_OBJECT_NULL);

      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_AXIS_ENC_OBJECT_NULL);
    }

    error = encArray_[i]->validate();

    if (error) {
      LOGERR("%s/%s:%d: ax%d.enc%d (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             data_.axisId_,
             i,
             error);

      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }
  }

  if (data_.command_.encSource == ECMC_DATA_SOURCE_INTERNAL) {
    error = encArray_[data_.command_.primaryEncIndex]->validate();

    if (error) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }
  }

  if (traj_ == NULL) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_TRAJ_OBJECT_NULL);
  }

  error = traj_->validate();

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  if (mon_ == NULL) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_MON_OBJECT_NULL);
  }

  error = mon_->validate();

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  error = seq_.validate();

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  error = ecmcAxisBase::validateBase();

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  return 0;
}
