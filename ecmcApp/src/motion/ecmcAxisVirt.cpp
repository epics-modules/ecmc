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
                           int axisID,
                           double sampleTime) :  
              ecmcAxisBase(asynPortDriver,
                          axisID,
                          sampleTime) {
  PRINT_ERROR_PATH("axis[%d].error", axisID);
  initVars();
  data_.axisId_   = axisID;
  data_.axisType_ = ECMC_AXIS_TYPE_VIRTUAL;
  seq_.setCntrl(NULL);
  data_.sampleTime_ = sampleTime;

  LOGINFO15("%s/%s:%d: axis[%d]=new;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            axisID);
  printCurrentState();
}

ecmcAxisVirt::~ecmcAxisVirt()
{}

void ecmcAxisVirt::printCurrentState() {
  ecmcAxisBase::printCurrentState();
  LOGINFO15("%s/%s:%d: axis[%d].type=%s;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            "ECMC_AXIS_TYPE_VIRTUAL");
  LOGINFO15("%s/%s:%d: axis[%d].sampleTime=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            data_.sampleTime_);
  LOGINFO15("%s/%s:%d: axis[%d].temporaryLocalTrajSource_=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            temporaryLocalTrajSource_ > 0);
}

void ecmcAxisVirt::initVars() {
  initDone_                 = false;
  temporaryLocalTrajSource_ = false;
}

void ecmcAxisVirt::execute(bool masterOK) {

  ecmcAxisBase::preExecute(masterOK);

  if (masterOK) {
    // Trajectory (External or internal)
    if (data_.command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {         
      data_.status_.currentPositionSetpoint = traj_->getNextPosSet();
      data_.status_.currentVelocitySetpoint = traj_->getVel();
    } else {    // External source (Transform)
      data_.status_.currentPositionSetpoint =
        data_.status_.externalTrajectoryPosition;
      data_.status_.currentVelocitySetpoint =
        data_.status_.externalTrajectoryVelocity;
      data_.interlocks_.noExecuteInterlock = false;  // Only valid in local mode
      data_.refreshInterlocks();
    }

    // Encoder (External or internal)
    if (data_.command_.encSource == ECMC_DATA_SOURCE_INTERNAL) {
      data_.status_.currentPositionActual = enc_->getActPos();
      data_.status_.currentVelocityActual = enc_->getActVel();
    } else {    // External source (Transform)
      data_.status_.currentPositionActual =
        data_.status_.externalEncoderPosition;
      data_.status_.currentVelocityActual =
        data_.status_.externalEncoderVelocity;
    }

    traj_->setStartPos(data_.status_.currentPositionSetpoint);
    seq_.execute();
    data_.status_.cntrlOutput = 0;
    mon_->execute();

    // Switch to internal trajectory if interlock temporary
    bool trajLock =
      ((data_.interlocks_.trajSummaryInterlockFWD &&
        data_.status_.currentVelocitySetpoint > 0) ||
       (data_.interlocks_.trajSummaryInterlockBWD &&
        data_.status_.currentVelocitySetpoint < 0));

    if (trajLock &&
        (data_.command_.trajSource != ECMC_DATA_SOURCE_INTERNAL)) {
      if (!temporaryLocalTrajSource_) {  // Initiate rampdown
        temporaryLocalTrajSource_ = true;
        traj_->setStartPos(data_.status_.currentPositionActual);
        traj_->initStopRamp(data_.status_.currentPositionActual,
                            data_.status_.currentVelocityActual,
                            0);
      }
      statusData_.onChangeData.trajSource   = ECMC_DATA_SOURCE_INTERNAL;
      data_.status_.currentPositionSetpoint = traj_->getNextPosSet();
      data_.status_.currentVelocitySetpoint = traj_->getVel();
    } else {
      temporaryLocalTrajSource_ = false;
    }

    if (getEnabled() && masterOK && !getError()) {

      mon_->setEnable(true);
      data_.status_.cntrlError = getPosErrorMod();
    } else {
      
      mon_->setEnable(false);

      if (getExecute()) {
        setExecute(false);
      }
      data_.status_.currentPositionSetpoint =
        data_.status_.currentPositionActual;
      traj_->setStartPos(data_.status_.currentPositionSetpoint);
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

    // Write to hardware
    //refreshExternalOutputSources();
  }

  // No drive object so update needed variables
  data_.status_.currentvelocityFFRaw = 0;
  data_.status_.enabled              = data_.command_.enable;

  ecmcAxisBase::postExecute(masterOK);
}

int ecmcAxisVirt::setOpMode(operationMode mode) {
  // NO DRIVE
  return setErrorID(__FILE__,
                    __FUNCTION__,
                    __LINE__,
                    ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

operationMode ecmcAxisVirt::getOpMode() {
  // NO DRIVE
  return ECMC_MODE_OP_AUTO;
}

ecmcPIDController * ecmcAxisVirt::getCntrl() {
  return NULL;
}

ecmcDriveBase * ecmcAxisVirt::getDrv() {
  return NULL;
}

int ecmcAxisVirt::validate() {
  int error = 0;

  if (enc_ == NULL) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_ENC_OBJECT_NULL);
  }

  if (data_.command_.encSource == ECMC_DATA_SOURCE_INTERNAL) {
    error = enc_->validate();

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
