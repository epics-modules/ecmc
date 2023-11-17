/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAxisReal.cpp
*
*  Created on: Mar 10, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcAxisReal.h"

ecmcAxisReal::ecmcAxisReal(ecmcAsynPortDriver *asynPortDriver,
                           int                 axisID,
                           double              sampleTime,
                           ecmcDriveTypes      drvType,
                           ecmcTrajTypes       trajType) :
  ecmcAxisBase(asynPortDriver,
               axisID,
               sampleTime,
               trajType) {
  initVars();
  data_.axisId_     = axisID;
  data_.axisType_   = ECMC_AXIS_TYPE_REAL;
  data_.sampleTime_ = sampleTime;

  // Create drive
  switch (drvType) {
  case ECMC_STEPPER:
    drv_              = new ecmcDriveStepper(asynPortDriver_, &data_);
    currentDriveType_ = ECMC_STEPPER;
    break;

  case ECMC_DS402:
    drv_              = new ecmcDriveDS402(asynPortDriver_, &data_);
    currentDriveType_ = ECMC_DS402;
    break;

  default:
    LOGERR("%s/%s:%d: DRIVE TYPE NOT SUPPORTED (%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
    exit(1);

    break;
  }

  // Create PID
  cntrl_ = new ecmcPIDController(asynPortDriver_, &data_, data_.sampleTime_);

  seq_.setCntrl(cntrl_);
}

ecmcAxisReal::~ecmcAxisReal() {
  delete cntrl_;
  cntrl_ = NULL;
  delete drv_;
  drv_ = NULL;
}

void ecmcAxisReal::initVars() {
  currentDriveType_         = ECMC_NO_DRIVE;
  temporaryLocalTrajSource_ = false;
}

void ecmcAxisReal::execute(bool masterOK) {
  ecmcAxisBase::preExecute(masterOK);

  drv_->readEntries();

  // Trajectory (External or internal)
  if (data_.command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
    data_.status_.currentPositionSetpoint = traj_->getNextPosSet();
    data_.status_.currentVelocitySetpoint = traj_->getVel();
  } else {    // External source (PLC)
    data_.status_.currentPositionSetpoint =
      data_.status_.externalTrajectoryPosition;
    data_.status_.currentVelocitySetpoint =
      data_.status_.externalTrajectoryVelocity;
    data_.interlocks_.noExecuteInterlock = false;  // Only valid in local mode
    data_.refreshInterlocks();
  }

  if (data_.command_.encSource == ECMC_DATA_SOURCE_INTERNAL) {
    data_.status_.currentPositionActual =
      encArray_[data_.command_.primaryEncIndex]->getActPos();
    data_.status_.currentVelocityActual =
      encArray_[data_.command_.primaryEncIndex]->getActVel();
  } else { // External source
    data_.status_.currentPositionActual = data_.status_.externalEncoderPosition;
    data_.status_.currentVelocityActual = data_.status_.externalEncoderVelocity;
  }

  traj_->setStartPos(data_.status_.currentPositionSetpoint);
  seq_.execute();
  mon_->execute();

  // Switch to internal trajectory temporary if interlock
  bool trajLock =
    ((data_.interlocks_.trajSummaryInterlockFWD &&
      data_.status_.currentPositionSetpoint >
      data_.status_.currentPositionSetpointOld) ||
     (data_.interlocks_.trajSummaryInterlockBWD &&
      data_.status_.currentPositionSetpoint <
      data_.status_.currentPositionSetpointOld));

  if (trajLock &&
      (data_.command_.trajSource != ECMC_DATA_SOURCE_INTERNAL)) {
    if (!temporaryLocalTrajSource_) {  // Initiate rampdown
      temporaryLocalTrajSource_ = true;
      traj_->setStartPos(data_.status_.currentPositionActual);
      traj_->initStopRamp(data_.status_.currentPositionActual,
                          data_.status_.currentVelocityActual,
                          0);
    }
    statusData_.onChangeData.statusWd.trajsource = ECMC_DATA_SOURCE_INTERNAL;  // Temporary
    data_.status_.currentPositionSetpoint        = traj_->getNextPosSet();
    data_.status_.currentVelocitySetpoint        = traj_->getVel();
  } else {
    temporaryLocalTrajSource_ = false;
  }

  if (data_.interlocks_.driveSummaryInterlock && !traj_->getBusy()) {
    cntrl_->reset();
  }

  // CSP Write raw actpos  and actpos to drv obj
  drv_->setCspActPos(
    encArray_[data_.command_.primaryEncIndex]->getRawPosRegister(),
    data_.status_.currentPositionActual);

  if (getEnabled() && masterOK) {
    // Calc position error
    data_.status_.cntrlError = ecmcMotionUtils::getPosErrorModWithSign(
      data_.status_.currentPositionSetpoint,
      data_.status_.currentPositionSetpointOld,
      data_.status_.currentPositionActual,
      data_.command_.moduloRange);
    double cntrOutput = 0;

    if (data_.command_.drvMode == ECMC_DRV_MODE_CSV) {
      // ***************** CSV *****************
      // Controller deadband
      if (!data_.status_.busy && mon_->getCtrlInDeadband()) {
        cntrl_->reset();  // Keep now for leagcy reasons...
        cntrOutput = 0;
      } else {
        cntrOutput = cntrl_->control(data_.status_.cntrlError,
                                     data_.status_.currentVelocitySetpoint);
      }
      mon_->setEnable(true);
      drv_->setVelSet(cntrOutput);  // Actual control
    } else {
      // ***************** CSP *****************
      mon_->setEnable(true);
      drv_->setCspPosSet(data_.status_.currentPositionSetpoint);  // Actual control
    }
  } else {
    mon_->setEnable(false);

    if (getExecute()) {
      setExecute(false);
    }

    // Only update if enable cmd is low to avoid change of setpoint
    // during between enable and enabled
    if (!getEnable() && !beforeFirstEnable_ && masterOK) {
      data_.status_.currentPositionSetpoint =
        data_.status_.currentPositionActual;
      traj_->setStartPos(data_.status_.currentPositionSetpoint);
    }

    if (data_.status_.enabledOld && !data_.status_.enabled &&
        data_.status_.enableOld && data_.command_.enable) {
      setEnable(false);
      setErrorID(__FILE__,
                 __FUNCTION__,
                 __LINE__,
                 ERROR_AXIS_AMPLIFIER_ENABLED_LOST);
    }

    // CSV
    drv_->setVelSet(0);

    // CSP
    drv_->setCspPosSet(data_.status_.currentPositionActual);
    cntrl_->reset();
  }

  if (!masterOK) {
    if (getEnabled() || getEnable()) {
      setEnable(false);
    }
    cntrl_->reset();
    drv_->setVelSet(0);
    setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_AXIS_HARDWARE_STATUS_NOT_OK);
  }

  // Write to hardware
  // refreshExternalOutputSources();
  drv_->writeEntries();

  if (std::abs(drv_->getScale()) > 0) {
    data_.status_.currentvelocityFFRaw = cntrl_->getOutFFPart() /
                                         drv_->getScale();
  } else {
    data_.status_.currentvelocityFFRaw = 0;
  }

  ecmcAxisBase::postExecute(masterOK);
}

ecmcPIDController * ecmcAxisReal::getCntrl() {
  return cntrl_;
}

ecmcDriveBase * ecmcAxisReal::getDrv() {
  return drv_;
}

int ecmcAxisReal::validate() {
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

  if (drv_ == NULL) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_DRV_OBJECT_NULL);
  }

  error = drv_->validate();

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  // Set drv ref to sequencer (used for CSP)
  seq_.setDrv(drv_);

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

  if (cntrl_ == NULL) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_CNTRL_OBJECT_NULL);
  }

  error = cntrl_->validate();

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
