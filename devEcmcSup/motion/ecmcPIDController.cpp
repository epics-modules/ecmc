/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPIDController.cpp
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcPIDController.h"

#include <stdio.h>
#include <stdlib.h>

ecmcPIDController::ecmcPIDController(ecmcAxisData *axisData,
                                     double        sampleTime) {
  data_ = axisData;
  initVars();

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
}

ecmcPIDController::ecmcPIDController(ecmcAxisData *axisData,
                                     double        kp,
                                     double        ki,
                                     double        kd,
                                     double        kff,
                                     double        sampleTime,
                                     double        outMax,
                                     double        outMin) {
  data_ = axisData;
  initVars();

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
  kp_         = kp;
  ki_         = ki;
  kd_         = kd;
  kff_        = kff;
  outputMax_  = outMax;
  outputMin_  = outMin;
  sampleTime_ = sampleTime;
}

void ecmcPIDController::initVars() {
  errorReset();
  outputP_            = 0;
  outputI_            = 0;
  outputD_            = 0;
  outputIMax_         = 0;  // For integrator
  outputIMin_         = 0;
  outputMax_          = 0;  // For combined PID output
  outputMin_          = 0;
  ff_                 = 0;
  controllerErrorOld_ = 0;
  kp_                 = 0;
  ki_                 = 0;
  kd_                 = 0;
  kff_                = 0;
  sampleTime_         = 0;
  settingMade_       = false;
}

ecmcPIDController::~ecmcPIDController()
{}

void ecmcPIDController::reset() {
  outputP_            = 0;
  outputI_            = 0;
  outputD_            = 0;
  ff_                 = 0;
  controllerErrorOld_ = 0;
}

void ecmcPIDController::setIRange(double iMax, double iMin) {
  if(iMax != 0 || iMin != 0 ) settingMade_ = true;
  outputIMax_ = iMax;
  outputIMin_ = iMin;
}

double ecmcPIDController::getOutPPart() {
  return outputP_;
}

double ecmcPIDController::getOutIPart() {
  return outputI_;
}

double ecmcPIDController::getOutDPart() {
  return outputD_;
}

double ecmcPIDController::getOutFFPart() {
  return ff_;
}

double ecmcPIDController::getOutTot() {
  return data_->status_.cntrlOutput;
}

void ecmcPIDController::setKp(double kp) {
  if(kp != 0) settingMade_ = true;
  kp_ = kp;
}

void ecmcPIDController::setKi(double ki) {
  if(ki != 0) settingMade_ = true;
  ki_ = ki;
}

void ecmcPIDController::setKd(double kd) {
  if(kd != 0) settingMade_ = true;
  kd_ = kd;
}

void ecmcPIDController::setKff(double kff) {
  if(kff != 0) settingMade_ = true;
  kff_ = kff;
}

void ecmcPIDController::setOutMax(double outMax) {
  if(outMax != 0) settingMade_ = true;
  outputMax_ = outMax;
}

void ecmcPIDController::setOutMin(double outMin) {
  if(outMin != 0) if(outMin!=0) settingMade_ = true;
  outputMin_ = outMin;
}

void ecmcPIDController::setIOutMax(double outMax) {
  if(outMax != 0) settingMade_ = true;
  outputIMax_ = outMax;
}

void ecmcPIDController::setIOutMin(double outMin) {
  if(outMin != 0) settingMade_ = true;
  outputIMin_ = outMin;
}

double ecmcPIDController::control(double posError, double ff) {
  // Simple PID loop with FF.
  // Consider to make base class to derive other controller types

  if (!data_->command_.enable || data_->interlocks_.driveSummaryInterlock) {
    reset();
    return 0;
  }

  ff_                       = ff * kff_;                     
  outputP_ = posError * kp_;
  outputI_ = outputI_ + posError * ki_;

  // Enabled only when limits differ and max>min
  if ((outputIMax_ != outputIMin_) && (outputIMax_ > outputIMin_)) {
    if (outputI_ > outputIMax_) {
      outputI_ = outputIMax_;
    }

    if (outputI_ < outputIMin_) {
      outputI_ = outputIMin_;
    }
  }
  outputD_ =
    (posError - controllerErrorOld_) * kd_;
  data_->status_.cntrlOutput = outputP_ + outputI_ + outputD_ + ff_;

  // Enabled only when limits differ and max>min
  if ((outputMax_ != outputMin_) && (outputMax_ > outputMin_)) {
    if (data_->status_.cntrlOutput > outputMax_) {
      data_->status_.cntrlOutput = outputMax_;
    }

    if (data_->status_.cntrlOutput < outputMin_) {
      data_->status_.cntrlOutput = outputMin_;
    }
  }
  controllerErrorOld_ = posError;
  return data_->status_.cntrlOutput;
}

int ecmcPIDController::validate() {
  if (data_->sampleTime_ <= 0) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_CNTRL_INVALID_SAMPLE_TIME);
  }

  // Output warning if CSP and any of the position control parameters have been set.
  if(data_->command_.drvMode == ECMC_DRV_MODE_CSP && settingMade_) {
    LOGERR("%s/%s:%d: WARNING: Axis %d in CSP-mode (ecmc position control loop disabled). Settings of ecmc position control loop params will be discarded."
    " Position control loop params needs to be set directlly in drive (where the position loop is executed).\n", __FILE__, __FUNCTION__, __LINE__, data_->axisId_);
  }
  return 0;
}
