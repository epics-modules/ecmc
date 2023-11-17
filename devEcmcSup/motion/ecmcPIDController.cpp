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

ecmcPIDController::ecmcPIDController(ecmcAsynPortDriver *asynPortDriver,
                                     ecmcAxisData       *axisData,
                                     double              sampleTime)
  : ecmcError(&(axisData->status_.errorCode),
              &(axisData->status_.warningCode)) {
  data_ = axisData;
  initVars();
  asynPortDriver_ = asynPortDriver;
  initAsyn();

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
}

ecmcPIDController::ecmcPIDController(ecmcAsynPortDriver *asynPortDriver,
                                     ecmcAxisData       *axisData,
                                     double              kp,
                                     double              ki,
                                     double              kd,
                                     double              kff,
                                     double              sampleTime,
                                     double              outMax,
                                     double              outMin)
  : ecmcError(&(axisData->status_.errorCode),
              &(axisData->status_.warningCode)) {
  data_ = axisData;
  initVars();
  asynPortDriver_ = asynPortDriver;
  initAsyn();

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
  settingMade_        = false;
  asynPortDriver_     = NULL;
  asynKp_             = NULL;
  asynKi_             = NULL;
  asynKd_             = NULL;
  asynKff_            = NULL;
  kp_inner_           = 0;
  ki_inner_           = 0;
  kd_inner_           = 0;
  innerTol_           = 0;
}

ecmcPIDController::~ecmcPIDController() {}

void ecmcPIDController::reset() {
  outputP_            = 0;
  outputI_            = 0;
  outputD_            = 0;
  ff_                 = 0;
  controllerErrorOld_ = 0;
  data_->status_.cntrlOutput = 0;
}

void ecmcPIDController::setIRange(double iMax, double iMin) {
  if ((iMax != 0) || (iMin != 0))settingMade_ = true;
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
  if (kp != 0)settingMade_ = true;
  kp_ = kp;
  asynKp_->refreshParam(1);
}

void ecmcPIDController::setKi(double ki) {
  if (ki != 0)settingMade_ = true;
  ki_ = ki;
  asynKi_->refreshParam(1);
}

void ecmcPIDController::setKd(double kd) {
  if (kd != 0)settingMade_ = true;
  kd_ = kd;
  asynKd_->refreshParam(1);
}

void ecmcPIDController::setKff(double kff) {
  if (kff != 0)settingMade_ = true;
  kff_ = kff;
  asynKff_->refreshParam(1);
}

void ecmcPIDController::setOutMax(double outMax) {
  if (outMax != 0)settingMade_ = true;
  outputMax_ = outMax;
}

void ecmcPIDController::setOutMin(double outMin) {
  if (outMin != 0)if (outMin != 0)settingMade_ = true;
  outputMin_ = outMin;
}

void ecmcPIDController::setIOutMax(double outMax) {
  if (outMax != 0)settingMade_ = true;
  outputIMax_ = outMax;
}

void ecmcPIDController::setIOutMin(double outMin) {
  if (outMin != 0)settingMade_ = true;
  outputIMin_ = outMin;
}

double ecmcPIDController::control(double posError, double ff) {
  // Simple PID loop with FF.
  // Consider to make base class to derive other controller types

  if (!data_->command_.enable || data_->interlocks_.driveSummaryInterlock) {
    reset();
    return 0;
  }

  // Default control params
  kp_use_ = kp_;
  ki_use_ = ki_;
  kd_use_ = kd_;

  // Use other control params if distance to target is less than innerTol_
  if (innerTol_ > 0) { // tolerance must be higher than 0
    if (std::abs(data_->status_.currentTargetPosition -
                 data_->status_.currentPositionActual) < innerTol_) {
      kp_use_ = kp_inner_;
      ki_use_ = ki_inner_;
      kd_use_ = kd_inner_;
    }
  }

  ff_      = ff * kff_;
  outputP_ = posError * kp_use_;
  outputI_ = outputI_ + posError * ki_use_;

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
    (posError - controllerErrorOld_) * kd_use_;
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
  if ((data_->command_.drvMode == ECMC_DRV_MODE_CSP) && settingMade_) {
    LOGERR("%s/%s:%d: WARNING: Axis %d in CSP-mode (ecmc position control loop disabled). Settings of ecmc position control loop params will be discarded."
           " Position control loop params needs to be set directlly in drive (where the position loop is executed).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_->axisId_);
  }

  if ((asynKp_ == NULL) || (asynKi_ == NULL) || (asynKd_ == NULL) ||
      (asynKff_ == NULL)) {
    LOGERR("%s/%s:%d: Error: Axis %d: Kp,ki, kd or kff asyn param NULL .\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_->axisId_);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MAIN_ASYN_PORT_DRIVER_NULL);
  }
  return 0;
}

double ecmcPIDController::getKp() {
  return kp_;
}

double ecmcPIDController::getKi() {
  return ki_;
}

double ecmcPIDController::getKd() {
  return kd_;
}

double ecmcPIDController::getKff() {
  return kff_;
}

int ecmcPIDController::initAsyn() {
  // Add Asynparms for new encoder
  if (asynPortDriver_ == NULL) {
    LOGERR("%s/%s:%d: ERROR (axis %d): AsynPortDriver object NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_->axisId_,
           ERROR_AXIS_ASYN_PORT_OBJ_NULL);
    return ERROR_AXIS_ASYN_PORT_OBJ_NULL;
  }

  char  buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char *name                  = NULL;
  unsigned int charCount      = 0;
  ecmcAsynDataItem *paramTemp = NULL;

  // Kp
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_AX_STR "%d." ECMC_ASYN_CNTRL_KP_NAME,
                       data_->axisId_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate (%s). Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      ECMC_AX_STR "%d." ECMC_ASYN_CNTRL_KP_NAME "%d",
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }

  name      = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                                asynParamFloat64,
                                                (uint8_t *)&kp_,
                                                8,
                                                ECMC_EC_F64,
                                                0);

  if (!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->refreshParam(1);
  asynKp_ = paramTemp;

  // Ki
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_AX_STR "%d." ECMC_ASYN_CNTRL_KI_NAME,
                       data_->axisId_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate (%s). Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      ECMC_AX_STR "%d." ECMC_ASYN_CNTRL_KP_NAME "%d",
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }

  name      = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                                asynParamFloat64,
                                                (uint8_t *)&ki_,
                                                8,
                                                ECMC_EC_F64,
                                                0);

  if (!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }

  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->refreshParam(1);
  asynKi_ = paramTemp;

  // Kd
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_AX_STR "%d." ECMC_ASYN_CNTRL_KD_NAME,
                       data_->axisId_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate (%s). Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      ECMC_AX_STR "%d." ECMC_ASYN_CNTRL_KP_NAME "%d",
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }

  name      = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                                asynParamFloat64,
                                                (uint8_t *)&kd_,
                                                8,
                                                ECMC_EC_F64,
                                                0);

  if (!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }

  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->refreshParam(1);
  asynKd_ = paramTemp;

  // Kff
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_AX_STR "%d." ECMC_ASYN_CNTRL_KFF_NAME,
                       data_->axisId_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate (%s). Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      ECMC_AX_STR "%d." ECMC_ASYN_CNTRL_KP_NAME "%d",
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }

  name      = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                                asynParamFloat64,
                                                (uint8_t *)&kff_,
                                                8,
                                                ECMC_EC_F64,
                                                0);

  if (!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }

  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->refreshParam(1);
  asynKff_ = paramTemp;

  return 0;
}

void ecmcPIDController::setInnerCtrlParams(double kp,
                                           double ki,
                                           double kd,
                                           double tol) {
  kp_inner_ =  kp;
  ki_inner_ =  ki;
  kd_inner_ =  kd;
  innerTol_ = tol;
}
