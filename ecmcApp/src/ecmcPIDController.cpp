#include "ecmcPIDController.h"

#include <stdio.h>
#include <stdlib.h>

ecmcPIDController::ecmcPIDController(ecmcAxisData *axisData,
                                     double        sampleTime) {
  PRINT_ERROR_PATH("axis[%d].controller.error", axisData->axisId_);
  data_ = axisData;
  initVars();

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
  LOGINFO15("%s/%s:%d: axis[%d].controller=new;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_);
  printCurrentState();
}

ecmcPIDController::ecmcPIDController(ecmcAxisData *axisData,
                                     double        kp,
                                     double        ki,
                                     double        kd,
                                     double        kff,
                                     double        sampleTime,
                                     double        outMax,
                                     double        outMin) {
  PRINT_ERROR_PATH("axis[%d].controller.error", axisData->axisId_);
  data_ = axisData;
  initVars();

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
  LOGINFO15("%s/%s:%d: axis[%d].controller=new;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_);
  kp_         = kp;
  ki_         = ki;
  kd_         = kd;
  kff_        = kff;
  outputMax_  = outMax;
  outputMin_  = outMin;
  sampleTime_ = sampleTime;
  printCurrentState();
}

void ecmcPIDController::printCurrentState() {
  LOGINFO15("%s/%s:%d: axis[%d].controller.kp=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            kp_);
  LOGINFO15("%s/%s:%d: axis[%d].controller.ki=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            ki_);
  LOGINFO15("%s/%s:%d: axis[%d].controller.kd=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            kd_);
  LOGINFO15("%s/%s:%d: axis[%d].controller.kff=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            kff_);
  LOGINFO15("%s/%s:%d: axis[%d].controller.outputMin=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            outputMin_);
  LOGINFO15("%s/%s:%d: axis[%d].controller.outputMax=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            outputMax_);
  LOGINFO15("%s/%s:%d: axis[%d].controller.outputIMin=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            outputMin_);
  LOGINFO15("%s/%s:%d: axis[%d].controller.outputIMax=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            outputIMax_);
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
  if (kp_ != kp) {
    LOGINFO15("%s/%s:%d: axis[%d].controller.kp=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              kp);
  }

  kp_ = kp;
}

void ecmcPIDController::setKi(double ki) {
  if (ki_ != ki) {
    LOGINFO15("%s/%s:%d: axis[%d].controller.ki=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              ki);
  }

  ki_ = ki;
}

void ecmcPIDController::setKd(double kd) {
  if (kd_ != kd) {
    LOGINFO15("%s/%s:%d: axis[%d].controller.kd=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              kd);
  }

  kd_ = kd;
}

void ecmcPIDController::setKff(double kff) {
  if (kff_ != kff) {
    LOGINFO15("%s/%s:%d: axis[%d].controller.kff=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              kff);
  }

  kff_ = kff;
}

void ecmcPIDController::setOutMax(double outMax) {
  if (outputMax_ != outMax) {
    LOGINFO15("%s/%s:%d: axis[%d].controller.outputMax=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              outMax);
  }

  outputMax_ = outMax;
}

void ecmcPIDController::setOutMin(double outMin) {
  if (outputMin_ != outMin) {
    LOGINFO15("%s/%s:%d: axis[%d].controller.outputMin=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              outMin);
  }

  outputMin_ = outMin;
}

void ecmcPIDController::setIOutMax(double outMax) {
  if (outputIMax_ != outMax) {
    LOGINFO15("%s/%s:%d: axis[%d].controller.outputIMax=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              outMax);
  }

  outputIMax_ = outMax;
}

void ecmcPIDController::setIOutMin(double outMin) {
  if (outputIMin_ != outMin) {
    LOGINFO15("%s/%s:%d: axis[%d].controller.outputIMin=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              outMin);
  }

  outputIMin_ = outMin;
}

double ecmcPIDController::control(double set, double act, double ff) {
  // Simple PID loop with FF.
  // Consider to make base class to derive other controller types

  if (!data_->command_.enable || data_->interlocks_.driveSummaryInterlock) {
    reset();
    return 0;
  }

  ff_                       = ff * kff_;
  data_->status_.cntrlError = data_->status_.currentPositionSetpoint -
                              data_->status_.currentPositionActual;
  outputP_ = data_->status_.cntrlError * kp_;
  outputI_ = outputI_ + data_->status_.cntrlError * ki_;

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
    (data_->status_.cntrlError - controllerErrorOld_) * kd_;
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
  controllerErrorOld_ = data_->status_.cntrlError;
  return data_->status_.cntrlOutput;
}

int ecmcPIDController::validate() {
  if (data_->sampleTime_ <= 0) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_CNTRL_INVALID_SAMPLE_TIME);
  }
  return 0;
}
