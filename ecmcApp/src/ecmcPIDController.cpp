#include "ecmcPIDController.hpp"

#include <stdio.h>

ecmcPIDController::ecmcPIDController(double sampleTime)
{
  initVars();
  sampleTime_=sampleTime;
}

ecmcPIDController::ecmcPIDController(double kp, double ki, double kd, double kff, double sampleTime, double outMax, double outMin)
{
  initVars();
  kp_=kp;
  ki_=ki;
  kd_=kd;
  kff_=kff;
  outputMax_=outMax;
  outputMin_=outMin;
  sampleTime_=sampleTime;
}

void ecmcPIDController::initVars()
{
  errorReset();
  controllerError_=0;
  setpoint_=0;
  actual_=0;
  outputP_=0;
  outputI_=0;
  outputD_=0;
  outputIMax_=0;       //For integrator
  outputIMin_=0;
  outputMax_=0;        //For combined PID output
  outputMin_=0;
  ff_=0;
  output_=0;
  controllerErrorOld_=0;
  sampleTime_=0;
  enable_=false;
  kp_=0;
  ki_=0;
  kd_=0;
  kff_=0;
  outputMax_=0;
  outputMin_=0;
  interlock_=true;
}

ecmcPIDController::~ecmcPIDController()
{
  ;
}

void ecmcPIDController::reset()
{
  setpoint_=0;
  actual_=0;
  outputP_=0;
  outputI_=0;
  outputD_=0;
  ff_=0;
  controllerError_=0;
  controllerErrorOld_=0;
  output_=0;
  //enable_=false;
}

double ecmcPIDController::getCntrlError()
{
  return controllerError_;
}

void ecmcPIDController::setIRange(double iMax, double iMin)
{
  outputIMax_=iMax;
  outputIMin_=iMin;
}

double ecmcPIDController::getOutPPart()
{
  return outputP_;
}

double ecmcPIDController::getOutIPart()
{
  return outputI_;
}

double ecmcPIDController::getOutDPart()
{
  return outputD_;
}

double ecmcPIDController::getOutFFPart()
{
  return ff_;
}

double ecmcPIDController::getOutTot()
{
  return output_;
}

void ecmcPIDController::setKp(double kp)
{
  kp_=kp;
}

void ecmcPIDController::setKi(double ki){
  ki_=ki;
}

void ecmcPIDController::setKd(double kd)
{
  kd_=kd;
}

void ecmcPIDController::setKff(double kff)
{
  kff_=kff;
}

void ecmcPIDController::setOutMax(double outMax)
{
  outputMax_=outMax;
}

void ecmcPIDController::setOutMin(double outMin)
{
  outputMin_=outMin;
}

void ecmcPIDController::setIOutMax(double outMax)
{
  outputIMax_=outMax;
}

void ecmcPIDController::setIOutMin(double outMin)
{
  outputIMin_=outMin;
}

double ecmcPIDController::control(double set, double act, double ff)
{
  //Simple PID loop with FF.. Consider to make base class to derive other controller types

  if(!enable_ || interlock_){
    reset();
    return output_;
  }
  setpoint_=set;
  actual_=act;
  ff_=ff*kff_;
  controllerError_=setpoint_-actual_;
  outputP_=controllerError_*kp_;
  outputI_=outputI_+controllerError_*ki_;
  if(outputIMax_!=outputIMin_ && outputIMax_>outputIMin_){  //Enabled only when limits differ and max>min
    if(outputI_>outputIMax_){
      outputI_=outputIMax_;
    }
    if(outputI_<outputIMin_){
      outputI_=outputIMin_;
    }
  }
  outputD_=(controllerError_-controllerErrorOld_)*kd_;
  output_=outputP_+outputI_+outputD_+ff_;

  if(outputMax_!=outputMin_ && outputMax_>outputMin_){ //Enabled only when limits differ and max>min
    if(output_>outputMax_){
      output_=outputMax_;
    }
    if(output_<outputMin_){
      output_=outputMin_;
    }
  }
  controllerErrorOld_=controllerError_;
  return output_;
}

void ecmcPIDController::setSampleTime(double sampleTime)
{
  sampleTime_=sampleTime;
}

double ecmcPIDController::getSampleTime()
{
  return sampleTime_;
}

void ecmcPIDController::setEnable(bool enable)
{
  enable_=enable;
}

bool ecmcPIDController::getEnable()
{
  return enable_;
}

int ecmcPIDController::validate()
{
  if(sampleTime_<=0){
    return setErrorID(ERROR_CNTRL_INVALID_SAMPLE_TIME);
  }
  return 0;
}

int ecmcPIDController::setInterlock(bool interlock)
{
  interlock_=interlock;
  return 0;
}
