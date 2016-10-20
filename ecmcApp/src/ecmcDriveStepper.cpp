#include "ecmcDriveStepper.hpp"

ecmcDriveStepper::ecmcDriveStepper()
{
  initVars();
}

ecmcDriveStepper::ecmcDriveStepper(double scale)
{
  initVars();
  scale_=scale;
}
ecmcDriveStepper::~ecmcDriveStepper()
{
  ;
}

bool ecmcDriveStepper::getEnable()
{
  return enableOutput_;
}

bool ecmcDriveStepper::getEnabled()
{
  return enabledInput_;
}

int ecmcDriveStepper::setEnable(bool enable)
{
  if(interlock_ && enable){
    enableOutput_=false;
    return setErrorID(ERROR_DRV_DRIVE_INTERLOCKED);
  }

  if(enableBrake_){
    if(!enable ){
      brakeOutput_=0;  //brake locked when 0 . TODO: Apply brake some cycles before enable is low
    }
    else{
      brakeOutput_=1;  //brake open when 1
    }
  }

  enableOutput_=enable;
  return 0;
}

