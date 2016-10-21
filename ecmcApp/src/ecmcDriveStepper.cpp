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

void ecmcDriveStepper::initVars()
{
  ecmcDriveBase::initVars();
}


bool ecmcDriveStepper::getEnable()
{
  return enableCmd_;
}

bool ecmcDriveStepper::getEnabled()
{
  return enabledStatus_;
}

int ecmcDriveStepper::setEnable(bool enable)
{
  if(interlock_ && enable){
    enableCmd_=false;
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

  enableCmd_=enable;
  return 0;
}

int ecmcDriveStepper::validate()
{

  int errorCode=ecmcDriveBase::validate();
  if(errorCode){
    return setErrorID(errorCode);
  }

  return 0;
}

void ecmcDriveStepper::writeEntries()
{
  controlWord_=(uint64_t)enableCmd_;
  ecmcDriveBase::writeEntries();

}

void ecmcDriveStepper::readEntries()
{
  ecmcDriveBase::readEntries();
  enabledStatus_= statusWord_>0;
}
