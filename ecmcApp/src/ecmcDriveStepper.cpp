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
