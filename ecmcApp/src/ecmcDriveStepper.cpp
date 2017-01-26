#include "ecmcDriveStepper.hpp"

ecmcDriveStepper::ecmcDriveStepper(ecmcAxisData *axisData) : ecmcDriveBase(axisData)
{
  initVars();
  data_=axisData;
}

ecmcDriveStepper::ecmcDriveStepper(ecmcAxisData *axisData,double scale) : ecmcDriveBase(axisData)
{
  initVars();
  scale_=scale;
  data_=axisData;
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
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  return 0;
}

void ecmcDriveStepper::writeEntries()
{
  switch(data_->command_.operationModeCmd){
    case ECMC_MODE_OP_AUTO:
      controlWord_=!getError() && (uint64_t)data_->command_.enable;
      break;
    case ECMC_MODE_OP_MAN:
      controlWord_=!getError() && (uint64_t)manualModeEnable_;
      break;
  }
  controlWord_=(uint64_t)data_->command_.enable;
  ecmcDriveBase::writeEntries();
}

void ecmcDriveStepper::readEntries()
{
  ecmcDriveBase::readEntries();
  data_->status_.enabled= statusWord_>0;
}
