#include "ecmcDriveDS402.hpp"

ecmcDriveDS402::ecmcDriveDS402()
{
  initVars();
}

ecmcDriveDS402::ecmcDriveDS402(double scale)
{
  initVars();
  scale_=scale;
}
ecmcDriveDS402::~ecmcDriveDS402()
{
  ;
}

bool ecmcDriveDS402::getEnable()
{
  return enableOutput_;
}

bool ecmcDriveDS402::getEnabled()
{
  return enabledInput_;
}

int ecmcDriveDS402::setEnable(bool enable)
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

int ecmcDriveDS402::validate()
{
  int errorCode=validateEntry(0); //Control word
  if(errorCode){
    return setErrorID(errorCode);
  }

  int bitCount=0;  //DS402 must have atleast 8 bit control word
  getEntryBitCount(0,&bitCount);
  if(bitCount<8){
    return setErrorID(ERROR_DRV_DS402_CONTROL_WORD_BIT_COUNT_ERROR);
  }

  errorCode=validateEntry(1); //Velocity Setpoint entry output
  if(errorCode){
    return setErrorID(errorCode);
  }

  errorCode=validateEntry(2); //Status word
  if(errorCode){
    return setErrorID(errorCode);
  }

  bitCount=0;  //DS402 must have atleast 8 bit control word
  getEntryBitCount(0,&bitCount);
  if(bitCount<8){
    return setErrorID(ERROR_DRV_DS402_STATUS_WORD_BIT_COUNT_ERROR);
  }

  if(enableBrake_){
    errorCode=validateEntry(3); //brake output
    if(errorCode){
      return setErrorID(errorCode);
    }
  }

  if(enableReduceTorque_){
    errorCode=validateEntry(4); //reduce torque output
    if(errorCode){
      return setErrorID(errorCode);
    }
  }

  if(scaleDenom_==0){
    return setErrorID(ERROR_DRV_SCALE_DENOM_ZERO);
  }
  return 0;
}
