#include "ecmcDrive.hpp"

ecmcDrive::ecmcDrive()
{
  initVars();
}

ecmcDrive::ecmcDrive(double scale)
{
  initVars();
  scale_=scale;
}

void ecmcDrive::initVars()
{
  errorReset();
  interlock_=true;
  scale_=0;
  scaleNum_=0;
  scaleDenom_=0;
  velSetRawOutput_=0;
  velSet_=0;
  enableOutput_=0;
  enabledInput_=0;
  enableBrake_=0;
  enableReduceTorque_=0;
  reduceTorqueOutput_=0;
  brakeOutput_=0;
}

ecmcDrive::~ecmcDrive()
{
  ;
}

int ecmcDrive::setVelSet(double vel)
{
  if(interlock_){
    velSet_=0;
    velSetRawOutput_=0;
    enableOutput_=0;
    return 0;
  }
  velSet_=vel;
  velSetRawOutput_=velSet_/scale_;
  return 0;
}

void ecmcDrive::setScaleNum(double scaleNum)
{
  scaleNum_=scaleNum;
  if(std::abs(scaleDenom_)>0){
    scale_=scaleNum_/scaleDenom_;
  }
}

int ecmcDrive::setScaleDenom(double scaleDenom)
{
  scaleDenom_=scaleDenom;
  if(scaleDenom_==0){
    return setErrorID(ERROR_DRV_SCALE_DENOM_ZERO);
  }
  scale_=scaleNum_/scaleDenom_;
  return 0;
}

double ecmcDrive::getScale()
{
  return scale_;
}

double ecmcDrive::getVelSet()
{
  return velSet_;
}

int ecmcDrive::getVelSetRaw()
{
  return velSetRawOutput_;
}

bool ecmcDrive::getEnable()
{
  return enableOutput_;
}

bool ecmcDrive::getEnabled()
{
  return enabledInput_;
}

void ecmcDrive::writeEntries()
{
  if (getError()){
    return;
  }

  int errorCode=0;
  errorCode=writeEcEntryValue(0,(uint64_t)enableOutput_);
  if(errorCode){
    setErrorID(errorCode);
  }

  errorCode=writeEcEntryValue(1,(uint64_t)velSetRawOutput_);
  if(errorCode){
    setErrorID(errorCode);
  }

  if(enableBrake_){
    errorCode=writeEcEntryValue(3,(uint64_t)brakeOutput_);
    if(errorCode){
      setErrorID(errorCode);
    }
  }

  if(enableReduceTorque_){
    errorCode=writeEcEntryValue(4,(uint64_t)reduceTorqueOutput_);
    if(errorCode){
      setErrorID(errorCode);
    }
  }

}

void ecmcDrive::readEntries()
{

  if(getError()){
    return;
  }

  uint64_t tempRaw=0;

  if(readEcEntryValue(2,&tempRaw)){
    setErrorID(ERROR_DRV_ENABLED_READ_ENTRY_FAIL);
    enableOutput_=false;
    enabledInput_=false;
    return;
  }
  enabledInput_= tempRaw>0;
}

int ecmcDrive::setEnable(bool enable)
{
  if(interlock_){
    return setErrorID(ERROR_DRV_DRIVE_INTERLOCKED);
  }

  if(enableBrake_){
    if(!enable ){
      brakeOutput_=0;  //brake locked when 0 . TODO: Apply brake soke cycles before enable is low
    }
    else{
      brakeOutput_=1;  //brake open when 1
    }
  }

  enableOutput_=enable;
  return 0;
}

void ecmcDrive::setInterlock(bool interlock)
{
  interlock_=interlock;
}

bool ecmcDrive::getInterlock()
{
  return interlock_;
}

int ecmcDrive::setVelSetRaw(int vel)
{
  velSetRawOutput_=vel;
  return 0;
}

int ecmcDrive::validate()
{
  int errorCode=validateEntry(0); //Enable entry output
  if(errorCode){
    return setErrorID(errorCode);
  }

  errorCode=validateEntry(1); //Velocity Setpoint entry output
  if(errorCode){
    return setErrorID(errorCode);
  }

  errorCode=validateEntry(2); //Enabled entry input
  if(errorCode){
    return setErrorID(errorCode);
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

int ecmcDrive::setEnableBrake(bool enable)
{
  if(enable){
    int errorCode=validateEntry(3); //brake output
    if(errorCode){
      enableBrake_=false;
      return setErrorID(ERROR_DRV_BRAKE_ENTRY_NULL);
    }
  }
  enableBrake_=enable;
  return 0;
}

int ecmcDrive::setEnableReduceTorque(bool enable)
{
  if(enable){
    int errorCode=validateEntry(4); //brake output
    if(errorCode){
	enableReduceTorque_=false;
      return setErrorID(ERROR_DRV_REDUCE_TORQUE_ENTRY_NULL);
    }
  }
  enableReduceTorque_=enable;

  return 0;
}

int ecmcDrive::getEnableBrake()
{
  return enableBrake_;
}

int ecmcDrive::getEnableReduceTorque()
{
  return enableReduceTorque_;
}

int ecmcDrive::setBrake(bool value)
{
  brakeOutput_=value;
  return 0;
}

int ecmcDrive::setReduceTorque(bool value)
{
  reduceTorqueOutput_=value;
  return 0;
}

int ecmcDrive::setAtTarget(bool atTarget)
{
  reduceTorqueOutput_=atTarget && enableReduceTorque_;
  return 0;
}
