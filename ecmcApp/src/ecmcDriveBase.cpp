#include "ecmcDriveBase.hpp"

ecmcDriveBase::ecmcDriveBase()
{
  initVars();
}

ecmcDriveBase::ecmcDriveBase(double scale)
{
  initVars();
  scale_=scale;
}

void ecmcDriveBase::initVars()
{
  errorReset();
  interlock_=true;
  scale_=0;
  scaleNum_=0;
  scaleDenom_=0;
  velSetRawOutput_=0;
  velSet_=0;
  enableCmd_=0;
  enabledStatus_=0;
  enableBrake_=0;
  enableReduceTorque_=0;
  reduceTorqueOutput_=0;
  brakeOutput_=0;
  controlWord_=0;
  statusWord_=0;
}

ecmcDriveBase::~ecmcDriveBase()
{
  ;
}

int ecmcDriveBase::setVelSet(double vel)
{
  if(interlock_){
    velSet_=0;
    velSetRawOutput_=0;
    enableCmd_=0;
    return 0;
  }
  velSet_=vel;
  velSetRawOutput_=velSet_/scale_;
  return 0;
}

void ecmcDriveBase::setScaleNum(double scaleNum)
{
  scaleNum_=scaleNum;
  if(std::abs(scaleDenom_)>0){
    scale_=scaleNum_/scaleDenom_;
  }
}

int ecmcDriveBase::setScaleDenom(double scaleDenom)
{
  scaleDenom_=scaleDenom;
  if(scaleDenom_==0){
    return setErrorID(ERROR_DRV_SCALE_DENOM_ZERO);
  }
  scale_=scaleNum_/scaleDenom_;
  return 0;
}

double ecmcDriveBase::getScale()
{
  return scale_;
}

double ecmcDriveBase::getVelSet()
{
  return velSet_;
}

int ecmcDriveBase::getVelSetRaw()
{
  return velSetRawOutput_;
}

/*int ecmcDriveBase::setEnable(bool enable)
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
}*/

void ecmcDriveBase::setInterlock(bool interlock)
{
  interlock_=interlock;
}

bool ecmcDriveBase::getInterlock()
{
  return interlock_;
}

int ecmcDriveBase::setVelSetRaw(int vel)
{
  velSetRawOutput_=vel;
  return 0;
}

int ecmcDriveBase::setEnableBrake(bool enable)
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

int ecmcDriveBase::setEnableReduceTorque(bool enable)
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

int ecmcDriveBase::getEnableBrake()
{
  return enableBrake_;
}

int ecmcDriveBase::getEnableReduceTorque()
{
  return enableReduceTorque_;
}

int ecmcDriveBase::setBrake(bool value)
{
  brakeOutput_=value;
  return 0;
}

int ecmcDriveBase::setReduceTorque(bool value)
{
  reduceTorqueOutput_=value;
  return 0;
}

int ecmcDriveBase::setAtTarget(bool atTarget)
{
  reduceTorqueOutput_=atTarget && enableReduceTorque_;
  return 0;
}

void ecmcDriveBase::writeEntries()
{
  if (getError()){
    return;
  }

  int errorCode=0;
  errorCode=writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD,(uint64_t)controlWord_); //will only write the number of bits configured in st.cmd file defined by link commands
  if(errorCode){
    setErrorID(errorCode);
  }

  errorCode=writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT,(uint64_t)velSetRawOutput_);
  if(errorCode){
    setErrorID(errorCode);
  }

  if(enableBrake_){
    errorCode=writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT,(uint64_t)brakeOutput_);
    if(errorCode){
      setErrorID(errorCode);
    }
  }

  if(enableReduceTorque_){
    errorCode=writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT,(uint64_t)reduceTorqueOutput_);
    if(errorCode){
      setErrorID(errorCode);
    }
  }
}

void ecmcDriveBase::readEntries()
{
  if(getError()){
    return;
  }
  if(readEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD,&statusWord_)){
    setErrorID(ERROR_DRV_ENABLED_READ_ENTRY_FAIL);
    enableCmd_=false;
    enabledStatus_=false;
    return;
  }
}

int ecmcDriveBase::validate()
{
  int errorCode=validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD); //Enable entry output OR controlword
  if(errorCode){
    return setErrorID(errorCode);
  }

  errorCode=validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT); //Velocity Setpoint entry output
  if(errorCode){
    return setErrorID(errorCode);
  }

  errorCode=validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD); //Enabled entry input OR statusword
  if(errorCode){
    return setErrorID(errorCode);
  }

  if(enableBrake_){
    errorCode=validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT); //brake output
    if(errorCode){
      return setErrorID(errorCode);
    }
  }

  if(enableReduceTorque_){
    errorCode=validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT); //reduce torque output
    if(errorCode){
      return setErrorID(errorCode);
    }
  }

  if(scaleDenom_==0){
    return setErrorID(ERROR_DRV_SCALE_DENOM_ZERO);
  }
  return 0;
}

bool ecmcDriveBase::getEnable()
{
  return enableCmd_;
}

bool ecmcDriveBase::getEnabled()
{
  return enabledStatus_;
}

int ecmcDriveBase::setEnable(bool enable)
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
