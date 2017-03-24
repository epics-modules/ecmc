#include "ecmcDriveBase.hpp"

ecmcDriveBase::ecmcDriveBase(ecmcAxisData *axisData)
{
  PRINT_ERROR_PATH("axis[%d].drive.error",axisData->axisId_);
  data_=axisData;
  initVars();
  if(!data_){
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n",__FILE__,__FUNCTION__,__LINE__);
    exit(EXIT_FAILURE);
  }

}

ecmcDriveBase::ecmcDriveBase(ecmcAxisData *axisData,double scale)
{
  PRINT_ERROR_PATH("axis[%d].drive.error",axisData->axisId_);
  data_=axisData;
  initVars();
  if(!data_){
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n",__FILE__,__FUNCTION__,__LINE__);
    exit(EXIT_FAILURE);
  }
  scale_=scale;

}
void ecmcDriveBase::printCurrentState()
{
  LOGINFO15("%s/%s:%d: axis[%d].drive.sampleTime=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,data_->sampleTime_);
}

void ecmcDriveBase::initVars()
{
  errorReset();
  scale_=0;
  scaleNum_=0;
  scaleDenom_=0;
  velSet_=0;
  enableBrake_=0;
  enableReduceTorque_=0;
  controlWord_=0;
  statusWord_=0;
  manualModeEnable_=false;
}

ecmcDriveBase::~ecmcDriveBase()
{
  ;
}

int ecmcDriveBase::setVelSet(double vel)
{
  if(!driveInterlocksOK()){
    velSet_=0;
    data_->status_.currentVelocitySetpointRaw=0;
    return 0;
  }
  velSet_=vel;
  data_->status_.currentVelocitySetpointRaw=velSet_/scale_;
  return 0;
}

void ecmcDriveBase::setScaleNum(double scaleNum)
{
  if(scaleNum_!=scaleNum){
    LOGINFO15("%s/%s:%d: axis[%d].drive.scaleNum=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,scaleNum);
  }
  scaleNum_=scaleNum;
  if(std::abs(scaleDenom_)>0){
    scale_=scaleNum_/scaleDenom_;
  }
}

int ecmcDriveBase::setScaleDenom(double scaleDenom)
{
  if(scaleDenom_!=scaleDenom){
    LOGINFO15("%s/%s:%d: axis[%d].drive.scaleDenom=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,scaleDenom);
  }
  scaleDenom_=scaleDenom;
  if(scaleDenom_==0){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DRV_SCALE_DENOM_ZERO);
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
  return data_->status_.currentVelocitySetpointRaw;
}

int ecmcDriveBase::setEnable(bool enable)
{
  //Only allowed in manual mode
  if(data_->command_.operationModeCmd!=ECMC_MODE_OP_MAN){
    manualModeEnable_=false;
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DRV_COMMAND_NOT_ALLOWED_IN_AUTO_MODE);
  }

  if(manualModeEnable_!=enable){
    LOGINFO15("%s/%s:%d: axis[%d].drive.manualModeEnable=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,enable);
  }

  manualModeEnable_=enable;
  return 0;
}

int ecmcDriveBase::setVelSetRaw(int vel)
{
  data_->status_.currentVelocitySetpointRaw=vel;
  return 0;
}

int ecmcDriveBase::setEnableBrake(bool enable)
{
  if(enable){
    int errorCode=validateEntry(3); //brake output
    if(errorCode){
      enableBrake_=false;
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DRV_BRAKE_ENTRY_NULL);
    }
  }

  if(enableBrake_!=enable){
    LOGINFO15("%s/%s:%d: axis[%d].drive.brakeEnable=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,enable);
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
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DRV_REDUCE_TORQUE_ENTRY_NULL);
    }
  }

  if(enableReduceTorque_!=enable){
    LOGINFO15("%s/%s:%d: axis[%d].drive.reduceTorqueEnable=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,enable);
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

void ecmcDriveBase::writeEntries()
{
  if(!driveInterlocksOK() && data_->command_.enable){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DRV_DRIVE_INTERLOCKED);
  }

  if(getError()){
    controlWord_=0;
    data_->status_.currentVelocitySetpointRaw=0;
  }

  int errorCode=0;
  errorCode=writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD,(uint64_t)controlWord_); //will only write the number of bits configured in st.cmd file defined by link commands
  if(errorCode){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  errorCode=writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT,(uint64_t)data_->status_.currentVelocitySetpointRaw);
  if(errorCode){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  if(enableBrake_){
    errorCode=writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT,(uint64_t) data_->command_.enable);
    if(errorCode){
      setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
    }
  }

  if(enableReduceTorque_){
    errorCode=writeEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT,(uint64_t)data_->status_.atTarget);
    if(errorCode){
      setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
    }
  }
}

void ecmcDriveBase::readEntries()
{
  if(readEcEntryValue(ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD,&statusWord_)){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DRV_ENABLED_READ_ENTRY_FAIL);
    statusWord_=0;
    return;
  }
}

int ecmcDriveBase::validate()
{
  int errorCode=validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD); //Enable entry output OR controlword
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  errorCode=validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT); //Velocity Setpoint entry output
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  errorCode=validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD); //Enabled entry input OR statusword
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  if(enableBrake_){
    errorCode=validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT); //brake output
    if(errorCode){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
    }
  }

  if(enableReduceTorque_){
    errorCode=validateEntry(ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT); //reduce torque output
    if(errorCode){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
    }
  }

  if(scaleDenom_==0){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DRV_SCALE_DENOM_ZERO);
  }
  return 0;
}

bool ecmcDriveBase::getEnable()
{
  switch(data_->command_.operationModeCmd){
    case ECMC_MODE_OP_AUTO:
      return data_->command_.enable;
      break;
    case ECMC_MODE_OP_MAN:
      return manualModeEnable_;
      break;
  }
  return data_->command_.enable;
}

bool ecmcDriveBase::getEnabled()
{
  return data_->status_.enabled;
}

bool ecmcDriveBase::driveInterlocksOK()
{
 return !(data_->interlocks_.driveSummaryInterlock || data_->interlocks_.etherCatMasterInterlock);
}
