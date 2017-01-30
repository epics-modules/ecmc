/*
 * cMcuAxisReal.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: anderssandstrom
 */

#include "ecmcAxisReal.h"

ecmcAxisReal::ecmcAxisReal(int axisID, double sampleTime) :  ecmcAxisBase(axisID,sampleTime)
{
  initVars();

  data_.axisType_=ECMC_AXIS_TYPE_REAL;
  data_.sampleTime_=sampleTime;
  currentDriveType_=ECMC_STEPPER;
  drv_=new ecmcDriveStepper(&data_);
  if(!drv_){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_DRV_OBJECT_NULL);
  }

  cntrl_=new ecmcPIDController(&data_,data_.sampleTime_);
  if(!cntrl_){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_CNTRL_OBJECT_NULL);
  }

  seq_.setCntrl(cntrl_);
}

ecmcAxisReal::~ecmcAxisReal()
{
  delete cntrl_;
  delete drv_;
}

void ecmcAxisReal::initVars()
{
  initDone_=false;
  data_.command_.operationModeCmd=ECMC_MODE_OP_AUTO;
  currentDriveType_=ECMC_STEPPER;
  temporaryLocalTrajSource_=false;
}

void ecmcAxisReal::execute(bool masterOK)
{
  ecmcAxisBase::preExecute(masterOK);

  if(data_.command_.operationModeCmd==ECMC_MODE_OP_AUTO){

    drv_->readEntries();

    //Trajectory (External or internal)
    if((externalInputTrajectoryIF_->getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL)){
      data_.status_.currentPositionSetpoint=traj_->getNextPosSet();
      data_.status_.currentVelocitySetpoint=traj_->getVel();
    }
    else{ //External source (Transform)
      data_.status_.currentPositionSetpoint=data_.status_.externalTrajectoryPosition;
      data_.status_.currentVelocitySetpoint=data_.status_.externalTrajectoryVelocity;
      data_.interlocks_.noExecuteInterlock=false; //Only valid in local mode
      data_.refreshInterlocks();
    }

    //Encoder (External or internal)
    if(externalInputEncoderIF_->getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL){
      data_.status_.currentPositionActual=enc_->getActPos();
      data_.status_.currentVelocityActual=enc_->getActVel();
    }
    else{ //External source (Transform)
      data_.status_.currentPositionActual=data_.status_.externalEncoderPosition;
      data_.status_.currentVelocityActual=data_.status_.externalEncoderVelocity;
    }

    traj_->setStartPos(data_.status_.currentPositionSetpoint);
    seq_.execute();
    mon_->execute();

    //Switch to internal trajectory temporary if interlock
    if(data_.interlocks_.trajSummaryInterlock && externalInputTrajectoryIF_->getDataSourceType()!=ECMC_DATA_SOURCE_INTERNAL){
      if(!temporaryLocalTrajSource_){//Initiate rampdown
	temporaryLocalTrajSource_=true;
        traj_->setStartPos(data_.status_.currentPositionActual);
        traj_->initStopRamp(data_.status_.currentPositionActual,data_.status_.currentVelocityActual,0);
      }
      data_.status_.currentPositionSetpoint=traj_->getNextPosSet();
      data_.status_.currentVelocitySetpoint=traj_->getVel();
    }
    else{
      temporaryLocalTrajSource_=false;
    }

    if(data_.interlocks_.driveSummaryInterlock && !traj_->getBusy()){
      cntrl_->reset();
    }

    if(getEnabled() && masterOK){
      mon_->setEnable(true);
      drv_->setVelSet(cntrl_->control(data_.status_.currentPositionSetpoint,data_.status_.currentPositionActual,data_.status_.currentVelocitySetpoint)); //Actual control
    }
    else{
      mon_->setEnable(false);
      if(getExecute()){
	setExecute(false);
      }
      data_.status_.currentPositionSetpoint=data_.status_.currentPositionActual;
      traj_->setStartPos(data_.status_.currentPositionSetpoint);

      if(data_.status_.enabledOld && !data_.status_.enabled && data_.status_.enableOld){
	  setEnable(false);
	  setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_AMPLIFIER_ENABLED_LOST);
      }
      drv_->setVelSet(0);
      cntrl_->reset();
    }

    if(!masterOK){
      if(getEnabled() || getEnable()){
        setEnable(false);
      }
      cntrl_->reset();
      drv_->setVelSet(0);
      setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_HARDWARE_STATUS_NOT_OK);
    }

    //Write to hardware
    refreshExternalOutputSources();
    drv_->writeEntries();
  }
  else if(data_.command_.operationModeCmd==ECMC_MODE_OP_MAN){  //MANUAL MODE: Raw Output..
    mon_->readEntries();
    enc_->readEntries();
    if(!data_.status_.limitBwd || !data_.status_.limitFwd){ //PRIMITIVE CHECK FOR LIMIT SWITCHES
      drv_->setVelSet(0);
    }
    drv_->writeEntries();
  }

  if(std::abs(drv_->getScale())>0){
    data_.status_.currentvelocityFFRaw=cntrl_->getOutFFPart()/drv_->getScale();
  }
  else{
    data_.status_.currentvelocityFFRaw=0;
  }

  ecmcAxisBase::postExecute(masterOK);
}

int ecmcAxisReal::setEnable(bool enable)
{
  if(!enable){ //Remove execute if enable is going down
    setExecute(false);
  }

  if(enable && validate()){
    setExecute(false);
    return getErrorID();
  }

  int error=setEnableLocal(enable);
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  //Cascade commands via command transformation
  return setEnable_Transform();
}

bool ecmcAxisReal::getEnable()
{
  return data_.command_.enable;
}

bool ecmcAxisReal::getEnabled()
{
  return data_.status_.enabled && data_.command_.enable;
}

int ecmcAxisReal::setOpMode(operationMode mode)
{
  if(mode==ECMC_MODE_OP_MAN){
    data_.command_.enable=false;
    drv_->setVelSet(0);
  }
  data_.command_.operationModeCmd=mode;
  return 0;
}

operationMode ecmcAxisReal::getOpMode()
{
  return data_.command_.operationModeCmd;
}

int ecmcAxisReal::getCntrlError(double* error)
{
  *error=data_.status_.cntrlError;
  return 0;
}

ecmcPIDController *ecmcAxisReal::getCntrl()
{
  return cntrl_;
}

ecmcDriveBase *ecmcAxisReal::getDrv()
{
  return drv_;
}

void ecmcAxisReal::printStatus()
{
  printOutData_.atTarget=data_.status_.atTarget;
  printOutData_.axisID=data_.axisId_;
  printOutData_.busy=data_.status_.busy;
  printOutData_.cntrlError=data_.status_.cntrlError;
  printOutData_.cntrlOutput=data_.status_.cntrlOutput;
  printOutData_.enable=getEnabled();
  printOutData_.error=getErrorID();
  printOutData_.execute=getExecute();
  printOutData_.homeSwitch=data_.status_.homeSwitch;
  printOutData_.limitBwd=data_.status_.limitBwd;
  printOutData_.limitFwd=data_.status_.limitFwd;
  printOutData_.positionActual=data_.status_.currentPositionActual;
  printOutData_.positionError=data_.status_.currentPositionSetpoint-data_.status_.currentPositionActual;
  printOutData_.positionSetpoint=data_.status_.currentPositionSetpoint;
  printOutData_.seqState=seq_.getSeqState();
  printOutData_.trajInterlock=data_.interlocks_.interlockStatus;
  printOutData_.velocityActual=data_.status_.currentVelocityActual;
  printOutData_.velocitySetpoint=data_.status_.currentVelocitySetpoint;
  printOutData_.velocitySetpointRaw=data_.status_.currentVelocitySetpointRaw;
  printOutData_.velocityFFRaw=data_.status_.currentvelocityFFRaw;

  if(memcmp(&printOutDataOld_,&printOutData_,sizeof(printOutData_))!=0){
    printAxisStatus(printOutData_);
  }

  printOutDataOld_=printOutData_;
  return;
}

int ecmcAxisReal::validate()
{
  int error=0;
  if(enc_==NULL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_ENC_OBJECT_NULL);
  }

  error=enc_->validate();
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  if(traj_==NULL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_TRAJ_OBJECT_NULL);
  }

  error=traj_->validate();
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  if(drv_==NULL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_DRV_OBJECT_NULL);
  }

  error=drv_->validate();
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  if(mon_==NULL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_MON_OBJECT_NULL);
  }

  error=mon_->validate();
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  if(cntrl_==NULL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_CNTRL_OBJECT_NULL);
  }

  error=cntrl_->validate();
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  error=seq_.validate();
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  error=ecmcAxisBase::validateBase();
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  return 0;
}

int ecmcAxisReal::setDriveType(ecmcDriveTypes driveType)
{
  if(currentDriveType_==driveType){
    return 0;
  }
  switch(driveType){
    case ECMC_STEPPER:
      delete drv_;
      drv_ =new ecmcDriveStepper(&data_);
      currentDriveType_=ECMC_STEPPER;
      break;
    case ECMC_DS402:
      delete drv_;
      drv_ =new ecmcDriveDS402(&data_);
      currentDriveType_=ECMC_DS402;
      break;
    default:
      return ERROR_AXIS_FUNCTION_NOT_SUPPRTED;
      break;
  }

  return 0;
}
