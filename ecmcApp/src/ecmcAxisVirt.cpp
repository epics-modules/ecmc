/*
 * cMcuAxisVirt.cpp
 *
 *  Created on: Mar 14, 2016
 *      Author: anderssandstrom
 */

#include "ecmcAxisVirt.h"

ecmcAxisVirt::ecmcAxisVirt(int axisID, double sampleTime) :  ecmcAxisBase(axisID,sampleTime)
{
  initVars();
  data_.axisType_=ECMC_AXIS_TYPE_VIRTUAL;
  seq_.setCntrl(NULL);
  data_.sampleTime_=sampleTime;
}

ecmcAxisVirt::~ecmcAxisVirt()
{
}

void ecmcAxisVirt::initVars()
{
  initDone_=false;
}

void ecmcAxisVirt::execute(bool masterOK)
{
  ecmcAxisBase::preExecute(masterOK);

  if(masterOK){

    //Trajectory (External or internal)
    if((externalInputTrajectoryIF_->getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL)/* || (externalInputTrajectoryIF_->getDataSourceType()!=ECMC_DATA_SOURCE_INTERNAL && mon_->getTrajInterlock())*/){
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
    data_.status_.cntrlOutput=0;
    mon_->execute();

    //Switch to internal trajectory if interlock temporary
    if(data_.interlocks_.trajSummaryInterlock && externalInputTrajectoryIF_->getDataSourceType()!=ECMC_DATA_SOURCE_INTERNAL){
      traj_->setStartPos(data_.status_.currentPositionActual);
      traj_->initStopRamp(data_.status_.currentPositionActual,data_.status_.currentVelocityActual,0);
      data_.status_.currentPositionSetpoint=traj_->getNextPosSet();
      data_.status_.currentVelocitySetpoint=traj_->getVel();
    }

    if(getEnabled() && masterOK && !getError()){
      mon_->setEnable(true);
      data_.status_.cntrlError=data_.status_.currentPositionSetpoint-data_.status_.currentPositionActual;
    }
    else{
      mon_->setEnable(false);
      if(getExecute()){
	setExecute(false);
      }
      data_.status_.currentPositionSetpoint=data_.status_.currentPositionActual;
      traj_->setStartPos(data_.status_.currentPositionSetpoint);
    }

    if(!masterOK){
      if(getEnabled() || getEnable()){
        setEnable(false);
      }
      setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_HARDWARE_STATUS_NOT_OK);
    }

    //Write to hardware
    refreshExternalOutputSources();
  }

  data_.status_.currentvelocityFFRaw=0;
  ecmcAxisBase::postExecute(masterOK);
}

int ecmcAxisVirt::setEnable(bool enable)
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

bool ecmcAxisVirt::getEnable()
{
  return traj_->getEnable();
}

bool ecmcAxisVirt::getEnabled()
{
  return getEnable();
}

int ecmcAxisVirt::setOpMode(operationMode mode)
{
  //NO DRIVE
  return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

operationMode ecmcAxisVirt::getOpMode()
{
  //NO DRIVE
  return ECMC_MODE_OP_AUTO;
}

int ecmcAxisVirt::getCntrlError(double* error)
{
  *error=traj_->getCurrentPosSet()-enc_->getActPos();
  return 0;
}

ecmcPIDController *ecmcAxisVirt::getCntrl()
{
  return NULL;
}

ecmcDriveBase *ecmcAxisVirt::getDrv()
{
  return NULL;
}

void ecmcAxisVirt::printStatus()
{
  printOutData_.atTarget=mon_->getAtTarget();
  printOutData_.axisID=data_.axisId_;
  printOutData_.busy=data_.status_.busy;
  printOutData_.cntrlError=0;
  printOutData_.cntrlOutput=0;
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
  printOutData_.velocitySetpointRaw=0;
  printOutData_.velocityFFRaw=0;

  if(memcmp(&printOutDataOld_,&printOutData_,sizeof(printOutData_))!=0){
    printAxisStatus(printOutData_);
  }

  printOutDataOld_=printOutData_;
  return;
}

int ecmcAxisVirt::validate()
{
  int error=0;
  if(enc_==NULL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_ENC_OBJECT_NULL);
  }

  if(externalInputEncoderIF_->getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL){
    error=enc_->validate();
    if(error){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }
  }

  if(traj_==NULL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_TRAJ_OBJECT_NULL);
  }

  error=traj_->validate();
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
