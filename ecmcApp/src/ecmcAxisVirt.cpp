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
  axisType_=ECMC_AXIS_TYPE_VIRTUAL;
  seq_.setCntrl(NULL);
}

ecmcAxisVirt::~ecmcAxisVirt()
{
}

void ecmcAxisVirt::initVars()
{
  initDone_=false;
  sampleTime_=1;
}

void ecmcAxisVirt::execute(bool masterOK)
{
  if(masterOK){
    if(inStartupPhase_){
      //Auto reset hardware error
      if(getErrorID()==ERROR_AXIS_HARDWARE_STATUS_NOT_OK){
        errorReset();
      }
      setInStartupPhase(false);
    }

    //Read from hardware
    mon_->readEntries();
    if(externalInputEncoderIF_->getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL){
      enc_->readEntries();
    }

    refreshExternalInputSources();

    //Trajectory (External or internal)
    if((externalInputTrajectoryIF_->getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL)/* || (externalInputTrajectoryIF_->getDataSourceType()!=ECMC_DATA_SOURCE_INTERNAL && mon_->getTrajInterlock())*/){
      currentPositionSetpoint_=traj_->getNextPosSet();
      currentVelocitySetpoint_=traj_->getVel();
    }
    else{ //External source (Transform)
      currentPositionSetpoint_=externalTrajectoryPosition_;
      currentVelocitySetpoint_=externalTrajectoryVelocity_;
    }

    //Encoder (External or internal)
    if(externalInputEncoderIF_->getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL){
      currentPositionActual_=enc_->getActPos();
      currentVelocityActual_=enc_->getActVel();

    }
    else{ //External source (Transform)
      currentPositionActual_=externalEncoderPosition_;
      currentVelocityActual_=externalEncoderVelocity_;
    }

    mon_->setDistToStop(traj_->distToStop(currentVelocityActual_));

    traj_->setStartPos(currentPositionSetpoint_);
    mon_->setCurrentPosSet(currentPositionSetpoint_);
    mon_->setVelSet(currentVelocitySetpoint_);
    mon_->setActPos(currentPositionActual_);
    mon_->setActVel(currentVelocityActual_);
    mon_->setAxisErrorStateInterlock(getError());

    seq_.execute();

    mon_->setCntrlOutput(0); //From last scan
    mon_->execute();

    //Switch to internal trajectory if interlock temporary
    if(mon_->getTrajInterlock() && externalInputTrajectoryIF_->getDataSourceType()!=ECMC_DATA_SOURCE_INTERNAL){
      //externalInputTrajectoryIF_->setDataSourceType(ECMC_DATA_SOURCE_INTERNAL);
      traj_->setInterlock(mon_->getTrajInterlock());
      traj_->setStartPos(currentPositionActual_);
      traj_->initStopRamp(currentPositionActual_,currentVelocityActual_,0);
      currentPositionSetpoint_=traj_->getNextPosSet();
      currentVelocitySetpoint_=traj_->getVel();
      mon_->setCurrentPosSet(currentPositionSetpoint_);
      mon_->setVelSet(currentVelocitySetpoint_);
    }

    traj_->setInterlock(mon_->getTrajInterlock());

    if(getEnabled() && masterOK && !getError()){
      mon_->setEnable(true);
    }
    else{
      mon_->setEnable(false);
      if(getExecute()){
	setExecute(false);
      }
      currentPositionSetpoint_=currentPositionActual_;
      traj_->setStartPos(currentPositionSetpoint_);
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
  return  traj_->getEnable() /*&& mon_->getEnable()*/;
}

bool ecmcAxisVirt::getEnabled()
{
  return  getEnable() /*&& mon_->getEnable()*/;
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
  printOutData_.axisID=axisID_;
  printOutData_.busy=seq_.getBusy();
  printOutData_.cntrlError=0;
  printOutData_.cntrlOutput=0;
  printOutData_.enable=getEnabled();
  printOutData_.error=getErrorID();
  printOutData_.execute=traj_->getExecute();
  printOutData_.homeSwitch=mon_->getHomeSwitch();
  printOutData_.limitBwd=mon_->getHardLimitBwd();
  printOutData_.limitFwd=mon_->getHardLimitFwd();
  printOutData_.positionActual=currentPositionActual_;
  printOutData_.positionError=currentPositionSetpoint_ -currentPositionActual_;
  printOutData_.positionSetpoint=currentPositionSetpoint_;
  printOutData_.seqState=seq_.getSeqState();
  printOutData_.trajInterlock=mon_->getTrajInterlock();
  printOutData_.velocityActual=currentVelocityActual_;
  printOutData_.velocitySetpoint=currentVelocitySetpoint_;
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
