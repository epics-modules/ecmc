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
  data_.axisId_=axisID;

  LOGINFO15("%s/%s:%d: axis[%d]=new;\n",__FILE__, __FUNCTION__, __LINE__,axisID);
  LOGINFO15("%s/%s:%d: axis[%d].type=%s;\n",__FILE__, __FUNCTION__, __LINE__,axisID,"ECMC_AXIS_TYPE_VIRTUAL");
  LOGINFO15("%s/%s:%d: axis[%d].sampleTime=%lf;\n",__FILE__, __FUNCTION__, __LINE__,axisID,sampleTime);
}

ecmcAxisVirt::~ecmcAxisVirt()
{
}

void ecmcAxisVirt::initVars()
{
  initDone_=false;
  temporaryLocalTrajSource_=false;
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
      if(!temporaryLocalTrajSource_){//Initiate rampdown
	temporaryLocalTrajSource_=true;
	traj_->setStartPos(data_.status_.currentPositionActual);
	traj_->initStopRamp(data_.status_.currentPositionActual,data_.status_.currentVelocityActual,0);
      }
      statusData_.onChangeData.trajSource=ECMC_DATA_SOURCE_INTERNAL;  //Temporary
      data_.status_.currentPositionSetpoint=traj_->getNextPosSet();
      data_.status_.currentVelocitySetpoint=traj_->getVel();
    }
    else{
      temporaryLocalTrajSource_=false;
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

  //No drive object so update needed variables
  data_.status_.currentvelocityFFRaw=0;
  data_.status_.enabled=data_.command_.enable;

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

void ecmcAxisVirt::refreshDebugInfoStruct()
{
  statusData_.onChangeData.atTarget=mon_->getAtTarget();
  statusData_.axisID=data_.axisId_;
  statusData_.cycleCounter=cycleCounter_;
  statusData_.onChangeData.busy=data_.status_.busy;
  statusData_.onChangeData.cntrlError=0;
  statusData_.onChangeData.cntrlOutput=0;
  statusData_.onChangeData.enable=data_.command_.enable;
  statusData_.onChangeData.enabled=getEnabled();
  statusData_.onChangeData.error=getErrorID();
  statusData_.onChangeData.execute=getExecute();
  statusData_.onChangeData.homeSwitch=data_.status_.homeSwitch;
  statusData_.onChangeData.limitBwd=data_.status_.limitBwd;
  statusData_.onChangeData.limitFwd=data_.status_.limitFwd;
  statusData_.onChangeData.positionActual=data_.status_.currentPositionActual;
  statusData_.onChangeData.positionError=data_.status_.currentTargetPosition-data_.status_.currentPositionActual;
  statusData_.onChangeData.positionSetpoint=data_.status_.currentPositionSetpoint;
  statusData_.onChangeData.positionTarget=data_.status_.currentTargetPosition;
  statusData_.onChangeData.seqState=seq_.getSeqState();
  statusData_.onChangeData.trajInterlock=data_.interlocks_.interlockStatus;
  statusData_.onChangeData.velocityActual=data_.status_.currentVelocityActual;
  statusData_.onChangeData.velocitySetpoint=data_.status_.currentVelocitySetpoint;
  statusData_.onChangeData.velocitySetpointRaw=0;
  statusData_.onChangeData.velocityFFRaw=0;
  statusData_.onChangeData.positionRaw=enc_->getRawPos();
  statusData_.onChangeData.homed=enc_->getHomed();
  statusData_.acceleration=traj_->getAcc();
  statusData_.deceleration=traj_->getDec();
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
