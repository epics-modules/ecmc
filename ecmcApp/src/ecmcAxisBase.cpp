/*
 * cMcuAxisBase.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: anderssandstrom
 */

#include "ecmcAxisBase.h"

ecmcAxisBase::ecmcAxisBase(int axisID, double sampleTime)
{
  initVars();
  sampleTime_=sampleTime;
  axisID_=axisID;

  commandTransform_=new ecmcCommandTransform(2,ECMC_MAX_AXES);  //currently two commands
  commandTransform_->addCmdPrefix(TRANSFORM_EXPR_COMMAND_EXECUTE_PREFIX,ECMC_CMD_TYPE_EXECUTE);
  commandTransform_->addCmdPrefix(TRANSFORM_EXPR_COMMAND_ENABLE_PREFIX,ECMC_CMD_TYPE_ENABLE);

  externalInputTrajectoryIF_=new ecmcMasterSlaveIF(axisID_,ECMC_TRAJECTORY_INTERFACE,sampleTime_);
  externalInputEncoderIF_=new ecmcMasterSlaveIF(axisID_,ECMC_ENCODER_INTERFACE,sampleTime_);
}

ecmcAxisBase::~ecmcAxisBase()
{
  delete commandTransform_;
  delete externalInputEncoderIF_;
  delete externalInputTrajectoryIF_;
}

axisType ecmcAxisBase::getAxisType()
{
  return axisType_;
}

int ecmcAxisBase::getAxisID()
{
  return axisID_;
}

void ecmcAxisBase::setReset(bool reset)
{
  reset_=reset;
  if(reset_){
    errorReset();
    if(getMon()!=NULL){
      getMon()->errorReset();
    }
    if(getEnc()!=NULL){
      getEnc()->errorReset();
    }
    if(getTraj()!=NULL){
      getTraj()->errorReset();
    }
    if(getSeq()!=NULL){
      getSeq()->errorReset();
    }
    if(getDrv()!=NULL){
      getDrv()->errorReset();
    }
    if(getCntrl()!=NULL){
      getCntrl()->errorReset();
    }
  }
}

bool ecmcAxisBase::getReset()
{
  return reset_;
}

void ecmcAxisBase::initVars()
{
  axisID_=0;
  axisType_=ECMC_AXIS_TYPE_BASE;
  reset_=false;
  cascadedCommandsEnable_=false;
  enableCommandTransform_=false;
  inStartupPhase_=false;
  for(int i=0; i<ECMC_MAX_AXES;i++){
    axes_[i]=NULL;
  }
  realtime_=false;
  externalExecute_=false;

  externalTrajectoryPosition_=0;
  externalTrajectoryVelocity_=0;
  externalTrajectoryInterlock_=ECMC_INTERLOCK_EXTERNAL;

  externalEncoderPosition_=0;
  externalEncoderVelocity_=0;
  externalEncoderInterlock_=ECMC_INTERLOCK_EXTERNAL;

  currentPositionActual_=0;
  currentPositionSetpoint_=0;
  currentVelocityActual_=0;
  currentVelocitySetpoint_=0;

  sampleTime_=1/1000;
}

int ecmcAxisBase::setEnableCascadedCommands(bool enable)
{
  cascadedCommandsEnable_=enable;
  return 0;
}

bool ecmcAxisBase::getCascadedCommandsEnabled()
{
  return cascadedCommandsEnable_;
}

int ecmcAxisBase::setAxisArrayPointer(ecmcAxisBase *axis,int index)
{
  if(index>=ECMC_MAX_AXES || index<0){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_INDEX_OUT_OF_RANGE);
  }
  axes_[index]=axis;
  return 0;
}

bool ecmcAxisBase::checkAxesForEnabledTransfromCommands(commandType type)
{
  for(int i=0;i<ECMC_MAX_AXES;i++){
    if(axes_[i]!=NULL){
     if(axes_[i]->getCascadedCommandsEnabled()){
       return true;
     }
    }
  }
  return false;
}

int ecmcAxisBase::setEnableCommandsTransform(bool enable)
{
  if(realtime_){
    if(enable){
      int error=commandTransform_->validate();
      if(error){
	return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
      }
    }
  }
  enableCommandTransform_=enable;
  return 0;
}

bool ecmcAxisBase::getEnableCommandsTransform()
{
  return enableCommandTransform_;
}

int ecmcAxisBase::fillCommandsTransformData()
{
  int error=0;
  //Execute
  for(int i=0;i<ECMC_MAX_AXES;i++){
    if(axes_[i]!=NULL){
      error=commandTransform_->setData(axes_[i]->getExecute(),ECMC_CMD_TYPE_EXECUTE,i);
      if(error){
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
      }
    }
    else{
      error=commandTransform_->setData(0,ECMC_CMD_TYPE_EXECUTE,i);
      if(error){
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
      }

    }
  }

  //Enable
  for(int i=0;i<ECMC_MAX_AXES;i++){
    if(axes_[i]!=NULL){
      error=commandTransform_->setData(axes_[i]->getEnable(),ECMC_CMD_TYPE_ENABLE,i);
      if(error){
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
      }
    }
    else{
      error=commandTransform_->setData(0,ECMC_CMD_TYPE_ENABLE,i);
      if(error){
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
      }
    }
  }
  return 0;
}

int ecmcAxisBase::setCommandsTransformExpression(std::string expression)
{
  return commandTransform_->setExpression(expression);
}

int ecmcAxisBase::setEnable_Transform()
{
  if(checkAxesForEnabledTransfromCommands(ECMC_CMD_TYPE_ENABLE) && enableCommandTransform_){  //Atleast one axis have enabled getting execute from transform
    if(!commandTransform_->getCompiled()){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_TRANSFORM_ERROR_OR_NOT_COMPILED);
    }
    int error=fillCommandsTransformData();
    if(error){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }
    //Execute transform
    commandTransform_->refresh();

    //write changes to axes
    for(int i=0;i<ECMC_MAX_AXES;i++){
      if(commandTransform_==NULL){
	return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_INVERSE_TRANSFORM_NULL);
      }
      if(axes_[i]!=NULL){
        if(axes_[i]->getCascadedCommandsEnabled() && commandTransform_->getDataChanged(ECMC_CMD_TYPE_ENABLE,i) && i!=axisID_){ //Do not set on axis_no again
          int error= axes_[i]->setEnable(commandTransform_->getData(ECMC_CMD_TYPE_ENABLE,i));
          if(error){
            return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
          }
        }
      }
    }
  }
  return 0;
}

int ecmcAxisBase::setExecute_Transform()
{
  if(checkAxesForEnabledTransfromCommands(ECMC_CMD_TYPE_EXECUTE) && enableCommandTransform_){  //Atleast one axis have enabled getting execute from transform

    if(!commandTransform_->getCompiled()){
      LOGINFO7("%s/%s:%d: Error: Command transform not compiled for axis %d.\n",__FILE__, __FUNCTION__, __LINE__,axisID_);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_TRANSFORM_ERROR_OR_NOT_COMPILED);
    }

    int error=fillCommandsTransformData();
    if(error){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }

    //Execute transform
    commandTransform_->refresh();

    //write changes to axes
    for(int i=0;i<ECMC_MAX_AXES;i++){
      if(axes_[i]!=NULL){
        if(axes_[i]->getCascadedCommandsEnabled() && commandTransform_->getDataChanged(ECMC_CMD_TYPE_EXECUTE,i) && i!=axisID_){ //Do not set on axis_no again
          //int error= axes_[i]->setExecute(commandTransform_->getData(ECMC_CMD_TYPE_EXECUTE,i));
          int error= axes_[i]->setExternalExecute(commandTransform_->getData(ECMC_CMD_TYPE_EXECUTE,i));
          if(error){
            return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
          }
        }
      }
    }
  }
  return 0;
}

ecmcCommandTransform *ecmcAxisBase::getCommandTransform()
{
  return commandTransform_;
}

void ecmcAxisBase::setInStartupPhase(bool startup)
{
  inStartupPhase_=startup;
}

int ecmcAxisBase::setDriveType(ecmcDriveTypes driveType)
{
  return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

int ecmcAxisBase::setTrajTransformExpression(std::string expressionString)
{
   ecmcCommandTransform *transform=externalInputTrajectoryIF_->getExtInputTransform();
  if(!transform){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
  }

  int error=transform->setExpression(expressionString);
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  error=externalInputTrajectoryIF_->validate();
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  return 0;
}

int ecmcAxisBase::setEncTransformExpression(std::string expressionString)
{
  ecmcCommandTransform *transform=externalInputEncoderIF_->getExtInputTransform();
  if(!transform){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
  }

  int error=transform->setExpression(expressionString);
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  error=externalInputEncoderIF_->validate();
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  return 0;
}

int ecmcAxisBase::setTrajDataSourceType(dataSource refSource)
{
  if(getEnable()){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED);
  }

  //If realtime: Ensure that transform object is compiled and ready to go
  if(refSource!=ECMC_DATA_SOURCE_INTERNAL && realtime_){
    ecmcCommandTransform * transform=externalInputTrajectoryIF_->getExtInputTransform();
    if(!transform){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRAJ_TRANSFORM_NULL);
    }
    int error =transform->validate();
    if(error){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }
  }

  int error =externalInputTrajectoryIF_->validate(refSource); //Check if object is ok to go to refSource
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  externalInputTrajectoryIF_->setDataSourceType(refSource);
  return 0;
}

int ecmcAxisBase::setEncDataSourceType(dataSource refSource)
{
  if(getEnable()){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED);
  }

  //If realtime: Ensure that ethercat enty for actual position is linked
  if(refSource==ECMC_DATA_SOURCE_INTERNAL && realtime_){
    int error=getEnc()->validateEntry(ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION);
    if(error){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }
  }

  //If realtime: Ensure that transform object is compiled and ready to go
  if(refSource!=ECMC_DATA_SOURCE_INTERNAL && realtime_){
    ecmcCommandTransform * transform=externalInputEncoderIF_->getExtInputTransform();
    if(!transform){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRAJ_TRANSFORM_NULL);
    }
    int error =transform->validate();
    if(error){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }
  }

  int error =externalInputEncoderIF_->validate(refSource); //Check if object is ok to go to refSource
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  externalInputEncoderIF_->setDataSourceType(refSource);
  return 0;
}

int ecmcAxisBase::setRealTimeStarted(bool realtime)
{
  realtime_=realtime;
  return 0;
}

bool ecmcAxisBase::getError()
{
  int error= ecmcAxisBase::getErrorID();
  if(error){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }
  return ecmcError::getError();
}

int ecmcAxisBase::getErrorID()
{
  //General
  if(ecmcError::getError()){
    return ecmcError::getErrorID();
  }

  //Monitor
  ecmcMonitor *mon =getMon();
  if(mon){
    if(mon->getError()){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,mon->getErrorID());
    }
  }

  //Encoder
  ecmcEncoder *enc =getEnc();
  if(enc){
    if(enc->getError()){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,enc->getErrorID());
    }
  }

  //Drive
  ecmcDriveBase *drv =getDrv();
  if(drv){
    if(drv->getError()){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,drv->getErrorID());
    }
  }

  //Trajectory
  ecmcTrajectoryTrapetz *traj =getTraj();
  if(traj){
    if(traj->getError()){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,traj->getErrorID());
    }
  }

  //Controller
  ecmcPIDController *cntrl =getCntrl();
  if(cntrl){
    if(cntrl->getError()){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,cntrl->getErrorID());
    }
  }

  //Sequencer
  ecmcSequencer *seq =getSeq();
  if(seq){
    if(seq->getErrorID()){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,seq->getErrorID());
    }
  }

  return ecmcError::getErrorID();
}

int ecmcAxisBase::setEnableLocal(bool enable)
{
  int error=0;
  ecmcDriveBase *drv =getDrv();
  if(drv){
    error=drv->setEnable(enable);
    if(error){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }
  }

  ecmcTrajectoryTrapetz *traj =getTraj();
  if(traj){
    traj->setEnable(enable);
  }

  ecmcMonitor *mon =getMon();
  if(mon){
    mon->setEnable(enable);
  }

 /* ecmcEncoder *enc =getEnc();
  if(enc){
    error=enc->setEnable(enable);
    if(error){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }
  }*/

  ecmcPIDController *cntrl =getCntrl();
  if(cntrl){
    cntrl->setEnable(enable);
  }

/*  ecmcSequencer *seq =getSeq();
  if(seq){
    error=seq->setEnable(enable);
    if(error){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }
  }*/
  enable_=enable;
  return 0;
}

void ecmcAxisBase::errorReset()
{
  //Monitor
  ecmcMonitor *mon =getMon();
  if(mon){
    mon->errorReset();
  }

  //Encoder
  ecmcEncoder *enc =getEnc();
  if(enc){
    enc->errorReset();
  }

  //Drive
  ecmcDriveBase *drv =getDrv();
  if(drv){
    drv->errorReset();
  }

  //Trajectory
  ecmcTrajectoryTrapetz *traj =getTraj();
  if(traj){
    traj->errorReset();
  }

  //Controller
  ecmcPIDController *cntrl =getCntrl();
  if(cntrl){
    cntrl->errorReset();
  }

  //Sequencer
  ecmcSequencer *seq =getSeq();
  if(seq){
    seq->errorReset();
  }

  ecmcError::errorReset();
}

int ecmcAxisBase::setExternalExecute(bool execute)
{
/*  ecmcSequencer *seq=getSeq();
  if(!seq){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_SEQ_OBJECT_NULL);
  }
  externalExecute_=execute;
  seq->setExternalExecute(execute);*/
  return 0;
}

int ecmcAxisBase::refreshExternalInputSources()
{
  //Trajectory

  int error=externalInputTrajectoryIF_->refreshInputs();
  if(error){
     return setErrorID(error);
  }
  externalTrajectoryPosition_=externalInputTrajectoryIF_->getInputPos();
  externalTrajectoryVelocity_=externalInputTrajectoryIF_->getInputVel();
  externalTrajectoryInterlock_=externalInputTrajectoryIF_->getInputIlock();

  //Encoder
  error=externalInputEncoderIF_->refreshInputs();
  if(error){
     return setErrorID(error);
  }
  externalEncoderPosition_=externalInputEncoderIF_->getInputPos();
  externalEncoderVelocity_=externalInputEncoderIF_->getInputVel();
  externalEncoderInterlock_=externalInputEncoderIF_->getInputIlock();

  return 0;
 /* int error=0;

  //External Setpoint (Trajectory)
  if(externalInputTrajectoryIF_->getDataSourceType()!=ECMC_DATA_SOURCE_INTERNAL || externalInputTrajectoryIF_-> getInterlockDefined()){
    error=externalInputTrajectoryIF_->transformRefresh();
    if(error){
      setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }
    error=externalInputTrajectoryIF_->getExtInputPos(axisID_,ECMC_TRANSFORM_VAR_TYPE_TRAJ,&externalTrajectoryPosition_);
    if(error){
      setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }

    externalTrajectoryVelocity_=(externalTrajectoryPosition_-externalTrajectoryPositionOld_)/sampleTime_;
    externalTrajectoryPositionOld_=externalTrajectoryPosition_;

    if(!externalInputTrajectoryIF_->getExtInputInterlock(axisID_,ECMC_TRANSFORM_VAR_TYPE_IL)){ //1=OK, 0=STOP
      externalTrajectoryInterlock_=ECMC_INTERLOCK_TRANSFORM;
    }
    else{
	externalTrajectoryInterlock_= ECMC_INTERLOCK_NONE;
    }
  }
  else{
       }

  //External Actual value (Encoder)
  if(externalInputEncoderIF_->getDataSourceType()!=ECMC_DATA_SOURCE_INTERNAL || externalInputEncoderIF_->getInterlockDefined()){
    error=externalInputEncoderIF_->transformRefresh();
    if(error){
      setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }

    error=externalInputEncoderIF_->getExtInputPos(axisID_,ECMC_TRANSFORM_VAR_TYPE_ENC, &externalEncoderPosition_);
    if(error){
      setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }

    externalEncoderVelocity_=(externalEncoderPosition_-externalEncoderPositionOld_)/sampleTime_;
    externalEncoderPositionOld_=externalEncoderPosition_;

    if(!externalInputEncoderIF_->getExtInputInterlock(ax externalTrajectoryPosition_=0;
      externalTrajectoryPositionOld_=0;
      externalTrajectoryVelocity_=0;
      externalTrajectoryInterlock_=ECMC_INTERLOCK_NONE;
    isID_,ECMC_TRANSFORM_VAR_TYPE_IL)){ //1=OK, 0=STOP
      externalEncoderInterlock_= ECMC_INTERLOCK_TRANSFORM;
    }Trajectory
    else{
      externalEncoderInterlock_= ECMC_INTERLOCK_NONE;
    }
  }
  else{
    externalEncoderPosition_=0;
    externalEncoderVelocity_=0;
    externalEncoderInterlock_=ECMC_INTERLOCK_NONE;
  }

  if(getMon()){
    getMon()->setExtTrajInterlock(externalTrajectoryInterlock_);
    getMon()->setExtEncInterlock(externalEncoderInterlock_);
  }
  return 0;*/
}

int ecmcAxisBase::refreshExternalOutputSources()
{
  externalInputTrajectoryIF_->getOutputDataInterface()->setPosition(currentPositionSetpoint_);
  externalInputTrajectoryIF_->getOutputDataInterface()->setVelocity(currentVelocitySetpoint_);
  externalInputEncoderIF_->getOutputDataInterface()->setPosition(currentPositionActual_);
  externalInputEncoderIF_->getOutputDataInterface()->setVelocity(currentVelocityActual_);

  if(getMon()){
    bool il=getMon()->getTrajInterlock()==0;
    externalInputEncoderIF_->getOutputDataInterface()->setInterlock(il);
    externalInputTrajectoryIF_->getOutputDataInterface()->setInterlock(il);
  }
  return 0;
}

ecmcMasterSlaveIF *ecmcAxisBase::getExternalTrajIF()
{
  return externalInputTrajectoryIF_;
}

ecmcMasterSlaveIF *ecmcAxisBase::getExternalEncIF()
{
  return externalInputEncoderIF_;
}

int ecmcAxisBase::validateBase()
{
  int error=externalInputEncoderIF_->validate();
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  error=externalInputTrajectoryIF_->validate();
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  return 0;
}


