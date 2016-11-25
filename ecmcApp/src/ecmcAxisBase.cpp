/*
 * cMcuAxisBase.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: anderssandstrom
 */

#include "ecmcAxisBase.h"

ecmcAxisBase::ecmcAxisBase()
{
  initVars();
  commandTransform_=new ecmcCommandTransform(2,ECMC_MAX_AXES);  //currently two commands
  commandTransform_->addCmdPrefix(TRANSFORM_EXPR_COMMAND_EXECUTE_PREFIX,ECMC_CMD_TYPE_EXECUTE);
  commandTransform_->addCmdPrefix(TRANSFORM_EXPR_COMMAND_ENABLE_PREFIX,ECMC_CMD_TYPE_ENABLE);
}

ecmcAxisBase::~ecmcAxisBase()
{
  delete commandTransform_;
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
    return setErrorID(ERROR_AXIS_INDEX_OUT_OF_RANGE);
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

int ecmcAxisBase::setEnableCommandsTransform(bool  enable)
{
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
        return setErrorID(error);
      }
    }
    else{
      error=commandTransform_->setData(0,ECMC_CMD_TYPE_EXECUTE,i);
      if(error){
        return setErrorID(error);
      }

    }
  }

  //Enable
  for(int i=0;i<ECMC_MAX_AXES;i++){
    if(axes_[i]!=NULL){
      error=commandTransform_->setData(axes_[i]->getEnable(),ECMC_CMD_TYPE_ENABLE,i);
      if(error){
        return setErrorID(error);
      }
    }
    else{
      error=commandTransform_->setData(0,ECMC_CMD_TYPE_ENABLE,i);
      if(error){
        return setErrorID(error);
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
      return setErrorID(ERROR_TRANSFORM_EXPR_NOT_COMPILED);
    }
    int error=fillCommandsTransformData();
    if(error){
      return setErrorID(error);
    }
    //Execute transform
    commandTransform_->refresh();

    //write changes to axes
    for(int i=0;i<ECMC_MAX_AXES;i++){
      if(commandTransform_==NULL){
	return setErrorID(ERROR_AXIS_INVERSE_TRANSFORM_NULL);
      }
      if(axes_[i]!=NULL){
        if(axes_[i]->getCascadedCommandsEnabled() && commandTransform_->getDataChanged(ECMC_CMD_TYPE_ENABLE,i) && i!=axisID_){ //Do not set on axis_no again
          int error= axes_[i]->setEnable(commandTransform_->getData(ECMC_CMD_TYPE_ENABLE,i));
          if(error){
            return setErrorID(error);
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
      printf("NOTCOMPILED**********Axis number%d",axisID_);
      return setErrorID(ERROR_TRANSFORM_EXPR_NOT_COMPILED);
    }

    int error=fillCommandsTransformData();
    if(error){
      return setErrorID(error);
    }

    //Execute transform
    commandTransform_->refresh();

    //write changes to axes
    for(int i=0;i<ECMC_MAX_AXES;i++){
      if(axes_[i]!=NULL){
        if(axes_[i]->getCascadedCommandsEnabled() && commandTransform_->getDataChanged(ECMC_CMD_TYPE_EXECUTE,i) && i!=axisID_){ //Do not set on axis_no again
          int error= axes_[i]->setExecute(commandTransform_->getData(ECMC_CMD_TYPE_EXECUTE,i));
          if(error){
            return setErrorID(error);
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
  return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

int ecmcAxisBase::setTrajTransformExpression(std::string expressionString)
{
  if(!getTraj()){
    return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
  }

  int error=getTraj()->getExtInputTransform()->setExpression(expressionString);
  if(error){
    return setErrorID(error);
  }
  return 0;
}

int ecmcAxisBase::setEncTransformExpression(std::string expressionString)
{
  if(!getEnc()){
    return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
  }

  int error=getEnc()->getExtInputTransform()->setExpression(expressionString);
  if(error){
    return setErrorID(error);
  }
  return 0;
}

int ecmcAxisBase::setTrajDataSourceType(dataSource refSource)
{
  if(!getTraj()){
    return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
  }

  //If realtime: Ensure that transform object is compiled and ready to go
  if(refSource!=ECMC_DATA_SOURCE_INTERNAL && realtime_){
    ecmcTransform * transform=getTraj()->getExtInputTransform();
    if(!transform){
      return setErrorID(ERROR_TRAJ_TRANSFORM_NULL);
    }
    int error =transform->validate();
    if(error){
      return setErrorID(error);
    }
  }

  int error=getTraj()->setDataSourceType(refSource);
  if(error){
    return setErrorID(error);
  }
  error=getTraj()->validate();
  if(error){
    return setErrorID(error);
  }
  return 0;
}

int ecmcAxisBase::setEncDataSourceType(dataSource refSource)
{
  if(!getEnc()){
    return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
  }

  //If realtime: Ensure that ethercat enty for actual position is linked
  if(refSource==ECMC_DATA_SOURCE_INTERNAL && realtime_){
    int error=getEnc()->validateEntry(ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION);
    if(error){
      return setErrorID(error);
    }
  }

  //If realtime: Ensure that transform object is compiled and ready to go
  if(refSource!=ECMC_DATA_SOURCE_INTERNAL && realtime_){
    ecmcTransform * transform=getEnc()->getExtInputTransform();
    if(!transform){
      return setErrorID(ERROR_TRAJ_TRANSFORM_NULL);
    }
    int error =transform->validate();
    if(error){
      return setErrorID(error);
    }
  }

  int error=getEnc()->setDataSourceType(refSource);
  if(error){
    return setErrorID(error);
  }
  if(realtime_) {
    error=getEnc()->validate();
    if(error){
      return setErrorID(error);
    }
  }
  return 0;
}

int ecmcAxisBase::setRealTimeStarted(bool realtime)
{
  realtime_=realtime;
}
