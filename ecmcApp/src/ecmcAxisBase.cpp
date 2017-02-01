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

  data_.sampleTime_=sampleTime;
  data_.axisId_=axisID;
  data_.command_.operationModeCmd=ECMC_MODE_OP_AUTO;

  commandTransform_=new ecmcCommandTransform(2,ECMC_MAX_AXES);  //currently two commands
  commandTransform_->addCmdPrefix(TRANSFORM_EXPR_COMMAND_EXECUTE_PREFIX,ECMC_CMD_TYPE_EXECUTE);
  commandTransform_->addCmdPrefix(TRANSFORM_EXPR_COMMAND_ENABLE_PREFIX,ECMC_CMD_TYPE_ENABLE);

  externalInputTrajectoryIF_=new ecmcMasterSlaveIF(data_.axisId_,ECMC_TRAJECTORY_INTERFACE,data_.sampleTime_);
  externalInputEncoderIF_=new ecmcMasterSlaveIF(data_.axisId_,ECMC_ENCODER_INTERFACE,data_.sampleTime_);

  enc_=new ecmcEncoder(&data_,data_.sampleTime_);
  if(!enc_){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_ENC_OBJECT_NULL);
    return;
  }

  traj_=new ecmcTrajectoryTrapetz(&data_,data_.sampleTime_);
  if(!traj_){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_TRAJ_OBJECT_NULL);
    return;
  }

  mon_ =new ecmcMonitor(&data_);
  if(!mon_){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_MON_OBJECT_NULL);
    return;
  }

  seq_.setAxisDataRef(&data_);
  seq_.setTraj(traj_);
  seq_.setMon(mon_);
  seq_.setEnc(enc_);
  int error=seq_.setExtTrajIF(externalInputTrajectoryIF_);
  if(error){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_ASSIGN_EXT_INTERFACE_TO_SEQ_FAILED);
  }
}

ecmcAxisBase::~ecmcAxisBase()
{
  delete enc_;
  delete traj_;
  delete mon_;
  delete commandTransform_;
  delete externalInputEncoderIF_;
  delete externalInputTrajectoryIF_;
}

void ecmcAxisBase::preExecute(bool masterOK)
{
  data_.interlocks_.etherCatMasterInterlock=!masterOK;
  data_.refreshInterlocks();
  data_.status_.distToStop=traj_->distToStop(data_.status_.currentVelocitySetpoint);
  if(data_.command_.trajSource==ECMC_DATA_SOURCE_INTERNAL){
    data_.status_.busy=seq_.getBusy();
  }
  else{
    data_.status_.busy=true;
  }

  if(data_.status_.inStartupPhase && masterOK){
    //Auto reset hardware error if starting up
    if(getErrorID()==ERROR_AXIS_HARDWARE_STATUS_NOT_OK){
      errorReset();
    }
    setInStartupPhase(false);
  }
  mon_->readEntries();
  if(externalInputEncoderIF_->getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL){
    enc_->readEntries();
  }

  refreshExternalInputSources();
}

void ecmcAxisBase::postExecute(bool masterOK)
{
  data_.status_.enabledOld=data_.status_.enabled;
  data_.status_.enableOld=getEnable();
  data_.status_.executeOld=getExecute();
  data_.status_.currentPositionSetpointOld=data_.status_.currentPositionSetpoint;
  data_.status_.cntrlOutputOld=data_.status_.cntrlOutput;
}

axisType ecmcAxisBase::getAxisType()
{
  return data_.axisType_;
}

int ecmcAxisBase::getAxisID()
{
  return data_.axisId_;
}

void ecmcAxisBase::setReset(bool reset)
{
  data_.command_.reset=reset;
  if(reset){
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
  return data_.command_.reset;
}

void ecmcAxisBase::initVars()
{
  data_.axisType_=ECMC_AXIS_TYPE_BASE;
  data_.command_.reset=false;
  cascadedCommandsEnable_=false;
  enableCommandTransform_=false;
  data_.status_.inStartupPhase=false;
  for(int i=0; i<ECMC_MAX_AXES;i++){
    axes_[i]=NULL;
  }
  data_.status_.inRealtime=false;
  //externalExecute_=false;

  data_.status_.externalTrajectoryPosition=0;
  data_.status_.externalTrajectoryVelocity=0;
  data_.status_.externalTrajectoryInterlock=ECMC_INTERLOCK_EXTERNAL;

  data_.status_.externalEncoderPosition=0;
  data_.status_.externalEncoderVelocity=0;
  data_.status_.externalEncoderInterlock=ECMC_INTERLOCK_EXTERNAL;

  data_.status_.currentPositionActual=0;
  data_.status_.currentPositionSetpoint=0;
  data_.status_.currentVelocityActual=0;
  data_.status_.currentVelocitySetpoint=0;

  data_.sampleTime_=1/1000;
  memset(&printOutData_,0,sizeof(printOutData_));
  memset(&printOutDataOld_,0,sizeof(printOutDataOld_));
  printHeaderCounter_=0;
  data_.status_.enabledOld=false;
  data_.status_.enableOld=false;
  data_.status_.executeOld=false;
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
  if(data_.status_.inRealtime){
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
        if(axes_[i]->getCascadedCommandsEnabled() && commandTransform_->getDataChanged(ECMC_CMD_TYPE_ENABLE,i) && i!=data_.axisId_){ //Do not set on axis_no again
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
      LOGINFO7("%s/%s:%d: Error: Command transform not compiled for axis %d.\n",__FILE__, __FUNCTION__, __LINE__,data_.axisId_);
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
        if(axes_[i]->getCascadedCommandsEnabled() && commandTransform_->getDataChanged(ECMC_CMD_TYPE_EXECUTE,i) && i!=data_.axisId_){ //Do not set on axis_no again
          int error= axes_[i]->setExecute(commandTransform_->getData(ECMC_CMD_TYPE_EXECUTE,i));
          //int error= axes_[i]->setExternalExecute(commandTransform_->getData(ECMC_CMD_TYPE_EXECUTE,i));
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
  data_.status_.inStartupPhase=startup;
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
  if(getEnable() && refSource!=ECMC_DATA_SOURCE_INTERNAL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED);
  }

  //If realtime: Ensure that transform object is compiled and ready to go
  if(refSource!=ECMC_DATA_SOURCE_INTERNAL && data_.status_.inRealtime){
    ecmcCommandTransform * transform=externalInputTrajectoryIF_->getExtInputTransform();
    if(!transform){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRAJ_TRANSFORM_NULL);
    }
    int error =transform->validate();
    if(error){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }
  }

  int error=externalInputTrajectoryIF_->validate(refSource); //Check if object is ok to go to refSource
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }
  error=externalInputTrajectoryIF_->setDataSourceType(refSource);
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  if(refSource!=ECMC_DATA_SOURCE_INTERNAL){
    data_.interlocks_.noExecuteInterlock=false;
    data_.refreshInterlocks();
  }

  data_.command_.trajSource=refSource;
  return 0;
}

int ecmcAxisBase::setEncDataSourceType(dataSource refSource)
{
  if(getEnable()){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED);
  }

  //If realtime: Ensure that ethercat enty for actual position is linked
  if(refSource==ECMC_DATA_SOURCE_INTERNAL && data_.status_.inRealtime){
    int error=getEnc()->validateEntry(ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION);
    if(error){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }
  }

  //If realtime: Ensure that transform object is compiled and ready to go
  if(refSource!=ECMC_DATA_SOURCE_INTERNAL && data_.status_.inRealtime){
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

  error=externalInputEncoderIF_->setDataSourceType(refSource);
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }
  data_.command_.encSource=refSource;
  return 0;
}

int ecmcAxisBase::setRealTimeStarted(bool realtime)
{
  data_.status_.inRealtime=realtime;
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
  //GeneralsetErrorID
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

  ecmcTrajectoryTrapetz *traj =getTraj();
  if(traj){
    data_.status_.currentPositionSetpoint=data_.status_.currentPositionActual;
    traj->setStartPos(data_.status_.currentPositionSetpoint);
    traj->setEnable(enable);
  }

  data_.command_.enable=enable;
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

/*int ecmcAxisBase::setExternalExecute(bool execute)
{
  externalExecute_=execute;
  return 0;
}*/

int ecmcAxisBase::refreshExternalInputSources()
{
  //Trajectory

  int error=externalInputTrajectoryIF_->refreshInputs();
  if(error){
     return setErrorID(error);
  }
  data_.status_.externalTrajectoryPosition=externalInputTrajectoryIF_->getInputPos();
  data_.status_.externalTrajectoryVelocity=externalInputTrajectoryIF_->getInputVel();
  data_.interlocks_.trajTransformInterlock=externalInputTrajectoryIF_->getInputIlock();
  data_.refreshInterlocks();

  //Encoder
  error=externalInputEncoderIF_->refreshInputs();
  if(error){
     return setErrorID(error);
  }
  data_.status_.externalEncoderPosition=externalInputEncoderIF_->getInputPos();
  data_.status_.externalEncoderVelocity=externalInputEncoderIF_->getInputVel();
  data_.interlocks_.encTransformInterlock=externalInputEncoderIF_->getInputIlock();
  data_.refreshInterlocks();
  return 0;
}

int ecmcAxisBase::refreshExternalOutputSources()
{
  externalInputTrajectoryIF_->getOutputDataInterface()->setPosition(data_.status_.currentPositionSetpoint);
  externalInputTrajectoryIF_->getOutputDataInterface()->setVelocity(data_.status_.currentVelocitySetpoint);

  externalInputEncoderIF_->getOutputDataInterface()->setPosition(data_.status_.currentPositionActual);
  externalInputEncoderIF_->getOutputDataInterface()->setVelocity(data_.status_.currentVelocityActual);

  if(getMon()){
    externalInputEncoderIF_->getOutputDataInterface()->setInterlock(data_.interlocks_.trajSummaryInterlock);
    externalInputTrajectoryIF_->getOutputDataInterface()->setInterlock(data_.interlocks_.trajSummaryInterlock);
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

int ecmcAxisBase::getPosAct(double *pos)
{
  *pos=data_.status_.currentPositionActual;
  return 0;
}

int ecmcAxisBase::getVelAct(double *vel)
{
  *vel=data_.status_.currentVelocityActual;
  return 0;
}

int ecmcAxisBase::getPosSet(double *pos)
{
  if(externalInputTrajectoryIF_->getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL && getSeq()){
    *pos=data_.command_.positionTarget;
  }
  else{
    *pos=data_.status_.currentPositionSetpoint;
  }

  return 0;

}

ecmcEncoder *ecmcAxisBase::getEnc()
{
  return enc_;
}

ecmcTrajectoryTrapetz  *ecmcAxisBase::getTraj()
{
  return traj_;
}

ecmcMonitor *ecmcAxisBase::getMon()
{
  return mon_;
}

ecmcSequencer *ecmcAxisBase::getSeq()
{
  return &seq_;
}

int ecmcAxisBase::getAxisHomed(bool *homed)
{
  *homed=enc_->getHomed();
  return 0;
}

int ecmcAxisBase::getEncScaleNum(double *scale)
{
  *scale=enc_->getScaleNum();
  return 0;
}

int ecmcAxisBase::setEncScaleNum(double scale)
{
  enc_->setScaleNum(scale);
  return 0;
}

int ecmcAxisBase::getEncScaleDenom(double *scale)
{
  *scale=enc_->getScaleDenom();
  return 0;
}

int ecmcAxisBase::setEncScaleDenom(double scale)
{
  enc_->setScaleDenom(scale);
  return 0;
}

int ecmcAxisBase::getEncPosRaw(int64_t *rawPos)
{
  *rawPos=enc_->getRawPos();
  return 0;
}

int ecmcAxisBase::setCommand(motionCommandTypes command)
{
  seq_.setCommand(command);
  return 0;
}

int ecmcAxisBase::setCmdData(int cmdData)
{
  seq_.setCmdData(cmdData);
  return 0;
}

motionCommandTypes ecmcAxisBase::getCommand()
{
  return seq_.getCommand();
}

int ecmcAxisBase::getCmdData()
{
  return seq_.getCmdData();
}

void ecmcAxisBase::printAxisStatus(ecmcAxisStatusPrintOutType data)
{
  // Only print header once per 25 status lines
  if(printHeaderCounter_<=0){
    LOGINFO("\nAxis\tPos set\t\tPos act\t\tPos err\t\tCntrl out\tDist left\tVelAct\t\tVelFF\t\tVelFFraw\tVelDrvRaw\tError\tEn Ex Bu St Ta IL L+ L- Ho\n");
    printHeaderCounter_=25;
  }
  printHeaderCounter_--;

  LOGINFO("%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%i\t\t%x",
       data.axisID,
       data.positionSetpoint,
       data.positionActual,
       data.cntrlError,
       data.cntrlOutput,
       data.positionError,
       data.velocityActual,
       data.velocitySetpoint,
       data.velocityFFRaw,
       data.velocitySetpointRaw,
       data.error);

   LOGINFO("\t%d  %d  %d  %d  %d  %d  %d  %d  %d\n",
       data.enable,
       data.execute,
       data.busy,
       data.seqState,
       data.atTarget,
       data.trajInterlock,
       data.limitFwd,
       data.limitBwd,
       data.homeSwitch);
}

int ecmcAxisBase::setExecute(bool execute)
{
  //Internal trajectory source
  if(externalInputTrajectoryIF_->getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL){
    if(execute && !getEnable()){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_NOT_ENABLED);
    }

    if(execute && !data_.status_.executeOld && data_.status_.busy){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_BUSY);
    }

    int error=seq_.setExecute(execute);
    if(error){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
    }
  }
  return setExecute_Transform();
}

bool ecmcAxisBase::getExecute()
{
  if(externalInputTrajectoryIF_->getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL){
    return seq_.getExecute();
  }
  else{
    return true;
  }
}

bool ecmcAxisBase::getBusy()
{
  return data_.status_.busy && data_.status_.enabled;
}

int ecmcAxisBase::getDebugInfoData(ecmcAxisStatusPrintOutType *data)
{
  if(data==NULL){
    return ERROR_AXIS_DATA_POINTER_NULL;
  }

  memcpy(data,&printOutData_,sizeof(*data));
  return 0;
}
