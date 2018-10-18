/*
 * cMcuAxisReal.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: anderssandstrom
 */

#include "ecmcAxisReal.h"

ecmcAxisReal::ecmcAxisReal(int axisID, double sampleTime) :  ecmcAxisBase(axisID,sampleTime)
{
  PRINT_ERROR_PATH("axis[%d].error",axisID);
  initVars();
  data_.axisId_=axisID;
  data_.axisType_=ECMC_AXIS_TYPE_REAL;
  data_.sampleTime_=sampleTime;
  currentDriveType_=ECMC_STEPPER;

  LOGINFO15("%s/%s:%d: axis[%d]=new;\n",__FILE__, __FUNCTION__, __LINE__,axisID);

  drv_=new ecmcDriveStepper(&data_);
  if(!drv_){
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR DRIVE OBJECT.\n",__FILE__,__FUNCTION__,__LINE__);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_DRV_OBJECT_NULL);
    exit(EXIT_FAILURE);
  }

  cntrl_=new ecmcPIDController(&data_,data_.sampleTime_);
  if(!cntrl_){
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR PID-CONTROLLER OBJECT.\n",__FILE__,__FUNCTION__,__LINE__);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_CNTRL_OBJECT_NULL);
    exit(EXIT_FAILURE);
  }

  seq_.setCntrl(cntrl_);
  printCurrentState();
}

ecmcAxisReal::~ecmcAxisReal()
{
  delete cntrl_;
  delete drv_;
}

void ecmcAxisReal::printCurrentState()
{
  ecmcAxisBase::printCurrentState();
  LOGINFO15("%s/%s:%d: axis[%d].type=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_.axisId_,"ECMC_AXIS_TYPE_REAL");
  LOGINFO15("%s/%s:%d: axis[%d].sampleTime=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_.axisId_,data_.sampleTime_);
  printDriveType();
  printOpModeState();
}

void ecmcAxisReal::printOpModeState()
{
  switch(data_.command_.operationModeCmd){
    case ECMC_MODE_OP_AUTO:
      LOGINFO15("%s/%s:%d: axis[%d].operationMode=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_.axisId_,"ECMC_MODE_OP_AUTO");
      break;
    case ECMC_MODE_OP_MAN:
      LOGINFO15("%s/%s:%d: axis[%d].operationMode=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_.axisId_,"ECMC_MODE_OP_MAN");
      break;
    default:
      LOGINFO15("%s/%s:%d: axis[%d].operationMode=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_.axisId_,data_.command_.operationModeCmd);
      break;
  }
}

void ecmcAxisReal::printDriveType()
{
  switch(currentDriveType_){
    case ECMC_STEPPER:
      LOGINFO15("%s/%s:%d: axis[%d].driveType=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_.axisId_,"ECMC_STEPPER");
      break;
    case ECMC_DS402:
      LOGINFO15("%s/%s:%d: axis[%d].driveType=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_.axisId_,"ECMC_DS402");
      break;
    default:
      LOGINFO15("%s/%s:%d: axis[%d].driveType=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_.axisId_,currentDriveType_);
      break;
  }
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
    else{ //External source (Transform)currentVelocitySetpoint
      data_.status_.currentPositionActual=data_.status_.externalEncoderPosition;
      data_.status_.currentVelocityActual=data_.status_.externalEncoderVelocity;
    }

    traj_->setStartPos(data_.status_.currentPositionSetpoint);

    seq_.execute();
    mon_->execute();

    //Switch to internal trajectory temporary if interlock
    bool trajLock=((data_.interlocks_.trajSummaryInterlockFWD && data_.status_.currentPositionSetpoint>data_.status_.currentPositionSetpointOld) || (data_.interlocks_.trajSummaryInterlockBWD && data_.status_.currentPositionSetpoint<data_.status_.currentPositionSetpointOld));
    if( trajLock && externalInputTrajectoryIF_->getDataSourceType()!=ECMC_DATA_SOURCE_INTERNAL){
      if(!temporaryLocalTrajSource_){//Initiate rampdown
	temporaryLocalTrajSource_=true;
        traj_->setStartPos(data_.status_.currentPositionActual);
        traj_->initStopRamp(data_.status_.currentPositionActual,data_.status_.currentVelocityActual,0);
      }
      statusData_.onChangeData.trajSource=ECMC_DATA_SOURCE_INTERNAL; //Temporary
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
      double cntrOutput=0;
      if(mon_->getEnableAtTargetMon() && !data_.status_.busy && mon_->getAtTarget()){ //Controller deadband
	cntrl_->reset();
	cntrOutput=0;
      }
      else{
	cntrOutput=cntrl_->control(data_.status_.currentPositionSetpoint,data_.status_.currentPositionActual,data_.status_.currentVelocitySetpoint);
      }
      mon_->setEnable(true);
      drv_->setVelSet(cntrOutput); //Actual control
    }
    else{
      mon_->setEnable(false);
      if(getExecute()){
	setExecute(false);
      }

      if(!getEnable()){  //Only update if enable cmd is low to avoid change of setpoint during between enable and enabled
        data_.status_.currentPositionSetpoint=data_.status_.currentPositionActual;
        traj_->setStartPos(data_.status_.currentPositionSetpoint);
      }

      if(data_.status_.enabledOld && !data_.status_.enabled && data_.status_.enableOld && data_.command_.enable){
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

int ecmcAxisReal::setOpMode(operationMode mode)
{
  if(mode==ECMC_MODE_OP_MAN){
    data_.command_.enable=false;
    drv_->setVelSet(0);
  }

  data_.command_.operationModeCmd=mode;
  printOpModeState();
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

void ecmcAxisReal::refreshDebugInfoStruct()
{
  statusData_.onChangeData.atTarget=data_.status_.atTarget;
  statusData_.axisID=data_.axisId_;
  statusData_.cycleCounter=cycleCounter_;
  statusData_.onChangeData.busy=data_.status_.busy;
  statusData_.onChangeData.cntrlError=data_.status_.cntrlError;
  statusData_.onChangeData.cntrlOutput=data_.status_.cntrlOutput;
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
  statusData_.onChangeData.lastActiveInterlock=data_.interlocks_.lastActiveInterlock;
  statusData_.onChangeData.velocityActual=data_.status_.currentVelocityActual;
  statusData_.onChangeData.velocitySetpoint=data_.status_.currentVelocitySetpoint;
  statusData_.onChangeData.velocitySetpointRaw=data_.status_.currentVelocitySetpointRaw;
  statusData_.onChangeData.velocityFFRaw=data_.status_.currentvelocityFFRaw;
  statusData_.onChangeData.cmdData=data_.command_.cmdData;
  statusData_.onChangeData.command=data_.command_.command;
  statusData_.onChangeData.positionRaw=enc_->getRawPosMultiTurn();
  statusData_.onChangeData.homed=enc_->getHomed();
  statusData_.acceleration=traj_->getAcc();
  statusData_.deceleration=traj_->getDec();
  statusData_.reset=data_.command_.reset;
  statusData_.moving=data_.status_.moving;
  statusData_.stall=data_.interlocks_.lagTrajInterlock
      || data_.interlocks_.lagDriveInterlock
      || data_.interlocks_.velocityDiffTrajInterlock
      || data_.interlocks_.velocityDiffDriveInterlock;
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
      if(!drv_){
        LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR DRIVE OBJECT (%x).\n",__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_DRV_OBJECT_NULL);
        setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_DRV_OBJECT_NULL);
        exit(EXIT_FAILURE);
      }
      currentDriveType_=ECMC_STEPPER;
      break;
    case ECMC_DS402:
      delete drv_;
      drv_ =new ecmcDriveDS402(&data_);
      if(!drv_){
        LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR DRIVE OBJECT (%x).\n",__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_DRV_OBJECT_NULL);
        setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_DRV_OBJECT_NULL);
        exit(EXIT_FAILURE);
      }
      currentDriveType_=ECMC_DS402;
      break;
    default:
      LOGERR("%s/%s:%d: DRIVE TYPE NOT SUPPORTED (%x).\n",__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
      return ERROR_AXIS_FUNCTION_NOT_SUPPRTED;
      break;
  }
  printDriveType();
  return 0;
}
