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
  axisType_=ECMC_AXIS_TYPE_REAL;

  currentDriveType_=ECMC_STEPPER;
  drv_=new ecmcDriveStepper();
  if(!drv_){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_DRV_OBJECT_NULL);
  }
  cntrl_=new ecmcPIDController(sampleTime_);
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
  operationMode_=ECMC_MODE_OP_AUTO;
  sampleTime_=1;
  currentDriveType_=ECMC_STEPPER;
  enabledOld_=false;
  enableCmdOld_=false;
  executeCmdOld_=false;
  trajInterlockOld=true;
}

void ecmcAxisReal::execute(bool masterOK)
{
  if(operationMode_==ECMC_MODE_OP_AUTO){

    if(inStartupPhase_ && masterOK){
      //Auto reset hardware error if starting up
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
    drv_->readEntries();

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

    mon_->setCntrlOutput(cntrl_->getOutTot()); //From last scan
    mon_->execute();

    //Switch to internal trajectory temporary if interlock
    if(mon_->getTrajInterlock() && externalInputTrajectoryIF_->getDataSourceType()!=ECMC_DATA_SOURCE_INTERNAL){
      traj_->setInterlock(mon_->getTrajInterlock());
      traj_->setStartPos(currentPositionActual_);
      traj_->initStopRamp(currentPositionActual_,currentVelocityActual_,0);
      currentPositionSetpoint_=traj_->getNextPosSet();
      currentVelocitySetpoint_=traj_->getVel();
      mon_->setCurrentPosSet(currentPositionSetpoint_);
      mon_->setVelSet(currentVelocitySetpoint_);
    }

    traj_->setInterlock(mon_->getTrajInterlock());
    drv_->setInterlock(mon_->getDriveInterlock()); //TODO consider change logic so high interlock is OK and low not
    cntrl_->setInterlock(mon_->getDriveInterlock()); //TODO consider change logic so high interlock is OK and low not
    drv_->setAtTarget(mon_->getAtTarget());  //Reduce torque

    if(mon_->getDriveInterlock() && !traj_->getBusy()){
      cntrl_->reset();
    }

    if(getEnable() && masterOK /*&& !getError()*/){
      mon_->setEnable(true);
      drv_->setVelSet(cntrl_->control(currentPositionSetpoint_,currentPositionActual_,currentVelocitySetpoint_)); //Actual control
    }

    else{
      mon_->setEnable(false);
      if(getExecute()){
	setExecute(false);
      }
      currentPositionSetpoint_=currentPositionActual_;
      traj_->setStartPos(currentPositionSetpoint_);

      if(enabledOld_ && !drv_->getEnabled() && enableCmdOld_){
	  setEnable(false);
	  setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_AMPLIFIER_ENABLED_LOST);
      }
      drv_->setVelSet(0);
      cntrl_->reset();
    }

    if(!masterOK){
      if(getEnable()){
        setEnable(false);
      }
      cntrl_->reset();
      drv_->setVelSet(0);
      drv_->setInterlock(true);
      setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_HARDWARE_STATUS_NOT_OK);
    }

    //Write to hardware
    refreshExternalOutputSources();
    drv_->writeEntries();
  }
  else if(operationMode_==ECMC_MODE_OP_MAN){  //MANUAL MODE: Raw Output..
    mon_->readEntries();
    enc_->readEntries();
    if(!mon_->getHardLimitBwd() || !mon_->getHardLimitFwd()){ //PRIMITIVE CHECK FOR LIMIT SWITCHES
      drv_->setVelSet(0);
      drv_->setEnable(false);
    }
    drv_->writeEntries();
  }
  enabledOld_=drv_->getEnabled();
  enableCmdOld_=getEnable();
  executeCmdOld_=getExecute();
  trajInterlockOld=mon_->getTrajInterlock();
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
  return drv_->getEnable() && drv_->getEnabled() && traj_->getEnable() && cntrl_->getEnable() /*&& mon_->getEnable()*/;
}

int ecmcAxisReal::setOpMode(operationMode mode)
{
  if(mode==ECMC_MODE_OP_MAN){
    drv_->setEnable(false);
    drv_->setVelSet(0);
    drv_->setInterlock(false);
  }
  operationMode_=mode;
  return 0;
}

operationMode ecmcAxisReal::getOpMode()
{
  return operationMode_;
}

int ecmcAxisReal::getCntrlError(double* error)
{
  *error=cntrl_->getCntrlError();
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
  printOutData_.atTarget=mon_->getAtTarget();
  printOutData_.axisID=axisID_;
  printOutData_.busy=seq_.getBusy();
  printOutData_.cntrlError=cntrl_->getCntrlError();
  printOutData_.cntrlOutput=cntrl_->getOutTot();
  printOutData_.enable=getEnable();
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
  printOutData_.velocitySetpointRaw=drv_->getVelSetRaw();

  if(drv_->getScale()!=0){
    printOutData_.velocityFFRaw=cntrl_->getOutFFPart()/drv_->getScale();
  }
  else{
    printOutData_.velocityFFRaw=0;
  }

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
      drv_ =new ecmcDriveStepper();
      currentDriveType_=ECMC_STEPPER;
      break;
    case ECMC_DS402:
      delete drv_;
      drv_ =new ecmcDriveDS402();
      currentDriveType_=ECMC_DS402;
      break;
    default:
      return ERROR_AXIS_FUNCTION_NOT_SUPPRTED;
      break;
  }

  return 0;
}
