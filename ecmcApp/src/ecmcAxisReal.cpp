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
  axisID_=axisID;
  axisType_=ECMC_AXIS_TYPE_REAL;
  sampleTime_=sampleTime;
  enc_=new ecmcEncoder(sampleTime_);

  traj_=new ecmcTrajectoryTrapetz(sampleTime_);
  mon_ =new ecmcMonitor();
  currentDriveType_=ECMC_STEPPER;
  drv_=new ecmcDriveStepper();
  cntrl_=new ecmcPIDController(sampleTime_);

  seq_.setCntrl(cntrl_);
  seq_.setTraj(traj_);
  seq_.setMon(mon_);
  seq_.setEnc(enc_);
  int error=getSeq()->setExtTrajIF(externalInputTrajectoryIF_);
  if(error){
    setErrorID(ERROR_AXIS_ASSIGN_EXT_INTERFACE_TO_SEQ_FAILED);
  }
}

ecmcAxisReal::~ecmcAxisReal()
{
  delete cntrl_;
  delete enc_;
  delete traj_;
  delete mon_;
  delete drv_;
  delete externalInputTrajectoryIF_;
  delete externalInputTrajectoryIF_;
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

    if(!getEnable()){
      currentPositionSetpoint_=currentPositionActual_;
    }

    traj_->setStartPos(currentPositionSetpoint_);
    mon_->setCurrentPosSet(currentPositionSetpoint_);
    mon_->setVelSet(currentVelocitySetpoint_);
    mon_->setActPos(currentPositionActual_);
    mon_->setActVel(currentVelocityActual_);

    seq_.execute();

    mon_->setCntrlOutput(cntrl_->getOutTot()); //From last scan
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
    drv_->setInterlock(mon_->getDriveInterlock()); //TODO consider change logic so high interlock is OK and low not
    cntrl_->setInterlock(mon_->getDriveInterlock()); //TODO consider change logic so high interlock is OK and low not
    drv_->setAtTarget(mon_->getAtTarget());  //Reduce torque

    if(mon_->getDriveInterlock() && !traj_->getBusy()){
      cntrl_->reset();
    }

    if(getEnable() && masterOK && !getError()){
      mon_->setEnable(true);
      drv_->setVelSet(cntrl_->control(currentPositionSetpoint_,currentPositionActual_,currentVelocitySetpoint_)); //Actual control
    }

    else{
      mon_->setEnable(false);
      if(getExecute()){
	setExecute(false);
	traj_->setStartPos(0);
      }
      if(enabledOld_ && !drv_->getEnabled() && enableCmdOld_){
	  setEnable(false);
	  setErrorID(ERROR_AXIS_AMPLIFIER_ENABLED_LOST);
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
      setErrorID(ERROR_AXIS_HARDWARE_STATUS_NOT_OK);
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

int ecmcAxisReal::setExecute(bool execute)
{
  if(execute && !getEnable()){
    return setErrorID(ERROR_AXIS_NOT_ENABLED);
  }

  int error=seq_.setExecute(execute);
  if(error){
    return setErrorID(error);
  }

  return setExecute_Transform();
}

bool ecmcAxisReal::getExecute()
{
  return seq_.getExecute();
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
    return setErrorID(error);
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

int ecmcAxisReal::getActPos(double *pos)
{
  *pos=currentPositionActual_;
  return 0;
}

int ecmcAxisReal::getActVel(double *vel)
{
  *vel=currentVelocityActual_;
  return 0;
}

int ecmcAxisReal::getAxisHomed(bool *homed)
{
  *homed=enc_->getHomed();
  return 0;
}

int ecmcAxisReal::getEncScaleNum(double *scale)
{
  *scale=enc_->getScaleNum();
  return 0;
}

int ecmcAxisReal::setEncScaleNum(double scale)
{
  enc_->setScaleNum(scale);
  return 0;
}

int ecmcAxisReal::getEncScaleDenom(double *scale)
{
  *scale=enc_->getScaleDenom();
  return 0;
}

int ecmcAxisReal::setEncScaleDenom(double scale)
{
  enc_->setScaleDenom(scale);
  return 0;
}

int ecmcAxisReal::getCntrlError(double* error)
{
  *error=cntrl_->getCntrlError();
  return 0;
}

int ecmcAxisReal::getEncPosRaw(int64_t *rawPos)
{
  *rawPos=enc_->getRawPos();
  return 0;
}

int ecmcAxisReal::setCommand(motionCommandTypes command)
{
  seq_.setCommand(command);
  return 0;
}

int ecmcAxisReal::setCmdData(int cmdData)
{
  seq_.setCmdData(cmdData);
  return 0;
}

motionCommandTypes ecmcAxisReal::getCommand()
{
  return seq_.getCommand();
}

int ecmcAxisReal::getCmdData()
{
  return seq_.getCmdData();
}


ecmcEncoder *ecmcAxisReal::getEnc()
{
  return enc_;
}

ecmcPIDController *ecmcAxisReal::getCntrl()
{
  return cntrl_;
}

ecmcDriveBase *ecmcAxisReal::getDrv()
{
  return drv_;
}

ecmcTrajectoryTrapetz  *ecmcAxisReal::getTraj()
{
  return traj_;
}

ecmcMonitor *ecmcAxisReal::getMon()
{
  return mon_;
}

ecmcSequencer *ecmcAxisReal::getSeq()
{
  return &seq_;
}

void ecmcAxisReal::printStatus()
{
  if(drv_->getScale()!=0){
    printf("%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%i\t%x",
        axisID_,
        /*traj_->getCurrentPosSet()*/ currentPositionSetpoint_,
        /*enc_->getActPos()*/ currentPositionActual_,
        cntrl_->getCntrlError(),
        cntrl_->getOutTot(),
        /*traj_->getTargetPos()-enc_->getActPos()*/currentPositionSetpoint_ -currentPositionActual_,
        /*enc_->getActVel()*/ currentVelocityActual_,
        /*traj_->getVel()*/ currentVelocitySetpoint_,
        cntrl_->getOutFFPart()/drv_->getScale(),
        drv_->getVelSetRaw(),
        getErrorID());
  }
  else{
    printf("%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%i\t%x",
        axisID_,
        /*traj_->getCurrentPosSet()*/ currentPositionSetpoint_,
        /*enc_->getActPos()*/ currentPositionActual_,
        cntrl_->getCntrlError(),
        cntrl_->getOutTot(),
        /*traj_->getTargetPos()-enc_->getActPos()*/currentPositionSetpoint_ -currentPositionActual_,
        /*enc_->getActVel()*/currentVelocityActual_,
        /*traj_->getVel()*/ currentVelocitySetpoint_,
        0.0,
        drv_->getVelSetRaw(),
        getErrorID());
  }
  printf("\t%d  %d  %d  %d  %d  %d  %d  %d  %d\n",
      getEnable(),
      traj_->getExecute(),
      seq_.getBusy(),
      seq_.getSeqState(),
      mon_->getAtTarget(),
      traj_->getInterlockStatus(),
      mon_->getHardLimitFwd(),
      mon_->getHardLimitBwd(),
      mon_->getHomeSwitch());
}

int ecmcAxisReal::validate()
{
  int error=0;
  if(enc_==NULL){
    return setErrorID(ERROR_AXIS_ENC_OBJECT_NULL);
  }

  error=enc_->validate();
  if(error){
    return setErrorID(error);
  }

  if(traj_==NULL){
    return setErrorID(ERROR_AXIS_TRAJ_OBJECT_NULL);
  }

  error=traj_->validate();
  if(error){
    return setErrorID(error);
  }

  if(drv_==NULL){
    return setErrorID(ERROR_AXIS_DRV_OBJECT_NULL);
  }

  error=drv_->validate();
  if(error){
    return setErrorID(error);
  }

  if(mon_==NULL){
    return setErrorID(ERROR_AXIS_MON_OBJECT_NULL);
  }

  error=mon_->validate();
  if(error){
    return setErrorID(error);
  }

  if(cntrl_==NULL){
    return setErrorID(ERROR_AXIS_CNTRL_OBJECT_NULL);
  }

  error=cntrl_->validate();
  if(error){
    return setErrorID(error);
  }

  error=seq_.validate();
  if(error){
    return setErrorID(error);
  }

  error=ecmcAxisBase::validateBase();
  if(error){
    return setErrorID(error);
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
