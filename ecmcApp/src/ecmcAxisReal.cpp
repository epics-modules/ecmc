/*
 * cMcuAxisReal.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: anderssandstrom
 */

#include "ecmcAxisReal.h"

ecmcAxisReal::ecmcAxisReal(int axisID, double sampleTime)
{
  initVars();
  axisID_=axisID;
  axisType_=ECMC_AXIS_TYPE_REAL;
  sampleTime_=sampleTime;
  enc_=new ecmcEncoder(sampleTime_);
  traj_=new ecmcTrajectory(sampleTime_);
  mon_ =new ecmcMonitor();
  currentDriveType_=ECMC_STEPPER;
  drv_=new ecmcDriveStepper();
  cntrl_=new ecmcPIDController(sampleTime_);

  seq_.setCntrl(cntrl_);
  seq_.setTraj(traj_);
  seq_.setMon(mon_);
  seq_.setEnc(enc_);
}

ecmcAxisReal::~ecmcAxisReal()
{
  delete cntrl_;
  delete enc_;
  delete traj_;
  delete mon_;
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
    enc_->readEntries();
    drv_->readEntries();
    double encActPos=enc_->getActPos();
    traj_->setHardLimitFwd(mon_->getHardLimitFwd());
    traj_->setHardLimitBwd(mon_->getHardLimitBwd());
    traj_->setStartPos(encActPos);

    double trajCurrSet=traj_->getNextPosSet();

    seq_.setHomeSensor(mon_->getHomeSwitch());
    seq_.execute();

    mon_->setActPos(encActPos);
    mon_->setCurrentPosSet(trajCurrSet);
    mon_->setActVel(enc_->getActVel());
    mon_->setTargetVel(traj_->getVel());
    mon_->setCntrlOutput(cntrl_->getOutTot()); //From last scan
    mon_->execute();
    traj_->setInterlock(mon_->getTrajInterlock()); //TODO consider change logic so high interlock is OK and low not
    drv_->setInterlock(mon_->getDriveInterlock()); //TODO consider change logic so high interlock is OK and low not
    cntrl_->setInterlock(mon_->getDriveInterlock()); //TODO consider change logic so high interlock is OK and low not
    drv_->setAtTarget(mon_->getAtTarget());  //Reduce torque

    if(getEnable() && masterOK && !getError()){
      drv_->setVelSet(cntrl_->control(trajCurrSet,encActPos,traj_->getVel())); //Actual control
    }
    else{
      if(getExecute()){
	setExecute(false);
	traj_->setStartPos(encActPos);
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

  traj_->setEnable(enable);
  cntrl_->setEnable(enable);
  mon_->setEnable(enable);
  int error=drv_->setEnable(enable);
  if(error){
    return setErrorID(error);
  }
  return setEnable_Transform();
}

bool ecmcAxisReal::getEnable()
{
  return drv_->getEnable() && drv_->getEnabled() && traj_->getEnable() && cntrl_->getEnable() && mon_->getEnable();
}

int ecmcAxisReal::getErrorID()
{
  //if(ecmcError::getErrorID()==ERROR_AXIS_HARDWARE_STATUS_NOT_OK){
  if(ecmcError::getError()){
    return ecmcError::getErrorID();
  }

  if(mon_->getError()){
    return setErrorID(mon_->getErrorID());
  }
  if(enc_->getError()){
    return setErrorID(enc_->getErrorID());
  }
  if(drv_->getError()){
    return setErrorID(drv_->getErrorID());
  }
  if(traj_->getError()){
    return setErrorID(traj_->getErrorID());
  }
  if(cntrl_->getError()){
    return setErrorID(cntrl_->getErrorID());
  }
  if(seq_.getErrorID()){
    return setErrorID(seq_.getErrorID());
  }
  return ecmcError::getErrorID();
}

bool ecmcAxisReal::getError()
{
  bool bErr=enc_->getError() || drv_->getError() || mon_->getError() || traj_->getError() || cntrl_->getError() || seq_.getError();
  if(bErr){
    setError(bErr);
  }
  return ecmcError::getError();
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
  *pos=enc_->getActPos();
  return 0;
}

int ecmcAxisReal::getActVel(double *vel)
{
  *vel=enc_->getActVel();
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

void ecmcAxisReal::errorReset()
{
  traj_->errorReset();
  enc_->errorReset();
  mon_->errorReset();
  drv_->errorReset();
  cntrl_->errorReset();
  seq_.errorReset();
  ecmcError::errorReset();
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

ecmcTrajectory *ecmcAxisReal::getTraj()
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
        traj_->getCurrentPosSet(),
        enc_->getActPos(),
        cntrl_->getCntrlError(),
        cntrl_->getOutTot(),
        traj_->getTargetPos()-enc_->getActPos(),
        enc_->getActVel(),
        traj_->getVel(),
        cntrl_->getOutFFPart()/drv_->getScale(),
        drv_->getVelSetRaw(),
        getErrorID());
  }
  else{
    printf("%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%i\t%x",
        axisID_,
        traj_->getCurrentPosSet(),
        enc_->getActPos(),
        cntrl_->getCntrlError(),
        cntrl_->getOutTot(),
        traj_->getTargetPos()-enc_->getActPos(),
        enc_->getActVel(),
        traj_->getVel(),
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
  int errorRet=0;
  if(enc_==NULL){
    return setErrorID(ERROR_AXIS_ENC_OBJECT_NULL);
  }

  errorRet=enc_->validate();
  if(errorRet){
    return setErrorID(errorRet);
  }

  if(traj_==NULL){
    return setErrorID(ERROR_AXIS_TRAJ_OBJECT_NULL);
  }

  errorRet=traj_->validate();
  if(errorRet){
    return setErrorID(errorRet);
  }

  if(drv_==NULL){
    return setErrorID(ERROR_AXIS_DRV_OBJECT_NULL);
  }

  errorRet=drv_->validate();
  if(errorRet){
    return setErrorID(errorRet);
  }

  if(mon_==NULL){
    return setErrorID(ERROR_AXIS_MON_OBJECT_NULL);
  }

  errorRet=mon_->validate();
  if(errorRet){
    return setErrorID(errorRet);
  }

  if(cntrl_==NULL){
    return setErrorID(ERROR_AXIS_CNTRL_OBJECT_NULL);
  }

  errorRet=cntrl_->validate();
  if(errorRet){
    return setErrorID(errorRet);
  }

  errorRet=seq_.validate();
  if(errorRet){
    return setErrorID(errorRet);
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
