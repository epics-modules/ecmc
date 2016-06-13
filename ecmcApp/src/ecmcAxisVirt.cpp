/*
 * cMcuAxisVirt.cpp
 *
 *  Created on: Mar 14, 2016
 *      Author: anderssandstrom
 */

#include "ecmcAxisVirt.h"

ecmcAxisVirt::ecmcAxisVirt(int axisID, double sampleTime)
{
  initVars();
  axisID_=axisID;
  axisType_=ECMC_AXIS_TYPE_VIRTUAL;
  sampleTime_=sampleTime;

  traj_=new ecmcTrajectory(sampleTime_);
  mon_=new ecmcMonitor();
  enc_=new ecmcEncoder(sampleTime_);
  seq_.setCntrl(NULL);
  seq_.setTraj(traj_);
  seq_.setMon(mon_);
  seq_.setEnc(enc_);
}

ecmcAxisVirt::~ecmcAxisVirt()
{
  delete traj_;
  delete mon_;
}

void ecmcAxisVirt::initVars()
{
  initDone_=false;
  sampleTime_=1;
  enc_=NULL;
  traj_=NULL;
  mon_=NULL;
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
    enc_->readEntries();
    traj_->setHardLimitFwd(mon_->getHardLimitFwd());
    traj_->setHardLimitBwd(mon_->getHardLimitBwd());
    double dTrajCurrSet=traj_->getNextPosSet();
    traj_->setStartPos(dTrajCurrSet);

    seq_.setHomeSensor(mon_->getHomeSwitch());
    seq_.execute();

    mon_->setActPos(enc_->getActPos());
    mon_->setCurrentPosSet(dTrajCurrSet);
    mon_->setActVel(enc_->getActVel());
    mon_->setTargetVel(traj_->getVel());
    mon_->execute();
    traj_->setInterlock(mon_->getTrajInterlock());
  }
  else{
    if(getEnable()){
	  setEnable(false);
	}
    setErrorID(ERROR_AXIS_HARDWARE_STATUS_NOT_OK);
  }
}

int ecmcAxisVirt::setExecute(bool execute)
{
  int error =seq_.setExecute(execute);
  if(error){
    return setErrorID(error);
  }
  return setExecute_Transform();
}

bool ecmcAxisVirt::getExecute()
{
  return seq_.getExecute();
}

int ecmcAxisVirt::setEnable(bool enable)
{
  traj_->setEnable(enable);
  mon_->setEnable(enable);
  return setEnable_Transform();
}

bool ecmcAxisVirt::getEnable()
{
  return  traj_->getEnable() && mon_->getEnable();
}

int ecmcAxisVirt::getErrorID()
{
  if(mon_->getError()){
    return setErrorID(mon_->getErrorID());
  }
  if(enc_->getError()){
    return setErrorID(enc_->getErrorID());
  }
  if(traj_->getError()){
    return setErrorID(traj_->getErrorID());
  }
  if(seq_.getError()){
    return setErrorID(seq_.getErrorID());
  }
  return ecmcError::getErrorID();
}

bool ecmcAxisVirt::getError()
{
  bool bErr=mon_->getError() || traj_->getError() || seq_.getError() || enc_->getError();
  if( bErr){
    setError(bErr);
  }
  return ecmcError::getError();
}

int ecmcAxisVirt::setOpMode(operationMode mode)
{
  //NO DRIVE
  return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

operationMode ecmcAxisVirt::getOpMode()
{
  //NO DRIVE
  return ECMC_MODE_OP_AUTO;
}

int ecmcAxisVirt::getActPos(double *pos)
{
  *pos=enc_->getActPos();
  return 0;
}

int ecmcAxisVirt::getActVel(double *vel)
{
  *vel=enc_->getActVel();
  return 0;
}

int ecmcAxisVirt::getAxisHomed(bool *homed)
{
  *homed=enc_->getHomed();
  return 0;
}

int ecmcAxisVirt::getEncScaleNum(double *scale)
{
  *scale=enc_->getScaleNum();
  return 0;
}

int ecmcAxisVirt::setEncScaleNum(double scale)
{
  return enc_->setScaleNum(scale);
}

int ecmcAxisVirt::getEncScaleDenom(double *scale)
{
  *scale=enc_->getScaleDenom();
  return 0;
}

int ecmcAxisVirt::setEncScaleDenom(double scale)
{
  return enc_->setScaleDenom(scale);
}

int ecmcAxisVirt::getCntrlError(double* error)
{
  *error=traj_->getCurrentPosSet()-enc_->getActPos();
  return 0;
}

int ecmcAxisVirt::getEncPosRaw(int64_t *rawPos)
{
  *rawPos=enc_->getRawPos();
  return 0;
}

int ecmcAxisVirt::setCommand(motionCommandTypes command)
{
  seq_.setCommand(command);
  return 0;
}

int ecmcAxisVirt::setCmdData(int cmdData)
{
  seq_.setCmdData(cmdData);
  return 0;
}

void ecmcAxisVirt::errorReset()
{
  traj_->errorReset();
  mon_->errorReset();
  enc_->errorReset();
  seq_.errorReset();
  ecmcError::errorReset();
}

motionCommandTypes ecmcAxisVirt::getCommand()
{
  return seq_.getCommand();
}

int ecmcAxisVirt::getCmdData()
{
  return seq_.getCmdData();
}

ecmcEncoder *ecmcAxisVirt::getEnc()
{
  return enc_;
}

ecmcPIDController *ecmcAxisVirt::getCntrl()
{
  return NULL;
}

ecmcDrive *ecmcAxisVirt::getDrv()
{
  return NULL;
}

ecmcTrajectory *ecmcAxisVirt::getTraj()
{
  return traj_;
}

ecmcMonitor *ecmcAxisVirt::getMon()
{
  return mon_;
}

ecmcSequencer *ecmcAxisVirt::getSeq()
{
  return &seq_;
}

void ecmcAxisVirt::printStatus()
{

  double cntrlError=0;
  int retError =getCntrlError(&cntrlError);
  if(retError){
    cntrlError=0.0;
  }

  printf("%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%i\t%x",
      axisID_,
      traj_->getCurrentPosSet(),
      enc_->getActPos(),
      cntrlError,
      0.0,
      traj_->getTargetPos()-enc_->getActPos(),
      enc_->getActVel(),
      traj_->getVel(),
      0.0,
      0,
      getErrorID());

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

int ecmcAxisVirt::validate()
{
  int retError=0;
  if(enc_==NULL){
    return setErrorID(ERROR_AXIS_ENC_OBJECT_NULL);
  }

  retError=enc_->validate();
  if(retError){
    return setErrorID(retError);
  }

  if(traj_==NULL){
    return setErrorID(ERROR_AXIS_TRAJ_OBJECT_NULL);
  }

  retError=traj_->validate();
  if(retError){
    return setErrorID(retError);
  }

  if(mon_==NULL){
    return setErrorID(ERROR_AXIS_MON_OBJECT_NULL);
  }

  retError=mon_->validate();
  if(retError){
    return setErrorID(retError);
  }

  retError=seq_.validate();
  if(retError){
    return setErrorID(retError);
  }

  return 0;
}
