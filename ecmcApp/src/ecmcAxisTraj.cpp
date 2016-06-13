

/*
 * cMcuAxisTraj.cpp
 *
 *  Created on: Mar 14, 2016
 *      Author: anderssandstrom
 */

#include "ecmcAxisTraj.h"


ecmcAxisTraj::ecmcAxisTraj(int axisID, double sampleTime)
{
  initVars();
  axisID_=axisID;
  axisType_=ECMC_AXIS_TYPE_VIRTUAL;
  sampleTime_=sampleTime;

  traj_=new ecmcTrajectory(sampleTime_);
  mon_=new ecmcMonitor();
  seq_.setCntrl(NULL);
  seq_.setTraj(traj_);
  seq_.setMon(mon_);
  seq_.setEnc(NULL);
}

ecmcAxisTraj::~ecmcAxisTraj()
{
  delete traj_;
  delete mon_;
}

void ecmcAxisTraj::initVars()
{
  initDone_=false;
  sampleTime_=1;
  traj_=NULL;
  mon_=NULL;
}

void ecmcAxisTraj::execute(bool masterOK)
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
    traj_->setHardLimitFwd(mon_->getHardLimitFwd());
    traj_->setHardLimitBwd(mon_->getHardLimitBwd());
    double dTrajCurrSet=traj_->getNextPosSet();
    traj_->setStartPos(dTrajCurrSet);

    seq_.setHomeSensor(mon_->getHomeSwitch());
    seq_.execute();

    mon_->setActPos(dTrajCurrSet);
    mon_->setCurrentPosSet(dTrajCurrSet);
    mon_->setActVel(traj_->getVel());
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

int ecmcAxisTraj::setExecute(bool execute)
{
  int error=seq_.setExecute(execute);
  if(error){
    return setErrorID(error);
  }
  return setExecute_Transform();
}

bool ecmcAxisTraj::getExecute()
{
  return seq_.getExecute();
}

int ecmcAxisTraj::setEnable(bool enable)
{
  traj_->setEnable(enable);
  mon_->setEnable(enable);
  return setEnable_Transform();
}

bool ecmcAxisTraj::getEnable()
{
  return  traj_->getEnable() && mon_->getEnable();
}

int ecmcAxisTraj::getErrorID()
{
  if(mon_->getError()){
    return setErrorID(mon_->getErrorID());
  }
  if(traj_->getError()){
    return setErrorID(traj_->getErrorID());
  }
  if(seq_.getError()){
    return setErrorID(seq_.getErrorID());
  }
  return ecmcError::getErrorID();
}

bool ecmcAxisTraj::getError()
{
  bool bErr=mon_->getError() || traj_->getError() || seq_.getError();
  if( bErr){
    setError(bErr);
  }
  return ecmcError::getError();
}

int ecmcAxisTraj::setOpMode(operationMode nMode)
{
  //NO DRIVE
  return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

operationMode ecmcAxisTraj::getOpMode()
{
  //NO DRIVE
  return ECMC_MODE_OP_AUTO;
}

int ecmcAxisTraj::getActPos(double *pos)
{
  *pos=traj_->getCurrentPosSet();
  return 0;
}

int ecmcAxisTraj::getActVel(double *vel)
{
  *vel=traj_->getVel();
  return 0;
}

int ecmcAxisTraj::getAxisHomed(bool *homed)
{
  *homed=true;
  return 0;
}

int ecmcAxisTraj::getEncScaleNum(double *scale)
{
  *scale=1;
  return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

int ecmcAxisTraj::setEncScaleNum(double scale)
{
  return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

int ecmcAxisTraj::getEncScaleDenom(double *scale)
{
  *scale=1;
  return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

int ecmcAxisTraj::setEncScaleDenom(double scale)
{
  return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

int ecmcAxisTraj::getCntrlError(double* error)
{
  *error=0.0;
  return 0;
}

int ecmcAxisTraj::getEncPosRaw(int64_t *rawPos)
{
  *rawPos=round(traj_->getCurrentPosSet());
  return 0;
}

int ecmcAxisTraj::setCommand(motionCommandTypes command)
{
  if(command==ECMC_CMD_HOMING){
    return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
  }
  seq_.setCommand(command);
  return 0;
}

int ecmcAxisTraj::setCmdData(int cmdData)
{
  seq_.setCmdData(cmdData);
  return 0;
}

void ecmcAxisTraj::errorReset()
{
  traj_->errorReset();
  mon_->errorReset();
  seq_.errorReset();
  ecmcError::errorReset();
}

motionCommandTypes ecmcAxisTraj::getCommand()
{
  return seq_.getCommand();
}

int ecmcAxisTraj::getCmdData()
{
  return seq_.getCmdData();
}

ecmcEncoder *ecmcAxisTraj::getEnc()
{
  return NULL;
}

ecmcPIDController *ecmcAxisTraj::getCntrl()
{
  return NULL;
}

ecmcDrive *ecmcAxisTraj::getDrv()
{
  return NULL;
}

ecmcTrajectory *ecmcAxisTraj::getTraj()
{
  return traj_;
}

ecmcMonitor *ecmcAxisTraj::getMon()
{
  return mon_;
}

ecmcSequencer *ecmcAxisTraj::getSeq()
{
  return &seq_;
}

void ecmcAxisTraj::printStatus()
{
  double currPos=traj_->getCurrentPosSet();
  printf("%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%i\t%x",
      axisID_,
      currPos,
      currPos,
      0.0,
      0.0,
      traj_->getTargetPos()-currPos,
      traj_->getVel(),
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

int ecmcAxisTraj::validate()
{
  int retError=0;

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
