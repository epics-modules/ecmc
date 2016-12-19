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
  axisID_=axisID;
  axisType_=ECMC_AXIS_TYPE_VIRTUAL;
  sampleTime_=sampleTime;

  traj_=new ecmcTrajectoryTrapetz(sampleTime_);
  mon_=new ecmcMonitor();
  enc_=new ecmcEncoder(sampleTime_);
  seq_.setCntrl(NULL);
  seq_.setTraj(traj_);
  seq_.setMon(mon_);
  seq_.setEnc(enc_);
  int error=getSeq()->setExtTrajIF(externalInputTrajectoryIF_);
  if(error){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_ASSIGN_EXT_INTERFACE_TO_SEQ_FAILED);
  }
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
    if(externalInputEncoderIF_->getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL){
      enc_->readEntries();
    }

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

    mon_->setCntrlOutput(0); //From last scan
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

    if(getEnable() && masterOK && !getError()){
      mon_->setEnable(true);
    }

    else{
      mon_->setEnable(false);
      if(getExecute()){
	setExecute(false);
	traj_->setStartPos(0);
      }
    }

    if(!masterOK){
      if(getEnable()){
        setEnable(false);
      }
      setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_HARDWARE_STATUS_NOT_OK);
    }

    //Write to hardware
    refreshExternalOutputSources();
  }
}

int ecmcAxisVirt::setExecute(bool execute)
{
  if(execute && !getEnable()){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_NOT_ENABLED);
  }

  int error =seq_.setExecute(execute);
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }
  return setExecute_Transform();
}

bool ecmcAxisVirt::getExecute()
{
  return seq_.getExecute();
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

bool ecmcAxisVirt::getEnable()
{
  return  traj_->getEnable() /*&& mon_->getEnable()*/;
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

int ecmcAxisVirt::getActPos(double *pos)
{
  *pos=currentPositionActual_;
  return 0;
}

int ecmcAxisVirt::getActVel(double *vel)
{
  *vel=currentVelocityActual_;
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

ecmcDriveBase *ecmcAxisVirt::getDrv()
{
  return NULL;
}

ecmcTrajectoryTrapetz *ecmcAxisVirt::getTraj()
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
