/*
 *
 *  Created on: Jan 14, 2016
 *      Author: anderssandstrom
 */

#include "ecmcSequencer.hpp"

ecmcSequencer::ecmcSequencer()
{
  initVars();
}

ecmcSequencer::~ecmcSequencer()
{
  ;
}

void ecmcSequencer::initVars()
{
  errorReset();
  homeSensorOld_=false;
  executeOld_=false;
  seqInProgress_=false;
  currSeqDirection_=ECMC_DIR_FORWARD;
  seqState_=0;
  traj_=NULL;
  enc_=NULL;
  mon_=NULL;
  cntrl_=NULL;
  jogVel_=0;
  homeVelTwordsCam_=0; //ADR command
  homeVelOffCam_=0; //ADR command
  homeDirection_=ECMC_DIR_FORWARD;
  homePosition_=0;
  jogFwd_=false;
  jogBwd_=false;
  enableSoftLimitBwdBackup_=false;
  enableSoftLimitFwdBackup_=false;
  hwLimitSwitchBwdOld_=false;
  hwLimitSwitchFwdOld_=false;
  homePosLatch1_=0;
  homePosLatch2_=0;
  seqTimeout_=0; //disabled
  seqTimeCounter_=0;
}

void ecmcSequencer::execute()
{  //Cyclic execution
  if(traj_==NULL){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_TRAJ_NULL);
    return;
  }
  if(mon_==NULL){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_MON_NULL);
    return;
  }

  hwLimitSwitchBwdOld_=hwLimitSwitchBwd_;
  hwLimitSwitchFwdOld_=hwLimitSwitchFwd_;
  hwLimitSwitchBwd_=data_->status_.limitBwd;
  hwLimitSwitchFwd_=data_->status_.limitFwd;

  homeSensorOld_=homeSensor_;
  homeSensor_=data_->status_.homeSwitch;

  if(!seqInProgress_){
    return;
  }
  seqTimeCounter_++;
  if(seqTimeCounter_>seqTimeout_ && seqTimeout_>0){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_TIMEOUT);
    stopSeq();
    return;
  }
  int seqReturnVal=0;
  switch(data_->command_.command){
    case ECMC_CMD_JOG:
      ;
      break;
    case ECMC_CMD_HOMING:
      switch (data_->command_.cmdData){
        case 1:
          seqReturnVal=seqHoming1();
          if(seqReturnVal>0){//Error
            setErrorID(__FILE__,__FUNCTION__,__LINE__,seqReturnVal);
            stopSeq();
          }
          else if(seqReturnVal==0){//Homing ready
            stopSeq();
          }
          break;
        case 2:
          seqReturnVal=seqHoming2();
          if(seqReturnVal>0){//Error
            setErrorID(__FILE__,__FUNCTION__,__LINE__,seqReturnVal);
            stopSeq();
          }
          else if(seqReturnVal==0){//Homing ready
            stopSeq();
          }
          break;
        case 3:
          seqReturnVal=seqHoming3();
          if(seqReturnVal>0){//Error
            setErrorID(__FILE__,__FUNCTION__,__LINE__,seqReturnVal);
            stopSeq();
          }
          else if(seqReturnVal==0){//Homing ready
            stopSeq();
          }
          break;
        case 4:
          seqReturnVal=seqHoming4();
          if(seqReturnVal>0){//Error
            setErrorID(__FILE__,__FUNCTION__,__LINE__,seqReturnVal);
            stopSeq();
          }
          else if(seqReturnVal==0){//Homing ready
            stopSeq();
          }
          break;
        case 5:
          seqReturnVal=seqHoming5();
          if(seqReturnVal>0){//Error
            setErrorID(__FILE__,__FUNCTION__,__LINE__,seqReturnVal);
            stopSeq();
          }
          else if(seqReturnVal==0){//Homing ready
            stopSeq();
          }
          break;
        case 6:
          seqReturnVal=seqHoming6();
          if(seqReturnVal>0){//Error
            setErrorID(__FILE__,__FUNCTION__,__LINE__,seqReturnVal);
            stopSeq();
          }
          else if(seqReturnVal==0){//Homing ready
            stopSeq();
          }
          break;
        default:
          setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_CMD_DATA_UNDEFINED);
          break;
      }
      break;
    default:
      setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_CMD_UNDEFINED);
      break;
  }
}

int  ecmcSequencer::setExecute(bool execute)
{
  if(traj_==NULL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_TRAJ_NULL);
  }

  executeOld_=data_->command_.execute;
  data_->command_.execute=execute;
  seqInProgress_=false;
  seqState_=0;

  switch (data_->command_.command){
    case ECMC_CMD_JOG:
      //Triggered via jog inputs
      break;
    case ECMC_CMD_MOVEVEL:
      if(data_->command_.execute  && !executeOld_){
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setTargetVel(data_->command_.velocityTarget);
        data_->command_.positionTarget=checkSoftLimits(data_->command_.positionTarget); //not needed
        traj_->setTargetPos(data_->command_.positionTarget);
      }
      traj_->setExecute(data_->command_.execute);
      break;
    case ECMC_CMD_MOVEREL:
      if(data_->command_.execute && !executeOld_){
        traj_->setMotionMode(ECMC_MOVE_MODE_POS);
        traj_->setTargetVel(data_->command_.velocityTarget);
        traj_->setTargetPos(checkSoftLimits(traj_->getCurrentPosSet()+data_->command_.positionTarget));
      }
      traj_->setExecute(data_->command_.execute);
      break;
    case ECMC_CMD_MOVEABS:
      if(data_->command_.execute && !executeOld_){
        traj_->setMotionMode(ECMC_MOVE_MODE_POS);
        traj_->setTargetVel(data_->command_.velocityTarget);
        switch(data_->command_.cmdData){
          case 0: //Normal positioning
            data_->command_.positionTarget=checkSoftLimits(data_->command_.positionTarget);
            traj_->setTargetPos(data_->command_.positionTarget);
            break;
          case 1: //Go to external transform current value (transform value as TragetPosition)
            double targPos=0;
            int errorCode=getExtTrajSetpoint(&targPos);
            if(errorCode){
              return errorCode;
            }
            data_->command_.positionTarget=checkSoftLimits(targPos);
            traj_->setTargetPos(data_->command_.positionTarget);
            break;
        }
      }
      traj_->setExecute(data_->command_.execute);
      break;
    case ECMC_CMD_MOVEMODULO:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_COMMAND_NOT_SUPPORTED);
      break;
    case ECMC_CMD_HOMING:
      if(data_->command_.execute && !executeOld_){
        stopSeq();
        if(traj_!=NULL && enc_!=NULL && mon_!=NULL && cntrl_!=NULL){
          seqInProgress_=true;
          busy_=true;
        }
        else{
          if(traj_==NULL){
            return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_TRAJ_NULL);
          }
          else{
            traj_->setExecute(false);
          }

          if(enc_==NULL){
            return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_ENC_NULL);
          }

          if(mon_==NULL){
            return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_MON_NULL);
          }

          if(cntrl_==NULL){
            return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_CNTRL_NULL);
          }
        }
      }
      else if(!data_->command_.execute){
        stopSeq();
        traj_->setExecute(data_->command_.execute);
      }
      break;
    case ECMC_CMD_SUPERIMP:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_COMMAND_NOT_SUPPORTED);
      break;
    case ECMC_CMD_GEAR:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_COMMAND_NOT_SUPPORTED);
      break;
    default:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_COMMAND_NOT_SUPPORTED);
      break;
  }

  return 0;
}

bool ecmcSequencer::getExecute()
{
  return data_->command_.execute;
}

void ecmcSequencer::setCommand(motionCommandTypes command)
{
  data_->command_.command=command;
}

motionCommandTypes ecmcSequencer::getCommand()
{
  return data_->command_.command;
}

void ecmcSequencer::setCmdData(int cmdData)
{
  data_->command_.cmdData=cmdData;
}

int ecmcSequencer::getCmdData()
{
  return data_->command_.cmdData;
}

void ecmcSequencer::setTraj(ecmcTrajectoryTrapetz *traj)
{
  traj_=traj;
}

void ecmcSequencer::setEnc(ecmcEncoder *enc)
{
  enc_=enc;
}

void ecmcSequencer::setMon(ecmcMonitor *mon)
{
  mon_=mon;
}

void ecmcSequencer::setCntrl(ecmcPIDController *cntrl)
{
  cntrl_=cntrl;
}

bool ecmcSequencer::getBusy()
{
  if(traj_==NULL){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_TRAJ_NULL);
    return false;
  }

  if(data_->command_.command==ECMC_CMD_HOMING){
    return busy_;
  }
  else{
    return traj_->getBusy();
  }
}

void ecmcSequencer::setJogVel(double vel)
{
  jogVel_=vel;
}

double ecmcSequencer::getJogVel()
{
  return jogVel_;
}

int ecmcSequencer::setHomeVelTwordsCam(double vel)
{
  homeVelTwordsCam_=vel;
  return 0;
}

int ecmcSequencer::setHomeVelOffCam(double vel)
{
  homeVelOffCam_=vel;
  return 0;
}

double ecmcSequencer::getHomeVelTwordsCam()
{
  return homeVelTwordsCam_;
}

double ecmcSequencer::getHomeVelOffCam()
{
  return homeVelOffCam_;
}

int ecmcSequencer::setHomeDir(motionDirection dir)
{
  homeDirection_=dir;
  return 0;
}

motionDirection ecmcSequencer::getHomeDir()
{
  return homeDirection_;
}

void ecmcSequencer::setHomePosition(double pos)
{
  homePosition_=pos;
}

double ecmcSequencer::getHomePosition()
{
  return homePosition_;
}

void ecmcSequencer::setTargetPos(double pos)
{
  double tempPos=checkSoftLimits(pos);

  if(pos>tempPos){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_SOFT_LIMIT_FWD);
    return;
  }
  if(pos<tempPos){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_SOFT_LIMIT_BWD);
    return;
  }

  data_->command_.positionTarget=pos;
}

double ecmcSequencer::getTargetPos()
{
  return data_->command_.positionTarget;
}

void ecmcSequencer::setTargetVel(double velTarget)
{
  data_->command_.velocityTarget=velTarget;
}

double ecmcSequencer::getTargetVel()
{
  return data_->command_.velocityTarget;
}

void ecmcSequencer::setJogFwd(bool jog)
{
  jogFwd_=jog;
  if(traj_==NULL){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_TRAJ_NULL);
    return;
  }
  if(data_->command_.command==ECMC_CMD_JOG && jogFwd_){
    traj_->setTargetVel(jogVel_);
    traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
    traj_->setExecute(jogFwd_);
  }
  else{
    traj_->setTargetVel(data_->command_.velocityTarget);
  }
}

bool ecmcSequencer::getJogFwd()
{
  if(traj_==NULL){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_TRAJ_NULL);
    return false;
  }
  return jogFwd_;
}

void ecmcSequencer::setJogBwd(bool jog)
{
  jogBwd_=jog;
  if(traj_==NULL){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_TRAJ_NULL);
    return;
  }
  if(data_->command_.command==ECMC_CMD_JOG && jogBwd_){
    traj_->setTargetVel(-jogVel_);
    traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
    traj_->setExecute(jogBwd_);
  }
  else{
    traj_->setTargetVel(data_->command_.velocityTarget);
  }
}

bool ecmcSequencer::getJogBwd()
{
  if(traj_==NULL){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_TRAJ_NULL);
    return false;
  }
  return jogBwd_;
}

double ecmcSequencer::checkSoftLimits(double posSetpoint)
{
  if(!mon_){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_MON_NULL);
    return posSetpoint;
  }

  double dSet=posSetpoint;
  double currPos=enc_->getActPos();

  if(posSetpoint>data_->command_.softLimitFwd && data_->command_.enableSoftLimitFwd && posSetpoint>currPos){
    dSet=data_->command_.softLimitFwd;
  }
  if(posSetpoint<data_->command_.softLimitBwd && data_->command_.enableSoftLimitBwd && posSetpoint<currPos){
    dSet=data_->command_.softLimitBwd;;
  }
  return dSet;
}

ecmcTrajectoryTrapetz * ecmcSequencer::getTraj()
{
  return traj_;
}

int ecmcSequencer::seqHoming1() //nCmdData==1
{
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for negative edge of bwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in positive direction
  // State 3 Latch encoder value on falling or rising edge of bwd limit switch sensor.
  // State 4 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit


  int retValue=traj_->getErrorID();  //Abort if error from trajectory
  if(retValue){
    return retValue;
  }

  //Sequence code
  switch(seqState_){
    case 0:  //Set parameters and start initial motion
      enc_->setHomed(false);
      enableSoftLimitBwdBackup_=data_->command_.enableSoftLimitBwd; //Read setting to be able to restore later
      enableSoftLimitFwdBackup_=data_->command_.enableSoftLimitFwd; //Read setting to be able to restore later
      data_->command_.enableSoftLimitBwd=false; //Disable softlimits for homing
      data_->command_.enableSoftLimitFwd=false;
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(0);
      if(hwLimitSwitchBwd_){
        currSeqDirection_=ECMC_DIR_BACKWARD;  //StartDirection
        traj_->setTargetVel(-homeVelTwordsCam_); //high speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);
        seqState_=1;
      }
      else{ //Already at bwd limit jump to step 2
        currSeqDirection_=ECMC_DIR_FORWARD;  //StartDirection
        seqState_=2;
      }
      break;

    case 1: //Wait for negative limit switch and turn other direction
      if(hwLimitSwitchBwdOld_ && !hwLimitSwitchBwd_){
        traj_->setExecute(0);
        //Switch direction
        currSeqDirection_=ECMC_DIR_FORWARD;
        seqState_=2;
      }
      break;

    case 2: //Wait for standstill and then trigger move
      retValue=checkHWLimitsAndStop(0,1); // should never go to forward limit switch
      if(retValue){
        return retValue;
      }
      if(!traj_->getBusy()){
        traj_->setTargetVel(homeVelOffCam_); //low speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);//Trigg new movement
        seqState_=3;
      }
      else{
        traj_->setExecute(0);
      }
      break;

    case 3: //Latch encoder value on falling or rising edge of bwd limit switch
      retValue=checkHWLimitsAndStop(0,1); // should never go to forward or backward limit switch
      if(retValue){
        return retValue;
      }
      if(hwLimitSwitchBwd_!=hwLimitSwitchBwdOld_){
        homePosLatch1_=enc_->getActPos();
        seqState_=4;
      }
      break;
    case 4:  //Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit
      retValue=checkHWLimitsAndStop(1,1); // should never go to forward limit or backward switch
      if(retValue){
        return retValue;
      }
      traj_->setExecute(0);
      if(!traj_->getBusy()){ //Wait for stop ramp ready
        data_->command_.positionTarget=traj_->getCurrentPosSet();
        if(mon_->getAtTarget())//Wait for controller to settle in order to minimize bump
        {
          double currPos=enc_->getActPos()-homePosLatch1_+homePosition_;
          traj_->setCurrentPosSet(currPos);
          enc_->setActPos(currPos);
          enc_->setHomed(true);
          cntrl_->reset();  //TODO.. Should this really be needed.. Error should be zero anyway.. Controller jumps otherwise.. PROBLEM
          homePosLatch1_=0;
          homePosLatch2_=0;
          stopSeq();
        }
      }
      break;
  }
  return -seqState_;
}

int ecmcSequencer::seqHoming2() //nCmdData==2
{
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for positive edge of bwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in negative direction
  // State 3 Latch encoder value on falling or rising edge of fwd limit switch sensor.
  // State 4 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit

  int retValue=traj_->getErrorID();  //Abort if error from trajectory
  if(retValue){
    return retValue;
  }

  //Sequence code
  switch(seqState_){
    case 0:  //Set parameters and start initial motion
      enc_->setHomed(false);
      enableSoftLimitBwdBackup_=data_->command_.enableSoftLimitBwd; //Read setting to be able to restore later
      enableSoftLimitFwdBackup_=data_->command_.enableSoftLimitFwd; //Read setting to be able to restore later
      data_->command_.enableSoftLimitBwd=false; //Disable softlimits for homing
      data_->command_.enableSoftLimitFwd=false;
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(0);
      if(hwLimitSwitchFwd_){
        currSeqDirection_=ECMC_DIR_FORWARD;  //StartDirection
        traj_->setTargetVel(homeVelTwordsCam_); //high speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);
        seqState_=1;
      }
      else{ //Already at bwd limit jump to step 2
        currSeqDirection_=ECMC_DIR_BACKWARD;  //StartDirection
        seqState_=2;
      }
      break;

    case 1: //Wait for positive limit switch and turn other direction
      if(hwLimitSwitchFwdOld_ && !hwLimitSwitchFwd_){
        traj_->setExecute(0);
        //Switch direction
        currSeqDirection_=ECMC_DIR_BACKWARD;
        seqState_=2;
      }
      break;

    case 2: //Wait for standstill and then trigger move
      retValue=checkHWLimitsAndStop(1,0); // should never go to forward limit switch
      if(retValue){
        return retValue;
      }
      if(!traj_->getBusy()){
        traj_->setTargetVel(-homeVelOffCam_); //low speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);//Trigg new movement
        seqState_=3;
      }
      else{
        traj_->setExecute(0);
      }
      break;

    case 3: //Latch encoder value on falling or rising edge of home sensor
      retValue=checkHWLimitsAndStop(1,0); // should never go to forward or backward limit switch
      if(retValue){
        return retValue;
      }
      if(hwLimitSwitchFwd_!=hwLimitSwitchFwdOld_){
        homePosLatch1_=enc_->getActPos();
        seqState_=4;
      }
      break;
    case 4:  //Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit
      retValue=checkHWLimitsAndStop(1,1); // should never go to forward limit or backward switch
      if(retValue){
        return retValue;
      }
      traj_->setExecute(0);
      if(!traj_->getBusy()){ //Wait for stop ramp ready
        data_->command_.positionTarget=traj_->getCurrentPosSet();
        if(mon_->getAtTarget())//Wait for controller to settle in order to minimize bump
        {
          double currPos=enc_->getActPos()-homePosLatch1_+homePosition_;
          traj_->setCurrentPosSet(currPos);
          enc_->setActPos(currPos);
          enc_->setHomed(true);
          cntrl_->reset();  //TODO.. Should this really be needed.. Error should be zero anyway.. Controller jumps otherwise.. PROBLEM
          homePosLatch1_=0;
          homePosLatch2_=0;
          stopSeq();
        }
      }
      break;
  }
  return -seqState_;
}

int ecmcSequencer::seqHoming3() //nCmdData==3
{
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for negative edge of bwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in positive direction
  // State 3 Latch encoder value on falling or rising edge of home sensor.
  // State 4 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit


  int retValue=traj_->getErrorID();  //Abort if error from trajectory
  if(retValue){
    return retValue;
  }

  //Sequence code
  switch(seqState_){
    case 0:  //Set parameters and start initial motion
      enc_->setHomed(false);
      enableSoftLimitBwdBackup_=data_->command_.enableSoftLimitBwd; //Read setting to be able to restore later
      enableSoftLimitFwdBackup_=data_->command_.enableSoftLimitFwd; //Read setting to be able to restore later
      data_->command_.enableSoftLimitBwd=false; //Disable softlimits for homing
      data_->command_.enableSoftLimitFwd=false;
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(0);
      if(hwLimitSwitchBwd_){
        currSeqDirection_=ECMC_DIR_BACKWARD;  //StartDirection
        traj_->setTargetVel(-homeVelTwordsCam_); //high speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);
        seqState_=1;
      }
      else{ //Already at bwd limit jump to step 2
        currSeqDirection_=ECMC_DIR_FORWARD;  //StartDirection
        seqState_=2;
      }
      break;

    case 1: //Wait for negative limit switch and turn other direction
      if(hwLimitSwitchBwdOld_ && !hwLimitSwitchBwd_){
        traj_->setExecute(0);
        //Switch direction
        currSeqDirection_=ECMC_DIR_FORWARD;
        seqState_=2;
      }
      data_->command_.enableSoftLimitBwd=false; //Disable softlimits for homing
      data_->command_.enableSoftLimitFwd=false;
      break;

    case 2: //Wait for standstill and then trigger move
      retValue=checkHWLimitsAndStop(0,1); // should never go to forward limit switch
      if(retValue){
        return retValue;
      }
      if(!traj_->getBusy()){
        traj_->setTargetVel(homeVelOffCam_); //low speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);//Trigg new movement
        seqState_=3;
      }
      else{
        traj_->setExecute(0);
      }
      break;

    case 3: //Latch encoder value on falling or rising edge of home sensor
      retValue=checkHWLimitsAndStop(0,1); // should never go to forward or backward limit switch
      if(retValue){
        return retValue;
      }
      if(homeSensor_!=homeSensorOld_){
        homePosLatch1_=enc_->getActPos();
        seqState_=4;
      }
      break;
    case 4:  //Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit
      retValue=checkHWLimitsAndStop(1,1); // should never go to forward limit or backward switch
      if(retValue){
        return retValue;
      }
      traj_->setExecute(0);
      if(!traj_->getBusy()){ //Wait for stop ramp ready
        data_->command_.positionTarget=traj_->getCurrentPosSet();
        if(mon_->getAtTarget())//Wait for controller to settle in order to minimize bump
        {
          double currPos=enc_->getActPos()-homePosLatch1_+homePosition_;
          traj_->setCurrentPosSet(currPos);
          enc_->setActPos(currPos);
          enc_->setHomed(true);
          cntrl_->reset();  //TODO.. Should this really be needed.. Error should be zero anyway.. Controller jumps otherwise.. PROBLEM
          homePosLatch1_=0;
          homePosLatch2_=0;
          stopSeq();
        }
      }
      break;
  }
  return -seqState_;
}

int ecmcSequencer::seqHoming4() //nCmdData==4
{
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for positive edge of bwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in negative direction
  // State 3 Latch encoder value on falling or rising edge of home sensor.
  // State 4 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit


  int retValue=traj_->getErrorID();  //Abort if error from trajectory
  if(retValue){
    return retValue;
  }

  //Sequence code
  switch(seqState_){
    case 0:  //Set parameters and start initial motion
      enc_->setHomed(false);
      enableSoftLimitBwdBackup_=data_->command_.enableSoftLimitBwd; //Read setting to be able to restore later
      enableSoftLimitFwdBackup_=data_->command_.enableSoftLimitFwd; //Read setting to be able to restore later
      data_->command_.enableSoftLimitBwd=false; //Disable softlimits for homing
      data_->command_.enableSoftLimitFwd=false;
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(0);
      if(hwLimitSwitchFwd_){
        currSeqDirection_=ECMC_DIR_FORWARD;  //StartDirection
        traj_->setTargetVel(homeVelTwordsCam_); //high speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);
        seqState_=1;
      }
      else{ //Already at bwd limit jump to step 2
        currSeqDirection_=ECMC_DIR_BACKWARD;  //StartDirection
        seqState_=2;
      }
      break;

    case 1: //Wait for negative limit switch and turn other direction
      if(hwLimitSwitchFwdOld_ && !hwLimitSwitchFwd_){
        traj_->setExecute(0);
        //Switch direction
        currSeqDirection_=ECMC_DIR_BACKWARD;
        seqState_=2;
      }
      break;

    case 2: //Wait for standstill and then trigger move
      retValue=checkHWLimitsAndStop(1,0); // should never go to forward limit switch
      if(retValue){
        return retValue;
      }
      if(!traj_->getBusy()){
        traj_->setTargetVel(-homeVelOffCam_); //low speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);//Trigg new movement
        seqState_=3;
      }
      else{
        traj_->setExecute(0);
      }
      break;

    case 3: //Latch encoder value on falling or rising edge of home sensor
      retValue=checkHWLimitsAndStop(1,0); // should never go to forward or backward limit switch
      if(retValue){
        return retValue;
      }
      if(homeSensor_!=homeSensorOld_){
        homePosLatch1_=enc_->getActPos();
        seqState_=4;
      }
      break;
    case 4:  //Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit
      retValue=checkHWLimitsAndStop(1,1); // should never go to forward limit or backward switch
      if(retValue){
        return retValue;
      }
      traj_->setExecute(0);
      if(!traj_->getBusy()){ //Wait for stop ramp ready
        data_->command_.positionTarget=traj_->getCurrentPosSet();
        if(mon_->getAtTarget())//Wait for controller to settle in order to minimize bump
        {
          double currPos=enc_->getActPos()-homePosLatch1_+homePosition_;
          traj_->setCurrentPosSet(currPos);
          enc_->setActPos(currPos);
          enc_->setHomed(true);
          cntrl_->reset();  //TODO.. Should this really be needed.. Error should be zero anyway.. Controller jumps otherwise.. PROBLEM
          homePosLatch1_=0;
          homePosLatch2_=0;
          stopSeq();
        }
      }
      break;
  }
  return -seqState_;
}

int ecmcSequencer::seqHoming5() //nCmdData==5
{
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for negative edge of bwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in positive direction
  // State 3 Latch encoder value on falling or rising edge of home sensor. Continue movement
  // State 4 Wait for falling or rising edge of home sensor then stop
  // State 5 Wait for standstill and the trigger move
  // State 6 Latch value on falling or rising edge of home sensor. Stop motion
  // State 7 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit


  int retValue=traj_->getErrorID();  //Abort if error from trajectory
  if(retValue){
    return retValue;
  }

  //Sequence code
  switch(seqState_){
    case 0:  //Set parameters and start initial motion
      enc_->setHomed(false);
      enableSoftLimitBwdBackup_=data_->command_.enableSoftLimitBwd; //Read setting to be able to restore later
      enableSoftLimitFwdBackup_=data_->command_.enableSoftLimitFwd; //Read setting to be able to restore later
      data_->command_.enableSoftLimitBwd=false; //Disable softlimits for homing
      data_->command_.enableSoftLimitFwd=false;
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(0);
      if(hwLimitSwitchBwd_){
        currSeqDirection_=ECMC_DIR_BACKWARD;  //StartDirection
        traj_->setTargetVel(-homeVelTwordsCam_); //high speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);
        seqState_=1;
      }
      else{ //Already at bwd limit jump to step 2
        currSeqDirection_=ECMC_DIR_FORWARD;  //StartDirection
        seqState_=2;
      }
      break;

    case 1: //Wait for negative limit switch and turn other direction
      if(hwLimitSwitchBwdOld_ && !hwLimitSwitchBwd_){
        traj_->setExecute(0);
        //Switch direction
        currSeqDirection_=ECMC_DIR_FORWARD;
        seqState_=2;
      }
      break;

    case 2: //Wait for standstill and then trigger move
      retValue=checkHWLimitsAndStop(0,1); // should never go to forward limit switch
      if(retValue){
        return retValue;
      }
      if(!traj_->getBusy()){
        traj_->setTargetVel(homeVelOffCam_); //low speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);//Trigg new movement
        seqState_=3;
      }
      else{
        traj_->setExecute(0);
      }
      break;

    case 3: //Latch encoder value on falling or rising edge of home sensor
      retValue=checkHWLimitsAndStop(0,1); // should never go to forward or backward limit switch
      if(retValue){
        return retValue;
      }
      if(homeSensor_!=homeSensorOld_){
        homePosLatch1_=enc_->getActPos();
        seqState_=4;
      }
      break;

    case 4: //Wait for falling or rising edge of home sensor then stop
      retValue=checkHWLimitsAndStop(1,1); // should never go to forward or backward limit switch
      if(retValue){
        return retValue;
      }

      if(homeSensor_!=homeSensorOld_){
        traj_->setExecute(0);
        seqState_=5;
      }
      break;

    case 5: //Wait for standstill and the trigger move
      retValue=checkHWLimitsAndStop(1,1); // should never go to forward or backward limit switch
      if(retValue){
        return retValue;
      }

      if(!traj_->getBusy()){ //Trigg new movement
        traj_->setTargetVel(-homeVelOffCam_); //low speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);
        seqState_=6;
      }
      else{
        traj_->setExecute(0);
      }
      break;

    case 6: //Latch value on falling or rising edge of home sensor. Stop motion
      retValue=checkHWLimitsAndStop(1,1); // should never go to forward or backward limit switch
      if(retValue){
        return retValue;
      }
      if(homeSensor_!=homeSensorOld_){
        homePosLatch2_=enc_->getActPos();
        traj_->setExecute(0);
        seqState_=7;
      }
      break;

    case 7:  //Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit
      retValue=checkHWLimitsAndStop(1,1); // should never go to forward limit or backward switch
      if(retValue){
        return retValue;
      }
      traj_->setExecute(0);
      if(!traj_->getBusy()){ //Wait for stop ramp ready
        data_->command_.positionTarget=traj_->getCurrentPosSet();
        if(mon_->getAtTarget())//Wait for controller to settle in order to minimize bump
        {
          double currPos=enc_->getActPos()-((homePosLatch2_+homePosLatch1_)/2)+homePosition_;
          traj_->setCurrentPosSet(currPos);
          enc_->setActPos(currPos);
          enc_->setHomed(true);
          cntrl_->reset();  //TODO.. Should this really be needed.. Error should be zero anyway.. Controller jumps otherwise.. PROBLEM
          homePosLatch1_=0;
          homePosLatch2_=0;
          stopSeq();
        }
      }
      break;
  }

  return -seqState_;
}

int ecmcSequencer::seqHoming6() //nCmdData==6
{
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for negative edge of fwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in negative direction
  // State 3 Latch encoder value on falling or rising edge of home sensor. Continue movement
  // State 4 Wait for falling or rising edge of home sensor then stop
  // State 5 Wait for standstill and the trigger move
  // State 6 Latch value on falling or rising edge of home sensor. Stop motion
  // State 7 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit


  int retValue=traj_->getErrorID();  //Abort if error from trajectory
  if(retValue){
    return retValue;
  }

  //Sequence code
  switch(seqState_){
    case 0:  //Set parameters and start initial motion
      enc_->setHomed(false);
      enableSoftLimitBwdBackup_=data_->command_.enableSoftLimitBwd; //Read setting to be able to restore later
      enableSoftLimitFwdBackup_=data_->command_.enableSoftLimitFwd; //Read setting to be able to restore later
      data_->command_.enableSoftLimitBwd=false; //Disable softlimits for homing
      data_->command_.enableSoftLimitFwd=false;
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(0);
      if(hwLimitSwitchFwd_){
        currSeqDirection_=ECMC_DIR_FORWARD ;  //StartDirection
        traj_->setTargetVel(homeVelTwordsCam_); //high speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);
        seqState_=1;
      }
      else{ //Already at bwd limit jump to step 2
        currSeqDirection_=ECMC_DIR_BACKWARD; //StartDirection
        seqState_=2;
      }
      break;

    case 1: //Wait for negative limit switch and turn other direction
      if(hwLimitSwitchFwdOld_ && !hwLimitSwitchFwd_){
        traj_->setExecute(0);
        //Switch direction
        currSeqDirection_=ECMC_DIR_FORWARD;
        seqState_=2;
      }
      break;

    case 2: //Wait for standstill and then trigger move
      retValue=checkHWLimitsAndStop(1,0); // should never go to forward limit switch
      if(retValue){
        return retValue;
      }
      if(!traj_->getBusy()){
        traj_->setTargetVel(-homeVelOffCam_); //low speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);//Trigg new movement
        seqState_=3;
      }
      else{
        traj_->setExecute(0);
      }
      break;

    case 3: //Latch encoder value on falling or rising edge of home sensor
      retValue=checkHWLimitsAndStop(1,0); // should never go to forward or backward limit switch
      if(retValue){
        return retValue;
      }
      if(homeSensor_!=homeSensorOld_){
        homePosLatch1_=enc_->getActPos();
        seqState_=4;
      }
      break;

    case 4: //Wait for falling or rising edge of home sensor then stop
      retValue=checkHWLimitsAndStop(1,1); // should never go to forward or backward limit switch
      if(retValue){
        return retValue;
      }

      if(homeSensor_!=homeSensorOld_){
        traj_->setExecute(0);
        seqState_=5;
      }
      break;

    case 5: //Wait for standstill and the trigger move
      retValue=checkHWLimitsAndStop(1,1); // should never go to forward or backward limit switch
      if(retValue){
        return retValue;
      }

      if(!traj_->getBusy()){ //Trigg new movement
        traj_->setTargetVel(homeVelOffCam_); //low speed
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setExecute(1);
        seqState_=6;
      }
      else{
        traj_->setExecute(0);
      }
      break;

    case 6: //Latch value on falling or rising edge of home sensor. Stop motion
      retValue=checkHWLimitsAndStop(1,1); // should never go to forward or backward limit switch
      if(retValue){
        return retValue;
      }
      if(homeSensor_!=homeSensorOld_){
        homePosLatch2_=enc_->getActPos();
        traj_->setExecute(0);
        seqState_=7;
      }
      break;

    case 7:  //Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit
      retValue=checkHWLimitsAndStop(1,1); // should never go to forward limit or backward switch
      if(retValue){
        return retValue;
      }
      traj_->setExecute(0);
      if(!traj_->getBusy()){ //Wait for stop ramp ready
        data_->command_.positionTarget=traj_->getCurrentPosSet();
        if(mon_->getAtTarget())//Wait for controller to settle in order to minimize bump
        {
          double currPos=enc_->getActPos()-((homePosLatch2_+homePosLatch1_)/2)+homePosition_;
          traj_->setCurrentPosSet(currPos);
          enc_->setActPos(currPos);
          enc_->setHomed(true);
          cntrl_->reset();  //TODO.. Should this really be needed.. Error should be zero anyway.. Controller jumps otherwise.. PROBLEM
          homePosLatch1_=0;
          homePosLatch2_=0;
          stopSeq();
        }
      }
      break;
  }

  return -seqState_;
}

int ecmcSequencer::checkHWLimitsAndStop(bool checkBWD,bool checkFWD)
{
  if(traj_==NULL){
    stopSeq();
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_TRAJ_NULL);
  }
  if(mon_==NULL){
    stopSeq();
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_MON_NULL);
  }

  if(!data_->status_.limitFwd && checkFWD){
    stopSeq();
    traj_->setExecute(0);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_SEQ_FAILED);
  }

  if(!data_->status_.limitBwd && checkBWD){
    stopSeq();
    traj_->setExecute(0);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_SEQ_FAILED);
  }
  return 0;
}

int ecmcSequencer::getSeqState()
{
  return seqState_;
}

int ecmcSequencer::stopSeq(){
  if(traj_!=NULL){
    traj_->setExecute(false);
  }

  if(mon_!=NULL){

      data_->command_.enableSoftLimitBwd=enableSoftLimitBwdBackup_;
      data_->command_.enableSoftLimitFwd=enableSoftLimitFwdBackup_;
  }

  seqInProgress_=false;
  busy_=false;
  seqState_=0;
  seqTimeCounter_=0;
  return 0;
}

int ecmcSequencer::validate()
{
  return 0;
}

int ecmcSequencer::setSequenceTimeout(int timeout)
{
  seqTimeout_=timeout;
  return 0;
}

int ecmcSequencer::setExtTrajIF(ecmcMasterSlaveIF * extIf)
{
  externalInputTrajectoryIF_=extIf;
  return 0;
}

int ecmcSequencer::getExtTrajSetpoint(double *pos)
{
  if(!externalInputTrajectoryIF_){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_SEQ_EXTERNAL_DATA_INTERFACE_NULL);
  }

  int error=externalInputTrajectoryIF_->validate();
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  error=externalInputTrajectoryIF_->refreshInputs();
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }
  double tempPos=0;
  error=externalInputTrajectoryIF_->getExtInputPos(ECMC_TRANSFORM_VAR_TYPE_TRAJ,&tempPos);  //For this axis
  if(error){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,error);
  }

  *pos=tempPos;
  return 0;
}

int ecmcSequencer::setAxisDataRef(ecmcAxisData* data)
{
  data_= data;
  return 0;
}
