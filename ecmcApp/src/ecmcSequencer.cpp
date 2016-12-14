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
  //homeSensor_=false;
  homeSensorOld_=false;
  execute_=false;
  executeOld_=false;
  seqInProgress_=false;
  currSeqDirection_=ECMC_DIR_FORWARD;
  seqState_=0;
  command_=ECMC_CMD_JOG;
  cmdData_=0;
  traj_=NULL;
  enc_=NULL;
  mon_=NULL;
  cntrl_=NULL;
  busy_=false;
  jogVel_=0;
  homeVelTwordsCam_=0; //ADR command
  homeVelOffCam_=0; //ADR command
  homeDirection_=ECMC_DIR_FORWARD;
  homePosition_=0;
  targetPosition_=0;
  targetVelocity_=0;
  jogFwd_=false;
  jogBwd_=false;
  enableSoftLimitBwdBackup_=false;
  enableSoftLimitFwdBackup_=false;
  hwLimitSwitchBwd_=false;
  hwLimitSwitchFwd_=false;
  hwLimitSwitchBwdOld_=false;
  hwLimitSwitchFwdOld_=false;
  homePosLatch1_=0;
  homePosLatch2_=0;
  seqTimeout_=0; //disabled
  seqTimeCounter_=0;
  externalExecute_=false;
}

void ecmcSequencer::execute()
{  //Cyclic execution
  if(traj_==NULL){
    setErrorID(ERROR_SEQ_TRAJ_NULL);
    return;
  }
  if(mon_==NULL){
    setErrorID(ERROR_SEQ_MON_NULL);
    return;
  }

  hwLimitSwitchBwdOld_=hwLimitSwitchBwd_;
  hwLimitSwitchFwdOld_=hwLimitSwitchFwd_;
  hwLimitSwitchBwd_=mon_->getHardLimitBwd();
  hwLimitSwitchFwd_=mon_->getHardLimitFwd();

  homeSensorOld_=homeSensor_;
  homeSensor_=mon_->getHomeSwitch();

  if(!seqInProgress_){
    return;
  }
  seqTimeCounter_++;
  if(seqTimeCounter_>seqTimeout_ && seqTimeout_>0){
    setErrorID(ERROR_SEQ_TIMEOUT);
    stopSeq();
    return;
  }
  int seqReturnVal=0;
  switch(command_){
    case ECMC_CMD_JOG:
      ;
      break;
    case ECMC_CMD_HOMING:
      switch (cmdData_){
        case 0:
          seqReturnVal=seq1SimpleHoming();
          if(seqReturnVal>0){//Error
            setErrorID(seqReturnVal);
            stopSeq();
          }
          else if(seqReturnVal==0){//Homing ready
            stopSeq();
          }
          break;
        case 1:
          seqReturnVal=seqHoming1();
          if(seqReturnVal>0){//Error
            setErrorID(seqReturnVal);
            stopSeq();
          }
          else if(seqReturnVal==0){//Homing ready
            stopSeq();
          }
          break;
        case 2:
          seqReturnVal=seqHoming2();
          if(seqReturnVal>0){//Error
            setErrorID(seqReturnVal);
            stopSeq();
          }
          else if(seqReturnVal==0){//Homing ready
            stopSeq();
          }
          break;
        case 3:
          seqReturnVal=seqHoming3();
          if(seqReturnVal>0){//Error
            setErrorID(seqReturnVal);
            stopSeq();
          }
          else if(seqReturnVal==0){//Homing ready
            stopSeq();
          }
          break;
        case 4:
          seqReturnVal=seqHoming4();
          if(seqReturnVal>0){//Error
            setErrorID(seqReturnVal);
            stopSeq();
          }
          else if(seqReturnVal==0){//Homing ready
            stopSeq();
          }
          break;
        case 5:
          seqReturnVal=seqHoming5();
          if(seqReturnVal>0){//Error
            setErrorID(seqReturnVal);
            stopSeq();
          }
          else if(seqReturnVal==0){//Homing ready
            stopSeq();
          }
          break;
        case 6:
          seqReturnVal=seqHoming6();
          if(seqReturnVal>0){//Error
            setErrorID(seqReturnVal);
            stopSeq();
          }
          else if(seqReturnVal==0){//Homing ready
            stopSeq();
          }
          break;
        default:
          setErrorID(ERROR_SEQ_CMD_DATA_UNDEFINED);
          break;
      }
      break;
    default:
      setErrorID(ERROR_SEQ_CMD_UNDEFINED);
      break;
  }
}

int  ecmcSequencer::setExecute(bool execute)
{
  if(traj_==NULL){
    return setErrorID(ERROR_SEQ_TRAJ_NULL);
  }

  executeOld_=execute_;
  execute_=execute;
  seqInProgress_=false;
  seqState_=0;

  switch (command_){
    case ECMC_CMD_JOG:
      //Triggered via jog inputs
      break;
    case ECMC_CMD_MOVEVEL:
      if(execute_ && !executeOld_){
        traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
        traj_->setTargetVel(targetVelocity_);
        targetPosition_=checkSoftLimits(targetPosition_); //not needed
        traj_->setTargetPos(targetPosition_);
      }
      traj_->setExecute(execute_);
      break;
    case ECMC_CMD_MOVEREL:
      if(execute_ && !executeOld_){
        traj_->setMotionMode(ECMC_MOVE_MODE_POS);
        traj_->setTargetVel(targetVelocity_);
        traj_->setTargetPos(checkSoftLimits(traj_->getCurrentPosSet()+targetPosition_));
      }
      traj_->setExecute(execute_);
      break;
    case ECMC_CMD_MOVEABS:
      if(execute_ && !executeOld_){
        traj_->setMotionMode(ECMC_MOVE_MODE_POS);
        traj_->setTargetVel(targetVelocity_);
        switch(cmdData_){
          case 0: //Normal positioning
            targetPosition_=checkSoftLimits(targetPosition_);
            traj_->setTargetPos(targetPosition_);
            break;
/*          case 1: //Go to external transform current value (transform value as TragetPosition)
            double targPos=0;
            int errorCode=traj_->getCurrentExternalSetpoint(&targPos);
            if(errorCode){
              return errorCode;execute_
            }
            targetPosition_=checkSoftLimits(targPos);
            traj_->setTargetPos(targetPosition_);
            break;*/
        }
      }
      traj_->setExecute(execute_);
      break;
    case ECMC_CMD_MOVEMODULO:
      return setErrorID(ERROR_SEQ_COMMAND_NOT_SUPPORTED);
      break;
    case ECMC_CMD_HOMING:
      if(execute_ && !executeOld_){
        stopSeq();
        if(traj_!=NULL && enc_!=NULL && mon_!=NULL && cntrl_!=NULL){
          seqInProgress_=true;
          busy_=true;
        }
        else{
          if(traj_==NULL){
            return setErrorID(ERROR_SEQ_TRAJ_NULL);
          }
          else{
            traj_->setExecute(false);
          }

          if(enc_==NULL){
            return setErrorID(ERROR_SEQ_ENC_NULL);
          }

          if(mon_==NULL){
            return setErrorID(ERROR_SEQ_MON_NULL);
          }

          if(cntrl_==NULL){
            return setErrorID(ERROR_SEQ_CNTRL_NULL);
          }
        }
      }
      else if(!execute_){
        stopSeq();
        traj_->setExecute(execute_);
      }
      break;
    case ECMC_CMD_SUPERIMP:
      return setErrorID(ERROR_SEQ_COMMAND_NOT_SUPPORTED);
      break;
    case ECMC_CMD_GEAR:
      return setErrorID(ERROR_SEQ_COMMAND_NOT_SUPPORTED);
      break;
    default:
      return setErrorID(ERROR_SEQ_COMMAND_NOT_SUPPORTED);
      break;
  }

  return 0;
}

bool ecmcSequencer::getExecute()
{
  return execute_;
}

void ecmcSequencer::setCommand(motionCommandTypes command)
{
  command_=command;
}

motionCommandTypes ecmcSequencer::getCommand()
{
  return command_;
}

void ecmcSequencer::setCmdData(int cmdData)
{
  cmdData_=cmdData;
}

int ecmcSequencer::getCmdData()
{
  return cmdData_;
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
    setErrorID(ERROR_SEQ_TRAJ_NULL);
    return false;
  }
  if(command_==ECMC_CMD_HOMING){
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
    setErrorID(ERROR_SEQ_SOFT_LIMIT_FWD);
    return;
  }
  if(pos<tempPos){
    setErrorID(ERROR_SEQ_SOFT_LIMIT_BWD);
    return;
  }

  targetPosition_=pos;

  if(mon_==NULL){
    setErrorID(ERROR_SEQ_MON_NULL);
    return;
  }
  mon_->setTargetPos(pos);
}

double ecmcSequencer::getTargetPos()
{
  return targetPosition_;
}

void ecmcSequencer::setTargetVel(double velTarget)
{
  targetVelocity_=velTarget;
}

double ecmcSequencer::getTargetVel()
{
  return targetVelocity_;
}

void ecmcSequencer::setJogFwd(bool jog)
{
  jogFwd_=jog;
  if(traj_==NULL){
    setErrorID(ERROR_SEQ_TRAJ_NULL);
    return;
  }
  if(command_==ECMC_CMD_JOG && jogFwd_){
    traj_->setTargetVel(jogVel_);
    traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
    traj_->setExecute(jogFwd_);
  }
  else{
    traj_->setTargetVel(targetVelocity_);
  }
}

bool ecmcSequencer::getJogFwd()
{
  if(traj_==NULL){
    setErrorID(ERROR_SEQ_TRAJ_NULL);
    return false;
  }
  return jogFwd_;
}

void ecmcSequencer::setJogBwd(bool jog)
{
  jogBwd_=jog;
  if(traj_==NULL){
    setErrorID(ERROR_SEQ_TRAJ_NULL);
    return;
  }
  if(command_==ECMC_CMD_JOG && jogBwd_){
    traj_->setTargetVel(-jogVel_);
    traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
    traj_->setExecute(jogBwd_);
  }
  else{
    traj_->setTargetVel(targetVelocity_);
  }
}

bool ecmcSequencer::getJogBwd()
{
  if(traj_==NULL){
    setErrorID(ERROR_SEQ_TRAJ_NULL);
    return false;
  }
  return jogBwd_;
}

double ecmcSequencer::checkSoftLimits(double posSetpoint)
{
  if(!mon_){
    setErrorID(ERROR_SEQ_MON_NULL);
    return posSetpoint;
  }

  double dSet=posSetpoint;
  double currPos=enc_->getActPos();

  if(posSetpoint>mon_->getSoftLimitFwd() && mon_->getEnableSoftLimitFwd() && posSetpoint>currPos){
    dSet=mon_->getSoftLimitFwd();
  }
  if(posSetpoint<mon_->getSoftLimitBwd() && mon_->getEnableSoftLimitBwd()&& posSetpoint<currPos){
    dSet=mon_->getSoftLimitBwd();;
  }
  return dSet;
}

ecmcTrajectoryTrapetz * ecmcSequencer::getTraj()
{
  return traj_;
}

int ecmcSequencer::seq1SimpleHoming() //nCmdData==0
{
  // Return > 0 error
  // Return < 0 progress
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam (hw switches are monitored)
  // State 1 Wait for positive edge of home sensor then stop motion
  // State 2 Wait for stop then trigger motion in opposite direction, speed =_dHomeVelOffCam
  // StatnRetValuee 3 Wait for falling edge of home sensor. Sets offsets of encoder and stops motion.. Sequence are successfully done..

  int retValue=checkHWLimitsAndStop(true,true); // should never go to a limit switch
  if(retValue){
    seqInProgress_=false;
    traj_->setExecute(false);
    seqState_=0;
    return retValue;
  }

  //Sequence code
  switch(seqState_){
    case 0:  //Set parameters and start initial motion
      enc_->setHomed(false);
      traj_->setExecute(false);
      currSeqDirection_=homeDirection_;  //StartDirection
      if(currSeqDirection_==ECMC_DIR_FORWARD){
        traj_->setTargetVel(homeVelTwordsCam_);
      }
      else{
        traj_->setTargetVel(-homeVelTwordsCam_);
      }
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(true);
      seqState_=1;
      retValue= -seqState_;
      break;
    case 1: //Wait for home switch positive edge the go back slow in the other direction
      if(homeSensor_ && !homeSensorOld_){
        traj_->setExecute(false);
        //Switch direction
        if(currSeqDirection_==ECMC_DIR_FORWARD){
          traj_->setTargetVel(-homeVelOffCam_);
          currSeqDirection_=ECMC_DIR_BACKWARD;
        }
        else if(currSeqDirection_==ECMC_DIR_BACKWARD){
          traj_->setTargetVel(homeVelOffCam_);
          currSeqDirection_=ECMC_DIR_FORWARD;
        }
        seqState_=2;
      }
      retValue= -seqState_;
      break;
    case 2: //Wait for standstill and the trigger move
      if(!traj_->getBusy()){ //Trigg new movement
        traj_->setExecute(true);enc_->setHomed(false);
        seqState_=3;
      }
      else{
        traj_->setExecute(false);
      }
      retValue= -seqState_;
      break;
    case 3: //Wait for falling edge of home sensor then set offset in encoder and stop motion
      if(!homeSensor_ && homeSensorOld_){
        seqInProgress_=false;
        seqState_=0;
        traj_->setExecute(false);
        enc_->setOffset(homePosition_-enc_->getActPos());
        enc_->setHomed(true);
        retValue= 0; //Ready
      }
      else{
        retValue=-seqState_;
      }
      break;
  }
  return retValue;
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
      enableSoftLimitBwdBackup_=mon_->getEnableSoftLimitBwd(); //Read setting to be able to restore later
      enableSoftLimitFwdBackup_=mon_->getEnableSoftLimitFwd(); //Read setting to be able to restore later
      mon_->setEnableSoftLimitBwd(false); //Disable softlimits for homing
      mon_->setEnableSoftLimitFwd(false);
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
        mon_->setTargetPos(traj_->getCurrentPosSet());
        if(mon_->getAtTarget())//Wait for controller to settle in order to minimize bump
        {
          double currPos=enc_->getActPos()-homePosLatch1_+homePosition_;
          traj_->setCurrentPosSet(currPos);
          enc_->setActPos(currPos);
          enc_->setHomed(true);
          mon_->setActPos(currPos);
          mon_->setTargetPos(currPos);
          cntrl_->reset();  //TODO.. Should this really be needed.. Error should be zero anyway.. Controller jumps otherwise.. PROBLEM
          cntrl_->setEnable(true);
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
      enableSoftLimitBwdBackup_=mon_->getEnableSoftLimitBwd(); //Read setting to be able to restore later
      enableSoftLimitFwdBackup_=mon_->getEnableSoftLimitFwd(); //Read setting to be able to restore later
      mon_->setEnableSoftLimitBwd(false); //Disable softlimits for homing
      mon_->setEnableSoftLimitFwd(false);
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
        mon_->setTargetPos(traj_->getCurrentPosSet());
        if(mon_->getAtTarget())//Wait for controller to settle in order to minimize bump
        {
          double currPos=enc_->getActPos()-homePosLatch1_+homePosition_;
          traj_->setCurrentPosSet(currPos);
          enc_->setActPos(currPos);
          enc_->setHomed(true);
          mon_->setActPos(currPos);
          mon_->setTargetPos(currPos);
          cntrl_->reset();  //TODO.. Should this really be needed.. Error should be zero anyway.. Controller jumps otherwise.. PROBLEM
          cntrl_->setEnable(true);
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
      enableSoftLimitBwdBackup_=mon_->getEnableSoftLimitBwd(); //Read setting to be able to restore later
      enableSoftLimitFwdBackup_=mon_->getEnableSoftLimitFwd(); //Read setting to be able to restore later
      mon_->setEnableSoftLimitBwd(false); //Disable softlimits for homing
      mon_->setEnableSoftLimitFwd(false);
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
      mon_->setEnableSoftLimitBwd(false); //Disable softlimits for homing
      mon_->setEnableSoftLimitFwd(false);
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
        mon_->setTargetPos(traj_->getCurrentPosSet());
        if(mon_->getAtTarget())//Wait for controller to settle in order to minimize bump
        {
          double currPos=enc_->getActPos()-homePosLatch1_+homePosition_;
          traj_->setCurrentPosSet(currPos);
          enc_->setActPos(currPos);
          enc_->setHomed(true);
          mon_->setActPos(currPos);
          mon_->setTargetPos(currPos);
          cntrl_->reset();  //TODO.. Should this really be needed.. Error should be zero anyway.. Controller jumps otherwise.. PROBLEM
          cntrl_->setEnable(true);
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
      enableSoftLimitBwdBackup_=mon_->getEnableSoftLimitBwd(); //Read setting to be able to restore later
      enableSoftLimitFwdBackup_=mon_->getEnableSoftLimitFwd(); //Read setting to be able to restore later
      mon_->setEnableSoftLimitBwd(false); //Disable softlimits for homing
      mon_->setEnableSoftLimitFwd(false);
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
        mon_->setTargetPos(traj_->getCurrentPosSet());
        if(mon_->getAtTarget())//Wait for controller to settle in order to minimize bump
        {
          double currPos=enc_->getActPos()-homePosLatch1_+homePosition_;
          traj_->setCurrentPosSet(currPos);
          enc_->setActPos(currPos);
          enc_->setHomed(true);
          mon_->setActPos(currPos);
          mon_->setTargetPos(currPos);
          cntrl_->reset();  //TODO.. Should this really be needed.. Error should be zero anyway.. Controller jumps otherwise.. PROBLEM
          cntrl_->setEnable(true);
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
      enableSoftLimitBwdBackup_=mon_->getEnableSoftLimitBwd(); //Read setting to be able to restore later
      enableSoftLimitFwdBackup_=mon_->getEnableSoftLimitFwd(); //Read setting to be able to restore later
      mon_->setEnableSoftLimitBwd(false); //Disable softlimits for homing
      mon_->setEnableSoftLimitFwd(false);
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
        mon_->setTargetPos(traj_->getCurrentPosSet());
        if(mon_->getAtTarget())//Wait for controller to settle in order to minimize bump
        {
          double currPos=enc_->getActPos()-((homePosLatch2_+homePosLatch1_)/2)+homePosition_;
          traj_->setCurrentPosSet(currPos);
          enc_->setActPos(currPos);
          enc_->setHomed(true);
          mon_->setActPos(currPos);
          mon_->setTargetPos(currPos);
          cntrl_->reset();  //TODO.. Should this really be needed.. Error should be zero anyway.. Controller jumps otherwise.. PROBLEM
          cntrl_->setEnable(true);
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
      enableSoftLimitBwdBackup_=mon_->getEnableSoftLimitBwd(); //Read setting to be able to restore later
      enableSoftLimitFwdBackup_=mon_->getEnableSoftLimitFwd(); //Read setting to be able to restore later
      mon_->setEnableSoftLimitBwd(false); //Disable softlimits for homing
      mon_->setEnableSoftLimitFwd(false);
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
        mon_->setTargetPos(traj_->getCurrentPosSet());
        if(mon_->getAtTarget())//Wait for controller to settle in order to minimize bump
        {
          double currPos=enc_->getActPos()-((homePosLatch2_+homePosLatch1_)/2)+homePosition_;
          traj_->setCurrentPosSet(currPos);
          enc_->setActPos(currPos);
          enc_->setHomed(true);
          mon_->setActPos(currPos);
          mon_->setTargetPos(currPos);
          cntrl_->reset();  //TODO.. Should this really be needed.. Error should be zero anyway.. Controller jumps otherwise.. PROBLEM
          cntrl_->setEnable(true);
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
    return setErrorID(ERROR_SEQ_TRAJ_NULL);
  }
  if(mon_==NULL){
    stopSeq();
    return setErrorID(ERROR_SEQ_MON_NULL);
  }

  if(!mon_->getHardLimitFwd() && checkFWD){
    stopSeq();
    traj_->setExecute(0);
    return setErrorID(ERROR_SEQ_SEQ_FAILED);
  }

  if(!mon_->getHardLimitBwd() && checkBWD){
    stopSeq();
    traj_->setExecute(0);
    return setErrorID(ERROR_SEQ_SEQ_FAILED);
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
    mon_->setEnableSoftLimitBwd(enableSoftLimitBwdBackup_);
    mon_->setEnableSoftLimitFwd(enableSoftLimitFwdBackup_);
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

int ecmcSequencer::setExternalExecute(bool execute)
{
  externalExecute_=execute;
/// TODO FUNCTIONALLITY NOT IMPLEMETED YET
  return 0;
}
