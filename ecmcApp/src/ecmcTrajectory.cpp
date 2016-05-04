#include "ecmcTrajectory.hpp"

#include <stdio.h>

ecmcTrajectory::ecmcTrajectory(double sampleTime) : ecmcError(), ecmcMasterSlaveIF(sampleTime)
{
  initVars();
  sampleTime_=sampleTime;
  setSampleTime(sampleTime_);
  initTraj();
}

ecmcTrajectory::ecmcTrajectory(double velocityTarget, double acceleration, double deceleration, double jerk,double sampleTime) : ecmcError(), ecmcMasterSlaveIF(sampleTime)
{
  initVars();
  velocityTarget_=velocityTarget;
  acceleration_=acceleration;
  deceleration_=deceleration;
  decelerationEmergency_=deceleration;  //Make parameter
  jerk_=jerk;
  sampleTime_=sampleTime;
  setSampleTime(sampleTime_);
  initTraj();
}

ecmcTrajectory::~ecmcTrajectory()
{

}

void ecmcTrajectory::initVars()
{
  errorReset();
  distToStop_=0;
  velocityTarget_=0;
  acceleration_=0;
  deceleration_=0;
  decelerationEmergency_=0;
  jerk_=0;
  sampleTime_=1;
  posSetMinus1_=0;
  posSetMinus2_=0;
  targetPosition_=0;
  currentPositionSetpoint_=0;
  stepACC_=0;
  stepDEC_=0;
  stepNOM_=0;
  stepDECEmerg_=0;
  velocity_=0;
  softLimitBwd_=0;
  softLimitFwd_=0;
  trajInProgress_=false;
  enableSoftLimitBwd_=true;
  enableSoftLimitFwd_=true;
  index_=0;
  execute_=0;
  executeOld_=0;
  currentPosInterlock_=true;
  startPosition_=0;
  hwLimitSwitchFwd_=false;
  hwLimitSwitchBwd_=false;
  relPosOffset_=0;
  enable_=false;
  enableOld_=false;
  motionMode_=ECMC_MOVE_MODE_POS;
  prevStepSize_=0;
  coordSysteMode_=ECMC_COORD_ABS;
  setDataSourceType(ECMC_DATA_SOURCE_INTERNAL);
  setDirection_=ECMC_DIR_FORWARD;
  actDirection_=ECMC_DIR_FORWARD;
  gearRatio_=1;
  externalInterlock_=ECMC_INTERLOCK_BOTH_LIMITS;
  interlockStatus_=ECMC_INTERLOCK_NONE;
  enableHardLimitFWDAlarms_=false;
  enableHardLimitBWDAlarms_=false;
}

void ecmcTrajectory::initTraj()
{
  stepNOM_=std::abs(velocityTarget_)*sampleTime_;

  //if(_dAcceleration==0){
  //  _dStepACC=_dStepNOM; //No acc phase = linear trajectory instead of trapezoidal
  //}
  //else{
  stepACC_=0.5*acceleration_*sampleTime_*sampleTime_*2; //WHY*2?????
  //}

  //if(_dDeceleration==0){
  //  _dStepDEC=_dStepNOM;
  //}
  //else{
   stepDEC_=0.5*deceleration_*sampleTime_*sampleTime_*2;
  //}

  //if(_dDecelerationEmergency==0){
  //  _dStepDECEmerg=_dStepNOM*5;
  //}
  //else{
  stepDECEmerg_=0.5*decelerationEmergency_*sampleTime_*sampleTime_*2;
  //}
}

double ecmcTrajectory::getCurrentPosSet()
{
  return currentPositionSetpoint_;
}

void ecmcTrajectory::setCurrentPosSet(double posSet)
{
  currentPositionSetpoint_=posSet;
  posSetMinus2_=currentPositionSetpoint_;
  posSetMinus1_=currentPositionSetpoint_;
  prevStepSize_=0;
  velocity_=0;
  setDirection_=ECMC_DIR_STANDSTILL;
  actDirection_=ECMC_DIR_STANDSTILL;
}

double ecmcTrajectory::getNextPosSet()
{
  //"Main" of trajectory generator. Needs to be called exactly once per cycle. Updates trajectory setpoint

  double nextSetpoint=currentPositionSetpoint_;
  double nextVelocity=velocity_;

  bool stopped=false;
  stopMode nStopMode=ECMC_STOP_MODE_EMERGENCY;

  if (!trajInProgress_ || currentPosInterlock_ || !enable_){

    if(execute_ && !enable_){
      setErrorID(ERROR_TRAJ_EXECUTE_BUT_NO_ENABLE);
    }
    posSetMinus2_=currentPositionSetpoint_;
    posSetMinus1_=currentPositionSetpoint_;
    prevStepSize_=0;
    velocity_=0;
    setDirection_=ECMC_DIR_STANDSTILL;
    actDirection_=ECMC_DIR_STANDSTILL;
    return currentPositionSetpoint_;
  }
  index_++;
  transformRefresh(); //Only interlock valid for internal source (no transformation of trajectory value allowed)
  if(getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL){
    nextSetpoint=internalTraj(&nextVelocity);
  }
  else{ //EXTERNAL via transform
    getExtInputPos(&nextSetpoint);
    getExtInputVel(&nextVelocity);
  }

  if(!getExtInputInterlock()){ //1=OK, 0=STOP
    externalInterlock_=ECMC_INTERLOCK_TRANSFORM;
  }

  actDirection_=checkDirection(currentPositionSetpoint_,nextSetpoint);

  if(coordSysteMode_==ECMC_COORD_REL){
    nextSetpoint=checkSoftLimits(nextSetpoint-relPosOffset_,actDirection_);
  }
  else{
    nextSetpoint=checkSoftLimits(nextSetpoint,actDirection_);
  }
  actDirection_=checkDirection(currentPositionSetpoint_,nextSetpoint);

  nStopMode=checkInterlocks(actDirection_,nextSetpoint);//Check if next position is allowed

  if(nStopMode!=ECMC_STOP_MODE_RUN){//STOP=>calculate new setpoint and velocity for stop ramp.
    nextSetpoint=moveStop(nStopMode,currentPositionSetpoint_, velocity_,velocityTarget_,&stopped,&nextVelocity);
    if(stopped){
      trajInProgress_=false;
      velocity_=0;
    }
  }

  currentPositionSetpoint_=updateSetpoint(nextSetpoint,nextVelocity); //Take new setpoint from internal or external source as current setpoint

  return currentPositionSetpoint_;
}

double ecmcTrajectory::updateSetpoint(double nextSetpoint,double nextVelocity)
{
  posSetMinus2_=posSetMinus1_;
  posSetMinus1_=currentPositionSetpoint_;
  currentPositionSetpoint_=nextSetpoint;
  prevStepSize_=currentPositionSetpoint_-posSetMinus1_;
  velocity_=nextVelocity;
  distToStop_=distToStop(velocity_);
  getOutputDataInterface()->setPosition(currentPositionSetpoint_);
  getOutputDataInterface()->setVelocity(velocity_);
  return currentPositionSetpoint_;
}

double ecmcTrajectory::internalTraj(double *actVelocity)
{
  double posSetTemp=currentPositionSetpoint_;
  switch(motionMode_){
    case ECMC_MOVE_MODE_POS:
      posSetTemp=movePos(currentPositionSetpoint_,targetPosition_,distToStop_,velocity_,velocityTarget_);
      *actVelocity=(posSetTemp-currentPositionSetpoint_)/sampleTime_;
      break;
    case ECMC_MOVE_MODE_VEL:
      posSetTemp=moveVel(currentPositionSetpoint_, velocity_,velocityTarget_);
      *actVelocity=(posSetTemp-currentPositionSetpoint_)/sampleTime_;
      break;
  }
  return posSetTemp;
}

double ecmcTrajectory::moveVel(double currSetpoint, double currVelo,double targetVelo)
{
  double positionStep=0;
  double posSetTemp=0;

  if(!execute_ && std::abs(currVelo)<std::abs(targetVelo)*0.01){ //TODO not nice with hardcoded zero speed tolerance. change to setting
    stop();
    return currSetpoint;
  }

  if(std::abs(currVelo)<std::abs(targetVelo)){
    positionStep=std::abs(prevStepSize_)+stepACC_;
  }
  else{
    positionStep=stepNOM_;
  }

  if(setDirection_==ECMC_DIR_FORWARD){ //dir forward
    posSetTemp=currSetpoint+positionStep;
  }
  else{
    posSetTemp=currSetpoint-positionStep;
  }
  return posSetTemp;
}

double ecmcTrajectory::movePos(double currSetpoint,double targetSetpoint,double stopDistance, double currVelo,double targetVelo)
{
  double positionStep=0;
  double posSetTemp=0;
  bool timeToStop=false;
  bool changeDir=false;

  changeDir=((targetSetpoint-currSetpoint)*currVelo<0 && std::abs(currVelo))>0;  //Setpoint change on the fly
  timeToStop=stopDistance > std::abs(targetSetpoint-currSetpoint) || changeDir;

  if(!timeToStop){
    if(std::abs(currVelo)<std::abs(targetVelo)){
      positionStep=std::abs(prevStepSize_)+stepACC_;
    }
    else{
      positionStep=stepNOM_;
    }
  }
  else{
    positionStep=std::abs(prevStepSize_)-stepDEC_;
  }

  if (setDirection_==ECMC_DIR_FORWARD){//Positive direction
    if(currVelo>=0){
      posSetTemp=currSetpoint+positionStep;
    }
    else{
      posSetTemp=currSetpoint-positionStep;//Change direction if target position changed during the movement..
    }
    if(currSetpoint>targetSetpoint){
      posSetTemp=targetSetpoint;
      stop();
    }
  }
  else{//Negative
    if (currVelo<=0){
      posSetTemp=currSetpoint-positionStep;
    }
    else{
      posSetTemp=currSetpoint+positionStep;//Change direction if target position changed during the movement..
    }

    if(currSetpoint<targetSetpoint){
      posSetTemp=targetSetpoint;
      stop();
    }
  }
  return posSetTemp;
}

double ecmcTrajectory::moveStop(stopMode stopMode,double currSetpoint, double currVelo,double targetVelo, bool *stopped,double *velocity)
{
  double positionStep;
  double posSetTemp=0;
  *stopped=false;
  motionDirection nDir;
  if(currVelo>=0)
    nDir=ECMC_DIR_FORWARD;
  else
    nDir=ECMC_DIR_BACKWARD;

  if(stopMode==ECMC_STOP_MODE_EMERGENCY){
    positionStep=std::abs(prevStepSize_)-stepDECEmerg_;  //Brake fast if HWlimit
  }
  else{
    positionStep=std::abs(prevStepSize_)-stepDEC_;
  }

  if(nDir==ECMC_DIR_FORWARD){
    posSetTemp=checkSoftLimits(currSetpoint+positionStep,nDir);
  }
  else if(nDir==ECMC_DIR_BACKWARD){
    posSetTemp=checkSoftLimits(currSetpoint-positionStep,nDir);
  }

  *velocity=(posSetTemp-currSetpoint)/sampleTime_;

  if(std::abs(currVelo)<0.001*std::abs(targetVelo)  || positionStep<2.1*stepDECEmerg_){  //TODO will not work always!! Need better way to calculate stand still new parameter
    *stopped=true;
    *velocity=0;
    return currSetpoint;
  }

  return posSetTemp;
}

stopMode ecmcTrajectory::checkInterlocks(motionDirection dir,double newSetpoint)
{
  //All interlocks should be evaluated in this function.. NOTE LIST IMPORTANT INTERLOCK FIRST (EMERGENCY LEVEL)!

  interlockStatus_=ECMC_INTERLOCK_NONE;
  if(dir==ECMC_DIR_FORWARD && !hwLimitSwitchFwd_){
    if(enableHardLimitFWDAlarms_){
      setErrorID(ERROR_TRAJ_HARD_LIMIT_FWD_INTERLOCK);
    }
    interlockStatus_=ECMC_INTERLOCK_HARD_FWD;
    return ECMC_STOP_MODE_EMERGENCY;
  }

  if(dir==ECMC_DIR_BACKWARD && !hwLimitSwitchBwd_){
    if(enableHardLimitBWDAlarms_){
      setErrorID(ERROR_TRAJ_HARD_LIMIT_BWD_INTERLOCK);
    }
    interlockStatus_=ECMC_INTERLOCK_HARD_BWD;
    return ECMC_STOP_MODE_EMERGENCY;
  }

  if(externalInterlock_!=ECMC_INTERLOCK_NONE){
    interlockStatus_=externalInterlock_;
    switch(externalInterlock_){
      case ECMC_INTERLOCK_POSITION_LAG:
        setErrorID(ERROR_TRAJ_POS_LAG_INTERLOCK);
        return ECMC_STOP_MODE_EMERGENCY;
        break;
      case ECMC_INTERLOCK_BOTH_LIMITS:
        setErrorID(ERROR_TRAJ_BOTH_LIMIT_INTERLOCK);
        return ECMC_STOP_MODE_EMERGENCY;
        break;
      case ECMC_INTERLOCK_TRANSFORM:
        setErrorID(ERROR_TRAJ_TRANSFORM_INTERLOCK_ERROR);
        return ECMC_STOP_MODE_EMERGENCY;
        break;
      case ECMC_INTERLOCK_MAX_SPEED:
        setErrorID(ERROR_TRAJ_MAX_SPEED_INTERLOCK);
        return ECMC_STOP_MODE_EMERGENCY;
      default:
        break;
    }
  }

  if(!execute_){
    interlockStatus_=ECMC_INTERLOCK_NO_EXECUTE;
    return ECMC_STOP_MODE_NORMAL;
  }

  if(enableSoftLimitBwd_ && (dir==ECMC_DIR_BACKWARD) && (distToStop_ > std::abs(softLimitBwd_-newSetpoint) || newSetpoint <= softLimitBwd_)){ //DistToStop will be form previous cycle
    setErrorID(ERROR_TRAJ_SOFT_LIMIT_BWD_INTERLOCK);
    interlockStatus_=ECMC_INTERLOCK_SOFT_BWD;
    return ECMC_STOP_MODE_NORMAL;
  }

  if(enableSoftLimitFwd_ && (dir==ECMC_DIR_FORWARD) && (distToStop_ > std::abs(softLimitFwd_-newSetpoint) || newSetpoint >= softLimitFwd_)){ //DistToStop will be form previous cycle
    setErrorID(ERROR_TRAJ_SOFT_LIMIT_FWD_INTERLOCK);
    interlockStatus_=ECMC_INTERLOCK_SOFT_FWD;
    return ECMC_STOP_MODE_NORMAL;
  }

  return ECMC_STOP_MODE_RUN;  //No interlock active
}

double ecmcTrajectory::distToStop(double vel)
{
  return std::abs(0.5*vel*vel/deceleration_)+2*std::abs(vel*sampleTime_)-2*stepDEC_;;
}

void ecmcTrajectory::setTargetPos(double pos)
{
  targetPosition_=pos;
  index_=0;
}

bool ecmcTrajectory::getBusy()
{
  return trajInProgress_;
}

int ecmcTrajectory::getIndex()
{
  return index_;
}

double ecmcTrajectory::getVel()
{
  return velocity_;  //Velocity of trajectory (use for velocity feedforward)
}

void ecmcTrajectory::setTargetVel(double velTarget)
{
  velocityTarget_=velTarget; //Steady state velocity setpoint
  initTraj();
}

double ecmcTrajectory::getTargetVel()
{
  return velocityTarget_;
}

void ecmcTrajectory::setAcc(double acc)
{
  acceleration_=acc;
  initTraj();
}

void ecmcTrajectory::setEmergDec(double dec)
{
  decelerationEmergency_=dec;
  initTraj();
}

void ecmcTrajectory::setDec(double dec)
{
  deceleration_=dec;
  if(decelerationEmergency_==0){
    decelerationEmergency_=deceleration_*3;
  }
  initTraj();
}

void ecmcTrajectory::setJerk(double jerk)  //Not used
{
  jerk_=jerk;
  initTraj();
}

double ecmcTrajectory::getAcc()
{
  return acceleration_;
}

double ecmcTrajectory::getDec()
{
  return deceleration_;
}

double ecmcTrajectory::getJerk()
{
  return jerk_;
}

void ecmcTrajectory::setSoftLimitBwd(double limit)
{
  softLimitBwd_=limit;
}

void ecmcTrajectory::setSoftLimitFwd(double limit)
{
  softLimitFwd_=limit;
}

double ecmcTrajectory::getSoftLimitBwd()
{
  return softLimitBwd_;
}

double ecmcTrajectory::getSoftLimitFwd()
{
  return softLimitFwd_;
}

void ecmcTrajectory::setEnableSoftLimitBwd(bool enable)
{
  enableSoftLimitBwd_=enable;
}

void ecmcTrajectory::setEnableSoftLimitFwd(bool enable)
{
  enableSoftLimitFwd_=enable;
}

bool ecmcTrajectory::getEnableSoftLimitBwd()
{
  return enableSoftLimitBwd_;
}

bool ecmcTrajectory::getEnableSoftLimitFwd()
{
  return enableSoftLimitFwd_;
}

double ecmcTrajectory::checkSoftLimits(double posSetpoint, motionDirection direction)
{
  double set=posSetpoint;
  if(posSetpoint>softLimitFwd_ && enableSoftLimitFwd_ && direction==ECMC_DIR_FORWARD){
    set=currentPositionSetpoint_; //Don't move futher
  }

  if(posSetpoint<softLimitBwd_ && enableSoftLimitBwd_ && direction==ECMC_DIR_BACKWARD){
    set=currentPositionSetpoint_;
  }
  return set;
}

void ecmcTrajectory::setHardLimitFwd(bool switchState)
{ //1=OK 0=break
  hwLimitSwitchFwd_=switchState;
}

void ecmcTrajectory::setHardLimitBwd(bool switchState)
{ //1=OK 0=break
  hwLimitSwitchBwd_=switchState;
}

double ecmcTrajectory::getTargetPos()
{
  return targetPosition_;
}

bool ecmcTrajectory::getExecute()
{
  return execute_;
}

void ecmcTrajectory::setEnable(bool enable)
{
  enableOld_=enable_;
  enable_=enable;
  velocity_=0;
  if(!enableOld_ && enable_ ){
    posSetMinus2_=startPosition_;
    posSetMinus1_=startPosition_;
    currentPositionSetpoint_=startPosition_;
    prevStepSize_=0;
    velocity_=0;
    distToStop_=0;
    getOutputDataInterface()->setPosition(currentPositionSetpoint_);
    getOutputDataInterface()->setVelocity(velocity_);
  }
}

bool ecmcTrajectory::getEnable()
{
  return enable_;
}

void ecmcTrajectory::setExecute(bool execute)
{
  executeOld_=execute_;
  execute_=execute;
  if(!enable_ && execute_){
    setErrorID(ERROR_TRAJ_NOT_ENABLED);
    execute_=false;
    velocity_=0;
    return;
  }

  if(!executeOld_ && execute_){
    currentPositionSetpoint_=startPosition_;
    if(getDataSourceType()==ECMC_DATA_SOURCE_INTERNAL){
      switch(motionMode_){
        case ECMC_MOVE_MODE_VEL:
          if(velocityTarget_<0){
            setDirection_=ECMC_DIR_BACKWARD;
          }
          else{
            setDirection_=ECMC_DIR_FORWARD;
          }
          break;
        case ECMC_MOVE_MODE_POS:
          if(targetPosition_<currentPositionSetpoint_){
            setDirection_=ECMC_DIR_BACKWARD;
          }
          else{
            setDirection_=ECMC_DIR_FORWARD;
          }
          break;
      }
    }
    else{ //External source
      if(getNumExtInputSources()==0){
        setErrorID(ERROR_TRAJ_EXT_MASTER_SOURCE_NULL);
        relPosOffset_=0;
        execute_=false;
        velocity_=0;
        trajInProgress_=false;
         return;
      }
      if(coordSysteMode_==ECMC_COORD_REL){
        transformRefresh();
        double tempPos=0;
        int errorCode=getExtInputPos(&tempPos);
        if(errorCode){
          setErrorID(errorCode);
          relPosOffset_=0;
          execute_=false;
          velocity_=0;
          trajInProgress_=false;
          return;
        }
        relPosOffset_=tempPos-startPosition_;
      }
      else{
        relPosOffset_=0;
      }
      setDirection_=ECMC_DIR_STANDSTILL; //undefined
    }

    if(!trajInProgress_){
      posSetMinus1_=currentPositionSetpoint_;
      posSetMinus2_=currentPositionSetpoint_;
      velocity_=0;
    }
    initTraj();
    currentPositionSetpoint_=startPosition_;
    trajInProgress_=true; //Trigger new trajectory
  }
}

void ecmcTrajectory::setStartPos(double pos)
{
  startPosition_=pos;
  currentPosInterlock_=false;
  if(!enable_){
    getOutputDataInterface()->setPosition(startPosition_);
    getOutputDataInterface()->setVelocity(0);
  }
}

bool ecmcTrajectory::getInterlocked()
{
  return currentPosInterlock_ || externalInterlock_!=ECMC_INTERLOCK_NONE;  //TODO not complete...*/;
}

void ecmcTrajectory::setInterlock(interlockTypes interlock)
{
  externalInterlock_=interlock;
}

bool ecmcTrajectory::getHardLimitFwd()
{
  return hwLimitSwitchFwd_;
}

bool ecmcTrajectory::getHardLimitBwd()
{
  return hwLimitSwitchBwd_;
}

void  ecmcTrajectory::setMotionMode(motionMode mode)
{
  motionMode_=mode;
}

void  ecmcTrajectory::setCoordSystMode(coordSystMode mode)
{
  coordSysteMode_=mode;
}

void ecmcTrajectory::stop()
{
  velocity_=0;
  trajInProgress_=false;
}

int ecmcTrajectory::setGearRatio(double ratioNum, double ratioDenom)
{
  if(std::abs(ratioDenom)>0){
    gearRatio_=ratioNum/ratioDenom;
    return 0;
  }
  else{
    gearRatio_=ratioNum;
    return setErrorID(ERROR_TRAJ_GEAR_RATIO_DENOM_ZERO);
  }
}

double ecmcTrajectory::getGearRatio()
{
  return gearRatio_;
}

interlockTypes ecmcTrajectory::getInterlockStatus()
{
  return interlockStatus_;
}

int ecmcTrajectory::setEnableHardLimitFWDAlarm(bool enable)
{
  enableHardLimitFWDAlarms_=enable;
  return 0;
}

int ecmcTrajectory::setEnableHardLimitBWDAlarm(bool enable)
{
  enableHardLimitBWDAlarms_=enable;
  return 0;
}

double ecmcTrajectory::getSampleTime()
{
  return sampleTime_;
}

int ecmcTrajectory::validate()
{
  if(sampleTime_<=0){
    return setErrorID(ERROR_TRAJ_INVALID_SAMPLE_TIME);
  }

  if(getDataSourceType()!=ECMC_DATA_SOURCE_INTERNAL){  //EXTERNAL

    int sourceCount=getNumExtInputSources();
    if(sourceCount==0){
      return setErrorID(ERROR_TRAJ_EXT_MASTER_SOURCE_COUNT_ZERO);
    }


    ecmcTransform * transform=getExtInputTransform();
    if(transform==NULL){
      return setErrorID(ERROR_TRAJ_TRANSFORM_NULL);
    }
    if(transform->validate()){
      return setErrorID(ERROR_TRAJ_TRANSFORM_VALIDATION_ERROR);
    }
  }

  if(getOutputDataInterface()==NULL){
    return setErrorID(ERROR_TRAJ_SLAVE_INTERFACE_NULL);
  }
  return 0;
}

motionDirection ecmcTrajectory::checkDirection(double oldPos,double newPos)
{
  if(newPos>oldPos){
    return ECMC_DIR_FORWARD;
  }
  else if(newPos<oldPos){
    return ECMC_DIR_BACKWARD;
  }

  return ECMC_DIR_STANDSTILL;
}

int ecmcTrajectory::getCurrentExternalSetpoint(double* value)
{
  int errorCode=transformRefresh();
  if(errorCode){
    return errorCode;
  }

  return getExtInputPos(value);
}
