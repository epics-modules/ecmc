/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcAxisSequencer.cpp
*
*  Created on: Jan 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcAxisSequencer.h"

ecmcAxisSequencer::ecmcAxisSequencer() {
  initVars();
}

ecmcAxisSequencer::~ecmcAxisSequencer()
{}

void ecmcAxisSequencer::initVars() {
  homeSensorOld_         = false;
  executeOld_            = false;
  seqInProgress_         = false;
  currSeqDirection_      = ECMC_DIR_FORWARD;
  seqState_              = 0;
  traj_                  = NULL;
  enc_                   = NULL;
  mon_                   = NULL;
  cntrl_                 = NULL;
  drv_                   = NULL;
  jogVel_                = 0;
  homeVelTwordsCam_      = 0;
  homeVelOffCam_         = 0;
  homePosition_          = 0;
  jogFwd_                = false;
  jogBwd_                = false;
  hwLimitSwitchBwdOld_   = false;
  hwLimitSwitchFwdOld_   = false;
  homePosLatch1_         = 0;
  homePosLatch2_         = 0;
  seqTimeout_            = 0;
  seqTimeCounter_        = 0;
  seqStateOld_           = 0;
  seqInProgressOld_      = 0;
  localSeqBusy_          = false;
  data_                  = NULL;
  oldencRawAbsPosReg_    = 0;
  encRawAbsPosReg_       = 0;
  homeLatchCountOffset_  = 0;
  homeLatchCountAct_     = 0;
  overUnderFlowLatch_    = ECMC_ENC_NORMAL;
  enablePos_             = true;
  enableConstVel_        = true;
  enableHome_            = true;
  homeEnablePostMove_    = false;
  homePostMoveTargetPos_ = 0;
  seqPosHomeState_       = 0;
}

// Cyclic execution
void ecmcAxisSequencer::execute() {
  if (traj_ == NULL) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_TRAJ_NULL);
    return;
  }

  if (mon_ == NULL) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_MON_NULL);
    return;
  }

  // Reset busy (set in setExecute)
  if (data_->command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
    if (data_->command_.command == ECMC_CMD_HOMING) {
      data_->status_.busy = localSeqBusy_ || traj_->getBusy();
    } else {
      data_->status_.busy = traj_->getBusy();
    }
  } else {    // Sync to other axis
    data_->status_.busy = true;
  }

  hwLimitSwitchBwdOld_ = hwLimitSwitchBwd_;
  hwLimitSwitchFwdOld_ = hwLimitSwitchFwd_;
  hwLimitSwitchBwd_    = data_->status_.limitBwd;
  hwLimitSwitchFwd_    = data_->status_.limitFwd;
  homeSensorOld_       = homeSensor_;
  homeSensor_          = data_->status_.homeSwitch;
  seqInProgressOld_    = seqInProgress_;

  if (!seqInProgress_) {
    return;
  }

  seqStateOld_         = seqState_;
  seqTimeCounter_++;

  if ((seqTimeCounter_ > seqTimeout_) && (seqTimeout_ > 0)) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_TIMEOUT);
    stopSeq();
    return;
  }
  int seqReturnVal          = 0;
  ecmcHomingType homingType = (ecmcHomingType)data_->command_.cmdData;

  switch (data_->command_.command) {
  case ECMC_CMD_JOG:
    ;
    break;

  case ECMC_CMD_HOMING:

    switch (homingType) {
    case ECMC_SEQ_HOME_LOW_LIM:
      seqReturnVal = seqHoming1();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    case ECMC_SEQ_HOME_HIGH_LIM:
      seqReturnVal = seqHoming2();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    case ECMC_SEQ_HOME_LOW_LIM_HOME:
      seqReturnVal = seqHoming3();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    case ECMC_SEQ_HOME_HIGH_LIM_HOME:
      seqReturnVal = seqHoming4();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    case ECMC_SEQ_HOME_LOW_LIM_HOME_HOME:
      seqReturnVal = seqHoming5();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    case ECMC_SEQ_HOME_HIGH_LIM_HOME_HOME:
      seqReturnVal = seqHoming6();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    case ECMC_SEQ_HOME_BWD_HOME:
      seqReturnVal = seqHoming7();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    case ECMC_SEQ_HOME_FWD_HOME:
      seqReturnVal = seqHoming8();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    case ECMC_SEQ_HOME_BWD_HOME_HOME:
      seqReturnVal = seqHoming9();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;
    
    case ECMC_SEQ_HOME_FWD_HOME_HOME:
      seqReturnVal = seqHoming10();      

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    case ECMC_SEQ_HOME_LOW_LIM_INDEX:
      seqReturnVal = seqHoming11();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    case ECMC_SEQ_HOME_HIGH_LIM_INDEX:
      seqReturnVal = seqHoming12();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    case ECMC_SEQ_HOME_LOW_LIM_SINGLE_TURN_ABS:
      seqReturnVal = seqHoming21();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    case ECMC_SEQ_HOME_HIGH_LIM_SINGLE_TURN_ABS:
      seqReturnVal = seqHoming22();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    case ECMC_SEQ_HOME_SET_POS:
      seqReturnVal = seqHoming15();

      if (seqReturnVal > 0) {  // Error
        setErrorID(__FILE__, __FUNCTION__, __LINE__, seqReturnVal);
        stopSeq();
      } else if (seqReturnVal == 0) {  // Homing ready
        stopSeq();
      }
      break;

    default:
      setErrorID(__FILE__, __FUNCTION__, __LINE__,
                 ERROR_SEQ_CMD_DATA_UNDEFINED);
      break;
    }
    break;

  default:
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_CMD_UNDEFINED);
    break;
  }
}

int ecmcAxisSequencer::setExecute(bool execute) {

  int errorCode=0;
  if (traj_ == NULL) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_TRAJ_NULL);
  }

  executeOld_             = data_->command_.execute;
  data_->command_.execute = execute;
  seqInProgress_          = false;
  seqState_               = 0;

  if (data_->command_.execute  && !executeOld_) {
    
    errorCode = checkVelAccDec();
    if (errorCode) {
      return errorCode;
    }
  }
  
  switch (data_->command_.command) {
  case ECMC_CMD_JOG:

    // Triggered via jog inputs
    break;

  case ECMC_CMD_MOVEVEL:

    if (data_->command_.execute  && !executeOld_) {
      if(!enableConstVel_) {
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_MOTION_CMD_NOT_ENABLED);
      }

      data_->status_.busy = true;
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setTargetVel(data_->command_.velocityTarget);                  
    }

    errorCode =  traj_->setExecute(data_->command_.execute);
    if (errorCode) {
      return errorCode;
    }

    break;

  case ECMC_CMD_MOVEREL:

    if (data_->command_.execute && !executeOld_) {
      if(!enablePos_) {
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_MOTION_CMD_NOT_ENABLED);
      }

      data_->status_.busy = true;
      traj_->setMotionMode(ECMC_MOVE_MODE_POS);
      traj_->setTargetVel(data_->command_.velocityTarget);
      traj_->setTargetPos(data_->command_.positionTarget);
    }
    errorCode = traj_->setExecute(data_->command_.execute);
    if (errorCode) {
      return errorCode;
    }

    break;

  case ECMC_CMD_MOVEABS:

    if (data_->command_.execute && !executeOld_) {
      if(!enablePos_) {
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_MOTION_CMD_NOT_ENABLED);
      }

      if(data_->command_.moduloRange>0) {
        if(data_->command_.positionTarget<0 || data_->command_.positionTarget>= data_->command_.moduloRange) {
          LOGERR(
            "%s/%s:%d: ERROR: Invalid target position. Allowed range 0..%lf (0x%x).\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->command_.moduloRange,
            ERROR_SEQ_TARGET_POS_OUT_OF_RANGE);
          return setErrorID(__FILE__,
                            __FUNCTION__,
                            __LINE__,
                              ERROR_SEQ_TARGET_POS_OUT_OF_RANGE);
        }
      }
      data_->status_.busy = true;
      traj_->setMotionMode(ECMC_MOVE_MODE_POS);
      traj_->setTargetVel(data_->command_.velocityTarget);

      switch (data_->command_.cmdData) {
      case 0:     // Normal positioning
        traj_->setTargetPos(data_->command_.positionTarget);
        break;

      // Go to external transform curr value (as targetPosition)
      case 1:
        double targPos   = 0;
        int    errorCode = getExtTrajSetpoint(&targPos);

        if (errorCode) {
          return errorCode;
        }
        traj_->setTargetPos(data_->command_.positionTarget);
        break;
      }
    }
    errorCode = traj_->setExecute(data_->command_.execute);
    if (errorCode) {
      return errorCode;
    }

    break;

  case ECMC_CMD_MOVEMODULO:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_COMMAND_NOT_SUPPORTED);

    break;

  case ECMC_CMD_HOMING:

    if (data_->command_.execute && !executeOld_) {

      stopSeq();
      if(!enableHome_) {
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_MOTION_CMD_NOT_ENABLED);
      }

      if ((traj_ != NULL) && (enc_ != NULL) && (mon_ != NULL) &&
          (cntrl_ != NULL || data_->axisType_ == ECMC_AXIS_TYPE_VIRTUAL)) {
        seqInProgress_      = true;
        localSeqBusy_       = true;
        data_->status_.busy = true;
      } else {
        if (traj_ == NULL) {
          return setErrorID(__FILE__,
                            __FUNCTION__,
                            __LINE__,
                            ERROR_SEQ_TRAJ_NULL);
        } else {
          traj_->setExecute(false);
        }

        if (enc_ == NULL) {
          return setErrorID(__FILE__,
                            __FUNCTION__,
                            __LINE__,
                            ERROR_SEQ_ENC_NULL);
        }

        if (mon_ == NULL) {
          return setErrorID(__FILE__,
                            __FUNCTION__,
                            __LINE__,
                            ERROR_SEQ_MON_NULL);
        }

        if (cntrl_ == NULL && data_->axisType_ != ECMC_AXIS_TYPE_VIRTUAL) {
          return setErrorID(__FILE__,
                            __FUNCTION__,
                            __LINE__,
                            ERROR_SEQ_CNTRL_NULL);
        }
      }
    } else if (!data_->command_.execute) {
      stopSeq();
      errorCode = traj_->setExecute(data_->command_.execute);
      if (errorCode) {
        return errorCode;
      }
    }
    break;

  case ECMC_CMD_SUPERIMP:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_COMMAND_NOT_SUPPORTED);

    break;

  case ECMC_CMD_GEAR:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_COMMAND_NOT_SUPPORTED);

    break;

  default:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_COMMAND_NOT_SUPPORTED);

    break;
  }

  return 0;
}

bool ecmcAxisSequencer::getExecute() {
  return data_->command_.execute;
}

void ecmcAxisSequencer::setCommand(motionCommandTypes command) {
  data_->command_.command = command;
}

motionCommandTypes ecmcAxisSequencer::getCommand() {
  return data_->command_.command;
}

void ecmcAxisSequencer::setCmdData(int cmdData) {
  data_->command_.cmdData = cmdData;
}

int ecmcAxisSequencer::getCmdData() {
  return data_->command_.cmdData;
}

void ecmcAxisSequencer::setTraj(ecmcTrajectoryTrapetz *traj) {
  traj_ = traj;
}

void ecmcAxisSequencer::setEnc(ecmcEncoder *enc) {
  enc_ = enc;
}

void ecmcAxisSequencer::setMon(ecmcMonitor *mon) {
  mon_ = mon;
}

void ecmcAxisSequencer::setCntrl(ecmcPIDController *cntrl) {
  cntrl_ = cntrl;
}

void ecmcAxisSequencer::setDrv(ecmcDriveBase *drv) {
  drv_ = drv;
}

bool ecmcAxisSequencer::getBusy() {
  return data_->status_.busy;
}

void ecmcAxisSequencer::setJogVel(double vel) {
  jogVel_ = vel;
}

double ecmcAxisSequencer::getJogVel() {
  return jogVel_;
}

int ecmcAxisSequencer::setHomeVelTwordsCam(double vel) {
  homeVelTwordsCam_ = vel;
  return 0;
}

int ecmcAxisSequencer::setHomeVelOffCam(double vel) {
  homeVelOffCam_ = vel;
  return 0;
}

double ecmcAxisSequencer::getHomeVelTwordsCam() {
  return homeVelTwordsCam_;
}

double ecmcAxisSequencer::getHomeVelOffCam() {
  return homeVelOffCam_;
}

void ecmcAxisSequencer::setHomePosition(double pos) {
  homePosition_ = pos;
}

double ecmcAxisSequencer::getHomePosition() {
  return homePosition_;
}

void ecmcAxisSequencer::setTargetPos(double pos) {

  if(data_->command_.command == ECMC_CMD_MOVEREL) {
    pos = traj_->getCurrentPosSet() + pos;
  }
  pos = checkSoftLimits(pos);
  data_->command_.positionTarget = pos;

  // "On the fly change"
  if( getBusy() ) {
    traj_->setTargetPos(data_->command_.positionTarget);
  }  
}

void ecmcAxisSequencer::setTargetPos(double pos, bool force) {
  if (force) {
    data_->command_.positionTarget = pos;
  } else {
    setTargetPos(pos);
  }
}

double ecmcAxisSequencer::getTargetPos() {
  return data_->command_.positionTarget;
}

void ecmcAxisSequencer::setTargetVel(double velTarget) {
  // silent restriction to max velocity
  if(mon_->getEnableMaxVelMon()) {
    double maxVelo = std::abs(mon_->getMaxVel());
    if(velTarget >= 0) { // positive velo
      if(velTarget > maxVelo){
        velTarget = maxVelo;
      }
    } else {  // negative velo
      if(velTarget < -maxVelo){
        velTarget = -maxVelo;
      }
    }
  }

  data_->command_.velocityTarget = velTarget;
  traj_->setTargetVel(velTarget);
}

double ecmcAxisSequencer::getTargetVel() {
  return data_->command_.velocityTarget;
}

void ecmcAxisSequencer::setJogFwd(bool jog) {
  jogFwd_ = jog;

  if (traj_ == NULL) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_TRAJ_NULL);
    return;
  }

  if ((data_->command_.command == ECMC_CMD_JOG) && jogFwd_) {
    traj_->setTargetVel(jogVel_);
    traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
    traj_->setExecute(jogFwd_);
  } else {
    traj_->setTargetVel(data_->command_.velocityTarget);
  }
}

bool ecmcAxisSequencer::getJogFwd() {
  if (traj_ == NULL) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_TRAJ_NULL);
    return false;
  }
  return jogFwd_;
}

void ecmcAxisSequencer::setJogBwd(bool jog) {
  jogBwd_ = jog;

  if (traj_ == NULL) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_TRAJ_NULL);
    return;
  }

  if ((data_->command_.command == ECMC_CMD_JOG) && jogBwd_) {
    traj_->setTargetVel(-jogVel_);
    traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
    traj_->setExecute(jogBwd_);
  } else {
    traj_->setTargetVel(data_->command_.velocityTarget);
  }
}

bool ecmcAxisSequencer::getJogBwd() {
  if (traj_ == NULL) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_TRAJ_NULL);
    return false;
  }
  return jogBwd_;
}

double ecmcAxisSequencer::checkSoftLimits(double posSetpoint) {
  if (!mon_) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_MON_NULL);
    return posSetpoint;
  }

  double dSet    = posSetpoint;
  double dAct   = data_->status_.currentPositionSetpoint;

  // soft limit FWD
  if ((posSetpoint > data_->command_.softLimitFwd) &&
      data_->command_.enableSoftLimitFwd && dSet > dAct) {
    dSet = dAct;
  }

  // soft limit BWD
  if ((posSetpoint < data_->command_.softLimitBwd) &&
      data_->command_.enableSoftLimitBwd && dSet < dAct) {
    dSet = dAct;
  }
  return dSet;
}

ecmcTrajectoryTrapetz * ecmcAxisSequencer::getTraj() {
  return traj_;
}

int ecmcAxisSequencer::seqHoming15() {  // nCmdData==15
  // Return = 0 ready
  // State 0 set encoder position to same as fHomePosition
  // Sequence code
  switch (seqState_) {
    case 0:    // Set parameters and start initial motion
      traj_->setCurrentPosSet(homePosition_);
      traj_->setTargetPos(homePosition_);
      enc_->setActPos(homePosition_);
      enc_->setHomed(true);
      data_->status_.currentPositionActual = homePosition_;
      data_->status_.currentPositionSetpoint = homePosition_;
    
      if(cntrl_) {
        cntrl_->reset();
      }
      
      // trigg post home motion if enabled and not alreday triggered
      if(homeEnablePostMove_) {
        if(seqState_ < 1000) {
          seqState_ = 1000;
        }
      } 
      else {
        stopSeq();
      }
      break;
  }

  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming1() {  // nCmdData==1
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for negative edge of bwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in positive direction
  // State 3 Latch encoder value on falling or rising edge of bwd limit switch sensor.
  // State 4 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit


  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  // Sequence code
  switch (seqState_) {
  case 0:    // Set parameters and start initial motion
    initHomingSeq();

    if (hwLimitSwitchBwd_) {
      currSeqDirection_ = ECMC_DIR_BACKWARD;  // StartDirection
      traj_->setTargetVel(-homeVelTwordsCam_);   // high speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 1;
    } else {  // Already at bwd limit jump to step 2
      currSeqDirection_ = ECMC_DIR_FORWARD;  // StartDirection
      seqState_         = 2;
    }
    break;

  // Wait for negative limit switch and turn other direction
  case 1:
    if (hwLimitSwitchBwdOld_ && !hwLimitSwitchBwd_) {
      traj_->setExecute(0);

      // Switch direction
      currSeqDirection_ = ECMC_DIR_FORWARD;
      seqState_         = 2;
    }
    break;

  // Wait for standstill and then trigger move
  case 2:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {
      traj_->setTargetVel(homeVelOffCam_);   // low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);  // Trigg new movement
      seqState_ = 3;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Latch encoder value on falling or rising edge of bwd limit switch
  case 3:
    // should never go to forward or backward limit switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }

    if (hwLimitSwitchBwd_ != hwLimitSwitchBwdOld_) {
      homePosLatch1_ = enc_->getActPos();
      seqState_      = 4;
    }
    break;

  // Wait for standstill before rescale of encoder.
  // Calculate encoder offset and set encoder homed bit.
  case 4:
    // should never go to forward limit or backward switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }
    traj_->setExecute(0);

    if (!traj_->getBusy()) {  // Wait for stop ramp ready
      data_->command_.positionTarget = traj_->getCurrentPosSet();

      if (mon_->getAtTarget()) {  // Wait for controller to settle in order to minimize bump
        double currPos = enc_->getActPos() - homePosLatch1_ + homePosition_;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }
  
  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming2() {  // nCmdData==2
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for positive edge of bwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in negative direction
  // State 3 Latch encoder value on falling or rising edge of fwd limit switch sensor.
  // State 4 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit

  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  // Sequence code
  switch (seqState_) {
  // Set parameters and start initial motion
  case 0:
    initHomingSeq();

    if (hwLimitSwitchFwd_) {
      currSeqDirection_ = ECMC_DIR_FORWARD;  // StartDirection
      traj_->setTargetVel(homeVelTwordsCam_);  // High speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 1;
    } else {  // Already at bwd limit jump to step 2
      currSeqDirection_ = ECMC_DIR_BACKWARD;  // StartDirection
      seqState_         = 2;
    }
    break;

  // Wait for positive limit switch and turn other direction
  case 1:
    if (hwLimitSwitchFwdOld_ && !hwLimitSwitchFwd_) {
      traj_->setExecute(0);

      // Switch direction
      currSeqDirection_ = ECMC_DIR_BACKWARD;
      seqState_         = 2;
    }
    break;

  // Wait for standstill and then trigger move
  case 2:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {
      traj_->setTargetVel(-homeVelOffCam_);  // Low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);  // Trigg new movement
      seqState_ = 3;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Latch encoder value on falling or rising edge of home sensor
  case 3:
    // should never go to forward or backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }

    if (hwLimitSwitchFwd_ != hwLimitSwitchFwdOld_) {
      homePosLatch1_ = enc_->getActPos();
      seqState_      = 4;
    }
    break;
  // Wait for standstill before rescale of encoder.
  // Calculate encoder offset and set encoder homed bit.
  case 4:
    // should never go to forward limit or backward switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }
    traj_->setExecute(0);

    if (!traj_->getBusy()) {  // Wait for stop ramp ready
      data_->command_.positionTarget = traj_->getCurrentPosSet();

      if (mon_->getAtTarget()) {  // Wait for controller to settle in order to minimize bump
        double currPos = enc_->getActPos() - homePosLatch1_ + homePosition_;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }

  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming3() {  // nCmdData==3
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for negative edge of bwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in positive direction
  // State 3 Latch encoder value on falling or rising edge of home sensor.
  // State 4 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit

  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  // Sequence code
  switch (seqState_) {
  // Set parameters and start initial motion
  case 0:
    initHomingSeq();

    if (hwLimitSwitchBwd_) {
      currSeqDirection_ = ECMC_DIR_BACKWARD;  // StartDirection
      traj_->setTargetVel(-homeVelTwordsCam_);   // high speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 1;
    } else {  // Already at bwd limit jump to step 2
      currSeqDirection_ = ECMC_DIR_FORWARD;  // StartDirection
      seqState_         = 2;
    }
    break;

  // Wait for negative limit switch and turn other direction
  case 1:
    if (hwLimitSwitchBwdOld_ && !hwLimitSwitchBwd_) {
      traj_->setExecute(0);

      // Switch direction
      currSeqDirection_ = ECMC_DIR_FORWARD;
      seqState_         = 2;
    }
    break;

  // Wait for standstill and then trigger move
  case 2:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {
      traj_->setTargetVel(homeVelOffCam_);   // low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);  // Trigg new movement
      seqState_ = 3;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Latch encoder value on falling or rising edge of home sensor
  case 3:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      LOGERR(
        "%s/%s:%d: ERROR: Failed to find first flank on home sensor before limit switch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
    }

    if (homeSensor_ != homeSensorOld_) {
      homePosLatch1_ = enc_->getActPos();
      seqState_      = 4;
    }
    break;

  // Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit
  case 4:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }
    traj_->setExecute(0);

    if (!traj_->getBusy()) {  // Wait for stop ramp ready
      data_->command_.positionTarget = traj_->getCurrentPosSet();
      // Wait for controller to settle in order to minimize bump.
      if ((mon_->getAtTarget() && mon_->getEnableAtTargetMon()) ||
          !mon_->getEnableAtTargetMon()) {
        double currPos = enc_->getActPos() - homePosLatch1_ + homePosition_;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }

  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming4() {  // nCmdData==4
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for positive edge of bwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in negative direction
  // State 3 Latch encoder value on falling or rising edge of home sensor.
  // State 4 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit

  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  // Sequence code
  switch (seqState_) {
  // Set parameters and start initial motion
  case 0:
    initHomingSeq();

    if (hwLimitSwitchFwd_) {
      currSeqDirection_ = ECMC_DIR_FORWARD;  // StartDirection
      traj_->setTargetVel(homeVelTwordsCam_);   // high speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 1;
    } else {  // Already at bwd limit jump to step 2
      currSeqDirection_ = ECMC_DIR_BACKWARD;  // StartDirection
      seqState_         = 2;
    }
    break;

  // Wait for negative limit switch and turn other direction
  case 1:

    if (hwLimitSwitchFwdOld_ && !hwLimitSwitchFwd_) {
      traj_->setExecute(0);

      // Switch direction
      currSeqDirection_ = ECMC_DIR_BACKWARD;
      seqState_         = 2;
    }
    break;

  // Wait for standstill and then trigger move
  case 2:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {
      traj_->setTargetVel(-homeVelOffCam_);  // Low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);  // Trigg new movement
      seqState_ = 3;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Latch encoder value on falling or rising edge of home sensor
  case 3:
    // should never go to forward or backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      LOGERR(
        "%s/%s:%d: ERROR: Failed to find first flank on home sensor before limit switch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
    }

    if (homeSensor_ != homeSensorOld_) {
      homePosLatch1_ = enc_->getActPos();
      seqState_      = 4;
    }
    break;

  // Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit
  case 4:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }
    traj_->setExecute(0);
    // Wait for stop ramp ready
    if (!traj_->getBusy()) {
      data_->command_.positionTarget = traj_->getCurrentPosSet();
      // Wait for controller to settle in order to minimize bump
      if ((mon_->getAtTarget() && mon_->getEnableAtTargetMon()) ||
          !mon_->getEnableAtTargetMon()) {
        double currPos = enc_->getActPos() - homePosLatch1_ + homePosition_;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }
  
  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming5() {  // nCmdData==5
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


  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  // Sequence code
  switch (seqState_) {
  // Set parameters and start initial motion
  case 0:
    initHomingSeq();

    if (hwLimitSwitchBwd_) {
      currSeqDirection_ = ECMC_DIR_BACKWARD;  // StartDirection
      traj_->setTargetVel(-homeVelTwordsCam_);  // High speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 1;
    } else {  // Already at bwd limit jump to step 2
      currSeqDirection_ = ECMC_DIR_FORWARD;  // StartDirection
      seqState_         = 2;
    }
    break;

  // Wait for negative limit switch and turn other direction
  case 1:

    if (hwLimitSwitchBwdOld_ && !hwLimitSwitchBwd_) {
      traj_->setExecute(0);

      // Switch direction
      currSeqDirection_ = ECMC_DIR_FORWARD;
      seqState_         = 2;
    }
    break;

  // Wait for standstill and then trigger move
  case 2:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);
    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {
      traj_->setTargetVel(homeVelOffCam_);   // low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);  // Trigg new movement
      seqState_ = 3;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Latch encoder value on falling or rising edge of home sensor
  case 3:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);
    if (retValue) {
      LOGERR(
        "%s/%s:%d: ERROR: Failed to find first flank on home sensor before limit switch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
    }

    if (homeSensor_ != homeSensorOld_) {
      homePosLatch1_ = enc_->getActPos();
      seqState_      = 4;
    }
    break;

  // Wait for falling or rising edge of home sensor then stop
  case 4:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);
    if (retValue) {
      LOGERR(
        "%s/%s:%d: ERROR: Failed to find second flank on home sensor before limit switch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_NO_SECOND_HOME_SWITCH_FLANK);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_NO_SECOND_HOME_SWITCH_FLANK);
    }

    if (homeSensor_ != homeSensorOld_) {
      traj_->setExecute(0);
      seqState_ = 5;
    }
    break;

  // Wait for standstill and the trigger move
  case 5:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {  // Trigg new movement
      traj_->setTargetVel(-homeVelOffCam_);   // low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 6;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Latch value on falling or rising edge of home sensor. Stop motion
  case 6:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }

    if (homeSensor_ != homeSensorOld_) {
      homePosLatch2_ = enc_->getActPos();
      traj_->setExecute(0);
      seqState_ = 7;
    }
    break;

  // Wait for standstill before rescale of encoder.
  // Calculate encoder offset and set encoder homed bit
  case 7:
    // should never go to backward switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }
    traj_->setExecute(0);

    if (!traj_->getBusy()) {  // Wait for stop ramp ready
      data_->command_.positionTarget = traj_->getCurrentPosSet();
      // Wait for controller to settle in order to minimize bump
      if (mon_->getAtTarget()) {
        double currPos = enc_->getActPos() -
                         ((homePosLatch2_ + homePosLatch1_) / 2) +
                         homePosition_;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }

  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming6() {  // nCmdData==6
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


  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  // Sequence code
  switch (seqState_) {
  // Set parameters and start initial motion
  case 0:
    initHomingSeq();
    if (hwLimitSwitchFwd_) {
      currSeqDirection_ = ECMC_DIR_FORWARD;   // StartDirection
      traj_->setTargetVel(homeVelTwordsCam_);  // High speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 1;
    } else {  // Already at bwd limit jump to step 2
      currSeqDirection_ = ECMC_DIR_BACKWARD;  // StartDirection
      seqState_         = 2;
    }
    break;

  // Wait for negative limit switch and turn other direction
  case 1:

    if (hwLimitSwitchFwdOld_ && !hwLimitSwitchFwd_) {
      traj_->setExecute(0);

      // Switch direction
      currSeqDirection_ = ECMC_DIR_FORWARD;
      seqState_         = 2;
    }
    break;

  // Wait for standstill and then trigger move
  case 2:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {
      traj_->setTargetVel(-homeVelOffCam_);   // low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);  // Trigg new movement
      seqState_ = 3;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Latch encoder value on falling or rising edge of home sensor
  case 3:
    // should never go to forward or backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      LOGERR(
        "%s/%s:%d: ERROR: Failed to find first flank on home sensor before limit switch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
    }

    if (homeSensor_ != homeSensorOld_) {
      homePosLatch1_ = enc_->getActPos();
      seqState_      = 4;
    }
    break;

  // Wait for falling or rising edge of home sensor then stop
  case 4:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      LOGERR(
        "%s/%s:%d: ERROR: Failed to find second flank on home sensor before limit switch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_NO_SECOND_HOME_SWITCH_FLANK);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_NO_SECOND_HOME_SWITCH_FLANK);
    }

    if (homeSensor_ != homeSensorOld_) {
      traj_->setExecute(0);
      seqState_ = 5;
    }
    break;

  // Wait for standstill and the trigger move
  case 5:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {  // Trigg new movement
      traj_->setTargetVel(homeVelOffCam_);  // Low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 6;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Latch value on falling or rising edge of home sensor. Stop motion
  case 6:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }

    if (homeSensor_ != homeSensorOld_) {
      homePosLatch2_ = enc_->getActPos();
      traj_->setExecute(0);
      seqState_ = 7;
    }
    break;

  // Wait for standstill before rescale of encoder.
  // Calculate encoder offset and set encoder homed bit.
  case 7:
    // should never go to forward limit or backward switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }
    traj_->setExecute(0);

    if (!traj_->getBusy()) {  // Wait for stop ramp ready
      data_->command_.positionTarget = traj_->getCurrentPosSet();
      // Wait for controller to settle in order to minimize bump
      if (mon_->getAtTarget()) {
        double currPos = enc_->getActPos() -
                         ((homePosLatch2_ + homePosLatch1_) / 2) +
                         homePosition_;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }

  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming7() {  // nCmdData==7
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in backward direction
  // State 1 Latch encoder value on rising edge of homesentor
  // State 2 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit


  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  // Sequence code
  switch (seqState_) {
  case 0:    // Set parameters and start initial motion
    initHomingSeq();
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1,0);

    if (retValue) {
      return retValue;
    }
    currSeqDirection_ = ECMC_DIR_BACKWARD;  // StartDirection
    if (!traj_->getBusy()) {
      traj_->setTargetVel(-homeVelOffCam_);   // low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);  // Trigg new movement
      seqState_ = 1;
    }    
    break;

  // Latch encoder value on rising edge of homesensor
  case 1:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }

    if (homeSensor_  && !homeSensorOld_) {
      homePosLatch1_ = enc_->getActPos();
      seqState_      = 2;
    }
    break;

  // Wait for standstill before rescale of encoder.
  // Calculate encoder offset and set encoder homed bit.
  case 2:
    traj_->setExecute(0);

    if (!traj_->getBusy()) {  // Wait for stop ramp ready
      data_->command_.positionTarget = traj_->getCurrentPosSet();

      if (mon_->getAtTarget()) {  // Wait for controller to settle in order to minimize bump
        double currPos = enc_->getActPos() - homePosLatch1_ + homePosition_;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }

  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming8() {  // nCmdData==8
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in forward direction
  // State 1 Latch encoder value on rising edge of homesentor
  // State 2 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit


  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  // Sequence code
  switch (seqState_) {
  case 0:    // Set parameters and start initial motion
    initHomingSeq();
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(0,1);

    if (retValue) {
      return retValue;
    }
    currSeqDirection_ = ECMC_DIR_FORWARD;  // StartDirection
    if (!traj_->getBusy()) {
      traj_->setTargetVel(homeVelOffCam_);   // low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);  // Trigg new movement
      seqState_ = 1;
    }    
    break;

  // Latch encoder value on rising edge of homesensor
  case 1:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(0,1);

    if (retValue) {
      return retValue;
    }

    if (homeSensor_  && !homeSensorOld_) {
      homePosLatch1_ = enc_->getActPos();
      seqState_      = 2;
    }
    break;

  // Wait for standstill before rescale of encoder.
  // Calculate encoder offset and set encoder homed bit.
  case 2:
    traj_->setExecute(0);

    if (!traj_->getBusy()) {  // Wait for stop ramp ready
      data_->command_.positionTarget = traj_->getCurrentPosSet();

      if (mon_->getAtTarget()) {  // Wait for controller to settle in order to minimize bump
        double currPos = enc_->getActPos() - homePosLatch1_ + homePosition_;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }
  
  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming9() {  // nCmdData==9
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in backward, speed =_dHomeVelTwordsCam
  // State 1 Latch encoder value rising edge of home sensor. Continue movement
  // State 2 Wait for falling edge of home sensor then stop
  // State 3 Wait for standstill and the trigger move
  // State 4 Latch value on rising edge of home sensor. Stop motion
  // State 5 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit

  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  // Sequence code
  switch (seqState_) {
  // Set parameters and start initial motion
  case 0:
    initHomingSeq();
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1,0);

    if (retValue) {
      return retValue;
    }
    currSeqDirection_ = ECMC_DIR_BACKWARD;  // StartDirection
    if (!traj_->getBusy()) {
      traj_->setTargetVel(-homeVelOffCam_);   // low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);  // Trigg new movement
      seqState_ = 1;
    }    
    break;

  // Latch encoder value on rising edge of home sensor
  case 1:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);
    if (retValue) {
      LOGERR(
        "%s/%s:%d: ERROR: Failed to find first flank on home sensor before limit switch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
    }

    if (homeSensor_  && !homeSensorOld_) {
      homePosLatch1_ = enc_->getActPos();    
      seqState_      = 2;
    }
    break;

  // Wait for falling edge of home sensor then stop
  case 2:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);
    if (retValue) {
      LOGERR(
        "%s/%s:%d: ERROR: Failed to find second flank on home sensor before limit switch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_NO_SECOND_HOME_SWITCH_FLANK);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_NO_SECOND_HOME_SWITCH_FLANK);
    }

    if (!homeSensor_ && homeSensorOld_) {
      traj_->setExecute(0);
      seqState_ = 3;
    }
    break;

  // Wait for standstill and the trigger move
  case 3:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {  // Trigg new movement
      traj_->setTargetVel(homeVelOffCam_);   // low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 4;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Latch value on rising edge of home sensor. Stop motion
  case 4:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }

    if (homeSensor_ && !homeSensorOld_) {
      homePosLatch2_ = enc_->getActPos();
      traj_->setExecute(0);
      seqState_ = 5;
    }
    break;

  // Wait for standstill before rescale of encoder.
  // Calculate encoder offset and set encoder homed bit
  case 5:
    traj_->setExecute(0);

    if (!traj_->getBusy()) {  // Wait for stop ramp ready
      data_->command_.positionTarget = traj_->getCurrentPosSet();
      // Wait for controller to settle in order to minimize bump
      if (mon_->getAtTarget()) {
        double currPos = enc_->getActPos() -
                         ((homePosLatch2_ + homePosLatch1_) / 2) +
                         homePosition_;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }

  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming10() {  // nCmdData==10
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready
  
  // State 0 set parameters and trigger motion in forward, speed =_dHomeVelTwordsCam
  // State 1 Latch encoder value rising edge of home sensor. Continue movement
  // State 2 Wait for falling edge of home sensor then stop
  // State 3 Wait for standstill and the trigger move
  // State 4 Latch value on rising edge of home sensor. Stop motion
  // State 5 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit

  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  // Sequence code
  switch (seqState_) {
  // Set parameters and start initial motion
  case 0:
    initHomingSeq();
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0,1);

    if (retValue) {
      return retValue;
    }
    currSeqDirection_ = ECMC_DIR_FORWARD;  // StartDirection
    if (!traj_->getBusy()) {
      traj_->setTargetVel(homeVelOffCam_);   // low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);  // Trigg new movement
      seqState_ = 1;
    }    
    break;

  // Latch encoder value on rising edge of home sensor
  case 1:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0,1);
    if (retValue) {
      LOGERR(
        "%s/%s:%d: ERROR: Failed to find first flank on home sensor before limit switch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
    }

    if (homeSensor_ && !homeSensorOld_) {
      homePosLatch1_ = enc_->getActPos();    
      seqState_      = 2;
    }
    break;

  // Wait for falling edge of home sensor then stop
  case 2:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0,1);
    if (retValue) {
      LOGERR(
        "%s/%s:%d: ERROR: Failed to find second flank on home sensor before limit switch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_NO_SECOND_HOME_SWITCH_FLANK);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_NO_SECOND_HOME_SWITCH_FLANK);
    }

    if (!homeSensor_ && homeSensorOld_) {
      traj_->setExecute(0);
      seqState_ = 3;
    }
    break;

  // Wait for standstill and the trigger move
  case 3:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0,1);

    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {  // Trigg new movement
      traj_->setTargetVel(-homeVelOffCam_);   // low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 4;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Latch value on rising edge of home sensor. Stop motion
  case 4:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }

    if (homeSensor_ && !homeSensorOld_) {
      homePosLatch2_ = enc_->getActPos();
      traj_->setExecute(0);
      seqState_ = 5;
    }
    break;

  // Wait for standstill before rescale of encoder.
  // Calculate encoder offset and set encoder homed bit
  case 5:
    traj_->setExecute(0);

    if (!traj_->getBusy()) {  // Wait for stop ramp ready
      data_->command_.positionTarget = traj_->getCurrentPosSet();
      // Wait for controller to settle in order to minimize bump
      if (mon_->getAtTarget()) {
        double currPos = enc_->getActPos() -
                         ((homePosLatch2_ + homePosLatch1_) / 2) +
                         homePosition_;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }

  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming11() {  // nCmdData==11
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for negative edge of bwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in positive direction
  // State 3 Wait for leaving limit switch.
  // State 4 Arm hw latch and wait for the deifned counts of latches (rearm for each latch)
  // State 5 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit


  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  if (homeLatchCountOffset_ <= 0) {
    LOGERR(
      "%s/%s:%d: ERROR: Sequence aborted. Latch count out of range (%d). Allowed: value>0 (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      homeLatchCountOffset_,
      ERROR_SEQ_LATCH_COUNT_OUT_OF_RANGE);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_LATCH_COUNT_OUT_OF_RANGE);
  }

  // Sequence code
  switch (seqState_) {
  // Set parameters and start initial motion
  case 0:
    initHomingSeq();

    if (hwLimitSwitchBwd_) {
      currSeqDirection_ = ECMC_DIR_BACKWARD;  // StartDirection
      traj_->setTargetVel(-homeVelTwordsCam_);   // high speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 1;
    } else {  // Already at bwd limit jump to step 2
      currSeqDirection_ = ECMC_DIR_FORWARD;  // StartDirection
      seqState_         = 2;
    }
    break;

  // Wait for negative limit switch and turn other direction
  case 1:
    if (hwLimitSwitchBwdOld_ && !hwLimitSwitchBwd_) {
      traj_->setExecute(0);

      // Switch direction
      currSeqDirection_ = ECMC_DIR_FORWARD;
      enc_->setArmLatch(false);   // ensure latch is not armed
      seqState_ = 2;
    }
    break;

  case 2:   // Wait for standstill and then trigger move
    retValue = checkHWLimitsAndStop(0, 1);  // should never go to forward limit switch

    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {
      traj_->setTargetVel(homeVelOffCam_);   // low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);  // Trigg new movement
      enc_->setArmLatch(true);   // ensure latch is armed
      seqState_ = 3;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Wait for an falling or rising edge of bwd limit switch
  case 3:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }

    if (hwLimitSwitchBwd_ != hwLimitSwitchBwdOld_) {
      seqState_ = 4;
    }
    break;

  // Start supervising encoder latches
  case 4:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);
    if (retValue) {
      LOGERR(
        "%s/%s:%d: ERROR: Failed to find first flank on home sensor before limit switch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
    }

    if (enc_->getNewValueLatched()) {
      homeLatchCountAct_++;

      if (homeLatchCountAct_ >= homeLatchCountOffset_) {
        homePosLatch1_ = enc_->getLatchPosEng();
        seqState_      = 5;
      }
      enc_->setArmLatch(false);
    } else {
      enc_->setArmLatch(true);
    }
    break;

  // Wait for standstill before rescale of encoder.
  // Calculate encoder offset and set encoder homed bit.
  case 5:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);
    if (retValue) {
      return retValue;
    }
    traj_->setExecute(0);

    if (!traj_->getBusy()) {  // Wait for stop ramp ready
      data_->command_.positionTarget = traj_->getCurrentPosSet();
      // Wait for controller to settle in order to minimize bump.
      if ((mon_->getAtTarget() && mon_->getEnableAtTargetMon()) ||
          !mon_->getEnableAtTargetMon()) {
        double currPos = enc_->getActPos() - homePosLatch1_ + homePosition_;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }
  
  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming12() {  // nCmdData==12
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for positive edge of bwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in negative direction
  // State 3 Wait for leaving limit switch.
  // State 4 Arm hw latch and wait for the deifned counts of latches (rearm for each latch)
  // State 5 Wait for standstill before rescale of encoder. Calculate encoder offset and set encoder homed bit

  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  if (homeLatchCountOffset_ <= 0) {
    LOGERR(
      "%s/%s:%d: ERROR: Sequence aborted. Latch count out of range (%d). Allowed: value>0 (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      homeLatchCountOffset_,
      ERROR_SEQ_LATCH_COUNT_OUT_OF_RANGE);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_LATCH_COUNT_OUT_OF_RANGE);
  }

  // Sequence code
  switch (seqState_) {
  // Set parameters and start initial motion
  case 0:
    initHomingSeq();
    if (hwLimitSwitchFwd_) {
      currSeqDirection_ = ECMC_DIR_FORWARD;  // StartDirection
      traj_->setTargetVel(homeVelTwordsCam_);   // high speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 1;
    } else {  // Already at bwd limit jump to step 2
      currSeqDirection_ = ECMC_DIR_BACKWARD;  // StartDirection
      seqState_         = 2;
    }
    break;

  // Wait for negative limit switch and turn other direction
  case 1:

    if (hwLimitSwitchFwdOld_ && !hwLimitSwitchFwd_) {
      traj_->setExecute(0);
      // Switch direction
      currSeqDirection_ = ECMC_DIR_BACKWARD;
      enc_->setArmLatch(false);   // ensure latch is not armed
      seqState_ = 2;
    }
    break;

  // Wait for standstill and then trigger move
  case 2:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {
      traj_->setTargetVel(-homeVelOffCam_);  // Low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);  // Trigg new movement
      enc_->setArmLatch(true);  // ensure latch is armed
      seqState_ = 3;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Wait for leaving limit switch
  case 3:
    // should never go to forward or backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }

    if (hwLimitSwitchFwd_ != hwLimitSwitchFwdOld_) {
      seqState_ = 4;
    }
    break;

  // Latch encoder value on falling or rising edge of home sensor
  case 4:
    // should never go to forward or backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      LOGERR(
        "%s/%s:%d: ERROR: Failed to find first flank on home sensor before limit switch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
    }

    if (enc_->getNewValueLatched()) {
      homeLatchCountAct_++;

      if (homeLatchCountAct_ >= homeLatchCountOffset_) {
        homePosLatch1_ = enc_->getLatchPosEng();
        seqState_      = 5;
      }
      enc_->setArmLatch(false);
    } else {
      enc_->setArmLatch(true);
    }
    break;

  // Wait for standstill before rescale of encoder.
  // Calculate encoder offset and set encoder homed bit.
  case 5:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);
    if (retValue) {
      return retValue;
    }
    traj_->setExecute(0);
    if (!traj_->getBusy()) {  // Wait for stop ramp ready
      data_->command_.positionTarget = traj_->getCurrentPosSet();
      // Wait for controller to settle in order to minimize bump
      if ((mon_->getAtTarget() && mon_->getEnableAtTargetMon()) ||
          !mon_->getEnableAtTargetMon()) {
        double currPos = enc_->getActPos() - homePosLatch1_ + homePosition_;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }

  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming21() {  // nCmdData==21 Resolver homing (keep absolute bits)
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready

  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for negative edge of bwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in positive direction
  // State 3 Wait for leaving limit switch.
  // State 4 Wait for over/underflow of absolute encoder bits.
  // State 5 Wait for standstill before rescale of encoder. Keep encoder absolute bits

  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  if ((enc_->getAbsBits() < ECMC_ENCODER_ABS_BIT_MIN) ||
      (enc_->getAbsBits() > enc_->getBits())) {
    LOGERR(
      "%s/%s:%d: ERROR: Sequence aborted. Encoder absolute bit count out of range (%d). Allowed range: %d:%d. (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      enc_->getAbsBits(),
      enc_->getAbsBits(),
      enc_->getBits(),
      ERROR_SEQ_ERROR_ABS_BIT_OUT_OF_RANGE);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_ERROR_ABS_BIT_OUT_OF_RANGE);
  }

  if (homeLatchCountOffset_ <= 0) {
    LOGERR(
      "%s/%s:%d: ERROR: Sequence aborted. Latch count out of range (%d). Allowed: value>0 (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      homeLatchCountOffset_,
      ERROR_SEQ_LATCH_COUNT_OUT_OF_RANGE);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_LATCH_COUNT_OUT_OF_RANGE);
  }

  double distToAbsBitsUnderOverFlow = 0;
  double currPos                    = 0;

  // Sequence code
  switch (seqState_) {
  // Set parameters and start initial motion
  case 0:
    initHomingSeq();
    if (hwLimitSwitchBwd_) {
      currSeqDirection_ = ECMC_DIR_BACKWARD;  // StartDirection
      traj_->setTargetVel(-homeVelTwordsCam_);   // high speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 1;
    } else {  // Already at bwd limit jump to step 2
      currSeqDirection_ = ECMC_DIR_FORWARD;  // StartDirection
      seqState_         = 2;
    }
    break;

  // Wait for negative limit switch and turn other direction
  case 1:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }

    if (hwLimitSwitchBwdOld_ && !hwLimitSwitchBwd_) {
      traj_->setExecute(0);

      // Switch direction
      currSeqDirection_ = ECMC_DIR_FORWARD;
      seqState_         = 2;
    }
    break;

  // Wait for standstill and then trigger move
  case 2:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {
      traj_->setTargetVel(homeVelOffCam_);  // Low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);  // Trigg new movement
      seqState_ = 3;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Wait for an falling or rising edge of bwd limit switch
  case 3:
    // should never go to forward or backward limit switch
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }

    if (hwLimitSwitchBwd_ != hwLimitSwitchBwdOld_) {
      traj_->setExecute(0);
      seqState_ = 4;
    }
    break;

  // Wait to stop and the home by calculating ref position
  case 4:
    // should never go to forward limit or backward switch
    retValue = checkHWLimitsAndStop(0, 1);
    if (retValue) {
      return retValue;
    }
    traj_->setExecute(0);

    if (!traj_->getBusy()) {  // Wait for stop ramp ready
      data_->command_.positionTarget = traj_->getCurrentPosSet();

      if ((mon_->getAtTarget() && mon_->getEnableAtTargetMon()) ||
          !mon_->getEnableAtTargetMon()) {
        // now stopped just at limit switch
        if (enc_->getScale() < 0) {
          distToAbsBitsUnderOverFlow = std::abs(
            enc_->getRawAbsPosRegister() * enc_->getScale());
        } else {
          distToAbsBitsUnderOverFlow = enc_->getAbsRangeEng() - std::abs(
            enc_->getRawAbsPosRegister() * enc_->getScale());
        }
        currPos = homePosition_ - (homeLatchCountOffset_ - 1) *
                  enc_->getAbsRangeEng() - distToAbsBitsUnderOverFlow;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }

  postHomeMove();

  return -seqState_;
}

int ecmcAxisSequencer::seqHoming22() {  // nCmdData==22 Resolver homing (keep absolute bits)
  // Return > 0 error
  // Return < 0 progress (negation of current seq state returned)
  // Return = 0 ready
  // State 0 set parameters and trigger motion in nHomeDirection, speed =_dHomeVelTwordsCam
  // State 1 Wait for positive edge of bwd limit switch sensor then stop motion. Velocity changed to _dHomeVelOffCam
  // State 2 Wait for stop and trigger motion in negative direction
  // State 3 Wait for leaving limit switch.
  // State 4 Wait for over/underflow of absolute encoder bits.
  // State 5 Wait for standstill before rescale of encoder. Keep encoder absolute bits

  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  if ((enc_->getAbsBits() < ECMC_ENCODER_ABS_BIT_MIN) ||
      (enc_->getAbsBits() > enc_->getBits())) {
    LOGERR(
      "%s/%s:%d: ERROR: Sequence aborted. Encoder absolute bit count out of range (%d). Allowed range: %d:%d. (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      enc_->getAbsBits(),
      enc_->getAbsBits(),
      enc_->getBits(),
      ERROR_SEQ_ERROR_ABS_BIT_OUT_OF_RANGE);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_ERROR_ABS_BIT_OUT_OF_RANGE);
  }

  if (homeLatchCountOffset_ <= 0) {
    LOGERR(
      "%s/%s:%d: ERROR: Sequence aborted. Latch count out of range (%d). Allowed: value>0 (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      homeLatchCountOffset_,
      ERROR_SEQ_LATCH_COUNT_OUT_OF_RANGE);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_LATCH_COUNT_OUT_OF_RANGE);
  }

  double distToAbsBitsUnderOverFlow = 0;
  double currPos                    = 0;

  // Sequence code
  switch (seqState_) {
  // Set parameters and start initial motion
  case 0:
    initHomingSeq();
    if (hwLimitSwitchFwd_) {
      currSeqDirection_ = ECMC_DIR_FORWARD;  // StartDirection
      traj_->setTargetVel(homeVelTwordsCam_);   // high speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);
      seqState_ = 1;
    } else {  // Already at fwd limit jump to step 2
      currSeqDirection_ = ECMC_DIR_BACKWARD;        // StartDirection
      seqState_         = 2;
    }
    break;
  // Wait for positive limit switch and turn other direction
  case 1:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);
    if (retValue) {
      return retValue;
    }
    if (hwLimitSwitchFwdOld_ && !hwLimitSwitchFwd_) {
      traj_->setExecute(0);
      // Switch direction
      currSeqDirection_ = ECMC_DIR_BACKWARD;
      seqState_         = 2;
    }
    break;

  // Wait for standstill and then trigger move
  case 2:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }

    if (!traj_->getBusy()) {
      traj_->setTargetVel(-homeVelOffCam_);  // Low speed
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setExecute(1);        // Trigg new movement
      seqState_ = 3;
    } else {
      traj_->setExecute(0);
    }
    break;

  // Wait for an falling or rising edge of fwd limit switch
  case 3:
    // should never go to forward or backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      return retValue;
    }

    if (hwLimitSwitchBwd_ != hwLimitSwitchBwdOld_) {
      traj_->setExecute(0);
      seqState_ = 4;
    }

  // Wait for over/under-flow of absolute bits of encoder
  case 4:
    // should never go to backward limit switch
    retValue = checkHWLimitsAndStop(1, 0);

    if (retValue) {
      LOGERR(
        "%s/%s:%d: ERROR: Failed to find first flank on home sensor before limit switch (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_NO_HOME_SWITCH_FLANK);
    }
    traj_->setExecute(0);

    if (!traj_->getBusy()) {  // Wait for stop ramp ready
      data_->command_.positionTarget = traj_->getCurrentPosSet();

      if ((mon_->getAtTarget() && mon_->getEnableAtTargetMon()) ||
          !mon_->getEnableAtTargetMon()) {
        // now stopped just at limit switch
        if (enc_->getScale() < 0) {
          distToAbsBitsUnderOverFlow = enc_->getAbsRangeEng() - std::abs(
            enc_->getRawAbsPosRegister() * enc_->getScale());
        } else {
          distToAbsBitsUnderOverFlow = std::abs(
            enc_->getRawAbsPosRegister() * enc_->getScale());
        }
        currPos = homePosition_ + (homeLatchCountOffset_ - 1) *
                  enc_->getAbsRangeEng() + distToAbsBitsUnderOverFlow;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }
  
  postHomeMove();

  return -seqState_;
}

// Issue post move after successful finalized homing
int ecmcAxisSequencer::postHomeMove() {
  
  
  switch (seqState_) {
    
    // Wait one cycle
    case 1000:
      // If already there then do not move
      if(data_->status_.currentPositionSetpoint==homePostMoveTargetPos_ && seqState_) {
        stopSeq();
        return 0;
      }
      traj_->setExecute(0);
      seqState_ = 1001;
      break;
    
    // Trigg motion
    case 1001:
      if (!traj_->getBusy()){
        traj_->setMotionMode(ECMC_MOVE_MODE_POS);
        traj_->setTargetVel(homeVelTwordsCam_);
        traj_->setTargetPos(homePostMoveTargetPos_);
        traj_->setExecute(1);
        seqState_ = 1002;
      }
      break;
    
    // wait for stop and stop sequence
    case 1002:
      if (!traj_->getBusy()){
        if(traj_->getCurrentPosSet()!=homePostMoveTargetPos_) {          
          LOGERR("%s/%s:%d: ERROR: Post home move failed (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_SEQ_HOME_POST_MOVE_FAILED);
          setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_HOME_POST_MOVE_FAILED);        
        }        
        stopSeq();
      }
      
      break;
  }

  return 0;
}

int ecmcAxisSequencer::checkHWLimitsAndStop(bool checkBWD, bool checkFWD) {
  if (traj_ == NULL) {
    stopSeq();
    LOGERR("%s/%s:%d: ERROR: Trajectory object NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_SEQ_TRAJ_NULL);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_TRAJ_NULL);
  }

  if (mon_ == NULL) {
    stopSeq();
    LOGERR("%s/%s:%d: ERROR: Monitor object NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_SEQ_MON_NULL);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_MON_NULL);
  }

  if (!data_->status_.limitFwdFiltered && checkFWD) {
    LOGERR(
      "%s/%s:%d: ERROR: Unexpected activation of forward limit switch in homing state %d of sequence %d (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      seqState_,
      data_->command_.cmdData,
      ERROR_SEQ_SEQ_FAILED);
    stopSeq();
    return ERROR_SEQ_SEQ_FAILED;
  }

  if (!data_->status_.limitBwdFiltered && checkBWD) {
    LOGERR(
      "%s/%s:%d: ERROR: Unexpected activation of backward limit switch in homing state %d of sequence %d (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      seqState_,
      data_->command_.cmdData,
      ERROR_SEQ_SEQ_FAILED);
    stopSeq();
    return ERROR_SEQ_SEQ_FAILED;
  }
  return 0;
}

int ecmcAxisSequencer::getSeqState() {
  return seqState_;
}

int ecmcAxisSequencer::stopSeq() {
  if (traj_ != NULL) {
    traj_->setExecute(false);
  }

  seqInProgress_  = false;
  localSeqBusy_   = false;
  seqState_       = 0;
  seqTimeCounter_ = 0;
  return 0;
}


int ecmcAxisSequencer::validate() {
  return 0;
}

int ecmcAxisSequencer::setSequenceTimeout(int timeout) {
  seqTimeout_ = timeout;
  return 0;
}

int ecmcAxisSequencer::getExtTrajSetpoint(double *pos) {
  *pos = data_->status_.externalTrajectoryPosition;  
  return 0;
}

int ecmcAxisSequencer::setAxisDataRef(ecmcAxisData *data) {
  data_ = data;
  return 0;
}

int ecmcAxisSequencer::checkVelAccDec() {
  if (data_->command_.command == ECMC_CMD_HOMING && data_->command_.cmdData != ECMC_SEQ_HOME_SET_POS) {
    if ((std::abs(homeVelTwordsCam_) == 0) ||
        (std::abs(homeVelOffCam_) == 0)) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_ERROR_VELOCITY_ZERO);
    }
  } else {
    // Sanity check of target velocity
    if (std::abs(data_->command_.velocityTarget) == 0) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_ERROR_VELOCITY_ZERO);
    }
  }

  if (traj_ == NULL) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_TRAJ_NULL);
  }

  // Sanity check of acceleration
  if (traj_->getAcc() <= 0 && data_->command_.cmdData != ECMC_SEQ_HOME_SET_POS) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_ERROR_ACCELERATION_ZERO);
  }

  // Sanity check of deceleration
  if (traj_->getDec() <= 0 && data_->command_.cmdData != ECMC_SEQ_HOME_SET_POS) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_ERROR_DECELERATION_ZERO);
  }

  return 0;
}

void ecmcAxisSequencer::initHomingSeq() {
  enc_->setHomed(false);
  traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
  traj_->setExecute(0);
}

void ecmcAxisSequencer::finalizeHomingSeq(double newPosition) {
  traj_->setCurrentPosSet(newPosition);
  traj_->setTargetPos(newPosition);
  enc_->setActPos(newPosition);
  // Not nice but otherwise one cycle will have wrong values du to exe order.  
  data_->status_.currentPositionActual = newPosition;
  data_->status_.currentPositionSetpoint = newPosition;
  if(drv_) {    
    // drv_->setCspActPos(enc_->getRawPosRegister(), newPosition);
    // drv_->setCspRecalcOffset(newPosition);
    // drv_->setCspPosSet(newPosition);
    drv_->setCspRef(enc_->getRawPosRegister(),newPosition,newPosition);
  }
  enc_->setHomed(true);
  enc_->setArmLatch(false);
  if(cntrl_) {
    cntrl_->reset();
  }
  homePosLatch1_      = 0;
  homePosLatch2_      = 0;
  homeLatchCountAct_  = 0;
  overUnderFlowLatch_ = ECMC_ENC_NORMAL;

  // See if trigg post home motion
  if(homeEnablePostMove_) {
    seqState_ = 1000;
  } 
  else {
    stopSeq();
  }
}

void ecmcAxisSequencer::setHomeLatchCountOffset(int count) {
  homeLatchCountOffset_ = count;
}

int ecmcAxisSequencer::setAllowMotionFunctions(bool enablePos,
                                               bool enableConstVel,
                                               bool enableHome) {
  enablePos_ = enablePos;
  enableConstVel_ = enableConstVel;
  enableHome_ = enableHome;
  return 0;
}

int ecmcAxisSequencer::getAllowPos() {
  return enablePos_;
}

int ecmcAxisSequencer::getAllowConstVelo(){
  return enableConstVel_;
}

int ecmcAxisSequencer::getAllowHome(){
  return enableHome_;
}

void ecmcAxisSequencer::setHomePostMoveTargetPosition(double targetPos) {
  homePostMoveTargetPos_ = targetPos;
}

void ecmcAxisSequencer::setHomePostMoveEnable(double enable) {
  homeEnablePostMove_ = enable;
}
