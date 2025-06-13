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

ecmcAxisSequencer::~ecmcAxisSequencer() {}

void ecmcAxisSequencer::initVars() {
  homeSensorOld_         = false;
  executeOld_            = false;
  seqInProgress_         = false;
  currSeqDirection_      = ECMC_DIR_FORWARD;
  seqState_              = 0;
  traj_                  = NULL;
  mon_                   = NULL;
  cntrl_                 = NULL;
  drv_                   = NULL;
  jogVel_                = 0;
  homeVelTowardsCam_     = 0;
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
  defaultAcc_            = 0;
  defaultDec_            = 0;
  acc_                   = 0;
  dec_                   = 0;
  modeSetEntry_          = NULL;
  modeActEntry_          = NULL;
  modeAct_               = 0;
  modeMotionCmd_         = 0;
  modeHomingCmd_         = 0;
  modeMotionCmdSet_      = 0;
  modeHomingCmdSet_      = 0;
  homeTrigStatOld_       = false;
  monPosLagEnaStatePriorHome_ = false;
  monPosLagRestoreNeeded_ = false;
  pvt_                   = NULL;
  pvtOk_                 = false;
  temporaryLocalTrajSource_ = false;
  temporaryLocalTrajSourceOld_ = false;
  trajLock_              = true;
  trajLockOld_           = true;
  newTrajLockEdge_       = true;
  posSource_             = 0;
  pvtStopping_           = false;
  pvtmode_               = false;
}

void ecmcAxisSequencer::init(double sampleTime) {
}

// Cyclic execution
void ecmcAxisSequencer::execute() {

  pvtmode_ = pvtOk_ && 
      (/*data_->command_.command == ECMC_CMD_MOVEPVTREL ||*/
       data_->command_.command == ECMC_CMD_MOVEPVTABS);

  // Trajectory (External or internal)
  if (data_->command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {

    if(pvtmode_ && !pvtStopping_ && pvt_->getExecute()) {
      data_->status_.currentPositionSetpoint = pvt_->getCurrPosition();
      data_->status_.currentVelocitySetpoint = pvt_->getCurrVelocity();      
    } else {
      data_->status_.currentPositionSetpoint = traj_->getNextPosSet();
      data_->status_.currentVelocitySetpoint = traj_->getNextVel();
    }

  } else {    // External source (PLC or other)
    data_->status_.currentPositionSetpoint =
      data_->status_.externalTrajectoryPosition;
    data_->status_.currentVelocitySetpoint =
      data_->status_.externalTrajectoryVelocity;
    //data_->interlocks_.noExecuteInterlock = false;  // Only valid in local mode
    //data_->refreshInterlocks();
  }

  if (data_->command_.encSource == ECMC_DATA_SOURCE_INTERNAL) {
    data_->status_.currentPositionActual =
      encArray_[data_->command_.primaryEncIndex]->getActPos();
    data_->status_.currentVelocityActual =
      encArray_[data_->command_.primaryEncIndex]->getActVel();
  } else { // External
    data_->status_.currentPositionActual =
      data_->status_.externalEncoderPosition;
    data_->status_.currentVelocityActual =
      data_->status_.externalEncoderVelocity;
  }

  executeInternal();

  mon_->execute();

  // Below code moved from real axis.. Above similar from virt axis, sligth diff
  trajLockOld_ = trajLock_;
  trajLock_ =
    ((data_->interlocks_.trajSummaryInterlockFWD &&
      data_->status_.currentPositionSetpoint >
      data_->status_.currentPositionSetpointOld) ||
     (data_->interlocks_.trajSummaryInterlockBWD &&
      data_->status_.currentPositionSetpoint <
      data_->status_.currentPositionSetpointOld));
  
  // If internal source and not PVT-mode then interlocks are handled in trajectory generator
  // TODO Would be nice to change this design..
  trajLock_ = trajLock_ && (data_->command_.trajSource != ECMC_DATA_SOURCE_INTERNAL || pvtmode_);
  newTrajLockEdge_ = trajLock_ && !trajLockOld_; // && data_->command_.execute && executeOld_;

  // Only init stopramp once if PVT
  if(newTrajLockEdge_) {
    if(pvtmode_) {
      if(!pvtStopping_ && pvt_->getBusy()) {
        initStop();
        //data_->status_.currentPositionSetpoint = traj_->getNextPosSet();
        //data_->status_.currentVelocitySetpoint = traj_->getNextVel();

      }
    } else {
      if(!temporaryLocalTrajSource_) {
        initStop();
        //data_->status_.currentPositionSetpoint = traj_->getNextPosSet();
        //data_->status_.currentVelocitySetpoint = traj_->getNextVel();
      }
    }
  }

  if(temporaryLocalTrajSource_ || trajLock_) {
    data_->status_.currentPositionSetpoint = traj_->getNextPosSet();
    data_->status_.currentVelocitySetpoint = traj_->getNextVel();
    temporaryLocalTrajSource_ = traj_->getBusy();  // Reset
  }

  if(temporaryLocalTrajSourceOld_ != temporaryLocalTrajSource_) {
    printf("ecmcAxisSequencer::execute(): Temporary Local source %d\n",temporaryLocalTrajSource_);
  }

  temporaryLocalTrajSourceOld_ = temporaryLocalTrajSource_;

  pvtStopping_ = traj_->getBusy() && pvtStopping_;

  // PVT
  if(pvtOk_  && !pvtStopping_ && data_->command_.execute) {
    // Go to next pvt time step
    if(pvt_->nextSampleStep() && pvt_->getExecute()) {
      traj_->setCurrentPosSet(data_->status_.currentPositionSetpoint);
    }
  }
  traj_->setStartPos(data_->status_.currentPositionSetpoint);
}

void ecmcAxisSequencer::executeInternal() {
  // read mode if entry is linked
  uint64_t tempRaw = 0;

  if (modeActEntry_) {
    modeActEntry_->readValue(&tempRaw);
    modeAct_ = (int)tempRaw;
  }

  data_->status_.seqState = seqState_;

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
    // HOMING 
    if (data_->command_.command == ECMC_CMD_HOMING) {
      data_->status_.busy = localSeqBusy_ || traj_->getBusy();
    } else {
     // PVT 
     if(pvtmode_ && !pvtStopping_) {
        data_->status_.busy = (pvt_->getBusy() && data_->status_.enabled) ||
                              !data_->status_.startupFinsished;
      } else {
      // Normal motion
        data_->status_.busy = (traj_->getBusy() && data_->status_.enabled) ||
                              !data_->status_.startupFinsished;
      }
    }
  } else { // Sync to other axis
    data_->status_.busy = true;
  }

  // Set target position for different scenarios
  if(pvtmode_ && !data_->status_.busy) {
    data_->status_.currentTargetPosition = data_->status_.currentPositionSetpoint;
    data_->status_.currentTargetPositionModulo = data_->status_.currentPositionSetpoint;
  } else {
    if (data_->command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
      data_->status_.currentTargetPosition       = traj_->getTargetPos();
      data_->status_.currentTargetPositionModulo = traj_->getTargetPosMod();
    } else {  // Synchronized to other axis
      data_->status_.currentTargetPosition =
        data_->status_.currentPositionSetpoint;
      data_->status_.currentTargetPositionModulo =
        data_->status_.currentPositionSetpoint;
    }
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

  seqStateOld_ = seqState_;
  seqTimeCounter_++;

  if ((seqTimeCounter_ > seqTimeout_) && (seqTimeout_ > 0)) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_TIMEOUT);
    stopSeq();
    return;
  }
  int seqReturnVal         = 0;
  ecmcHomingType homeSeqId = (ecmcHomingType)data_->command_.cmdData;

  switch (data_->command_.command) {
  case ECMC_CMD_JOG:
    ;
    break;

  case ECMC_CMD_HOMING:
        
    switch (homeSeqId) {
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

    case ECMC_SEQ_HOME_USE_ENC_CFGS:
      // This number is only used to get the cmd data from encoder object
      // that means that if this is executing, something is very wrong...
      // In other words, the encoder object needs to have a valid homeproc.
      setErrorID(__FILE__, __FUNCTION__, __LINE__,
                 ERROR_SEQ_CMD_DATA_UNDEFINED);
      stopSeq();
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

    case ECMC_SEQ_HOME_TRIGG_EXTERN:
      seqReturnVal = seqHoming26();

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
  int errorCode = 0;
  int modeSet   = 0;

  if (traj_ == NULL) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_SEQ_TRAJ_NULL);
  }

  executeOld_             = data_->command_.execute;
  data_->command_.execute = execute;
  seqInProgress_          = false;
  seqState_               = 0;

  setTrajAccAndDec();
  if (data_->command_.execute  && !executeOld_) {

    // velo for homing is set in a different way
    if (data_->command_.command != ECMC_CMD_HOMING) {
      errorCode = checkVelAccDec();

      if (errorCode) {
        return errorCode;
      }
    }
  }

  // default mode motion
  if (modeMotionCmdSet_) {
    modeSet = modeMotionCmd_;
  }

  switch (data_->command_.command) {
  case ECMC_CMD_JOG:

    // Triggered via jog inputs
    break;

  case ECMC_CMD_MOVEVEL:

    if (data_->command_.execute  && !executeOld_) {
      if (!enableConstVel_) {
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_SEQ_MOTION_CMD_NOT_ENABLED);
      }

      // Only allow cmdData 0 (no different modes implemented)
      if (data_->command_.cmdData != 0) {
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_SEQ_CMD_DATA_UNDEFINED);
      }

      data_->status_.busy = true;
      traj_->setStartPos(data_->status_.currentPositionSetpoint);
      traj_->setCurrentPosSet(data_->status_.currentPositionSetpoint);
      traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
      traj_->setTargetVel(data_->command_.velocityTarget);
      if(data_->command_.controlWord_.enableDbgPrintout) {
        printf("ECMC_CMD_MOVEVEL targ velo: %lf\n",data_->command_.velocityTarget);
      }
    }

    errorCode =  traj_->setExecute(data_->command_.execute);

    if (errorCode) {
      return errorCode;
    }

    break;

  case ECMC_CMD_MOVEREL:

    if (data_->command_.execute && !executeOld_) {
      if (!enablePos_) {
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_SEQ_MOTION_CMD_NOT_ENABLED);
      }

      // Only allow cmdData 0 (no different modes implemented)
      if (data_->command_.cmdData != 0) {
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_SEQ_CMD_DATA_UNDEFINED);
      }

      data_->status_.busy = true;
      traj_->setMotionMode(ECMC_MOVE_MODE_POS);
      traj_->setStartPos(data_->status_.currentPositionSetpoint);
      traj_->setCurrentPosSet(data_->status_.currentPositionSetpoint);
      traj_->setTargetVel(data_->command_.velocityTarget);
      traj_->setTargetPos(data_->command_.positionTarget);
      if(data_->command_.controlWord_.enableDbgPrintout) {
        printf("ECMC_CMD_MOVEREL pos targ: %lf, targ velo: %lf, traj busy %d\n",data_->command_.positionTarget,data_->command_.velocityTarget, traj_->getBusy());
      }
    }
    errorCode = traj_->setExecute(data_->command_.execute);

    if (errorCode) {
      return errorCode;
    }

    break;

  case ECMC_CMD_MOVEABS:
    
    if (data_->command_.execute && !executeOld_) {
      if (!enablePos_) {
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_SEQ_MOTION_CMD_NOT_ENABLED);
      }
      
      data_->status_.busy = true;
      traj_->setStartPos(data_->status_.currentPositionSetpoint);
      traj_->setCurrentPosSet(data_->status_.currentPositionSetpoint);
      traj_->setMotionMode(ECMC_MOVE_MODE_POS);
      traj_->setTargetVel(data_->command_.velocityTarget);
      if(data_->command_.controlWord_.enableDbgPrintout) {
        printf("ECMC_CMD_MOVEABS pos targ: %lf, targ velo: %lf, traj busy %d\n",data_->command_.positionTarget,data_->command_.velocityTarget, traj_->getBusy());
      }
      double targPos   = 0;
      int    errorCode = 0;

      switch (data_->command_.cmdData) {
      case 0:     // Normal positioning
        traj_->setTargetPos(data_->command_.positionTarget);
        break;

      // Go to external transform curr value (as targetPosition)
      case 1:

        errorCode = getExtTrajSetpoint(&targPos);

        if (errorCode) {
          return errorCode;
        }
        traj_->setTargetPos(targPos);
        break;

      default:
        // Not valid cmddata
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_SEQ_CMD_DATA_UNDEFINED);
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

  // PVT is only abs here (relative is handled by the PVt controller)
  case ECMC_CMD_MOVEPVTABS:
    if(data_->command_.controlWord_.enableDbgPrintout) {
      printf("RUNNING PVT ABS execute %d\n",data_->command_.execute);
    }
    if (data_->command_.execute && !executeOld_) {
      errorCode = validatePVT();
      if(errorCode) {
        return errorCode;
      }
      
      // Set offset 0 since pvt is absolute
      //pvt_->setPositionOffset(0);
      pvt_->setExecute(0);
      errorCode = pvt_->setExecute(1);
      if (errorCode) {
        return errorCode;
      }
      data_->interlocks_.noExecuteInterlock = false;
    } else if(!data_->command_.execute){
      errorCode = pvt_->setExecute(0);
      // needed since this is evaluated in trajectoy which is not in use
      data_->interlocks_.noExecuteInterlock = true;            
    }
    data_->refreshInterlocks();
    break;

  case ECMC_CMD_HOMING:
    // set mode to homing
    if (modeHomingCmdSet_) {
      modeSet = modeHomingCmd_;
    }

    if (data_->command_.execute && !executeOld_) {
      // oldPrimaryEnc_ = data_->command_.primaryEncIndex;
      // encoder data source must be internal for homing
      traj_->setStartPos(data_->status_.currentPositionSetpoint);
      if (data_->command_.encSource != ECMC_DATA_SOURCE_INTERNAL) {
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_SEQ_HOME_ENC_SOURCE_NOT_INTERNAL);
      }

      stopSeq();
      latchPosLagMonStateBeforeSeq();

      if (!enableHome_) {
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_SEQ_MOTION_CMD_NOT_ENABLED);
      }

      if ((traj_ != NULL) &&
          (getPrimEnc() != NULL) &&
          (mon_ != NULL) &&
          ((cntrl_ != NULL) || (data_->axisType_ == ECMC_AXIS_TYPE_VIRTUAL))) {
        seqInProgress_      = true;
        localSeqBusy_       = true;
        data_->status_.busy = true;

        // Use the parameters defined in encoder object
        if (data_->command_.cmdData == ECMC_SEQ_HOME_USE_ENC_CFGS) {
          readHomingParamsFromEnc();
        }

        if (data_->command_.cmdData == ECMC_SEQ_HOME_NOT_VALID) {
          // Homing not allowed
          return setErrorID(__FILE__,
                            __FUNCTION__,
                            __LINE__,
                            ERROR_SEQ_HOME_NOT_ALLOWED);
        }
      } else {
        if (traj_ == NULL) {
          return setErrorID(__FILE__,
                            __FUNCTION__,
                            __LINE__,
                            ERROR_SEQ_TRAJ_NULL);
        } else {
          traj_->setExecute(false);
        }

        if (getPrimEnc() == NULL) {
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

        if ((cntrl_ == NULL) && (data_->axisType_ != ECMC_AXIS_TYPE_VIRTUAL)) {
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

  if (data_->command_.execute  && !executeOld_) {
    // write mode if entry is linked
    if (modeSetEntry_) {
      modeSetEntry_->writeValue((uint64_t)modeSet);
    }
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

void ecmcAxisSequencer::setTraj(ecmcTrajectoryBase *traj) {
  traj_ = traj;
}

void ecmcAxisSequencer::setEnc(ecmcEncoder **encArray) {
  encArray_ = encArray;
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

void ecmcAxisSequencer::setTargetPos(double pos) {
  if (data_->command_.command == ECMC_CMD_MOVEREL) {
    pos = traj_->getCurrentPosSet() + pos;
  }
  pos                            = checkSoftLimits(pos);
  data_->command_.positionTarget = pos;

  // "On the fly change"
  if (getBusy()) {
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
  if (mon_->getEnableMaxVelMon()) {
    double maxVelo = std::abs(mon_->getMaxVel());

    if (velTarget >= 0) { // positive velo
      if (velTarget > maxVelo) {
        velTarget = maxVelo;
      }
    } else {  // negative velo
      if (velTarget < -maxVelo) {
        velTarget = -maxVelo;
      }
    }
  }

  data_->command_.velocityTarget = velTarget;

  // Do not write to traj if homing
  if (data_->command_.command != ECMC_CMD_HOMING) {
    traj_->setTargetVel(velTarget);
  }
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

  double dSet = posSetpoint;
  double dAct = data_->status_.currentPositionSetpoint;

  // soft limit FWD
  if ((posSetpoint > data_->command_.softLimitFwd) &&
      data_->command_.controlWord_.enableSoftLimitFwd && (dSet > dAct)) {
    dSet = dAct;
    setWarningID(WARNING_SEQ_SETPOINT_SOFTLIM_FWD_VILOATION);
  }

  // soft limit BWD
  if ((posSetpoint < data_->command_.softLimitBwd) &&
      data_->command_.controlWord_.enableSoftLimitBwd && (dSet < dAct)) {
    dSet = dAct;
    setWarningID(WARNING_SEQ_SETPOINT_SOFTLIM_BWD_VILOATION);
  }

  return dSet;
}

ecmcTrajectoryBase * ecmcAxisSequencer::getTraj() {
  return traj_;
}

int ecmcAxisSequencer::seqHoming15() {  // nCmdData==15
  // Return = 0 ready
  // State 0 set encoder position to same as fHomePosition
  // Sequence code

  switch (seqState_) {
  case 0:      // Set parameters and start initial motion
    traj_->setCurrentPosSet(homePosition_);
    traj_->setTargetPos(homePosition_);
    getPrimEnc()->setActPos(homePosition_);
    getPrimEnc()->setHomed(true);
    data_->status_.currentPositionActual   = homePosition_;
    data_->status_.currentPositionSetpoint = homePosition_;

    if (cntrl_) {
      cntrl_->reset();
    }

    // trigg post home motion if enabled and not alreday triggered
    if (homeEnablePostMove_) {
      if (seqState_ < 1000) {
        seqState_ = 1000;
      }
    } else {
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

  // State 0 set parameters and trigger motion in nHomeDirection, speed = HomeVelTowardsCam
  // State 1 Wait for negative edge of bwd limit switch sensor then stop motion. Velocity changed to HomeVelOffCam
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
      traj_->setTargetVel(-homeVelTowardsCam_);   // high speed
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
      homePosLatch1_ = getPrimEnc()->getActPos();
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
        double currPos =
          getPrimEnc()->getActPos() -
          homePosLatch1_ + homePosition_;
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

  // State 0 set parameters and trigger motion in nHomeDirection, speed =HomeVelTowardsCam
  // State 1 Wait for positive edge of bwd limit switch sensor then stop motion. Velocity changed to HomeVelOffCam
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
      traj_->setTargetVel(homeVelTowardsCam_);  // High speed
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
      homePosLatch1_ = getPrimEnc()->getActPos();
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
        double currPos =
          getPrimEnc()->getActPos() -
          homePosLatch1_ + homePosition_;
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

  // State 0 set parameters and trigger motion in nHomeDirection, speed =HomeVelTowardsCam
  // State 1 Wait for negative edge of bwd limit switch sensor then stop motion. Velocity changed to HomeVelOffCam
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
      traj_->setTargetVel(-homeVelTowardsCam_);   // high speed
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
      homePosLatch1_ = getPrimEnc()->getActPos();
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
        double currPos =
          getPrimEnc()->getActPos() -
          homePosLatch1_ + homePosition_;
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

  // State 0 set parameters and trigger motion in nHomeDirection, speed =HomeVelTowardsCam
  // State 1 Wait for positive edge of bwd limit switch sensor then stop motion. Velocity changed to HomeVelOffCam
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
      traj_->setTargetVel(homeVelTowardsCam_);   // high speed
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
      homePosLatch1_ = getPrimEnc()->getActPos();
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
        double currPos =
          getPrimEnc()->getActPos() -
          homePosLatch1_ + homePosition_;
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

  // State 0 set parameters and trigger motion in nHomeDirection, speed =HomeVelTowardsCam
  // State 1 Wait for negative edge of bwd limit switch sensor then stop motion. Velocity changed to HomeVelOffCam
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
      traj_->setTargetVel(-homeVelTowardsCam_);  // High speed
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
      homePosLatch1_ = getPrimEnc()->getActPos();
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
      homePosLatch2_ = getPrimEnc()->getActPos();
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
        double currPos =
          getPrimEnc()->getActPos() -
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

  // State 0 set parameters and trigger motion in nHomeDirection, speed =HomeVelTowardsCam
  // State 1 Wait for negative edge of fwd limit switch sensor then stop motion. Velocity changed to HomeVelOffCam
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
      traj_->setTargetVel(homeVelTowardsCam_);  // High speed
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
      homePosLatch1_ = getPrimEnc()->getActPos();
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
      homePosLatch2_ = getPrimEnc()->getActPos();
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
        double currPos =
          getPrimEnc()->getActPos() -
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
    retValue = checkHWLimitsAndStop(1, 0);

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
      homePosLatch1_ = getPrimEnc()->getActPos();
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
        double currPos =
          getPrimEnc()->getActPos() -
          homePosLatch1_ + homePosition_;
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
    retValue = checkHWLimitsAndStop(0, 1);

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
    retValue = checkHWLimitsAndStop(0, 1);

    if (retValue) {
      return retValue;
    }

    if (homeSensor_  && !homeSensorOld_) {
      homePosLatch1_ = getPrimEnc()->getActPos();
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
        double currPos =
          getPrimEnc()->getActPos() -
          homePosLatch1_ + homePosition_;
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

  // State 0 set parameters and trigger motion in backward, speed =HomeVelTowardsCam
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
    retValue = checkHWLimitsAndStop(1, 0);

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
      homePosLatch1_ = getPrimEnc()->getActPos();
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
      homePosLatch2_ = getPrimEnc()->getActPos();
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
        double currPos =
          getPrimEnc()->getActPos() -
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

  // State 0 set parameters and trigger motion in forward, speed =HomeVelTowardsCam
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
    retValue = checkHWLimitsAndStop(0, 1);

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

    if (homeSensor_ && !homeSensorOld_) {
      homePosLatch1_ = getPrimEnc()->getActPos();
      seqState_      = 2;
    }
    break;

  // Wait for falling edge of home sensor then stop
  case 2:
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

    if (!homeSensor_ && homeSensorOld_) {
      traj_->setExecute(0);
      seqState_ = 3;
    }
    break;

  // Wait for standstill and the trigger move
  case 3:
    // should never go to forward limit switch
    retValue = checkHWLimitsAndStop(0, 1);

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
      homePosLatch2_ = getPrimEnc()->getActPos();
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
        double currPos =
          getPrimEnc()->getActPos() -
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

  // State 0 set parameters and trigger motion in nHomeDirection, speed =HomeVelTowardsCam
  // State 1 Wait for negative edge of bwd limit switch sensor then stop motion. Velocity changed to HomeVelOffCam
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
      traj_->setTargetVel(-homeVelTowardsCam_);   // high speed
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
      getPrimEnc()->setArmLatch(false);   // ensure latch is not armed
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
      getPrimEnc()->setArmLatch(true);   // ensure latch is armed
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

    if (getPrimEnc()->getNewValueLatched()) {
      homeLatchCountAct_++;

      if (homeLatchCountAct_ >= homeLatchCountOffset_) {
        homePosLatch1_ =
          getPrimEnc()->getLatchPosEng();
        seqState_ = 5;
      }
      getPrimEnc()->setArmLatch(false);
    } else {
      getPrimEnc()->setArmLatch(true);
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
        double currPos =
          getPrimEnc()->getActPos() -
          homePosLatch1_ + homePosition_;
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

  // State 0 set parameters and trigger motion in nHomeDirection, speed =HomeVelTowardsCam
  // State 1 Wait for positive edge of bwd limit switch sensor then stop motion. Velocity changed to HomeVelOffCam
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
      traj_->setTargetVel(homeVelTowardsCam_);   // high speed
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
      getPrimEnc()->setArmLatch(false);   // ensure latch is not armed
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
      getPrimEnc()->setArmLatch(true);  // ensure latch is armed
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

    if (getPrimEnc()->getNewValueLatched()) {
      homeLatchCountAct_++;

      if (homeLatchCountAct_ >= homeLatchCountOffset_) {
        homePosLatch1_ =
          getPrimEnc()->getLatchPosEng();
        seqState_ = 5;
      }
      getPrimEnc()->setArmLatch(false);
    } else {
      getPrimEnc()->setArmLatch(true);
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
        double currPos =
          getPrimEnc()->getActPos() -
          homePosLatch1_ + homePosition_;
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

  // State 0 set parameters and trigger motion in nHomeDirection, speed =HomeVelTowardsCam
  // State 1 Wait for negative edge of bwd limit switch sensor then stop motion. Velocity changed to HomeVelOffCam
  // State 2 Wait for stop and trigger motion in positive direction
  // State 3 Wait for leaving limit switch.
  // State 4 Wait for over/underflow of absolute encoder bits.
  // State 5 Wait for standstill before rescale of encoder. Keep encoder absolute bits

  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  if ((getPrimEnc()->getAbsBits() <
       ECMC_ENCODER_ABS_BIT_MIN) ||
      (getPrimEnc()->getAbsBits() >
       getPrimEnc()->getBits())) {
    LOGERR(
      "%s/%s:%d: ERROR: Sequence aborted. Encoder absolute bit count out of range (%d). Allowed range: %d:%d. (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      getPrimEnc()->getAbsBits(),
      getPrimEnc()->getAbsBits(),
      getPrimEnc()->getBits(),
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
      traj_->setTargetVel(-homeVelTowardsCam_);   // high speed
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
        if (getPrimEnc()->getScale() < 0) {
          distToAbsBitsUnderOverFlow = std::abs(
            getPrimEnc()->getRawAbsPosRegister() *
            getPrimEnc()->getScale());
        } else {
          distToAbsBitsUnderOverFlow =
            getPrimEnc()->getAbsRangeEng() -
            std::abs(
              getPrimEnc()->getRawAbsPosRegister() *
              getPrimEnc()->getScale());
        }
        currPos = homePosition_ - (homeLatchCountOffset_ - 1) *
                  getPrimEnc()->getAbsRangeEng()
                  - distToAbsBitsUnderOverFlow;
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
  // State 0 set parameters and trigger motion in nHomeDirection, speed =HomeVelTowardsCam
  // State 1 Wait for positive edge of bwd limit switch sensor then stop motion. Velocity changed to HomeVelOffCam
  // State 2 Wait for stop and trigger motion in negative direction
  // State 3 Wait for leaving limit switch.
  // State 4 Wait for over/underflow of absolute encoder bits.
  // State 5 Wait for standstill before rescale of encoder. Keep encoder absolute bits

  int retValue = traj_->getErrorID();  // Abort if error from trajectory

  if (retValue) {
    return retValue;
  }

  if ((getPrimEnc()->getAbsBits() <
       ECMC_ENCODER_ABS_BIT_MIN) ||
      (getPrimEnc()->getAbsBits() >
       getPrimEnc()->getBits())) {
    LOGERR(
      "%s/%s:%d: ERROR: Sequence aborted. Encoder absolute bit count out of range (%d). Allowed range: %d:%d. (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      getPrimEnc()->getAbsBits(),
      getPrimEnc()->getAbsBits(),
      getPrimEnc()->getBits(),
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
      traj_->setTargetVel(homeVelTowardsCam_);   // high speed
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
        if (getPrimEnc()->getScale() < 0) {
          distToAbsBitsUnderOverFlow =
            getPrimEnc()->getAbsRangeEng() -
            std::abs(
              getPrimEnc()->getRawAbsPosRegister() *
              getPrimEnc()->getScale());
        } else {
          distToAbsBitsUnderOverFlow = std::abs(
            getPrimEnc()->getRawAbsPosRegister() *
            getPrimEnc()->getScale());
        }
        currPos = homePosition_ + (homeLatchCountOffset_ - 1) *
                  getPrimEnc()->getAbsRangeEng()
                  + distToAbsBitsUnderOverFlow;
        finalizeHomingSeq(currPos);
      }
    }
    break;
  }

  postHomeMove();

  return -seqState_;
}

// External triggered homig (set trigg bit and then wait for status bit to be 1)
int ecmcAxisSequencer::seqHoming26() {
  // Sequence code
  switch (seqState_) {
  case 0:
    // wait for drive mode to be set

    if (!autoModeSetHoming()) {
      return -2000;   // negative state that is not used
    }

    if (!getPrimEnc()->getHomeExtTriggEnabled()) {
      LOGERR(
        "%s/%s:%d: ERROR: Homing sequence not supported by encoder (hw links not set) (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_HOME_SEQ_NOT_SUPPORTED);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_SEQ_HOME_SEQ_NOT_SUPPORTED);
    }
        
    // Trigger motion (just set output bit)
    initHomingSeq();

    // Must disable following error mon since traj generation is external (will be auto restored by the restorePosLagMonAfterSeq())
    mon_->setEnableLagMon(false); 

    getPrimEnc()->setHomeExtTrigg(1);
    seqState_ = 1;
    break;

  case 1:  // Get status of homig seq (read bit)
    if (!homeTrigStatOld_ && getPrimEnc()->getHomeExtTriggStat()) {  // consider check for edge..
      getPrimEnc()->setHomeExtTrigg(0);
      finalizeHomingSeq(homePosition_);
    }
    break;
  }

  homeTrigStatOld_ = getPrimEnc()->getHomeExtTriggStat();
  postHomeMove();

  return -seqState_;
}

// Issue post move after successful finalized homing (seqState_ set to 1000 in finalizeHomingSeq )
int ecmcAxisSequencer::postHomeMove() {
  switch (seqState_) {
  // Wait one cycle
  case 1000:

    // wait for drive mode to be set
    if (autoModeSetMotion()) {
      seqState_ = 1001;
    }
    break;

  case 1001:

    // If already there then do not move
    if ((data_->status_.currentPositionSetpoint == homePostMoveTargetPos_) &&
        seqState_) {
      stopSeq();
      return 0;
    }
    traj_->setExecute(0);
    seqState_ = 1002;
    break;

  // Trigg motion
  case 1002:

    if (!traj_->getBusy()) {
      traj_->setMotionMode(ECMC_MOVE_MODE_POS);
      traj_->setTargetVel(homeVelTowardsCam_);
      traj_->setTargetPos(homePostMoveTargetPos_);
      traj_->setExecute(1);
      seqState_ = 1003;
    }
    break;

  // wait for stop and stop sequence
  case 1003:

    if (!traj_->getBusy()) {
      if (traj_->getCurrentPosSet() != homePostMoveTargetPos_) {
        LOGERR("%s/%s:%d: ERROR: Post home move failed (0x%x).\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_SEQ_HOME_POST_MOVE_FAILED);
        setErrorID(__FILE__,
                   __FUNCTION__,
                   __LINE__,
                   ERROR_SEQ_HOME_POST_MOVE_FAILED);
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

  if (getPrimEnc() != NULL) {
    getPrimEnc()->setHomeExtTrigg(0);
  }

  seqInProgress_  = false;
  localSeqBusy_   = false;
  seqState_       = 0;
  seqTimeCounter_ = 0;
  restorePosLagMonAfterSeq();
  return 0;
}

int ecmcAxisSequencer::validate() {
  return 0;
}

int ecmcAxisSequencer::setSequenceTimeout(int timeout) {
  if (data_->sampleTime_ > 0) {
    // Seconds
    seqTimeout_ = timeout / data_->sampleTime_;
  } else {
    seqTimeout_ = timeout;
  }
  return 0;
}

int ecmcAxisSequencer::getExtTrajSetpoint(double *pos) {
  *pos = data_->status_.externalTrajectoryPosition;
  return 0;
}

int ecmcAxisSequencer::setAxisDataRef(ecmcAxisData *data) {
  data_ = data;

  // Set external error code ints (to be collected in axis base class)
  setExternalPtrs(&(data_->status_.errorCode), &(data_->status_.warningCode));  
  return 0;
}

int ecmcAxisSequencer::checkVelAccDec() {
  if ((data_->command_.command == ECMC_CMD_HOMING) &&
      (data_->command_.cmdData != ECMC_SEQ_HOME_SET_POS)) {
    if ((std::abs(homeVelTowardsCam_) == 0) ||
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
  if ((traj_->getAcc() <= 0) &&
      (data_->command_.cmdData != ECMC_SEQ_HOME_SET_POS)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_ERROR_ACCELERATION_ZERO);
  }

  // Sanity check of deceleration
  if ((traj_->getDec() <= 0) &&
      (data_->command_.cmdData != ECMC_SEQ_HOME_SET_POS)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_SEQ_ERROR_DECELERATION_ZERO);
  }

  return 0;
}

void ecmcAxisSequencer::initHomingSeq() {
  getPrimEnc()->setHomed(false);
  traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
  traj_->setExecute(0);
}

void ecmcAxisSequencer::finalizeHomingSeq(double newPosition) {
  
  // Prep all objects for setpoint step (except encoders)
  setNewPositionCtrlDrvTrajBumpless(newPosition);

  // home all encoders to the new position
  for (int i = 0; i < data_->status_.encoderCount; i++) {
    // Ref all encoders that are configured to be homed. Always ref primary encoder.
    if (encArray_[i]->getRefAtHoming() || i == data_->command_.primaryEncIndex ) {
      if(data_->command_.controlWord_.enableDbgPrintout) {
        printf("INFO: Axis [%d]: Setting new position to encoder[%d]: %lf\n",data_->axisId_,i,newPosition);
      }
      encArray_[i]->setActPos(newPosition);
      encArray_[i]->setHomed(true);
      encArray_[i]->setArmLatch(false);
    }
  }

  homePosLatch1_      = 0;
  homePosLatch2_      = 0;
  homeLatchCountAct_  = 0;
  overUnderFlowLatch_ = ECMC_ENC_NORMAL;
   
  // See if trigg post home motion
  if (homeEnablePostMove_) {
    seqState_ = 1000;
  } else {
    stopSeq();
  }
}

void ecmcAxisSequencer::setHomeLatchCountOffset(int count) {
  homeLatchCountOffset_ = count;
}

int ecmcAxisSequencer::setAllowMotionFunctions(bool enablePos,
                                               bool enableConstVel,
                                               bool enableHome) {
  enablePos_      = enablePos;
  enableConstVel_ = enableConstVel;
  enableHome_     = enableHome;
  return 0;
}

int ecmcAxisSequencer::getAllowPos() {
  return enablePos_;
}

int ecmcAxisSequencer::getAllowConstVelo() {
  return enableConstVel_;
}

int ecmcAxisSequencer::getAllowHome() {
  return enableHome_;
}

void ecmcAxisSequencer::setHomePostMoveTargetPosition(double targetPos) {
  homePostMoveTargetPos_ = targetPos;
}

void ecmcAxisSequencer::setHomePostMoveEnable(double enable) {
  homeEnablePostMove_ = enable;
}

void ecmcAxisSequencer::setNewPositionCtrlDrvTrajBumpless(double newPosition) {
  traj_->setCurrentPosSet(newPosition);
  traj_->setTargetPos(newPosition);

  // Not nice but otherwise one cycle will have wrong values du to exe order.
  data_->status_.currentPositionActual   = newPosition;
  data_->status_.currentPositionSetpoint = newPosition;

  if (drv_) {
    drv_->setCspRef(
      getCSPEnc()->getRawPosRegister(),
      newPosition,
      newPosition);
  }

  if (cntrl_) {
    cntrl_->reset();
  }
}

void ecmcAxisSequencer::readHomingParamsFromEnc() {
  // This param is only accessibele in encoder object, so always read
  setHomeLatchCountOffset(
    getPrimEnc()->getHomeLatchCountOffset());

  // Check if encoder has parameters then overwrite existing parameters if any
  if (!getPrimEnc()->getHomeParamsValid()) {
    LOGERR(
      "%s/%s:%d: WARNING: No valid homing info stored for encoder, falling back to axis params.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return;
  }

  // Overwrite homing seqence id with what is stored in encoder object
  setCmdData((ecmcHomingType)getPrimEnc()->getHomeSeqId());

  // Encoder has parameters stored so read those..
  homeVelTowardsCam_ = getPrimEnc()->getHomeVelTowardsCam();

  if (std::abs(homeVelTowardsCam_) == 0) {
    homeVelTowardsCam_ = data_->command_.velocityTarget;
  }
  homeVelOffCam_ = getPrimEnc()->getHomeVelOffCam();

  if (std::abs(homeVelOffCam_) == 0) {
    homeVelOffCam_ = data_->command_.velocityTarget;
  }

  homePosition_          = getPrimEnc()->getHomePosition();
  homeEnablePostMove_    = getPrimEnc()->getHomePostMoveEnable();
  homePostMoveTargetPos_ = getPrimEnc()->getHomePostMoveTargetPosition();

  double temp = getPrimEnc()->getHomeAcc();

  if (temp > 0) {
    traj_->setAcc(temp);
  }

  temp = getPrimEnc()->getHomeDec();

  if (temp > 0) {
    traj_->setDec(temp);
  }
}

int ecmcAxisSequencer::setHomeVelTowardsCam(double vel) {
  homeVelTowardsCam_ = vel;
  return 0;
}

int ecmcAxisSequencer::setHomeVelOffCam(double vel) {
  homeVelOffCam_ = vel;
  return 0;
}

double ecmcAxisSequencer::getHomeVelTowardsCam() {
  return homeVelTowardsCam_;
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

ecmcEncoder * ecmcAxisSequencer::getPrimEnc() {
  return encArray_[data_->command_.primaryEncIndex];
}

ecmcEncoder * ecmcAxisSequencer::getCSPEnc() {
  if(data_->command_.cspDrvEncIndex < 0) {
    return encArray_[data_->command_.primaryEncIndex];
  } 
  return encArray_[data_->command_.cspDrvEncIndex];
}

void ecmcAxisSequencer::setDefaultAcc(double acc) {
  defaultAcc_ = acc;

  if (defaultDec_ == 0) {
    defaultDec_ = defaultAcc_;
  }
}

void ecmcAxisSequencer::setDefaultDec(double dec) {
  defaultDec_ = dec;

  if (defaultAcc_ == 0) {
    defaultAcc_ = defaultDec_;
  }
}

void ecmcAxisSequencer::setAcc(double acc) {
  acc_ = acc;

  if (data_->command_.command != ECMC_CMD_HOMING) {
    getTraj()->setAcc(acc_);
  }
}

void ecmcAxisSequencer::setDec(double dec) {
  dec_ = dec;

  if (data_->command_.command != ECMC_CMD_HOMING) {
    getTraj()->setDec(dec_);
  }
}

void ecmcAxisSequencer::setTrajAccAndDec() {
  // Revert to defaullt acc and dec if needed
  if (acc_ > 0) {
    getTraj()->setAcc(acc_);
  } else if (defaultAcc_ > 0) {
    getTraj()->setAcc(defaultAcc_);
  }

  if (dec_ > 0) {
    getTraj()->setDec(dec_);
  } else if (defaultDec_ > 0) {
    getTraj()->setDec(defaultDec_);
  }
}

int ecmcAxisSequencer::setAutoModeSetEntry(ecmcEcEntry *entry) {
  modeSetEntry_ = entry;
  return 0;
}

int ecmcAxisSequencer::setAutoModeActEntry(ecmcEcEntry *entry) {
  modeActEntry_ = entry;
  return 0;
}

int ecmcAxisSequencer::setAutoModeHomigCmd(int homing) {
  modeHomingCmdSet_ = 1;
  modeHomingCmd_    = homing;
  return 0;
}

int ecmcAxisSequencer::setAutoModeMotionCmd(int motion) {
  modeMotionCmdSet_ = 1;
  modeMotionCmd_    = motion;
  return 0;
}

// Set motion mode
bool ecmcAxisSequencer::autoModeSetMotion() {
  if (modeSetEntry_ && modeMotionCmdSet_) {
    // write mode if entry is linked
    modeSetEntry_->writeValue((uint64_t)modeMotionCmd_);
  }

  // wait for drive mode to be set
  if ((modeMotionCmd_ == modeAct_) || !modeActEntry_) {
    return 1;
  }
  return 0;  // use seq step that is not used by post move and other seqs
}

// Set homing mode
bool ecmcAxisSequencer::autoModeSetHoming() {
  if (modeSetEntry_ && modeHomingCmdSet_) {
    // write mode if entry is linked
    modeSetEntry_->writeValue((uint64_t)modeHomingCmd_);
  }

  // wait for drive mode to be set
  if ((modeHomingCmd_ == modeAct_) || !modeActEntry_) {
    return 1;
  }
  return 0;  // Use seq step that is not used by post move and other seqs
}

// PVT object created somewhere else can be assigned here (normally from motorRecordAxis::buildProfile())
int ecmcAxisSequencer::setPVTObject(ecmcAxisPVTSequence* pvt) {
  if(pvt_) {
    if(pvt_->getBusy()) {
      LOGERR(
        "%s/%s:%d: ERROR: Current PVT object busy (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_SEQ_PVT_OBJECT_BUSY );
      return ERROR_SEQ_PVT_OBJECT_BUSY; 
    }
  }
  pvtOk_ = false; // need new validation since new object
  pvt_   = pvt;
  // Allow pvt object to get data 
  pvt_->setAxisDataRef(data_);

  if(data_->command_.controlWord_.enableDbgPrintout) {
    printf("ecmcAxisSequencer::setPVTObject(pvt): INFO: PVT object assigned\n");
  }
  return 0;
}

int ecmcAxisSequencer::validatePVT() {
  pvtOk_ = false;
  if(!pvt_) {
    LOGERR(
      "%s/%s:%d: ERROR: PVT object not assigned (PVT == NULL) (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_SEQ_PVT_OBJECT_NULL);
    return ERROR_SEQ_PVT_OBJECT_NULL;
  }

  int errorCode = pvt_->validateRT();
  if(errorCode) {
    return errorCode;
  }

  // TODO
  // Check softlimits, max velo and max acc
  pvtOk_ = true;
  return 0;
}

void ecmcAxisSequencer::setEnable(int enable) {
  if (enable && !data_->command_.controlWord_.enableCmd) {
     traj_->setStartPos(data_->status_.currentPositionActual);
     traj_->setCurrentPosSet(data_->status_.currentPositionActual);
     traj_->setTargetPos(data_->status_.currentPositionActual);
     data_->status_.currentTargetPosition =
       data_->status_.currentPositionActual;
     data_->status_.currentPositionSetpoint =
       data_->status_.currentPositionActual;
     data_->status_.currentPositionSetpointOld =
       data_->status_.currentPositionSetpoint;
     data_->status_.currentVelocitySetpoint = 0;
  }
  traj_->setEnable(enable);

  if(pvtOk_) {
    pvt_->setExecute(0);
  }
}

void ecmcAxisSequencer::initStop() {
  if(data_->command_.controlWord_.enableDbgPrintout) {
    printf("ecmcAxisSequencer::initStopPVT(): Initiating new stopramp...\n");
  }

  traj_->setEnable(1);
  data_->command_.trajSource = ECMC_DATA_SOURCE_INTERNAL;

  temporaryLocalTrajSource_ = true;

  traj_->setStartPos(data_->status_.currentPositionActual);
  traj_->setCurrentPosSet(data_->status_.currentPositionActual);
  traj_->setMotionMode(ECMC_MOVE_MODE_VEL);
  traj_->setTargetVel(0);    
  traj_->initStopRamp(data_->status_.currentPositionSetpoint,
                      data_->status_.currentVelocitySetpoint,
                      0);
  traj_->setExecute(0);
  traj_->setExecute(1);
  
  if(pvtmode_ && !pvtStopping_) {
    data_->command_.command = ECMC_CMD_MOVEABS;
    printf("PVT stopping...\n");
    pvtStopping_ = true;  // Latch stop if in PVT
    pvt_->setExecute(0);  // stop PVT

  }
  data_->status_.currentPositionSetpoint = traj_->getNextPosSet();
  data_->status_.currentVelocitySetpoint = traj_->getNextVel();
  data_->command_.positionTarget = traj_->getCurrentPosSet();
}

// To auto restore poslag monitoring if needed (for instance for external trigged homing seq)
void ecmcAxisSequencer::latchPosLagMonStateBeforeSeq() {  
  monPosLagEnaStatePriorHome_ = mon_->getEnableLagMon();
  monPosLagRestoreNeeded_ = true;
}

void ecmcAxisSequencer::restorePosLagMonAfterSeq() {
  if(monPosLagRestoreNeeded_) {
    mon_->setEnableLagMon(monPosLagEnaStatePriorHome_);
  }
  monPosLagRestoreNeeded_ = false;
}

ecmcAxisPVTSequence* ecmcAxisSequencer::getPVTObject() {
  return pvt_;
}
