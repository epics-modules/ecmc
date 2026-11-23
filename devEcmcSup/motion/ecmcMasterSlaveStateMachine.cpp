/*************************************************************************\
* Copyright (c) 2024 Paul Scherrer Institut
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcMasterSlaveStateMachine.cpp
*
*  Created on: Jun 09, 2025
*      Author: anderssandstrom
*
\*************************************************************************/
#include "ecmcMasterSlaveStateMachine.h"
#include "ecmcRtLogger.h"

#define ecmcRtLoggerLogDebug(...) \
  ECMC_RT_LOG_DEBUG_SOURCE(ECMC_RT_LOG_SOURCE_MASTER_SLAVE, index_, __VA_ARGS__)

ecmcMasterSlaveStateMachine::ecmcMasterSlaveStateMachine(ecmcAsynPortDriver *asynPortDriver,
                                                         int index,
                                                         const char *name,
                                                         double sampleTimeS,
                                                         ecmcAxisGroup *masterGrp,
                                                         ecmcAxisGroup *slaveGrp,
                                                         int autoDisbleMasters,
                                                         int autoDisbleSlaves){
  asynPortDriver_           = asynPortDriver;
  asynControl_              = NULL;
  asynState_                = NULL;
  asynStatus_               = NULL;
  index_                    = index;
  name_                     = name;
  sampleTimeS_              = sampleTimeS;
  timeCounter_              = 0;
  masterGrp_                = masterGrp;
  slaveGrp_                 = slaveGrp;
  validationOK_             = false;
  masterTrajectoryStartAllowed_ = false;
  asynInitOk_               = false;  
  status_                   = ECMC_MST_SLV_STATUS_IDLE;
  statusWord_               = ECMC_MST_SLV_STATUS_IDLE;
  state_                    = ECMC_MST_SLV_STATE_IDLE;    
  idleCounter_              = 0;
  masterGroupWasBusy_       = false;
  masterGroupReachedTarget_ = false;
  masterDisableInProgress_  = false;
  slaveTrajSourceExternalWaitCycles_ = 0;
  masterGroupBusyCycles_    = 0;
  masterAtTargetTimeoutS_ = MST_SLV_MASTER_AT_TARGET_TIMEOUT_DEFAULT_S;
  masterAtTargetTimeS_    = 0;
  masterPrepareTimeoutS_ = masterAtTargetTimeoutS_;
  masterPrepareTimeS_    = 0;
  executeCycleCounter_   = 0;
  transitionCount_       = 0;
  lastTransitionCycle_   = 0;
  lastFaultCycle_        = 0;
  previousState_         = ECMC_MST_SLV_STATE_IDLE;
  trackedState_          = ECMC_MST_SLV_STATE_IDLE;
  lastTransitionReason_  = ECMC_MST_SLV_TRANSITION_NONE;
  lastFaultCode_         = 0;
  lastFaultStatus_       = ECMC_MST_SLV_STATUS_IDLE;
  memset(&control_,0,sizeof(control_));
  memset(&controlOld_,0,sizeof(controlOld_));

  control_.enable             = 1;
  control_.autoDisableMasters = autoDisbleMasters;
  control_.autoDisableSlaves  = autoDisbleSlaves;
  control_.enableDbgPrintouts = false;
  
  slaveGrp_->setMRIgnoreDisableStatusCheck(true);
  masterGrp_->setMRIgnoreDisableStatusCheck(true);

  ecmcRtLoggerLogInfo("%s/%s:%d: INFO: Master/slave state machine[%d] %s created.\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          index_,
          name_.c_str());

  int errorCode = initAsyn();
  if (errorCode) {
    setErrorID(errorCode);
  } else {
    masterGrp_->setMotionStartPermissive(&masterTrajectoryStartAllowed_);
  }
};

ecmcMasterSlaveStateMachine::~ecmcMasterSlaveStateMachine(){
  masterGrp_->setMotionStartPermissive(nullptr);
};

const char* ecmcMasterSlaveStateMachine::getName(){
  return name_.c_str();
};

int ecmcMasterSlaveStateMachine::getIndex() const {
  return index_;
}

int ecmcMasterSlaveStateMachine::getState() const {
  return static_cast<int>(state_);
}

int ecmcMasterSlaveStateMachine::getStatus() const {
  return status_;
}

uint32_t ecmcMasterSlaveStateMachine::getStatusWord() const {
  return buildStatusWord();
}

int ecmcMasterSlaveStateMachine::getEnabled() const {
  return control_.enable;
}

int ecmcMasterSlaveStateMachine::getAutoDisableMasters() const {
  return control_.autoDisableMasters;
}

int ecmcMasterSlaveStateMachine::getAutoDisableSlaves() const {
  return control_.autoDisableSlaves;
}

int ecmcMasterSlaveStateMachine::getPreviousState() const {
  return static_cast<int>(previousState_);
}

int ecmcMasterSlaveStateMachine::getLastTransitionReason() const {
  return static_cast<int>(lastTransitionReason_);
}

uint64_t ecmcMasterSlaveStateMachine::getExecuteCycleCount() const {
  return executeCycleCounter_;
}

uint64_t ecmcMasterSlaveStateMachine::getTransitionCount() const {
  return transitionCount_;
}

uint64_t ecmcMasterSlaveStateMachine::getLastTransitionCycle() const {
  return lastTransitionCycle_;
}

int ecmcMasterSlaveStateMachine::getLastFaultCode() const {
  return lastFaultCode_;
}

int ecmcMasterSlaveStateMachine::getLastFaultStatus() const {
  return lastFaultStatus_;
}

uint64_t ecmcMasterSlaveStateMachine::getLastFaultCycle() const {
  return lastFaultCycle_;
}

void ecmcMasterSlaveStateMachine::transitionTo(
  masterSlaveStates newState,
  masterSlaveTransitionReason reason) {
  if(state_ == newState) {
    return;
  }
  previousState_ = state_;
  state_ = newState;
  trackedState_ = newState;
  lastTransitionReason_ = reason;
  lastTransitionCycle_ = executeCycleCounter_;
  if(transitionCount_ < UINT64_MAX) {
    transitionCount_++;
  }
}

void ecmcMasterSlaveStateMachine::latchFault(int errorCode, int status) {
  lastFaultCode_ = errorCode;
  lastFaultStatus_ = status;
  lastFaultCycle_ = executeCycleCounter_;
}

uint32_t ecmcMasterSlaveStateMachine::buildStatusWord() const {
  uint32_t value = static_cast<uint32_t>(status_) &
                   ECMC_MST_SLV_STATUS_WORD_PHASE_MASK;
  value |= (static_cast<uint32_t>(previousState_) <<
            ECMC_MST_SLV_STATUS_WORD_PREVIOUS_STATE_SHIFT) &
           ECMC_MST_SLV_STATUS_WORD_PREVIOUS_STATE_MASK;
  value |= (static_cast<uint32_t>(lastTransitionReason_) <<
            ECMC_MST_SLV_STATUS_WORD_TRANSITION_REASON_SHIFT) &
           ECMC_MST_SLV_STATUS_WORD_TRANSITION_REASON_MASK;
  if(lastFaultCode_) {
    value |= ECMC_MST_SLV_STATUS_WORD_HISTORICAL_FAULT;
  }
  return value;
}

void ecmcMasterSlaveStateMachine::resetMasterRuntimeState() {
  masterTrajectoryStartAllowed_ = false;
  masterGroupWasBusy_ = false;
  masterGroupReachedTarget_ = false;
  masterDisableInProgress_ = false;
  slaveTrajSourceExternalWaitCycles_ = 0;
  masterGroupBusyCycles_ = 0;
  masterAtTargetTimeS_ = 0;
  masterPrepareTimeS_ = 0;
}

void ecmcMasterSlaveStateMachine::enterIdleFromMaster() {
  slaveGrp_->setEnable(0);
  slaveGrp_->setMRCnen(0);
  slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);
  slaveGrp_->setErrorReset();
  masterGrp_->setEnableAutoDisable(1);
  transitionTo(ECMC_MST_SLV_STATE_IDLE,
               ECMC_MST_SLV_TRANSITION_MASTER_COMPLETE);
  status_ = ECMC_MST_SLV_STATUS_IDLE;
  resetMasterRuntimeState();
}

void ecmcMasterSlaveStateMachine::abortMasterToIdle(int errorCode,
                                                    const char *reason) {
  status_ = ECMC_MST_SLV_STATUS_FORCED_TIMEOUT_RECOVERY;
  latchFault(errorCode, status_);
  ecmcRtLoggerLogError("%s/%s:%d: ERROR: Master/slave state machine[%d] %s: %s; disabling all axes.\n",
                       __FILE__,
                       __FUNCTION__,
                       __LINE__,
                       index_,
                       name_.c_str(),
                       reason);
  masterGrp_->setError(errorCode);
  slaveGrp_->setEnable(0);
  masterGrp_->setEnable(0);
  slaveGrp_->setMRCnen(0);
  masterGrp_->setMRCnen(0);
  slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);
  masterGrp_->setEnableAutoDisable(1);
  transitionTo(
    ECMC_MST_SLV_STATE_IDLE,
    errorCode == ERROR_MST_SLV_SM_PREPARE_MASTER_TIMEOUT ?
      ECMC_MST_SLV_TRANSITION_PREPARE_TIMEOUT :
      ECMC_MST_SLV_TRANSITION_MASTER_DISABLE_TIMEOUT);
  resetMasterRuntimeState();
}

void ecmcMasterSlaveStateMachine::execute(){

  if(executeCycleCounter_ < UINT64_MAX) {
    executeCycleCounter_++;
  }

  if(state_ != trackedState_) {
    previousState_ = trackedState_;
    trackedState_ = state_;
    lastTransitionReason_ = ECMC_MST_SLV_TRANSITION_EXTERNAL_STATE_WRITE;
    lastTransitionCycle_ = executeCycleCounter_;
    if(transitionCount_ < UINT64_MAX) {
      transitionCount_++;
    }
  }

  //always update
  refreshAsyn();

  setMrIgnoreEnableAlarm();

  if(!control_.enable) {
    // unblock commands
    if(state_ != ECMC_MST_SLV_STATE_IDLE) {
      masterGrp_->setBlocked(false);
      slaveGrp_->setBlocked(false);
      slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);
      transitionTo(ECMC_MST_SLV_STATE_IDLE,
                   ECMC_MST_SLV_TRANSITION_CONTROL_DISABLED);
      status_ = ECMC_MST_SLV_STATUS_IDLE;
      resetMasterRuntimeState();
    }
    masterTrajectoryStartAllowed_ = true;
    controlOld_ = control_;
    return;
  }

  if (!controlOld_.enable) {
    masterTrajectoryStartAllowed_ = false;
  }

  if(!validationOK_) {
    return;
  }
  
  if(timeCounter_ < MST_SLV_START_DELAY_S) {
    timeCounter_+=sampleTimeS_;
    return;
  }

  switch(state_) {
    case ECMC_MST_SLV_STATE_IDLE:
      stateIdle();
      break;
    case ECMC_MST_SLV_STATE_SLAVES:
      idleCounter_ = 0;
      stateSlave();
      break;
    case ECMC_MST_SLV_STATE_MASTERS:
      idleCounter_ = 0;
      stateMaster();
      break;
    case ECMC_MST_SLV_STATE_RESET:
      idleCounter_ = 0;
      stateReset();
      break;
  };
  controlOld_ = control_;
};

int ecmcMasterSlaveStateMachine::stateIdle(){
  masterTrajectoryStartAllowed_ = false;
  status_ = ECMC_MST_SLV_STATUS_IDLE;
  
  // Slaved axis busy will stay high for 2 cycles after traj source change.
  // Needed in case stop ramp for the slaves is needed.
  // TODO: Fix.. Need better solution here
  if(idleCounter_ < 3) {
    idleCounter_++;
    return 0;
  }

  slaveGrp_->setBlocked(false);
  masterGrp_->setBlocked(false);

  //masterGrp_->setEnable(false);
  //slaveGrp_->setEnable(false);

  const ecmcAxisGroupStatusSummary slaveStatus = slaveGrp_->getStatusSummary(false);
  const ecmcAxisGroupStatusSummary masterStatus = masterGrp_->getStatusSummary(false);
  const bool anySlaveBusy       = slaveStatus.anyBusy;
  const int anySlaveErrorId     = slaveStatus.firstErrorId;
  const bool anyMasterEnabled   = masterStatus.anyEnabled;
  const bool anyMasterEnableCmd = masterStatus.anyEnableCmd;
  const bool anySlaveTrajAnyExt = slaveStatus.anyTrajExternal;

  // State transision to SLAVE
  if( anySlaveBusy &&
    !anySlaveTrajAnyExt &&
    !anyMasterEnabled &&
    !anyMasterEnableCmd) {
    // (un)block commands
    slaveGrp_->setBlocked(false);
    masterGrp_->setBlocked(true);
    transitionTo(ECMC_MST_SLV_STATE_SLAVES,
                 ECMC_MST_SLV_TRANSITION_SLAVE_COMMAND);
    status_ = ECMC_MST_SLV_STATUS_SLAVE_ACTIVE;
    resetMasterRuntimeState();
    if(control_.enableDbgPrintouts) {
      ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Master/slave state machine[%d] %s: state changed IDLE -> SLAVE.\n",
                           __FILE__,
                           __FUNCTION__,
                           __LINE__,
                           index_,
                           name_.c_str());
    }

  // State transision to MASTER
  } else if(anyMasterEnabled &&
            anySlaveErrorId == 0 ) {
    const bool allSlavesEnabled = slaveStatus.allEnabled;
    // Preparing the slaves may be unnecessary after a direct physical move,
    // but the other virtual masters can still be disabled. Entering MASTER
    // with a partially enabled master group trips the lost-enable check.
    if(allSlavesEnabled && masterStatus.allEnabled && !anySlaveBusy){
      const bool anyMasterBusy = masterStatus.anyBusy;
      if(anyMasterBusy) {
        slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_EXTERNAL);
        slaveTrajSourceExternalWaitCycles_ =
          MST_SLV_TRAJ_SRC_CHANGE_WAIT_CYCLES;
        transitionTo(ECMC_MST_SLV_STATE_MASTERS,
                     ECMC_MST_SLV_TRANSITION_MASTER_COMMAND);
        status_ = ECMC_MST_SLV_STATUS_WAIT_SLAVE_EXTERNAL;
        masterGroupReachedTarget_ = false;
        masterDisableInProgress_ = false;
        masterAtTargetTimeS_ = 0;
        masterPrepareTimeS_ = 0;
        // (un)block commands
        masterGrp_->setBlocked(false);
        slaveGrp_->setBlocked(true);
        if(control_.enableDbgPrintouts) {
          ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Master/slave state machine[%d] %s: state changed IDLE -> MASTER.\n",
                               __FILE__,
                               __FUNCTION__,
                               __LINE__,
                               index_,
                               name_.c_str());
        }
      }

    } else {
      status_ = ECMC_MST_SLV_STATUS_PREPARING_MASTER;
      masterPrepareTimeS_ += sampleTimeS_;
      if((masterPrepareTimeoutS_ >= 0) &&
         (masterPrepareTimeS_ > masterPrepareTimeoutS_)) {
        abortMasterToIdle(ERROR_MST_SLV_SM_PREPARE_MASTER_TIMEOUT,
                          "master/slave preparation for MASTER state timed out");
        return 0;
      }
      int errorSlave = slaveGrp_->setEnable(1);
      int errorMaster = masterGrp_->setEnable(1);
      if(errorSlave || errorMaster) {
        latchFault(errorSlave ? errorSlave : errorMaster,
                   ECMC_MST_SLV_STATUS_PREPARING_MASTER);
        slaveGrp_->setEnable(0);
        masterGrp_->setEnable(0);
        slaveGrp_->setMRCnen(0);
        masterGrp_->setMRCnen(0);
        slaveGrp_->setMRSync(1);
        masterGrp_->setMRSync(1);
        slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);        
        if(errorSlave) {
          masterGrp_->setSlavedAxisInError();
        }
        if(control_.enableDbgPrintouts) {
          ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Master/slave state machine[%d] %s: enabling axes failed.\n",
                               __FILE__,
                               __FUNCTION__,
                               __LINE__,
                               index_,
                               name_.c_str());
          ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Master/slave state machine[%d] %s: state changed IDLE -> IDLE.\n",
                               __FILE__,
                               __FUNCTION__,
                               __LINE__,
                               index_,
                               name_.c_str());
        }
        status_ = ECMC_MST_SLV_STATUS_IDLE;
        resetMasterRuntimeState();
        
      }
      if(anySlaveBusy) {
        slaveGrp_->halt();
      }
    }
  } else if(anySlaveErrorId > 0){
    masterGrp_->setSlavedAxisInError();
  } else {
    masterPrepareTimeS_ = 0;
  }

  return 0;
}

int ecmcMasterSlaveStateMachine::stateSlave(){
  masterTrajectoryStartAllowed_ = false;

  slaveGrp_->setBlocked(false);
  masterGrp_->setBlocked(true);
  status_ = ECMC_MST_SLV_STATUS_SLAVE_ACTIVE;

  const ecmcAxisGroupStatusSummary slaveStatus = slaveGrp_->getStatusSummary(false);
  const ecmcAxisGroupStatusSummary masterStatus = masterGrp_->getStatusSummary(false);
  const bool anySlaveBusy = slaveStatus.anyBusy;

  // Maybe add atTarget here?! 
  // Keep like this since this can be handled by adding diableTimout in slave axes and autoDisableSlaves=false
  if(!anySlaveBusy) {
    // Auto disable also if the axis has no cfg to auto disable
    if(control_.autoDisableSlaves || (!slaveGrp_->getAxisAutoDisableEnabled())) {
      slaveGrp_->setEnable(0);
      //slaveGrp_->setMRCnen(0);     
    }
    
    if(control_.autoDisableMasters) {      
      masterGrp_->setEnable(0);
      masterGrp_->setMRCnen(0);
    }

    slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);

    // Sync the master axes
    masterGrp_->setMRSync(1);
    masterGrp_->setMRStop(1);

    transitionTo(ECMC_MST_SLV_STATE_IDLE,
                 ECMC_MST_SLV_TRANSITION_SLAVE_COMPLETE);
    status_ = ECMC_MST_SLV_STATUS_IDLE;
    resetMasterRuntimeState();
    if(control_.enableDbgPrintouts) {
      ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Master/slave state machine[%d] %s: state changed SLAVE -> IDLE.\n",
                           __FILE__,
                           __FUNCTION__,
                           __LINE__,
                           index_,
                           name_.c_str());
    }
  }

  const bool anyMasterEnabled = masterStatus.anyEnabled;
  const bool anyMasterEnableCmd = masterStatus.anyEnableCmd;
  if(anyMasterEnabled || anyMasterEnableCmd) {
    masterGrp_->setEnable(0);
    masterGrp_->setMRCnen(0);
  }
  return 0;
}

int ecmcMasterSlaveStateMachine::stateMaster(){

  masterGrp_->setBlocked(false);
  slaveGrp_->setBlocked(true);
  status_ = ECMC_MST_SLV_STATUS_MASTER_MOVING;

  const ecmcAxisGroupStatusSummary slaveStatus = slaveGrp_->getStatusSummary(false);
  const ecmcAxisGroupStatusSummary masterStatus = masterGrp_->getStatusSummary(false);
  const bool slaveAnyError = slaveStatus.firstErrorId > 0;
  // Reuse the group summary: no extra axis scan in the RT cycle. Physical
  // axes remain independent; only new master trajectories wait for readiness.
  masterTrajectoryStartAllowed_ = slaveStatus.allEnabled &&
                                  slaveStatus.allTrajExternal &&
                                  !slaveStatus.anyIlocked && !slaveAnyError;
  const bool masterAnyEnabled = masterStatus.anyEnabled;
  const bool masterAnyBusy = masterStatus.anyBusy;
  const bool masterAnyError = masterStatus.firstErrorId > 0;

  if(slaveAnyError) {
    masterGrp_->setSlavedAxisInError();
  }

  if(masterAnyEnabled && !masterAnyBusy) {
    if(control_.autoDisableMasters) {
      slaveGrp_->setEnable(0);
      masterGrp_->setEnable(0);
      slaveGrp_->setMRCnen(0);
      masterGrp_->setMRCnen(0);
    }
  }

  if(masterGroupReachedTarget_ &&
     (!masterStatus.allEnableCmd || !slaveStatus.allEnableCmd ||
      !masterStatus.allEnabled || !slaveStatus.allEnabled)) {
    masterDisableInProgress_ = true;
  }

  const bool recoverMasterDisable = masterDisableInProgress_ &&
                                    masterAnyBusy &&
                                    !masterAnyError;

  if(recoverMasterDisable) {
    slaveGrp_->setEnable(1);
    masterGrp_->setEnable(1);
    slaveGrp_->setMRCnen(1);
    masterGrp_->setMRCnen(1);
    masterGroupReachedTarget_ = false;
    masterAtTargetTimeS_ = 0;
  }
  
  bool lostEnableCmd = false;
  if(masterAnyBusy && !masterAnyError && !recoverMasterDisable) {
    lostEnableCmd = !slaveStatus.allEnableCmd || !masterStatus.allEnableCmd;
  }
  
  // One master or slave axis gets killed during motion then kill all and goto IDLE
  if((masterAnyBusy && lostEnableCmd) || masterAnyError) {
    bool lostEnabled = masterAnyEnabled && !masterStatus.allEnabled;
    lostEnabled = lostEnabled || (slaveStatus.anyEnabled && !slaveStatus.allEnabled);
    if(lostEnabled || masterAnyError) {
      ecmcRtLoggerLogError(
        "Master/slave state machine[%d] %s: group state before abort: masters allEnableCmd=%d allEnabled=%d error=0x%x; slaves allEnableCmd=%d allEnabled=%d error=0x%x.\n",
        index_, name_.c_str(), masterStatus.allEnableCmd, masterStatus.allEnabled,
        masterStatus.firstErrorId, slaveStatus.allEnableCmd, slaveStatus.allEnabled,
        slaveStatus.firstErrorId);
      latchFault(masterAnyError ? masterStatus.firstErrorId :
                 ERROR_AXIS_SLAVED_AXIS_INTERLOCK,
                 ECMC_MST_SLV_STATUS_FORCED_TIMEOUT_RECOVERY);
      transitionTo(ECMC_MST_SLV_STATE_IDLE,
                   ECMC_MST_SLV_TRANSITION_LOST_ENABLE_OR_ERROR);
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Master/slave state machine[%d] %s: at least one axis lost enable during motion; disabling all axes.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             index_,
             name_.c_str());
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Master/slave state machine[%d] %s: state changed MASTER -> RESET.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             index_,
             name_.c_str());
      masterGrp_->halt();
      masterGrp_->setEnable(0);
      masterGrp_->setMRStop(1);
      masterGrp_->setMRSync(1);
      slaveGrp_->halt();
      slaveGrp_->setEnable(0);
      slaveGrp_->setMRStop(1);
      slaveGrp_->setMRSync(1);
      slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);
      if(!masterAnyError) { // Dont overwrite error if master error
        masterGrp_->setSlavedAxisIlocked();
      }
      masterGrp_->setEnableAutoDisable(1);
      //stateReset(); // A bit nasty but ....
      resetMasterRuntimeState();
      return 0;
    }
  }

  // Refresh once here so decisions below are based on the latest state after
  // potential enable/disable actions above.
  const ecmcAxisGroupStatusSummary slaveStatusNow = slaveGrp_->getStatusSummary();
  const ecmcAxisGroupStatusSummary masterStatusNow = masterGrp_->getStatusSummary();

  if(masterGroupReachedTarget_ &&
     (!masterStatusNow.allEnableCmd || !slaveStatusNow.allEnableCmd ||
      !masterStatusNow.allEnabled || !slaveStatusNow.allEnabled)) {
    masterDisableInProgress_ = true;
  }

  if(masterDisableInProgress_ && masterStatusNow.anyBusy) {
    status_ = ECMC_MST_SLV_STATUS_PREPARING_MASTER;
    slaveGrp_->setEnable(1);
    masterGrp_->setEnable(1);
    slaveGrp_->setMRCnen(1);
    masterGrp_->setMRCnen(1);

    if(slaveStatusNow.allEnabled && !slaveStatusNow.anyBusy) {
      slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_EXTERNAL);
      slaveTrajSourceExternalWaitCycles_ =
        MST_SLV_TRAJ_SRC_CHANGE_WAIT_CYCLES;
      masterDisableInProgress_ = false;
      masterGroupReachedTarget_ = false;
      masterAtTargetTimeS_ = 0;
    }
    return 0;
  }

  if(!slaveStatusNow.allEnableCmd && !masterDisableInProgress_) {
    slaveGrp_->setEnable(0);
    masterGrp_->setEnable(0);
    slaveGrp_->setMRCnen(0);
    masterGrp_->setMRCnen(0);
    slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);
    masterGrp_->setEnableAutoDisable(1);
    transitionTo(ECMC_MST_SLV_STATE_IDLE,
                 ECMC_MST_SLV_TRANSITION_MASTER_COMPLETE);
    status_ = ECMC_MST_SLV_STATUS_IDLE;
    resetMasterRuntimeState();
    if(control_.enableDbgPrintouts) {
      ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Master/slave state machine[%d] %s: one or more slave enable commands removed.\n",
                           __FILE__,
                           __FUNCTION__,
                           __LINE__,
                           index_,
                           name_.c_str());
      ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Master/slave state machine[%d] %s: state changed MASTER -> IDLE.\n",
                           __FILE__,
                           __FUNCTION__,
                           __LINE__,
                           index_,
                           name_.c_str());
    }
    return 0;
  }

  // Ilock or if any slaved axis is changing to internal source
  const bool anySlaveIlocked = slaveStatusNow.anyIlocked;
  const bool allSlaveTrajExternal = slaveStatusNow.allTrajExternal;
  if(allSlaveTrajExternal) {
    slaveTrajSourceExternalWaitCycles_ = 0;
  }
  if(!anySlaveIlocked && !allSlaveTrajExternal &&
     !masterDisableInProgress_ &&
     (slaveTrajSourceExternalWaitCycles_ > 0)) {
    status_ = ECMC_MST_SLV_STATUS_WAIT_SLAVE_EXTERNAL;
    slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_EXTERNAL);
    slaveTrajSourceExternalWaitCycles_--;
    return 0;
  }
  if((anySlaveIlocked || !allSlaveTrajExternal) && !masterDisableInProgress_){
    if (anySlaveIlocked) {
      slaveGrp_->logActiveInterlocks();
    }
    status_ = ECMC_MST_SLV_STATUS_FORCED_TIMEOUT_RECOVERY;
    latchFault(anySlaveIlocked ? ERROR_AXIS_SLAVED_AXIS_INTERLOCK :
               ERROR_MST_SLV_SM_SLAVE_TRAJ_SRC_TIMEOUT,
               status_);
    slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);
    slaveGrp_->setMRSync(1);
    slaveGrp_->setMRStop(1);
    slaveGrp_->halt();
    if(anySlaveIlocked) {
      masterGrp_->setSlavedAxisIlocked();
    } else {
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Master/slave state machine[%d] %s: slave axes did not switch to external trajectory source in time; leaving MASTER state.\n",
                           __FILE__,
                           __FUNCTION__,
                           __LINE__,
                           index_,
                           name_.c_str());
      masterGrp_->setError(ERROR_MST_SLV_SM_SLAVE_TRAJ_SRC_TIMEOUT);
      masterGrp_->setSlavedAxisTrajSourceChanged();
    }
    masterGrp_->setEnableAutoDisable(1);
    transitionTo(ECMC_MST_SLV_STATE_SLAVES,
                 ECMC_MST_SLV_TRANSITION_SLAVE_TRAJ_SOURCE_FAILED);
    resetMasterRuntimeState();
    if(control_.enableDbgPrintouts) {
      ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Master/slave state machine[%d] %s: slaved axis interlock=%d, all slave trajectory external=%d.\n",
                           __FILE__,
                           __FUNCTION__,
                           __LINE__,
                           index_,
                           name_.c_str(),
                           anySlaveIlocked,
                           allSlaveTrajExternal);
      ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Master/slave state machine[%d] %s: state changed MASTER -> SLAVE.\n",
                           __FILE__,
                           __FUNCTION__,
                           __LINE__,
                           index_,
                           name_.c_str());
    }
    return 0;
  }
  
  // Done?
  if(!masterStatusNow.anyEnabled) {
    enterIdleFromMaster();
    if(control_.enableDbgPrintouts) {
      ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Master/slave state machine[%d] %s: state changed MASTER -> IDLE.\n",
                           __FILE__,
                           __FUNCTION__,
                           __LINE__,
                           index_,
                           name_.c_str());
    }
    return 0;
  }

  if(masterStatusNow.anyBusy) {
    if(!masterGroupWasBusy_) {
      masterGrp_->resetPendingStallChecks();
    }
    masterGroupReachedTarget_ = false;
    masterDisableInProgress_ = false;
    masterAtTargetTimeS_ = 0;
    masterGroupWasBusy_ = true;
    if(masterGroupBusyCycles_ < UINT64_MAX) {
      masterGroupBusyCycles_++;
    }
  } else if(masterGroupWasBusy_) {
    masterGrp_->armStallCheckForAxesNotAtTarget(masterGroupBusyCycles_);
    masterGroupWasBusy_ = false;
    masterGroupBusyCycles_ = 0;
  } else if(masterStatusNow.allAtTarget) {
    masterGroupReachedTarget_ = true;
  }

  const bool masterAutoDisableWindow = masterGroupReachedTarget_ &&
                                       !masterStatusNow.anyBusy;

  if(masterAutoDisableWindow) {
    status_ = ECMC_MST_SLV_STATUS_WAIT_MASTER_DISABLE;
    masterAtTargetTimeS_ += sampleTimeS_;
    if((masterAtTargetTimeoutS_ >= 0) &&
       (masterAtTargetTimeS_ > masterAtTargetTimeoutS_)) {
      abortMasterToIdle(ERROR_MST_SLV_SM_MASTER_AT_TARGET_TIMEOUT,
                        "master axes did not leave MASTER state after reaching target");
      return 0;
    }
  } else {
    masterAtTargetTimeS_ = 0;
    if(!masterStatusNow.anyBusy) {
      status_ = ECMC_MST_SLV_STATUS_WAIT_MASTER_AT_TARGET;
    }
  }

  // Once all master axes have reached target after the last master-group move,
  // keep auto-disable allowed and bound how long the slave axes may stay blocked.
  masterGrp_->setEnableAutoDisable(masterAutoDisableWindow);

  // ensure attarget/reduced current of slave axes
  const bool mastersAllowSlaveReducedTorque =
    masterStatusNow.allWithinSlvCtrlDb;
  slaveGrp_->setAxisIsWithinCtrlDBExtTraj(
    mastersAllowSlaveReducedTorque);
  return 0;
}

int ecmcMasterSlaveStateMachine::stateReset() {
  masterTrajectoryStartAllowed_ = false;
  slaveGrp_->setEnable(0);
  masterGrp_->setEnable(0);
  slaveGrp_->setMRCnen(0);
  masterGrp_->setMRCnen(0);
  slaveGrp_->setErrorReset();
  masterGrp_->setErrorReset();
  slaveGrp_->setTrajSrc(ECMC_DATA_SOURCE_INTERNAL);
  masterGrp_->setEnableAutoDisable(1);
  masterGrp_->setBlocked(false);
  slaveGrp_->setBlocked(false);
  transitionTo(ECMC_MST_SLV_STATE_IDLE,
               ECMC_MST_SLV_TRANSITION_RESET_COMPLETE);
  status_ = ECMC_MST_SLV_STATUS_IDLE;
  resetMasterRuntimeState();
  if(control_.enableDbgPrintouts) {
    ecmcRtLoggerLogDebug("%s/%s:%d: DEBUG: Master/slave state machine[%d] %s: state changed RESET -> IDLE.\n",
                         __FILE__,
                         __FUNCTION__,
                         __LINE__,
                         index_,
                         name_.c_str());
  }
  return  0;
}

int ecmcMasterSlaveStateMachine::setMasterAtTargetTimeout(double timeoutS) {
  masterAtTargetTimeoutS_ = timeoutS;
  masterPrepareTimeoutS_ = timeoutS;
  masterAtTargetTimeS_ = 0;
  masterPrepareTimeS_ = 0;
  return 0;
}

double ecmcMasterSlaveStateMachine::getMasterAtTargetTimeout() const {
  return masterAtTargetTimeoutS_;
}

int ecmcMasterSlaveStateMachine::validate(){
  validationOK_ = false;

  if( masterGrp_ == NULL || slaveGrp_ == NULL){
    return ERROR_MST_SLV_SM_GRP_NULL;
  };

  if ((masterGrp_->size() == 0) || (slaveGrp_->size() == 0)) {
    return ERROR_MST_SLV_SM_GRP_EMPTY;
  }

  for(int axisIndex = 0; axisIndex < ECMC_MAX_AXES; axisIndex++) {
    if(masterGrp_->inGroup(axisIndex) && slaveGrp_->inGroup(axisIndex)) {
      ecmcRtLoggerLogError(
        "%s/%s:%d: ERROR: Master/slave state machine[%d] %s: axis %d belongs to both master and slave groups.\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        index_,
        name_.c_str(),
        axisIndex);
      return ERROR_MST_SLV_SM_GROUP_AXIS_OVERLAP;
    }
  }

  if( !asynInitOk_){
    return ERROR_MST_SLV_SM_GRP_INIT_ASYN_FAILED;
  };

  validationOK_ = true;
  return 0;
};

int ecmcMasterSlaveStateMachine::validateAxisOwnership(
  ecmcMasterSlaveStateMachine *other) {
  if(!other || other == this) {
    return 0;
  }

  for(int axisIndex = 0; axisIndex < ECMC_MAX_AXES; axisIndex++) {
    const bool usedHere = masterGrp_->inGroup(axisIndex) ||
                          slaveGrp_->inGroup(axisIndex);
    const bool usedThere = other->masterGrp_->inGroup(axisIndex) ||
                           other->slaveGrp_->inGroup(axisIndex);
    if(usedHere && usedThere) {
      validationOK_ = false;
      other->validationOK_ = false;
      ecmcRtLoggerLogError(
        "%s/%s:%d: ERROR: Master/slave state machines[%d] %s and [%d] %s both control axis %d.\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        index_,
        name_.c_str(),
        other->index_,
        other->name_.c_str(),
        axisIndex);
      return ERROR_MST_SLV_SM_AXIS_OWNERSHIP_CONFLICT;
    }
  }
  return 0;
}



int ecmcMasterSlaveStateMachine::initAsyn() {
  if (asynPortDriver_ == NULL) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Master/slave state machine[%d]: AsynPortDriver object is NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index_,
           ERROR_AXIS_ASYN_PORT_OBJ_NULL);
    return ERROR_AXIS_ASYN_PORT_OBJ_NULL;
  }

  ecmcAsynDataItem *paramTemp = NULL;
  int errorCode               = 0;

  // Control
  errorCode = createAsynParam(ECMC_MST_SLV_OBJ_STR "%d.control",
                              asynParamInt32,
                              ECMC_EC_U32,
                              (uint8_t *)&(control_),
                              sizeof(control_),
                              &paramTemp);

  if (errorCode) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Master/slave state machine[%d]: failed to create control-word asyn parameter.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index_);
    return errorCode;
  }

  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);
  paramTemp->refreshParam(1);
  asynControl_ = paramTemp;

  // State
  errorCode = createAsynParam(ECMC_MST_SLV_OBJ_STR "%d." ECMC_MST_SLVS_STR_STATE,
                              asynParamInt32,
                              ECMC_EC_U32,
                              (uint8_t *)&(state_),
                              sizeof(state_),
                              &paramTemp);

  if (errorCode) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Master/slave state machine[%d]: failed to create state asyn parameter.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index_);
    return errorCode;
  }
  paramTemp->setAllowWriteToEcmc(true);
  paramTemp->refreshParam(1);
  asynState_ = paramTemp;

  // Status
  errorCode = createAsynParam(ECMC_MST_SLV_OBJ_STR "%d." ECMC_MST_SLVS_STR_STATUS,
                              asynParamInt32,
                              ECMC_EC_U32,
                              (uint8_t *)&(statusWord_),
                              sizeof(statusWord_),
                              &paramTemp);

  if (errorCode) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Master/slave state machine[%d]: failed to create status asyn parameter.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index_);
    return errorCode;
  }

  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  asynStatus_ = paramTemp;

  // asyn init fine!
  asynInitOk_ = true;
  return 0;
}

int ecmcMasterSlaveStateMachine::createAsynParam(const char        *nameFormat,
                                  asynParamType      asynType,
                                  ecmcEcDataType     ecmcType,
                                  uint8_t           *data,
                                  size_t             bytes,
                                  ecmcAsynDataItem **asynParamOut) {
  if (asynPortDriver_ == NULL) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Master/slave state machine[%d]: AsynPortDriver object is NULL for %s (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index_,
      nameFormat,
      ERROR_AXIS_ASYN_PORT_OBJ_NULL);
    return ERROR_AXIS_ASYN_PORT_OBJ_NULL;
  }
  *asynParamOut = NULL;
  char  buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char *name                  = NULL;
  unsigned int charCount      = 0;
  ecmcAsynDataItem *paramTemp = NULL;

  charCount = snprintf(buffer,
                       sizeof(buffer),
                       nameFormat,
                       index_);

  if (charCount >= sizeof(buffer) - 1) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Master/slave state machine[%d]: Failed to generate %s; buffer too small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index_,
      nameFormat,
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }
  name      = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                                asynType,
                                                data,
                                                bytes,
                                                ecmcType,
                                                0);

  if (!paramTemp) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Master/slave state machine[%d]: Failed to create default parameter for %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->setAllowWriteToEcmc(false);  
  paramTemp->refreshParam(1);
  *asynParamOut = paramTemp;
  return 0;
}

void ecmcMasterSlaveStateMachine::refreshAsyn() {
  statusWord_ = buildStatusWord();
  asynStatus_->refreshParamRT(0);
  asynControl_->refreshParamRT(0);
  asynState_->refreshParamRT(0);
}

void ecmcMasterSlaveStateMachine::setMrIgnoreEnableAlarm() {
  // Change check MR status check on edge of enable
  if(!control_.enable && controlOld_.enable) {
    slaveGrp_->setMRIgnoreDisableStatusCheck(false);
    masterGrp_->setMRIgnoreDisableStatusCheck(false);
  }

  if(control_.enable && !controlOld_.enable) {
    slaveGrp_->setMRIgnoreDisableStatusCheck(true);
    masterGrp_->setMRIgnoreDisableStatusCheck(true);
  }
}

bool ecmcMasterSlaveStateMachine::getStartupReady() const {
  return masterGrp_ && slaveGrp_ && masterGrp_->getStartupReady() && slaveGrp_->getStartupReady();
}

bool ecmcMasterSlaveStateMachine::isMasterAxis(int axisIndex) const {
  return masterGrp_ && masterGrp_->inGroup(axisIndex);
}

int ecmcMasterSlaveStateMachine::getStartupBlockingAxis() const {
  const int master = masterGrp_ ? masterGrp_->getStartupBlockingAxis() : -1;
  return master >= 0 ? master : (slaveGrp_ ? slaveGrp_->getStartupBlockingAxis() : -1);
}
