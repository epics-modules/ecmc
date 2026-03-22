/*************************************************************************\
* Copyright (c) 2024 Paul Scherrer Institut
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcPVTController.cpp
*
*  Created on: September 04, 2024
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcPVTController.h"

namespace {
const char* pvtStateToString(ecmcPVTSMType state) {
  switch (state) {
  case ECMC_PVT_IDLE:
    return "ECMC_PVT_IDLE";
  case ECMC_PVT_ENABLE_AXES:
    return "ECMC_PVT_ENABLE_AXES";
  case ECMC_PVT_TRIGG_MOVE_AXES_TO_START:
    return "ECMC_PVT_TRIGG_MOVE_AXES_TO_START";
  case ECMC_PVT_WAIT_FOR_AXES_TO_REACH_START:
    return "ECMC_PVT_WAIT_FOR_AXES_TO_REACH_START";
  case ECMC_PVT_TRIGG_PVT:
    return "ECMC_PVT_TRIGG_PVT";
  case ECMC_PVT_EXECUTE_PVT:
    return "ECMC_PVT_EXECUTE_PVT";
  case ECMC_PVT_ABORT:
    return "ECMC_PVT_ABORT";
  case ECMC_PVT_ERROR:
    return "ECMC_PVT_ERROR";
  default:
    return "ECMC_PVT_UNKNOWN";
  }
}
}  // namespace

ecmcPVTController::ecmcPVTController(ecmcAsynPortDriver *asynPortDriver,
                                     double sampleTime) {
  asynPortDriver_ = asynPortDriver;
  asynSoftTrigger_ = NULL;
  sampleTime_     = sampleTime;
  halfSampleTime_ = sampleTime / 2;
  nextTime_       = 0;
  endTime_        = 0;
  executeOld_     = 0;
  execute_        = 0;
  state_          = ECMC_PVT_IDLE;
  stateOld_       = ECMC_PVT_IDLE;
  busy_           = 0;
  startPositions_.reserve(ECMC_MAX_AXES);
  axes_.reserve(ECMC_MAX_AXES);
  pvtObjs_.reserve(ECMC_MAX_AXES);
  triggerDefined_ = 0;
  triggerEcEntryIndex_ = 0;
  triggerCount_        = 0;
  triggerStartTime_    = 0;
  triggerEndTime_      = 0;
  triggerTimeBetween_  = 0;
  triggerValidatedOK_  = false;
  triggerCurrentId_    = 0;
  setTriggerDuration(0.1);
  clearPVTAxes();
  newTrg_              = 0;
  triggerOutputHigh_   = false;
  axesBusyState_       = false;
  axesBusyStateValid_  = false;
  softTrigger_         = 0;
  initAsyn();
}

ecmcPVTController::~ecmcPVTController() {
}

void ecmcPVTController::addAxis(ecmcAxisBase* axis) {
  axes_.push_back(axis);
  pvtObjs_.push_back(axis ? axis->getPVTObject() : NULL);
  axesBusyStateValid_ = false;
  if(axis) {
    LOGINFO4("ecmcPVTController::addAxis(%d)\n", axis->getAxisID());
  } else {
    LOGINFO4("ecmcPVTController::addAxis(NaN)\n");
  }
}

void ecmcPVTController::clearPVTAxes() {
  // make sure global PVT busy is not high
  setAxesBusy(false);
  axes_.clear();
  pvtObjs_.clear();
  startPositions_.clear();
  axesBusyStateValid_ = false;
}

size_t ecmcPVTController::getCurrentTriggerId() {
  return triggerCurrentId_;
}

double ecmcPVTController::getCurrentTime() {
  return nextTime_;
}

void ecmcPVTController::setExecute(bool execute) {
  executeOld_ = execute_;
  execute_ = execute;
  triggerCurrentId_ = 0;
  softTrigger_ = 0;
  setTriggerOutput(false);
  int error = 0;
  if(execute_ && !executeOld_) {
    error=validate();
    if(error) {
      setErrorID(error);
      busy_ = 0;
      setAxesBusy(false);
      execute_ = 0;
      executeOld_ = 0;
      return;
    }
    state_ = ECMC_PVT_ENABLE_AXES;
    busy_ = 1;
    setAxesBusy(true);
  }
  if(!execute) {
    if(busy_) {
      abortPVT();
    }
  }
}

// Execute by ecmc RT
void ecmcPVTController::execute() {
  int error = 0;
  int axesAtStartPosition = 0;
  bool seqDone = 0;
  int enabledState = 0;
  const size_t axisCount = axes_.size();

  switch(state_) {
    case  ECMC_PVT_IDLE:
      if (busy_ || !axesBusyStateValid_ || axesBusyState_) {
        setAxesBusy(false);
      }
      busy_ = 0;
      break;
    case ECMC_PVT_ENABLE_AXES:      
      if(setEnable(1) < 0) {
        state_ = ECMC_PVT_ERROR;
        LOGERR("ecmcPVTController::execute(): Error: Enabling axes failed\n");
      }
      enabledState = checkEnabledState(1);
      if(enabledState < 0) {
        state_ = ECMC_PVT_ERROR;
        LOGERR("ecmcPVTController::execute(): Error: Enabling axes failed\n");
      } else if(enabledState == 1) {
        LOGINFO12("ecmcPVTController::execute(): All axes enabled.\n");
        state_ = ECMC_PVT_TRIGG_MOVE_AXES_TO_START;
      }
      break;
    case ECMC_PVT_TRIGG_MOVE_AXES_TO_START:
      setAxesBusy(false);
      initPVT();  // prepare pvt objects
      error = triggMoveAxesToStart();
      if(error) {
        setErrorID(error);
        state_ = ECMC_PVT_ERROR;
        LOGERR("ecmcPVTController::execute(): Error: Axis in error state when trigger move to start position\n");
        break;
      }
      setAxesBusy(true);
      state_ = ECMC_PVT_WAIT_FOR_AXES_TO_REACH_START;
      break;

    case ECMC_PVT_WAIT_FOR_AXES_TO_REACH_START:

      if(anyAxisInterlocked()) {
        abortPVT();
        setErrorID(ERROR_PVT_CTRL_AXIS_INTERLOCK);
        break;
      }
      axesAtStartPosition = axesAtStart();
      if(axesAtStartPosition < 0) {
        setErrorID(-axesAtStartPosition);
        state_ = ECMC_PVT_ERROR;
        LOGERR("ecmcPVTController::execute(): Error: Axis in error state when moving to start position\n");
      }
      if(axesAtStartPosition > 0 ) {
        state_ = ECMC_PVT_TRIGG_PVT;
        nextTime_ = -sampleTime_; // Start at -1 sample
        // Set time to 0 in all PVT objects
        for(size_t i = 0; i < axisCount; i++ ) {
          auto * const pvt = pvtObjs_[i];
          pvt->setNextTime(0);
          pvt->setCurrTime(0);
        }    
        // get end time from first axis
        auto * const firstPvt = pvtObjs_[0];
        endTime_ = firstPvt->endTime();
        //printf("ecmcPVTController: All axes in position, trigger PVT sequence\n");
      }
      break;

    case ECMC_PVT_TRIGG_PVT:
      error =  triggPVT();  
      if(error){
        setErrorID(error);
        state_ = ECMC_PVT_ERROR;
        LOGERR("ecmcPVTController::execute(): Error: Triggering of PVT objects failed\n");
        break;
      }
      //printf("ecmcPVTController: Executing PVT sequence\n");
      state_ = ECMC_PVT_EXECUTE_PVT;      
      triggerCurrentId_ = 0;
      softTrigger_ = 0;
      break;

    case ECMC_PVT_EXECUTE_PVT:
      // The actual PVT seq
      if(anyAxisInterlocked()) {
        abortPVT();
        setErrorID(ERROR_PVT_CTRL_AXIS_INTERLOCK);
        break;
      }
      // Increase time
      nextTime_ = nextTime_ + sampleTime_;
  
      if(nextTime_> endTime_) {    
        nextTime_ = endTime_;
        seqDone = true;
      }

      for(size_t i = 0; i < axisCount; i++ ) {
        auto * const axis = axes_[i];
        auto * const pvt = pvtObjs_[i];
        if(pvt->getBusy()) {
          pvt->setNextTime(nextTime_);
          if(seqDone) {
            pvt->setBusy(false);
            axis->setTargetPosToCurrSetPos();
            state_ =  ECMC_PVT_IDLE;
            busy_ = false;
            axis->setCommand(ECMC_CMD_MOVEABS);
          }
        }
      }

      break;

    case ECMC_PVT_ABORT:
      for(size_t i = 0; i < axisCount; i++ ) {
        auto * const axis = axes_[i];
        auto * const pvt = pvtObjs_[i];
        pvt->setBusy(false);
        axis->getSeq()->setGlobalBusy(0);
        axis->setTargetPosToCurrSetPos();
        axis->setCommand(ECMC_CMD_MOVEABS);
      }
      setAxesBusy(0);

      // Wait for axes to stop
      if(axisNotBusy()) {
        state_ =  ECMC_PVT_IDLE;
      }

      break;
    case ECMC_PVT_ERROR:
      busy_ = 0;
      for(size_t i = 0; i < axisCount; i++ ) {
        auto * const axis = axes_[i];
        auto * const pvt = pvtObjs_[i];
        pvt->setBusy(false);
        axis->setTargetPosToCurrSetPos();
        state_ =  ECMC_PVT_IDLE;
      }
      setAxesBusy(false);
      break;
  }
      
  if(state_!=stateOld_) {
    LOGINFO12("PVT state changed to: %s, old state %s\n",
              pvtStateToString(state_),
              pvtStateToString(stateOld_));
  }
  stateOld_=state_;

  //#### triggering below (soft and hw) #####
  if(!busy_) {
    setTriggerOutput(false);
    return;
  }

  // reset trigger
  if(nextTime_ > triggerEndTime_+ triggerDuration_) {
    setTriggerOutput(false);
    return; // Done
  }

  // Do not trigger if in wrong state or no triggers
  if(triggerCount_<= 0 || state_!=ECMC_PVT_EXECUTE_PVT) {
    return;
  }
  
  double nextTriggerTime = triggerStartTime_+ triggerCurrentId_ * triggerTimeBetween_;

  if(triggerCurrentId_ < triggerCount_) {
    if(nextTime_ >= (nextTriggerTime + triggerTimeBetween_)) {
      triggerCurrentId_ = triggerCurrentId_ + 1;
      nextTriggerTime = triggerStartTime_+ triggerCurrentId_ * triggerTimeBetween_;
    }
  }

  if( nextTime_ > (nextTriggerTime + halfSampleTime_) && nextTime_ <= (nextTriggerTime + triggerDuration_)) {
    // here we want to also latch data
    setTriggerOutput(true);

    if(newTrg_) { // new pulse: Trigger DAQ in PVTSequence
      softTrigger_++;
      refreshAsyn();
      for(size_t i = 0; i < axisCount; i++ ) {
        auto * const axis = axes_[i];
        auto * const pvt = pvtObjs_[i];
        pvt->setTrgDAQ();
        // Compensate with one sampleTime since this is "next time"
        LOGINFO12("axis[%d]: DAQ trigger %zu at time %lf\n",
                  axis->getAxisID(),
                  triggerCurrentId_ + 1,
                  nextTime_ - sampleTime_);
      }
    }
    newTrg_ = false;
  } else {
    setTriggerOutput(false);
    newTrg_ = true; // trigger data latch at next pulse
  }
}

bool  ecmcPVTController::getBusy() {
  return busy_;
}

int ecmcPVTController::triggMoveAxesToStart() {
  const size_t axisCount = axes_.size();
  startPositions_.resize(axisCount);
  double startPosition = 0;
  int error = 0;
  for(size_t i = 0; i < axisCount; i++ ) {
    auto * const axis = axes_[i];
    auto * const pvt = pvtObjs_[i];
    pvt->startPosition(&startPosition);
    if(pvt->getRelMode()) {
      const double currSetpoint = axis->getCurrentPositionSetpoint();
      startPosition = currSetpoint + startPosition;
      pvt->setPositionOffset(currSetpoint);
    } else {    
      pvt->setPositionOffset(0);
    }
//    printf("ecmcPVTController::triggMoveAxesToStart(): Execute moveAbsolutePosition\n");
//    printf("ecmcPVTController::triggMoveAxesToStart(): localBusy %d, global %d\n", axes_[i]->getLocalBusy(),axes_[i]->getGlobalBusy());
    error = axis->moveAbsolutePosition(startPosition);
    if(error) {
      return error;
    }

    // Save all startpositions to be able to verify that the axes arrived
    startPositions_[i] = startPosition;
  }
  return 0;
}

// negative error
int ecmcPVTController::axesAtStart() {
  int error = 0;
  const size_t axisCount = axes_.size();
  for(size_t i = 0; i < axisCount; i++ ) {
    auto * const axis = axes_[i];
    error = axis->getErrorID();
    if(error) {
      return -error;
    }
    if(axis->getTrajBusy()) {
      return 0;
    }
    auto * const mon = axis->getMon();
    if(mon->getEnableAtTargetMon()) {
      if(!mon->getAtTarget()) {
        return 0;
      }
    }
    //if(axes_[i]->getCurrentPositionSetpoint() != startPositions_[i]) {
    //  printf("ecmcPVTController:: target set %lf, actual set %lf\n",axes_[i]->getCurrentPositionSetpoint(), startPositions_[i]);
    //  return 0;
    //}
  }
  // All axes in correct position to start
  return 1;
}

// negative error
int ecmcPVTController::triggPVT() {
  int error = 0;
  const size_t axisCount = axes_.size();
  for(size_t i = 0; i < axisCount; i++ ) {
    auto * const axis = axes_[i];
    error = axis->getErrorID();
    if(error) {
      return -error;
    }
    
    // Relative are handled with PVTobject with an offset
    error = axis->movePVTAbs(true);  // ignore busy.. have checked before
    if(error) {
      return error;
    }
  }
  // All axes in correct position to start
  return 0;
}

void ecmcPVTController::errorReset() {
  ecmcError::errorReset();
  if(state_ == ECMC_PVT_ERROR) {
    state_ = ECMC_PVT_IDLE;
  }
}

int ecmcPVTController::validate() {
  if(axes_.size()== 0) {
    LOGERR("ecmcPVTController::validate(): Error axis count zero\n");
    return setErrorID(ERROR_PVT_CTRL_AXIS_COUNT_ZERO);
  }

  const int axisBindingError = validateAxisBindings();
  if (axisBindingError) {
    return setErrorID(axisBindingError);
  }

  // trigger
  triggerValidatedOK_ = false;
  if(!triggerDefined_ || triggerCount_ <= 0 ) {
    return 0;   
  }

  triggerValidatedOK_ = true;
  return 0; 
}

int ecmcPVTController::validateAxisBindings() {
  const size_t axisCount = axes_.size();
  for (size_t i = 0; i < axisCount; i++) {
    if (!axes_[i]) {
      LOGERR("ecmcPVTController::validateAxisBindings(): Error axis pointer NULL at index %zu\n",
             i);
      return ERROR_PVT_CTRL_AXIS_NULL;
    }
    if (!pvtObjs_[i]) {
      LOGERR("ecmcPVTController::validateAxisBindings(): Error PVT object NULL for axis %d\n",
             axes_[i]->getAxisID());
      return ERROR_PVT_CTRL_PVT_NULL;
    }
  }
  return 0;
}

int ecmcPVTController::anyAxisInterlocked() {
  const size_t axisCount = axes_.size();
  for(size_t i = 0; i < axisCount; i++ ) {
    auto * const axis = axes_[i];
    if(axis->getSumInterlockOrStop() || axis->getErrorID() > 0) {
      return 1;
    }
  }
  return 0;
}

int ecmcPVTController::abortPVT() {
  const size_t axisCount = axes_.size();
  for(size_t i = 0; i < axisCount; i++ ) {
    auto * const axis = axes_[i];
    axis->stopMotion(0);
    auto * const pvt = pvtObjs_[i];
    pvt->setBusy(false);
  }
  setAxesBusy(false);
  setTriggerOutput(false);
  state_ = ECMC_PVT_ABORT;
  // All axes in correct position to start
  return 0;
}

int ecmcPVTController::axisNotBusy() {
  const size_t axisCount = axes_.size();
  for(size_t i = 0; i < axisCount; i++ ) {
    if(axes_[i]->getTrajBusy()) {
      return 0;
    }
  }
  return 1;
}

void ecmcPVTController::initPVT() {
  const size_t axisCount = axes_.size();
  const trgMode trgModeSelect = triggerCount_ == 0 ?
                                TRG_INT_ON_SEG_CHANGE : TRG_EXT_ON_PULSE_TRG;
  for(size_t i = 0; i < axisCount; i++ ) {
    auto * const pvt = pvtObjs_[i];
    pvt->setExecute(0);
    pvt->setTrgDAQMode(trgModeSelect);
  }
  triggerCurrentId_ = 0;
  newTrg_           = 1;
  setTriggerOutput(false);
}

int ecmcPVTController::setEcEntry(ecmcEcEntry *entry,int entryIndex, int bitIndex) {
  
  int error = setEntryAtIndex(entry,
                              entryIndex,
                              bitIndex);
  if(error) {
    triggerDefined_ = 0;
    return error;
  }
  
  error = validateEntryBit(entryIndex);
  if(error) {
    return setErrorID(error);
  }
  triggerEcEntryIndex_ = entryIndex;
  triggerDefined_ = 1;
  LOGINFO4("ecmcPVTController::setEcEntry(): Trigger defined.\n");
  return 0;
}

int ecmcPVTController::setTriggerInfo(double start, double between, double end, size_t count) { 
  triggerStartTime_   = start;
  triggerEndTime_     = end;
  triggerTimeBetween_ = between;
  triggerCount_       = count;
  setTriggerOutput(false);
  return 0;
}

int ecmcPVTController::setTriggerDuration(double durationS) {
  triggerDuration_ = durationS + sampleTime_;
  setTriggerOutput(false);
  return 0;
}

int ecmcPVTController::setAxesBusy(bool busy) {
  if (axesBusyStateValid_ && axesBusyState_ == busy) {
    return 0;
  }

  const size_t axisCount = axes_.size();
  for(size_t i = 0; i < axisCount; i++ ) {
    auto * const axis = axes_[i];
    if(axis != NULL) {
      axis->getSeq()->setGlobalBusy(busy);
    }
  }
  axesBusyState_ = busy;
  axesBusyStateValid_ = true;
  return 0;
}

ecmcPVTSMType  ecmcPVTController::getSMState() {
  return state_;
}

void ecmcPVTController::setTriggerOutput(bool high) {
  if(!triggerDefined_) {
    triggerOutputHigh_ = false;
    return;
  }

  if(triggerOutputHigh_ == high) {
    return;
  }

  writeEcEntryValue(triggerEcEntryIndex_, high ? 1 : 0);
  triggerOutputHigh_ = high;
}

int ecmcPVTController::setEnable(bool enable) {  
  const size_t axisCount = axes_.size();
  for(size_t i = 0; i < axisCount; i++ ) {
    auto * const axis = axes_[i];
    if(axis != NULL) {
      // Always disable
      if(!enable) {
        axis->setEnable(enable);
        break;
      }
      // do not enable if error
      if(axis->getError()) {
        return -1;
      }      
      if(!axis->getEnable()) {
        axis->setEnable(enable);
      }
    }
  }
  return 0;
}

int ecmcPVTController::checkEnabledState(bool enabled) {
  const size_t axisCount = axes_.size();
  for(size_t i = 0; i < axisCount; i++ ) {
    auto * const axis = axes_[i];
    if(axis != NULL) {
      if(axis->getEnabled() != enabled) {
        return 0;
      }
      //printf("ecmcPVTController::triggMoveAxesToStart(): localBusy %d, global %d\n", axis->getLocalBusy(),axis->getGlobalBusy());
    } else {
      return -1;
    }
  }
  return 1;
}

void ecmcPVTController::refreshAsyn() {
  if(asynSoftTrigger_ == NULL) {
    return;
  }

  // force update
  asynSoftTrigger_->refreshParamRT(1);
}

void ecmcPVTController::initAsyn() {

 if (asynPortDriver_ == NULL) {
    LOGERR("%s/%s:%d: PVT: ERROR: AsynPortDriver object NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_AXIS_ASYN_PORT_OBJ_NULL);
           return;
  }

  ecmcAsynDataItem *paramTemp = NULL;

  paramTemp = asynPortDriver_->addNewAvailParam("pvt.softtrigger",
                              asynParamInt32,
                              (uint8_t *)&(softTrigger_),
                              sizeof(softTrigger_),
                              ECMC_EC_S32,
                              0);

  if (paramTemp == NULL) {
    LOGERR("%s/%s:%d: PVT: ERROR: Failed create pvt.softtrigger asyn parameter.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
           return;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  asynSoftTrigger_ = paramTemp;
  LOGINFO4("ecmcPVTController::initAsyn(): pvt.softtrigger created.\n");

}
