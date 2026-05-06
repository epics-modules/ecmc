/*************************************************************************\
* Copyright (c) 2026 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcMcRuntime.cpp
*
*  Created on: Mar 27, 2026
*
\*************************************************************************/

#include "ecmcMcRuntime.h"

#include "ecmcAxisBase.h"
#include "ecmcAxisData.h"
#include "ecmcErrorsList.h"
#include "ecmcGlobalsExtern.h"

namespace {

ecmcAxisBase *resolveAxis(const ecmcMcAxisRef &axisRef, int *errorCode) {
  if (errorCode) {
    *errorCode = 0;
  }

  if (axisRef.axisIndex < 0 || axisRef.axisIndex >= ECMC_MAX_AXES) {
    if (errorCode) {
      *errorCode = ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE;
    }
    return nullptr;
  }

  auto *axis = axes[axisRef.axisIndex];
  if (!axis) {
    if (errorCode) {
      *errorCode = ERROR_MAIN_AXIS_OBJECT_NULL;
    }
    return nullptr;
  }

  return axis;
}

bool isMoveAbsoluteActive(ecmcAxisBase *axis) {
  return axis->getExecute() &&
         axis->getCommand() == ECMC_CMD_MOVEABS &&
         axis->getLocalBusy();
}

bool isMoveRelativeActive(ecmcAxisBase *axis) {
  return axis->getExecute() &&
         axis->getCommand() == ECMC_CMD_MOVEREL &&
         axis->getLocalBusy();
}

bool isHomingActive(ecmcAxisBase *axis) {
  return axis->getExecute() &&
         axis->getCommand() == ECMC_CMD_HOMING &&
         axis->getBusy();
}

bool isMoveVelocityActive(ecmcAxisBase *axis) {
  return axis->getExecute() &&
         axis->getCommand() == ECMC_CMD_MOVEVEL &&
         axis->getLocalBusy();
}

bool isAxisAtTarget(ecmcAxisBase *axis) {
  auto *status = axis->getAxisStatusDataPtr();
  return status && status->statusWord_.attarget;
}

bool axisStillOwnsCommand(ecmcAxisBase *axis, motionCommandTypes command) {
  return axis->getExecute() && axis->getCommand() == command;
}

bool axisDiscreteMoveDone(ecmcAxisBase *axis, motionCommandTypes command) {
  if (!axisStillOwnsCommand(axis, command)) {
    return false;
  }

  const bool atTargetMonEnabled =
    axis->getMon() && axis->getMon()->getEnableAtTargetMon();
  return !atTargetMonEnabled || isAxisAtTarget(axis);
}

bool isAxisHomed(ecmcAxisBase *axis) {
  bool homed = false;
  axis->getAxisHomed(&homed);
  return homed;
}

}  // namespace

void ecmcMcFBBase::beginCycle() {
  Done           = false;
  CommandAborted = false;
  Active         = false;
}

bool ecmcMcFBBase::risingEdge(bool execute) {
  const bool edge = execute && !executeOld_;
  executeOld_     = execute;
  return edge;
}

void ecmcMcFBBase::setError(uint32_t errorId) {
  Error   = true;
  ErrorID = errorId;
  Busy    = false;
  Active  = false;
}

void ecmcMcFBBase::clearError() {
  Error   = false;
  ErrorID = 0;
}

int ecmcMcPower::run(ecmcMcAxisRef axisRef, bool enable) {
  beginCycle();

  int errorCode = 0;
  auto *axis    = resolveAxis(axisRef, &errorCode);
  if (!axis) {
    setError(errorCode);
    Valid  = false;
    Status = false;
    return errorCode;
  }

  if (axis->getErrorID()) {
    setError(axis->getErrorID());
  } else {
    clearError();
  }

  if (axis->getEnable() != enable) {
    errorCode = axis->setEnable(enable);
    if (errorCode) {
      setError(errorCode);
      Valid  = false;
      Status = false;
      return errorCode;
    }
  }

  Status = axis->getEnabled();
  Valid  = !Error;
  Busy   = enable && !Status;
  Active = enable;

  return 0;
}

int ecmcMcReset::run(ecmcMcAxisRef axisRef, bool execute) {
  beginCycle();

  int errorCode = 0;
  auto *axis    = resolveAxis(axisRef, &errorCode);
  if (!axis) {
    issuedReset_ = false;
    executeOld_  = execute;
    setError(errorCode);
    return errorCode;
  }

  const bool trigger = risingEdge(execute);

  if (!execute) {
    issuedReset_ = false;
    Busy         = false;
    Active       = false;
    if (axis->getErrorID()) {
      setError(axis->getErrorID());
    } else {
      clearError();
    }
    return 0;
  }

  Active = true;

  if (trigger) {
    axis->setReset(true);
    issuedReset_ = true;
  }

  errorCode = axis->getErrorID();
  if (errorCode) {
    Busy = true;
    setError(errorCode);
    return errorCode;
  }

  clearError();
  Busy = false;

  if (issuedReset_) {
    Done        = true;
    issuedReset_ = false;
  }

  return 0;
}

int ecmcMcMoveAbsolute::run(ecmcMcAxisRef axisRef,
                            bool          execute,
                            double        position,
                            double        velocity,
                            double        acceleration,
                            double        deceleration) {
  beginCycle();

  int errorCode = 0;
  auto *axis    = resolveAxis(axisRef, &errorCode);
  if (!axis) {
    issuedCommand_      = false;
    awaitingStandstill_ = false;
    executeOld_         = execute;
    setError(errorCode);
    return errorCode;
  }

  if (axis->getErrorID()) {
    issuedCommand_      = false;
    awaitingStandstill_ = false;
    setError(axis->getErrorID());
    executeOld_ = execute;
    return static_cast<int>(ErrorID);
  }

  clearError();

  const bool trigger = risingEdge(execute);

  if (!execute) {
    issuedCommand_      = false;
    awaitingStandstill_ = false;
    Busy                = false;
    Active              = false;
    return 0;
  }

  if (trigger) {
    errorCode = axis->moveAbsolutePosition(position,
                                           velocity,
                                           acceleration,
                                           deceleration);
    if (errorCode) {
      issuedCommand_      = false;
      awaitingStandstill_ = false;
      setError(errorCode);
      return errorCode;
    }
    issuedCommand_      = true;
    awaitingStandstill_ = false;
  } else if (issuedCommand_ && isMoveAbsoluteActive(axis)) {
    errorCode = axis->moveAbsolutePosition(position,
                                           velocity,
                                           acceleration,
                                           deceleration);
    if (errorCode) {
      issuedCommand_      = false;
      awaitingStandstill_ = false;
      setError(errorCode);
      return errorCode;
    }
  }

  const bool moveAbsActive = isMoveAbsoluteActive(axis);

  if (issuedCommand_ && moveAbsActive) {
    Busy   = true;
    Active = true;
    return 0;
  }

  if (issuedCommand_ && !moveAbsActive) {
    issuedCommand_      = false;
    awaitingStandstill_ = true;
  }

  Busy   = false;
  Active = false;

  if (awaitingStandstill_) {
    if (!axisStillOwnsCommand(axis, ECMC_CMD_MOVEABS)) {
      CommandAborted = true;
      awaitingStandstill_ = false;
    } else if (axisDiscreteMoveDone(axis, ECMC_CMD_MOVEABS)) {
      Done = true;
      awaitingStandstill_ = false;
    }
  }

  return 0;
}

int ecmcMcMoveRelative::run(ecmcMcAxisRef axisRef,
                            bool          execute,
                            double        distance,
                            double        velocity,
                            double        acceleration,
                            double        deceleration) {
  beginCycle();

  int errorCode = 0;
  auto *axis    = resolveAxis(axisRef, &errorCode);
  if (!axis) {
    issuedCommand_      = false;
    awaitingStandstill_ = false;
    executeOld_         = execute;
    setError(errorCode);
    return errorCode;
  }

  if (axis->getErrorID()) {
    issuedCommand_      = false;
    awaitingStandstill_ = false;
    setError(axis->getErrorID());
    executeOld_ = execute;
    return static_cast<int>(ErrorID);
  }

  clearError();

  const bool trigger = risingEdge(execute);

  if (!execute) {
    issuedCommand_      = false;
    awaitingStandstill_ = false;
    Busy                = false;
    Active              = false;
    return 0;
  }

  if (trigger) {
    errorCode = axis->moveRelativePosition(distance,
                                           velocity,
                                           acceleration,
                                           deceleration);
    if (errorCode) {
      issuedCommand_      = false;
      awaitingStandstill_ = false;
      setError(errorCode);
      return errorCode;
    }
    issuedCommand_      = true;
    awaitingStandstill_ = false;
  } else if (issuedCommand_ && isMoveRelativeActive(axis)) {
    errorCode = axis->moveRelativePosition(distance,
                                           velocity,
                                           acceleration,
                                           deceleration);
    if (errorCode) {
      issuedCommand_      = false;
      awaitingStandstill_ = false;
      setError(errorCode);
      return errorCode;
    }
  }

  const bool moveRelActive = isMoveRelativeActive(axis);

  if (issuedCommand_ && moveRelActive) {
    Busy   = true;
    Active = true;
    return 0;
  }

  if (issuedCommand_ && !moveRelActive) {
    issuedCommand_      = false;
    awaitingStandstill_ = true;
  }

  Busy   = false;
  Active = false;

  if (awaitingStandstill_) {
    if (!axisStillOwnsCommand(axis, ECMC_CMD_MOVEREL)) {
      CommandAborted = true;
      awaitingStandstill_ = false;
    } else if (axisDiscreteMoveDone(axis, ECMC_CMD_MOVEREL)) {
      Done = true;
      awaitingStandstill_ = false;
    }
  }

  return 0;
}

int ecmcMcHome::run(ecmcMcAxisRef axisRef,
                    bool          execute,
                    int           seqId,
                    double        homePosition,
                    double        velocityTowardsCam,
                    double        velocityOffCam,
                    double        acceleration,
                    double        deceleration) {
  beginCycle();

  int errorCode = 0;
  auto *axis    = resolveAxis(axisRef, &errorCode);
  if (!axis) {
    issuedCommand_ = false;
    executeOld_    = execute;
    setError(errorCode);
    return errorCode;
  }

  if (axis->getErrorID()) {
    issuedCommand_ = false;
    setError(axis->getErrorID());
    executeOld_ = execute;
    return static_cast<int>(ErrorID);
  }

  clearError();

  const bool trigger = risingEdge(execute);

  if (!execute) {
    issuedCommand_ = false;
    Busy           = false;
    Active         = false;
    return 0;
  }

  if (trigger) {
    errorCode = axis->moveHome(seqId,
                               homePosition,
                               velocityTowardsCam,
                               velocityOffCam,
                               acceleration,
                               deceleration);
    if (errorCode) {
      issuedCommand_ = false;
      setError(errorCode);
      return errorCode;
    }
    issuedCommand_ = true;
  }

  const bool homingActive = isHomingActive(axis);

  Busy   = issuedCommand_ && homingActive;
  Active = homingActive;

  if (issuedCommand_ && !homingActive) {
    if (isAxisHomed(axis)) {
      Done = true;
    } else {
      CommandAborted = true;
    }
    issuedCommand_ = false;
  }

  return 0;
}

int ecmcMcMoveVelocity::run(ecmcMcAxisRef axisRef,
                            bool          execute,
                            double        velocity,
                            double        acceleration,
                            double        deceleration) {
  beginCycle();

  int errorCode = 0;
  auto *axis    = resolveAxis(axisRef, &errorCode);
  if (!axis) {
    issuedCommand_ = false;
    executeOld_    = execute;
    InVelocity     = false;
    setError(errorCode);
    return errorCode;
  }

  if (axis->getErrorID()) {
    issuedCommand_ = false;
    InVelocity     = false;
    setError(axis->getErrorID());
    executeOld_ = execute;
    return static_cast<int>(ErrorID);
  }

  clearError();

  const bool trigger = risingEdge(execute);

  if (!execute) {
    issuedCommand_ = false;
    InVelocity     = false;
    Busy           = false;
    Active         = false;
    return 0;
  }

  if (trigger) {
    errorCode = axis->moveVelocity(velocity, acceleration, deceleration);
    if (errorCode) {
      issuedCommand_ = false;
      InVelocity     = false;
      setError(errorCode);
      return errorCode;
    }
    issuedCommand_ = true;
  } else if (issuedCommand_ && isMoveVelocityActive(axis)) {
    errorCode = axis->moveVelocity(velocity, acceleration, deceleration);
    if (errorCode) {
      issuedCommand_ = false;
      InVelocity     = false;
      setError(errorCode);
      return errorCode;
    }
  }

  const bool moveVelActive = isMoveVelocityActive(axis);

  Busy       = issuedCommand_ && moveVelActive;
  Active     = moveVelActive;
  InVelocity = moveVelActive;

  if (issuedCommand_ && !moveVelActive) {
    CommandAborted = true;
    issuedCommand_ = false;
  }

  return 0;
}

int ecmcMcHalt::run(ecmcMcAxisRef axisRef, bool execute) {
  beginCycle();

  int errorCode = 0;
  auto *axis    = resolveAxis(axisRef, &errorCode);
  if (!axis) {
    haltIssued_ = false;
    executeOld_ = execute;
    setError(errorCode);
    return errorCode;
  }

  if (axis->getErrorID()) {
    haltIssued_ = false;
    setError(axis->getErrorID());
    executeOld_ = execute;
    return static_cast<int>(ErrorID);
  }

  clearError();

  const bool trigger = risingEdge(execute);

  if (!execute) {
    haltIssued_ = false;
    Busy        = false;
    Active      = false;
    return 0;
  }

  if (trigger) {
    errorCode = axis->setExecute(false);
    if (errorCode) {
      haltIssued_ = false;
      setError(errorCode);
      return errorCode;
    }
    haltIssued_ = true;
  }

  Busy   = haltIssued_ && axis->getBusy();
  Active = Busy;

  if (haltIssued_ && !axis->getBusy()) {
    Done       = true;
    haltIssued_ = false;
  }

  return 0;
}

int ecmcMcReadStatus::run(ecmcMcAxisRef axisRef, bool enable) {
  beginCycle();

  int errorCode = 0;
  auto *axis    = resolveAxis(axisRef, &errorCode);
  if (!axis) {
    Valid = false;
    setError(errorCode);
    return errorCode;
  }

  if (!enable) {
    Valid              = false;
    ErrorStop          = false;
    Disabled           = false;
    Stopping           = false;
    Homing             = false;
    StandStill         = false;
    DiscreteMotion     = false;
    ContinuousMotion   = false;
    SynchronizedMotion = false;
    Busy               = false;
    clearError();
    return 0;
  }

  auto *status = axis->getAxisStatusDataPtr();
  if (!status) {
    Valid = false;
    setError(ERROR_MAIN_AXIS_OBJECT_NULL);
    return static_cast<int>(ErrorID);
  }

  const int command     = axis->getCommand();
  const bool axisBusy   = status->statusWord_.busy || axis->getBusy();
  const bool enabled    = axis->getEnabled();
  const bool errorStop  = axis->getErrorID() != 0;
  const bool homing     = axisBusy && command == ECMC_CMD_HOMING;
  const bool discrete   = axisBusy &&
                          (command == ECMC_CMD_MOVEABS ||
                           command == ECMC_CMD_MOVEREL);
  const bool continuous = axisBusy && command == ECMC_CMD_MOVEVEL;

  ErrorStop          = errorStop;
  Disabled           = !enabled;
  Homing             = homing;
  DiscreteMotion     = discrete;
  ContinuousMotion   = continuous;
  Stopping           = axisBusy && !axis->getExecute() && !homing;
  StandStill         = enabled && !axisBusy && !errorStop;
  SynchronizedMotion = false;
  Busy               = false;
  Active             = false;
  Valid              = true;

  if (errorStop) {
    setError(axis->getErrorID());
    return static_cast<int>(ErrorID);
  }

  clearError();
  return 0;
}

int ecmcMcReadActualPosition::run(ecmcMcAxisRef axisRef, bool enable) {
  beginCycle();

  int errorCode = 0;
  auto *axis    = resolveAxis(axisRef, &errorCode);
  if (!axis) {
    Valid    = false;
    Position = 0.0;
    setError(errorCode);
    return errorCode;
  }

  if (!enable) {
    Valid = false;
    Busy  = false;
    clearError();
    return 0;
  }

  errorCode = axis->getPosAct(&Position);
  if (errorCode) {
    Valid = false;
    setError(errorCode);
    return errorCode;
  }

  if (axis->getErrorID()) {
    Valid = false;
    setError(axis->getErrorID());
    return static_cast<int>(ErrorID);
  }

  clearError();
  Valid = true;
  Busy  = false;
  return 0;
}

int ecmcMcReadActualVelocity::run(ecmcMcAxisRef axisRef, bool enable) {
  beginCycle();

  int errorCode = 0;
  auto *axis    = resolveAxis(axisRef, &errorCode);
  if (!axis) {
    Valid    = false;
    Velocity = 0.0;
    setError(errorCode);
    return errorCode;
  }

  if (!enable) {
    Valid = false;
    Busy  = false;
    clearError();
    return 0;
  }

  errorCode = axis->getVelAct(&Velocity);
  if (errorCode) {
    Valid = false;
    setError(errorCode);
    return errorCode;
  }

  if (axis->getErrorID()) {
    Valid = false;
    setError(axis->getErrorID());
    return static_cast<int>(ErrorID);
  }

  clearError();
  Valid = true;
  Busy  = false;
  return 0;
}
