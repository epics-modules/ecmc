#!/usr/bin/env python3
"""Production master/slave state transitions coupled to a real trajectory.

Controlled group/drive boundaries model delayed brake opening and delayed source
acknowledgement; production state-machine methods and trajectory calculations run
unchanged. No EPICS network, IOC, relaxed lag threshold or test-side ready wait.
"""
from pathlib import Path
import os
import subprocess
import tempfile

repo = Path(__file__).resolve().parents[1]
sup = repo / 'devEcmcSup'
source = (sup / 'motion/ecmcMasterSlaveStateMachine.cpp').read_text()
axis_source = (sup / 'motion/ecmcAxisBase.cpp').read_text()
header = (sup / 'motion/ecmcMasterSlaveStateMachine.h').read_text()
group_header = (sup / 'motion/ecmcAxisGroup.h').read_text()


def method(signature, source=source):
    start = source.index(signature)
    brace = source.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


preamble = r'''
#include "ecmcTrajectoryTrapetz.h"
#include "ecmcAsynPortDriver.h"
#include "ecmcErrorsList.h"
#include <cassert>
#include <string>
#include <iostream>
#include <cstdint>
#include <algorithm>
#define ecmcRtLoggerLogInfo(...) ((void)0)
#define ecmcRtLoggerLogError(...) ((void)0)
#define ecmcRtLoggerLogDebug(...) ((void)0)
'''
start = group_header.index('struct ecmcAxisGroupStatusSummary')
preamble += group_header[start:group_header.index('};', start) + 2]
preamble += r'''

class ecmcAxisSequencer {
public:
  ecmcAxisData &data;
  ecmcTrajectoryBase &trajectory;
  ecmcAxisSequencer(ecmcAxisData &d, ecmcTrajectoryBase &t) : data(d), trajectory(t) {}
  void setGlobalBusy(bool busy) { data.status_.statusWord_.globalBusy = busy; }
  int setExecute(bool execute) {
    data.status_.statusWord_.execute = execute;
    const int result = trajectory.setExecute(execute);
    data.status_.statusWord_.localBusy = trajectory.getBusy();
    return result;
  }
};
class ecmcAxisBase : public ecmcError {
public:
  ecmcAxisData &data_;
  ecmcTrajectoryBase &trajectory;
  ecmcAxisSequencer seq_;
  ecmcAsynDataItem publication;
  bool blocked = false, startupHeld = false;
  bool enableAutoEnable_ = true, autoEnableRequest_ = false, motionStartPending_ = false;
  bool encoderPowerEnablePending_ = false, encoderPowerMoveCompleted_ = false;
  bool encoderPowerReady() { return true; }
  int prepareEncoderPowerMotion() { assert(false); return 0; }
  const bool *motionStartPermissive_ = nullptr;
  double autoEnableTimeCounter_ = 0, autoEnableTimoutS_ = 5;
  int executed = 0;
  motionCommandTypes ignoredMotionMessage_ = ECMC_CMD_NOCMD;
  ecmcAxisBase(ecmcAxisData &data, ecmcTrajectoryBase &t) : data_(data), trajectory(t), seq_(data,t) {
    setExternalPtrs(&data.status_.errorCode, &data.status_.warningCode);
    data.axAsynParams_[ECMC_ASYN_AX_CONTROL_BIN_ID] = &publication;
    data.axAsynParams_[ECMC_ASYN_AX_WARNING_ID] = &publication;
    data.control_.command = ECMC_CMD_MOVEABS;
    data.status_.command = ECMC_CMD_MOVEABS;
    data.status_.sampleTime = .001;
  }
  int setExecute(bool);
  int setExecute(bool, bool);
  void clearDisabledBusy();
  void autoEnableSM();
  bool getStartupCommandsHeld() { return startupHeld; }
  bool getEnable() { return data_.status_.statusWord_.enable; }
  bool getExecute() { return data_.status_.statusWord_.execute; }
  bool getBusy() { return data_.status_.statusWord_.localBusy || data_.status_.statusWord_.globalBusy; }
  bool getBlocked() { return blocked; }
  ecmcAxisSequencer *getSeq() { return &seq_; }
  void bumpMotionCommandExecuteCounter() { ++executed; }
  void clearStaleSoftLimitInterlockForValidTarget() {}
  int setEnable(bool enabled) {
    if (!enabled) setExecute(false, true);
    data_.status_.statusWord_.enable = data_.status_.statusWord_.enabled = enabled;
    trajectory.setEnable(enabled);
    return 0;
  }
  double tick() {
    const double position = trajectory.getNextPosSet();
    data_.status_.statusWord_.localBusy = trajectory.getBusy();
    autoEnableSM();
    data_.statusOld_ = data_.status_;
    return position;
  }
};
class ecmcAxisGroup {
public:
  ecmcAxisGroupStatusSummary status{};
  ecmcTrajectoryBase *trajectory = nullptr;
  ecmcAxisBase *axis = nullptr;
  ecmcAxisData *data = nullptr;
  bool blocked = false, enableRequested = false;
  bool otherMasterEnabled = true, otherMasterEnableCmd = true;
  int enableCalls = 0;
  dataSource requestedSource = ECMC_DATA_SOURCE_INTERNAL;
  int error = 0, interlockLogs = 0;
  ecmcAxisGroupStatusSummary getStatusSummary(bool = true) {
    auto result = status;
    if (trajectory) {
      result.anyEnabled = data->status_.statusWord_.enabled || otherMasterEnabled;
      result.allEnabled = data->status_.statusWord_.enabled && otherMasterEnabled;
      result.anyEnableCmd = data->status_.statusWord_.enable || otherMasterEnableCmd;
      result.allEnableCmd = data->status_.statusWord_.enable && otherMasterEnableCmd;
      result.anyBusy = result.allBusy = axis ? axis->getBusy() : trajectory->getBusy();
      result.allAtTarget = !result.anyBusy;
    }
    result.firstErrorId = error;
    return result;
  }
  void setMotionStartPermissive(const bool *permission) {
    if (axis) axis->motionStartPermissive_ = permission;
  }
  int setEnable(bool value) {
    ++enableCalls;
    enableRequested = value;
    if (axis) otherMasterEnableCmd = otherMasterEnabled = value;
    if (!value && trajectory) {
      data->status_.statusWord_.enable = data->status_.statusWord_.enabled = false;
      if (axis) axis->setEnable(false);
      else trajectory->setEnable(false);
    }
    return 0;
  }
  void setBlocked(bool value) { blocked = value; if (axis) axis->blocked = value; }
  int setTrajSrc(dataSource value) { requestedSource = value; return 0; }
  void setError(int code) { error = code; if (axis) axis->setErrorID(code); }
  void setSlavedAxisInError() { setError(ERROR_AXIS_SLAVED_AXIS_IN_ERROR); }
  void setSlavedAxisIlocked() { setError(ERROR_AXIS_SLAVED_AXIS_INTERLOCK); }
  void setSlavedAxisTrajSourceChanged() { setError(ERROR_AXIS_SLAVED_AXIS_TRAJ_SOURCE_CHANGED); }
  void setErrorReset() { error = 0; if (axis) axis->errorReset(); }
  void halt() { if (axis) axis->setExecute(false, true); else if (trajectory) trajectory->setExecute(false); }
  void logActiveInterlocks() { assert(status.anyIlocked); ++interlockLogs; }
  void setMRCnen(bool) {}
  void setMRStop(bool) {}
  void setMRSync(bool) {}
  void setMRIgnoreDisableStatusCheck(bool) {}
  void setEnableAutoDisable(bool) {}
  bool autoDisableEnabled = true;
  bool getAxisAutoDisableEnabled() { return autoDisableEnabled; }
  void resetPendingStallChecks() {}
  void armStallCheckForAxesNotAtTarget(uint64_t) {}
  void setAxisIsWithinCtrlDBExtTraj(bool) {}
};
'''
preamble += '\n'.join(line for line in header.splitlines() if not line.startswith('#include'))
preamble = preamble.replace('  private:', '  public:')
# Compile actual methods; only publication/configuration boundaries are stubs.
methods = '\n'.join(method(signature) for signature in (
    'ecmcMasterSlaveStateMachine::ecmcMasterSlaveStateMachine(',
    'ecmcMasterSlaveStateMachine::~ecmcMasterSlaveStateMachine()',
    'void ecmcMasterSlaveStateMachine::execute()',
    'int ecmcMasterSlaveStateMachine::stateIdle()',
    'int ecmcMasterSlaveStateMachine::stateSlave()',
    'int ecmcMasterSlaveStateMachine::stateMaster()',
    'int ecmcMasterSlaveStateMachine::stateReset()',
    'void ecmcMasterSlaveStateMachine::resetMasterRuntimeState()',
    'void ecmcMasterSlaveStateMachine::transitionTo(',
    'void ecmcMasterSlaveStateMachine::latchFault(',
    'void ecmcMasterSlaveStateMachine::enterIdleFromMaster()',
    'void ecmcMasterSlaveStateMachine::abortMasterToIdle('))
methods += '\n'.join(method(signature, axis_source) for signature in (
    'int ecmcAxisBase::setExecute(bool execute) {',
    'int ecmcAxisBase::setExecute(bool execute, bool ignoreBusy) {',
    'void ecmcAxisBase::autoEnableSM() {'))
methods += r'''
int ecmcMasterSlaveStateMachine::initAsyn() { return 0; }
void ecmcMasterSlaveStateMachine::refreshAsyn() {}
void ecmcMasterSlaveStateMachine::setMrIgnoreEnableAlarm() {}
'''
# Exercise the production disabled-state busy cleanup, including the cycle
# where enable feedback has arrived but the axis state has not advanced yet.
disabled = axis_source.split('case ECMC_AXIS_STATE_DISABLED:', 1)[1]
disabled = disabled.split('    statusWord.busy = false;', 1)[0]
methods += '\nvoid ecmcAxisBase::clearDisabledBusy() {\n' + disabled + '\n}\n'
main = r'''
int main() {
  // Native execute may arrive on the final DISABLED cycle after explicit
  // enable. Losing globalBusy here leaves the group waiting for a master
  // request while the master waits for group readiness, until 0x1432f.
  {
    ecmcAxisData d;
    ecmcTrajectoryTrapetz t(d, .001);
    ecmcAxisBase a(d, t);
    bool ready = false;
    a.motionStartPermissive_ = &ready;
    a.setEnable(true);
    t.setAcc(1); t.setDec(1); t.setTargetVel(.4); t.setTargetPos(.2);
    assert(a.setExecute(true) == 0 && a.motionStartPending_);
    a.clearDisabledBusy();
    assert(a.getBusy());
    for (int i = 0; i < 50; ++i) assert(a.tick() == 0);
    ready = true;
    for (int i = 0; i < 2000; ++i) a.tick();
    assert(a.executed == 1 && !a.getError());
    assert(std::abs(t.ecmcTrajectoryBase::getCurrentPosSet() - .2) < 1e-9);
    // Ordinary disabled cleanup still clears stale global busy.
    a.setEnable(false);
    a.seq_.setGlobalBusy(true);
    a.clearDisabledBusy();
    assert(!a.getBusy());
  }
  // Encoder-power live fixture: disabling the axis auto-disable flag invokes
  // the physical group's immediate-disable fallback when SLAVE completes.
  // A long positive timeout retains enable without changing that SM policy.
  for (bool axisAutoDisable : {false,true}) {
    ecmcAxisGroup masters, slaves;
    slaves.status.allEnabled = slaves.status.anyEnabled = slaves.status.allEnableCmd = true;
    slaves.autoDisableEnabled = axisAutoDisable;
    ecmcMasterSlaveStateMachine machine(nullptr, 0, "physical completion", .001, &masters, &slaves, 0, 0);
    machine.state_ = ECMC_MST_SLV_STATE_SLAVES;
    machine.stateSlave();
    assert(machine.state_ == ECMC_MST_SLV_STATE_IDLE);
    assert(slaves.enableCalls == (axisAutoDisable ? 0 : 1));
    if (!axisAutoDisable) assert(!slaves.enableRequested);
  }
  // Live failure: slaves are already enabled after physical preparation, CEN
  // receives enable/move, and the idle GAP master is still disabled. The fast
  // path must prepare the whole master group before entering MASTER state.
  for (bool skipMasterPreparation : {true, false}) {
    ecmcAxisData data;
    ecmcTrajectoryTrapetz trajectory(data, .001);
    ecmcAxisBase axis(data, trajectory);
    ecmcAxisGroup masters, slaves;
    masters.axis = &axis; masters.data = &data; masters.trajectory = &trajectory;
    masters.otherMasterEnabled = masters.otherMasterEnableCmd = false;
    slaves.status.allEnabled = slaves.status.anyEnabled = slaves.status.allEnableCmd = true;
    ecmcMasterSlaveStateMachine machine(nullptr, 0, "partial master", .001, &masters, &slaves, 0, 0);
    machine.validationOK_ = true;
    machine.timeCounter_ = MST_SLV_START_DELAY_S;
    machine.idleCounter_ = 3;
    data.status_.statusWord_.enable = data.status_.statusWord_.enabled = true;
    trajectory.setEnable(true); trajectory.setAcc(1); trajectory.setDec(1);
    trajectory.setTargetVel(.4); trajectory.setTargetPos(.6);
    assert(axis.setExecute(true) == 0);
    axis.tick();
    if (skipMasterPreparation) {
      // Negative control: the old fast path entered MASTER with GAP disabled.
      // The production abort path then produces exactly the reported error.
      machine.state_ = ECMC_MST_SLV_STATE_MASTERS;
      machine.stateMaster();
      assert(machine.lastFaultCode_ == ERROR_AXIS_SLAVED_AXIS_INTERLOCK);
      assert(masters.error == ERROR_AXIS_SLAVED_AXIS_INTERLOCK);
      continue;
    }
    machine.stateIdle();
    assert(machine.state_ == ECMC_MST_SLV_STATE_IDLE);
    assert(machine.status_ == ECMC_MST_SLV_STATUS_PREPARING_MASTER);
    assert(masters.enableCalls == 1 && masters.otherMasterEnableCmd);
    assert(!trajectory.getExecute() && axis.getBusy());
    machine.stateIdle();
    assert(machine.state_ == ECMC_MST_SLV_STATE_MASTERS);
    slaves.status.allTrajExternal = true;
    for (int i = 0; i < 3000; ++i) {
      axis.tick(); machine.execute();
      assert(machine.lastFaultCode_ == 0 && masters.error == 0);
    }
    assert(std::abs(trajectory.ecmcTrajectoryBase::getCurrentPosSet() - .6) < 1e-9);
  }
  for (bool guarded : {false, true}) for (int brakeDelay : {0, 50, 250, 1000}) {
    ecmcAxisData data;
    ecmcTrajectoryTrapetz trajectory(data, .001);
    ecmcAxisBase axis(data, trajectory);
    ecmcAxisGroup masters, slaves;
    masters.axis = &axis;
    masters.trajectory = &trajectory;
    masters.data = &data;
    ecmcMasterSlaveStateMachine machine(nullptr, 0, "slit", .001, &masters, &slaves, 0, 0);
    // Removing only the start permission reproduces the old sequencing while
    // keeping exactly the same trajectory and state-machine calculations.
    if (!guarded) axis.motionStartPermissive_ = nullptr;
    machine.validationOK_ = true;
    data.status_.statusWord_.enabled = data.status_.statusWord_.enable = true;
    trajectory.setAcc(1); trajectory.setDec(1); trajectory.setEmergDec(2);
    trajectory.setTargetVel(.4); trajectory.setStartPos(0); trajectory.setEnable(true);
    trajectory.setTargetPos(.15); axis.setExecute(true);
    if (guarded) {
      assert(axis.motionStartPending_ && axis.getBusy());
      assert(!trajectory.getExecute() && !trajectory.getBusy() && axis.executed == 0);
    }
    double physical = 0, previous = 0, maxLag = 0;
    int enabledTicks = 0, sourceTicks = 0;
    for (int cycle = 0; cycle < 3000; ++cycle) {
      const double setpoint = axis.tick();
      if (slaves.status.allEnabled && slaves.status.allTrajExternal) physical += setpoint - previous;
      // This detects lost initial motion even if the final master position is correct.
      maxLag = std::max(maxLag, std::abs(setpoint - physical));
      if (guarded) assert(std::abs(setpoint - physical) < 1e-12);
      previous = setpoint;
      machine.execute();
      if (slaves.enableRequested && ++enabledTicks > brakeDelay) {
        slaves.status.allEnableCmd = slaves.status.allEnabled = slaves.status.anyEnabled = true;
      }
      if (slaves.requestedSource == ECMC_DATA_SOURCE_EXTERNAL && ++sourceTicks > 3) {
        slaves.status.allTrajExternal = slaves.status.anyTrajExternal = true;
      }
      assert(machine.lastFaultCode_ == 0 && masters.error == 0);
    }
    assert(!trajectory.getBusy());
    if (guarded) assert(std::abs(physical - .15) < 1e-9);
    else {
      assert(physical < .15 && maxLag > 0);
      if (brakeDelay >= 250) assert(maxLag > .03); // live following-error threshold
    }
    // A subsequent cold start must re-arm the barrier, not inherit readiness.
    machine.stateReset();
    assert(!machine.masterTrajectoryStartAllowed_);
    // Disabled state machine leaves standalone master movement available.
    machine.control_.enable = false;
    machine.execute();
    assert(machine.masterTrajectoryStartAllowed_);
  }
  // No acknowledgement is bounded by the existing preparation timeout.
  ecmcAxisData data;
  ecmcTrajectoryTrapetz trajectory(data, .001);
  ecmcAxisBase axis(data, trajectory);
  ecmcAxisGroup masters, slaves;
  masters.axis = &axis;
  masters.trajectory = &trajectory; masters.data = &data;
  ecmcMasterSlaveStateMachine machine(nullptr, 0, "timeout", .001, &masters, &slaves, 0, 0);
  machine.validationOK_ = true;
  machine.masterPrepareTimeoutS_ = .05;
  data.status_.statusWord_.enable = data.status_.statusWord_.enabled = true;
  trajectory.setEnable(true); trajectory.setAcc(1); trajectory.setDec(1);
  trajectory.setTargetVel(.4); trajectory.setTargetPos(.15); axis.setExecute(true);
  for (int cycle = 0; cycle < 250; ++cycle) {
    assert(axis.tick() == 0);
    machine.execute();
  }
  assert(machine.lastFaultCode_ == ERROR_MST_SLV_SM_PREPARE_MASTER_TIMEOUT);
  assert(!trajectory.getBusy());

  // Drive readiness alone cannot start motion if source acknowledgement fails.
  machine.stateReset();
  machine.masterPrepareTimeoutS_ = 1;
  data.status_.statusWord_.enable = data.status_.statusWord_.enabled = true;
  trajectory.setEnable(true); trajectory.setTargetPos(.15); axis.setExecute(true);
  slaves.status.allEnableCmd = slaves.status.allEnabled = slaves.status.anyEnabled = true;
  slaves.status.allTrajExternal = false;
  slaves.requestedSource = ECMC_DATA_SOURCE_INTERNAL;
  for (int cycle = 0; cycle < 50; ++cycle) {
    assert(axis.tick() == 0);
    machine.execute();
  }
  assert(machine.lastFaultCode_ == ERROR_MST_SLV_SM_SLAVE_TRAJ_SRC_TIMEOUT);

  // A genuine slave interlock remains fatal and is logged before group cleanup.
  machine.stateReset();
  machine.state_ = machine.trackedState_ = ECMC_MST_SLV_STATE_MASTERS;
  data.status_.statusWord_.enable = data.status_.statusWord_.enabled = true;
  masters.otherMasterEnabled = masters.otherMasterEnableCmd = true;
  data.status_.statusWord_.globalBusy = true;
  slaves.status.allEnableCmd = slaves.status.allEnabled = true;
  slaves.status.allTrajExternal = true;
  slaves.status.anyIlocked = true;
  machine.stateMaster();
  assert(machine.lastFaultCode_ == ERROR_AXIS_SLAVED_AXIS_INTERLOCK);
  assert(slaves.interlockLogs == 1 && masters.error == ERROR_AXIS_SLAVED_AXIS_INTERLOCK);

  // Pending starts cancel on STOP, disable, errors, holds and command blocking;
  // returning readiness later must not replay an abandoned command.
  for (int cancellation = 0; cancellation < 5; ++cancellation) {
    ecmcAxisData d;
    ecmcTrajectoryTrapetz t(d, .001);
    ecmcAxisBase a(d, t);
    bool ready = false;
    a.motionStartPermissive_ = &ready;
    d.status_.statusWord_.enable = d.status_.statusWord_.enabled = true;
    t.setEnable(true); t.setAcc(1); t.setDec(1); t.setTargetVel(.4); t.setTargetPos(.15);
    assert(a.setExecute(true) == 0 && a.motionStartPending_ && a.getBusy());
    if (cancellation == 0) a.setExecute(false);
    if (cancellation == 1) a.setEnable(false);
    if (cancellation == 2) a.setErrorID(ERROR_AXIS_SLAVED_AXIS_INTERLOCK);
    if (cancellation == 3) a.startupHeld = true;
    if (cancellation == 4) a.blocked = true;
    a.tick();
    assert(!a.motionStartPending_ && !a.getBusy());
    ready = true; a.errorReset(); a.startupHeld = a.blocked = false;
    for (int i = 0; i < 20; ++i) assert(a.tick() == 0);
    assert(a.executed == 0 && !t.getExecute());
  }

  // Coordinated waiting also works with explicit enable and auto-enable off.
  ecmcAxisData manualData;
  ecmcTrajectoryTrapetz manualTraj(manualData, .001);
  ecmcAxisBase manual(manualData, manualTraj);
  bool manualReady = false;
  manual.motionStartPermissive_ = &manualReady;
  manual.enableAutoEnable_ = false;
  manual.autoEnableTimoutS_ = -1;
  manualData.status_.statusWord_.enable = manualData.status_.statusWord_.enabled = true;
  manualTraj.setEnable(true); manualTraj.setAcc(1); manualTraj.setDec(1);
  manualTraj.setTargetVel(.4); manualTraj.setTargetPos(.15);
  manual.ignoredMotionMessage_ = ECMC_CMD_MOVEABS;
  manual.setExecute(true);
  for (int i = 0; i < 250; ++i) assert(manual.tick() == 0);
  assert(manual.getBusy() && manual.executed == 0);
  assert(manual.ignoredMotionMessage_ == ECMC_CMD_MOVEABS);
  manualReady = true;
  for (int i = 0; i < 2000; ++i) manual.tick();
  assert(std::abs(manualTraj.ecmcTrajectoryBase::getCurrentPosSet() - .15) < 1e-9);
  assert(manual.executed == 1);
  assert(manual.ignoredMotionMessage_ == ECMC_CMD_NOCMD);
  std::cout << "Master/slave startup: real state transitions wait for drive/brake/source acknowledgement; no missed motion, reset/disabled behavior and timeout verified.\n";
}
'''
with tempfile.TemporaryDirectory(prefix='ecmc-master-start-') as directory:
    generated = Path(directory) / 'checks.cpp'
    generated.write_text(preamble + '\n' + methods + '\n' + main)
    executable = Path(directory) / 'checks'
    includes = [repo / 'tests/drive/offline', repo / 'tools/plc-test/offline']
    includes += [sup / name for name in ('motion', 'main', 'com', 'misc', 'ethercat')]
    sources = [repo / 'tests/drive/support.cpp', sup / 'motion/ecmcTrajectoryBase.cpp',
               sup / 'motion/ecmcTrajectoryTrapetz.cpp', sup / 'motion/ecmcAxisData.cpp',
               sup / 'main/ecmcError.cpp']
    command = [os.environ.get('CXX', 'c++'), '-std=c++17', '-O0', '-g0', '-UNDEBUG',
               '-fsanitize=address,undefined', '-fno-sanitize-recover=all']
    command += ['-I' + str(path) for path in includes]
    subprocess.run(command + [str(generated)] + [str(path) for path in sources] + ['-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
