#!/usr/bin/env python3
"""Compile production DMOV transitions with stand-ins (no EPICS or hardware).

Run: python3 tests/test_motor_record_dmov.py
Tests the command latch and completion portion of poll(), not the full IOC,
motor record processing, command acceptance, or real mutex contention.
Source markers deliberately fail if the extracted regions are reorganized.
"""
# Expectation basis: INFERRED: fresh completion; DOC: sample acknowledgement; COMPAT: callbacks.
# Rationale, sources and open questions: EXPECTATIONS.md
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'devEcmcSup/motor/ecmcMotorRecordAxis.cpp').read_text()


def between(text, start, end):
    begin = text.index(start)
    return text[begin:text.index(end, begin)]


# Extract whole small methods using their top-level closing braces.
methods = ''.join(between(source, signature, '\n}') + '\n}\n' for signature in (
    'void ecmcMotorRecordAxis::newCmd()',
    'bool ecmcMotorRecordAxis::dataIsSampledAfterNewCmd()',
))
completion = between(source, '  // Wait once for an RT sample',
                     '  //if(!drvlocal.moveReady)')
publish = between(source, '  setIntegerParam(pC_->motorStatusMoving_, !drvlocal.moveReady);\n'
                  '  setIntegerParam(pC_->motorStatusDone_,   drvlocal.moveReady);\n\n',
                  '  updateError();')
initialization = between(source, '  ecmcCycleCounterAtNewCmd_ = 0;',
                         '  updateFirstPollDone_ = false;')
# Exercise each accepted-command latch independently, including rearming.
latches = ''
for name in ('move', 'home', 'moveVelocity'):
    command = source[source.index('asynStatus ecmcMotorRecordAxis::' + name + '('):]
    latch = between(command, '    drvlocal.moveReady = false;',
                    '#ifndef motorWaitPollsBeforeReadyString')
    latches += 'void latch_' + name + '() {\n' + latch + '}\n'

stubs = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
int lockDepth = 0;
void *ecmcRTMutex = &lockDepth;
void epicsMutexLock(void *) { assert(lockDepth == 0); ++lockDepth; }
void epicsMutexUnlock(void *) { assert(lockDepth == 1); --lockDepth; }
struct Axis {
  uint32_t cycle = 0;
  int getCycleCounter() { assert(lockDepth == 1); return static_cast<int32_t>(cycle); }
};
struct Controller { int motorStatusMoving_ = 0, motorStatusDone_ = 1; };
class ecmcMotorRecordAxis {
public:
  struct Word { bool execute = true, enable = true, enabled = true, attarget = false; };
  struct Status { uint32_t cycleCounter = 0; int errorCode = 0; Word statusWord_; };
  struct Local {
    Axis *ecmcAxis = nullptr;
    bool moveReady = false, ecmcBusy = false;
    unsigned int ecmcMotionCommandExecuteCounter = 0;
    bool ecmcSummaryInterlock = false, ecmcSoftLimitInterlock = false;
    bool ecmcAtTargetMonEnable = true;
    Status status_;
  } drvlocal;
  Axis live;
  Controller controller;
  Controller *pC_ = &controller;
  uint32_t ecmcCycleCounterAtNewCmd_;
  bool waitingForStatusAfterCommand_;
  unsigned int observedMotionCommandExecuteCounter_;
  bool motionCommandExecuteCounterInitialized_;
  int params[2] = {-1, -1}, callbacks = 0;
  void setIntegerParam(int id, int value) { params[id] = value; }
  void callParamCallbacks() { assert(lockDepth == 0); ++callbacks; }
  void newCmd();
  bool dataIsSampledAfterNewCmd();
  void expect(bool done) {
    pollCompletion();
    assert(drvlocal.moveReady == done);
    assert(params[controller.motorStatusDone_] == done);
    assert(params[controller.motorStatusMoving_] == !done);
  }
'''
implementation = (stubs + 'ecmcMotorRecordAxis() { drvlocal.ecmcAxis = &live;\n'
                  + initialization + '}\n' + latches
                  + 'void pollCompletion() {\n' + completion + publish
                  + '}\n};\n' + methods)

tests = r'''
void testFreshSamples() {
  ecmcMotorRecordAxis axis;
  axis.drvlocal.status_.statusWord_.attarget = true;
  axis.expect(false); // Startup must see the first RT sample.
  axis.drvlocal.status_.cycleCounter = 1;
  axis.expect(true);

  const uint32_t boundaries[] = {42u, 0x7fffffffu, 0xffffffffu};
  for (uint32_t counter : boundaries) {
    axis.live.cycle = counter;
    axis.drvlocal.status_.cycleCounter = counter - 10u; // Cached status is older.
    int callbacks = axis.callbacks;
    axis.newCmd();
    assert(axis.callbacks == callbacks + 1);
    assert(axis.params[1] == 0 && axis.params[0] == 1);
    assert(axis.ecmcCycleCounterAtNewCmd_ == counter); // Must latch LIVE counter.
    assert(axis.waitingForStatusAfterCommand_);
    // poll() reads status under the same lock; model that fresh read here.
    axis.drvlocal.status_.cycleCounter = counter;
    axis.expect(false);
    axis.expect(false); // Repeated polls of the same sample cannot finish.
    axis.drvlocal.status_.cycleCounter = counter + 1u;
    axis.expect(true); // Includes signed boundary and UINT32_MAX -> 0.
    axis.drvlocal.status_.cycleCounter = counter + 0x80000000u;
    axis.expect(true); // Long idle must not resurrect DMOV=0.
    axis.drvlocal.status_.cycleCounter = counter;
    axis.expect(true); // Nor may a complete counter wrap do so.
  }
}

void testCommandRearming() {
  using Latch = void (ecmcMotorRecordAxis::*)();
  const Latch latches[] = {&ecmcMotorRecordAxis::latch_move,
                          &ecmcMotorRecordAxis::latch_home,
                          &ecmcMotorRecordAxis::latch_moveVelocity};
  for (Latch latch : latches) {
    ecmcMotorRecordAxis axis;
    axis.drvlocal.status_.statusWord_.attarget = true;
    axis.drvlocal.status_.cycleCounter = 1;
    axis.expect(true);
    for (uint32_t cycle : {100u, 200u}) {
      axis.live.cycle = cycle;
      epicsMutexLock(ecmcRTMutex); // Enclosing production command holds this lock.
      (axis.*latch)();
      epicsMutexUnlock(ecmcRTMutex);
      assert(!axis.drvlocal.moveReady && axis.waitingForStatusAfterCommand_);
      assert(axis.ecmcCycleCounterAtNewCmd_ == cycle);
      axis.drvlocal.status_.cycleCounter = cycle;
      axis.expect(false);
      axis.drvlocal.status_.cycleCounter = cycle + 1;
      axis.expect(true);
    }
  }
}

void testNativeMotionAfterIdle() {
  // Native/PLC motion bypasses move()/home()/moveVelocity(), so no motor
  // command latch runs. An idle motor must still reflect native Busy.
  for (bool monitor : {false, true}) {
    ecmcMotorRecordAxis axis;
    axis.drvlocal.ecmcAtTargetMonEnable = monitor;
    axis.drvlocal.status_.statusWord_.attarget = true;
    axis.drvlocal.status_.cycleCounter = 1;
    axis.expect(true);
    assert(!axis.waitingForStatusAfterCommand_);
    ++axis.drvlocal.ecmcMotionCommandExecuteCounter;
    axis.drvlocal.ecmcBusy = true;
    axis.drvlocal.status_.cycleCounter = 2;
    axis.expect(false); // Busy wins even with a stale AtTarget indication.
    axis.drvlocal.status_.statusWord_.attarget = false;
    axis.expect(false);
    axis.drvlocal.ecmcBusy = false;
    axis.expect(!monitor); // Preserve normal settling after the native move.
    axis.drvlocal.status_.statusWord_.attarget = true;
    axis.expect(true);
    // A second native search must also clear the completed state.
    ++axis.drvlocal.ecmcMotionCommandExecuteCounter;
    axis.drvlocal.ecmcBusy = true;
    axis.expect(false);
    axis.drvlocal.ecmcBusy = false;
    axis.drvlocal.status_.statusWord_.attarget = false;
    axis.drvlocal.status_.statusWord_.execute = false;
    axis.expect(true); // Native abort completes without AtTarget.
  }
}

void testStaleBusyCannotRestartCompletedMotor() {
  ecmcMotorRecordAxis axis;
  axis.drvlocal.status_.statusWord_.attarget = true;
  axis.drvlocal.status_.cycleCounter = 1;
  axis.expect(true);

  // A delayed Busy sample without a newly executed motion command must not
  // revoke a terminal motor state after STOP.
  axis.drvlocal.ecmcBusy = true;
  axis.expect(true);
  axis.drvlocal.ecmcBusy = false;
  axis.expect(true);

  // Execute can become visible before Busy. Keep it pending, then recognize
  // the motion when Busy arrives instead of losing the native command.
  ++axis.drvlocal.ecmcMotionCommandExecuteCounter;
  axis.expect(true);
  axis.drvlocal.ecmcBusy = true;
  axis.expect(false);
}

void testCompletionReasons() {
  // Each termination reason must work independently, with AtTarget false
  // except for successful arrival. Busy must win even if a reason is present.
  for (int reason = 0; reason < 9; ++reason) {
    ecmcMotorRecordAxis axis;
    axis.live.cycle = 100;
    axis.newCmd();
    switch (reason) {
      case 0: break; // Still settling, monitor enabled: NOT done.
      case 1: axis.drvlocal.status_.statusWord_.attarget = true; break;
      case 2: axis.drvlocal.status_.errorCode = 123; break;
      case 3: axis.drvlocal.ecmcSummaryInterlock = true; break;
      case 4: axis.drvlocal.ecmcSoftLimitInterlock = true; break;
      case 5: axis.drvlocal.status_.statusWord_.execute = false; break;
      case 6: axis.drvlocal.status_.statusWord_.enable = false; break;
      case 7: axis.drvlocal.status_.statusWord_.enabled = false; break;
      case 8: axis.drvlocal.ecmcAtTargetMonEnable = false; break;
    }
    axis.drvlocal.status_.cycleCounter = 100;
    axis.expect(false); // Even terminal status cannot bypass sample gating.
    axis.drvlocal.status_.cycleCounter = 101;
    axis.drvlocal.ecmcBusy = true;
    axis.expect(false);
    axis.drvlocal.ecmcBusy = false;
    axis.expect(reason != 0);
    if (reason == 0) {
      axis.drvlocal.status_.statusWord_.attarget = true;
      axis.expect(true); // Normal move finishes after settling.
    }
    // Completion stays latched even if target/error/interlock indications clear.
    axis.drvlocal.status_.statusWord_ = ecmcMotorRecordAxis::Word{};
    axis.drvlocal.status_.errorCode = 0;
    axis.drvlocal.ecmcSummaryInterlock = false;
    axis.drvlocal.ecmcSoftLimitInterlock = false;
    axis.drvlocal.ecmcAtTargetMonEnable = true;
    axis.expect(true);
  }
}

int main() {
  testFreshSamples();
  testCommandRearming();
  testNativeMotionAfterIdle();
  testStaleBusyCannotRestartCompletedMotor();
  testCompletionReasons();
  puts("Motor-record DMOV: fresh samples, wraparound, idle, command rearming, "
       "native motion after idle, completion, stops, errors, and interlocks passed.");
}
'''

with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / 'motor_record_dmov.cpp'
    exe = Path(directory) / 'motor_record_dmov'
    cpp.write_text('#include <initializer_list>\n' + implementation + tests)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-O2',
                    '-Wall', '-Wextra', '-Werror', str(cpp), '-o', str(exe)],
                   check=True)
    subprocess.run([str(exe)], check=True)
