#!/usr/bin/env python3
"""Exercise the real asyn status setter across fault completion and reset.

Requires a motor source checkout: MOTOR_SOURCE points to motorApp/MotorSrc.
This checks aggregate status/ready suppression, not the full IOC callback queue.
"""
import os
import ast
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
default = root.parent.parent / 'motorECMC_7.2.8.1/motor/motorApp/MotorSrc'
motor = Path(os.environ.get('MOTOR_SOURCE', str(default)))
if not (motor / 'asynMotorAxis.cpp').exists():
    print('SKIP: set MOTOR_SOURCE to the installed motor module MotorSrc directory')
    raise SystemExit(0)
source = (motor / 'asynMotorAxis.cpp').read_text()
start = source.index('asynStatus asynMotorAxis::setIntegerParam(int function, int value)')
setter = source[start:source.index('\n}', start)+2]
record = (motor / 'motorRecord.cc').read_text()
start = record.index('    /* Get motor-now-moving indicator. */')
moving = record[start:record.index('    if (pmr->movn != old_movn)', start)]
ecmc = (root / 'devEcmcSup/motor/ecmcMotorRecordAxis.cpp').read_text()
completion_start = ecmc.index('  // Wait once for an RT sample')
completion = ecmc[completion_start:ecmc.index('  //if(!drvlocal.moveReady)', completion_start)]
error_start = ecmc.index('void ecmcMotorRecordAxis::callParamCallbacksUpdateError()')
error_method = ecmc[error_start:ecmc.index('\nasynStatus ecmcMotorRecordAxis::readEcmcAxisStatusData()', error_start)]
# Reuse the existing compiler stand-ins for error classification, but replace
# its status setter with the actual motor implementation and inspect every
# resulting callback through the actual motor-record Moving calculation.
tree = ast.parse((root / 'tests/test_motor_error_publication.py').read_text())
error_stubs = next(ast.literal_eval(node.value) for node in tree.body
                   if isinstance(node, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == 'stubs' for t in node.targets))
error_stubs = error_stubs[:error_stubs.index('class ecmcMotorRecordAxis')]
program = r'''
#include <cassert>
#include <cstdint>
using asynStatus = int;
using epicsUInt32 = uint32_t;
struct Controller {
  int motorStatusDirection_=0, motorStatusDone_=1, motorStatusProblem_=2;
  int motorStatusHomed_=3, motorStatus_=10;
  int motorFlagsHomeOnLs_=20, motorFlagsRwSoftLimits_=21;
  int motorStatusMoving_=4, ecmcMotorRecordErr_=5, ecmcMotorRecordErrId_=6;
  int params[32] = {};
  int setIntegerParam(int, int key, int value) { params[key]=value; return 0; }
};
class asynMotorAxis {
public:
  Controller controller, *pC_=&controller;
  int axisNo_=6, waitNumPollsBeforeReady_=0, statusChanged_=0;
  struct { uint32_t status=0, flagsValue=0, flagsWritten=0; } status_;
  asynStatus setIntegerParam(int function, int value);
  void done(int value) { setIntegerParam(pC_->motorStatusDone_,value); }
  void problem(int value) { setIntegerParam(pC_->motorStatusProblem_,value); }
  bool recordMoving() {
    struct { struct { bool RA_DONE, RA_PROBLEM; } Bits; } msta = {
      {bool(status_.status & 2),bool(status_.status & 4)}};
    struct { int movn; } instance = {}, *pmr=&instance;
    bool ls_active=false;
    auto motor_fully_stopped_on_ls_activated = [](decltype(pmr),bool) { return false; };
''' + moving + r'''
    return pmr->movn;
  }
};
''' + setter + r'''
'''
program += error_stubs + r'''
class ecmcMotorRecordAxis : public asynMotorAxis {
public:
  struct Word { bool attarget=false, execute=true, enable=true, enabled=true; };
  struct Local {
    bool moveReady=false, ecmcBusy=false, ecmcSummaryInterlock=false;
    unsigned int ecmcMotionCommandExecuteCounter=0;
    bool ecmcSoftLimitInterlock=false, ecmcAtTargetMonEnable=true;
    struct { int errorCode=0; Word statusWord_; } status_;
    int nErrorIdMcu=0, eeAxisError=0, old_eeAxisError=0;
    int eeAxisWarning=0, old_eeAxisWarning=0, nErrorIdEpicsOld=0;
    int nCommandActive=0, nCommandActiveOld=0;
    struct { bool sErrorMessage=false; } dirty;
    char sErrorMessage[256]="fault", cmdErrorMessage[256]="";
    bool illegalInTargetWindow=false;
  } drvlocal;
  void *pPrintOutAsynUser=nullptr;
  const char *modNamEMC="test";
  bool terminated=false;
  unsigned int observedMotionCommandExecuteCounter_=0;
  bool motionCommandExecuteCounterInitialized_=false;
  int callbacks=0;
  bool dataIsSampledAfterNewCmd() { return true; }
  void updateEcmcErrorMsg(const char *) {}
  void callParamCallbacks() {
    ++callbacks;
    if(terminated) assert(!recordMoving());
  }
  void callParamCallbacksUpdateError();
  void completion() {
''' + completion + r'''
    setIntegerParam(pC_->motorStatusMoving_, !drvlocal.moveReady);
    setIntegerParam(pC_->motorStatusDone_, drvlocal.moveReady);
  }
};
''' + error_method + r'''
int main() {
  for(int fault : {0x1460e,0x14417}) {
    ecmcMotorRecordAxis axis;
    axis.done(0);
    axis.drvlocal.ecmcBusy=true;
    axis.completion();
    assert(axis.recordMoving());
    axis.drvlocal.nErrorIdMcu=fault;
    axis.drvlocal.status_.errorCode=fault;
    axis.callParamCallbacksUpdateError();
    axis.drvlocal.ecmcBusy=false;
    axis.drvlocal.status_.statusWord_.execute=false;
    axis.drvlocal.status_.statusWord_.enable=false;
    axis.drvlocal.status_.statusWord_.enabled=false;
    axis.completion();
    axis.terminated=true;
    axis.callParamCallbacks();
    axis.drvlocal.nErrorIdMcu=0;
    axis.drvlocal.status_.errorCode=0;
    // Error callbacks before/after a subsequent completion poll must both
    // preserve terminated state, including when AtTarget remains false.
    axis.callParamCallbacksUpdateError();
    for(int poll=0;poll<5;++poll) {
      axis.completion();
      axis.callParamCallbacksUpdateError();
    }
    assert(axis.callbacks>=7);
    axis.terminated=false;
    ++axis.drvlocal.ecmcMotionCommandExecuteCounter;
    axis.drvlocal.ecmcBusy=true;
    axis.drvlocal.status_.statusWord_=ecmcMotorRecordAxis::Word();
    axis.completion();
    assert(axis.recordMoving());
  }
  // Accepted motor request: the driver explicitly publishes not-done first,
  // consuming the ready-suppression guard before terminal completion.
  for(int wait : {0,1,2}) {
    asynMotorAxis axis;
    axis.waitNumPollsBeforeReady_=wait;
    axis.done(0);
    assert(axis.waitNumPollsBeforeReady_==0 && axis.recordMoving());
    axis.problem(1); // injected drive/encoder fault
    assert(!axis.recordMoving());
    axis.done(1); // confirmed termination, even with AtTarget false
    axis.problem(0); // reset clears Problem
    assert(!axis.recordMoving());
    for(int poll=0;poll<5;++poll) {
      axis.done(1);
      assert(!axis.recordMoving());
    }
    axis.done(0); // fresh request must become pending again
    assert(axis.recordMoving());
    axis.done(1);
    assert(!axis.recordMoving());
  }
  // Negative control: Problem can hide an unset Done bit. Clearing Problem
  // then produces apparent motion without any native command being issued.
  asynMotorAxis stale;
  stale.done(0); stale.problem(1);
  assert(!stale.recordMoving());
  stale.problem(0);
  assert(stale.recordMoving());
  // Another negative control: a requested Done=1 is suppressed in both the
  // parameter library and aggregate MSTA by waitNumPollsBeforeReady_.
  asynMotorAxis suppressed;
  suppressed.waitNumPollsBeforeReady_=1;
  suppressed.problem(1); suppressed.done(1);
  assert(suppressed.controller.params[1]==0);
  suppressed.problem(0);
  assert(suppressed.recordMoving());
}
'''
# Initializer-list loops require this standard header explicitly.
program = '#include <initializer_list>\n' + program
with tempfile.TemporaryDirectory(prefix='motor-fault-reset-') as tmp:
    cpp, binary = Path(tmp)/'status.cpp', Path(tmp)/'status'
    cpp.write_text(program)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall', '-Wextra',
                    '-Werror', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('ECMC completion/error callbacks with real motor setter: drive/encoder fault reset stays done; stale and suppressed Done expose false motion.')
