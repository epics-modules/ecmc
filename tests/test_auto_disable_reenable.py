#!/usr/bin/env python3
"""Production enable/auto-disable methods with delayed drive feedback; no IOC."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'devEcmcSup/motion/ecmcAxisBase.cpp').read_text()
def method(signature):
    start = source.index(signature)
    return source[start:source.index('\n}', start) + 2]

program = r'''
#include <cassert>
#define ecmcRtLoggerLogDebug(...) ((void)0)
const int ERROR_AXIS_NOT_ENABLED=1, ERROR_AXIS_SAFETY_IL_ACTIVE=2;
struct Device {
  void initFilter(double) {}
  void setStartPos(double) {}
  void setCurrentPosSet(double) {}
  void setTargetPos(double) {}
  void setEnable(bool) {}
  bool getEnableAtTargetMon() { return true; }
};
class ecmcAxisBase {
public:
  struct {
    struct {
      struct { bool enable=false, enabled=true, attarget=true, busy=false; } statusWord_;
      int axisId=1;
      double sampleTime=.001, currentPositionActual=0;
      double currentPositionSetpoint=0, currentVelocitySetpoint=0;
    } status_;
    struct { struct { bool enableCmd=false, enableDbgPrintout=false; } controlWord_; } control_;
  } data_;
  Device device, *extEncVeloFilter_=&device, *extTrajVeloFilter_=&device;
  Device *mon_=&device, *traj_=&device;
  bool enableAutoResetError_=false, firstEnableDone_=true;
  bool enableAutoDisable_=true, autoDisableAtTgtLtchEnable_=true;
  bool autoDisableAtTargetLatched_=true;
  double autoDisbleTimeCounter_=4, autoDisableAfterS_=3;
  int disables=0;
  void errorReset() {}
  int getErrorID() { return 0; }
  Device *getMon() { return mon_; }
  void setMRStop(int) {}
  int setEnableLocal(bool);
  void autoDisableSM();
  void setEnable(bool value) {
    if (data_.status_.statusWord_.enable == value) return;
    setEnableLocal(value);
    data_.status_.statusWord_.enable=value;
    data_.control_.controlWord_.enableCmd=value;
    if (!value) ++disables;
    // Keep enabled feedback high, as during asynchronous brake closing.
  }
};
'''
program += method('int ecmcAxisBase::setEnableLocal(bool enable)') + '\n'
program += method('void ecmcAxisBase::autoDisableSM()') + '\n'
program += r'''
int main() {
  ecmcAxisBase a;
  a.setEnable(true);
  assert(a.autoDisbleTimeCounter_ == 0);
  assert(!a.autoDisableAtTargetLatched_);
  // The group repeats enable while waiting for drive/brake readiness.
  for (int cycle=0; cycle<1000; ++cycle) {
    a.setEnable(true);
    a.autoDisableSM();
    assert(a.data_.status_.statusWord_.enable && a.disables==0);
  }
  // Repeated enable=1 must not restart the timer and defeat idle shutdown.
  for (int cycle=0; cycle<2100 && !a.disables; ++cycle) {
    a.setEnable(true);
    a.autoDisableSM();
  }
  assert(a.disables==1 && !a.data_.status_.statusWord_.enable);
  // Retry before enabled feedback drops must again get a full idle period.
  a.setEnable(true);
  a.autoDisableSM();
  assert(a.data_.status_.statusWord_.enable && a.disables==1);
  a.data_.status_.statusWord_.busy=true;
  a.autoDisableSM();
  assert(a.autoDisbleTimeCounter_==0);
}
'''
with tempfile.TemporaryDirectory(prefix='ecmc-reenable-') as tmp:
    cpp, exe = Path(tmp)/'test.cpp', Path(tmp)/'test'
    cpp.write_text(program)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall', '-Wextra',
                    '-Werror', str(cpp), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print('Auto-disable: re-enable with delayed feedback, idle timeout and busy reset passed.')
