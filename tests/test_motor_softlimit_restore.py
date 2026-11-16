#!/usr/bin/env python3
"""Exercise production motor startup code with lightweight EPICS/asyn stand-ins."""
# Expectation basis: DOC: restore/units/retry contract; COMPAT: legacy behavior and reporting.
# Rationale, sources and open questions: EXPECTATIONS.md
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'devEcmcSup/motor/ecmcMotorRecordAxis.cpp').read_text()
controller_source = (root / 'devEcmcSup/motor/ecmcMotorRecordController.cpp').read_text()


def method(signature, text=source):
    start = text.index(signature)
    brace = text.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


implementation = method('static bool ecmcMotorRecordSoftLimitEqual(') + '\n'
implementation += '\n'.join(method(signature) for signature in (
    'const char *ecmcMotorRecordAxis::getSoftLimitRestoreStatus()',
    'void ecmcMotorRecordAxis::captureRestoredSoftLimits(',
    'asynStatus ecmcMotorRecordAxis::syncEcmcSoftLimits(bool',
    'asynStatus ecmcMotorRecordAxis::syncMotorSoftLimits(bool',
    'asynStatus ecmcMotorRecordAxis::syncRestoredMotorSoftLimitsToEcmc()',
    'asynStatus ecmcMotorRecordAxis::initializeSoftLimits()',
    'asynStatus ecmcMotorRecordAxis::syncSoftLimitInterfaces(bool updateParams,',
    'asynStatus ecmcMotorRecordAxis::readEcmcAxisStatusData()',
    'asynStatus ecmcMotorRecordAxis::setPosition(double value)',
    'bool ecmcMotorRecordAxis::dataIsSampledAfterNewCmd()',
))
# Run the real poll entry through startup initialization; subsequent motion/status
# reporting is outside this test's scope.
poll = method('asynStatus ecmcMotorRecordAxis::poll(bool *moving)')
implementation += '\n' + poll[:poll.index('  // Wait once for an RT sample')]
implementation += '\n  ++callbacksPublished; (void)moving; (void)timeBefore; return asynSuccess;\n}\n'
implementation += method('void motorRecordInitHook(', controller_source)

stubs = r'''
#include <cassert>
#include <cstring>
#include <math.h>
#include <map>
#include <string>
#include <cstdint>
enum asynStatus { asynSuccess, asynError };
#define LOGINFO(...) ((void)0)
#define LOGERR(...) ((void)0)
#define asynPrint(...) ((void)0)
#define motorWaitPollsBeforeReadyString 1
constexpr int ECMC_DATA_SOURCE_EXTERNAL = 1;
void *ecmcRTMutex = nullptr;
void epicsMutexLock(void *) {}
void epicsMutexUnlock(void *) {}
bool iocReady = false;
int ecmcIocRunning() { return iocReady; }
double ecmcMotorRecordgetNowTimeSecs() { return 0; }
struct ecmcAxisDataStatus { int errorCode = 0; uint32_t cycleCounter = 0; };
constexpr int eeAxisWarningNoWarning = 0;
struct Monitor {
  int *writes = nullptr; bool *fail = nullptr, *failForward = nullptr;
  int restoreSoftLimitPair(double l, double h) {
    ++*writes;
    if (*fail || *failForward) return 1;
    low = l; high = h; return 0;
  }
  double low = -100, high = 100;
  bool lowEnabled = true, highEnabled = true;
  double getSoftLimitBwd() { return low; }
  double getSoftLimitFwd() { return high; }
  bool getEnableSoftLimitBwd() { return lowEnabled; }
  bool getEnableSoftLimitFwd() { return highEnabled; }
  bool getSafetyInterlock() { return false; }
  bool getAtTarget() { return false; }
  bool getEnableAtTargetMon() { return false; }
  bool getSumInterlock() { return false; }
};
struct Axis {
  Monitor monitor;
  bool rtStarted = false, startup = true, validStatus = true;
  ecmcAxisDataStatus status;
  double loadedPosition = 0;
  int positionError = 0;
  uint32_t getCycleCounter() { return status.cycleCounter; }
  int setPosition(double value) {
    if (positionError) return positionError;
    loadedPosition = value; return 0;
  }
  Monitor *getMon() { return &monitor; }
  bool getRealTimeStarted() { return rtStarted; }
  bool getInStartupPhase() { return startup; }
  bool getBusy() { return false; }
  int getPrintDbg() { return 0; }
  bool getSoftLimitInterlock() { return false; }
  int getTrajDataSourceType() { return 0; }
  bool getMRIgnoreDisableStatusCheck() { return false; }
  ecmcAxisDataStatus *getAxisStatusDataPtr() {
    return validStatus ? &status : nullptr;
  }
};
struct Controller {
  enum { motorLowLimit_, motorHighLimit_, motorLowLimitRO_, motorHighLimitRO_,
         ecmcMotorRecordCfgDLLM_En_, ecmcMotorRecordCfgDLLM_,
         ecmcMotorRecordCfgDHLM_En_, ecmcMotorRecordCfgDHLM_,
         motorStop_, motorClosedLoop_, motorStatusDone_ };
  std::map<int, double> values;
  asynStatus getDoubleParam(int, int key, double *value) {
    if (!values.count(key)) return asynError;
    *value = values[key]; return asynSuccess;
  }
  void callParamCallbacks() {}
};
struct asynMotorAxis {
  Controller *controller;
  void setDoubleParam(int key, double value) { controller->values[key] = value; }
  void setIntegerParam(int key, int value) { controller->values[key] = value; }
};
struct ecmcMotorRecordAxis : asynMotorAxis {
  Controller *pC_;
  int axisNo_ = 1, triggstop_ = 0, writes = 0;
  int callbacksPublished = 0;
  bool updateFirstPollDone_ = false, softLimitRestoreErrorLogged_ = false;
  uint32_t ecmcCycleCounterAtNewCmd_ = 0;
  bool waitingForStatusAfterCommand_ = false;
  bool restoredSoftLimitsCaptured_ = false;
  const char *getSoftLimitRestoreStatus();
  bool restoredSoftLimitsReady_ = false;
  double restoredSoftLimitLow_ = 0, restoredSoftLimitHigh_ = 0;
  bool interlockStopActive_ = false, sync = true, failWrite = false;
  bool failForwardWrite = false;
  struct {
    Axis *ecmcAxis;
    bool restoreMotorSoftlimits = true, axisInStartup = false;
    bool ecmcSafetyInterlock = false, ecmcBusy = false, ecmcAtTarget = false;
    bool ecmcAtTargetMonEnable = false, ecmcSummaryInterlock = false;
    bool ecmcSoftLimitInterlock = false, ecmcIgnoreDisableAxisStatus = false;
    int axisPrintDbg = 0, ecmcTrjSrc = 0, nErrorIdMcu = 0, axisId = 1;
    int eeAxisWarning = 0;
    bool moveReady = true;
    ecmcAxisDataStatus status_;
  } drvlocal;
  ecmcMotorRecordAxis(Controller &c, Axis &a) : pC_(&c) {
    controller = &c; drvlocal.ecmcAxis = &a;
    a.monitor.writes = &writes; a.monitor.fail = &failWrite; a.monitor.failForward = &failForwardWrite;
  }
  void publishStartupDiagnostics() {} // Publication is tested separately.
  bool isSoftLimitSyncEnabled() { return sync; }
  bool interlockStopRisingEdge(bool) { return false; }
  void updateError() {}
  void callParamCallbacks() {}
  int applyMotorSoftLimitChange(bool forward, double value) {
    ++writes;
    if (failWrite || (forward && failForwardWrite)) return 1;
    auto &mon = drvlocal.ecmcAxis->monitor;
    (forward ? mon.high : mon.low) = value;
    return 0;
  }
  asynStatus syncEcmcSoftLimits(bool, bool);
  asynStatus syncMotorSoftLimits(bool, bool);
  asynStatus syncRestoredMotorSoftLimitsToEcmc();
  void captureRestoredSoftLimits(double, double, double);
  asynStatus initializeSoftLimits();
  asynStatus syncSoftLimitInterfaces(bool update, bool seed = false);
  asynStatus readEcmcAxisStatusData();
  asynStatus poll(bool *);
  asynStatus setPosition(double);
  bool dataIsSampledAfterNewCmd();
};

// Database/asyn plumbing for exercising the actual init hook on two motor
// records. The second belongs to another driver and must remain untouched.
enum initHookState { initHookAfterInitDevSup, initHookAfterIocBuilt, otherHook };
constexpr unsigned MF_DRIVER_USES_EGU = 8;
constexpr int INST_IO = 1;
struct DBLINK { int type = INST_IO; const char *port = "ECMC"; };
struct motorRecord {
  DBLINK out;
  unsigned mflg = 0;
  double dllm = -100, dhlm = 150, mres = 0.10986328125;
};
motorRecord records[2];
struct DBENTRY { int index = 0; };
struct DBADDR { motorRecord *precord = nullptr; };
void *pdbbase = nullptr;
void dbInitEntry(void *, DBENTRY *) {}
void dbFinishEntry(DBENTRY *) {}
int dbFindRecordType(DBENTRY *, const char *) { return 0; }
long dbFirstRecord(DBENTRY *e) { e->index = 0; return 0; }
long dbNextRecord(DBENTRY *e) { return ++e->index >= 2; }
const char *dbGetRecordName(DBENTRY *e) { return e->index ? "other" : "ecmc"; }
int dbNameToAddr(const char *name, DBADDR *a) {
  a->precord = &records[std::string(name) == "other" ? 1 : 0]; return 0;
}
bool recordLocked = false;
void dbScanLock(motorRecord *) { assert(!recordLocked); recordLocked = true; }
void dbScanUnlock(motorRecord *) { assert(recordLocked); recordLocked = false; }
struct ecmcMotorRecordController {
  ecmcMotorRecordAxis *axis = nullptr;
  bool locked = false;
  void bindStartupMotorRecord(int n, motorRecord *record) {
    assert(locked && n == 1 && record == &records[0]);
  }
  void lock() { assert(recordLocked && !locked); locked = true; }
  void unlock() { assert(locked); locked = false; }
  ecmcMotorRecordAxis *getAxis(int n) { assert(locked && n == 1); return axis; }
} hookController;
template <typename T> T *findDerivedAsynPortDriver(const char *port) {
  return std::string(port) == "ECMC" ? &hookController : nullptr;
}
struct asynUser {};
struct Manager {
  asynUser user;
  asynUser *createAsynUser(void *, void *) { return &user; }
  void freeAsynUser(asynUser *) {}
} manager;
Manager *pasynManager = &manager;
struct Utils {
  asynStatus parseLink(asynUser *, DBLINK *link, char **port, int *axis, char **) {
    *port = const_cast<char *>(link->port); *axis = 1; return asynSuccess;
  }
  void parseLinkFree(asynUser *, char **, char **) {}
} utils;
Utils *pasynEpicsUtils = &utils;
'''

tests = r'''
int main() {
  records[1].out.port = "OTHER_DRIVER";
  motorRecordInitHook(otherHook);
  assert(!records[0].mflg && !records[1].mflg);
  motorRecordInitHook(initHookAfterInitDevSup);
  assert(records[0].mflg & MF_DRIVER_USES_EGU);
  assert(!records[1].mflg);

  Controller c;
  Axis a;
  ecmcMotorRecordAxis m(c, a);
  assert(std::string(m.getSoftLimitRestoreStatus()) == "waiting for IOC running");
  iocReady = true;
  assert(std::string(m.getSoftLimitRestoreStatus()) == "waiting for post-autosave snapshot");
  m.captureRestoredSoftLimits(20, 10, 1);
  assert(std::string(m.getSoftLimitRestoreStatus()).find("invalid snapshot") == 0);
  m.restoredSoftLimitsCaptured_ = false;
  iocReady = false;
  bool moving = false;
  // Constructor readback may refresh ECMC PVs, but must not seed motor limits.
  assert(m.syncSoftLimitInterfaces(true, true) == asynSuccess);
  assert(c.values[Controller::ecmcMotorRecordCfgDHLM_] == 100);
  assert(!c.values.count(Controller::motorHighLimit_));
  assert(!c.values.count(Controller::motorHighLimitRO_));

  // Before RT starts, even IOC-ready polls must not consume restoration.
  iocReady = true;
  assert(m.poll(&moving) == asynSuccess);
  assert(m.drvlocal.axisInStartup && !m.updateFirstPollDone_ && !m.writes);
  a.rtStarted = true;
  assert(m.poll(&moving) == asynSuccess);
  assert(!m.updateFirstPollDone_ && !m.writes);

  // Gate disabled/axis already started: still wait for IOC/autosave pass 1.
  a.startup = false;
  iocReady = false;
  c.values[Controller::motorLowLimit_] = -100;
  c.values[Controller::motorHighLimit_] = 100;
  assert(m.poll(&moving) == asynSuccess);
  assert(!m.updateFirstPollDone_ && !m.writes);
  assert(m.callbacksPublished == 1); // Restore cannot starve motor status.
  c.values[Controller::motorHighLimit_] = 150; // Actual user's saved value.
  iocReady = true;

  // Invalid status, disabled sync, and missing parameters must all retry.
  a.validStatus = false;
  assert(m.poll(&moving) == asynError);
  assert(!m.updateFirstPollDone_ && !m.writes);
  a.validStatus = true;
  m.sync = false;
  assert(m.poll(&moving) == asynSuccess);
  assert(!m.updateFirstPollDone_ && !m.writes);
  m.sync = true;
  c.values.erase(Controller::motorHighLimit_);
  assert(m.poll(&moving) == asynSuccess);
  assert(!m.updateFirstPollDone_ && !m.writes);
  // Command caches may be absent or contain raw values from early motor writes.
  // Only the canonical post-autosave dial limits are authoritative.
  c.values[Controller::motorLowLimit_] = -910.222222222;
  c.values[Controller::motorHighLimit_] = 1365.333333333;
  hookController.axis = &m;
  motorRecordInitHook(initHookAfterIocBuilt);
  assert(m.restoredSoftLimitsReady_ && !hookController.locked && !recordLocked);
  m.failWrite = true;
  assert(m.poll(&moving) == asynSuccess);
  assert(!m.updateFirstPollDone_);
  m.syncSoftLimitInterfaces(true, true);
  assert(c.values[Controller::motorHighLimit_] == 1365.333333333);
  assert(!c.values.count(Controller::motorHighLimitRO_));

  // Successful retry publishes the restored pair and enables normal sync.
  m.failWrite = false;
  m.failForwardWrite = true;
  assert(m.poll(&moving) == asynSuccess);
  assert(!m.updateFirstPollDone_);
  assert(c.values[Controller::motorHighLimit_] == 1365.333333333);
  m.failForwardWrite = false;
  assert(m.poll(&moving) == asynSuccess);
  assert(m.updateFirstPollDone_);
  assert(std::string(m.getSoftLimitRestoreStatus()) == "complete");
  assert(a.monitor.low == -100 && a.monitor.high == 150);
  assert(c.values[Controller::motorHighLimitRO_] == 150);
  int writes = m.writes;
  c.values[Controller::motorHighLimit_] = 175;
  assert(m.poll(&moving) == asynSuccess);
  assert(m.writes == writes); // Startup restore does not run again.
  a.monitor.high = 180;
  m.syncSoftLimitInterfaces(true);
  assert(c.values[Controller::motorHighLimitRO_] == 180);

  // Loading a coordinate is allowed during startup, but its immediate cached
  // readback must not acknowledge startup before the RT thread samples it.
  a.status.cycleCounter = 42;
  m.drvlocal.status_.cycleCounter = 42;
  assert(m.setPosition(5) == asynSuccess && a.loadedPosition == 5);
  assert(m.waitingForStatusAfterCommand_ && !m.drvlocal.moveReady);
  assert(c.values[Controller::motorStatusDone_] == 0);
  assert(!m.dataIsSampledAfterNewCmd());
  m.drvlocal.status_.cycleCounter = 43;
  assert(m.dataIsSampledAfterNewCmd());
  assert(!m.waitingForStatusAfterCommand_);
  a.positionError = 123;
  assert(m.setPosition(8) == asynError && a.loadedPosition == 5);
  assert(!m.waitingForStatusAfterCommand_);

  // Resolution magnitude never rescales dial limits; only its sign changes
  // the raw EGU direction. Invalid snapshots cannot release restoration.
  for (double resolution : {0.10986328125, 1.0, 2.5, -0.10986328125}) {
    Controller scaleC;
    Axis scaleA;
    ecmcMotorRecordAxis scale(scaleC, scaleA);
    scaleA.rtStarted = true;
    scaleA.startup = false;
    scale.captureRestoredSoftLimits(-100, 150, 0);
    scale.poll(&moving);
    assert(!scale.updateFirstPollDone_ && !scale.writes);
    scale.captureRestoredSoftLimits(-100, 150, resolution);
    scale.poll(&moving);
    assert(scale.updateFirstPollDone_);
    assert(scaleA.monitor.low == (resolution < 0 ? -150 : -100));
    assert(scaleA.monitor.high == (resolution < 0 ? 100 : 150));
  }

  // Legacy behavior without restore still seeds YAML/controller limits.
  Controller legacyC;
  Axis legacyA;
  ecmcMotorRecordAxis legacy(legacyC, legacyA);
  legacy.drvlocal.restoreMotorSoftlimits = false;
  legacyA.rtStarted = true;
  legacyA.startup = false;
  iocReady = false;
  assert(legacy.poll(&moving) == asynSuccess);
  assert(legacy.updateFirstPollDone_ && !legacy.writes);
  assert(legacyC.values[Controller::motorHighLimit_] == 100);
  assert(legacyC.values[Controller::motorLowLimit_] == -100);
}
'''

with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / 'softlimit_restore.cpp'
    exe = Path(directory) / 'softlimit_restore'
    cpp.write_text(stubs + implementation + tests)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O2',
                    '-Wall', '-Wextra', '-Werror', str(cpp), '-o', str(exe)],
                   check=True)
    subprocess.run([str(exe)], check=True)
print('Motor soft-limit restore: startup ordering, saved values, retries, and legacy checks passed.')
