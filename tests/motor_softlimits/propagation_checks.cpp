// Real driver methods + real monitor; no fake motor-record limit enforcement.
#include "ecmcMonitor.h"
#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
using std::isfinite;
static std::string context;
static int scenarios = 0;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(context + ": line " + \
  std::to_string(__LINE__) + ": " #x); } while (0)
#undef LOGINFO
#undef LOGERR
#define LOGINFO(...) ((void)0)
#define LOGERR(...) ((void)0)
#define asynPrint(...) ((void)0)
#define motorFlagsRwSoftLimitsString "MOTOR_FLAGS_RW_SOFT_LIMITS"
static int lockDepth = 0;
static int mutexToken;
void *ecmcRTMutex = &mutexToken;
static void epicsMutexLock(void *) { CHECK(lockDepth == 0); ++lockDepth; }
static void epicsMutexUnlock(void *) { CHECK(lockDepth == 1); --lockDepth; }
static bool iocReady = true;
static bool ecmcIocRunning() { return iocReady; }

// Capture the parameter boundary, not EPICS field processing or device support.
struct Controller {
  enum { motorFlagsRwSoftLimits_, motorLowLimit_, motorHighLimit_,
    motorLowLimitRO_, motorHighLimitRO_, ecmcMotorRecordCfgDLLM_,
    ecmcMotorRecordCfgDHLM_, ecmcMotorRecordCfgDLLM_En_, ecmcMotorRecordCfgDHLM_En_ };
  std::map<int, double> values;
  std::map<int, int> doubleWrites;
  int callbacks = 0;
  asynStatus getIntegerParam(int axis, int key, int *value) {
    CHECK(axis == 1); *value = static_cast<int>(values.at(key)); return asynSuccess;
  }
  asynStatus getDoubleParam(int axis, int key, double *value) {
    CHECK(axis == 1);
    auto entry = values.find(key);
    if (entry == values.end()) return asynError;
    *value = entry->second;
    return asynSuccess;
  }
  asynStatus setIntegerParam(int axis, int key, int value) {
    CHECK(axis == 1); values[key] = value; return asynSuccess;
  }
  asynStatus setDoubleParam(int axis, int key, double value) {
    CHECK(axis == 1); values[key] = value; ++doubleWrites[key]; return asynSuccess;
  }
  void callParamCallbacks() { CHECK(lockDepth == 0); ++callbacks; }
};
struct asynMotorAxis {
  Controller *controller;
  int highCalls = 0, lowCalls = 0;
  double forwardedHigh = 0, forwardedLow = 0;
  asynStatus baseResult = asynSuccess;
  asynStatus setIntegerParam(int key, int value) {
    return controller->setIntegerParam(1, key, value);
  }
  asynStatus setDoubleParam(int key, double value) {
    return controller->setDoubleParam(1, key, value);
  }
  asynStatus setHighLimit(double value) {
    CHECK(lockDepth == 0); ++highCalls; forwardedHigh = value; return baseResult;
  }
  asynStatus setLowLimit(double value) {
    CHECK(lockDepth == 0); ++lowCalls; forwardedLow = value; return baseResult;
  }
};
struct Axis {
  ecmcMonitor *monitor;
  ecmcMonitor *getMon() { return monitor; }
};
struct ecmcMotorRecordAxis : asynMotorAxis {
  Controller *pC_;
  int axisNo_ = 1;
  bool updateFirstPollDone_ = true, softLimitRestoreErrorLogged_ = false;
  bool restoredSoftLimitsCaptured_ = false, restoredSoftLimitsReady_ = false;
  double restoredSoftLimitLow_ = 0, restoredSoftLimitHigh_ = 0;
  struct { Axis *ecmcAxis; bool restoreMotorSoftlimits = false;
    bool syncEcmcMrSoftlimits = false; } drvlocal;
  ecmcMotorRecordAxis(Controller &c, Axis &axis) : pC_(&c) {
    controller = &c; drvlocal.ecmcAxis = &axis;
  }
  void callParamCallbacks() { pC_->callParamCallbacks(); }
  int applyMotorSoftLimitChange(bool, double);
  bool isSoftLimitSyncEnabled();
  bool softLimitValueMatches(bool, double);
  asynStatus updateSoftLimitSyncState(bool, bool *);
  asynStatus syncEcmcSoftLimits(bool, bool);
  asynStatus syncMotorSoftLimits(bool, bool);
  asynStatus syncSoftLimitInterfaces(bool, bool = false);
  asynStatus readBackSoftLimits(bool);
  const char *getSoftLimitRestoreStatus();
  void captureRestoredSoftLimits(double, double, double);
  asynStatus syncRestoredMotorSoftLimitsToEcmc();
  asynStatus initializeSoftLimits();
  asynStatus setHighLimit(double);
  asynStatus setLowLimit(double);
};
#include "motor_limit_methods.h"

struct Bench {
  ecmcAxisData data;
  std::array<ecmcEncoder *, ECMC_MAX_ENCODERS> encoders{};
  std::unique_ptr<ecmcMonitor> mon;
  Controller controller;
  Axis axis;
  ecmcMotorRecordAxis motor;
  Bench() : axis{nullptr}, motor(controller, axis) {
    ++scenarios;
    data.status_.sampleTime = 0.001;
    data.status_.axisId = 1;
    data.status_.command = ECMC_CMD_MOVEABS;
    data.status_.statusWord_.instartup = false;
    mon = std::make_unique<ecmcMonitor>(data, encoders.data());
    axis.monitor = mon.get();
    mon->setLimitSwitchBwdPLCOverride(true);
    mon->setLimitSwitchFwdPLCOverride(true);
    mon->setLimitSwitchBwdPLCOverrideValue(true);
    mon->setLimitSwitchFwdPLCOverrideValue(true);
    mon->setEnableLagMon(false); mon->setEnableAtTargetMon(false);
    mon->setEnableStallMon(false); mon->setEnableCheckEncsDiff(false);
    mon->setEnable(true);
    CHECK(mon->validate() == 0);
    controller.values[Controller::motorFlagsRwSoftLimits_] = 0;
  }
  void ecmcLimits(double low, double high, bool bwd = true, bool fwd = true) {
    CHECK(mon->restoreSoftLimitPair(low, high) == 0);
    CHECK(mon->setEnableSoftLimitBwd(bwd) == 0);
    CHECK(mon->setEnableSoftLimitFwd(fwd) == 0);
  }
  void motorFields(double low, double high) {
    // Inputs supplied by the record/device-support boundary; no conversion here.
    controller.values[Controller::motorLowLimit_] = low;
    controller.values[Controller::motorHighLimit_] = high;
    controller.values[Controller::motorLowLimitRO_] = low;
    controller.values[Controller::motorHighLimitRO_] = high;
  }
  void checkMotor(double low, double high) {
    CHECK(controller.values.at(Controller::motorLowLimit_) == low);
    CHECK(controller.values.at(Controller::motorHighLimit_) == high);
    CHECK(controller.values.at(Controller::motorLowLimitRO_) == low);
    CHECK(controller.values.at(Controller::motorHighLimitRO_) == high);
  }
  void checkEcmc(double low, double high, bool bwd = true, bool fwd = true) {
    CHECK(mon->getSoftLimitBwd() == low && mon->getSoftLimitFwd() == high);
    CHECK(mon->getEnableSoftLimitBwd() == bwd && mon->getEnableSoftLimitFwd() == fwd);
    CHECK(lockDepth == 0);
  }
  void pollLimits() {
    // Same limit calls as poll(), without unrelated status/motor-record logic.
    bool rising = false;
    CHECK(motor.updateSoftLimitSyncState(false, &rising) == asynSuccess);
    if (!rising) CHECK(motor.syncSoftLimitInterfaces(false) == asynSuccess);
  }
  void monitorTarget(double target, bool bwd, bool fwd) {
    // Test the actual ECMC enforcement layer, without modeling motorRecord.c.
    data.control_.controlWord_.enableCmd = true;
    data.status_.statusWord_.enabled = true;
    data.statusOld_.statusWord_.enabled = true;
    data.status_.currentPositionSetpoint = 0;
    data.status_.currentTargetPosition = target;
    mon->readEntries(); mon->execute();
    CHECK(data.interlocks_.bwdSoftLimitInterlock == bwd);
    CHECK(data.interlocks_.fwdSoftLimitInterlock == fwd);
  }
};

static void independentLayers() {
  // Sync off keeps the writable layers independent, while enabled ECMC limits
  // remain the motor record's maximum permitted range.
  for (int scenario : {1, 2, 3}) {
    context = "no-sync scenario " + std::to_string(scenario);
    Bench b;
    const bool ecmcOn = scenario != 1;
    b.ecmcLimits(-10, 10, ecmcOn, ecmcOn);
    b.motorFields(scenario == 2 ? 0 : -4, scenario == 2 ? 0 : 6);
    for (int i = 0; i < 3; ++i) b.pollLimits();
    CHECK(b.controller.values.at(Controller::motorLowLimit_) ==
          (scenario == 2 ? 0 : -4));
    CHECK(b.controller.values.at(Controller::motorHighLimit_) ==
          (scenario == 2 ? 0 : 6));
    CHECK(b.controller.values.at(Controller::motorLowLimitRO_) ==
          (ecmcOn && scenario != 2 ? -10 : 0));
    CHECK(b.controller.values.at(Controller::motorHighLimitRO_) ==
          (ecmcOn && scenario != 2 ? 10 : 0));
    b.checkEcmc(-10, 10, ecmcOn, ecmcOn);
    CHECK(b.controller.values.at(Controller::ecmcMotorRecordCfgDLLM_) == -10);
    CHECK(b.controller.values.at(Controller::ecmcMotorRecordCfgDHLM_) == 10);
    // Driver receives an edit even when the record's own limits are disabled.
    CHECK(b.motor.setHighLimit(20) == asynSuccess);
    CHECK(b.motor.setLowLimit(-20) == asynSuccess);
    CHECK(b.motor.highCalls == 1 && b.motor.forwardedHigh == 20);
    CHECK(b.motor.lowCalls == 1 && b.motor.forwardedLow == -20);
    b.checkEcmc(-10, 10, ecmcOn, ecmcOn);
    b.pollLimits();
    CHECK(b.controller.values.at(Controller::motorLowLimit_) ==
          (scenario == 2 ? 0 : -4));
    CHECK(b.controller.values.at(Controller::motorHighLimit_) ==
          (scenario == 2 ? 0 : 6));
    b.monitorTarget(12, false, ecmcOn);
    b.monitorTarget(-12, ecmcOn, false);
    b.monitorTarget(2, false, false);
    // ECMC-side edits remain independent too, and config readbacks still update.
    b.ecmcLimits(-8, 9, ecmcOn, ecmcOn); b.pollLimits();
    CHECK(b.controller.values.at(Controller::motorLowLimit_) ==
          (scenario == 2 ? 0 : -4));
    CHECK(b.controller.values.at(Controller::motorHighLimit_) ==
          (scenario == 2 ? 0 : 6));
    CHECK(b.controller.values.at(Controller::motorLowLimitRO_) ==
          (ecmcOn && scenario != 2 ? -8 : 0));
    CHECK(b.controller.values.at(Controller::motorHighLimitRO_) ==
          (ecmcOn && scenario != 2 ? 9 : 0));
    CHECK(b.controller.values.at(Controller::ecmcMotorRecordCfgDLLM_) == -8);
    CHECK(b.controller.values.at(Controller::ecmcMotorRecordCfgDHLM_) == 9);
  }
  context = "no sync constrains motor range without changing ECMC";
  Bench b; b.ecmcLimits(-10, 10); b.motorFields(-20, 20);
  CHECK(b.motor.setHighLimit(20) == asynSuccess);
  CHECK(b.motor.setLowLimit(-20) == asynSuccess);
  b.pollLimits();
  CHECK(b.controller.values.at(Controller::motorLowLimit_) == -20);
  CHECK(b.controller.values.at(Controller::motorHighLimit_) == 20);
  CHECK(b.controller.values.at(Controller::motorLowLimitRO_) == -10);
  CHECK(b.controller.values.at(Controller::motorHighLimitRO_) == 10);
  b.checkEcmc(-10, 10);
  b.monitorTarget(15, false, true); b.monitorTarget(-15, true, false);

  context = "unchanged no-sync range is not republished";
  Bench stable; stable.ecmcLimits(-10, 10); stable.motorFields(-4, 6);
  stable.pollLimits();
  const int lowWrites = stable.controller.doubleWrites[Controller::motorLowLimitRO_];
  const int highWrites = stable.controller.doubleWrites[Controller::motorHighLimitRO_];
  stable.pollLimits(); stable.pollLimits();
  CHECK(stable.controller.doubleWrites[Controller::motorLowLimitRO_] == lowWrites);
  CHECK(stable.controller.doubleWrites[Controller::motorHighLimitRO_] == highWrites);
}

static void syncedLayers() {
  for (bool bwd : {false, true}) for (bool fwd : {false, true}) {
    context = "sync runtime/readback directional enables";
    Bench b; b.ecmcLimits(-10, 10, bwd, fwd); b.motorFields(-4, 6);
    b.controller.values[Controller::motorFlagsRwSoftLimits_] = 1;
    b.pollLimits();
    // COMPAT: regular readback changes RO parameters, not writable commands.
    CHECK(b.controller.values.at(Controller::motorLowLimit_) == -4);
    CHECK(b.controller.values.at(Controller::motorHighLimit_) == 6);
    CHECK(b.controller.values.at(Controller::motorLowLimitRO_) == ((bwd || fwd) ? -10 : 0));
    CHECK(b.controller.values.at(Controller::motorHighLimitRO_) == ((bwd || fwd) ? 10 : 0));
    CHECK(b.motor.setHighLimit(7) == asynSuccess);
    CHECK(b.motor.setLowLimit(-7) == asynSuccess);
    b.checkEcmc(-7, 7, bwd, fwd);
    // COMPAT: an already-applied value is forwarded without changing ECMC.
    // Individual endpoint setters are not the validated restore-pair API.
    CHECK(b.motor.setHighLimit(7) == asynSuccess);
    b.checkEcmc(-7, 7, bwd, fwd);
    b.controller.values[Controller::motorFlagsRwSoftLimits_] = 0;
    b.pollLimits();
    CHECK(b.motor.setHighLimit(30) == asynSuccess);
    b.checkEcmc(-7, 7, bwd, fwd);
  }
  context = "sync zero sentinel and re-enable";
  Bench b; b.ecmcLimits(-10, 10); b.motorFields(-10, 10);
  b.controller.values[Controller::motorFlagsRwSoftLimits_] = 1;
  CHECK(b.motor.setLowLimit(0) == asynSuccess);
  CHECK(b.motor.setHighLimit(0) == asynSuccess);
  b.pollLimits(); b.checkEcmc(0, 0, false, false);
  CHECK(b.motor.setHighLimit(4) == asynSuccess);
  b.checkEcmc(0, 4); // leaving the zero pair enables both limits
}

static void startupPolicies() {
  for (bool sync : {false, true}) {
    context = "COMPAT startup seed sync=" + std::to_string(sync);
    Bench b; b.ecmcLimits(-10, 10); b.motorFields(-4, 6);
    b.motor.updateFirstPollDone_ = false;
    b.controller.values[Controller::motorFlagsRwSoftLimits_] = sync;
    CHECK(b.motor.initializeSoftLimits() == asynSuccess);
    // COMPAT: legacy initial seed forces writable motor values even with sync
    // off, and enabled ECMC limits always constrain the motor-record range.
    CHECK(b.controller.values.at(Controller::motorLowLimit_) == -10);
    CHECK(b.controller.values.at(Controller::motorHighLimit_) == 10);
    CHECK(b.controller.values.at(Controller::motorLowLimitRO_) == -10);
    CHECK(b.controller.values.at(Controller::motorHighLimitRO_) == 10);
    b.checkEcmc(-10, 10);
    CHECK(b.motor.updateFirstPollDone_);
    b.motorFields(-3, 5);
    CHECK(b.motor.initializeSoftLimits() == asynSuccess);
    b.checkMotor(-3, 5); // startup seed runs once
  }
  context = "sync off retains the ECMC read-only dial range";
  Bench b; b.ecmcLimits(-10, 10); b.motorFields(-4, 6);
  b.controller.values[Controller::motorFlagsRwSoftLimits_] = 1;
  b.pollLimits();
  CHECK(b.controller.values.at(Controller::motorLowLimitRO_) == -10);
  CHECK(b.controller.values.at(Controller::motorHighLimitRO_) == 10);
  b.controller.values[Controller::motorFlagsRwSoftLimits_] = 0;
  b.pollLimits();
  CHECK(b.controller.values.at(Controller::motorLowLimitRO_) == -10);
  CHECK(b.controller.values.at(Controller::motorHighLimitRO_) == 10);
  CHECK(b.controller.values.at(Controller::motorLowLimit_) == -4);
  CHECK(b.controller.values.at(Controller::motorHighLimit_) == 6);
  b.checkEcmc(-10, 10);
  context = "restore requested while sync disabled stays pending";
  {
    Bench b; b.ecmcLimits(-10, 10); b.motorFields(-4, 6);
    b.motor.updateFirstPollDone_ = false;
    b.motor.drvlocal.restoreMotorSoftlimits = true;
    b.motor.captureRestoredSoftLimits(-4, 6, 1);
    CHECK(b.motor.initializeSoftLimits() == asynError);
    CHECK(!b.motor.updateFirstPollDone_);
    b.checkEcmc(-10, 10); b.checkMotor(-4, 6);
    b.controller.values[Controller::motorFlagsRwSoftLimits_] = 1;
    CHECK(b.motor.initializeSoftLimits() == asynSuccess);
    b.checkEcmc(-4, 6);
  }
}

#include "coordinate_checks.h"

int main() {
  try {
    independentLayers(); syncedLayers(); startupPolicies(); coordinateBoundary();
    std::cout << "Motor/ECMC soft limits: " << scenarios << " propagation scenarios passed; "
      "see the separate OFF/DIR processing result above.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
