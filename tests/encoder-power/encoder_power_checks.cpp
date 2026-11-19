// Production encoder/axis/trajectory stepping with an emulated relay and BiSS
// sample. No IOC, motor record, EtherCAT master, or realtime thread is created.
#include "ecmcAxisVirt.h"
#include "ecmcDataItem.h"
#include "ecmcErrorsList.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>

asynUser *pPrintOutAsynUser = nullptr;
unsigned int debug_print_flags = 0, die_on_error_flags = 0;
void ecmcRtLoggerLogError(const char *, ...) {}

static const char *scenario = "";
#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(scenario)+":"+std::to_string(__LINE__)+": " #x); } while(0)

struct PowerTestAxis : ecmcAxisVirt {
  using ecmcAxisVirt::ecmcAxisVirt;
  ecmcAxisData *dataForTest() { return &data_; }
  bool feedbackAvailable() { return encoderPowerFeedbackReady(); }
};

struct Rig {
  ecmcAsynPortDriver port;
  std::unique_ptr<PowerTestAxis> axis;
  uint32_t raw=10000, ready=0, alarm=1, relay=0, offRaw=0;
  uint32_t rawBias=0, counter=0, counterReady=1;
  int sourceShift=0;
  bool withBackup=false;
  int healthy=1, counterHealthy=1;
  double physical=10, supplySeconds=0;
  bool activeHigh, invertedReady, neverReady=false, lostReady=false;
  bool withReady;
  ecmcDataItem position{"position"}, status{"ready"}, alarmItem{"alarm"}, output{"relay"};
  ecmcDataItem counterItem{"counter"}, counterStatus{"counterReady"};
  Rig(bool high=true, bool invert=false, bool readyLink=true, bool backup=false, bool startRuntime=true)
    : activeHigh(high), invertedReady(invert), withReady(readyLink) {
    relay=high ? 0 : 1;
    axis=std::make_unique<PowerTestAxis>(&port,1,.001,ECMC_TRAPETZ);
    bind(position,raw,ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION,false);
    bind(output,relay,ECMC_ENCODER_ENTRY_INDEX_POWER,true,0);
    if(withReady) bind(status,ready,ECMC_ENCODER_ENTRY_INDEX_READY,false,0);
    if(withReady) bind(alarmItem,alarm,ECMC_ENCODER_ENTRY_INDEX_ALARM_0,false,0);
    auto *enc=axis->getPrimEnc();
    CHECK(enc->setBits(32)==0); CHECK(enc->setAbsBits(26)==0);
    CHECK(enc->setType(ECMC_ENCODER_TYPE_ABSOLUTE)==0);
    CHECK(enc->setScaleNum(1)==0); CHECK(enc->setScaleDenom(1000)==0);
    CHECK(enc->setInvHwReady(invert)==0);
    CHECK(enc->setPowerPolarity(high)==0);
    CHECK(enc->setPowerDelay(.05)==0);
    CHECK(enc->setPowerTimeout(.2)==0);
    CHECK(enc->setPowerEnable(true)==0);
    CHECK(axis->setEncDataSourceType(ECMC_DATA_SOURCE_INTERNAL)==0);
    axis->setAllowMotionFunctions(true,true,true);
    axis->setAcc(1); axis->setDec(1); axis->setTargetVel(.2);
    auto *mon=axis->getMon();
    mon->setLimitSwitchBwdPLCOverride(true); mon->setLimitSwitchFwdPLCOverride(true);
    mon->setHomeSwitchPLCOverride(true);
    mon->setLimitSwitchBwdPLCOverrideValue(true); mon->setLimitSwitchFwdPLCOverrideValue(true);
    mon->setHomeSwitchPLCOverrideValue(true);
    mon->setEnableLagMon(false); mon->setEnableAtTargetMon(true);
    mon->setAtTargetTol(.005); mon->setAtTargetTime(2);
    CHECK(axis->setAutoEnableTimeout(.02)==0);
    CHECK(axis->setEnableAutoEnable(true)==0);
    CHECK(axis->validate()==0);
    if(backup) {
      withBackup=true;
      addCounter();
      CHECK(axis->setEncoderPowerBackup(1,2,.01)==0);
      CHECK(axis->validate()==0);
    }
    if (startRuntime) CHECK(axis->setRealTimeStarted(true)==0);
  }
  void addCounter() {
    CHECK(axis->addEncoder()==0);
    int error=0;
    auto *counterEnc=axis->getEnc(1,&error); CHECK(counterEnc && !error);
    ecmcDataAccess access;
    counterItem.setEcmcDataPointer(reinterpret_cast<uint8_t*>(&counter),sizeof(counter));
    counterItem.setEcmcDataType(ECMC_EC_U32); counterItem.setAllowWriteToEcmc(false);
    CHECK(ecmcDataAccess::bind(counterItem,&counterHealthy,0,&access));
    CHECK(counterEnc->setDataAtIndex(access,ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION,-1)==0);
    counterStatus.setEcmcDataPointer(reinterpret_cast<uint8_t*>(&counterReady),sizeof(counterReady));
    counterStatus.setEcmcDataType(ECMC_EC_U32); counterStatus.setAllowWriteToEcmc(false);
    CHECK(ecmcDataAccess::bind(counterStatus,&counterHealthy,0,&access));
    CHECK(counterEnc->setDataAtIndex(access,ECMC_ENCODER_ENTRY_INDEX_READY,0)==0);
    CHECK(counterEnc->setBits(32)==0);
    CHECK(counterEnc->setType(ECMC_ENCODER_TYPE_INCREMENTAL)==0);
    CHECK(counterEnc->setScaleNum(1)==0); CHECK(counterEnc->setScaleDenom(1000)==0);
  }
  void bind(ecmcDataItem &item, uint32_t &value, int slot, bool write, int bit=-1) {
    item.setEcmcDataPointer(reinterpret_cast<uint8_t*>(&value),sizeof(value));
    item.setEcmcDataType(ECMC_EC_U32); item.setAllowWriteToEcmc(write);
    ecmcDataAccess access;
    CHECK(ecmcDataAccess::bind(item,&healthy,0,&access));
    access.writable = write;
    CHECK(axis->getPrimEnc()->setDataAtIndex(access,slot,bit)==0);
  }
  bool powered() const { return bool(relay&1)==activeHigh; }
  void tick() {
    if(powered()) supplySeconds+=.001; else supplySeconds=0;
    const bool valid=powered() && supplySeconds>=.012 && !neverReady && !lostReady;
    ready=(invertedReady ? !valid : valid);
    // Off-state hardware alarms must not generate false idle/startup alarms.
    alarm=powered() ? 0 : 1;
    raw=powered() ? uint32_t(std::llround(physical*1000) + rawBias + sourceShift) : offRaw;
    counter=uint32_t(std::llround(physical*1000)+100000);
    const bool wasPowered=powered();
    axis->execute(true);
    if(withBackup && wasPowered && !powered() && !axis->getErrorID()) {
      CHECK(axis->getPrimaryEncoderIndex()==2); // reference/select before relay drops
      CHECK(axis->feedbackAvailable());
    }
    physical+=axis->getAxisStatusDataPtr()->currentVelocitySetpoint*.001;
  }
  void ticks(int count) { for(int i=0;i<count;++i) tick(); }
  void idle() {
    for (int i=0; i<2500; ++i) {
      tick();
      CHECK(!axis->getEnable()); CHECK(!axis->getExecute());
      CHECK(!axis->getErrorID() || axis->getErrorID()==ERROR_AXIS_HW_NOT_READY);
      if (!axis->getInStartupPhase() && !powered()) break;
    }
    CHECK(!axis->getInStartupPhase()); CHECK(!axis->getBusy());
    CHECK(!axis->getErrorID()); CHECK(!powered());
    int error=0; CHECK(axis->getEnc(0,&error)->getPowerState()==1);
    CHECK(!axis->getEnable());
  }
  void waitMotion() {
    for(int i=0;i<300 && !axis->getExecute();++i) tick();
    CHECK(!axis->getErrorID()); CHECK(axis->getExecute()); CHECK(axis->getEnable());
    CHECK(axis->getPrimEnc()->getPowerReady());
    CHECK(supplySeconds>=.05);
  }
  void finishMove() {
    for(int i=0;i<3000 && axis->getBusy();++i) tick();
    CHECK(!axis->getErrorID()); CHECK(!axis->getBusy());
    for(int i=0;i<20 && !axis->getMon()->getAtTarget();++i) tick();
    CHECK(axis->getMon()->getAtTarget());
  }
  void sleepAfterMove() {
    CHECK(axis->getPrimEnc()->setPowerOffDelay(.08)==0);
    CHECK(axis->moveAbsolutePosition(10.2,.2,1,1)==0);
    waitMotion(); finishMove();
    CHECK(powered()); ticks(100);
    CHECK(axis->getEnable()); CHECK(axis->getEnabled()); CHECK(!powered());
    CHECK(axis->getPrimEnc()->getPowerOff()); CHECK(!axis->getErrorID());
  }
  void disable() {
    CHECK(axis->stopMotion(1)==0);
    ticks(10);
    CHECK(!axis->getEnable()); CHECK(!axis->getBusy());
    if(withBackup) ticks(2100); // independent encoder dwell survives drive disable
    CHECK(!powered());
  }
};

int main() {
  try {
    int count=0;
    {
      scenario="filter configuration follows selected encoder, not primary feedback";
      Rig r(true,false,true,true,false);
      auto *primary=r.axis->getPrimEnc();
      int error=0; auto *secondary=r.axis->getEnc(1,&error); CHECK(secondary && !error);
      CHECK(primary!=secondary); CHECK(r.axis->getPrimaryEncoderIndex()==1);
      CHECK(primary->setPosFilterSize(10)==0); CHECK(primary->setPosFilterEnable(false)==0);
      CHECK(primary->setVeloFilterSize(100)==0); CHECK(primary->setVelFilterEnable(false)==0);
      CHECK(r.axis->selectConfigEncoder(2)==0);
      for (int size : {1,10,100}) {
        CHECK(r.axis->setEncPosFiltSize(size)==0);
        CHECK(r.axis->getConfigEnc()->getPosFilterSize()==size);
        CHECK(primary->getPosFilterSize()==10);
      }
      for (bool enable : {false,true}) {
        CHECK(r.axis->setEncPosFiltEnable(enable)==0);
        CHECK(secondary->getPosFilterEnable()==enable);
        CHECK(!primary->getPosFilterEnable());
      }
      for (int size : {1,25,100}) {
        CHECK(r.axis->setEncVeloFiltSize(size)==0);
        CHECK(secondary->getVeloFilterSize()==size);
        CHECK(primary->getVeloFilterSize()==100);
      }
      for (bool enable : {false,true}) {
        CHECK(r.axis->setEncVeloFiltEnable(enable)==0);
        CHECK(secondary->getVelFilterEnable()==enable);
        CHECK(!primary->getVelFilterEnable());
      }
      CHECK(r.axis->selectConfigEncoder(1)==0);
      CHECK(r.axis->setEncPosFiltSize(25)==0); CHECK(primary->getPosFilterSize()==25);
      CHECK(r.axis->setEncVeloFiltSize(50)==0); CHECK(primary->getVeloFilterSize()==50);
      CHECK(r.axis->setEncPosFiltEnable(false)==0); CHECK(!primary->getPosFilterEnable());
      CHECK(r.axis->setEncVeloFiltEnable(false)==0); CHECK(!primary->getVelFilterEnable());
      CHECK(secondary->getPosFilterSize()==100 && secondary->getPosFilterEnable());
      CHECK(secondary->getVeloFilterSize()==100 && secondary->getVelFilterEnable());
      CHECK(r.axis->getPrimaryEncoderIndex()==1); ++count;
    }
    for (bool legacy : {false,true}) {
      scenario=legacy ? "shared encoder control with latch fallback" :
                        "shared encoder control with native touch probe";
      Rig r; r.idle();
      auto *enc=r.axis->getPrimEnc();
      uint32_t control=0x8000, stored=0, capture=10000;
      ecmcDataItem controlItem{"probeControl"}, statusItem{"probeStatus"}, capturedItem{"probeCapture"};
      r.bind(controlItem,control,legacy ? ECMC_ENCODER_ENTRY_INDEX_LATCH_CONTROL :
                                        ECMC_ENCODER_ENTRY_INDEX_TOUCH_PROBE_CONTROL,true,0);
      r.bind(statusItem,stored,legacy ? ECMC_ENCODER_ENTRY_INDEX_LATCH_STATUS :
                                      ECMC_ENCODER_ENTRY_INDEX_TOUCH_PROBE_STATUS,false,0);
      r.bind(capturedItem,capture,legacy ? ECMC_ENCODER_ENTRY_INDEX_LATCH_VALUE :
                                         ECMC_ENCODER_ENTRY_INDEX_TOUCH_PROBE_VALUE,false);
      const size_t before=r.port.parameterCount();
      CHECK(enc->setTouchProbeArmControlWord(17,5)==0);
      CHECK(r.port.parameterCount()==before+6); // Six optional capture results; control word already exists.
      CHECK(enc->setTouchProbeArmControlWord(17,5)==0);
      CHECK(r.port.parameterCount()==before+6); // Reconfiguration does not duplicate parameters.
      if (legacy) CHECK(enc->setHomeLatchArmControlWord(17,5)==0);
      CHECK(enc->validate()==0);
      auto *word=r.port.find("ax1.enc.control1"); CHECK(word && word->writable);
      CHECK(!r.port.find("ax1.enc.touchprobe.armcmd1"));
      CHECK(!r.port.find("ax1.enc.touchprobe.armed1"));
      auto read=[&]() { uint32_t value=0; memcpy(&value,word->data,sizeof(value)); return value; };
      auto write=[&](uint32_t value,uint32_t mask) {
        uint32_t merged=(read() & ~mask) | (value & mask);
        CHECK(enc->encoderAsynWriteControl(&merged,sizeof(merged),asynParamUInt32Digital)==asynSuccess);
      };
      CHECK(read()==0);
      write(1,1); // Enable without arming must only send idle.
      CHECK(read()==1); CHECK(enc->writeEntries()==0); CHECK(control==0x8000);
      write(2,2); // Arm preserves enable and the unrelated physical PDO bits.
      CHECK(read()==3); CHECK(enc->writeEntries()==0); CHECK(control==0x8011);
      write(0,2); // Disarm retains enable for subsequent arm requests.
      CHECK(read()==1); CHECK(enc->writeEntries()==0); CHECK(control==0x8000);
      write(2,2); CHECK(enc->writeEntries()==0); CHECK(control==0x8011);
      write(0,1); // Disable while armed forces arm low and transmits idle first.
      CHECK(read()==0); CHECK(enc->writeEntries()==0); CHECK(control==0x8000);
      CHECK(enc->writeEntries()==0); CHECK(control==0x8000);
      write(2,2); CHECK(read()==0); // An arm bit cannot bypass disabled ownership.
      CHECK(enc->writeEntries()==0); CHECK(control==0x8000);
      write(4,4); CHECK(read()==4); // Auto-rearm is independent of control enable.
      write(1,1); CHECK(read()==5);
      write(2,2); CHECK(read()==7); CHECK(enc->writeEntries()==0); CHECK(control==0x8011);
      write(0,4); CHECK(read()==3); // Changing auto-rearm preserves enable/arm.
      // Power and capture masked writes preserve each other's accepted bits.
      write(8,8); CHECK(read()==11); CHECK(enc->getPowerControl()==1);
      write(0,1); CHECK(read()==8); CHECK(enc->getPowerControl()==1);
      write(0,8); CHECK(read()==0); CHECK(enc->getPowerControl()==0);
      write(3,3); CHECK(read()==3);
      uint32_t invalid=16;
      CHECK(enc->encoderAsynWriteControl(&invalid,sizeof(invalid),asynParamUInt32Digital)==asynError);
      CHECK(enc->encoderAsynWriteControl(nullptr,sizeof(invalid),asynParamUInt32Digital)==asynError);
      CHECK(enc->encoderAsynWriteControl(&invalid,1,asynParamUInt32Digital)==asynError);
      CHECK(enc->encoderAsynWriteControl(&invalid,sizeof(invalid),asynParamInt32)==asynError);
      CHECK(read()==3);
      r.axis->dataForTest()->control_.controlWord_.blockCom=1;
      invalid=0;
      CHECK(enc->encoderAsynWriteControl(&invalid,sizeof(invalid),asynParamUInt32Digital)==asynError);
      CHECK(read()==3);
      r.axis->dataForTest()->control_.controlWord_.blockCom=0;
      // Native/PLC commands immediately update the same backing word. A PV
      // masked write must preserve them without waiting for cyclic publication.
      CHECK(r.axis->setTouchProbeArm(0,false)==0); CHECK(read()==0);
      CHECK(r.axis->setTouchProbeArm(0,true)==0); CHECK(read()==7);
      write(0,4); CHECK(read()==3);
      enc->refreshTouchProbeAsyn(0,true); CHECK(word->lastRTValue==3);
      CHECK(r.axis->setTouchProbeArm(0,false)==0);
      CHECK(enc->writeEntries()==0); CHECK(control==0x8000); ++count;
    }
    {
      scenario="probe control rejects an encoder without probe or latch links";
      Rig r; uint32_t request=3;
      CHECK(r.axis->getPrimEnc()->encoderAsynWriteControl(&request,sizeof(request),asynParamUInt32Digital)==asynError);
      ++count;
    }
    for(bool high : {false,true}) for(bool readyInput : {false,true})
      for(bool backup : {false,true}) {
      scenario="initial startup powers and reads feedback without enabling motion";
      Rig r(high,false,readyInput,backup);
      r.ticks(25);
      CHECK(r.powered()); CHECK(r.axis->getInStartupPhase());
      CHECK(!r.axis->getEnable()); CHECK(!r.axis->getExecute());
      CHECK(!r.axis->getErrorID());
      CHECK(std::abs(r.physical-10)<1e-12);
      r.idle();
      int error=0; auto *source=r.axis->getEnc(0,&error);
      CHECK(std::abs(source->getActPos()-10)<.001);
      CHECK(std::abs(r.physical-10)<1e-12);
      if (backup) {
        CHECK(r.axis->getPrimaryEncoderIndex()==2);
        CHECK(std::abs(r.axis->getPrimEnc()->getActPos()-10)<.001);
      }
      ++count;
    }
    {
      scenario="initial startup missing ready times out without enabling drive";
      Rig r; r.neverReady=true; r.ticks(250);
      CHECK(r.axis->getErrorID()==ERROR_ENC_POWER_TIMEOUT);
      CHECK(r.axis->getInStartupPhase()); CHECK(!r.powered());
      CHECK(!r.axis->getEnable()); CHECK(!r.axis->getExecute());
      CHECK(std::abs(r.physical-10)<1e-12); ++count;
    }
    {
      scenario="initial startup waits for valid process data before power timeout";
      Rig r; r.healthy=0; r.ticks(400);
      CHECK(!r.powered()); CHECK(r.axis->getInStartupPhase());
      CHECK(r.axis->getPrimEnc()->getPowerError()==0);
      CHECK(!r.axis->getEnable()); CHECK(!r.axis->getExecute());
      r.healthy=1; r.idle(); ++count;
    }
    {
      scenario="initial read also verifies powered source in explicit counter mode";
      Rig r(true,false,true,true);
      r.axis->setInStartupPhase(true);
      CHECK(r.axis->setEncoderMode(2)==0);
      r.ticks(25); CHECK(r.powered()); CHECK(r.axis->getInStartupPhase());
      r.idle();
      CHECK(r.axis->getPrimaryEncoderIndex()==2);
      int error=0; CHECK(std::abs(r.axis->getEnc(0,&error)->getActPos()-10)<.001);
      ++count;
    }
    for(bool high : {false,true}) for(bool inverted : {false,true}) {
      scenario="startup/ABS/polarities";
      Rig r(high,inverted); r.idle();
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0);
      r.ticks(30);
      CHECK(!r.axis->getEnable()); CHECK(!r.axis->getExecute());
      CHECK(!r.axis->getErrorID()); CHECK(r.powered());
      r.waitMotion();
      CHECK(std::abs(r.axis->getSeq()->getTargetPos()-10.2)<.001);
      CHECK(std::abs(r.axis->getEncVelo())<.01);
      r.disable(); CHECK(!r.axis->getErrorID()); ++count;
    }
    {
      scenario="no ready input";
      Rig r(true,false,false); r.idle(); r.neverReady=true;
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0);
      r.waitMotion(); r.disable(); ++count;
    }
    {
      scenario="relative uses fresh position";
      Rig r; r.idle(); r.physical=10.7;
      CHECK(r.axis->moveRelativePosition(.2,.2,1,1)==0);
      r.waitMotion();
      CHECK(std::abs(r.axis->getSeq()->getTargetPos()-10.9)<.001);
      CHECK(std::abs(r.axis->getAxisStatusDataPtr()->currentPositionSetpoint-10.7)<.002);
      r.disable(); ++count;
    }
    {
      scenario="ready timeout";
      Rig r; r.idle(); r.neverReady=true;
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0);
      r.ticks(250);
      CHECK(r.axis->getErrorID()==ERROR_ENC_POWER_TIMEOUT);
      CHECK(!r.axis->getEnable()); CHECK(!r.axis->getExecute()); CHECK(!r.powered());
      CHECK(r.axis->getPrimEnc()->getPowerState()==4); ++count;
    }
    for(bool disable : {false,true}) {
      scenario="cancel warmup";
      Rig r; r.idle();
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0);
      r.ticks(20);
      CHECK(r.axis->stopMotion(disable)==0);
      r.ticks(300);
      CHECK(!r.axis->getEnable()); CHECK(!r.axis->getExecute()); CHECK(!r.powered());
      CHECK(!r.axis->getBusy()); CHECK(!r.axis->getErrorID()); ++count;
    }
    {
      scenario="ready lost while enabled";
      Rig r; r.idle(); CHECK(r.axis->setEnable(true)==0); r.ticks(80);
      CHECK(r.axis->getEnable()); CHECK(r.powered());
      r.lostReady=true; r.ticks(5);
      CHECK(r.axis->getErrorID()==ERROR_ENC_NOT_READY);
      CHECK(!r.axis->getEnable()); CHECK(!r.powered()); ++count;
    }
    for(bool relative : {false,true}) {
      scenario="wake with amplifier already enabled";
      Rig r; r.idle(); r.sleepAfterMove();
      CHECK(r.axis->setEnableAutoEnable(false)==0);
      r.physical=10.5; r.ticks(20); // retained position cannot observe this drift
      CHECK(std::abs(r.axis->getPrimEnc()->getActPos()-10.2)<.005);
      CHECK((relative ? r.axis->moveRelativePosition(.2,.2,1,1) :
                        r.axis->moveAbsolutePosition(10.7,.2,1,1))==0);
      r.ticks(30);
      CHECK(r.axis->getEnable()); CHECK(r.axis->getBusy());
      CHECK(!r.axis->getExecute()); CHECK(r.powered());
      CHECK(std::abs(r.axis->getAxisStatusDataPtr()->currentVelocitySetpoint)<1e-12);
      r.waitMotion();
      CHECK(std::abs(r.axis->getSeq()->getTargetPos()-10.7)<.001);
      r.finishMove(); r.ticks(100); CHECK(!r.powered());
      CHECK(r.axis->getEnable()); r.disable(); ++count;
    }
    {
      scenario="atTarget loss restarts power-off delay";
      Rig r; r.idle(); CHECK(r.axis->getPrimEnc()->setPowerOffDelay(.08)==0);
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0);
      r.waitMotion(); r.finishMove(); r.ticks(30); CHECK(r.powered());
      const double settled=r.physical; r.physical+=.02; r.ticks(60);
      CHECK(!r.axis->getMon()->getAtTarget()); CHECK(r.powered());
      r.physical=settled; r.ticks(50); CHECK(r.powered());
      r.ticks(50); CHECK(!r.powered()); CHECK(!r.axis->getErrorID()); ++count;
    }
    {
      scenario="tighter controller deadband";
      Rig r; r.idle(); CHECK(r.axis->getPrimEnc()->setPowerOffDelay(.08)==0);
      CHECK(r.axis->getMon()->setCtrlDeadband(.001)==0);
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0);
      r.waitMotion(); r.finishMove();
      r.physical=10.203; r.ticks(100);
      CHECK(r.axis->getMon()->getAtTarget()); CHECK(r.powered());
      r.physical=10.2; r.ticks(100); CHECK(!r.powered()); ++count;
    }
    for(bool cancel : {false,true}) {
      scenario=cancel ? "STOP during wake with holding torque" : "wake ready timeout with holding torque";
      Rig r; r.idle(); r.sleepAfterMove(); r.neverReady=true;
      CHECK(r.axis->moveRelativePosition(.2,.2,1,1)==0); r.ticks(20);
      if(cancel) {
        CHECK(r.axis->stopMotion(0)==0); r.neverReady=false; r.physical=10.5;
      }
      r.ticks(cancel ? 60 : 250);
      CHECK(!r.axis->getExecute()); CHECK(!r.axis->getBusy());
      if(cancel) { CHECK(!r.axis->getErrorID()); CHECK(r.axis->getEnable());
        CHECK(std::abs(r.axis->getAxisStatusDataPtr()->currentPositionSetpoint-10.5)<.001);
        r.disable(); }
      else { CHECK(r.axis->getErrorID()==ERROR_ENC_POWER_TIMEOUT); CHECK(!r.axis->getEnable()); }
      ++count;
    }
    {
      scenario="atTarget is mandatory";
      Rig r; r.idle(); r.axis->getMon()->setEnableAtTargetMon(false);
      CHECK(r.axis->validate()==ERROR_ENC_POWER_CONFIG); ++count;
    }
    {
      scenario="explicit wake without another drive enable edge";
      Rig r; r.idle(); r.sleepAfterMove();
      CHECK(r.axis->setEnable(true)==0); r.ticks(80);
      CHECK(r.powered()); CHECK(r.axis->getPrimEnc()->getPowerReady());
      CHECK(r.axis->getEnable()); CHECK(!r.axis->getBusy());
      CHECK(!r.axis->getErrorID()); r.disable(); ++count;
    }
    {
      scenario="external source cannot bypass powered-off feedback";
      Rig r; r.idle(); r.sleepAfterMove();
      CHECK(r.axis->getSeq()->setTrajDataSourceTypeInternal(ECMC_DATA_SOURCE_EXTERNAL,1)==ERROR_AXIS_BUSY);
      CHECK(r.axis->getTrajDataSourceType()==ECMC_DATA_SOURCE_INTERNAL);
      CHECK(!r.powered()); CHECK(!r.axis->getErrorID());
      r.disable(); ++count;
    }
    {
      scenario="retarget during warmup";
      Rig r; r.idle(); r.sleepAfterMove();
      CHECK(r.axis->moveRelativePosition(.1,.2,1,1)==0); r.ticks(20);
      CHECK(r.axis->moveAbsolutePosition(10.6,.2,1,1)==0); r.ticks(10);
      CHECK(!r.axis->getExecute()); r.waitMotion();
      CHECK(std::abs(r.axis->getSeq()->getTargetPos()-10.6)<.001);
      r.disable(); ++count;
    }
    for(bool highValidWord : {false,true}) for(bool withReady : {false,true}) {
      scenario="off-state words cannot add/remove raw counter turns";
      Rig r(true,false,withReady);
      r.rawBias=highValidWord ? 0xffff0000u : 0;
      r.offRaw=highValidWord ? 0 : 0xffffffffu;
      CHECK(r.axis->getPrimEnc()->setOffset(-double(r.rawBias)/1000)==0);
      r.idle(); r.sleepAfterMove();
      auto *enc=r.axis->getPrimEnc();
      const uint64_t valid=enc->getRawPosRegister();
      CHECK(std::abs(double(valid)-double(r.offRaw)) > .7*double(0xffffffffu));
      const int64_t multiTurn=enc->getRawPosMultiTurn();
      const double position=enc->getActPos();
      for(int cycle=0;cycle<4;++cycle) {
        for(uint32_t invalid : {0u,0xffffffffu,r.offRaw}) {
          r.offRaw=invalid;
          for(int n=0;n<20;++n) {
            r.tick();
            CHECK(enc->getRawPosRegister()==valid);
            CHECK(enc->getRawPosMultiTurn()==multiTurn);
            CHECK(std::abs(enc->getActPos()-position)<.0001);
          }
        }
        CHECK(r.axis->setEnable(true)==0); r.ticks(70);
        CHECK(enc->getPowerReady()); CHECK(r.powered());
        CHECK(enc->getRawPosRegister()==valid);
        CHECK(enc->getRawPosMultiTurn()==multiTurn);
        CHECK(std::abs(enc->getActPos()-position)<.0001);
        CHECK(std::abs(enc->getActVel())<.01);
        CHECK(!r.axis->getErrorID());
        r.ticks(100); CHECK(!r.powered());
      }
      ++count;
    }
    for(bool overflow : {false,true}) {
      scenario="power cycle preserves a legitimately accumulated turn";
      Rig r;
      const double span=4294967296.0/1000;
      r.physical=overflow ? span-.04 : .04;
      r.offRaw=overflow ? 0xffffffffu : 0;
      r.idle(); CHECK(r.axis->getPrimEnc()->setPowerOffDelay(.08)==0);
      CHECK(r.axis->setEnable(true)==0); r.ticks(80);
      const double target=r.physical+(overflow ? .08 : -.08);
      CHECK(r.axis->moveAbsolutePosition(target,.2,1,1)==0);
      r.waitMotion(); r.finishMove(); r.ticks(100);
      auto *enc=r.axis->getPrimEnc();
      CHECK(enc->getPowerOff());
      const int64_t legitimate=enc->getRawPosMultiTurn();
      CHECK(overflow ? legitimate>int64_t(0xffffffffu) : legitimate<0);
      const double position=enc->getActPos();
      for(int cycle=0;cycle<3;++cycle) {
        CHECK(r.axis->setEnable(true)==0); r.ticks(70);
        CHECK(enc->getPowerReady());
        CHECK(enc->getRawPosMultiTurn()==legitimate);
        CHECK(std::abs(enc->getActPos()-position)<.0001);
        CHECK(std::abs(enc->getActPos()-r.physical)<.002);
        CHECK(!r.axis->getErrorID()); r.ticks(100); CHECK(enc->getPowerOff());
      }
      r.disable(); ++count;
    }
    for(bool relative : {false,true}) for(double sign : {-1.0,1.0}) {
      scenario="wake jump exceeds five atTarget tolerances";
      Rig r; r.idle(); r.sleepAfterMove();
      auto *mon=r.axis->getMon();
      const double tolerance=mon->getAtTargetTol();
      mon->setEnableLagMon(true); mon->setPosLagTol(2*tolerance); mon->setPosLagTime(1);
      const double retained=r.axis->getPrimEnc()->getActPos();
      r.physical+=sign*6*tolerance;
      const double fresh=r.physical;
      CHECK(std::abs(fresh-retained)>5*tolerance);
      CHECK(std::abs(fresh-retained)>mon->getPosLagTol());
      r.ticks(20);
      CHECK(std::abs(r.axis->getPrimEnc()->getActPos()-retained)<.001);
      CHECK(!r.axis->getErrorID());
      CHECK(r.axis->setEnableAutoEnable(false)==0);
      const double target=fresh+.08;
      CHECK((relative ? r.axis->moveRelativePosition(.08,.2,1,1) :
                        r.axis->moveAbsolutePosition(target,.2,1,1))==0);
      r.ticks(30);
      CHECK(!r.axis->getExecute()); CHECK(r.axis->getEnable());
      CHECK(std::abs(r.axis->getAxisStatusDataPtr()->currentVelocitySetpoint)<1e-12);
      r.waitMotion();
      CHECK(std::abs(r.axis->getPrimEnc()->getActVel())<.01);
      CHECK(std::abs(r.axis->getSeq()->getTargetPos()-target)<.001);
      CHECK(std::abs(r.axis->getAxisStatusDataPtr()->currentPositionSetpoint-fresh)<.002);
      r.finishMove(); CHECK(!r.axis->getErrorID()); r.disable(); ++count;
    }
    for(bool readyInput : {false,true}) {
      scenario="counter handover and wake";
      Rig r(true,false,readyInput,true); r.idle();
      int error=0; auto *source=r.axis->getEnc(0,&error); auto *backup=r.axis->getEnc(1,&error);
      CHECK(r.axis->getPrimaryEncoderIndex()==2); CHECK(!r.powered());
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0);
      r.waitMotion(); CHECK(r.axis->getPrimaryEncoderIndex()==1);
      r.finishMove(); r.ticks(2100);
      CHECK(!r.powered()); CHECK(r.axis->getEnable());
      CHECK(r.axis->getPrimaryEncoderIndex()==2); CHECK(r.axis->feedbackAvailable());
      CHECK(std::abs(backup->getActPos()-source->getActPos())<.001);
      const double previous=backup->getActPos(); r.physical+=.003; r.ticks(10);
      CHECK(backup->getActPos()>previous+.002); // feedback is live, not held
      CHECK(std::abs(backup->getActVel())<1);
      CHECK(r.axis->moveRelativePosition(.08,.2,1,1)==0);
      r.ticks(20); CHECK(r.axis->feedbackAvailable());
      CHECK(r.axis->getPrimaryEncoderIndex()==2); // counter still controls during warmup
      r.waitMotion(); CHECK(r.axis->getPrimaryEncoderIndex()==1);
      CHECK(std::abs(r.axis->getSeq()->getTargetPos()-(previous+.003+.08))<.002);
      r.finishMove(); CHECK(!r.axis->getErrorID()); r.disable(); ++count;
    }
    for(int mismatch : {-30,30}) {
      scenario="counter/absolute wake mismatch blocks motion";
      Rig r(true,false,true,true); r.idle();
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0); r.waitMotion(); r.finishMove(); r.ticks(2100);
      CHECK(r.axis->getPrimaryEncoderIndex()==2); r.sourceShift=mismatch;
      const double previous=r.physical;
      CHECK(r.axis->moveRelativePosition(.08,.2,1,1)==0); r.ticks(100);
      CHECK(r.axis->getErrorID()==ERROR_ENC_POWER_MISMATCH);
      CHECK(!r.axis->getEnable()); CHECK(!r.axis->getExecute());
      CHECK(r.axis->getPrimaryEncoderIndex()==2);
      CHECK(std::abs(r.physical-previous)<.001);
      r.sourceShift=0; r.axis->errorReset(); r.ticks(100);
      CHECK(!r.axis->getExecute()); CHECK(!r.axis->getBusy());
      CHECK(r.axis->moveRelativePosition(.08,.2,1,1)==0); r.waitMotion(); r.finishMove();
      CHECK(!r.axis->getErrorID()); r.disable(); ++count;
    }
    {
      scenario="unready backup prevents relay handover";
      Rig r(true,false,true,true); r.idle();
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0); r.waitMotion(); r.finishMove();
      r.counterReady=0; r.ticks(5);
      CHECK(r.axis->getErrorID()!=0); CHECK(!r.axis->getEnable());
      CHECK(!r.axis->getExecute()); ++count;
    }
    {
      scenario="within-limit handover rebases without following error";
      Rig r(true,false,true,true); r.idle();
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0); r.waitMotion(); r.finishMove(); r.ticks(2100);
      r.sourceShift=5; // .005 mm mismatch, below .01 handover tolerance
      auto *mon=r.axis->getMon();
      mon->setEnableLagMon(true); mon->setPosLagTol(.002); mon->setPosLagTime(1);
      CHECK(r.axis->moveRelativePosition(.08,.2,1,1)==0); r.waitMotion();
      CHECK(r.axis->getPrimaryEncoderIndex()==1);
      CHECK(std::abs(r.axis->getSeq()->getTargetPos()-10.285)<.002);
      r.finishMove(); CHECK(!r.axis->getErrorID()); r.disable(); ++count;
    }
    {
      scenario="handover configuration rejects invalid backup";
      Rig r; r.idle();
      CHECK(r.axis->setEncoderPowerBackup(1,1,.01)==ERROR_ENC_POWER_CONFIG);
      CHECK(r.axis->setEncoderPowerBackup(1,2,.01)==ERROR_ENC_POWER_CONFIG);
      CHECK(r.axis->setEncoderPowerBackup(1,1,0)==ERROR_ENC_POWER_CONFIG);
      CHECK(r.axis->getEncoderPowerBackup()==0);
      CHECK(r.axis->setEncoderPowerBackup(1,0,.03)==0);
      CHECK(r.axis->getEncoderPowerDiffTol()==.03); ++count;
    }
    {
      scenario="handover dwell is independent of normal drive disable";
      Rig r(true,false,true,true); r.idle();
      int error=0; auto *source=r.axis->getEnc(0,&error);
      CHECK(source->setPowerOffDelay(.08)==0);
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0); r.waitMotion(); r.finishMove();
      CHECK(r.axis->setEnable(false)==0); r.ticks(30);
      CHECK(!r.axis->getEnable()); CHECK(r.powered());
      CHECK(r.axis->getPrimaryEncoderIndex()==1);
      r.ticks(100); CHECK(!r.powered()); CHECK(r.axis->getPrimaryEncoderIndex()==2);
      CHECK(!r.axis->getErrorID()); ++count;
    }
    {
      scenario="invalid counter transport cannot remain in control";
      Rig r(true,false,true,true); r.idle();
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0); r.waitMotion(); r.finishMove(); r.ticks(2100);
      CHECK(r.axis->getPrimaryEncoderIndex()==2); r.counterHealthy=0; r.ticks(5);
      CHECK(r.axis->getErrorID()==ERROR_ENC_NOT_READY);
      CHECK(!r.axis->feedbackAvailable());
      CHECK(!r.axis->getEnable()); CHECK(!r.axis->getExecute()); ++count;
    }
    for(bool backup : {false,true}) {
      scenario="Force ON persists until explicit Auto release";
      Rig r(true,false,true,backup); r.idle();
      int error=0; auto *source=r.axis->getEnc(0,&error);
      CHECK(source->setPowerOffDelay(.08)==0);
      CHECK(source->setPowerControl(1)==0); r.ticks(100);
      CHECK(source->getPowerReady()); CHECK(r.powered());
      CHECK(!r.axis->getEnable()); CHECK(!r.axis->getExecute());
      r.ticks(300); CHECK(r.powered());
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0); r.waitMotion(); r.finishMove();
      r.ticks(300); CHECK(r.powered()); CHECK(source->getPowerControl()==1);
      CHECK(source->setPowerControl(0)==0); r.ticks(30); CHECK(r.powered());
      r.ticks(100); CHECK(!r.powered()); CHECK(!r.axis->getErrorID());
      r.disable(); ++count;
    }
    for(bool backup : {false,true}) {
      scenario="manual release without a move waits full settling delay";
      Rig r(true,false,true,backup); r.idle();
      int error=0; auto *source=r.axis->getEnc(0,&error);
      CHECK(source->setPowerOffDelay(.08)==0);
      CHECK(source->setPowerControl(1)==0); r.ticks(200);
      CHECK(r.powered()); CHECK(!r.axis->getEnable());
      CHECK(source->setPowerControl(0)==0); r.ticks(30); CHECK(r.powered());
      r.ticks(100); CHECK(!r.powered()); CHECK(!r.axis->getErrorID()); ++count;
    }
    {
      scenario="manual release during motion cannot cut encoder power";
      Rig r; r.idle(); auto *source=r.axis->getPrimEnc();
      CHECK(source->setPowerOffDelay(.08)==0);
      CHECK(source->setPowerControl(1)==0); r.ticks(100);
      CHECK(r.axis->moveAbsolutePosition(10.4,.2,1,1)==0); r.waitMotion();
      CHECK(source->setPowerControl(0)==0); r.ticks(200);
      CHECK(r.axis->getBusy()); CHECK(r.powered());
      r.finishMove(); r.ticks(100); CHECK(!r.powered()); CHECK(!r.axis->getErrorID()); ++count;
    }
    {
      scenario="Force ON does not bypass encoder faults";
      Rig r; r.idle(); auto *source=r.axis->getPrimEnc();
      CHECK(source->setPowerControl(1)==0); r.ticks(100);
      r.lostReady=true; r.ticks(10);
      CHECK(r.axis->getErrorID()==ERROR_ENC_NOT_READY);
      CHECK(!r.powered()); CHECK(!r.axis->getEnable()); CHECK(!r.axis->getExecute());
      CHECK(source->setPowerControl(0)==0); r.lostReady=false;
      r.axis->errorReset(); r.ticks(100); CHECK(!r.powered()); ++count;
    }
    {
      scenario="axis packed Force ON and fixed encoder modes";
      Rig r(true,false,true,true); r.idle();
      CHECK(!r.port.find("ax1.enc.pwrctrl1"));
      CHECK(!r.port.find("ax1.enc.pwrctrl2"));
      CHECK(r.axis->setEncoderMode(9)==ERROR_ENC_POWER_CONFIG);
      CHECK(r.axis->setEncoderMode(2)==0); // counter is encoder 2 in this fixture
      CHECK(r.axis->getPrimaryEncoderIndex()==2);
      CHECK(r.axis->getEncoderMode()==2);
      r.ticks(100); CHECK(!r.powered());
      CHECK(r.axis->feedbackAvailable());
      // Fixed counter motion neither powers nor waits for the absolute source.
      const double initial=r.axis->getPrimEnc()->getActPos();
      CHECK(r.axis->moveAbsolutePosition(initial+.04,.2,1,1)==0);
      r.ticks(40); CHECK(r.axis->getExecute()); CHECK(!r.powered());
      CHECK(r.axis->setEncoderMode(1)==ERROR_AXIS_BUSY);
      r.finishMove(); CHECK(!r.powered()); CHECK(!r.axis->getErrorID());
      CHECK(r.axis->setEnable(false)==0); r.ticks(20);
      // Force ON for inspection must leave fixed counter feedback selected.
      CHECK(r.axis->setEncoderForceOn(1)==0); r.ticks(100);
      CHECK(r.powered()); CHECK(r.axis->getPrimaryEncoderIndex()==2);
      CHECK(r.axis->getEncoderForceOn()==1);
      CHECK(r.axis->setEncoderMode(1)==0); // fixed absolute is encoder 1
      r.ticks(2200); CHECK(r.powered()); CHECK(r.axis->getPrimaryEncoderIndex()==1);
      CHECK(r.axis->setEncoderForceOn(0)==0); r.ticks(2200);
      CHECK(r.powered()); // fixed powered encoder stays powered
      CHECK(r.axis->setEncoderMode(0)==0); r.ticks(2100);
      CHECK(!r.powered()); CHECK(r.axis->getPrimaryEncoderIndex()==2);
      CHECK(!r.axis->getErrorID()); ++count;
    }
    {
      scenario="packed axis control selects mode and publishes status without extra parameters";
      Rig r(true,false,true,true); r.idle();
      ecmcAsynAxisControlType word{};
      word.encoderMode=2;
      CHECK(r.axis->axisAsynWriteCmd(&word,sizeof(word),asynParamUInt32Digital)==asynSuccess);
      CHECK(r.axis->getEncoderMode()==2);
      uint32_t encoderControl=ecmcEncoder::POWER_FORCE_ON;
      int error=0; auto *enc=r.axis->getEnc(0,&error); CHECK(enc && !error);
      CHECK(enc->encoderAsynWriteControl(&encoderControl,sizeof(encoderControl),asynParamUInt32Digital)==asynSuccess);
      r.ticks(100); CHECK(r.powered());
      uint32_t control=0,status=0;
      auto *data=r.axis->dataForTest();
      memcpy(&control,&data->control_.controlWord_,4);
      memcpy(&status,&data->status_.statusWord_,4);
      CHECK(((control>>14)&15)==2); CHECK((control&(1u<<18))==0);
      CHECK(((status>>25)&15)==2); CHECK((status&(1u<<29))==0);
      auto *param=r.port.find("ax1.enc.control1"); CHECK(param && param->writable);
      CHECK((param->lastRTValue & ecmcEncoder::POWER_FORCE_ON)!=0);
      uint32_t oldAxisControl=control | (1u<<18);
      CHECK(r.axis->axisAsynWriteCmd(&oldAxisControl,4,asynParamUInt32Digital)==asynError);
      encoderControl=0;
      CHECK(enc->encoderAsynWriteControl(&encoderControl,4,asynParamUInt32Digital)==asynSuccess);
      CHECK(r.axis->getEncoderForceOn()==0);
      // Power writes obey both communication and coordinated-owner blocks.
      r.axis->dataForTest()->control_.controlWord_.blockCom=1;
      encoderControl=8;
      CHECK(enc->encoderAsynWriteControl(&encoderControl,4,asynParamUInt32Digital)==asynError);
      r.axis->dataForTest()->control_.controlWord_.blockCom=0;
      r.axis->dataForTest()->status_.statusWord_.blocked=1;
      CHECK(enc->encoderAsynWriteControl(&encoderControl,4,asynParamUInt32Digital)==asynError);
      r.axis->dataForTest()->status_.statusWord_.blocked=0;
      auto *counter=r.axis->getEnc(1,&error); CHECK(counter && !error);
      CHECK(counter->encoderAsynWriteControl(&encoderControl,4,asynParamUInt32Digital)==asynError);
      CHECK(enc->getPowerControl()==0);
      CHECK(r.axis->setEncoderForceOn(2)==ERROR_ENC_POWER_CONFIG);
      word.encoderMode=15;
      CHECK(r.axis->axisAsynWriteCmd(&word,sizeof(word),asynParamUInt32Digital)==asynError);
      CHECK(r.axis->getEncoderMode()==2); ++count;
    }
    {
      scenario="ignored execute preserves reset and disable in packed control writes";
      Rig r; r.idle();
      auto *axis = r.axis.get();
      auto *data = axis->dataForTest();
      CHECK(axis->setAllowMotionFunctions(false, true, true) == 0);
      CHECK(axis->setIgnoreDisabledMotionCommands(1) == 0);
      CHECK(axis->setCommand(ECMC_CMD_MOVEABS) == 0);
      const auto write = [axis](ecmcAsynAxisControlType word) {
        return axis->axisAsynWriteCmd(&word, sizeof(word), asynParamUInt32Digital);
      };
      auto word = data->control_.controlWord_;
      word.enableCmd = 1;
      word.executeCmd = 1;
      word.resetCmd = 1;
      axis->setErrorID(ERROR_MON_MAX_POSITION_LAG_EXCEEDED);
      CHECK(write(word) == asynSuccess);
      CHECK(axis->getErrorID() == 0 && !axis->getEnable() && !axis->getExecute());
      CHECK(!axis->getBusy() && !axis->getReset());

      CHECK(axis->setEnable(true) == 0);
      axis->setErrorID(ERROR_MON_MAX_POSITION_LAG_EXCEEDED);
      word = data->control_.controlWord_;
      word.enableCmd = 0;
      word.executeCmd = 1;
      word.resetCmd = 1;
      word.enableDbgPrintout = !word.enableDbgPrintout;
      CHECK(write(word) == asynSuccess);
      CHECK(axis->getErrorID() == 0 && !axis->getEnable() && !axis->getExecute());
      CHECK(!axis->getBusy() && !axis->getReset());
      CHECK(data->control_.controlWord_.enableDbgPrintout == word.enableDbgPrintout);
      ++count;
    }
    {
      scenario="fixed powered encoder waits for warmup without a backup pair";
      Rig r; r.idle();
      CHECK(r.axis->setEncoderMode(1)==0);
      r.ticks(20); CHECK(r.powered()); CHECK(!r.axis->feedbackAvailable());
      CHECK(r.axis->moveRelativePosition(.05,.2,1,1)==0);
      r.waitMotion();
      CHECK(std::abs(r.axis->getSeq()->getTargetPos()-10.05)<.002);
      r.finishMove(); CHECK(r.axis->feedbackAvailable());
      r.ticks(2200); CHECK(r.powered());
      CHECK(r.axis->setEnable(false)==0); r.ticks(20);
      CHECK(r.axis->setEncoderMode(0)==0); r.ticks(2100);
      CHECK(!r.powered()); CHECK(!r.axis->getErrorID()); ++count;
    }
    {
      scenario="generic selection accepts encoder 3 and retains its own reference";
      Rig r(true,false,true,true); r.idle();
      CHECK(r.axis->addEncoder()==0);
      int error=0; auto *third=r.axis->getEnc(2,&error); CHECK(third && !error);
      ecmcDataAccess access;
      CHECK(ecmcDataAccess::bind(r.counterItem,&r.counterHealthy,0,&access));
      CHECK(third->setDataAtIndex(access,ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION,-1)==0);
      CHECK(third->setType(ECMC_ENCODER_TYPE_INCREMENTAL)==0);
      CHECK(third->setBits(32)==0); CHECK(third->setScaleNum(1)==0);
      CHECK(third->setScaleDenom(1000)==0);
      CHECK(r.axis->validate()==0);
      r.ticks(20); third->setActPos(27); r.ticks(10);
      CHECK(r.axis->setEncoderMode(3)==0); r.ticks(20);
      CHECK(r.axis->getPrimaryEncoderIndex()==3);
      CHECK(std::abs(third->getActPos()-27)<.001);
      CHECK(!r.powered()); CHECK(r.axis->feedbackAvailable()); ++count;
    }
    {
      scenario="encoder status publishes power, readiness, homed and absolute independently";
      Rig r(true,false,true,true); r.idle();
      CHECK(!r.port.find("ax1.enc.powerstate1"));
      auto *sourceStatus=r.port.find("ax1.enc.status1");
      auto *backupStatus=r.port.find("ax1.enc.status2");
      CHECK(sourceStatus && backupStatus && !sourceStatus->writable && !backupStatus->writable);
      auto powerState=[](uint32_t value) { return (value & ECMC_ENC_STATUS_POWER_STATE_MASK) >> ECMC_ENC_STATUS_POWER_STATE_SHIFT; };
      uint32_t value=sourceStatus->lastRTValue;
      CHECK(powerState(value)==1);
      CHECK((value & ECMC_ENC_STATUS_POWER_ENABLED)!=0);
      CHECK((value & (ECMC_ENC_STATUS_POWER_ON|ECMC_ENC_STATUS_READY))==0);
      CHECK((value & (ECMC_ENC_STATUS_HOMED|ECMC_ENC_STATUS_ABSOLUTE))==
        (ECMC_ENC_STATUS_HOMED|ECMC_ENC_STATUS_ABSOLUTE));
      CHECK((backupStatus->lastRTValue & ECMC_ENC_STATUS_ABSOLUTE)==0);
      int error=0; auto *backup=r.axis->getEnc(1,&error);
      backup->setHomed(true); r.ticks(2);
      CHECK((backupStatus->lastRTValue & ECMC_ENC_STATUS_HOMED)!=0);
      backup->setHomed(false); r.ticks(2);
      CHECK((backupStatus->lastRTValue & ECMC_ENC_STATUS_HOMED)==0);
      CHECK(r.axis->setEncoderForceOn(1)==0); r.ticks(10);
      value=sourceStatus->lastRTValue; CHECK(powerState(value)==2);
      CHECK(value & ECMC_ENC_STATUS_POWER_ON); CHECK(!(value & ECMC_ENC_STATUS_READY));
      r.ticks(100); value=sourceStatus->lastRTValue;
      CHECK(powerState(value)==3); CHECK(value & ECMC_ENC_STATUS_READY);
      r.lostReady=true; r.ticks(5); value=sourceStatus->lastRTValue;
      CHECK(powerState(value)==4); CHECK(!(value & ECMC_ENC_STATUS_READY)); ++count;
    }
    {
      scenario="configure powered source before the backup exists";
      Rig r(true,false,true,false,false);
      CHECK(r.axis->setEncoderPowerBackup(1,2,.01)==0);
      CHECK(r.axis->getEncoderPowerBackup()==2);
      CHECK(r.axis->validate()==ERROR_ENC_POWER_CONFIG); // no null dereference
      r.addCounter(); r.withBackup=true;
      r.axis->errorReset(); CHECK(r.axis->validate()==0);
      CHECK(r.axis->setRealTimeStarted(true)==0); r.idle();
      CHECK(r.axis->moveAbsolutePosition(10.2,.2,1,1)==0);
      r.waitMotion(); r.finishMove(); r.ticks(2100);
      CHECK(!r.powered()); CHECK(r.axis->getPrimaryEncoderIndex()==2);
      CHECK(!r.axis->getErrorID()); ++count;
    }
    {
      scenario="final validation rejects an unsuitable later encoder";
      Rig r(true,false,true,false,false);
      CHECK(r.axis->setEncoderPowerBackup(1,2,.01)==0);
      r.addCounter(); int error=0; auto *backup=r.axis->getEnc(1,&error);
      CHECK(backup->setType(ECMC_ENCODER_TYPE_ABSOLUTE)==0);
      CHECK(r.axis->validate()==ERROR_ENC_POWER_CONFIG);
      CHECK(!r.axis->getEnable()); ++count;
    }

    {
      scenario="ordinary encoder motion bypasses unused power coordination";
      Rig r(true,false,true,false,false);
      auto *enc=r.axis->getPrimEnc();
      CHECK(enc->setPowerEnable(false)==0);
      CHECK(r.axis->setRealTimeStarted(true)==0);
      auto tick=[&]() {
        r.ready=1; r.alarm=0;
        r.raw=uint32_t(std::llround(r.physical*1000));
        r.axis->execute(true);
        r.physical+=r.axis->getAxisStatusDataPtr()->currentVelocitySetpoint*.001;
      };
      for(int i=0;i<100;++i) tick();
      CHECK(!r.axis->getInStartupPhase()); CHECK(!r.axis->getErrorID());
      CHECK(enc->getPowerState()==0); CHECK(!enc->powerPositionSyncPending());
      CHECK(r.axis->moveAbsolutePosition(10.05,.2,1,1)==0);
      for(int i=0;i<1500;++i) tick();
      CHECK(!r.axis->getBusy()); CHECK(!r.axis->getErrorID());
      CHECK(std::abs(r.axis->getAxisStatusDataPtr()->currentPositionSetpoint-10.05)<1e-8);
      CHECK(std::abs(enc->getActPos()-10.05)<.002);
      ++count;
    }
    {
      scenario="pending power position sync survives disabling power control";
      Rig r(true,false,true,false,false);
      auto *enc=r.axis->getPrimEnc();
      r.ready=1; r.alarm=0; r.raw=12000;
      enc->requestPower(true);
      for(int i=0;i<100;++i) enc->readEntries(true);
      CHECK(enc->getPowerReady()); CHECK(enc->powerPositionSyncPending());
      enc->requestPower(false);
      enc->readEntries(true);
      CHECK(enc->getPowerOff()); CHECK(enc->powerPositionSyncPending());
      CHECK(enc->setPowerEnable(false)==0);
      CHECK(r.axis->setRealTimeStarted(true)==0);
      for(int i=0;i<100;++i) r.axis->execute(true);
      CHECK(!enc->powerPositionSyncPending()); CHECK(!r.axis->getErrorID());
      CHECK(!r.axis->getInStartupPhase()); CHECK(std::abs(enc->getActPos()-12)<1e-8);
      // Re-enable power control after ordinary operation: the next motion must
      // still wake and wait for feedback, with no stale cached membership.
      CHECK(enc->setPowerEnable(true)==0);
      r.physical=12;
      CHECK(r.axis->moveAbsolutePosition(12.05,.2,1,1)==0);
      r.waitMotion(); r.finishMove();
      CHECK(!r.axis->getErrorID()); CHECK(enc->getPowerReady());
      ++count;
    }
    std::cout<<"Encoder relay power: "<<count<<" production axis/encoder scenarios passed.\n";
  } catch(const std::exception &error) { std::cerr<<error.what()<<'\n'; return 1; }
}
