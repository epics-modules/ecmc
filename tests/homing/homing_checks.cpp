// Production axis/sequencer/encoder/monitor/trajectory; ideal velocity follower.
// No EPICS motor record, drive electronics, network, PID dynamics or RT thread.
#include "ecmcAxisVirt.h"
#include "ecmcDataAccess.h"
#include "ecmcDataItem.h"
#include "ecmcPositionCompare.h"
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
asynUser *pPrintOutAsynUser = nullptr;
unsigned int debug_print_flags = 0, die_on_error_flags = 0;
void ecmcRtLoggerLogError(const char *, ...) {}
static std::string context;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(context + ": line " + std::to_string(__LINE__) + ": " #x); } while (0)
struct Bench {
  ecmcAsynPortDriver port;
  std::unique_ptr<ecmcAxisVirt> axis;
  uint32_t raw=0;
  int healthy=1;
  ecmcDataItem item{"test.counter"};
  double physical=0;
  bool absolute=false, cam=true;
  Bench(bool abs, double start) : physical(start), absolute(abs) {
    axis=std::make_unique<ecmcAxisVirt>(&port,1,.001,ECMC_TRAPETZ);
    auto *enc=axis->getPrimEnc();
    item.setEcmcDataPointer(reinterpret_cast<uint8_t *>(&raw),sizeof(raw));
    item.setEcmcDataType(ECMC_EC_U32); item.setAllowWriteToEcmc(false);
    ecmcDataAccess access;
    CHECK(ecmcDataAccess::bind(item,&healthy,false,&access));
    CHECK(enc->setDataAtIndex(access,ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION,-1)==0);
    CHECK(enc->setBits(abs ? 32 : 16)==0);
    CHECK(enc->setAbsBits(abs ? 26 : 0)==0);
    CHECK(enc->setType(abs ? ECMC_ENCODER_TYPE_ABSOLUTE : ECMC_ENCODER_TYPE_INCREMENTAL)==0);
    CHECK(enc->setScaleNum(1)==0); CHECK(enc->setScaleDenom(abs ? 4096 : 12800)==0);
    CHECK(enc->setPosFilterEnable(false)==0);
    enc->setHomeSeqId(1);
    enc->setHomeVelTowardsCam(.2); enc->setHomeVelOffCam(.1);
    enc->setHomeAcc(1); enc->setHomeDec(1);
    enc->setHomePosition(0); enc->setHomeLatchCountOffset(1);
    CHECK(axis->setEncDataSourceType(ECMC_DATA_SOURCE_INTERNAL)==0);
    axis->setAllowMotionFunctions(true,true,true);
    axis->setAcc(1); axis->setDec(1);
    auto *mon=axis->getMon();
    mon->setLimitSwitchBwdPLCOverride(true); mon->setLimitSwitchFwdPLCOverride(true);
    mon->setHomeSwitchPLCOverride(true);
    mon->setAtTargetTol(.01); mon->setAtTargetTime(100); mon->setEnableAtTargetMon(true);
    mon->setPosLagTol(.02); mon->setPosLagTime(100); mon->setEnableLagMon(true);
    CHECK(axis->validate()==0);
    for(int i=0;i<10;++i) tick();
    CHECK(!axis->getInStartupPhase());
    CHECK(!axis->getAxisStatusDataPtr()->statusWord_.instartup);
    CHECK(!axis->getAutoExecuteOnTargetPos());
    CHECK(axis->setPosition(start)==0);
    CHECK(axis->setEnable(true)==0);
    for(int i=0;i<10;++i) tick();
    CHECK(axis->getErrorID()==0);
  }
  void tick() {
    physical+=axis->getAxisStatusDataPtr()->currentVelocitySetpoint*.001;
    CHECK(std::abs(physical)<.8);
    raw=absolute ? uint32_t(std::llround((physical+1380)*4096)) : uint16_t(std::llround(physical*12800));
    auto *mon=axis->getMon();
    mon->setLimitSwitchBwdPLCOverrideValue(physical>-.4);
    mon->setLimitSwitchFwdPLCOverrideValue(physical<.4);
    mon->setHomeSwitchPLCOverrideValue(cam && physical>=-.08 && physical<=.08);
    axis->execute(true);
  }
  void start(int seq) {
    CHECK(axis->moveHome(seq,0,.2,.1,1,1)==0);
    CHECK(axis->getBusy());
    // Explicitly enabled fixture: global Busy must not hide sequence gaps.
    CHECK(!axis->getAxisStatusDataPtr()->statusWord_.globalBusy);
  }
  void finish() {
    for(int i=0;i<15000;++i) {
      tick(); CHECK(axis->getErrorID()==0);
      if(axis->getSeq()->getSeqState()!=0) CHECK(axis->getBusy());
      if(i>2 && !axis->getBusy() && axis->getPrimEnc()->getHomed()) return;
    }
    CHECK(!axis->getBusy()); CHECK(axis->getPrimEnc()->getHomed());
  }
};
static void checkTimedPositionCompare() {
  ecmcPositionCompare compare;
  uint64_t output = 0, activate = 3, startTime = 0;
  auto link = [&](uint64_t &word, int index) {
    ecmcDataAccess access;
    access.data = reinterpret_cast<uint8_t *>(&word);
    access.type = ECMC_EC_U64;
    access.bits = 64;
    access.writable = true;
    access.validation = ecmcDataAccess::DIRECT_STATUS;
    CHECK(compare.setDataAtIndex(access, index, -1) == 0);
  };
  link(output, ECMC_POS_COMPARE_ENTRY_OUTPUT);
  link(activate, ECMC_POS_COMPARE_ENTRY_ACTIVATE);
  link(startTime, ECMC_POS_COMPARE_ENTRY_START_TIME);
  CHECK(compare.configure(20000000, 100000000, 3, 0, 200000000, 0) == 0);
  CHECK(compare.validate() == 0);

  // The terminal latches a scheduled output on an idle-to-schedule edge.
  // Advance both the simulated sample clock and the output clock by 1 ms.
  constexpr uint64_t epoch = 1000000000;
  uint64_t lastActivate = activate, pendingTime = 0, pendingOutput = 0, level = 0;
  uint64_t rise = 0, fall = 0;
  int writes = 0, edges = 0;
  auto tick = [&](int ms) {
    const uint64_t now = epoch + uint64_t(ms) * 1000000;
    if (pendingTime && now >= pendingTime) {
      CHECK(pendingOutput != level);
      level = pendingOutput;
      if (level) rise = now;
      else fall = now;
      ++edges;
      pendingTime = 0;
    }
    compare.execute(true, ms * .00025, .25, 0., now, true, now);
    if (lastActivate == 3 && activate == 0) {
      CHECK(!pendingTime && startTime > now);
      pendingTime = startTime;
      pendingOutput = output;
      ++writes;
    }
    lastActivate = activate;
  };

  CHECK(compare.arm(.125, 1, 1) == 0);
  for (int ms = 0; ms <= 800; ++ms) tick(ms);
  CHECK(compare.getStatus().state == ECMC_POS_COMPARE_FIRED);
  CHECK(compare.getStatus().sequence == 1 && writes == 2 && edges == 2);
  CHECK(rise == epoch + 500000000 && fall == rise + 200000000);
  CHECK(level == 0 && activate == 3);

  CHECK(compare.arm(.25, 1, 1) == 0);
  tick(801);
  CHECK(compare.cancel() == 0);
  for (int ms = 802; ms <= 1200; ++ms) tick(ms);
  CHECK(compare.getStatus().state == ECMC_POS_COMPARE_DISABLED);
  CHECK(compare.getStatus().sequence == 1 && writes == 2 && edges == 2 && level == 0);
}

int main() {
  try {
    int scenarios=0;
    context="position compare with a valid simulated clock";
    checkTimedPositionCompare(); ++scenarios;
    context="homing settling: trajectory idle but sequence busy";
    {
      Bench b(false,0);
      b.start(1);
      bool settling=false;
      for(int i=0;i<15000;++i) {
        b.tick(); CHECK(b.axis->getErrorID()==0);
        if(b.axis->getSeq()->getSeqState()==4 &&
           !b.axis->getAxisStatusDataPtr()->statusWord_.localBusy) {
          settling=true;
          CHECK(b.axis->getBusy());
          CHECK(!b.axis->getAxisStatusDataPtr()->statusWord_.globalBusy);
        }
        if(i>2 && !b.axis->getBusy() && b.axis->getPrimEnc()->getHomed()) break;
      }
      CHECK(settling);
      CHECK(b.axis->getPrimEnc()->getHomed());
      CHECK(!b.axis->getBusy());
      ++scenarios;
    }
    context="ignored moves preserve homing";
    {
      Bench b(false, 0);
      CHECK(!b.axis->ignoreMotionCommand(ECMC_CMD_MOVEABS));
      b.axis->setAllowMotionFunctions(false, false, true);
      b.axis->setIgnoreDisabledMotionCommands(1);
      b.start(1);
      for (int i=0; i<100; ++i) {
        CHECK(b.axis->moveAbsolutePosition(100, 1, 1, 1)==0);
        CHECK(b.axis->moveRelativePosition(100, 1, 1, 1)==0);
        CHECK(b.axis->moveVelocity(1, 1, 1)==0);
        CHECK(b.axis->getCommand()==ECMC_CMD_HOMING);
        CHECK(b.axis->getExecute());
        CHECK(b.axis->getErrorID()==0);
        b.tick();
      }
      CHECK(b.axis->ignoredMotionCommandCount_ == 300);
      CHECK(b.axis->lastIgnoredMotionCommand_ == ECMC_CMD_MOVEVEL);
      CHECK(b.axis->ignoredMotionMessage_ == ECMC_CMD_MOVEVEL);
      b.finish();
      b.axis->setAllowMotionFunctions(true, true, true);
      b.tick();
      CHECK(b.axis->ignoredMotionMessage_ == ECMC_CMD_NOCMD);
      CHECK(b.axis->ignoredMotionCommandCount_ == 300);
      b.axis->setAllowMotionFunctions(false, false, true);
      CHECK(b.axis->moveVelocity(1, 1, 1) == 0);
      CHECK(b.axis->ignoredMotionMessage_ == ECMC_CMD_MOVEVEL);
      b.start(1);
      CHECK(b.axis->ignoredMotionMessage_ == ECMC_CMD_NOCMD);
      CHECK(b.axis->stopMotion(1)==0);
      CHECK(!b.axis->getEnable());
    }

    for(bool absolute : {false,true}) for(int seq : {1,2,3,4,5,6,7,8,9,10,14,15,21,22}) {
      if(!absolute && (seq==21 || seq==22)) continue;
      context="home="+std::to_string(seq)+" abs="+std::to_string(absolute);
      double start=(seq==7 || seq==9) ? .25 : ((seq==8 || seq==10) ? -.25 : (seq==15 ? .2 : 0));
      Bench b(absolute,start); b.start(seq);
      // Encoder-configured request 14 resolves only in sequencer status;
      // the request-backed asyn PV must not be confused with this value.
      CHECK(b.axis->getSeq()->getCmdData()==(seq==14 ? 1 : seq));
      b.finish();
      double actual=b.axis->getPrimEnc()->getActPos();
      if(seq==15) { CHECK(std::abs(b.physical-.2)<.001); CHECK(std::abs(actual)<.001); }
      else if(seq==21 || seq==22) {
        double phase=b.physical+1380;
        CHECK(std::abs(actual-(seq==21 ? phase-16384 : phase))<.005);
      } else {
        double edge=(seq==1 || seq==14) ? -.4 : (seq==2 ? .4 :
          ((seq==3 || seq==8) ? -.08 : ((seq==4 || seq==7) ? .08 : 0)));
        CHECK(std::abs(b.physical-actual-edge)<.005);
      }
      ++scenarios;
    }
    for(bool absolute : {false,true}) {
      context="post-home move abs="+std::to_string(absolute);
      Bench b(absolute,0);
      b.axis->getPrimEnc()->setHomePostMoveTargetPosition(.2);
      b.axis->getPrimEnc()->setHomePostMoveEnable(1);
      b.start(14);
      bool postMove=false, postMotion=false;
      for(int i=0;i<15000;++i) {
        b.tick(); CHECK(b.axis->getErrorID()==0);
        if(b.axis->getSeq()->getSeqState()!=0) CHECK(b.axis->getBusy());
        if(b.axis->getSeq()->getSeqState()==1003) {
          postMove=true;
          // Busy must span the entire phase, including before the first
          // trajectory sample and during final settling, not just travel.
          CHECK(b.axis->getBusy());
          if(std::abs(b.axis->getAxisStatusDataPtr()->currentVelocitySetpoint)>.001) {
            postMotion=true; CHECK(b.axis->getBusy());
          }
          CHECK(b.axis->getPrimEnc()->getHomed());
        }
        if(i>2 && !b.axis->getBusy() && b.axis->getPrimEnc()->getHomed()
           && b.axis->getSeq()->getSeqState()==0) break;
      }
      if(!postMove) std::cerr << "post state=" << b.axis->getSeq()->getSeqState() << " busy=" << b.axis->getBusy() << " physical=" << b.physical << " actual=" << b.axis->getPrimEnc()->getActPos() << "\n";
      CHECK(postMove); CHECK(postMotion); CHECK(!b.axis->getBusy());
      CHECK(b.axis->getPrimEnc()->getHomed());
      CHECK(std::abs(b.axis->getPrimEnc()->getActPos()-.2)<.005);
      CHECK(std::abs(b.physical+.2)<.005);
      ++scenarios;
    }
    for(bool absolute : {false,true}) {
      for(int mode=0; mode<4; ++mode) {
        const int sequence=mode==1 ? 3 : (mode==2 ? 9 : 1);
        const int phase=mode==1 ? 3 : (mode==2 ? 2 : 1);
        context="abort="+std::to_string(mode)+" abs="+std::to_string(absolute);
        Bench b(absolute,sequence==9 ? .25 : 0);
        b.start(sequence);
        bool reached=false;
        for(int i=0;i<10000;++i) {
          b.tick(); CHECK(b.axis->getErrorID()==0);
          if(b.axis->getSeq()->getSeqState()==phase) { reached=true; break; }
        }
        CHECK(reached); CHECK(!b.axis->getPrimEnc()->getHomed());
        CHECK(b.axis->stopMotion(mode==3)==0);
        for(int i=0;i<1000;++i) b.tick();
        CHECK(!b.axis->getBusy()); CHECK(!b.axis->getPrimEnc()->getHomed());
        CHECK(b.axis->getSeq()->getSeqState()==0);
        CHECK(b.axis->getMon()->getEnableLagMon());
        const double stopped=b.physical;
        for(int i=0;i<1000;++i) b.tick();
        CHECK(std::abs(b.physical-stopped)<.001);
        CHECK(!b.axis->getPrimEnc()->getHomed());
        CHECK(b.axis->setEnable(true)==0);
        for(int i=0;i<10;++i) b.tick();
        b.start(1); b.finish();
        ++scenarios;
      }
      context="missing cam abs="+std::to_string(absolute);
      Bench b(absolute,.25); b.cam=false; b.start(7);
      for(int i=0;i<10000 && !b.axis->getErrorID();++i) b.tick();
      CHECK(b.axis->getErrorID()==ERROR_SEQ_SEQ_FAILED);
      CHECK(!b.axis->getPrimEnc()->getHomed());
      for(int i=0;i<1000;++i) b.tick();
      CHECK(!b.axis->getBusy()); CHECK(b.axis->getSeq()->getSeqState()==0);
      CHECK(b.axis->getMon()->getEnableLagMon());
      ++scenarios;
    }
    std::cout<<"Production homing: "<<scenarios<<" success/abort scenarios passed.\n";
  } catch(const std::exception &e) { std::cerr<<e.what()<<"\n"; return 1; }
}
