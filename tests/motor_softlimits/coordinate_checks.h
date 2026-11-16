// Extracted ESS motorRecord.cc and motorDevSup.c run against real ECMC methods.
// Field storage, transaction routing and event/LVIO publication are adapters.
// This does not execute a whole IOC or the motor motion-processing function.
#include <cstring>
struct motor_private {
  struct { bool motorROdialLimitsValid = false;
    bool motorRWdialLimitsValid = false;
    double motorDialLowLimit = -10, motorDialHighLimit = 10; } motorSoftLimits;
  struct { double val = 0, dval = 0, motorHighLimitRaw = 0, motorLowLimitRaw = 0; } last;
};
struct motorAsynPvt {
  struct { struct { double motorHighLimitRaw = 0, motorLowLimitRaw = 0; } MotorSoftLimits; } status;
};
struct motorRecord {
  double off = 0, mres = 1, dllm = -10, dhlm = 10, llm = -10, hlm = 10;
  double rhlm = 0, rllm = 0, val = 0, dval = 0, rbv = 0, drbv = 0;
  int dir = 0, foff = 1, mflg = 8;
  void *dset = nullptr;
  void *dpvt = nullptr;
  motor_private *priv = nullptr;
  Bench *bench = nullptr;
};
using RTN_STATUS = int;
enum motor_cmnd { SET_LOW_LIMIT, SET_HIGH_LIMIT };
enum { motorDIR_Pos, motorDIR_Neg };
enum { motorRecordOFF, motorRecordDIR };
constexpr int MF_DRIVER_USES_EGU = 8, OK = 0, TRUE = 1, FALSE = 0;
#define Debug(...) ((void)0)
#define MARK(...) ((void)0)
#define MARK_AUX(...) ((void)0)
static void recalcLVIO(motorRecord *) {} // Motion-record LVIO is outside this fixture.
struct motor_dset {
  int (*start_trans)(motorRecord *);
  int (*build_trans)(motor_cmnd, double *, motorRecord *);
  int (*end_trans)(motorRecord *);
};
static int start(motorRecord *) { return OK; }
static int send(motorRecord *) { return OK; }
static int route(motor_cmnd cmd, double *value, motorRecord *record) {
  // Transaction endpoint: actual coordinate conversion has already happened.
  return cmd == SET_HIGH_LIMIT ? record->bench->motor.setHighLimit(*value) :
                                record->bench->motor.setLowLimit(*value);
}
#define INIT_MSG() (*pdset->start_trans)(pmr)
#define WRITE_MSG(cmd,parms) (*pdset->build_trans)((cmd), (parms), pmr)
#define SEND_MSG() (*pdset->end_trans)(pmr)
static void set_dial_highlimit(motorRecord *);
static void set_dial_lowlimit(motorRecord *);
#include "motor_record_methods.h"

#ifdef ECMC_TEST_REAL_MOTOR_RECORD
static void coordinateBoundary() {
  // DOC: user = direction * dial + OFF (motor record documentation).
  // Expected user ranges are explicit independently calculated test vectors.
  struct Vector { double off; int dir; double low, high; };
  const Vector vectors[] = {{0, motorDIR_Pos, -4, 6},
    {100, motorDIR_Pos, 96, 106}, {-100, motorDIR_Pos, -104, -94},
    {0, motorDIR_Neg, -6, 4}, {100, motorDIR_Neg, 94, 104},
    {-100, motorDIR_Neg, -106, -96}};
  int cases = 0;
  motor_dset transactions{start, route, send};
  for (const auto &v : vectors) for (double resolution : {0.1, 1., 2.5, -0.1})
    for (bool sync : {false, true}) {
      context = "actual motor coordinate OFF=" + std::to_string(v.off) +
        " DIR=" + std::to_string(v.dir) + " MRES=" + std::to_string(resolution) +
        " sync=" + std::to_string(sync);
      ++cases;
      Bench b; b.ecmcLimits(-10, 10); b.motorFields(-10, 10);
      b.controller.values[Controller::motorFlagsRwSoftLimits_] = sync;
      motor_private priv;
      motorRecord rec;
      rec.bench = &b; rec.priv = &priv; rec.dset = &transactions;
      rec.off = v.off; rec.dir = v.dir; rec.mres = resolution;
      rec.llm = v.low; set_user_lowlimit(&rec);
      rec.hlm = v.high; set_user_highlimit(&rec);
      CHECK(rec.dllm == -4 && rec.dhlm == 6);
      CHECK(rec.llm == v.low && rec.hlm == v.high);
      b.checkEcmc(sync ? (resolution < 0 ? -6 : -4) : -10,
                  sync ? (resolution < 0 ? 4 : 6) : 10);
      // Real dial-to-native conversion changes sign, never MRES magnitude.
      CHECK(b.motor.forwardedHigh == (resolution < 0 ? 4 : 6));
      CHECK(b.motor.forwardedLow == (resolution < 0 ? -6 : -4));
      const int calls = b.motor.highCalls + b.motor.lowCalls;
      rec.dval = 2; rec.drbv = 3;
      priv.last.dval = 1;
      rec.off += 17;
      coordinateSpecial(&rec, motorRecordOFF);
      CHECK(rec.llm == v.low + 17 && rec.hlm == v.high + 17);
      CHECK(rec.dllm == -4 && rec.dhlm == 6);
      CHECK(rec.val == rec.off + (v.dir == motorDIR_Pos ? 2 : -2));
      CHECK(priv.last.val == rec.off + (v.dir == motorDIR_Pos ? 1 : -1));
      // Changing OFF must not send a new native limit or alter ECMC's range.
      CHECK(b.motor.highCalls + b.motor.lowCalls == calls);
      b.checkEcmc(sync ? (resolution < 0 ? -6 : -4) : -10,
                  sync ? (resolution < 0 ? 4 : 6) : 10);
      rec.dir = v.dir == motorDIR_Pos ? motorDIR_Neg : motorDIR_Pos;
      coordinateSpecial(&rec, motorRecordDIR); // frozen OFF
      CHECK(rec.dllm == -4 && rec.dhlm == 6);
      CHECK(rec.llm == rec.off + (v.dir == motorDIR_Pos ? -6 : -4));
      CHECK(rec.hlm == rec.off + (v.dir == motorDIR_Pos ? 4 : 6));
      CHECK(b.motor.highCalls + b.motor.lowCalls == calls);
      const double userBefore = rec.val, offsetBefore = rec.off;
      rec.foff = 0;
      rec.dir = v.dir;
      coordinateSpecial(&rec, motorRecordDIR); // variable OFF preserves VAL
      CHECK(rec.val == userBefore);
      CHECK(rec.off == offsetBefore + (v.dir == motorDIR_Pos ? -4 : 4));
      CHECK(rec.dllm == -4 && rec.dhlm == 6);
      CHECK(b.motor.highCalls + b.motor.lowCalls == calls);
    }
  // Actual record clamping when driver RO dial limits are valid, regardless
  // of whether a command propagates back to ECMC. Covers scenario 3's wording.
  for (bool validReadback : {false, true}) for (const auto &v : vectors) {
    context = "actual motor range restriction, RO valid=" + std::to_string(validReadback);
    ++cases;
    Bench b; b.ecmcLimits(-10, 10);
    motor_private priv; priv.motorSoftLimits.motorROdialLimitsValid = validReadback;
    motorRecord rec; rec.priv = &priv; rec.bench = &b; rec.dset = &transactions;
    rec.off = v.off; rec.dir = v.dir;
    rec.hlm = v.off + 20; set_user_highlimit(&rec);
    rec.llm = v.off - 20; set_user_lowlimit(&rec);
    CHECK(rec.dllm == (validReadback ? -10 : -20));
    CHECK(rec.dhlm == (validReadback ? 10 : 20));
    b.checkEcmc(-10, 10);
  }
  for (const auto &v : vectors) {
    context = "zero dial sentinel with OFF and DIR";
    ++cases;
    motor_private priv;
    motorRecord rec; rec.priv = &priv; rec.off = v.off; rec.dir = v.dir;
    rec.dllm = rec.dhlm = 0;
    set_userlimits(&rec);
    CHECK(rec.llm == 0 && rec.hlm == 0);
    rec.off += 17; coordinateSpecial(&rec, motorRecordOFF);
    CHECK(rec.llm == 0 && rec.hlm == 0);
  }
  // Actual device-support interpretation without synchronization: the
  // writable seed is one-time (COMPAT), while enabled ECMC limits still bound
  // the motor record through the read-only dial range.
  for (const auto &v : vectors) {
    context = "startup read-only limits with OFF=" + std::to_string(v.off);
    ++cases;
    Bench b; b.ecmcLimits(-10, 10); b.motorFields(-4, 6);
    b.motor.updateFirstPollDone_ = false;
    CHECK(b.motor.initializeSoftLimits() == asynSuccess);
    CHECK(b.controller.values.at(Controller::motorLowLimitRO_) == -10);
    CHECK(b.controller.values.at(Controller::motorHighLimitRO_) == 10);
    CHECK(b.controller.values.at(Controller::motorLowLimit_) == -10);
    CHECK(b.controller.values.at(Controller::motorHighLimit_) == 10);
    motor_private priv;
    motorAsynPvt device;
    device.status.MotorSoftLimits.motorLowLimitRaw =
      b.controller.values.at(Controller::motorLowLimitRO_);
    device.status.MotorSoftLimits.motorHighLimitRaw =
      b.controller.values.at(Controller::motorHighLimitRO_);
    motorRecord rec;
    rec.bench = &b; rec.priv = &priv; rec.dset = &transactions; rec.dpvt = &device;
    rec.off = v.off; rec.dir = v.dir;
    new_motor_soft_limits(&rec, 0); // sync/MF_RW_SOFT_LIMITS disabled
    CHECK(priv.motorSoftLimits.motorROdialLimitsValid);
    // The record clamps independent motor limits to the enabled ECMC range.
    rec.hlm = v.off + 20; set_user_highlimit(&rec);
    rec.llm = v.off - 20; set_user_lowlimit(&rec);
    CHECK(rec.dllm == -10 && rec.dhlm == 10);
    b.checkEcmc(-10, 10);
    // The zero pair disables the motor limits without touching ECMC.
    rec.dllm = rec.dhlm = 0; set_userlimits(&rec);
    CHECK(rec.llm == 0 && rec.hlm == 0);
    // Runtime ECMC edits stay independent and update the bounding RO range.
    b.ecmcLimits(-8, 9); b.pollLimits();
    CHECK(b.controller.values.at(Controller::motorLowLimitRO_) == -8);
    CHECK(b.controller.values.at(Controller::motorHighLimitRO_) == 9);
    b.monitorTarget(9.5, false, true);
    b.checkEcmc(-8, 9);
  }
  std::cout << "Actual ESS motor OFF/DIR/limit processing: " << cases << " cases passed.\n";
}
#else
static void coordinateBoundary() {
  std::cout << "SKIP actual motor-record coordinates (ESS motor source not provided).\n";
}
#endif
