#ifndef ECMCDRIVEBASE_H_
#define ECMCDRIVEBASE_H_

#include <stdio.h>
#include <cmath>

#include "../main/ecmcDefinitions.h"
#include "../main/ecmcError.h"
#include "../ethercat/ecmcEcEntry.h"
#include "../ethercat/ecmcEcEntryLink.h"
#include "../ethercat/ecmcEcPdo.h"
#include "ecmcAxisData.h"

// DRIVE
#define ERROR_DRV_DRIVE_INTERLOCKED 0x14600
#define ERROR_DRV_ASSIGN_ENTRY_FAILED 0x14601
#define ERROR_DRV_SCALE_DENOM_ZERO 0x14602
#define ERROR_DRV_ENABLE_ENTRY_NULL 0x14603
#define ERROR_DRV_VEL_SET_ENTRY_NULL 0x14604
#define ERROR_DRV_ENABLED_ENTRY_NULL 0x14605
#define ERROR_DRV_ENABLED_READ_ENTRY_FAIL 0x14606
#define ERROR_DRV_BRAKE_ENTRY_NULL  0x14607
#define ERROR_DRV_REDUCE_TORQUE_ENTRY_NULL  0x14608
#define ERROR_DRV_COMMAND_NOT_ALLOWED_IN_AUTO_MODE 0x14609
#define ERROR_DRV_BRAKE_OPEN_DELAY_TIME_INVALID 0x1460A
#define ERROR_DRV_BRAKE_CLOSE_AHEAD_TIME_INVALID 0x1460B

enum ecmcDriveTypes {
  ECMC_STEPPER = 0,
  ECMC_DS402   = 1,
};

enum ecmcBrakeStates {
  ECMC_BRAKE_CLOSED  = 0,
  ECMC_BRAKE_OPENING = 1,
  ECMC_BRAKE_OPEN    = 2,
  ECMC_BRAKE_CLOSING = 3
};

class ecmcDriveBase : public ecmcEcEntryLink {
 public:
  explicit ecmcDriveBase(ecmcAxisData *axisData);
  ecmcDriveBase(ecmcAxisData *axisData,
                double        scale);
  virtual ~ecmcDriveBase();
  virtual void initVars();
  virtual int  validate();
  virtual void readEntries();
  virtual void writeEntries();
  virtual void printCurrentState();
  virtual void errorReset();
  int          setEnable(bool enable);
  bool         getEnable();
  bool         getEnabled();
  double       getScaleNum(void);
  void         setScaleNum(double scaleNum);
  int          setScaleDenom(double scaleDenom);
  double       getScale();
  int          setVelSet(double vel);
  double       getVelSet();
  int          setVelSetRaw(int rawVel);
  int          getVelSetRaw();
  int          setEnableBrake(bool enable);
  int          setEnableReduceTorque(bool enable);
  int          getEnableBrake();
  int          getEnableReduceTorque();
  int          setAxisDataRef(ecmcAxisData *data);
  int          setBrakeOpenDelayTime(int delayTime);
  int          setBrakeCloseAheadTime(int aheadTime);

 protected:
  int          updateBrakeState();
  bool         driveInterlocksOK();
  bool enableAmpCmd_;
  bool enableAmpCmdOld_;
  double scale_;
  double scaleNum_;
  double scaleDenom_;
  double velSet_;
  uint64_t controlWord_;
  uint64_t statusWord_;
  ecmcAxisData *data_;

 private:
  bool manualModeEnableAmpCmd_;
  bool manualModeEnableAmpCmdOld_;
  int brakeOpenDelayTime_;
  int brakeCloseAheadTime_;
  bool brakeOutputCmdOld_;
  bool reduceTorqueOutputCmdOld_;
  bool enableBrake_;
  bool enableReduceTorque_;
  bool brakeOutputCmd_;
  bool reduceTorqueOutputCmd_;
  ecmcBrakeStates brakeState_;
  int brakeCounter_;
  operationMode opeationMode_;
  bool enableCmdOld_;
};
#endif  // ifndef ECMCDRIVEBASE_H_
