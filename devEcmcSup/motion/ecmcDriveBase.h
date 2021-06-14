/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcDriveBase.h
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

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
#define ERROR_DRV_ASYN_PORT_OBJ_NULL 0x1460C
#define ERROR_DRV_ASYN_PRINT_TO_BUFFER_FAIL 0x1460D
#define ERROR_DRV_HW_ALARM_0 0x1460E
#define ERROR_DRV_HW_ALARM_1 0x1460F
#define ERROR_DRV_HW_ALARM_2 0x14610
#define ERROR_DRV_WARNING_READ_ENTRY_FAIL 0x14611
#define ERROR_DRV_ALARM_READ_ENTRY_FAIL 0x14612


enum ecmcDriveTypes {
  ECMC_STEPPER  = 0,
  ECMC_DS402    = 1,
  ECMC_NO_DRIVE = 2,
};

enum ecmcBrakeStates {
  ECMC_BRAKE_CLOSED  = 0,
  ECMC_BRAKE_OPENING = 1,
  ECMC_BRAKE_OPEN    = 2,
  ECMC_BRAKE_CLOSING = 3
};

class ecmcDriveBase : public ecmcEcEntryLink {
 public:
  explicit ecmcDriveBase(ecmcAsynPortDriver *asynPortDriver,
                         ecmcAxisData *axisData);
  /*ecmcDriveBase(ecmcAxisData *axisData,
                double        scale);*/
  virtual ~ecmcDriveBase();
  virtual void initVars();
  virtual int  validate();
  virtual void readEntries();
  virtual void writeEntries();
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
  // CSP
  int          setCspPosSet(double posEng);
  int          setCspRecalcOffset(double posEng);
  void         setCspActPos(int64_t posRaw, double posAct);
  void         setCspRef(int64_t posRaw, double posAct,  double posSet);
  
 protected:
  int          updateBrakeState();
  bool         driveInterlocksOK();
  int          initAsyn();
  void         refreshAsyn();
  bool enableAmpCmd_;
  bool enableAmpCmdOld_;
  double scale_;
  double scaleNum_;
  double scaleDenom_;
  double velSet_;
  double cspPosSet_;
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
  ecmcAsynPortDriver *asynPortDriver_;
  ecmcAsynDataItem  *asynControlWd_;
  ecmcAsynDataItem  *asynStatusWd_;
  int64_t cspRawActPos_;
  double cspActPos_;
  int64_t cspRawPosOffset_;
  uint64_t hwReset_;
  uint64_t hwErrorAlarm0_;
  uint64_t hwErrorAlarm0Old_;
  uint64_t hwErrorAlarm1_;
  uint64_t hwErrorAlarm1Old_;
  uint64_t hwErrorAlarm2_;
  uint64_t hwErrorAlarm2Old_;
  uint64_t hwWarning_;
  uint64_t hwWarningOld_;
  bool hwResetDefined_;
  bool hwErrorAlarm0Defined_;
  bool hwErrorAlarm1Defined_;
  bool hwErrorAlarm2Defined_;
  bool hwWarningDefined_;

};

#endif  // ifndef ECMCDRIVEBASE_H_
