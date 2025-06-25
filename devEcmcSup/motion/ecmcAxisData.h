/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAxisData.h
*
*  Created on: Jan 24, 2017
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef SRC_ECMCAXISDATA_H_
#define SRC_ECMCAXISDATA_H_

#include <stdio.h>
#include "string.h"
#include "ecmcDefinitions.h"
#include "ecmcAsynDataItem.h"

typedef struct {
  bool enableCmd          : 1;
  bool executeCmd         : 1;
  bool stopCmd            : 1;
  bool resetCmd           : 1;
  bool encSourceCmd       : 1;            // 0 = internal, 1 = plc
  bool trajSourceCmd      : 1;            // 0 = internal, 1 = plc
  bool plcEnableCmd       : 1;            // 0 = disable, 1 = enable
  bool plcCmdsAllowCmd    : 1;            // 0 = not allow, 1 = allow
  bool enableSoftLimitBwd : 1;
  bool enableSoftLimitFwd : 1;
  bool enableDbgPrintout  : 1;
  bool tweakBwdCmd        : 1;
  bool tweakFwdCmd        : 1;
  int  spareBitsCmd       : 19;
} ecmcAsynAxisControlType;

typedef struct {
  double             positionTargetAbs;
  double             positionTargetRel;
  double             velocityTarget;
  double             tweakValue;
  double             softLimitBwd;
  double             softLimitFwd;
  double             moduloRange;
  ecmcMotionModType  moduloType;
  motionCommandTypes command;
  int                cmdData;
  int                primaryEncIndex;    // used for control
  int                cspDrvEncIndex;     // encoder channel used for drive local CSP (defaults to primary)
  int                cfgEncIndex;        // Encoder currrently configured
  int                drvModeSet;
  double             accelerationTarget;
  double             decelerationTarget;
  bool               allowSourceChangeWhenEnabled;
  ecmcAsynAxisControlType controlWord_;
  ecmcAsynDataItem  *asynDataItemCtrlWord;
  driveMode  drvMode;
} ecmcAxisDataCommand;

typedef struct {
  unsigned char enable          : 1;
  unsigned char enabled         : 1;
  unsigned char execute         : 1;
  unsigned char busy            : 1;
  unsigned char attarget        : 1;
  unsigned char moving          : 1;
  unsigned char limitfwd        : 1;
  unsigned char limitbwd        : 1;
  unsigned char homeswitch      : 1;
  unsigned char homed           : 1;
  unsigned char inrealtime      : 1;
  unsigned char trajsource      : 1;
  unsigned char encsource       : 1;
  unsigned char plccmdallowed   : 1;
  unsigned char softlimfwdena   : 1;
  unsigned char softlimbwdena   : 1;
  unsigned char instartup       : 1;
  unsigned char sumilockfwd     : 1;
  unsigned char sumilockbwd     : 1;
  unsigned char softlimilockfwd : 1;
  unsigned char softlimilockbwd : 1;
  unsigned char axisType        : 1;
  unsigned char seqstate        : 4;
  unsigned char lastilock       : 6;
} ecmcAxisStatusWordType;

typedef struct {
  double  externalTrajectoryPosition;
  double  externalTrajectoryPositionOld;
  double  externalTrajectoryVelocity;
  double  externalEncoderPosition;
  double  externalEncoderPositionOld;
  double  externalEncoderVelocity;
  double  currentPositionActual;
  double  currentPositionSetpoint;
  double  currentCSPPositionSetpointOffset;  // currentPositionSetpoint plus controller error..
  double  currentTargetPosition;
  double  currentTargetPositionModulo;
  double  currentPositionSetpointOld;
  double  currentVelocityActual;
  double  currentVelocitySetpoint;
  int64_t currentVelocitySetpointRaw;
  double  currentVelocityTarget;
  int64_t currentPositionSetpointRaw;
  int64_t currentPositionActualRaw;
  double  currentvelocityFFRaw;
  double  cntrlError;
  double  cntrlOutput;
  double  cntrlOutputOld;
  double  currentAccelerationSetpoint;
  double  currentDecelerationSetpoint;
  int     command; ///#  xx
  int     cmdData; ///#  xx
  int     encoderCount;
  bool    ctrlWithinDeadband;
  bool    limitFwdFiltered;
  bool    limitBwdFiltered;
  bool    homeSwitchFiltered;
  bool    startupFinsished;
  double  distToStop;
  int     errorCode;
  int     warningCode;
  int     axisId;
  int     cycleCounter;
  axisTypes axisType;
  double  sampleTime;
  ecmcAxisStatusWordType statusWord_;  
} ecmcAxisDataStatus;

typedef struct {
  bool           hardwareInterlock;  // Interlock on external I/O
  bool           bwdLimitInterlock;
  bool           fwdLimitInterlock;
  bool           fwdSoftLimitInterlock;
  bool           bwdSoftLimitInterlock;
  bool           cntrlOutputHLTrajInterlock;
  bool           cntrlOutputHLDriveInterlock;
  bool           lagTrajInterlock;
  bool           lagDriveInterlock;
  bool           bothLimitsLowInterlock;
  bool           maxVelocityTrajInterlock;
  bool           maxVelocityDriveInterlock;
  bool           velocityDiffTrajInterlock;
  bool           velocityDiffDriveInterlock;
  bool           axisErrorStateInterlock;
  bool           noExecuteInterlock;
  bool           driveSummaryInterlock;
  bool           trajSummaryInterlockFWD;
  bool           trajSummaryInterlockBWD;
  bool           trajSummaryInterlockFWDEpics;
  bool           trajSummaryInterlockBWDEpics;
  bool           etherCatMasterInterlock;
  bool           plcInterlock;
  bool           plcInterlockFWD;
  bool           plcInterlockBWD;
  bool           encDiffInterlock;
  bool           safetyInterlock;  // can only be set and reset by plugin
  bool           analogInterlock;
  bool           stallInterlock;
  interlockTypes lastActiveInterlock;
  interlockTypes interlockStatus;
  stopMode       currStopMode;
} ecmcAxisDataInterlocks;

class ecmcAxisData {
public:
  ecmcAxisData();
  ~ecmcAxisData();
  stopMode refreshInterlocks();
  void     clearInterlocks();
  ecmcAxisDataCommand control_;
  ecmcAxisDataCommand controlOld_;  // last cycle
  ecmcAxisDataStatus status_;       // last cycle
  ecmcAxisDataStatus statusOld_;
  ecmcAxisDataInterlocks interlocks_;
  ecmcAxisDataInterlocks interlocksOld_;
  ecmcAsynDataItem *axAsynParams_[ECMC_ASYN_AX_PAR_COUNT];
private:
  int      setSummaryInterlocks();
  stopMode refreshInterlocksInternal();
};

#endif  /* SRC_ECMCAXISDATA_H_ */
