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

typedef struct {
  double             positionTarget;
  double             velocityTarget;
  double             softLimitBwd;
  double             softLimitFwd;
  double             moduloRange;
  ecmcMotionModType  moduloType;
  motionCommandTypes command;
  int                cmdData;
  int                primaryEncIndex;   // used for control
  int                cfgEncIndex;       // Encoder currrently configured
  int                drvModeSet;
  bool               enable             : 1;
  bool               execute            : 1;
  bool               reset              : 1;
  bool               enableSoftLimitBwd : 1;
  bool               enableSoftLimitFwd : 1;
  bool               enableDbgPrintout  : 1;
  dataSource trajSource;
  dataSource encSource;
  driveMode  drvMode;
} ecmcAxisDataCommand;

typedef struct {
  double  externalTrajectoryPosition;
  double  externalTrajectoryPositionOld;
  double  externalTrajectoryVelocity;
  double  externalEncoderPosition;
  double  externalEncoderPositionOld;
  double  externalEncoderVelocity;
  double  currentPositionActual;
  double  currentPositionSetpoint;
  double  currentTargetPosition;
  double  currentTargetPositionModulo;
  double  currentPositionSetpointOld;
  double  currentVelocityActual;
  double  currentVelocitySetpoint;
  int64_t currentVelocitySetpointRaw;
  int64_t currentPositionSetpointRaw;
  double  currentvelocityFFRaw;
  double  cntrlError;
  double  cntrlOutput;
  double  cntrlOutputOld;
  bool    enabled;
  bool    enabledOld;
  bool    enableOld;
  bool    executeOld;
  bool    busy;
  bool    busyOld;
  bool    moving;
  bool    movingOld;
  int     seqState;
  int     encoderCount;
  bool    atTarget;
  bool    ctrlWinthinDeadband;
  bool    limitFwd;
  bool    limitBwd;
  bool    limitFwdFiltered;
  bool    limitBwdFiltered;
  bool    homeSwitch;
  bool    homeSwitchFiltered;
  bool    inStartupPhase;
  bool    startupFinsished;
  bool    inRealtime;
  double  distToStop;
  int     errorCode;
  int     warningCode;
} ecmcAxisDataStatus;

typedef struct {
  bool           hardwareInterlock;  // Interlock on external I/O
  bool           bwdLimitInterlock;
  bool           fwdLimitInterlock;
  bool           unexpectedLimitSwitchBehaviourInterlock;
  bool           trajTransformInterlock;
  bool           encTransformInterlock;
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
  ecmcAxisDataCommand command_;
  ecmcAxisDataStatus status_;
  ecmcAxisDataInterlocks interlocks_;
  ecmcAxisDataInterlocks interlocksOld_;
  int axisId_;
  axisType axisType_;
  double sampleTime_;

private:
  int      setSummaryInterlocks();
  stopMode refreshInterlocksInternal();
};

#endif  /* SRC_ECMCAXISDATA_H_ */
