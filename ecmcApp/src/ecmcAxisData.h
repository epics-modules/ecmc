
#ifndef SRC_ECMCAXISDATA_H_
#define SRC_ECMCAXISDATA_H_

#include <stdio.h>
#include "ecmcDefinitions.h"
#include "string.h"

typedef struct {
  double             positionTarget;
  double             velocityTarget;
  double             softLimitBwd;
  double             softLimitFwd;
  motionCommandTypes command;
  int                cmdData;
  bool               enable;
  bool               execute;
  bool               reset;
  bool               enableSoftLimitBwd;
  bool               enableSoftLimitFwd;
  dataSource         trajSource;
  dataSource         encSource;
  operationMode      operationModeCmd;
} ecmcAxisDataCommand;

typedef struct {
  double externalTrajectoryPosition;
  double externalTrajectoryVelocity;
  double externalTrajectoryInterlock;
  double externalEncoderPosition;
  double externalEncoderVelocity;
  double externalEncoderInterlock;
  double currentPositionActual;
  double currentPositionSetpoint;
  double currentTargetPosition;
  double currentPositionSetpointOld;
  double currentVelocityActual;
  double currentVelocitySetpoint;
  int    currentVelocitySetpointRaw;
  double currentvelocityFFRaw;
  double positionError;
  double cntrlError;
  double cntrlOutput;
  double cntrlOutputOld;
  bool   enabled;
  bool   enabledOld;
  bool   enableOld;
  bool   executeOld;
  bool   busy;
  bool   busyOld;
  bool   moving;
  bool   movingOld;
  int    seqState;
  bool   atTarget;
  bool   limitFwd;
  bool   limitBwd;
  bool   limitFwdFiltered;
  bool   limitBwdFiltered;
  bool   homeSwitch;
  bool   homeSwitchFiltered;
  bool   inStartupPhase;
  bool   inRealtime;
  double distToStop;
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
  bool           etherCatMasterInterlock;
  bool           plcInterlock;
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
  int axisId_;
  axisType axisType_;
  double sampleTime_;

 private:
  int setSummaryInterlocks();
};

#endif  /* SRC_ECMCAXISDATA_H_ */
