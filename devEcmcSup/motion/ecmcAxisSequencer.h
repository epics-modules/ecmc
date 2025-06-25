/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAxisSequencer.h
*
*  Created on: Jan 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ecmcAxisSequencer_H_
#define ecmcAxisSequencer_H_

#include "ecmcError.h"
#include "ecmcEncoder.h"
#include "ecmcMonitor.h"
#include "ecmcPIDController.h"
#include "ecmcTrajectoryBase.h"
#include "ecmcAxisData.h"
#include "ecmcDriveBase.h"
#include "ecmcAxisPVTSequence.h"

enum localTrajDataSource {
  ECMC_TRAJ_SRC_TRAJ = 0,
  ECMC_TRAJ_SRC_PLC  = 1,
  ECMC_TRAJ_SRC_PVT  = 2  
};

class ecmcAxisSequencer : public ecmcError {
public:
  ecmcAxisSequencer();
  ~ecmcAxisSequencer();
  void                 init(double sampleTime);
  int                  setExecute(bool execute);
  bool                 getExecute();
  void                 execute();
  void                 setCommand(motionCommandTypes command);
  motionCommandTypes   getCommand();
  void                 setCmdData(int cmdData);
  int                  getCmdData();
  void                 setTraj(ecmcTrajectoryBase *traj);
  ecmcTrajectoryBase*  getTraj();
  void                 setEnc(ecmcEncoder **encArray);
  void                 setMon(ecmcMonitor *mon);
  void                 setDrv(ecmcDriveBase *drv);
  void                 setCntrl(ecmcPIDController *con);
  bool                 getBusy();
  void                 setJogVel(double velTarget);
  double               getJogVel();
  int                  setHomeVelTowardsCam(double vel);
  int                  setHomeVelOffCam(double vel);
  double               getHomeVelTowardsCam();
  double               getHomeVelOffCam();
  void                 setHomePosition(double pos);
  double               getHomePosition();
  void                 setDefaultAcc(double acc);
  void                 setDefaultDec(double dec);
  void                 setAcc(double acc);
  void                 setDec(double dec);
  int                  setPVTObject(ecmcAxisPVTSequence* pvt);
  ecmcAxisPVTSequence* getPVTObject();
  int                  validatePVT();
  int                  setTrajDataSourceType(dataSource refSource);
  int                  setTrajDataSourceTypeInternal(
                       dataSource refSource,
                       int        force);

  // Home on hardware latch (index or external)
  // Homing will be made after <count> latches have been identified
  // only valid for certain home sequences
  void         setHomeLatchCountOffset(int count);
  void         setTargetPos(double pos);
  void         setTargetPos(double pos,
                            bool   force);
  double       getTargetPos();
  void         setTargetVel(double velTarget);
  double       getTargetVel();
  void         setJogFwd(bool jog);
  bool         getJogFwd();
  void         setJogBwd(bool jog);
  bool         getJogBwd();
  int          getSeqState();
  int          validate();
  int          setSequenceTimeout(int timeout);
  int          setExternalExecute(bool execute);
  int          setAxisDataRef(ecmcAxisData *data);
  int          setAllowMotionFunctions(bool enablePos,
                                       bool enableConstVel,
                                       bool enableHome);
  void         setHomePostMoveTargetPosition(double targetPos);
  void         setHomePostMoveEnable(double enable);
  void         setNewPositionCtrlDrvTrajBumpless(double newPosition);
  int          getAllowPos();
  int          getAllowConstVelo();
  int          getAllowHome();
  int          setAutoModeSetEntry(ecmcEcEntry *entry);
  int          setAutoModeActEntry(ecmcEcEntry *entry);
  int          setAutoModeHomigCmd(int homing);
  int          setAutoModeMotionCmd(int motion);

private:
  void         executeInternal();
  ecmcEncoder* getPrimEnc();
  ecmcEncoder* getCSPEnc();

  void         initVars();
  double       checkSoftLimits(double posSetpoint);
  void         readHomingParamsFromEnc();
  bool         autoModeSetHoming();
  bool         autoModeSetMotion();
  int          seqHoming1(); // nCmdData==1
  int          seqHoming2(); // nCmdData==2
  int          seqHoming3(); // nCmdData==3
  int          seqHoming4(); // nCmdData==4
  int          seqHoming5(); // nCmdData==5
  int          seqHoming6(); // nCmdData==6
  int          seqHoming7(); // nCmdData==7
  int          seqHoming8(); // nCmdData==8
  int          seqHoming9(); // nCmdData==9
  int          seqHoming10(); // nCmdData==10
  int          seqHoming11(); // nCmdData==11
  int          seqHoming12(); // nCmdData==12
  // int    seqHoming14();  Is used but no method needed!!
  int          seqHoming15(); // nCmdData==15
  int          seqHoming21(); // nCmdData==21
  int          seqHoming22(); // nCmdData==22
  int          seqHoming26(); // nCmdData==26
  int          checkHWLimitsAndStop(bool checkBWD,
                                    bool checkFWD);
  int          stopSeq();
  int          getExtTrajSetpoint(double *pos);
  int          checkVelAccDec();
  void         initHomingSeq();
  void         finalizeHomingSeq(double newPosition);
  int          postHomeMove();
  void         setTrajAccAndDec();
  void         initStop();
  void         latchPosLagMonStateBeforeSeq();
  void         restorePosLagMonAfterSeq();

  int seqState_;
  int seqStateOld_;
  int seqTimeout_;
  int seqTimeCounter_;
  int seqPosHomeState_;
  bool hwLimitSwitchFwd_;
  bool hwLimitSwitchFwdOld_;
  bool hwLimitSwitchBwd_;
  bool hwLimitSwitchBwdOld_;
  bool homeSensor_;
  bool homeSensorOld_;
  bool seqInProgress_;
  bool seqInProgressOld_;
  bool jogFwd_;
  bool jogBwd_;
  bool executeOld_;
  bool localSeqBusy_;
  bool homeEnablePostMove_;
  double homePostMoveTargetPos_;
  double jogVel_;
  double homeVelTowardsCam_;
  double homeVelOffCam_;
  double homePosition_;
  double homePosLatch1_;
  double homePosLatch2_;
  motionDirection currSeqDirection_;
  ecmcTrajectoryBase *traj_;
  ecmcEncoder **encArray_;
  ecmcMonitor *mon_;
  ecmcPIDController *cntrl_;
  ecmcDriveBase *drv_;
  ecmcAxisData *data_;
  ecmcAxisPVTSequence *pvt_;
  uint64_t oldencRawAbsPosReg_;
  uint64_t encRawAbsPosReg_;
  ecmcOverUnderFlowType overUnderFlowLatch_;
  int homeLatchCountOffset_;
  int homeLatchCountAct_;
  bool enablePos_;
  bool enableConstVel_;
  bool enableHome_;

  // Entries for drive modes
  ecmcEcEntry *modeSetEntry_;
  ecmcEcEntry *modeActEntry_;
  int modeAct_;
  int modeMotionCmd_;  // Mode to write when running normal motion (CSP or CSV depending on how axis cfg)
  int modeHomingCmd_;  // Mode for homing
  int modeMotionCmdSet_;
  int modeHomingCmdSet_;
  bool pvtOk_;
  bool trajLock_;
  bool trajLockOld_;
  bool pvtStopping_;
  bool pvtmode_;
  int posSource_;
  int homeTrigStatOld_;
  bool monPosLagEnaStatePriorHome_;
  bool monPosLagRestoreNeeded_;
};

#endif  /* ecmcAxisSequencer_H_ */
