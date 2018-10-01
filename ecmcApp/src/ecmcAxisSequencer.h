/*
 * cMcuSequencer.h
 *
 *  Created on: Jan 14, 2016
 *      Author: anderssandstrom
 */

#ifndef ecmcAxisSequencer_H_
#define ecmcAxisSequencer_H_
#include "ecmcEncoder.h"
#include "ecmcError.h"
#include "ecmcMonitor.hpp"
#include "ecmcPIDController.hpp"
#include "ecmcTrajectoryTrapetz.hpp"
#include "ecmcMasterSlaveIF.h"
#include "ecmcAxisData.h"

//SEQUENCER ERRORS
#define ERROR_SEQ_TRAJ_NULL 0x14D00
#define ERROR_SEQ_ENC_NULL 0x14D01
#define ERROR_SEQ_MON_NULL 0x14D02
#define ERROR_SEQ_CNTRL_NULL 0x14D03
#define ERROR_SEQ_SEQ_FAILED 0x14D04
#define ERROR_SEQ_COMMAND_NOT_SUPPORTED 0x14D05
#define ERROR_SEQ_SOFT_LIMIT_FWD 0x14D06
#define ERROR_SEQ_SOFT_LIMIT_BWD 0x14D07
#define ERROR_SEQ_TIMEOUT 0x14D08
#define ERROR_SEQ_CMD_UNDEFINED 0x14D09
#define ERROR_SEQ_CMD_DATA_UNDEFINED 0x14D0A
#define ERROR_SEQ_EXTERNAL_DATA_INTERFACE_NULL 0x14D0B
#define ERROR_SEQ_NO_HOME_SWITCH_FLANK 0x14D0C
#define ERROR_SEQ_NO_SECOND_HOME_SWITCH_FLANK 0x14D0D
#define ERROR_SEQ_ERROR_ABS_BIT_COUNT_ZERO 0x14D0E
#define ERROR_SEQ_ERROR_POSITION_SANITY_CHECK_FAILED 0x14D0F


//Homing
enum ecmcHomingType{
  ECMC_SEQ_HOME_NOT_VALID = 0,
  ECMC_SEQ_HOME_LOW_LIM = 1,
  ECMC_SEQ_HOME_HIGH_LIM = 2,
  ECMC_SEQ_HOME_LOW_LIM_HOME = 3,
  ECMC_SEQ_HOME_HIGH_LIM_HOME = 4,
  ECMC_SEQ_HOME_LOW_LIM_HOME_HOME = 5,
  ECMC_SEQ_HOME_HIGH_LIM_HOME_HOME = 6,
  ECMC_SEQ_HOME_SET_POS = 15,
  ECMC_SEQ_HOME_LOW_LIM_SINGLE_TURN_ABS = 21,
  ECMC_SEQ_HOME_HIGH_LIM_SINGLE_TURN_ABS = 22,
};

class ecmcAxisSequencer : public ecmcError
{
public:
  ecmcAxisSequencer();
  ~ecmcAxisSequencer();
  int    setExecute(bool execute);
  bool   getExecute();
  void   execute();
  void   setCommand(motionCommandTypes command);
  motionCommandTypes getCommand();
  void   setCmdData(int cmdData);
  int    getCmdData();
  void   setTraj(ecmcTrajectoryTrapetz *traj);
  ecmcTrajectoryTrapetz *getTraj();
  void   setEnc(ecmcEncoder *enc);
  void   setMon(ecmcMonitor *mon);
  void   setCntrl(ecmcPIDController *con);
  bool   getBusy();
  void   setJogVel(double velTarget);
  double getJogVel();
  int    setHomeVelTwordsCam(double vel);
  int    setHomeVelOffCam(double vel);
  double getHomeVelTwordsCam();
  double getHomeVelOffCam();
  int    setHomeDir(motionDirection dir);
  motionDirection getHomeDir();
  void   setHomePosition(double pos);
  double getHomePosition();
  void   setTargetPos(double pos);
  void   setTargetPos(double pos, bool force);
  double getTargetPos();
  void   setTargetVel(double velTarget);
  double getTargetVel();
  void   setJogFwd(bool jog);
  bool   getJogFwd();
  void   setJogBwd(bool jog);
  bool   getJogBwd();
  int    getSeqState();
  int    validate();
  int    setSequenceTimeout(int timeout);
  int    setExternalExecute(bool execute);
  int    setExtTrajIF(ecmcMasterSlaveIF *extIf);
  int    setAxisDataRef(ecmcAxisData* data);
  void   printCurrentState();
  void   printHomeDirection();
private:
  void   initVars();
  double checkSoftLimits(double posSetpoint);
  int    seqHoming1();  //nCmdData==1
  int    seqHoming2();  //nCmdData==2
  int    seqHoming3();  //nCmdData==3
  int    seqHoming4();  //nCmdData==4
  int    seqHoming5();  //nCmdData==5
  int    seqHoming6();  //nCmdData==6
  int    seqHoming15(); //nCmdData==15
  int    seqHoming21(); //nCmdData==21
  int    seqHoming22(); //nCmdData==22
  int    checkHWLimitsAndStop(bool checkBWD,bool checkFWD);
  int    stopSeq();
  int    getExtTrajSetpoint(double *pos);

  int    seqState_;
  int    seqStateOld_;
  int    seqTimeout_;
  int    seqTimeCounter_;
  bool   hwLimitSwitchFwd_;
  bool   hwLimitSwitchFwdOld_;
  bool   hwLimitSwitchBwd_;
  bool   hwLimitSwitchBwdOld_;
  bool   homeSensor_;
  bool   homeSensorOld_;
  bool   seqInProgress_;
  bool   seqInProgressOld_;
  bool   jogFwd_;
  bool   jogBwd_;
  bool   enableSoftLimitBwdBackup_;
  bool   enableSoftLimitFwdBackup_;
  bool   executeOld_;
  bool   busy_;
  double jogVel_;
  double homeVelTwordsCam_; //ADR command
  double homeVelOffCam_; //ADR command
  double homePosition_;
  double homePosLatch1_;
  double homePosLatch2_;
  motionDirection currSeqDirection_;
  motionDirection homeDirection_;
  ecmcTrajectoryTrapetz *traj_;
  ecmcEncoder *enc_;
  ecmcMonitor *mon_;
  ecmcPIDController *cntrl_;
  ecmcMasterSlaveIF *externalInputTrajectoryIF_;
  ecmcAxisData* data_;
  uint64_t oldEncAbsPosReg_;
  uint64_t encAbsPosReg_;
};

#endif /* ecmcAxisSequencer_H_ */
