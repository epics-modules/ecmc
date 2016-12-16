/*
 * cMcuSequencer.h
 *
 *  Created on: Jan 14, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCSEQUENCER_HPP_
#define ECMCSEQUENCER_HPP_
#include "ecmcEncoder.h"
#include "ecmcError.h"
#include "ecmcMonitor.hpp"
#include "ecmcPIDController.hpp"
#include "ecmcTrajectoryTrapetz.hpp"
#include "ecmcMasterSlaveIF.h"

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

class ecmcSequencer : public ecmcError
{
public:
  ecmcSequencer();
  ~ecmcSequencer();
  int setExecute(bool execute);
  bool getExecute();
  void execute();
  void setCommand(motionCommandTypes command);
  motionCommandTypes getCommand();
  void setCmdData(int cmdData);
  int getCmdData();
  void setTraj(ecmcTrajectoryTrapetz *traj);
  ecmcTrajectoryTrapetz *getTraj();
  void setEnc(ecmcEncoder *enc);
  void setMon(ecmcMonitor *mon);
  void setCntrl(ecmcPIDController *con);
  bool getBusy();
  void setJogVel(double velTarget);
  double getJogVel();
  int  setHomeVelTwordsCam(double vel);
  int setHomeVelOffCam(double vel);
  double getHomeVelTwordsCam();
  double getHomeVelOffCam();
  int setHomeDir(motionDirection dir);
  motionDirection getHomeDir();
  void setHomePosition(double pos);
  double getHomePosition();
  void setTargetPos(double pos);
  double getTargetPos();
  void setTargetVel(double velTarget);
  double getTargetVel();
  void setJogFwd(bool jog);
  bool getJogFwd();
  void setJogBwd(bool jog);
  bool getJogBwd();
  int getSeqState();
  int validate();
  int setSequenceTimeout(int timeout);
  int setExternalExecute(bool execute);
  int setExtTrajIF(ecmcMasterSlaveIF *extIf);
private:
  void initVars();
  double checkSoftLimits(double posSetpoint);
  int seq1SimpleHoming();
  int seqHoming1(); //nCmdData==1
  int seqHoming2(); //nCmdData==2
  int seqHoming3(); //nCmdData==3
  int seqHoming4(); //nCmdData==4
  int seqHoming5(); //nCmdData==5
  int seqHoming6(); //nCmdData==6
  int checkHWLimitsAndStop(bool checkBWD,bool checkFWD);
  int stopSeq();
  int getExtTrajSetpoint(double *pos);
  bool hwLimitSwitchFwd_;
  bool hwLimitSwitchFwdOld_;
  bool hwLimitSwitchBwd_;
  bool hwLimitSwitchBwdOld_;
  bool homeSensor_;
  bool homeSensorOld_;
  bool seqInProgress_;
  motionDirection currSeqDirection_;
  bool execute_;
  bool executeOld_;
  motionCommandTypes command_;
  int cmdData_;
  ecmcTrajectoryTrapetz *traj_;
  ecmcEncoder *enc_;
  ecmcMonitor *mon_;
  ecmcPIDController *cntrl_;
  bool busy_;
  double jogVel_;
  double homeVelTwordsCam_; //ADR command
  double homeVelOffCam_; //ADR command
  motionDirection homeDirection_;
  double  homePosition_;
  double targetPosition_;
  double targetVelocity_;
  bool jogFwd_;
  bool jogBwd_;
  bool enableSoftLimitBwdBackup_;
  bool enableSoftLimitFwdBackup_;
  int seqState_;
  double homePosLatch1_;
  double homePosLatch2_;
  int seqTimeout_;
  int seqTimeCounter_;
  bool externalExecute_;
  ecmcMasterSlaveIF *externalInputTrajectoryIF_;
};

#endif /* ECMCSEQUENCER_HPP_ */
