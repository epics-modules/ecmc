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
#include "ecmcTrajectory.hpp"

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

class ecmcSequencer : public ecmcError
{
public:
  ecmcSequencer();
  ~ecmcSequencer();
  void setHomeSensor(bool switchState);
  int setExecute(bool execute);
  bool getExecute();
  void execute();
  void setCommand(motionCommandTypes command);
  motionCommandTypes getCommand();
  void setCmdData(int cmdData);
  int getCmdData();
  void setTraj(ecmcTrajectory *traj);
  ecmcTrajectory * getTraj();
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
  void setSoftLimitBwd(double limit);
  void setSoftLimitFwd(double limit);
  double getSoftLimitBwd();
  double getSoftLimitFwd();
  bool getAtSoftLimitBwd();
  bool getAtSoftLimitFwd();
  void setEnableSoftLimitBwd(bool enable);
  void setEnableSoftLimitFwd(bool enable);
  bool getEnableSoftLimitBwd();
  bool getEnableSoftLimitFwd();
  int setGearRatio(double ratioNum, double ratioDenom);
  int getGearRatio(double *ratio);
  int getSeqState();
  int validate();
  int setEnableAlarmAtHardLimit(bool enable);
  bool getEnableAlarmAtHardLimit();
  int setSequenceTimeout(int timeout);
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
  int setSoftLimitsToTraj();
  int stopSeq();
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
  ecmcTrajectory *traj_;
  ecmcEncoder*enc_;
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
  double softLimitBwd_;
  double softLimitFwd_;
  bool   enableSoftLimitBwd_;
  bool   enableSoftLimitFwd_;
  int seqState_;
  double homePosLatch1_;
  double homePosLatch2_;
  bool enableAlarmAtHardlimit_;
  int seqTimeout_;
  int seqTimeCounter_;
};

#endif /* ECMCSEQUENCER_HPP_ */
