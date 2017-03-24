#ifndef MOTIONMONITOR_H
#define MOTIONMONITOR_H
#include <cmath>
#include "ecmcEcEntry.h"
#include "ecmcError.h"
#include "ecmcTrajectoryTrapetz.hpp"
#include "ecmcAxisData.h"

//MONITOR ERRORS
#define ERROR_MON_ASSIGN_ENTRY_FAILED 0x14C00
#define ERROR_MON_ENTRY_READ_FAIL 0x14C01
#define ERROR_MON_ENTRY_HARD_BWD_NULL 0x14C02
#define ERROR_MON_ENTRY_HARD_FWD_NULL 0x14C03
#define ERROR_MON_ENTRY_HOME_NULL 0x14C04
#define ERROR_MON_ENTRY_HARDWARE_INTERLOCK_NULL 0x14C05
#define ERROR_MON_CNTRL_OUTPUT_EXCEED_LIMIT 0x14C06
#define ERROR_MON_MAX_VELOCITY_EXCEEDED 0x14C07
#define ERROR_MON_MAX_POSITION_LAG_EXCEEDED 0x14C08
#define ERROR_MON_EXTERNAL_HARDWARE_INTERLOCK 0x14C09
#define ERROR_MON_CNTRL_OUTPUT_INCREASE_AT_LIMIT 0x14C0A
#define ERROR_MON_SOFT_LIMIT_FWD_INTERLOCK 0x14C0B
#define ERROR_MON_SOFT_LIMIT_BWD_INTERLOCK 0x14C0C
#define ERROR_MON_HARD_LIMIT_FWD_INTERLOCK 0x14C0D
#define ERROR_MON_HARD_LIMIT_BWD_INTERLOCK 0x14C0E
#define ERROR_MON_POS_LAG_INTERLOCK 0x14C0F
#define ERROR_MON_BOTH_LIMIT_INTERLOCK 0x14C10
#define ERROR_MON_DISTANCE_TO_STOP_ZERO 0x14C11
#define ERROR_MON_ENTRY_EXT_INTERLOCK_NULL 0x14C12
#define ERROR_MON_UNEXPECTED_LIMIT_SWITCH_BEHAVIOUR_INTERLOCK 0x14C13
#define ERROR_MON_VELOCITY_DIFFERENCE_EXCEEDED 0x14C14

#define ECMC_MON_SWITCHES_FILTER_CYCLES 5

class ecmcMonitor : public ecmcEcEntryLink
{
public:
  ecmcMonitor(ecmcAxisData *axisData);
  ecmcMonitor(ecmcAxisData *axisData,bool enableAtTargetMon, bool enableLagMon);
  void initVars();
  ~ecmcMonitor();
  bool getHardLimitFwd();
  bool getHardLimitBwd();
  void setAtTargetTol(double tol);
  double getAtTargetTol();
  void setAtTargetTime(int time);
  int getAtTargetTime();
  bool getAtTarget();
  void setEnableAtTargetMon(bool enable);
  bool getEnableAtTargetMon();
  void setPosLagTol(double tol);
  double getPosLagTol();
  void setPosLagTime(int time);
  int getPosLagTime();
  void setEnableLagMon(bool enable);
  bool getEnableLagMon();
  bool getHomeSwitch();
  int setMaxVel(double vel);
  int setEnableMaxVelMon(bool enable);
  bool getEnableMaxVelMon();
  int setMaxVelDriveTime(int time);
  int setMaxVelTrajTime(int time);
  double getCurrentPosSet();
  void execute();
  void readEntries();
  void setEnable(bool enable);
  bool getEnable();
  int validate();
  int reset();
  void errorReset();
  int setEnableHardwareInterlock(bool enable);
  int setCntrlOutputHL(double outputHL);
  int setEnableCntrlHLMon(bool enable);
  bool getEnableCntrlHLMon();
  int setEnableVelocityDiffMon(bool enable);
  bool getEnableVelocityDiffMon();
  int setVelDiffTimeTraj(int time);
  int setVelDiffTimeDrive(int time);
  int setVelDiffMaxDifference(double velo);
  int setEnableSoftLimitBwd(bool enable);
  int setEnableSoftLimitFwd(bool enable);
  int setEnableHardLimitBWDAlarm(bool enable);
  int setEnableHardLimitFWDAlarm(bool enable);
  int setSoftLimitBwd(double limit);
  int setSoftLimitFwd(double limit);
  int getEnableAlarmAtHardLimit();
  double getSoftLimitBwd();
  double getSoftLimitFwd();
  bool getEnableSoftLimitBwd();
  bool getEnableSoftLimitFwd();
  bool getAtSoftLimitBwd();
  bool getAtSoftLimitFwd();
  void printCurrentState();
private:
  int checkLimits();
  int checkAtTarget();
  int checkPositionLag();
  int checkMaxVelocity();
  int checkVelocityDiff();
  int checkCntrlMaxOutput();
  int filterSwitches();
  void printInterlockStatus(interlockTypes ilock);
  bool   enable_;
  double atTargetTol_;           //Tolnoerance for reached target. Example 0.1 deg
  int    atTargetTime_;          //Number of cycles the position error needs to be below dInTargetTol before the bAtTarget bit goes high
  int    enableAtTargetMon_;     //Enable At target monitoring
  double posLagTol_;             //Tolerance used during trajectory to monitor position lag. Example 0.1 deg
  int    posLagTime_;            //Number of cycles the position error needs to be above dPosLagTol before the following error bit goes high
  int    enableLagMon_;          //Enable lag monitoring
  int    atTargetCounter_;       //
  int    lagMonCounter_;         //
  bool   hardBwdOld_;
  bool   hardFwdOld_;
  double lagError_;
  double maxVel_;
  bool   enableMaxVelMon_;
  int    maxVelCounterDrive_;;
  int    maxVelCounterTraj_;
  int    maxVelDriveILDelay_;
  int    maxVelTrajILDelay_;
  bool   enableHardwareInterlock_;
  double cntrlOutputHL_;
  bool enableCntrlHLMon_;
  bool enableVelocityDiffMon_;
  int velocityDiffCounter_;
  int velDiffTimeTraj_;
  int velDiffTimeDrive_;
  double velDiffMaxDiff_;
  double enableAlarmAtHardlimitBwd_;
  double enableAlarmAtHardlimitFwd_;
  ecmcAxisData* data_;

  int switchFilterCounter_;
  bool limitFwdFilterBuffer_[ECMC_MON_SWITCHES_FILTER_CYCLES];
  bool limitBwdFilterBuffer_[ECMC_MON_SWITCHES_FILTER_CYCLES];
  bool homeFilterBuffer_[ECMC_MON_SWITCHES_FILTER_CYCLES];
  interlockTypes interlockStatusOld_;
};
#endif
