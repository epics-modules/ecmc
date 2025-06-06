/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcMonitor.h
*
*  Created on: Jan 12, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef MOTIONMONITOR_H
#define MOTIONMONITOR_H
#include <cmath>
#include "ecmcEcEntry.h"
#include "ecmcError.h"
#include "ecmcTrajectoryTrapetz.h"
#include "ecmcAxisData.h"

// MONITOR ERRORS
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
#define ERROR_MON_TOL_OUT_OF_RANGE 0x14C15
#define ERROR_MON_TIME_OUT_OF_RANGE 0x14C16
#define ERROR_MON_POLARITY_OUT_OF_RANGE 0x14C17
#define ERROR_MON_ENTRY_ANALOG_INTERLOCK_NULL 0x14C18
#define ERROR_MON_STALL 0x14C19

// MONITOR WARNINGS
#define WARNING_MON_SOFT_LIMIT_FWD_INTERLOCK 0x114C00
#define WARNING_MON_SOFT_LIMIT_BWD_INTERLOCK 0x114C01
#define WARNING_MON_HARD_LIMIT_FWD_INTERLOCK 0x114C02
#define WARNING_MON_HARD_LIMIT_BWD_INTERLOCK 0x114C03

#define ECMC_MON_SWITCHES_FILTER_CYCLES 5

enum ecmcSwitchPolarity {
  ECMC_POLARITY_NC = 0,
  ECMC_POLARITY_NO = 1
};

class ecmcMonitor : public ecmcEcEntryLink {
public:
  explicit ecmcMonitor(ecmcAxisData *axisData,
                       ecmcEncoder **encArray);

  void               initVars();
  ~ecmcMonitor();
  bool               getHardLimitFwd();
  bool               getHardLimitBwd();
  int                setAtTargetTol(double tol);
  double             getAtTargetTol();
  int                setAtTargetTime(int time);
  int                getAtTargetTime();
  bool               getAtTarget();
  void               setEnableAtTargetMon(bool enable);
  bool               getEnableAtTargetMon();
  int                setCtrlDeadband(double tol);
  int                setCtrlDeadbandTime(int time);
  bool               getCtrlInDeadband();
  int                setPosLagTol(double tol);
  double             getPosLagTol();
  int                setPosLagTime(int time);
  int                getPosLagTime();
  void               setEnableLagMon(bool enable);
  bool               getEnableLagMon();
  bool               getHomeSwitch();
  int                setMaxVel(double vel);
  double             getMaxVel(void);
  int                setEnableMaxVelMon(bool enable);
  bool               getEnableMaxVelMon();
  int                setMaxVelDriveTime(int time);
  int                setMaxVelTrajTime(int time);
  int                setLatchAtLimit(bool latchOnLimit);
  int                getLatchAtLimit();
  double             getCurrentPosSet();
  void               execute();
  void               readEntries();
  void               setEnable(bool enable);
  bool               getEnable();
  int                validate();
  int                reset();
  void               errorReset();
  int                setPLCInterlock(bool              ilock,
                                     plcInterlockTypes type);
  int                setEnableHardwareInterlock(bool enable);
  int                setHardwareInterlockPolarity(ecmcSwitchPolarity pol);
  int                setHardLimitBwdPolarity(ecmcSwitchPolarity pol);
  int                setHardLimitFwdPolarity(ecmcSwitchPolarity pol);
  int                setHomePolarity(ecmcSwitchPolarity pol);
  ecmcSwitchPolarity getHardLimitBwdPolarity();
  ecmcSwitchPolarity getHardLimitFwdPolarity();
  ecmcSwitchPolarity getHomePolarity();
  ecmcSwitchPolarity getHardwareInterlockPolarity();
  int                setCntrlOutputHL(double outputHL);
  int                setEnableCntrlHLMon(bool enable);
  bool               getEnableCntrlHLMon();
  int                setEnableVelocityDiffMon(bool enable);
  bool               getEnableVelocityDiffMon();
  int                setVelDiffTimeTraj(int time);
  int                setVelDiffTimeDrive(int time);
  int                setVelDiffMaxDifference(double velo);
  int                setEnableSoftLimitBwd(bool enable);
  int                setEnableSoftLimitFwd(bool enable);
  int                setEnableHardLimitBWDAlarm(bool enable);
  int                setEnableHardLimitFWDAlarm(bool enable);
  int                setSoftLimitBwd(double limit);
  int                setSoftLimitFwd(double limit);
  int                getEnableAlarmAtHardLimit();
  double             getSoftLimitBwd();
  double             getSoftLimitFwd();
  bool               getEnableSoftLimitBwd();
  bool               getEnableSoftLimitFwd();
  bool               getAtSoftLimitBwd();
  bool               getAtSoftLimitFwd();
  int                setEnableSoftLimitAlarm(bool enable);
  int                setEnableCheckEncsDiff(bool enable);

  // Special interlock for ecmc_plugin_safety for interface with safety PLC
  void               setSafetyInterlock(int interlock);
  int                getSafetyInterlock();

  // Analog interlock for temperature sensors
  int                setAnalogInterlockPolarity(ecmcSwitchPolarity pol);
  int                setAnalogRawLimit(double analogLimit);
  int                setEnableAnalogInterlock(bool enable);
  int                getSumInterlock();
  
  /* Stall monitoring for ABS and REL moves by checking that axis has arrived atTarget 
     after a minmium timeout or timeFactor multiplied by last movement time (traj busy).
     Needs atTraget monitoring enabled. Time factor default 10.0.
  */
  void               setEnableStallMon(bool enable);
  void               setStallMinTimeOut(double timeCycles);
  void               setStallTimeFactor(double timeFactor);
  bool               getEnableStallMon();
  double             getStallTimeFactor();
  int                setLimitSwitchFwdPLCOverride(bool overrideSwitch);
  int                setLimitSwitchBwdPLCOverride(bool overrideSwitch);
  int                setHomeSwitchPLCOverride(bool overrideSwitch);
  void               setLimitSwitchFwdPLCOverrideValue(bool switchValue);
  void               setLimitSwitchBwdPLCOverrideValue(bool switchValue);
  void               setHomeSwitchPLCOverrideValue(bool switchValue);
  int                setHomeSwitchEnable(bool enable);
  // Set reduce trq drive bit for slaved axes
  void               setCtrlWithinDBExtTraj(bool within);

private:
  int                checkLimits();
  int                checkAtTarget();
  int                checkPositionLag();
  int                checkEncoderDiff();
  int                checkMaxVelocity();
  int                checkVelocityDiff();
  int                checkCntrlMaxOutput();
  int                checkStall();
  int                filterSwitches();
  int                checkPolarity(ecmcSwitchPolarity pol);
  bool enable_;

  // Tolnoerance for reached target. Example 0.1 deg
  double atTargetTol_;

  // Number of cycles the position error needs to be below dInTargetTol before
  // the bAtTarget bit goes high.
  int atTargetTime_;

  // Enable At target monitoring
  int enableAtTargetMon_;

  // Tolerance used during trajectory to monitor position lag. Example 0.1 deg
  double posLagTol_;

  // Number of cycles the position error needs to be above dPosLagTol
  // before the following error bit goes high.
  int posLagTime_;

  // Enable lag monitoring
  int enableLagMon_;
  int atTargetCounter_;
  int lagMonCounter_;
  bool hardBwdOld_;
  bool hardFwdOld_;
  double maxVel_;
  bool enableMaxVelMon_;
  int maxVelCounterDrive_;
  int maxVelCounterTraj_;
  int maxVelDriveILDelay_;
  int maxVelTrajILDelay_;
  bool enableHardwareInterlock_;
  double cntrlOutputHL_;
  bool enableCntrlHLMon_;
  bool enableVelocityDiffMon_;
  int velocityDiffCounter_;
  int velDiffTimeTraj_;
  int velDiffTimeDrive_;
  double velDiffMaxDiff_;
  double enableAlarmAtHardlimitBwd_;
  double enableAlarmAtHardlimitFwd_;
  ecmcAxisData *data_;
  int switchFilterCounter_;
  bool limitFwdFilterBuffer_[ECMC_MON_SWITCHES_FILTER_CYCLES];
  bool limitBwdFilterBuffer_[ECMC_MON_SWITCHES_FILTER_CYCLES];
  bool homeFilterBuffer_[ECMC_MON_SWITCHES_FILTER_CYCLES];
  bool latchOnLimit_;  // stop even if just bounce
  bool enableAlarmOnSofLimits_;
  interlockTypes interlockStatusOld_;
  ecmcSwitchPolarity hardwareInterlockPolarity_;
  ecmcSwitchPolarity lowLimPolarity_;
  ecmcSwitchPolarity highLimPolarity_;
  ecmcSwitchPolarity homePolarity_;
  ecmcEncoder **encArray_;
  int enableDiffEncsMon_;
  double ctrlDeadbandTol_; // controller deadband
  int ctrlDeadbandCounter_;
  int ctrlDeadbandTime_;
  double analogRawLimit_;
  int enableAnalogInterlock_;
  ecmcSwitchPolarity analogPolarity_;
  double stallTimeFactor_;
  int enableStallMon_;
  uint64_t maxStallCounter_;
  uint64_t stallLastMotionCmdCycles_;
  uint64_t stallCheckAtTargetAtCycle_;
  double stallMinTimeoutCycles_;
  bool limitSwitchFwdPLCOverride_;
  bool limitSwitchBwdPLCOverride_;
  bool limitSwitchFwdPLCOverrideValue_;
  bool limitSwitchBwdPLCOverrideValue_;
  bool homeSwitchPLCOverride_;
  bool homeSwitchPLCOverrideValue_;
  bool enableHomeSensor_;
  bool ctrlWinthinDBExternalTraj_;
};

#endif  // ifndef MOTIONMONITOR_H
