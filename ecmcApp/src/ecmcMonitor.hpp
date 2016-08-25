#ifndef MOTIONMONITOR_H
#define MOTIONMONITOR_H
#include <cmath>
#include "ecmcEcEntry.h"
#include "ecmcError.h"
#include "ecmcTrajectory.hpp"

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

class ecmcMonitor : public ecmcEcEntryLink
{
public:
  ecmcMonitor();
  ecmcMonitor(bool enableAtTargetMon, bool enableLagMon);
  void initVars();
  ~ecmcMonitor();
  bool getHardLimitFwd();
  bool getHardLimitBwd();
  bool getMotionInterlock();
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
  void setHomeSwitch(bool switchState);
  void setActPos(double pos);
  void setTargetPos(double pos);
  double getTargetPos();
  int setTargetVel(double vel);
  int setActVel(double vel);
  int setMaxVel(double vel);
  int setEnableMaxVelMon(bool enable);
  bool getEnableMaxVelMon();
  int setMaxVelDriveTime(int time);
  int setMaxVelTrajTime(int time);
  void setCurrentPosSet(double pos);
  double getCurrentPosSet();
  void execute();
  void readEntries();
  void setEnable(bool enable);
  bool getEnable();
  interlockTypes getTrajInterlock();
  bool getDriveInterlock();
  int validate();
  int reset();
  int setEnableHardwareInterlock(bool enable);
  int setCntrlOutput(double output);
  int setCntrlOutputHL(double outputHL);
  int setEnableCntrlHLMon(bool enable);
  bool getEnableCntrlHLMon();
  int setEnableCntrlOutIncreaseAtLimitMon(bool enable);
  bool getEnableCntrlOutIncreaseAtLimitMon();
  int setCntrlKff(double kff); //Used ensure that motion is reasonable in relation to controller output

private:
  bool   enable_;
  double atTargetTol_;           //Tolerance for reached target. Example 0.1 deg
  int    atTargetTime_;          //Number of cycles the position error needs to be below dInTargetTol before the bAtTarget bit goes high
  int    enableAtTargetMon_;     //Enable At target monitoring
  double posLagTol_;             //Tolerance used during trajectory to monitor position lag. Example 0.1 deg
  int    posLagTime_;            //Number of cycles the position error needs to be above dPosLagTol before the following error bit goes high
  int    enableLagMon_;          //Enable lag monitoring
  double actPos_;
  bool   motionInterlock_;
  bool   atTarget_;              //High if error is below dAtTargetTol for a nAtTargetTime cycles
  int    atTargetCounter_;       //
  bool   lagErrorTraj_;          //Lag monitoring error
  bool   lagErrorDrive_;         //Lag monitoring error
  int    lagMonCounter_;         //
  bool   hardBwd_;               //At hard low limit
  bool   hardFwd_;               //At hard high limit
  bool   homeSwitch_;            //Home switch
  double targetPos_;
  double currSetPos_;
  double lagError_;
  bool   bothLimitsLowInterlock_;
  double targetVel_;
  double actVel_;
  double maxVel_;
  bool   enableMaxVelMon_;
  bool   velErrorTraj_;
  bool   velErrorDrive_;
  int    maxVelCounterDrive_;;
  int    maxVelCounterTraj_;
  int    maxVelDriveILDelay_;
  int    maxVelTrajILDelay_;
  bool   enableHardwareInterlock_;
  bool   hardwareInterlock_;
  double cntrlOutput_;
  double cntrlOutputHL_;
  double cntrlOutputOld_;
  bool cntrlOutputHLErrorTraj_;
  bool cntrlOutputHLErrorDrive_;
  bool enableCntrlHLMon_;
  bool enableCntrlOutIncreaseAtLimitMon_;     //Enable Mechanical stop monitoring
  bool cntrlOutIncreaseAtLimitErrorTraj_;
  bool cntrlOutIncreaseAtLimitErrorDrive_;
  int cntrlOutIncreaseAtLimitCounter_;
  double positionError_;
  double positionErrorOld_;
  int reasonableMoveCounter_;
  int cycleCounter_;
  double cntrlKff_;

};
#endif
