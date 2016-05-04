#ifndef MOTIONMONITOR_H
#define MOTIONMONITOR_H
#include <cmath>
#include "ecmcEcEntry.h"
#include "ecmcError.h"
#include "ecmcErrorsList.h"
#include "ecmcTrajectory.hpp"

#define MaxMcuMonitorEntries 10

class ecmcMonitor : public ecmcError
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
  int setEntryAtIndex(ecmcEcEntry *entry,int index);
  void readEntries();
  void setEnable(bool enable);
  bool getEnable();
  interlockTypes getTrajInterlock();
  bool getDriveInterlock();
  int validate();
  int reset();

private:
  bool   enable_;
  double atTargetTol_;           //Tolerance for reached target. Example 0.1 deg
  int    atTargetTime_;          //Number of cycles the position error needs to be below dInTargetTol before the bAtTarget bit goes high
  int    enableAtTargetMon_;     //Enable At target minitoring
  double posLagTol_;             //Tolerance used during trajectory to monitor position lag. Example 0.1 deg
  int    posLagTime_;            //Number of cycles the position error needs to be above dPosLagTol before the following error bit goes high
  int    enableLagMon_;          //Enable lag minitoring
  double actPos_;
  bool   motionInterlock_;
  bool   atTarget_;              //High if error is below dAtTargetTol for a nAtTargetTime cycles
  int    atTargetCounter_;       //
  bool   lagErrorTraj_;          //Lag monitoring error
  bool   lagErrorDrive_;         //Lag monitoring error
  int    lagMonCounter_;         //
  bool   hardBwd_;               //At hard low limit
  bool   HardFwd_;               //At hard high limit
  bool   homeSwitch_;            //Home switch
  double targetPos_;
  double currSetPos_;
  double lagError_;
  bool   bothLimitsLowInterlock_;
  ecmcEcEntry *entryArray_[MaxMcuMonitorEntries];  //CH 0 hardbwd, CH 1 hardfwd CH 2 Home
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
};
#endif
