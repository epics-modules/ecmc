#ifndef TRAJGEN_H
#define TRAJGEN_H
#include <cmath>
#include <string.h>

#include "ecmcDefinitions.h"
#include "ecmcEncoder.h"
#include "ecmcError.h"
#include "ecmcMasterSlaveData.h"
#include "ecmcTransform.h"

class ecmcTrajectory : public ecmcError, public ecmcMasterSlaveIF
{
public:
  ecmcTrajectory(double sampleTime);
  ecmcTrajectory(double velocityTarget, double acceleration, double deceleration, double jerk,double sampleTime);
  ~ecmcTrajectory();
  void initVars();
  double getNextPosSet();
  double getCurrentPosSet();
  void setCurrentPosSet(double posSet);
  double getVel();
  int getIndex();
  bool getBusy();
  void setTargetVel(double velTarget);
  double getTargetVel();
  void setAcc(double acc);
  double getAcc();
  void setDec(double dec);
  double getDec();
  void setEmergDec(double dec);
  void setJerk(double jerk);
  double getJerk();
  void setTargetPos(double pos);
  double getTargetPos();
  void setStartPos(double pos);
  void setSoftLimitBwd(double limit);
  int setGearRatio(double ratioNum, double ratioDenom);
  double getGearRatio();
  void setSoftLimitFwd(double limit);
  double getSoftLimitBwd();
  double getSoftLimitFwd();
  bool getAtSoftLimitBwd();
  bool getAtSoftLimitFwd();
  void setEnableSoftLimitBwd(bool enable);
  void setEnableSoftLimitFwd(bool enable);
  bool getEnableSoftLimitBwd();
  bool getEnableSoftLimitFwd();
  void setHardLimitFwd(bool switchState);
  void setHardLimitBwd(bool switchState);
  bool getHardLimitFwd();
  bool getHardLimitBwd();
  bool getExecute();
  void setExecute(bool execute);
  bool getInterlocked();
  void setInterlock(interlockTypes interlock);
  void setEnable(bool enable);
  bool getEnable();
  void setMotionMode(motionMode mode);
  void setCoordSystMode(coordSystMode mode);
  interlockTypes getInterlockStatus();
  int setEnableHardLimitFWDAlarm(bool enable);
  int setEnableHardLimitBWDAlarm(bool enable);
  double getSampleTime();
  int validate();
  int getCurrentExternalSetpoint(double* value);

private:
  double distToStop(double vel);
  void initTraj();
  double checkSoftLimits(double posSetpoint,motionDirection direction);
  void stop();
  double internalTraj(double *velocity);
  double moveVel(double currSetpoint, double currVelo,double targetVelo);
  double movePos(double currSetpoint,double targetSetpoint,double stopDistance, double currVelo,double targetVelo);
  double moveStop(stopMode stopMode,double currSetpoint, double currVelo,double targetVelo, bool *stopped,double *velocity);
  stopMode checkInterlocks(motionDirection dir,double newSetpoint);
  double updateSetpoint(double nextSetpoint,double nextVelocity);
  motionDirection checkDirection(double oldPos, double newPos);
  double acceleration_;
  double deceleration_;
  double emergDeceleration_;
  double decelerationEmergency_; //TODO Check why two emergemcy decel
  double velocityTarget_;
  double targetPosition_;
  double jerk_;
  double sampleTime_; //s
  double posSetMinus1_;
  double posSetMinus2_;
  double stepACC_;
  double stepDEC_;
  double stepNOM_;
  double stepDECEmerg_;
  double currentPositionSetpoint_;
  double velocity_;
  bool   trajInProgress_;
  double softLimitBwd_;
  double softLimitFwd_;
  bool   enableSoftLimitBwd_;
  bool   enableSoftLimitFwd_;
  int    index_;
  bool   execute_;
  bool   executeOld_;
  bool   currentPosInterlock_;
  double startPosition_;
  bool   hwLimitSwitchFwd_;
  bool   hwLimitSwitchBwd_;
  double relPosOffset_;
  bool   enable_;
  bool   enableOld_;
  bool   internalStopCmd_;
  double distToStop_;
  double prevStepSize_;
  double gearRatio_;
  interlockTypes externalInterlock_; //Interlock from Monitor class
  motionDirection actDirection_;
  motionDirection setDirection_;
  coordSystMode coordSysteMode_;
  motionMode motionMode_;
  interlockTypes interlockStatus_;
  bool enableHardLimitFWDAlarms_;
  bool enableHardLimitBWDAlarms_;
};
#endif
