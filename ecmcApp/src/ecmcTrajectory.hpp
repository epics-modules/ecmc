#ifndef TRAJGEN_H
#define TRAJGEN_H
#include <cmath>
#include <string.h>

#include "ecmcDefinitions.h"
#include "ecmcEncoder.h"
#include "ecmcError.h"
#include "ecmcMasterSlaveData.h"
#include "ecmcTransform.h"

/// Error codes for class ecmcTrajectory
#define ERROR_TRAJ_EXT_ENC_NULL 0x14E00
#define ERROR_TRAJ_EXT_TRAJ_NULL 0x14E01
#define ERROR_TRAJ_NOT_ENABLED 0x14E02
#define ERROR_TRAJ_GEAR_RATIO_DENOM_ZERO 0x14E03
#define ERROR_TRAJ_SOFT_LIMIT_FWD_INTERLOCK 0x14E04
#define ERROR_TRAJ_SOFT_LIMIT_BWD_INTERLOCK  0x14E05
#define ERROR_TRAJ_HARD_LIMIT_FWD_INTERLOCK  0x14E06
#define ERROR_TRAJ_HARD_LIMIT_BWD_INTERLOCK  0x14E07
#define ERROR_TRAJ_POS_LAG_INTERLOCK 0x14E08
#define ERROR_TRAJ_BOTH_LIMIT_INTERLOCK 0x14E09
#define ERROR_TRAJ_EXTERNAL_INTERLOCK 0x14E0A
#define ERROR_TRAJ_EXECUTE_BUT_NO_ENABLE 0x14E0B
#define ERROR_TRAJ_EXT_TRANSFORM_NULL 0x14E0C
#define ERROR_TRAJ_TRANSFORM_NOT_COMPILED 0x14E0D
#define ERROR_TRAJ_TRANSFORM_INTERLOCK_ERROR 0x14E0E
#define ERROR_TRAJ_TRANSFORM_NULL 0x14E0F
#define ERROR_TRAJ_EXT_MASTER_SOURCE_NULL 0x14E10
#define ERROR_TRAJ_EXT_MASTER_SOURCE_COUNT_ZERO 0x14E11
#define ERROR_TRAJ_INVALID_SAMPLE_TIME 0x14E12
#define ERROR_TRAJ_TRANSFORM_VALIDATION_ERROR 0x14E13
#define ERROR_TRAJ_SLAVE_INTERFACE_NULL 0x14E14
#define ERROR_TRAJ_MAX_SPEED_INTERLOCK 0x14E15

/**
 * \class ecmcTrajectory
 *
 * \ingroup ecmc
 *
 * \brief Implements a trapezoidal motion setpoint trajectory
 *
 * This class implements trapezoidal motion setpoint trajectory divided into three segments:
 * 1. Acceleration phase with constant acceleration
 * 2. Constant velocity phase (acceleration=0)
 * 3. Deceleration phase with constant deceleration
 *
 * Supports:
 * 1. Constant velocity
 * 2. Relative positioning
 * 3. Absolute positioning
 * 4. Interlocks (hard limits, soft limits, external interlocks)
 * 5. Synchronization to external source of data (through ecmcMasterSlaveIF)
 *
 * \date $Date: 2005/04/14 14:16:20 $
 *
 * Contact: anders.sandstrom@esss.se
 *
 * Created on: 2015-11-01
 *
 */
class ecmcTrajectory : public ecmcError, public ecmcMasterSlaveIF
{
public:
  ecmcTrajectory(double sampleTime);
  ecmcTrajectory(double velocityTarget, double acceleration, double deceleration, double jerk,double sampleTime);
  ~ecmcTrajectory();
  /** \breif Calculates and returns next position setpoint.
   * This function should only be executed once for each sample period.
   */
  double getNextPosSet();
  /** \breif Returns current position setpoint.
   * This function can be called several times each sample period.
   */
  double getCurrentPosSet();
  /// Sets position setpoint.
  void setCurrentPosSet(double posSet);
  /** \breif Returns current velocity of trajectory.
   * Useful for feed-forward purpose of the velocity directly to the velocity control loop.
   */
  double getVel();
  /// Returns sample period count since beginning of trajectory.
  int getIndex();
  /// Returns if trajectory generator is busy (motion in progress).
  bool getBusy();
  /** \breif Sets target velocity of trajectory (max velocity).
   * Note: This is the max velocity of the trajectory generator. The actual velocity can be higher.
   */
  void setTargetVel(double velTarget);
  ///Returns target velocity.
  double getTargetVel();
  ///Sets acceleration.
  void setAcc(double acc);
  ///Returns acceleration.
  double getAcc();
  ///Sets deceleration.
  void setDec(double dec);
  ///Returns deceleration.
  double getDec();
  /** \breif Sets emergency deceleration.
   * Used for ramp down at hard limits.
   */
  void setEmergDec(double dec);
  /// Currently not implemented (will be used when s-shaped trajectory is implemented).
  void setJerk(double jerk);
  /// Not used.
  double getJerk();
  /// Sets target position (end position of trajectory).
  void setTargetPos(double pos);
  /// returns target position (end position of trajectory).
  double getTargetPos();
  /** \breif Sets start position of trajectory.
   * Normally encoder position at amplifier enable
   */
  void setStartPos(double pos);
  /// Sets backward direction soft limit position
  void setSoftLimitBwd(double limit);
  /// Sets gear ratio. Only used when synchronizing to external source.
  int setGearRatio(double ratioNum, double ratioDenom);
  /// Returns gear ratio. Only used when synchronizing to external source.
  double getGearRatio();
  /// Sets forward direction soft limit position.
  void setSoftLimitFwd(double limit);
  /// Returns backward direction soft limit position.
  double getSoftLimitBwd();
  /// Returns forward direction soft limit position.
  double getSoftLimitFwd();
  /// Check if trajectory generated setpoint is at backward  soft limit.
  bool getAtSoftLimitBwd();
  /// Check if trajectory generated setpoint is at forward  soft limit.
  bool getAtSoftLimitFwd();
  /// Enable/disable backward soft limit.
  void setEnableSoftLimitBwd(bool enable);
  /// Enable/disable forward soft limit.
  void setEnableSoftLimitFwd(bool enable);
  /// Return if backward soft limit is enabled.
  bool getEnableSoftLimitBwd();
  /// Return if forward soft limit is enabled.
  bool getEnableSoftLimitFwd();
  /** \breif Sets forward hard limit switch state.
   * The state of the switch should be normally closed (switch=high when motion is OK).
   * A emergency deceleration phase will be started if a switch state of open (=0) is encountered.
   */
  void setHardLimitFwd(bool switchState);
  /** \breif Sets backward hard limit switch state.
   * The state of the switch should be normally closed (switch=high when motion is OK).
   * A emergency deceleration phase will be started if a switch state of open (=0) is encountered.
   */
  void setHardLimitBwd(bool switchState);
  ///Returns the forward hard limit switch state.
  bool getHardLimitFwd();
  ///Returns the backward hard limit switch state.
  bool getHardLimitBwd();
  /** \breif Triggers a new motion trajectory.
   * A new trajectory is triggered on a rising edge of the execute input (setExecute(0),setExecute(1))
   * The current trajectory will be stopped (interlocked) if execute is set to zero when busy.
   * Note: The trajectory needs to be enabled (setEnable(1)) before execution of an trajectory can be performed.
   */
  void setExecute(bool execute);
  /// Returns current value of execute.
  bool getExecute();
  /// Returns if trajectory is interlocked.
  bool getInterlocked();
  /// Sets interlock of trajectory.
  void setInterlock(interlockTypes interlock);
  /// Enables/disables trajectory.
  void setEnable(bool enable);
  /// Returns enable state.
  bool getEnable();
  /// Sets motion mode (positioning or constant velocity)
  void setMotionMode(motionMode mode);
  /// Sets coordinate system mode (absolute or relative)
  void setCoordSystMode(coordSystMode mode);
  /// Returns interlock state
  interlockTypes getInterlockStatus();
  /// Enable/disable alarm at forward hard limit
  int setEnableHardLimitFWDAlarm(bool enable);
  /// Enable/disable alarm at forward hard limit
  int setEnableHardLimitBWDAlarm(bool enable);
  /// Return sample time setting of trajectory generator.
  double getSampleTime();
  /// Function used to validate all settings before going into runtime mode.
  int validate();
  /// Return current setpoint from external source (only used when synchronizing).
  int getCurrentExternalSetpoint(double* value);

private:
  void initVars();
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
  double decelerationEmergency_;
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
