/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcTrajectoryTrapetz.h
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef SRC_ECMCTRAJECTORYTRAPETZ_H_
#define SRC_ECMCTRAJECTORYTRAPETZ_H_

#include <string.h>
#include <cmath>
#include "../main/ecmcDefinitions.h"
#include "../main/ecmcError.h"
#include "ecmcEncoder.h"
#include "ecmcAxisData.h"

/// Error codes for class ecmcTrajectoryTrapetz
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
#define ERROR_TRAJ_MOD_FACTOR_OUT_OF_RANGE 0x14E16
#define ERROR_TRAJ_MOD_TYPE_OUT_OF_RANGE 0x14E17

/**
 * \class ecmcTrajectoryTrapetz
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
 *
 * \date $Date: 2005/04/14 14:16:20 $
 *
 * Contact: anders.sandstrom@esss.se
 *
 * Created on: 2015-11-01
 *
 */
class ecmcTrajectoryTrapetz : public ecmcError {
 public:
  ecmcTrajectoryTrapetz(ecmcAxisData *axisData,
                        double        sampleTime);
  ecmcTrajectoryTrapetz(ecmcAxisData *axisData,
                        double        velocityTarget,
                        double        acceleration,
                        double        deceleration,
                        double        jerk,
                        double        sampleTime);
  ~ecmcTrajectoryTrapetz();

  /** \brief Calculates and returns next position setpoint.
   * This function should only be executed once for each sample period.
   */
  double          getNextPosSet();

  /** \brief Returns current position setpoint.
   * This function can be called several times each sample period.
   */
  double          getCurrentPosSet();

  /// Sets position setpoint.
  void            setCurrentPosSet(double posSet);

  /** \brief Returns current velocity of trajectory.
   * Useful for feed-forward purpose of the velocity directly to the velocity control loop.
   */
  double          getVel();

  /// Returns sample period count since beginning of trajectory.
  int             getIndex();

  /// Returns if trajectory generator is busy (motion in progress).
  bool            getBusy();

  /** \brief Sets target velocity of trajectory (max velocity).
   * Note: This is the max velocity of the trajectory generator. The actual velocity can be higher.
   */
  void            setTargetVel(double velTarget);

  /// Returns target velocity.
  double          getTargetVel();

  /// Sets acceleration.
  void            setAcc(double acc);

  /// Returns acceleration.
  double          getAcc();

  /// Sets deceleration.
  void            setDec(double dec);

  /// Returns deceleration.
  double          getDec();

  /** \brief Sets emergency deceleration.
   * Used for ramp down at hard limits.
   */
  void            setEmergDec(double dec);

  /** Currently not implemented (will be used when 
   * s-shaped trajectory is implemented).
   */
  void            setJerk(double jerk);

  /// Not used.
  double          getJerk();

  /// Sets target position (end position of trajectory).
  void            setTargetPos(double pos);

  /// returns target position (end position of trajectory).
  double          getTargetPos();

  /** \brief Sets start position of trajectory.
   * Normally encoder position at amplifier enable
   */
  void            setStartPos(double pos);
  int             setExecute(bool execute);
  bool            getExecute();
  void            setEnable(bool enable);
  bool            getEnable();
  void            setMotionMode(motionMode mode);
  interlockTypes  getInterlockStatus();
  double          getSampleTime();
  int             validate();
  double          distToStop(double vel);
  int             initStopRamp(double currentPos,
                               double currentVel,
                               double currentAcc);
  motionDirection getCurrSetDir();
  motionDirection checkDirection(double oldPos,
                                 double newPos);

 private:
  void            initVars();
  void            initTraj();
  double          internalTraj(double *velocity);
  double          moveVel(double currSetpoint,
                          double prevStepSize,
                          double stepNom,
                          bool  *trajBusy);
  double          movePos(double currSetpoint,
                          double targetSetpoint,
                          double stopDistance,
                          double prevStepSize, // currVelo
                          double stepNom,   //targetVelo
                          bool  *trajBusy);
  double          moveStop(stopMode stopMode,
                           double   currSetpoint,
                           double   prevStepSize,
                           bool    *stopped,
                           double  *velocity);
  stopMode        checkInterlocks();
  double          updateSetpoint(double nextSetpoint,
                                 double nextVelocity);
  double          dist(double from,
                       double to,
                       motionDirection direction);
  double          checkModuloPos(double pos,
                       motionDirection direction);
  double acceleration_;
  double deceleration_;
  double decelerationEmergency_;
  double velocityTarget_;
  double targetPosition_;
  double jerk_;
  double sampleTime_;
  double posSetMinus1_;
  double stepACC_;
  double stepDEC_;
  double stepNOM_;
  double stepDECEmerg_;
  double currentPositionSetpoint_;
  double velocity_;
  bool busy_;
  int index_;
  bool execute_;
  bool executeOld_;
  double startPosition_;
  bool enable_;
  bool enableOld_;
  bool internalStopCmd_;
  double distToStop_;
  double prevStepSize_;
  double thisStepSize_;
  motionMode motionMode_;
  interlockTypes interlockStatus_;
  ecmcAxisData *data_;
  stopMode latchedStopMode_;
  bool switchTargetOnTheFly_;
  double stepStableTol_;
};
#endif  // ifndef SRC_ECMCTRAJECTORYTRAPETZ_H_
