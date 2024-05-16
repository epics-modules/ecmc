/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcTrajectoryBase.h
*
*  Created on: Nov 26, 2021
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef SRC_ECMCTRAJECTORYBASE_H_
#define SRC_ECMCTRAJECTORYBASE_H_

#include <string.h>
#include <cmath>
#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "ecmcEncoder.h"
#include "ecmcAxisData.h"

/// Error codes for class ecmcTrajectoryBase
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
#define ERROR_TRAJ_MOD_POS_CHANGE_WHILE_BUSY_NOT_ALLOWED 0x14E18

enum ecmcTrajTypes {
  ECMC_TRAPETZ = 0,
  ECMC_S_CURVE = 1,
  ECMC_NO_TRAJ = 2,
};

/**
 * \class ecmcTrajectoryBase
 *
 * \ingroup ecmc
 *
 * \brief Base class for trajectories
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
class ecmcTrajectoryBase : public ecmcError {
public:
  explicit ecmcTrajectoryBase(ecmcAxisData *axisData,
                              double        sampleTime);
  virtual ~ecmcTrajectoryBase();

  /** \brief Calculates and returns next position setpoint.
   * This function should only be executed once for each sample period.
   * Needs to call updateSetpoint() in order to use the newly calculated setpoints
   */
  double         getNextPosSet();

  /** \brief Calculation of position , velocoity and acceleration setpoints.
   * Must be implemented in derived class
   * returns position setpoint
   */
  virtual double internalTraj(double *actVelocity,
                              double *actAcceleration,
                              bool   *trajBusy) = 0;

  /** \brief Sets target velocity of trajectory (max velocity).
     * Note: This is the max velocity of the trajectory generator. The actual velocity can be higher.
     */
  virtual  void  setTargetVel(double velTarget);

  /// Sets acceleration.
  virtual void   setAcc(double acc);

  /// Sets deceleration.
  virtual void   setDec(double dec);

  /** \brief Sets emergency deceleration.
   * Used for ramp down at hard limits.
   */
  virtual void   setEmergDec(double dec);

  /** Only used for s-shaped trajectory.
   */
  virtual void   setJerk(double jerk);

  /// Sets target position (end position of trajectory).
  void           setTargetPos(double pos);

  /** Sets target position in derived classses (end position of trajectory).
  *
  *   Needed because of modulo handling (possible to have different target in derived
  *   class and base class)
  *   Must be implemnted..
  */
  virtual void   setTargetPosLocal(double pos) = 0;

  /// How long diatnce until stop
  virtual double distToStop(double vel) = 0;

  /// init stop ramp for when in external traj and need to stop
  virtual int    initStopRamp(double currentPos,
                              double currentVel,
                              double currentAcc);

  /// Sets position setpoint.
  virtual void    setCurrentPosSet(double posSet);

  /// Enable traj
  virtual void    setEnable(bool enable);

  /** \brief Returns current position setpoint.
   * This function can be called several times each sample period.
   */
  double          getCurrentPosSet();

  /** \brief Returns current velocity of trajectory.
   * Useful for feed-forward purpose of the velocity directly to the velocity control loop.
   */
  double          getNextVel();

  /// Returns sample period count since beginning of trajectory.
  int             getIndex();

  /// Returns if trajectory generator is busy (motion in progress).
  bool            getBusy();

  /// Returns target velocity.
  double          getTargetVel();

  /// Returns acceleration.
  double          getAcc();

  /// Returns deceleration.
  double          getDec();

  /// Not used.
  double          getJerk();

  /// returns target position (end position of trajectory).
  double          getTargetPos();

  /// returns modulo target position (end position of trajectory).
  double          getTargetPosMod();

  /** \brief Sets start position of trajectory.
   * Normally encoder position at amplifier enable
   */
  void            setStartPos(double pos);
  virtual int     setExecute(bool execute);
  bool            getExecute();
  bool            getEnable();
  void            setMotionMode(motionMode mode);
  interlockTypes  getInterlockStatus();
  double          getSampleTime();
  virtual int     validate();

  motionDirection getCurrSetDir();
  motionDirection checkDirection(double oldPos,
                                 double newPos);
  // Possability to set a reduced velo from external ecmc_plugin_safety 
  void setExternalMaxVelo(double veloLimit,
                          int active);
protected:
  virtual void initTraj();
  void         initVars();
  double       dist(double from,
                    double to);
  double       distModulo(double          from,
                          double          to,
                          motionDirection direction);

  double         checkModuloPos(double pos);
  virtual double updateSetpoint(double nextSetpoint,
                                double nextVelocity,
                                double nextAcceleration,
                                bool   busy);

  double targetAcceleration_;
  double targetDeceleration_;
  double targetDecelerationEmerg_;
  double targetVelocity_;
  double targetPosition_;
  double targetJerk_;
  double sampleTime_;
  double posSetMinus1_;
  double currentPositionSetpoint_;
  double currentVelocitySetpoint_;
  double currentAccelerationSetpoint_;
  bool busy_;
  int index_;
  bool execute_;
  bool executeOld_;
  double startPosition_;
  bool enable_;
  bool enableOld_;
  bool internalStopCmd_;
  double distToStop_;
  motionMode motionMode_;
  interlockTypes interlockStatus_;
  ecmcAxisData *data_;
  stopMode latchedStopMode_;
  double externalVeloLimit_;
  int externalVeloLimitActive_;
};
#endif  // ifndef SRC_ECMCTRAJECTORYBASE_H_
