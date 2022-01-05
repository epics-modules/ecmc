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

#include "ecmcTrajectoryBase.h"

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
class ecmcTrajectoryTrapetz : public ecmcTrajectoryBase {
 public:
  ecmcTrajectoryTrapetz(ecmcAxisData *axisData,
                        double        sampleTime);
  ~ecmcTrajectoryTrapetz();

   /** \brief Sets target velocity of trajectory (max velocity).
   * Note: This is the max velocity of the trajectory generator. The actual velocity can be higher.
   */
  void            setTargetVel(double velTarget);
  
  /// Sets acceleration.
  void            setAcc(double acc);

  /// Sets deceleration.
  void            setDec(double dec);

  /** \brief Sets emergency deceleration.
   * Used for ramp down at hard limits.
   */
  void            setEmergDec(double dec);

  /** Currently not implemented (will be used when 
   * s-shaped trajectory is implemented).
   */
  void            setJerk(double jerk);

  /// Sets target position (end position of trajectory).
  void            setTargetPosLocal(double pos);

  /// Enable traj
  void            setEnable(bool enable);

  /// Sets position setpoint.
  void            setCurrentPosSet(double posSet);

  double          distToStop(double vel);
  int             initStopRamp(double currentPos,
                               double currentVel,
                               double currentAcc);
 private:
  void            initVars();
  void            initTraj();
  double          internalTraj(double  *actVelocity,
                               double  *actAcceleration,
                               bool    *trajBusy);
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

  double stepACC_;
  double stepDEC_;
  double stepNOM_;
  double stepDECEmerg_;
  double prevStepSize_;
  double thisStepSize_;
  double stepStableTol_;
  double localCurrentPositionSetpoint_;
  double targetPositionLocal_;
  bool   switchTargetOnTheFly_;
  bool   localBusy_;
};
#endif  // ifndef SRC_ECMCTRAJECTORYTRAPETZ_H_
