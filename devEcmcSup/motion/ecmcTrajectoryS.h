/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcTrajectoryS.h
*
*  Created on: Nov 26, 2021
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef SRC_ECMCTRAJECTORYS_H_
#define SRC_ECMCTRAJECTORYS_H_

#include "ecmcTrajectoryBase.h"
#include <ruckig.hpp>

/**
 * \class ecmcTrajectoryS
 *
 * \ingroup ecmc
 *
 * \brief Implements a S-curve motion setpoint trajectory
 *
 * This class implements S-curve trajectory nased on based on ruckig sources
 * 
 * Supports:
 * 1. Constant velocity
 * 2. Relative positioning
 * 3. Absolute positioning
 * 4. Interlocks (hard limits, soft limits, external interlocks)
 *
 * Contact: anders.sandstrom@esss.se
 *
 * Created on: 2021-11-26
 *
 */

#define ERROR_TRAJ_RUCKIG_ERROR 0x14E80
#define ERROR_TRAJ_RUCKIG_INVALID_INPUT 0x14E81
#define ERROR_TRAJ_RUCKIG_TRAJ_DURATION 0x14E82
#define ERROR_TRAJ_RUCKIG_POS_LIMITS 0x14E83
#define ERROR_TRAJ_RUCKIG_NO_PHASE_SYNC 0x14E84
#define ERROR_TRAJ_RUCKIG_EXE_TIME_CALC 0x14E85
#define ERROR_TRAJ_RUCKIG_SYNC_CALC 0x14E86

using namespace ruckig;

class ecmcTrajectoryS : public ecmcTrajectoryBase {
 public:
  ecmcTrajectoryS(ecmcAxisData *axisData,
                  double        sampleTime);
  ~ecmcTrajectoryS();

  /** \brief Calculates and returns next position setpoint.
   * This function should only be executed once for each sample period.
   */
  double          getNextPosSet();

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
  void            setTargetPos(double pos);

  /// Enable traj
  void            setEnable(bool enable);

  /// Sets position setpoint.
  void            setCurrentPosSet(double posSet);

  double          distToStop(double vel);
  int             initStopRamp(double   currentPos,
                               double   currentVel,
                               double   currentAcc);
  int             setExecute(bool execute);

 private:
  void            initRuckig();
  bool            updateRuckig();
  double          internalTraj(double  *actVelocity,
                               double  *actAcceleration,
                               bool    *trajBusy);
  double          moveVel(double       *actVelocity,
                          double       *actAcceleration,
                          bool         *trajBusy);
  double          movePos(double       *actVelocity,
                          double       *actAcceleration,
                          bool         *trajBusy);
  double          moveStop(stopMode     stopMode,
                           double      *actVelocity,
                           double      *actAcceleration,
                           bool        *stopped);
//  double          updateSetpoint(double nextSetpoint,
//                                 double nextVelocity,
//                                 double nextAcceleration);
  // Ruckig
  void            initVars();
  Ruckig<DynamicDOFs>          *otg_;
  InputParameter<DynamicDOFs>  *input_;
  OutputParameter<DynamicDOFs> *output_;
  double                        stepNOM_;
  double localCurrentPositionSetpoint_;
  bool   localBusy_;
};
#endif  // ifndef SRC_ECMCTRAJECTORYS_H_
