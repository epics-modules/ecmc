/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcTrajectoryPVT.h
*
*  Created on: Nov 26, 2021
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef SRC_ECMCTRAJECTORYPVT_H_
#define SRC_ECMCTRAJECTORYPVT_H_

#include "ecmcTrajectoryBase.h"
#include "ecmcAxisPVTSequence.h"

/**
 * \class ecmcTrajectoryPVT
 *
 * \ingroup ecmc
 *
 * \brief Implements a PVT (position velocity time) motion setpoint trajectory
 *
 * This class implements S-curve trajectory nased on based on ruckig sources
 *
 * Supports:
 * 1. Constant velocity
 * 2. Relative positioning
 * 3. Absolute positioning
 * 4. Interlocks (hard limits, soft limits, external interlocks)
 *
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
#define ERROR_TRAJ_RUCKIG_JERK_ZERO 0x14E87

using namespace ruckig;

class ecmcTrajectoryPVT : public ecmcTrajectoryBase {
public:
  ecmcTrajectoryPVT(ecmcAxisData *axisData,
                  double        sampleTime);
  ~ecmcTrajectoryPVT();

  /** \brief Sets target velocity of trajectory (max velocity).
  * Note: This is the max velocity of the trajectory generator. The actual velocity can be higher.
  */
  void   setTargetVel(double velTarget);

  /// Sets target position (end position of trajectory).
  void   setTargetPosLocal(double pos);

  /// Sets position setpoint.
  void   setCurrentPosSet(double posSet);

  double distToStop(double vel);
  int    initStopRamp(double currentPos,
                      double currentVel,
                      double currentAcc);
  int    setExecute(bool execute);
  int    validate();

private:
  void   initRuckig();
  bool   updateRuckig();
  double internalTraj(double *actVelocity,
                      double *actAcceleration,
                      bool   *trajBusy);
  double moveVel(double *actVelocity,
                 double *actAcceleration,
                 bool   *trajBusy);
  double movePos(double *actVelocity,
                 double *actAcceleration,
                 bool   *trajBusy);
  double moveStop(stopMode stopMode,
                  double  *actVelocity,
                  double  *actAcceleration,
                  bool    *stopped);

  // Ruckig
  void initVars();
  Ruckig<DynamicDOFs> *otg_;
  InputParameter<DynamicDOFs> *input_;
  OutputParameter<DynamicDOFs> *output_;
  double stepNOM_;
  double localCurrentPositionSetpoint_;
  double targetPositionLocal_;
  double targetVelocityLocal_;
  bool localBusy_;
  double trajMaxVelo_;
};
#endif  // ifndef SRC_ECMCTRAJECTORYPVT_H_
