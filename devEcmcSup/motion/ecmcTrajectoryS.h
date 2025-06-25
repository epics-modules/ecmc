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

using namespace ruckig;

class ecmcTrajectoryS : public ecmcTrajectoryBase {
public:
  ecmcTrajectoryS(ecmcAxisData *axisData,
                  double        sampleTime);
  ~ecmcTrajectoryS();

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
#endif  // ifndef SRC_ECMCTRAJECTORYS_H_
