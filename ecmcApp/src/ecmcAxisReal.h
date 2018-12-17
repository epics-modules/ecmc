/*
 * cMcuAxisReal.h
 *
 *  Created on: Mar 10, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCAXISREAL_H_
#define ECMCAXISREAL_H_

#include <stdint.h>
#include <new>
#include "ecmcAxisBase.h"
#include "ecmcEncoder.h"

class ecmcAxisReal : public ecmcAxisBase {
 public:
  ecmcAxisReal(int    axisID,
               double sampleTime);
  ~ecmcAxisReal();
  void               execute(bool masterOK);
  int                setOpMode(operationMode mode);
  operationMode      getOpMode();
  int                getCntrlError(double *error);
  int                setEnable(bool enable);
  int                setDriveType(ecmcDriveTypes driveType);
  ecmcDriveBase    * getDrv();
  ecmcPIDController* getCntrl();
  int                validate();
  void               printCurrentState();

 protected:
  void               refreshDebugInfoStruct();

 private:
  void               printOpModeState();
  void               printDriveType();
  void               initVars();
  bool initDone_;
  bool temporaryLocalTrajSource_;
  ecmcDriveBase *drv_;
  ecmcPIDController *cntrl_;
  ecmcDriveTypes currentDriveType_;
};

#endif  /* ECMCAXISREAL_H_ */
