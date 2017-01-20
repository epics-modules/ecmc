/*
 * cMcuAxisReal.h
 *
 *  Created on: Mar 10, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCAXISREAL_H_
#define ECMCAXISREAL_H_

#include <stdint.h>
#include "ecmcAxisBase.h"
#include "ecmcEncoder.h"

class ecmcAxisReal: public ecmcAxisBase
{
public:
  ecmcAxisReal(int axisID, double sampleTime);
  ~ecmcAxisReal();
  void execute(bool masterOK);
  int setOpMode(operationMode mode);
  operationMode getOpMode();
  int getCntrlError(double* error);
  int setEnable(bool enable);
  bool getEnable();
  bool getEnabled();
  int setDriveType(ecmcDriveTypes driveType);
  ecmcDriveBase *getDrv();
  ecmcPIDController *getCntrl();
  void printStatus();
  int validate();

private:
  void initVars();
  bool initDone_;
  operationMode operationMode_;
  ecmcDriveBase *drv_;
  ecmcPIDController *cntrl_;
  ecmcDriveTypes currentDriveType_;
  bool enabledOld_;
  bool enableCmdOld_;
  bool executeCmdOld_;
  bool trajInterlockOld;
};

#endif /* ECMCAXISREAL_H_ */
