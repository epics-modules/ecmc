/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcAxisReal.h
*
*  Created on: Mar 10, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCAXISREAL_H_
#define ECMCAXISREAL_H_

#include <stdint.h>
#include <new>
#include "ecmcAxisBase.h"
#include "ecmcEncoder.h"

class ecmcAxisReal : public ecmcAxisBase {
 public:
  ecmcAxisReal(ecmcAsynPortDriver *asynPortDriver,
               int    axisID,
               double sampleTime,
               ecmcDriveTypes drvType,
               ecmcTrajTypes  trajType);
  ~ecmcAxisReal();
  void               execute(bool masterOK);
  ecmcDriveBase    * getDrv();
  ecmcPIDController* getCntrl();
  int                validate();

 private:
  void               initVars(); 
  bool temporaryLocalTrajSource_;
  ecmcDriveBase *drv_;
  ecmcPIDController *cntrl_;
  ecmcDriveTypes currentDriveType_;
};

#endif  /* ECMCAXISREAL_H_ */
