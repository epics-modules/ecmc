/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPIDController.h
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCPIDCONTROLLER_H
#define ECMCPIDCONTROLLER_H

#define ERROR_BASE_PID 0x14500

#include "../main/ecmcError.h"
#include "ecmcAxisData.h"
#include "../main/ecmcDefinitions.h"
#include "../com/ecmcAsynPortDriver.h"
#include "../main/ecmcErrorsList.h"

// CONTROLLER ERRORS
#define ERROR_CNTRL_INVALID_SAMPLE_TIME 0x15000

class ecmcPIDController : public ecmcError {
 public:
  ecmcPIDController(ecmcAsynPortDriver *asynPortDriver,
                    ecmcAxisData *axisData,
                    double        sampleTime);
  ecmcPIDController(ecmcAsynPortDriver *asynPortDriver,
                    ecmcAxisData *axisData,
                    double        kp,
                    double        ki,
                    double        kd,
                    double        kff,
                    double        sampleTime,
                    double        outMax,
                    double        outMin);
  ~ecmcPIDController();
  void   initVars();
  void   reset();
  void   setIRange(double iMax,
                   double iMin);
  double control(double posError,                
                 double ff);
  double getOutPPart();
  double getOutIPart();
  double getOutDPart();
  double getOutFFPart();
  double getOutTot();
  void   setKp(double kp);
  void   setKi(double ki);
  void   setKd(double kd);
  void   setKff(double kff);
  double   getKp();
  double   getKi();
  double   getKd();
  double   getKff();
  void   setOutMax(double outMax);
  void   setOutMin(double outMin);
  void   setIOutMax(double outMax);
  void   setIOutMin(double outMin);
  int    validate();

 private:
  int    initAsyn();
  double kp_, ki_, kd_, kff_;
  double outputP_;
  double outputI_;
  double outputD_;
  double outputIMax_;    // For Integrator
  double outputIMin_;
  double outputMax_;     // For combined PID output
  double outputMin_;
  double ff_;
  double controllerErrorOld_;
  double sampleTime_;
  ecmcAxisData *data_;
  bool   settingMade_;

  // Asyn
  ecmcAsynPortDriver     *asynPortDriver_;
  ecmcAsynDataItem       *asynKp_;
  ecmcAsynDataItem       *asynKi_;
  ecmcAsynDataItem       *asynKd_;
  ecmcAsynDataItem       *asynKff_;
};
#endif  // ifndef ECMCPIDCONTROLLER_H
