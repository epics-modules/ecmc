#ifndef ECMCPIDCONTROLLER_H
#define ECMCPIDCONTROLLER_H

#define ERROR_BASE_PID 0x14500

#include "ecmcError.h"
#include "ecmcAxisData.h"

// CONTROLLER ERRORS
#define ERROR_CNTRL_INVALID_SAMPLE_TIME 0x15000

class ecmcPIDController : public ecmcError {
 public:
  ecmcPIDController(ecmcAxisData *axisData,
                    double        sampleTime);
  ecmcPIDController(ecmcAxisData *axisData,
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
  double control(double set,
                 double act,
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
  void   setOutMax(double outMax);
  void   setOutMin(double outMin);
  void   setIOutMax(double outMax);
  void   setIOutMin(double outMin);
  int    validate();
  void   printCurrentState();

 private:
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
};
#endif  // ifndef ECMCPIDCONTROLLER_H
