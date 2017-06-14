#ifndef ECMCASYNLINK_H_
#define ECMCASYNLINK_H_
#include "ecrt.h"
#include "stdio.h"
#include <string>
#include <cmath>

#include "ecmcAsynPortDriver.h"

class ecmcAsynLink
{
public:
  ecmcAsynLink();
  ~ecmcAsynLink();
  void initVars();
  int setAsynParameterIndex(int index);
  int setAsynParameterSkipCycles(int skipCycles);
  int getAsynParameterIndex();
  int setAsynParameterType(asynParamType parType);
  int getAsynParameterType();
  int setAsynPortDriver(ecmcAsynPortDriver *asynPortDriver);
protected:
  int asynParameterIndex_;
  asynParamType asynParameterType_;
  ecmcAsynPortDriver *asynPortDriver_;
  int asynUpdateCycles_;
  int asynUpdateCycleCounter_;
};
#endif /* ECMCASYNLINK_H_ */
