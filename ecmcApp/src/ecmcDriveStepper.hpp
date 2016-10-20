#ifndef ECMCDRIVESTEPPER_H_
#define ECMCDRIVESTEPPER_H_
#include <stdio.h>
#include <cmath>

#include "ecmcDefinitions.h"
#include "ecmcEcEntry.h"
#include "ecmcEcEntryLink.h"
#include "ecmcEcPdo.h"
#include "ecmcError.h"
#include "ecmcDriveBase.hpp"

class ecmcDriveStepper : public ecmcDriveBase
{
public:
  ecmcDriveStepper();
  ecmcDriveStepper(double scale);
  ~ecmcDriveStepper();
  bool getEnable();
  bool getEnabled();
  int setEnable(bool enable);
private:
};
#endif
