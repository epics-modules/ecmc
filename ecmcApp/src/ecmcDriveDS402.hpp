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

#define ERROR_DRV_DS402_CONTROL_WORD_BIT_COUNT_ERROR 0x14650
#define ERROR_DRV_DS402_STATUS_WORD_BIT_COUNT_ERROR 0x14651

class ecmcDriveDS402: public ecmcDriveBase
{
public:
  ecmcDriveDS402();
  ecmcDriveDS402(double scale);
  ~ecmcDriveDS402();
  bool getEnable();
  bool getEnabled();
  int setEnable(bool enable);
  int validate();
private:

};
#endif
