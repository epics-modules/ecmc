#ifndef ECMCDRIVESTEPPER_H_
#define ECMCDRIVESTEPPER_H_

#include <stdio.h>
#include <cmath>
#include "../general/ecmcDefinitions.h"
#include "../general/ecmcError.h"
#include "../ethercat/ecmcEcEntry.h"
#include "../ethercat/ecmcEcEntryLink.h"
#include "../ethercat/ecmcEcPdo.h"
#include "ecmcDriveBase.h"

class ecmcDriveStepper : public ecmcDriveBase {
 public:
  explicit ecmcDriveStepper(ecmcAxisData *axisData);
  ecmcDriveStepper(ecmcAxisData *axisData,
                   double        scale);
  ~ecmcDriveStepper();
  int  validate();
  void writeEntries();
  void readEntries();
  void printCurrentState();

 private:
  void initVars();
};
#endif  // ifndef ECMCDRIVESTEPPER_H_
