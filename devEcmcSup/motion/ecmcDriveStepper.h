/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcDriveStepper.h
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCDRIVESTEPPER_H_
#define ECMCDRIVESTEPPER_H_

#include <stdio.h>
#include <cmath>
#include "../main/ecmcDefinitions.h"
#include "../main/ecmcError.h"
#include "../ethercat/ecmcEcEntry.h"
#include "../ethercat/ecmcEcEntryLink.h"
#include "../ethercat/ecmcEcPdo.h"
#include "ecmcDriveBase.h"

class ecmcDriveStepper : public ecmcDriveBase {
 public:
  explicit ecmcDriveStepper(ecmcAsynPortDriver *asynPortDriver,
                            ecmcAxisData *axisData);
  ecmcDriveStepper(ecmcAsynPortDriver *asynPortDriver,
                   ecmcAxisData *axisData,
                   double        scale);
  ~ecmcDriveStepper();
  int  validate();
  void writeEntries();
  void readEntries();
  bool getEnabledLocal();

 private:
  void initVars();
};
#endif  // ifndef ECMCDRIVESTEPPER_H_
