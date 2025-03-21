/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAxisVirt.h
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCAXISVIRT_H_
#define ECMCAXISVIRT_H_

#include "ecmcAxisBase.h"

// VIRTUAL AXIS
#define ERROR_VIRT_AXIS_TRAJ_NULL 0x14F00
#define ERROR_VIRT_AXIS_CREATE_TRANSFORM_INDEX_OUT_OF_RANGE 0x14F01
#define ERROR_VIRT_AXIS_INDEX_OUT_OF_RANGE 0x14F02
#define ERROR_VIRT_AXIS_LINKED_AXIS_TRAJ_NULL 0x14F03
#define ERROR_VIRT_AXIS_INDEX_NULL 0x14F04

class ecmcAxisVirt : public ecmcAxisBase {
public:
  ecmcAxisVirt(ecmcAsynPortDriver *asynPortDriver,
               int                 axisID,
               double              sampleTime,
               ecmcTrajTypes       trajType);
  ~ecmcAxisVirt();
  void               execute(bool masterOK);
  ecmcDriveBase*     getDrv();
  ecmcPIDController* getCntrl();
  int                validate();

private:
  void               initVars();
};

#endif /* ECMCAXISVIRT_H_ */
