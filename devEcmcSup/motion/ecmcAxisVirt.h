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
