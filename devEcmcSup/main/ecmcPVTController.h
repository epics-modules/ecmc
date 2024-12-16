/*************************************************************************\
* Copyright (c) 2024 Paul Scherrer Institut
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcPVTController.h
*
*  Created on: September 04, 2024
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCPVTCONTROLLER_H_
#define ECMCPVTCONTROLLER_H_
#include <vector>
#include "ecmcAxisBase.h"
#include "ecmcEcEntryLink.h"

class ecmcPVTController: public ecmcEcEntryLink {
  public:
    ecmcPVTController();
    ~ecmcPVTController();
    void addPVTAxis(ecmcAxisBase* axis);
    void clearPVTAxes();
    size_t getCurrentPointId();
    size_t getCurrentTriggerId();
    // linkTriggerOutput
    // setTimeArray
    // compensate cycles..
  private:
    std::vector<ecmcAxisBase*> pvtAxes_;
    double sampleTime_;
};
#endif  /* ECMCPVTCONTROLLER_H_ */
