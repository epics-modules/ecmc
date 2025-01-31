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

#include "ecmcEcEntryLink.h"
#include "ecmcAxisPVTSequence.h"
#include <vector>

class ecmcPVTController: public ecmcEcEntryLink {
  public:
    ecmcPVTController(double sampleTime);
    ~ecmcPVTController();
    void addPVTAxis(ecmcAxisPVTSequence* axis);
    void clearPVTAxes();
    size_t getCurrentPointId();
    size_t getCurrentTriggerId();
    double getCurrentTime();
    void execute();
    void setExecute(bool execute);

  private:
    double sampleTime_;
    double nextTime_, accTime_, endTime_;
    void checkIfTimeToTrigger();
    std::vector<ecmcAxisPVTSequence*> pvt_;
    bool executeOld_, execute_;
};
#endif  /* ECMCPVTCONTROLLER_H_ */
