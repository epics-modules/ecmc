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
#include <vector>

class ecmcPVTController: public ecmcEcEntryLink {
  public:
    ecmcPVTController(double sampleTime);
    ~ecmcPVTController();
    size_t getCurrentPointId();
    size_t getCurrentTriggerId();
    double getCurrentTime();
    void setCurrentTime(double time);
    void initNewSeq(double offsetTime);

    // linkTriggerOutput
    // setTimeArray
    // compensate cycles..

  private:
    double sampleTime_;
    double currTime_,offsetTime_;
    void checkIfTimeToTrigger();
};
#endif  /* ECMCPVTCONTROLLER_H_ */
