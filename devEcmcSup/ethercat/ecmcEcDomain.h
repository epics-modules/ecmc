/*************************************************************************\
* Copyright (c) 2023 Paul Scherrer Institute
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcEcDomain.h
*
*  Created on: Sept 29, 2023
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCECDOMAIN_H_
#define ECMCECDOMAIN_H_

#include "ecrt.h"
#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "ecmcAsynPortDriver.h"
#include "ecmcErrorsList.h"

class ecmcEcDomain : public ecmcError {
public:
  ecmcEcDomain(ecmcAsynPortDriver *asynPortDriver,
               ec_master_t        *master,
               int                 masterIndex,
               int                 objIndex,
               int                 exeCycles,
               int                 offsetCycles);
  ~ecmcEcDomain();
  ec_domain_t* getDomain();
  int          setAllowOffline(int allow);
  int          getAllowOffline();
  int          checkState();
  void         process();
  void         queue();
  void         updateAsyn();
  int          initAsyn();
  size_t       getSize();
  int          setFailedCyclesLimitInterlock(int cycles);
  void         slowExecute();
  uint8_t*     getDataPtr();
  int          getOK();

private:
  void         initVars();
  int objIndex_;
  int masterIndex_;
  ec_master_t *master_;
  ec_domain_t *domain_;

  // ec_domain_state_t stateOld_;
  ec_domain_state_t state_;
  int statusOk_;
  uint32_t statusWordOld_;
  int notOKCounter_;
  int notOKCounterTotal_;
  int notOKCounterMax_;
  int notOKCyclesLimit_;

  size_t size_;
  uint32_t statusWord_;
  int allowOffLine_;
  ecmcAsynPortDriver *asynPortDriver_;
  ecmcAsynDataItem *asynParStat_;
  ecmcAsynDataItem *asynParFailCount_;
  uint8_t *domainPd_;
  int exeCycles_;
  int offsetCycles_;
  int cycleCounter_;
};
#endif  /* ECMCECDOMAIN_H_ */
