/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcDriveDS402.h
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCDRIVEDS402_H_
#define ECMCDRIVEDS402_H_
#include <stdio.h>
#include <cmath>

#include "../main/ecmcDefinitions.h"
#include "../main/ecmcError.h"
#include "../ethercat/ecmcEcEntry.h"
#include "../ethercat/ecmcEcEntryLink.h"
#include "../ethercat/ecmcEcPdo.h"
#include "ecmcDriveBase.h"

// Error codes
#define ERROR_DRV_DS402_CONTROL_WORD_BIT_COUNT_ERROR 0x14650
#define ERROR_DRV_DS402_STATUS_WORD_BIT_COUNT_ERROR 0x14651
#define ERROR_DRV_DS402_STATE_MACHINE_TIME_OUT  0x14652
#define ERROR_DRV_DS402_CONTROL_WORD_START_BIT_ERROR 0x14653
#define ERROR_DRV_DS402_STATUS_WORD_START_BIT_ERROR 0x14654
#define ERROR_DRV_DS402_FAULT_STATE 0x14655

// Timeout in seconds (mostly for startup)
#define ERROR_DRV_DS402_STATE_MACHINE_TIME_OUT_TIME 15

/** From EL7201 docs
 * bit 0: Ready to swtich on
 * bit 1: Switched on
 * bit 2: Operation Enabeld
 * bit 3: Fault
 * bit 4:
 * bit 5:
 * bit 6: Switch on disabled
 * bit 7: Warning
 * bit 8: 
 * bit 9:
 * bit A: TxPDOToggle
 * bit B: Internal limit active
 * bit C: Target value ignored
 * bit D:
 * bit R:
 * bit F:*/
#define ECMC_DS402_READY_TO_SWITCH_ON_BIT 0
#define ECMC_DS402_SWITCHED_ON_BIT 1
#define ECMC_DS402_OPERATION_ENABLED_BIT 2
#define ECMC_DS402_FAULT_BIT 3
#define ECMC_DS402_SWITCH_ON_DISABLED_BIT 6
#define ECMC_DS402_SWITCH_ON_WARNING_BIT 7
#define ECMC_DS402_SWITCH_ON_INT_LIM 11


enum stateMachine_DS402 {
  ECMC_DS402_IDLE_STATE               = 0,
  ECMC_DS402_RESET_STATE              = 1,
  ECMC_DS402_SWITCH_ON_DISABLED_STATE = 2,
  ECMC_DS402_STARTUP_RESET            = 3,
  ECMC_DS402_READY_TO_SWITCH_ON_STATE = 4,
  ECMC_DS402_SWITCHED_ON_STATE        = 5,
  ECMC_DS402_OPERATION_ENABLED_STATE  = 6,
  ECMC_DS402_FAULT_STATE              = 7,
};

class ecmcDriveDS402 : public ecmcDriveBase {
 public:
  explicit ecmcDriveDS402(ecmcAsynPortDriver *asynPortDriver,
                          ecmcAxisData *axisData);
  ecmcDriveDS402(ecmcAsynPortDriver *asynPortDriver,
                 ecmcAxisData *axisData,
                 double        scale);
  ~ecmcDriveDS402();
  int  validate();
  void readEntries();
  void writeEntries();
  void errorReset();
  bool getEnabledLocal();

 private:
  void initVars();
  stateMachine_DS402 enableStateMachine_;  
  stateMachine_DS402 enableStateMachineOld_;
  int cycleCounter_;
  bool ds402WarningOld_;
  bool localEnabled_;
};
#endif  // ifndef ECMCDRIVEDS402_H_

