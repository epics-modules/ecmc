#ifndef ECMCDRIVEDS402_H_
#define ECMCDRIVEDS402_H_
#include <stdio.h>
#include <cmath>

#include "ecmcDefinitions.h"
#include "ecmcEcEntry.h"
#include "ecmcEcEntryLink.h"
#include "ecmcEcPdo.h"
#include "ecmcError.h"
#include "ecmcDriveBase.hpp"

// Error codes
#define ERROR_DRV_DS402_CONTROL_WORD_BIT_COUNT_ERROR 0x14650
#define ERROR_DRV_DS402_STATUS_WORD_BIT_COUNT_ERROR 0x14651
#define ERROR_DRV_DS402_STATE_MACHINE_TIME_OUT  0x14652
#define ERROR_DRV_DS402_CONTROL_WORD_START_BIT_ERROR 0x14653
#define ERROR_DRV_DS402_STATUS_WORD_START_BIT_ERROR 0x14654


#define ERROR_DRV_DS402_STATE_MACHINE_TIME_OUT_TIME 5000

#define ECMC_DS402_STATUS_MASK_1 0x004F // Important bits xxxx.xxxx.x1xx.1111
#define ECMC_DS402_STATUS_MASK_2 0x006F // Important bits xxxx.xxxx.x11x.1111

#define ECMC_DS402_INVALID_STATE_STATUS -1  // Mask 1
#define ECMC_DS402_NOT_READY_TO_SWITCH_ON_STATUS 0x0000  // Mask 1
#define ECMC_DS402_SWITCH_ON_DISABLED_STATUS 0x0040 // Mask 1
#define ECMC_DS402_READY_TO_SWITCH_ON_STATUS 0x0021 // Mask 2
#define ECMC_DS402_SWITCHED_ON_STATUS 0x0023 // Mask 2
#define ECMC_DS402_OPERATION_ENABLED_STATUS 0x0027 // Mask 2
#define ECMC_DS402_QUICK_STOP_ACTIVE_STATUS 0x0007 // Mask 2
#define ECMC_DS402_FAULT_REACTION_ACTIVE_STATUS 0x000F // Mask 1
#define ECMC_DS402_FAULT_STATUS 0x0008 // Mask 1

enum stateMachine_DS402{
  ECMC_DS402_RESET_STATE=1,
  ECMC_DS402_SWITCH_ON_DISABLED_STATE=2,
  ECMC_DS402_READY_TO_SWITCH_ON_STATE=3,
  ECMC_DS402_SWITCHED_ON_STATE=4,
  ECMC_DS402_OPERATION_ENABLED_STATE=5,
};

class ecmcDriveDS402: public ecmcDriveBase
{
public:
  ecmcDriveDS402(ecmcAxisData *axisData);
  ecmcDriveDS402(ecmcAxisData *axisData,double scale);
  ~ecmcDriveDS402();
  int validate();
  void readEntries();
  void writeEntries();
  void printCurrentState();
private:
  void initVars();
  int checkDS402State();
  int driveState_;
  bool enableSequenceRunning_;
  stateMachine_DS402 enableStateMachine_;
  int driveStateOld_;
  stateMachine_DS402 enableStateMachineOld_;
  int cycleCounter;
};
#endif
