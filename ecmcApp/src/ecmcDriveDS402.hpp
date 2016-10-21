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

#define ERROR_DRV_DS402_CONTROL_WORD_BIT_COUNT_ERROR 0x14650
#define ERROR_DRV_DS402_STATUS_WORD_BIT_COUNT_ERROR 0x14651

/*enum stateMachine_DS402{
  ECMC_DS402_NOT_READY_TO_SWITCH_ON=0,
  ECMC_DS402_SWITCH_ON_DISABLED=1,
  ECMC_DS402_READY_TO_SWITCH_ON=2,
  ECMC_DS402_SWITCHED_ON=3,
  ECMC_DS402_OPERATION_ENABLED=4,
  ECMC_DS402_QUICK_STOP_ACTIVE=5,
  ECMC_DS402_FAULT_REACTION_ACTIVE=6,
  ECMC_DS402_FAULT=7,
};*/

#define ECMC_DS402_STATUS_MASK_1 0x004F // Important bits xxxx.xxxx.x1xx.1111
#define ECMC_DS402_STATUS_MASK_2 0x006F // Important bits xxxx.xxxx.x11x.1111

#define ECMC_DS402_INVALID_STATE_STATUS -1  // Mask 1
#define ECMC_DS402_NOT_READY_TO_SWITCH_ON_STATUS 0x0000  // Mask 1
#define ECMC_DS402_SWITCH_ON_DISABLED_STATUS 0x0060 // Mask 1
#define ECMC_DS402_READY_TO_SWITCH_ON_STATUS 0x0021 // Mask 2
#define ECMC_DS402_SWITCHED_ON_STATUS 0x0023 // Mask 2
#define ECMC_DS402_OPERATION_ENABLED_STATUS 0x0027 // Mask 2
#define ECMC_DS402_QUICK_STOP_ACTIVE_STATUS 0x0007 // Mask 2
#define ECMC_DS402_FAULT_REACTION_ACTIVE_STATUS 0x000F // Mask 1
#define ECMC_DS402_FAULT_STATUS 0x0008 // Mask 1

enum stateMachine_DS402{
  ECMC_DS402_RESET_STATE=0,
  ECMC_DS402_SWITCH_ON_DISABLED_STATE=1,
  ECMC_DS402_READY_TO_SWITCH_ON_STATE=2,
  ECMC_DS402_SWITCHED_ON_STATE=3,
  ECMC_DS402_OPERATION_ENABLED_STATE=4,
};

class ecmcDriveDS402: public ecmcDriveBase
{
public:
  ecmcDriveDS402();
  ecmcDriveDS402(double scale);
  ~ecmcDriveDS402();
  int setEnable(bool enable);
  int validate();
  void readEntries();
  void writeEntries();

private:
  void initVars();
  int checkDS402State();
  int driveState_;
  bool enableCmdOld_;
  bool enableSequenceRunning_;
  stateMachine_DS402 enableStateMachine_;
};
#endif
