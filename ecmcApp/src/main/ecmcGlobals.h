
/*
 * ecmcGlobals.h
 *
 *  Created on: Jan 10, 2019
 *      Author: anderssandstrom
 */

#ifndef ECMC_GLOBALS_H_
#define ECMC_GLOBALS_H_

#include "../ethercat/ecmcEc.h"
#include "../motion/ecmcAxisBase.h"
#include "../misc/ecmcEvent.h"
#include "../misc/ecmcDataRecorder.h"
#include "../misc/ecmcDataStorage.h"
#include "../misc/ecmcCommandList.h"
#include "../plc/ecmcPLCMain.h"
#include "../motion/ecmcMotion.h"

ecmcAxisBase *axes[ECMC_MAX_AXES];
int axisDiagIndex;
int axisDiagFreq;
ecmcEc ec;

ecmcEvent          *events[ECMC_MAX_EVENT_OBJECTS];
ecmcDataRecorder   *dataRecorders[ECMC_MAX_DATA_RECORDERS_OBJECTS];
ecmcDataStorage    *dataStorages[ECMC_MAX_DATA_STORAGE_OBJECTS];
ecmcCommandList    *commandLists[ECMC_MAX_COMMANDS_LISTS];
ecmcPLCMain        *plcs;
ecmcAsynPortDriver *asynPort = NULL;

// Default asyn params
int asynParIdLatencyMin          = 0;
int asynParIdLatencyMax          = 0;
int asynParIdExecuteMin          = 0;
int asynParIdExecuteMax          = 0;
int asynParIdPeriodMin           = 0;
int asynParIdPeriodMax           = 0;
int asynParIdSendMin             = 0;
int asynParIdSendMax             = 0;
int asynParIdEcmcAppMode         = 0;
int asynParIdEcmcErrorId         = 0;
int asynParIdEcmcErrorMsg        = 0;
int asynSkipCyclesThread         = 0;
int asynUpdateCounterThread      = 0;
int asynThreadParamsEnable       = 0;
int asynSkipCyclesFastest        = -1;
int asynSkipUpdateCounterFastest = 0;
int asynParIdEcmcErrorReset      = 0;

#endif  /* ECMC_GLOBALS_H_ */
