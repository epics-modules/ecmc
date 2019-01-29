
/*
 * ecmcGlobalsExtern.h
 *
 *  Created on: Jan 10, 2019
 *      Author: anderssandstrom
 */

/* TODO: REDO WITHOUT GLOBALS */

#ifndef ECMC_GLOBALS_EXTERN_H_
#define ECMC_GLOBALS_EXTERN_H_

#include "../ethercat/ecmcEc.h"
#include "../motion/ecmcAxisBase.h"
#include "../misc/ecmcEvent.h"
#include "../misc/ecmcDataRecorder.h"
#include "../misc/ecmcDataStorage.h"
#include "../misc/ecmcCommandList.h"
#include "../plc/ecmcPLCMain.h"
#include "../motion/ecmcMotion.h"
#include "../ethercat/ecmcEthercat.h"


extern ecmcAxisBase *axes[ECMC_MAX_AXES];
extern int axisDiagIndex;
extern int axisDiagFreq;
extern ecmcEc ec;

extern ecmcEvent          *events[ECMC_MAX_EVENT_OBJECTS];
extern ecmcDataRecorder   *dataRecorders[ECMC_MAX_DATA_RECORDERS_OBJECTS];
extern ecmcDataStorage    *dataStorages[ECMC_MAX_DATA_STORAGE_OBJECTS];
extern ecmcCommandList    *commandLists[ECMC_MAX_COMMANDS_LISTS];
extern ecmcPLCMain        *plcs;
extern ecmcAsynPortDriver *asynPort;
extern ecmcAsynDataItem   *mainAsynParams[ECMC_ASYN_MAIN_PAR_COUNT];
extern ecmcMainThreadDiag threadDiag;

// Default asyn params
extern int asynParIdLatencyMin;
extern int asynParIdLatencyMax;
extern int asynParIdExecuteMin;
extern int asynParIdExecuteMax;
extern int asynParIdPeriodMin;
extern int asynParIdPeriodMax;
extern int asynParIdSendMin;
extern int asynParIdSendMax;
extern int asynParIdEcmcAppMode;
extern int asynParIdEcmcErrorId;
extern int asynParIdEcmcErrorMsg;
extern int asynSkipCyclesThread;
extern int asynUpdateCounterThread;
extern int asynThreadParamsEnable;
extern int asynSkipCyclesFastest;
extern int asynSkipUpdateCounterFastest;
extern int asynParIdEcmcErrorReset;

#endif  /* ECMC_GLOBALS_EXTERN_H_ */
