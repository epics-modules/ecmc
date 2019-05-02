
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
extern ecmcEc *ec;

extern ecmcEvent          *events[ECMC_MAX_EVENT_OBJECTS];
extern ecmcDataRecorder   *dataRecorders[ECMC_MAX_DATA_RECORDERS_OBJECTS];
extern ecmcDataStorage    *dataStorages[ECMC_MAX_DATA_STORAGE_OBJECTS];
extern ecmcCommandList    *commandLists[ECMC_MAX_COMMANDS_LISTS];
extern ecmcPLCMain        *plcs;
extern ecmcAsynPortDriver *asynPort;
extern ecmcAsynDataItem   *mainAsynParams[ECMC_ASYN_MAIN_PAR_COUNT];
extern ecmcMainThreadDiag threadDiag;
extern app_mode_type appModeCmd, appModeCmdOld, appModeStat;
extern int controllerError;
extern int controllerErrorOld;
extern int controllerReset;
extern const char   *controllerErrorMsg;

extern int asynSkipCyclesFastest;
extern int asynSkipUpdateCounterFastest;

#endif  /* ECMC_GLOBALS_EXTERN_H_ */
