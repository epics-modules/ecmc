/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcGlobals.h
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

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
#include "../com/ecmcAsynDataItem.h"
#include "../motor/ecmcMotorRecordController.h"
#include "../plugin/ecmcPluginLib.h"
#include "../plugin/ecmcPluginDataRefs.h"
#include "epicsMutex.h"

ecmcAxisBase *axes[ECMC_MAX_AXES];
ecmcEc                    *ec;
ecmcEvent                 *events[ECMC_MAX_EVENT_OBJECTS];
ecmcDataRecorder          *dataRecorders[ECMC_MAX_DATA_RECORDERS_OBJECTS];
ecmcDataStorage           *dataStorages[ECMC_MAX_DATA_STORAGE_OBJECTS];
ecmcCommandList           *commandLists[ECMC_MAX_COMMANDS_LISTS];
ecmcPLCMain               *plcs;
ecmcAsynPortDriver        *asynPort = NULL;
ecmcAsynDataItem          *mainAsynParams[ECMC_ASYN_MAIN_PAR_COUNT];
ecmcMainThreadDiag         threadDiag={0};
app_mode_type              appModeCmd, appModeCmdOld, appModeStat;
ecmcMotorRecordController *asynPortMotorRecord;
ecmcPluginLib             *plugins[ECMC_MAX_PLUGINS];
ecmcPluginDataRefs         pluginDataRefs;

// Mutex for motor record access
epicsMutexId               ecmcRTMutex;
int                        axisDiagIndex;
int                        axisDiagFreq;
int                        controllerError = -1;
int                        controllerErrorOld = -2;
int                        controllerReset = 0;
const char                *controllerErrorMsg = "NO_ERROR";
int32_t                    ecmcUpdatedCounter = 0;
int                        asynSkipCyclesFastest   = -1;
int                        asynSkipUpdateCounterFastest = 0;
int                        ecTimeoutSeconds = EC_START_TIMEOUT_S;
double                     mcuFrequency = MCU_FREQUENCY;
double                     mcuPeriod = MCU_PERIOD_NS;
int                        sampleRateChangeAllowed = 1;
int                        pluginsError = 0;

#endif  /* ECMC_GLOBALS_H_ */
