/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcGlobalsExtern.h
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom
*
* TODO: REDO WITHOUT GLOBALS
*
\*************************************************************************/

#ifndef ECMC_GLOBALS_EXTERN_H_
#define ECMC_GLOBALS_EXTERN_H_

#include "ecmcEc.h"
#include "ecmcAxisBase.h"
#include "ecmcDataStorage.h"
#include "ecmcPLCMain.h"
#include "ecmcMotion.h"
#include "ecmcEthercat.h"
#include "ecmcMotorRecordController.h"
#include "ecmcPluginLib.h"
#include "ecmcPVTController.h"
#include "ecmcLookupTable.h"
#include "epicsMutex.h"
#include "ecmcMasterSlaveStateMachine.h"

extern ecmcAxisBase *axes[ECMC_MAX_AXES];
extern ecmcAxisGroup *axisGroups[ECMC_MAX_AXES];
extern size_t axisGroupCounter;
extern ecmcEc *ec;
extern ecmcDataStorage  *dataStorages[ECMC_MAX_DATA_STORAGE_OBJECTS];
extern ecmcPLCMain *plcs;
extern ecmcAsynPortDriver *asynPort;
extern ecmcAsynDataItem   *mainAsynParams[ECMC_ASYN_MAIN_PAR_COUNT];
extern ecmcMainThreadDiag  threadDiag;
extern app_mode_type appModeCmd, appModeCmdOld, appModeStat;
extern ecmcMotorRecordController *asynPortMotorRecord;
extern ecmcPluginLib *plugins[ECMC_MAX_PLUGINS];
extern ecmcPluginLib *safetyplugin;
extern ecmcShm shmObj;
extern ecmcPVTController *pvtCtrl_;
extern ecmcLookupTable<double, double>  *luts[ECMC_MAX_LUTS];
extern ecmcMasterSlaveStateMachine *masterSlaveSMs[ECMC_MAX_MST_SLVS_SMS];

// Mutex for motor record access
extern epicsMutexId ecmcRTMutex;
extern int axisDiagIndex;
extern int axisDiagFreq;
extern int controllerError;
extern int controllerErrorOld;
extern int controllerReset;
extern const char *controllerErrorMsg;
extern uint64_t    ecmcUpdatedCounter;
extern int asynSkipCyclesFastest;
extern int asynSkipUpdateCounterFastest;
extern int ecTimeoutSeconds;
extern double mcuFrequency;
extern double mcuPeriod;
extern int    sampleRateChangeAllowed;
extern int    pluginsError;
extern int    safetypluginError;
extern int    currentMasterSlaveSMIndex;
extern int    allowCallbackEpicsState;
#endif  /* ECMC_GLOBALS_EXTERN_H_ */
