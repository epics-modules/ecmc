/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcMainThread.cpp
*
*  Created on: Jan 11, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#define __STDC_FORMAT_MACROS  // for printf uint_64_t
#include <inttypes.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/mman.h>	
#include <unistd.h>
#include <assert.h>
#include <sched.h>
#include <pthread.h>
#include <exception>
#include <algorithm>
#include <string>

#include "epicsThread.h"
#include "ecmcMainThread.h"
#include "ecmcGeneral.h"
#include "ecrt.h"
#include "ecmcDefinitions.h"
#include "ecmcErrorsList.h"
#include "ecmcGlobals.h"
#include "../com/ecmcOctetIF.h" 
#include "../ethercat/ecmcEthercat.h"
#include "../motion/ecmcMotion.h"
#include "../plc/ecmcPLC.h"
#include "../misc/ecmcMisc.h"
#include "../com/ecmcAsynPortDriver.h"
#include "../motor/ecmcMotorRecordController.h"

/****************************************************************************/
static unsigned int    counter = 0;
static struct timespec masterActivationTimeMonotonic = {};
static struct timespec masterActivationTimeOffset    = {};
static struct timespec masterActivationTimeRealtime  = {};

/*****************************************************************************/

void printStatus() {
  // Print axis diagnostics to screen
  if (PRINT_STDOUT_BIT12() && (axisDiagIndex < ECMC_MAX_AXES) &&
      (axisDiagIndex >= 0)) {
    if (axes[axisDiagIndex] != NULL) {
      axes[axisDiagIndex]->printAxisStatus();
    }
  }
}

void updateAsynParams(int force) {
  
  if(!asynPort->getAllowRtThreadCom()){
    return;
  }

  int errorCode=mainAsynParams[ECMC_ASYN_MAIN_PAR_LATENCY_MIN_ID]->refreshParamRT(force);
  if(errorCode==0){ //Reset after successfull write      
    threadDiag.latency_min_ns  = 0xffffffff;
  }
  errorCode=mainAsynParams[ECMC_ASYN_MAIN_PAR_LATENCY_MAX_ID]->refreshParamRT(force);
  if(errorCode==0){
    threadDiag.latency_max_ns  = 0;
  }
  errorCode=mainAsynParams[ECMC_ASYN_MAIN_PAR_PERIOD_MIN_ID]->refreshParamRT(force);
  if(errorCode==0){
    threadDiag.period_min_ns  = 0xffffffff;
  }
  errorCode=mainAsynParams[ECMC_ASYN_MAIN_PAR_PERIOD_MAX_ID]->refreshParamRT(force);
  if(errorCode==0){
    threadDiag.period_max_ns  = 0;
  }
  errorCode=mainAsynParams[ECMC_ASYN_MAIN_PAR_EXECUTE_MIN_ID]->refreshParamRT(force);
  if(errorCode==0){
    threadDiag.exec_min_ns  = 0xffffffff;
  }
  errorCode=mainAsynParams[ECMC_ASYN_MAIN_PAR_EXECUTE_MAX_ID]->refreshParamRT(force);
  if(errorCode==0){
    threadDiag.exec_max_ns  = 0;
  }
  errorCode=mainAsynParams[ECMC_ASYN_MAIN_PAR_SEND_MIN_ID]->refreshParamRT(force);
  if(errorCode==0){
    threadDiag.send_min_ns  = 0xffffffff;
  }    
  errorCode=mainAsynParams[ECMC_ASYN_MAIN_PAR_SEND_MAX_ID]->refreshParamRT(force);
  if(errorCode==0){
    threadDiag.send_max_ns  = 0;    
  }
  
  controllerErrorOld = controllerError;
  controllerError = getControllerError();
  if(controllerErrorOld != controllerError || force) { // update on change
    controllerErrorMsg = getErrorString(controllerError);    
    errorCode=mainAsynParams[ECMC_ASYN_MAIN_PAR_ERROR_ID_ID]->refreshParamRT(1);    
    errorCode=mainAsynParams[ECMC_ASYN_MAIN_PAR_ERROR_MSG_ID]->refreshParamRT(1,(uint8_t*)controllerErrorMsg,strlen(controllerErrorMsg));
  }

  // Asyn callbacks for all parameters (except arrays)
  if (asynSkipUpdateCounterFastest && !force) {
    asynSkipUpdateCounterFastest--;
  } else {      
    if (asynPort) {
      asynSkipUpdateCounterFastest = asynPort->getFastestUpdateRate()-1;
      if(asynSkipUpdateCounterFastest<0){
        asynSkipUpdateCounterFastest = 0;
      }
      if (asynPort->getAllowRtThreadCom()) {
        asynPort->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);
        /* refresh updated counter (To know in epics when refresh have been made)
        waveform*/
        ecmcUpdatedCounter++;
        mainAsynParams[ECMC_ASYN_MAIN_PAR_UPDATE_READY_ID]->refreshParamRT(1);
      }
    }
  }  
}

// ****** Threading
typedef void (*rtTHREADFUNC)(void *parm);

struct rtThreadOSD{
  pthread_t      thread;
  pthread_attr_t attr;
  void          *usr;
  rtTHREADFUNC   start;
};

typedef struct rtThreadOSD *rtThreadId;

static void* start_routine(void *arg) {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
  rtThreadId thread = (rtThreadId)arg;
  thread->start(thread->usr);
  return NULL;
}

rtThreadId rtThreadCreate(
  const char *name, unsigned int priority, unsigned int stackSize,
  rtTHREADFUNC funptr, void *parm) {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
  struct sched_param sched = {0};
  rtThreadId thread        = (rtThreadId)calloc(1, sizeof(struct rtThreadOSD));
  assert(thread != NULL);
  sched.sched_priority = priority;
  assert(pthread_attr_init(&thread->attr) == 0);

  if (priority) {
    assert(pthread_attr_setinheritsched(&thread->attr,
                                        PTHREAD_EXPLICIT_SCHED) == 0);
    assert(pthread_attr_setschedpolicy(&thread->attr,
                                       SCHED_FIFO) == 0);
    assert(pthread_attr_setschedparam(&thread->attr,
                                      &sched) == 0);   
  }

  assert(pthread_attr_setstacksize(&thread->attr, PTHREAD_STACK_MIN + stackSize)==0);

  thread->start = funptr;
  thread->usr   = parm;
  int result = pthread_create(&thread->thread,
                              &thread->attr,
                              start_routine,
                              thread);

  if (result == 0) {
    pthread_setname_np(thread->thread,name);
    return thread;
  } else {
    switch (result) {
    case EAGAIN: LOGERR("rtThreadCreate: EAGAIN (%d)\n", result);
      break;

    case EINVAL: LOGERR("rtThreadCreate: EINVAL (%d)\n", result);
      break;

    case EPERM: LOGERR("rtThreadCreate: EPERM (%d)\n", result);
      break;

    default:
      LOGERR("rtThreadCreate: other error %d\n", result);
      break;
    }
    return NULL;
  }
}

struct timespec timespec_add(struct timespec time1, struct timespec time2) {
  struct timespec result;

  if ((time1.tv_nsec + time2.tv_nsec) >= MCU_NSEC_PER_SEC) {
    result.tv_sec  = time1.tv_sec + time2.tv_sec + 1;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec - MCU_NSEC_PER_SEC;
  } else {
    result.tv_sec  = time1.tv_sec + time2.tv_sec;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
  }
  return result;
}

struct timespec timespec_sub(struct timespec time1, struct timespec time2) {
  struct timespec result;

  result.tv_sec  = time1.tv_sec - time2.tv_sec;
  result.tv_nsec = time1.tv_nsec - time2.tv_nsec;

  if ((time1.tv_nsec - time2.tv_nsec) < 0) {
    result.tv_sec  = result.tv_sec - 1;
    result.tv_nsec = result.tv_nsec + MCU_NSEC_PER_SEC;
  }
  return result;
}

void cyclic_task(void *usr) {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
  int i = 0;
  int ecStat = 0;
  struct timespec wakeupTime, sendTime, lastSendTime = {};
  struct timespec startTime, endTime, lastStartTime = {};
  struct timespec offsetStartTime = {};
  const struct timespec  cycletime = {0, (long int)mcuPeriod};

  offsetStartTime.tv_nsec = MCU_NSEC_PER_SEC / 10;
  offsetStartTime.tv_sec  = 0;

  // start 100ms + 1 period after  master activate (in setAppMode())
  wakeupTime = timespec_add(masterActivationTimeMonotonic, offsetStartTime);

  while (appModeCmd == ECMC_MODE_RUNTIME) {
    wakeupTime = timespec_add(wakeupTime, cycletime);

    /* Only lock asyn port when ec is started
     * otherwise deadlock in stratup phase
     * (sleep in waitforstartup() this is called
     * in asyn thread) .
     * */
    if (appModeStat == ECMC_MODE_RUNTIME) {
      if(asynPort) asynPort->unlock();      
    }
    // Mutex for motor record access
    if(ecmcRTMutex) epicsMutexUnlock(ecmcRTMutex);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);

    if (appModeStat == ECMC_MODE_RUNTIME) {      
      if (asynPort) {
        asynPort->lock();
        asynPort->updateTimeStamp();
      }
    }
    // Mutex for motor record access
    if(ecmcRTMutex) epicsMutexLock(ecmcRTMutex);

    clock_gettime(CLOCK_MONOTONIC, &startTime);
    
    threadDiag.latency_ns    = DIFF_NS(wakeupTime, startTime);
    threadDiag.period_ns     = DIFF_NS(lastStartTime, startTime);
    threadDiag.exec_ns       = DIFF_NS(lastStartTime, endTime);
    threadDiag.sendperiod_ns = DIFF_NS(lastSendTime, sendTime);
    lastStartTime = startTime;
    lastSendTime  = sendTime;

    if (threadDiag.latency_ns > threadDiag.latency_max_ns) {
      threadDiag.latency_max_ns = threadDiag.latency_ns;
    }

    if (threadDiag.latency_ns < threadDiag.latency_min_ns) {
      threadDiag.latency_min_ns = threadDiag.latency_ns;
    }

    if (threadDiag.period_ns > threadDiag.period_max_ns) {
      threadDiag.period_max_ns = threadDiag.period_ns;
    }

    if (threadDiag.period_ns < threadDiag.period_min_ns) {
      threadDiag.period_min_ns = threadDiag.period_ns;
    }

    if (threadDiag.exec_ns > threadDiag.exec_max_ns) {
      threadDiag.exec_max_ns = threadDiag.exec_ns;
    }

    if (threadDiag.exec_ns < threadDiag.exec_min_ns) {
      threadDiag.exec_min_ns = threadDiag.exec_ns;
    }

    if (threadDiag.sendperiod_ns > threadDiag.send_max_ns) {
      threadDiag.send_max_ns = threadDiag.sendperiod_ns;
    }

    if (threadDiag.sendperiod_ns < threadDiag.send_min_ns) {
      threadDiag.send_min_ns = threadDiag.sendperiod_ns;
    }
    if(ec->getInitDone()) {
      ec->receive();
      ec->checkDomainState();
    }
    ecStat = ec->statusOK() || !ec->getInitDone();
    // Motion
    for (i = 0; i < ECMC_MAX_AXES; i++) {
      if (axes[i] != NULL) {
        plcs->execute(AXIS_PLC_ID_TO_PLC_ID(i),ecStat);
        axes[i]->execute(ecStat);        
      }
    }

    // Data events
    for (i = 0; i < ECMC_MAX_EVENT_OBJECTS; i++) {
      if (events[i] != NULL) {
        events[i]->execute(ecStat);
      }
    }

    // PLCs
    if (plcs) {
      plcs->execute(ecStat);
    }

    if (counter) {
      counter--;
    } else {    // Lower freq      
      if (axisDiagFreq > 0) {
        counter = mcuFrequency / axisDiagFreq;
        if(ec->getInitDone()) {
          ec->checkState();
          ec->checkSlavesConfState();
        }
        printStatus();

        for (int i = 0; i < ECMC_MAX_AXES; i++) {
          if (axes[i] != NULL) {
            axes[i]->slowExecute();
          }
        }
        if(ec->getInitDone()) {
          ec->slowExecute();
        }
      }
    }
    if(asynPort->getEpicsState()>=14){
      updateAsynParams(0);
    }
    
    clock_gettime(CLOCK_MONOTONIC, &sendTime);
    if(ec->getInitDone()) {
      ec->send(masterActivationTimeOffset);
    }
    clock_gettime(CLOCK_MONOTONIC, &endTime);
  }
  appModeStat = ECMC_MODE_CONFIG;
}

/****************************************************************************/
// Main functions

int ecmcInitThread(void) {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
  LOGINFO("\nECMC Initializes.............\n");
  LOGINFO("ESS Open Source EtherCAT Motion Control Epics Module");
  LOGINFO("\nMode: Configuration\n");

  appModeStat   = ECMC_MODE_CONFIG;
  appModeCmd    = ECMC_MODE_CONFIG;
  appModeCmdOld = appModeCmd;

  axisDiagIndex = 0;
  axisDiagFreq  = 10;
  setDiagAxisEnable(1);

  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    axes[i] = NULL;
  }

  for (int i = 0; i < ECMC_MAX_EVENT_OBJECTS; i++) {
    events[i] = NULL;
  }

  for (int i = 0; i < ECMC_MAX_DATA_RECORDERS_OBJECTS; i++) {
    dataRecorders[i] = NULL;
  }

  for (int i = 0; i < ECMC_MAX_DATA_STORAGE_OBJECTS; i++) {
    dataStorages[i] = NULL;
  }

  for (int i = 0; i < ECMC_MAX_COMMANDS_LISTS; i++) {
    commandLists[i] = NULL;
  }

  plcs = NULL;

  return 0;
}

int waitForThreadToStart(int timeoutSeconds) {
  struct timespec timeToPause;

  timeToPause.tv_sec  = 1;
  timeToPause.tv_nsec = 0;

  for (int i = 0; i < timeoutSeconds; i++) {
    if(ec->getInitDone()) {
      LOGINFO("Starting up EtherCAT bus: %d second(s). Max wait time %d second(s).\n", i,timeoutSeconds);
    } else {
      LOGINFO("Starting up Realtime thread without EtherCAT support.\n");
    }
    clock_nanosleep(CLOCK_MONOTONIC, 0, &timeToPause, NULL);
    if(!ec->getInitDone()){
        return 0;
    }
    if (ec->statusOK()) {
      clock_nanosleep(CLOCK_MONOTONIC, 0, &timeToPause, NULL);
      LOGINFO("EtherCAT bus started!\n");
      return 0;
    }
  }
  LOGERR("Timeout error: EtherCAT bus did not start correctly in %ds.\n",
         timeoutSeconds);
  setAppMode(0);
  return ec->getErrorID();
}

int lockMem(int size) {
    LOGINFO("INFO: Locking memory\n");
    // lock memory
    if(mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    {
      LOGERR("WARNING: mlockall() failed (0x%x).\n",ERROR_MAIN_MLOCKALL_FAIL);
      //return ERROR_MAIN_MLOCKALL_FAIL;
    }
   	
   	/* Touch each page in this piece of memory to get it mapped into RAM 
       to avoid swapping*/
    int i;
   	char *buffer;
   	buffer = (char*)malloc(size);

   	for (i = 0; i < size; i += sysconf(_SC_PAGESIZE)) {
   		/* Each write to this buffer will generate a pagefault.
   		   Once the pagefault is handled a page will be locked in
   		   memory and never given back to the system. */
   		buffer[i] = 0;
   	}
   	free(buffer);

    return 0;
}

int startRTthread() {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
  int prio = ECMC_PRIO_HIGH;

  //if (rtThreadCreate(ECMC_RT_THREAD_NAME, prio, ECMC_STACK_SIZE, cyclic_task, NULL) == NULL) {
  if(epicsThreadCreate(ECMC_RT_THREAD_NAME, prio, ECMC_STACK_SIZE, cyclic_task, NULL) == NULL) {
  
    LOGERR(
      "ERROR: Can't create high priority thread, fallback to low priority\n");
    prio = ECMC_PRIO_LOW;
    //assert(rtThreadCreate(ECMC_RT_THREAD_NAME, prio, ECMC_STACK_SIZE, cyclic_task, NULL) != NULL);
    assert(epicsThreadCreate(ECMC_RT_THREAD_NAME, prio, ECMC_STACK_SIZE, cyclic_task, NULL) != NULL);    
  } else {
    LOGINFO4("INFO:\t\tCreated high priority thread for cyclic task\n");
  }

  return lockMem(ECMC_PRE_ALLOCATION_SIZE);
}

int setAppModeCfg(int mode) {
  LOGINFO4("INFO:\t\tApplication in configuration mode.\n");
  appModeCmdOld = appModeCmd;
  appModeCmd    = (app_mode_type)mode;

  if (asynPort) {
    asynPort->setAllowRtThreadCom(false);
  }

  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    if (axes[i] != NULL) {
      axes[i]->setRealTimeStarted(false);
    }
  }

  munlockall();
  return 0;
}

int setAppModeRun(int mode) {
  if (appModeStat == ECMC_MODE_RUNTIME) {
    return ERROR_MAIN_APP_MODE_ALREADY_RUNTIME;
  }

  // Block rt communication during startup 
  // (since sleep in waitForThreadToStart())
  asynPort->setAllowRtThreadCom(false);

  appModeCmdOld = appModeCmd;
  appModeCmd    = (app_mode_type)mode;

  appModeStat = ECMC_MODE_STARTUP;

  if (mainAsynParams[ECMC_ASYN_MAIN_PAR_APP_MODE_ID]) {
    mainAsynParams[ECMC_ASYN_MAIN_PAR_APP_MODE_ID]->refreshParam(1);    
    asynPort->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);
  }

  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    if (axes[i] != NULL) {
      axes[i]->setInStartupPhase(true);
    }
  }

  int errorCode = validateConfig();

  if (errorCode) {
    return errorCode;
  }

  clock_gettime(CLOCK_MONOTONIC, &masterActivationTimeMonotonic);
  // absolute clock (epoch)
  clock_gettime(CLOCK_REALTIME,  &masterActivationTimeRealtime);

  masterActivationTimeOffset = timespec_sub(masterActivationTimeRealtime,
                                            masterActivationTimeMonotonic);
  if(ec->getInitDone()) {
    ecrt_master_application_time(ec->getMaster(),
                               TIMESPEC2NS(masterActivationTimeRealtime));

    if (ec->activate()) {
      LOGERR("INFO:\t\tActivation of master failed.\n");
      return ERROR_MAIN_EC_ACTIVATE_FAILED;
    }
  } else {
      LOGERR("WARNING: EtherCAT master not initialized. Starting ECMC without EtherCAT support.\n");
  }
  errorCode = startRTthread();
  if(errorCode) {
    return errorCode;
  }
  
  LOGINFO4("INFO:\t\tApplication in runtime mode.\n");

  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    if (axes[i] != NULL) {
      axes[i]->setRealTimeStarted(true);
    }
  }

  errorCode = waitForThreadToStart(ecTimeoutSeconds > 0 ? ecTimeoutSeconds : EC_START_TIMEOUT_S);

  if (errorCode) {
    return errorCode;
  }
  appModeStat = ECMC_MODE_RUNTIME;

  if (asynPort) {
    asynPort->setAllowRtThreadCom(true);  // Set by epics state hooks
  }

  return 0;
}

int setAppMode(int mode) {
  LOGINFO4("%s/%s:%d mode=%d\n", __FILE__, __FUNCTION__, __LINE__, mode);

  switch ((app_mode_type)mode) {
  case ECMC_MODE_CONFIG:
    return setAppModeCfg(mode);

    break;

  case ECMC_MODE_RUNTIME:
    return setAppModeRun(mode);

    break;

  default:
    LOGERR("WARNING: Mode %d not implemented. (Config=%d, Runtime=%d).\n",
           mode,
           ECMC_MODE_CONFIG,
           ECMC_MODE_RUNTIME);
    return ERROR_MAIN_APP_MODE_NOT_SUPPORTED;

    break;
  }
  return 0;
}

int setEcStartupTimeout(int timeSeconds) {
  LOGINFO4("%s/%s:%d timeSeconds=%d\n", __FILE__, __FUNCTION__, __LINE__, timeSeconds);
  if(timeSeconds<=0) {
    LOGERR("ERROR: Invalid EtherCAT timeout value %d. Must be > 0 (0x%x).",
               timeSeconds,
               ERROR_MAIN_EC_TIMEOUT_OUT_OF_RANGE);
    return ERROR_MAIN_EC_TIMEOUT_OUT_OF_RANGE;
  }

  ecTimeoutSeconds = timeSeconds;

  return 0;
}

int setSampleRate(double sampleRate) {
  LOGINFO4("%s/%s:%d sampleRate=%lf\n", __FILE__, __FUNCTION__, __LINE__, sampleRate);

  if (!sampleRateChangeAllowed) {
    LOGERR(
      "%s/%s:%d: Error: Sample rate change not allowed. Change of sample rate is only allowed prior any object creation (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_MAIN_SAMPLE_RATE_CHANGE_NOT_ALLOWED);
    return ERROR_MAIN_SAMPLE_RATE_CHANGE_NOT_ALLOWED;
  }

  if(sampleRate < MCU_MIN_FREQUENCY || sampleRate > MCU_MAX_FREQUENCY) {
    LOGERR(
      "%s/%s:%d: Sample rate out of range. Allowed range  %lf.. %lfhz. Sample rate = %lfhz (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      MCU_MIN_FREQUENCY,
      MCU_MAX_FREQUENCY,
      mcuFrequency,
      ERROR_MAIN_SAMPLE_RATE_OUT_OF_RANGE);
    return ERROR_MAIN_SAMPLE_RATE_OUT_OF_RANGE;
  }

  mcuFrequency = sampleRate;
  mcuPeriod = (MCU_NSEC_PER_SEC / mcuFrequency);

  return 0;
}

int setSamplePeriodMs(double samplePeriodMs) {
  LOGINFO4("%s/%s:%d samplePeriod=%lf\n", __FILE__, __FUNCTION__, __LINE__, samplePeriodMs);

  if (!sampleRateChangeAllowed) {
    LOGERR(
      "%s/%s:%d: Error: Sample rate change not allowed. Change of sample rate is only allowed prior any object creation (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_MAIN_SAMPLE_RATE_CHANGE_NOT_ALLOWED);
    return ERROR_MAIN_SAMPLE_RATE_CHANGE_NOT_ALLOWED;
  }

  if(samplePeriodMs < MCU_MIN_PERIOD_NS || samplePeriodMs > MCU_MAX_PERIOD_NS) {
    LOGERR(
      "%s/%s:%d: Sample period out of range. Allowed range  %lf.. %lfms. Sample period = %lfms (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      MCU_MIN_PERIOD_NS/1e6,
      MCU_MAX_PERIOD_NS/1e6,
      mcuPeriod,
      ERROR_MAIN_SAMPLE_RATE_OUT_OF_RANGE);
    return ERROR_MAIN_SAMPLE_RATE_OUT_OF_RANGE;
  }

  mcuPeriod = samplePeriodMs*1e6;  // ms to ns
  mcuFrequency = MCU_NSEC_PER_SEC / mcuPeriod;

  return 0;
}

int validateConfig() {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);

  int errorCode = 0;
  int axisCount = 0;
  
  if(ec->getInitDone()){
    errorCode = ec->checkReadyForRuntime();
    if(errorCode) {
      return errorCode;
    }
  }
  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    if (axes[i] != NULL) {
      axisCount++;
      errorCode = axes[i]->validate();

      if (errorCode) {
        LOGERR("ERROR: Validation failed on axis %d with error code %x.",
               i,
               errorCode);
        return errorCode;
      }
    }
  }

  for (int i = 0; i < ECMC_MAX_EVENT_OBJECTS; i++) {
    if (events[i] != NULL) {
      errorCode = events[i]->validate();

      if (errorCode) {
        LOGERR("ERROR: Validation failed on event %d with error code %x.",
               i,
               errorCode);
        return errorCode;
      }
    }
  }

  for (int i = 0; i < ECMC_MAX_DATA_RECORDERS_OBJECTS; i++) {
    if (dataRecorders[i] != NULL) {
      errorCode = dataRecorders[i]->validate();

      if (errorCode) {
        LOGERR(
          "ERROR: Validation failed on data recorder %d with error code %x.",
          i,
          errorCode);
        return errorCode;
      }
    }
  }

  if (plcs) {
    errorCode = plcs->validate();

    if (errorCode) {
      LOGERR("ERROR: Validation failed on plc object with error code %x.",
             errorCode);
      return errorCode;
    }
  }
  
  return 0;
}