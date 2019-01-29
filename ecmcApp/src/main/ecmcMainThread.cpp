
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
#include <unistd.h>
#include <assert.h>
#include <sched.h>
#include <pthread.h>
#include <exception>
#include <algorithm>
#include <string>

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

/****************************************************************************/

static int controllerError = -1;
static const char   *controllerErrorMsg = "NO_ERROR";
static app_mode_type appModeCmd, appModeCmdOld, appModeStat;
static unsigned int  counter = 0;
static struct timespec     masterActivationTimeMonotonic = {};
static struct timespec     masterActivationTimeOffset    = {};
static struct timespec     masterActivationTimeRealtime  = {};
const struct timespec cycletime = {0, MCU_PERIOD_NS};

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
  thread->start = funptr;
  thread->usr   = parm;
  int result = pthread_create(&thread->thread,
                              &thread->attr,
                              start_routine,
                              thread);

  if (result == 0) {
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
  struct timespec wakeupTime, sendTime, lastSendTime = {};
  struct timespec startTime, endTime, lastStartTime = {};
  struct timespec offsetStartTime = {};
  /*uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0, sendperiod_ns = 0,
           latency_min_ns = 0, latency_max_ns = 0,
           period_min_ns = 0, period_max_ns = 0,
           exec_min_ns = 0, exec_max_ns = 0,
           send_min_ns = 0, send_max_ns = 0;*/

  offsetStartTime.tv_nsec = 49 * MCU_PERIOD_NS;
  offsetStartTime.tv_sec  = 0;

  // start 50 (49+1) cycle times after master activate
  wakeupTime = timespec_add(masterActivationTimeMonotonic, offsetStartTime);

  while (appModeCmd == ECMC_MODE_RUNTIME) {
    wakeupTime = timespec_add(wakeupTime, cycletime);

    /* Only lock asyn port when ec is started
     * otherwise deadlock in stratup phase
     * (sleep in waitforstartup() this is called
     * in asyn thread) .
     * */
    if (asynPort && (appModeStat == ECMC_MODE_RUNTIME)) {
      asynPort->unlock();
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);

    if (asynPort && (appModeStat == ECMC_MODE_RUNTIME)) {
      asynPort->lock();
    }
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

    ec.receive();
    ec.checkDomainState();

    // Motion
    for (i = 0; i < ECMC_MAX_AXES; i++) {
      if (axes[i] != NULL) {
        axes[i]->execute(ec.statusOK());
      }
    }

    // Data events
    for (i = 0; i < ECMC_MAX_EVENT_OBJECTS; i++) {
      if (events[i] != NULL) {
        events[i]->execute(ec.statusOK());
      }
    }

    // PLCs
    if (plcs) {
      plcs->execute(ec.statusOK());
    }

    // Update Asyn thread diagnostics parameters
    if (asynUpdateCounterThread) {
      asynUpdateCounterThread--;
    } else {
      asynUpdateCounterThread = asynSkipCyclesThread;

      if (asynPort && (asynSkipCyclesThread >= 0) && asynThreadParamsEnable) {
        if (asynPort->getAllowRtThreadCom()) {
          mainAsynParams[0]->refreshParamRT(0);  //LatencyMin
          /*asynPort->setIntegerParam(asynParIdLatencyMin,  latency_min_ns);
          asynPort->setIntegerParam(asynParIdLatencyMax,  latency_max_ns);
          asynPort->setIntegerParam(asynParIdExecuteMin,  exec_min_ns);
          asynPort->setIntegerParam(asynParIdExecuteMax,  exec_max_ns);
          asynPort->setIntegerParam(asynParIdPeriodMin,   period_min_ns);
          asynPort->setIntegerParam(asynParIdPeriodMax,   period_max_ns);
          asynPort->setIntegerParam(asynParIdSendMin,     send_min_ns);
          asynPort->setIntegerParam(asynParIdSendMax,     send_max_ns);
          controllerError = getControllerError();
          asynPort->setIntegerParam(asynParIdEcmcErrorId, controllerError);
          controllerErrorMsg = getErrorString(controllerError);
          asynPort->doCallbacksInt8Array(
                        (epicsInt8 *)controllerErrorMsg,
                        static_cast<int>(strlen(controllerErrorMsg)) + 1,
                        asynParIdEcmcErrorMsg,
                        0);*/
        }
        
        threadDiag.period_max_ns  = 0;
        threadDiag.period_min_ns  = 0xffffffff;
        threadDiag.exec_max_ns    = 0;
        threadDiag.exec_min_ns    = 0xffffffff;
        threadDiag.latency_max_ns = 0;
        threadDiag.latency_min_ns = 0xffffffff;
        threadDiag.send_max_ns    = 0;
        threadDiag.send_min_ns    = 0xffffffff;
        threadDiag.sendperiod_ns  = 0;
      }
    }

    if (counter) {
      counter--;
    } else {    // Lower freq
      if (axisDiagFreq > 0) {
        counter = MCU_FREQUENCY / axisDiagFreq;
        ec.checkState();
        ec.checkSlavesConfState();
        printStatus();

        for (int i = 0; i < ECMC_MAX_AXES; i++) {
          if (axes[i] != NULL) {
            axes[i]->slowExecute();
          }
        }
        ec.slowExecute();
      }
    }

    // Asyn callbacks for all parameters (except arrays)
    if (asynSkipUpdateCounterFastest) {
      asynSkipUpdateCounterFastest--;
    } else {
      asynSkipUpdateCounterFastest = asynSkipCyclesFastest;

      if (asynPort) {
        if (asynPort->getAllowRtThreadCom()) {
          asynPort->callParamCallbacks();
        }
      }
    }

    clock_gettime(CLOCK_MONOTONIC, &sendTime);
    ec.send(masterActivationTimeOffset);
    clock_gettime(CLOCK_MONOTONIC, &endTime);
  }
  appModeStat = ECMC_MODE_CONFIG;
}

/****************************************************************************/
// Main functions

int ecmcInit(void) {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
  LOGINFO("\nESS Open Source EtherCAT Motion Control Unit Initializes......\n");
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

int waitForEtherCATtoStart(int timeoutSeconds) {
  struct timespec timeToPause;

  timeToPause.tv_sec  = 1;
  timeToPause.tv_nsec = 0;

  for (int i = 0; i < timeoutSeconds; i++) {
    LOGINFO("Starting up EtherCAT bus: %d second(s).\n", i);
    clock_nanosleep(CLOCK_MONOTONIC, 0, &timeToPause, NULL);

    if (ec.statusOK()) {
      clock_nanosleep(CLOCK_MONOTONIC, 0, &timeToPause, NULL);
      LOGINFO("EtherCAT bus started!\n");
      return 0;
    }
  }
  LOGERR("Timeout error: EtherCAT bus did not start correctly in %ds.\n",
         timeoutSeconds);
  setAppMode(0);
  return ec.getErrorID();
}

void startRTthread() {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
  int prio = ECMC_PRIO_HIGH;

  if (rtThreadCreate("cyclic", prio, 0, cyclic_task, NULL) == NULL) {
    LOGERR(
      "ERROR: Can't create high priority thread, fallback to low priority\n");
    prio = ECMC_PRIO_LOW;
    assert(rtThreadCreate("cyclic", prio, 0, cyclic_task, NULL) != NULL);
  } else {
    LOGINFO4("INFO:\t\tCreated high priority thread for cyclic task\n");
  }
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
  return 0;
}

int setAppModeRun(int mode) {
  if (appModeStat == ECMC_MODE_RUNTIME) {
    return ERROR_MAIN_APP_MODE_ALREADY_RUNTIME;
  }

  // Block rt communication during startup 
  // (since sleep in waitForEtherCATtoStart())
  asynPort->setAllowRtThreadCom(false);

  appModeCmdOld = appModeCmd;
  appModeCmd    = (app_mode_type)mode;

  appModeStat = ECMC_MODE_STARTUP;

  if (asynPort && asynThreadParamsEnable) {
    asynPort->setIntegerParam(asynParIdEcmcAppMode, mode);
    asynPort->callParamCallbacks();
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
  ecrt_master_application_time(ec.getMaster(),
                               TIMESPEC2NS(masterActivationTimeRealtime));


  if (ec.activate()) {
    LOGERR("INFO:\t\tActivation of master failed.\n");
    return ERROR_MAIN_EC_ACTIVATE_FAILED;
  }
  LOGINFO4("INFO:\t\tApplication in runtime mode.\n");
  startRTthread();

  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    if (axes[i] != NULL) {
      axes[i]->setRealTimeStarted(true);
    }
  }
  errorCode = waitForEtherCATtoStart(30);

  if (errorCode) {
    return errorCode;
  }
  appModeStat = ECMC_MODE_RUNTIME;

  if (asynPort) {
    asynPort->setAllowRtThreadCom(true);
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

int prepareForRuntime() {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);

  // Update input sources for all trajectories and encoders (transforms)
  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    for (int j = 0; j < ECMC_MAX_AXES; j++) {
      if ((axes[i] != NULL) && (axes[j] != NULL)) {
        // Trajectory
        if (axes[i]->getExternalTrajIF() != NULL) {
          if (axes[j]->getExternalTrajIF() != NULL) {
            axes[j]->getExternalTrajIF()->addInputDataInterface(
              axes[i]->getExternalTrajIF()->getOutputDataInterface(),
              i);
          }

          if (axes[j]->getExternalEncIF() != NULL) {
            axes[j]->getExternalEncIF()->addInputDataInterface(
              axes[i]->getExternalTrajIF()->getOutputDataInterface(),
              i);
          }
        }

        // Encoder
        if (axes[i]->getExternalEncIF() != NULL) {
          if (axes[j]->getExternalTrajIF() != NULL) {
            axes[j]->getExternalTrajIF()->addInputDataInterface(
              axes[i]->getExternalEncIF()->getOutputDataInterface(),
              i + ECMC_MAX_AXES);
          }

          if (axes[j]->getExternalEncIF() != NULL) {
            axes[j]->getExternalEncIF()->addInputDataInterface(
              axes[i]->getExternalEncIF()->getOutputDataInterface(),
              i + ECMC_MAX_AXES);
          }
        }
        axes[i]->setAxisArrayPointer(axes[j], j);
      }
    }
  }
  return 0;
}

int validateConfig() {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
  prepareForRuntime();
  int errorCode = 0;
  int axisCount = 0;

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