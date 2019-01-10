
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

#include "com/cmd.h"
#include "hw_motor.h"
#include "ecrt.h"

// General
#include "general/ecmcDefinitions.h"
#include "general/ecmcErrorsList.h"
#include "general/ecmcGlobals.h" 

// Hardware
#include "ethercat/ecmcEthercat.h"

// Motion
#include "motion/ecmcMotion.h"

// PLC
#include "plc/ecmcPLC.h"

// Other
#include "general/ecmcGeneral.h"
#include "misc/ecmcMisc.h"
#include "com/ecmcAsynPortDriver.h"



/****************************************************************************/
static int controllerError = -1;
static const char   *controllerErrorMsg = "NO_ERROR";
static app_mode_type appModeCmd, appModeCmdOld, appModeStat;
static unsigned int  counter = 0;

static struct timespec     masterActivationTimeMonotonic = {};
static struct timespec     masterActivationTimeOffset    = {};
static struct timespec     masterActivationTimeRealtime  = {};




/****************************************************************************/

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

// ******

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

/*Available strings:
 *  ec<masterId>.s<slaveId>.<alias>  (defaults complete ecentry)
 *  ec<masterId>.s<slaveId>.<alias>.<bit> (only one bit)
*/
static int parseEcPath(char *ecPath,
                       int  *master,
                       int  *slave,
                       char *alias,
                       int  *bit) {
  int masterId = 0;
  int slaveId  = 0;
  int bitId    = 0;
  int nvals    = 0;

  nvals = sscanf(ecPath,
                 "ec%d.s%d.%[^.].%d",
                 &masterId,
                 &slaveId,
                 alias,
                 &bitId);

  if (nvals == 4) {
    *master = masterId;
    *slave  = slaveId;
    *bit    = bitId;
    return 0;
  }
  nvals = sscanf(ecPath, "ec%d.s%d.%s", &masterId, &slaveId, alias);

  if (nvals == 3) {
    *master = masterId;
    *slave  = slaveId;
    *bit    = -1;
    return 0;
  }
  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
}

static int parseObjectPath(char             *objPath,
                           int              *axis,
                           motionObjectType *objectType,
                           int              *objectFunction) {
  int  axisId = 0;
  char objectTypeStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char objectFunctionStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  nvals = 0;

  *objectType = ECMC_OBJ_INVALID;

  // Axis sub objects
  nvals = sscanf(objPath,
                 ECMC_AX_STR "%d.%[^.].%s",
                 &axisId,
                 objectTypeStr,
                 objectFunctionStr);

  if (nvals == 3) {
    *axis = axisId;

    // Drive
    nvals = strcmp(objectTypeStr, ECMC_DRV_STR);

    if (nvals == 0) {
      *objectType = ECMC_OBJ_DRIVE;

      // Enable
      nvals = strcmp(objectFunctionStr, ECMC_DRV_ENABLE_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD;
        return 0;
      }

      // Velocity
      nvals = strcmp(objectFunctionStr, ECMC_DRV_VELOCITY_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT;
        return 0;
      }

      // Enabled
      nvals = strcmp(objectFunctionStr, ECMC_DRV_ENABLED_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD;
        return 0;
      }

      // Break
      nvals = strcmp(objectFunctionStr, ECMC_DRV_BREAK_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT;
        return 0;
      }

      // Reduce Torque
      nvals = strcmp(objectFunctionStr, ECMC_DRV_REDUCETORQUE_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT;
        return 0;
      }
      return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
    }

    // Encoder
    nvals = strcmp(objectTypeStr, ECMC_ENC_STR);

    if (nvals == 0) {
      *objectType = ECMC_OBJ_ENCODER;

      // Actpos
      nvals = strcmp(objectFunctionStr, ECMC_ENC_ACTPOS_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION;
        return 0;
      }

      // Latch status
      nvals = strcmp(objectFunctionStr, ECMC_ENC_LATCH_STATUS_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_ENCODER_ENTRY_INDEX_LATCH_STATUS;
        return 0;
      }

      // Latch pos
      nvals = strcmp(objectFunctionStr, ECMC_ENC_LATCHPOS_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_ENCODER_ENTRY_INDEX_LATCH_VALUE;
        return 0;
      }

      // Latch control
      nvals = strcmp(objectFunctionStr, ECMC_ENC_LATCH_CONTROL_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_ENCODER_ENTRY_INDEX_LATCH_CONTROL;
        return 0;
      }
      return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
    }

    // Monitor
    nvals = strcmp(objectTypeStr, ECMC_MON_STR);

    if (nvals == 0) {
      *objectType = ECMC_OBJ_MONITOR;

      // Lowlim
      nvals = strcmp(objectFunctionStr, ECMC_MON_LOWLIM_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_MON_ENTRY_INDEX_LOWLIM;
        return 0;
      }

      // Highlim
      nvals = strcmp(objectFunctionStr, ECMC_MON_HIGHLIM_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_MON_ENTRY_INDEX_HIGHLIM;
        return 0;
      }

      // Home sensor
      nvals = strcmp(objectFunctionStr, ECMC_MON_HOMESENSOR_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_MON_ENTRY_INDEX_HOMESENSOR;
        return 0;
      }

      // ExternalInterupt
      nvals = strcmp(objectFunctionStr, ECMC_MON_EXTINTERLOCK_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_MON_ENTRY_INDEX_EXTINTERLOCK;
        return 0;
      }
      return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
    }
  }

  // Axis object only
  nvals = sscanf(objPath, ECMC_AX_STR "%d.%s", &axisId, objectFunctionStr);

  if (nvals == 2) {
    *objectType = ECMC_OBJ_AXIS;
    *axis       = axisId;

    // Health
    nvals = strcmp(objectFunctionStr, ECMC_AX_HEALTH_STR);

    if (nvals == 0) {
      *objectFunction = ECMC_AXIS_ENTRY_INDEX_HEALTH;
      return 0;
    }
    return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
  }

  // Ec object
  int masterId = 0;
  nvals = sscanf(objPath, ECMC_EC_STR "%d.%s", &masterId, objectFunctionStr);

  if (nvals == 2) {
    *objectType = ECMC_OBJ_EC;

    // Health
    nvals = strcmp(objectFunctionStr, ECMC_EC_HEALTH_STR);

    if (nvals == 0) {
      *objectFunction = ECMC_EC_ENTRY_INDEX_HEALTH;
      return 0;
    }
    return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
  }

  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
}

void cyclic_task(void *usr) {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
  int i = 0;
  struct timespec wakeupTime, sendTime, lastSendTime = {};
  struct timespec startTime, endTime, lastStartTime = {};
  struct timespec offsetStartTime = {};
  uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0, sendperiod_ns = 0,
           latency_min_ns = 0, latency_max_ns = 0,
           period_min_ns = 0, period_max_ns = 0,
           exec_min_ns = 0, exec_max_ns = 0,
           send_min_ns = 0, send_max_ns = 0;

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
    latency_ns    = DIFF_NS(wakeupTime, startTime);
    period_ns     = DIFF_NS(lastStartTime, startTime);
    exec_ns       = DIFF_NS(lastStartTime, endTime);
    sendperiod_ns = DIFF_NS(lastSendTime, sendTime);
    lastStartTime = startTime;
    lastSendTime  = sendTime;

    if (latency_ns > latency_max_ns) {
      latency_max_ns = latency_ns;
    }

    if (latency_ns < latency_min_ns) {
      latency_min_ns = latency_ns;
    }

    if (period_ns > period_max_ns) {
      period_max_ns = period_ns;
    }

    if (period_ns < period_min_ns) {
      period_min_ns = period_ns;
    }

    if (exec_ns > exec_max_ns) {
      exec_max_ns = exec_ns;
    }

    if (exec_ns < exec_min_ns) {
      exec_min_ns = exec_ns;
    }

    if (sendperiod_ns > send_max_ns) {
      send_max_ns = sendperiod_ns;
    }

    if (sendperiod_ns < send_min_ns) {
      send_min_ns = sendperiod_ns;
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
          asynPort->setIntegerParam(asynParIdLatencyMin,  latency_min_ns);
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
                        0);
        }
        period_max_ns  = 0;
        period_min_ns  = 0xffffffff;
        exec_max_ns    = 0;
        exec_min_ns    = 0xffffffff;
        latency_max_ns = 0;
        latency_min_ns = 0xffffffff;
        send_max_ns    = 0;
        send_min_ns    = 0xffffffff;
        sendperiod_ns  = 0;
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


/****************************************************************************/

int hw_motor_global_init(void) {
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


// Main functions
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

  // Block rt communication during startup (since sleep in waitForEtherCATtoStart())
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

/****************************************************************************/



int linkEcEntryToObject(char *ecPath, char *axPath) {
  LOGINFO4("%s/%s:%d ecPath=%s axPath=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ecPath,
           axPath);

  int  masterId   = -1;
  int  slaveIndex = -1;
  char alias[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  bitIndex = -1;

  int errorCode =
    parseEcPath(ecPath, &masterId, &slaveIndex, alias, &bitIndex);

  if (errorCode) {
    return errorCode;
  }

  int axisIndex               = 0;
  motionObjectType objectType = ECMC_OBJ_INVALID;
  int entryIndex              = 0;
  errorCode = parseObjectPath(axPath, &axisIndex, &objectType, &entryIndex);

  if (errorCode) {
    return errorCode;
  }

  switch (objectType) {
  case ECMC_OBJ_INVALID:
    return ERROR_MAIN_ECMC_LINK_INVALID;

    break;

  case ECMC_OBJ_DRIVE:
    return linkEcEntryToAxisDrv(slaveIndex,
                                alias,
                                axisIndex,
                                entryIndex,
                                bitIndex);

    break;

  case ECMC_OBJ_ENCODER:
    return linkEcEntryToAxisEnc(slaveIndex,
                                alias,
                                axisIndex,
                                entryIndex,
                                bitIndex);

    break;

  case ECMC_OBJ_MONITOR:
    return linkEcEntryToAxisMon(slaveIndex,
                                alias,
                                axisIndex,
                                entryIndex,
                                bitIndex);

    break;

  case ECMC_OBJ_CONTROLLER:
    return ERROR_MAIN_ECMC_LINK_INVALID;

    break;

  case ECMC_OBJ_TRAJECTORY:
    return ERROR_MAIN_ECMC_LINK_INVALID;

    break;

  case ECMC_OBJ_AXIS:

    if (entryIndex == ECMC_AXIS_ENTRY_INDEX_HEALTH) {
      return linkEcEntryToAxisStatusOutput(slaveIndex, alias, axisIndex);
    }
    return ERROR_MAIN_ECMC_LINK_INVALID;

    break;

  case ECMC_OBJ_EC:

    if (entryIndex == ECMC_EC_ENTRY_INDEX_HEALTH) {
      return linkEcEntryToEcStatusOutput(slaveIndex, alias);
    }
    return ERROR_MAIN_ECMC_LINK_INVALID;

    break;
  }
  return ERROR_MAIN_ECMC_LINK_INVALID;
}

