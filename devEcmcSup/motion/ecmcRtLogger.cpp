/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcRtLogger.cpp
*
*  Created on: Apr 10, 2026
*
\*************************************************************************/

#include "ecmcRtLogger.h"

#include <atomic>
#include <inttypes.h>

#include "epicsThread.h"
#include "ecmcAxisBase.h"
#include "ecmcError.h"
#include "ecmcOctetIF.h"

namespace {

constexpr size_t ECMC_RT_LOGGER_QUEUE_SIZE = 256;
constexpr double ECMC_RT_LOGGER_SLEEP_S    = 0.05;
const char      *ECMC_RT_LOGGER_THREAD     = "ecmc_rt_log";

struct ecmcRtLogEvent {
  uint32_t type;
  int32_t  axisId;
  int32_t  code;
  int32_t  arg0;
};

std::atomic<size_t> writeIndex_(0);
std::atomic<size_t> readIndex_(0);
std::atomic<uint32_t> droppedCount_(0);
std::atomic<int> enabled_(0);
std::atomic<int> started_(0);
ecmcRtLogEvent queue_[ECMC_RT_LOGGER_QUEUE_SIZE] = {};

const char *axisStateToString(int axisState) {
  switch (axisState) {
  case ECMC_AXIS_STATE_STARTUP:
    return "STARTUP";

  case ECMC_AXIS_STATE_DISABLED:
    return "DISABLED";

  case ECMC_AXIS_STATE_ENABLED:
    return "ENABLED";

  default:
    return "UNKNOWN";
  }
}

void reportDroppedEvents() {
  const uint32_t dropped = droppedCount_.exchange(0, std::memory_order_acq_rel);
  if (dropped == 0) {
    return;
  }

  LOGERR("%s/%s:%d: ERROR: RT log queue dropped %u events.\n",
         __FILE__,
         __FUNCTION__,
         __LINE__,
         (unsigned int)dropped);
}

void logEvent(const ecmcRtLogEvent &event) {
  switch (event.type) {
  case ECMC_RT_LOG_EVENT_AXIS_ERROR:
    LOGERR("%s/%s:%d: ERROR: Axis[%d]: RT error: %s (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           event.axisId,
           ecmcError::convertErrorIdToString(event.code),
           event.code);
    break;

  case ECMC_RT_LOG_EVENT_SEQ_ERROR:
    LOGERR("%s/%s:%d: ERROR: Axis[%d]: RT sequencer error: %s (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           event.axisId,
           ecmcError::convertErrorIdToString(event.code),
           event.code);
    break;

  case ECMC_RT_LOG_EVENT_AXIS_STATE:
    if (event.code == ECMC_AXIS_STATE_STARTUP) {
      LOGERR("%s/%s:%d: ERROR: Axis[%d]: RT state changed to %s.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             event.axisId,
             axisStateToString(event.code));
    } else {
      LOGINFO("%s/%s:%d: INFO: Axis[%d]: RT state changed to %s.\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              event.axisId,
              axisStateToString(event.code));
    }
    break;

  case ECMC_RT_LOG_EVENT_HOME_STATE:
    LOGINFO("%s/%s:%d: INFO: Axis[%d]: Homing sequence state=%d, cmdData=%d.\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            event.axisId,
            event.code,
            event.arg0);
    break;

  default:
    LOGERR("%s/%s:%d: ERROR: Unknown RT log event type %u.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           event.type);
    break;
  }
}

void drainQueue() {
  while (true) {
    const size_t readIndex = readIndex_.load(std::memory_order_relaxed);
    const size_t writeIndex = writeIndex_.load(std::memory_order_acquire);

    if (readIndex == writeIndex) {
      break;
    }

    const ecmcRtLogEvent event = queue_[readIndex];
    readIndex_.store((readIndex + 1) % ECMC_RT_LOGGER_QUEUE_SIZE,
                     std::memory_order_release);
    logEvent(event);
  }

  reportDroppedEvents();
}

void loggerTask(void *arg) {
  (void)arg;

  while (true) {
    drainQueue();
    epicsThreadSleep(ECMC_RT_LOGGER_SLEEP_S);
  }
}

inline void queueEvent(uint32_t type, int axisId, int code, int arg0) {
  if (!started_.load(std::memory_order_acquire) ||
      !enabled_.load(std::memory_order_acquire)) {
    return;
  }

  const size_t writeIndex = writeIndex_.load(std::memory_order_relaxed);
  const size_t nextWriteIndex = (writeIndex + 1) % ECMC_RT_LOGGER_QUEUE_SIZE;
  const size_t readIndex = readIndex_.load(std::memory_order_acquire);

  if (nextWriteIndex == readIndex) {
    droppedCount_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  queue_[writeIndex].type = type;
  queue_[writeIndex].axisId = axisId;
  queue_[writeIndex].code = code;
  queue_[writeIndex].arg0 = arg0;
  writeIndex_.store(nextWriteIndex, std::memory_order_release);
}

}  // namespace

int ecmcRtLoggerStart() {
  if (started_.load(std::memory_order_acquire)) {
    return 0;
  }

  epicsThreadId threadId = epicsThreadCreate(
    ECMC_RT_LOGGER_THREAD,
    epicsThreadPriorityLow,
    epicsThreadGetStackSize(epicsThreadStackSmall),
    loggerTask,
    NULL);

  if (!threadId) {
    LOGERR("%s/%s:%d: WARNING: Failed to create low priority RT log thread. Using direct RT logging.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    enabled_.store(0, std::memory_order_release);
    return 0;
  }

  started_.store(1, std::memory_order_release);
  return 0;
}

void ecmcRtLoggerSetEnabled(int enabled) {
  enabled_.store(enabled ? 1 : 0, std::memory_order_release);
}

int ecmcRtLoggerIsEnabled() {
  return started_.load(std::memory_order_acquire) &&
         enabled_.load(std::memory_order_acquire);
}

void ecmcRtLoggerQueueAxisError(int axisId, int errorCode) {
  queueEvent(ECMC_RT_LOG_EVENT_AXIS_ERROR, axisId, errorCode, 0);
}

void ecmcRtLoggerQueueSeqError(int axisId, int errorCode) {
  queueEvent(ECMC_RT_LOG_EVENT_SEQ_ERROR, axisId, errorCode, 0);
}

void ecmcRtLoggerQueueAxisState(int axisId, int axisState) {
  queueEvent(ECMC_RT_LOG_EVENT_AXIS_STATE, axisId, axisState, 0);
}

void ecmcRtLoggerQueueHomeState(int axisId, int cmdData, int seqState) {
  queueEvent(ECMC_RT_LOG_EVENT_HOME_STATE, axisId, seqState, cmdData);
}
