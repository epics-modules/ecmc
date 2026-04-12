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
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "epicsThread.h"
#include "ecmcOctetIF.h"
#include "ecmcRtLoggerPortDriver.h"

namespace {

constexpr size_t ECMC_RT_LOGGER_QUEUE_SIZE = 256;
constexpr size_t ECMC_RT_LOGGER_MSG_SIZE   = 192;
constexpr double ECMC_RT_LOGGER_SLEEP_S    = 0.05;
const char      *ECMC_RT_LOGGER_THREAD     = "ecmc_rt_log";

enum ecmcRtLogLevel {
  ECMC_RT_LOG_LEVEL_INFO    = 0,
  ECMC_RT_LOG_LEVEL_WARNING = 1,
  ECMC_RT_LOG_LEVEL_ERROR   = 2
};

struct ecmcRtLogEvent {
  int  level;
  int  sourceType;
  int  sourceIndex;
  char message[ECMC_RT_LOGGER_MSG_SIZE];
};

std::atomic<size_t> writeIndex_(0);
std::atomic<size_t> readIndex_(0);
std::atomic<unsigned int> droppedCount_(0);
std::atomic<int> enabled_(0);
std::atomic<int> started_(0);
std::atomic<unsigned int> controlWord_(ECMC_RT_LOGGER_CONTROL_DEFAULT);
ecmcRtLogEvent queue_[ECMC_RT_LOGGER_QUEUE_SIZE] = {};

bool levelEnabled(int level) {
  const unsigned int controlWord = controlWord_.load(std::memory_order_acquire);

  if (level == ECMC_RT_LOG_LEVEL_ERROR) {
    return (controlWord & ECMC_RT_LOGGER_CONTROL_ERROR_ENABLE) != 0;
  }
  if (level == ECMC_RT_LOG_LEVEL_WARNING) {
    return (controlWord & ECMC_RT_LOGGER_CONTROL_WARNING_ENABLE) != 0;
  }

  return (controlWord & ECMC_RT_LOGGER_CONTROL_INFO_ENABLE) != 0;
}

void printMessage(int level, const char *message) {
  if (level == ECMC_RT_LOG_LEVEL_ERROR) {
    LOGERR("%s", message);
  } else if (level == ECMC_RT_LOG_LEVEL_WARNING) {
    LOGWARNING("%s", message);
  } else {
    LOGINFO("%s", message);
  }
}

void reportDroppedEvents() {
  const unsigned int dropped =
    droppedCount_.exchange(0, std::memory_order_acq_rel);

  if (dropped == 0) {
    return;
  }

  ecmcRtLoggerPortDriverPublishDropped(dropped);

  LOGERR("%s/%s:%d: ERROR: RT log queue dropped %u events.\n",
         __FILE__,
         __FUNCTION__,
         __LINE__,
         dropped);
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
    ecmcRtLoggerPortDriverPublishMessage(event.level,
                                         event.sourceType,
                                         event.sourceIndex,
                                         event.message);
    printMessage(event.level, event.message);
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

void logMessageV(int level,
                 int sourceType,
                 int sourceIndex,
                 const char *fmt,
                 va_list args) {
  if (!levelEnabled(level)) {
    return;
  }

  char buffer[ECMC_RT_LOGGER_MSG_SIZE];
  va_list argsCopy;
  va_copy(argsCopy, args);
  vsnprintf(buffer, sizeof(buffer), fmt, argsCopy);
  va_end(argsCopy);

  if (!started_.load(std::memory_order_acquire) ||
      !enabled_.load(std::memory_order_acquire)) {
    printMessage(level, buffer);
    return;
  }

  const size_t writeIndex = writeIndex_.load(std::memory_order_relaxed);
  const size_t nextWriteIndex = (writeIndex + 1) % ECMC_RT_LOGGER_QUEUE_SIZE;
  const size_t readIndex = readIndex_.load(std::memory_order_acquire);

  if (nextWriteIndex == readIndex) {
    droppedCount_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  queue_[writeIndex].level = level;
  queue_[writeIndex].sourceType = sourceType;
  queue_[writeIndex].sourceIndex = sourceIndex;
  strncpy(queue_[writeIndex].message, buffer, ECMC_RT_LOGGER_MSG_SIZE - 1);
  queue_[writeIndex].message[ECMC_RT_LOGGER_MSG_SIZE - 1] = '\0';
  writeIndex_.store(nextWriteIndex, std::memory_order_release);
}

}  // namespace

int ecmcRtLoggerStart() {
  if (started_.load(std::memory_order_acquire)) {
    return 0;
  }

  if (ecmcRtLoggerPortDriverStart()) {
    LOGWARNING("%s/%s:%d: WARNING: Failed to create RT logger asyn port driver. Continuing with IOC log output only.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
  }

  epicsThreadId threadId = epicsThreadCreate(
    ECMC_RT_LOGGER_THREAD,
    epicsThreadPriorityLow,
    epicsThreadGetStackSize(epicsThreadStackSmall),
    loggerTask,
    NULL);

  if (!threadId) {
    LOGWARNING("%s/%s:%d: WARNING: Failed to create low priority RT log thread. Using direct RT logging.\n",
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

const char *ecmcRtLoggerGetAsynPortName() {
  return ecmcRtLoggerPortDriverGetPortName();
}

void ecmcRtLoggerSetControlWord(unsigned int controlWord) {
  controlWord_.store(controlWord, std::memory_order_release);
}

unsigned int ecmcRtLoggerGetControlWord() {
  return controlWord_.load(std::memory_order_acquire);
}

void ecmcRtLoggerLogInfoSource(int sourceType,
                               int sourceIndex,
                               const char *fmt,
                               ...) {
  va_list args;
  va_start(args, fmt);
  logMessageV(ECMC_RT_LOG_LEVEL_INFO,
              sourceType,
              sourceIndex,
              fmt,
              args);
  va_end(args);
}

void ecmcRtLoggerLogErrorSource(int sourceType,
                                int sourceIndex,
                                const char *fmt,
                                ...) {
  va_list args;
  va_start(args, fmt);
  logMessageV(ECMC_RT_LOG_LEVEL_ERROR,
              sourceType,
              sourceIndex,
              fmt,
              args);
  va_end(args);
}

void ecmcRtLoggerLogWarningSource(int sourceType,
                                  int sourceIndex,
                                  const char *fmt,
                                  ...) {
  va_list args;
  va_start(args, fmt);
  logMessageV(ECMC_RT_LOG_LEVEL_WARNING,
              sourceType,
              sourceIndex,
              fmt,
              args);
  va_end(args);
}

void ecmcRtLoggerLogInfo(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  logMessageV(ECMC_RT_LOG_LEVEL_INFO,
              ECMC_RT_LOG_SOURCE_UNKNOWN,
              -1,
              fmt,
              args);
  va_end(args);
}

void ecmcRtLoggerLogError(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  logMessageV(ECMC_RT_LOG_LEVEL_ERROR,
              ECMC_RT_LOG_SOURCE_UNKNOWN,
              -1,
              fmt,
              args);
  va_end(args);
}

void ecmcRtLoggerLogWarning(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  logMessageV(ECMC_RT_LOG_LEVEL_WARNING,
              ECMC_RT_LOG_SOURCE_UNKNOWN,
              -1,
              fmt,
              args);
  va_end(args);
}
