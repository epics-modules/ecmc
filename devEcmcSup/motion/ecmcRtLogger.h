/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcRtLogger.h
*
*  Created on: Apr 10, 2026
*
\*************************************************************************/

#ifndef ECMC_RT_LOGGER_H_
#define ECMC_RT_LOGGER_H_

#include "ecmcOctetIF.h"

int  ecmcRtLoggerStart();
void ecmcRtLoggerSetEnabled(int enabled);
int  ecmcRtLoggerIsEnabled();
void ecmcRtLoggerLogInfo(const char *fmt, ...);
void ecmcRtLoggerLogError(const char *fmt, ...);

#define ECMC_RT_LOGINFO4(fmt, ...) \
  do { \
    if (PRINT_STDOUT_BIT4()) { \
      ecmcRtLoggerLogInfo(fmt, ## __VA_ARGS__); \
    } \
  } while (0)

#define ECMC_RT_LOGINFO5(fmt, ...) \
  do { \
    if (PRINT_STDOUT_BIT5()) { \
      ecmcRtLoggerLogInfo(fmt, ## __VA_ARGS__); \
    } \
  } while (0)

#define ECMC_RT_LOGINFO12(fmt, ...) \
  do { \
    if (PRINT_STDOUT_BIT12()) { \
      ecmcRtLoggerLogInfo(fmt, ## __VA_ARGS__); \
    } \
  } while (0)

#endif  /* ECMC_RT_LOGGER_H_ */
