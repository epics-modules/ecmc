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

#define ECMC_RT_LOGGER_CONTROL_INFO_ENABLE  0x1
#define ECMC_RT_LOGGER_CONTROL_ERROR_ENABLE 0x2
#define ECMC_RT_LOGGER_CONTROL_DEFAULT \
  (ECMC_RT_LOGGER_CONTROL_INFO_ENABLE | ECMC_RT_LOGGER_CONTROL_ERROR_ENABLE)

int  ecmcRtLoggerStart();
void ecmcRtLoggerSetEnabled(int enabled);
int  ecmcRtLoggerIsEnabled();
const char *ecmcRtLoggerGetAsynPortName();
void ecmcRtLoggerSetControlWord(unsigned int controlWord);
unsigned int ecmcRtLoggerGetControlWord();
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
