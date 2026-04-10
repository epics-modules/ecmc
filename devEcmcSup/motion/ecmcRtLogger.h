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

#include <stdint.h>

enum ecmcRtLogEventType {
  ECMC_RT_LOG_EVENT_AXIS_ERROR = 0,
  ECMC_RT_LOG_EVENT_SEQ_ERROR  = 1,
  ECMC_RT_LOG_EVENT_AXIS_STATE = 2,
  ECMC_RT_LOG_EVENT_HOME_STATE = 3
};

int  ecmcRtLoggerStart();
void ecmcRtLoggerSetEnabled(int enabled);
int  ecmcRtLoggerIsEnabled();
void ecmcRtLoggerQueueAxisError(int axisId, int errorCode);
void ecmcRtLoggerQueueSeqError(int axisId, int errorCode);
void ecmcRtLoggerQueueAxisState(int axisId, int axisState);
void ecmcRtLoggerQueueHomeState(int axisId, int cmdData, int seqState);

#endif  /* ECMC_RT_LOGGER_H_ */
