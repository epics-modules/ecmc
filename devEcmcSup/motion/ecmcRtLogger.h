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

#define ECMC_RT_LOG_TYPE_BIT(type) (1u << (unsigned int)(type))

enum ecmcRtLoggerSourceType {
  ECMC_RT_LOG_SOURCE_UNKNOWN      = 0,
  ECMC_RT_LOG_SOURCE_AXIS         = 1,
  ECMC_RT_LOG_SOURCE_AXIS_BASE    = 2,
  ECMC_RT_LOG_SOURCE_AXIS_SEQ     = 3,
  ECMC_RT_LOG_SOURCE_AXIS_DATA    = 4,
  ECMC_RT_LOG_SOURCE_AXIS_MON     = 5,
  ECMC_RT_LOG_SOURCE_AXIS_ENC     = 6,
  ECMC_RT_LOG_SOURCE_AXIS_DRV     = 7,
  ECMC_RT_LOG_SOURCE_AXIS_PID     = 8,
  ECMC_RT_LOG_SOURCE_AXIS_TRAJ    = 9,
  ECMC_RT_LOG_SOURCE_AXIS_PVT     = 10,
  ECMC_RT_LOG_SOURCE_MOTOR        = 11,
  ECMC_RT_LOG_SOURCE_MOTOR_AXIS   = 12,
  ECMC_RT_LOG_SOURCE_MOTOR_CTRL   = 13,
  ECMC_RT_LOG_SOURCE_MASTER_SLAVE = 14,
  ECMC_RT_LOG_SOURCE_ETHERCAT     = 15,
  ECMC_RT_LOG_SOURCE_PLC          = 16,
};

enum ecmcRtLoggerFilterMode {
  ECMC_RT_LOG_FILTER_NONE     = 0,
  ECMC_RT_LOG_FILTER_ALL      = 1,
  ECMC_RT_LOG_FILTER_SELECTED = 2,
};

int  ecmcRtLoggerStart();
void ecmcRtLoggerSetEnabled(int enabled);
int  ecmcRtLoggerIsEnabled();
const char *ecmcRtLoggerGetAsynPortName();
void ecmcRtLoggerSetControlWord(unsigned int controlWord);
unsigned int ecmcRtLoggerGetControlWord();
void ecmcRtLoggerLogInfoSource(int sourceType,
                               int sourceIndex,
                               const char *fmt,
                               ...);
void ecmcRtLoggerLogErrorSource(int sourceType,
                                int sourceIndex,
                                const char *fmt,
                                ...);
void ecmcRtLoggerLogInfo(const char *fmt, ...);
void ecmcRtLoggerLogError(const char *fmt, ...);

#define ECMC_RT_LOG_INFO_SOURCE(sourceType, sourceIndex, fmt, ...) \
  ecmcRtLoggerLogInfoSource(sourceType, sourceIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_ERROR_SOURCE(sourceType, sourceIndex, fmt, ...) \
  ecmcRtLoggerLogErrorSource(sourceType, sourceIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_BASE_INFO(axisIndex, fmt, ...) \
  ECMC_RT_LOG_INFO_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_BASE, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_BASE_ERROR(axisIndex, fmt, ...) \
  ECMC_RT_LOG_ERROR_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_BASE, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_SEQ_INFO(axisIndex, fmt, ...) \
  ECMC_RT_LOG_INFO_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_SEQ, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_SEQ_ERROR(axisIndex, fmt, ...) \
  ECMC_RT_LOG_ERROR_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_SEQ, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_DATA_INFO(axisIndex, fmt, ...) \
  ECMC_RT_LOG_INFO_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_DATA, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_DATA_ERROR(axisIndex, fmt, ...) \
  ECMC_RT_LOG_ERROR_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_DATA, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_MON_INFO(axisIndex, fmt, ...) \
  ECMC_RT_LOG_INFO_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_MON, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_MON_ERROR(axisIndex, fmt, ...) \
  ECMC_RT_LOG_ERROR_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_MON, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_ENC_INFO(axisIndex, fmt, ...) \
  ECMC_RT_LOG_INFO_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_ENC, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_ENC_ERROR(axisIndex, fmt, ...) \
  ECMC_RT_LOG_ERROR_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_ENC, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_DRV_INFO(axisIndex, fmt, ...) \
  ECMC_RT_LOG_INFO_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_DRV, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_DRV_ERROR(axisIndex, fmt, ...) \
  ECMC_RT_LOG_ERROR_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_DRV, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_PID_INFO(axisIndex, fmt, ...) \
  ECMC_RT_LOG_INFO_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_PID, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_PID_ERROR(axisIndex, fmt, ...) \
  ECMC_RT_LOG_ERROR_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_PID, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_TRAJ_INFO(axisIndex, fmt, ...) \
  ECMC_RT_LOG_INFO_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_TRAJ, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_AXIS_TRAJ_ERROR(axisIndex, fmt, ...) \
  ECMC_RT_LOG_ERROR_SOURCE(ECMC_RT_LOG_SOURCE_AXIS_TRAJ, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_MOTOR_AXIS_INFO(axisIndex, fmt, ...) \
  ECMC_RT_LOG_INFO_SOURCE(ECMC_RT_LOG_SOURCE_MOTOR_AXIS, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_MOTOR_AXIS_ERROR(axisIndex, fmt, ...) \
  ECMC_RT_LOG_ERROR_SOURCE(ECMC_RT_LOG_SOURCE_MOTOR_AXIS, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_MOTOR_CTRL_INFO(axisIndex, fmt, ...) \
  ECMC_RT_LOG_INFO_SOURCE(ECMC_RT_LOG_SOURCE_MOTOR_CTRL, axisIndex, fmt, ## __VA_ARGS__)

#define ECMC_RT_LOG_MOTOR_CTRL_ERROR(axisIndex, fmt, ...) \
  ECMC_RT_LOG_ERROR_SOURCE(ECMC_RT_LOG_SOURCE_MOTOR_CTRL, axisIndex, fmt, ## __VA_ARGS__)

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
