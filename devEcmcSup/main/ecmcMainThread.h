/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcMainThread.h
*
*  Created on: Jan 11, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMC_MAIN_TASK_H_
#define ECMC_MAIN_TASK_H_

/**\file
 * \defgroup ecmc
 * Main interface for ECMC motion control.
 * \author Anders Sandström
 * \contact anders.sandstrom@esss.se
 */

#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <string.h>
#include "epicsThread.h" 
#include "ecmcDefinitions.h"

#define AXIS_CHECK_RETURN_USED_BUFFER(_axis) {init_axis(_axis); if (((_axis) <= 0) || ((_axis) >=ECMC_MAX_AXES)) return 0;}

#ifdef __cplusplus
extern "C" {
#endif  // ifdef __cplusplus

// Error Codes
#define ECMC_PARSER_READ_STORAGE_BUFFER_DATA_NULL 0x200000

/** 
 * \brief Initialization routine for ecmc.\n
 */
int ecmcInitThread(void);

/** \brief Set application mode
 *
 * Before entering runtime mode a validation of both hardware and motion
 * objects will be executed. See command validateConfig().\n
 *
 * \param[in] mode    Application mode to set.\n
 *   mode = 0: Configuration mode.\n
 *   mode = 1: Runtime mode.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set the application mode to runtime.\n
 * "Cfg.SetAppMode(1)" //Command string to ecmcCmdParser.c
 */
int setAppMode(int mode);

/** \brief Set EtherCAT startup timeout time
 *
 * Can be needed if many slaves in ethercat network. Default time is 30s.
 * 
 * \param[in] timeSeconds  Timeout in seconds (default 30s).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set EtherCAT startup timeout to 100s.\n
 * "Cfg.SetEcStartupTimeout(100)" //Command string to ecmcCmdParser.c
 */
int setEcStartupTimeout(int time_seconds);

/** \brief Set EtherCAT bus frame rate in [Hz] 
 *  EtherCAT bus period is only allowed to be changed prior any object creation.\n
 * 
 * \param[in] sampleRate  Sample rate of ethercat bus [Hz] (defaults to 1kHz).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note The EtherCAT bus time can also be set by "setSamplePeriodMs()"
 * 
 * \note Example: Set EtherCAT sample rate to 500Hz.\n
 * "Cfg.SetSampleRate(500)" //Command string to ecmcCmdParser.c
 */
int setSampleRate(double sampleRate);

/** \brief Set EtherCAT bus period in [ms] 
 *  EtherCAT bus period is only allowed to be changed prior any object creation.\n
 *  
 * \param[in] samplePeriodMs  Sample period of EtherCAT bus in [ms] (defaults to 1ms).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note The EtherCAT bus time can also be set by "setSampleRate()"
 * 
 * \note Example: Set EtherCAT sample period to 2ms.\n
 * "Cfg.SetSamplePeriodMs(2)" //Command string to ecmcCmdParser.c
 */
int setSamplePeriodMs(double samplePeriodMs);

/** \brief Update main asyn parameters
 *
 * \param[in] force Force update\n
 *
 */

void updateAsynParams(int force);

#ifdef __cplusplus
}
#endif  // ifdef __cplusplus

#endif  /* ECMC_MAIN_TASK_H_ */
