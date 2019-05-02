
#ifndef ECMC_MAIN_TASK_H_
# define ECMC_MAIN_TASK_H_

/**\file
 * \defgroup ecmc
 * Main interface for ECMC motion control.
 * \author Anders Sandström
 * \contact anders.sandstrom@esss.se
 */

# define __STDC_FORMAT_MACROS
# include <inttypes.h>
# include <string.h>
# include "main/ecmcDefinitions.h"

#define AXIS_CHECK_RETURN_USED_BUFFER(_axis) {init_axis(_axis); if (((_axis) <= 0) || ((_axis) >=ECMC_MAX_AXES)) return 0;}

# ifdef __cplusplus
extern "C" {
# endif  // ifdef __cplusplus

// Error Codes
# define ECMC_PARSER_READ_STORAGE_BUFFER_DATA_NULL 0x200000

/** 
 * \breif Initialization routine for ecmc.\n
 */
int ecmcInitThread(void);

/** \breif Sets application mode
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

/** \breif Update main asyn parameters
 *
 * \param[in] force Force update\n
 *
 */

void updateAsynParams(int force);

# ifdef __cplusplus
}
# endif  // ifdef __cplusplus

#endif  /* ECMC_MAIN_TASK_H_ */
