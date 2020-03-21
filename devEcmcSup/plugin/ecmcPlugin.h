/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPlugin.h
*
*  Created on: Mar 21, 2020
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMC_PLUGIN_H_
#define ECMC_PLUGIN_H_

#include "../com/ecmcOctetIF.h"        // Log Macros
#include "../main/ecmcErrorsList.h"
#include "../main/ecmcDefinitions.h"


# ifdef __cplusplus
extern "C" {
# endif  // ifdef __cplusplus

/** \brief Create a PLC object
 * called.\n
 *
 * \param[in] index PLC index number
 * \param[in] cycleTime CycleTime in seconds
 * \param[in] axisPLC If PLC for axis
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create PLC at index 0 (executing every 10th cycle).\n
 *  "Cfg.CreatePLC(0,9)" //Command string to ecmcCmdParser.c\n
 *
 * \note Example: Create PLC at index 0 (executing every cycle).\n
 *  "Cfg.CreatePLC(0)" //Command string to ecmcCmdParser.c\n
 */
int createPLC(int index,
              double cycleTime,
              int axisPLC);




# ifdef __cplusplus
}
# endif  // ifdef __cplusplus

#endif  /* ECMC_PLUGIN_H_ */