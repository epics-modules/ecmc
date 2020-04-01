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
* This file contains usefull functions for use in plugins
*
\*************************************************************************/

#ifndef ECMC_PLUGIN_H_
#define ECMC_PLUGIN_H_

#include "ecmcDataItem.h"

# ifdef __cplusplus
extern "C" {
# endif  // ifdef __cplusplus

/** \brief Get an ecmcDataItem obj by idStringWP
 * 
 *  Allows access to all regsistered ecmcDataItems in ecmc.\n
 *
 *  \param[in] idStringWP Identification string "with path".\n
 *                        examples: ec0.s1.AI_1\n
 *                                  ec0.s5.mm.CH1_ARRAY\n
 *                                  plcs.plc1.static.test\n
 *                                  ax1.enc.actpos\n
 *
 * \return ecmcDataItem object if success or otherwise NULL.\n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
ecmcDataItem* getEcmcDataItem(char *idStringWP);

/** \brief Get ecmcAsynPortObject (as void*)
 *
 * \return ecmcAsynPortObject object if success or otherwise NULL.\n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
void* getEcmcAsynPortDriver();

/** \brief Get ecmc sample rate [Hz]
 *
 * \return Sample in Hz \n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
double getEcmcSampleRate();

/** \brief Get ecmc sample time in [ms]
 *
 * \return Sample in milliseconds \n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
double getEcmcSampleTimeMS();

# ifdef __cplusplus
}
# endif  // ifdef __cplusplus

#endif  /* ECMC_PLUGIN_H_ */