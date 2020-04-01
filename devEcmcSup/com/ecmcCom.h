/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcCom.h
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMC_MISC_H_
#define ECMC_MISC_H_

#include "ecmcDataItem.h"

# ifdef __cplusplus
extern "C" {
# endif  // ifdef __cplusplus

/** \brief Initilize ecmc
 *
 *  \param[in] asynPortObject Asyn port object.\n
 * \return 0 if success or otherwise an error code.\n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
int ecmcInit(void *asynPortObject);

/** \brief Add default asyn parameters for ecmc
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
int ecmcAddDefaultAsynParams();

/** \brief Cleanup (for exit)
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */

void ecmcCleanup();

/** \brief Get an ecmcDataItem obj by idStringWP
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

/** \brief Get ecmcAsynPortObject
 *
 * \return ecmcAsynPortObject object if success or otherwise NULL.\n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
void* getEcmcAsynPortDriver();

# ifdef __cplusplus
}
# endif  // ifdef __cplusplus

#endif  /* ECMC_MISC_H_ */