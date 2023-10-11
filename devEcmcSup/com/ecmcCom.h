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
void ecmcCleanup(int signum);

# ifdef __cplusplus
}
# endif  // ifdef __cplusplus

#endif  /* ECMC_MISC_H_ */
