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

# ifdef __cplusplus
extern "C" {
# endif  // ifdef __cplusplus

/** \brief Get an ecmcDataItem obj by idStringWP
 * 
 *  Allows access to all regsistered ecmcDataItems in ecmc.\n
 *  \param[in] idStringWP Identification string "with path".\n
 *                        examples: ec0.s1.AI_1\n
 *                                  ec0.s5.mm.CH1_ARRAY\n
 *                                  plcs.plc1.static.test\n
 *                                  ax1.enc.actpos\n
 *
 * \return ecmcDataItem (void*) object if success or otherwise NULL.\n
 *
 * \note Object returns a (void*) so that plugins can be compiled without\n
 *  asyn classes if they are not used.\n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
void* getEcmcDataItem(char *idStringWP);

/** \brief Get an ecmcAsynDataItem obj by idStringWP
 * 
 *  Allows access to all regsistered ecmcAsynDataItems in ecmc.\n
 *  The ecmcAsynDataItems class is dervied of ecmcDataItem class.\n
 *
 *  \param[in] idStringWP Identification string "with path".\n
 *                        examples: ec0.s1.AI_1\n
 *                                  ec0.s5.mm.CH1_ARRAY\n
 *                                  plcs.plc1.static.test\n
 *                                  ax1.enc.actpos\n
 *
 * \return ecmcAsynDataItem (void*) object if success or otherwise NULL.\n
 *
 * \note Object returns a (void*) so that plugins can be compiled without\n
 *  asyn classes if they are not used.\n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
void* getEcmcAsynDataItem(char *idStringWP);

/** \brief Get ecmcAsynPortObject (as void*)
 *
 * \return ecmcAsynPortObject (void*) object if success or otherwise NULL.\n
*
 * \note Object returns a (void*) so that plugins can be compiled without\n
 *  asyn classes if they are not used.\n
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