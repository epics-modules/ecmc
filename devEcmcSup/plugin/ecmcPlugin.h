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

# ifdef __cplusplus
extern "C" {
# endif  // ifdef __cplusplus

/** \brief Load a ecmc plugin.\n
 *
 * \param[in] filenameWP Filename of sharded lib with path.
 * \param[in] Configuration string sent to plugin on load.
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create PLC at index 0 (executing every 10th cycle).\n
 *  "Cfg.LoadPlugin(./ecmcExampleLip.so)" //Command string to ecmcCmdParser.c\n
 */
int loadPlugin(int pluginId, const char* filenameWP, const char *config);

# ifdef __cplusplus
}
# endif  // ifdef __cplusplus

#endif  /* ECMC_PLUGIN_H_ */