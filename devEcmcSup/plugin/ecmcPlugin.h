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

/**
\file
    @brief Plugin commands
*/

#ifndef ECMC_PLUGIN_H_
#define ECMC_PLUGIN_H_

# ifdef __cplusplus
extern "C" {
# endif  // ifdef __cplusplus

/** \brief Load a ecmc plugin.\n
 *
 * \param[in] pluginId index of plugin.
 * \param[in] filenameWP Filename of sharded lib with path.
 * \param[in] config configuration string sent to plugin on load.
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create and load a plugin at index 0.\n
 *  "Cfg.LoadPlugin(0,./ecmcExampleLip.so,"TEST=10")" //Command string to ecmcCmdParser.c\n
 */
int loadPlugin(int pluginId, const char* filenameWP, const char *config);

/** \brief Printout details of plugin.\n
 *
 * \param[in] pluginId index of plugin.
 * 
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Printout information about plug in at index 1.\n
 *  "Cfg.ReportPlugin(1)" //Command string to ecmcCmdParser.c\n
 */
int reportPlugin(int pluginId);

# ifdef __cplusplus
}
# endif  // ifdef __cplusplus

#endif  /* ECMC_PLUGIN_H_ */