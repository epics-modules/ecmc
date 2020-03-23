/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPluginDataRefs.h
*
*  Created on: Mar 22, 2020
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMC_PLUGIN_DATA_REFS_H_
#define ECMC_PLUGIN_DATA_REFS_H_

#include "../ethercat/ecmcEc.h"

// Structure for making ecmc objects available in plugins
struct ecmcPluginDataRefs {
  double sampleTimeMS;
  ecmcEc *ec;
};

#endif  /* ECMC_PLUGIN_DATA_REFS_H_ */
