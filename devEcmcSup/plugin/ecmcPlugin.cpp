/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPlugin.cpp
*
*  Created on: Oct 21, 2020
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcPlugin.h"
#include "../com/ecmcOctetIF.h"        // Log Macros
#include "../main/ecmcDefinitions.h"
#include "../main/ecmcErrorsList.h"

// TODO: REMOVE GLOBALS
#include "../main/ecmcGlobalsExtern.h"

int loadPlugin(int pluginId, const char* filenameWP, const char* configStr) {
  LOGINFO4("%s/%s:%d pluginId = %d, filenameWP=%s, configStr=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           pluginId,
           filenameWP,
           configStr);

  if(pluginId < 0 || pluginId >= ECMC_MAX_PLUGINS){
    return ERROR_MAIN_PLUGIN_INDEX_OUT_OF_RANGE;
  }
  if(plugins[pluginId]){
    delete plugins[pluginId];
  }

  plugins[pluginId] = new ecmcPluginLib(pluginId);
  if(!plugins[pluginId]) {
    return ERROR_MAIN_PLUGIN_OBJECT_NULL;
  }

  int errorCode = plugins[pluginId]->load(filenameWP,configStr);
  if(errorCode) {
    delete plugins[pluginId];
    plugins[pluginId] = NULL;
    return errorCode;
  }

  return 0;
}

int reportPlugin(int pluginId) {
  LOGINFO4("%s/%s:%d pluginId = %d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           pluginId);
  
  if(pluginId < 0 || pluginId >= ECMC_MAX_PLUGINS){
    return ERROR_MAIN_PLUGIN_INDEX_OUT_OF_RANGE;
  }

  if(!plugins[pluginId]) {
    return ERROR_MAIN_PLUGIN_OBJECT_NULL;
  }

  plugins[pluginId]->report();
  return 0;
}

ecmcDataItem* getEcmcDataItem(char *idStringWP) {
  LOGINFO4("%s/%s:%d: idStringWP =%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           idStringWP);
  if(!asynPort) return NULL;

  return asynPort->findAvailDataItem(idStringWP);
}

void* getEcmcAsynPortDriver() {
  return (void*)asynPort;
}

double getEcmcSampleRate() {
  return mcuFrequency;
}

double getEcmcSampleTimeMS() {
  //mcuPeriod is in nano seconds
  return mcuPeriod/1000;
}
