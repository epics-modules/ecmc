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

// TODO: REMOVE GLOBALS
#include "../main/ecmcGlobalsExtern.h"

int loadPlugin(const char* filenameWP) {
  LOGINFO4("%s/%s:%d filenameWP=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           filenameWP);
  
  //find first free pluginindex
  int validId =  -1;
  for(int i = 0; i < ECMC_MAX_PLUGINS; ++i) {
    if(!plugins[i]){
      validId = i;
      break;
    }
  }

  if(validId<0){
    return ERROR_MAIN_PLUGIN_LIST_FULL;
  }

  plugins[validId] = new ecmcPluginLib();
  if(!plugins[validId]) {
    return ERROR_MAIN_PLUGIN_OBJECT_NULL;
  }

  int errorCode = plugins[validId]->load(filenameWP);
  if(errorCode) {
    delete plugins[validId];
    plugins[validId] = NULL;
    return errorCode;
  }

  return 0;
}
