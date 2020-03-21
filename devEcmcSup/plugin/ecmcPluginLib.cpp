/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPluginLib.cpp
*
*  Created on: Mar 21, 2020
*      Author: anderssandstrom
*      Credits to  https://github.com/sgreg/dynamic-loading 
*
\*************************************************************************/

#include "ecmcPluginLib.h"
#include <dirent.h>
#include <errno.h>
#include <dlfcn.h>
#include <unistd.h>

ecmcPluginLib::ecmcPluginLib() {
  initVars();
}

ecmcPluginLib::~ecmcPluginLib() {
  unloadLib();
}

void ecmcPluginLib::initVars() {
  errorReset();
  libFilenameWP_ = "";
  dlHandle_      = NULL;
  getDataFunc_   = NULL;
  data_          = NULL;
}

int ecmcPluginLib::loadLib(const char* libFilenameWP) {
  libFilenameWP_ = libFilenameWP;
  //Ensure that file exist
  if(access( Filename.c_str(), 0 ) != 0){
    LOGERR("%s/%s:%d: Error: Plugin %s: File not found (0x%x) .\n",
           __FILE__, __FUNCTION__, __LINE__,
           libFilenameWP_,ERROR_PLUGIN_FLIE_NOT_FOUND);
    return setErrorID(ERROR_PLUGIN_FLIE_NOT_FOUND);
  }

  dlHandle_ = dlopen(libFilenameWP_, RTLD_LAZY);
  
  if ((dlHandle_ = dlopen(libFilenameWP_, RTLD_NOW)) == NULL) {
    LOGERR("%s/%s:%d: Error: Plugin %s: Open failed with error: %s (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE_, libFilenameWP_, dlerror(),
           ERROR_PLUGIN_OPEN_FAIL);
    return setErrorID(ERROR_PLUGIN_OPEN_FAIL);
  }

  if ( (getDataFunc_ = dlsym(dlHandle_, "_plugin_get_data")) == NULL) {
    LOGERR("%s/%s:%d: Error: Plugin %s: GetDataFunc failed with error: %s (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE_, libFilenameWP_, dlerror(),
           ERROR_PLUGIN_GET_DATA_FUNC_FAIL);
    dlclose(dlHandle_);
    return setErrorID(ERROR_PLUGIN_GET_DATA_FUNC_FAIL);
  }

  if ( (data_ = getDataFunc_()) == NULL) {
     LOGERR("%s/%s:%d: Error: Plugin %s: Get data failed with error (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE_, libFilenameWP_,
           ERROR_PLUGIN_GET_DATA_FAIL);
    dlclose(dlHandle_);
    return setErrorID(ERROR_PLUGIN_GET_DATA_FAIL);
  }

  if (data_->ifVer != ECMC_PLUG_VERSION_MAGIC) {
    LOGERR("%s/%s:%d: Error: Plugin %s: Interface version missmatch (%d!=%d) (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE_, libFilenameWP_, ECMC_PLUG_VERSION_MAGIC,
           data->ifVer,ERROR_PLUGIN_VERSION_MISSMATCH);
    dlclose(dlHandle_);
    return setErrorID(ERROR_PLUGIN_VERSION_MISSMATCH);
  }
 
  if (data_->libName == NULL || strlen(data_->libName) == 0) {
    LOGERR("%s/%s:%d: Error: Plugin %s: Name undefinded in data (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE_, libFilenameWP_,
           ERROR_PLUGIN_LIB_NAME_UNDEFINED);
    dlclose(dlHandle_);
    return setErrorID(ERROR_PLUGIN_LIB_NAME_UNDEFINED);
  }
  
  // Module loaded
  return 0;
}

void ecmcPluginLib::unloadLib() {
 // call destruct function if defined
 if(data_->destructFnc) {
    data_->destructFnc();
  }

  // Clode lib
  dlclose(dlHandle_);

  libFilenameWP_ = "";
  dlHandle_      = NULL;
  getDataFunc_   = NULL;
  data_          = NULL;
}

