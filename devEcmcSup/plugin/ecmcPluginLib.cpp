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
  unload();
}

void ecmcPluginLib::initVars() {
  errorReset();
  libFilenameWP_ = "";
  dlHandle_      = NULL;
  getDataFunc_   = NULL;
  data_          = NULL;
  loaded_        = 0;
}

int ecmcPluginLib::load(const char* libFilenameWP) {
  
  // Unload old lib if loaded
  if(loaded_){
    unload();
  }

  libFilenameWP_ = libFilenameWP;
  
  // Ensure that file exist
  if(access( libFilenameWP, 0 ) != 0){
    LOGERR("%s/%s:%d: Error: Plugin %s: File not found (0x%x) .\n",
           __FILE__, __FUNCTION__, __LINE__,
           libFilenameWP_,ERROR_PLUGIN_FLIE_NOT_FOUND);
    return setErrorID(ERROR_PLUGIN_FLIE_NOT_FOUND);
  }

  dlHandle_ = dlopen(libFilenameWP_, RTLD_LAZY);
  
  if ((dlHandle_ = dlopen(libFilenameWP_, RTLD_NOW)) == NULL) {
    LOGERR("%s/%s:%d: Error: Plugin %s: Open failed with error: %s (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, libFilenameWP_, dlerror(),
           ERROR_PLUGIN_OPEN_FAIL);
    return setErrorID(ERROR_PLUGIN_OPEN_FAIL);
  }

  if ( (getDataFunc_ = (ecmcPluginData* (*)())dlsym(dlHandle_, "_plugin_get_data")) == NULL) {
    LOGERR("%s/%s:%d: Error: Plugin %s: GetDataFunc failed with error: %s (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, libFilenameWP_, dlerror(),
           ERROR_PLUGIN_GET_DATA_FUNC_FAIL);
    dlclose(dlHandle_);
    return setErrorID(ERROR_PLUGIN_GET_DATA_FUNC_FAIL);
  }

  if ( (data_ = getDataFunc_()) == NULL) {
     LOGERR("%s/%s:%d: Error: Plugin %s: Get data failed with error (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, libFilenameWP_,
           ERROR_PLUGIN_GET_DATA_FAIL);
    dlclose(dlHandle_);
    return setErrorID(ERROR_PLUGIN_GET_DATA_FAIL);
  }

  if (data_->ifVersion != ECMC_PLUG_VERSION_MAGIC) {
    LOGERR("%s/%s:%d: Error: Plugin %s: Interface version missmatch (%d!=%d) (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, libFilenameWP_, ECMC_PLUG_VERSION_MAGIC,
           data_->ifVersion,ERROR_PLUGIN_VERSION_MISSMATCH);
    dlclose(dlHandle_);
    return setErrorID(ERROR_PLUGIN_VERSION_MISSMATCH);
  }
 
  if (data_->name == NULL || strlen(data_->name) == 0) {
    LOGERR("%s/%s:%d: Error: Plugin %s: Name undefinded in data (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, libFilenameWP_,
           ERROR_PLUGIN_LIB_NAME_UNDEFINED);
    dlclose(dlHandle_);
    return setErrorID(ERROR_PLUGIN_LIB_NAME_UNDEFINED);
  }

  // Module loaded
  loaded_ = 1;
  LOGINFO4("%s/%s:%d: Info: Plugin %s: Loaded.\n",
           __FILE__, __FUNCTION__, __LINE__, libFilenameWP_);
 
  // call funcs
  int dummy=1;
  data_->constructFnc((void*)&dummy);
  data_->realtimeFnc(dummy);
  data_->destructFnc();
  report();
  return 0;
}

void ecmcPluginLib::unload() {
 
 // Call destruct function if defined
 if(data_->destructFnc) {
    data_->destructFnc();
  }

  // Close lib
  dlclose(dlHandle_);
  
  // Cleanup
  libFilenameWP_ = "";
  dlHandle_      = NULL;
  getDataFunc_   = NULL;
  data_          = NULL;
  loaded_        = false;
}

void ecmcPluginLib::report() {
  
  if(!loaded_){
    printf("Plugin not loaded.\n");
  }

  if(!data_)
  {
    printf("Plugin data NULL\n");
  }

  printf("Plugin info: \n");
  printf("  Name           = %s\n",data_->name);
  printf("  Version        = %d\n",data_->version);
  printf("  If version     = %d (ecmc = %d)\n",
        data_->ifVersion, ECMC_PLUG_VERSION_MAGIC);
  printf("  Construct func = @%p\n",data_->constructFnc);
  printf("  Destruct func  = @%p\n",data_->destructFnc);
  printf("  Realtime func  = @%p\n",data_->realtimeFnc);
  printf("  dlhandle       = @%p\n",dlHandle_);
  printf("  Plc functions:\n");
  // Loop funcs[]
  for(int i=0;i<ECMC_PLUGIN_MAX_PLC_FUNC_COUNT;++i){
 
    if(!data_->funcs[i].funcName || 
        strlen(data_->funcs[i].funcName) == 0 ||
        data_->funcs[i].argCount < 0 || 
        data_->funcs[i].argCount > ECMC_PLUGIN_MAX_PLC_ARG_COUNT ){
      break;
    }
    
    printf("    funcs[%d]:\n",i);
    printf("      Name       = %s\n",data_->funcs[i].funcName);
    printf("      Arg count  = %d\n",data_->funcs[i].argCount);
    switch(data_->funcs[i].argCount) {
      case 0:
        printf("      func       = @%p\n",data_->funcs[i].funcArg0);
        break;
      case 1:
        printf("      func       = @%p\n",data_->funcs[i].funcArg1);
        break;
      case 2:
        printf("      func       = @%p\n",data_->funcs[i].funcArg2);
        break;
      case 3:
        printf("      func       = @%p\n",data_->funcs[i].funcArg3);
        break;
      case 4:
        printf("      func       = @%p\n",data_->funcs[i].funcArg4);
        break;
      case 5:
        printf("      func       = @%p\n",data_->funcs[i].funcArg5);
        break;
      case 6:
        printf("      func       = @%p\n",data_->funcs[i].funcArg6);
        break;
      default:
        break;
    }
  }
  printf("\n");
}
