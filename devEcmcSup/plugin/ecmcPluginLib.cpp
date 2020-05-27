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
#include <stdlib.h>

ecmcPluginLib::ecmcPluginLib(int index) {
  initVars();
  index_ = index;
}

ecmcPluginLib::~ecmcPluginLib() {
  if(loaded_) {
    unload();
  }
}

void ecmcPluginLib::initVars() {
  errorReset();
  libFilenameWP_ = NULL;
  libConfigStr_  = NULL;
  dlHandle_      = NULL;
  getDataFunc_   = NULL;
  data_          = NULL;
  loaded_        = 0;
  index_         = 0;
}

int ecmcPluginLib::load(const char* libFilenameWP, const char* libConfigStr) {
  
  // Unload old lib if loaded
  if(loaded_){
    unload();
  }
  
  // Ensure that file exist
  if(access( libFilenameWP, 0 )!= 0){
    LOGERR("%s/%s:%d: Error: Plugin %s: File not found (0x%x) .\n",
           __FILE__, __FUNCTION__, __LINE__,
           libFilenameWP,ERROR_PLUGIN_FLIE_NOT_FOUND);
    return setErrorID(ERROR_PLUGIN_FLIE_NOT_FOUND);
  }

  dlHandle_ = dlopen(libFilenameWP, RTLD_LAZY);
  
  if ((dlHandle_ = dlopen(libFilenameWP, RTLD_NOW)) == NULL) {
    LOGERR("%s/%s:%d: Error: Plugin %s: Open failed with error: %s (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, libFilenameWP, dlerror(),
           ERROR_PLUGIN_OPEN_FAIL);
    return setErrorID(ERROR_PLUGIN_OPEN_FAIL);
  }

  if ( (getDataFunc_ = (ecmcPluginData* (*)())dlsym(dlHandle_, "_plugin_get_data")) == NULL) {
    LOGERR("%s/%s:%d: Error: Plugin %s: GetDataFunc failed with error: %s (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, libFilenameWP, dlerror(),
           ERROR_PLUGIN_GET_DATA_FUNC_FAIL);
    dlclose(dlHandle_);
    return setErrorID(ERROR_PLUGIN_GET_DATA_FUNC_FAIL);
  }

  if ( (data_ = getDataFunc_()) == NULL) {
     LOGERR("%s/%s:%d: Error: Plugin %s: Get data failed with error (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, libFilenameWP,
           ERROR_PLUGIN_GET_DATA_FAIL);
    dlclose(dlHandle_);
    return setErrorID(ERROR_PLUGIN_GET_DATA_FAIL);
  }

  if (data_->ifVersion != ECMC_PLUG_VERSION_MAGIC) {
    LOGERR("%s/%s:%d: Error: Plugin %s: Interface version missmatch (%d!=%d) (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, libFilenameWP, ECMC_PLUG_VERSION_MAGIC,
           data_->ifVersion,ERROR_PLUGIN_VERSION_MISSMATCH);
    dlclose(dlHandle_);
    return setErrorID(ERROR_PLUGIN_VERSION_MISSMATCH);
  }
 
  if (data_->name == NULL || strlen(data_->name) == 0) {
    LOGERR("%s/%s:%d: Error: Plugin %s: Name undefinded in data (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, libFilenameWP,
           ERROR_PLUGIN_LIB_NAME_UNDEFINED);
    dlclose(dlHandle_);
    return setErrorID(ERROR_PLUGIN_LIB_NAME_UNDEFINED);
  }

  // Module loaded
  loaded_ = 1;
  libFilenameWP_ = strdup(libFilenameWP);
  libConfigStr_  = strdup(libConfigStr);
  LOGINFO4("%s/%s:%d: Info: Plugin %s: Loaded.\n",
           __FILE__, __FUNCTION__, __LINE__, libFilenameWP_);
   
  // Call constructor
  int errorCode = data_->constructFnc(libConfigStr_);
  if(errorCode) {
    LOGERR("%s/%s:%d: Error: Plugin %s returned error @ constructFnc() (0x%x)."
           " Plugin unloads....\n",
           __FILE__, __FUNCTION__, __LINE__, libFilenameWP_,
           errorCode);
    unload();
    return errorCode;
  } 

  return 0;
}

void ecmcPluginLib::unload() {
 
 // Call destruct function if defined
 if(data_->destructFnc) {
    data_->destructFnc();
  }

  // Close lib
  if(loaded_) {
    dlclose(dlHandle_);
    free(libFilenameWP_);
    free(libConfigStr_);
  }
  // Cleanup
  libFilenameWP_ = NULL;
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
  printf("  Index                = %d\n",index_);
  printf("  Name                 = %s\n",data_->name);
  printf("  Description          = %s\n",data_->desc);
  printf("  Option description   = %s\n",data_->optionDesc);
  printf("  Filename             = %s\n",libFilenameWP_);
  printf("  Config string        = %s\n",libConfigStr_);
  printf("  Version              = %d\n",data_->version);
  printf("  Interface version    = %d (ecmc = %d)\n",
        data_->ifVersion, ECMC_PLUG_VERSION_MAGIC);
  printf("     max plc funcs     = %d\n",ECMC_PLUGIN_MAX_PLC_FUNC_COUNT);
  printf("     max plc func args = %d\n",ECMC_PLUGIN_MAX_PLC_ARG_COUNT);
  printf("     max plc consts    = %d\n",ECMC_PLUGIN_MAX_PLC_CONST_COUNT);
  printf("  Construct func       = @%p\n",data_->constructFnc);
  printf("  Enter realtime func  = @%p\n",data_->realtimeEnterFnc);
  printf("  Exit realtime func   = @%p\n",data_->realtimeExitFnc);
  printf("  Realtime func        = @%p\n",data_->realtimeFnc);
  printf("  Destruct func        = @%p\n",data_->destructFnc);
  printf("  dlhandle             = @%p\n",dlHandle_);
  printf("  Plc functions:\n");
  // Loop funcs[]
  for(int i=0;i<ECMC_PLUGIN_MAX_PLC_FUNC_COUNT;++i){

    int argCount = findArgCount(data_->funcs[i]);
    if(!data_->funcs[i].funcName || 
        strlen(data_->funcs[i].funcName) == 0 ||
        ((argCount > ECMC_PLUGIN_MAX_PLC_ARG_COUNT ||
        argCount < 0) && data_->funcs[i].funcGenericObj==NULL)){
      break;
    }

    //build prototype
    char protoBuffer[ECMC_PLUGIN_MAX_PLC_ARG_COUNT*(strlen("argX, ")+1)];
    char *pProto=&protoBuffer[0];
    memset(protoBuffer,0,sizeof(protoBuffer));
    for(int j = 0;j < argCount; ++j){
      snprintf(pProto,sizeof(protoBuffer)-strlen(pProto)-1,"arg%d, ",j);
      pProto+=strlen(pProto);
    }
    //remove last ", " (two chars) if any args
    if(strlen(protoBuffer)>2 && argCount > 0) {
      protoBuffer[strlen(protoBuffer)-1] = 0;
      protoBuffer[strlen(protoBuffer)-1] = 0;
    }
    printf("    funcs[%02d]:\n",i);
    printf("      Name       = \"%s(%s);\"\n",
           data_->funcs[i].funcName,
           protoBuffer);
    printf("      Desc       = %s\n",data_->funcs[i].funcDesc);
    // generic_function_t generic func object (allow strings)
    if(data_->funcs[i].funcGenericObj) {
      printf("      func       = @%p\n",data_->funcs[i].funcGenericObj);
    }
    else {
      printf("      Arg count  = %d\n",argCount);
      switch(argCount) {
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
        case 7:
          printf("      func       = @%p\n",data_->funcs[i].funcArg7);
          break;
        case 8:
          printf("      func       = @%p\n",data_->funcs[i].funcArg8);
          break;
        case 9:
          printf("      func       = @%p\n",data_->funcs[i].funcArg9);
          break;
        case 10:
          printf("      func       = @%p\n",data_->funcs[i].funcArg10);
          break;
        default:      
          break;
      }
    }
  }

  printf("  Plc constants:\n");
  for(int i=0;i<ECMC_PLUGIN_MAX_PLC_CONST_COUNT;++i){
    if(!data_->consts[i].constName || 
        strlen(data_->consts[i].constName) == 0){
      break;
    }
    printf("    consts[%02d]:\n",i);
    printf("      Name     = \"%s\" = %3.3lf\n",
          data_->consts[i].constName,
          data_->consts[i].constValue);
    printf("      Desc     = %s\n",data_->consts[i].constDesc);
    
  }
  printf("\n");
}

ecmcPluginData *ecmcPluginLib::getData() {
  return data_;
}

int ecmcPluginLib::exeRTFunc(int ecmcErrorCode) {
  if(!loaded_ || !data_) {
    return 0;
  }

  if(data_->realtimeFnc) {
    return data_->realtimeFnc(ecmcErrorCode);
  }

  return 0;
}

void ecmcPluginLib::exeDestructFunc() {
  if(!loaded_ || !data_) {
    return;
  }

  if(data_->destructFnc) {
    data_->destructFnc();
  }
}

int ecmcPluginLib::exeEnterRTFunc() {
  if(!loaded_ || !data_) {
    return 0;
  }

  if(data_->realtimeEnterFnc) {
    int errorCode = data_->realtimeEnterFnc();
    if(errorCode) {
      LOGERR("%s/%s:%d: Error: Plugin %s returned error @ realtimeEnterFnc() (0x%x).\n",
             __FILE__, __FUNCTION__, __LINE__, libFilenameWP_,
             errorCode);
      return errorCode;
    }
  }
  return 0;
}

int ecmcPluginLib::exeExitRTFunc() {
  if(!loaded_ || !data_) {
    return 0;
  }

  if(data_->realtimeExitFnc) {
    int errorCode = data_->realtimeExitFnc();
    if(errorCode) {
      LOGERR("%s/%s:%d: Error: Plugin %s returned error @ realtimeExitFnc() (0x%x).\n",
             __FILE__, __FUNCTION__, __LINE__, libFilenameWP_,
             errorCode);
      return errorCode;
    }
  }
  return 0;
}

int ecmcPluginLib::findArgCount(ecmcOnePlcFunc &func){
  
  if(func.funcArg0) {
    return 0;
  }
  if(func.funcArg1) {
    return 1;
  }
  if(func.funcArg2) {
    return 2;
  }
  if(func.funcArg3) {
    return 3;
  }
  if(func.funcArg4) {
    return 4;
  }
  if(func.funcArg5) {
    return 5;
  }
  if(func.funcArg6) {
    return 6;
  }
  return -1;
}
