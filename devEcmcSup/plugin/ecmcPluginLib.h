/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPluginLib.h
*
*  Created on: Mar 21, 2020
*      Author: anderssandstrom
*
\*************************************************************************/
#ifndef ECMC_PLUGIN_LIB_H_
#define ECMC_PLUGIN_LIB_H_

#include "../main/ecmcError.h"
#include "ecmcPluginDefs.h"
#include "ecmcPluginDataRefs.h"

#define ERROR_PLUGIN_FLIE_NOT_FOUND 0x231000
#define ERROR_PLUGIN_OPEN_FAIL 0x231001
#define ERROR_PLUGIN_GET_DATA_FUNC_FAIL 0x231002
#define ERROR_PLUGIN_GET_DATA_FAIL 0x231003
#define ERROR_PLUGIN_VERSION_MISSMATCH 0x231004
#define ERROR_PLUGIN_LIB_NAME_UNDEFINED 0x231005

class ecmcPluginLib : public ecmcError {
 public:
  explicit ecmcPluginLib();
  ~ecmcPluginLib();
  int  load(const char* libFilenameWP);
  void unload();
  void report();
  ecmcPluginData *getData();
  int  exeRTFunc(int ecmcErrorCode);
  void exeDestructFunc();
  /** Note dataToPlugin only valid in plugin between 
   *  the calls of exeEnterRTFunc and exeExitRTFunc */
  int exeEnterRTFunc(ecmcPluginDataRefs* dataToPlugin);
  int exeExitRTFunc();
 private:
  void  initVars();
  char* libFilenameWP_;
  void   *dlHandle_;
  struct ecmcPluginData *(*getDataFunc_)(void);
  struct ecmcPluginData *data_;
  int loaded_;
  int dummyDataForRTFunc_;
};

#endif  /* ECMC_PLUGIN_LIB_H_ */
