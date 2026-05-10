/*************************************************************************\
* Copyright (c) 2026 Paul Scherrer Institute
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcCppLogicLib.h
*
\*************************************************************************/

#ifndef ECMC_CPP_LOGIC_LIB_H_
#define ECMC_CPP_LOGIC_LIB_H_

#include "ecmcError.h"

class ecmcCppLogicLib : public ecmcError {
 public:
  struct Impl;

  explicit ecmcCppLogicLib(int index);
  ~ecmcCppLogicLib() override;

  int load(const char* libFilenameWP, const char* configStr);
  int appendMacros(const char* macrosText);
  void unload();
  void report();

  int exeRTFunc(int controllerErrorCode);
  int exeEnterRTFunc();
  int exeExitRTFunc();

  const char* getPortName() const;
  const char* getLastErrorMessage() const;

 private:
  Impl* impl_;
};

#endif
