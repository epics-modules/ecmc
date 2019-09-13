/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcError.h
*
*  Created on: Mar 16, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCERROR_H_
#define ECMCERROR_H_
#include <string.h>
#include <time.h>
#include "stdio.h"
#include "../com/ecmcOctetIF.h"

enum ecmcAlarmSeverity {
  ECMC_SEVERITY_NONE      = 0,
  ECMC_SEVERITY_WARNING   = 1,
  ECMC_SEVERITY_NORMAL    = 2,
  ECMC_SEVERITY_EMERGENCY = 3
};

#define PRINT_ERROR_PATH(fmt, ...)                                 \
  {                                                                \
    snprintf(errorPath_, sizeof(errorPath_), fmt, ## __VA_ARGS__); \
    errorPathValid_ = true;                                        \
    setErrorID(__FILE__, __FUNCTION__, __LINE__, 0);               \
  }

class ecmcError {
 public:
  ecmcError();
  virtual ~ecmcError();
  virtual int setErrorID(int errorID);
  virtual int setErrorID(int               errorID,
                         ecmcAlarmSeverity severity);
  virtual int setErrorID(const char *fileName,
                         const char *functionName,
                         int         lineNumber,
                         int         errorID);
  virtual int setErrorID(const char       *fileName,
                         const char       *functionName,
                         int               lineNumber,
                         int               errorID,
                         ecmcAlarmSeverity severity);
  virtual void              setError(bool error);
  virtual void              errorReset();
  virtual bool              getError();
  virtual int               getErrorID();
  virtual ecmcAlarmSeverity getSeverity();
  static const char       * convertErrorIdToString(int errorId);

 protected:
  char errorPath_[128];
  bool errorPathValid_;

 private:
  void initVars();
  bool error_;
  int errorId_;
  ecmcAlarmSeverity currSeverity_;
};

#endif  /* ECMCERROR_H_ */
