/*
 * cMcuError.h
 *
 *  Created on: Mar 16, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCERROR_H_
#define ECMCERROR_H_
#include <string.h>
#include "stdio.h"
#include "cmd.h"

enum ecmcAlarmSeverity{
  ECMC_SEVERITY_WARNING=0,
  ECMC_SEVERITY_NORMAL=1,
  ECMC_SEVERITY_EMERGENCY=2
};


class ecmcError
{
public:
  ecmcError();
  virtual ~ecmcError();
  virtual int setErrorID(int errorID);
  virtual int setErrorID(int errorID,ecmcAlarmSeverity severity);
  virtual int setErrorID(const char* fileName,const char* functionName,int lineNumber,int errorID);
  virtual int setErrorID(const char* fileName,const char* functionName,int lineNumber,int errorID,ecmcAlarmSeverity severity);
  virtual void setError(bool error);
  virtual void errorReset();
  virtual bool getError();
  virtual int getErrorID();
  static const char *convertErrorIdToString(int errorId);
private:
  void initVars();
  bool error_;
  int errorId_;
  ecmcAlarmSeverity currSeverity_;
};

#endif /* ECMCERROR_H_ */
