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

class ecmcError
{
public:
  ecmcError();
  virtual ~ecmcError();
  virtual int setErrorID(int errorID);
  virtual int setErrorID(const char* fileName,const char* functionName,int lineNumber,int errorID);
  virtual void setError(bool error);
  virtual void errorReset();
  virtual bool getError();
  virtual int getErrorID();
  static const char *convertErrorIdToString(int errorId);
private:
  void initVars();
  bool error_;
  int errorId_;
};

#endif /* ECMCERROR_H_ */
