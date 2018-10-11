/*
 * ecmcPLC.h
 *
 *  Created on: Oct 4, 2018
 *      Author: anderssandstrom
 */

#ifndef ecmcPLC_H_
#define ecmcPLC_H_

#include "ecmcPLCDataIF.h"
#include "ecmcDefinitions.h"
#include "exprtkWrap.h"
#include "ecmcAxisBase.h"
#include "ecmcEc.h"
#include <string.h>

#define ECMC_MAX_PLC_VARIABLES 1024

#define ERROR_PLC_EXPRTK_ALLOCATION_FAILED 0x20500
#define ERROR_PLC_COMPILE_ERROR 0x20501
#define ERROR_PLC_AXIS_ID_OUT_OF_RANGE 0x20502
#define ERROR_PLC_ADD_EXPR_LINE_ERROR 0x20503
#define ERROR_PLC_EXPR_LINE_TO_LONG 0x20504
#define ERROR_PLC_PLC_DATA_IF_NULL 0x20505

class ecmcPLC : public ecmcError
{
public:
  ecmcPLC(ecmcEc *ec,int skipCycles);
  ~ecmcPLC();
  int setExpr(char *exprStr);
  int setAxisArrayPointer(ecmcAxisBase *axis,int index);
  bool getCompiled();
  int validate();
  int execute(bool ecOK);
  std::string *getExpr();
  int addExprLine(char *exprStr);
  int clearExpr();
  int compile();
  int setEnable(int enable);

private:
  void initVars();
  int addAxisVar(int axisId,char *axisVarStr);
  int addEcVar(int ecId,char *ecVarStr);
  int addStaticVar(char *staticVarStr);
  int parseExpr(char *exprStr);
  int varExist(char *varName);
  std::string exprStr_;
  bool compiled_;
  exprtkWrap *exprtk_;
  ecmcPLCDataIF *dataArray_[ECMC_MAX_PLC_VARIABLES];
  ecmcAxisBase *axes_[ECMC_MAX_AXES];
  ecmcEc *ec_;
  int variableCount_;
  int inStartup_;
  int enable_;
  int skipCycles_;
  int skipCyclesCounter_;
};

#endif /* ecmcPLC_H_ */

