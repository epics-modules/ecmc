/*
 * ecmcPLC.h
 *
 *  Created on: Oct 4, 2018
 *      Author: anderssandstrom
 */

#ifndef ecmcPLC_H_
#define ecmcPLC_H_

#include "ecmcPLCAxisDataID.h"
#include "ecmcDefinitions.h"
#include "exprtkWrap.h"
#include "ecmcAxisBase.h"
#include "ecmcEc.h"

#define ECMC_MAX_PLC_VARIABLES 1024

#define ERROR_PLC_EXPRTK_ALLOCATION_FAILED 0x20500
#define ERROR_PLC_COMPILE_ERROR 0x20501
#define ERROR_PLC_AXIS_ID_OUT_OF_RANGE 0x20502

class ecmcPLC : public ecmcError
{
public:
  ecmcPLC(ecmcAxisBase *axes[ECMC_MAX_AXES],ecmcEc *ec);
  ~ecmcPLC();
  int setExpression(std::string expressionString);
  bool getCompiled();
  int validate();
  int refresh();
  std::string *getExpression();
private:
  void initVars();
  int compile();
  int addAxisVar(int axisId,char *axisVarStr);
  int addEcVar(int ecId,char *ecVarStr);
  std::string expressionString_;
  bool compiled_;
  exprtkWrap *exprtk_;
  ecmcPLCDataID dataArray_[ECMC_MAX_PLC_VARIABLES];
  ecmcAxisBase *axes_[ECMC_MAX_AXES];
  ecmcEc *ec_;
};

#endif /* ecmcPLC_H_ */
