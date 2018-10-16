/*
 * ecmcPLCs.h
 *
 *  Created on: Oct 15, 2018
 *      Author: anderssandstrom
 */

#ifndef ecmcPLCs_H_
#define ecmcPLCs_H_

#include "ecmcPLC.h"
#include "ecmcDefinitions.h"
#include "ecmcAxisBase.h"
#include "ecmcEc.h"

#define ERROR_PLCS_INDEX_OUT_OF_RANGE 0x20700
#define ERROR_PLCS_AXIS_INDEX_OUT_OF_RANGE 0x20701
#define ERROR_PLCS_EC_NOT_INITIALIZED 0x20702

#define CHECK_PLC_RETURN_IF_ERROR(index) {if(index>=ECMC_MAX_PLCS || index<0){LOGERR("ERROR: PLC index out of range.\n");return ERROR_PLCS_INDEX_OUT_OF_RANGE;}}

class ecmcPLCs : public ecmcError
{
public:
  ecmcPLCs(ecmcEc *ec);
  ~ecmcPLCs();
  int createPLC(int plcIndex, int skipCycles);
  int deletePLC(int plcIndex);
  int setAxisArrayPointer(ecmcAxisBase *axis,int index);
  int execute(bool ecOK);
  int setExpr(int plcIndex,char *expr);
  int getExpr(int plcIndex, std::string *expr);
  int addExprLine(int plcIndex,char *exprStr);
  int clearExpr(int plcIndex);
  int compileExpr(int plcIndex);
  int setEnable(int plcIndex,int enable);
  int getEnable(int plcIndex,int *enabled);
  int getCompiled(int plcIndex,int *compiled);
  int validate();
private:
  void initVars();
  ecmcPLC *plcs_[ECMC_MAX_PLCS];
  ecmcAxisBase *axes_[ECMC_MAX_AXES];
  ecmcEc *ec_;
  ecmcPLCDataIF *plcEnable_[ECMC_MAX_PLCS];
  ecmcPLCDataIF *plcError_[ECMC_MAX_PLCS];
  ecmcPLCDataIF *plcScanTime_[ECMC_MAX_PLCS];
};

#endif /* ecmcPLCs_H_ */

