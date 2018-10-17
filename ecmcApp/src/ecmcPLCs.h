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
#include "exprtkWrap.h"

#define ERROR_PLCS_INDEX_OUT_OF_RANGE 0x20700
#define ERROR_PLCS_AXIS_INDEX_OUT_OF_RANGE 0x20701
#define ERROR_PLCS_EC_NOT_INITIALIZED 0x20702
#define ERROR_PLCS_VARIABLE_NAME_TO_LONG 0x20703

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
  int parseExpr(int plcIndex,char * exprStr);
  int getExpr(int plcIndex, std::string *expr);
  int addExprLine(int plcIndex,char *exprStr);
  int clearExpr(int plcIndex);
  int compileExpr(int plcIndex);
  int setEnable(int plcIndex,int enable);
  int getEnable(int plcIndex,int *enabled);
  int getCompiled(int plcIndex,int *compiled);
  int validate();
  int  getErrorID();
  bool getError();
private:
  void initVars();
  int createNewGlobalDataIF(char * varName,ecmcDataSourceType dataSource,ecmcPLCDataIF **outDataIF);
  int createAndRegisterNewDataIF(int plcIndex,char * varName,ecmcDataSourceType dataSource);
  int getAxisIndex(char *varName);
  int addPLCDefaultVariables(int plcIndex,int skipCycles);
  int parseAxis(int plcIndex,char * exprStr);
  int parseEC(int plcIndex,char * exprStr);
  int parseStatic(int plcIndex,char * exprStr);
  int parseGlobal(int plcIndex,char * exprStr);
  int findGlobalDataIF(char * varName, ecmcPLCDataIF *outDataIF);
  int getPLCErrorID();
  int globalVariableCount_;
  ecmcPLC *plcs_[ECMC_MAX_PLCS];
  ecmcAxisBase *axes_[ECMC_MAX_AXES];
  ecmcEc *ec_;
  ecmcPLCDataIF *plcEnable_[ECMC_MAX_PLCS];
  ecmcPLCDataIF *plcError_[ECMC_MAX_PLCS];
  ecmcPLCDataIF *globalDataArray_[ECMC_MAX_PLC_VARIABLES];
};

#endif /* ecmcPLCs_H_ */