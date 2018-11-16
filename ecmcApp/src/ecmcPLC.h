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
#include <string.h>

#define ECMC_MAX_PLC_VARIABLES 1024
#define ECMC_MAX_PLC_VARIABLES_NAME_LENGTH 1024

#define ERROR_PLC_EXPRTK_ALLOCATION_FAILED 0x20500
#define ERROR_PLC_COMPILE_ERROR 0x20501
#define ERROR_PLC_AXIS_ID_OUT_OF_RANGE 0x20502
#define ERROR_PLC_ADD_EXPR_LINE_ERROR 0x20503
#define ERROR_PLC_EXPR_LINE_TO_LONG 0x20504
#define ERROR_PLC_PLC_DATA_IF_NULL 0x20505
#define ERROR_PLC_DATA_IF_ALLOCATION_FAILED 0x20506
#define ERROR_PLC_VARIABLE_COUNT_EXCEEDED 0x20507
#define ERROR_PLC_AXIS_OBJECT_NULL 0x20508
#define ERROR_PLC_DATA_STORAGE_INDEX_OUT_OF_RANGE 0x20509
#define ERROR_PLC_DATA_STORAGE_OBJECT_NULL 0x2050A

#define CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndex) {             \
  if(axIndex>=ECMC_MAX_AXES || axIndex<=0){                   \
    LOGERR("ERROR: Axis index out of range.\n");              \
    return (double)ERROR_PLC_AXIS_ID_OUT_OF_RANGE;}           \
    if(ecmcPLC::statAxes_[axIndex]==NULL){                    \
      LOGERR("ERROR: Axis object NULL\n");                    \
      return (double)ERROR_PLC_AXIS_OBJECT_NULL;              \
    }                                                         \
  }                                                           \

#define CHECK_PLC_DATA_STORAGE_RETURN_IF_ERROR(dsIndex) {     \
  if(dsIndex>=ECMC_MAX_DATA_STORAGE_OBJECTS || dsIndex<=0){   \
    LOGERR("ERROR: Data Storage index out of range.\n");      \
    return (double)ERROR_PLC_DATA_STORAGE_INDEX_OUT_OF_RANGE;}\
    if(ecmcPLC::statDs_[dsIndex]==NULL){                      \
      LOGERR("ERROR: Data Storage object NULL\n");            \
      return (double)ERROR_PLC_DATA_STORAGE_OBJECT_NULL;      \
    }                                                         \
  }                                                           \

class ecmcPLC : public ecmcError
{
public:
  ecmcPLC(int skipCycles);
  ~ecmcPLC();
  bool getCompiled();
  int  validate();
  int  execute(bool ecOK);
  std::string *getExpr();
  int  addExprLine(char *exprStr);
  int  clearExpr();
  int  compile();
  int  addAndReisterGlobalVar(ecmcPLCDataIF *dataIF);
  int  addAndRegisterLocalVar(char *localVarStr);
  int setAxisArrayPointer(ecmcAxisBase *axis,int index);
  int setDataStoragePointer(ecmcDataStorage *ds,int index);
  double  getSampleTime();
  static ecmcAxisBase *statAxes_[ECMC_MAX_AXES];
  static ecmcDataStorage *statDs_[ECMC_MAX_DATA_STORAGE_OBJECTS];
private:
  void initVars();
  int varExist(char *varName);
  int globalVarExist(const char *varName);
  int localVarExist(const char *varName);
  int getPLCErrorID(); //from PLC Code
  std::string exprStr_;
  bool compiled_;
  exprtkWrap *exprtk_;
  ecmcPLCDataIF *globalArray_[ECMC_MAX_PLC_VARIABLES];
  ecmcPLCDataIF *localArray_[ECMC_MAX_PLC_VARIABLES];
  int globalVariableCount_;
  int localVariableCount_;
  int inStartup_;
  int skipCycles_;
  int skipCyclesCounter_;
  double plcScanTimeInSecs_;
};

#endif /* ecmcPLC_H_ */

