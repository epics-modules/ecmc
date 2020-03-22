/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPLCMain.h
*
*  Created on: Oct 15, 2018
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMC_PLC_MAIN_H_
#define ECMC_PLC_MAIN_H_

#include "exprtkWrap.h"
#include <iostream>
#include <fstream>
#include <string>
#include "../com/ecmcAsynPortDriver.h"
#include "../main/ecmcDefinitions.h"
#include "../motion/ecmcAxisBase.h"
#include "../misc/ecmcDataStorage.h"
#include "../ethercat/ecmcEc.h"
#include "ecmcPLCTask.h"
#include "ecmcPLCDataIF.h"

#define ERROR_PLCS_INDEX_OUT_OF_RANGE 0x20700
#define ERROR_PLCS_AXIS_INDEX_OUT_OF_RANGE 0x20701
#define ERROR_PLCS_EC_NOT_INITIALIZED 0x20702
#define ERROR_PLCS_VARIABLE_NAME_TO_LONG 0x20703
#define ERROR_PLCS_DATA_STORAGE_INDEX_OUT_OF_RANGE 0x20704
#define ERROR_PLCS_FILE_NOT_FOUND 0x20705
#define ERROR_PLCS_INVALID_VAR_NAME 0x20706
#define ERROR_PLCS_PLC_NULL 0x20707
#define ERROR_PLCS_EC_VAR_BIT_ACCESS_NOT_ALLOWED 0x20708

#define CHECK_PLC_RETURN_IF_ERROR(index) {                        \
    if (index >= ECMC_MAX_PLCS + ECMC_MAX_AXES || index < 0) {    \
      LOGERR("ERROR: PLC index out of range.\n");                 \
      return ERROR_PLCS_INDEX_OUT_OF_RANGE;                       \
    }                                                             \
    if(!plcs_[index]) {                                           \
      LOGERR("ERROR: PLC object NULL.\n");                        \
      return ERROR_PLCS_PLC_NULL;                                 \
    }                                                             \
}                                                                 \

class ecmcPLCMain : public ecmcError {
 public:
  explicit ecmcPLCMain(ecmcEc *ec,
                       double mcuFreq,
                       ecmcAsynPortDriver *asynPortDriver);
  ~ecmcPLCMain();
  int  createPLC(int plcIndex,
                 int skipCycles);
  int  deletePLC(int plcIndex);
  int  setAxisArrayPointer(ecmcAxisBase *axis,
                           int           index);
  int  setDataStoragePointer(ecmcDataStorage *ds,
                             int              index);
  int  execute(bool ecOK);
  int  execute(int   plcIndex, bool ecOK);
  int  setExpr(int   plcIndex,
               char *expr);
  int  parseExpr(int         plcIndex,
                 const char *exprStr);
  std::string *getExpr(int plcIndex,
                       int *error);
  int  addExprLine(int         plcIndex,
                   const char *exprStr);
  int appendExprLine(int plcIndex,
                    const char *expr);
  int  loadPLCFile(int   plcIndex,
                   char *fileName);
  int  clearExpr(int plcIndex);
  int  compileExpr(int plcIndex);
  int  setEnable(int plcIndex,
                 int enable);
  int  getEnable(int  plcIndex,
                 int *enabled);
  int  getCompiled(int  plcIndex,
                   int *compiled);
  int  getCompiled(int  plcIndex);

  int  readStaticPLCVar(int         plcIndex,
                        const char *varName,
                        double     *data);
  int  writeStaticPLCVar(int         plcIndex,
                         const char *varName,
                         double      data);
  int  validate();
  int  validate(int plcIndex);
  int  getErrorID();
  bool getError();
  void errorReset();
  ecmcPLCTask* getPLCTaskForAxis(int axisId);

 private:
  void initVars();
  int  createNewGlobalDataIF(char              *varName,
                             ecmcDataSourceType dataSource,
                             ecmcPLCDataIF    **outDataIF);
  int  createAndRegisterNewDataIF(int                plcIndex,
                                  char              *varName,
                                  ecmcDataSourceType dataSource);
  int  getAxisIndex(char *varName);
  int  getDsIndex(char *varName);
  int  addPLCDefaultVariables(int plcIndex,
                              int skipCycles);
  int addPLCDefaultVariable(int plcIndex, 
                            const char *suffix,
                            ecmcPLCDataIF **dataIFOut);
  int  addMainDefaultVariables();
  int  updateAllScanTimeVars();
  int  updateAllScanTimeVars(int plcIndex);
  int  parseAxis(int         plcIndex,
                 const char *exprStr);
  int  parseEC(int         plcIndex,
               const char *exprStr);
  int  parseStatic(int         plcIndex,
                   const char *exprStr);
  int  parseGlobal(int         plcIndex,
                   const char *exprStr);
  int  parsePLC(int         plcIndex,
                const char *exprStr);
  int  parseDataStorage(int         plcIndex,
                        const char *exprStr);
  int  parseFunctions(int         plcIndex,
                      const char *exprStr);
  int  findGlobalDataIF(char           *varName,
                        ecmcPLCDataIF **outDataIF);
  int  getPLCErrorID();
  int  plcVarNameValid(const char *plcVar);
  int globalVariableCount_;
  //Dedicateed plcs then one per axis
  ecmcPLCTask        *plcs_[ECMC_MAX_PLCS + ECMC_MAX_AXES];
  ecmcAxisBase       *axes_[ECMC_MAX_AXES];
  ecmcDataStorage    *ds_[ECMC_MAX_DATA_STORAGE_OBJECTS];
  ecmcEc             *ec_;
  ecmcAsynPortDriver *asynPortDriver_;
  ecmcPLCDataIF      *plcEnable_[ECMC_MAX_PLCS + ECMC_MAX_AXES];
  ecmcPLCDataIF      *plcError_[ECMC_MAX_PLCS + ECMC_MAX_AXES];
  ecmcPLCDataIF      *plcFirstScan_[ECMC_MAX_PLCS + ECMC_MAX_AXES];
  ecmcPLCDataIF      *globalDataArray_[ECMC_MAX_PLC_VARIABLES];
  ecmcPLCDataIF      *ecStatus_;
  double              mcuFreq_;
};

#endif  /* ECMC_PLC_MAIN_H_ */
