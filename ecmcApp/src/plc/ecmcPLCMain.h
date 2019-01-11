/*
 * ecmcPLCMain.h
 *
 *  Created on: Oct 15, 2018
 *      Author: anderssandstrom
 */

#ifndef ECMC_PLC_MAIN_H_
#define ECMC_PLC_MAIN_H_

#include "exprtkWrap.h"
#include <iostream>
#include <fstream>
#include <string>
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

#define CHECK_PLC_RETURN_IF_ERROR(index) {        \
    if (index >= ECMC_MAX_PLCS || index < 0) {    \
      LOGERR("ERROR: PLC index out of range.\n"); \
      return ERROR_PLCS_INDEX_OUT_OF_RANGE;       \
    }                                             \
    if (plcs_[index] == NULL) {                   \
      LOGERR("ERROR: PLC index NULL.\n");         \
      return ERROR_PLCS_PLC_NULL;                 \
    }                                             \
}                                                 \

class ecmcPLCMain : public ecmcError {
 public:
  explicit ecmcPLCMain(ecmcEc *ec);
  ~ecmcPLCMain();
  int  createPLC(int plcIndex,
                 int skipCycles);
  int  deletePLC(int plcIndex);
  int  setAxisArrayPointer(ecmcAxisBase *axis,
                           int           index);
  int  setDataStoragePointer(ecmcDataStorage *ds,
                             int              index);
  int  execute(bool ecOK);
  int  setExpr(int   plcIndex,
               char *expr);
  int  parseExpr(int         plcIndex,
                 const char *exprStr);
  int  getExpr(int          plcIndex,
               std::string *expr);
  int  addExprLine(int         plcIndex,
                   const char *exprStr);
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
  int  readStaticPLCVar(int         plcIndex,
                        const char *varName,
                        double     *data);
  int  writeStaticPLCVar(int         plcIndex,
                         const char *varName,
                         double      data);
  int  validate();
  int  getErrorID();
  bool getError();
  void errorReset();

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
  int  updateAllScanTimeVars();
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
  ecmcPLCTask *plcs_[ECMC_MAX_PLCS];
  ecmcAxisBase *axes_[ECMC_MAX_AXES];
  ecmcDataStorage *ds_[ECMC_MAX_DATA_STORAGE_OBJECTS];
  ecmcEc *ec_;
  ecmcPLCDataIF *plcEnable_[ECMC_MAX_PLCS];
  ecmcPLCDataIF *plcError_[ECMC_MAX_PLCS];
  ecmcPLCDataIF *plcFirstScan_[ECMC_MAX_PLCS];
  ecmcPLCDataIF *globalDataArray_[ECMC_MAX_PLC_VARIABLES];
};

#endif  /* ECMC_PLC_MAIN_H_ */
