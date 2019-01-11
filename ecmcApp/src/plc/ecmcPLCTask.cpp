/*
 *
 *  Created on: Oct 4, 2018
 *      Author: anderssandstrom
 */

#include "ecmcPLCTask.h"

#include "ecmcPLCTask_libEc.inc"
#include "ecmcPLCTask_libMc.inc"
#include "ecmcPLCTask_libDs.inc"
#include "ecmcPLCTask_libFileIO.inc"

#define ecmcPLCTaskAddFunction(cmd, func) {          \
    errorCode = exprtk_->addFunction(cmd, func); \
    cmdCounter++;                                \
    if (errorCode) {                             \
      return errorCode;                          \
    }                                            \
}                                                \

ecmcPLCTask::ecmcPLCTask(int skipCycles) {
  initVars();
  skipCycles_        = skipCycles;
  exprtk_            = new exprtkWrap();
  plcScanTimeInSecs_ = 1 / MCU_FREQUENCY * (skipCycles + 1);
}

ecmcPLCTask::~ecmcPLCTask() {
  for (int i = 0; i < ECMC_MAX_PLC_VARIABLES; i++) {
    delete localArray_[i];
  }
}

void ecmcPLCTask::initVars() {
  errorReset();
  exprStr_             = "";
  compiled_            = false;
  globalVariableCount_ = 0;
  localVariableCount_  = 0;
  inStartup_           = 1;
  skipCycles_          = 0;
  skipCyclesCounter_   = 0;
  plcScanTimeInSecs_   = 0;

  for (int i = 0; i < ECMC_MAX_PLC_VARIABLES; i++) {
    globalArray_[i] = NULL;
    localArray_[i]  = NULL;
  }
  firstScanDone_   = 0;
  libMcLoaded_     = 0;
  libEcLoaded_     = 0;
  libDsLoaded_     = 0;
  libFileIOLoaded_ = 0;
}

int ecmcPLCTask::addAndRegisterLocalVar(char *localVarStr) {
  // Already added?addAndReisterGlobalVar
  if (localVarExist(localVarStr)) {
    return 0;
  }

  if ((localVariableCount_ >= ECMC_MAX_PLC_VARIABLES - 1) ||
      (localVariableCount_ < 0)) {
    LOGERR(
      "%s/%s:%d: PLC local variable count excedded. Adding %s failed (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      localVarStr,
      ERROR_PLC_VARIABLE_COUNT_EXCEEDED);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_VARIABLE_COUNT_EXCEEDED);
  }

  localArray_[localVariableCount_] = new ecmcPLCDataIF(localVarStr,
                                                       ECMC_RECORDER_SOURCE_STATIC_VAR);
  int errorCode = localArray_[localVariableCount_]->getErrorID();

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: PLC local variable: Create data interface failed. Adding %s failed (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      localVarStr,
      errorCode);
    delete localArray_[localVariableCount_];
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  if (exprtk_->addVariable(localVarStr,
                           localArray_[localVariableCount_]->getDataRef())) {
    LOGERR("%s/%s:%d: Failed to add variable %s to exprtk  (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           localVarStr,
           ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
    delete localArray_[localVariableCount_];
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
  }

  localVariableCount_++;
  return 0;
}

int ecmcPLCTask::compile() {
  exprtk_->setExpression(exprStr_);
  std::vector<std::string> varList;

  if (exprtk_->compile()) {
    compiled_ = false;
    LOGERR("%s/%s:%d: Error: Transform compile error: %s.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           exprtk_->getParserError().c_str());
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_PLC_COMPILE_ERROR);
  }

  compiled_ = true;
  return 0;
}

bool ecmcPLCTask::getCompiled() {
  return compiled_;
}

int ecmcPLCTask::execute(bool ecOK) {
  if (!compiled_ || (skipCyclesCounter_ < skipCycles_)) {
    skipCyclesCounter_++;
    return 0;
  }
  skipCyclesCounter_ = 0;

  if (ecOK) {
    inStartup_ = 0;
  }

  // Wait for EC OK
  if (!ecOK && inStartup_) {
    return 0;
  }

  if (!exprtk_) {
    return 0;
  }

  for (int i = 0; i < localVariableCount_; i++) {
    if (localArray_[i]) {
      localArray_[i]->read();
    }
  }

  for (int i = 0; i < globalVariableCount_; i++) {
    if (globalArray_[i]) {
      globalArray_[i]->read();
    }
  }

  // Run equation
  exprtk_->refresh();

  for (int i = 0; i < localVariableCount_; i++) {
    if (localArray_[i]) {
      localArray_[i]->write();
    }
  }

  for (int i = 0; i < globalVariableCount_; i++) {
    if (globalArray_[i]) {
      globalArray_[i]->write();
    }
  }

  firstScanDone_ = 1;

  return 0;
}

std::string * ecmcPLCTask::getExpr() {
  return &exprStr_;
}

int ecmcPLCTask::addExprLine(char *exprStr) {
  try {
    exprStr_ += exprStr;
  }
  catch (const std::exception& e) {
    LOGERR("%s/%s:%d: Append of expression line failed: %s (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           e.what(),
           ERROR_PLC_ADD_EXPR_LINE_ERROR);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_ADD_EXPR_LINE_ERROR);
  }
  compiled_ = false;

  return 0;
}

int ecmcPLCTask::clearExpr() {
  exprStr_  = "";
  compiled_ = false;

  for (int i = 0; i < localVariableCount_; i++) {
    if (localArray_[i]) {
      delete localArray_[i];
    }
  }
  localVariableCount_ = 0;
  return 0;
}

int ecmcPLCTask::localVarExist(const char *varName) {
  for (int i = 0; i < localVariableCount_; i++) {
    if (localArray_[i]) {
      int n = strcmp(varName, localArray_[i]->getVarName());

      if (n == 0) {
        return 1;
      }
    }
  }
  return 0;
}

int ecmcPLCTask::globalVarExist(const char *varName) {
  for (int i = 0; i < globalVariableCount_; i++) {
    if (globalArray_[i]) {
      int n = strcmp(varName, globalArray_[i]->getVarName());

      if (n == 0) {
        return 1;
      }
    }
  }
  return 0;
}

int ecmcPLCTask::validate() {
  if (!compiled_) {
    LOGERR(
      "%s/%s:%d: Error: Validation of PLC object failed: Not compiled (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_PLC_COMPILE_ERROR);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_PLC_COMPILE_ERROR);
  }

  if (getErrorID()) {
    return getErrorID();
  }

  int errorCode = 0;

  // Check global variables
  for (int i = 0; i < globalVariableCount_; i++) {
    if (globalArray_[i]) {
      errorCode = globalArray_[i]->validate();

      if (errorCode) {
        LOGERR(
          "%s/%s:%d: Error: Validation of Global PLCDataIF  %s at index %d  failed (0x%x).\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          globalArray_[i]->getVarName(),
          i,
          errorCode);
        return errorCode;
      }
    } else {
      LOGERR(
        "%s/%s:%d: Error: Validation of Global PLCDataIF failed. PLCDataIf NULL (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_PLC_PLC_DATA_IF_NULL);
      return ERROR_PLC_PLC_DATA_IF_NULL;
    }
  }

  // Check local variables
  for (int i = 0; i < localVariableCount_; i++) {
    if (localArray_[i]) {
      errorCode = localArray_[i]->validate();

      if (errorCode) {
        LOGERR(
          "%s/%s:%d: Error: Validation of Global PLCDataIF  %s at index %d  failed (0x%x).\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          localArray_[i]->getVarName(),
          i,
          errorCode);
        return errorCode;
      }
    } else {
      LOGERR(
        "%s/%s:%d: Error: Validation of Local PLCDataIF failed. PLCDataIf NULL (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_PLC_PLC_DATA_IF_NULL);
      return ERROR_PLC_PLC_DATA_IF_NULL;
    }
  }

  return 0;
}

int ecmcPLCTask::addAndReisterGlobalVar(ecmcPLCDataIF *dataIF) {
  if (!dataIF) {
    LOGERR("%s/%s:%d: Data IF NULL  (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_PLC_PLC_DATA_IF_NULL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_PLC_DATA_IF_NULL);
  }

  // Check if already added
  if (!globalVarExist(dataIF->getVarName())) {
    if ((globalVariableCount_ >= ECMC_MAX_PLC_VARIABLES - 1) ||
        (globalVariableCount_ < 0)) {
      LOGERR(
        "%s/%s:%d: PLC global variable count excedded. Adding %s failed (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        dataIF->getVarName(),
        ERROR_PLC_VARIABLE_COUNT_EXCEEDED);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_PLC_VARIABLE_COUNT_EXCEEDED);
    }

    if (exprtk_->addVariable(dataIF->getExprTkVarName(),
                             dataIF->getDataRef())) {
      LOGERR("%s/%s:%d: Failed to add variable %s to exprtk  (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             dataIF->getVarName(),
             ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
    }

    globalArray_[globalVariableCount_] = dataIF;
    globalVariableCount_++;
  }
  return 0;
}

double ecmcPLCTask::getSampleTime() {
  return 1 / MCU_FREQUENCY * (skipCycles_ + 1);
}

int ecmcPLCTask::getFirstScanDone() {
  return firstScanDone_;
}

int ecmcPLCTask::setAxisArrayPointer(ecmcAxisBase *axis, int index) {
  if ((index >= ECMC_MAX_AXES) || (index < 0)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_AXIS_ID_OUT_OF_RANGE);
  }
  ecmcPLCTask::statAxes_[index] = axis;
  return 0;
}

int ecmcPLCTask::setDataStoragePointer(ecmcDataStorage *ds, int index) {
  if ((index >= ECMC_MAX_DATA_STORAGE_OBJECTS) || (index < 0)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_AXIS_ID_OUT_OF_RANGE);
  }
  ecmcPLCTask::statDs_[index] = ds;
  return 0;
}

int ecmcPLCTask::parseFunctions(const char *exprStr) {
  // look for Ec function
  int errorCode = 0;

  if (!libEcLoaded_) {
    if (findEcFunction(exprStr)) {
      errorCode = loadEcLib();

      if (errorCode) {
        return errorCode;
      }
    }
  }

  // look for Ds function
  if (!libDsLoaded_) {
    if (findDsFunction(exprStr)) {
      errorCode = loadDsLib();

      if (errorCode) {
        return errorCode;
      }
    }
  }

  // look for Mc function
  if (!libMcLoaded_) {
    if (findMcFunction(exprStr)) {
      errorCode = loadMcLib();

      if (errorCode) {
        return errorCode;
      }
    }
  }

  // look for File IO function
  if (!libFileIOLoaded_) {
    if (findFileIOFunction(exprStr)) {
      errorCode = loadFileIOLib();

      if (errorCode) {
        return errorCode;
      }
    }
  }

  return 0;
}

bool ecmcPLCTask::findEcFunction(const char *exprStr) {
  for (int i = 0; i < ec_cmd_count; i++) {
    if (strstr(exprStr, ecLibCmdList[i])) {
      return true;
    }
  }
  return false;
}

bool ecmcPLCTask::findMcFunction(const char *exprStr) {
  for (int i = 0; i < mc_cmd_count; i++) {
    if (strstr(exprStr, mcLibCmdList[i])) {
      return true;
    }
  }
  return false;
}

bool ecmcPLCTask::findDsFunction(const char *exprStr) {
  for (int i = 0; i < ds_cmd_count; i++) {
    if (strstr(exprStr, dsLibCmdList[i])) {
      return true;
    }
  }
  return false;
}

bool ecmcPLCTask::findFileIOFunction(const char *exprStr) {
  for (int i = 0; i < fileIO_cmd_count; i++) {
    if (strstr(exprStr, fileIOLibCmdList[i])) {
      return true;
    }
  }
  return false;
}

int ecmcPLCTask::loadEcLib() {
  int errorCode  = 0;
  int cmdCounter = 0;

  ecmcPLCTaskAddFunction("ec_set_bit",   ec_set_bit);
  ecmcPLCTaskAddFunction("ec_clr_bit",   ec_clr_bit);
  ecmcPLCTaskAddFunction("ec_flp_bit",   ec_flp_bit);
  ecmcPLCTaskAddFunction("ec_chk_bit",   ec_chk_bit);
  ecmcPLCTaskAddFunction("ec_print_hex", ec_print_hex);
  ecmcPLCTaskAddFunction("ec_print_bin", ec_print_bin);
  ecmcPLCTaskAddFunction("ec_get_err",   ec_get_err);

  if (ec_cmd_count != cmdCounter) {
    LOGERR("%s/%s:%d: PLC Lib EC command count missmatch (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_PLC_LIB_CMD_COUNT_MISS_MATCH);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_LIB_CMD_COUNT_MISS_MATCH);
  }
  libEcLoaded_ = 1;
  return 0;
}

int ecmcPLCTask::loadMcLib() {
  int errorCode  = 0;
  int cmdCounter = 0;

  ecmcPLCTaskAddFunction("mc_move_abs",     mc_move_abs);
  ecmcPLCTaskAddFunction("mc_move_rel",     mc_move_rel);
  ecmcPLCTaskAddFunction("mc_move_vel",     mc_move_vel);
  ecmcPLCTaskAddFunction("mc_home",         mc_home);
  ecmcPLCTaskAddFunction("mc_halt",         mc_halt);
  ecmcPLCTaskAddFunction("mc_power",        mc_power);
  ecmcPLCTaskAddFunction("mc_get_err",      ec_get_err);
  ecmcPLCTaskAddFunction("mc_reset",        mc_reset);
  ecmcPLCTaskAddFunction("mc_get_busy",     mc_get_busy);
  ecmcPLCTaskAddFunction("mc_get_homed",    mc_get_homed);
  ecmcPLCTaskAddFunction("mc_get_axis_err", mc_get_axis_err);

  if (mc_cmd_count != cmdCounter) {
    LOGERR("%s/%s:%d: PLC Lib MC command count missmatch (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_PLC_LIB_CMD_COUNT_MISS_MATCH);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_LIB_CMD_COUNT_MISS_MATCH);
  }
  libMcLoaded_ = 1;
  return 0;
}

int ecmcPLCTask::loadDsLib() {
  int errorCode  = 0;
  int cmdCounter = 0;

  ecmcPLCTaskAddFunction("ds_append_data", ds_append_data);
  ecmcPLCTaskAddFunction("ds_clear_data",  ds_clear_data);
  ecmcPLCTaskAddFunction("ds_get_data",    ds_get_data);
  ecmcPLCTaskAddFunction("ds_set_data",    ds_set_data);
  ecmcPLCTaskAddFunction("ds_get_buff_id", ds_get_buff_id);
  ecmcPLCTaskAddFunction("ds_set_buff_id", ds_set_buff_id);
  ecmcPLCTaskAddFunction("ds_get_err",     ds_get_err);
  ecmcPLCTaskAddFunction("ds_is_full",     ds_is_full);
  ecmcPLCTaskAddFunction("ds_get_size",    ds_get_size);

  if (ds_cmd_count != cmdCounter) {
    LOGERR("%s/%s:%d: PLC Lib DS command count missmatch (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_PLC_LIB_CMD_COUNT_MISS_MATCH);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_LIB_CMD_COUNT_MISS_MATCH);
  }
  libDsLoaded_ = 1;
  return 0;
}

int ecmcPLCTask::loadFileIOLib() {
  int errorCode = exprtk_->addFileIO();

  if (errorCode) {
    return errorCode;
  }
  libFileIOLoaded_ = 1;
  return errorCode;
}

int ecmcPLCTask::readStaticPLCVar(const char *varName, double *data) {
  ecmcPLCDataIF *dataIF = NULL;
  int errorCode         = findLocalVar(varName, &dataIF);

  if (errorCode || (dataIF == NULL)) {
    LOGERR("%s/%s:%d: PLC static variable %s not found  (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           varName,
           errorCode);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }
  *data = dataIF->getData();
  return 0;
}

int ecmcPLCTask::writeStaticPLCVar(const char *varName, double data) {
  ecmcPLCDataIF *dataIF = NULL;
  int errorCode         = findLocalVar(varName, &dataIF);

  if (errorCode || (dataIF == NULL)) {
    LOGERR("%s/%s:%d: PLC static variable %s not found  (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           varName,
           errorCode);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }
  dataIF->setData(data);
  return 0;
}

int ecmcPLCTask::findLocalVar(const char *varName, ecmcPLCDataIF **outDataIF) {
  for (int i = 0; i < localVariableCount_; i++) {
    if (localArray_[i]) {
      int n = strcmp(varName, localArray_[i]->getVarName());

      if (n == 0) {
        *outDataIF = localArray_[i];
        return 0;
      }
    }
  }
  return ERROR_PLC_VARIABLE_NOT_FOUND;
}
