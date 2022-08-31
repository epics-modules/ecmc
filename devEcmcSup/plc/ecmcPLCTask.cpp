/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPLCTask.cpp
*
*  Created on: Oct 4, 2018
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcPLCTask.h"
#include "../main/ecmcErrorsList.h"
#include "ecmcPLCTask_libDs.inc"
#include "ecmcPLCTask_libEc.inc"
#include "ecmcPLCTask_libMc.inc"
#include "ecmcPLCTask_libFileIO.inc"

#define ecmcPLCTaskAddFunction(cmd, func) {          \
    errorCode = exprtk_->addFunction(cmd, func); \
    cmdCounter++;                                \
    if (errorCode) {                             \
      return errorCode;                          \
    }                                            \
}                                                \

ecmcPLCTask::ecmcPLCTask(int plcIndex, 
                         int skipCycles,
                         double mcuFreq,
                         ecmcAsynPortDriver *asynPortDriver) {
  initVars();
  plcIndex_          = plcIndex;
  skipCycles_        = skipCycles;
  asynPortDriver_    = asynPortDriver;
  exprtk_            = new exprtkWrap();
  mcuFreq_           = mcuFreq;
  plcScanTimeInSecs_ = 1 / mcuFreq_ * (skipCycles + 1);
  initAsyn(plcIndex);
}

ecmcPLCTask::~ecmcPLCTask() {
  for (int i = 0; i < ECMC_MAX_PLC_VARIABLES; i++) {
    delete localArray_[i];
    localArray_[i] = NULL;
  }
}

void ecmcPLCTask::initVars() {
  errorReset();
  plcIndex_            = 0;
  exprStr_             = "";
  exprStrRaw_          = "";
  compiled_            = false;
  globalVariableCount_ = 0;
  localVariableCount_  = 0;
  inStartup_           = 1;
  skipCycles_          = 0;
  skipCyclesCounter_   = 0;
  plcScanTimeInSecs_   = 0;  
  for (int i = 0; i < ECMC_MAX_PLC_VARIABLES; i++) {
    globalArray_[i]      = NULL;
    localArray_[i]       = NULL;
  }
  for (int i = 0; i < ECMC_MAX_PLUGINS; i++) {
    plugins_[i] = 0;
    libPluginsLoaded_[i] = 0;
  }
  firstScanDone_       = 0;
  libMcLoaded_         = 0;
  libEcLoaded_         = 0;
  libDsLoaded_         = 0;
  libFileIOLoaded_     = 0;
  asynPortDriver_      = 0;
  newExpr_             = 0;
  mcuFreq_             = MCU_FREQUENCY;
  asynParamExpr_       = NULL;
}

int ecmcPLCTask::addAndRegisterLocalVar(char *localVarStr) {
  // Already added?
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

  localArray_[localVariableCount_] = new ecmcPLCDataIF(plcIndex_,
                                                       plcScanTimeInSecs_*1000,
                                                       localVarStr,
                                                       ECMC_RECORDER_SOURCE_STATIC_VAR,
                                                       asynPortDriver_);
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
    localArray_[localVariableCount_] = NULL;
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  if (exprtk_->addVariable(localVarStr,
                           localArray_[localVariableCount_]->getDataRef())) {
    LOGERR("%s/%s:%d: Failed to add variable %s to exprtk  (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           localVarStr,
           ERROR_PLC_ADD_VARIABLE_FAIL);
    delete localArray_[localVariableCount_];
    localArray_[localVariableCount_] = NULL;
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_ADD_VARIABLE_FAIL);
  }

  localVariableCount_++;
  return 0;
}

int ecmcPLCTask::compile() {
  exprtk_->setExpression(exprStr_);
  std::vector<std::string> varList;

  if (exprtk_->compile()) {
    compiled_ = false;
    LOGERR("%s/%s:%d: Error: PLC%d compile error: %s.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           plcIndex_,
           exprtk_->getParserError().c_str());
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_PLC_COMPILE_ERROR);
  }

  compiled_ = true;
  newExpr_  = false;
  exprStrRaw_ = "";
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
      localArray_[i]->updateAsyn(0);
    }
  }

  for (int i = 0; i < globalVariableCount_; i++) {
    if (globalArray_[i]) {
      globalArray_[i]->write();
      // Update globals "centrally2 in ecmcPLCMain
      // to get asyn sample rate correct.
    }
  }

  firstScanDone_ = 1;

  return 0;
}

std::string * ecmcPLCTask::getExpr() {
  return &exprStr_;
}

std::string * ecmcPLCTask::getRawExpr() {
  return &exprStrRaw_;
}

int ecmcPLCTask::appendRawExpr(const char *exprStr) {  
  try {
    exprStrRaw_ += exprStr;
    exprStrRaw_ += "\n";  //Add Enter
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
  newExpr_ = true;
  return 0;
}

int ecmcPLCTask::addExprLine(const char *exprStr) {
  try {
    exprStr_ += exprStr;
    updateAsyn();
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

int ecmcPLCTask::clearRawExpr() {
  exprStrRaw_  = "";
  return 0;
}

int ecmcPLCTask::clearExpr() {
  exprStr_  = "";
  updateAsyn();
  compiled_ = false;

  for (int i = 0; i < localVariableCount_; i++) {
    if (localArray_[i]) {
      delete localArray_[i];
      localArray_[i] = NULL;
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
  if(exprStr_.length()==0) {
    return 0;  // Not used.. return OK
  }

  int errorCode = compile();

  if (!compiled_ || errorCode) {
    LOGERR(
      "%s/%s:%d: Error: Validation of PLC object failed (index %d): Not compiled (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      plcIndex_,
      errorCode);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_PLC_COMPILE_ERROR);
  }

  if (getErrorID()) {
    return getErrorID();
  }

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
             ERROR_PLC_ADD_VARIABLE_FAIL);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_PLC_ADD_VARIABLE_FAIL);
    }

    globalArray_[globalVariableCount_] = dataIF;
    globalVariableCount_++;
  }
  return 0;
}

double ecmcPLCTask::getSampleTime() {
  return 1 / mcuFreq_ * (skipCycles_ + 1);
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

int ecmcPLCTask::setEcPointer(ecmcEc *ec) {
  ecmcPLCTask::statEc_ = ec;
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

  for(int i = 0; i < ECMC_MAX_PLUGINS; ++i) {
    if(!libPluginsLoaded_[i]) {
      if (findPluginFunction(plugins_[i] ,exprStr) || 
          findPluginConstant(plugins_[i] ,exprStr) ) {
        errorCode = loadPluginLib(plugins_[i]);
        if (errorCode) {
          return errorCode;
        }
        libPluginsLoaded_[i]=1;
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

bool ecmcPLCTask::findPluginFunction(ecmcPluginLib* plugin, const char *exprStr){

 if(!plugin){
    return 0;
  }
  ecmcPluginData *data = plugin->getData(); 
  if(data == NULL){
    return 0;
  }
  // Loop funcs[]
  for(int i = 0; i < ECMC_PLUGIN_MAX_PLC_FUNC_COUNT; ++i){
    int argCount = plugin->findArgCount(data->funcs[i]);
    if(!data->funcs[i].funcName || 
        strlen(data->funcs[i].funcName) == 0 ||
        ((argCount < 0 || 
        argCount > ECMC_PLUGIN_MAX_PLC_ARG_COUNT) &&
        data->funcs[i].funcGenericObj==NULL)){
      break;
    }

    if (strstr(exprStr, data->funcs[i].funcName)) {
      return true;
    }
  }

  return false;
}

bool ecmcPLCTask::findPluginConstant(ecmcPluginLib* plugin, const char *exprStr){

 if(!plugin){
    return 0;
  }
  ecmcPluginData *data = plugin->getData(); 
  if(data == NULL){
    return 0;
  }
  // Loop consts[]
  for(int i = 0; i < ECMC_PLUGIN_MAX_PLC_FUNC_COUNT; ++i){

    if(!data->consts[i].constName || 
        strlen(data->consts[i].constName) == 0){
      break;
    }

    if (strstr(exprStr, data->consts[i].constName)) {
      return true;
    }
  }

  return false;
}

int ecmcPLCTask::loadPluginLib(ecmcPluginLib* plugin){
  int errorCode = 0;
  int cmdCounter = 0;

  if(!plugin){
    return 0;
  }
  ecmcPluginData *data = plugin->getData(); 
  if(data == NULL){
    return 0;
  }

  for(int i = 0; i < ECMC_PLUGIN_MAX_PLC_FUNC_COUNT; ++i){
    int argCount = plugin->findArgCount(data->funcs[i]);
    if(!data->funcs[i].funcName || 
        strlen(data->funcs[i].funcName) == 0 ||
        ((argCount < 0 || 
        argCount > ECMC_PLUGIN_MAX_PLC_ARG_COUNT) &&
        data->funcs[i].funcGenericObj==NULL)){
      break;
    }

    if(data->funcs[i].funcGenericObj && strlen(data->funcs[i].funcName) > 0) {      
      // load generic_function_t generic func object (allow strings)    
      ecmcPLCTaskAddFunction(data->funcs[i].funcName,data->funcs[i].funcGenericObj);
    }
    else {
      switch(argCount) {
        case 0:
          ecmcPLCTaskAddFunction(data->funcs[i].funcName,data->funcs[i].funcArg0);
          break;
        case 1:
          ecmcPLCTaskAddFunction(data->funcs[i].funcName,data->funcs[i].funcArg1);
          break;
        case 2:
          ecmcPLCTaskAddFunction(data->funcs[i].funcName,data->funcs[i].funcArg2);
          break;
        case 3:
          ecmcPLCTaskAddFunction(data->funcs[i].funcName,data->funcs[i].funcArg3);
          break;
        case 4:
          ecmcPLCTaskAddFunction(data->funcs[i].funcName,data->funcs[i].funcArg4);
          break;
        case 5:
          ecmcPLCTaskAddFunction(data->funcs[i].funcName,data->funcs[i].funcArg5);
          break;
        case 6:
          ecmcPLCTaskAddFunction(data->funcs[i].funcName,data->funcs[i].funcArg6);
          break;
        case 7:
          ecmcPLCTaskAddFunction(data->funcs[i].funcName,data->funcs[i].funcArg7);
          break;
        case 8:
          ecmcPLCTaskAddFunction(data->funcs[i].funcName,data->funcs[i].funcArg8);
          break;
        case 9:
          ecmcPLCTaskAddFunction(data->funcs[i].funcName,data->funcs[i].funcArg9);
          break;
        case 10:
          ecmcPLCTaskAddFunction(data->funcs[i].funcName,data->funcs[i].funcArg10);
          break;
        default:
          break;
      }
    } 
  }

  // Load constants
  for(int i=0;i<ECMC_PLUGIN_MAX_PLC_CONST_COUNT;++i){
    if(!data->consts[i].constName || 
        strlen(data->consts[i].constName) == 0){
      break;
    }
    errorCode = exprtk_->addConstant(data->consts[i].constName,data->consts[i].constValue);
    if (errorCode) {
      return errorCode;
    }
  }

  return 0;
}

int ecmcPLCTask::loadEcLib() {
  int errorCode  = 0;
  int cmdCounter = 0;

  ecmcPLCTaskAddFunction("ec_set_bit",     ec_set_bit);
  ecmcPLCTaskAddFunction("ec_clr_bit",     ec_clr_bit);
  ecmcPLCTaskAddFunction("ec_flp_bit",     ec_flp_bit);
  ecmcPLCTaskAddFunction("ec_chk_bit",     ec_chk_bit);
  ecmcPLCTaskAddFunction("ec_print_hex",   ec_print_hex);
  ecmcPLCTaskAddFunction("ec_print_bin",   ec_print_bin);
  ecmcPLCTaskAddFunction("ec_get_err",     ec_get_err);
  ecmcPLCTaskAddFunction("ec_wrt_bit",     ec_wrt_bit);
  ecmcPLCTaskAddFunction("ec_mm_cp",       ec_mm_cp);
  ecmcPLCTaskAddFunction("ec_err_rst",     ec_err_rst);
  ecmcPLCTaskAddFunction("ec_wrt_bits",    ec_wrt_bits);
  ecmcPLCTaskAddFunction("ec_chk_bits",    ec_chk_bits);
  ecmcPLCTaskAddFunction("ec_get_time",    ec_get_time);
  ecmcPLCTaskAddFunction("ec_get_mm_type", ec_get_mm_type);
  ecmcPLCTaskAddFunction("ec_get_mm_data", ec_get_mm_data);
  ecmcPLCTaskAddFunction("ec_set_mm_data", ec_set_mm_data);
  ecmcPLCTaskAddFunction("ec_get_mm_size", ec_get_mm_size);
  ecmcPLCTaskAddFunction("ec_get_time_l32", ec_get_time_l32);
  ecmcPLCTaskAddFunction("ec_get_time_u32", ec_get_time_u32);
  ecmcPLCTaskAddFunction("ec_mm_append_to_ds", ec_mm_append_to_ds);
  ecmcPLCTaskAddFunction("ec_mm_append_to_ds_scale_offset", ec_mm_append_to_ds_scale_offset);
  ecmcPLCTaskAddFunction("ec_mm_push_asyn", ec_mm_push_asyn);
  
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
  ecmcPLCTaskAddFunction("mc_set_enable_motion_funcs", mc_set_enable_motion_funcs);
  ecmcPLCTaskAddFunction("mc_move_ext_pos", mc_move_ext_pos);
  ecmcPLCTaskAddFunction("mc_home_pos",     mc_home_pos);
  ecmcPLCTaskAddFunction("mc_get_act_pos",  mc_get_act_pos);
  
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
  ecmcPLCTaskAddFunction("ds_push_asyn",   ds_push_asyn);
  ecmcPLCTaskAddFunction("ds_get_avg",     ds_get_avg);
  ecmcPLCTaskAddFunction("ds_get_min",     ds_get_min);
  ecmcPLCTaskAddFunction("ds_get_max",     ds_get_max);
  ecmcPLCTaskAddFunction("ds_err_rst",     ds_err_rst);
  ecmcPLCTaskAddFunction("ds_append_to_ds",ds_append_to_ds);
  
  
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

//Check if new expression is loaded after compile
int ecmcPLCTask::getNewExpr() {
  return newExpr_;
}

int ecmcPLCTask::initAsyn(int plcIndex) {
  
  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];  
  char *name = buffer;
  ecmcAsynDataItem *paramTemp=NULL;
  int chars = 0;

  // ECMC_PLC_EXPR_STR  
   if(plcIndex < ECMC_MAX_PLCS){
    chars = snprintf(name,
                     EC_MAX_OBJECT_PATH_CHAR_LENGTH - 1,
                     ECMC_PLC_DATA_STR "%d." ECMC_PLC_EXPR_STR,
                     plcIndex);

    if (chars >= EC_MAX_OBJECT_PATH_CHAR_LENGTH - 1) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_PLC_VARIABLE_NAME_TO_LONG);
    }
  }
  else {  // Axis PLC
    chars = snprintf(name,
                     EC_MAX_OBJECT_PATH_CHAR_LENGTH - 1,
                     ECMC_AX_STR "%d." ECMC_PLC_DATA_STR "." ECMC_PLC_EXPR_STR,
                     plcIndex-ECMC_MAX_PLCS);

    if (chars >= EC_MAX_OBJECT_PATH_CHAR_LENGTH - 1) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_PLC_VARIABLE_NAME_TO_LONG);
    }
  }

  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt8Array,
                                         (uint8_t *)exprStr_.c_str(),
                                         strlen(exprStr_.c_str()),
                                         ECMC_EC_S8,
                                         plcScanTimeInSecs_*1000, //milliseconds
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }  
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->setArrayCheckSize(false);
  paramTemp->refreshParam(1,(uint8_t*)exprStr_.c_str(),strlen(exprStr_.c_str()));
  
  asynParamExpr_ = paramTemp;  
  return 0;
}

void ecmcPLCTask::updateAsyn() {
  asynParamExpr_->refreshParam(1,(uint8_t*)exprStr_.c_str(),strlen(exprStr_.c_str()));
}

int ecmcPLCTask::setPluginPointer(ecmcPluginLib *plugin, int index) {
  
  if(index < 0 && index >= ECMC_MAX_PLUGINS) {
    return ERROR_PLC_PLUGIN_INDEX_OUT_OF_RANGE;
  }

  plugins_[index] = plugin;
  return 0;
}
