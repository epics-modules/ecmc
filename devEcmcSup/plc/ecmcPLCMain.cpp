/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcPLCMain.cpp
*
*  Created on: Oct 15, 2018
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcPLCMain.h"

ecmcPLCMain::ecmcPLCMain(ecmcEc             *ec,
                         double              mcuFreq,
                         ecmcAsynPortDriver *asynPortDriver) {
  initVars();
  asynPortDriver_ = asynPortDriver;
  ec_             = ec;
  mcuFreq_        = mcuFreq;
  addMainDefaultVariables();
}

ecmcPLCMain::~ecmcPLCMain() {
  for (int i = 0; i < ECMC_MAX_PLCS + ECMC_MAX_AXES; i++) {
    delete plcs_[i];
    plcs_[i] = NULL;
  }
}

void ecmcPLCMain::initVars() {
  globalVariableCount_ = 0;

  for (int i = 0; i < ECMC_MAX_PLCS + ECMC_MAX_AXES; i++) {
    plcs_[i]         = NULL;
    plcEnable_[i]    = NULL;
    plcError_[i]     = NULL;
    plcFirstScan_[i] = NULL;
  }

  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    axes_[i] = NULL;
  }

  for (int i = 0; i < ECMC_MAX_DATA_STORAGE_OBJECTS; i++) {
    ds_[i] = NULL;
  }

  for (int i = 0; i < ECMC_MAX_PLC_VARIABLES; i++) {
    globalDataArray_[i] = 0;
  }

  for (int i = 0; i < ECMC_MAX_PLUGINS; i++) {
    plugins_[i] = 0;
  }

  asynPortDriver_ = NULL;
  ec_             = NULL;
  memset(&shm_, 0, sizeof(ecmcShm));
  ecStatus_ = NULL;
  mcuFreq_  = MCU_FREQUENCY;
  inVarSection_ = false;
}

int ecmcPLCMain::createPLC(int plcIndex, int skipCycles) {
  if ((plcIndex < 0) || (plcIndex >= ECMC_MAX_PLCS + ECMC_MAX_AXES)) {
    return ERROR_PLCS_INDEX_OUT_OF_RANGE;
  }

  if (!ec_) return ERROR_PLCS_EC_NOT_INITIALIZED;

  if (plcs_[plcIndex]) {
    return ERROR_PLCS_PLC_INDEX_ALREADY_CREATED;
  }

  plcs_[plcIndex] = new ecmcPLCTask(plcIndex,
                                    skipCycles,
                                    mcuFreq_,
                                    asynPortDriver_);

  int errorCode = plcs_[plcIndex]->getErrorID();

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  // Set plugin pointers
  for (int i = 0; i < ECMC_MAX_PLUGINS; i++) {
    plcs_[plcIndex]->setPluginPointer(plugins_[i], i);
  }

  errorCode = addPLCDefaultVariables(plcIndex, skipCycles);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  return 0;
}

int ecmcPLCMain::setAxisGroupArrayPointer(ecmcAxisGroup *grp, int index) {
  if ((index >= ECMC_MAX_AXES) || (index < 0)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLCS_AXIS_INDEX_OUT_OF_RANGE);
  }
  axisGrp_[index] = grp;
  return 0;
}

int ecmcPLCMain::setAxisArrayPointer(ecmcAxisBase *axis, int index) {
  if ((index >= ECMC_MAX_AXES) || (index < 0)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLCS_AXIS_INDEX_OUT_OF_RANGE);
  }
  axes_[index] = axis;
  return 0;
}

int ecmcPLCMain::setDataStoragePointer(ecmcDataStorage *ds, int index) {
  if ((index >= ECMC_MAX_DATA_STORAGE_OBJECTS) || (index < 0)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLCS_AXIS_INDEX_OUT_OF_RANGE);
  }
  ds_[index] = ds;
  return 0;
}

int ecmcPLCMain::validate(int plcIndex) {
  if (plcs_[plcIndex] != NULL) {
    int errorCode = plcs_[plcIndex]->validate();

    if (errorCode) {
      return errorCode;
    }
  }
  return 0;
}

int ecmcPLCMain::validate() {
  // Parse and Compile all PLCs before runtime (all objects should be availabe)
  // Compile Axis PLC:S if needed

  int errorCode = 0;

  for (int i = 0; i < ECMC_MAX_PLCS + ECMC_MAX_AXES; i++) {
    if (plcs_[i]) {
      if (!getCompiled(i)) {
        errorCode = addExprLine(i, plcs_[i]->getRawExpr()->c_str());

        if (errorCode) {
          return errorCode;
        }
      }
    }
  }

  // Set scantime
  errorCode = updateAllScanTimeVars();

  if (errorCode) {
    return errorCode;
  }

  // Validate
  for (int i = 0; i < ECMC_MAX_PLCS + ECMC_MAX_AXES; i++) {
    int errorCode = validate(i);

    if (errorCode) {
      return errorCode;
    }
  }

  return 0;
}

int ecmcPLCMain::execute(bool ecOK) {
  // refresh ec<id>.masterstatus
  if (ecStatus_) {
    ecStatus_->setData((double)ecOK);
  }

  // ONLY EXECUTE NORMAL PLCS (AXIS PLCs are executed from main thread)
  for (int plcIndex = 0; plcIndex < ECMC_MAX_PLCS; plcIndex++) {
    if (plcs_[plcIndex] != NULL) {
      if (plcEnable_[plcIndex]) {
        if (plcEnable_[plcIndex]->getData()) {
          plcs_[plcIndex]->execute(ecOK);

          if (ecOK) {
            if (plcFirstScan_[plcIndex]) {
              plcFirstScan_[plcIndex]->setData(
                plcs_[plcIndex]->getFirstScanDone() == 0);                              // First scan
            }
          }
        }
      }
    }
  }

  /** update asyn params here for all globals to get sample rate correct
      (if globals are used in many plcs) */
  for (int i = 0; i < globalVariableCount_; ++i) {
    if (globalDataArray_[i]) {
      globalDataArray_[i]->updateAsyn(0);
    }
  }

  return 0;
}

int ecmcPLCMain::execute(int plcIndex, bool ecOK) {
  if (plcs_[plcIndex] != NULL) {
    if (plcEnable_[plcIndex]) {
      if (plcEnable_[plcIndex]->getData()) {
        plcs_[plcIndex]->execute(ecOK);

        if (ecOK) {
          if (plcFirstScan_[plcIndex]) {
            plcFirstScan_[plcIndex]->setData(
              plcs_[plcIndex]->getFirstScanDone() == 0);                              // First scan
          }
        }
      }
    }
  }
  return 0;
}

std::string * ecmcPLCMain::getExpr(int plcIndex, int *error) {
  if ((plcIndex >= ECMC_MAX_PLCS + ECMC_MAX_AXES) || (plcIndex < 0)) {
    LOGERR("ERROR: PLC index out of range.\n");
    *error =  ERROR_PLCS_INDEX_OUT_OF_RANGE;
    return NULL;
  }

  if (!plcs_[plcIndex]) {
    LOGERR("ERROR: PLC NULL\n");
    *error =  ERROR_PLCS_PLC_NULL;
    return NULL;
  }

  return plcs_[plcIndex]->getExpr();
}

int ecmcPLCMain::setExpr(int plcIndex, char *expr) {
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  plcs_[plcIndex]->clearExpr();
  int errorCode = addExprLine(plcIndex, expr);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  return compileExpr(plcIndex);
}

int ecmcPLCMain::addExprLine(int plcIndex, const char *expr) {
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)

  int errorCode = parseExpr(plcIndex, expr);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  /*
    Special case:
    Remove unsupported syntax for simulation slave (ec<id>.s-1.xxx to ec<id>.d1.xxx )
    Expression TK cant handle "-" in var name
  */

  int   ecId, ecSlave;
  char *localExpr = strdup(expr);
  assert(strcmp(localExpr, expr) == 0);
  char *strEc       = localExpr;
  char *strSimSlave = 0;

  while ((strEc = strstr(strEc, ECMC_EC_STR)) && strlen(strEc) > 0) {
    // Sanity check
    int nvals = sscanf(strEc,
                       ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d.",
                       &ecId,
                       &ecSlave);

    if (nvals == 2) {
      if (ecSlave < 0) {  // Simulation slave
        strSimSlave = strstr(strEc, "." ECMC_SLAVE_CHAR);

        if (strSimSlave && (strlen(strSimSlave) > 2)) {
          strSimSlave    = strSimSlave + 1;
          strSimSlave[0] = ECMC_DUMMY_SLAVE_STR[0];
          strSimSlave[1] = ECMC_DUMMY_SLAVE_STR[1];
        }
      }
    }
    strEc++;
  }
  errorCode = plcs_[plcIndex]->addExprLine(localExpr);
  free(localExpr);
  return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
}

int ecmcPLCMain::appendExprLine(int plcIndex, const char *expr) {
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  int errorCode = plcs_[plcIndex]->appendRawExpr(expr);
  return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
}

std::string ecmcPLCMain::preProcessVarDeclaration(std::string line) {

  std::regex decl(R"(^\s*([A-Za-z_][A-Za-z0-9_\.]*)\s*:\s*([A-Za-z0-9_\.]+)\s*;?)");
 
  std::string t = trim(line);
 
  if (t == "VAR") { inVarSection_ = true; return "// " + line + "\n"; }
  if (t == "END_VAR") { inVarSection_ = false; return "// " + line + "\n"; }
 
  if (inVarSection_) {
    std::smatch m;
    if (std::regex_search(line, m, decl)) {
      varAlias_[m[1]] = m[2];
    }
    return "// " + line + "\n"; // do not emit the VAR block
  }
 
  // mask -> replace -> unmask
  std::vector<std::string> slots;
  std::string masked = mask_line(line, slots);
  replace_aliases_inplace(masked, varAlias_);
  return unmask_line(masked, slots) + "\n";

//  std::string trimmed = trim(line);
//
//  if (trimmed.empty()) {
//      return "\n";
//  }
//
//  if (startsWith(trimmed, "//") || startsWith(trimmed, "#")) {
//    if(inVarSection_ ) {
//      return "// " + line + '\n';
//    }
//    return line + '\n';
//  }
//
//  // Detect VAR / END_VAR
//  if (trimmed == "VAR") {
//      // new varaibel section, then clear old table
//      if(!inVarSection_ ) {
//        varMap_.clear();
//      }
//      inVarSection_ = true;      
//      return "// " + line + '\n';
//  } else if (trimmed == "END_VAR") {
//      inVarSection_ = false;
//      return "// " + line + '\n';
//  }
// 
//  if (inVarSection_) {
//      // Match alias : full.path;
//      std::regex varPattern(R"(^([A-Za-z_][A-Za-z0-9_\.]*)\s*:\s*([A-Za-z0-9_\.]+)\s*;?)");
//      std::smatch match;
//      if (std::regex_search(trimmed, match, varPattern)) {
//          std::string alias = match[1];
//          std::string fullName = match[2];
//          varMap_[alias] = fullName;
//      }
//      return "// " + line + '\n';
//  } else {
//      // Replace all aliases with full variable names
//      for (const auto& it : varMap_) {
//          const std::string& alias = it.first;
//          const std::string& fullName = it.second;
//          std::regex aliasPattern("\\b" + alias + "\\b");
//          line = std::regex_replace(line, aliasPattern, fullName);
//      }
//      line = line + '\n';
//      return line;
//  }
//  
//  return "";
}

int ecmcPLCMain::loadPLCFile(int plcIndex, char *fileName) {
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  std::ifstream plcFile;
  plcFile.open(fileName);


  // Variable declation substitution
  inVarSection_ = false;
  varAlias_.clear();

  if (!plcFile.good()) {
    LOGERR("%s/%s:%d: ERROR PLC%d: File not found: %s (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           plcIndex,
           fileName,
           ERROR_PLCS_FILE_NOT_FOUND);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLCS_FILE_NOT_FOUND);
  }

  std::string line, lineNoComments;
  int lineNumber = 1;
  int errorCode  = 0;

  printf("Loading PLC code file %s to plc index %d\n ", fileName, plcIndex);
  while (std::getline(plcFile, line)) {    

    line = preProcessVarDeclaration(line);
    // Remove Comments (everything after #)
    lineNoComments = line.substr(0, line.find(ECMC_PLC_FILE_COMMENT_CHAR));

    if (lineNoComments.length() > 0) {
      lineNoComments.append("\n");
      errorCode = appendExprLine(plcIndex, lineNoComments.c_str());
      if (errorCode) {
        LOGERR(
          "%s/%s:%d: ERROR PLC%d: Error appending file at line %d: %s (0x%x).\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          plcIndex,
          lineNumber,
          line.c_str(),
          errorCode);
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
      }
    }
    printf("%s",line.c_str());
    lineNumber++;
  }

  // Set enable as default
  errorCode = setEnable(plcIndex, 1);

  if (errorCode) {
    LOGERR("%s/%s:%d: ERROR PLC%d: Error Enabling PLC file: %s (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           plcIndex,
           fileName,
           errorCode);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  return 0;
}

int ecmcPLCMain::clearExpr(int plcIndex) {
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  return plcs_[plcIndex]->clearRawExpr();
}

int ecmcPLCMain::compileExpr(int plcIndex) {
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)

  if (plcs_[plcIndex]->getNewExpr())plcs_[plcIndex]->clearExpr();
  else {
    return 0;
  }

  int errorCode =
    addExprLine(plcIndex, plcs_[plcIndex]->getRawExpr()->c_str());

  if (errorCode) {
    return errorCode;
  }

  return plcs_[plcIndex]->compile();
}

int ecmcPLCMain::setEnable(int plcIndex, int enable) {
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  plcEnable_[plcIndex]->setData(static_cast<double>(enable));
  plcEnable_[plcIndex]->updateAsyn(1);
  return 0;
}

int ecmcPLCMain::getEnable(int plcIndex, int *enabled) {
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  *enabled = static_cast<int>(plcEnable_[plcIndex]->getData());
  return 0;
}

int ecmcPLCMain::deletePLC(int plcIndex) {
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  delete plcs_[plcIndex];
  plcs_[plcIndex] = NULL;
  return 0;
}

int ecmcPLCMain::getCompiled(int plcIndex, int *compiled) {
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  *compiled = plcs_[plcIndex]->getCompiled();
  return 0;
}

int ecmcPLCMain::getCompiled(int plcIndex) {
  if ((plcIndex >= ECMC_MAX_PLCS + ECMC_MAX_AXES) || (plcIndex < 0)) {
    LOGERR("ERROR: PLC index out of range.\n");
    return 0;
  }
  return plcs_[plcIndex]->getCompiled();
}

int ecmcPLCMain::parseExpr(int plcIndex, const char *exprStr) {
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);

  // Axis (global)
  int errorCode = parseAxis(plcIndex, exprStr);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  // EtherCAT (global)
  errorCode = parseEC(plcIndex, exprStr);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  // Static  (local)
  errorCode = parseStatic(plcIndex, exprStr);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  // Global
  errorCode = parseGlobal(plcIndex, exprStr);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  // PLC
  errorCode = parsePLC(plcIndex, exprStr);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  // Data storage
  errorCode = parseDataStorage(plcIndex, exprStr);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  // Functions
  errorCode = parseFunctions(plcIndex, exprStr);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  return 0;
}

int ecmcPLCMain::findGlobalDataIF(char *varName, ecmcPLCDataIF **outDataIF) {
  for (int i = 0; i < globalVariableCount_; i++) {
    if (globalDataArray_[i]) {
      int n = strcmp(varName, globalDataArray_[i]->getVarName());

      if (n == 0) {
        *outDataIF = globalDataArray_[i];
        return 0;
      }
    }
  }
  return 0;
}

int ecmcPLCMain::createNewGlobalDataIF(char              *varName,
                                       ecmcDataSourceType dataSource,
                                       ecmcPLCDataIF    **outDataIF) {
  // Axes data
  // Ec data
  // Global data
  // DataStorage
  if ((globalVariableCount_ >= ECMC_MAX_PLC_VARIABLES - 1) ||
      (globalVariableCount_ < 0)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_VARIABLE_COUNT_EXCEEDED);
  }
  int errorCode = 0;
  int axisId    = -1;
  int dsId      = -1;

  switch (dataSource) {
  case ECMC_RECORDER_SOURCE_NONE:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_SOURCE_INVALID);

    break;

  case ECMC_RECORDER_SOURCE_ETHERCAT:

    if (!ec_) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_PLC_EC_NULL);
    }
    globalDataArray_[globalVariableCount_] = new ecmcPLCDataIF(-1,
                                                               -1,
                                                               ec_,
                                                               varName,
                                                               asynPortDriver_);
    errorCode =
      globalDataArray_[globalVariableCount_]->getErrorID();

    if (errorCode) {
      delete globalDataArray_[globalVariableCount_];
      globalDataArray_[globalVariableCount_] = NULL;
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }

    *outDataIF = globalDataArray_[globalVariableCount_];
    globalVariableCount_++;
    break;

  case ECMC_RECORDER_SOURCE_AXIS:
    axisId = getAxisIndex(varName);

    if ((axisId >= ECMC_MAX_AXES) || (axisId < 0)) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_PLC_AXIS_ID_OUT_OF_RANGE);
    }

    if (!axes_[axisId]) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_PLC_AXIS_ID_OUT_OF_RANGE);
    }
    globalDataArray_[globalVariableCount_] = new ecmcPLCDataIF(-1,
                                                               -1,
                                                               axes_[axisId],
                                                               varName,
                                                               asynPortDriver_);
    errorCode =
      globalDataArray_[globalVariableCount_]->getErrorID();

    if (errorCode) {
      delete globalDataArray_[globalVariableCount_];
      globalDataArray_[globalVariableCount_] = NULL;
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }

    *outDataIF = globalDataArray_[globalVariableCount_];
    globalVariableCount_++;
    break;

  case ECMC_RECORDER_SOURCE_STATIC_VAR:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_SOURCE_INVALID);

    break;

  case ECMC_RECORDER_SOURCE_GLOBAL_VAR:
    globalDataArray_[globalVariableCount_] = new ecmcPLCDataIF(-1,
                                                               -1,
                                                               varName,
                                                               ECMC_RECORDER_SOURCE_GLOBAL_VAR,
                                                               asynPortDriver_);
    errorCode =
      globalDataArray_[globalVariableCount_]->getErrorID();

    if (errorCode) {
      delete globalDataArray_[globalVariableCount_];
      globalDataArray_[globalVariableCount_] = NULL;
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }

    *outDataIF = globalDataArray_[globalVariableCount_];
    globalVariableCount_++;
    break;

  case ECMC_RECORDER_SOURCE_DATA_STORAGE:
    dsId = getDsIndex(varName);

    if ((dsId >= ECMC_MAX_DATA_RECORDERS_OBJECTS) || (dsId < 0)) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_PLCS_DATA_STORAGE_INDEX_OUT_OF_RANGE);
    }

    if (!ds_[dsId]) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_PLCS_DATA_STORAGE_INDEX_OUT_OF_RANGE);
    }

    globalDataArray_[globalVariableCount_] = new ecmcPLCDataIF(-1,
                                                               -1,
                                                               ds_[dsId],
                                                               varName,
                                                               asynPortDriver_);
    errorCode =
      globalDataArray_[globalVariableCount_]->getErrorID();

    if (errorCode) {
      delete globalDataArray_[globalVariableCount_];
      globalDataArray_[globalVariableCount_] = NULL;
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }

    *outDataIF = globalDataArray_[globalVariableCount_];
    globalVariableCount_++;
    break;
  }
  return 0;
}

int ecmcPLCMain::getAxisIndex(char *varName) {
  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  axisId = 0;
  int  nvals  = sscanf(varName,
                       ECMC_AX_STR "%d." ECMC_PLC_VAR_FORMAT,
                       &axisId,
                       buffer);

  if (nvals == 2) {
    return axisId;
  }
  return -1;
}

int ecmcPLCMain::getDsIndex(char *varName) {
  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  dsId  = 0;
  int  nvals = sscanf(varName,
                      ECMC_PLC_DATA_STORAGE_STR "%d." ECMC_PLC_VAR_FORMAT,
                      &dsId,
                      buffer);

  if (nvals == 2) {
    return dsId;
  }
  return -1;
}

/*
 * find and register all axis variables in string
 */
int ecmcPLCMain::parseAxis(int plcIndex, const char *exprStr) {
  // find  and register all axis variables in string
  int   nvals = 0;
  int   axisId;
  int   errorCode = 0;
  char *strLocal  = strdup(exprStr);
  char *strAxis   = strLocal;
  char  varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];

  while ((strAxis = strstr(strAxis, ECMC_AX_STR)) && strlen(strAxis) > 0) {
    // Sanity check
    nvals = sscanf(strAxis,
                   ECMC_AX_STR "%d." ECMC_PLC_VAR_FORMAT,
                   &axisId,
                   varName);

    if (nvals == 2) {
      if ((axisId >= ECMC_MAX_AXES) || (axisId < 0)) {
        free(strLocal);
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_PLC_AXIS_ID_OUT_OF_RANGE);
      }

      if (!axes_[axisId]) {
        free(strLocal);
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_PLC_AXIS_ID_OUT_OF_RANGE);
      }
      varName[0] = '\0';
      nvals      = sscanf(strAxis, ECMC_PLC_VAR_FORMAT, varName);

      if (nvals == 1) {
        errorCode = createAndRegisterNewDataIF(plcIndex,
                                               varName,
                                               ECMC_RECORDER_SOURCE_AXIS);

        if (errorCode) {
          free(strLocal);
          return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
        }
      }
    }
    strAxis++;
  }
  free(strLocal);
  return 0;
}

/*
 * find and register all ec variables in string
 */
int ecmcPLCMain::parseEC(int plcIndex, const char *exprStr) {
  int   ecId, ecSlave;
  char *strLocal = strdup(exprStr);
  char *strEc    = strLocal;
  char  varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char  varNameTemp[EC_MAX_OBJECT_PATH_CHAR_LENGTH];

  varName[0]     = '\0';
  varNameTemp[0] = '\0';

  while ((strEc = strstr(strEc, ECMC_EC_STR)) && strlen(strEc) > 0) {
    int nvals = sscanf(strEc,
                       ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d." ECMC_PLC_VAR_FORMAT,
                       &ecId,
                       &ecSlave,
                       varNameTemp);

    if (nvals == 3) {
      // Ensure not bit access
      unsigned int charCount = snprintf(varName,
                                        sizeof(varName),
                                        ECMC_EC_STR "%d.%s%d.%s",
                                        ecId,
                                        ECMC_SLAVE_CHAR,
                                        ecSlave,
                                        varNameTemp);

      if (charCount >= sizeof(varName) - 1) {
        free(strLocal);
        LOGERR("%s/%s:%d:  Error: Variable name %s (0x%x).\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               varNameTemp,
               ERROR_PLCS_VARIABLE_NAME_TO_LONG);
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_PLCS_VARIABLE_NAME_TO_LONG);
      }

      int errorCode = createAndRegisterNewDataIF(plcIndex,
                                                 varName,
                                                 ECMC_RECORDER_SOURCE_ETHERCAT);

      if (errorCode) {
        free(strLocal);
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
      }
    }
    strEc++;
  }
  free(strLocal);
  return 0;
}

int ecmcPLCMain::parseStatic(int plcIndex, const char *exprStr) {
  // find static variable
  char *strLocal  = strdup(exprStr);
  char *strStatic = strLocal;
  char  varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];

  varName[0] = '\0';

  while ((strStatic =
            strstr(strStatic, ECMC_STATIC_VAR)) && strlen(strStatic) > 0) {
    // Sanity check
    int nvals = sscanf(strStatic, ECMC_PLC_VAR_FORMAT, varName);

    if (nvals == 1) {
      int errorCode = plcs_[plcIndex]->addAndRegisterLocalVar(varName);
      if (errorCode) {
        free(strLocal);
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
      }
    }
    strStatic++;
  }
  free(strLocal);
  return 0;
}

int ecmcPLCMain::parseFunctions(int plcIndex, const char *exprStr) {
  return plcs_[plcIndex]->parseFunctions(exprStr);
}

int ecmcPLCMain::parseGlobal(int plcIndex, const char *exprStr) {
  // find global variable
  char *strLocal  = strdup(exprStr);
  char *strGlobal = strLocal;
  char  varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];

  varName[0] = '\0';

  while ((strGlobal =
            strstr(strGlobal, ECMC_GLOBAL_VAR)) && strlen(strGlobal) > 0) {
    // Sanity check
    int nvals = sscanf(strGlobal, ECMC_PLC_VAR_FORMAT, varName);

    if (nvals == 1) {
      int errorCode = createAndRegisterNewDataIF(plcIndex,
                                                 varName,
                                                 ECMC_RECORDER_SOURCE_GLOBAL_VAR);

      if (errorCode) {
        free(strLocal);
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
      }
    }
    strGlobal++;
  }
  free(strLocal);
  return 0;
}

int ecmcPLCMain::parseDataStorage(int plcIndex, const char *exprStr) {
  // find plc variable
  char *strLocal = strdup(exprStr);
  char *strDS    = strLocal;
  char  varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];

  varName[0] = '\0';

  while ((strDS =
            strstr(strDS, ECMC_PLC_DATA_STORAGE_STR)) && strlen(strDS) > 0) {
    // Sanity check 1
    int tempInt = 0;
    int nvals   = sscanf(strDS,
                         ECMC_PLC_DATA_STORAGE_STR "%d." ECMC_PLC_VAR_FORMAT,
                         &tempInt,
                         varName);

    if (nvals == 2) {
      varName[0] = '\0';

      // Sanity check 2
      int nvals = sscanf(strDS, ECMC_PLC_VAR_FORMAT, varName);

      if (nvals == 1) {
        int errorCode = createAndRegisterNewDataIF(plcIndex,
                                                   varName,
                                                   ECMC_RECORDER_SOURCE_DATA_STORAGE);

        if (errorCode) {
          free(strLocal);
          return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
        }
      }
    }
    strDS++;
  }
  free(strLocal);
  return 0;
}

int ecmcPLCMain::parsePLC(int plcIndex, const char *exprStr) {
  // find plc variable
  char *strLocal = strdup(exprStr);
  char *strPLC   = strLocal;
  char  varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];

  varName[0] = '\0';

  while ((strPLC = strstr(strPLC, ECMC_PLC_DATA_STR)) && strlen(strPLC) > 0) {
    // Sanity check 1
    int tempInt = 0;
    int nvals   = sscanf(strPLC,
                         ECMC_PLC_DATA_STR "%d." ECMC_PLC_VAR_FORMAT,
                         &tempInt,
                         varName);

    if (nvals == 2) {
      if (plcVarNameValid(varName)) {
        varName[0] = '\0';

        // Sanity check 2
        int nvals = sscanf(strPLC, ECMC_PLC_VAR_FORMAT, varName);

        if (nvals == 1) {
          int errorCode = createAndRegisterNewDataIF(plcIndex,
                                                     varName,
                                                     ECMC_RECORDER_SOURCE_GLOBAL_VAR);

          if (errorCode) {
            free(strLocal);
            return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
          }
        }
      } else {
        LOGERR("%s/%s:%d: Invalid variable name: %s (0x%x).\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               varName,
               ERROR_PLCS_INVALID_VAR_NAME);
        free(strLocal);
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_PLCS_INVALID_VAR_NAME);
      }
    }
    strPLC++;
  }
  free(strLocal);
  return 0;
}

/*
* Ensure that plc variable name is correct
*/
int ecmcPLCMain::plcVarNameValid(const char *plcVar) {
  int length = strlen(plcVar);

  if (strstr(plcVar,
             ECMC_PLC_ENABLE_DATA_STR) &&
      (length == strlen(ECMC_PLC_ENABLE_DATA_STR))) {
    return 1;
  }

  if (strstr(plcVar,
             ECMC_PLC_ERROR_DATA_STR) &&
      (length == strlen(ECMC_PLC_ERROR_DATA_STR))) {
    return 1;
  }

  if (strstr(plcVar,
             ECMC_PLC_SCAN_TIME_DATA_STR) &&
      (length == strlen(ECMC_PLC_SCAN_TIME_DATA_STR))) {
    return 1;
  }

  if (strstr(plcVar,
             ECMC_PLC_FIRST_SCAN_STR) &&
      (length == strlen(ECMC_PLC_FIRST_SCAN_STR))) {
    return 1;
  }
  return 0;
}

/*
 * Create new dataIF if needed and register in PLC
 */
int ecmcPLCMain::createAndRegisterNewDataIF(int                plcIndex,
                                            char              *varName,
                                            ecmcDataSourceType dataSource) {
  ecmcPLCDataIF *dataIF = NULL;
  int errorCode         = findGlobalDataIF(varName, &dataIF);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  if (!dataIF) {
    // create new
    errorCode = createNewGlobalDataIF(varName, dataSource, &dataIF);

    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  }

  // Register in dedicated PLC if needed
  errorCode = plcs_[plcIndex]->addAndReisterGlobalVar(dataIF);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }
  return 0;
}

int ecmcPLCMain::addMainDefaultVariables() {
  if (ec_->getInitDone()) {
    int masterId = ec_->getMasterIndex();

    // Add ec<index>.masterstatus
    char varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
    int  chars = snprintf(varName,
                          EC_MAX_OBJECT_PATH_CHAR_LENGTH - 1,
                          ECMC_EC_STR "%d." ECMC_ASYN_EC_PAR_MASTER_STAT_NAME,
                          masterId);

    if (chars >= EC_MAX_OBJECT_PATH_CHAR_LENGTH - 1) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_PLCS_VARIABLE_NAME_TO_LONG);
    }

    globalDataArray_[globalVariableCount_] = new ecmcPLCDataIF(-1,
                                                               -1,
                                                               varName,
                                                               ECMC_RECORDER_SOURCE_GLOBAL_VAR,
                                                               asynPortDriver_);
    int errorCode =
      globalDataArray_[globalVariableCount_]->getErrorID();

    if (errorCode) {
      delete globalDataArray_[globalVariableCount_];
      globalDataArray_[globalVariableCount_] = NULL;
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }

    ecStatus_ = globalDataArray_[globalVariableCount_];
    ecStatus_->setReadOnly(1);
    globalVariableCount_++;
  }
  return 0;
}

int ecmcPLCMain::addPLCDefaultVariables(int plcIndex, int skipCycles) {
  int errorCode = 0;

  if (ec_->getInitDone()) {
    // Add ec<id>.masterstatus
    errorCode = plcs_[plcIndex]->addAndReisterGlobalVar(ecStatus_);

    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  }

  ecmcPLCDataIF *dataIF = NULL;

  // Add plc<index>.enable
  errorCode =
    addPLCDefaultVariable(plcIndex, ECMC_PLC_ENABLE_DATA_STR, &dataIF);

  if (errorCode) {
    return errorCode;
  }
  plcEnable_[plcIndex] = dataIF;

  // Add plc<index>.error
  errorCode =
    addPLCDefaultVariable(plcIndex, ECMC_PLC_ERROR_DATA_STR, &dataIF);

  if (errorCode) {
    return errorCode;
  }
  plcError_[plcIndex] = dataIF;

  // Add plc<index>.scantime
  errorCode = addPLCDefaultVariable(plcIndex,
                                    ECMC_PLC_SCAN_TIME_DATA_STR,
                                    &dataIF);

  if (errorCode) {
    return errorCode;
  }
  dataIF->setReadOnly(1);
  dataIF->setData(1 / mcuFreq_ * (skipCycles + 1));

  // Add plc<index>.firstscan
  errorCode =
    addPLCDefaultVariable(plcIndex, ECMC_PLC_FIRST_SCAN_STR, &dataIF);

  if (errorCode) {
    return errorCode;
  }
  dataIF->setReadOnly(1);
  dataIF->setData(1);
  plcFirstScan_[plcIndex] = dataIF;

  return 0;
}

int ecmcPLCMain::getPLCErrorID() {
  for (int i = 0; i < ECMC_MAX_PLCS; i++) {
    if (plcError_[i]) {
      if (plcError_[i]->getData()) {
        return static_cast<int>(plcError_[i]->getData());
      }
    }
  }
  return 0;
}

bool ecmcPLCMain::getError() {
  return getPLCErrorID() || ecmcError::getError();
}

int ecmcPLCMain::getErrorID() {
  if (getPLCErrorID()) {
    return getPLCErrorID();
  }
  return ecmcError::getErrorID();
}

void ecmcPLCMain::errorReset() {
  // reset all plc error
  for (int i = 0; i < ECMC_MAX_PLCS; i++) {
    if (plcError_[i]) {
      if (plcError_[i]->getData()) {
        plcError_[i]->setData(0);
      }
    }
  }
  ecmcError::errorReset();
}

int ecmcPLCMain::updateAllScanTimeVars(int plcIndex) {
  char varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];

  if (plcs_[plcIndex]) {
    int chars = snprintf(varName,
                         EC_MAX_OBJECT_PATH_CHAR_LENGTH - 1,
                         ECMC_PLC_DATA_STR "%d." ECMC_PLC_SCAN_TIME_DATA_STR,
                         plcIndex);

    if (chars >= EC_MAX_OBJECT_PATH_CHAR_LENGTH - 1) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_PLCS_VARIABLE_NAME_TO_LONG);
    }
    ecmcPLCDataIF *dataIF = NULL;
    int errorCode         = findGlobalDataIF(varName, &dataIF);

    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }

    if (dataIF) {
      dataIF->setReadOnly(1);
      dataIF->setData(plcs_[plcIndex]->getSampleTime());
    }
  }

  return 0;
}

int ecmcPLCMain::updateAllScanTimeVars() {
  // Update all scantime variables for all axes)
  for (int i = 0; i < ECMC_MAX_PLCS + ECMC_MAX_AXES; i++) {
    int error = updateAllScanTimeVars(i);

    if (error) {
      return error;
    }
  }
  return 0;
}

int ecmcPLCMain::readStaticPLCVar(int plcIndex, const char  *varName,
                                  double *data) {
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  return plcs_[plcIndex]->readStaticPLCVar(varName, data);
}

int ecmcPLCMain::writeStaticPLCVar(int plcIndex, const char  *varName,
                                   double data) {
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  return plcs_[plcIndex]->writeStaticPLCVar(varName, data);
}

ecmcPLCTask * ecmcPLCMain::getPLCTaskForAxis(int axisId) {
  /*
   PLCs in array:
   0             .. (ECMC_MAX_PLCS-1)                   : Normal PLCs (exe here)
   ECMC_MAX_PLCS .. (ECMC_MAX_PLCS + ECMC_MAX_AXES-1)   : PLC objects for axis syncs
                                                          exe in axis objects
  */
  int error = 0;
  int index = ECMC_MAX_PLCS + axisId;

  if ((index >= ECMC_MAX_PLCS + ECMC_MAX_AXES) || (index < ECMC_MAX_PLCS)) {
    LOGERR("%s/%s:%d: ERROR: Axis PLC index out of range.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    return NULL;
  }

  if (!plcs_[index]) {
    // Always run sync with axis
    try {
      error = createPLC(index, 0);
      LOGERR("%s/%s:%d: ERROR: Fail create PLC for axis %d (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             axisId,
             error);
      return NULL;
    }
    catch (std::exception& e) {
      delete plcs_[index];
      LOGERR(
        "%s/%s:%d: EXCEPTION %s WHEN ALLOCATE MEMORY FOR AXIS PLC OBJECT.\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        e.what());
      return NULL;
    }
  }

  return plcs_[index];
}

int ecmcPLCMain::addPLCDefaultVariable(int             plcIndex,
                                       const char     *suffix,
                                       ecmcPLCDataIF **dataIFOut) {
  char varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  chars = 0;

  // Normal PLC
  if (plcIndex < ECMC_MAX_PLCS) {
    chars = snprintf(varName,
                     EC_MAX_OBJECT_PATH_CHAR_LENGTH - 1,
                     ECMC_PLC_DATA_STR "%d.%s",
                     plcIndex, suffix);

    if (chars >= EC_MAX_OBJECT_PATH_CHAR_LENGTH - 1) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_PLCS_VARIABLE_NAME_TO_LONG);
    }
  } else {   // Axis PLC
    chars = snprintf(varName,
                     EC_MAX_OBJECT_PATH_CHAR_LENGTH - 1,
                     ECMC_AX_STR "%d." ECMC_PLC_DATA_STR ".%s",
                     plcIndex - ECMC_MAX_PLCS, suffix);

    if (chars >= EC_MAX_OBJECT_PATH_CHAR_LENGTH - 1) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_PLCS_VARIABLE_NAME_TO_LONG);
    }
  }
  int errorCode = createAndRegisterNewDataIF(plcIndex,
                                             varName,
                                             ECMC_RECORDER_SOURCE_GLOBAL_VAR);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  errorCode = findGlobalDataIF(varName, dataIFOut);

  if (errorCode) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
  }

  if (!*dataIFOut) {
    LOGERR(
      "%s/%s:%d: Failed allocation of ecmcPLCDataIF object %s (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      varName,
      ERROR_PLC_DATA_IF_ALLOCATION_FAILED);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_DATA_IF_ALLOCATION_FAILED);
  }
  return 0;
}

int ecmcPLCMain::setPluginPointer(ecmcPluginLib *plugin, int index) {
  if ((index < 0) && (index >= ECMC_MAX_PLUGINS)) {
    return ERROR_PLCS_PLUGIN_INDEX_OUT_OF_RANGE;
  }

  plugins_[index] = plugin;
  return 0;
}

int ecmcPLCMain::addLib(int plcIndex, ecmcPLCLib* lib) {
  int errorCode = 0;
  
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  if(!lib) {
    LOGERR("%s/%s:%d: PLC lib NULL(0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_PLC_LIB_NULL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_PLC_LIB_NULL);
  }

  for(size_t i = 0; i < lib->getFunctionCount(); i++){
    ecmcPLCLibFunc* func = lib->getFunction(i);
    if(func) {
      errorCode = parseExpr(plcIndex, func->getExpression().c_str());
      if(errorCode) {
        return errorCode;
      }
    }
  }
  // Finaly add the lib
  return plcs_[plcIndex]->addLib(lib);
}

// Simple trim helper
std::string ecmcPLCMain::trim(const std::string& s)
{
    return std::regex_replace(s, std::regex("^\\s+|\\s+$"), "");
}

bool ecmcPLCMain::startsWith(const std::string& s, const std::string& p) {
    return s.size() >= p.size() && s.compare(0, p.size(), p) == 0;
}

std::string ecmcPLCMain::escape_regex_alias(const std::string& a) {
    // a contains [A-Za-z0-9_.], only '.' needs escaping
    std::string r; r.reserve(a.size()*2);
    for (char c : a) r += (c == '.') ? "\\." : std::string(1, c);
    return r;
}

// Mask single-quoted strings and trailing comments. Returns masked line.
// Fills 'slots' with originals, in order, to restore later.
std::string ecmcPLCMain::mask_line(const std::string& line, std::vector<std::string>& slots)
{
    std::string out;
    out.reserve(line.size());
    const std::string SEP = "\x1F"; // unlikely delimiter
    bool in_str = false;
    char prev = '\0';
    for (std::size_t i = 0; i < line.size(); ++i) {
        char c = line[i];
        // handle start of trailing comment when not in string
        if (!in_str) {
            if (c == '/' && i + 1 < line.size() && line[i+1] == '/') {
                // mask rest as a comment slot
                std::string comment = line.substr(i);
                std::size_t id = slots.size();
                slots.push_back(comment);
                out += SEP + "C" + std::to_string(id) + SEP;
                return out; // rest consumed
            }
            if (c == '#') {
                std::string comment = line.substr(i);
                std::size_t id = slots.size();
                slots.push_back(comment);
                out += SEP + "C" + std::to_string(id) + SEP;
                return out;
            }
        }
        // toggle single-quoted string (respect backslash escape)
        if (c == '\'' && prev != '\\') {
            if (!in_str) {
                // beginning of string: gather whole literal
                std::size_t j = i + 1;
                bool closed = false; prev = '\0';
                for (; j < line.size(); ++j) {
                    char d = line[j];
                    if (d == '\'' && prev != '\\') { closed = true; ++j; break; }
                    prev = d;
                }
                std::string lit = line.substr(i, j - i); // include quotes
                std::size_t id = slots.size();
                slots.push_back(lit);
                out += SEP + "S" + std::to_string(id) + SEP;
                i = j - 1; // advance
                prev = '\0';
                continue;
            }
        }
        out.push_back(c);
        prev = c;
    }
    return out;
}
 
std::string ecmcPLCMain::unmask_line(const std::string& line, const std::vector<std::string>& slots)
{
    std::string out; out.reserve(line.size());
    const std::string SEP = "\x1F";
    for (std::size_t i = 0; i < line.size();) {
        if (line[i] == '\x1F') {
            // token format: \x1F [S|C] <digits> \x1F
            std::size_t j = line.find('\x1F', i + 1);
            if (j == std::string::npos) { out.append(line, i, std::string::npos); break; }
            std::string token = line.substr(i + 1, j - (i + 1)); // e.g. "S0"
            if (token.size() >= 2) {
                std::size_t id = std::size_t(std::stoul(token.substr(1)));
                out += (id < slots.size()) ? slots[id] : "";
            }
            i = j + 1;
        } else {
            out.push_back(line[i++]);
        }
    }
    return out;
}
 
// Replace aliases on code-only text, avoiding partial matches.
// Pattern: (^|[^A-Za-z0-9_.]) alias ([^A-Za-z0-9_.]|$)  -> keep $1/$3
void ecmcPLCMain::replace_aliases_inplace(std::string& code,
                                    const std::unordered_map<std::string,std::string>& m)
{
    for (const auto& kv : m) {
        const std::string alias_escaped = escape_regex_alias(kv.first);
        const std::string repl = kv.second;
        const std::string pattern =
            "(^|[^A-Za-z0-9_\\.])(" + alias_escaped + ")([^A-Za-z0-9_\\.]|$)";
        std::regex rx(pattern);
        // We need to repeatedly apply because replacement can change text length.
        std::string tmp;
        for (;;) {
            tmp = std::regex_replace(code, rx, "$1" + repl + "$3");
            if (tmp == code) break;
            code.swap(tmp);
        }
    }
}
