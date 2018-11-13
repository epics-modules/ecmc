/*
 *
 *  Created on: Oct 4, 2018
 *      Author: anderssandstrom
 */

#include "ecmcPLC.h"

ecmcPLC::ecmcPLC(int skipCycles)
{
  initVars();
  skipCycles_=skipCycles;
  exprtk_=new exprtkWrap();
  plcScanTimeInSecs_=1/MCU_FREQUENCY*(skipCycles+1);
}

ecmcPLC::~ecmcPLC()
{
  for(int i=0;i<ECMC_MAX_PLC_VARIABLES;i++){
    delete localArray_[i];
  }
}

void ecmcPLC::initVars()
{
  errorReset();
  exprStr_="";
  compiled_=false;
  globalVariableCount_=0;
  localVariableCount_=0;
  inStartup_=1;
  skipCycles_=0;
  skipCyclesCounter_=0;
  plcScanTimeInSecs_=0;
  for(int i=0;i<ECMC_MAX_PLC_VARIABLES;i++){
    globalArray_[i]=NULL;
    localArray_[i]=NULL;
  }
}

int ecmcPLC::addAndRegisterLocalVar(char *localVarStr)
{
  //Already added?addAndReisterGlobalVar
  if(localVarExist(localVarStr)){
    return 0;
  }

  if(localVariableCount_>=ECMC_MAX_PLC_VARIABLES-1 || localVariableCount_<0){
    LOGERR("%s/%s:%d: PLC local variable count excedded. Adding %s failed (0x%x).\n",__FILE__,__FUNCTION__,__LINE__,localVarStr,ERROR_PLC_VARIABLE_COUNT_EXCEEDED);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_VARIABLE_COUNT_EXCEEDED);
  }

  localArray_[localVariableCount_]=new ecmcPLCDataIF(localVarStr,ECMC_RECORDER_SOURCE_STATIC_VAR);
  int errorCode=localArray_[localVariableCount_]->getErrorID();
  if(errorCode){
    LOGERR("%s/%s:%d: PLC local variable: Create data interface failed. Adding %s failed (0x%x).\n",__FILE__,__FUNCTION__,__LINE__,localVarStr,errorCode);
    delete localArray_[localVariableCount_];    
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  if(exprtk_->addVariable(localVarStr,localArray_[localVariableCount_]->getDataRef())){
    LOGERR("%s/%s:%d: Failed to add variable %s to exprtk  (0x%x).\n",__FILE__,__FUNCTION__,__LINE__,localVarStr,ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
    delete localArray_[localVariableCount_];
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
  }

  localVariableCount_++;
  return 0;
}

int ecmcPLC::compile()
{
  if(exprtk_->compile(exprStr_)){
    compiled_=false;
    LOGERR("%s/%s:%d: Error: Transform compile error: %s.\n",__FILE__, __FUNCTION__, __LINE__,exprtk_->getParserError().c_str());
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_COMPILE_ERROR);
  }

  compiled_=true;
  return 0;
}

bool ecmcPLC::getCompiled()
{
  return compiled_;
}

int ecmcPLC::execute(bool ecOK)
{
  if(!compiled_ || skipCyclesCounter_<skipCycles_){
    skipCyclesCounter_++;
    return 0;
  }
  skipCyclesCounter_=0;

  if(ecOK){
    inStartup_=0;
  }

  //Wait for EC OK
  if(!ecOK and inStartup_){
    return 0;
  }

  if(!exprtk_){
    return 0;
  }

  for(int i=0; i<localVariableCount_;i++){
    if(localArray_[i]){
      localArray_[i]->read();
    }
  }

  for(int i=0; i<globalVariableCount_;i++){
    if(globalArray_[i]){
      globalArray_[i]->read();
    }
  }

  // Run equation
  exprtk_->refresh();

  for(int i=0; i<localVariableCount_;i++){
    if(localArray_[i]){
      localArray_[i]->write();
    }
  }

    for(int i=0; i<globalVariableCount_;i++){
    if(globalArray_[i]){
      globalArray_[i]->write();
    }
  }


  return 0;
}

std::string *ecmcPLC::getExpr()
{
  return &exprStr_;
}

int ecmcPLC::addExprLine(char *exprStr)
{
  try {
    exprStr_+=exprStr;
  }
  catch (const std::exception& e) {
    LOGERR("%s/%s:%d: Append of expression line failed: %s (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,e.what(),ERROR_PLC_ADD_EXPR_LINE_ERROR);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_ADD_EXPR_LINE_ERROR);
  }
  compiled_=false;
  
  return 0;
}

int ecmcPLC::clearExpr()
{
  exprStr_="";
  compiled_=false;
  for(int i=0; i<localVariableCount_;i++){
    if(localArray_[i]){
      delete localArray_[i];
    }
  }
  localVariableCount_=0;
  return 0;
}

int ecmcPLC::localVarExist(const char *varName)
{
  for(int i=0; i<localVariableCount_;i++){
    if(localArray_[i]){
      int n =strcmp(varName,localArray_[i]->getVarName());
      if(n==0){
        return 1;
      }
    }
  }
  return 0;
}

int ecmcPLC::globalVarExist(const char *varName)
{
  for(int i=0; i<globalVariableCount_;i++){
    if(globalArray_[i]){   
      int n = strcmp(varName,globalArray_[i]->getVarName());
      if(n==0){
        return 1;
      }
    }
  }
  return 0;
}

int ecmcPLC::validate()
{
  if(!compiled_){
    LOGERR("%s/%s:%d: Error: Validation of PLC object failed: Not compiled (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_PLC_COMPILE_ERROR);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_COMPILE_ERROR);
  }

  if(getErrorID()){
    return getErrorID();
  }

  int errorCode=0;

  //Check global variables
  for(int i=0; i<globalVariableCount_;i++){
    if(globalArray_[i]){
      errorCode=globalArray_[i]->validate();
      if(errorCode){
        LOGERR("%s/%s:%d: Error: Validation of Global PLCDataIF  %s at index %d  failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,globalArray_[i]->getVarName(),i,errorCode);
	      return errorCode;
      }
    }
    else{
      LOGERR("%s/%s:%d: Error: Validation of Global PLCDataIF failed. PLCDataIf NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_PLC_PLC_DATA_IF_NULL);
      return ERROR_PLC_PLC_DATA_IF_NULL;
    }
  }

  //Check local variables
  for(int i=0; i<localVariableCount_;i++){
    if(localArray_[i]){
      errorCode=localArray_[i]->validate();
      if(errorCode){
        LOGERR("%s/%s:%d: Error: Validation of Global PLCDataIF  %s at index %d  failed (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,localArray_[i]->getVarName(),i,errorCode);
	      return errorCode;
      }
    }
    else{
      LOGERR("%s/%s:%d: Error: Validation of Local PLCDataIF failed. PLCDataIf NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_PLC_PLC_DATA_IF_NULL);
      return ERROR_PLC_PLC_DATA_IF_NULL;
    }
  }

  return 0;
}

int ecmcPLC::addAndReisterGlobalVar(ecmcPLCDataIF *dataIF)
{
  if(!dataIF){
    LOGERR("%s/%s:%d: Data IF NULL  (0x%x).\n",__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_PLC_DATA_IF_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_PLC_DATA_IF_NULL);
  }

  //Check if already added
  if(!globalVarExist(dataIF->getVarName())){
    if(globalVariableCount_>=ECMC_MAX_PLC_VARIABLES-1 || globalVariableCount_<0){
      LOGERR("%s/%s:%d: PLC global variable count excedded. Adding %s failed (0x%x).\n",__FILE__,__FUNCTION__,__LINE__,dataIF->getVarName(),ERROR_PLC_VARIABLE_COUNT_EXCEEDED);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_VARIABLE_COUNT_EXCEEDED);
    }

    if(exprtk_->addVariable(dataIF->getExprTkVarName(),dataIF->getDataRef())){
      LOGERR("%s/%s:%d: Failed to add variable %s to exprtk  (0x%x).\n",__FILE__,__FUNCTION__,__LINE__,dataIF->getVarName(),ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
    }

    globalArray_[globalVariableCount_]=dataIF;
    globalVariableCount_++;
  }
  return 0;
}

double ecmcPLC::getSampleTime()
{
  return 1/MCU_FREQUENCY*(skipCycles_+1);
}