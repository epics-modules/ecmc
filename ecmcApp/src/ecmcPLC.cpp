/*
 *
 *  Created on: Oct 4, 2018
 *      Author: anderssandstrom
 */

#include "ecmcPLC.h"

ecmcPLC::ecmcPLC(ecmcEc *ec)
{
  initVars();
  exprtk_=new exprtkWrap();
  if(!exprtk_){
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR EXPRTK.\n",__FILE__,__FUNCTION__,__LINE__);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_EXPRTK_ALLOCATION_FAILED);
    exit(EXIT_FAILURE);
  }
  ec_=ec;
}

ecmcPLC::~ecmcPLC()
{
  for(int i=0;i<ECMC_MAX_PLC_VARIABLES;i++){
    delete dataArray_[i];
  }
}

int ecmcPLC::setAxisArrayPointer(ecmcAxisBase *axis,int index)
{
  if(index>=ECMC_MAX_AXES || index<0){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_AXIS_INDEX_OUT_OF_RANGE);
  }
  axes_[index]=axis;
  return 0;
}

void ecmcPLC::initVars()
{
  errorReset();
  exprStr_="";
  compiled_=false;
  variableCount_=0;
  inStartup_=1;
  enable_=0;
  for(int i=0;i<ECMC_MAX_PLC_VARIABLES;i++){
    dataArray_[i]=NULL;
  }
}

int ecmcPLC::setExpr(char *exprStr)
{
  exprStr_=exprStr;
  compiled_=false;
  variableCount_=0;

  // Expr cleared (Allow ";" as empty expression)
  if(exprStr_.length()<=1){
    return 0;
  }

  int errorCode=parseExpr(exprStr);

  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  errorCode=compile();
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  return 0;
}

int ecmcPLC::parseExpr(char * exprStr)
{
  if(strlen(exprStr)>=ECMC_MAX_PLC_VARIABLES-1){
    LOGERR("%s/%s:%d: ERROR: Expression to long (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_PLC_EXPR_LINE_TO_LONG);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_EXPR_LINE_TO_LONG);
  }

  // Find axes
  int nvals=0;
  int axisId;
  int errorCode=0;
  char *strAxis=exprStr;
  char varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  while((strAxis=strstr(strAxis,ECMC_AX_STR)) && strlen(strAxis)>0){
    // Sanity check
    nvals = sscanf(strAxis,ECMC_AX_STR"%d.%[0-9a-zA-Z._]",&axisId,varName);
    if (nvals == 2){
      if(axisId>=ECMC_MAX_AXES || !axes_[axisId]){
	return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_AXIS_ID_OUT_OF_RANGE);
      }
      varName[0]='\0';
      nvals = sscanf(strAxis,"%[0-9a-zA-Z._]",varName);
      if (nvals == 1){
        errorCode=addAxisVar(axisId,varName);
        if(errorCode){
	  return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
        }
      }
    }
    strAxis++;
  }

  //find EC
  int ecId;
  char *strEc=exprStr;
  varName[0]='\0';
  while((strEc=strstr(strEc,ECMC_EC_STR)) && strlen(strEc)>0){
    //Sanity check
    nvals = sscanf(strEc, ECMC_EC_STR"%d.%[0-9a-zA-Z._]",&ecId,varName);
    if (nvals == 2){
      varName[0]='\0';
      nvals = sscanf(strEc,"%[0-9a-zA-Z._]",varName);
      if (nvals == 1){
        errorCode=addEcVar(ecId,varName);
        if(errorCode){
	  return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
        }
      }
    }
    strEc++;
  }

  //find static variable
  char *strStatic=exprStr;
  varName[0]='\0';
  while((strStatic=strstr(strStatic,ECMC_STATIC_VAR)) && strlen(strStatic)>0){
    //Sanity check
    nvals = sscanf(strStatic,"%[0-9a-zA-Z._]",varName);
    if (nvals == 1){
      errorCode=addStaticVar(varName);
      if(errorCode){
	return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
      }
    }
    strStatic++;
  }

  return 0;
}

int ecmcPLC::addAxisVar(int axisId, char *axisVarStr)
{
  //Already added?
  if(varExist(axisVarStr)){
    return 0;
  }
  dataArray_[variableCount_]=new ecmcPLCDataIF(axes_[axisId],axisVarStr);
  int errorCode=dataArray_[variableCount_]->getErrorID();
  if(errorCode){
    delete dataArray_[variableCount_];
    return errorCode;
  }

  if(exprtk_->addVariable(axisVarStr,dataArray_[variableCount_]->getDataRef())){
    delete dataArray_[variableCount_];
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
  }

  variableCount_++;
  return 0;
}

int ecmcPLC::addEcVar(int ecId,char *ecVarStr)
{
  //Already added?
  if(varExist(ecVarStr)){
    return 0;
  }
  dataArray_[variableCount_]=new ecmcPLCDataIF(ec_,ecVarStr);
  int errorCode=dataArray_[variableCount_]->getErrorID();
  if(errorCode){
    delete dataArray_[variableCount_];
    return errorCode;
  }

  if(exprtk_->addVariable(ecVarStr,dataArray_[variableCount_]->getDataRef())){
    delete dataArray_[variableCount_];
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
  }

  variableCount_++;
  return 0;
}

int ecmcPLC::addStaticVar(char *staticVarStr)
{
  //Already added?
  if(varExist(staticVarStr)){
    return 0;
  }
  dataArray_[variableCount_]=new ecmcPLCDataIF(staticVarStr);
  int errorCode=dataArray_[variableCount_]->getErrorID();
  if(errorCode){
    delete dataArray_[variableCount_];
    return errorCode;
  }

  if(exprtk_->addVariable(staticVarStr,dataArray_[variableCount_]->getDataRef())){
    delete dataArray_[variableCount_];
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
  }

  variableCount_++;
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
  if(!compiled_ or !enable_){
    return 0;
  }
  //Wait for EC OK
  if(ecOK){
    inStartup_=0;
  }

  //Wait for EC OK
  if(!ecOK and inStartup_){
    return 0;
  }

  if(exprtk_==NULL){
    return 0;
  }

  // Update data from sources
  for(int i=0; i<variableCount_;i++){
    if(dataArray_[i]){
      dataArray_[i]->read();
    }
  }

  // Run equation
  exprtk_->refresh();

  // Update changed data
  for(int i=0; i<variableCount_;i++){
    if(dataArray_[i]){
      dataArray_[i]->write();
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
  int errorCode=parseExpr(exprStr);

  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  return 0;
}

int ecmcPLC::clearExpr()
{
  exprStr_="";
  compiled_=false;
  for(int i=0; i<variableCount_;i++){
    if(dataArray_[i]){
      delete dataArray_[i];
    }
  }
  variableCount_=0;
  return 0;
}

int ecmcPLC::varExist(char *varName)
{
  for(int i=0; i<variableCount_;i++){
    if(dataArray_[i]){
      int n =strcmp(varName,dataArray_[i]->getVarName());
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
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_COMPILE_ERROR);
  }

  if(getErrorID()){
    return getErrorID();
  }

  int errorCode=0;
  for(int i=0; i<variableCount_;i++){
    if(dataArray_[i]){
      errorCode=dataArray_[i]->validate();
      if(errorCode){
	return errorCode;
      }
    }
    else{
      return ERROR_PLC_PLC_DATA_IF_NULL;
    }
  }
  return 0;
}

int ecmcPLC::setEnable(int enable)
{
  enable_=enable;
  return 0;
}
