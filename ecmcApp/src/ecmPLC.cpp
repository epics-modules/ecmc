/*
 *
 *  Created on: Oct 4, 2018
 *      Author: anderssandstrom
 */

#include "ecmcPLC.h"

ecmcPLC::ecmcPLC(ecmcAxisBase *axes[ECMC_MAX_AXES],ecmcEc *ec)
{
  initVars();
  exprtk_=new exprtkWrap();
  if(!exprtk_){
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR EXPRTK.\n",__FILE__,__FUNCTION__,__LINE__);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_EXPRTK_ALLOCATION_FAILED);
    exit(EXIT_FAILURE);
  }
  ec_=ec;
  axes_=axes;
}

ecmcPLC::~ecmcPLC()
{
  for(int i=0;i<ECMC_MAX_PLC_VARIABLES;i++){
    delete dataArray_[i];
  }
}

void ecmcPLC::initVars()
{
  errorReset();
  expressionString_="";
  compiled_=false;
  for(int i=0;i<ECMC_MAX_PLC_VARIABLES;i++){
    dataArray_[i]=0;
  }
}

int ecmcPLC::setExpression(std::string expressionString)
{
  expressionString_=expressionString;
  compiled_=false;

  // Expression cleared (Allow ";" as empty expression)
  if(expressionString_.length()<=1){
    return 0;
  }

  int errorCode=compile();
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  return 0;
}

int ecmcPLC::parseExpression(std::string expressionString)
{
  variableCount_=0;
  // Find axes
  size_t pos=-1;
  int nvals=0;
  int axisId;
  int errorCode=0;
  char objectFunctionStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  while(pos=expressionString.find(ECMC_AX_STR,pos+1)>0){
    nvals = sscanf(expressionString.c_str()[pos], ECMC_AX_STR"%d.%s0-9a-zA-Z._",&axisId,objectFunctionStr);
    if (nvals == 2){
      if(axisId>=ECMC_MAX_AXES){
	return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_AXIS_ID_OUT_OF_RANGE);
      }
      errorCode=addAxisVar(axisId,objectFunctionStr);
      if(errorCode){
	return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
      }
    }
  }

  //find EC
  pos=0;
  int ecId;
  while(pos=expressionString.find(ECMC_EC_STR,pos+1)>0){
    //Sanity check
    nvals = sscanf(expressionString.c_str()[pos], ECMC_EC_STR"%d.%s[0-9a-zA-Z._]",&ecId,objectFunctionStr);
    if (nvals == 2){
      errorCode=addEcVar(ecId,expressionString.c_str()[pos]);
      if(errorCode){
	return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
      }
    }
  }

  return compile();
}

int ecmcPLC::addAxisVar(int axisId, char *axisVarStr)
{
  dataArray_[variableCount_]=new ecmcPLCDataIF(axes_[axisId],axisVarStr)
  int errorCode=dataArray_[variableCount_].getErrorID();
  if(errorCode){
    delete dataArray_[variableCount_];
    return errorCode;
  }

  if(exprtk_->addVariable(,)){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
  }

  variableCount_++;
  return 0;
}

int ecmcPLC::addEcVar(int ecId,char *ecVarStr)
{
  dataArray_[variableCount_]=new ecmcPLCDataIF(ec_,ecVarStr)
  int errorCode=dataArray_[variableCount_].getErrorID();
  if(errorCode){
    delete dataArray_[variableCount_];
    return errorCode;
  }

  if(exprtk_->addVariable(varNameBuffer,outputArray_[i+commandIndex*elementsPerCommand_])){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
  }

  variableCount_++;
  return 0;
}

int ecmcPLC::compile()
{
  if(exprtk_->compile(expressionString_)){
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

int ecmcPLC::refresh()
{
  //Update data from sources
  for(int i=0; i<variableCount_;i++){
    if(dataArray_[i]){
      dataArray_[i].read();
    }
  }

  // Run equation
  exprtk_->refresh();

  //Update changed data
  for(int i=0; i<variableCount_;i++){
    if(dataArray_[i]){
      dataArray_[i].write();
    }
  }

  return 0;
}

int ecmcPLC::validate()
{
  if(!compiled_){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_COMPILE_ERROR);
  }
  return 0;
}

std::string *ecmcPLC::getExpression()
{
  return &expressionString_;
}
