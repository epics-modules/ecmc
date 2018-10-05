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
  for(int i=0;i<ECMC_MAX_PLC_VARIABLES;i++){
    dataArray_[i]=NULL;
  }
}

int ecmcPLC::setExpression(char *exprStr)
{
  exprStr_=exprStr;
  compiled_=false;

  // Expression cleared (Allow ";" as empty expression)
  if(exprStr_.length()<=1){
    return 0;
  }

  int errorCode=parseExpression(exprStr);

  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  errorCode=compile();
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  return 0;
}

int ecmcPLC::parseExpression(char * exprStr)
{
  variableCount_=0;
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

  return 0;
}

int ecmcPLC::addAxisVar(int axisId, char *axisVarStr)
{
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

int ecmcPLC::compile()
{
  if(exprtk_->compile(exprStr_)){
    compiled_=false;
    LOGERR("%s/%s:%d: Error: Transform compile error: %s.\n",__FILE__, __FUNCTION__, __LINE__,exprtk_->getParserError().c_str());
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_COMPILE_ERROR);
  }
  printf("Compile OK expression %s\n",exprStr_.c_str());
  compiled_=true;
  return 0;
}

bool ecmcPLC::getCompiled()
{
  return compiled_;
}

int ecmcPLC::refresh()
{
  if(exprtk_==NULL){
    return 0;
  }

  //Update data from sources
  for(int i=0; i<variableCount_;i++){
    if(dataArray_[i]){
      dataArray_[i]->read();
    }
  }

  // Run equation
  exprtk_->refresh();

  //Update changed data
  for(int i=0; i<variableCount_;i++){
    if(dataArray_[i]){
      dataArray_[i]->write();
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
  return &exprStr_;
}
