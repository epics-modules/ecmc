/*
 *  ecmcCommandTransform.cpp
 *
 *  Created on: April 11, 2016
 *      Author: anderssandstrom
 */

#include "ecmcCommandTransform.h"

ecmcCommandTransform::ecmcCommandTransform(int commandCount, int elementsPerCommand)
{
  PRINT_ERROR_PATH("commandTransfrom.error");
  LOGINFO7("%s/%s:%d: commandTransform=new;\n",__FILE__, __FUNCTION__, __LINE__);
  initVars();
  commandCount_=commandCount;
  elementsPerCommand_=elementsPerCommand;

  try{
    inputArray_.resize(elementsPerCommand*commandCount);
    outputArray_.resize(elementsPerCommand*commandCount);
  }
  catch(std::exception &e)
  {
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_VECTOR_ALLOCATION_FAILED);
    LOGINFO7("%s/%s:%d: INFO: Exception in allocation of vector: %s. Error number: %x\n",__FILE__, __FUNCTION__, __LINE__,e.what(),ERROR_TRANSFORM_VECTOR_ALLOCATION_FAILED);
  }

  for(int i=0;i<elementsPerCommand*commandCount;i++){
    inputArray_[i]=0;
    outputArray_[i]=0;
  }
  symbolTable_.add_constants();
  printCurrentState();
}

void ecmcCommandTransform::printCurrentState()
{
  LOGINFO7("%s/%s:%d: commandTransform.commandCount=%d;\n",__FILE__, __FUNCTION__, __LINE__,commandCount_);
  LOGINFO7("%s/%s:%d: commandTransform.elemenstPerCommand=%d;\n",__FILE__, __FUNCTION__, __LINE__,elementsPerCommand_);
  LOGINFO7("%s/%s:%d: commandTransform.compiled=%d;\n",__FILE__, __FUNCTION__, __LINE__,compiled_>0);
  LOGINFO7("%s/%s:%d: commandTransform.expression=%s;\n",__FILE__, __FUNCTION__, __LINE__,expressionString_.c_str());
}

void ecmcCommandTransform::initVars()
{
  errorReset();
  expressionString_="";
  compiled_=false;

  commandCount_=0;
  elementsPerCommand_=0;
}

int ecmcCommandTransform::addCmdPrefix(std::string commandPrefix, int commandIndex)
{
  for(int i=0;i<elementsPerCommand_;i++){
    char varNameBuffer [100];
    snprintf ( varNameBuffer, 100,"%s%d",commandPrefix.c_str(),i);
    if(!symbolTable_.add_variable(varNameBuffer,outputArray_[i+commandIndex*elementsPerCommand_])){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
    }
  }
  return 0;
}

int ecmcCommandTransform::setExpression(std::string expressionString)
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
  LOGINFO7("%s/%s:%d: commandTransform.expression=%s;\n",__FILE__, __FUNCTION__, __LINE__,expressionString_.c_str());
  return 0;
}

ecmcCommandTransform::~ecmcCommandTransform()
{

}

int ecmcCommandTransform::compile()
{
  expression_.register_symbol_table(symbolTable_);
  if(!parser_.compile(expressionString_,expression_)){
    compiled_=false;
    LOGINFO7("%s/%s:%d: Error: Transform compile error: %s.\n",__FILE__, __FUNCTION__, __LINE__,parser_.error().c_str());
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_COMPILE_ERROR);
  }
  compiled_=true;
  LOGINFO7("%s/%s:%d: commandTransform.compiled=%d;\n",__FILE__, __FUNCTION__, __LINE__,compiled_>0);
  return 0;
}

bool ecmcCommandTransform::getCompiled()
{
  return compiled_;
}

int ecmcCommandTransform::refresh()
{
  expression_.value();
  return 0;
}

int ecmcCommandTransform::validate()
{
  if(!compiled_){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_COMPILE_ERROR);
  }
  return 0;
}

int ecmcCommandTransform::setData(double data,int commandIndex,int index)
{
  int totalIndex=commandIndex*elementsPerCommand_+index;
  if(commandIndex>=commandCount_ || commandIndex<0 || index>=elementsPerCommand_ || index<0 || totalIndex>=(int)outputArray_.size()){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_INPUT_INDEX_OUT_OF_RANGE);
  }

  inputArray_[totalIndex]=data;
  outputArray_[totalIndex]=data;
  return 0;
}

double ecmcCommandTransform::getData(int commandIndex,int index)
{
  int totalIndex=commandIndex*elementsPerCommand_+index;
  if(commandIndex>=commandCount_ || commandIndex<0 || index>=elementsPerCommand_ || index<0 || totalIndex>=(int)outputArray_.size()){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_INPUT_INDEX_OUT_OF_RANGE);
    return 0;
  }

  return outputArray_[totalIndex];
}

bool ecmcCommandTransform::getDataChanged(int commandIndex,int index)
{
  int totalIndex=commandIndex*elementsPerCommand_+index;
  if(commandIndex>=commandCount_ || commandIndex<0 || index>=elementsPerCommand_ || index<0 || totalIndex>=(int)outputArray_.size()){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_TRANSFORM_INPUT_INDEX_OUT_OF_RANGE);
  }

  return outputArray_[totalIndex]!=inputArray_[totalIndex];
}


std::string *ecmcCommandTransform::getExpression()
{
  return &expressionString_;
}
