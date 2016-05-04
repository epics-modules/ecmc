/*
 * cMcuCommandTransform.cpp
 *
 *  Created on: April 11, 2016
 *      Author: anderssandstrom
 */

#include "ecmcCommandTransform.h"

ecmcCommandTransform::  ecmcCommandTransform(int commandCount, int elementsPerCommand)
{
  initVars();
  commandCount_=commandCount;
  elementsPerCommand_=elementsPerCommand;
  inputArray_.resize(elementsPerCommand*commandCount);
  outputArray_.resize(elementsPerCommand*commandCount);
  for(int i=0;i<elementsPerCommand*commandCount;i++){
    inputArray_[i]=0;
    outputArray_[i]=0;
  }
  symbolTable_.add_constants();
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
      return setErrorID(ERROR_TRANSFORM_ERROR_ADD_VARIABLE);
    }
  }
  return 0;
}

int ecmcCommandTransform::setExpression(std::string expressionString)
{
  expressionString_=expressionString;
  compiled_=false;

  int errorCode=compile();
  if(errorCode){
    return setErrorID(errorCode);
  }

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
    printf("*******TRANSFORM COMPILE ERROR: %s\n",parser_.error().c_str());
    return setErrorID(ERROR_TRANSFORM_COMPILE_ERROR);
  }
  compiled_=true;
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
  return 0;
}

int ecmcCommandTransform::setData(double data,int commandIndex,int index)
{
  int totalIndex=commandIndex*elementsPerCommand_+index;
  if(commandIndex>=commandCount_ || commandIndex<0 || index>=elementsPerCommand_ || index<0 || totalIndex>=(int)outputArray_.size()){
    return setErrorID(ERROR_TRANSFORM_INPUT_INDEX_OUT_OF_RANGE);
  }

  inputArray_[totalIndex]=data;
  outputArray_[totalIndex]=data;
  return 0;
}

double ecmcCommandTransform::getData(int commandIndex,int index)
{
  int totalIndex=commandIndex*elementsPerCommand_+index;
  if(commandIndex>=commandCount_ || commandIndex<0 || index>=elementsPerCommand_ || index<0 || totalIndex>=(int)outputArray_.size()){
    setErrorID(ERROR_TRANSFORM_INPUT_INDEX_OUT_OF_RANGE);
    return 0;
  }

  return outputArray_[totalIndex];
}

bool ecmcCommandTransform::getDataChanged(int commandIndex,int index)
{
  int totalIndex=commandIndex*elementsPerCommand_+index;
  if(commandIndex>=commandCount_ || commandIndex<0 || index>=elementsPerCommand_ || index<0 || totalIndex>=(int)outputArray_.size()){
    return setErrorID(ERROR_TRANSFORM_INPUT_INDEX_OUT_OF_RANGE);
  }

  return outputArray_[totalIndex]!=inputArray_[totalIndex];
}


std::string *ecmcCommandTransform::getExpression()
{
  return &expressionString_;
}
