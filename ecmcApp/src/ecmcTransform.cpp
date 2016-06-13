/*
 * ecmcTransform.cpp
 *
 *  Created on: Mar 4, 2016
 *      Author: anderssandstrom
 */

#include "ecmcTransform.h"

ecmcTransform::ecmcTransform()
{
  initVars();
}

ecmcTransform::ecmcTransform(std::string expressionString, double sampleTime)
{
  initVars();
  sampleTime_=sampleTime;
  expressionString_=expressionString;
}

void ecmcTransform::initVars()
{
  errorReset();
  expressionString_="";
  for(int i=0;i<MAX_TRANSFORM_INPUTS;i++){
    inputArray_[i]=0;
  }
  compiled_=false;
  output_=0;
  outputOld_=0;
  diffOutput_=0;
  sampleTime_=0;
  initCounter_=0;
  diffOutOK_=false;
  //_bEnable=false;
  interlock_=1;  //OK
  dataSource_=ECMC_DATA_SOURCE_INTERNAL;
  varsRegistered_=false;
}

int ecmcTransform::setExpression(std::string expressionString)
{
  expressionString_=expressionString;
  compiled_=false;

  int errorCode=compile();
  if(errorCode){
    return setErrorID(errorCode);
  }

  return 0;
}

ecmcTransform::~ecmcTransform()
{

}

int ecmcTransform::compile()
{
  if(!varsRegistered_){
    symbolTable.add_constants();

    //Trajectory
    for(int i=0;i<ECMC_MAX_AXES;i++){
      char buffer [100];
      snprintf ( buffer, 100,"%s%d",TRANSFORM_EXPR_INPUT_TRAJ_VAR_NAME_PREFIX,i);
      if(!symbolTable.add_variable(buffer,inputArray_[i])){  //..TODO not nice with *2 in index. could go outside if defs are changed
        return ERROR_TRANSFORM_ERROR_ADD_VARIABLE;
      }
    }

    //Encoder
    for(int i=0;i<ECMC_MAX_AXES;i++){
      char buffer [100];
      snprintf ( buffer, 100,"%s%d",TRANSFORM_EXPR_INPUT_ENC_VAR_NAME_PREFIX,i);
      if(!symbolTable.add_variable(buffer,inputArray_[i+ECMC_MAX_AXES])){  //..TODO not nice with *2 in index. could go outside if defs are changed
        return ERROR_TRANSFORM_ERROR_ADD_VARIABLE;
      }
    }
     //add interlock variable
    if(!symbolTable.add_variable(TRANSFORM_EXPR_INTERLOCK_VAR_NAME,interlock_)){
      return ERROR_TRANSFORM_ERROR_ADD_VARIABLE;
    }

    //add output variable
    if(!symbolTable.add_variable(TRANSFORM_EXPR_OUTPUT_VAR_NAME,output_)){
      return ERROR_TRANSFORM_ERROR_ADD_VARIABLE;
    }
    varsRegistered_=true;
    expression_.register_symbol_table(symbolTable);
  }

  if(!parser_.compile(expressionString_,expression_)){
    compiled_=false;
    printf("*******TRANSFORM COMPILE ERROR: %s\n",parser_.error().c_str());
    return ERROR_TRANSFORM_COMPILE_ERROR;
  }
  compiled_=true;
  return 0;
}

bool ecmcTransform::getCompiled()
{
  return compiled_;
}

int ecmcTransform::refresh()
{
  if(!compiled_){
    return setErrorID(ERROR_TRANSFORM_EXPR_NOT_COMPILED);
  }

  for(int i=0;i<MAX_TRANSFORM_INPUTS;i++){
    if(inputData_[i]!=NULL){
      inputArray_[i]=inputData_[i]->getPosition();
    }
  }
  outputOld_=output_;
  expression_.value(); //_dOutput is linked to "out" variable.

  diffOutput_=(output_-outputOld_)/sampleTime_;

  if(initCounter_<2){
    initCounter_++;
  }
  else{
    diffOutOK_=true;
  }

  return 0;
}

int ecmcTransform::getOutput(double *val)
{
  *val=output_;
  return 0;
}

int ecmcTransform::getDiffOutput(double *val)
{
  if(diffOutOK_){
    *val=diffOutput_;
  }
  else{
    *val=0;
  }
  return 0;
}

double ecmcTransform::getSampleTime()
{
 return sampleTime_;
}

void ecmcTransform::setSampleTime(double sampleTime)
{
  sampleTime_=sampleTime;
}

int ecmcTransform::addInputDataObject(ecmcMasterSlaveData *data,int index)
{
  if(index>=MAX_TRANSFORM_INPUTS && index<0){
    return setErrorID(ERROR_TRANSFORM_INPUT_INDEX_OUT_OF_RANGE);
  }
  inputData_[index]=data;
  return 0;
}

int ecmcTransform::validate()
{
  if(dataSource_!=ECMC_DATA_SOURCE_INTERNAL && !compiled_){
    return setErrorID(ERROR_TRANSFORM_EXPR_NOT_COMPILED);
  }
  return 0;
}

bool ecmcTransform::getInterlock()
{
  return interlock_>0;
}

int ecmcTransform::setDataSource(dataSource data_source)
{
  dataSource_=data_source;
  return 0;
}

std::string *ecmcTransform::getExpression()
{
  return &expressionString_;
}
