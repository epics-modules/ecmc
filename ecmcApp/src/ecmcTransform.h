/*
 * ecmcTransform.h
 *
 *  Created on: Mar 4, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCTRANSFORM_H_
#define ECMCTRANSFORM_H_

#include "exprtk.hpp"
#include <stdio.h>
#include <iostream>
#include <string>

#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "ecmcMasterSlaveData.h"

//TRANSFORM
#define ERROR_TRANSFORM_EXPR_NOT_COMPILED 0x30000
#define ERROR_TRANSFORM_INPUT_INDEX_OUT_OF_RANGE 0x30001
#define ERROR_TRANSFORM_COMPILE_ERROR 0x30002
#define ERROR_TRANSFORM_INPUT_DATA_SOURCE_NULL 0x30003
#define ERROR_TRANSFORM_ERROR_ADD_VARIABLE 0x30004
#define ERROR_TRANSFORM_INPUT_DATA_SOURCE_COUNT_ZERO 0x30005
#define ERROR_TRANSFORM_NOT_ENABLED 0x30006
#define ERROR_TRANSFORM_VECTOR_ALLOCATION_FAILED 0x30007

typedef exprtk::symbol_table<double> symbol_table_double;
typedef exprtk::expression<double>   expression_double;
typedef exprtk::parser<double>       parser_double;


class ecmcTransform : public ecmcError
{
public:
  ecmcTransform();
  ecmcTransform(std::string expressionString, double sampleTime);
  ~ecmcTransform();
  int addInputDataObject(ecmcMasterSlaveData *data,int index);
  int setExpression(std::string expressionString);
  std::string *getExpression();
  bool getCompiled();
  int getOutput(double *val);
  int getDiffOutput(double *val);
  double getSampleTime();
  void setSampleTime(double sampleTime);
  int setDataSource(dataSource dataSource);
  int validate();
  bool getInterlock();
  int refresh();

private:
  void initVars();
  int compile();
  symbol_table_double symbolTable;
  expression_double expression_;
  parser_double parser_;
  std::string expressionString_;
  double inputArray_[MAX_TRANSFORM_INPUTS];
  bool compiled_;
  double output_;
  double diffOutput_;
  double outputOld_;
  double sampleTime_;
  int initCounter_;
  bool diffOutOK_;
  bool enable_;
  double interlock_;
  ecmcMasterSlaveData *inputData_[MAX_TRANSFORM_INPUTS];
  dataSource dataSource_;
  bool varsRegistered_;
};

#endif /* ECMCTRANSFORM_H_ */
