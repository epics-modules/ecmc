/*
 * cMcuTransform.h
 *
 *  Created on: Mar 4, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCCOMMANDTRANSFORM_H_
#define ECMCCOMMANDTRANSFORM_H_

#include "exprtk.hpp"
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include "cmd.h"

#include "ecmcDefinitions.h"
#include "ecmcError.h"

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


class ecmcCommandTransform : public ecmcError
{
public:
  ecmcCommandTransform(int commandCount, int elementsPerCommand);
  ~ecmcCommandTransform();
  int setExpression(std::string expressionString);
  int addCmdPrefix(std::string commandPrefix, int commandIndex);
  int setData(double data,int commandIndex,int index);
  double getData(int commandIndex,int index);
  bool getDataChanged(int commandIndex,int index);
  bool getCompiled();
  int validate();
  int refresh();
  std::string *getExpression();
  void printCurrentState();
private:
  void initVars();
  int compile();
  symbol_table_double symbolTable_;
  expression_double expression_;
  parser_double parser_;
  std::string expressionString_;
  std::vector<double> inputArray_;
  std::vector<double> outputArray_;
  bool compiled_;
  int commandCount_;
  int elementsPerCommand_;

};

#endif /* ECMCCOMMANDTRANSFORM_H_ */
