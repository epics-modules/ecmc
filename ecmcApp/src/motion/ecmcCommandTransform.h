/*
 * cMcuTransform.h
 *
 *  Created on: Mar 4, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCCOMMANDTRANSFORM_H_
#define ECMCCOMMANDTRANSFORM_H_

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

#include "exprtkWrap.h"
#include "../general/ecmcDefinitions.h"
#include "../general/ecmcError.h"
#include "../com/cmd.h"

#define ERROR_TRANSFORM_EXPR_NOT_COMPILED 0x30000
#define ERROR_TRANSFORM_INPUT_INDEX_OUT_OF_RANGE 0x30001
#define ERROR_TRANSFORM_COMPILE_ERROR 0x30002
#define ERROR_TRANSFORM_INPUT_DATA_SOURCE_NULL 0x30003
#define ERROR_TRANSFORM_ERROR_ADD_VARIABLE 0x30004
#define ERROR_TRANSFORM_INPUT_DATA_SOURCE_COUNT_ZERO 0x30005
#define ERROR_TRANSFORM_NOT_ENABLED 0x30006
#define ERROR_TRANSFORM_VECTOR_ALLOCATION_FAILED 0x30007
#define ERROR_TRANSFORM_EXPRTK_ALLOCATION_FAILED 0x30008

class ecmcCommandTransform : public ecmcError {
 public:
  ecmcCommandTransform(int commandCount,
                       int elementsPerCommand);
  ~ecmcCommandTransform();
  int          setExpression(std::string expressionString);
  int          addCmdPrefix(std::string commandPrefix,
                            int         commandIndex);
  int          setData(double data,
                       int    commandIndex,
                       int    index);
  double       getData(int commandIndex,
                       int index);
  bool         getDataChanged(int commandIndex,
                              int index);
  bool         getCompiled();
  int          validate();
  int          refresh();
  std::string* getExpression();
  void         printCurrentState();

 private:
  void         initVars();
  int          compile();
  std::string expressionString_;
  std::vector<double>inputArray_;
  std::vector<double>outputArray_;
  bool compiled_;
  int commandCount_;
  int elementsPerCommand_;
  exprtkWrap *exprtk_;
};

#endif  /* ECMCCOMMANDTRANSFORM_H_ */
