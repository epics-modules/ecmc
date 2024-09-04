/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcPLCLibFunc.cpp
*
*  Created on: Aug 4, 2024
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMC_PLC_LIB_FUNC_H_
#define ECMC_PLC_LIB_FUNC_H_

#include <string>
#include <vector>
#include <stdexcept>
#include <sstream>
#include "ecmcError.h"

class ecmcPLCLibFunc {
public:
  explicit ecmcPLCLibFunc(std::string fucntionName,std::string params,std::string expression);
  ~ecmcPLCLibFunc();
  std::string getFuncionName();
  std::string getExpression();
  std::vector<std::string> &getParams();
  size_t getParamCount();
  static std::string trim(const std::string& str);
private:
  std::string funcionName_;
  std::string params_;
  std::vector<std::string> paramsVector_;
  std::string expression_;
};

#endif  /* ECMC_PLC_LIB_FUNC_H_ */
