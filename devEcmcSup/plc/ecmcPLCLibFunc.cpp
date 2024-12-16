/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcPLCLibFunc.cpp
*
*  Created on: Oct 4, 2018
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcPLCLibFunc.h"
#include "ecmcErrorsList.h"
#include <iostream>

ecmcPLCLibFunc::ecmcPLCLibFunc(std::string fucntionName,std::string params,std::string expression) {
  funcionName_ = fucntionName;
  params_      = params;
  expression_  = expression;
  paramsVector_.clear();
  std::stringstream ss(params_);
  std::string param;
  while (std::getline(ss, param, ',')) {
   paramsVector_.push_back(param);
  }
}

ecmcPLCLibFunc::~ecmcPLCLibFunc() {

}

// Function to trim leading and trailing whitespaces
std::string ecmcPLCLibFunc::trim(const std::string& str) {
  size_t first = str.find_first_not_of(' ');
  size_t last = str.find_last_not_of(' ');
  if( (last-first) > 0) {
    return str.substr(first, (last - first + 1));
  } else {
    return str;
  }
}

std::string ecmcPLCLibFunc::getFuncionName() {
  return funcionName_;
}

std::string ecmcPLCLibFunc::getExpression() {
  return expression_;
}

std::vector<std::string> &ecmcPLCLibFunc::getParams() {
  return paramsVector_;
}

size_t ecmcPLCLibFunc::getParamCount() {
  return paramsVector_.size();
}
