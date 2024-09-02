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

  // Output the parsed results
  std::cout << "Function Name: " << funcionName_ << std::endl;
  std::cout << "Parameters: ";
  std::stringstream ss(params_);
  std::string param;
  while (std::getline(ss, param, ',')) {
    std::cout << trim(param) << " ";
    paramsVector_.push_back(param);
  }
  std::cout << std::endl;
  std::cout << "Function Body:\n" << expression_ << std::endl;
  std::cout << "---------------------" << std::endl;
  std::cout.flush();
}

ecmcPLCLibFunc::~ecmcPLCLibFunc() {

}

// Function to trim leading and trailing whitespaces
std::string ecmcPLCLibFunc::trim(const std::string& str) {
    size_t first = str.find_first_not_of(' ');
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
}

std::string ecmcPLCLibFunc::getFuncionName() {
  return funcionName_;
}

std::string ecmcPLCLibFunc::getExpression() {
  return expression_;
}

std::vector<std::string> ecmcPLCLibFunc::getParams() {
  return paramsVector_;
}

size_t ecmcPLCLibFunc::getParamCount() {
  return paramsVector_.size();
}
