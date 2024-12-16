/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcPLCLib.cpp
*
*  Created on: Aug 4, 2024
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMC_PLC_LIB_H_
#define ECMC_PLC_LIB_H_

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "ecmcPLCLibFunc.h"

#include "ecmcError.h"

class ecmcPLCLib {
public:
  explicit ecmcPLCLib(std::string filename);
  ~ecmcPLCLib();
  ecmcPLCLibFunc* getFunction(size_t index);
  size_t getFunctionCount();

private:
  std::string extractFunctionBody(const std::string& code, size_t startPos);
  std::string removeBraces(std::string& str);
  std::string removeLinesStartingWithHash(const std::string& input) ;
  void        parseFile(std::string filename);
  std::string filename_;
  std::string libname_;
  std::vector<ecmcPLCLibFunc*> funcs_;
  size_t counter_;
};

#endif  /* ECMC_PLC_LIB_H_ */
