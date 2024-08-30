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
#include "ecmcPLCLibFunc.h"

#include "ecmcError.h"

class ecmcPLCLib {
public:
  explicit ecmcPLCLib(std::string filename);
  ~ecmcPLCLib();

private:
  void parseFile(std::string filename);
  std::string filename_;
  std::string libname_;
  std::vector<ecmcPLCLibFunc> funcs_;
};

#endif  /* ECMC_PLC_LIB_H_ */
