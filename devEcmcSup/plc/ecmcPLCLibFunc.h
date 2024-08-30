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
#include "ecmcError.h"

class ecmcPLCLibFunc {
public:
  explicit ecmcPLCLibFunc(std::string fucntionStr);
  ~ecmcPLCLibFunc();

private:

  std::string expression_;
  std::vector<std::string> args_;
};

#endif  /* ECMC_PLC_LIB_FUNC_H_ */
