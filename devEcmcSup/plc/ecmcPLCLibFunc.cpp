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

ecmcPLCLibFunc::ecmcPLCLibFunc(std::string functionStr) {
  std::cout << functionStr;
}

ecmcPLCLibFunc::~ecmcPLCLibFunc() {
}
