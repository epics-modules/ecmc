/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcPLCLib.cpp
*
*  Created on: Oct 4, 2018
*      Author: anderssandstrom
*
\*************************************************************************/

#define ECMC_PLC_FUNC_CALL_STR "function("

#include "ecmcPLCLib.h"
#include "ecmcErrorsList.h"

ecmcPLCLib::ecmcPLCLib(std::string libname,std::string filename) {
  filename_ = filename;
  libname_ = libname;
  funcs_.clear();
  parseFile(filename_);
}

ecmcPLCLib::~ecmcPLCLib() {
}

void ecmcPLCLibFunc::parseFile(std::string filename) {

  std::ifstream plcLibFile;
  plcLibFile.open(filename);

  if (!plcLibFile.good()) {
    throw std::runtime_error "PLC-lib file not found.");
  }

  std::string line, lineNoComments;
  int lineNumber = 1;
  int errorCode  = 0;
  int functionCounter = 0;
  std::string thisFunctionStr.clear();
  bool functionInProgress = false;
  while (std::getline(plcLibFile, line)) {
    // Remove Comments (everything after #)
    lineNoComments = line.substr(0, line.find(ECMC_PLC_FILE_COMMENT_CHAR));    
    if (lineNoComments.length() > 0) {
      lineNoComments.append("\n");
      lineNumber++;
    }

    if(lineNoComments.find(ECMC_PLC_FUNC_CALL_STR)==0) {
      printf("Function %s found at line %d\n",lineNoComments.c_str(),lineNumber);
      if(functionInProgress) {
        funcs_.append(new ecmcPLCLibFunc(thisFunctionStr));
        thisFunctionStr.clear();  // prepare for next function
        functionCounter++;
      }
      functionInProgress = true;
      thisFunctionStr.append(lineNoComments);
    }
  }
  // add last function if not done
  if(thisFunctionStr.size()>0) {
    funcs_.append(new ecmcPLCLibFunc(thisFunctionStr));
    thisFunctionStr.clear();  // prepare for next function
    functionCounter++;
  }
  std::cout << "Added " << functionCounter << " functions\n";
}

