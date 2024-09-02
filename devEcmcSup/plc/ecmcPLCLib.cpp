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

#define ECMC_PLC_FUNC_CALL_STR "function"

#include "ecmcPLCLib.h"
#include "ecmcErrorsList.h"
#include <stack>

ecmcPLCLib::ecmcPLCLib(std::string filename) {
  filename_ = filename;
  funcs_.clear();
  counter_ = 0;
  parseFile(filename_);
}

ecmcPLCLib::~ecmcPLCLib() {
}

// Function to trim leading and trailing whitespaces
std::string ecmcPLCLib::trim(const std::string& str) {
    size_t first = str.find_first_not_of(' ');
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
}
 
// Function to find function bodies with nested braces
std::string ecmcPLCLib::extractFunctionBody(const std::string& code, size_t startPos) {
    std::stack<char> braces;
    size_t pos = startPos;
    while (pos < code.length()) {
        if (code[pos] == '{') {
            braces.push('{');
        } else if (code[pos] == '}') {
            braces.pop();
            if (braces.empty()) {
                return code.substr(startPos, pos - startPos + 1);
            }
        }
        pos++;
    }
    return ""; // Return empty if no complete function body is found
}

void ecmcPLCLib::parseFile(std::string filename) {
  std::ifstream plcLibFile;
  plcLibFile.open(filename);

  if (!plcLibFile.good()) {
    throw std::runtime_error ("PLC-lib file not found.");
  }

  std::stringstream code;
  code << plcLibFile.rdbuf();
  std::vector<std::string> function_names;
  std::vector<std::string> parameter_lists;
  std::vector<std::string> function_bodies;
 
  size_t pos = 0;
  while ((pos = code.str().find(ECMC_PLC_FUNC_CALL_STR, pos)) != std::string::npos) {
    // Extract function name
    size_t nameStart = pos + 8;
    size_t nameEnd = code.str().find('(', nameStart);
    std::string function_name = trim(code.str().substr(nameStart, nameEnd - nameStart));
 
    // Extract parameters
    size_t paramsStart = nameEnd + 1;
    size_t paramsEnd = code.str().find(')', paramsStart);
    std::string parameters = trim(code.str().substr(paramsStart, paramsEnd - paramsStart));
 
    // Extract function body
    size_t bodyStart = code.str().find('{', paramsEnd);
    std::string function_body = extractFunctionBody(code.str(), bodyStart);
 
    ecmcPLCLibFunc * func= new ecmcPLCLibFunc(function_name,parameters,function_body);
    funcs_.push_back(func); 
    counter_ ++;
    pos = bodyStart + function_body.length();
  }
  
  return;
}
