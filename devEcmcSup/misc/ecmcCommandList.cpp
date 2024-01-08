/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcCommadList.cpp
*
*  Created on: Jun 2, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcCommandList.h"

ecmcCommandList::ecmcCommandList(int index) {
  PRINT_ERROR_PATH("commandList[%d].error", index);
  initVars();
  index_ = index;
  LOGINFO8("%s/%s:%d: commandList[%d]=new;\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index);
  printCurrentState();
}

ecmcCommandList::~ecmcCommandList() {
  clearCommandList();
}

void ecmcCommandList::printCurrentState() {
  LOGINFO8("%s/%s:%d: commandList[%d].commands=%d;\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index_,
           commandCounter_);
  LOGINFO8("%s/%s:%d: commandList[%d].commands=%d;\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index_,
           commandCounter_);
  LOGINFO8("%s/%s:%d: commandList[%d].enable=%d;\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index_,
           enable_ > 0);
}

void ecmcCommandList::initVars() {
  errorReset();
  commandCounter_ = 0;
  clearCommandList();
  enable_ = false;

  try {
    commandList_.reserve(ECMC_MAX_COMMANDS_IN_COMMANDS_LISTS);
  }
  catch (std::exception& e) {
    setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_COMMAND_LIST_VECTOR_ALLOCATION_FAILED);
    LOGINFO8(
      "%s/%s:%d: Error: Command list %d. Exception in allocation of vector: %s. Error number: 0x%x\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index_,
      e.what(),
      ERROR_COMMAND_LIST_VECTOR_ALLOCATION_FAILED);
  }
}

int ecmcCommandList::executeEvent(int masterOK) {
  // TODO consider running this in low prio thread in order to
  // not disturbe realtime

  if (getError() || !enable_) {
    return getErrorID();
  }

  clearBuffer(&resultBuffer_);

  for (unsigned int i = 0; i < commandList_.size(); i++) {
    LOGINFO8("%s/%s:%d: INFO: Command list %d. Executing command %s.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             index_,
             commandList_[i].c_str());
    int errorCode = motorHandleOneArg(commandList_[i].c_str(), &resultBuffer_);

    if (errorCode) {
      LOGINFO8(
        "%s/%s:%d: ERROR: Command %s resulted in buffer overflow error: %s.\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        commandList_[i].c_str(),
        resultBuffer_.buffer);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_COMMAND_LIST_COMMAND_RETURN_VALUE_NOT_OK);
    }

    LOGINFO8("%s/%s:%d: INFO: Command %s returned: %s.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             commandList_[i].c_str(),
             resultBuffer_.buffer);

    // Check return value
    if (strcmp(resultBuffer_.buffer, "OK")) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_COMMAND_LIST_COMMAND_RETURN_VALUE_NOT_OK);
    }
    clearBuffer(&resultBuffer_);
  }
  return 0;
}

int ecmcCommandList::clearCommandList() {
  commandList_.clear();
  commandCounter_ = 0;
  LOGINFO8("%s/%s:%d: INFO: Command list %d cleared.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index_);
  LOGINFO8("%s/%s:%d: commandList[%d].commands=%d;\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index_,
           commandCounter_);
  return 0;
}

int ecmcCommandList::addCommand(std::string command) {
  if (commandCounter_ >= ECMC_MAX_COMMANDS_IN_COMMANDS_LISTS) {
    LOGINFO8("%s/%s:%d: ERROR: Command list %d. Command list full (0x%x).\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             index_,
             ERROR_COMMAND_LIST_VECTOR_FULL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_COMMAND_LIST_VECTOR_FULL);
  }

  try {
    commandList_.push_back(command);
  }
  catch (std::exception& e) {
    LOGINFO8(
      "%s/%s:%d: ERROR: Command list %d. Add command failed. Exception: %s (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index_,
      e.what(),
      ERROR_COMMAND_LIST_VECTOR_ALLOCATION_FAILED);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_COMMAND_LIST_VECTOR_ALLOCATION_FAILED);
  }
  commandCounter_++;

  LOGINFO8("%s/%s:%d: commandList[%d].command[%d]=%s;\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index_,
           commandCounter_,
           command.c_str());
  LOGINFO8("%s/%s:%d: commandList[%d].commands=%d;\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index_,
           commandCounter_);

  return 0;
}

int ecmcCommandList::getCommandCount() {
  return commandCounter_;
}

int ecmcCommandList::validate() {
  return 0;
}

int ecmcCommandList::setEnable(int enable) {
  if (enable_ != enable) {
    LOGINFO8("%s/%s:%d: commandList[%d].enable=%d;\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             index_,
             enable > 0);
  }
  enable_ = enable;
  validate();

  return 0;
}

void ecmcCommandList::printStatus() {
  for (unsigned int i = 0; i < commandList_.size(); i++) {
    LOGINFO8("%s/%s:%d: Command index: %d, command: %s, Error: 0x%x.\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             i,
             commandList_[i].c_str(),
             getErrorID());
  }
}
