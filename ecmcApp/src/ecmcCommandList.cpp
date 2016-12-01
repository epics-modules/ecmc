/*
 * ecmcCommadList.cpp
 *
 *  Created on: Jun 2, 2016
 *      Author: anderssandstrom
 */

#include "ecmcCommandList.h"

ecmcCommandList::ecmcCommandList (int index)
{
  initVars();
  index_=index;
}

ecmcCommandList::~ecmcCommandList ()
{
  clearCommandList();
}

void ecmcCommandList::initVars()
{
  commandCounter_=0;
  clearCommandList();
  enableDiagnosticPrintouts_=false;;
  execute_=false;;

  try{
    commandList_.reserve(ECMC_MAX_COMMANDS_IN_COMMANDS_LISTS);
  }
  catch(std::exception &e)
  {
    setErrorID(ERROR_COMMAND_LIST_VECTOR_ALLOCATION_FAILED);
    PRINT_DIAG(("Command list: %d. Exception in allocation of vector: %s. Error number: %x\n",index_,e.what(),ERROR_COMMAND_LIST_VECTOR_ALLOCATION_FAILED));
  }
}

int ecmcCommandList::executeEvent(int masterOK) //Master state not critical for this function
{
  //TODO consider running this in low prio thread in order to not disturbe realtime
  //TODO add wait time between commands (must be separate thread in this case)

  if(getError() || !execute_){
    return getErrorID();
  }

  clearBuffer(&resultBuffer_);
//  int errorCode=0;
  for(unsigned int i=0; i < commandList_.size(); i++){

    PRINT_DIAG(("Command list: %d. Executing command: %s.\n",index_,commandList_[i].c_str()));
    int errorCode=motorHandleOneArg(commandList_[i].c_str(),&resultBuffer_);
    if(errorCode){
      PRINT_DIAG(("Command %s resulted in buffer overflow: %s\n",commandList_[i].c_str(),resultBuffer_.buffer));
      return setErrorID(ERROR_COMMAND_LIST_COMMAND_RETURN_VALUE_NOT_OK); //TODO change error code
    }

    //Check return value
    if (strcmp(resultBuffer_.buffer,"OK")) {
      PRINT_DIAG(("Command %s returned: %s\n",commandList_[i].c_str(),resultBuffer_.buffer));
      return setErrorID(ERROR_COMMAND_LIST_COMMAND_RETURN_VALUE_NOT_OK);
    }

    PRINT_DIAG(("Command list: %d. Command returned: %s.\n",index_,resultBuffer_.buffer));
    clearBuffer(&resultBuffer_);
  }
  return 0;
}

int ecmcCommandList::clearCommandList()
{
  commandList_.clear();
  commandCounter_=0;
  PRINT_DIAG(("Command list cleared.\n"))
  return 0;
}

int ecmcCommandList::addCommand(std::string command)
{
  if(commandCounter_>=ECMC_MAX_COMMANDS_IN_COMMANDS_LISTS){
    PRINT_DIAG(("Command list: %d. Command list full. Error number: %x\n",index_,ERROR_COMMAND_LIST_VECTOR_FULL));
    return setErrorID(ERROR_COMMAND_LIST_VECTOR_FULL);
  }

  try{
    commandList_.push_back(command);
  }
  catch(std::exception &e)
  {
    PRINT_DIAG(("Command list: %d. Add command failed. Exception: %s. Error number: %x\n",index_,e.what(),ERROR_COMMAND_LIST_VECTOR_ALLOCATION_FAILED));
    return setErrorID(ERROR_COMMAND_LIST_VECTOR_ALLOCATION_FAILED);
  }

  commandCounter_++;
  PRINT_DIAG(("Command list: %d. Command added: %s. Command count: %d\n",index_,command.c_str(),commandCounter_));
  return 0;
}

int ecmcCommandList::getCommandCount()
{
  return commandCounter_;
}

int ecmcCommandList::validate()
{
  return 0;
}

int ecmcCommandList::setExecute(int execute)
{
  execute_=execute;
  validate();
  PRINT_DIAG(("Command list: %d. Set command execute: %d. Command count: %d\n",index_,execute,commandCounter_));
  return 0;
}

int ecmcCommandList::setEnablePrintOuts(bool enable)
{
  enableDiagnosticPrintouts_=enable;
  return 0;
}

void ecmcCommandList::printStatus()
{
  for(unsigned int i=0; i < commandList_.size(); i++){
    PRINT_DIAG(("Command index: %d, command: %s, Error: %x\n",i,commandList_[i].c_str(),getErrorID()));
  }
}
