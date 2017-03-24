/*
 * ecmcCommadList.h
 *
 *  Created on: Jun 2, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCCOMMANDLIST_H_
#define ECMCCOMMANDLIST_H_

#include "stdio.h"
#include <string>
#include <vector>

#include "ecmcError.h"
#include "ecmcDefinitions.h"
#include "ecmcEventConsumer.h"
#include "cmd_EAT.h"
#include "cmd.h"


//Command List
#define ERROR_COMMAND_LIST_NULL 0x20400
#define ERROR_COMMAND_LIST_INDEX_OUT_OF_RANGE 0x20401
#define ERROR_COMMAND_LIST_COMMAND_WRITE_FAILED 0x20402
#define ERROR_COMMAND_LIST_COMMAND_READ_FAILED 0x20403
#define ERROR_COMMAND_LIST_COMMAND_RETURN_VALUE_NOT_OK 0x20404
#define ERROR_COMMAND_LIST_VECTOR_ALLOCATION_FAILED 0x20405
#define ERROR_COMMAND_LIST_VECTOR_FULL 0x20406
#define ERROR_COMMAND_LIST_RESULT_BUFFER_OVERFLOW 0x20407

class ecmcCommandList : public ecmcEventConsumer, public ecmcError
{
public:
  ecmcCommandList (int index);
  ~ecmcCommandList ();
  int setEnable(int enable);
  int validate();
  int executeEvent(int masterOK);//Override ecmcEventConsumer
  int addCommand(std::string command);
  int clearCommandList();
  int getCommandCount();
  void printCurrentState();
private:
  void initVars();
  void printStatus();
  std::vector<std::string> commandList_;
  int commandCounter_;
  int enable_;
  int index_;
  ecmcOutputBufferType resultBuffer_;
};

#endif /* ECMCCOMMANDLIST_H_ */
