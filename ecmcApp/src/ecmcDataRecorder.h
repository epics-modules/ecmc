/*
 * ecmcDataRecorder.h
 *
 *  Created on: Jun 1, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCDATARECORDER_H_
#define ECMCDATARECORDER_H_

#include "stdio.h"

#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "ecmcDataStorage.h"
#include "ecmcEcEntry.h"
#include "ecmcEvent.h"
#include "ecmcEventConsumer.h"

//Data recorder
#define ERROR_DATA_RECORDER_BUFFER_NULL 0x20100
#define ERROR_DATA_RECORDER_DATA_ECENTRY_NULL 0x20101
#define ERROR_DATA_RECORDER_ECENTRY_READ_FAIL 0x20102
#define ERROR_DATA_RECORDER_EVENT_NULL 0x20103
#define ERROR_DATA_RECORDER_HARDWARE_STATUS_NOT_OK 0x20104

class ecmcDataRecorder : public ecmcEventConsumer ,public ecmcEcEntryLink
{
public:
  ecmcDataRecorder (int index);
  ~ecmcDataRecorder ();
  int setExecute(int execute);
  int setDataStorage(ecmcDataStorage* buffer);
  int validate();
  int setEnablePrintOuts(bool enable);
  int executeEvent(int masterOK);//Override ecmcEventConsumer
private:
  void initVars();
  void printStatus();
  void setInStartupPhase(bool startup);
  ecmcDataStorage* dataBuffer_;
  int enableDiagnosticPrintouts_;
  int index_;
  int execute_;
  uint64_t data_;
  int inStartupPhase_;
};

#endif /* ECMCDATARECORDER_H_ */
