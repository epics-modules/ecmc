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
#include "ecmcAxisBase.h"

// Data recorder
#define ERROR_DATA_RECORDER_BUFFER_NULL 0x20100
#define ERROR_DATA_RECORDER_DATA_ECENTRY_NULL 0x20101
#define ERROR_DATA_RECORDER_ECENTRY_READ_FAIL 0x20102
#define ERROR_DATA_RECORDER_EVENT_NULL 0x20103
#define ERROR_DATA_RECORDER_HARDWARE_STATUS_NOT_OK 0x20104
#define ERROR_DATA_RECORDER_NO_DATA_SOURCE_CHOOSEN 0x20105
#define ERROR_DATA_RECORDER_AXIS_DATA_NULL 0x20106
#define ERROR_DATA_RECORDER_AXIS_DATA_TYPE_NOT_CHOOSEN 0x20107

class ecmcDataRecorder : public ecmcEventConsumer, public ecmcEcEntryLink {
 public:
  explicit ecmcDataRecorder(int index);
  ~ecmcDataRecorder();
  int  setEnable(int enable);
  int  getEnabled(int *enabled);
  int  setDataStorage(ecmcDataStorage *buffer);
  int  validate();
  int  executeEvent(int masterOK);  // Override ecmcEventConsumer
  int  setAxisDataSource(ecmcAxisStatusType *axisData,
                         ecmcAxisDataType    dataTypeToRecord);
  int  setDataSourceType(ecmcDataSourceType type);
  void printCurrentState();

 private:
  void initVars();
  void printStatus();
  void setInStartupPhase(bool startup);
  int  getData(double *data);
  int  getAxisData(double *data);
  int  getEtherCATData(double *data);
  void printDataSource();
  void printAxisDataSource();
  ecmcDataStorage *dataBuffer_;
  int index_;
  int enable_;
  double data_;
  int inStartupPhase_;
  ecmcAxisStatusType *axisData_;
  ecmcAxisDataType axisDataTypeToRecord_;
  ecmcDataSourceType dataSource_;
};

#endif  /* ECMCDATARECORDER_H_ */
