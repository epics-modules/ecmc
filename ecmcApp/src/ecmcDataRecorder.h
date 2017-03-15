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

//Data recorder
#define ERROR_DATA_RECORDER_BUFFER_NULL 0x20100
#define ERROR_DATA_RECORDER_DATA_ECENTRY_NULL 0x20101
#define ERROR_DATA_RECORDER_ECENTRY_READ_FAIL 0x20102
#define ERROR_DATA_RECORDER_EVENT_NULL 0x20103
#define ERROR_DATA_RECORDER_HARDWARE_STATUS_NOT_OK 0x20104
#define ERROR_DATA_RECORDER_NO_DATA_SOURCE_CHOOSEN 0x20105
#define ERROR_DATA_RECORDER_AXIS_DATA_NULL 0x20106
#define ERROR_DATA_RECORDER_AXIS_DATA_TYPE_NOT_CHOOSEN 0x20107

enum ecmcAxisDataRecordType{
  ECMC_RECORDER_AXIS_DATA_NONE=0,
  ECMC_RECORDER_AXIS_DATA_AXIS_ID=1,
  ECMC_RECORDER_AXIS_DATA_POS_SET=2,
  ECMC_RECORDER_AXIS_DATA_POS_ACT=3,
  ECMC_RECORDER_AXIS_DATA_CNTRL_ERROR=4,
  ECMC_RECORDER_AXIS_DATA_POS_TARGET=5,
  ECMC_RECORDER_AXIS_DATA_POS_ERROR=6,
  ECMC_RECORDER_AXIS_DATA_POS_RAW=7,
  ECMC_RECORDER_AXIS_DATA_CNTRL_OUT=8,
  ECMC_RECORDER_AXIS_DATA_VEL_SET=9,
  ECMC_RECORDER_AXIS_DATA_VEL_ACT=10,
  ECMC_RECORDER_AXIS_DATA_VEL_SET_FF_RAW=11,
  ECMC_RECORDER_AXIS_DATA_VEL_SET_RAW=12,
  ECMC_RECORDER_AXIS_DATA_CYCLE_COUNTER=13,
  ECMC_RECORDER_AXIS_DATA_ERROR=14,
  ECMC_RECORDER_AXIS_DATA_COMMAND=15,
  ECMC_RECORDER_AXIS_DATA_CMD_DATA=16,
  ECMC_RECORDER_AXIS_DATA_SEQ_STATE=17,
  ECMC_RECORDER_AXIS_DATA_INTERLOCK_TYPE=18,
  ECMC_RECORDER_AXIS_DATA_TRAJ_SOURCE=19,
  ECMC_RECORDER_AXIS_DATA_ENC_SOURCE=20,
  ECMC_RECORDER_AXIS_DATA_ENABLE=21,
  ECMC_RECORDER_AXIS_DATA_ENABLED=22,
  ECMC_RECORDER_AXIS_DATA_EXECUTE=23,
  ECMC_RECORDER_AXIS_DATA_BUSY=24,
  ECMC_RECORDER_AXIS_DATA_AT_TARGET=25,
  ECMC_RECORDER_AXIS_DATA_HOMED=26,
  ECMC_RECORDER_AXIS_DATA_LIMIT_BWD=27,
  ECMC_RECORDER_AXIS_DATA_LIMIT_FWD=28,
  ECMC_RECORDER_AXIS_DATA_HOME_SWITCH=29,
};

enum ecmcDataSourceType{
  ECMC_RECORDER_SOURCE_NONE=0,
  ECMC_RECORDER_SOURCE_ETHERCAT=1,
  ECMC_RECORDER_SOURCE_AXIS=2,
};

class ecmcDataRecorder : public ecmcEventConsumer ,public ecmcEcEntryLink
{
public:
  ecmcDataRecorder (int index);
  ~ecmcDataRecorder ();
  int setEnable(int enable);
  int getEnabled(int *enabled);
  int setDataStorage(ecmcDataStorage* buffer);
  int validate();
  int executeEvent(int masterOK);//Override ecmcEventConsumer
  int setAxisDataSource(ecmcAxisStatusType *axisData,ecmcAxisDataRecordType dataTypeToRecord);
  int setDataSourceType(ecmcDataSourceType type);
private:
  void initVars();
  void printStatus();
  void setInStartupPhase(bool startup);
  int getData(double *data);
  int getAxisData(double *data);
  int getEtherCATData(double *data);
  ecmcDataStorage* dataBuffer_;
  int index_;
  int enable_;
  double data_;
  int inStartupPhase_;
  ecmcAxisStatusType *axisData_;
  ecmcAxisDataRecordType axisDataTypeToRecord_;
  ecmcDataSourceType dataSource_;
};

#endif /* ECMCDATARECORDER_H_ */
