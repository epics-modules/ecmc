/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPLCTask_libDs.inc
*
*  Created on: Nov 19, 2018
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ecmcPLC_libDs_inc_
#define ecmcPLC_libDs_inc_

#define CHECK_PLC_DATA_STORAGE_RETURN_IF_ERROR(dsIndex) {           \
    ds_errorCode = 0;                                               \
    if (dsIndex >= ECMC_MAX_DATA_STORAGE_OBJECTS || dsIndex <= 0) { \
      ds_errorCode = ERROR_PLC_DATA_STORAGE_INDEX_OUT_OF_RANGE;     \
      LOGERR("ERROR: Data Storage index out of range.\n");          \
      return (double)ERROR_PLC_DATA_STORAGE_INDEX_OUT_OF_RANGE;     \
    }                                                               \
    if (ecmcPLCTask::statDs_[dsIndex] == NULL) {                        \
      ds_errorCode = ERROR_PLC_DATA_STORAGE_OBJECT_NULL;            \
      LOGERR("ERROR: Data Storage object NULL\n");                  \
      return (double)ERROR_PLC_DATA_STORAGE_OBJECT_NULL;            \
    }                                                               \
}                                                                   \

#define CHECK_PLC_DATA_STORAGE_RETURN_ZERO_IF_ERROR(dsIndex) {      \
    ds_errorCode = 0;                                               \
    if (dsIndex >= ECMC_MAX_DATA_STORAGE_OBJECTS || dsIndex <= 0) { \
      ds_errorCode = ERROR_PLC_DATA_STORAGE_INDEX_OUT_OF_RANGE;     \
      LOGERR("ERROR: Data Storage index out of range.\n");          \
      return 0;                                                     \
    }                                                               \
    if (ecmcPLCTask::statDs_[dsIndex] == NULL) {                        \
      ds_errorCode = ERROR_PLC_DATA_STORAGE_OBJECT_NULL;            \
      LOGERR("ERROR: Data Storage object NULL\n");                  \
      return 0;                                                     \
    }                                                               \
}                                                                   \

#define CHECK_PLC_DATA_STORAGE_RETURN_MINUS_ONE_IF_ERROR(dsIndex) { \
    ds_errorCode = 0;                                               \
    if (dsIndex >= ECMC_MAX_DATA_STORAGE_OBJECTS || dsIndex <= 0) { \
      ds_errorCode = ERROR_PLC_DATA_STORAGE_INDEX_OUT_OF_RANGE;     \
      LOGERR("ERROR: Data Storage index out of range.\n");          \
      return -1;                                                    \
    }                                                               \
    if (ecmcPLCTask::statDs_[dsIndex] == NULL) {                        \
      ds_errorCode = ERROR_PLC_DATA_STORAGE_OBJECT_NULL;            \
      LOGERR("ERROR: Data Storage object NULL\n");                  \
      return -1;                                                    \
    }                                                               \
}                                                                   \

const char *dsLibCmdList[] = { "ds_append_data(",
                               "ds_clear_data(",
                               "ds_get_data(",
                               "ds_set_data(",
                               "ds_get_buff_id(",
                               "ds_set_buff_id(",
                               "ds_get_err(",
                               "ds_is_full(",
                               "ds_get_size(",
                               "ds_push_asyn(",
                               "ds_get_avg(",
                               "ds_get_min(",
                               "ds_get_max("
                               "ds_err_rst("
};

static int ds_errorCode = 0;
static int ds_cmd_count = sizeof(dsLibCmdList) / sizeof(dsLibCmdList[0]);

inline double ds_append_data(double dsIndex, double data) {
  int index = (int)dsIndex;

  CHECK_PLC_DATA_STORAGE_RETURN_IF_ERROR(index);
  ds_errorCode = ecmcPLCTask::statDs_[index]->appendData(data);
  return (double)ds_errorCode;
}

inline double ds_clear_data(double dsIndex) {
  int index = (int)dsIndex;

  CHECK_PLC_DATA_STORAGE_RETURN_IF_ERROR(index);
  ds_errorCode = ecmcPLCTask::statDs_[index]->clearBuffer();
  return (double)ds_errorCode;
}

inline double ds_get_data(double dsIndex, double dataIndex) {
  int index = (int)dsIndex;

  CHECK_PLC_DATA_STORAGE_RETURN_ZERO_IF_ERROR(index);
  double temp = 0;
  ds_errorCode =
    ecmcPLCTask::statDs_[index]->getDataElement(int(dataIndex), &temp);
  return temp;
}

inline double ds_set_data(double dsIndex, double dataIndex, double data) {
  int index = (int)dsIndex;

  CHECK_PLC_DATA_STORAGE_RETURN_IF_ERROR(index);
  ds_errorCode = ecmcPLCTask::statDs_[index]->setDataElement(int(dataIndex), data);
  return (double)ds_errorCode;
}

inline double ds_get_buff_id(double dsIndex) {
  int index = (int)dsIndex;

  CHECK_PLC_DATA_STORAGE_RETURN_MINUS_ONE_IF_ERROR(index);
  return ecmcPLCTask::statDs_[index]->getCurrentIndex();
}

inline double ds_set_buff_id(double dsIndex, double bufferIndex) {
  int index = (int)dsIndex;

  CHECK_PLC_DATA_STORAGE_RETURN_IF_ERROR(index);
  ds_errorCode = ecmcPLCTask::statDs_[index]->setCurrentPosition(bufferIndex);
  return (double)ds_errorCode;
}

inline double ds_get_err() {
  return (double)ds_errorCode;
}

inline double ds_is_full(double dsIndex) {
  int index = (int)dsIndex;

  CHECK_PLC_DATA_STORAGE_RETURN_MINUS_ONE_IF_ERROR(index);
  ds_errorCode = 0;
  return (double)ecmcPLCTask::statDs_[index]->isStorageFull();
}

inline double ds_get_size(double dsIndex) {
  int index = (int)dsIndex;

  CHECK_PLC_DATA_STORAGE_RETURN_MINUS_ONE_IF_ERROR(index);
  ds_errorCode = 0;
  return (double)ecmcPLCTask::statDs_[index]->getSize();
}

inline double ds_push_asyn(double dsIndex) {
  int index = (int)dsIndex;

  CHECK_PLC_DATA_STORAGE_RETURN_MINUS_ONE_IF_ERROR(index);
  ds_errorCode = 0;
  return (double)ecmcPLCTask::statDs_[index]->updateAsyn(1);
}

inline double ds_get_avg(double dsIndex) {
  int index = (int)dsIndex;

  CHECK_PLC_DATA_STORAGE_RETURN_ZERO_IF_ERROR(index);
  ds_errorCode = 0;
  return (double)ecmcPLCTask::statDs_[index]->getAvg();
}

inline double ds_get_min(double dsIndex) {
  int index = (int)dsIndex;

  CHECK_PLC_DATA_STORAGE_RETURN_ZERO_IF_ERROR(index);
  ds_errorCode = 0;
  return (double)ecmcPLCTask::statDs_[index]->getMin();
}

inline double ds_get_max(double dsIndex) {
  int index = (int)dsIndex;

  CHECK_PLC_DATA_STORAGE_RETURN_ZERO_IF_ERROR(index);
  ds_errorCode = 0;
  return (double)ecmcPLCTask::statDs_[index]->getMax();
}

inline double ds_err_rst() {
  
  ds_errorCode = 0;
  return (double) 0;
}

#endif  /* ecmcPLC_libDs_inc_ */
