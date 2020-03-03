/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcMisc.cpp
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcMisc.h"

// TODO: REMOVE GLOBALS
#include "../main/ecmcGlobalsExtern.h"

int createEvent(int indexEvent) {
  LOGINFO4("%s/%s:%d indexEvent=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexEvent);

  if ((indexEvent >= ECMC_MAX_EVENT_OBJECTS) || (indexEvent < 0)) {
    return ERROR_MAIN_EVENT_INDEX_OUT_OF_RANGE;
  }

  delete events[indexEvent];
  events[indexEvent] = new ecmcEvent(1 / mcuFrequency, indexEvent);

  if (!events[indexEvent]) {
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR EVENT OBJECT.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    exit(EXIT_FAILURE);
  }
  return events[indexEvent]->getErrorID();
}

int createDataStorage(int index, int elements, int bufferType) {
  
  LOGINFO4("%s/%s:%d index=%d elements=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index,
           elements);

  if ((index >= ECMC_MAX_DATA_STORAGE_OBJECTS) || (index < 0)) {
    return ERROR_MAIN_DATA_STORAGE_INDEX_OUT_OF_RANGE;
  }

  if (elements <= 0) {
    return ERROR_MAIN_DATA_STORAGE_INVALID_SIZE;
  }

  delete dataStorages[index];
  dataStorages[index] = new ecmcDataStorage(asynPort, index,
                                            elements,
                                            (ecmcDSBufferType)bufferType);

  if (!dataStorages[index]) {
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR DATA STORAGE OBJECT.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    exit(EXIT_FAILURE);
  }

  return dataStorages[index]->getErrorID();
}

int linkStorageToRecorder(int indexStorage, int indexRecorder) {
  LOGINFO4("%s/%s:%d indexStorage=%d indexRecorder=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage,
           indexRecorder);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);
  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  return dataRecorders[indexRecorder]->setDataStorage(
    dataStorages[indexStorage]);
}

int linkEcEntryToEvent(int   indexEvent,
                       int   eventEntryIndex,
                       int   slaveIndex,
                       char *entryIDString,
                       int   bitIndex) {
  LOGINFO4(
    "%s/%s:%d indexEvent=%d eventEntryIndex=%d slave_index=%d entry=%s bitIndex=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    indexEvent,
    eventEntryIndex,
    slaveIndex,
    entryIDString,
    bitIndex);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave = NULL;

  if (slaveIndex >= 0) {
    slave = ec->findSlave(slaveIndex);
  } else {    // simulation slave
    slave = ec->getSlave(slaveIndex);
  }

  if (slave == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID = entryIDString;

  ecmcEcEntry *entry = slave->findEntry(sEntryID);

  if (entry == NULL) return ERROR_MAIN_EC_ENTRY_NULL;

  return events[indexEvent]->setEntryAtIndex(entry, eventEntryIndex, bitIndex);
}

int setEventType(int indexEvent, int type) {
  LOGINFO4("%s/%s:%d indexEvent=%d recordingType=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexEvent,
           type);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->setEventType((eventType)type);
}

int setEventSampleTime(int indexEvent, int sampleTime) {
  LOGINFO4("%s/%s:%d indexEvent=%d sampleTime=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexEvent,
           sampleTime);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->setDataSampleTime(sampleTime);
}

int setEventEnable(int indexEvent, int enable) {
  LOGINFO4("%s/%s:%d indexEvent=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexEvent,
           enable);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->setEnable(enable);
}

int getEventEnabled(int indexEvent, int *enabled) {
  LOGINFO4("%s/%s:%d indexEvent=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexEvent);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->getEnabled(enabled);
}

int clearStorage(int indexStorage) {
  LOGINFO4("%s/%s:%d indexStorage=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->clearBuffer();
}

int getStorageDataIndex(int indexStorage, int *index) {
  LOGINFO4("%s/%s:%d indexStorage=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  *index = dataStorages[indexStorage]->getCurrentIndex();
  return 0;
}

int setStorageEnablePrintouts(int indexStorage, int enable) {
  LOGINFO4("%s/%s:%d indexStorage=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage,
           enable);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  WRITE_DIAG_BIT(FUNCTION_DATA_STORAGE_DIAGNOSTICS_BIT, enable);
  return 0;
}

int printStorageBuffer(int indexStorage) {
  LOGINFO4("%s/%s:%d indexStorage=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->printBuffer();
}

int readStorageBuffer(int indexStorage, double **data, int *size) {
  LOGINFO4("%s/%s:%d indexStorage=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->getData(data, size);
}

int writeStorageBuffer(int indexStorage, double *data, int size) {
  LOGINFO4("%s/%s:%d indexStorage=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->setData(data, size);
}

int appendStorageBuffer(int indexStorage, double *data, int size) {
  LOGINFO4("%s/%s:%d indexStorage=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->appendData(data, size);
}

int setDataStorageCurrentDataIndex(int indexStorage, int position) {
  LOGINFO4("%s/%s:%d indexStorage=%d position=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexStorage,
           position);

  CHECK_STORAGE_RETURN_IF_ERROR(indexStorage);

  return dataStorages[indexStorage]->setCurrentPosition(position);
}

int setEventTriggerEdge(int indexEvent, int triggerEdge) {
  LOGINFO4("%s/%s:%d indexEvent=%d triggerEdge=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexEvent,
           triggerEdge);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->setTriggerEdge((triggerEdgeType)triggerEdge);
}

int setEventEnableArmSequence(int indexEvent, int enable) {
  LOGINFO4("%s/%s:%d indexEvent=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexEvent,
           enable);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->setEnableArmSequence(enable);
}

int setEventEnablePrintouts(int indexEvent, int enable) {
  LOGINFO4("%s/%s:%d indexEvent=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexEvent,
           enable);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  WRITE_DIAG_BIT(FUNCTION_EVENTS_DIAGNOSTICS_BIT, enable);
  return 0;
}

int triggerEvent(int indexEvent) {
  LOGINFO4("%s/%s:%d indexEvent=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexEvent);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->triggerEvent(ec->statusOK());
}

int armEvent(int indexEvent) {
  LOGINFO4("%s/%s:%d indexEvent=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexEvent);

  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->arm();
}

int createRecorder(int indexRecorder) {
  LOGINFO4("%s/%s:%d indexRecorder=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexRecorder);

  if ((indexRecorder >= ECMC_MAX_DATA_RECORDERS_OBJECTS) ||
      (indexRecorder < 0)) {
    return ERROR_MAIN_DATA_RECORDER_INDEX_OUT_OF_RANGE;
  }

  delete dataRecorders[indexRecorder];
  dataRecorders[indexRecorder] = new ecmcDataRecorder(indexRecorder);

  if (!dataRecorders[indexRecorder]) {
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR DATA RECORDER OBJECT.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    exit(EXIT_FAILURE);
  }

  return dataRecorders[indexRecorder]->getErrorID();
}

int linkEcEntryToRecorder(int   indexRecorder,
                          int   recorderEntryIndex,
                          int   slaveIndex,
                          char *entryIDString,
                          int   bitIndex) {
  LOGINFO4(
    "%s/%s:%d indexRecorder=%d recorderEntryIndex=%d slave_index=%d entry=%s bitIndex=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    indexRecorder,
    recorderEntryIndex,
    slaveIndex,
    entryIDString,
    bitIndex);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  ecmcEcSlave *slave = NULL;

  if (slaveIndex >= 0) {
    slave = ec->findSlave(slaveIndex);
  } else {    // simulation slave
    slave = ec->getSlave(slaveIndex);
  }

  if (slave == NULL) return ERROR_MAIN_EC_SLAVE_NULL;

  std::string sEntryID = entryIDString;

  ecmcEcEntry *entry = slave->findEntry(sEntryID);

  if (entry == NULL) return ERROR_MAIN_EC_ENTRY_NULL;

  int error = dataRecorders[indexRecorder]->setEntryAtIndex(entry,
                                                            recorderEntryIndex,
                                                            bitIndex);

  if (error) {
    return error;
  }

  // set source to EtherCAT
  return dataRecorders[indexRecorder]->setDataSourceType(
    ECMC_RECORDER_SOURCE_ETHERCAT);
}

int linkAxisDataToRecorder(int indexRecorder, int axisIndex, int dataToStore) {
  LOGINFO4("%s/%s:%d indexRecorder=%d axisIndex=%d dataToStore=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexRecorder,
           axisIndex,
           dataToStore);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);
  CHECK_AXIS_RETURN_IF_ERROR(axisIndex);

  int error = dataRecorders[indexRecorder]->setAxisDataSource(
    axes[axisIndex]->getDebugInfoDataPointer(),
    (ecmcAxisDataType)dataToStore);

  if (error) {
    return error;
  }

  // set source to Axis data
  return dataRecorders[indexRecorder]->setDataSourceType(
    ECMC_RECORDER_SOURCE_AXIS);
}

int setRecorderEnablePrintouts(int indexRecorder, int enable) {
  LOGINFO4("%s/%s:%d indexRecorder=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexRecorder,
           enable);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  WRITE_DIAG_BIT(FUNCTION_DATA_RECORDER_DIAGNOSTICS_BIT, enable);
  return 0;
}

int setRecorderEnable(int indexRecorder, int enable) {
  LOGINFO4("%s/%s:%d indexRecorder=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexRecorder,
           enable);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  return dataRecorders[indexRecorder]->setEnable(enable);
}

int getRecorderEnabled(int indexRecorder, int *enabled) {
  LOGINFO4("%s/%s:%d indexRecorder=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexRecorder);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  return dataRecorders[indexRecorder]->getEnabled(enabled);
}

int linkRecorderToEvent(int indexRecorder, int indexEvent, int consumerIndex) {
  LOGINFO4("%s/%s:%d indexRecorder=%d indexEvent=%d consumerIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexRecorder,
           indexEvent,
           consumerIndex);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);
  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);
  return events[indexEvent]->linkEventConsumer(dataRecorders[indexRecorder],
                                               consumerIndex);
}

int triggerRecorder(int indexRecorder) {
  LOGINFO4("%s/%s:%d indexRecorder=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexRecorder);

  CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder);

  return dataRecorders[indexRecorder]->executeEvent(ec->statusOK());
}


int createCommandList(int indexCommandList) {
  LOGINFO4("%s/%s:%d indexCommandList=%d \n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexCommandList);

  if ((indexCommandList >= ECMC_MAX_COMMANDS_LISTS) ||
      (indexCommandList < 0)) {
    return ERROR_COMMAND_LIST_INDEX_OUT_OF_RANGE;
  }

  delete commandLists[indexCommandList];
  commandLists[indexCommandList] = new ecmcCommandList(indexCommandList);

  if (!commandLists[indexCommandList]) {
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR COMAMND-LIST OBJECT.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    exit(EXIT_FAILURE);
  }

  return commandLists[indexCommandList]->getErrorID();
}

int linkCommandListToEvent(int indexCommandList,
                           int indexEvent,
                           int consumerIndex) {
  LOGINFO4("%s/%s:%d indexCommandList=%d indexEvent=%d consumerIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexCommandList,
           indexEvent,
           consumerIndex);

  CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex);
  CHECK_EVENT_RETURN_IF_ERROR(indexEvent);

  return events[indexEvent]->linkEventConsumer(commandLists[indexCommandList],
                                               consumerIndex);
}

int setCommandListEnable(int indexCommandList, int enable) {
  LOGINFO4("%s/%s:%d indexCommandList=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexCommandList,
           enable);

  CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex);

  return commandLists[indexCommandList]->setEnable(enable);
}

int setCommandListEnablePrintouts(int indexCommandList, int enable) {
  LOGINFO4("%s/%s:%d indexCommandList=%d enable=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexCommandList,
           enable);

  CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex);

  WRITE_DIAG_BIT(FUNCTION_COMMAND_LIST_DIAGNOSTICS_BIT, enable);
  return 0;
}

int addCommandListCommand(int indexCommandList, char *expr) {
  LOGINFO4("%s/%s:%d indexCommandList=%d value=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexCommandList,
           expr);

  CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex);

  std::string tempExpr = expr;

  return commandLists[indexCommandList]->addCommand(tempExpr);
}

int triggerCommandList(int indexCommandList) {
  LOGINFO4("%s/%s:%d indexCommandList=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           indexCommandList);

  CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex);
  // No need for state of ethercat master
  return commandLists[indexCommandList]->executeEvent(1);
}
