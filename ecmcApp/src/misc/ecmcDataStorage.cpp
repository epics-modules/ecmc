/*
 * ecmcDataStorage.cpp
 *
 *  Created on: May 27, 2016
 *      Author: anderssandstrom
 */

#include "ecmcDataStorage.h"

ecmcDataStorage::ecmcDataStorage(ecmcAsynPortDriver *asynPortDriver,
                                 int index,
                                 int size,
                                 storageType bufferType) {
  PRINT_ERROR_PATH("dataStorage[%d].error", index);
  initVars();
  index_=index;
  setBufferSize(size);
  bufferElementCount_ = size;
  bufferType_         = bufferType;
  asynPortDriver_ = asynPortDriver;
  LOGINFO9("%s/%s:%d: dataStorage[%d]=new;\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index);
  printCurrentState();
  initAsyn();
}

ecmcDataStorage::~ecmcDataStorage() {
  delete buffer_;
}

void ecmcDataStorage::printCurrentState() {
  LOGINFO9("%s/%s:%d: dataStorage[%d].bufferSize=%d;\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index_,
           bufferElementCount_);

  switch (bufferType_) {
  case ECMC_STORAGE_NORMAL_BUFFER:
    LOGINFO9("%s/%s:%d: dataStorage[%d].bufferType=%s;\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             index_,
             "ECMC_STORAGE_NORMAL_BUFFER");
    break;

  case ECMC_STORAGE_RING_BUFFER:
    LOGINFO9("%s/%s:%d: dataStorage[%d].bufferType=%s;\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             index_,
             "ECMC_STORAGE_RING_BUFFER");
    break;

  case ECMC_STORAGE_FIFO_BUFFER:
    LOGINFO9("%s/%s:%d: dataStorage[%d].bufferType=%s;\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             index_,
             "ECMC_STORAGE_FIFO_BUFFER");
    break;

  default:
    LOGINFO9("%s/%s:%d: dataStorage[%d].bufferType=%d;\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             index_,
             bufferType_);
    break;
  }
}

int ecmcDataStorage::getIndex() {
  return index_;
}

void ecmcDataStorage::initVars() {
  errorReset();
  bufferType_         = ECMC_STORAGE_NORMAL_BUFFER;
  bufferElementCount_ = ECMC_DEFAULT_DATA_STORAGE_SIZE;
  buffer_             = NULL;
  currentBufferIndex_ = 0;
  bufferFullCounter_  = 0;
  index_              = 0;
  asynPortDriver_     = NULL;
  dataAsynDataItem_   = NULL;
  fullAsynDataItem_   = NULL;
  indexAsynDataItem_  = NULL;
  sizeAsynDataItem_   = NULL;
}

int ecmcDataStorage::clearBuffer() {
  if (buffer_ == NULL) {
    LOGINFO9(
      "%s/%s:%d: ERROR: Data storage %d. Clear buffer failed. Buffer NULL (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index_,
      ERROR_DATA_STORAGE_NULL);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_DATA_STORAGE_NULL);
  }
  memset(buffer_, 0, bufferElementCount_ * sizeof(double));
  currentBufferIndex_ = 0;
  bufferFullCounter_  = 0;
  isFull_ = 0;
  updateAsyn(0);
  return 0;
}

int ecmcDataStorage::setBufferSize(int elements) {  
  bufferElementCount_ = elements;
  bufferFullCounter_  = 0;
  isFull_ = 0;
  double * tempBuffer = new double[elements];
  if (tempBuffer == NULL) {
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR DATA STORAGE OBJECT.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_DATA_STORAGE_NULL);
    exit(EXIT_FAILURE);  
  }  
  //Set new adress to asyn interface
  if(dataAsynDataItem_){
    dataAsynDataItem_->setEcmcDataPointer((uint8_t*)tempBuffer,bufferElementCount_*sizeof(double));
    updateAsyn(1);
  }
  delete buffer_;
  buffer_ = tempBuffer;
  
  return 0;
}

int ecmcDataStorage::isStorageFull() {
  isFull_=bufferFullCounter_ >= bufferElementCount_;
  return isFull_;
}

int ecmcDataStorage::getSize() {
  return bufferElementCount_;
}

int ecmcDataStorage::getCurrentIndex() {
  return currentBufferIndex_;
}

int ecmcDataStorage::printBuffer() {
  int start = 0;
  int end   = bufferElementCount_;

  if (bufferType_ == ECMC_STORAGE_NORMAL_BUFFER) {
    end = currentBufferIndex_;
  }
  printf("Printout of data storage buffer %d.\n", index_);

  for (int i = start; i < end; i++) {
    printf("%lf, ", buffer_[i]);
  }
  printf("\n");
  return 0;
}

int ecmcDataStorage::getData(double **data, int *size) {
  *data = buffer_;
  *size = bufferElementCount_;
  return 0;
}

int ecmcDataStorage::getDataElement(int index, double *data) {
  if ((index < 0) || (index >= bufferElementCount_)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_STORAGE_POSITION_OUT_OF_RANGE);
  }
  *data = buffer_[index];
  return 0;
}

int ecmcDataStorage::getDataElement(double *data) {
  return getDataElement(currentBufferIndex_, data);
}

int ecmcDataStorage::setDataElement(int index, double data) {
  if ((index < 0) || (index >= bufferElementCount_)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_STORAGE_POSITION_OUT_OF_RANGE);
  }
  buffer_[index] = data;
  return 0;
}

int ecmcDataStorage::setDataElement(double data) {
  return setDataElement(currentBufferIndex_, data);
}

int ecmcDataStorage::setData(double *data, int size) {
  currentBufferIndex_ = 0;  // Start from beginning
  return appendData(data, size);
}

int ecmcDataStorage::appendDataNormal(double *data, int size) {
  // Fill untill buffer is full. Discard other data

  int sizeToCopy = size;

  if (sizeToCopy > bufferElementCount_) {
    sizeToCopy = bufferElementCount_;
  }

  if (sizeToCopy > bufferElementCount_ - currentBufferIndex_) {
    sizeToCopy = bufferElementCount_ - currentBufferIndex_;
  }

  if (sizeToCopy > 0) {
    memcpy(buffer_ + currentBufferIndex_, data, sizeToCopy * sizeof(double));
    currentBufferIndex_ = currentBufferIndex_ + sizeToCopy;
  }

  if (bufferFullCounter_ < bufferElementCount_) {
    bufferFullCounter_ = bufferFullCounter_ + sizeToCopy;
  }
  isStorageFull();

  return 0;
}

int ecmcDataStorage::appendDataRing(double *data, int size) {
  if (size > bufferElementCount_) {
    LOGINFO9(
      "%s/%s:%d: ERROR: Data storage %d. Buffer size to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index_,
      ERROR_DATA_STORAGE_SIZE_TO_SMALL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_STORAGE_SIZE_TO_SMALL);
  }

  // Fill untill buffer is full. Start over in beginning
  int sizeToCopy = size;

  if (sizeToCopy > bufferElementCount_) {
    sizeToCopy = bufferElementCount_;
  }

  if (sizeToCopy > bufferElementCount_ - currentBufferIndex_) {
    sizeToCopy = bufferElementCount_ - currentBufferIndex_;
  }

  if (sizeToCopy > 0) {
    memcpy(buffer_ + currentBufferIndex_, data, sizeToCopy * sizeof(double));
    currentBufferIndex_ = currentBufferIndex_ + sizeToCopy;
  }

  if (sizeToCopy < size) {
    // If ring buffer then copy rest of data to the beginning of the buffer
    memcpy(buffer_, data + sizeToCopy, (size - sizeToCopy) * sizeof(double));
    currentBufferIndex_ = (size - sizeToCopy);
  }

  if (bufferFullCounter_ < bufferElementCount_) {
    bufferFullCounter_ = bufferFullCounter_ + sizeToCopy;
  }
  isStorageFull();

  return 0;
}

int ecmcDataStorage::appendDataFifo(double *data, int size) {
  // Always add in end
  if (size < bufferElementCount_) {
    // Move old data left
    int bytesToMove = sizeof(double) * (bufferElementCount_ - size);
    memmove(buffer_, buffer_ + size, bytesToMove);
  }

  int sizeToCopy = size;

  if (sizeToCopy >= bufferElementCount_) {
    sizeToCopy = bufferElementCount_;
  }

  memcpy(buffer_ + (bufferElementCount_ - sizeToCopy),
         data,
         sizeof(double) * sizeToCopy);

  if (bufferFullCounter_ < bufferElementCount_) {
    bufferFullCounter_ = bufferFullCounter_ + sizeToCopy;
  }
  isStorageFull();
  return 0;
}

int ecmcDataStorage::appendData(double *data, int size) {
  if (buffer_ == NULL) {
    LOGINFO9(
      "%s/%s:%d: ERROR: Data storage %d. Append data failed. Buffer NULL (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index_,
      ERROR_DATA_STORAGE_NULL);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_DATA_STORAGE_NULL);
  }

  if (size > bufferElementCount_) {
    LOGINFO9(
      "%s/%s:%d: ERROR: Data storage %d. Buffer size to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index_,
      ERROR_DATA_STORAGE_SIZE_TO_SMALL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_STORAGE_SIZE_TO_SMALL);
  }
  int errorCode = 0;
  switch (bufferType_) {
  case ECMC_STORAGE_NORMAL_BUFFER:
     errorCode = appendDataNormal(data, size);     
    break;

  case ECMC_STORAGE_RING_BUFFER:
    errorCode = appendDataRing(data, size);
    break;

  case ECMC_STORAGE_FIFO_BUFFER:
    errorCode = appendDataFifo(data, size);
    break;
  }
  if(errorCode) {
    return errorCode;
  }

  return updateAsyn(0);  
}

int ecmcDataStorage::appendData(double data) {
  return appendData(&data, 1);
}

int ecmcDataStorage::setCurrentPosition(int position) {
  if ((position > bufferElementCount_) || (position < 0)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_DATA_STORAGE_POSITION_OUT_OF_RANGE);
  }

  if (currentBufferIndex_ != position) {
    LOGINFO9("%s/%s:%d: dataStorage[%d].dataIndex=%d;\n",
             __FILE__,
             __FUNCTION__,
             __LINE__,
             index_,
             position);
  }

  currentBufferIndex_ = position;
  return 0;
}

int ecmcDataStorage::initAsyn() {
  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];  
  char *name = buffer;
  unsigned int charCount = 0;

  // "ds%d.data"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_PLC_DATA_STORAGE_STR"%d."ECMC_DATA_STORAGE_DATA_DATA_STR,
                       index_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_DATA_STORAGE_ASYN_PARAM_REGISTER_FAIL);
    return ERROR_DATA_STORAGE_ASYN_PARAM_REGISTER_FAIL;
  }
  name = buffer;

  dataAsynDataItem_ = asynPortDriver_->addNewAvailParam(name,
                                    asynParamFloat64Array, //default type
                                    (uint8_t *)(buffer_),
                                    bufferElementCount_*sizeof(double),
                                    0);

  if(!dataAsynDataItem_) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  dataAsynDataItem_->allowWriteToEcmc(true);
  dataAsynDataItem_->refreshParam(1);
  
  // "ds%d.index"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_PLC_DATA_STORAGE_STR"%d."ECMC_DATA_STORAGE_DATA_INDEX_STR,
                       index_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_DATA_STORAGE_ASYN_PARAM_REGISTER_FAIL);
    return ERROR_DATA_STORAGE_ASYN_PARAM_REGISTER_FAIL;
  }
  name = buffer;
  indexAsynDataItem_ = asynPortDriver_->addNewAvailParam(name,
                                    asynParamInt32, //default type
                                    (uint8_t *)&(currentBufferIndex_),
                                    sizeof(currentBufferIndex_),
                                    0);
  if(!indexAsynDataItem_) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  indexAsynDataItem_->allowWriteToEcmc(false);
  indexAsynDataItem_->refreshParam(1);  

  // "ds%d.full"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_PLC_DATA_STORAGE_STR"%d."ECMC_DATA_STORAGE_DATA_FULL_STR,
                       index_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_DATA_STORAGE_ASYN_PARAM_REGISTER_FAIL);
    return ERROR_DATA_STORAGE_ASYN_PARAM_REGISTER_FAIL;
  }
  name = buffer;
  fullAsynDataItem_ = asynPortDriver_->addNewAvailParam(name,
                                    asynParamInt32, //default type
                                    (uint8_t *)&(isFull_),
                                    sizeof(isFull_),
                                    0);
  if(!fullAsynDataItem_) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  fullAsynDataItem_->allowWriteToEcmc(false);
  fullAsynDataItem_->refreshParam(1);

  // "ds%d.size"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_PLC_DATA_STORAGE_STR"%d."ECMC_DATA_STORAGE_DATA_SIZE_STR,
                       index_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_DATA_STORAGE_ASYN_PARAM_REGISTER_FAIL);
    return ERROR_DATA_STORAGE_ASYN_PARAM_REGISTER_FAIL;
  }
  name = buffer;
  sizeAsynDataItem_ = asynPortDriver_->addNewAvailParam(name,
                                    asynParamInt32, //default type
                                    (uint8_t *)&(bufferElementCount_),
                                    sizeof(bufferElementCount_),
                                    0);
  if(!sizeAsynDataItem_) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  sizeAsynDataItem_->allowWriteToEcmc(false);
  sizeAsynDataItem_->refreshParam(1);
  
  asynPortDriver_->callParamCallbacks();

  return 0;
}

int ecmcDataStorage::updateAsyn(bool force) {  
  dataAsynDataItem_->refreshParamRT(force);
  fullAsynDataItem_->refreshParamRT(force);
  indexAsynDataItem_->refreshParamRT(force);
  sizeAsynDataItem_-> refreshParamRT(force);  
  return 0;
}