/*
 * ecmcDataStorage.cpp
 *
 *  Created on: May 27, 2016
 *      Author: anderssandstrom
 */

#include "ecmcDataStorage.h"

ecmcDataStorage::ecmcDataStorage (int index)
{
  index_=index;
  PRINT_ERROR_PATH("dataStorage[%d].error",index_);
  initVars();
  setBufferSize(ECMC_DEFAULT_DATA_STORAGE_SIZE);
  bufferType_=ECMC_STORAGE_NORMAL_BUFFER;
  LOGINFO9("%s/%s:%d: dataStorage[%d]=new;\n",__FILE__, __FUNCTION__, __LINE__,index);
  printCurrentState();
}

ecmcDataStorage::ecmcDataStorage (int index, int size,storageType bufferType)
{
  index_=index;
  PRINT_ERROR_PATH("dataStorage[%d].error",index_);
  initVars();
  setBufferSize(size);
  bufferElementCount_=size;
  bufferType_=bufferType;
  LOGINFO9("%s/%s:%d: dataStorage[%d]=new;\n",__FILE__, __FUNCTION__, __LINE__,index);
  printCurrentState();
}

ecmcDataStorage::~ecmcDataStorage ()
{
  delete buffer_;
}

void ecmcDataStorage::printCurrentState()
{
  LOGINFO9("%s/%s:%d: dataStorage[%d].bufferSize=%d;\n",__FILE__, __FUNCTION__, __LINE__,index_,bufferElementCount_);
  switch(bufferType_){
    case ECMC_STORAGE_NORMAL_BUFFER:
      LOGINFO9("%s/%s:%d: dataStorage[%d].bufferType=%s;\n",__FILE__, __FUNCTION__, __LINE__,index_,"ECMC_STORAGE_NORMAL_BUFFER");
      break;
    case ECMC_STORAGE_RING_BUFFER:
      LOGINFO9("%s/%s:%d: dataStorage[%d].bufferType=%s;\n",__FILE__, __FUNCTION__, __LINE__,index_,"ECMC_STORAGE_RING_BUFFER");
      break;
    case ECMC_STORAGE_FIFO_BUFFER:
      LOGINFO9("%s/%s:%d: dataStorage[%d].bufferType=%s;\n",__FILE__, __FUNCTION__, __LINE__,index_,"ECMC_STORAGE_FIFO_BUFFER");
      break;
    default:
      LOGINFO9("%s/%s:%d: dataStorage[%d].bufferType=%d;\n",__FILE__, __FUNCTION__, __LINE__,index_,bufferType_);
      break;
  }
}

int ecmcDataStorage::getIndex()
{
  return index_;
}

void ecmcDataStorage::initVars()
{
  errorReset();
  bufferType_=ECMC_STORAGE_NORMAL_BUFFER;
  bufferElementCount_=ECMC_DEFAULT_DATA_STORAGE_SIZE;
  buffer_=NULL;
  currentBufferIndex_=0;
  bufferFullCounter_=0;
}

int ecmcDataStorage::clearBuffer()
{
  if(buffer_==NULL){
    LOGINFO9("%s/%s:%d: ERROR: Data storage %d. Clear buffer failed. Buffer NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,index_,ERROR_DATA_STORAGE_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_STORAGE_NULL);
  }
  memset(buffer_,0,bufferElementCount_*sizeof(double));
  currentBufferIndex_=0;
  bufferFullCounter_=0;
  return 0;
}

int ecmcDataStorage::setBufferSize(int elements)
{
  delete buffer_;
  bufferElementCount_=elements;
  bufferFullCounter_=0;
  buffer_=new double[elements];
  if(buffer_==NULL){
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR DATA STORAGE OBJECT.\n",__FILE__,__FUNCTION__,__LINE__);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_STORAGE_NULL);
    exit(EXIT_FAILURE);
  }
  return 0;
}

int ecmcDataStorage::isStorageFull()
{
  return bufferFullCounter_>=bufferElementCount_;
}

int ecmcDataStorage::getSize()
{
  return bufferElementCount_;
}

int ecmcDataStorage::getCurrentIndex()
{
  return currentBufferIndex_;
}

int ecmcDataStorage::printBuffer()
{
  int start=0;
  int end=bufferElementCount_;
  if(bufferType_==ECMC_STORAGE_NORMAL_BUFFER){
    end=currentBufferIndex_;
  }
  printf("Printout of data storage buffer %d.\n",index_);
  for(int i=start;i<end ;i++){
    printf("%lf, ",buffer_[i]);
  }
  printf("\n");
  return 0;
}

int ecmcDataStorage::getData(double **data, int *size)
{
  *data=buffer_;
  *size=bufferElementCount_;
  return 0;
}

int ecmcDataStorage::getDataElement(int index,double *data)
{
  if(index<0 || index>=bufferElementCount_){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_STORAGE_POSITION_OUT_OF_RANGE);
  }
  *data=buffer_[index];
  return 0;
}

int ecmcDataStorage::setDataElement(int index,double data)
{
  if(index<0 || index>=bufferElementCount_){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_STORAGE_POSITION_OUT_OF_RANGE);
  }
  buffer_[index]=data;
  return 0;
}

int ecmcDataStorage::setData(double *data, int size)
{
  currentBufferIndex_=0; //Start from beginning
  return appendData(data,size);
}

int ecmcDataStorage::appendDataNormal(double *data, int size)
{
  //Fill untill buffer is full. Discard other data

  int sizeToCopy=size;
  if(sizeToCopy>bufferElementCount_){
    sizeToCopy=bufferElementCount_;
  }
  if(sizeToCopy>bufferElementCount_-currentBufferIndex_){
    sizeToCopy=bufferElementCount_-currentBufferIndex_;
  }

  if(sizeToCopy>0){
    memcpy(buffer_+currentBufferIndex_,data ,sizeToCopy*sizeof(double));
    currentBufferIndex_=currentBufferIndex_+sizeToCopy;
  }

  if(bufferFullCounter_<bufferElementCount_){
    bufferFullCounter_=bufferFullCounter_+sizeToCopy;
  }
  return 0;
}

int ecmcDataStorage::appendDataRing(double *data, int size)
{
  if(size>bufferElementCount_){
    LOGINFO9("%s/%s:%d: ERROR: Data storage %d. Buffer size to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,index_,ERROR_DATA_STORAGE_SIZE_TO_SMALL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_STORAGE_SIZE_TO_SMALL);
  }

  //Fill untill buffer is full. Start over in beginning
  int sizeToCopy=size;
  if(sizeToCopy>bufferElementCount_){
    sizeToCopy=bufferElementCount_;
  }
  if(sizeToCopy>bufferElementCount_-currentBufferIndex_){
    sizeToCopy=bufferElementCount_-currentBufferIndex_;
  }

  if(sizeToCopy>0){
    memcpy(buffer_+currentBufferIndex_,data ,sizeToCopy*sizeof(double));
    currentBufferIndex_=currentBufferIndex_+sizeToCopy;
  }

  if(sizeToCopy<size){
    //If ring buffer then copy the rest of the data to the beginning of the buffer
    memcpy(buffer_, data+sizeToCopy ,(size-sizeToCopy)*sizeof(double));
    currentBufferIndex_=(size-sizeToCopy);
  }
  
  if(bufferFullCounter_<bufferElementCount_){
    bufferFullCounter_=bufferFullCounter_+size;
  }

  return 0;
}

int ecmcDataStorage::appendDataFifo(double *data, int size)
{ 
  //Always add in end   
  if(size<bufferElementCount_){
    //Move old data left
    int bytesToMove=sizeof(double)*(bufferElementCount_-size);
    memmove(buffer_,buffer_+size,bytesToMove);
  }

  int sizeToCopy=size;
  if(sizeToCopy>=bufferElementCount_){
    sizeToCopy=bufferElementCount_;
//    memcpy(buffer_,data,sizeof(double)*sizeToCopy);
  }
//  else{
    memcpy(buffer_+(bufferElementCount_-sizeToCopy),data,sizeof(double)*sizeToCopy);
//  }

  if(bufferFullCounter_<bufferElementCount_){
    bufferFullCounter_=bufferFullCounter_+size;
  }

  return 0;
}

int ecmcDataStorage::appendData(double *data, int size)
{
  if(buffer_==NULL){
    LOGINFO9("%s/%s:%d: ERROR: Data storage %d. Append data failed. Buffer NULL (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,index_,ERROR_DATA_STORAGE_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_STORAGE_NULL);
  }

  if(size>bufferElementCount_){
    LOGINFO9("%s/%s:%d: ERROR: Data storage %d. Buffer size to small (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,index_,ERROR_DATA_STORAGE_SIZE_TO_SMALL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_STORAGE_SIZE_TO_SMALL);
  }

  switch (bufferType_){
    case ECMC_STORAGE_NORMAL_BUFFER:
      return appendDataNormal(data,size);
      break;

    case ECMC_STORAGE_RING_BUFFER:
      return appendDataRing(data,size);
      break;

    case ECMC_STORAGE_FIFO_BUFFER:
      return appendDataFifo(data,size);
      break;
  }
  return 0;
}

int ecmcDataStorage::appendData(double data)
{
  return appendData(&data,1);
}

int ecmcDataStorage::setCurrentPosition(int position)
{
  if(position>bufferElementCount_ || position<0){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_DATA_STORAGE_POSITION_OUT_OF_RANGE);
  }

  if(currentBufferIndex_!=position){
      LOGINFO9("%s/%s:%d: dataStorage[%d].dataIndex=%d;\n",__FILE__, __FUNCTION__, __LINE__,index_,position);
  }

  currentBufferIndex_=position;
  return 0;
}
