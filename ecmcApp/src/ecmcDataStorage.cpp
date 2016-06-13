/*
 * ecmcDataStorage.cpp
 *
 *  Created on: May 27, 2016
 *      Author: anderssandstrom
 */

#include "ecmcDataStorage.h"

ecmcDataStorage::ecmcDataStorage (int index)
{
  initVars();
  buffer_=new double[ECMC_DEFAULT_DATA_STORAGE_SIZE];
  if(buffer_==NULL){
    PRINT_DIAG(("Index: %d. Error: %x\n",index_,ERROR_DATA_STORAGE_NULL));
    setErrorID(ERROR_DATA_STORAGE_NULL);
  }
  index_=index;
}

ecmcDataStorage::ecmcDataStorage (int index, int size,storageType bufferType)
{
  initVars();
  bufferElementCount_=size;
  buffer_=new double[size];
  if(buffer_==NULL){
    PRINT_DIAG(("Index: %d. Error: %x\n",index_,ERROR_DATA_STORAGE_NULL));
    setErrorID(ERROR_DATA_STORAGE_NULL);
  }
  bufferType_=bufferType;
  index_=index;
}

ecmcDataStorage::~ecmcDataStorage ()
{
  delete buffer_;
}

void ecmcDataStorage::initVars()
{
  bufferType_=ECMC_STORAGE_LIFO_BUFFER;
  bufferElementCount_=ECMC_DEFAULT_DATA_STORAGE_SIZE;
  buffer_=NULL;
  currentBufferIndex_=0;
  enableDiagnosticPrintouts_=false;
  index_=0;
}

int ecmcDataStorage::clearBuffer()
{
  if(buffer_==NULL){
    PRINT_DIAG(("Index: %d. Error: %x\n",index_,ERROR_DATA_STORAGE_NULL));
    return setErrorID(ERROR_DATA_STORAGE_NULL);
  }
  memset(buffer_,0,bufferElementCount_*sizeof(double));
  currentBufferIndex_=0;
  return 0;
}

int ecmcDataStorage::setBufferSize(int elements)
{
  delete buffer_;
  bufferElementCount_=elements;
  buffer_=new double[bufferElementCount_];
  return 0;
}

int ecmcDataStorage::appendData(double data)
{
  if(buffer_==NULL)
  {
    PRINT_DIAG(("Index: %d. Error: %x\n",index_,ERROR_DATA_STORAGE_NULL));
    return setErrorID(ERROR_DATA_STORAGE_NULL);
  }

  if(currentBufferIndex_>(bufferElementCount_-1) && bufferType_==ECMC_STORAGE_RING_BUFFER){
    currentBufferIndex_=0;
  }

  if(currentBufferIndex_>(bufferElementCount_-1) && bufferType_==ECMC_STORAGE_LIFO_BUFFER){
    PRINT_DIAG(("Index: %d.Error: %x\n",index_,ERROR_DATA_STORAGE_FULL));
    return setErrorID(ERROR_DATA_STORAGE_FULL);
  }

  buffer_[currentBufferIndex_]=data;
  currentBufferIndex_++;
  return 0;
}

int ecmcDataStorage::isStorgeFull()
{
  return currentBufferIndex_==bufferElementCount_;
}

int ecmcDataStorage::getSize()
{
  return bufferElementCount_;
}

int ecmcDataStorage::printBuffer()
{
  int start=0;
  int end=bufferElementCount_;
  if(bufferType_==ECMC_STORAGE_LIFO_BUFFER){
    end=currentBufferIndex_;
  }
  printf("Printout of data storage buffer %d.\n",index_);
  for(int i=start;i<end ;i++){
    printf("%lf ",buffer_[i]);
  }
  printf("\n");
  return 0;
}

int ecmcDataStorage::setEnablePrintOuts(bool enable)
{
  enableDiagnosticPrintouts_=enable;
  return 0;
}
