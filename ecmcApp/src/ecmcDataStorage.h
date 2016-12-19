/*
 * ecmcDataStorage.h
 *
 *  Created on: May 27, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCDATASTORAGE_H_
#define ECMCDATASTORAGE_H_

#include "stdio.h"

#include "ecmcError.h"
#include "ecmcDefinitions.h"

//Data storage
#define ERROR_DATA_STORAGE_FULL 0x20200
#define ERROR_DATA_STORAGE_NULL 0x20201
#define ERROR_DATA_STORAGE_SIZE_TO_SMALL 0x20202
#define ERROR_DATA_STORAGE_POSITION_OUT_OF_RANGE 0x20203

enum storageType{
  ECMC_STORAGE_LIFO_BUFFER=0,
  ECMC_STORAGE_RING_BUFFER=1,
};


class ecmcDataStorage: public ecmcError
{
public:
  ecmcDataStorage (int index);
  ecmcDataStorage (int index,int size,storageType bufferType);
  ~ecmcDataStorage ();
  int setBufferSize(int elements);
  int clearBuffer();
  int appendData(double data);
  int isStorgeFull();
  int printBuffer();
  int getSize();
  int getData(double **data, int *size);
  int setData(double *data, int size);
  int appendData(double *data, int size);
  int setCurrentPosition(int position);

private:
  void initVars();
  int currentBufferIndex_;
  double* buffer_;
  int bufferElementCount_;
  storageType bufferType_;
  int index_;
};

#endif /* ECMCDATASTORAGE_H_ */
