/*
 * ecmcDataStorage.h
 *
 *  Created on: May 27, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCDATASTORAGE_H_
#define ECMCDATASTORAGE_H_

#include <stdlib.h>
#include "stdio.h"
#include "../main/ecmcError.h"
#include "../main/ecmcDefinitions.h"
#include "../com/ecmcAsynPortDriver.h"

// Data storage
#define ERROR_DATA_STORAGE_FULL 0x20200
#define ERROR_DATA_STORAGE_NULL 0x20201
#define ERROR_DATA_STORAGE_SIZE_TO_SMALL 0x20202
#define ERROR_DATA_STORAGE_POSITION_OUT_OF_RANGE 0x20203
#define ERROR_DATA_STORAGE_ASYN_PARAM_REGISTER_FAIL 0x20204

enum ecmcDSBufferType {
  // Fill from beginning. Stop when full.
  ECMC_STORAGE_NORMAL_BUFFER = 0,
  // Fill from beginning. Start over in beginning
  ECMC_STORAGE_RING_BUFFER   = 1,
  // Fill from end (newwst value in the end). Old values shifted out.
  ECMC_STORAGE_FIFO_BUFFER   = 2,
};

class ecmcDataStorage : public ecmcError {
 public:
  ecmcDataStorage(ecmcAsynPortDriver *asynPortDriver,
                  int         index,
                  int         size,
                  ecmcDSBufferType bufferType);
  ~ecmcDataStorage();
  int  setBufferSize(int elements);
  int  clearBuffer();
  int  appendData(double data);
  int  isStorageFull();
  int  printBuffer();
  int  getSize();
  int  getCurrentIndex();
  int  getIndex();
  int  getData(double **data,
               int     *size);
  int  getDataElement(int     index,
                      double *data);
  int  getDataElement(double *data);
  int  setData(double *data,
               int     size);
  int  setDataElement(int    index,
                      double data);
  int  setDataElement(double data);
  int  appendData(double *data,
                  int     size);
  int  setCurrentPosition(int position);
  void printCurrentState();
  int  updateAsyn(bool force);

 private:
  int  appendDataFifo(double *data,
                      int     size);
  int  appendDataRing(double *data,
                      int     size);
  int  appendDataNormal(double *data,
                        int     size);
  void initVars();
  int  initAsyn();
  int currentBufferIndex_;
  double *buffer_;
  int bufferElementCount_;
  ecmcDSBufferType bufferType_;
  int index_;
  int bufferFullCounter_;
  ecmcAsynPortDriver *asynPortDriver_;
  ecmcAsynDataItem  *dataAsynDataItem_;
  ecmcAsynDataItem  *statusAsynDataItem_;
  ecmcAsynDataItem  *indexAsynDataItem_;
  ecmcAsynDataItem  *sizeAsynDataItem_;
  int isFull_;
  uint32_t statusWord_;

};

#endif  /* ECMCDATASTORAGE_H_ */
