/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEcMemMap.h
*
*  Created on: Dec 2, 2015
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCECMEMMAP_H_
#define ECMCECMEMMAP_H_
#include <string>
#include <cmath>
#include "stdio.h"
#include "ecrt.h"
#include "../main/ecmcDefinitions.h"
#include "../main/ecmcError.h"
#include "../com/ecmcOctetIF.h"  // Logging macros
#include "../com/ecmcAsynPortDriver.h"
#include "ecmcEcEntry.h"

#define ERROR_MEM_MAP_SIZE_OUT_OF_RANGE 0x211000
#define ERROR_MEM_ASYN_VAR_BUFFER_OUT_OF_RANGE 0x211001
#define ERROR_MEM_INDEX_OUT_OF_RANGE 0x211002
#define ERROR_MEM_INVALID_DATA_TYPE 0x211003

class ecmcEcMemMap : public ecmcError {
 public:
  ecmcEcMemMap(ecmcAsynPortDriver *asynPortDriver,
               int masterId,
               int slaveId,
               ecmcEcEntry   *startEntry,
               size_t         byteSize,
               ec_direction_t nDirection,
               ecmcEcDataType dt,
               std::string    id);
  ~ecmcEcMemMap();
  void        initVars();
  int         write(uint8_t *values,
                    size_t   byteToWrite,
                    size_t  *bytesWritten);
  int         read(uint8_t *values,
                   size_t   bytesToRead,
                   size_t  *bytesRead);
  int         updateInputProcessImage();
  int         updateOutProcessImage();
  std::string getIdentificationName();
  int         setDomainSize(size_t size);
  int         validate();
  int         getByteSize();
  uint8_t*    getBufferPointer();
  ecmcEcDataType getDataType();
  int         getDoubleDataAtIndex(size_t index,
                                   double *data);
  int         setDoubleDataAtIndex(size_t index,
                                   double data);
  size_t      getElementCount();
  size_t      getBytesPerElement();
  int         updateAsyn(bool force);

 private:
  int                initAsyn();
  size_t             byteSize_;
  size_t             elements_;
  size_t             bytesPerElement_;
  size_t             domainSize_;
  int                byteOffset_;
  int                masterId_;
  int                slaveId_;
  uint8_t           *domainAdr_;  
  uint8_t           *adr_;
  uint8_t           *buffer_;
  ec_direction_t     direction_;
  std::string        idString_;
  char              *idStringChar_;
  ecmcEcEntry       *startEntry_;
  ecmcAsynPortDriver*asynPortDriver_;
  ecmcAsynDataItem  *memMapAsynParam_;
  ecmcEcDataType     dataType_;
  int8_t             *int8Ptr_;
  uint8_t            *uint8Ptr_;
  int16_t            *int16Ptr_;
  uint16_t           *uint16Ptr_;
  int32_t            *int32Ptr_;
  uint32_t           *uint32Ptr_;
  int64_t            *int64Ptr_;
  uint64_t           *uint64Ptr_;
  float              *float32Ptr_;
  double             *float64Ptr_;
};
#endif  /* ECMCECMEMMAP_H_ */
