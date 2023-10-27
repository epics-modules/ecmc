/*************************************************************************\
* Copyright (c) 2023 Paul Scherrer Institute
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEcData.h
*
*  Created on: Oct 27, 2023
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ecmcEcData_H_
#define ecmcEcData_H_
#include <string>
#include <cmath>
#include "stdio.h"
#include "ecrt.h"
#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "ecmcOctetIF.h"  // Logging macros
#include "ecmcAsynPortDriver.h"
#include "ecmcEcEntry.h"

#define ERROR_ECDATAITEM_MAP_SIZE_OUT_OF_RANGE 0x212000
#define ERROR_ECDATAITEM_ASYN_VAR_BUFFER_OUT_OF_RANGE 0x212001
#define ERROR_ECDATAITEM_INDEX_OUT_OF_RANGE 0x212002
#define ERROR_ECDATAITEM_INVALID_DATA_TYPE 0x212003

// Access to arbitraty data of up to 64 bit size in the ethercat process image.
// A start entry, byte and bit offset needs to be defined.

class ecmcEcData : public ecmcError {
 public:
  ecmcEcData(ecmcAsynPortDriver *asynPortDriver,
               int masterId,
               int slaveId,
               ecmcEcDomain  *domain,
               ecmcEcEntry   *startEntry,
               size_t         byteSize,
               ec_direction_t nDirection,
               ecmcEcDataType dt,
               std::string    id);
  ~ecmcEcData();
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
  int         setDomainSize();
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
  ecmcAsynDataItem  *dataAsynParam_;
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
  ecmcEcDomain       *domain_;

  // New utiliy functions
  uint8_t  Read1Bit(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     Write1Bit(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value);
  uint8_t  Read2Bits(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     Write2Bits(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value);
  uint8_t  Read3Bits(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     Write3Bits(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value);
  uint8_t  Read4Bits(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     Write4Bits(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value);
  uint8_t  ReadUInt8(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     WriteUInt8(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value);
  int8_t   ReadInt8(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     WriteInt8(uint8_t* buffer, size_t byteOffset, int bitOffset, int8_t value);
  uint16_t ReadUInt16(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     WriteUInt16(uint8_t* buffer, size_t byteOffset, int bitOffset, uint16_t value);
  int16_t  ReadInt16(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     WriteInt16(uint8_t* buffer, size_t byteOffset, int bitOffset, int16_t value);
  uint32_t ReadUInt32(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     WriteUInt32(uint8_t* buffer, size_t byteOffset, int bitOffset, uint32_t value);
  int32_t  ReadInt32(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     WriteInt32(uint8_t* buffer, size_t byteOffset, int bitOffset, int32_t value);
  uint64_t ReadUInt64(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     WriteUInt64(uint8_t* buffer, size_t byteOffset, int bitOffset, uint64_t value);
  int64_t  ReadInt64(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     WriteInt64(uint8_t* buffer, size_t byteOffset, int bitOffset, int64_t value);
  float    ReadFloat(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     WriteFloat(uint8_t* buffer, size_t byteOffset, int bitOffset, float value);
  double   ReadDouble(uint8_t* buffer, size_t byteOffset, int bitOffset);
  void     WriteDouble(uint8_t* buffer, size_t byteOffset, int bitOffset, double value);

};
#endif  /* ecmcEcData_H_ */
