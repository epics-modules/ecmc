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


#define WARNING_DATA_ITEM_EC_ENTRY_DIR_MISSMATCH 0x21150


// Access to arbitraty data of up to 64 bit size in the ethercat process image.
// A start entry, byte and bit offset needs to be defined.

class ecmcEcData :public ecmcEcEntry {
 public:
  ecmcEcData(ecmcAsynPortDriver *asynPortDriver,
             int masterId,
             int slaveId,
             ecmcEcEntry   *startEntry,
             size_t entryByteOffset,
             size_t entryBitOffset,
             ec_direction_t nDirection,
             ecmcEcDataType dt,
             std::string    id);
  ~ecmcEcData();

  // Overridden ecmcEcEntry functions
  int             updateInputProcessImage();
  int             updateOutProcessImage();
  int             validate();
  
 private:
  void            initVars();
  
  //byte and bit offset from entry
  size_t          entryByteOffset_;
  size_t          entryBitOffset_;
  size_t          byteSize_;
  ecmcEcEntry    *startEntry_;

  int8_t         *int8Ptr_;
  uint8_t        *uint8Ptr_;
  int16_t        *int16Ptr_;
  uint16_t       *uint16Ptr_;
  int32_t        *int32Ptr_;
  uint32_t       *uint32Ptr_;
  int64_t        *int64Ptr_;
  uint64_t       *uint64Ptr_;
  float          *float32Ptr_;
  double         *float64Ptr_;
 
  // Utiliy functions
  static uint8_t  Read1Bit(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     Write1Bit(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value);
  static uint8_t  Read2Bits(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     Write2Bits(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value);
  static uint8_t  Read3Bits(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     Write3Bits(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value);
  static uint8_t  Read4Bits(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     Write4Bits(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value);
  static uint8_t  ReadUInt8(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     WriteUInt8(uint8_t* buffer, size_t byteOffset, int bitOffset, uint8_t value);
  static int8_t   ReadInt8(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     WriteInt8(uint8_t* buffer, size_t byteOffset, int bitOffset, int8_t value);
  static uint16_t ReadUInt16(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     WriteUInt16(uint8_t* buffer, size_t byteOffset, int bitOffset, uint16_t value);
  static int16_t  ReadInt16(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     WriteInt16(uint8_t* buffer, size_t byteOffset, int bitOffset, int16_t value);
  static uint32_t ReadUInt32(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     WriteUInt32(uint8_t* buffer, size_t byteOffset, int bitOffset, uint32_t value);
  static int32_t  ReadInt32(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     WriteInt32(uint8_t* buffer, size_t byteOffset, int bitOffset, int32_t value);
  static uint64_t ReadUInt64(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     WriteUInt64(uint8_t* buffer, size_t byteOffset, int bitOffset, uint64_t value);
  static int64_t  ReadInt64(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     WriteInt64(uint8_t* buffer, size_t byteOffset, int bitOffset, int64_t value);
  static float    ReadFloat(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     WriteFloat(uint8_t* buffer, size_t byteOffset, int bitOffset, float value);
  static double   ReadDouble(uint8_t* buffer, size_t byteOffset, int bitOffset);
  static void     WriteDouble(uint8_t* buffer, size_t byteOffset, int bitOffset, double value);

};
#endif  /* ecmcEcData_H_ */
