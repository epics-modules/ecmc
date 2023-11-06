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

class ecmcEcData : public ecmcEcEntry {
public:
  ecmcEcData(ecmcAsynPortDriver *asynPortDriver,
             int                 masterId,
             int                 slaveId,
             ecmcEcEntry        *startEntry,
             size_t              entryByteOffset,
             size_t              entryBitOffset,
             ec_direction_t      nDirection,
             ecmcEcDataType      dt,
             std::string         id);
  ~ecmcEcData();

  // Overridden ecmcEcEntry functions
  int  updateInputProcessImage();
  int  updateOutProcessImage();
  int  validate();

private:
  void initVars();

  // byte and bit offset from entry
  size_t entryByteOffset_;
  size_t entryBitOffset_;
  size_t byteSize_;
  ecmcEcEntry *startEntry_;

  static uint8_t read_1_bit_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_1_bit_offset(uint8_t *buffer, int byteOffset, int bitOffset, uint8_t value);
  static uint8_t read_2_bit_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_2_bit_offset(uint8_t *buffer, int byteOffset, int bitOffset, uint8_t value);
  static uint8_t read_3_bit_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_3_bit_offset(uint8_t *buffer, int byteOffset, int bitOffset, uint8_t value);
  static uint8_t read_4_bit_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_4_bit_offset(uint8_t *buffer, int byteOffset, int bitOffset, uint8_t value);
  static uint8_t read_uint8_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_uint8_offset(uint8_t *buffer, int byteOffset, int bitOffset, uint8_t value);
  static int8_t read_int8_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_int8_offset(uint8_t *buffer, int byteOffset, int bitOffset, int8_t value);
  static uint16_t read_uint16_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_uint16_offset(uint8_t *buffer, int byteOffset, int bitOffset, uint16_t value);
  static int16_t read_int16_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_int16_offset(uint8_t *buffer, int byteOffset, int bitOffset, int16_t value);
  static uint32_t read_uint32_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_uint32_offset(uint8_t *buffer, int byteOffset, int bitOffset, uint32_t value);
  static int32_t read_int32_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_int32_offset(uint8_t *buffer, int byteOffset, int bitOffset, int32_t value);
  static uint64_t read_uint64_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_uint64_offset(uint8_t *buffer, int byteOffset, int bitOffset, uint64_t value);
  static int64_t read_int64_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_int64_offset(uint8_t *buffer, int byteOffset, int bitOffset, int64_t value);
  static float read_float_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_float_offset(uint8_t *buffer, int byteOffset, int bitOffset, float value);
  static double read_double_offset(uint8_t *buffer, int byteOffset, int bitOffset);
  static void write_double_offset(uint8_t *buffer, int byteOffset, int bitOffset, double value);

};
#endif  /* ecmcEcData_H_ */
