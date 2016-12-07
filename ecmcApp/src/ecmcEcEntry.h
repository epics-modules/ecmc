/*
 * cMcuEntry.h
 *
 *  Created on: Dec 2, 2015
 *      Author: anderssandstrom
 */

#ifndef ECMCECENTRY_H_
#define ECMCECENTRY_H_
#include "ecrt.h"
#include "stdio.h"
#include <string>
#include <cmath>

#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "cmd.h" //Logging macros

#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))

//ECENTRY ERRORS
#define ERROR_EC_ENTRY_DATA_POINTER_NULL 0x21000
#define ERROR_EC_ENTRY_INVALID_OFFSET 0x21001
#define ERROR_EC_ENTRY_INVALID_DOMAIN_ADR 0x21002
#define ERROR_EC_ENTRY_INVALID_BIT_LENGTH 0x21003
#define ERROR_EC_ENTRY_LINK_FAILED 0x21004
#define ERROR_EC_ENTRY_INDEX_OUT_OF_RANGE 0x21005
#define ERROR_EC_ENTRY_INVALID_BIT_INDEX 0x21006
#define ERROR_EC_ENTRY_READ_FAIL 0x21007
#define ERROR_EC_ENTRY_WRITE_FAIL 0x21008

class ecmcEcEntry : public ecmcError
{
public:
  ecmcEcEntry(uint16_t entryIndex,uint8_t  entrySubIndex, uint8_t bits, ec_direction_t nDirection, std::string id);
  ecmcEcEntry(uint8_t bitLength,uint8_t *domainAdr, std::string id);  //only used for simulation purpose
  ~ecmcEcEntry();
  void initVars();
  uint16_t getEntryIndex();
  uint8_t getEntrySubIndex();
  int getBits();
  int getEntryInfo(ec_pdo_entry_info_t *info);
  void setAdrOffsets(int byteOffset,int bitOffset);
  void setDomainAdr(uint8_t *domainAdr); //After activate
  int writeValue(uint64_t value);
  int writeBit(int bitNumber, uint64_t value);
  int readValue(uint64_t *value);
  int readBit(int bitNumber, uint64_t* value);
  int updateInputProcessImage();
  int updateOutProcessImage();
  std::string getIdentificationName();

private:
  uint8_t *domainAdr_;
  uint16_t entryIndex_;
  uint8_t  entrySubIndex_;
  int adrOffset_;
  int bitLength_;
  int bitOffset_;
  int byteOffset_;
  uint64_t value_;
  ec_direction_t direction_;
  bool sim_;
  std::string idString_;
};
#endif /* ECMCECENTRY_H_ */
