#ifndef ECMCECENTRYARRAY_H_
#define ECMCECENTRYARRAY_H_
#include "ecrt.h"
#include "stdio.h"
#include <string>
#include <cmath>

#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "cmd.h" //Logging macros
#include "ecmcAsynPortDriver.h"
#include "ecmcEcEntry.h"


class ecmcEcEntryArray : public ecmcError
{
public:
  ecmcEcEntryArray(ecmcEcEntry *startEntry,size_t byteSize,int type,ec_direction_t nDirection, std::string id);
  ~ecmcEcEntryArray();
  void initVars();
  int write(uint8_t *values, size_t byteToWrite, size_t *bytesWritten);
  int read(uint8_t *values, size_t bytesToRead, size_t *bytesRead);
  int updateInputProcessImage();
  int updateOutProcessImage();
  std::string getIdentificationName();
private:
  int updateAsyn(bool force);
  uint8_t *domainAdr_;
  int adrOffset_;
  int byteOffset_;
  ec_direction_t direction_;
  std::string idString_;
  ecmcEcEntry *startEntry_;
  size_t byteSize_;
  uint8_t *buffer_;
  int type_;
};
#endif /* ECMCECENTRYARRAY_H_ */
