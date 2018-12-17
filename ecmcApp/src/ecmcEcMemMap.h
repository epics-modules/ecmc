#ifndef ECMCECMEMMAP_H_
#define ECMCECMEMMAP_H_
#include <string>
#include <cmath>
#include "stdio.h"
#include "ecrt.h"
#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "cmd.h"  // Logging macros
#include "ecmcAsynPortDriver.h"
#include "ecmcEcEntry.h"

#define ERROR_MEM_MAP_SIZE_OUT_OF_RANGE 0x211000


class ecmcEcMemMap : public ecmcError, public ecmcAsynLink {
 public:
  ecmcEcMemMap(ecmcEcEntry   *startEntry,
               size_t         byteSize,
               int            type,
               ec_direction_t nDirection,
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

 private:
  int         updateAsyn(bool force);
  int         validate();
  uint8_t *domainAdr_;
  int byteOffset_;
  ec_direction_t direction_;
  std::string idString_;
  ecmcEcEntry *startEntry_;
  size_t byteSize_;
  uint8_t *buffer_;
  int type_;
  size_t domainSize_;
  int validationDone_;
};
#endif  /* ECMCECMEMMAP_H_ */
