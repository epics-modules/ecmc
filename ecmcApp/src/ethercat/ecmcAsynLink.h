#ifndef ECMCASYNLINK_H_
#define ECMCASYNLINK_H_

#include "asynPortDriver.h"

class ecmcAsynLink
{
public:
  ecmcAsynLink();
  virtual ~ecmcAsynLink();
  virtual int readInt32(epicsInt32 *value) = 0;
  virtual int writeInt32(epicsInt32 value) = 0;
  virtual int readUInt32Digital(epicsUInt32 *value,
                                epicsUInt32 mask) = 0;
  virtual int writeUInt32Digital(epicsUInt32 value,
                                 epicsUInt32 mask) = 0;
  virtual int readFloat64(epicsFloat64 *value) = 0;
  virtual int writeFloat64(epicsFloat64 value) = 0;
  virtual int readInt8Array(epicsInt8 *value, 
                            size_t nElements, size_t *nIn) = 0;
  virtual int writeInt8Array(epicsInt8 *value,
                             size_t nElements) = 0;
  virtual int readInt16Array(epicsInt16 *value,
                             size_t nElements, size_t *nIn) = 0;
  virtual int writeInt16Array(epicsInt16 *value,
                              size_t nElements) = 0;
  virtual int readInt32Array(epicsInt32 *value,
                             size_t nElements, size_t *nIn) = 0;
  virtual int writeInt32Array(epicsInt32 *value,
                              size_t nElements) = 0;
  virtual int readFloat32Array(epicsFloat32 *value,
                               size_t nElements, size_t *nIn) = 0;
  virtual int writeFloat32Array(epicsFloat32 *value,
                                size_t nElements) = 0;
  virtual int readFloat64Array(epicsFloat64 *value,
                               size_t nElements, size_t *nIn) = 0;
  virtual int writeFloat64Array(epicsFloat64 *value,
                                size_t nElements) = 0;
};
#endif /* ECMCASYNLINK_H_ */
