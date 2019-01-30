/*
 * ecmcAsynPortDriver.h
 */

#ifndef ECMC_ASYN_PORT_DRIVER_H_
#define ECMC_ASYN_PORT_DRIVER_H_

#include "asynPortDriver.h"
#include "ecmcAsynPortDriverUtils.h"
#include "ecmcAsynDataItem.h"
#include <epicsEvent.h>

class ecmcAsynPortDriver : public asynPortDriver {
 public:
  ecmcAsynPortDriver(const char *portName,
                     int         paramTableSize,
                     int         autoConnect,
                     int         priority,
                     double      defaultSampleRateMS);
  virtual asynStatus writeOctet(asynUser   *pasynUser,
                                const char *value,
                                size_t      maxChars,
                                size_t     *nActual);
  virtual asynStatus readOctet(asynUser *pasynUser,
                               char     *value,
                               size_t    maxChars,
                               size_t   *nActual,
                               int      *eomReason);
  virtual asynStatus writeInt32(asynUser  *pasynUser,
                                epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser    *pasynUser,
                                  epicsFloat64 value);
  virtual asynStatus readInt8Array(asynUser  *pasynUser,
                                   epicsInt8 *value,
                                   size_t     nElements,
                                   size_t    *nIn);
  virtual asynStatus readInt16Array(asynUser   *pasynUser,
                                    epicsInt16 *value,
                                    size_t      nElements,
                                    size_t     *nIn);
  virtual asynStatus readInt32Array(asynUser   *pasynUser,
                                    epicsInt32 *value,
                                    size_t      nElements,
                                    size_t     *nIn);
  virtual asynStatus readFloat32Array(asynUser     *pasynUser,
                                      epicsFloat32 *value,
                                      size_t        nElements,
                                      size_t       *nIn);
  virtual asynStatus readFloat64Array(asynUser     *pasynUser,
                                      epicsFloat64 *value,
                                      size_t        nElements,
                                      size_t       *nIn);
  virtual asynStatus drvUserCreate(asynUser *pasynUser,
                                   const char *drvInfo,
                                   const char **pptypeName,
                                   size_t *psize);
                                   
  virtual void report(FILE *fp, int details);
  void      setAllowRtThreadCom(bool allowRtCom);
  bool      getAllowRtThreadCom();
  asynUser* getTraceAsynUser();
  ecmcAsynDataItem *addNewDefaultParam(const char * name,
                                       asynParamType type,
                                       uint8_t *data,
                                       size_t bytes,
                                       bool dieIfFail);
   int32_t getFastestUpdateRate();
   int32_t calcFastestUpdateRate();
 private:
  int readArrayGeneric(asynUser   *pasynUser,
                       epicsUInt8 *value,
                       size_t      nElements,
                       size_t     *nIn,
                       size_t      typeSize,
                       const char *functionName);
  void initVars();
  asynStatus validateDrvInfo(const char *drvInfo);
  asynStatus getRecordInfoFromDrvInfo(const char *drvInfo,
                                      ecmcParamInfo *paramInfo);
  asynStatus parseInfofromDrvInfo(const char* drvInfo,
                                     ecmcParamInfo *paramInfo);
  ecmcAsynDataItem *createNewDefaultParam(const char * name, asynParamType type,bool dieIfFail);                                     
  asynStatus appendAsynDataItem(ecmcAsynDataItem *dataItem,bool dieIfFail);
  bool allowRtThreadCom_;
  ecmcAsynDataItem  **pEcmcParamArray_;
  int ecmcParamArrayCount_;
  int paramTableSize_;
  int defaultSampleTimeMS_;
  int defaultMaxDelayTimeMS_;
  ECMCTIMESOURCE defaultTimeSource_;
  int autoConnect_;
  unsigned int priority_;
  int32_t fastestParamUpdateCycles_;
};

#endif  /* ECMC_ASYN_PORT_DRIVER_H_ */
