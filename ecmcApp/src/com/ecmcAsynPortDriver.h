/*
 * ecmcAsynPortDriver.h
 */

#ifndef ECMC_ASYN_PORT_DRIVER_H_
#define ECMC_ASYN_PORT_DRIVER_H_

#include "asynPortDriver.h"
#include <epicsEvent.h>
#include <epicsTime.h>
#include "../com/ecmcAsynPortDriverUtils.h"
#include "../com/ecmcAsynDataItem.h"

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
  virtual asynStatus readInt32(asynUser *pasynUser,
                               epicsInt32 *value);
  virtual asynStatus writeFloat64(asynUser    *pasynUser,
                                  epicsFloat64 value);
  virtual asynStatus readFloat64(asynUser *pasynUser,
                                 epicsFloat64 *value);                                 
  virtual asynStatus writeInt8Array(asynUser *pasynUser,
                                    epicsInt8 *value,
                                    size_t nElements);
  virtual asynStatus readInt8Array(asynUser  *pasynUser,
                                   epicsInt8 *value,
                                   size_t     nElements,
                                   size_t    *nIn);
  virtual asynStatus writeInt16Array(asynUser *pasynUser,
                                     epicsInt16 *value,
                                     size_t nElements);
  virtual asynStatus readInt16Array(asynUser   *pasynUser,
                                    epicsInt16 *value,
                                    size_t      nElements,
                                    size_t     *nIn);
  virtual asynStatus writeInt32Array(asynUser *pasynUser,
                                     epicsInt32 *value,
                                     size_t nElements);
  virtual asynStatus readInt32Array(asynUser   *pasynUser,
                                    epicsInt32 *value,
                                    size_t      nElements,
                                    size_t     *nIn);
  virtual asynStatus writeFloat32Array(asynUser *pasynUser,
                                       epicsFloat32 *value,
                                       size_t nElements);
  virtual asynStatus readFloat32Array(asynUser     *pasynUser,
                                      epicsFloat32 *value,
                                      size_t        nElements,
                                      size_t       *nIn);
  virtual asynStatus writeFloat64Array(asynUser *pasynUser,
                                       epicsFloat64 *value,
                                       size_t nElements);
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
  ecmcAsynDataItem *addNewAvailParam(const char * name,
                                     asynParamType type,
                                     uint8_t *data,
                                     size_t bytes,
                                     bool dieIfFail);
   int32_t getFastestUpdateRate();
   int32_t calcFastestUpdateRate();
   void    refreshAllInUseParamsRT();

 private:
  // int readArrayGeneric(asynUser   *pasynUser,
  //                      epicsUInt8 *value,
  //                      size_t      nElements,
  //                      size_t     *nIn,
  //                      size_t      typeSize,
  //                      const char *functionName);
  // asynStatus writeArrayGeneric(asynUser *pasynUser,
  //                             asynParamType allowedType,
  //                             const void *epicsDataBuffer,
  //                             size_t nEpicsBufferBytes);     
  void initVars();
  asynStatus checkParamNameAndId(int paramIndex,const char *functionName);
  asynStatus validateDrvInfo(const char *drvInfo);
  asynStatus getRecordInfoFromDrvInfo(const char *drvInfo,
                                      ecmcParamInfo *paramInfo);
  asynStatus parseInfofromDrvInfo(const char* drvInfo,
                                     ecmcParamInfo *paramInfo);
  ecmcAsynDataItem *createNewParam(const char * name,
                                   asynParamType type,
                                   bool dieIfFail);
  asynStatus appendInUseParam(ecmcAsynDataItem *dataItem,bool dieIfFail);
  asynStatus appendAvailParam(ecmcAsynDataItem *dataItem, bool dieIfFail);
  ecmcAsynDataItem *findAvailParam(const char * name);
  void reportParamInfo(FILE *fp,ecmcAsynDataItem *param, int listIndex);
  bool allowRtThreadCom_;
  ecmcAsynDataItem  **pEcmcParamAvailArray_;
  ecmcAsynDataItem  **pEcmcParamInUseArray_;
  int ecmcParamAvailCount_;
  int ecmcParamInUseCount_;
  int paramTableSize_;
  int defaultSampleTimeMS_;
  int defaultMaxDelayTimeMS_;
  ECMCTIMESOURCE defaultTimeSource_;
  int autoConnect_;
  unsigned int priority_;
  int32_t fastestParamUpdateCycles_;
};

#endif  /* ECMC_ASYN_PORT_DRIVER_H_ */
