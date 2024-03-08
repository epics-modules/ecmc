/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAsynPortDriver.h
*
*  Created on: Jan 29, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMC_ASYN_PORT_DRIVER_H_
#define ECMC_ASYN_PORT_DRIVER_H_


#include <epicsEvent.h>
#include <epicsTime.h>

#include "asynPortDriver.h"
#ifndef VERSION_INT
#  define VERSION_INT(V, R, M,\
                      P) (((V) << 24) | ((R) << 16) | ((M) << 8) | (P))
#endif // ifndef VERSION_INT

#define VERSION_INT_4_37            VERSION_INT(4, 37, 0, 0)
#define ECMC_ASYN_VERSION_INT VERSION_INT(ASYN_VERSION,\
                                          ASYN_REVISION,\
                                          ASYN_MODIFICATION,\
                                          0)

#if ECMC_ASYN_VERSION_INT >= VERSION_INT_4_37
#define ECMC_ASYN_ASYNPARAMINT64
#endif // if ECMC_ASYN_VERSION_INT >= VERSION_INT_4_37

#include "ecmcAsynDataItem.h"
#include "ecmcDefinitions.h"


class ecmcAsynPortDriver : public asynPortDriver {
public:
  ecmcAsynPortDriver(const char *portName,
                     int         paramTableSize,
                     int         autoConnect,
                     int         priority,
                     double      defaultSampleRateMS);
  ~ecmcAsynPortDriver();
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
  virtual asynStatus readInt32(asynUser   *pasynUser,
                               epicsInt32 *value);
  virtual asynStatus writeUInt32Digital(asynUser   *pasynUser,
                                        epicsUInt32 value,
                                        epicsUInt32 mask);
  virtual asynStatus readUInt32Digital(asynUser    *pasynUser,
                                       epicsUInt32 *value,
                                       epicsUInt32  mask);
  virtual asynStatus writeFloat64(asynUser    *pasynUser,
                                  epicsFloat64 value);
  virtual asynStatus readFloat64(asynUser     *pasynUser,
                                 epicsFloat64 *value);
  virtual asynStatus writeInt8Array(asynUser  *pasynUser,
                                    epicsInt8 *value,
                                    size_t     nElements);
  virtual asynStatus readInt8Array(asynUser  *pasynUser,
                                   epicsInt8 *value,
                                   size_t     nElements,
                                   size_t    *nIn);
  virtual asynStatus writeInt16Array(asynUser   *pasynUser,
                                     epicsInt16 *value,
                                     size_t      nElements);
  virtual asynStatus readInt16Array(asynUser   *pasynUser,
                                    epicsInt16 *value,
                                    size_t      nElements,
                                    size_t     *nIn);
  virtual asynStatus writeInt32Array(asynUser   *pasynUser,
                                     epicsInt32 *value,
                                     size_t      nElements);
  virtual asynStatus readInt32Array(asynUser   *pasynUser,
                                    epicsInt32 *value,
                                    size_t      nElements,
                                    size_t     *nIn);
  virtual asynStatus writeFloat32Array(asynUser     *pasynUser,
                                       epicsFloat32 *value,
                                       size_t        nElements);
  virtual asynStatus readFloat32Array(asynUser     *pasynUser,
                                      epicsFloat32 *value,
                                      size_t        nElements,
                                      size_t       *nIn);
  virtual asynStatus writeFloat64Array(asynUser     *pasynUser,
                                       epicsFloat64 *value,
                                       size_t        nElements);
  virtual asynStatus readFloat64Array(asynUser     *pasynUser,
                                      epicsFloat64 *value,
                                      size_t        nElements,
                                      size_t       *nIn);

#ifdef ECMC_ASYN_ASYNPARAMINT64
  virtual asynStatus readInt64(asynUser   *pasynUser,
                               epicsInt64 *value);
  virtual asynStatus writeInt64(asynUser  *pasynUser,
                                epicsInt64 value);

  virtual asynStatus writeInt64Array(asynUser   *pasynUser,
                                     epicsInt64 *value,
                                     size_t      nElements);
  virtual asynStatus readInt64Array(asynUser   *pasynUser,
                                    epicsInt64 *value,
                                    size_t      nElements,
                                    size_t     *nIn);
#endif //ECMC_ASYN_ASYNPARAMINT64

  virtual asynStatus drvUserCreate(asynUser    *pasynUser,
                                   const char  *drvInfo,
                                   const char **pptypeName,
                                   size_t      *psize);

  virtual void      report(FILE *fp,
                           int   details);
  void              grepParam(FILE       *fp,
                              const char *pattern,
                              int details);
  void              grepRecord(FILE       *fp,
                               const char *pattern);
  void              setAllowRtThreadCom(bool allowRtCom);
  bool              getAllowRtThreadCom();
  asynUser*         getTraceAsynUser();
  ecmcAsynDataItem* addNewAvailParam(const char    *name,
                                     asynParamType  type,
                                     uint8_t       *data,
                                     size_t         bytes,
                                     ecmcEcDataType dt,
                                     double         sampleTimeMs,
                                     bool           dieIfFail);
  ecmcAsynDataItem* addNewAvailParam(const char    *name,
                                     asynParamType  type,
                                     uint8_t       *data,
                                     size_t         bytes,
                                     ecmcEcDataType dt,
                                     bool           dieIfFail);
  int32_t           getFastestUpdateRate();
  int32_t           calcFastestUpdateRate();
  int               getDefaultSampleTimeMs();
  void              refreshAllInUseParamsRT();
  int               getEpicsState();
  void              setEpicsState(int state);

  ecmcDataItem*     findAvailDataItem(const char *name);
  ecmcAsynDataItem* findAvailParam(const char *name);
  bool              checkParamExist(const char *name);

private:
  void              initVars();
  asynStatus        checkParamNameAndId(int         paramIndex,
                                        const char *functionName);
  ecmcAsynDataItem* createNewParam(const char   *name,
                                   asynParamType type,
                                   bool          dieIfFail);
  asynStatus        appendInUseParam(ecmcAsynDataItem *dataItem,
                                     bool              dieIfFail);
  asynStatus        appendAvailParam(ecmcAsynDataItem *dataItem,
                                     bool              dieIfFail);

  void              reportParamInfo(FILE             *fp,
                                    ecmcAsynDataItem *param,
                                    int               listIndex,
                                    int               details);
  bool allowRtThreadCom_;
  ecmcAsynDataItem **pEcmcParamAvailArray_;
  ecmcAsynDataItem **pEcmcParamInUseArray_;
  int ecmcParamAvailCount_;
  int ecmcParamInUseCount_;
  int paramTableSize_;
  int defaultSampleTimeMS_;
  int defaultMaxDelayTimeMS_;
  ECMCTIMESOURCE defaultTimeSource_;
  int autoConnect_;
  unsigned int priority_;
  int32_t fastestParamUpdateCycles_;
  friend class paramList;
  int epicsState_;
};

#endif  /* ECMC_ASYN_PORT_DRIVER_H_ */
