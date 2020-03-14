/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcAsynDataItem.h
*
*  Created on: Jan 29, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMC_ASYN_DATA_ITEM_H_
#define ECMC_ASYN_DATA_ITEM_H_

#include <bits/stdc++.h> 
#include "inttypes.h"
#include "../main/ecmcDefinitions.h"
#include "ecmcAsynPortDriverUtils.h"
#include "asynPortDriver.h"
#include "ecmcOctetIF.h"  //LOG macros

#define ERROR_ASYN_PORT_NULL 0x220000
#define ERROR_ASYN_DATA_NULL 0x220001
#define ERROR_ASYN_DATA_TYPE_NOT_SUPPORTED 0x220002
#define ERROR_ASYN_CREATE_PARAM_FAIL 0x220003
#define ERROR_ASYN_PARAM_NOT_VALIDATED 0x220004
#define ERROR_ASYN_SUPPORTED_TYPES_ARRAY_FULL 0x220005
#define ERROR_ASYN_DATA_BUFFER_TO_SMALL 0x220006
#define ERROR_ASYN_WRITE_VALUE_OUT_OF_RANGE 0x220007
#define ERROR_ASYN_REFRESH_FAIL 0x220008
#define ERROR_ASYN_CMD_FAIL 0x220009

#define ERROR_ASYN_MAX_SUPPORTED_TYPES_COUNT 10
#define ERROR_ASYN_NOT_REFRESHED_RETURN -1

typedef asynStatus(*ecmcExeCmdFcn)(void*,size_t,asynParamType,void*);

class ecmcAsynPortDriver;  //Include in cpp

typedef struct ecmcParamInfo{
  char           *recordName;
  char           *recordType;
  char           *scan;
  char           *dtyp;
  char           *inp;
  char           *out;
  char           *drvInfo;
  int            initialized;
  asynParamType  asynType;
  char*          asynTypeStr;
  asynUser       *pasynUser;
  int            asynAddr;
  bool           isIOIntr;
  double         sampleTimeMS;      // milli seconds
  int32_t        sampleTimeCycles;  // milli seconds
  int            index;             // also used as hUser for ads callback
  char           *name;
  size_t         ecmcSize;          // Last refresh
  size_t         ecmcMaxSize;       // Max buffer size
  bool           ecmcDataIsArray;
  int            ecmcDataPointerValid;
  int            alarmStatus;
  int            alarmSeverity;
  bool           refreshNeeded;
  bool           arrayCheckSize;
  bool           cmdUint64ToFloat64;
  bool           cmdInt64ToFloat64;
  bool           cmdFloat64ToInt32;
}ecmcParamInfo;

class ecmcAsynDataItem
{
public:
  ecmcAsynDataItem (ecmcAsynPortDriver *asynPortDriver,
                    const char *paramName,
                    asynParamType asynParType,
                    ecmcEcDataType dt,
                    double updateRateMs); //if "-1" then realtime loop
  ecmcAsynDataItem (ecmcAsynPortDriver *asynPortDriver,
                    const char *paramName,
                    asynParamType asynParType,
                    ecmcEcDataType dt); //sample time -1 update rate
  ecmcAsynDataItem (ecmcAsynPortDriver *asynPortDriver);
  ~ecmcAsynDataItem ();
  int setEcmcDataPointer(uint8_t *data,size_t bytes);  
  int refreshParam(int force);
  int refreshParam(int force, size_t bytes);
  int refreshParam(int force, uint8_t *data, size_t bytes);
  int refreshParamRT(int force);
  int refreshParamRT(int force, size_t bytes);
  int refreshParamRT(int force, uint8_t *data, size_t bytes);
  int createParam();
  int createParam(const char *paramName,asynParamType asynParType);
  int createParam(const char *paramName,asynParamType asynParType, uint8_t *data,size_t bytes);
  int setAsynParSampleTimeMS(double sampleTime);
  int getAsynParameterIndex();
  int setAsynParameterType(asynParamType parType);
  asynParamType getAsynParameterType();
  int setAsynPortDriver(ecmcAsynPortDriver *asynPortDriver);  
  bool initialized();
  char * getName();
  char * getDrvInfo();
  char * getDtyp();
  char * getRecordType();
  char * getRecordName();
  int32_t getSampleTimeCycles();
  double getSampleTimeMs();
  ecmcParamInfo *getParamInfo();
  int addSupportedAsynType(asynParamType type);
  bool asynTypeSupported(asynParamType type);
  int getSupportedAsynTypeCount();
  asynParamType getAsynType();
  char * getAsynTypeName();
  asynParamType getSupportedAsynType(int index);
  void allowWriteToEcmc(bool allowWrite);
  bool writeToEcmcAllowed();
  bool willRefreshNext();
  asynStatus setAlarmParam(int alarm,int severity);
  int getAlarmStatus();
  int getAlarmSeverity();
  void setEcmcMaxValueInt(int64_t intMax);
  void setEcmcMinValueInt(int64_t intMin);
  void setEcmcBitCount(size_t bits);
  int64_t getEcmcMinValueInt();
  int64_t getEcmcMaxValueInt();
  size_t getEcmcBitCount();
  void setArrayCheckSize(bool check);
  bool getArrayCheckSize();
  void setType(ecmcEcDataType dt);
    
  asynStatus setDrvInfo(const char *drvInfo);

  asynStatus readInt32(epicsInt32 *value);
  asynStatus writeInt32(epicsInt32 value);
  asynStatus readUInt32Digital(epicsUInt32 *value,
                               epicsUInt32 mask);
  asynStatus writeUInt32Digital(epicsUInt32 value,
                                epicsUInt32 mask);
  asynStatus readFloat64(epicsFloat64 *value);
  asynStatus writeFloat64(epicsFloat64 value);
  asynStatus readInt8Array(epicsInt8 *value, 
                           size_t nElements,
                           size_t *nIn);
  asynStatus writeInt8Array(epicsInt8 *value,
                            size_t nElements);
  asynStatus readInt16Array(epicsInt16 *value,
                            size_t nElements,
                            size_t *nIn);
  asynStatus writeInt16Array(epicsInt16 *value,
                             size_t nElements);
  asynStatus readInt32Array(epicsInt32 *value,
                            size_t nElements,
                            size_t *nIn);
  asynStatus writeInt32Array(epicsInt32 *value,
                             size_t nElements);
  asynStatus readFloat32Array(epicsFloat32 *value,
                              size_t nElements,
                              size_t *nIn);
  asynStatus writeFloat32Array(epicsFloat32 *value,
                               size_t nElements);
  asynStatus readFloat64Array(epicsFloat64 *value,
                              size_t nElements,
                              size_t *nIn);
  asynStatus writeFloat64Array(epicsFloat64 *value,
                               size_t nElements);
  
  ecmcEcDataType getEcDataType();

  asynStatus setExeCmdFunctPtr(ecmcExeCmdFcn func, void* userObj);

private:
  asynStatus validateDrvInfo(const char *drvInfo);
  asynStatus getRecordInfoFromDrvInfo(const char *drvInfo);
  asynStatus parseInfofromDrvInfo(const char* drvInfo);

  int asynTypeIsArray(asynParamType asynParType);
  asynStatus readGeneric(uint8_t *data,
                         size_t bytesToRead,
                         asynParamType type,
                         size_t *readBytes);
  asynStatus writeGeneric(uint8_t *data,
                          size_t bytes,
                          asynParamType type,
                          size_t *writtenBytes);
  ecmcAsynPortDriver *asynPortDriver_;
  int asynUpdateCycleCounter_;
  uint8_t *data_;
  ecmcParamInfo paramInfo_;
  bool allowWriteToEcmc_;
  asynParamType supportedTypes_[ERROR_ASYN_MAX_SUPPORTED_TYPES_COUNT];
  int supportedTypesCounter_;  
  int checkIntRange_;
  //value limits for writes
  int64_t intMax_;
  int64_t intMin_;
  size_t intBits_;
  ecmcEcDataType dataType_;

  // Add function to allow action on writes
  asynStatus (*fctPtrExeCmd_)(void* data, size_t bytes, asynParamType asynParType,void *userObj);
  bool useExeCmdFunc_;
  void* exeCmdUserObj_;
  double updateRateMs_;
};

#endif /* ECMC_ASYN_DATA_ITEM_H_ */
