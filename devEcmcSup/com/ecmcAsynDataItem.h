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

#ifndef ECMC_IS_PLUGIN
#include "../main/ecmcDefinitions.h"
#include "../com/ecmcAsynPortDriverUtils.h"
#else
#include "ecmcDefinitions.h"
#include "ecmcAsynPortDriverUtils.h"
#endif
#include "asynPortDriver.h"
#include "ecmcDataItem.h"

#ifndef VERSION_INT
#  define VERSION_INT(V,R,M,P) ( ((V)<<24) | ((R)<<16) | ((M)<<8) | (P))
#endif

#define VERSION_INT_4_37            VERSION_INT(4,37,0,0)
#define ECMC_ASYN_VERSION_INT VERSION_INT(ASYN_VERSION,ASYN_REVISION,ASYN_MODIFICATION,0)

#if ECMC_ASYN_VERSION_INT >= VERSION_INT_4_37
#define ECMC_ASYN_ASYNPARAMINT64
#endif

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

// Asyn Parameter informtaion
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
  bool           dataIsArray;
  int            alarmStatus;
  int            alarmSeverity;
  bool           refreshNeeded;
  bool           cmdUint64ToFloat64;
  bool           cmdInt64ToFloat64;
  bool           cmdFloat64ToInt32;
}ecmcParamInfo;

/**
*  This class handles all asyn related information.
*  All ecmc related information is handled in the class ecmcDataItem.
*/
class ecmcAsynDataItem : public ecmcDataItem
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

  int refreshParam(int force);
  int refreshParam(int force, size_t bytes);
  int refreshParam(int force, uint8_t *data, size_t bytes);
  int refreshParamRT(int force);
  int refreshParamRT(int force, size_t bytes);
  int refreshParamRT(int force, uint8_t *data, size_t bytes);

  int createParam();
  int createParam(const char *paramName, asynParamType asynParType);
  int createParam(const char *paramName, asynParamType asynParType, 
                  uint8_t *data, size_t bytes);

  int setAsynParSampleTimeMS(double sampleTime);
  int getAsynParameterIndex();
  int setAsynParameterType(asynParamType parType);
  asynParamType getAsynParameterType();
  int setAsynPortDriver(ecmcAsynPortDriver *asynPortDriver);  
  bool linkedToAsynClient();  // Param linked to record or other client
  char *getParamName();
  char *getDrvInfo();
  char *getDtyp();
  char *getRecordType();
  char *getRecordName();
  int32_t getSampleTimeCycles();
  double getSampleTimeMs();
  ecmcParamInfo *getParamInfo();
  int addSupportedAsynType(asynParamType type);
  bool asynTypeSupported(asynParamType type);
  int getSupportedAsynTypeCount();
  asynParamType getAsynType();
  char * getAsynTypeName();
  asynParamType getSupportedAsynType(int index);
  bool willRefreshNext();
  asynStatus setAlarmParam(int alarm,int severity);
  int getAlarmStatus();
  int getAlarmSeverity();
    
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

#ifdef ECMC_ASYN_ASYNPARAMINT64
  asynStatus readInt64(epicsInt64 *value);
  asynStatus writeInt64(epicsInt64 value);

  asynStatus writeInt64Array(epicsInt64 *value,
                             size_t nElements);
  asynStatus readInt64Array(epicsInt64 *value,
                            size_t nElements,
                            size_t *nIn);
#endif //ECMC_ASYN_ASYNPARAMINT64

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

  // variables
  ecmcAsynPortDriver *asynPortDriver_;
  
  ecmcParamInfo       paramInfo_;
  asynParamType supportedTypes_[ERROR_ASYN_MAX_SUPPORTED_TYPES_COUNT];
  int asynUpdateCycleCounter_;  
  int supportedTypesCounter_;  

  // Add function to allow action on writes
  asynStatus (*fctPtrExeCmd_)(void* data, size_t bytes, asynParamType asynParType,void *userObj);
  bool useExeCmdFunc_;
  void* exeCmdUserObj_;

  // Baseclass virtuals from ecmcDataItem class
  void refresh();

};

#endif /* ECMC_ASYN_DATA_ITEM_H_ */
