
#ifndef ECMC_ASYN_DATA_ITEM_H_
#define ECMC_ASYN_DATA_ITEM_H_

#include "inttypes.h"
#include "../main/ecmcDefinitions.h"
#include "ecmcAsynPortDriverUtils.h"
#include "asynPortDriver.h"
#include "ecmcOctetIF.h"  //LOG macros
#include "../ethercat/ecmcAsynLink.h"

#define ERROR_ASYN_PORT_NULL 0x220000
#define ERROR_ASYN_DATA_NULL 0x220001
#define ERROR_ASYN_DATA_TYPE_NOT_SUPPORTED 0x220002
#define ERROR_ASYN_CREATE_PARAM_FAIL 0x220003
#define ERROR_ASYN_PARAM_NOT_VALIDATED 0x220004
#define ERROR_ASYN_SUPPORTED_TYPES_ARRAY_FULL 0x220005
#define ERROR_ASYN_DATA_BUFFER_TO_SMALL 0x220006

#define ERROR_ASYN_MAX_SUPPORTED_TYPES_COUNT 10
#define ERROR_ASYN_NOT_REFRESHED_RETURN -1

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
  size_t       ecmcSize;          // Last refresh
  size_t       ecmcMaxSize;       // Max buffer size
  bool           ecmcDataIsArray;
  int            ecmcDataPointerValid;
  ECMCTIMESOURCE timeBase;
  uint64_t       timeStampRaw;
  epicsTimeStamp epicsTimestamp;
  int            alarmStatus;
  int            alarmSeverity;
  bool           refreshNeeded;
}ecmcParamInfo;

class ecmcAsynDataItem
{
public:
  ecmcAsynDataItem (ecmcAsynPortDriver *asynPortDriver,
                    const char *paramName,
                    asynParamType asynParType);
  /*ecmcAsynDataItem (ecmcAsynPortDriver *asynPortDriver,
                    ecmcAsynLink *asynLink,
                    const char *paramName,
                    asynParamType asynParType);*/

  ~ecmcAsynDataItem ();
  int setEcmcDataPointer(uint8_t *data,size_t bytes);
  int refreshParam(int force);
  int refreshParam(int force, size_t bytes);
  int refreshParam(int force, uint8_t *data, size_t bytes);
  int refreshParamRT(int force);
  int refreshParamRT(int force, size_t bytes);
  int refreshParamRT(int force, uint8_t *data, size_t bytes);
  //int writeParam(uint8_t *data, size_t bytes);
  //int readParam(uint8_t *data, size_t *bytes);
  int createParam();
  int createParam(const char *paramName,asynParamType asynParType);
  int createParam(const char *paramName,asynParamType asynParType, uint8_t *data,size_t bytes);
  int setAsynParSampleTimeMS(double sampleTime);
  int getAsynParameterIndex();
  int setAsynParameterType(asynParamType parType);
  int getAsynParameterType();
  int setAsynPortDriver(ecmcAsynPortDriver *asynPortDriver);  
  bool initialized();
  char * getName();  
  int32_t getSampleTimeCycles();
  ecmcParamInfo *getParamInfo();
  int addSupportedAsynType(asynParamType type);
  bool asynTypeSupported(asynParamType type);
  int getSupportedAsynTypeCount();
  asynParamType getSupportedAsynType(int index);
  void allowWriteToEcmc(bool allowWrite);
  bool writeToEcmcAllowed();
  bool willRefreshNext();
  asynStatus setAlarmParam(int alarm,int severity);
  int getAlarmStatus();
  int getAlarmSeverity();
  //int setAsynLink(ecmcAsynLink *asynLink);
  //ecmcAsynLink *getAsynLink();  

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

private:
  int asynTypeIsArray(asynParamType asynParType);
  asynStatus checkTypeAndSize(asynParamType type, size_t bytes);
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
  //size_t bytes_; //take from paramInfo_ instead
  ecmcParamInfo *paramInfo_;
  bool allowWriteToEcmc_;
  asynParamType supportedTypes_[ERROR_ASYN_MAX_SUPPORTED_TYPES_COUNT];
  int supportedTypesCounter_;
  ecmcAsynLink *asynLink_;
  //value limits for writes
  int64_t maxU8_;
  int64_t minS8_;
  int64_t maxS8_;
  int64_t maxU16_;
  int64_t minS16_;
  int64_t maxS16_;
};

#endif /* ECMC_ASYN_DATA_ITEM_H_ */
