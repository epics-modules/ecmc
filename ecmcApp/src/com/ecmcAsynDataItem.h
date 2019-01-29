
#ifndef ECMC_ASYN_DATA_ITEM_H_
#define ECMC_ASYN_DATA_ITEM_H_

#include "inttypes.h"
#include "../main/ecmcDefinitions.h"
#include "ecmcAsynPortDriverUtils.h"
#include "asynPortDriver.h"

#define ERROR_ASYN_PORT_NULL 0x220000
#define ERROR_ASYN_DATA_NULL 0x220001
#define ERROR_ASYN_DATA_TYPE_NOT_SUPPORTED 0x220002
#define ERROR_ASYN_CREATE_PARAM_FAIL 0x220003

class ecmcAsynPortDriver;  //Include in cpp

class ecmcAsynDataItem
{
public:
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
  int getAsynParameterType();
  int setAsynPortDriver(ecmcAsynPortDriver *asynPortDriver);  
  ecmcParamInfo *getParamInfo();

private:
  ecmcAsynPortDriver *asynPortDriver_;
  int asynUpdateCycleCounter_;
  uint8_t *data_;
  size_t bytes_;
  ecmcParamInfo *paramInfo_;
};

#endif /* ECMC_ASYN_DATA_ITEM_H_ */
