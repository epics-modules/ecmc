/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAsynDataItem.cpp
*
*  Created on: Jan 29, 2019
*      Author: anderssandstrom
*              Jeong Han Lee
*
\*************************************************************************/

#include "ecmcAsynDataItem.h"
#include "ecmcOctetIF.h"  // LOG macros
#include "ecmcAsynPortDriver.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsExport.h>
#include <epicsEvent.h>
#include <dbCommon.h>
#include <dbBase.h>
#include <dbStaticLib.h>
#include <dbAccess.h>

static const char *driverName = "ecmcAsynPortDriver";

extern double mcuFrequency;
extern double mcuPeriod;
static int compar(const void *pkey, const void *pelem) {
  return *(int *)pkey - *(int *)pelem;
}

ecmcAsynDataItem::ecmcAsynDataItem(ecmcAsynPortDriver *asynPortDriver,
                                   const char         *paramName,
                                   asynParamType       asynParType,
                                   ecmcEcDataType      dt,
                                   double              updateRateMs) :
  ecmcDataItem(paramName) {
  asynPortDriver_            = asynPortDriver;
  asynUpdateCycleCounter_    = 0;
  supportedTypesCounter_     = 0;
  fctPtrExeCmd_              = NULL;
  useExeCmdFunc_             = false;
  exeCmdUserObj_             = NULL;
  dataItem_.dataType         = dt;
  dataItem_.dataElementSize  = getEcDataTypeByteSize(dt);
  dataItem_.dataUpdateRateMs = updateRateMs;

  for (int i = 0; i < ERROR_ASYN_MAX_SUPPORTED_TYPES_COUNT; i++) {
    supportedTypes_[i] = asynParamNotDefined;
  }
  memset(&paramInfo_, 0, sizeof(ecmcParamInfo));
  paramInfo_.name        = strdup(paramName);
  paramInfo_.asynType    = asynParType;
  paramInfo_.dataIsArray = asynTypeIsArray(asynParType);
  addSupportedAsynType(asynParType);
}

ecmcAsynDataItem::ecmcAsynDataItem(ecmcAsynPortDriver *asynPortDriver,
                                   const char         *paramName,
                                   asynParamType       asynParType,
                                   ecmcEcDataType      dt) :
  ecmcDataItem(paramName) {
  asynPortDriver_           = asynPortDriver;
  asynUpdateCycleCounter_   = 0;
  supportedTypesCounter_    = 0;
  fctPtrExeCmd_             = NULL;
  useExeCmdFunc_            = false;
  exeCmdUserObj_            = NULL;
  dataItem_.dataType        = dt;
  dataItem_.dataElementSize = getEcDataTypeByteSize(dt);

  for (int i = 0; i < ERROR_ASYN_MAX_SUPPORTED_TYPES_COUNT; i++) {
    supportedTypes_[i] = asynParamNotDefined;
  }
  memset(&paramInfo_, 0, sizeof(ecmcParamInfo));
  paramInfo_.name        = strdup(paramName);
  paramInfo_.asynType    = asynParType;
  paramInfo_.dataIsArray = asynTypeIsArray(asynParType);
  addSupportedAsynType(asynParType);
}

ecmcAsynDataItem::ecmcAsynDataItem(ecmcAsynPortDriver *asynPortDriver) :
  ecmcDataItem("empty") {
  memset(&paramInfo_, 0, sizeof(ecmcParamInfo));
  asynPortDriver_         = asynPortDriver;
  asynUpdateCycleCounter_ = 0;
  supportedTypesCounter_  = 0;
  dataItem_.dataType      = ECMC_EC_NONE;
  paramInfo_.name         = strdup("empty");
  paramInfo_.asynType     = asynParamNotDefined;
  paramInfo_.dataIsArray  = 0;
  fctPtrExeCmd_           = NULL;
  useExeCmdFunc_          = false;
  exeCmdUserObj_          = NULL;

  for (int i = 0; i < ERROR_ASYN_MAX_SUPPORTED_TYPES_COUNT; i++) {
    supportedTypes_[i] = asynParamNotDefined;
  }
}

ecmcAsynDataItem::~ecmcAsynDataItem() {
  free(paramInfo_.recordName);
  paramInfo_.recordName = NULL;
  free(paramInfo_.recordType);
  paramInfo_.recordType = NULL;
  free(paramInfo_.scan);
  paramInfo_.scan = NULL;
  free(paramInfo_.dtyp);
  paramInfo_.dtyp = 0;
  free(paramInfo_.inp);
  paramInfo_.inp = NULL;
  free(paramInfo_.out);
  paramInfo_.out = NULL;
  free(paramInfo_.drvInfo);
  paramInfo_.drvInfo = NULL;
  free(paramInfo_.asynTypeStr);
  paramInfo_.asynTypeStr = NULL;
  free(paramInfo_.name);
  paramInfo_.name = NULL;
}

int ecmcAsynDataItem::refreshParamRT(int force) {
  if (!asynPortDriver_->getAllowRtThreadCom()) {
    return ERROR_ASYN_NOT_REFRESHED_RETURN;
  }
  return refreshParam(force, dataItem_.data, dataItem_.dataSize);
}

int ecmcAsynDataItem::refreshParam(int force) {
  return refreshParam(force, dataItem_.data, dataItem_.dataSize);
}

int ecmcAsynDataItem::refreshParamRT(int force, size_t bytes) {
  if (!asynPortDriver_->getAllowRtThreadCom()) {
    return ERROR_ASYN_NOT_REFRESHED_RETURN;
  }
  return refreshParam(force, dataItem_.data, bytes);
}

int ecmcAsynDataItem::refreshParam(int force, size_t bytes) {
  return refreshParam(force, dataItem_.data, bytes);
}

int ecmcAsynDataItem::refreshParamRT(int force, uint8_t *data, size_t bytes) {
  if (!asynPortDriver_->getAllowRtThreadCom()) {
    return ERROR_ASYN_NOT_REFRESHED_RETURN;
  }
  return refreshParam(force, data, bytes);
}

void ecmcAsynDataItem::refresh() {
  // Call base class refresh
  ecmcDataItem::refresh();
}

/*
* Returns 0 if refreshed.
* Retrun -1 or error code if not refreshed.
*/
int ecmcAsynDataItem::refreshParam(int force, uint8_t *data, size_t bytes) {
  // set data pointer and size if param is not initialized (linked to record)
  dataItem_.data     = data;
  dataItem_.dataSize = bytes;

  /** Just asyn related below so call refresh() here
  * (which calls baseclass::refresh) to update other data subscribers!
  */
  refresh();

  // Do not update if not linked to epics-record
  if (!paramInfo_.initialized) {
    return 0;
  }


  if (!force) {
    if (paramInfo_.sampleTimeCycles < 0) {
      return ERROR_ASYN_NOT_REFRESHED_RETURN;
    }

    if ((paramInfo_.sampleTimeCycles >= 0) &&
        (asynUpdateCycleCounter_ < paramInfo_.sampleTimeCycles - 1)) {
      asynUpdateCycleCounter_++;
      return ERROR_ASYN_NOT_REFRESHED_RETURN;  // Not refreshed
    }
  }

  if ((data == 0) || (bytes < 0)) {
    return ERROR_ASYN_DATA_NULL;
  }

  if ((bytes > ecmcMaxSize_) && arrayCheckSize_) {
    bytes = ecmcMaxSize_;
  }

  dataItem_.dataSize = bytes;

  asynStatus stat = asynError;

  switch (paramInfo_.asynType) {
  case asynParamUInt32Digital:
    stat = asynPortDriver_->setUIntDigitalParam(ECMC_ASYN_DEFAULT_LIST,
                                                paramInfo_.index,
                                                *((epicsInt32 *)data),
                                                0xFFFFFFFF);
    break;

  case asynParamInt32:

    switch (dataItem_.dataType) {
    case ECMC_EC_S8:
      stat = asynPortDriver_->setIntegerParam(ECMC_ASYN_DEFAULT_LIST,
                                              paramInfo_.index,
                                              *((epicsInt8 *)data));
      break;

    case ECMC_EC_S16:
      stat = asynPortDriver_->setIntegerParam(ECMC_ASYN_DEFAULT_LIST,
                                              paramInfo_.index,
                                              *((epicsInt16 *)data));
      break;

    default:
      stat = asynPortDriver_->setIntegerParam(ECMC_ASYN_DEFAULT_LIST,
                                              paramInfo_.index,
                                              *((epicsInt32 *)data));
    }
    break;

  case asynParamFloat64:

    if (paramInfo_.cmdInt64ToFloat64) {
      if (dataItem_.dataSize == sizeof(int64_t)) {
        stat = asynPortDriver_->setDoubleParam(ECMC_ASYN_DEFAULT_LIST,
                                               paramInfo_.index,
                                               static_cast<epicsFloat64>(*(
                                                                           int64_t
                                                                           *)
                                                                         data));
        break;
      }
    }

    if (paramInfo_.cmdUint64ToFloat64) {
      if (dataItem_.dataSize == sizeof(uint64_t)) {
        stat = asynPortDriver_->setDoubleParam(ECMC_ASYN_DEFAULT_LIST,
                                               paramInfo_.index,
                                               static_cast<epicsFloat64>(*(
                                                                           uint64_t
                                                                           *)
                                                                         data));
        break;
      }
    }

    if (paramInfo_.cmdUint32ToFloat64) {
      if (dataItem_.dataSize == sizeof(uint32_t)) {
        stat = asynPortDriver_->setDoubleParam(ECMC_ASYN_DEFAULT_LIST,
                                               paramInfo_.index,
                                               static_cast<epicsFloat64>(*(
                                                                           uint32_t
                                                                           *)
                                                                         data));
        break;
      }
    }

    if (paramInfo_.cmdFloat64ToInt32) {
      if (dataItem_.dataSize == sizeof(double)) {
        stat = asynPortDriver_->setIntegerParam(ECMC_ASYN_DEFAULT_LIST,
                                                paramInfo_.index,
                                                static_cast<epicsInt32>(*(
                                                                          double
                                                                          *)
                                                                        data));
        break;
      }
    }

    if (dataItem_.dataType == ECMC_EC_F32) {
      stat = asynPortDriver_->setDoubleParam(ECMC_ASYN_DEFAULT_LIST,
                                             paramInfo_.index,
                                             static_cast<epicsFloat64>(*(float
                                                                         *)data));
      break;
    }

    stat = asynPortDriver_->setDoubleParam(ECMC_ASYN_DEFAULT_LIST,
                                           paramInfo_.index,
                                           *((epicsFloat64 *)data));
    break;

  case asynParamInt8Array:
    stat = asynPortDriver_->doCallbacksInt8Array((epicsInt8 *)data,
                                                 bytes,
                                                 paramInfo_.index,
                                                 0);
    break;

  case asynParamInt16Array:
    stat = asynPortDriver_->doCallbacksInt16Array((epicsInt16 *)data,
                                                  bytes / sizeof(epicsInt16),
                                                  paramInfo_.index,
                                                  0);
    break;

  case asynParamInt32Array:
    stat = asynPortDriver_->doCallbacksInt32Array((epicsInt32 *)data,
                                                  bytes / sizeof(epicsInt32),
                                                  paramInfo_.index,
                                                  0);
    break;

  case asynParamFloat32Array:
    stat = asynPortDriver_->doCallbacksFloat32Array((epicsFloat32 *)data,
                                                    bytes / sizeof(epicsFloat32),
                                                    paramInfo_.index,
                                                    0);
    break;

  case asynParamFloat64Array:
    stat = asynPortDriver_->doCallbacksFloat64Array((epicsFloat64 *)data,
                                                    bytes / sizeof(epicsFloat64),
                                                    paramInfo_.index,
                                                    0);
    break;

#ifdef ECMC_ASYN_ASYNPARAMINT64
  case asynParamInt64:
    stat = asynPortDriver_->setInteger64Param(ECMC_ASYN_DEFAULT_LIST,
                                              paramInfo_.index,
                                              *((epicsInt64 *)data));
    break;

  case asynParamInt64Array:
    stat = asynPortDriver_->doCallbacksInt64Array((epicsInt64 *)data,
                                                  bytes / sizeof(epicsInt64),
                                                  paramInfo_.index,
                                                  0);
    break;
#endif // ECMC_ASYN_ASYNPARAMINT64

  default:
    return ERROR_ASYN_DATA_TYPE_NOT_SUPPORTED;

    break;
  }

  asynUpdateCycleCounter_ = 0;

  if (stat != asynSuccess) {
    asynPrint(
      asynPortDriver_->getTraceAsynUser(),
      ASYN_TRACE_ERROR,
      "ecmcAsynDataItem::refreshParam: ERROR: Refresh failed for parameter %s, bytes %zu, force %d, sample time %d (0x%x).\n",
      getName(),
      bytes,
      force,
      paramInfo_.sampleTimeCycles,
      ERROR_ASYN_REFRESH_FAIL);
    return ERROR_ASYN_REFRESH_FAIL;
  }
  return 0;
}

int ecmcAsynDataItem::createParam() {
  return createParam(dataItem_.name, paramInfo_.asynType);
}

int ecmcAsynDataItem::createParam(const char   *paramName,
                                  asynParamType asynParType,
                                  uint8_t      *data,
                                  size_t        bytes) {
  setEcmcDataPointer(data, bytes);
  return createParam(paramName, asynParType);
}

int ecmcAsynDataItem::createParam(const char   *paramName,
                                  asynParamType asynParType) {
  if (asynPortDriver_ == 0) {
    return ERROR_ASYN_PORT_NULL;
  }
  paramInfo_.name     = strdup(paramName);
  paramInfo_.asynType = asynParType;

  // ECMC double, epics record int32
  if (paramInfo_.cmdFloat64ToInt32 &&
      (paramInfo_.asynType == asynParamFloat64) &&
      (dataItem_.dataSize == sizeof(epicsFloat64))) {
    asynStatus status = asynPortDriver_->createParam(ECMC_ASYN_DEFAULT_LIST,
                                                     paramName,
                                                     asynParamInt32,
                                                     &paramInfo_.index);
    return (status == asynSuccess) ? 0 : ERROR_ASYN_CREATE_PARAM_FAIL;
  }

  asynStatus status = asynPortDriver_->createParam(ECMC_ASYN_DEFAULT_LIST,
                                                   paramName,
                                                   paramInfo_.asynType,
                                                   &paramInfo_.index);
  return (status == asynSuccess) ? 0 : ERROR_ASYN_CREATE_PARAM_FAIL;
}

int ecmcAsynDataItem::getAsynParameterIndex() {
  return paramInfo_.index;
}

int ecmcAsynDataItem::setAsynParameterType(asynParamType parType) {
  paramInfo_.asynType = parType;
  return 0;
}

asynParamType ecmcAsynDataItem::getAsynParameterType() {
  return paramInfo_.asynType;
}

int ecmcAsynDataItem::setAsynPortDriver(ecmcAsynPortDriver *asynPortDriver) {
  asynPortDriver_ = asynPortDriver;
  return 0;
}

int ecmcAsynDataItem::setAsynParSampleTimeMS(double sampleTime) {
  paramInfo_.sampleTimeMS     = sampleTime;
  paramInfo_.sampleTimeCycles = (int32_t)(sampleTime / 1000.0 * mcuFrequency);

  if (paramInfo_.sampleTimeMS == -1) {
    paramInfo_.sampleTimeCycles = -1;
  }
  return 0;
}

ecmcParamInfo * ecmcAsynDataItem::getParamInfo() {
  return &paramInfo_;
}

bool ecmcAsynDataItem::linkedToAsynClient() {
  return paramInfo_.initialized;
}

int32_t ecmcAsynDataItem::getSampleTimeCycles() {
  return paramInfo_.sampleTimeCycles;
}

double ecmcAsynDataItem::getSampleTimeMs() {
  return paramInfo_.sampleTimeCycles * 1000.0 / mcuFrequency;
}

char * ecmcAsynDataItem::getParamName() {
  return paramInfo_.name;
}

char * ecmcAsynDataItem::getDrvInfo() {
  return paramInfo_.drvInfo;
}

char * ecmcAsynDataItem::getDtyp() {
  return paramInfo_.dtyp;
}

char * ecmcAsynDataItem::getRecordType() {
  return paramInfo_.recordType;
}

char * ecmcAsynDataItem::getRecordName() {
  return paramInfo_.recordName;
}

int ecmcAsynDataItem::addSupportedAsynType(asynParamType type) {
  // check so not already in list
  if (asynTypeSupported(type)) {
    return 0;
  }

  if (supportedTypesCounter_ < ERROR_ASYN_MAX_SUPPORTED_TYPES_COUNT - 1) {
    supportedTypes_[supportedTypesCounter_] = type;
    supportedTypesCounter_++;
    return 0;
  }
  return ERROR_ASYN_SUPPORTED_TYPES_ARRAY_FULL;
}

bool ecmcAsynDataItem::asynTypeSupported(asynParamType type) {
  std::qsort(supportedTypes_,
             supportedTypesCounter_,
             sizeof(supportedTypes_[0]),
             compar);
  asynParamType *found = (asynParamType *)std::bsearch(&type,
                                                       supportedTypes_,
                                                       supportedTypesCounter_,
                                                       sizeof(supportedTypes_[0
                                                              ]),
                                                       compar);

  if (found)return true;
  else return false;
}

int ecmcAsynDataItem::getSupportedAsynTypeCount() {
  return supportedTypesCounter_;
}

asynParamType ecmcAsynDataItem::getAsynType() {
  return paramInfo_.asynType;
}

char * ecmcAsynDataItem::getAsynTypeName() {
  return paramInfo_.asynTypeStr;
}

asynParamType ecmcAsynDataItem::getSupportedAsynType(int index) {
  if (index < supportedTypesCounter_) {
    return supportedTypes_[index];
  }
  return asynParamNotDefined;
}

bool ecmcAsynDataItem::willRefreshNext() {
  return asynUpdateCycleCounter_ >= paramInfo_.sampleTimeCycles - 1;
}

/** Set parameter alarm state.
 *
 * \param[in] paramInfo Parameter information.
 * \param[in] alarm Alarm type (EPICS def).
 * \param[in] severity Alarm severity (EPICS def).
 *
 * \return asynSuccess or asynError.
 */
asynStatus ecmcAsynDataItem::setAlarmParam(int alarm, int severity) {
  asynStatus stat;
  int oldAlarmStatus = 0;

  stat = asynPortDriver_->getParamAlarmStatus(ECMC_ASYN_DEFAULT_LIST,
                                              getAsynParameterIndex(),
                                              &oldAlarmStatus);

  if (stat != asynSuccess) {
    return asynError;
  }

  bool doCallbacks = false;

  if (oldAlarmStatus != alarm) {
    stat = asynPortDriver_->setParamAlarmStatus(ECMC_ASYN_DEFAULT_LIST,
                                                getAsynParameterIndex(),
                                                alarm);

    if (stat != asynSuccess) {
      return asynError;
    }
    paramInfo_.alarmStatus = alarm;
    doCallbacks            = true;
  }

  int oldAlarmSeverity = 0;
  stat = asynPortDriver_->getParamAlarmSeverity(ECMC_ASYN_DEFAULT_LIST,
                                                getAsynParameterIndex(),
                                                &oldAlarmSeverity);

  if (stat != asynSuccess) {
    return asynError;
  }

  if (oldAlarmSeverity != severity) {
    stat = asynPortDriver_->setParamAlarmSeverity(ECMC_ASYN_DEFAULT_LIST,
                                                  getAsynParameterIndex(),
                                                  severity);

    if (stat != asynSuccess) {
      return asynError;
    }
    paramInfo_.alarmSeverity = severity;
    doCallbacks              = true;
  }

  if (!doCallbacks || !asynPortDriver_->getAllowRtThreadCom()) {
    return asynSuccess;
  }

  // Alarm status or severity changed=>Do callbacks with old buffered data (if nElemnts==0 then no data in record...)
  if (paramInfo_.dataIsArray && (dataItem_.dataSize > 0)) {
    refreshParamRT(1);
  } else {
    stat = asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST,
                                               ECMC_ASYN_DEFAULT_ADDR);
  }

  return stat;
}

int ecmcAsynDataItem::getAlarmStatus() {
  return paramInfo_.alarmStatus;
}

int ecmcAsynDataItem::getAlarmSeverity() {
  return paramInfo_.alarmSeverity;
}

int ecmcAsynDataItem::asynTypeIsArray(asynParamType asynParType) {
  switch (asynParType) {
  case asynParamInt8Array:
    return 1;

    break;

  case asynParamInt16Array:
    return 1;

    break;

  case asynParamInt32Array:
    return 1;

    break;

  case asynParamFloat32Array:
    return 1;

    break;

  case asynParamFloat64Array:
    return 1;

    break;

#ifdef ECMC_ASYN_ASYNPARAMINT64
  case asynParamInt64Array:
    return 1;

#endif // ECMC_ASYN_ASYNPARAMINT64

  default:
    return 0;

    break;
  }
  return 0;
}

asynStatus ecmcAsynDataItem::writeGeneric(uint8_t      *data,
                                          size_t        bytesToWrite,
                                          asynParamType type,
                                          size_t       *writtenBytes) {
  if (!asynTypeSupported(type)) {
    LOGERR(
      "%s/%s:%d: ERROR: %s asyn type not supported (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      getName(),
      ERROR_ASYN_DATA_TYPE_NOT_SUPPORTED);
    return asynError;
  }

  // Execute function instead of copy data
  if (useExeCmdFunc_) {
    return fctPtrExeCmd_((void *)data, bytesToWrite, type, exeCmdUserObj_);
  }

  size_t bytes = bytesToWrite;

  if (asynTypeIsArray(type)) {
    if (bytes > ecmcMaxSize_) {
      bytes = ecmcMaxSize_;
    }
  } else {
    if (bytes > ecmcMaxSize_) {
      LOGERR(
        "%s/%s:%d: ERROR: %s write error. Data buffer to small (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        getName(),
        ERROR_ASYN_DATA_BUFFER_TO_SMALL);
      return asynError;
    }
  }

  // Write function in  ecmcDataItem
  write(data, bytes);
  *writtenBytes = bytes;

  // refresh params
  int errorCode = refreshParamRT(1);

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: %s write error. Refresh of params failed (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      getName(),
      errorCode);
    return asynError;
  }

  // Trigger callbacks if not array
  if (!asynTypeIsArray(type)) {
    return asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST,
                                               ECMC_ASYN_DEFAULT_ADDR);
  }

  return asynSuccess;
}

asynStatus ecmcAsynDataItem::readGeneric(uint8_t      *data,
                                         size_t        bytesToRead,
                                         asynParamType type,
                                         size_t       *readBytes) {
  if (!asynTypeSupported(type)) {
    LOGERR(
      "%s/%s:%d: ERROR: %s asyn type not supported (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      getName(),
      ERROR_ASYN_DATA_TYPE_NOT_SUPPORTED);
    return asynError;
  }

  size_t bytes = bytesToRead;
  *readBytes = 0;

  if (asynTypeIsArray(type)) {
    if ((bytes > ecmcMaxSize_) && arrayCheckSize_) {
      bytes = ecmcMaxSize_;
    }
  } else {
    if (bytes > ecmcMaxSize_) {
      LOGERR(
        "%s/%s:%d: ERROR: %s read error. Data buffer to small (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        getName(),
        ERROR_ASYN_DATA_BUFFER_TO_SMALL);
      return asynError;
    }
  }

  // Read function in  ecmcDataItem
  read(data, bytes);
  *readBytes = bytes;

  return asynSuccess;
}

asynStatus ecmcAsynDataItem::readInt32(epicsInt32 *value) {
  // Check if cmd. ECMC double, epics record int32
  if (paramInfo_.cmdFloat64ToInt32) {
    if ((paramInfo_.asynType == asynParamFloat64) &&
        (dataItem_.dataSize == sizeof(epicsFloat64))) {
      *value = static_cast<epicsInt32>(*(epicsFloat64 *)dataItem_.data);
      return asynSuccess;
    } else {
      LOGERR(
        "%s/%s:%d: ERROR: %s read error. cmdFloat64ToInt32 fail. Size or type error (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        getName(),
        ERROR_ASYN_CMD_FAIL);
      return asynError;
    }
  }

  size_t bytesRead = 0;
  return readGeneric((uint8_t *)value, sizeof(epicsInt32),
                     asynParamInt32, &bytesRead);
}

asynStatus ecmcAsynDataItem::writeInt32(epicsInt32 value) {
  // Check if cmd. ECMC double, epics record int32
  if (paramInfo_.cmdFloat64ToInt32) {
    if ((paramInfo_.asynType == asynParamFloat64) &&
        (dataItem_.dataSize == sizeof(epicsFloat64))) {
      epicsFloat64 temp = static_cast<epicsFloat64>(value);
      memcpy(dataItem_.data, &temp, sizeof(epicsFloat64));

      return refreshParamRT(1) ? asynError : asynSuccess;
    } else {
      LOGERR(
        "%s/%s:%d: ERROR: %s read error. cmdFloat64ToInt32 fail. Size or type error (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        getName(),
        ERROR_ASYN_CMD_FAIL);
      return asynError;
    }
  }

  size_t bytesWritten = 0;

  if (checkIntRange_) {
    if ((value > intMax_) || (value < intMin_)) {
      LOGERR(
        "%s/%s:%d: Error: %s value Out Of Range %d (allowed Range %" PRId64 "..%" PRId64 ") (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        getName(),
        value,
        intMin_,
        intMax_,
        ERROR_ASYN_WRITE_VALUE_OUT_OF_RANGE);
      return asynError;
    }
  }

  return writeGeneric((uint8_t *)&value, sizeof(epicsInt32),
                      asynParamInt32, &bytesWritten);
}

asynStatus ecmcAsynDataItem::readUInt32Digital(epicsUInt32 *value,
                                               epicsUInt32  mask) {
  epicsUInt32 tempValue = 0;
  size_t bytesRead      = 0;
  asynStatus stat       = readGeneric((uint8_t *)&tempValue,
                                      sizeof(epicsUInt32),
                                      asynParamUInt32Digital,
                                      &bytesRead);

  if (stat != asynSuccess) {
    return stat;
  }

  *value = tempValue & mask;
  return asynSuccess;
}

asynStatus ecmcAsynDataItem::writeUInt32Digital(epicsUInt32 value,
                                                epicsUInt32 mask) {
  if (checkIntRange_ && ((value > intMax_) || (value < intMin_))) {
    LOGERR(
      "%s/%s:%d: Error: %s value Out Of Range %d (allowed Range %" PRId64 "..%" PRId64 ") (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      getName(),
      value,
      intMin_,
      intMax_,
      ERROR_ASYN_WRITE_VALUE_OUT_OF_RANGE);
    return asynError;
  }

  epicsUInt32 tempVal1 = 0;
  asynStatus  stat     = readUInt32Digital(&tempVal1, ~mask);

  if (stat != asynSuccess) {
    return stat;
  }
  epicsUInt32 tempVal2 = tempVal1 | (value & mask);
  size_t bytesWritten  = 0;
  return writeGeneric((uint8_t *)&tempVal2, sizeof(epicsUInt32),
                      asynParamUInt32Digital, &bytesWritten);
}

asynStatus ecmcAsynDataItem::readFloat64(epicsFloat64 *value) {
  // Check if cmd. ECMC int64, epics record double
  if (paramInfo_.cmdInt64ToFloat64) {
    if ((paramInfo_.asynType == asynParamFloat64) &&
        (dataItem_.dataSize == sizeof(int64_t))) {
      *value = static_cast<epicsFloat64>(*(int64_t *)dataItem_.data);
      return asynSuccess;
    } else {
      LOGERR(
        "%s/%s:%d: ERROR: %s read error. cmdInt64ToFloat64 fail. Size or type error (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        getName(),
        ERROR_ASYN_CMD_FAIL);
      return asynError;
    }
  }

  // Check if cmd. ECMC uint64, epics record double
  if (paramInfo_.cmdUint64ToFloat64) {
    if ((paramInfo_.asynType == asynParamFloat64) &&
        (dataItem_.dataSize == sizeof(uint64_t))) {
      *value = static_cast<epicsFloat64>(*(uint64_t *)dataItem_.data);
      return asynSuccess;
    } else {
      LOGERR(
        "%s/%s:%d: ERROR: %s read error. cmdUint64ToFloat64 fail. Size or type error (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        getName(),
        ERROR_ASYN_CMD_FAIL);
      return asynError;
    }
  }

  if (paramInfo_.cmdUint32ToFloat64) {
    if ((paramInfo_.asynType == asynParamFloat64) &&
        (dataItem_.dataSize == sizeof(uint32_t))) {
      *value = static_cast<epicsFloat64>(*(uint32_t *)dataItem_.data);
      return asynSuccess;
    } else {
      LOGERR(
        "%s/%s:%d: ERROR: %s read error. cmdUint64ToFloat64 fail. Size or type error (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        getName(),
        ERROR_ASYN_CMD_FAIL);
      return asynError;
    }
  }
  // Special case F32
  if (dataItem_.dataType == ECMC_EC_F32) {
    if ((paramInfo_.asynType == asynParamFloat64) &&
        (dataItem_.dataSize >= sizeof(float))) {
      *value = static_cast<epicsFloat64>(*(float *)dataItem_.data);
      return asynSuccess;
    } else {
      LOGERR(
        "%s/%s:%d: ERROR: %s read error. F32 fail. Size or type error (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        getName(),
        ERROR_ASYN_CMD_FAIL);
      return asynError;
    }
  }

  size_t bytesRead = 0;
  return readGeneric((uint8_t *)value, sizeof(epicsFloat64),
                     asynParamFloat64, &bytesRead);
}

asynStatus ecmcAsynDataItem::writeFloat64(epicsFloat64 value) {
  // Check if cmd. ECMC int64, epics record double
  if (paramInfo_.cmdInt64ToFloat64) {
    if ((paramInfo_.asynType == asynParamFloat64) &&
        (dataItem_.dataSize == sizeof(int64_t))) {
      int64_t temp = static_cast<int64_t>(value);
      memcpy(dataItem_.data, &temp, sizeof(int64_t));
      return asynSuccess;
    } else {
      LOGERR(
        "%s/%s:%d: ERROR: %s write error. cmdInt64ToFloat64 fail. Size or type error (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        getName(),
        ERROR_ASYN_CMD_FAIL);
      return asynError;
    }
  }

  // Check if cmd. ECMC uint64, epics record double
  if (paramInfo_.cmdUint64ToFloat64) {
    if ((paramInfo_.asynType == asynParamFloat64) &&
        (dataItem_.dataSize == sizeof(uint64_t))) {
      uint64_t temp = static_cast<uint64_t>(value);
      memcpy(dataItem_.data, &temp, sizeof(uint64_t));
      return asynSuccess;
    } else {
      LOGERR(
        "%s/%s:%d: ERROR: %s write error. cmdUint64ToFloat64 fail. Size or type error (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        getName(),
        ERROR_ASYN_CMD_FAIL);
      return asynError;
    }
  }

  // Special case F32
  if (dataItem_.dataType == ECMC_EC_F32) {
    if ((paramInfo_.asynType == asynParamFloat64) &&
        (dataItem_.dataSize >= sizeof(float))) {
      float temp = static_cast<float>(value);
      memcpy(dataItem_.data, &temp, sizeof(float));
      return asynSuccess;
    } else {
      LOGERR(
        "%s/%s:%d: ERROR: %s write error. F32 write fail. Size or type error (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        getName(),
        ERROR_ASYN_CMD_FAIL);
      return asynError;
    }
  }

  size_t bytesWritten = 0;
  return writeGeneric((uint8_t *)&value, sizeof(epicsFloat64),
                      asynParamFloat64, &bytesWritten);
}

asynStatus ecmcAsynDataItem::readInt8Array(epicsInt8 *value,
                                           size_t     nElements,
                                           size_t    *nIn) {
  return readGeneric((uint8_t *)value, nElements * sizeof(epicsInt8),
                     asynParamInt8Array, nIn);
}

asynStatus ecmcAsynDataItem::writeInt8Array(epicsInt8 *value,
                                            size_t     nElements) {
  size_t bytesWritten = 0;

  return writeGeneric((uint8_t *)value, nElements * sizeof(epicsInt8),
                      asynParamInt8Array, &bytesWritten);
}

asynStatus ecmcAsynDataItem::readInt16Array(epicsInt16 *value,
                                            size_t      nElements,
                                            size_t     *nIn) {
  return readGeneric((uint8_t *)value, nElements * sizeof(epicsInt16),
                     asynParamInt16Array, nIn);
}

asynStatus ecmcAsynDataItem::writeInt16Array(epicsInt16 *value,
                                             size_t      nElements) {
  size_t bytesWritten = 0;

  return writeGeneric((uint8_t *)value, nElements * sizeof(epicsInt16),
                      asynParamInt16Array, &bytesWritten);
}

asynStatus ecmcAsynDataItem::readInt32Array(epicsInt32 *value,
                                            size_t      nElements,
                                            size_t     *nIn) {
  return readGeneric((uint8_t *)value, nElements * sizeof(epicsInt32),
                     asynParamInt32Array, nIn);
}

asynStatus ecmcAsynDataItem::writeInt32Array(epicsInt32 *value,
                                             size_t      nElements) {
  size_t bytesWritten = 0;

  return writeGeneric((uint8_t *)value, nElements * sizeof(epicsInt32),
                      asynParamInt32Array, &bytesWritten);
}

asynStatus ecmcAsynDataItem::readFloat32Array(epicsFloat32 *value,
                                              size_t        nElements,
                                              size_t       *nIn) {
  return readGeneric((uint8_t *)value, nElements * sizeof(epicsFloat32),
                     asynParamFloat32Array, nIn);
}

asynStatus ecmcAsynDataItem::writeFloat32Array(epicsFloat32 *value,
                                               size_t        nElements) {
  size_t bytesWritten = 0;

  return writeGeneric((uint8_t *)value, nElements * sizeof(epicsFloat32),
                      asynParamFloat32Array, &bytesWritten);
}

asynStatus ecmcAsynDataItem::readFloat64Array(epicsFloat64 *value,
                                              size_t        nElements,
                                              size_t       *nIn) {
  return readGeneric((uint8_t *)value, nElements * sizeof(epicsFloat64),
                     asynParamFloat64Array, nIn);
}

asynStatus ecmcAsynDataItem::writeFloat64Array(epicsFloat64 *value,
                                               size_t        nElements) {
  size_t bytesWritten = 0;

  return writeGeneric((uint8_t *)value, nElements * sizeof(epicsFloat64),
                      asynParamFloat64Array, &bytesWritten);
}

#ifdef ECMC_ASYN_ASYNPARAMINT64

asynStatus ecmcAsynDataItem::readInt64(epicsInt64 *value) {
  size_t bytesRead = 0;

  return readGeneric((uint8_t *)value, sizeof(epicsInt64),
                     asynParamInt64, &bytesRead);
}

asynStatus ecmcAsynDataItem::writeInt64(epicsInt64 value) {
  size_t bytesWritten = 0;

  return writeGeneric((uint8_t *)&value, sizeof(epicsInt64),
                      asynParamInt64, &bytesWritten);
}

asynStatus ecmcAsynDataItem::writeInt64Array(epicsInt64 *value,
                                             size_t      nElements) {
  size_t bytesWritten = 0;

  return writeGeneric((uint8_t *)value, nElements * sizeof(epicsInt64),
                      asynParamInt64Array, &bytesWritten);
}

asynStatus ecmcAsynDataItem::readInt64Array(epicsInt64 *value,
                                            size_t      nElements,
                                            size_t     *nIn) {
  return readGeneric((uint8_t *)value, nElements * sizeof(epicsInt64),
                     asynParamInt64Array, nIn);
}

#endif //ECMC_ASYN_ASYNPARAMINT64

/** Validates drvInfo string
 * \param[in] drvInfo String containing information about the parameter.
 * \return asynSuccess or asynError.
 * The drvInfo string is what is after the asyn() in the "INP" or "OUT"
 * field of an record.
 */
asynStatus ecmcAsynDataItem::validateDrvInfo(const char *drvInfo) {
  const char *functionName = "validateDrvInfo";

  asynPrint(asynPortDriver_->getTraceAsynUser(),
            ASYN_TRACE_FLOW,
            "%s:%s: drvInfo: %s\n",
            driverName,
            functionName,
            drvInfo);

  if (strlen(drvInfo) == 0) {
    asynPrint(asynPortDriver_->getTraceAsynUser(),
              ASYN_TRACE_ERROR,
              "Invalid drvInfo string: Length 0 (%s).\n",
              drvInfo);
    return asynError;
  }

  // Check '?' mark last or '=' last
  const char *read = strrchr(drvInfo, '?');

  if (read) {
    if (strlen(read) == 1) {
      return asynSuccess;
    }
  }

  const char *write = strrchr(drvInfo, '=');

  if (write) {
    if (strlen(write) == 1) {
      return asynSuccess;
    }
  }

  asynPrint(asynPortDriver_->getTraceAsynUser(),
            ASYN_TRACE_ERROR,
            "Invalid drvInfo string (%s).\n",
            drvInfo);

  return asynError;
}

/** Get asyn type from record.
 * \param[in] drvInfo String containing information about the parameter.
 * \param[in/out] paramInfo Parameter information structure.
 * \return asynSuccess or asynError.
 */
asynStatus ecmcAsynDataItem::getRecordInfoFromDrvInfo(const char *drvInfo) {
  const char *functionName = "getRecordInfoFromDrvInfo";

  asynPrint(asynPortDriver_->getTraceAsynUser(),
            ASYN_TRACE_FLOW,
            "%s:%s: drvInfo: %s\n",
            driverName,
            functionName,
            drvInfo);

  bool isInput  = false;
  bool isOutput = false;
  DBENTRY *pdbentry;
  pdbentry = dbAllocEntry(pdbbase);
  long status      = dbFirstRecordType(pdbentry);
  bool recordFound = false;

  if (status) {
    dbFreeEntry(pdbentry);
    return asynError;
  }

  while (!status) {
    paramInfo_.recordType = strdup(dbGetRecordTypeName(pdbentry));
    status                = dbFirstRecord(pdbentry);

    while (!status) {
      paramInfo_.recordName = strdup(dbGetRecordName(pdbentry));

      if (!dbIsAlias(pdbentry)) {
        status = dbFindField(pdbentry, "INP");

        if (!status) {
          paramInfo_.inp = strdup(dbGetString(pdbentry));

          isInput = true;
          char port[ECMC_MAX_FIELD_CHAR_LENGTH];
          int  adr;
          int  timeout;
          char currdrvInfo[ECMC_MAX_FIELD_CHAR_LENGTH];
          int  nvals = sscanf(paramInfo_.inp,
                              ECMC_ASYN_INP_FORMAT,
                              port,
                              &adr,
                              &timeout,
                              currdrvInfo);

          if (nvals == 4) {
            // Ensure correct port and drvinfo
            if ((strcmp(port,
                        asynPortDriver_->portName) == 0) &&
                (strcmp(drvInfo, currdrvInfo) == 0)) {
              recordFound = true;  // Correct port and drvinfo!\n");
            }
          } else {
            int mask = 0;
            nvals = sscanf(paramInfo_.inp,
                           ECMC_ASYN_MASK_INP_FORMAT,
                           port,
                           &adr,
                           &mask,
                           &timeout,
                           currdrvInfo);

            if (nvals == 5) {
              // Ensure correct port and drvinfo
              if ((strcmp(port,
                          asynPortDriver_->portName) == 0) &&
                  (strcmp(drvInfo, currdrvInfo) == 0)) {
                recordFound = true;  // Correct port and drvinfo!\n");
              }
            }
          }
        } else {
          isInput = false;
        }
        status = dbFindField(pdbentry, "OUT");

        if (!status) {
          paramInfo_.out = strdup(dbGetString(pdbentry));
          isOutput       = true;
          char port[ECMC_MAX_FIELD_CHAR_LENGTH];
          int  adr;
          int  timeout;
          char currdrvInfo[ECMC_MAX_FIELD_CHAR_LENGTH];
          int  nvals = sscanf(paramInfo_.out,
                              ECMC_ASYN_INP_FORMAT,
                              port,
                              &adr,
                              &timeout,
                              currdrvInfo);

          if (nvals == 4) {
            // Ensure correct port and drvinfo
            if ((strcmp(port,
                        asynPortDriver_->portName) == 0) &&
                (strcmp(drvInfo, currdrvInfo) == 0)) {
              recordFound = true;  // Correct port and drvinfo!\n");
            }
          } else {
            int mask = 0;
            nvals = sscanf(paramInfo_.out,
                           ECMC_ASYN_MASK_INP_FORMAT,
                           port,
                           &adr,
                           &mask,
                           &timeout,
                           currdrvInfo);

            if (nvals == 5) {
              // Ensure correct port and drvinfo
              if ((strcmp(port,
                          asynPortDriver_->portName) == 0) &&
                  (strcmp(drvInfo, currdrvInfo) == 0)) {
                recordFound = true;  // Correct port and drvinfo!\n");
              }
            }
          }
        } else {
          isOutput = false;
        }

        if (recordFound) {
          // Correct record found. Collect data from fields
          // DTYP
          status = dbFindField(pdbentry, "DTYP");

          if (!status) {
            paramInfo_.dtyp = strdup(dbGetString(pdbentry));

            // paramInfo_.asynType=stringToAsynType(dbGetString(pdbentry));
          } else {
            paramInfo_.dtyp = 0;

            // paramInfo_.asynType=asynParamNotDefined;
          }

          // drvInput (not a field)
          paramInfo_.drvInfo = strdup(drvInfo);
          dbFreeEntry(pdbentry);

          // The correct record was found and the paramInfo structure is filled
          return asynSuccess;
        } else {
          // Not correct record. Do cleanup.
          if (isInput) {
            free(paramInfo_.inp);
            paramInfo_.inp = 0;
          }

          if (isOutput) {
            free(paramInfo_.out);
            paramInfo_.out = 0;
          }
          paramInfo_.drvInfo = 0;
          paramInfo_.scan    = 0;
          paramInfo_.dtyp    = 0;
          isInput            = false;
          isOutput           = false;
        }
      }
      status = dbNextRecord(pdbentry);
      free(paramInfo_.recordName);
      paramInfo_.recordName = 0;
      recordFound           = false;
    }
    status = dbNextRecordType(pdbentry);
    free(paramInfo_.recordType);
    paramInfo_.recordType = 0;
  }
  dbFreeEntry(pdbentry);

  // Record not found so other asyn port driver/client
  paramInfo_.recordType = strdup("none");
  paramInfo_.recordName = strdup("asynPortDriver");
  paramInfo_.inp        = strdup("none");
  paramInfo_.dtyp       = strdup("none");
  paramInfo_.out        = strdup("none");
  paramInfo_.drvInfo    = strdup(drvInfo);
  paramInfo_.scan       = 0;

  return asynError;
}

/** Get variable information from drvInfo string.
 * \param[in] drvInfo String containing information about the parameter.
 * \param[in/out] paramInfo Parameter information structure.
 * \return asynSuccess or asynError.
 * Methods checks if input or output ('?' or '=') and parses options:
 * - "ADSPORT" (Ams port for varaible)\n
 * - "TS_MS" (sample time ms)\n
 * - "TIMEBASE" ("PLC" or "EPICS")\n
 * Also supports the following commands:
 * - ".AMSPORTSTATE." (Read/write AMS-port state)\n
 * - ".ADR.*" (absolute access)\n
 */
asynStatus ecmcAsynDataItem::parseInfofromDrvInfo(const char *drvInfo) {
  const char *functionName = "parseInfofromDrvInfo";

  asynPrint(asynPortDriver_->getTraceAsynUser(), ASYN_TRACE_FLOW,
            "%s:%s: drvInfo: %s\n",
            driverName,
            functionName,
            drvInfo);

  // Check if input or output
  paramInfo_.isIOIntr = false;
  const char *temp = strrchr(drvInfo, '?');

  if (temp) {
    if (strlen(temp) == 1) {
      paramInfo_.isIOIntr = true; // All inputs will be created I/O intr
    }
  }

  asynPrint(
    asynPortDriver_->getTraceAsynUser(),
    ASYN_TRACE_FLOW,
    "%s:%s: drvInfo %s is %s\n",
    driverName,
    functionName,
    drvInfo,
    paramInfo_.isIOIntr ? "I/O Intr (end with ?)" : "not I/O Intr (end with =)");

  // take part after last "/" if option or complete string..
  char buffer[ECMC_MAX_FIELD_CHAR_LENGTH];

  // See if option (find last '/')
  const char *drvInfoEnd = strrchr(drvInfo, '/');

  if (drvInfoEnd) { // found '/'
    int nvals = sscanf(drvInfoEnd, "/%s", buffer);

    if (nvals == 1) {
      paramInfo_.name                              = strdup(buffer);
      paramInfo_.name[strlen(paramInfo_.name) - 1] = 0; // Strip ? or = from end
    } else {
      asynPrint(
        asynPortDriver_->getTraceAsynUser(),
        ASYN_TRACE_ERROR,
        "%s:%s: Failed to parse PLC address string from drvInfo (%s)\n",
        driverName,
        functionName,
        drvInfo);
      return asynError;
    }
  } else {   // No options
    paramInfo_.name                              = strdup(drvInfo);
    paramInfo_.name[strlen(paramInfo_.name) - 1] = 0; // Strip ? or = from end
  }

  // Check if ECMC_OPTION_T_SAMPLE_RATE_MS option
  const char *option = ECMC_OPTION_T_SAMPLE_RATE_MS;
  paramInfo_.sampleTimeMS = asynPortDriver_->getDefaultSampleTimeMs();
  const char *isThere = strstr(drvInfo, option);

  if (isThere) {
    if (strlen(isThere) < (strlen(option) + strlen("=0/"))) {
      asynPrint(
        asynPortDriver_->getTraceAsynUser(),
        ASYN_TRACE_ERROR,
        "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n",
        driverName,
        functionName,
        option,
        drvInfo);
      return asynError;
    }

    int nvals = sscanf(isThere + strlen(option),
                       "=%lf/",
                       &paramInfo_.sampleTimeMS);

    if (nvals != 1) {
      paramInfo_.sampleTimeMS = asynPortDriver_->getDefaultSampleTimeMs();
      asynPrint(
        asynPortDriver_->getTraceAsynUser(),
        ASYN_TRACE_ERROR,
        "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
        driverName,
        functionName,
        option,
        drvInfo);
      return asynError;
    }
  }

  // ensure that sample rate is not faster than realtime loop or plc scan rates
  double dataUpdateRateMs = dataItem_.dataUpdateRateMs;

  if (dataUpdateRateMs < 0) { // updated at realtime loop rate
    dataUpdateRateMs = mcuPeriod / 1E6;
  }

  if ((paramInfo_.sampleTimeMS < dataUpdateRateMs) &&
      (paramInfo_.sampleTimeMS > 0)) {
    asynPrint(
      asynPortDriver_->getTraceAsynUser(),
      ASYN_TRACE_ERROR,
      "%s:%s: WARNING: Record sample rate faster than parameter update rate (%3.1lfms<%3.1lfms),"
      " %3.1lfms will be used. (drvInfo = %s).\n",
      driverName,
      functionName,
      paramInfo_.sampleTimeMS,
      dataUpdateRateMs,
      dataUpdateRateMs,
      drvInfo);
    paramInfo_.sampleTimeMS = dataUpdateRateMs;
  }

  paramInfo_.sampleTimeCycles =
    (int32_t)(paramInfo_.sampleTimeMS / dataUpdateRateMs);

  if (paramInfo_.sampleTimeMS == -1) {
    paramInfo_.sampleTimeCycles = -1;
  }

  // Check if TYPE option
  option                 = ECMC_OPTION_TYPE;
  paramInfo_.asynTypeStr = NULL;
  paramInfo_.asynType    = asynParamNotDefined;

  isThere = strstr(drvInfo, option);

  if (isThere) {
    if (strlen(isThere) < (strlen(option) + strlen("=0/"))) {
      asynPrint(
        asynPortDriver_->getTraceAsynUser(),
        ASYN_TRACE_ERROR,
        "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n",
        driverName,
        functionName,
        option,
        drvInfo);
      return asynError;
    }
    int nvals;
    nvals = sscanf(isThere + strlen(option), "=%[^/]", buffer);

    if (nvals != 1) {
      asynPrint(
        asynPortDriver_->getTraceAsynUser(),
        ASYN_TRACE_ERROR,
        "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
        driverName,
        functionName,
        option,
        drvInfo);
      return asynError;
    }
    paramInfo_.asynTypeStr = strdup(buffer);
    paramInfo_.asynType    = stringToAsynType(paramInfo_.asynTypeStr);
  }

  // Check if CMD option
  option                        = ECMC_OPTION_CMD;
  paramInfo_.cmdUint64ToFloat64 = false;
  paramInfo_.cmdInt64ToFloat64  = false;
  paramInfo_.cmdFloat64ToInt32  = false;
  paramInfo_.cmdUint32ToFloat64 = false;

  isThere = strstr(drvInfo, option);

  if (isThere) {
    if (strlen(isThere) < (strlen(option) + strlen("=0/"))) {
      asynPrint(
        asynPortDriver_->getTraceAsynUser(),
        ASYN_TRACE_ERROR,
        "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n",
        driverName,
        functionName,
        option,
        drvInfo);
      return asynError;
    }
    int nvals;
    nvals = sscanf(isThere + strlen(option), "=%[^/]", buffer);

    if (nvals != 1) {
      asynPrint(
        asynPortDriver_->getTraceAsynUser(),
        ASYN_TRACE_ERROR,
        "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
        driverName,
        functionName,
        option,
        drvInfo);
      return asynError;
    }
    bool cmdOK = false;

    // Check UINT642FLOAT64
    isThere = strstr(buffer, ECMC_OPTION_CMD_UINT64_TO_FLOAT64);

    if (isThere) {
      paramInfo_.cmdUint64ToFloat64 = true;
      cmdOK                         = true;
    }

    if (!cmdOK) {
      // Check UINT322FLOAT64
      isThere = strstr(buffer, ECMC_OPTION_CMD_UINT32_TO_FLOAT64);

      if (isThere) {
        paramInfo_.cmdUint32ToFloat64 = true;
        cmdOK                         = true;
      }
    }

    if (!cmdOK) {
      // Check INT2FLOAT64
      isThere = strstr(buffer, ECMC_OPTION_CMD_INT_TO_FLOAT64);

      if (isThere) {
        paramInfo_.cmdInt64ToFloat64 = true;
        cmdOK                        = true;
      }
    }

    if (!cmdOK) {
      // Check FLOAT64TOINT
      isThere = strstr(buffer, ECMC_OPTION_CMD_FLOAT64_INT);

      if (isThere) {
        paramInfo_.cmdFloat64ToInt32 = true;
        cmdOK                        = true;
      }
    }


    if (!cmdOK) {
      asynPrint(
        asynPortDriver_->getTraceAsynUser(),
        ASYN_TRACE_ERROR,
        "%s:%s: Failed to parse %s option from drvInfo (%s). Command %s not valid.\n",
        driverName,
        functionName,
        option,
        drvInfo,
        buffer);
      return asynError;
    }
  }

  return asynSuccess;
}

asynStatus ecmcAsynDataItem::setDrvInfo(const char *drvInfo) {
  const char *functionName = "setDrvInfo";

  if (validateDrvInfo(drvInfo) != asynSuccess) {
    return asynError;
  }

  // Parse options and name
  asynStatus status = parseInfofromDrvInfo(drvInfo);

  if (status != asynSuccess) {
    return asynError;
  }

  status = getRecordInfoFromDrvInfo(drvInfo);

  if (status != asynSuccess) {
    asynPrint(
      asynPortDriver_->getTraceAsynUser(),
      ASYN_TRACE_INFO,
      "%s:%s: INFO: Failed to find record with drvInfo %s. Must be a connetion from other asyn client (motor record).\n",
      driverName,
      functionName,
      drvInfo);

    // allow not linked to record (motorrecord could be motor record or other asyn client?!)
  }

  return asynSuccess;
}

/**
 * Function to be executed when asyn write occurs.
 * This function needs to handle everything that should happen.
 * if data should be copied then
*/
asynStatus ecmcAsynDataItem::setExeCmdFunctPtr(ecmcExeCmdFcn func,
                                               void         *userObj) {
  fctPtrExeCmd_  = func;
  exeCmdUserObj_ = userObj;
  useExeCmdFunc_ = true;
  return asynSuccess;
}
