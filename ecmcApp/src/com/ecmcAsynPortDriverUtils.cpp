/*
* ecmcAsynPortDriverUtils.cpp
*
* Utilities and definitions used by ecmcAsynPortDriver-class.
*
* Author: Anders Sandström
*
* Created January 25, 2019
*/

#include "ecmcAsynPortDriverUtils.h"
#include <string.h>
#include <stdlib.h>
#include <initHooks.h>
#include "epicsTime.h"


/** Convert asyn type enum to string.
 *
 * \param[in] type Asyn type (from asynDriver https://github.com/epics-modules/asyn)
 *
 * \return Asyn type string.
 */
const char *asynTypeToString(long type)
{
  switch (type) {
    case asynParamInt32:
      return "asynParamInt32";
    case asynParamFloat64:
      return "asynParamFloat64";
    case asynParamUInt32Digital:
      return "asynParamUInt32Digital";
    case asynParamOctet:
      return "asynParamOctet";
    case asynParamInt8Array:
      return "asynParamInt8Array";
    case asynParamInt16Array:
      return "asynParamInt16Array";
    case asynParamInt32Array:
      return "asynParamInt32Array";
    case asynParamFloat32Array:
      return "asynParamFloat32Array";
    case asynParamFloat64Array:
      return "asynParamFloat64Array";
    case asynParamGenericPointer:
      return "asynParamGenericPointer";
    default:
      return "asynUnknownType";
  }
}

/** Convert EPICS dtyp field to asyn type.
 *
 * \param[in] typeStr field
 *
 * \return asyn param type enum.
 */

asynParamType stringToAsynType(char *typeStr)
{
  if(strcmp("asynFloat64",typeStr)==0){
    return asynParamFloat64;
  }
  if(strcmp("asynInt32",typeStr)==0){
    return asynParamInt32;
  }
  if(strcmp("asynInt8ArrayIn",typeStr)==0 || strcmp("asynInt8ArrayOut",typeStr)==0){
    return asynParamInt8Array;
  }
  if(strcmp("asynInt16ArrayIn",typeStr)==0 || strcmp("asynInt16ArrayOut",typeStr)==0){
    return asynParamInt16Array;
  }
  if(strcmp("asynInt32ArrayIn",typeStr)==0 || strcmp("asynInt32ArrayOut",typeStr)==0){
    return asynParamInt32Array;
  }
  if(strcmp("asynFloat32ArrayIn",typeStr)==0 || strcmp("asynFloat32ArrayOut",typeStr)==0){
    return asynParamFloat32Array;
  }
  if(strcmp("asynFloat64ArrayIn",typeStr)==0 || strcmp("asynFloat64ArrayOut",typeStr)==0){
    return asynParamFloat64Array;
  }

  //  asynParamUInt32Digital,
  //  asynParamOctet,
  //  asynParamGenericPointer

  return asynParamNotDefined;
}

/** Convert EPICS state to string.
 *
 * \param[in] state EPICS state enum.
 *
 * \return EPICS state string.
 */
const char* epicsStateToString(int state)
{
  switch(state) {
    case initHookAtIocBuild:
      return "initHookAtIocBuild";
      break;
    case initHookAtBeginning:
      return "initHookAtBeginning";
      break;
    case initHookAfterCallbackInit:
      return "initHookAfterCallbackInit";
      break;
    case initHookAfterCaLinkInit:
      return "initHookAfterCaLinkInit";
      break;
    case initHookAfterInitDrvSup:
      return "initHookAfterInitDrvSup";
      break;
    case initHookAfterInitRecSup:
      return "initHookAfterInitRecSup";
      break;
    case initHookAfterInitDevSup:
      return "initHookAfterInitDevSup";
      break;
    case initHookAfterInitDatabase:
      return "initHookAfterInitDatabase";
      break;
    case initHookAfterFinishDevSup:
      return "initHookAfterFinishDevSup";
      break;
    case initHookAfterScanInit:
      return "initHookAfterScanInit";
      break;
    case initHookAfterInitialProcess:
      return "initHookAfterInitialProcess";
      break;
    case initHookAfterIocBuilt:
      return "initHookAfterIocBuilt";
      break;
    case initHookAtIocRun:
      return "initHookAtIocRun";
      break;
    case initHookAfterDatabaseRunning:
      return "initHookAfterDatabaseRunning";
      break;
    case initHookAfterCaServerRunning:
      return "initHookAfterCaServerRunning";
      break;
    case initHookAfterIocRunning:
      return "initHookAfterIocRunning";
      break;
    case initHookAtIocPause:
      return "initHookAtIocPause";
      break;
    case initHookAfterCaServerPaused:
      return "initHookAfterCaServerPaused";
      break;
    case initHookAfterDatabasePaused:
      return "initHookAfterDatabasePaused";
      break;
    case initHookAfterIocPaused:
      return "initHookAfterIocPaused";
      break;
     case initHookAfterInterruptAccept:
      return "initHookAfterInterruptAccept";
      break;
    default:
      return "Unknown state";
      break;
  }
  return "Unknown state";
}

#define WINDOWS_TICK_PER_SEC 10000000
#define SEC_TO_UNIX_EPOCH 11644473600LL

/** Convert Windows timestamp to EPICS timestamp
 *
 * \param[in] plcTime Timestamp from ams router (Windows format, epoch start jan
 * 1 1601).
 * \param[out] ts Epics timestamp (Epoch start jan 1 1990).
 *
 * \return 0 or error code.
 */
int windowsToEpicsTimeStamp(uint64_t plcTime, epicsTimeStamp *ts)
{
  if(!ts){
    return 1;
  }
  //move from WindowsTime to Epics time 1601 jan 1 to 1990 jan 1 (POSIX_TIME_AT_EPICS_EPOCH defined in epicsTime.h)
  plcTime=plcTime-(POSIX_TIME_AT_EPICS_EPOCH + SEC_TO_UNIX_EPOCH)*WINDOWS_TICK_PER_SEC;

  ts->secPastEpoch=(uint32_t)(plcTime/WINDOWS_TICK_PER_SEC);
  ts->nsec=(uint32_t)((plcTime-(ts->secPastEpoch*WINDOWS_TICK_PER_SEC))*100);

  return 0;
}

ECMC_SOURCE dataSourceFromVariableName(char* varName)
{
  
  if(

  return ECMC_SOURCE_UNDEFINED;
}