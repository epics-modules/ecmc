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
  if(strcmp("asynUInt32Digital",typeStr)==0){
    return asynParamUInt32Digital;
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

/*Available strings:
 *  ec<masterId>.s<slaveId>.<alias>  (defaults complete ecentry)
 *  ec<masterId>.s<slaveId>.<alias>.<bit> (only one bit)
*/
int parseEcPath(char *ecPath,
                       int  *master,
                       int  *slave,
                       char *alias,
                       int  *bit) {
  int masterId = 0;
  int slaveId  = 0;
  int bitId    = 0;
  int nvals    = 0;

  nvals = sscanf(ecPath,
                 "ec%d.s%d.%[^.].%d",
                 &masterId,
                 &slaveId,
                 alias,
                 &bitId);

  if (nvals == 4) {
    *master = masterId;
    *slave  = slaveId;
    *bit    = bitId;
    return 0;
  }
  nvals = sscanf(ecPath, "ec%d.s%d.%s", &masterId, &slaveId, alias);

  if (nvals == 3) {
    *master = masterId;
    *slave  = slaveId;
    *bit    = -1;
    return 0;
  }
  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
}

/*int parseObjectPath(char             *objPath,
                           int              *axis,
                           motionObjectType *objectType,
                           int              *objectFunction) {


  motionObjectType objType;
  int objIndex=0;
  int errorCode=getEcmcObjectType(objPath,&objIndex,&objType);
  if(errorCode){
    return errorCode;
  }

  switch(objType){
    case ECMC_OBJ_INVALID:
      return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
      break;
    case ECMC_OBJ_AXIS:
      *axis=objIndex;

       // get sub object here!!

      break;
    case ECMC_OBJ_EC:
      *axis=objIndex;
      break;
    case ECMC_OBJ_DS:
      *axis=objIndex;
      break;
    case ECMC_OBJ_MAIN:
      *axis=objIndex;
      break;
    case ECMC_OBJ_THREAD:
      *axis=objIndex;
      break;
  }

  int  axisId = 0;
  char objectTypeStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char objectFunctionStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  nvals = 0;

  *objectType = ECMC_OBJ_INVALID;

  // Axis sub objects
  nvals = sscanf(objPath,
                 ECMC_AX_STR "%d.%[^.].%s",
                 &axisId,
                 objectTypeStr,
                 objectFunctionStr);

  if (nvals == 3) {
    *axis = axisId;

    // Drive
    nvals = strcmp(objectTypeStr, ECMC_DRV_STR);

    if (nvals == 0) {
      *objectType = ECMC_OBJ_DRIVE;

      // Enable
      nvals = strcmp(objectFunctionStr, ECMC_DRV_ENABLE_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD;
        return 0;
      }

      // Velocity
      nvals = strcmp(objectFunctionStr, ECMC_DRV_VELOCITY_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT;
        return 0;
      }

      // Enabled
      nvals = strcmp(objectFunctionStr, ECMC_DRV_ENABLED_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD;
        return 0;
      }

      // Break
      nvals = strcmp(objectFunctionStr, ECMC_DRV_BREAK_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT;
        return 0;
      }

      // Reduce Torque
      nvals = strcmp(objectFunctionStr, ECMC_DRV_REDUCETORQUE_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT;
        return 0;
      }
      return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
    }

    // Encoder
    nvals = strcmp(objectTypeStr, ECMC_ENC_STR);

    if (nvals == 0) {
      *objectType = ECMC_OBJ_ENCODER;

      // Actpos
      nvals = strcmp(objectFunctionStr, ECMC_ENC_ACTPOS_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION;
        return 0;
      }

      // Latch status
      nvals = strcmp(objectFunctionStr, ECMC_ENC_LATCH_STATUS_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_ENCODER_ENTRY_INDEX_LATCH_STATUS;
        return 0;
      }

      // Latch pos
      nvals = strcmp(objectFunctionStr, ECMC_ENC_LATCHPOS_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_ENCODER_ENTRY_INDEX_LATCH_VALUE;
        return 0;
      }

      // Latch control
      nvals = strcmp(objectFunctionStr, ECMC_ENC_LATCH_CONTROL_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_ENCODER_ENTRY_INDEX_LATCH_CONTROL;
        return 0;
      }
      return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
    }

    // Monitor
    nvals = strcmp(objectTypeStr, ECMC_MON_STR);

    if (nvals == 0) {
      *objectType = ECMC_OBJ_MONITOR;

      // Lowlim
      nvals = strcmp(objectFunctionStr, ECMC_MON_LOWLIM_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_MON_ENTRY_INDEX_LOWLIM;
        return 0;
      }

      // Highlim
      nvals = strcmp(objectFunctionStr, ECMC_MON_HIGHLIM_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_MON_ENTRY_INDEX_HIGHLIM;
        return 0;
      }

      // Home sensor
      nvals = strcmp(objectFunctionStr, ECMC_MON_HOMESENSOR_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_MON_ENTRY_INDEX_HOMESENSOR;
        return 0;
      }

      // ExternalInterupt
      nvals = strcmp(objectFunctionStr, ECMC_MON_EXTINTERLOCK_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_MON_ENTRY_INDEX_EXTINTERLOCK;
        return 0;
      }
      return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
    }
  }

  // Axis object only
  nvals = sscanf(objPath, ECMC_AX_STR "%d.%s", &axisId, objectFunctionStr);

  if (nvals == 2) {
    *objectType = ECMC_OBJ_AXIS;
    *axis       = axisId;

    // Health
    nvals = strcmp(objectFunctionStr, ECMC_AX_HEALTH_STR);

    if (nvals == 0) {
      *objectFunction = ECMC_AXIS_ENTRY_INDEX_HEALTH;
      return 0;
    }
    return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
  }

  // Ec object
  int masterId = 0;
  nvals = sscanf(objPath, ECMC_EC_STR "%d.%s", &masterId, objectFunctionStr);

  if (nvals == 2) {
    *objectType = ECMC_OBJ_EC;

    // Health
    nvals = strcmp(objectFunctionStr, ECMC_EC_HEALTH_STR);

    if (nvals == 0) {
      *objectFunction = ECMC_EC_ENTRY_INDEX_HEALTH;
      return 0;
    }
    return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
  }

  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
}*/

int getEcMainFuncType(char *objPath,
                      int *objectFunction) {

  char objectFunctionStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  nvals = 0;
  int  masterId = 0;
  
  nvals = sscanf(objPath, ECMC_EC_STR "%d.%s", &masterId, objectFunctionStr);
  if (nvals == 2) {

    // Health
    nvals = strcmp(objectFunctionStr, ECMC_EC_HEALTH_STR);

    if (nvals == 0) {
      *objectFunction = ECMC_EC_ENTRY_INDEX_HEALTH;
      return 0;
    }
  }
  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
}

int getAxMainFuncType(char *objPath,
                              int *objectFunction) {

  int  axisId = 0;
  char objectFunctionStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  nvals = 0;
  
  // Axis sub objects
  nvals = sscanf(objPath,
                 ECMC_AX_STR "%d.%s",
                 &axisId,
                 objectFunctionStr);

  if (nvals == 2) {
    // Health
    nvals = strcmp(objectFunctionStr, ECMC_AX_HEALTH_STR);

    if (nvals == 0) {
      *objectFunction = ECMC_AXIS_ENTRY_INDEX_HEALTH;
      return 0;
    }
  }
  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
}

int getAxDriveFuncType(char *objPath,                              
                              int *objectFunction) {

  int  axisId = 0;
  char objectTypeStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char objectFunctionStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  nvals = 0;
  
  // Axis sub objects
  nvals = sscanf(objPath,
                 ECMC_AX_STR "%d.%[^.].%s",
                 &axisId,
                 objectTypeStr,
                 objectFunctionStr);

  if (nvals == 3) {

    // Drive
    nvals = strcmp(objectTypeStr, ECMC_DRV_STR);

    if (nvals == 0) {

      // Enable
      nvals = strcmp(objectFunctionStr, ECMC_DRV_ENABLE_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD;
        return 0;
      }

      // Velocity
      nvals = strcmp(objectFunctionStr, ECMC_DRV_VELOCITY_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT;
        return 0;
      }

      // Enabled
      nvals = strcmp(objectFunctionStr, ECMC_DRV_ENABLED_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD;
        return 0;
      }

      // Break
      nvals = strcmp(objectFunctionStr, ECMC_DRV_BREAK_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT;
        return 0;
      }

      // Reduce Torque
      nvals = strcmp(objectFunctionStr, ECMC_DRV_REDUCETORQUE_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT;
        return 0;
      }
    }
  }
  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
}

int getAxEncFuncType(char *objPath,                              
                              int *objectFunction) {

  int  axisId = 0;
  char objectTypeStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char objectFunctionStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  nvals = 0;

  // Axis sub objects
  nvals = sscanf(objPath,
                 ECMC_AX_STR "%d.%[^.].%s",
                 &axisId,
                 objectTypeStr,
                 objectFunctionStr);

  if (nvals == 3) {  

    // Encoder
    nvals = strcmp(objectTypeStr, ECMC_ENC_STR);

    if (nvals == 0) {      

      // Actpos
      nvals = strcmp(objectFunctionStr, ECMC_ENC_ACTPOS_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION;
        return 0;
      }

      // Latch status
      nvals = strcmp(objectFunctionStr, ECMC_ENC_LATCH_STATUS_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_ENCODER_ENTRY_INDEX_LATCH_STATUS;
        return 0;
      }

      // Latch pos
      nvals = strcmp(objectFunctionStr, ECMC_ENC_LATCHPOS_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_ENCODER_ENTRY_INDEX_LATCH_VALUE;
        return 0;
      }

      // Latch control
      nvals = strcmp(objectFunctionStr, ECMC_ENC_LATCH_CONTROL_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_ENCODER_ENTRY_INDEX_LATCH_CONTROL;
        return 0;
      }
    }
  }
  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
}

int getAxMonFuncType(char *objPath,                              
                            int *objectFunction) {

  int  axisId = 0;
  char objectTypeStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char objectFunctionStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  nvals = 0;
  
  // Axis sub objects
  nvals = sscanf(objPath,
                 ECMC_AX_STR "%d.%[^.].%s",
                 &axisId,
                 objectTypeStr,
                 objectFunctionStr);
  if (nvals == 3) {  
  // Monitor
    nvals = strcmp(objectTypeStr, ECMC_MON_STR);

    if (nvals == 0) {

      // Lowlim
      nvals = strcmp(objectFunctionStr, ECMC_MON_LOWLIM_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_MON_ENTRY_INDEX_LOWLIM;
        return 0;
      }

      // Highlim
      nvals = strcmp(objectFunctionStr, ECMC_MON_HIGHLIM_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_MON_ENTRY_INDEX_HIGHLIM;
        return 0;
      }

      // Home sensor
      nvals = strcmp(objectFunctionStr, ECMC_MON_HOMESENSOR_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_MON_ENTRY_INDEX_HOMESENSOR;
        return 0;
      }

      // ExternalInterupt
      nvals = strcmp(objectFunctionStr, ECMC_MON_EXTINTERLOCK_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_MON_ENTRY_INDEX_EXTINTERLOCK;
        return 0;
      }      
    }
  }
  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
}

int getAxSubObjectType(char *objPath,
                              axisSubObjectType *objectType) {

  *objectType=ECMC_AX_SUB_OBJ_INVALID;
  int  axisId = 0;
  char objectTypeStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char objectFunctionStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  nvals = 0;

  // Axis sub objects
  nvals = sscanf(objPath,
                 ECMC_AX_STR "%d.%[^.].%s",
                 &axisId,
                 objectTypeStr,
                 objectFunctionStr);

  if (nvals == 3) {

    // Drive
    nvals = strcmp(objectTypeStr, ECMC_DRV_STR);

    if (nvals == 0) {
      *objectType = ECMC_AX_SUB_OBJ_DRIVE;
      return 0;
    }

    // Encoder
    nvals = strcmp(objectTypeStr, ECMC_ENC_STR);

    if (nvals == 0) {
      *objectType = ECMC_AX_SUB_OBJ_ENCODER;
      return 0;
    }

    // Monitor
    nvals = strcmp(objectTypeStr, ECMC_MON_STR);

    if (nvals == 0) {
      *objectType = ECMC_AX_SUB_OBJ_MONITOR;
      return 0;
    }
  }

 // Axis object only
  nvals = sscanf(objPath, ECMC_AX_STR "%d.%s", &axisId, objectFunctionStr);

  if (nvals == 2) {
    *objectType = ECMC_AX_SUB_OBJ_MAIN;
    return 0;
  }

  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
}

int getMainObjectType(char           *objPath,
                             int            *objIndex,
                             mainObjectType *objectType) {

  char objectFunctionStr[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  nvals = 0;
  int objectIndex=0;
  *objectType = ECMC_OBJ_INVALID;
   
  // Axis object only
  nvals = sscanf(objPath, ECMC_AX_STR "%d.%s", &objectIndex, objectFunctionStr);

  if (nvals == 2) {
    *objectType = ECMC_OBJ_AXIS;
    *objIndex   = objectIndex;
    return 0;
  }

  // Ec object
  nvals = sscanf(objPath, ECMC_EC_STR "%d.%s", &objectIndex, objectFunctionStr);

  if (nvals == 2) {
    *objectType = ECMC_OBJ_EC;
    *objIndex=objectIndex;
    return 0;
  }

  // Ds object
  nvals = sscanf(objPath, ECMC_PLC_DATA_STORAGE_STR "%d.%s", &objectIndex, objectFunctionStr);

  if (nvals == 2) {
    *objectType = ECMC_OBJ_DS;
    *objIndex=objectIndex;
    return 0;
  }

  // Main object
  nvals = sscanf(objPath, ECMC_MAIN_STR ".%s", objectFunctionStr);

  if (nvals == 1) {
    *objectType = ECMC_OBJ_MAIN;
    *objIndex=0;  // Not used
    return 0;
  }

  // Thread object
  nvals = sscanf(objPath, ECMC_THREAD_STR ".%s", objectFunctionStr);

  if (nvals == 1) {
    *objectType = ECMC_OBJ_THREAD;
    *objIndex=0; // Not used
    return 0;
  }

  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;             
}