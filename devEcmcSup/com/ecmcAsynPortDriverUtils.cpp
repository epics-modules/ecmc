/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcAsynPortDriverUtils.cpp
*
*  Created on: Jan 25, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcAsynPortDriverUtils.h"
#include "../main/ecmcErrorsList.h"

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
#ifdef ECMC_ASYN_ASYNPARAMINT64
    case asynParamInt64:          
      return "asynParamInt64";
    case asynParamInt64Array:
      return "asynParamInt64Array";
#endif // ECMC_ASYN_ASYNPARAMINT64

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

 #ifdef ECMC_ASYN_ASYNPARAMINT64
  if(strcmp("asynInt64ArrayIn",typeStr)==0 || strcmp("asynInt64ArrayOut",typeStr)==0){
    return asynParamInt64Array;
  }
  if(strcmp("asynInt64",typeStr) == 0 ){
    return asynParamInt64;
  }
#endif // ECMC_ASYN_ASYNPARAMINT64


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

      // Position
      nvals = strcmp(objectFunctionStr,ECMC_DRV_POSITION_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_POSITION_SETPOINT;
        return 0;
      }

      // Reset alarm
      nvals = strcmp(objectFunctionStr,ECMC_DRV_RESET_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_RESET;
        return 0;
      }

      // Warning
      nvals = strcmp(objectFunctionStr,ECMC_DRV_WARNING_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_WARNING;
        return 0;
      }

      // Alarm 0
      nvals = strcmp(objectFunctionStr,ECMC_DRV_ALARM_0_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_ALARM_0;
        return 0;
      }

      // Alarm 1
      nvals = strcmp(objectFunctionStr,ECMC_DRV_ALARM_1_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_ALARM_1;
        return 0;
      }

      // Alarm 2
      nvals = strcmp(objectFunctionStr,ECMC_DRV_ALARM_2_STR);

      if (nvals == 0) {
        *objectFunction = ECMC_DRIVEBASE_ENTRY_INDEX_ALARM_2;
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

// Convert string to datatype
ecmcEcDataType getEcDataTypeFromStr(const char* dt) {
  int n=0; 
  n = strcmp(dt,EC_DT_BIT1);
  if (n == 0) {
    return ECMC_EC_B1;
  }

  n = strcmp(dt,EC_DT_BIT2);
  if (n == 0) {
    return ECMC_EC_B2;
  }

  n = strcmp(dt,EC_DT_BIT3);
  if (n == 0) {
    return ECMC_EC_B3;
  }

  n = strcmp(dt,EC_DT_BIT4);
  if (n == 0) {
    return ECMC_EC_B4;
  }

  n = strcmp(dt,EC_DT_U8);
  if (n == 0) {
    return ECMC_EC_U8;
  }

  n = strcmp(dt,EC_DT_S8);
  if (n == 0) {
    return ECMC_EC_S8;
  }

  n = strcmp(dt,EC_DT_U16);
  if (n == 0) {
    return ECMC_EC_U16;
  }

  n = strcmp(dt,EC_DT_S16);
  if (n == 0) {
    return ECMC_EC_S16;
  }

  n = strcmp(dt,EC_DT_U32);
  if (n == 0) {
    return ECMC_EC_U32;
  }

  n = strcmp(dt,EC_DT_S32);
  if (n == 0) {
    return ECMC_EC_S32;
  }

  n = strcmp(dt,EC_DT_U64);
  if (n == 0) {
    return ECMC_EC_U64;
  }

  n = strcmp(dt,EC_DT_S64);
  if (n == 0) {
    return ECMC_EC_S64;
  }

  n = strcmp(dt,EC_DT_F32);
  if (n == 0) {
    return ECMC_EC_F32;
  }

  n = strcmp(dt,EC_DT_F64);
  if (n == 0) {
    return ECMC_EC_F64;
  }

  // Not a valid type
  return ECMC_EC_NONE;
}

// Convert string to datatype
size_t getEcDataTypeBits(ecmcEcDataType dt) {
  
  switch(dt) {
  case ECMC_EC_NONE:
    return 0;
    break;

  case ECMC_EC_B1:
    return 1;
    break;

  case ECMC_EC_B2:
    return 2;
    break;

  case ECMC_EC_B3:
    return 3;
    break;

  case ECMC_EC_B4:
    return 4;
    break;

  case ECMC_EC_U8:
    return 8;
    break;

  case ECMC_EC_S8:
    return 8;
    break;

  case ECMC_EC_U16:
    return 16;
    break;

  case ECMC_EC_S16:
    return 16;
    break;

  case ECMC_EC_U32:
    return 32;
    break;

  case ECMC_EC_S32:
    return 32;
    break;

  case ECMC_EC_U64:
    return 64;
    break;

  case ECMC_EC_S64:
    return 64;
    break;

  case ECMC_EC_F32:
    return 32;
    break;

  case ECMC_EC_F64:
    return 64;
    break;

  default:
    return 0;
    break;
  }
  // Not a valid type
  return 0;
}

/*
* For legacy support of old Cfg.EcAddEntryComplete() syntax
* \note Will not work for data of types double or real
*/
ecmcEcDataType getEcDataType(size_t bitLength,bool signedVal) {
  switch(bitLength)
  {
    case 1:
      return ECMC_EC_B1;
      break;

    case 2:
      return ECMC_EC_B2;
      break;

    case 3:
      return ECMC_EC_B3;
      break;

    case 4:
      return ECMC_EC_B4;
      break;

    case 8:
      if(signedVal){
        return ECMC_EC_S8;
      }
      else {
        return ECMC_EC_U8;
      }
      break;

    case 16:
      if(signedVal){
        return ECMC_EC_S16;
      }
      else {
        return ECMC_EC_U16;
      }    
      break;

    case 32:
      if(signedVal){
        return ECMC_EC_S32;
      }
      else {
        return ECMC_EC_U32;
      }    
    
      break;
    case 64:
      if(signedVal){
        return ECMC_EC_S64;
      }
      else {
        return ECMC_EC_U64;
      }    
    
      break;
    
    default:    
      return ECMC_EC_NONE;
      break;
  }

  return ECMC_EC_NONE;
}

uint64_t getEcDataTypeMaxVal(ecmcEcDataType dt) {
  switch(dt) {
  case ECMC_EC_NONE:
    return 0;
    break;

  case ECMC_EC_B1:
    return 1;
    break;

  case ECMC_EC_B2:
    return 3;
    break;

  case ECMC_EC_B3:
    return 7;
    break;

  case ECMC_EC_B4:
    return 15;
    break;

  case ECMC_EC_U8:
    return std::numeric_limits<uint8_t>::max();
    break;

  case ECMC_EC_S8:
    return std::numeric_limits<int8_t>::max();
    break;

  case ECMC_EC_U16:
    return std::numeric_limits<uint16_t>::max();
    break;

  case ECMC_EC_S16:
    return std::numeric_limits<int16_t>::max();
    break;

  case ECMC_EC_U32:
    return std::numeric_limits<uint32_t>::max();
    break;

  case ECMC_EC_S32:
    return std::numeric_limits<int32_t>::max();
    break;

  case ECMC_EC_U64:
    return std::numeric_limits<uint64_t>::max();
    break;

  case ECMC_EC_S64:
    return std::numeric_limits<int64_t>::max();
    break;

  case ECMC_EC_F32:
    return -1; //Not used of F32
    break;

  case ECMC_EC_F64:
    return -1; //Not used of F64
    break;

  default:
    return 0;
    break;
  }
  return 0;
}

int64_t getEcDataTypeMinVal(ecmcEcDataType dt) {
  switch(dt) {
  case ECMC_EC_NONE:
    return 0;
    break;

  case ECMC_EC_B1:
    return 0;
    break;

  case ECMC_EC_B2:
    return 0;
    break;

  case ECMC_EC_B3:
    return 0;
    break;

  case ECMC_EC_B4:
    return 0;
    break;

  case ECMC_EC_U8:
    return 0;
    break;

  case ECMC_EC_S8:
    return std::numeric_limits<int8_t>::min();
    break;

  case ECMC_EC_U16:
    return 0;
    break;

  case ECMC_EC_S16:
    return std::numeric_limits<int16_t>::min();
    break;

  case ECMC_EC_U32:
    return 0;
    break;

  case ECMC_EC_S32:
    return std::numeric_limits<int32_t>::min();
    break;

  case ECMC_EC_U64:
    return 0;
    break;

  case ECMC_EC_S64:
    return std::numeric_limits<int64_t>::min();
    break;

  case ECMC_EC_F32:
    return 1; //Not used of F32
    break;

  case ECMC_EC_F64:
    return 1; //Not used of F64
    break;

  default:
    return 0;
    break;
  }
  return 0;
}

int getEcDataTypeSigned(ecmcEcDataType dt) {
  switch(dt) {
  case ECMC_EC_NONE:
    return 0;
    break;

  case ECMC_EC_B1:
    return 0;
    break;

  case ECMC_EC_B2:
    return 0;
    break;

  case ECMC_EC_B3:
    return 0;
    break;

  case ECMC_EC_B4:
    return 0;
    break;

  case ECMC_EC_U8:
    return 0;
    break;

  case ECMC_EC_S8:
    return 1;
    break;

  case ECMC_EC_U16:
    return 0;
    break;

  case ECMC_EC_S16:
    return 1;
    break;

  case ECMC_EC_U32:
    return 0;
    break;

  case ECMC_EC_S32:
    return 1;
    break;

  case ECMC_EC_U64:
    return 0;
    break;

  case ECMC_EC_S64:
    return 1;
    break;

  case ECMC_EC_F32:
    return 0;
    break;

  case ECMC_EC_F64:
    return 0;
    break;

  default:
    return 0;
    break;
  }

  return 0;
}

const char*getEcDataTypeStr(ecmcEcDataType dt) {
  switch(dt) {
  case ECMC_EC_NONE:
    return EC_DT_NONE;
    break;

  case ECMC_EC_B1:
    return EC_DT_BIT1;
    break;

  case ECMC_EC_B2:
    return EC_DT_BIT2;
    break;

  case ECMC_EC_B3:
    return EC_DT_BIT3;
    break;

  case ECMC_EC_B4:
    return EC_DT_BIT4;
    break;

  case ECMC_EC_U8:
    return EC_DT_U8;
    break;

  case ECMC_EC_S8:
    return EC_DT_S8;
    break;

  case ECMC_EC_U16:
    return EC_DT_U16;
    break;

  case ECMC_EC_S16:
    return EC_DT_S16;
    break;

  case ECMC_EC_U32:
    return EC_DT_U32;
    break;

  case ECMC_EC_S32:
    return EC_DT_S32;
    break;

  case ECMC_EC_U64:
    return EC_DT_U64;
    break;

  case ECMC_EC_S64:
    return EC_DT_S64;
    break;

  case ECMC_EC_F32:
    return EC_DT_F32;
    break;

  case ECMC_EC_F64:
    return EC_DT_F64;
    break;

  default:
    return EC_DT_NONE;
    break;
  }

  return EC_DT_NONE;
}

size_t getEcDataTypeByteSize(ecmcEcDataType dt){
  switch(dt) {
  case ECMC_EC_NONE:
    return 0;
    break;

  case ECMC_EC_B1:
    return 1;
    break;

  case ECMC_EC_B2:
    return 1;
    break;

  case ECMC_EC_B3:
    return 1;
    break;

  case ECMC_EC_B4:
    return 1;
    break;

  case ECMC_EC_U8:
    return 1;
    break;

  case ECMC_EC_S8:
    return 1;
    break;

  case ECMC_EC_U16:
    return 2;
    break;

  case ECMC_EC_S16:
    return 2;
    break;

  case ECMC_EC_U32:
    return 4;
    break;

  case ECMC_EC_S32:
    return 4;
    break;

  case ECMC_EC_U64:
    return 8;
    break;

  case ECMC_EC_S64:
    return 8;
    break;

  case ECMC_EC_F32:
    return 4;
    break;

  case ECMC_EC_F64:
    return 8;
    break;

  default:
    return 0;
    break;
  }

  return 0;
}
