#include "ecmcGeneral.h"


// TODO: REMOVE GLOBALS
#include "../main/ecmcGlobalsExtern.h"

int getControllerError() {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);

  // EtherCAT errors
  if (ec.getError()) {
    return ec.getErrorID();
  }

  // Event errors
  for (int i = 0; i < ECMC_MAX_EVENT_OBJECTS; i++) {
    if (events[i] != NULL) {
      if (events[i]->getError()) {
        return events[i]->getErrorID();
      }
    }
  }

  // DataRecorders
  for (int i = 0; i < ECMC_MAX_DATA_RECORDERS_OBJECTS; i++) {
    if (dataRecorders[i] != NULL) {
      if (dataRecorders[i]->getError()) {
        return dataRecorders[i]->getErrorID();
      }
    }
  }

  // Data Storages
  for (int i = 0; i < ECMC_MAX_DATA_STORAGE_OBJECTS; i++) {
    if (dataStorages[i] != NULL) {
      if (dataStorages[i]->getError()) {
        return dataStorages[i]->getErrorID();
      }
    }
  }

  // CommandLists
  for (int i = 0; i < ECMC_MAX_COMMANDS_LISTS; i++) {
    if (commandLists[i] != NULL) {
      if (commandLists[i]->getError()) {
        return commandLists[i]->getErrorID();
      }
    }
  }

  // PLC:s
  if (plcs != NULL) {
    if (plcs->getError()) {
      return plcs->getErrorID();
    }
  }

  // Axes
  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    if (axes[i] != NULL) {
      if (axes[i]->getError()) {
        return axes[i]->getErrorID();
      }
    }
  }

  return 0;
}

int controllerErrorReset() {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);

  // EtherCAT errors
  ec.errorReset();

  // Event errors
  for (int i = 0; i < ECMC_MAX_EVENT_OBJECTS; i++) {
    if (events[i] != NULL) {
      events[i]->errorReset();
    }
  }

  // DataRecorders
  for (int i = 0; i < ECMC_MAX_DATA_RECORDERS_OBJECTS; i++) {
    if (dataRecorders[i] != NULL) {
      dataRecorders[i]->errorReset();
    }
  }

  // Data Storages
  for (int i = 0; i < ECMC_MAX_DATA_STORAGE_OBJECTS; i++) {
    if (dataStorages[i] != NULL) {
      dataStorages[i]->errorReset();
    }
  }

  // CommandLists
  for (int i = 0; i < ECMC_MAX_COMMANDS_LISTS; i++) {
    if (commandLists[i] != NULL) {
      commandLists[i]->errorReset();
    }
  }

  // Axes
  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    if (axes[i] != NULL) {
      axes[i]->errorReset();
    }
  }

  // PLCs
  if (plcs) {
    plcs->errorReset();
  }

  return 0;
}

const char* getErrorString(int error_number) {
  LOGINFO4("%s/%s:%d error_no=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           error_number);

  return ecmcError::convertErrorIdToString(error_number);
}


int setEnableTimeDiag(int value) {
  LOGINFO4("%s/%s:%d enable=%d \n", __FILE__, __FUNCTION__, __LINE__, value);

  WRITE_DIAG_BIT(FUNCTION_TIMING_DIAGNOSTICS_BIT, value);

  return 0;
}

int setEnableFunctionCallDiag(int value) {
  LOGINFO4("%s/%s:%d value=%d\n", __FILE__, __FUNCTION__, __LINE__, value);

  WRITE_DIAG_BIT(FUNCTION_CALL_DIAGNOSTICS_BIT, value);

  return 0;
}


/*Available strings:
 *  ec<masterId>.s<slaveId>.<alias>  (defaults complete ecentry)
 *  ec<masterId>.s<slaveId>.<alias>.<bit> (only one bit)
*/
static int parseEcPath(char *ecPath,
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

static int parseObjectPath(char             *objPath,
                           int              *axis,
                           motionObjectType *objectType,
                           int              *objectFunction) {
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
}

int linkEcEntryToObject(char *ecPath, char *axPath) {
  LOGINFO4("%s/%s:%d ecPath=%s axPath=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ecPath,
           axPath);

  int  masterId   = -1;
  int  slaveIndex = -1;
  char alias[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int  bitIndex = -1;

  int errorCode =
    parseEcPath(ecPath, &masterId, &slaveIndex, alias, &bitIndex);

  if (errorCode) {
    return errorCode;
  }

  int axisIndex               = 0;
  motionObjectType objectType = ECMC_OBJ_INVALID;
  int entryIndex              = 0;
  errorCode = parseObjectPath(axPath, &axisIndex, &objectType, &entryIndex);

  if (errorCode) {
    return errorCode;
  }

  switch (objectType) {
  case ECMC_OBJ_INVALID:
    return ERROR_MAIN_ECMC_LINK_INVALID;

    break;

  case ECMC_OBJ_DRIVE:
    return linkEcEntryToAxisDrv(slaveIndex,
                                alias,
                                axisIndex,
                                entryIndex,
                                bitIndex);

    break;

  case ECMC_OBJ_ENCODER:
    return linkEcEntryToAxisEnc(slaveIndex,
                                alias,
                                axisIndex,
                                entryIndex,
                                bitIndex);

    break;

  case ECMC_OBJ_MONITOR:
    return linkEcEntryToAxisMon(slaveIndex,
                                alias,
                                axisIndex,
                                entryIndex,
                                bitIndex);

    break;

  case ECMC_OBJ_CONTROLLER:
    return ERROR_MAIN_ECMC_LINK_INVALID;

    break;

  case ECMC_OBJ_TRAJECTORY:
    return ERROR_MAIN_ECMC_LINK_INVALID;

    break;

  case ECMC_OBJ_AXIS:

    if (entryIndex == ECMC_AXIS_ENTRY_INDEX_HEALTH) {
      return linkEcEntryToAxisStatusOutput(slaveIndex, alias, axisIndex);
    }
    return ERROR_MAIN_ECMC_LINK_INVALID;

    break;

  case ECMC_OBJ_EC:

    if (entryIndex == ECMC_EC_ENTRY_INDEX_HEALTH) {
      return linkEcEntryToEcStatusOutput(slaveIndex, alias);
    }
    return ERROR_MAIN_ECMC_LINK_INVALID;

    break;
  }
  return ERROR_MAIN_ECMC_LINK_INVALID;
}