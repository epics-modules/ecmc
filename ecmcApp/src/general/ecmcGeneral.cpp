#include "ecmcGeneral.h"


// TODO: REMOVE GLOBALS
#include "../general/ecmcGlobalsExtern.h"

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