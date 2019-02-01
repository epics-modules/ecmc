
#include "ecmcCom.h"

// TODO: REMOVE GLOBALS
#include "../main/ecmcGlobalsExtern.h"

/*int linkEcEntryToAsynParameter(int         masterIndex,
                               int         busPosition,
                               const char *entryIDString,
                               int         asynParType,
                               int         skipCycles) {
  LOGINFO4(
    "%s/%s:%d masterIndex=%d busPosition=%d alias=%s type=%d,skipCycles=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    masterIndex,
    busPosition,
    entryIDString,
    asynParType,
    skipCycles);

  if (asynPort == NULL) {
    return ERROR_MAIN_ASYN_PORT_DRIVER_NULL;
  }

  if (!ec.getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  if (ec.getMasterIndex() != masterIndex) {
    return ERROR_MAIN_EC_MASTER_NULL;
  }

  int err = ec.linkEcEntryToAsynParameter(asynPort,
                                          entryIDString,
                                          asynParType,
                                          skipCycles);

  if (err) {
    return err;
  }

  if ((skipCycles < asynSkipCyclesFastest) || (asynSkipCyclesFastest < 0)) {
    asynSkipCyclesFastest = skipCycles;
  }

  return 0;
}*/

/*int linkEcMemMapToAsynParameter(int         masterIndex,
                                const char *memMapIDString,
                                int         asynParType,
                                int         skipCycles) {
  LOGINFO4("%s/%s:%d masterIndex=%d alias=%s type=%d,skipCycles=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           masterIndex,
           memMapIDString,
           asynParType,
           skipCycles);

  if (asynPort == NULL) {
    return ERROR_MAIN_ASYN_PORT_DRIVER_NULL;
  }

  if (!ec.getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  if (ec.getMasterIndex() != masterIndex) {
    return ERROR_MAIN_EC_MASTER_NULL;
  }

  int err = ec.linkEcMemMapToAsynParameter(asynPort,
                                           memMapIDString,
                                           asynParType,
                                           skipCycles);

  if (err) {
    return err;
  }

  if ((skipCycles < asynSkipCyclesFastest) || (asynSkipCyclesFastest < 0)) {
    asynSkipCyclesFastest = skipCycles;
  }

  return 0;
}*/

int ecmcInitAsyn(void *asynPortObject) {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
  asynPort = reinterpret_cast<ecmcAsynPortDriver *>(asynPortObject);
  ec.setAsynPortDriver(asynPort);

  //Main params
  int errorCode=ecmcAddDefaultAsynParams();
  if(errorCode) {
    return errorCode;
  }


  return 0;
}

/*int addDefaultAsynEc(int masterIndex) {
  LOGINFO4("%s/%s:%d masterIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           masterIndex);

  if (asynPort == NULL) {
    return ERROR_MAIN_ASYN_PORT_DRIVER_NULL;
  }

  if (!ec.getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  if (ec.getMasterIndex() != masterIndex) {
    return ERROR_MAIN_EC_MASTER_NULL;
  }

  
  return 0;
}*/

/*int addDefaultAsynEcSlave(int masterIndex,
                          int busPosition,
                          int regAsynParams,
                          int skipCycles) {
  LOGINFO4(
    "%s/%s:%d masterIndex=%d busPosition=%d regAsynParams=%d skipCycles=%d\n",
    __FILE__,
    __FUNCTION__,
    __LINE__,
    masterIndex,
    busPosition,
    regAsynParams,
    skipCycles);

  if (asynPort == NULL) {
    return ERROR_MAIN_ASYN_PORT_DRIVER_NULL;
  }

  if (!ec.getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  if (ec.getMasterIndex() != masterIndex) {
    return ERROR_MAIN_EC_MASTER_NULL;
  }

  ecmcEcSlave *tempSlave = ec.findSlave(busPosition);

  if (!tempSlave) {
    return ERROR_MAIN_EC_SLAVE_NULL;
  }

  int err = tempSlave->initAsyn(asynPort,
                                regAsynParams,
                                skipCycles,
                                masterIndex);

  if (err) {
    return err;
  }

  if ((skipCycles < asynSkipCyclesFastest) || (asynSkipCyclesFastest < 0)) {
    asynSkipCyclesFastest = skipCycles;
  }

  return 0;
}*/

/*int addDefaultAsynAxis(int regAsynParams, int axisIndex, int skipCycles) {
  LOGINFO4("%s/%s:%d regAsynParams=%d axisIndex=%d skipCycles=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           regAsynParams,
           axisIndex,
           skipCycles);

  if (asynPort == NULL) {
    return ERROR_MAIN_ASYN_PORT_DRIVER_NULL;
  }

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);


  int err = axes[axisIndex]->initAsyn(asynPort, regAsynParams, skipCycles);

  if (err) {
    return err;
  }

  if ((skipCycles < asynSkipCyclesFastest) || (asynSkipCyclesFastest < 0)) {
    asynSkipCyclesFastest = skipCycles;
  }

  return 0;
}

int addDiagAsynAxis(int regAsynParams, int axisIndex, int skipCycles) {
  LOGINFO4("%s/%s:%d regAsynParams=%d axisIndex=%d skipCycles=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           regAsynParams,
           axisIndex,
           skipCycles);

  if (asynPort == NULL) {
    return ERROR_MAIN_ASYN_PORT_DRIVER_NULL;
  }

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);

  // Array so updated in axis base object
  return axes[axisIndex]->initDiagAsyn(asynPort, regAsynParams, skipCycles);
}*/

int ecmcAddDefaultAsynParams() {
  LOGINFO4("%s/%s:%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);

  if (asynPort == NULL) {
    return ERROR_MAIN_ASYN_PORT_DRIVER_NULL;
  }

  for(int i=0;i<ECMC_ASYN_MAIN_PAR_COUNT;i++) {
    mainAsynParams[i] = NULL;
  }

  const char * name;

  // ECMC_ASYN_MAIN_PAR_LATENCY_MIN_NAME
  name = ECMC_ASYN_MAIN_PAR_LATENCY_MIN_NAME;
  ecmcAsynDataItem *paramTemp=NULL;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt32,
                                           (uint8_t *)&(threadDiag.latency_min_ns),
                                           sizeof(threadDiag.latency_min_ns),
                                           0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_LATENCY_MIN_ID] = paramTemp;

  // ECMC_ASYN_MAIN_PAR_LATENCY_MAX_NAME
  name = ECMC_ASYN_MAIN_PAR_LATENCY_MAX_NAME;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt32,
                                           (uint8_t *)&(threadDiag.latency_max_ns),
                                           sizeof(threadDiag.latency_max_ns),
                                           0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_LATENCY_MAX_ID] = paramTemp;

  // ECMC_ASYN_MAIN_PAR_PERIOD_MIN_NAME
  name = ECMC_ASYN_MAIN_PAR_PERIOD_MIN_NAME;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt32,
                                           (uint8_t *)&(threadDiag.period_min_ns),
                                           sizeof(threadDiag.period_min_ns),
                                           0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_PERIOD_MIN_ID] = paramTemp;

  // ECMC_ASYN_MAIN_PAR_PERIOD_MAX_NAME
  name = ECMC_ASYN_MAIN_PAR_PERIOD_MAX_NAME;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt32,
                                           (uint8_t *)&(threadDiag.period_max_ns),
                                           sizeof(threadDiag.period_max_ns),
                                           0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_PERIOD_MAX_ID] = paramTemp;

  // ECMC_ASYN_MAIN_PAR_EXECUTE_MIN_NAME
  name = ECMC_ASYN_MAIN_PAR_EXECUTE_MIN_NAME;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt32,
                                           (uint8_t *)&(threadDiag.exec_min_ns),
                                           sizeof(threadDiag.exec_min_ns),
                                           0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_EXECUTE_MIN_ID] = paramTemp;

  // ECMC_ASYN_MAIN_PAR_EXECUTE_MAX_NAME
  name = ECMC_ASYN_MAIN_PAR_EXECUTE_MAX_NAME;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt32,
                                           (uint8_t *)&(threadDiag.exec_max_ns),
                                           sizeof(threadDiag.exec_max_ns),
                                           0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_EXECUTE_MAX_ID] = paramTemp;

  // ECMC_ASYN_MAIN_PAR_SEND_MIN_NAME
  name = ECMC_ASYN_MAIN_PAR_SEND_MIN_NAME;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt32,
                                           (uint8_t *)&(threadDiag.send_min_ns),
                                           sizeof(threadDiag.send_min_ns),
                                           0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_SEND_MIN_ID] = paramTemp;

  // ECMC_ASYN_MAIN_PAR_SEND_MAX_NAME
  name = ECMC_ASYN_MAIN_PAR_SEND_MAX_NAME;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt32,
                                           (uint8_t *)&(threadDiag.send_max_ns),
                                           sizeof(threadDiag.send_max_ns),
                                           0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_SEND_MAX_ID] = paramTemp;

  // ECMC_ASYN_MAIN_PAR_APP_MODE_NAME
  name = ECMC_ASYN_MAIN_PAR_APP_MODE_NAME;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt32,
                                           (uint8_t *)&(appModeCmd),
                                           sizeof(appModeCmd),
                                           0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_APP_MODE_ID] = paramTemp;  

  // ECMC_ASYN_MAIN_PAR_ERROR_ID_NAME
  name = ECMC_ASYN_MAIN_PAR_ERROR_ID_NAME;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt32,
                                           (uint8_t *)&(controllerError),
                                           sizeof(controllerError),
                                           0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_ERROR_ID_ID] = paramTemp;

  // ECMC_ASYN_MAIN_PAR_RESET_NAME
  name = ECMC_ASYN_MAIN_PAR_RESET_NAME;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt32,
                                           (uint8_t *)&(controllerReset),
                                           sizeof(controllerReset),
                                           0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(true);
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_RESET_ID] = paramTemp;  

  // ECMC_ASYN_MAIN_PAR_ERROR_MSG_NAME
  name = ECMC_ASYN_MAIN_PAR_ERROR_MSG_NAME;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt8Array,
                                           (uint8_t *)(controllerErrorMsg),
                                           strlen(controllerErrorMsg),
                                           0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_ERROR_MSG_ID] = paramTemp;

  return 0;
}