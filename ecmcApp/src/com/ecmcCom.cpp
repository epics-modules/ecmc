
#include "ecmcCom.h"

// TODO: REMOVE GLOBALS
#include "../general/ecmcGlobalsExtern.h"

int linkEcEntryToAsynParameter(int         masterIndex,
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
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
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
}

int linkEcMemMapToAsynParameter(int         masterIndex,
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
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
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
}

int initEcmcAsyn(void *asynPortObject) {
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
  asynPort = reinterpret_cast<ecmcAsynPortDriver *>(asynPortObject);
  ec.setAsynPortDriver(asynPort);
  return 0;
}

int addDefaultAsynEc(int masterIndex, int regAsynParams, int skipCycles) {
  LOGINFO4("%s/%s:%d masterIndex=%d regAsynParams=%d skipCycles=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           masterIndex,
           regAsynParams,
           skipCycles);

  if (asynPort == NULL) {
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
  }

  if (!ec.getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  if (ec.getMasterIndex() != masterIndex) {
    return ERROR_MAIN_EC_MASTER_NULL;
  }

  int err = ec.initAsyn(asynPort, regAsynParams, skipCycles);

  if (err) {
    return err;
  }

  if ((skipCycles < asynSkipCyclesFastest) || (asynSkipCyclesFastest < 0)) {
    asynSkipCyclesFastest = skipCycles;
  }

  return 0;
}

int addDefaultAsynEcSlave(int masterIndex,
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
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
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
}

int addDefaultAsynAxis(int regAsynParams, int axisIndex, int skipCycles) {
  LOGINFO4("%s/%s:%d regAsynParams=%d axisIndex=%d skipCycles=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           regAsynParams,
           axisIndex,
           skipCycles);

  if (asynPort == NULL) {
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
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
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
  }

  CHECK_AXIS_RETURN_IF_ERROR_AND_BLOCK_COM(axisIndex);

  // Array so updated in axis base object
  return axes[axisIndex]->initDiagAsyn(asynPort, regAsynParams, skipCycles);
}

int addDefaultAsynParams(int regAsynParams, int skipCycles) {
  LOGINFO4("%s/%s:%d regAsynParams=%d skipCycles=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           regAsynParams,
           skipCycles);

  if (asynPort == NULL) {
    return ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL;
  }

  asynSkipCyclesThread = skipCycles;

  if (!regAsynParams) {
    return 0;
  }

  // Timing info (only updated in real time)!
  asynStatus status = asynPort->createParam("ecmc.thread.latency.min",
                                            asynParamInt32,
                                            &asynParIdLatencyMin);

  if (status != asynSuccess) {
    LOGERR(
      "%s/%s:%d: ERROR: Add default asyn parameter thread.latency.min failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }
  asynPort->setIntegerParam(asynParIdLatencyMin, 0);

  status = asynPort->createParam("ecmc.thread.latency.max",
                                 asynParamInt32,
                                 &asynParIdLatencyMax);

  if (status != asynSuccess) {
    LOGERR(
      "%s/%s:%d: ERROR: Add default asyn parameter thread.latency.max failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }
  asynPort->setIntegerParam(asynParIdLatencyMax, 0);

  status = asynPort->createParam("ecmc.thread.period.min",
                                 asynParamInt32,
                                 &asynParIdPeriodMin);

  if (status != asynSuccess) {
    LOGERR(
      "%s/%s:%d: ERROR: Add default asyn parameter thread.period.min failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }
  asynPort->setIntegerParam(asynParIdPeriodMin, 0);

  status = asynPort->createParam("ecmc.thread.period.max",
                                 asynParamInt32,
                                 &asynParIdPeriodMax);

  if (status != asynSuccess) {
    LOGERR(
      "%s/%s:%d: ERROR: Add default asyn parameter thread.period.max failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }
  asynPort->setIntegerParam(asynParIdPeriodMax, 0);

  status = asynPort->createParam("ecmc.thread.execute.min",
                                 asynParamInt32,
                                 &asynParIdExecuteMin);

  if (status != asynSuccess) {
    LOGERR(
      "%s/%s:%d: ERROR: Add default asyn parameter thread.execute.min failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }
  asynPort->setIntegerParam(asynParIdExecuteMin, 0);

  status = asynPort->createParam("ecmc.thread.execute.max",
                                 asynParamInt32,
                                 &asynParIdExecuteMax);

  if (status != asynSuccess) {
    LOGERR(
      "%s/%s:%d: ERROR: Add default asyn parameter thread.execute.max failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }
  asynPort->setIntegerParam(asynParIdExecuteMax, 0);

  status = asynPort->createParam("ecmc.thread.send.min",
                                 asynParamInt32,
                                 &asynParIdSendMin);

  if (status != asynSuccess) {
    LOGERR(
      "%s/%s:%d: ERROR: Add default asyn parameter thread.send.min failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }
  asynPort->setIntegerParam(asynParIdSendMin, 0);

  status = asynPort->createParam("ecmc.thread.send.max",
                                 asynParamInt32,
                                 &asynParIdSendMax);

  if (status != asynSuccess) {
    LOGERR(
      "%s/%s:%d: ERROR: Add default asyn parameter thread.send.max failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }
  asynPort->setIntegerParam(asynParIdSendMax, 0);

  status = asynPort->createParam("ecmc.appmode",
                                 asynParamInt32,
                                 &asynParIdEcmcAppMode);

  if (status != asynSuccess) {
    LOGERR("%s/%s:%d: ERROR: Add default asyn parameter appmode failed.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    return asynError;
  }
  asynPort->setIntegerParam(asynParIdEcmcAppMode, 0);

  status = asynPort->createParam("ecmc.error.id",
                                 asynParamInt32,
                                 &asynParIdEcmcErrorId);

  if (status != asynSuccess) {
    LOGERR(
      "%s/%s:%d: ERROR: Add default asyn parameter ecmc.error.id failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }
  asynPort->setIntegerParam(asynParIdEcmcErrorId, 0);

  status = asynPort->createParam("ecmc.error.reset",
                                 asynParamInt32,
                                 &asynParIdEcmcErrorReset);

  if (status != asynSuccess) {
    LOGERR(
      "%s/%s:%d: ERROR: Add default asyn parameter ecmc.error.reset failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }
  asynPort->setIntegerParam(asynParIdEcmcErrorReset, 0);

  status = asynPort->createParam("ecmc.error.msg",
                                 asynParamInt8Array,
                                 &asynParIdEcmcErrorMsg);

  if (status != asynSuccess) {
    LOGERR(
      "%s/%s:%d: ERROR: Add default asyn parameter ecmc.error.msg failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }

  asynPort->doCallbacksInt8Array((epicsInt8 *)"NO_ERROR\0",
                                 9,
                                 asynParIdEcmcErrorMsg,
                                 0);

  asynPort->callParamCallbacks();
  asynThreadParamsEnable = 1;

  if ((skipCycles < asynSkipCyclesFastest) || (asynSkipCyclesFastest < 0)) {
    asynSkipCyclesFastest = skipCycles;
  }

  return 0;
}

