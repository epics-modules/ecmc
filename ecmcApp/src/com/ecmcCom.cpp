
#include "ecmcCom.h"
#include "ecmcGeneral.h"

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

#define ECMC_TEST_ASYN_ARRAY_ELEMENT_SIZE 128
ecmcAsynDataItem   *testAsynParams[10];
double  testDouble = 0;
int32_t testInt32 = 0;
int8_t  testInt8Array[ECMC_TEST_ASYN_ARRAY_ELEMENT_SIZE];
int16_t testInt16Array[ECMC_TEST_ASYN_ARRAY_ELEMENT_SIZE];
int32_t testInt32Array[ECMC_TEST_ASYN_ARRAY_ELEMENT_SIZE];
float   testFloatArray[ECMC_TEST_ASYN_ARRAY_ELEMENT_SIZE];
double  testDoubleArray[ECMC_TEST_ASYN_ARRAY_ELEMENT_SIZE];

int ecmcAddTestParams() {

  const char * name;
  //Initilaize test data area
  memset(testInt8Array,0,sizeof(testInt8Array));
  memset(testInt16Array,0,sizeof(testInt16Array));
  memset(testInt32Array,0,sizeof(testInt32Array));
  memset(testFloatArray,0,sizeof(testFloatArray));
  memset(testDoubleArray,0,sizeof(testDoubleArray));
  
  // Test int 32
  name = "ecmc.test.int32";
  ecmcAsynDataItem *paramTemp=NULL;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt32,
                                           (uint8_t *)&(testInt32),
                                           sizeof(testInt32),
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
  testAsynParams[0] = paramTemp;

  // Test double
  name = "ecmc.test.double";
  paramTemp=NULL;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamFloat64,
                                           (uint8_t *)&(testDouble),
                                           sizeof(testDouble),
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
  testAsynParams[1] = paramTemp;

  // Test int8 array (string)
  name = "ecmc.test.int8array";
  paramTemp=NULL;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt8Array,
                                           (uint8_t *)testInt8Array,
                                           sizeof(testInt8Array),
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
  testAsynParams[2] = paramTemp;

  // Test int16 array
  name = "ecmc.test.int16array";
  paramTemp=NULL;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt16Array,
                                           (uint8_t *)testInt16Array,
                                           sizeof(testInt16Array),
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
  testAsynParams[3] = paramTemp;

  // Test int32 array
  name = "ecmc.test.int32array";
  paramTemp=NULL;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt32Array,
                                           (uint8_t *)testInt32Array,
                                           sizeof(testInt32Array),
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
  testAsynParams[4] = paramTemp;

  // Test float array
  name = "ecmc.test.float32array";
  paramTemp=NULL;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamFloat32Array,
                                           (uint8_t *)testFloatArray,
                                           sizeof(testFloatArray),
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
  testAsynParams[5] = paramTemp;

  // Test double array
  name = "ecmc.test.double64array";
  paramTemp=NULL;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamFloat64Array,
                                           (uint8_t *)testDoubleArray,
                                           sizeof(testDoubleArray),
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
  testAsynParams[6] = paramTemp;

  return 0;
}

int ecmcAddDefaultAsynParams() {

  // Add test params

  ecmcAddTestParams();

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
  controllerErrorMsg=getErrorString(controllerError);
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
  paramTemp->refreshParam(1,(uint8_t*)controllerErrorMsg,strlen(controllerErrorMsg));
  mainAsynParams[ECMC_ASYN_MAIN_PAR_ERROR_MSG_ID] = paramTemp;

  return 0;
}