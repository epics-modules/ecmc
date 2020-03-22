/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcCom.cpp
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcCom.h"
#include "ecmcGeneral.h"

// TODO: REMOVE GLOBALS
#include "../main/ecmcGlobalsExtern.h"

int ecmcInit(void *asynPortObject) {
  
  LOGINFO4("%s/%s:%d\n", __FILE__, __FUNCTION__, __LINE__);

  ecmcRTMutex = epicsMutexCreate();
  if(!ecmcRTMutex) {
    LOGERR("ERROR: Failed to create Mutex (epicsMutexCreate())  (0x%x)",
            ERROR_MAIN_RT_MUTEX_NULL);
    return ERROR_MAIN_RT_MUTEX_NULL;
  }

  asynPort = reinterpret_cast<ecmcAsynPortDriver *>(asynPortObject);  
  ec = new ecmcEc();

  if(!ec) {
    LOGERR("ERROR: Fail allocate ec master (0x%x)",ERROR_MAIN_EC_NULL);
    return ERROR_MAIN_EC_NULL;
  }

  ec->setAsynPortDriver(asynPort);

  //Main asyn params
  int errorCode=ecmcAddDefaultAsynParams();
  if(errorCode) {
    return errorCode;
  }

  return 0;
}

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

  for(int i=0;i < ECMC_TEST_ASYN_ARRAY_ELEMENT_SIZE; i++) {
    testInt8Array[i] = i;
    testInt16Array[i] = i*2;
    testInt32Array[i] = i*3;
    testFloatArray[i] = i*4;
    testDoubleArray[i] = i*5;  
  }
  
  // Test int 32
  name = "ecmc.test.int32";
  ecmcAsynDataItem *paramTemp=NULL;
  paramTemp = asynPort->addNewAvailParam(name,
                                         asynParamInt32,
                                         (uint8_t *)&(testInt32),
                                         sizeof(testInt32),
                                         ECMC_EC_S32,
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
                                           ECMC_EC_F64,
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
                                           ECMC_EC_S8,
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
                                           ECMC_EC_S16,
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
                                           ECMC_EC_S32,
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
                                           ECMC_EC_F32,
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
                                           ECMC_EC_F64,
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

void ecmcDelDefaultAsynParams() {
  for(int i=0;i<ECMC_ASYN_MAIN_PAR_COUNT; i++) {
    delete mainAsynParams[i];
    mainAsynParams[i] = NULL;
  }
}

void ecmcCleanup() {

  ecmcDelDefaultAsynParams();

  delete plcs;
  plcs = NULL;
  
  for(int i = 0; i < ECMC_MAX_EVENT_OBJECTS; i++) {
    delete events[i];
    events[i] = NULL;
  }

  for(int i = 0;i < ECMC_MAX_DATA_RECORDERS_OBJECTS; i++) {
    delete dataRecorders[i];
    dataRecorders[i] = NULL;
  }

  for(int i = 0; i < ECMC_MAX_DATA_STORAGE_OBJECTS; i++) {
    delete dataStorages[i];
    dataStorages[i] = NULL;
  }

  for(int i = 0;i < ECMC_MAX_COMMANDS_LISTS; i++) {
    delete commandLists[i];
    commandLists[i] = NULL;
  }

  delete ec;
  ec = NULL;
}

/**
 * 
 * Callback function for asynWrites (commands): ecmc.error.reset
 * userObj = NULL
 *  Will controllerErrorReset()   
 * 
 */ 
asynStatus asynWriteReset(void* data, size_t bytes, asynParamType asynParType,void *userObj) {
  // userobj = NULL
  if(asynParType!=asynParamInt32 || bytes != sizeof(asynParamInt32)) {
    return asynError;
  }
  int reset = *(int32_t*)data;
  if(reset){
    return (asynStatus)controllerErrorReset();
  }
  return asynSuccess;
}

int ecmcAddDefaultAsynParams() {

  // Add test params.. Uncomment to get a few parameters usefull for testing
  //ecmcAddTestParams();

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
                                           ECMC_EC_S32,
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
                                           ECMC_EC_S32,
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
                                           ECMC_EC_S32,
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
                                           ECMC_EC_S32,
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
                                           ECMC_EC_S32,
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
                                           ECMC_EC_S32,
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
                                           ECMC_EC_S32,
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
                                           ECMC_EC_S32,
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
                                           ECMC_EC_S32,
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
                                           ECMC_EC_S32,
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
                                           ECMC_EC_S32,
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
  paramTemp->setExeCmdFunctPtr(asynWriteReset,NULL); // Nu object defined
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_RESET_ID] = paramTemp;  

  // ECMC_ASYN_MAIN_PAR_ERROR_MSG_NAME
  controllerErrorMsg=getErrorString(controllerError);
  name = ECMC_ASYN_MAIN_PAR_ERROR_MSG_NAME;
  paramTemp = asynPort->addNewAvailParam(name,
                                           asynParamInt8Array,
                                           (uint8_t *)(controllerErrorMsg),
                                           strlen(controllerErrorMsg),
                                           ECMC_EC_S8,
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
  paramTemp->setArrayCheckSize(false);
  paramTemp->refreshParam(1,(uint8_t*)controllerErrorMsg,strlen(controllerErrorMsg));
  mainAsynParams[ECMC_ASYN_MAIN_PAR_ERROR_MSG_ID] = paramTemp;

// ECMC_ASYN_MAIN_PAR_UPDATE_READY_NAME  
  name = ECMC_ASYN_MAIN_PAR_UPDATE_READY_NAME;
  paramTemp = asynPort->addNewAvailParam(name,
                                         asynParamInt32Array,
                                         (uint8_t *)(&ecmcUpdatedCounter),
                                         4,
                                         ECMC_EC_S32,
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
  paramTemp->setArrayCheckSize(false);
  paramTemp->refreshParam(1);
  mainAsynParams[ECMC_ASYN_MAIN_PAR_UPDATE_READY_ID] = paramTemp;

  return 0;
}