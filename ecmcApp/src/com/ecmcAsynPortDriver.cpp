#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <iocsh.h>

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
#include <initHooks.h>
#include <alarm.h>

#include "ecmcAsynPortDriver.h"
#include "../main/gitversion.h"
#include "ecmcOctetIF.h"
#include "ecmcCmdParser.h"

#include "../main/ecmcMainThread.h"
#include "../ethercat/ecmcEthercat.h"
#include "../main/ecmcGeneral.h"
#include "../com/ecmcCom.h"

static const char *driverName = "ecmcAsynPortDriver";


static int allowCallbackEpicsState=0;
static initHookState currentEpicsState=initHookAtIocBuild;
static ecmcAsynPortDriver *ecmcAsynPortObj;

/** Callback hook for EPICS state.
 * \param[in] state EPICS state
 * \return void
 * Will be called be the EPICS framework with the current EPICS state as it changes.
 */
static void getEpicsState(initHookState state)
{
  const char* functionName = "getEpicsState";

  if(!ecmcAsynPortObj){
    printf("%s:%s: ERROR: ecmcAsynPortObj==NULL\n", driverName, functionName);
    return;
  }

  asynUser *asynTraceUser=ecmcAsynPortObj->getTraceAsynUser();  
  switch(state) {
      break;
    //case initHookAfterScanInit:
    case initHookAfterIocRunning:
      allowCallbackEpicsState=1;
      ecmcAsynPortObj->calcFastestUpdateRate();
      /*ecmcAsynPortObj->setAllowRtThreadCom(1);      
      //make all callbacks if data arrived from callback before interrupts were registered (before allowCallbackEpicsState==1)
      if(!ecmcAsynPortObj){
        printf("%s:%s: ERROR: ecmcAsynPortObj==NULL\n", driverName, functionName);
        return;
      }
      ecmcAsynPortObj->fireAllCallbacksLock();
      updateAsynParams(1);*/      
      ecmcAsynPortObj->refreshAllInUseParamsRT();
      asynPrint(asynTraceUser, ASYN_TRACE_INFO , "%s:%s: All Parameters refreshed.\n", driverName, functionName);
      break;
    default:
      break;
  }

  currentEpicsState=state;
  asynPrint(asynTraceUser, ASYN_TRACE_INFO , "%s:%s: EPICS state: %s (%d). Allow callbacks: %s.\n", driverName, functionName,epicsStateToString((int)state),(int)state,ecmcAsynPortObj->getAllowRtThreadCom() ? "true" : "false");
}

/** Register EPICS hook function
 * \return void
 */
int initHook(void)
{
  return(initHookRegister(getEpicsState));
}

/** Constructor for the ecmcAsynPortDriver class.
  * Calls constructor for the asynPortDriver base class.
  * \param[in] portName The name of the asyn port driver to be created. */
ecmcAsynPortDriver::ecmcAsynPortDriver(
  const char *portName  /*, int maxPoints*/,
  int         paramTableSize,
  int         autoConnect,
  int         priority,
  double      defaultSampleRateMS)
  : asynPortDriver(portName,
                   1,
                   /* maxAddr */
                   paramTableSize,
                   asynInt32Mask | asynFloat64Mask | asynFloat32ArrayMask |
                   asynFloat64ArrayMask | asynEnumMask | asynDrvUserMask |
                   asynOctetMask | asynInt8ArrayMask | asynInt16ArrayMask |
                   asynInt32ArrayMask | asynUInt32DigitalMask,
                   /* Interface mask */
                   asynInt32Mask | asynFloat64Mask | asynFloat32ArrayMask |
                   asynFloat64ArrayMask | asynEnumMask | asynDrvUserMask |
                   asynOctetMask | asynInt8ArrayMask | asynInt16ArrayMask |
                   asynInt32ArrayMask | asynUInt32DigitalMask,
                   /* Interrupt mask */
                   ASYN_CANBLOCK , /*NOT ASYN_MULTI_DEVICE*/
                   autoConnect,
                   /* Autoconnect */
                   priority,
                   /* Default priority */
                   0) { /* Default stack size*/
  initVars();
  const char* functionName = "ecmcAsynPortDriver";
  allowRtThreadCom_ = 1;  // Allow at startup (RT thread not started)
  pEcmcParamInUseArray_  = new ecmcAsynDataItem*[paramTableSize];  
  pEcmcParamAvailArray_  = new ecmcAsynDataItem*[paramTableSize];
  for(int i=0; i<paramTableSize; i++) {
    pEcmcParamInUseArray_[i]=NULL;
    pEcmcParamAvailArray_[i]=NULL;
  }
  paramTableSize_   = paramTableSize;
  autoConnect_      = autoConnect;
  priority_         = priority;
  defaultSampleTimeMS_ = defaultSampleRateMS;
  fastestParamUpdateCycles_=(int32_t)(defaultSampleRateMS/1000.0*(double)MCU_FREQUENCY);
  if(paramTableSize_<1){  //If paramTableSize_==1 then only stream device or motor record can use the driver through the "default access" param below.
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Param table size to small: %d\n", driverName, functionName,paramTableSize_);
    exit(1);
    return;
  }

  // Add first param for other access (like motor record or stream device).
  ecmcAsynDataItem *paramTemp = new ecmcAsynDataItem(this,ECMC_ASYN_PAR_OCTET_NAME,asynParamNotDefined);
  if(paramTemp->createParam()){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: createParam for %s failed.\n", driverName, functionName,ECMC_ASYN_PAR_OCTET_NAME);
    delete paramTemp;
    exit(1);    
  }

  if(appendAvailParam(paramTemp,1)!=asynSuccess){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Append asyn octet param %s failed.\n", driverName, functionName,ECMC_ASYN_PAR_OCTET_NAME);
    delete paramTemp;
    exit(1);
  }  

  if(appendInUseParam(paramTemp,1)!=asynSuccess){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Append asyn octet param %s failed.\n", driverName, functionName,ECMC_ASYN_PAR_OCTET_NAME);
    delete paramTemp;
    exit(1);
  }
  
  int errorCode = ecmcInitAsyn((void *)this);
  if (errorCode) {
    asynPrint(pasynUserSelf,
              ASYN_TRACE_ERROR,
              "ecmcAsynPortDriverConfigure: ERROR: New AsynPortDriver (setAsynPort()) failed (0x%x).\n",
              errorCode);
    exit(1);
    }  
}

/** 
 * Initiate variables
 */
void ecmcAsynPortDriver::initVars() {
  allowRtThreadCom_      = 0;
  pEcmcParamInUseArray_  = NULL;
  pEcmcParamAvailArray_  = NULL;
  ecmcParamInUseCount_   = 0;
  ecmcParamAvailCount_   = 0;
  paramTableSize_        = 0;
  defaultSampleTimeMS_   = 0;
  defaultMaxDelayTimeMS_ = 0;
  defaultTimeSource_     = ECMC_TIME_BASE_ECMC;
  autoConnect_           = 0;
  priority_              = 0;
}

/** Append parameter to in-use list\n
 * All parameters that are linked to records are found in this list\n
 * 
  * \param[in] dataItem Parameter to add\n
  * \param[in] dieIfFail Exit if add fails\n 
  * 
  * returns asynError or asunSuccess
  * */
asynStatus ecmcAsynPortDriver::appendInUseParam(ecmcAsynDataItem *dataItem, bool dieIfFail){
  const char* functionName = "appendInUseParam";
  if(!dataItem){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Error: DataItem NULL.", driverName, functionName);
    if(dieIfFail){
      exit(1);
    }
    return asynError;
  }

  if(ecmcParamInUseCount_>=(paramTableSize_-1)){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Parameter table full. Parameter with name %s will be discarded.", 
              driverName, functionName,dataItem->getParamInfo()->name);
    if(dieIfFail){
      exit(1);
    }
    return asynError;
  }
  pEcmcParamInUseArray_[ecmcParamInUseCount_]=dataItem;
  ecmcParamInUseCount_++; 
  return asynSuccess;
}

/** Append parameter to list of availabe parameters
  * \param[in] dataItem Parameter to add 
  * \param[in] dieIfFail Exit if add fails. 
  * 
  * returns asynError or asunSuccess
  * */
asynStatus ecmcAsynPortDriver::appendAvailParam(ecmcAsynDataItem *dataItem, bool dieIfFail){
  const char* functionName = "appendAvailParam";
  if(!dataItem){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Error: DataItem NULL.", driverName, functionName);
    if(dieIfFail){
      exit(1);
    }
    return asynError;
  }

  if(ecmcParamAvailCount_>=(paramTableSize_-1)){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Parameter table full (available params). Parameter with name %s will be discarded.", 
              driverName, functionName,dataItem->getParamInfo()->name);
    if(dieIfFail){
      exit(1);
    }
    return asynError;
  }
  pEcmcParamAvailArray_[ecmcParamAvailCount_]=dataItem;
  ecmcParamAvailCount_++; 
  return asynSuccess;
}

/** Find parameter in list of available paremeters by name\n
  * \param[in] name Parameter name\n
  * 
  * returns parameter if found otherwise NULL\n
  * */
ecmcAsynDataItem *ecmcAsynPortDriver::findAvailParam(const char * name) {
  //const char* functionName = "findAvailParam";
  for(int i=0;i<ecmcParamAvailCount_;i++) {
    if(pEcmcParamAvailArray_[i]) {      
      if(strcmp(pEcmcParamAvailArray_[i]->getName(),name)==0) {
        return pEcmcParamAvailArray_[i];
      }
    }
  }
  return NULL;
} 

/** Create and add new parameter to list of available parameters\n
  * \param[in] name Parameter name\n
  * \param[in] type Parameter type\n
  * \param[in] data Pointer to data\n
  * \param[in] bytes size of data\n
  * \param[in] dieIfFail Exit if method fails\n
  * 
  * returns parameter (ecmcAsynDataItem)\n
  * */
ecmcAsynDataItem *ecmcAsynPortDriver::addNewAvailParam(const char * name, 
                                                       asynParamType type,
                                                       uint8_t *data,
                                                       size_t bytes,
                                                       bool dieIfFail) {

  const char* functionName = "addNewAvailParam";

  ecmcAsynDataItem *paramTemp = new ecmcAsynDataItem(this,name,type);
  
  int errorCode=paramTemp->setEcmcDataPointer(data, bytes);
  if(errorCode) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
               "%s:%s: ERROR: Set data pointer to asyn parameter %s failed.", 
               driverName, functionName, name);
    delete paramTemp;
    return NULL;
  }
  asynStatus status = appendAvailParam(paramTemp,0);
  if (status != asynSuccess) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
               "%s:%s: ERROR: Append asyn parameter %s to list failed.", 
               driverName, functionName, name);
    delete paramTemp;
    return NULL;
  }
  
  /*errorCode = paramTemp->validate();
  if(errorCode) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
               "%s:%s: ERROR: Validation of asyn parameter %s failed (0x%x).", 
               driverName, functionName, name,errorCode);
    delete paramTemp;
    return NULL;
  }*/

  return paramTemp;
}

asynStatus ecmcAsynPortDriver::readOctet(asynUser *pasynUser,
                                         char     *value,
                                         size_t    maxChars,
                                         size_t   *nActual,
                                         int      *eomReason) {
  size_t thisRead   = 0;
  int    reason     = 0;
  asynStatus status = asynSuccess;
  //int function = pasynUser->reason;

  /*
   * Feed what writeIt() gave us into the MCU
   */

  *value = '\0';

  // lock();
  if (CMDreadIt(value, maxChars)) status = asynError;

  if (status == asynSuccess) {
    thisRead = strlen(value);
    *nActual = thisRead;

    /* May be not enough space ? */

    if (thisRead > maxChars - 1) {
      reason |= ASYN_EOM_CNT;
    } else {
      reason |= ASYN_EOM_EOS;
    }

    if ((thisRead == 0) && (pasynUser->timeout == 0)) {
      status = asynTimeout;
    }
  } else {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s FAIL in ecmcAsynPortDriver::readOctet.\n", portName);
  }

  if (eomReason) *eomReason = reason;

  *nActual = thisRead;
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s thisRead=%zu data=\"%s\"\n",
            portName,
            thisRead,
            value);

  // unlock();

  return status;
}

asynStatus ecmcAsynPortDriver::writeOctet(asynUser   *pasynUser,
                                          const char *value,
                                          size_t      maxChars,
                                          size_t     *nActual) {

  //int function = pasynUser->reason;
  size_t thisWrite  = 0;
  asynStatus status = asynError;

  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s write.\n",  /*ecmcController_p->*/ portName);
  asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, value, maxChars,
              "%s write %zu\n",
              portName,
              maxChars);
  *nActual = 0;

  if (maxChars == 0) {
    return asynSuccess;
  }

  // lock();
  if (!(CMDwriteIt(value, maxChars))) {
    thisWrite = maxChars;
    *nActual  = thisWrite;
    status    = asynSuccess;
  }

  // unlock();
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s wrote %zu return %s.\n",
            portName,
            *nActual,
            pasynManager->strStatus(status));
  return status;
}

/** 
 * Ensure name and id is valid for parameter at reads or writes\n
 * \param[in] paramIndex Index of parameter.\n
 * \param[in] functionName Name of calling function.\n
 *
 * \return asynSuccess or asynError.
 */
asynStatus ecmcAsynPortDriver::checkParamNameAndId(int paramIndex, const char *functionName) {

  const char *paramName;
  /* Fetch the parameter string name for possible use in debugging */
  getParamName(paramIndex, &paramName);
  
  // Check object
  if(!pEcmcParamInUseArray_[paramIndex]) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
           "%s:%s: Error: Parameter object NULL for function %d (%s).\n",
            driverName, functionName, paramIndex, paramName);
    return asynError;
  }
 
  // Check name
  if(strcmp(paramName,pEcmcParamInUseArray_[paramIndex]->getName())!=0) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
           "%s:%s: Error: Parameter name missmatch for function %d (%s != %s).\n",
            driverName, functionName, paramIndex, paramName,pEcmcParamInUseArray_[paramIndex]->getName());
    return asynError;
  }
  
  return asynSuccess;
}

asynStatus ecmcAsynPortDriver::writeInt32(asynUser  *pasynUser,
                                          epicsInt32 value) {
  int function = pasynUser->reason;  
  const char *functionName = "writeInt32";
  
  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->writeInt32(value);
}

asynStatus ecmcAsynPortDriver::readInt32(asynUser *pasynUser,
                                         epicsInt32 *value) {                                           
  int function = pasynUser->reason;
  const char *functionName = "readInt32";

  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->readInt32(value);;
}

asynStatus ecmcAsynPortDriver::writeFloat64(asynUser *pasynUser,
                                            epicsFloat64 value) {
  int function = pasynUser->reason;
  const char *functionName = "writeFloat64";

  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->writeFloat64(value);
}

asynStatus ecmcAsynPortDriver::readFloat64(asynUser *pasynUser,
                                           epicsFloat64 *value) {
  int function = pasynUser->reason;
  const char *functionName = "readFloat64";

  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->readFloat64(value);
}


asynUser * ecmcAsynPortDriver::getTraceAsynUser() {
  return pasynUserSelf;
}

/** Write array of a certain data type to PLC.
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] allowedType Allowed ads type to read.
 * \param[out] data Data to write.
 * \param[in] nEpicsBufferBytes Bytes to write.
 *
 * \return asynSuccess or asynError.
 */
// asynStatus ecmcAsynPortDriver::writeArrayGeneric(asynUser *pasynUser,asynParamType allowedType,const void *data,size_t nEpicsBufferBytes)
// {
//   const char* functionName = "writeArrayGeneric";
//   asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

//   int function=pasynUser->reason;

//   if(!pEcmcParamInUseArray_[function] || function>=paramTableSize_){
//     asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: pAdsParamArray NULL or index (pasynUser->reason) our of range\n", driverName, functionName);
//     pasynUser->alarmStatus=WRITE_ALARM;
//     pasynUser->alarmSeverity=INVALID_ALARM;
//     return asynError;
//   }
  
//   //Only support same datatype as in PLC
//   if(!pEcmcParamInUseArray_[function]->asynTypeSupported(allowedType)){
//     asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Data types not compatible. Write canceled.\n", driverName, functionName);
//     pEcmcParamInUseArray_[function]->setAlarmParam(WRITE_ALARM,INVALID_ALARM);
//     return asynError;
//   }

//   ecmcParamInfo *paramInfo=pEcmcParamInUseArray_[function]->getParamInfo();

//   size_t bytesToWrite=nEpicsBufferBytes;
//   if(paramInfo->ecmcSize<nEpicsBufferBytes){
//     bytesToWrite=paramInfo->ecmcSize;
//   }

//   //Write 
//   /*if(pEcmcParamInUseArray_[function]->writeParam((uint8_t*)data,bytesToWrite)){
//     pEcmcParamInUseArray_[function]->setAlarmParam(WRITE_ALARM,INVALID_ALARM);
//     return asynError;
//   }*/

//   // Do callbacks to refresh params
//   int errorCode = pEcmcParamInUseArray_[function]->refreshParamRT(1);
//   if(errorCode>0) {
//     asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: Error in refreshing parameter after write (0x%x).\n", driverName, functionName,errorCode);
//     return asynError;
//   }

//   //Only reset if write alarm
//   if(pEcmcParamInUseArray_[function]->getAlarmStatus() == WRITE_ALARM){
//     return pEcmcParamInUseArray_[function]->setAlarmParam(NO_ALARM,NO_ALARM);
//   }

//   return asynSuccess;
// }

/** Overrides asynPortDriver::writeInt8Array.
 * Writes int8Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Input data buffer.
 * \param[in] nElements Input data size.
 *
 * \return asynSuccess or asynError.
 */
asynStatus ecmcAsynPortDriver::writeInt8Array(asynUser *pasynUser, 
                                              epicsInt8 *value,
                                              size_t nElements) {
  int function = pasynUser->reason;
  const char *functionName = "writeInt8Array";

  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->writeInt8Array(value,nElements);
}

asynStatus ecmcAsynPortDriver::readInt8Array(asynUser  *pasynUser,
                                             epicsInt8 *value,
                                             size_t     nElements,
                                             size_t    *nIn) {
  int function = pasynUser->reason;
  const char *functionName = "readInt8Array";

  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->readInt8Array(value,nElements,nIn);
}

/** Overrides asynPortDriver::writeInt16Array.
 * Writes int16Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Input data buffer.
 * \param[in] nElements Input data size.
 *
 * \return asynSuccess or asynError.
 */
asynStatus ecmcAsynPortDriver::writeInt16Array(asynUser *pasynUser,
                                               epicsInt16 *value,
                                               size_t nElements) {
  int function = pasynUser->reason;
  const char *functionName = "writeInt16Array";

  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->writeInt16Array(value,nElements);
}

asynStatus ecmcAsynPortDriver::readInt16Array(asynUser   *pasynUser,
                                              epicsInt16 *value,
                                              size_t      nElements,
                                              size_t     *nIn) {
  int function = pasynUser->reason;
  const char *functionName = "readInt16Array";

  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->readInt16Array(value,nElements,nIn);
}

/** Overrides asynPortDriver::writeInt32Array.
 * Writes int32Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Input data buffer.
 * \param[in] nElements Input data size.
 *
 * \return asynSuccess or asynError.
 */
asynStatus ecmcAsynPortDriver::writeInt32Array(asynUser *pasynUser,
                                               epicsInt32 *value,
                                               size_t nElements) {
  int function = pasynUser->reason;
  const char *functionName = "writeInt32Array";

  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->writeInt32Array(value,nElements);
}

asynStatus ecmcAsynPortDriver::readInt32Array(asynUser   *pasynUser,
                                              epicsInt32 *value,
                                              size_t      nElements,
                                              size_t     *nIn) {
  int function = pasynUser->reason;
  const char *functionName = "readInt32Array";

  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->readInt32Array(value,nElements,nIn);
}

/** Overrides asynPortDriver::writeFloat32Array.
 * Writes float32Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Input data buffer.
 * \param[in] nElements Input data size.
 *
 * \return asynSuccess or asynError.
 */
asynStatus ecmcAsynPortDriver::writeFloat32Array(asynUser *pasynUser,
                                                 epicsFloat32 *value,
                                                 size_t nElements) {
  int function = pasynUser->reason;
  const char *functionName = "writeFloat32Array";

  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->writeFloat32Array(value,nElements);
}

asynStatus ecmcAsynPortDriver::readFloat32Array(asynUser     *pasynUser,
                                                epicsFloat32 *value,
                                                size_t        nElements,
                                                size_t       *nIn) {
  int function = pasynUser->reason;
  const char *functionName = "readFloat32Array";

  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->readFloat32Array(value,nElements,nIn);
}

/** Overrides asynPortDriver::writeFloat64Array.
 * Writes float6432Array
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] value Input data buffer.
 * \param[in] nElements Input data size.
 *
 * \return asynSuccess or asynError.
 */
asynStatus ecmcAsynPortDriver::writeFloat64Array(asynUser *pasynUser,
                                                 epicsFloat64 *value,
                                                 size_t nElements) {
  int function = pasynUser->reason;
  const char *functionName = "writeFloat64Array";

  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->writeFloat64Array(value,nElements);
}

asynStatus ecmcAsynPortDriver::readFloat64Array(asynUser     *pasynUser,
                                                epicsFloat64 *value,
                                                size_t        nElements,
                                                size_t       *nIn) {
  int function = pasynUser->reason;
  const char *functionName = "readFloat64Array";

  if(checkParamNameAndId(function,functionName) != asynSuccess) {
    return asynError;
  }

  return pEcmcParamInUseArray_[function]->readFloat64Array(value,nElements,nIn);
}

// int ecmcAsynPortDriver::readArrayGeneric(asynUser   *pasynUser,
//                                          epicsUInt8 *value,
//                                          size_t      nElements,
//                                          size_t     *nIn,
//                                          size_t      typeSize,
//                                          const char *functionName) {
//   const char *paramName;
//   int errorId  = 0;
//   int function = pasynUser->reason;

//   getParamName(function, &paramName);

//   size_t bytesRead = 0;
//   char   buffer[1024];
//   int    masterIndex = 0;
//   int    nvals       = sscanf(paramName, "ec%d.mm.%s", &masterIndex, buffer);

//   if (nvals == 2) {
//     errorId = readEcMemMap(buffer,
//                            reinterpret_cast<uint8_t *>(value),
//                            nElements * typeSize,
//                            &bytesRead);
//   }

//   if (errorId) {
//     asynPrint(pasynUser,
//               ASYN_TRACE_ERROR,
//               "%s:%s: error, read of parameter %s failed with error code 0x%x.\n",
//               driverName,
//               functionName,
//               paramName,
//               errorId);
//     *nIn = 0;
//     return asynError;
//   }
//   *nIn = bytesRead / typeSize;
//   return errorId;
// }

void ecmcAsynPortDriver::setAllowRtThreadCom(bool allowRtCom) {
  allowRtThreadCom_ = allowRtCom;
}

bool ecmcAsynPortDriver::getAllowRtThreadCom() {
  return allowRtThreadCom_;
}

/** Overrides asynPortDriver::drvUserCreate.
 * This function is called by the asyn-framework for each record that is linked to this asyn port.
 * \param[in] pasynUser Pointer to asyn user structure
 * \param[in] drvInfo String containing information about the parameter.
 * \param[out] pptypeName
 * \param[out] psize size of pptypeName.
 * \return asynSuccess or asynError.
 * The drvInfo string is what is after the asyn() in the "INP" or "OUT"
 * field of an record.
 */
asynStatus ecmcAsynPortDriver::drvUserCreate(asynUser *pasynUser,const char *drvInfo,const char **pptypeName,size_t *psize)
{
  const char* functionName = "drvUserCreate";
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName,drvInfo);
  printf("%s:%s: drvInfo: %s\n", driverName, functionName,drvInfo);
  if(validateDrvInfo(drvInfo)!=asynSuccess){
    return asynError;
  }

  int addr=0;
  asynStatus status = getAddress(pasynUser, &addr);
  if (status != asynSuccess){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: getAddress() failed.\n",driverName, functionName);
    return(status);
  }

  // Collect data from drvInfo string and recordpasynUser->reason=index;
  ecmcParamInfo *newParamInfo=new ecmcParamInfo();
  memset(newParamInfo,0,sizeof(ecmcParamInfo));
  newParamInfo->sampleTimeMS=defaultSampleTimeMS_;      
  newParamInfo->asynAddr=addr;

  // Parse options and name
  status=parseInfofromDrvInfo(drvInfo,newParamInfo);
  if(status!=asynSuccess){
    return asynError;
  }

  status=getRecordInfoFromDrvInfo(drvInfo, newParamInfo);
  if(status!=asynSuccess){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Failed to find record with drvInfo %s.\n", driverName, functionName,drvInfo);
    return asynError;
  }

  int index=0;
  status=findParam(newParamInfo->name,&index);  
  
  if(status!=asynSuccess) {
    
    // Param not found see if found in available list
    ecmcAsynDataItem * param = findAvailParam(newParamInfo->name);
    if(!param) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Parameter %s not found (drvInfo=%s).\n",
                driverName, functionName,newParamInfo->name,drvInfo);
      return asynError;
    }

    //Ensure that type is supported
    if( !param->asynTypeSupported(newParamInfo->asynType)) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Error: asynType %s not suppotrted for parameter %s. Supported types are:\n", 
               driverName, functionName,newParamInfo->asynTypeStr,param->getName());
      for(int i=0;i < param->getSupportedAsynTypeCount();i++) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: AsynType: %s (%d)\n",
                driverName, functionName,
                asynTypeToString((long)param->getSupportedAsynType(i)),
                param->getSupportedAsynType(i));
      }
      return asynError;      
    }
    //Type supported so use it.
    param->setAsynParameterType(newParamInfo->asynType);

    // Add parameter to In use list
    status = appendInUseParam(param,0);
    if(status!=asynSuccess) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Append parameter %s to in-use list failed.\n",
                driverName, functionName,newParamInfo->name);
      return asynError;
    }
    
    // Create asyn param
    int errorCode=param->createParam();
    if(errorCode) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Create parameter %s failed (0x%x).\n",
                driverName, functionName,newParamInfo->name,errorCode);
      return asynError;
    }

    // Ensure that parameter index is correct
    if(param->getAsynParameterIndex() != (ecmcParamInUseCount_-1)) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Parameter index missmatch for  %s  (%d != %d).\n",
                driverName, functionName,newParamInfo->name,param->getAsynParameterIndex(),ecmcParamInUseCount_-1);
      return asynError;
    }

    //Update index
    index = param->getAsynParameterIndex();

    asynPrint(pasynUser, ASYN_TRACE_INFO, "%s:%s: Parameter %s added to in-use list at index %d.\n",
            driverName, functionName,newParamInfo->name,index);

    //Now we have linked a parameter from available list into the inUse list successfully
  }

  //Asyn parameter found. Update with new info from record defs
  asynPrint(pasynUser, ASYN_TRACE_INFO, "%s:%s: Parameter index found at: %d for %s.\n",
            driverName, functionName,index,newParamInfo->name);
  if(!pEcmcParamInUseArray_[index]) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s:pAdsParamArray_[%d]==NULL (drvInfo=%s).\n",
              driverName, functionName,index,drvInfo);
    return asynError;
  }

  ecmcParamInfo *existentParInfo = pEcmcParamInUseArray_[index]->getParamInfo();
 
  if(!existentParInfo->initialized) {
    pEcmcParamInUseArray_[index]->setAsynParSampleTimeMS(newParamInfo->sampleTimeMS);    
    existentParInfo->recordName=strdup(newParamInfo->recordName);
    existentParInfo->recordType=strdup(newParamInfo->recordType);
    existentParInfo->dtyp=strdup(newParamInfo->dtyp);
    existentParInfo->drvInfo=strdup(newParamInfo->drvInfo);
  }

  // Ensure that sample time is the shortest (if several records 
  // on the same parameter)
  if(newParamInfo->sampleTimeMS <  existentParInfo->sampleTimeMS ) {
    pEcmcParamInUseArray_[index]->setAsynParSampleTimeMS(newParamInfo->sampleTimeMS);      
  }

  if(pasynUser->timeout < newParamInfo->sampleTimeMS*2/1000.0) {
    pasynUser->timeout = (newParamInfo->sampleTimeMS*2)/1000;
  }
  
  pEcmcParamInUseArray_[index]->refreshParam(1);
  callParamCallbacks();

  existentParInfo->initialized=1;

  return asynPortDriver::drvUserCreate(pasynUser,existentParInfo->name,pptypeName,psize);
}

/** Validates drvInfo string
 * \param[in] drvInfo String containing information about the parameter.
 * \return asynSuccess or asynError.
 * The drvInfo string is what is after the asyn() in the "INP" or "OUT"
 * field of an record.
 */
asynStatus ecmcAsynPortDriver::validateDrvInfo(const char *drvInfo)
{
  const char* functionName = "validateDrvInfo";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName,drvInfo);

  if(strlen(drvInfo)==0){
    asynPrint(pasynUserSelf,ASYN_TRACE_ERROR,"Invalid drvInfo string: Length 0 (%s).\n",drvInfo);
    return asynError;
  }

  //Check '?' mark last or '=' last
  const char* read=strrchr(drvInfo,'?');
  if(read){
    if(strlen(read)==1){
      return asynSuccess;
    }
  }

  const char* write=strrchr(drvInfo,'=');
  if(write){
    if(strlen(write)==1){
      return asynSuccess;
    }
  }

  asynPrint(pasynUserSelf,ASYN_TRACE_ERROR,"Invalid drvInfo string (%s).\n", drvInfo);
  return asynError;
}

/** Get asyn type from record.
 * \param[in] drvInfo String containing information about the parameter.
 * \param[in/out] paramInfo Parameter information structure.
 * \return asynSuccess or asynError.
 */
asynStatus ecmcAsynPortDriver::getRecordInfoFromDrvInfo(const char *drvInfo, ecmcParamInfo *paramInfo)
{
  const char* functionName = "getRecordInfoFromDrvInfo";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName, drvInfo);

  bool isInput=false;
  bool isOutput=false;
  DBENTRY *pdbentry;
  pdbentry = dbAllocEntry(pdbbase);
  long status = dbFirstRecordType(pdbentry);
  bool recordFound=false;
  if(status || !paramInfo) {
    dbFreeEntry(pdbentry);
    return asynError;
  }

  while(!status) {
    paramInfo->recordType=strdup(dbGetRecordTypeName(pdbentry));
    status = dbFirstRecord(pdbentry);
    while(!status) {
      paramInfo->recordName=strdup(dbGetRecordName(pdbentry));
      if(!dbIsAlias(pdbentry)){
        status=dbFindField(pdbentry,"INP");
        if(!status){
          paramInfo->inp=strdup(dbGetString(pdbentry));

          isInput=true;
          char port[ECMC_MAX_FIELD_CHAR_LENGTH];
          int adr;
          int timeout;
          char currdrvInfo[ECMC_MAX_FIELD_CHAR_LENGTH];
          int nvals=sscanf(paramInfo->inp,ECMC_ASYN_INP_FORMAT,port,&adr,&timeout,currdrvInfo);
          if(nvals==4){
            // Ensure correct port and drvinfo
            if(strcmp(port,portName)==0 && strcmp(drvInfo,currdrvInfo)==0){
              recordFound=true;  // Correct port and drvinfo!\n");
            }
          } else {
            int mask=0;
            nvals=sscanf(paramInfo->inp,ECMC_ASYN_MASK_INP_FORMAT,port,&adr,&mask,&timeout,currdrvInfo);
            if(nvals==5){
              // Ensure correct port and drvinfo
              if(strcmp(port,portName)==0 && strcmp(drvInfo,currdrvInfo)==0){
                recordFound=true;  // Correct port and drvinfo!\n");
              }
            }
          }
        }
        else{
          isInput=false;
        }
        status=dbFindField(pdbentry,"OUT");
        if(!status){
          paramInfo->out=strdup(dbGetString(pdbentry));
          isOutput=true;
          char port[ECMC_MAX_FIELD_CHAR_LENGTH];
          int adr;
          int timeout;
          char currdrvInfo[ECMC_MAX_FIELD_CHAR_LENGTH];
          int nvals=sscanf(paramInfo->out,ECMC_ASYN_INP_FORMAT,port,&adr,&timeout,currdrvInfo);
          if(nvals==4){
            // Ensure correct port and drvinfo
            if(strcmp(port,portName)==0 && strcmp(drvInfo,currdrvInfo)==0){
              recordFound=true;  // Correct port and drvinfo!\n");
            }
          } else {
            int mask=0;
            nvals=sscanf(paramInfo->out,ECMC_ASYN_MASK_INP_FORMAT,port,&adr,&mask,&timeout,currdrvInfo);
            if(nvals==5){
              // Ensure correct port and drvinfo
              if(strcmp(port,portName)==0 && strcmp(drvInfo,currdrvInfo)==0){
                recordFound=true;  // Correct port and drvinfo!\n");
              }
            }
          }
        }
        else{
          isOutput=false;
        }

        if(recordFound){
          // Correct record found. Collect data from fields
          //DTYP
          status=dbFindField(pdbentry,"DTYP");
          if(!status){
            paramInfo->dtyp=strdup(dbGetString(pdbentry));
            paramInfo->asynType=stringToAsynType(dbGetString(pdbentry));
          }
          else{
            paramInfo->dtyp=0;
            paramInfo->asynType=asynParamNotDefined;
          }

          //drvInput (not a field)
          paramInfo->drvInfo=strdup(drvInfo);
          dbFreeEntry(pdbentry);
          // The correct record was found and the paramInfo structure is filled
          return asynSuccess;  
        }
        else{
          //Not correct record. Do cleanup.
          if(isInput){
            free(paramInfo->inp);
            paramInfo->inp=0;
          }
          if(isOutput){
            free(paramInfo->out);
            paramInfo->out=0;
          }
          paramInfo->drvInfo=0;
          paramInfo->scan=0;
          paramInfo->dtyp=0;
          isInput=false;
          isOutput=false;
        }
      }
      status = dbNextRecord(pdbentry);
      free(paramInfo->recordName);
      paramInfo->recordName=0;
      recordFound = false;
    }
    status = dbNextRecordType(pdbentry);
    free(paramInfo->recordType);
    paramInfo->recordType=0;
  }
  dbFreeEntry(pdbentry);  

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
asynStatus ecmcAsynPortDriver::parseInfofromDrvInfo(const char* drvInfo,ecmcParamInfo *paramInfo)
{
  const char* functionName = "parseInfofromDrvInfo";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,
            "%s:%s: drvInfo: %s\n",
            driverName,
            functionName,
            drvInfo);

  //Check if input or output
  paramInfo->isIOIntr=false;
  const char* temp=strrchr(drvInfo,'?');
  if(temp){
    if(strlen(temp)==1){
      paramInfo->isIOIntr=true; //All inputs will be created I/O intr
    }
  }

  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: drvInfo %s is %s\n",
            driverName,
            functionName,
            drvInfo,
            paramInfo->isIOIntr ? "I/O Intr (end with ?)": " not I/O Intr (end with =)");

  //take part after last "/" if option or complete string..
  char buffer[ECMC_MAX_FIELD_CHAR_LENGTH];
  //See if option (find last '/')
  const char *drvInfoEnd=strrchr(drvInfo,'/');
  if(drvInfoEnd){ // found '/'
    int nvals=sscanf(drvInfoEnd,"/%s",buffer);
    if(nvals==1){
      paramInfo->name=strdup(buffer);
      paramInfo->name[strlen(paramInfo->name)-1]=0; //Strip ? or = from end
    }
    else{
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Failed to parse PLC address string from drvInfo (%s)\n",
                driverName,
                functionName,
                drvInfo);
      return asynError;
    }
  }
  else{  //No options
    paramInfo->name=strdup(drvInfo);
    paramInfo->name[strlen(paramInfo->name)-1]=0; //Strip ? or = from end
  }

  //Check if ECMC_OPTION_T_SAMPLE_RATE_MS option
  const char *option=ECMC_OPTION_T_SAMPLE_RATE_MS;
  paramInfo->sampleTimeMS=defaultSampleTimeMS_;
  const char *isThere=strstr(drvInfo,option);
  if(isThere){
    if(strlen(isThere)<(strlen(option)+strlen("=0/"))){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n",
                driverName,
                functionName,
                option,
                drvInfo);
      return asynError;
    }

    int nvals = sscanf(isThere+strlen(option),"=%lf/",&paramInfo->sampleTimeMS);
    if(nvals==1) {
      paramInfo->sampleTimeCycles=(int32_t)paramInfo->sampleTimeMS/1000.0*(double)MCU_FREQUENCY;
    } 
    else {
      paramInfo->sampleTimeMS=defaultSampleTimeMS_;
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
                driverName,
                functionName,
                option,
                drvInfo);
      return asynError;
    }
  }

  //Check if ECMC_OPTION_TIMEBASE option
  option=ECMC_OPTION_TIMEBASE;
  paramInfo->timeBase=defaultTimeSource_;
  isThere=strstr(drvInfo,option);
  if(isThere){
    int minLen=strlen(ECMC_OPTION_TIMEBASE_ECMC);
    int epicsLen=strlen(ECMC_OPTION_TIMEBASE_EPICS);
    if(epicsLen<minLen){
      minLen=epicsLen;
    }
    if(strlen(isThere)<(strlen(option)+strlen("=/")+minLen)){ //Allowed "ECMC" or "EPICS"
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n",
                driverName,
                functionName,
                option,
                drvInfo);
      return asynError;
    }

    int nvals = sscanf(isThere+strlen(option),"=%[^/]/",buffer);
    if(nvals!=1){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
                driverName,
                functionName,
                option,
                drvInfo);
      return asynError;
    }

    if(strcmp(ECMC_OPTION_TIMEBASE_ECMC,buffer)==0){
      paramInfo->timeBase=ECMC_TIME_BASE_ECMC;
    }

    if(strcmp(ECMC_OPTION_TIMEBASE_EPICS,buffer)==0){
      paramInfo->timeBase=ECMC_TIME_BASE_EPICS;
    }
  }
  
  //Check if TYPE option
  option=ECMC_OPTION_TYPE;
  paramInfo->asynTypeStr=NULL;
  paramInfo->asynType=asynParamNotDefined;

  isThere=strstr(drvInfo,option);
  if(isThere){
    if(strlen(isThere)<(strlen(option)+strlen("=0/"))){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n",
                driverName,
                functionName,
                option,
                drvInfo);
      return asynError;
    }
    int nvals;
    nvals = sscanf(isThere+strlen(option),"=%[^/]",buffer);
     if(nvals!=1){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
                driverName,
                functionName,
                option,
                drvInfo);
      return asynError;
    }
    paramInfo->asynTypeStr=strdup(buffer);
    paramInfo->asynType=stringToAsynType(paramInfo->asynTypeStr);
  }

  return asynSuccess;
}

int32_t ecmcAsynPortDriver::getFastestUpdateRate() { 
   return fastestParamUpdateCycles_;
}

int32_t ecmcAsynPortDriver::calcFastestUpdateRate() {

  fastestParamUpdateCycles_=(int32_t)(defaultSampleTimeMS_/1000.0*(double)MCU_FREQUENCY);
  for(int i=0;i<ecmcParamInUseCount_;i++) {
    if(pEcmcParamInUseArray_[i]) {
      if(!pEcmcParamInUseArray_[i]->initialized()) {        
        continue;
      }
      if(pEcmcParamInUseArray_[i]->getSampleTimeCycles()<fastestParamUpdateCycles_) {
        fastestParamUpdateCycles_ = pEcmcParamInUseArray_[i]->getSampleTimeCycles();
      } 
    }
  }
  return fastestParamUpdateCycles_;
}
void ecmcAsynPortDriver::refreshAllInUseParamsRT() {

  for(int i=0;i<ecmcParamInUseCount_;i++) {
    if(pEcmcParamInUseArray_[i]) {
      if(!pEcmcParamInUseArray_[i]->initialized()) {        
        continue;
      }
      pEcmcParamInUseArray_[i]->refreshParamRT(1);
    }
  }
}

void ecmcAsynPortDriver::reportParamInfo(FILE *fp, ecmcAsynDataItem *param,int listIndex) {

  if(!param) {
    return;
  }
  ecmcParamInfo *paramInfo=param->getParamInfo();
  if(!paramInfo) {
    return;
  }

  fprintf(fp,"  Parameter %d:\n",listIndex);
  fprintf(fp,"    Param name:                %s\n",paramInfo->name);
  fprintf(fp,"    Param index:               %d\n",paramInfo->index);
  fprintf(fp,"    Param type:                %s (%d)\n",asynTypeToString((long)paramInfo->asynType),paramInfo->asynType);
  //supported types
  fprintf(fp,"    Supported asyn types:\n");
  for(int i=0;i<param->getSupportedAsynTypeCount();i++) {
    fprintf(fp,"      - %s (%d)\n",asynTypeToString(
                            (long)param->getSupportedAsynType(i)),
                            param->getSupportedAsynType(i));
  }
  fprintf(fp,"    Param linked to record:    %s\n",paramInfo->initialized ? "true" : "false");
  if(!paramInfo->initialized) {  //No record linked to record (no more valid data)
    fprintf(fp,"    ECMC data pointer valid:   %s\n",paramInfo->ecmcDataPointerValid ? "true" : "false"); 
    fprintf(fp,"    ECMC size [bytes]:         %lu\n",paramInfo->ecmcSize); 
    fprintf(fp,"    ECMC data is array:        %s\n",paramInfo->ecmcDataIsArray ? "true" : "false");      
    fprintf(fp,"    ECMC write allowed:        %s\n",param->writeToEcmcAllowed() ? "true" : "false");      
    fprintf(fp,"\n");
    return;
  }
  fprintf(fp,"    Param drvInfo:             %s\n",paramInfo->drvInfo);
  fprintf(fp,"    Param sample time [ms]:    %lf\n",paramInfo->sampleTimeMS);
  fprintf(fp,"    Param sample cycles []:    %d\n",paramInfo->sampleTimeCycles);
  //fprintf(fp,"    Param max delay time [ms]: %lf\n",paramInfo->maxDelayTimeMS);
  fprintf(fp,"    Param isIOIntr:            %s\n",paramInfo->isIOIntr ? "true" : "false");
  fprintf(fp,"    Param asyn addr:           %d\n",paramInfo->asynAddr);
  fprintf(fp,"    Param time source:         %s\n",
                           (paramInfo->timeBase == ECMC_TIME_BASE_ECMC) ?
                           ECMC_OPTION_TIMEBASE_ECMC : ECMC_OPTION_TIMEBASE_EPICS);
  fprintf(fp,"    Param epics time:          %us:%uns\n",
                           paramInfo->epicsTimestamp.secPastEpoch,
                           paramInfo->epicsTimestamp.nsec);
  //fprintf(fp,"    Param array buffer size:   %lu\n",paramInfo->arrayDataBufferSize);
  fprintf(fp,"    Param alarm:               %d\n",paramInfo->alarmStatus);
  fprintf(fp,"    Param severity:            %d\n",paramInfo->alarmSeverity);
  fprintf(fp,"    ECMC data pointer valid:   %s\n",paramInfo->ecmcDataPointerValid ? "true" : "false");
  fprintf(fp,"    ECMC size [bytes]:         %lu\n",paramInfo->ecmcSize);
  fprintf(fp,"    ECMC data is array:        %s\n",paramInfo->ecmcDataIsArray ? "true" : "false");
  fprintf(fp,"    ECMC write allowed:        %s\n",param->writeToEcmcAllowed() ? "true" : "false");
  // Value range only applicable for ints
  if(param->getEcmcMinValueInt() != param->getEcmcMaxValueInt()) {
    fprintf(fp,"    ECMC Value Range:          %ld..%ld, %ld bit(s)\n",
            param->getEcmcMinValueInt(),
            param->getEcmcMaxValueInt(),
            param->getEcmcBitCount());    
  }
  fprintf(fp,"    Record name:               %s\n",paramInfo->recordName);
  fprintf(fp,"    Record type:               %s\n",paramInfo->recordType);
  fprintf(fp,"    Record dtyp:               %s\n",paramInfo->dtyp);      
  fprintf(fp,"\n");
}

/** Report of configured parameters.
 * \param[in] fp Output file.
 * \param[in] details Details of printout. A higher number results in more
 *            details.
 * \return void
 * Check ads state of all connected ams ports and reconnects if needed.
 */
void ecmcAsynPortDriver::report(FILE *fp, int details)
{
  const char* functionName = "report";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  if(!fp){
    fprintf(fp,"%s:%s: ERROR: File NULL.\n", driverName, functionName);
    return;
  }

  if (details >= 1) {
    fprintf(fp,"####################################################################:\n");
    fprintf(fp, "General information:\n");
    fprintf(fp, "  Port:                         %s\n",portName);
    fprintf(fp, "  Auto-connect:                 %s\n",autoConnect_ ? "true" : "false");
    fprintf(fp, "  Priority:                     %d\n",priority_);
    fprintf(fp, "  Param. table size:            %d\n",paramTableSize_);
    fprintf(fp, "  Param. count:                 %d\n",ecmcParamInUseCount_);
    fprintf(fp, "  Default sample time [ms]:     %d\n",defaultSampleTimeMS_);
    fprintf(fp, "  Fastest update rate [cycles]: %d\n",fastestParamUpdateCycles_);
    //fprintf(fp, "  Default max delay time [ms]: %d\n",defaultMaxDelayTimeMS_);
    fprintf(fp, "  Default time source:          %s\n",(defaultTimeSource_==ECMC_TIME_BASE_ECMC) ? ECMC_OPTION_TIMEBASE_ECMC : ECMC_OPTION_TIMEBASE_EPICS);
    fprintf(fp,"\n");
  }

  if( details >= 2){
    //print all parameters in use
    fprintf(fp,"####################################################################:\n");
    fprintf(fp,"Parameters in use:\n");
    for(int i=0; i<ecmcParamInUseCount_;i++){
      if(!pEcmcParamInUseArray_[i]){
        fprintf(fp,"%s:%s: ERROR: Parameter array null at index %d\n", driverName, functionName,i);
        return;
      }      
      reportParamInfo(fp, pEcmcParamInUseArray_[i],i);
    }
  }

  if( details >= 3){
    //print all available parameters
    fprintf(fp,"####################################################################:\n");
    fprintf(fp,"Available parameters:\n");
    for(int i=0; i<ecmcParamAvailCount_;i++){
      if(!pEcmcParamAvailArray_[i]){
        fprintf(fp,"%s:%s: ERROR: Parameter array null at index %d\n", driverName, functionName,i);
        return;
      }      
      reportParamInfo(fp, pEcmcParamAvailArray_[i],i);
    }
  }
  fprintf(fp,"####################################################################:\n");
}

/* Configuration routine.  Called directly, or from the iocsh function below */

extern "C" {
static int maxParameters;
static int parameterCounter;

/* global asynUser for Printing */
asynUser *pPrintOutAsynUser;


/** EPICS iocsh callable function to call constructor for the ecmcAsynPortDriver class.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] paramTableSize The max number of parameters.
  * \param[in] priority Priority.
  * \param[in] disableAutoConnect Disable auto connect 
  * \param[in] default parameter update rate in milli seconds (for I/O intr records)*/
int ecmcAsynPortDriverConfigure(const char *portName,
                                int         paramTableSize,
                                int         priority,
                                int         disableAutoConnect,
                                double      defaultSampleRateMS) {
  if(portName == NULL) {
    printf("Error: portName missing.\n");
    return asynError;  
  }

  parameterCounter = 0;
  maxParameters    = paramTableSize;
  ecmcAsynPortObj  = new ecmcAsynPortDriver(portName,
                                            paramTableSize,
                                            disableAutoConnect == 0,
                                            priority,
                                            defaultSampleRateMS);
  initHook();

  printf ("ecmcAsynPortDriverConfigure: portName = %s, paramTableSize = %d, disableAutoConnect = %d, priority = %d, defaultSampleRateMS = %lf\n",
           portName,paramTableSize,disableAutoConnect,priority,defaultSampleRateMS);
             
  if (ecmcAsynPortObj) {
    asynUser *traceUser = ecmcAsynPortObj->getTraceAsynUser();

    if (!traceUser) {
      printf(
        "ecmcAsynPortDriverConfigure: ERROR: Failed to retrieve asynUser for trace. \n");
      return asynError;
    }

    pPrintOutAsynUser = pasynManager->duplicateAsynUser(traceUser, 0, 0);

    if (!pPrintOutAsynUser) {
      printf(
        "ecmcAsynPortDriverConfigure: ERROR: Failed to duplicate asynUser for trace. \n");
      return asynError;
    }

    /*int errorCode = initEcmcAsyn((void *)ecmcAsynPortObj);

    if (errorCode) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_ERROR,
                "ecmcAsynPortDriverConfigure: ERROR: New AsynPortDriver (setAsynPort()) failed (0x%x).\n",
                errorCode);
      return asynError;
    }*/

    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "ecmcAsynPortDriverConfigure: INFO: New AsynPortDriver success (%s,%i,%i,%i).",
              portName,
              paramTableSize,
              disableAutoConnect == 0,
              priority);

    /* Moved to constructor
    // Add one "generic" parameter for Motor-record
    int comParamIndex = 0;
    asynStatus status = ecmcAsynPortObj->createParam("default_com",
                                                     asynParamOctet,
                                                     &comParamIndex);

    if (status != asynSuccess) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_ERROR,
                "ecmcAsynPortDriverConfigure: ERROR: Create default communication parameter failed.\n");
      return asynError;
    }
    */
    return asynSuccess;
  } else {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_ERROR,
              "ecmcAsynPortDriverConfigure: ERROR: New AsynPortDriver failed.");
    return asynError;
  }
}

// Parse asyn datatype

/*static int parseAsynDataType(const char *asynTypeString) {
  
  // "asynInt8ArrayIn"
  // "asynInt8ArrayOut"
  // "asynInt16ArrayIn"
  // "asynInt16ArrayOut"
  // "asynInt32ArrayIn"
  // "asynInt32ArrayOut"
  // "asynFloat32ArrayIn"
  // "asynFloat32ArrayOut"
  // "asynFloat64ArrayIn"
  // "asynFloat64ArrayOut"
  // "asynParamInt8Array"
  // "asynParamInt16Array"
  // "asynParamInt32Array"
  // "asynParamFloat32Array"
  // "asynParamFloat64Array"
  

  int asynType = -10;
  int res      = strcmp(asynTypeString, "asynInt32");

  if (res == 0) {
    asynType = asynParamInt32;
  }

  res = strcmp(asynTypeString, "asynFloat64");

  if (res == 0) {
    asynType = asynParamFloat64;
  }

  res = strcmp(asynTypeString, "asynInt8ArrayIn");

  if (res == 0) {
    asynType = asynParamInt8Array;
  }

  res = strcmp(asynTypeString, "asynInt16ArrayIn");

  if (res == 0) {
    asynType = asynParamInt16Array;
  }

  res = strcmp(asynTypeString, "asynInt32ArrayIn");

  if (res == 0) {
    asynType = asynParamInt32Array;
  }

  res = strcmp(asynTypeString, "asynFloat32ArrayIn");

  if (res == 0) {
    asynType = asynParamFloat32Array;
  }

  res = strcmp(asynTypeString, "asynFloat64ArrayIn");

  if (res == 0) {
    asynType = asynParamFloat64Array;
  }

  res = strcmp(asynTypeString, "asynUInt32Digital");

  if (res == 0) {
    asynType = asynParamUInt32Digital;
  }

  return asynType;
}*/

/* EPICS iocsh shell command: ecmcAsynPortDriverConfigure*/

static const iocshArg initArg0 = { "port name", iocshArgString };
static const iocshArg initArg1 = { "parameter table size", iocshArgInt };
static const iocshArg initArg2 = { "priority", iocshArgInt };
static const iocshArg initArg3 = { "disable auto connect", iocshArgInt };
static const iocshArg initArg4 = { "default param sample rate (ms)", iocshArgDouble};

static const iocshArg *const initArgs[] = { &initArg0,
                                            &initArg1,
                                            &initArg2,
                                            &initArg3, 
                                            &initArg4};
static const iocshFuncDef    initFuncDef =
{ "ecmcAsynPortDriverConfigure", 5, initArgs };
static void initCallFunc(const iocshArgBuf *args) {
  ecmcAsynPortDriverConfigure(args[0].sval,
                              args[1].ival,
                              args[2].ival,
                              args[3].ival,
                              args[4].dval);
}

/** \breif EPICS iocsh command for adding asyn-parameter(s)
 * linked to objects in ECMC.\n
 *
 * Fast access of different types of objects in ECMC via asyn parameters is
 * possible:
 * 1. EtherCAT entries\n
 * 2. EtherCAT memory maps\n
 * 3. EtherCAT master diagnostics\n
 * 4. EtherCAT slave diagnostics\n
 * 5. Timing diagnostics\n
 * 6. Motion axis information\n
 * 7. Motion axis diagnostic array\n
 *
 * For example it's possible to access EtherCAT data directly from EPICS records
 * by linking an EtherCAT memory map or entry to an asyn parameter.
 * Update frequency of the asyn parameter can be changed with the "skipCycles"
 * parameter. Maximum update frequency is the same frequency as the EtherCAT
 * realtime bus (skipCycles=0).\n
 * This function can be called from within the iocsh context:
 * "ecmcAsynPortDriverAddParameter()".\n
 *
 *  \param[in] portName Name of asyn port (created with iocsh command:
 *  "ecmcAsynPortDriverConfigure()").\n
 *  \param[in] idString String for defining which parameter(s) to add (and link):\n
 *             idString = ec<masterindex>.mm.<memory map id>  (access to
 *             memory map).\n
 *             idString = ec<masterindex>.default  (set of ec diag params).\n
 *             idString = ec<masterindex>.s<slaveIndex>.default
 *             (set of ec slave diag params).\n
 *             idString = ec<masterindex>.s<busposition>.<ethercat entry id>
 *             (access to ethercat entry, "default" cannot be asynInt8ArrayInused as
 *             <ethercat entry id> ).\n
 *             idString = thread.default  (set of timing diag params).\n
 *             idString = ax<axis index>.default  (setpoint and actual value).\n
 *             idString = ax<axis index>.diagnostic (make diagnostic "string"
 *              available).\n
 *
 *  \param[in] asynParType Data type to be transfered (valid only for
 *             ec<masterindex>.mm.<memory map id>) and
 *             ec<masterindex>.s<busposition>.<ethercat entry id>.
 *             For other types of idString, use asynInt32).
 *             asynParType="asynInt32": 32 bit int (ethercat entry)\n
 *             asynParType="asynFloat64": 64 bit float (ethercat entry)\n
 *             asynParType="asynInt8ArrayIn": array of 8 bit int input (memory map)\n
 *             asynParType="asynInt16ArrayIn": array of 16 bit int input (memory map)\n
 *             asynParType="asynInt32ArrayIn": array of 32 bit int input (memory map)\n
 *             asynParType="asynFloat32ArrayIn": array of 32 bit float input (memory map)\n
 *             asynParType="asynFloat64ArrayIn": array of 64 bit float input (memory map)\n
 *  \param[in] skipCycles Number of realtime loops in between updates of asyn-
 *  parameter.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Link  asyn parameter to a memory map called "CH1_ARRAY",
 * skip cycles=0 (update at realtime loop freq):
 * ecmcAsynPortDriverAddParameter(ASYNPORT,ec0.mm.CH1_ARRAY,"asynInt16ArrayIn",0)
 *
 * \note Example: Link  asyn parameter to EtherCAT entry called "AI_1" on
 * slave 5, skip cycles =9 (skip 9 cycles then update=> every 10 value will
 * trigger asyn update):
 * ecmcAsynPortDriverAddParameter(ASYNPORT,ec0.s5.AI_1,"asynInt32",9)
 *
 * \note Example: Generate general diag asyn parameters
 *  skip cycles =9 (skip 9 cycles then update=> every 10 value will
 * trigger asyn update):
 * ecmcAsynPortDriverAddParameter(ASYNPORT,ecmc.default,"asynInt32",9)
 *
 * \note Example: Generate diag asyn parameters for ec master 0
 *  skip cycles =9 (skip 9 cycles then update=> every 10 value will
 * trigger asyn update):
 * ecmcAsynPortDriverAddParameter(ASYNPORT,ec0.default,"asynInt32",9)
 *
 * \note Example: Generate diag asyn parameters for ec master 0 slave 7
 *  skip cycles =9 (skip 9 cycles then update=> every 10 value will
 * trigger asyn update):
 * ecmcAsynPortDriverAddParameter(ASYNPORT,ec0.s7.default,"asynInt32",9)
 *
 * \note Example: Generate asyn parameters for motion axis 8
 *  skip cycles =9 (skip 9 cycles then update=> every 10 value will
 * trigger asyn update):
 * ecmcAsynPortDriverAddParameter(ASYNPORT,ax8.default,"asynInt32",9)
 *
 * \note Example: Generate asyn parameter for axis diagnostic array
 * containing most importatnt diag info.\n
 *  skip cycles =9 (skip 9 cycles then update=> every 10 value will
 * trigger asyn update):
 * ecmcAsynPortDriverAddParameter(ASYNPORT,ax8.diagnostic,"asynInt8ArrayIn",9)
 *
 */
int ecmcAsynPortDriverAddParameter(const char *portName,
                                   const char *idString,
                                   const char *asynTypeString,
                                   int         skipCycles) {

  printf("Error: ecmcAsynPortDriverAddParameter is an obsolete command.\n");
  return 0;

  /*if(portName == NULL) {
    printf("Error: portName missing.\n");
    return asynError;  
  }

  if(idString == NULL) {
    printf("Error: idString missing.\n");
    return asynError;  
  }

  if(asynTypeString == NULL) {
    printf("Error: asynTypeString missing.\n");
    return asynError;  
  }

  if (!ecmcAsynPortObj) {
    printf(
      "ecmcAsynPortDriverAddParameter: ERROR: asynPortDriver object NULL (ecmcAsynPortObj==NULL).\n");
    return asynError;
  }

  if (!pPrintOutAsynUser) {
    printf(
      "ecmcAsynPortDriverAddParameter: ERROR: asynUser trace object NULL (pPrintOutAsynUser==NULL).\n");
    return asynError;
  }

  if (0 != strcmp(ecmcAsynPortObj->portName, portName)) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_ERROR,
              "ecmcAsynPortDriverAddParameter: ERROR: Port name mismatch. Desired port: %s not accessible. Accessible port: %s.\n",
              portName,
              ecmcAsynPortObj->portName);
    return asynError;
  }

  if (parameterCounter >= maxParameters) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_ERROR,
              "ecmcAsynPortDriverAddParameter: ERROR: asynPortDriverObject full (max allowed number of parameters = %i).\n",
              maxParameters);
    return asynError;
  }

  int errorCode = 0;

  // Check if EtherCAT memorymap
  char buffer[1024];
  int  masterIndex = 0;
  int  nvals       = sscanf(idString, "ec%d.mm.%s", &masterIndex, buffer);

  if (nvals == 2) {
    int asynType = parseAsynDataType(asynTypeString);

    if (asynType <= 0) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_ERROR,
                "ecmcAsynPortDriverAddParameter: ERROR: Parameter type not supported (use asynParamInt32 or asynParamFloat64).\n");
      return asynError;
    }

    errorCode = linkEcMemMapToAsynParameter(masterIndex,
                                            idString,
                                            asynType,
                                            skipCycles);

    if (errorCode) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_ERROR,
                "ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",
                idString,
                errorCode);
      return asynError;
    }

    parameterCounter++;
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",
              idString);
    return asynSuccess;
  }

  // Check if default parameters for ec slave
  int busPosition = -10;
  nvals = sscanf(idString, "ec%d.s%d.%s", &masterIndex, &busPosition, buffer);

  if ((nvals == 3) && (strcmp("default", buffer) == 0)) {
    errorCode = addDefaultAsynEcSlave(masterIndex, busPosition, 1, skipCycles);

    if (errorCode) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_ERROR,
                "ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",
                idString,
                errorCode);
      return asynError;
    }
    parameterCounter++;
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",
              idString);
    return asynSuccess;
  }

  // Check if EtherCAT EtherCAT entry
  busPosition = -10;
  nvals       = sscanf(idString,
                       "ec%d.s%d.%s",
                       &masterIndex,
                       &busPosition,
                       buffer);

  if (nvals == 3) {
    int asynType = parseAsynDataType(asynTypeString);

    if (asynType <= 0) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_ERROR,
                "ecmcAsynPortDriverAddParameter: ERROR: Parameter type not supported (use asynParamInt32 or asynParamFloat64).\n");
      return asynError;
    }
    errorCode = linkEcEntryToAsynParameter(masterIndex,
                                           busPosition,
                                           idString,
                                           asynType,
                                           skipCycles);

    if (errorCode) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_ERROR,
                "ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",
                idString,
                errorCode);
      return asynError;
    }
    parameterCounter++;
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",
              idString);
    return asynSuccess;
  }

  // Check if default parameters for ecmc
  nvals = strcmp(idString, "ecmc.default");

  if (nvals == 0) {
    errorCode = addDefaultAsynParams(1, skipCycles);

    if (errorCode) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_ERROR,
                "ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",
                idString,
                errorCode);
      return asynError;
    }
    parameterCounter++;
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",
              idString);
    return asynSuccess;
  }

  // Check if default or diagnositc parameters for axis
  int axisIndex = 0;
  buffer[0] = '\0';
  nvals     = sscanf(idString, "ax%d.%s", &axisIndex, buffer);

  if ((nvals == 2) && (strcmp(buffer, "default") == 0)) {
    errorCode = addDefaultAsynAxis(1, axisIndex, skipCycles);

    if (errorCode) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_ERROR,
                "ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",
                idString,
                errorCode);
      return asynError;
    }
    parameterCounter++;
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",
              idString);
    return asynSuccess;
  }

  if ((nvals == 2) && (strcmp(buffer, "diagnostic") == 0)) {
    errorCode = addDiagAsynAxis(1, axisIndex, skipCycles);

    if (errorCode) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_ERROR,
                "ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",
                idString,
                errorCode);
      return asynError;
    }
    parameterCounter++;
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",
              idString);
    return asynSuccess;
  }

  // Check if default parameters for ec
  nvals = sscanf(idString, "ec%d.default", &masterIndex);

  if (nvals == 1) {
    errorCode = addDefaultAsynEc(masterIndex, 1, skipCycles);

    if (errorCode) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_ERROR,
                "ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",
                idString,
                errorCode);
      return asynError;
    }
    parameterCounter++;
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",
              idString);
    return asynSuccess;
  }

  // No parameter assigned...
  asynPrint(pPrintOutAsynUser,
            ASYN_TRACE_ERROR,
            "ecmcAsynPortDriverAddParameter:\
      ERROR: No defined data access transfer type in idString. Vaild syntax:\
      ec<master>.default, ec<master>.s<slavenumber>.defualt, \
      ec<master>.s<slavenumber>.<alias>, ec<master>.mm.<alias>, thread.default,\
      ax<index>.default, ax<index>.diagnostic.\n");
  return asynError;
  */
}

/* EPICS iocsh shell command:  ecmcAsynPortDriverAddParameter*/
static const iocshArg initArg0_2 = { "port name", iocshArgString };
static const iocshArg initArg1_2 = { "id string", iocshArgString };
static const iocshArg initArg2_2 = { "asynType", iocshArgString };
static const iocshArg initArg3_2 = { "skipCycles", iocshArgInt };

static const iocshArg *const initArgs_2[] = { &initArg0_2,
                                              &initArg1_2,
                                              &initArg2_2,
                                              &initArg3_2
};

static const iocshFuncDef initFuncDef_2 =
{ "ecmcAsynPortDriverAddParameter", 4, initArgs_2 };
static void initCallFunc_2(const iocshArgBuf *args) {
  ecmcAsynPortDriverAddParameter(args[0].sval,
                                 args[1].sval,
                                 args[2].sval,
                                 args[3].ival);
}

/* EPICS iocsh shell command:  ecmcConfigOrDie*/
static ecmcOutputBufferType ecmcConfigBuffer;
int ecmcConfigOrDie(const char *ecmcCommand) {

  if(ecmcCommand == NULL) {
    printf("Error: Command missing.\n");
    return asynError;  
  }

  clearBuffer(&ecmcConfigBuffer);
  int errorCode = motorHandleOneArg(ecmcCommand, &ecmcConfigBuffer);

  if (errorCode) {
    LOGINFO("ERROR: Command %s resulted in buffer overflow error: %s.\n",
            ecmcCommand,
            ecmcConfigBuffer.buffer);
    exit(EXIT_FAILURE);
  }

  // Check return value
  if (strcmp(ecmcConfigBuffer.buffer, "OK")) {
    int ecmcError = 0;
    int nvals     = sscanf(ecmcConfigBuffer.buffer,
                           ECMC_RETURN_ERROR_STRING "%d",
                           &ecmcError);

    if (nvals == 1) {
      LOGINFO("ECMC returned error: %s (0x%x)\n",
              getErrorString(ecmcError),
              ecmcError);
    } else {
      LOGINFO("ECMC did not return \"OK\": %s\n", ecmcConfigBuffer.buffer);
    }
    exit(EXIT_FAILURE);
  }
  LOGINFO("%s\n", ecmcConfigBuffer.buffer);

  return 0;
}

static const iocshArg initArg0_3 =
{ "Ecmc Command", iocshArgString };
static const iocshArg *const initArgs_3[]  = { &initArg0_3 };
static const iocshFuncDef    initFuncDef_3 =
{ "ecmcConfigOrDie", 1, initArgs_3 };
static void initCallFunc_3(const iocshArgBuf *args) {
  ecmcConfigOrDie(args[0].sval);
}

/* EPICS iocsh shell command:  ecmcConfig*/
int ecmcConfig(const char *ecmcCommand) {
  
  if(ecmcCommand == NULL) {
    printf("Error: Command missing.\n");
    return asynError;  
  }

  clearBuffer(&ecmcConfigBuffer);
  int errorCode = motorHandleOneArg(ecmcCommand, &ecmcConfigBuffer);

  if (errorCode) {
    LOGINFO("ERROR: Command \"%s\" resulted in error code: %s.\n",
            ecmcCommand,
            ecmcConfigBuffer.buffer);
  }

  LOGINFO("%s\n", ecmcConfigBuffer.buffer);
  return 0;
}

static const iocshArg initArg0_4 =
{ "Ecmc Command", iocshArgString };
static const iocshArg *const initArgs_4[]  = { &initArg0_4 };
static const iocshFuncDef    initFuncDef_4 = { "ecmcConfig", 1, initArgs_4 };
static void initCallFunc_4(const iocshArgBuf *args) {
  ecmcConfig(args[0].sval);
}

void ecmcAsynPortDriverRegister(void) {
  iocshRegister(&initFuncDef,   initCallFunc);
  iocshRegister(&initFuncDef_2, initCallFunc_2);
  iocshRegister(&initFuncDef_3, initCallFunc_3);
  iocshRegister(&initFuncDef_4, initCallFunc_4);
}

epicsExportRegistrar(ecmcAsynPortDriverRegister);
}