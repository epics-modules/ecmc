/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcAsynPortDriver.cpp
*
*  Created on: Jan 29, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <string>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <iocsh.h>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsExport.h>
#include <epicsEvent.h>
#include <envDefs.h>
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
#include "exprtkWrap.h"

static const char *driverName = "ecmcAsynPortDriver";

extern double mcuFrequency;

static int allowCallbackEpicsState=0;
static initHookState currentEpicsState=initHookAtIocBuild;
static ecmcAsynPortDriver *ecmcAsynPortObj=NULL;

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
      /** Make all callbacks if data arrived from callback before interrupts 
        were registered (before allowCallbackEpicsState==1)
        */
      ecmcAsynPortObj->refreshAllInUseParamsRT();      
      break;
    default:
      break;
  }

  currentEpicsState=state;
  asynPrint(asynTraceUser,
            ASYN_TRACE_INFO ,
            "%s:%s: EPICS state: %s (%d). Allow callbacks: %s.\n",
            driverName,
            functionName,
            epicsStateToString((int)state),
            (int)state,ecmcAsynPortObj->getAllowRtThreadCom() ? "true" : "false");
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
                   ECMC_STACK_SIZE)
                   {
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
  fastestParamUpdateCycles_=(int32_t)(defaultSampleRateMS/1000.0*mcuFrequency);
  /* If paramTableSize_==1 then only stream device or motor record
  can use the driver through the "default access" param below.
  */
  if(paramTableSize_<1){
    asynPrint(pasynUserSelf,
              ASYN_TRACE_ERROR,
              "%s:%s: Param table size to small: %d\n",
              driverName,
              functionName,
              paramTableSize_);
    exit(1);
    return;
  }

  // Add first param for other access (like motor record or stream device).
  ecmcAsynDataItem *paramTemp = new ecmcAsynDataItem(this,
                                                     ECMC_ASYN_PAR_OCTET_NAME,
                                                     asynParamNotDefined,
                                                     ECMC_EC_NONE);
  if(paramTemp->createParam()){
    asynPrint(pasynUserSelf,
              ASYN_TRACE_ERROR,
              "%s:%s: createParam for %s failed.\n",
              driverName,
              functionName,
              ECMC_ASYN_PAR_OCTET_NAME);
    delete paramTemp;
    paramTemp = NULL;
    exit(1);    
  }

  if(appendAvailParam(paramTemp,1)!=asynSuccess){
    asynPrint(pasynUserSelf,
              ASYN_TRACE_ERROR,
              "%s:%s: Append asyn octet param %s failed.\n",
              driverName,
              functionName,
              ECMC_ASYN_PAR_OCTET_NAME);
    delete paramTemp;
    paramTemp = NULL;
    exit(1);
  }  

  if(appendInUseParam(paramTemp,1)!=asynSuccess){
    asynPrint(pasynUserSelf,
              ASYN_TRACE_ERROR,
              "%s:%s: Append asyn octet param %s failed.\n",
              driverName,
              functionName,
              ECMC_ASYN_PAR_OCTET_NAME);
    delete paramTemp;
    paramTemp = NULL;
    exit(1);
  }
  
  int errorCode = ecmcInit((void *)this);
  if (errorCode) {
    asynPrint(pasynUserSelf,
              ASYN_TRACE_ERROR,
              "ecmcAsynPortDriverConfigure: ERROR: Init ECMC failed (0x%x).\n",
              errorCode);
    exit(1);
    }  
}

ecmcAsynPortDriver::~ecmcAsynPortDriver(){
  delete pEcmcParamInUseArray_; 
  pEcmcParamInUseArray_ = NULL;
  delete pEcmcParamAvailArray_; 
  pEcmcParamAvailArray_ = NULL;
  ecmcCleanup();
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
    asynPrint(pasynUserSelf,
    ASYN_TRACE_ERROR,
    "%s:%s: Error: DataItem NULL.",
    driverName,
    functionName);
    if(dieIfFail){
      exit(1);
    }
    return asynError;
  }

  if(ecmcParamInUseCount_>=(paramTableSize_-1)){
    asynPrint(pasynUserSelf,
              ASYN_TRACE_ERROR,
              "%s:%s: Parameter table full. Parameter with name %s will be discarded.", 
              driverName,
              functionName,
              dataItem->getParamInfo()->name);
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
    asynPrint(pasynUserSelf,
    ASYN_TRACE_ERROR,
    "%s:%s: Error: DataItem NULL.\n",
    driverName,
    functionName);
    if(dieIfFail){
      exit(1);
    }
    return asynError;
  }

  if(ecmcParamAvailCount_>=(paramTableSize_-1)){
    asynPrint(pasynUserSelf,
              ASYN_TRACE_ERROR,
              "%s:%s: ERROR: Parameter table full (available params).\n" 
              "Parameter with name %s will be discarded (max params = %d).\n"
              "Increase paramtable size in call to ecmcAsynPortDriverConfigure().\n", 
              driverName,
              functionName,
              dataItem->getParamInfo()->name,
              paramTableSize_);
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
  * \param[in] type Asyn parameter type\n
  * \param[in] dt   Data type\n
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
                                                       ecmcEcDataType dt,
                                                       bool dieIfFail) {

  const char* functionName = "addNewAvailParam";

  ecmcAsynDataItem *paramTemp = new ecmcAsynDataItem(this,name,type,dt);
  
  int errorCode=paramTemp->setEcmcDataPointer(data, bytes);
  if(errorCode) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
               "%s:%s: ERROR: Set data pointer to asyn parameter %s failed.\n", 
               driverName,
               functionName,
               name);
    delete paramTemp;
    paramTemp = NULL;
    return NULL;
  }
  asynStatus status = appendAvailParam(paramTemp,0);
  if (status != asynSuccess) {
    /*asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
               "%s:%s: ERROR: Append asyn parameter %s to list failed.\n", 
               driverName,
               functionName,
               name);*/
    delete paramTemp;
    paramTemp = NULL;
    return NULL;
  }
 
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

  asynPrint(pasynUser, 
            ASYN_TRACE_FLOW,
            "%s write.\n",
            portName);
  asynPrintIO(pasynUser,
              ASYN_TRACEIO_DRIVER,
              value,
              maxChars,
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
  asynPrint(pasynUser,
            ASYN_TRACE_FLOW,
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
            driverName, functionName, paramIndex, paramName,
            pEcmcParamInUseArray_[paramIndex]->getName());
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

  int addr=0;
  asynStatus status = getAddress(pasynUser, &addr);
  if (status != asynSuccess){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: getAddress() failed.\n",driverName, functionName);
    return(status);
  }

  ecmcAsynDataItem * newParam = new ecmcAsynDataItem(this);
  status = newParam->setDrvInfo(drvInfo);
  if(status!=asynSuccess){
    delete newParam;
    return asynError;
  }

  int index=0;
  status=findParam(ECMC_ASYN_DEFAULT_LIST,newParam->getName(),&index);  
  
  if(status!=asynSuccess) {
    
    // Param not found see if found in available list
    ecmcAsynDataItem * param = findAvailParam(newParam->getName());
    if(!param) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Parameter %s not found (drvInfo=%s).\n",
                driverName, functionName,newParam->getName(),drvInfo);
      delete newParam;
      return asynError;
    }

    //Ensure that type is supported
    if( !param->asynTypeSupported(newParam->getAsynType())) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Error: asynType %s not supported for parameter %s. Supported types are:\n", 
               driverName, functionName,newParam->getAsynTypeName(),param->getName());
      for(int i=0;i < param->getSupportedAsynTypeCount();i++) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: AsynType: %s (%d)\n",
                driverName, functionName,
                asynTypeToString((long)param->getSupportedAsynType(i)),
                param->getSupportedAsynType(i));
      }
      delete newParam;
      return asynError;      
    }
    // Type supported so use it.    
    param->setAsynParameterType(newParam->getAsynType());        
    param->getParamInfo()->cmdFloat64ToInt32 = newParam->getParamInfo()->cmdFloat64ToInt32;
    param->getParamInfo()->cmdInt64ToFloat64 = newParam->getParamInfo()->cmdInt64ToFloat64;
    param->getParamInfo()->cmdUint64ToFloat64 = newParam->getParamInfo()->cmdUint64ToFloat64;

    // Add parameter to In use list
    status = appendInUseParam(param,0);
    if(status!=asynSuccess) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Append parameter %s to in-use list failed.\n",
                driverName, functionName,newParam->getName());
      delete newParam;
      return asynError;
    }
    
    // Create asyn param
    int errorCode=param->createParam();
    if(errorCode) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Create parameter %s failed (0x%x).\n",
                driverName, functionName,newParam->getName(),errorCode);
      delete newParam;
      return asynError;
    }

    // Ensure that parameter index is correct
    if(param->getAsynParameterIndex() != (ecmcParamInUseCount_-1)) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Parameter index missmatch for  %s  (%d != %d).\n",
                driverName, functionName,newParam->getName(),param->getAsynParameterIndex(),ecmcParamInUseCount_-1);
      delete newParam;                
      return asynError;
    }

    //Update index
    index = param->getAsynParameterIndex();

    asynPrint(pasynUser, ASYN_TRACE_INFO, "%s:%s: Parameter %s linked to record (asyn reason %d).\n",
            driverName, functionName,newParam->getName(),index);

    //Now we have linked a parameter from available list into the inUse list successfully
  }

  //Asyn parameter found. Update with new info from record defs
  if(!pEcmcParamInUseArray_[index]) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s:pAdsParamArray_[%d]==NULL (drvInfo=%s).\n",
              driverName, functionName,index,drvInfo);
    delete newParam;
    return asynError;
  }
  
  ecmcParamInfo *existentParInfo = pEcmcParamInUseArray_[index]->getParamInfo();
 
  if(!existentParInfo->initialized) {
    pEcmcParamInUseArray_[index]->setAsynParSampleTimeMS(newParam->getSampleTimeMs());    
    existentParInfo->recordName=strdup(newParam->getRecordName());
    existentParInfo->recordType=strdup(newParam->getRecordType());
    existentParInfo->dtyp=strdup(newParam->getDtyp());
    existentParInfo->drvInfo=strdup(newParam->getDrvInfo());

    if(existentParInfo->cmdInt64ToFloat64 && pEcmcParamInUseArray_[index]->getEcmcBitCount() !=64) {      
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Command " ECMC_OPTION_CMD_INT_TO_FLOAT64 " is only valid for 8 byte parameters (drvInfo = %s).\n",
                driverName, functionName,drvInfo);
      delete newParam;
      return asynError;
    }
    
    if(existentParInfo->cmdUint64ToFloat64 && pEcmcParamInUseArray_[index]->getEcmcBitCount() !=64) {      
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Command " ECMC_OPTION_CMD_UINT_TO_FLOAT64 " is only valid for 8 byte parameters (drvInfo = %s).\n",
                driverName, functionName,drvInfo);
      delete newParam;
      return asynError;
    }

    if(existentParInfo->cmdFloat64ToInt32 && pEcmcParamInUseArray_[index]->getParamInfo()->ecmcSize !=8) {      
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Command " ECMC_OPTION_CMD_FLOAT64_INT " is only valid for 8 byte parameters (drvInfo = %s).\n",
                driverName, functionName,drvInfo);
      delete newParam;
      return asynError;
    }

  }

  // Ensure that sample time is the shortest (if several records 
  // on the same parameter)
  if(newParam->getSampleTimeMs() <  existentParInfo->sampleTimeMS ) {
    pEcmcParamInUseArray_[index]->setAsynParSampleTimeMS(newParam->getSampleTimeMs());      
  }

  if(pasynUser->timeout < newParam->getSampleTimeMs()*2/1000.0) {
    pasynUser->timeout = (newParam->getSampleTimeMs()*2)/1000;
  }

  delete newParam;

  existentParInfo->initialized=1;
  pEcmcParamInUseArray_[index]->refreshParam(1);
  callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);

  return asynPortDriver::drvUserCreate(pasynUser,existentParInfo->name,pptypeName,psize);
}

int32_t ecmcAsynPortDriver::getFastestUpdateRate() { 
   return fastestParamUpdateCycles_;
}

int32_t ecmcAsynPortDriver::calcFastestUpdateRate() {

  fastestParamUpdateCycles_=(int32_t)(defaultSampleTimeMS_/1000.0*mcuFrequency);
  for(int i=0;i<ecmcParamInUseCount_;i++) {
    if(pEcmcParamInUseArray_[i]) {
      if(!pEcmcParamInUseArray_[i]->initialized()) {        
        continue;
      }
      if(pEcmcParamInUseArray_[i]->getSampleTimeCycles()<fastestParamUpdateCycles_ && 
         pEcmcParamInUseArray_[i]->getSampleTimeCycles()>=0) {
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
    fprintf(fp,"    ECMC Data type:            %s\n",getEcDataTypeStr(param->getEcDataType()));
    fprintf(fp,"\n");
    return;
  }
  fprintf(fp,"    Param drvInfo:             %s\n",paramInfo->drvInfo);
  fprintf(fp,"    Param sample time [ms]:    %.0lf\n",paramInfo->sampleTimeMS);
  fprintf(fp,"    Param sample cycles []:    %d\n",paramInfo->sampleTimeCycles);
  fprintf(fp,"    Param isIOIntr:            %s\n",paramInfo->isIOIntr ? "true" : "false");
  fprintf(fp,"    Param asyn addr:           %d\n",paramInfo->asynAddr);
  fprintf(fp,"    Param alarm:               %d\n",paramInfo->alarmStatus);
  fprintf(fp,"    Param severity:            %d\n",paramInfo->alarmSeverity);
  fprintf(fp,"    ECMC data pointer valid:   %s\n",paramInfo->ecmcDataPointerValid ? "true" : "false");
  fprintf(fp,"    ECMC size [bits]:          %lu\n",param->getEcmcBitCount());
  fprintf(fp,"    ECMC max size [bytes]:     %lu\n",paramInfo->ecmcMaxSize);
  fprintf(fp,"    ECMC data is array:        %s\n",paramInfo->ecmcDataIsArray ? "true" : "false");
  fprintf(fp,"    ECMC write allowed:        %s\n",param->writeToEcmcAllowed() ? "true" : "false");
  fprintf(fp,"    ECMC Data type:            %s\n",getEcDataTypeStr(param->getEcDataType()));
  
  // Value range only applicable for ints
  if(param->getEcmcMinValueInt() != param->getEcmcMaxValueInt()) {
    fprintf(fp,"    ECMC Value Range:          %ld..%ld, %ld bit(s)\n",
            param->getEcmcMinValueInt(),
            param->getEcmcMaxValueInt(),
            param->getEcmcBitCount());    
  }
  fprintf(fp,"    ECMC Cmd: Uint2Float64:    %s\n",paramInfo->cmdUint64ToFloat64 ? "true" : "false");
  fprintf(fp,"    ECMC Cmd: Int2Float64:     %s\n",paramInfo->cmdInt64ToFloat64 ? "true" : "false");
  fprintf(fp,"    ECMC Cmd: Float642Int:     %s\n",paramInfo->cmdFloat64ToInt32 ? "true" : "false");
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

  if (details >= 0) {
    fprintf(fp,"####################################################################:\n");
    fprintf(fp, "General information:\n");
    fprintf(fp, "  Port:                         %s\n",portName);
    fprintf(fp, "  Auto-connect:                 %s\n",autoConnect_ ? "true" : "false");
    fprintf(fp, "  Priority:                     %d\n",priority_);
    fprintf(fp, "  Param. table size:            %d\n",paramTableSize_);
    fprintf(fp, "  Param. count:                 %d\n",ecmcParamInUseCount_);
    fprintf(fp, "  Default sample time [ms]:     %d\n",defaultSampleTimeMS_);
    fprintf(fp, "  Fastest update rate [cycles]: %d\n",fastestParamUpdateCycles_);
    fprintf(fp,"\n");
  }

  if( details >= 1){
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

  if( details >= 2){
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
  if (details >= 0) {
    fprintf(fp,"####################################################################:\n");
  }
  
  if( details >= 3){
    fprintf(fp,"Report from base class (asynPortDriver):\n");
    asynPortDriver::report(fp,details);
    fprintf(fp,"####################################################################:\n");
  }
}

void ecmcAsynPortDriver::grepParam(FILE *fp, const char *pattern) {
  
  const char* functionName = "grepParam";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  if(!fp){
    fprintf(fp,"%s:%s: ERROR: File NULL.\n", driverName, functionName);
    return;
  }

  //print all parameters that fit pattern
  fprintf(fp,"####################################################################:\n");
  fprintf(fp,"ecmc parameters that fit pattern %s:\n",pattern);
  for(int i=0; i<ecmcParamAvailCount_;i++){
    if(pEcmcParamAvailArray_[i]){
      ecmcParamInfo *paramInfo=pEcmcParamAvailArray_[i]->getParamInfo();
      if(paramInfo) {
        if(epicsStrGlobMatch(paramInfo->name,pattern)) {
          reportParamInfo(fp, pEcmcParamAvailArray_[i],i);
        }
      }
    }         
  }
}

void ecmcAsynPortDriver::grepRecord(FILE *fp, const char *pattern) {
  
  const char* functionName = "grepRecord";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s:\n", driverName, functionName);

  if(!fp){
    fprintf(fp,"%s:%s: ERROR: File NULL.\n", driverName, functionName);
    return;
  }

  //print all parameters that fit pattern
  fprintf(fp,"####################################################################:\n");
  fprintf(fp,"ecmc records that fit pattern %s:\n",pattern);
  for(int i=0; i<ecmcParamAvailCount_;i++){
    if(pEcmcParamAvailArray_[i]){
      ecmcParamInfo *paramInfo=pEcmcParamAvailArray_[i]->getParamInfo();
      if(paramInfo) {
        // Match param-name or record-name
        if(paramInfo->initialized) {
          if(epicsStrGlobMatch(paramInfo->recordName,pattern)) {           
            reportParamInfo(fp, pEcmcParamAvailArray_[i],i);
          }
        }
      }
    }         
  }
}

int ecmcAsynPortDriver::getDefaultSampleTimeMs() {
  return defaultSampleTimeMS_;
}
/* Configuration routine.  Called directly, or from the iocsh function below */

extern "C" {
static int maxParameters;
static int parameterCounter;
static int ecmcInitialized = 0;

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

  if(ecmcInitialized) {
    printf("ecmcAsynPortDriverConfigure: Error: ECMC already initialized. Command ignored.\n");
    return asynError;     
  }

  if(portName == NULL) {
    printf("ecmcAsynPortDriverConfigure: Error: portName missing.\n");
    return asynError;  
  }

  int paramTableMin = ECMC_ASYN_MAIN_PAR_COUNT + ECMC_ASYN_EC_PAR_COUNT + 1;
  if(paramTableSize < paramTableMin) {
     paramTableSize = paramTableMin;
      printf(
        "ecmcAsynPortDriverConfigure: WARNING: Parameter table to small. Increasing to minimum value (%d)\n",
        paramTableMin);      
  } 

  parameterCounter = 0;
  maxParameters    = paramTableSize;
  ecmcAsynPortObj  = new ecmcAsynPortDriver(portName,
                                            paramTableSize,
                                            disableAutoConnect == 0,
                                            priority,
                                            defaultSampleRateMS);

  ecmcInitialized = 1;

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

    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "ecmcAsynPortDriverConfigure: INFO: New AsynPortDriver success (%s,%i,%i,%i).",
              portName,
              paramTableSize,
              disableAutoConnect == 0,
              priority);

    return asynSuccess;
  } else {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_ERROR,
              "ecmcAsynPortDriverConfigure: ERROR: New AsynPortDriver failed.");
    return asynError;
  }
}

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

/** \brief Obsolete EPICS iocsh command for adding asyn-parameter(s)
 */
int ecmcAsynPortDriverAddParameter(const char *portName,
                                   const char *idString,
                                   const char *asynTypeString,
                                   int         skipCycles) {

  printf("Error: ecmcAsynPortDriverAddParameter is an obsolete command.\n");
  printf("       Use the command \"asynReport 3\" to list available parameters.\n");
  return 0;
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

  if(!ecmcAsynPortObj) {
    printf("Error: No ecmcAsynPortDriver object found (ecmcAsynPortObj==NULL).\n");
    printf("       Use ecmcAsynPortDriverConfigure() to create object.\n");
    return asynError;  
  }

  if(!ecmcCommand) {
    printf("Error: Command missing.\n");
    printf("       Use \"ecmcConfigOrDie <command>\" to configure ecmc system\n");
    return asynError;  
  }
  
  ecmcAsynPortObj->lock();

  clearBuffer(&ecmcConfigBuffer);
  int errorCode = motorHandleOneArg(ecmcCommand, &ecmcConfigBuffer);
  
  ecmcAsynPortObj->unlock();
  
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
      LOGINFO("ECMC command \"%s\" returned error: %s (0x%x)\n",
              ecmcCommand,
              getErrorString(ecmcError),
              ecmcError);
    } else {
      LOGINFO("ECMC did not return \"OK\": %s\n", ecmcConfigBuffer.buffer);
    }
    exit(EXIT_FAILURE);
  }
  LOGINFO("%s\n", ecmcConfigBuffer.buffer);
  // Set return variable
  epicsEnvSet(ECMC_IOCSH_CFG_CMD_RETURN_VAR_NAME,ecmcConfigBuffer.buffer);
  
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
  
  if(!ecmcAsynPortObj) {
    printf("Error: No ecmcAsynPortDriver object found (ecmcAsynPortObj==NULL).\n");
    printf("       Use ecmcAsynPortDriverConfigure() to create object.\n");
    return asynError;  
  }

  if(!ecmcCommand) {
    printf("Error: Command missing.\n");
    printf("       Use \"ecmcConfig <command>\" to configure ecmc system.\n");
    return asynError;  
  }
  
  ecmcAsynPortObj->lock();

  clearBuffer(&ecmcConfigBuffer);
  int errorCode = motorHandleOneArg(ecmcCommand, &ecmcConfigBuffer);

  ecmcAsynPortObj->unlock();

  if (errorCode) {
    LOGINFO("ERROR: Command \"%s\" resulted in error code: %s.\n",
            ecmcCommand,
            ecmcConfigBuffer.buffer);
  }

  LOGINFO("%s\n", ecmcConfigBuffer.buffer);
  // Set return variable
  epicsEnvSet(ECMC_IOCSH_CFG_CMD_RETURN_VAR_NAME,ecmcConfigBuffer.buffer);
  
  return 0;
}

static const iocshArg initArg0_4 =
{ "Ecmc Command", iocshArgString };
static const iocshArg *const initArgs_4[]  = { &initArg0_4 };
static const iocshFuncDef    initFuncDef_4 = { "ecmcConfig", 1, initArgs_4 };
static void initCallFunc_4(const iocshArgBuf *args) {
  ecmcConfig(args[0].sval);
}

/* EPICS iocsh shell command: ecmcReport (same as asynReport but only ECMC)*/
int ecmcReport(int level) {
  
  if(!ecmcAsynPortObj) {
    printf("Error: No ecmcAsynPortDriver object found (ecmcAsynPortObj==NULL).\n");
    printf("       Use ecmcAsynPortDriverConfigure() to create object.\n");
    return asynError;  
  }
  
  ecmcAsynPortObj->report(stdout,level);

  return 0;
}

static const iocshArg initArg0_5 =
{ "Details level", iocshArgInt };
static const iocshArg *const initArgs_5[]  = { &initArg0_5 };
static const iocshFuncDef    initFuncDef_5 = { "ecmcReport", 1, initArgs_5 };
static void initCallFunc_5(const iocshArgBuf *args) {
  ecmcReport(args[0].ival);
}

/* EPICS iocsh shell command: ecmcGrepParam*/
int ecmcGrepParam(const char *pattern) {
  
  if(!ecmcAsynPortObj) {
    printf("Error: No ecmcAsynPortDriver object found (ecmcAsynPortObj==NULL).\n");
    printf("       Use ecmcAsynPortDriverConfigure() to create object.\n");
    return asynError;  
  }
  
  if(!pattern) {
    printf("Error: Pattern missing.\n");
    printf("       Use \"ecmcGrepParam <pattern>\" to list ecmc params/records.\n");
    return asynError;  
  }

  ecmcAsynPortObj->grepParam(stdout,pattern);

  return 0;
}

static const iocshArg initArg0_6 =
{ "Pattern", iocshArgString };
static const iocshArg *const initArgs_6[]  = { &initArg0_6 };
static const iocshFuncDef    initFuncDef_6 = { "ecmcGrepParam", 1, initArgs_6 };
static void initCallFunc_6(const iocshArgBuf *args) {
  ecmcGrepParam(args[0].sval);
}

/* EPICS iocsh shell command: ecmcGrepRecord*/
int ecmcGrepRecord(const char *pattern) {
  
  if(!ecmcAsynPortObj) {
    printf("Error: No ecmcAsynPortDriver object found (ecmcAsynPortObj==NULL).\n");
    printf("       Use ecmcAsynPortDriverConfigure() to create object.\n");
    return asynError;  
  }
  
  if(!pattern) {
    printf("Error: Pattern missing.\n");
    printf("       Use \"ecmcGrepRecord <pattern>\" to list ecmc params/records.\n");
    return asynError;  
  }

  ecmcAsynPortObj->grepRecord(stdout,pattern);

  return 0;
}

static const iocshArg initArg0_7 =
{ "Pattern", iocshArgString };
static const iocshArg *const initArgs_7[]  = { &initArg0_7 };
static const iocshFuncDef    initFuncDef_7 = { "ecmcGrepRecord", 1, initArgs_7 };
static void initCallFunc_7(const iocshArgBuf *args) {
  ecmcGrepRecord(args[0].sval);
}

static const char * allowedSpecInt[] = {
  "d", "i", "o", "u", "x", "l", "\0"
};

static const char * allowedSpecFloat[] = {
  "a","A", "e", "E", "f", "F", "g" ,"G", "\0"
};

int formatIsDouble(const char* format) {

  if(!format) {
    printf("Invalid or empty format string.\n");
    return -1;
  } 
  
  if(strlen(format)>= (ECMC_CMD_MAX_SINGLE_CMD_LENGTH -1)){
    printf("Format string to long (max length %d).\n",ECMC_CMD_MAX_SINGLE_CMD_LENGTH);
    return -1;
  }

  const char *firstProcent = strchr(format,'%');
  if(!firstProcent) {
    return -1;
  }

  char flags[ECMC_CMD_MAX_SINGLE_CMD_LENGTH];
  memset(flags,0,sizeof(flags));
  int nvals = sscanf(firstProcent,"%%%[0-9 | +-.#hl]",flags);

  char specifiers[ECMC_CMD_MAX_SINGLE_CMD_LENGTH];
  memset(specifiers,0,sizeof(specifiers));
  char *formatStart = (char*)firstProcent+strlen(flags)+1;
  nvals = sscanf(formatStart,"%s",specifiers);
  
  if (nvals != 1) {
    printf("Format string error. Could not determine specifier in string %s.\n",specifiers);
    return -1;
  }

  //Check specifiers for int
  size_t i=0;
  const char* element=allowedSpecInt[i];
  while(element[0] != 0) {  
    if(strstr(specifiers,element)) {
      return 0;
    }
    i++;
    element=allowedSpecInt[i];
  }

  //Check specifiers for double
  i=0;
  element=allowedSpecFloat[i];
  while(element[0] != 0) {  
    if(strstr(specifiers,element)) {
      return 1;
    }
    i++;
    element=allowedSpecFloat[i];
  }
  
  return -1; //invalid
}

asynStatus evalExprTK(const char* expression, double *result) {

  // Evaluate expression
  exprtkWrap *exprtk = new exprtkWrap();
  if(!exprtk) {
     printf ("Failed allocation of exprtk expression parser.\n");
    return asynError;
  }
  double resultDouble = 0;

  exprtk->addVariable(ECMC_ENVSETCALC_RESULT_VAR, resultDouble);

  std::string exprStr="";

  // Check if "RESULT" variable in str. If not then simple expression.. Add in beginning
  if(strstr(expression,ECMC_ENVSETCALC_RESULT_VAR)) {
    exprStr = expression;   
  }
  else {
    exprStr = ECMC_ENVSETCALC_RESULT_VAR;   
    exprStr += ":=";
    exprStr += expression;       
  }

  //Check if need to add ";" last
  if(exprStr.c_str()[strlen(exprStr.c_str())-1] != ';') {
    exprStr += ";";
  }
  
  if(exprtk->compile(exprStr)) {
    printf ("Failed compile of expression with error message: %s.\n", exprtk->getParserError().c_str());
    return asynError;
  }
  exprtk->refresh();
  delete exprtk;  // not needed anymore (result in "resultDouble")

  *result = resultDouble;
  return asynSuccess;
}

void ecmcEpicsEnvSetCalcPrintHelp() {
  printf("\n");
  printf("       Use \"ecmcEpicsEnvSetCalc(<envVarName>,  <expression>, <format>)\" to evaluate the expression and assign the variable.\n");
  printf("          <envVarName> : EPICS environment variable name.\n");
  printf("          <expression> : Calculation expression (see exprTK for available functionality). Examples:\n");
  printf("                         Simple expression:\"5.5+${TEST_SCALE}*sin(${TEST_ANGLE}/10)\".\n");
  printf("                         Use of \"RESULT\" variable: \"if(${TEST_VAL}>5){RESULT:=100;}else{RESULT:=200;};\".\n");
  printf("          <format>     : Optional format string. Example \"%%lf\", \"%%8.3lf\", \"%%x\", \"%%04d\" or \"%%d\", defaults to \"%%d\".\n");
  printf("                         Can contain text like \"0x%%x\" or \"Hex value is 0x60%%x\".\n");
  printf("                         Must contain one numeric value where result of expression will be written.\n");
  printf("\n");
  printf("       Restrictions:\n");
  printf("         - Some flags and/or width/precision combinations might not be supported.\n");
  printf("         - Hex numbers in the expression is not allowed (but hex as output by formating is OK).\n");
  printf("         - Non floatingpoint values will be rounded to nearest int.\n");
  printf("\n");
}

/** EPICS iocsh shell command: ecmcEpicsEnvSetCalc
 *  Evaluates an expression and sets an EPICS environment variable
*/
int ecmcEpicsEnvSetCalc(const char *envVarName, const char *expression, const char *format) {

  const char *localFormat=format;
  if (!localFormat) {
    localFormat=ECMC_ENVSETCALC_DEF_FORMAT;
  }

  if(!envVarName) {
    printf("Error: Environment variable name  missing.\n");
    ecmcEpicsEnvSetCalcPrintHelp();
    return asynError;
  }

  if(strcmp(envVarName,"-h") == 0 || strcmp(envVarName,"--help") == 0 ) {
    ecmcEpicsEnvSetCalcPrintHelp();
    return asynSuccess;
  }

  if(!expression) {
    printf("Error: Expression missing.\n");
    ecmcEpicsEnvSetCalcPrintHelp();
    return asynError;
  }

  double resultDouble = 0;
  if(evalExprTK(expression, &resultDouble) != asynSuccess) {
    return asynError;
  }

  // Convert if int in format string
  int resultInt=round(resultDouble);
  char buffer[ECMC_CMD_MAX_SINGLE_CMD_LENGTH];
  unsigned int charCount = 0;
  memset(buffer,0,sizeof(buffer));
  
  int isDouble = formatIsDouble(localFormat);
  if(isDouble < 0) {
    printf ("Error: Failed to determine datatype from format. Invalid format string \"%s\".\n", localFormat);
    return asynError;
  }

  if(isDouble){
    charCount = snprintf(buffer,
                         sizeof(buffer),
                         localFormat,
                         resultDouble);
  }else{
    charCount = snprintf(buffer,
                         sizeof(buffer),
                         localFormat,
                         resultInt);
  }
  if (charCount >= sizeof(buffer) - 1) {
    printf ("Write buffer size exceeded, format results in a to long string.\n");
    return asynError;
  }

  epicsEnvSet(envVarName,buffer);
  return asynSuccess;
}

static const iocshArg initArg0_8 =
{ "Variable name", iocshArgString };
static const iocshArg initArg1_8 =
{ "Expression", iocshArgString };
static const iocshArg initArg2_8 =
{ "Format", iocshArgString };
static const iocshArg *const initArgs_8[]  = { &initArg0_8, &initArg1_8, &initArg2_8};
static const iocshFuncDef    initFuncDef_8 = { "ecmcEpicsEnvSetCalc", 3, initArgs_8 };
static void initCallFunc_8(const iocshArgBuf *args) {
  ecmcEpicsEnvSetCalc(args[0].sval,args[1].sval,args[2].sval);
}

void ecmcEpicsEnvSetCalcTenaryPrintHelp() {
  printf("\n");
  printf("       Use \"ecmcEpicsEnvSetCalcTenary(<envVarName>,  <expression>, <trueString>, <falseString>)\" to evaluate the expression and assign the variable.\n");
  printf("          <envVarName>  : EPICS environment variable name.\n");
  printf("          <expression>  : Calculation expression (see exprTK for available functionality). Examples:\n");
  printf("                          Simple expression:\"5.5+${TEST_SCALE}*sin(${TEST_ANGLE}/10)\".\n");
  printf("                          Use of \"RESULT\" variable: \"if(${TEST_VAL}>5){RESULT:=100;}else{RESULT:=200;};\".\n");
  printf("          <trueString>  : String to set <envVarName> if expression (or \"RESULT\") evaluates to true.\n");
  printf("          <falseString> : String to set <envVarName> if expression (or \"RESULT\") evaluates to false.\n");
  printf("\n");
}

/** EPICS iocsh shell command: ecmcEpicsEnvSetCalcTenary
 *  Evaluates an expression and sets an EPICS environment variable to:
 *   expression>0 : trueString
 *   expression<=0: falseString
*/
int ecmcEpicsEnvSetCalcTenary(const char *envVarName, const char *expression, const char *trueString, const char *falseString) {

  if(!envVarName) {
    printf("Error: Environment variable name  missing.\n");
    ecmcEpicsEnvSetCalcTenaryPrintHelp();
    return asynError;
  }

  if(strcmp(envVarName,"-h") == 0 || strcmp(envVarName,"--help") == 0 ) {
    ecmcEpicsEnvSetCalcTenaryPrintHelp();
    return asynSuccess;
  }

  if(!expression || ! trueString || ! falseString) {
    printf("Error: \"expression\", \"trueString\" and/or \"falseString\" missing.\n");
    ecmcEpicsEnvSetCalcTenaryPrintHelp();
    return asynError;
  }

  double resultDouble = 0;
  if(evalExprTK(expression, &resultDouble) != asynSuccess) {
    return asynError;
  }

  if(resultDouble) {
    epicsEnvSet(envVarName,trueString);
  }
  else{
    epicsEnvSet(envVarName,falseString);
  }

  return asynSuccess;
}

static const iocshArg initArg0_9 =
{ "Variable name", iocshArgString };
static const iocshArg initArg1_9 =
{ "Expression", iocshArgString };
static const iocshArg initArg2_9=
{ "True string", iocshArgString };
static const iocshArg initArg3_9 =
{ "False string", iocshArgString };

static const iocshArg *const initArgs_9[]  = { &initArg0_9, &initArg1_9, &initArg2_9, &initArg3_9 };
static const iocshFuncDef    initFuncDef_9 = { "ecmcEpicsEnvSetCalcTenary", 4, initArgs_9 };
static void initCallFunc_9(const iocshArgBuf *args) {
  ecmcEpicsEnvSetCalcTenary(args[0].sval,args[1].sval,args[2].sval,args[3].sval);
}

void ecmcFileExistPrintHelp() {
  printf("\n");
  printf("       Use \"ecmcFileExist(<filename>, <die>, <check epics path>)\" to check if a file exists.\n");
  printf("          <filename>         : Filename to check.\n");
  printf("          <die>              : Exit EPICS if file not exist. Optional, defaults to 0.\n");
  printf("          <check epics path> : Also look for files in \"EPICS_DB_INCLUDE_PATH\". Optional, defaults to 0.\n");
  printf("\n");
}

/** EPICS iocsh shell command: ecmcFileExist
 * Return if file exists otherwise "die"
*/
int ecmcFileExist(const char *filename, int die, int checkDirs) {
  if(!filename) {
    printf("Error: filename missing.\n");
    ecmcFileExistPrintHelp();
    return asynError;
  }

  if(strcmp(filename,"-h") == 0 || strcmp(filename,"--help") == 0 ) {
    ecmcFileExistPrintHelp();
    return asynSuccess;
  }
  
  // Check filename directlly
  int fileExist  = access( filename, 0 ) == 0;
  
  // Search EPICS_DB_INCLUDE_PATH if not found
  if(checkDirs && !fileExist) {
    char buffer[4096];
    char* dirs = getenv("EPICS_DB_INCLUDE_PATH");
    char* pdirs=dirs; 
    char *pdirs_old=pdirs;
    if(dirs){
      bool stop = false;
      while((pdirs=strchr(pdirs,':')) && !stop){
        memset(buffer,0,4096);
        int chars=(int)(pdirs-pdirs_old);
        strncpy(buffer,pdirs_old,chars);
        buffer[chars]='/';
        chars++;
        strncpy(&buffer[chars],filename,strlen(filename));
        //printf("Buffer %s\n",buffer);
        fileExist = access( buffer, 0 ) == 0;
        if(fileExist) {
          break;
        }
        if(strlen(pdirs)>0){
          pdirs++;
        }else{
          stop = true;
        }
        pdirs_old = pdirs;
      }

      //take the last also (if not already found)
      if(strlen(pdirs_old)>0 && !fileExist){
        memset(buffer,0,4096);
        int chars=strlen(pdirs_old);
        strncpy(buffer,pdirs_old,chars);
        buffer[chars]='/';
        chars++;
        strncpy(&buffer[chars],filename,strlen(filename));
        //printf("Buffer %s\n",buffer);
        fileExist = access( buffer, 0 ) == 0;
      }
    }
  }

  if(die && !fileExist) {
    printf("Error: File \"%s\" does not exist. ECMC shuts down.\n",filename);
    exit(EXIT_FAILURE);
  }
  epicsEnvSet(ECMC_IOCSH_CFG_CMD_RETURN_VAR_NAME,fileExist ? "1":"0");
  return asynSuccess;
}

static const iocshArg initArg0_10 =
{ "Filename", iocshArgString };
static const iocshArg initArg1_10 =
{ "DieIfNoFile", iocshArgInt };
static const iocshArg initArg2_10 =
{ "Check EPICS dirs", iocshArgInt };
static const iocshArg *const initArgs_10[]  = { &initArg0_10, &initArg1_10, &initArg2_10 };
static const iocshFuncDef    initFuncDef_10 = { "ecmcFileExist", 3, initArgs_10 };
static void initCallFunc_10(const iocshArgBuf *args) {
  ecmcFileExist(args[0].sval,args[1].ival,args[2].ival);
}

void ecmcAsynPortDriverRegister(void) {
  iocshRegister(&initFuncDef,   initCallFunc);
  iocshRegister(&initFuncDef_2, initCallFunc_2);
  iocshRegister(&initFuncDef_3, initCallFunc_3);
  iocshRegister(&initFuncDef_4, initCallFunc_4);
  iocshRegister(&initFuncDef_5, initCallFunc_5);
  iocshRegister(&initFuncDef_6, initCallFunc_6);
  iocshRegister(&initFuncDef_7, initCallFunc_7);
  iocshRegister(&initFuncDef_8, initCallFunc_8);
  iocshRegister(&initFuncDef_9, initCallFunc_9);
  iocshRegister(&initFuncDef_10, initCallFunc_10);
}

epicsExportRegistrar(ecmcAsynPortDriverRegister);
}
