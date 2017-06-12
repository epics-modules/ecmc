/*
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include "cmd.h"
#include "gitversion.h"

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <iocsh.h>

#include "ecmcAsynPortDriver.h"
#include <epicsExport.h>


#include "hw_motor.h"

static const char *driverName="ecmcAsynPortDriver";

/** Constructor for the ecmcAsynPortDriver class.
  * Calls constructor for the asynPortDriver base class.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] maxPoints The maximum  number of points in the volt and time arrays */
ecmcAsynPortDriver::ecmcAsynPortDriver(const char *portName/*, int maxPoints*/,int paramTableSize,int autoConnect,int priority)
   : asynPortDriver(portName, 
                    1, /* maxAddr */ 
		    paramTableSize,
                    asynInt32Mask | asynFloat64Mask | asynFloat32ArrayMask | asynFloat64ArrayMask | asynEnumMask | asynDrvUserMask | asynOctetMask | asynInt8ArrayMask | asynInt16ArrayMask | asynInt32ArrayMask, /* Interface mask */
		    asynInt32Mask | asynFloat64Mask | asynFloat32ArrayMask | asynFloat64ArrayMask | asynEnumMask | asynDrvUserMask | asynOctetMask | asynInt8ArrayMask | asynInt16ArrayMask | asynInt32ArrayMask,  /* Interrupt mask */
		    ASYN_CANBLOCK, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
		    autoConnect, /* Autoconnect */
		    priority, /* Default priority */
                    0) /* Default stack size*/    
{
  eventId_ = epicsEventCreate(epicsEventEmpty);
}

asynStatus ecmcAsynPortDriver::readOctet(asynUser *pasynUser, char *value, size_t maxChars,size_t *nActual, int *eomReason)
{
  size_t thisRead = 0;
  int reason = 0;
  asynStatus status = asynSuccess;

  /*
   * Feed what writeIt() gave us into the MCU
   */

  *value = '\0';
  //lock();
  if (CMDreadIt(value, maxChars)) status = asynError;
  if (status == asynSuccess) {
    thisRead = strlen(value);
    *nActual = thisRead;
    /* May be not enough space ? */
    //printf("readOctet: thisread: %d\n",thisRead);
    if (thisRead > maxChars-1) {
      reason |= ASYN_EOM_CNT;
    }
    else{
      reason |= ASYN_EOM_EOS;
    }

    if (thisRead == 0 && pasynUser->timeout == 0){
      status = asynTimeout;
    }
  }
  else{printf("FAIL");}

  if (eomReason) *eomReason = reason;

  *nActual = thisRead;
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s thisRead=%lu data=\"%s\"\n",
            portName,
            (unsigned long)thisRead, value);
  //unlock();
  return status;
}

asynStatus ecmcAsynPortDriver::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars,size_t *nActual)
{
  size_t thisWrite = 0;
  asynStatus status = asynError;

  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s write.\n", /*ecmcController_p->*/portName);
  asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, value, maxChars,
              "%s write %lu\n",
              portName,
              (unsigned long)maxChars);
  *nActual = 0;

  if (maxChars == 0){
    return asynSuccess;
  }
  //lock();
  if (!(CMDwriteIt(value, maxChars))) {
    thisWrite = maxChars;
    *nActual = thisWrite;
    status = asynSuccess;
  }
  //unlock();
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s wrote %lu return %s.\n",
            portName,
            (unsigned long)*nActual,
            pasynManager->strStatus(status));
  return status;
}

asynStatus ecmcAsynPortDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  const char *paramName;
  const char* functionName = "writeInt32";
  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  char buffer[1024];
  char *aliasBuffer=&buffer[0];
  int slavePosition=-10;
  int nvals = sscanf(paramName, "ec.s%d.%s", &slavePosition,aliasBuffer);
  if(nvals!=2){
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: error, parameter name not valid: %s.\n",
        driverName, functionName, paramName);
    return asynError;
  }
  if(slavePosition<-1){
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: error, slave bus position not valid (needs to be equal or larger than -1): %d.\n",
        driverName, functionName, slavePosition);
    return asynError;
  }
  if(strlen(aliasBuffer)<=0){
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: error, ethercat slave alias not valid: %s.\n",
        driverName, functionName,aliasBuffer);
    return asynError;
  }

  int errorId=writeEcEntryIDString(slavePosition,aliasBuffer,(uint64_t)value);
  if(errorId){
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: error, write of parameter %s failed with error code 0x%x.\n",
        driverName, functionName,aliasBuffer,errorId);
    return asynError;
  }

  /* Set the parameter in the parameter library. */
  asynStatus status = (asynStatus) setIntegerParam(function, value);
  if(status !=asynSuccess){
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: error, setIngerParam() failed.\n",
        driverName, functionName);
    return asynError;
  }

  /* Do callbacks so higher layers see any changes */
  status = (asynStatus) callParamCallbacks();

  return status;
}

asynStatus ecmcAsynPortDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  const char *paramName;
  const char* functionName = "writeFloat64";
  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  char buffer[1024];
  char *aliasBuffer=&buffer[0];
  int slavePosition=-10;
  int nvals = sscanf(paramName, "ec.s%d.%s", &slavePosition,aliasBuffer);
  if(nvals!=2){
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: error, parameter name not valid: %s.\n",
        driverName, functionName, paramName);
    return asynError;
  }
  if(slavePosition<-1){
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: error, slave bus position not valid (needs to be equal or larger than -1): %d.\n",
        driverName, functionName, slavePosition);
    return asynError;
  }
  if(strlen(aliasBuffer)<=0){
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: error, ethercat slave alias not valid: %s.\n",
        driverName, functionName,aliasBuffer);
    return asynError;
  }


  int errorId=writeEcEntryIDString(slavePosition,aliasBuffer,static_cast<uint64_t>(value));
  if(errorId){
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: error, write of parameter %s failed with error code 0x%x.\n",
        driverName, functionName,aliasBuffer,errorId);
    return asynError;
  }

  /* Set the parameter in the parameter library. */
  asynStatus status = (asynStatus) setDoubleParam(function, value);
  if(status !=asynSuccess){
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: error, setIngerParam() failed.\n",
        driverName, functionName);
    return asynError;
  }

  /* Do callbacks so higher layers see any changes */
  status = (asynStatus) callParamCallbacks();

  return status;
}

asynUser *ecmcAsynPortDriver::getTraceAsynUser()
{
  return pasynUserSelf;
}

asynStatus ecmcAsynPortDriver::readInt8Array(asynUser *pasynUser, epicsInt8 *value,size_t nElements, size_t *nIn)
{
  const char* functionName = "readInt8Array";

  int errorId=readArrayGeneric(pasynUser,(uint8_t *)value,nElements,nIn,2,functionName);

  if(errorId){
    return asynError;
  }

  return asynSuccess;
}

asynStatus ecmcAsynPortDriver::readInt16Array(asynUser *pasynUser, epicsInt16 *value,size_t nElements, size_t *nIn)
{
  const char* functionName = "readInt16Array";

  int errorId=readArrayGeneric(pasynUser,(uint8_t *)value,nElements,nIn,2,functionName);

  if(errorId){
    return asynError;
  }

  return asynSuccess;
}

//int function = pasynUser->reason;
//  const char *paramName;
//  const char* functionName = "readInt16Array";
//  int errorId=0;
//  /* Fetch the parameter string name for possible use in debugging */
//  getParamName(function, &paramName);
//
//  char buffer[1024];
//  int nvals = sscanf(paramName, "ec.mm.%s",buffer);
//  if (nvals == 1) {
//    errorId=readEcMemMap(buffer,(uint8_t*)value,nElements,nIn);
//  }
//
//  if(errorId){
//    asynPrint(pasynUser, ASYN_TRACE_ERROR,"%s:%s: error, read of parameter %s failed with error code 0x%x.\n",driverName, functionName,paramName,errorId);
//    return (asynError);
//  }
//  return (asynSuccess);


asynStatus ecmcAsynPortDriver::readInt32Array(asynUser *pasynUser, epicsInt32 *value,size_t nElements, size_t *nIn)
{
  const char* functionName = "readInt32Array";

  int errorId=readArrayGeneric(pasynUser,(uint8_t *)value,nElements,nIn,2,functionName);

  if(errorId){
    return asynError;
  }

  return asynSuccess;
}

asynStatus ecmcAsynPortDriver::readFloat32Array(asynUser *pasynUser, epicsFloat32 *value,size_t nElements, size_t *nIn)
{
  const char* functionName = "readFloat32rray";

  int errorId=readArrayGeneric(pasynUser,(uint8_t *)value,nElements,nIn,2,functionName);

  if(errorId){
    return asynError;
  }

  return asynSuccess;
}

asynStatus ecmcAsynPortDriver::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,size_t nElements, size_t *nIn)
{
  const char* functionName = "readFloat64Array";
  int errorId=readArrayGeneric(pasynUser,(uint8_t *)value,nElements,nIn,2,functionName);

  if(errorId){
    return asynError;
  }

  return asynSuccess;
}

int ecmcAsynPortDriver::readArrayGeneric(asynUser *pasynUser, epicsUInt8 *value,size_t nElements, size_t *nIn,size_t typeSize,const char* functionName)
{
  const char *paramName;
  int errorId=0;
  int function = pasynUser->reason;

  getParamName(function, &paramName);

  size_t bytesRead=0;
  char buffer[1024]	      ;
  int nvals = sscanf(paramName, "ec.mm.%s",buffer);
  if (nvals == 1) {
    errorId=readEcMemMap(buffer,(uint8_t*)value,nElements*typeSize,&bytesRead);
  }

  if(errorId){
    asynPrint(pasynUser, ASYN_TRACE_ERROR,"%s:%s: error, read of parameter %s failed with error code 0x%x.\n",driverName, functionName,paramName,errorId);
    *nIn=0;
    return (asynError);
  }
  *nIn=bytesRead/typeSize;
  return errorId;
}

/* Configuration routine.  Called directly, or from the iocsh function below */

extern "C" {

static ecmcAsynPortDriver *mytestAsynPort;
static int maxParameters;
static int parameterCounter;
/* global asynUser for Printing */
asynUser *pPrintOutAsynUser;

//ecmcAsynPortDriver(const char *portName, int maxPoints,int paramTableSize,int autoConnect)
/** EPICS iocsh callable function to call constructor for the ecmcAsynPortDriver class.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] paramTableSize The max number of parameters.
  * \param[in] priority Priority.
  * \param[in] disableAutoConnect Disable auto connect */
int ecmcAsynPortDriverConfigure(const char *portName,int paramTableSize,int priority, int disableAutoConnect)
{
  parameterCounter=0;
  maxParameters=paramTableSize;
  mytestAsynPort=new ecmcAsynPortDriver(portName,paramTableSize,disableAutoConnect==0,priority);
  if(mytestAsynPort){
    asynUser *traceUser= mytestAsynPort->getTraceAsynUser();
    if(!traceUser){
      printf("ecmcAsynPortDriverConfigure: ERROR: Failed to retrieve asynUser for trace. \n");
      return (asynError);
    }

    pPrintOutAsynUser = pasynManager->duplicateAsynUser(traceUser, 0, 0);

    if(!pPrintOutAsynUser){
      printf("ecmcAsynPortDriverConfigure: ERROR: Failed to duplicate asynUser for trace. \n");
      return (asynError);
    }

    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverConfigure: INFO: New AsynPortDriver success (%s,%i,%i,%i).",portName,paramTableSize,disableAutoConnect==0,priority);

    return(asynSuccess);
  }
  else{
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverConfigure: ERROR: New AsynPortDriver failed.");
    return(asynError);
  }
}

/* EPICS iocsh shell command: ecmcAsynPortDriverConfigure*/

static const iocshArg initArg0 = { "port name",iocshArgString};
static const iocshArg initArg1 = { "parameter table size",iocshArgInt};
static const iocshArg initArg2 = { "priority",iocshArgInt};
static const iocshArg initArg3 = { "disable auto connect",iocshArgInt};

static const iocshArg * const initArgs[] = {&initArg0,
                                            &initArg1,
                                            &initArg2,
                                            &initArg3};
static const iocshFuncDef initFuncDef = {"ecmcAsynPortDriverConfigure",4,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    ecmcAsynPortDriverConfigure(args[0].sval, args[1].ival,args[2].ival,args[3].ival);
}

//****************************** Add parameter
int ecmcAsynPortDriverAddParameter(const char *portName, const char *idString, const char *asynTypeString, int skipCycles)
{

  ecmcAsynDataAccessType dataAccessType=ECMC_ASYN_NONE;
  if(!mytestAsynPort){
    printf("ecmcAsynPortDriverAddParameter: ERROR: asynPortDriver object NULL (mytestAsynPort==NULL).\n");
    return(asynError);
  }

  if(!pPrintOutAsynUser){
    printf("ecmcAsynPortDriverAddParameter: ERROR: asynUser trace object NULL (pPrintOutAsynUser==NULL).\n");
    return(asynError);
  }

  if (0 != strcmp(mytestAsynPort->portName,portName)){
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: Port name missmatch. Desired port: %s not accessible. Accessible port: %s.\n",portName,mytestAsynPort->portName);
    return(asynError);
  }

  if(parameterCounter>=maxParameters){
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: asynPortDriverObject full (max allowed number of parameters = %i).\n",maxParameters);
    return(asynError);
  }

  //Check if EtherCAT memorymap (ECMC)
  char buffer[1024];
  int nvals = sscanf(idString, "ec.mm.%s",buffer);
  if (nvals == 1) {
    dataAccessType=ECMC_ASYN_ECMM;
  }

  //Check if EtherCAT EtherCAT entry
  int slave=-10;
  nvals = sscanf(idString, "ec.s%d.%s", &slave,buffer);
  if (nvals == 2){
    dataAccessType=ECMC_ASYN_EC;
  }

  if(dataAccessType==ECMC_ASYN_NONE){
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: No defined data access transfer type in idString (valid ec.s<slavenumber>. or ec.mm.).\n");
    return(asynError);
  }

  //Check types
  /*
  "asynInt8ArrayIn"
  "asynInt8ArrayOut"
  "asynInt16ArrayIn"
  "asynInt16ArrayOut"
  "asynInt32ArrayIn"
  "asynInt32ArrayOut"
  "asynFloat32ArrayIn"
  "asynFloat32ArrayOut"
  "asynFloat64ArrayIn"
  "asynFloat64ArrayOut"
  asynParamInt8Array
  asynParamInt16Array
  asynParamInt32Array
  asynParamFloat32Array
  asynParamFloat64Array
  */

  int asynType=-10;
  int res=strcmp(asynTypeString,"asynInt32");
  if (res == 0) {
    asynType=asynParamInt32;
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynParamInt32).\n",idString);
  }

  res=strcmp(asynTypeString,"asynFloat64");
  if (res == 0) {
    asynType=asynParamFloat64;
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynParamFloat64).\n",idString);
  }

  res=strcmp(asynTypeString,"asynInt8ArrayIn");
  if (res == 0) {
    asynType=asynParamInt8Array;
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynInt8ArrayIn).\n",idString);
  }

  res=strcmp(asynTypeString,"asynInt16ArrayIn");
  if (res == 0) {
    asynType=asynParamInt16Array;
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDrivealiasrAddParameter: INFO: Adding parameter: %s (asynInt16ArrayIn).\n",idString);
  }

  res=strcmp(asynTypeString,"asynInt32ArrayIn");
  if (res == 0) {
    asynType=asynParamInt32Array;
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynInt32ArrayIn).\n",idString);
  }

  res=strcmp(asynTypeString,"asynFloat32ArrayIn");
  if (res == 0) {
    asynType=asynParamFloat32Array;
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynFloat32ArrayIn).\n",idString);
  }

  res=strcmp(asynTypeString,"asynFloat64ArrayIn");
  if (res == 0) {
    asynType=asynParamFloat64Array;
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynFloat64ArrayIn).\n",idString);
  }

  if(asynType<=0){
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: Parameter type not supported (use asynParamInt32 or asynParamFloat64).\n");
    return(asynError);
  }

  int errorCode=10;
  switch(dataAccessType){
    case ECMC_ASYN_NONE:
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: No defined data access transfer type in alias (valid EC<slavenumber> or ECMM).\n");
      return asynError;
      break;
    case ECMC_ASYN_EC:
      errorCode=linkEcEntryToAsynParameter(mytestAsynPort,idString,asynType,skipCycles);
      break;
    case ECMC_ASYN_ECMM:
      errorCode=linkEcMemMapToAsynParameter(mytestAsynPort,idString,asynType,skipCycles);
      break;
    case ECMC_ASYN_AX:
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: No defined data access transfer type in alias (valid EC<slavenumber> or ECMM).\n");
      return asynError;
      break;
  }

  if(errorCode){
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",idString,errorCode);
    return asynError;
  }

  parameterCounter++;
  asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",idString);

  return asynSuccess;
}

/* EPICS iocsh shell command:  ecmcAsynPortDriverAddParameter*/
static const iocshArg initArg0_2 = { "port name",iocshArgString};
static const iocshArg initArg1_2 = { "id string",iocshArgString};
static const iocshArg initArg2_2 = { "asynType",iocshArgString};
static const iocshArg initArg3_2 = { "skipCycles",iocshArgInt};

static const iocshArg * const initArgs_2[] = {&initArg0_2,
                                              &initArg1_2,
					      &initArg2_2,
					      &initArg3_2
                                              };

static const iocshFuncDef initFuncDef_2 = {"ecmcAsynPortDriverAddParameter",4,initArgs_2};
static void initCallFunc_2(const iocshArgBuf *args)
{
  ecmcAsynPortDriverAddParameter(args[0].sval, args[1].sval,args[2].sval, args[3].ival);
}

void ecmcAsynPortDriverRegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
    iocshRegister(&initFuncDef_2,initCallFunc_2);
}
epicsExportRegistrar(ecmcAsynPortDriverRegister);

}



