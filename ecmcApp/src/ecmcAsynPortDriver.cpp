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
  int masterIndex=0;
  int nvals = sscanf(paramName, "ec%d.s%d.%s",&masterIndex, &slavePosition,aliasBuffer);
  if(nvals!=3){
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
  int masterIndex=0;
  int nvals = sscanf(paramName, "ec%d.s%d.%s",&masterIndex, &slavePosition,aliasBuffer);
  if(nvals!=3){
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

  uint64_t *temp=(uint64_t*)&value;
  int errorId=writeEcEntryIDString(slavePosition,aliasBuffer,*temp);//static_cast<uint64_t>(value));
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
  int masterIndex=0;
  int nvals = sscanf(paramName, "ec%d.mm.%s",&masterIndex,buffer);
  if (nvals == 2) {
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

static ecmcAsynPortDriver *ecmcAsynPortObj;
static int maxParameters;
static int parameterCounter;
/* global asynUser for Printing */
asynUser *pPrintOutAsynUser;


/** EPICS iocsh callable function to call constructor for the ecmcAsynPortDriver class.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] paramTableSize The max number of parameters.
  * \param[in] priority Priority.
  * \param[in] disableAutoConnect Disable auto connect */
int ecmcAsynPortDriverConfigure(const char *portName,int paramTableSize,int priority, int disableAutoConnect)
{
  parameterCounter=0;
  maxParameters=paramTableSize;
  ecmcAsynPortObj=new ecmcAsynPortDriver(portName,paramTableSize,disableAutoConnect==0,priority);
  if(ecmcAsynPortObj){
    asynUser *traceUser= ecmcAsynPortObj->getTraceAsynUser();
    if(!traceUser){
      printf("ecmcAsynPortDriverConfigure: ERROR: Failed to retrieve asynUser for trace. \n");
      return (asynError);
    }

    pPrintOutAsynUser = pasynManager->duplicateAsynUser(traceUser, 0, 0);

    if(!pPrintOutAsynUser){
      printf("ecmcAsynPortDriverConfigure: ERROR: Failed to duplicate asynUser for trace. \n");
      return (asynError);
    }

    int errorCode=initEcmcAsyn((void*)ecmcAsynPortObj);
    if(errorCode){
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverConfigure: ERROR: New AsynPortDriver (setAsynPort()) failed (0x%x).\n",errorCode);
      return asynError;
    }

    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverConfigure: INFO: New AsynPortDriver success (%s,%i,%i,%i).",portName,paramTableSize,disableAutoConnect==0,priority);

    return(asynSuccess);
  }
  else{
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverConfigure: ERROR: New AsynPortDriver failed.");
    return(asynError);
  }
}

//Parse asyn datatype

static int parseAsynDataType( const char *asynTypeString)
{

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
  asynParamFloat32ArrayinitEcmcAsyn(void* asynPortObject)
  asynParamFloat64Array
  */

  int asynType=-10;
  int res=strcmp(asynTypeString,"asynInt32");
  if (res == 0) {
    asynType=asynParamInt32;
  }

  res=strcmp(asynTypeString,"asynFloat64");
  if (res == 0) {
    asynType=asynParamFloat64;
  }

  res=strcmp(asynTypeString,"asynInt8ArrayIn");
  if (res == 0) {
    asynType=asynParamInt8Array;
  }

  res=strcmp(asynTypeString,"asynInt16ArrayIn");
  if (res == 0) {
    asynType=asynParamInt16Array;
  }

  res=strcmp(asynTypeString,"asynInt32ArrayIn");
  if (res == 0) {
    asynType=asynParamInt32Array;
  }

  res=strcmp(asynTypeString,"asynFloat32ArrayIn");
  if (res == 0) {
    asynType=asynParamFloat32Array;
  }

  res=strcmp(asynTypeString,"asynFloat64ArrayIn");
  if (res == 0) {
    asynType=asynParamFloat64Array;
  }

  return asynType;
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

/** \breif EPICS iocsh command for adding asyn-parameter(s)
 * linked to objects in ECMC.\n
 *
 * Fast access of different types of objects in ECMC via asyn parameters is
 * possible:
 * 1. EtherCAT entries\n
 * 2. EtherCAT memory maps\n
 * 3. EtherCAT master diagnostics\n
 * 4. EtherCAT slave diagnostics\n
 * 5. Motion axis information\n
 * 6. Timing diagnostics.\n
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
 *             (access to ethercat entry, "default" cannot be used as
 *             <ethercat entry id> ).\n
 *             idString = thread.default  (set of timing diag params).\n
 *             idString = ax<axis index>.default  (set of motion diag params).\n
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
 * \note Example: Generate diag asyn parameters for timing
 *  skip cycles =9 (skip 9 cycles then update=> every 10 value will
 * trigger asyn update):
 * ecmcAsynPortDriverAddParameter(ASYNPORT,thread.default,"asynInt32",9)
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
 * \note Example: Generate diag asyn parameters motion axis 8
 *  skip cycles =9 (skip 9 cycles then update=> every 10 value will
 * trigger asyn update):
 * ecmcAsynPortDriverAddParameter(ASYNPORT,ax8.default,"asynInt32",9)
 *
 */
int ecmcAsynPortDriverAddParameter(const char *portName, const char *idString, const char *asynTypeString, int skipCycles)
{
  if(!ecmcAsynPortObj){
    printf("ecmcAsynPortDriverAddParameter: ERROR: asynPortDriver object NULL (ecmcAsynPortObj==NULL).\n");
    return(asynError);
  }

  if(!pPrintOutAsynUser){
    printf("ecmcAsynPortDriverAddParameter: ERROR: asynUser trace object NULL (pPrintOutAsynUser==NULL).\n");
    return(asynError);
  }

  if (0 != strcmp(ecmcAsynPortObj->portName,portName)){
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: Port name mismatch. Desired port: %s not accessible. Accessible port: %s.\n",portName,ecmcAsynPortObj->portName);
    return(asynError);
  }

  if(parameterCounter>=maxParameters){
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: asynPortDriverObject full (max allowed number of parameters = %i).\n",maxParameters);
    return(asynError);
  }

  int errorCode=0;

  //Check if EtherCAT memorymap
  char buffer[1024];
  int masterIndex=0;
  int nvals = sscanf(idString, "ec%d.mm.%s",&masterIndex,buffer);
  if (nvals == 2) {
    int asynType=parseAsynDataType(asynTypeString);

    if(asynType<=0){
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: Parameter type not supported (use asynParamInt32 or asynParamFloat64).\n");
      return(asynError);
    }

    errorCode=linkEcMemMapToAsynParameter(masterIndex,idString,asynType,skipCycles);

    if(errorCode){
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",idString,errorCode);
      return asynError;
    }

    parameterCounter++;
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",idString);
    return asynSuccess;
  }

  //Check if default parameters for ec slave
  int busPosition=-10;
  nvals = sscanf(idString, "ec%d.s%d.%s",&masterIndex,&busPosition,buffer);
  if (nvals == 3 && strcmp("default",buffer)==0){
    errorCode=addDefaultAsynEcSlave(masterIndex,busPosition,skipCycles,masterIndex);
    if(errorCode){
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",idString,errorCode);
      return asynError;
    }
    parameterCounter++;
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",idString);
    return asynSuccess;
  }

  //Check if EtherCAT EtherCAT entry
  busPosition=-10;
  nvals = sscanf(idString, "ec%d.s%d.%s",&masterIndex,&busPosition,buffer);
  if (nvals == 3){
    int asynType=parseAsynDataType(asynTypeString);
    if(asynType<=0){
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: Parameter type not supported (use asynParamInt32 or asynParamFloat64).\n");
      return(asynError);
    }
    errorCode=linkEcEntryToAsynParameter(masterIndex,busPosition,idString,asynType,skipCycles);
    if(errorCode){
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",idString,errorCode);
      return asynError;
    }
    parameterCounter++;
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",idString);
    return asynSuccess;
  }

  //Check if default parameters for thread
  nvals=strcmp(idString,"thread.default");
  if (nvals == 0) {
    errorCode=addDefaultAsynThread(1,skipCycles);
    if(errorCode){
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",idString,errorCode);
      return asynError;
    }
    parameterCounter++;
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",idString);
    return asynSuccess;
  }

  //Check if default parameters for axis
  int axisIndex=0;
  nvals = sscanf(idString, "ax%d.default",&axisIndex);
  if (nvals == 1){
    errorCode=addDefaultAsynAxis(1,axisIndex,skipCycles);
    if(errorCode){
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",idString,errorCode);
      return asynError;
    }
    parameterCounter++;
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",idString);
    return asynSuccess;
  }

  //Check if default parameters for ec
   nvals = sscanf(idString, "ec%d.default",&masterIndex);
   if (nvals == 1){
     errorCode=addDefaultAsynEc(masterIndex,1,skipCycles);
     if(errorCode){
       asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",idString,errorCode);
       return asynError;
     }
     parameterCounter++;
     asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,"ecmcAsynPortDriverAddParameter: INFO: Parameter with alias=%s added successfully.\n",idString);
     return asynSuccess;
   }

  //No parameter assigned...
  asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,"ecmcAsynPortDriverAddParameter:\
      ERROR: No defined data access transfer type in idString. Vaild syntax:\
      ec<master>.default, ec<master>.s<slavenumber>.defualt, \
      ec<master>.s<slavenumber>.<alias>, ec<master>.mm.<alias>, thread.default\
      ax<index>.default.\n");
  return asynError;
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



