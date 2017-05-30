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

//static const char *driverName="ecmcAsynPortDriver";
void simTask(void *drvPvt);

/** Constructor for the ecmcAsynPortDriver class.
  * Calls constructor for the asynPortDriver base class.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] maxPoints The maximum  number of points in the volt and time arrays */
ecmcAsynPortDriver::ecmcAsynPortDriver(const char *portName/*, int maxPoints*/,int paramTableSize,int autoConnect,int priority)
   : asynPortDriver(portName, 
                    1, /* maxAddr */ 
		    paramTableSize,
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynEnumMask | asynDrvUserMask | asynOctetMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynEnumMask | asynOctetMask,  /* Interrupt mask */
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
  lock();
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
  unlock();
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
    pPrintOutAsynUser = pasynManager->createAsynUser(0, 0);
    printf("ecmcAsynPortDriverAddParameter: INFO: New AsynPortDriver success (%s,%i,%i,%i).",portName,paramTableSize,disableAutoConnect==0,priority);
    return(asynSuccess);
  }
  else{
    printf("ecmcAsynPortDriverAddParameter: ERROR: New AsynPortDriver failed.");
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
int ecmcAsynPortDriverAddParameter(const char *portName, int slaveNumber,const char *alias, int asynType)
{
  if (0 != strcmp(mytestAsynPort->portName,portName)){
    printf("ecmcAsynPortDriverAddParameter: ERROR: Port name missmatch. Desired port: %s not accessible. Accessible port: %s.\n",portName,mytestAsynPort->portName);
    return(asynError);
  }

  if(!mytestAsynPort){
    printf("ecmcAsynPortDriverAddParameter: ERROR: asynPortDriver object NULL (mytestAsynPort==NULL)..\n");
    return(asynError);
  }

  if(parameterCounter>=maxParameters){
    printf("ecmcAsynPortDriverAddParameter: ERROR: asynPortDriverObject full (max allowed number of parameters = %i).\n",maxParameters);
    return(asynError);
  }

  switch(asynType){
    case asynParamNotDefined:
      printf("ecmcAsynPortDriverAddParameter: ERROR:: Parameter type for %s not defined (asynParamNotDefined).\n",alias);
      return(asynError);
    case asynParamInt32:
      printf("ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynParamInt32).\n",alias);
      break;
    case asynParamUInt32Digital:
      printf("ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynParamUInt32Digital).\n",alias);
      break;
    case asynParamFloat64:
      printf("ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynParamFloat64).\n",alias);
      break;
    case asynParamOctet:
      printf("ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynParamOctet).\n",alias);
      break;
    case asynParamInt8Array:
      printf("ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynParamInt8Array).\n",alias);
      break;
    case asynParamInt16Array:
      printf("ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynParamInt16Array).\n",alias);
      break;
    case asynParamInt32Array:
      printf("ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynParamInt32Array).\n",alias);
      break;
    case asynParamFloat32Array:
      printf("ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynParamFloat32Array).\n",alias);
      break;
    case asynParamFloat64Array:
      printf("ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynParamFloat64Array).\n",alias);
      break;
    case asynParamGenericPointer:
      printf("ecmcAsynPortDriverAddParameter: INFO: Adding parameter: %s (asynParamGenericPointer).\n",alias);
      break;
    default:
      printf("ecmcAsynPortDriverAddParameter: ERROR: Parameter type %i not defined. Add parameter %s failed.\n",asynType,alias);
      return(asynError);
  }

  int errorCode=linkEcEntryToAsynParameter(mytestAsynPort,slaveNumber,alias,asynType);

  if(errorCode){
    printf("ecmcAsynPortDriverAddParameter: ERROR: Add parameter %s failed (0x%x).\n",alias,errorCode);
    return asynError;
  }

  parameterCounter++;
  printf("ecmcAsynPortDriverAddParameter: INFO: Parameter (alias=%s,busPosition=%d) added successfully.\n",alias,slaveNumber);

  return asynSuccess;
}

/* EPICS iocsh shell command:  ecmcAsynPortDriverAddParameter*/

static const iocshArg initArg0_2 = { "port name",iocshArgString};
static const iocshArg initArg1_2 = { "slave number",iocshArgInt};
static const iocshArg initArg2_2 = { "alias",iocshArgString};
static const iocshArg initArg3_2 = { "asynType",iocshArgInt};
static const iocshArg * const initArgs_2[] = {&initArg0_2,
                                              &initArg1_2,
					      &initArg2_2,
					      &initArg3_2};
static const iocshFuncDef initFuncDef_2 = {"ecmcAsynPortDriverAddParameter",4,initArgs_2};
static void initCallFunc_2(const iocshArgBuf *args)
{
  ecmcAsynPortDriverAddParameter(args[0].sval, args[1].ival,args[2].sval, args[3].ival);
}

void ecmcAsynPortDriverRegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
    iocshRegister(&initFuncDef_2,initCallFunc_2);
}
epicsExportRegistrar(ecmcAsynPortDriverRegister);

}
