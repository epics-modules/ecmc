/*
 * ecmcAsynPortDriver.cpp
 * 
 * Asyn driver that inherits from the asynPortDriver class to demonstrate its use.
 * It simulates a digital scope looking at a 1kHz 1000-point noisy sine wave.  Controls are
 * provided for time/division, volts/division, volt offset, trigger delay, noise amplitude, update time,
 * and run/stop.
 * Readbacks are provides for the waveform data, min, max and mean values.
 *
 * Author: Mark Rivers
 *
 * Created Feb. 5, 2009
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>

#include "ecmcAsynPortDriver.h"
#include <epicsExport.h>

static const char *driverName="ecmcAsynPortDriver";
void simTask(void *drvPvt);

/** Constructor for the ecmcAsynPortDriver class.
  * Calls constructor for the asynPortDriver base class.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] maxPoints The maximum  number of points in the volt and time arrays */
ecmcAsynPortDriver::ecmcAsynPortDriver(const char *portName/*, int maxPoints*/,int paramTableSize,int autoConnect)
   : asynPortDriver(portName, 
                    1, /* maxAddr */ 
		    paramTableSize,
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynEnumMask | asynDrvUserMask | asynOctetMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynEnumMask | asynOctetMask,  /* Interrupt mask */
                    0, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
		    autoConnect, /* Autoconnect */
                    0, /* Default priority */
                    0) /* Default stack size*/    
{
    asynStatus status;
    const char *functionName = "ecmcAsynPortDriver";
    eventId_ = epicsEventCreate(epicsEventEmpty);
    
    /* Create the thread that computes the waveforms in the background */
    status = (asynStatus)(epicsThreadCreate("ecmcAsynPortDriverTask",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::simTask,
                          this) == NULL);
    if (status) {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }
}

void simTask(void *drvPvt)
{
    ecmcAsynPortDriver *pPvt = (ecmcAsynPortDriver *)drvPvt;
    
    pPvt->simTask();
}

/** Simulation task that runs as a separate thread.  When the P_Run parameter is set to 1
  * to rub the simulation it computes a 1 kHz sine wave with 1V amplitude and user-controllable
  * noise, and displays it on
  * a simulated scope.  It computes waveforms for the X (time) and Y (volt) axes, and computes
  * statistics about the waveform. */
void ecmcAsynPortDriver::simTask(void)
{
    /* This thread computes the waveform and does callbacks with it */

    lock();
    /* Loop forever */    
    while (1) {
        // Release the lock while we wait for a command to start or wait for updateTime
        unlock();
        if (1) epicsEventWaitWithTimeout(eventId_, 0.5);
        else     (void) epicsEventWait(eventId_);
        // Take the lock again
        lock(); 
        int tempppp=0;
        getIntegerParam(0, &tempppp);
        setIntegerParam(0,tempppp+1);
        callParamCallbacks();
    }
}

/** Called when asyn clients call pasynInt32->write().
  * This function sends a signal to the simTask thread if the value of P_Run has changed.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
/*asynStatus ecmcAsynPortDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    const char* functionName = "writeInt32";

     Set the parameter in the parameter library.
    status = (asynStatus) setIntegerParam(function, value);
    
     Fetch the parameter string name for possible use in debugging
    getParamName(function, &paramName);

    if (function == P_Run) {
         If run was set then wake up the simulation task
        if (value) epicsEventSignal(eventId_);
    } 
    else if (function == P_VertGainSelect) {
        setVertGain();
    }
    else if (function == P_VoltsPerDivSelect) {
        setVoltsPerDiv();
    }
    else if (function == P_TimePerDivSelect) {
        setTimePerDiv();
    }
    else {
         All other parameters just get set in parameter list, no need to
         * act on them here
    }
    
     Do callbacks so higher layers see any changes
    status = (asynStatus) callParamCallbacks();
    
    if (status) 
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%d", 
                  driverName, functionName, status, function, paramName, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%d\n", 
              driverName, functionName, function, paramName, value);
    return status;
}*/

/** Called when asyn clients call pasynFloat64->write().
  * This function sends a signal to the simTask thread if the value of P_UpdateTime has changed.
  * For all  parameters it  sets the value in the parameter library and calls any registered callbacks.
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
/*asynStatus ecmcAsynPortDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int run;
    const char *paramName;
    const char* functionName = "writeFloat64";

     Set the parameter in the parameter library.
    status = (asynStatus) setDoubleParam(function, value);

     Fetch the parameter string name for possible use in debugging
    getParamName(function, &paramName);

    if (function == P_UpdateTime) {
         Make sure the update time is valid. If not change it and put back in parameter library
        if (value < MIN_UPDATE_TIME) {
            asynPrint(pasynUser, ASYN_TRACE_WARNING,
                "%s:%s: warning, update time too small, changed from %f to %f\n", 
                driverName, functionName, value, MIN_UPDATE_TIME);
            value = MIN_UPDATE_TIME;
            setDoubleParam(P_UpdateTime, value);
        }
         If the update time has changed and we are running then wake up the simulation task
        getIntegerParam(P_Run, &run);
        if (run) epicsEventSignal(eventId_);
    } else {
         All other parameters just get set in parameter list, no need to
         * act on them here
    }
    
     Do callbacks so higher layers see any changes
    status = (asynStatus) callParamCallbacks();
    
    if (status) 
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%f", 
                  driverName, functionName, status, function, paramName, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%f\n", 
              driverName, functionName, function, paramName, value);
    return status;
}*/

/** Called when asyn clients call pasynFloat64Array->read().
  * Returns the value of the P_Waveform or P_TimeBase arrays.  
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Pointer to the array to read.
  * \param[in] nElements Number of elements to read.
  * \param[out] nIn Number of elements actually read. */
/*asynStatus ecmcAsynPortDriver::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                         size_t nElements, size_t *nIn)
{
    int function = pasynUser->reason;
    size_t ncopy;
    int itemp;
    asynStatus status = asynSuccess;
    epicsTimeStamp timeStamp;
    const char *functionName = "readFloat64Array";

    getTimeStamp(&timeStamp);
    pasynUser->timestamp = timeStamp;
    getIntegerParam(P_MaxPoints, &itemp); ncopy = itemp;
    if (nElements < ncopy) ncopy = nElements;
    if (function == P_Waveform) {
        memcpy(value, pData_, ncopy*sizeof(epicsFloat64));
        *nIn = ncopy;
    }
    else if (function == P_TimeBase) {
        memcpy(value, pTimeBase_, ncopy*sizeof(epicsFloat64));
        *nIn = ncopy;
    }
    if (status) 
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d", 
                  driverName, functionName, status, function);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d\n", 
              driverName, functionName, function);
    return status;
}*/
    
/*asynStatus ecmcAsynPortDriver::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], size_t nElements, size_t *nIn)
{
    int function = pasynUser->reason;
    size_t i;

    if (function == P_VoltsPerDivSelect) {
        for (i=0; ((i<NUM_VERT_SELECTIONS) && (i<nElements)); i++) {
            if (strings[i]) free(strings[i]);
            strings[i] = epicsStrDup(voltsPerDivStrings_[i]);
            values[i] = voltsPerDivValues_[i];
            severities[i] = 0;
        }
    }
    else {
        *nIn = 0;
        return asynError;
    }
    *nIn = i;
    return asynSuccess;   
}*/

/*void ecmcAsynPortDriver::setVertGain()
{
    int igain, i;
    double gain;
    
    getIntegerParam(P_VertGainSelect, &igain);
    gain = igain;
    setDoubleParam(P_VertGain, gain);
    for (i=0; i<NUM_VERT_SELECTIONS; i++) {
        epicsSnprintf(voltsPerDivStrings_[i], MAX_ENUM_STRING_SIZE, "%.2f", allVoltsPerDivSelections[i] / gain);
        // The values are in mV
        voltsPerDivValues_[i] = (int)(allVoltsPerDivSelections[i] / gain * 1000. + 0.5);
    }
    doCallbacksEnum(voltsPerDivStrings_, voltsPerDivValues_, voltsPerDivSeverities_, NUM_VERT_SELECTIONS, P_VoltsPerDivSelect, 0);
}*/

/*void ecmcAsynPortDriver::setVoltsPerDiv()
{
    int mVPerDiv;
    
    // Integer volts are in mV
    getIntegerParam(P_VoltsPerDivSelect, &mVPerDiv);
    setDoubleParam(P_VoltsPerDiv, mVPerDiv / 1000.);
}*/

/*void ecmcAsynPortDriver::setTimePerDiv()
{
    int microSecPerDiv;
    
    // Integer times are in microseconds
    getIntegerParam(P_TimePerDivSelect, &microSecPerDiv);
    setDoubleParam(P_TimePerDiv, microSecPerDiv / 1000000.);
}*/

/* Configuration routine.  Called directly, or from the iocsh function below */

extern "C" {

static ecmcAsynPortDriver *mytestAsynPort;
static int maxParameters;
static int parameterCounter;

//ecmcAsynPortDriver(const char *portName, int maxPoints,int paramTableSize,int autoConnect)
/** EPICS iocsh callable function to call constructor for the ecmcAsynPortDriver class.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] maxPoints The maximum  number of points in the volt and time arrays */
int ecmcAsynPortDriverConfigure(const char *portName,int paramTableSize,int autoConnect)
{
  parameterCounter=0;
  maxParameters=paramTableSize;
  mytestAsynPort=new ecmcAsynPortDriver(portName,paramTableSize,autoConnect);
  if(mytestAsynPort){
    printf("INFO: New AsynPortDriver success (%s,%i,%i%i).",portName,1000,paramTableSize,autoConnect);
    return(asynSuccess);
  }
  else{
    printf("ERROR: New AsynPortDriver failed.");
    return(asynError);
  }
}

/* EPICS iocsh shell command: ecmcAsynPortDriverConfigure*/

static const iocshArg initArg0 = { "port name",iocshArgString};
static const iocshArg initArg1 = { "Parameter table size",iocshArgInt};
static const iocshArg initArg2 = { "auto conncet",iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0,
                                            &initArg1,
                                            &initArg2};
static const iocshFuncDef initFuncDef = {"ecmcAsynPortDriverConfigure",3,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    ecmcAsynPortDriverConfigure(args[0].sval, args[1].ival,args[2].ival);
}

//****************************** Add parameter
int ecmcAsynPortDriverAddParameter(const char *parName, int asynType)
{
  asynStatus status;

  if(!mytestAsynPort){
    printf("ERROR: asynPortDriver object NULL (mytestAsynPort==NULL)..\n");
    return(asynError);
  }

  if(parameterCounter>=maxParameters){
    printf("ERROR: asynPortDriverObject full (max allowed number of parameters = %i).\n",maxParameters);
    return(asynError);
  }

  switch(asynType){
    case asynParamNotDefined:
      printf("ERROR:: Parameter type for %s not defined (asynParamNotDefined).\n",parName);
      return(asynError);
    case asynParamInt32:
      printf("INFO: Adding parameter: %s (asynParamInt32).\n",parName);
      break;
    case asynParamUInt32Digital:
      printf("INFO: Adding parameter: %s (asynParamUInt32Digital).\n",parName);
      break;
    case asynParamFloat64:
      printf("INFO: Adding parameter: %s (asynParamFloat64).\n",parName);
      break;
    case asynParamOctet:
      printf("INFO: Adding parameter: %s (asynParamOctet).\n",parName);
      break;
    case asynParamInt8Array:
      printf("INFO: Adding parameter: %s (asynParamInt8Array).\n",parName);
      break;
    case asynParamInt16Array:
      printf("INFO: Adding parameter: %s (asynParamInt16Array).\n",parName);
      break;
    case asynParamInt32Array:
      printf("INFO: Adding parameter: %s (asynParamInt32Array).\n",parName);
      break;
    case asynParamFloat32Array:
      printf("INFO: Adding parameter: %s (asynParamFloat32Array).\n",parName);
      break;
    case asynParamFloat64Array:
      printf("INFO: Adding parameter: %s (asynParamFloat64Array).\n",parName);
      break;
    case asynParamGenericPointer:
      printf("INFO: Adding parameter: %s (asynParamGenericPointer).\n",parName);
      break;
    default:
      printf("ERROR: Parameter type %i not defined. Add parameter %s failed.\n",asynType,parName);
      return(asynError);
  }

  int index=0;
  status = mytestAsynPort->createParam(parName,(asynParamType)asynType,&index);

  if(status==asynSuccess){
    parameterCounter++;
    printf("INFO: Parameter %s added successfully at index %i.\n",parName,index);
  }
  else{
    printf("ERROR: Add parameter %s failed.\n",parName);
  }
  return(status);
}

/* EPICS iocsh shell command:  ecmcAsynPortDriverAddParameter*/

static const iocshArg initArg0_2 = { "parName",iocshArgString};
static const iocshArg initArg1_2 = { "asynType",iocshArgInt};
static const iocshArg * const initArgs_2[] = {&initArg0_2,
                                              &initArg1_2};
static const iocshFuncDef initFuncDef_2 = {"ecmcAsynPortDriverAddParameter",2,initArgs_2};
static void initCallFunc_2(const iocshArgBuf *args)
{
  ecmcAsynPortDriverAddParameter(args[0].sval, args[1].ival);
}

void ecmcAsynPortDriverRegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
    iocshRegister(&initFuncDef_2,initCallFunc_2);
}
epicsExportRegistrar(ecmcAsynPortDriverRegister);

}
