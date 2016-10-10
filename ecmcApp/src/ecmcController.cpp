/*
FILENAME... ecmcController.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "ecmcController.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

const static char * const strEcmcConfigController = "ecmcConfigController";
const static char *const strCtrlReset = ".ctrl.ErrRst";

/** Creates a new ecmcController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MotorPortName     The name of the drvAsynSerialPort that was created previously to connect to the ecmc controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
ecmcController::ecmcController(const char *portName, const char *MotorPortName, int numAxes,
                               double movingPollPeriod, double idlePollPeriod, const char *optionsStr)
  :  asynMotorController(portName, numAxes, NUM_VIRTUAL_MOTOR_PARAMS,
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;

  memset(&ctrlLocal, 0, sizeof(ctrlLocal));
  /* Controller */
  createParam(ecmcMCUErrMsgString,     asynParamOctet,       &ecmcMCUErrMsg_);
  /* Per axis */
  createParam(ecmcErrString,           asynParamInt32,       &ecmcErr_);
  createParam(ecmcErrIdString,         asynParamInt32,       &ecmcErrId_);
  createParam(ecmcErrMsgString,        asynParamOctet,       &ecmcErrMsg_);

  createParam(ecmcProcHomString,       asynParamInt32,       &ecmcProcHom_);
  createParam(ecmcErrRstString,        asynParamInt32,       &ecmcErrRst_);

#ifdef CREATE_MOTOR_REC_RESOLUTION
  /* Latest asynMotorController does this, but not the version in 6.81 (or 6.9x) */
  createParam(motorRecResolutionString,        asynParamFloat64,      &motorRecResolution_);
  createParam(motorRecDirectionString,           asynParamInt32,      &motorRecDirection_);
  createParam(motorRecOffsetString,            asynParamFloat64,      &motorRecOffset_);
#endif

  asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "optionsStr=%s\n", optionsStr ? optionsStr : "NULL");

  if (optionsStr && optionsStr[0]) {
    const char * const local_is_str = "local=";

    char *pOptions = strdup(optionsStr);
    char *pThisOption = pOptions;
    char *pNextOption = pOptions;

    while (pNextOption && pNextOption[0]) {
      pNextOption = strchr(pNextOption, ';');
      if (pNextOption) {
        *pNextOption = '\0'; /* Terminate */
        pNextOption++;       /* Jump to (possible) next */
      }
      if (!strncmp(pThisOption, local_is_str, strlen(local_is_str))) {
        pThisOption += strlen(local_is_str);
        if (!strcmp("true", pThisOption)) ctrlLocal.local_no_ASYN_ = 1;
      }
      pThisOption = pNextOption;
    }
    free(pOptions);
  }
  /* End of configStr */

  /* Connect to ecmc controller */
  status = pasynOctetSyncIO->connect(MotorPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "cannot connect to motor controller\n");
  }
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new ecmcController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MotorPortName  The name of the drvAsynIPPPort that was created previously to connect to the ecmc controller
  * \param[in] numAxes           The number of axes that this controller supports (0 is not used)
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int ecmcCreateController(const char *portName, const char *MotorPortName, int numAxes,
                                    int movingPollPeriod, int idlePollPeriod, const char *optionsStr)
{
  new ecmcController(portName, MotorPortName, 1+numAxes, movingPollPeriod/1000., idlePollPeriod/1000., optionsStr);
  return asynSuccess;
}

extern "C" int ecmcConfigController(const char *portName,
                                    const char *configStr)
{
  ecmcController *pC;

  if (!portName || !configStr)  {
    printf("%sNULL parameter\n", strEcmcConfigController);
    return asynError;
  }
  pC = (ecmcController*) findAsynPortDriver(portName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           __FILE__, __FUNCTION__, portName);
    return asynError;
  }
  return pC->configController(configStr);
}

asynStatus ecmcController::configController(const char *value)
{
  char inString[MAX_CONTROLLER_STRING_SIZE];
  size_t configStrLen = strlen(value);
  asynStatus status = asynError;

  if (!strcmp(value, strCtrlReset)) {
    ctrlLocal.hasConfigError = 0;
    setMCUErrMsg("OK");
    return asynSuccess;
  }
  if (ctrlLocal.hasConfigError) {
    printf("port %s has errors. To reset use\n %s %s %s \n",
           portName, strEcmcConfigController, portName, strCtrlReset);
    return asynError;
  }

  status = writeReadOnErrorDisconnect_C(pasynUserController_,
                                        value, configStrLen,
                                        inString, sizeof(inString));
  inString[sizeof(inString) -1] = '\0';
  if (status) {
    ctrlLocal.isConnected = 0;
  } else {
    status = checkACK(value, configStrLen, inString);
  }
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "out=%s in=\"%s\" return=%s (%d)\n",
              value, inString,
              pasynManager->strStatus(status), (int)status);
    ctrlLocal.hasConfigError = 1;
    (void)setMCUErrMsg(inString);
  } else {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_INFO,
              "out=%s in=\"%s\"\n",
              value, inString);
  }

  printf("%s\n", inString);
  return status;
}

extern "C"
asynStatus writeReadOnErrorDisconnect_C(asynUser *pasynUser,
                                        const char *outdata, size_t outlen,
                                        char *indata, size_t inlen)
{
  size_t nwrite;
  asynStatus status = asynError;
  int eomReason;
  size_t nread;
  indata[0] = '\0';
  status = pasynOctetSyncIO->writeRead(pasynUser, outdata, outlen,
                                       indata, inlen,
                                       DEFAULT_CONTROLLER_TIMEOUT,
                                       &nwrite, &nread, &eomReason);
  if ((status == asynTimeout) || (!nread && (eomReason & ASYN_EOM_END)))
  {
    asynInterface *pasynInterface = NULL;
    asynCommon     *pasynCommon = NULL;
    pasynInterface = pasynManager->findInterface(pasynUser,
                                                 asynCommonType,
                                                 0 /* FALSE */);
    if (pasynInterface) {
      pasynCommon = (asynCommon *)pasynInterface->pinterface;
      status = pasynCommon->disconnect(pasynInterface->drvPvt,
                                       pasynUser);
      if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "out=%s status=%s (%d)\n",
                  outdata, pasynManager->strStatus(status), (int)status);
      }
    } else {
      asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "pasynInterface=%p pasynCommon=%p\n",
                pasynInterface, pasynCommon);
    }
    asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "out=%s nread=%lu status=%s (%d)\n",
              outdata,(unsigned long)nread,
              pasynManager->strStatus(status), status);
    return asynError; /* TimeOut -> Error */
  }
  return status;
}

/** Writes a string to the controller and reads a response.
  * Disconnects in case of error
  */
asynStatus ecmcController::writeReadOnErrorDisconnect(void)
{
  asynStatus status = asynError;
  size_t outlen = strlen(outString_);
  inString_[0] = '\0';
  status = writeReadOnErrorDisconnect_C(pasynUserController_, outString_, outlen,
                                        inString_, sizeof(inString_));
  handleStatusChange(status);
  if (status)
  {
    return asynError;
  }
  return status;
}

extern "C"
asynStatus checkACK(const char *outdata, size_t outlen,
                    const char *indata)
{
  size_t i;
  unsigned int numOK = 1;
  int res = 1;
  for( i = 0; i < outlen; i++) {
    if (outdata[i] == ';') numOK++;
  }
  switch(numOK) {
    case 1: res = strcmp(indata, "OK");  break;
    case 2: res = strcmp(indata, "OK;OK");  break;
    case 3: res = strcmp(indata, "OK:OK;OK");  break;
    case 4: res = strcmp(indata, "OK;OK;OK;OK");  break;
    default:
    ;
  }
  return res ? asynError : asynSuccess;
}

asynStatus ecmcController::setMCUErrMsg(const char *value)
{
  asynStatus status = setStringParam(ecmcMCUErrMsg_, value);
  if (!status) status = callParamCallbacks();
  return status;
}

void ecmcController::handleStatusChange(asynStatus status)
{

  if (status && ctrlLocal.isConnected) {
    /* Connected -> Disconnected */
    int i;
    ctrlLocal.isConnected = 0;
    setMCUErrMsg("MCU Disconnected");
    for (i=0; i<numAxes_; i++) {
      ecmcAxis *pAxis=getAxis(i);
      if (!pAxis) continue;
      pAxis->handleDisconnect();
    }
  } else if (!status && !ctrlLocal.isConnected) {
    /* Disconnected -> Connected */
    ctrlLocal.isConnected = 1;
    setMCUErrMsg("MCU Cconnected");
  }
}


/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void ecmcController::report(FILE *fp, int level)
{
  fprintf(fp, "Twincat motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an ecmcAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
ecmcAxis* ecmcController::getAxis(asynUser *pasynUser)
{
  return static_cast<ecmcAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an ecmcAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
ecmcAxis* ecmcController::getAxis(int axisNo)
{
  return static_cast<ecmcAxis*>(asynMotorController::getAxis(axisNo));
}


asynStatus ecmcController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  ecmcAxis *pAxis;
  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;

  (void)pAxis->setIntegerParam(function, value);
  return asynMotorController::writeInt32(pasynUser, value);
}

/** Code for iocsh registration */
/* ecmcCreateController */
static const iocshArg ecmcCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg ecmcCreateControllerArg1 = {"EPICS ASYN TCP motor port name", iocshArgString};
static const iocshArg ecmcCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg ecmcCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg ecmcCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg ecmcCreateControllerArg5 = {"controllerOptionsStr", iocshArgString};
static const iocshArg * const ecmcCreateControllerArgs[] = {&ecmcCreateControllerArg0,
                                                            &ecmcCreateControllerArg1,
                                                            &ecmcCreateControllerArg2,
                                                            &ecmcCreateControllerArg3,
                                                            &ecmcCreateControllerArg4,
                                                            &ecmcCreateControllerArg5};
static const iocshFuncDef ecmcCreateControllerDef = {"ecmcCreateController", 6, ecmcCreateControllerArgs};
static void ecmcCreateContollerCallFunc(const iocshArgBuf *args)
{
  ecmcCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].sval);
}

/* ecmcConfigController */
static const iocshArg ecmcConfigControllerArg0 = {"Controller port name", iocshArgString};
static const iocshArg ecmcConfigControllerArg1 = {"ConfigString",         iocshArgString};
static const iocshArg * const ecmcConfigControllerArgs[] = {&ecmcConfigControllerArg0,
                                                            &ecmcConfigControllerArg1};
static const iocshFuncDef ecmcConfigControllerDef = {strEcmcConfigController, 2, ecmcConfigControllerArgs};
static void ecmcConfigContollerCallFunc(const iocshArgBuf *args)
{
  ecmcConfigController(args[0].sval, args[1].sval);
}


/* ecmcCreateAxis */
static const iocshArg ecmcCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg ecmcCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg ecmcCreateAxisArg2 = {"axisFlags", iocshArgInt};
static const iocshArg ecmcCreateAxisArg3 = {"axisOptionsStr", iocshArgString};
static const iocshArg * const ecmcCreateAxisArgs[] = {&ecmcCreateAxisArg0,
                                                      &ecmcCreateAxisArg1,
                                                      &ecmcCreateAxisArg2,
                                                      &ecmcCreateAxisArg3};
static const iocshFuncDef ecmcCreateAxisDef = {"ecmcCreateAxis", 4, ecmcCreateAxisArgs};
static void ecmcCreateAxisCallFunc(const iocshArgBuf *args)
{
  ecmcCreateAxis(args[0].sval, args[1].ival, args[2].ival, args[3].sval);
}

static void ecmcControllerRegister(void)
{
  iocshRegister(&ecmcCreateControllerDef, ecmcCreateContollerCallFunc);
  iocshRegister(&ecmcConfigControllerDef, ecmcConfigContollerCallFunc);
  iocshRegister(&ecmcCreateAxisDef,       ecmcCreateAxisCallFunc);
}

extern "C" {
  epicsExportRegistrar(ecmcControllerRegister);
}
