/**********************************************************************
 * Asyn device support for the ECMC
 **********************************************************************/

#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <osiUnistd.h>
#include <osiSock.h>
/*
 * cmd.h before all EPICS stuff, otherwise
   __attribute__((format (printf,1,2)))
   will not work
*/
#include "cmd.h"

#include <cantProceed.h>
#include <errlog.h>
#include <iocsh.h>
#include <epicsAssert.h>
#include <epicsExit.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <osiUnistd.h>

#include <epicsExport.h>
#include "asynDriver.h"
#include "asynOctet.h"
#include "asynInterposeCom.h"
#include "asynInterposeEos.h"

/*
 * This structure holds the hardware-specific information for a single
 * asyn link.
 */
typedef struct {
  char              *portName;
  asynUser          *pasynUser;        /* Not currently used */

  asynInterface      common;
  asynInterface      octet;
} ecmcController_t;

/*
 * Close a connection
 */
static void
closeConnection(asynUser *pasynUser,ecmcController_t *ecmcController_p)
{
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s close connection\n",
            ecmcController_p->portName);
}

/*Beginning of asynCommon methods*/
/*
 * Report link parameters
 */
static void
asynCommonReport(void *drvPvt, FILE *fp, int details)
{
  ecmcController_t *ecmcController_p = (ecmcController_t *)drvPvt;

  assert(ecmcController_p);
  if (details >= 1) {
    fprintf(fp, "    Port %s\n",
            ecmcController_p->portName);
  }
}

/*
 * Clean up a connection on exit
 */
static void
cleanup (void *arg)
{
  ;
}

/*
 * Create a link
 */
static asynStatus
connectIt(void *drvPvt, asynUser *pasynUser)

{
  (void)drvPvt;
  (void)pasynUser;
  return asynSuccess;
}

static asynStatus
asynCommonConnect(void *drvPvt, asynUser *pasynUser)
{
  asynStatus status = connectIt(drvPvt, pasynUser);
  if (status == asynSuccess)
    pasynManager->exceptionConnect(pasynUser);
  return status;
}

static asynStatus
asynCommonDisconnect(void *drvPvt, asynUser *pasynUser)
{
  ecmcController_t *ecmcController_p = (ecmcController_t *)drvPvt;

  assert(ecmcController_p);
  printf("DDDDDDDDD %s/%s:%d\n",
         __FILE__, __FUNCTION__, __LINE__);
  closeConnection(pasynUser, ecmcController_p);
  return asynSuccess;
}

/* asynOctet methods */
static asynStatus writeIt(void *drvPvt, asynUser *pasynUser,
                          const char *data,
                          size_t numchars,
                          size_t *nbytesTransfered)
{
  ecmcController_t *ecmcController_p = (ecmcController_t *)drvPvt;
  size_t thisWrite = 0;
  asynStatus status = asynError;

  assert(ecmcController_p);
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s write.\n", ecmcController_p->portName);
  asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, data, numchars,
              "%s write %lu\n",
              ecmcController_p->portName,
              (unsigned long)numchars);
  *nbytesTransfered = 0;

  if (numchars == 0){
    return asynSuccess;
  }
  if (!(CMDwriteIt(data, numchars))) {
    thisWrite = numchars;
    *nbytesTransfered = thisWrite;
    status = asynSuccess;
  }

  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s wrote %lu return %s.\n",
            ecmcController_p->portName,
            (unsigned long)*nbytesTransfered,
            pasynManager->strStatus(status));
  return status;
}

static asynStatus readIt(void *drvPvt,
                         asynUser *pasynUser,
                         char *data,
                         size_t maxchars,
                         size_t *nbytesTransfered,
                         int *gotEom)
{
  ecmcController_t *ecmcController_p = (ecmcController_t *)drvPvt;
  size_t thisRead = 0;
  int reason = 0;
  asynStatus status = asynSuccess;

  assert(ecmcController_p);

  /*
   * Feed what writeIt() gave us into the MCU
   */
  *data = '\0';
  if (CMDreadIt(data, maxchars)) status = asynError;
  if (status == asynSuccess) {
    thisRead = strlen(data);
    *nbytesTransfered = thisRead;
    /* May be not enough space ? */
    if (thisRead > maxchars -1)  reason |= ASYN_EOM_CNT;

    if (gotEom) *gotEom = reason;

    if (thisRead == 0 && pasynUser->timeout == 0){
      status = asynTimeout;
    }
  }
  *nbytesTransfered = thisRead;
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s thisRead=%lu data=\"%s\"\n",
            ecmcController_p->portName,
            (unsigned long)thisRead, data);
  return status;
}

/*
 * Flush pending input
 */
static asynStatus
flushIt(void *drvPvt,asynUser *pasynUser)
{
  ecmcController_t *ecmcController_p = (ecmcController_t *)drvPvt;

  assert(ecmcController_p);
  asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s flush\n",
            ecmcController_p->portName);
  return asynSuccess;
}

/*
 * Clean up a ecmcController_pController
 */
static void
ecmcController_pCleanup(ecmcController_t *ecmcController_p)
{
  if (ecmcController_p) {
    free(ecmcController_p->portName);
    free(ecmcController_p);
  }
}

/*
 * asynCommon methods
 */
static const struct asynCommon drvAsynECMCPortAsynCommon = {
  asynCommonReport,
  asynCommonConnect,
  asynCommonDisconnect
};

/*
 * Configure and register
 */
epicsShareFunc int
drvAsynECMCPortConfigure(const char *portName,
                         unsigned int priority,
                         int noAutoConnect,
                         int noProcessEos)
{
  ecmcController_t *ecmcController_p;
  asynInterface *pasynInterface;
  asynStatus status;
  size_t nbytes;
  void *allocp;
  asynOctet *pasynOctet;
  printf("%s/%s:%d %s priority=%u noAutoConnect=%d noProcessEos=%d\n",
         __FILE__, __FUNCTION__, __LINE__,
         portName ? portName : "",
         priority,
         noAutoConnect,
         noProcessEos);

  if (!portName) {
    printf("drvAsynECMCPortConfigure bad parameter %s\n",
           portName ? portName : "");
    return -1;
  }

  /* Create a driver  */
  nbytes = sizeof(ecmcController_t) + sizeof(asynOctet);
  allocp = callocMustSucceed(1, nbytes,
                             "drvAsynECMCPortConfigure()");
  ecmcController_p = (ecmcController_t *)allocp;

  pasynOctet = (asynOctet *)(ecmcController_p+1);
  ecmcController_p->portName = epicsStrDup(portName);

  /* Link with higher level routines */
  allocp = callocMustSucceed(2, sizeof *pasynInterface,
                             "drvAsynECMCPortConfigure");

  pasynInterface = (asynInterface *)allocp;
  ecmcController_p->common.interfaceType = asynCommonType;
  ecmcController_p->common.pinterface  = (void *)&drvAsynECMCPortAsynCommon;
  ecmcController_p->common.drvPvt = ecmcController_p;
  if (pasynManager->registerPort(ecmcController_p->portName,
                                 ASYN_CANBLOCK,
                                 !noAutoConnect,
                                 priority,
                                 0) != asynSuccess) {
    printf("drvAsynECMCPortConfigure: Can't register port %s\n",
           portName);
    ecmcController_pCleanup(ecmcController_p);
    return -1;
  }
  /* Register interface */
  status = pasynManager->registerInterface(ecmcController_p->portName,
                                           &ecmcController_p->common);
  if(status != asynSuccess) {
    printf("drvAsynECMCPortConfigure: Can't register interface %s\n",
           portName);
    ecmcController_pCleanup(ecmcController_p);
    return -1;
  }
  pasynOctet->read = readIt;
  pasynOctet->write = writeIt;
  pasynOctet->flush = flushIt;
  ecmcController_p->octet.interfaceType = asynOctetType;
  ecmcController_p->octet.pinterface  = pasynOctet;
  ecmcController_p->octet.drvPvt = ecmcController_p;
  status = pasynOctetBase->initialize(ecmcController_p->portName,
                                      &ecmcController_p->octet, 0, 0, 1);
  printf("XXXXXXX %s/%s:%d %s\n",
         __FILE__, __FUNCTION__, __LINE__,
         portName ? portName : "");
  if(status != asynSuccess) {
    printf("EEEEEEEEE drvAsynECMCPortConfigure: pasynOctetBase->initialize failed.\n");
    ecmcController_pCleanup(ecmcController_p);
    return -1;
  }

  asynInterposeEosConfig(ecmcController_p->portName, -1, 1, 1);

  ecmcController_p->pasynUser = pasynManager->createAsynUser(0,0);
  status = pasynManager->connectDevice(ecmcController_p->pasynUser,
                                       ecmcController_p->portName, -1);
  if(status != asynSuccess) {
    printf("connectDevice failed %s\n",
           ecmcController_p->pasynUser->errorMessage);
    cleanup(ecmcController_p);
    return -1;
  }
  /*
   * Register for cleanup
   */
  epicsAtExit(cleanup, ecmcController_p);
  return 0;
}

/*
 * IOC shell command registration
 */
static const iocshArg drvAsynECMCPortConfigureArg0 = { "port name",iocshArgString};
static const iocshArg drvAsynECMCPortConfigureArg1 = { "priority",iocshArgInt};
static const iocshArg drvAsynECMCPortConfigureArg2 = { "disable auto-connect",iocshArgInt};
static const iocshArg drvAsynECMCPortConfigureArg3 = { "noProcessEos",iocshArgInt};
static const iocshArg *drvAsynECMCPortConfigureArgs[] = {
  &drvAsynECMCPortConfigureArg0, &drvAsynECMCPortConfigureArg1,
  &drvAsynECMCPortConfigureArg2, &drvAsynECMCPortConfigureArg3};
static const iocshFuncDef drvAsynECMCPortConfigureFuncDef =
  {"drvAsynECMCPortConfigure",4,drvAsynECMCPortConfigureArgs};

static void drvAsynECMCPortConfigureCallFunc(const iocshArgBuf *args)
{
  drvAsynECMCPortConfigure(args[0].sval, args[1].ival, args[2].ival, args[3].ival);
}

/*
 * This routine is called before multitasking has started, so there's
 * no race condition in the test/set of firstTime.
 */
static void
drvAsynECMCPortRegisterCommands(void)
{
  iocshRegister(&drvAsynECMCPortConfigureFuncDef,drvAsynECMCPortConfigureCallFunc);
}

extern "C" {
  epicsExportRegistrar(drvAsynECMCPortRegisterCommands);
}
