/*
 */

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

#include "../main/gitversion.h"
#include "ecmcOctetIF.h"
#include "ecmcCmdParser.h"
#include "ecmcAsynPortDriver.h"

#include "../main/ecmcMainThread.h"
#include "../ethercat/ecmcEthercat.h"
#include "../main/ecmcGeneral.h"
#include "../com/ecmcCom.h"


static const char *driverName = "ecmcAsynPortDriver";

/** Constructor for the ecmcAsynPortDriver class.
  * Calls constructor for the asynPortDriver base class.
  * \param[in] portName The name of the asyn port driver to be created. */
ecmcAsynPortDriver::ecmcAsynPortDriver(
  const char *portName  /*, int maxPoints*/,
  int         paramTableSize,
  int         autoConnect,
  int         priority)
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
                   ASYN_CANBLOCK,
                   autoConnect,
                   /* Autoconnect */
                   priority,
                   /* Default priority */
                   0) { /* Default stack size*/
  initVars();
  const char* functionName = "ecmcAsynPortDriver";
  allowRtThreadCom_ = 1;  // Allow at startup (RT thread not started)
  pEcmcParamArray_  = new ecmcParamInfo*[paramTableSize];
  paramTableSize_   = paramTableSize;
  autoConnect_      = autoConnect;
  priority_         = priority;

    if(paramTableSize_<1){  //If paramTableSize_==1 then only stream device or motor record can use the driver through the "default access" param below.
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Param table size to small: %d\n", driverName, functionName,paramTableSize_);
    return;
  }

  //Add first param for other access (like motor record or stream device).
  int index;
  asynStatus status=createParam("Default access",asynParamNotDefined,&index);
  if(status!=asynSuccess){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: createParam for default access failed.\n", driverName, functionName);
    return;
  }
  ecmcParamInfo *paramInfo=new ecmcParamInfo();
  memset(paramInfo,0,sizeof(ecmcParamInfo));
  paramInfo->recordName=strdup("Any record");
  paramInfo->recordType=strdup("No type");
  paramInfo->scan=strdup("No scan");
  paramInfo->dtyp=strdup("No dtyp");
  paramInfo->inp=strdup("No inp");
  paramInfo->out=strdup("No out");
  paramInfo->drvInfo=strdup("No drvinfo");
  paramInfo->asynType=asynParamNotDefined;
  paramInfo->paramIndex=index;  //also used as hUser for ads callback
  pEcmcParamArray_[0]=paramInfo;
  ecmcParamArrayCount_++;

  if(status!=asynSuccess){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: createParam for default access failed.\n", driverName, functionName);
    return;
  }
}

void ecmcAsynPortDriver::initVars() {
  allowRtThreadCom_      = 0;
  pEcmcParamArray_       = NULL;
  ecmcParamArrayCount_   = 0;
  paramTableSize_        = 0;
  defaultSampleTimeMS_   = 0;
  defaultMaxDelayTimeMS_ = 0;
  defaultTimeSource_     = ECMC_TIME_BASE_ECMC;
  autoConnect_           = 0;
  priority_              = 0;
}

asynStatus ecmcAsynPortDriver::readOctet(asynUser *pasynUser,
                                         char     *value,
                                         size_t    maxChars,
                                         size_t   *nActual,
                                         int      *eomReason) {
  size_t thisRead   = 0;
  int    reason     = 0;
  asynStatus status = asynSuccess;

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

    // printf("readOctet: thisread: %d\n",thisRead);
    if (thisRead > maxChars - 1) {
      reason |= ASYN_EOM_CNT;
    } else {
      reason |= ASYN_EOM_EOS;
    }

    if ((thisRead == 0) && (pasynUser->timeout == 0)) {
      status = asynTimeout;
    }
  } else {
    printf("FAIL");
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

asynStatus ecmcAsynPortDriver::writeInt32(asynUser  *pasynUser,
                                          epicsInt32 value) {
  int function = pasynUser->reason;
  const char *paramName;
  const char *functionName = "writeInt32";

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  // Check if error reset
  if (strcmp(paramName, "ecmc.error.reset") == 0) {
    controllerErrorReset();
    return asynSuccess;
  }

  char  buffer[1024];
  char *aliasBuffer   = &buffer[0];
  int   slavePosition = -10;
  int   masterIndex   = 0;
  int   nvals         = sscanf(paramName,
                               "ec%d.s%d.%s",
                               &masterIndex,
                               &slavePosition,
                               aliasBuffer);

  if (nvals != 3) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, parameter name not valid: %s.\n",
              driverName, functionName, paramName);
    return asynError;
  }

  if (slavePosition < -1) {
    asynPrint(pasynUser,
              ASYN_TRACE_ERROR,
              "%s:%s: error, slave bus position not valid (needs to be equal or larger than -1): %d.\n",
              driverName,
              functionName,
              slavePosition);
    return asynError;
  }

  if (strlen(aliasBuffer) <= 0) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, ethercat slave alias not valid: %s.\n",
              driverName, functionName, aliasBuffer);
    return asynError;
  }

  int errorId = writeEcEntryIDString(slavePosition,
                                     aliasBuffer,
                                     (uint64_t)value);

  if (errorId) {
    asynPrint(pasynUser,
              ASYN_TRACE_ERROR,
              "%s:%s: error, write of parameter %s failed with error code 0x%x.\n",
              driverName,
              functionName,
              aliasBuffer,
              errorId);
    return asynError;
  }

  /* Set the parameter in the parameter library. */
  asynStatus status = (asynStatus)setIntegerParam(function, value);

  if (status != asynSuccess) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, setIngerParam() failed.\n",
              driverName, functionName);
    return asynError;
  }

  /* Do callbacks so higher layers see any changes */
  status = (asynStatus)callParamCallbacks();

  return status;
}

asynStatus ecmcAsynPortDriver::writeFloat64(asynUser    *pasynUser,
                                            epicsFloat64 value) {
  int function = pasynUser->reason;
  const char *paramName;
  const char *functionName = "writeFloat64";

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  char  buffer[1024];
  char *aliasBuffer   = &buffer[0];
  int   slavePosition = -10;
  int   masterIndex   = 0;
  int   nvals         = sscanf(paramName,
                               "ec%d.s%d.%s",
                               &masterIndex,
                               &slavePosition,
                               aliasBuffer);

  if (nvals != 3) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, parameter name not valid: %s.\n",
              driverName, functionName, paramName);
    return asynError;
  }

  if (slavePosition < -1) {
    asynPrint(pasynUser,
              ASYN_TRACE_ERROR,
              "%s:%s: error, slave bus position not valid (needs to be equal or larger than -1): %d.\n",
              driverName,
              functionName,
              slavePosition);
    return asynError;
  }

  if (strlen(aliasBuffer) <= 0) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, ethercat slave alias not valid: %s.\n",
              driverName, functionName, aliasBuffer);
    return asynError;
  }

  uint64_t *temp = reinterpret_cast<uint64_t *>(&value);
  int errorId    = writeEcEntryIDString(slavePosition, aliasBuffer, *temp);

  if (errorId) {
    asynPrint(pasynUser,
              ASYN_TRACE_ERROR,
              "%s:%s: error, write of parameter %s failed with error code 0x%x.\n",
              driverName,
              functionName,
              aliasBuffer,
              errorId);
    return asynError;
  }

  /* Set the parameter in the parameter library. */
  asynStatus status = (asynStatus)setDoubleParam(function, value);

  if (status != asynSuccess) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, setIngerParam() failed.\n",
              driverName, functionName);
    return asynError;
  }

  /* Do callbacks so higher layers see any changes */
  status = (asynStatus)callParamCallbacks();

  return status;
}

asynUser * ecmcAsynPortDriver::getTraceAsynUser() {
  return pasynUserSelf;
}

asynStatus ecmcAsynPortDriver::readInt8Array(asynUser  *pasynUser,
                                             epicsInt8 *value,
                                             size_t     nElements,
                                             size_t    *nIn) {
  const char *functionName = "readInt8Array";

  int errorId = readArrayGeneric(pasynUser,
                                 reinterpret_cast<uint8_t *>(value),
                                 nElements,
                                 nIn,
                                 2,
                                 functionName);

  if (errorId) {
    return asynError;
  }

  return asynSuccess;
}

asynStatus ecmcAsynPortDriver::readInt16Array(asynUser   *pasynUser,
                                              epicsInt16 *value,
                                              size_t      nElements,
                                              size_t     *nIn) {
  const char *functionName = "readInt16Array";

  int errorId = readArrayGeneric(pasynUser,
                                 reinterpret_cast<uint8_t *>(value),
                                 nElements,
                                 nIn,
                                 2,
                                 functionName);

  if (errorId) {
    return asynError;
  }

  return asynSuccess;
}

asynStatus ecmcAsynPortDriver::readInt32Array(asynUser   *pasynUser,
                                              epicsInt32 *value,
                                              size_t      nElements,
                                              size_t     *nIn) {
  const char *functionName = "readInt32Array";

  int errorId = readArrayGeneric(pasynUser,
                                 reinterpret_cast<uint8_t *>(value),
                                 nElements,
                                 nIn,
                                 2,
                                 functionName);

  if (errorId) {
    return asynError;
  }

  return asynSuccess;
}

asynStatus ecmcAsynPortDriver::readFloat32Array(asynUser     *pasynUser,
                                                epicsFloat32 *value,
                                                size_t        nElements,
                                                size_t       *nIn) {
  const char *functionName = "readFloat32rray";

  int errorId = readArrayGeneric(pasynUser,
                                 reinterpret_cast<uint8_t *>(value),
                                 nElements,
                                 nIn,
                                 2,
                                 functionName);

  if (errorId) {
    return asynError;
  }

  return asynSuccess;
}

asynStatus ecmcAsynPortDriver::readFloat64Array(asynUser     *pasynUser,
                                                epicsFloat64 *value,
                                                size_t        nElements,
                                                size_t       *nIn) {
  const char *functionName = "readFloat64Array";
  int errorId = readArrayGeneric(pasynUser,
                                 reinterpret_cast<uint8_t *>(value),
                                 nElements,
                                 nIn,
                                 2,
                                 functionName);

  if (errorId) {
    return asynError;
  }

  return asynSuccess;
}

int ecmcAsynPortDriver::readArrayGeneric(asynUser   *pasynUser,
                                         epicsUInt8 *value,
                                         size_t      nElements,
                                         size_t     *nIn,
                                         size_t      typeSize,
                                         const char *functionName) {
  const char *paramName;
  int errorId  = 0;
  int function = pasynUser->reason;

  getParamName(function, &paramName);

  size_t bytesRead = 0;
  char   buffer[1024];
  int    masterIndex = 0;
  int    nvals       = sscanf(paramName, "ec%d.mm.%s", &masterIndex, buffer);

  if (nvals == 2) {
    errorId = readEcMemMap(buffer,
                           reinterpret_cast<uint8_t *>(value),
                           nElements * typeSize,
                           &bytesRead);
  }

  if (errorId) {
    asynPrint(pasynUser,
              ASYN_TRACE_ERROR,
              "%s:%s: error, read of parameter %s failed with error code 0x%x.\n",
              driverName,
              functionName,
              paramName,
              errorId);
    *nIn = 0;
    return asynError;
  }
  *nIn = bytesRead / typeSize;
  return errorId;
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
  printf("%s:%s: drvInfo: %s\n", driverName, functionName,drvInfo);
  if(validateDrvInfo(drvInfo)!=asynSuccess){
    return asynError;
  }

  int index=0;
  asynStatus status=findParam(drvInfo,&index);
  if(status==asynSuccess){
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: Parameter index found at: %d for %s. \n", driverName, functionName,index,drvInfo);
    if(!pEcmcParamArray_[index]){
      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s:pAdsParamArray_[%d]==NULL (drvInfo=%s).", driverName, functionName,index,drvInfo);
      return asynError;
    }
    return asynPortDriver::drvUserCreate(pasynUser,drvInfo,pptypeName,psize);
  }


  //Ensure space left in param table
  if(ecmcParamArrayCount_>=(paramTableSize_-1)){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: Parameter table full. Parameter with drvInfo %s will be discarded.", driverName, functionName,drvInfo);
    return asynError;
  }

  // Collect data from drvInfo string and recordpasynUser->reason=index;
  ecmcParamInfo *paramInfo=new ecmcParamInfo();
  memset(paramInfo,0,sizeof(ecmcParamInfo));
  paramInfo->sampleTimeMS=defaultSampleTimeMS_;
  paramInfo->maxDelayTimeMS=defaultMaxDelayTimeMS_;
  paramInfo->refreshNeeded=1;

  status=getRecordInfoFromDrvInfo(drvInfo, paramInfo);
  if(status!=asynSuccess){
    return asynError;
  }

  status=createParam(drvInfo,paramInfo->asynType,&index);
  if(status!=asynSuccess){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: createParam() failed.",driverName, functionName);
    return asynError;
  }
  asynPrint(pasynUser, ASYN_TRACE_INFO, "%s:%s: Parameter created: \"%s\" (index %d).\n", driverName, functionName,drvInfo,index);

  //Set default value for basic types...
  switch(paramInfo->asynType){
    case asynParamInt32:
      setIntegerParam(index,0);
      break;
    case asynParamFloat64:
      setDoubleParam(index,0);
      break;
    default:
      break;
  }

  paramInfo->paramIndex=index;

  int addr=0;
  status = getAddress(pasynUser, &addr);
  if (status != asynSuccess){
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: getAddress() failed.",driverName, functionName);
    return(status);
  }

  paramInfo->asynAddr=addr;

  status=parseInfofromDrvInfo(drvInfo,paramInfo);
  if(status!=asynSuccess){
    return asynError;
  }

  //link data from ECMC
  printf("VariableToLookFor %s\n", paramInfo->ecmcVariablePathStr);

  pasynUser->timeout=(paramInfo->maxDelayTimeMS*2)/1000;
  pEcmcParamArray_[ecmcParamArrayCount_]=paramInfo;
  ecmcParamArrayCount_++;

  return asynPortDriver::drvUserCreate(pasynUser,drvInfo,pptypeName,psize); //Assigns pasynUser->reason;
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
  if(status) {
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
          int nvals=sscanf(paramInfo->inp,"@asyn(%[^,],%d,%d)%s",port,&adr,&timeout,currdrvInfo);
          if(nvals==4){
            // Ensure correct port and drvinfo
            if(strcmp(port,portName)==0 && strcmp(drvInfo,currdrvInfo)==0){
              recordFound=true;  // Correct port and drvinfo!\n");
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
          int nvals=sscanf(paramInfo->out,"@asyn(%[^,],%d,%d)%s",port,&adr,&timeout,currdrvInfo);
          if(nvals==4){
            // Ensure correct port and drvinfo
            if(strcmp(port,portName)==0 && strcmp(drvInfo,currdrvInfo)==0){
              recordFound=true;  // Correct port and drvinfo!\n");
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
          return asynSuccess;  // The correct record was found and the paramInfo structure is filled
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
 * - "T_DLY_MS" (maximum delay time ms)\n
 * - "TS_MS" (sample time ms)\n
 * - "TIMEBASE" ("PLC" or "EPICS")\n
 * Also supports the following commands:
 * - ".AMSPORTSTATE." (Read/write AMS-port state)\n
 * - ".ADR.*" (absolute access)\n
 */
asynStatus ecmcAsynPortDriver::parseInfofromDrvInfo(const char* drvInfo,ecmcParamInfo *paramInfo)
{
  const char* functionName = "parseInfofromDrvInfo";
  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName,drvInfo);

  //Check if input or output
  paramInfo->isIOIntr=false;
  const char* temp=strrchr(drvInfo,'?');
  if(temp){
    if(strlen(temp)==1){
      paramInfo->isIOIntr=true; //All inputs will be created I/O intr
    }
  }

  asynPrint(pasynUserSelf,ASYN_TRACE_FLOW, "%s:%s: drvInfo %s is %s\n", driverName, functionName,drvInfo,paramInfo->isIOIntr ? "I/O Intr (end with ?)": " not I/O Intr (end with =)");

  //take part after last "/" if option or complete string..
  char buffer[ECMC_MAX_FIELD_CHAR_LENGTH];
  //See if option (find last '/')
  const char *drvInfoEnd=strrchr(drvInfo,'/');
  if(drvInfoEnd){ // found '/'
    int nvals=sscanf(drvInfoEnd,"/%s",buffer);
    if(nvals==1){
      paramInfo->ecmcVariablePathStr=strdup(buffer);
      paramInfo->ecmcVariablePathStr[strlen(paramInfo->ecmcVariablePathStr)-1]=0; //Strip ? or = from end
    }
    else{
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse PLC address string from drvInfo (%s)\n", driverName, functionName,drvInfo);
      return asynError;
    }
  }
  else{  //No options
    paramInfo->ecmcVariablePathStr=strdup(drvInfo);
    paramInfo->ecmcVariablePathStr[strlen(paramInfo->ecmcVariablePathStr)-1]=0; //Strip ? or = from end
  }

  //Check if ECMC_OPTION_T_MAX_DLY_MS option
  const char *option=ECMC_OPTION_T_MAX_DLY_MS;
  const char *isThere=strstr(drvInfo,option);
  if(isThere){
    if(strlen(isThere)<(strlen(option)+strlen("=0/"))){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }

    int nvals = sscanf(isThere+strlen(option),"=%lf/",&paramInfo->maxDelayTimeMS);

    if(nvals!=1){
      paramInfo->maxDelayTimeMS=defaultMaxDelayTimeMS_;
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }
  }

  //Check if ECMC_OPTION_T_SAMPLE_RATE_MS option
  option=ECMC_OPTION_T_SAMPLE_RATE_MS;
  paramInfo->sampleTimeMS=defaultSampleTimeMS_;
  isThere=strstr(drvInfo,option);
  if(isThere){
    if(strlen(isThere)<(strlen(option)+strlen("=0/"))){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }

    int nvals = sscanf(isThere+strlen(option),"=%lf/",&paramInfo->sampleTimeMS);

    if(nvals!=1){
      paramInfo->sampleTimeMS=defaultSampleTimeMS_;
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n", driverName, functionName,option,drvInfo);
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
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }

    int nvals = sscanf(isThere+strlen(option),"=%[^/]/",buffer);
    if(nvals!=1){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n", driverName, functionName,option,drvInfo);
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
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }
    int nvals;
    nvals = sscanf(isThere+strlen(option),"=%[^/]",buffer);
     if(nvals!=1){
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n", driverName, functionName,option,drvInfo);
      return asynError;
    }
    paramInfo->asynTypeStr=strdup(buffer);
    paramInfo->asynType=stringToAsynType(paramInfo->asynTypeStr);
  }

  return asynSuccess;
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
    fprintf(fp, "General information:\n");
    fprintf(fp, "  Port:                        %s\n",portName);
    fprintf(fp, "  Auto-connect:                %s\n",autoConnect_ ? "true" : "false");
    fprintf(fp, "  Priority:                    %d\n",priority_);
    fprintf(fp, "  Param. table size:           %d\n",paramTableSize_);
    fprintf(fp, "  Param. count:                %d\n",ecmcParamArrayCount_);
    fprintf(fp, "  Default sample time [ms]:    %d\n",defaultSampleTimeMS_);
    fprintf(fp, "  Default max delay time [ms]: %d\n",defaultMaxDelayTimeMS_);
    fprintf(fp, "  Default time source:         %s\n",(defaultTimeSource_==ECMC_TIME_BASE_ECMC) ? ECMC_OPTION_TIMEBASE_ECMC : ECMC_OPTION_TIMEBASE_EPICS);
    fprintf(fp,"\n");
  }
  if(details>=2){
    //print all parameters
    fprintf(fp,"Parameter details:\n");
    for(int i=0; i<ecmcParamArrayCount_;i++){
      if(!pEcmcParamArray_[i]){
        fprintf(fp,"%s:%s: ERROR: Parameter array null at index %d\n", driverName, functionName,i);
        return;
      }
      ecmcParamInfo *paramInfo=pEcmcParamArray_[i];
      fprintf(fp,"  Parameter %d:\n",i);
      if(i==0){
        fprintf(fp,"    Parameter 0 (pasynUser->reason==0) is reserved for Asyn octet interface (Motor Record and Stream Device access).\n");
        fprintf(fp,"\n");
        continue;
      }
      fprintf(fp,"    Param name:                %s\n",paramInfo->drvInfo);
      fprintf(fp,"    Param index:               %d\n",paramInfo->paramIndex);
      fprintf(fp,"    Param type:                %s (%d)\n",asynTypeToString((long)paramInfo->asynType),paramInfo->asynType);
      fprintf(fp,"    Param sample time [ms]:    %lf\n",paramInfo->sampleTimeMS);
      fprintf(fp,"    Param max delay time [ms]: %lf\n",paramInfo->maxDelayTimeMS);
      fprintf(fp,"    Param isIOIntr:            %s\n",paramInfo->isIOIntr ? "true" : "false");
      fprintf(fp,"    Param asyn addr:           %d\n",paramInfo->asynAddr);
      fprintf(fp,"    Param time source:         %s\n",(paramInfo->timeBase==ECMC_TIME_BASE_ECMC) ? ECMC_OPTION_TIMEBASE_ECMC : ECMC_OPTION_TIMEBASE_EPICS);
      fprintf(fp,"    Param epics time:          %us:%uns\n",paramInfo->epicsTimestamp.secPastEpoch,paramInfo->epicsTimestamp.nsec);
      fprintf(fp,"    Param array buffer alloc:  %s\n",paramInfo->arrayDataBuffer ? "true" : "false");
      fprintf(fp,"    Param array buffer size:   %lu\n",paramInfo->arrayDataBufferSize);
      fprintf(fp,"    Param alarm:               %d\n",paramInfo->alarmStatus);
      fprintf(fp,"    Param severity:            %d\n",paramInfo->alarmSeverity);
      fprintf(fp,"    ECMC adr str:              %s\n",paramInfo->ecmcVariablePathStr);
      //fprintf(fp,"    ECMC data type:            %s\n",adsTypeToString(paramInfo->ecmcDataType));
      //fprintf(fp,"    ECMC data type size:       %zu\n",adsTypeSize(paramInfo->ecmcDataType));
      fprintf(fp,"    ECMC data is array:        %s\n",paramInfo->ecmcDataIsArray ? "true" : "false");
      //fprintf(fp,"    ECMC data type warning:     %s\n",paramInfo->plcDataTypeWarn ? "true" : "false");
      fprintf(fp,"    Record name:               %s\n",paramInfo->recordName);
      fprintf(fp,"    Record type:               %s\n",paramInfo->recordType);
      fprintf(fp,"    Record dtyp:               %s\n",paramInfo->dtyp);
      fprintf(fp,"\n");
    }
  }
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
int ecmcAsynPortDriverConfigure(const char *portName,
                                int         paramTableSize,
                                int         priority,
                                int         disableAutoConnect) {
  if(portName == NULL) {
    printf("Error: portName missing.\n");
    return asynError;  
  }

  parameterCounter = 0;
  maxParameters    = paramTableSize;
  ecmcAsynPortObj  = new ecmcAsynPortDriver(portName,
                                            paramTableSize,
                                            disableAutoConnect == 0,
                                            priority);

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

    int errorCode = initEcmcAsyn((void *)ecmcAsynPortObj);

    if (errorCode) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_ERROR,
                "ecmcAsynPortDriverConfigure: ERROR: New AsynPortDriver (setAsynPort()) failed (0x%x).\n",
                errorCode);
      return asynError;
    }

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

static int parseAsynDataType(const char *asynTypeString) {
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
  "asynParamInt8Array"
  "asynParamInt16Array"
  "asynParamInt32Array"
  "asynParamFloat32Array"
  "asynParamFloat64Array"
  */

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
}

/* EPICS iocsh shell command: ecmcAsynPortDriverConfigure*/

static const iocshArg initArg0 = { "port name", iocshArgString };
static const iocshArg initArg1 = { "parameter table size", iocshArgInt };
static const iocshArg initArg2 = { "priority", iocshArgInt };
static const iocshArg initArg3 = { "disable auto connect", iocshArgInt };

static const iocshArg *const initArgs[] = { &initArg0,
                                            &initArg1,
                                            &initArg2,
                                            &initArg3 };
static const iocshFuncDef    initFuncDef =
{ "ecmcAsynPortDriverConfigure", 4, initArgs };
static void initCallFunc(const iocshArgBuf *args) {
  ecmcAsynPortDriverConfigure(args[0].sval,
                              args[1].ival,
                              args[2].ival,
                              args[3].ival);
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

  if(portName == NULL) {
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


