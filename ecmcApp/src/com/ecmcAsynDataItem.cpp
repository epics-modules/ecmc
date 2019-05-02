#include "../com/ecmcAsynDataItem.h"
#include "../com/ecmcAsynPortDriver.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
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

static const char *driverName = "ecmcAsynPortDriver";

ecmcAsynDataItem::ecmcAsynDataItem (ecmcAsynPortDriver *asynPortDriver, const char *paramName,asynParamType asynParType)
{
  checkIntRange_ = 0;
  intMax_ = 0;
  intMin_ = 0;
  intBits_ = 0;
  asynPortDriver_=asynPortDriver;  
  data_=0;  
  asynUpdateCycleCounter_=0;
  supportedTypesCounter_=0;
  allowWriteToEcmc_=false;
  for(int i=0;i<ERROR_ASYN_MAX_SUPPORTED_TYPES_COUNT;i++) {
    supportedTypes_[i]=asynParamNotDefined;
  }
  //paramInfo_= new ecmcParamInfo();
  memset(&paramInfo_,0,sizeof(ecmcParamInfo));
  paramInfo_.name=strdup(paramName);
  paramInfo_.asynType=asynParType;
  paramInfo_.ecmcDataIsArray = asynTypeIsArray(asynParType);
  addSupportedAsynType(asynParType);
}

ecmcAsynDataItem::ecmcAsynDataItem (ecmcAsynPortDriver *asynPortDriver)
{
  checkIntRange_ = 0;
  intMax_ = 0;
  intMin_ = 0;
  intBits_ = 0;
  asynPortDriver_=asynPortDriver;  
  data_=0;  
  asynUpdateCycleCounter_=0;
  supportedTypesCounter_=0;
  allowWriteToEcmc_=false;
  for(int i=0;i<ERROR_ASYN_MAX_SUPPORTED_TYPES_COUNT;i++) {
    supportedTypes_[i]=asynParamNotDefined;
  }
  //paramInfo_= new ecmcParamInfo();
  memset(&paramInfo_,0,sizeof(ecmcParamInfo));
}

ecmcAsynDataItem::~ecmcAsynDataItem ()
{
  free(paramInfo_.recordName);
  paramInfo_.recordName = NULL;
  free(paramInfo_.recordType);
  paramInfo_.recordType =NULL;
  free(paramInfo_.scan);
  paramInfo_.scan = NULL;
  free(paramInfo_.dtyp);
  paramInfo_.dtyp = 0;
  free(paramInfo_.inp);
  paramInfo_.inp = NULL;
  free(paramInfo_.out);
  paramInfo_.out = NULL;
  free(paramInfo_.drvInfo);
  paramInfo_.drvInfo = NULL;
  free(paramInfo_.asynTypeStr);
  paramInfo_.asynTypeStr = NULL;
  free(paramInfo_.name);
  paramInfo_.name = NULL;
  //delete paramInfo_;  
  //paramInfo_ = NULL;
}

int ecmcAsynDataItem::setEcmcDataPointer(uint8_t *data,size_t bytes)
{
  data_ = data;
  paramInfo_.ecmcSize=bytes;
  paramInfo_.ecmcMaxSize=bytes;
  paramInfo_.ecmcDataPointerValid =  data && bytes>0;
  return 0;
}

int ecmcAsynDataItem::refreshParamRT(int force)
{
  if(!asynPortDriver_->getAllowRtThreadCom()){
    return ERROR_ASYN_NOT_REFRESHED_RETURN;
  }
  return refreshParam(force,data_,paramInfo_.ecmcSize);
}

int ecmcAsynDataItem::refreshParam(int force)
{
  return refreshParam(force,data_,paramInfo_.ecmcSize);
}

int ecmcAsynDataItem::refreshParamRT(int force, size_t bytes)
{
  if(!asynPortDriver_->getAllowRtThreadCom()){
    return ERROR_ASYN_NOT_REFRESHED_RETURN;
  }
  return refreshParam(force,data_,bytes);
}

int ecmcAsynDataItem::refreshParam(int force, size_t bytes)
{
  return refreshParam(force,data_,bytes);
}

int ecmcAsynDataItem::refreshParamRT(int force,uint8_t *data, size_t bytes)
{
  if(!asynPortDriver_->getAllowRtThreadCom()){
    return ERROR_ASYN_NOT_REFRESHED_RETURN;
  }
  return refreshParam(force,data,bytes);
}

/*
* Returns 0 if refreshed.
* Retrun -1 or error code if not refreshed. 
*/
int ecmcAsynDataItem::refreshParam(int force,uint8_t *data, size_t bytes)
{
  // set data pointer and size if param is not initialized (linked to record)
  data_=data;
  paramInfo_.ecmcSize=bytes;

  if(!paramInfo_.initialized) {
    return 0;
  }

  if(paramInfo_.sampleTimeCycles < 0 && !force) {
    return ERROR_ASYN_NOT_REFRESHED_RETURN;
  }

  if(paramInfo_.sampleTimeCycles >= 0 && asynUpdateCycleCounter_< paramInfo_.sampleTimeCycles-1 && !force){
    asynUpdateCycleCounter_++;
    return ERROR_ASYN_NOT_REFRESHED_RETURN;  //Not refreshed
  }

  if(data==0 || bytes<0){
    return ERROR_ASYN_DATA_NULL;
  }

  if(bytes > paramInfo_.ecmcMaxSize && paramInfo_.arrayCheckSize) {
    bytes = paramInfo_.ecmcMaxSize;
  }

  paramInfo_.ecmcSize = bytes;

  //asynPrint(asynPortDriver_->getTraceAsynUser(), ASYN_TRACE_ERROR, "refreshParam: %s, bytes %lu, force %d, sample time %d\n",getName(),bytes,force,paramInfo_.sampleTimeCycles);
  asynStatus stat=asynError;
  switch(paramInfo_.asynType){
    case asynParamUInt32Digital:
      stat = asynPortDriver_->setUIntDigitalParam(paramInfo_.index,*((epicsInt32*)data),0xFFFFFFFF);
      break;
    case asynParamInt32:
      stat = asynPortDriver_->setIntegerParam(paramInfo_.index,*((epicsInt32*)data));
      break;
    case asynParamFloat64:
      stat = asynPortDriver_->setDoubleParam(paramInfo_.index,*((epicsFloat64*)data));
      break;
    case asynParamInt8Array:
      stat = asynPortDriver_->doCallbacksInt8Array((epicsInt8*)data,bytes, paramInfo_.index, 0);
      break;
    case asynParamInt16Array:
      stat = asynPortDriver_->doCallbacksInt16Array((epicsInt16*)data,bytes/sizeof(epicsInt16), paramInfo_.index, 0);
      break;
    case asynParamInt32Array:
      stat = asynPortDriver_->doCallbacksInt32Array((epicsInt32*)data,bytes/sizeof(epicsInt32), paramInfo_.index, 0);
      break;
    case asynParamFloat32Array:
      stat = asynPortDriver_->doCallbacksFloat32Array((epicsFloat32*)data,bytes/sizeof(epicsFloat32), paramInfo_.index, 0);
      break;
    case asynParamFloat64Array:
      stat = asynPortDriver_->doCallbacksFloat64Array((epicsFloat64*)data,bytes/sizeof(epicsFloat64), paramInfo_.index, 0);
      break;
    default:
      return ERROR_ASYN_DATA_TYPE_NOT_SUPPORTED;
      break;
  }

  asynUpdateCycleCounter_=0;    
  if(stat!=asynSuccess) {
    asynPrint(asynPortDriver_->getTraceAsynUser(), ASYN_TRACE_ERROR, "ecmcAsynDataItem::refreshParam: ERROR: Refresh failed for parameter %s, bytes %lu, force %d, sample time %d (0x%x).\n",
    getName(),bytes,force,paramInfo_.sampleTimeCycles,ERROR_ASYN_REFRESH_FAIL);
    return ERROR_ASYN_REFRESH_FAIL;
  }
  return 0;
}

int ecmcAsynDataItem::createParam()
{    
  return createParam(paramInfo_.name,paramInfo_.asynType);
}

int ecmcAsynDataItem::createParam(const char *paramName,asynParamType asynParType,uint8_t *data,size_t bytes)
{
  setEcmcDataPointer(data,bytes);
  return createParam(paramName,asynParType);
}

int ecmcAsynDataItem::createParam(const char *paramName,asynParamType asynParType)
{
  if(asynPortDriver_==0){
    return ERROR_ASYN_PORT_NULL;
  }
  paramInfo_.name=strdup(paramName);  
  paramInfo_.asynType=asynParType;  
  asynStatus status = asynPortDriver_->createParam(paramName,paramInfo_.asynType,&paramInfo_.index);
  return (status==asynSuccess) ? 0 : ERROR_ASYN_CREATE_PARAM_FAIL;
}

int ecmcAsynDataItem::getAsynParameterIndex()
{
  return paramInfo_.index;
}

int ecmcAsynDataItem::setAsynParameterType(asynParamType parType)
{
  paramInfo_.asynType=parType;
  return 0;
}

asynParamType ecmcAsynDataItem::getAsynParameterType()
{
  return paramInfo_.asynType;
}

int ecmcAsynDataItem::setAsynPortDriver(ecmcAsynPortDriver *asynPortDriver)
{
  asynPortDriver_=asynPortDriver;
  return 0;
}

int ecmcAsynDataItem::setAsynParSampleTimeMS(double sampleTime)
{
  paramInfo_.sampleTimeMS=sampleTime;
  paramInfo_.sampleTimeCycles=(int32_t)(sampleTime/1000.0*(double)MCU_FREQUENCY);
  return 0;
}

ecmcParamInfo *ecmcAsynDataItem::getParamInfo()
{
  return &paramInfo_;
}

bool ecmcAsynDataItem::initialized() {
  return paramInfo_.initialized;
}

int32_t ecmcAsynDataItem::getSampleTimeCycles() {
  return paramInfo_.sampleTimeCycles;
}

double ecmcAsynDataItem::getSampleTimeMs(){
  return paramInfo_.sampleTimeCycles*1000.0/MCU_FREQUENCY;
}

char *ecmcAsynDataItem::getName() {  
  return paramInfo_.name;
}

char * ecmcAsynDataItem::getDrvInfo() {
  return paramInfo_.drvInfo;
}

char * ecmcAsynDataItem::getDtyp() {
  return paramInfo_.dtyp;
}

char * ecmcAsynDataItem::getRecordType(){
  return paramInfo_.recordType;
}

char * ecmcAsynDataItem::getRecordName() {
  return paramInfo_.recordName;
}

int ecmcAsynDataItem::addSupportedAsynType(asynParamType type) {
  
  //check so not already in list
  if(asynTypeSupported(type)) {
    return 0;
  }
  
  if(supportedTypesCounter_<ERROR_ASYN_MAX_SUPPORTED_TYPES_COUNT-1) {
    supportedTypes_[supportedTypesCounter_]=type;
    supportedTypesCounter_++;
    return 0;
  }
  return ERROR_ASYN_SUPPORTED_TYPES_ARRAY_FULL;
}

bool ecmcAsynDataItem::asynTypeSupported(asynParamType type) {
  for(int i=0;i<supportedTypesCounter_;i++) {
   
   if (supportedTypes_[i]==type) {
     return true;
   }
  }
  return false;
}

int ecmcAsynDataItem::getSupportedAsynTypeCount() {
  return supportedTypesCounter_;
}

asynParamType ecmcAsynDataItem::getAsynType() {
  return paramInfo_.asynType;
}

char * ecmcAsynDataItem::getAsynTypeName() {
  return paramInfo_.asynTypeStr;
}

asynParamType ecmcAsynDataItem::getSupportedAsynType(int index) {
  if(index<supportedTypesCounter_) {
    return supportedTypes_[index];
  }
  return asynParamNotDefined;
}

void ecmcAsynDataItem::allowWriteToEcmc(bool allowWrite) {
  allowWriteToEcmc_ = allowWrite;
}

bool ecmcAsynDataItem::writeToEcmcAllowed() {
  return allowWriteToEcmc_;
}

bool ecmcAsynDataItem::willRefreshNext() {
  return asynUpdateCycleCounter_>= paramInfo_.sampleTimeCycles-1;
}

/** Set parameter alarm state.
 *
 * \param[in] paramInfo Parameter information.
 * \param[in] alarm Alarm type (EPICS def).
 * \param[in] severity Alarm severity (EPICS def).
 *
 * \return asynSuccess or asynError.
 */
asynStatus ecmcAsynDataItem::setAlarmParam(int alarm,int severity)
{
  asynStatus stat;
  int oldAlarmStatus=0;
  stat = asynPortDriver_->getParamAlarmStatus(getAsynParameterIndex(),&oldAlarmStatus);
  if(stat!=asynSuccess){
    //asynPrint(asynPortDriver_->getTraceAsynUser(), ASYN_TRACE_ERROR, "%s:%s: getParamAlarmStatus failed for parameter %s (%d).\n", driverName, functionName,paramInfo_.drvInfo,paramInfo_.paramIndex);
    return asynError;
  }

  bool doCallbacks=false;

  if(oldAlarmStatus!=alarm){
    stat = asynPortDriver_->setParamAlarmStatus(getAsynParameterIndex(),alarm);
    if(stat!=asynSuccess){
      //asynPrint(asynPortDriver_->getTraceAsynUser(), ASYN_TRACE_ERROR, "%s:%s: Failed set alarm status for parameter %s (%d).\n", driverName, functionName,paramInfo_.drvInfo,paramInfo_.paramIndex);
      return asynError;
    }
    paramInfo_.alarmStatus=alarm;
    doCallbacks=true;
  }

  int oldAlarmSeverity=0;
  stat = asynPortDriver_->getParamAlarmSeverity(getAsynParameterIndex(),&oldAlarmSeverity);
  if(stat!=asynSuccess){
    //asynPrint(asynPortDriver_->getTraceAsynUser(), ASYN_TRACE_ERROR, "%s:%s: getParamAlarmStatus failed for parameter %s (%d).\n", driverName, functionName,paramInfo_.drvInfo,paramInfo_.paramIndex);
    return asynError;
  }

  if(oldAlarmSeverity!=severity){
    stat = asynPortDriver_->setParamAlarmSeverity(getAsynParameterIndex(),severity);
    if(stat!=asynSuccess){
      //asynPrint(asynPortDriver_->getTraceAsynUser(), ASYN_TRACE_ERROR, "%s:%s: Failed set alarm severity for parameter %s (%d).\n", driverName, functionName,paramInfo_.drvInfo,paramInfo_.paramIndex);
      return asynError;
    }
    paramInfo_.alarmSeverity=severity;
    doCallbacks=true;
  }

  if(!doCallbacks || !asynPortDriver_->getAllowRtThreadCom()){
    return asynSuccess;
  }
  
  //Alarm status or severity changed=>Do callbacks with old buffered data (if nElemnts==0 then no data in record...)
  if(paramInfo_.ecmcDataIsArray && paramInfo_.ecmcSize>0){
    refreshParamRT(1);
  }
  else{
      stat = asynPortDriver_->callParamCallbacks();
  }

  return stat;
}

int ecmcAsynDataItem::getAlarmStatus() {
  return paramInfo_.alarmStatus;
}

int ecmcAsynDataItem::getAlarmSeverity() {
  return paramInfo_.alarmSeverity;
}

int ecmcAsynDataItem::asynTypeIsArray(asynParamType asynParType) {

  switch(asynParType){
    case asynParamInt8Array:
      return 1;      
      break;
    case asynParamInt16Array:
      return 1;
      break;
    case asynParamInt32Array:
      return 1;
      break;
    case asynParamFloat32Array:
      return 1;
      break;
    case asynParamFloat64Array:
      return 1;
      break;
    default:
      return 0;
      break;
  }
  return 0;
}

asynStatus ecmcAsynDataItem::writeGeneric(uint8_t *data, size_t bytesToWrite, asynParamType type, size_t *writtenBytes) {

   if(!asynTypeSupported(type)) {
    LOGERR(
      "%s/%s:%d: ERROR: %s asyn type not supported (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      getName(),
      ERROR_ASYN_DATA_TYPE_NOT_SUPPORTED);
      return asynError;
  }

  size_t bytes = bytesToWrite;
  if (asynTypeIsArray(type)) {
    if (bytes > paramInfo_.ecmcMaxSize) {
      bytes = paramInfo_.ecmcMaxSize;
    }
  } else {
    if ( bytes > paramInfo_.ecmcMaxSize ) {
    LOGERR(
      "%s/%s:%d: ERROR: %s write error. Data buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      getName(),
      ERROR_ASYN_DATA_BUFFER_TO_SMALL);
      return asynError;
    }
  }

  memcpy(data_, data, bytes);
  *writtenBytes = bytes; 

  //refresh params
  int errorCode=refreshParamRT(1);
  if(errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: %s write error. Refresh of params failed (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      getName(),
      errorCode);
      return asynError;
  }

  // Trigger callbacks if not array
  if(!asynTypeIsArray(type)) {
   return asynPortDriver_->callParamCallbacks();
  }
  
  return asynSuccess;
}

asynStatus ecmcAsynDataItem::readGeneric(uint8_t *data, size_t bytesToRead, asynParamType type, size_t *readBytes) {

   if(!asynTypeSupported(type)) {
    LOGERR(
      "%s/%s:%d: ERROR: %s asyn type not supported (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      getName(),
      ERROR_ASYN_DATA_TYPE_NOT_SUPPORTED);
      return asynError;
  }

  size_t bytes = bytesToRead;
  if (asynTypeIsArray(type)) {
    if (bytes > paramInfo_.ecmcMaxSize && paramInfo_.arrayCheckSize) {
      bytes = paramInfo_.ecmcMaxSize;
    }
  } else {
    if ( bytes > paramInfo_.ecmcMaxSize) {
    LOGERR(
      "%s/%s:%d: ERROR: %s read error. Data buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      getName(),
      ERROR_ASYN_DATA_BUFFER_TO_SMALL);
      return asynError;
    }
  }

  memcpy(data, data_, bytes);
  *readBytes = bytes; 
  return asynSuccess;
}

asynStatus ecmcAsynDataItem::readInt32(epicsInt32 *value) {

  size_t bytesRead = 0;
  return readGeneric((uint8_t*)value, sizeof(epicsInt32),
                     asynParamInt32, &bytesRead);
}

asynStatus ecmcAsynDataItem::writeInt32(epicsInt32 value) {

  size_t bytesWritten = 0;
  if(checkIntRange_ && (value > intMax_ || value < intMin_)) {
    LOGERR(
      "%s/%s:%d: Error: %s value Out Of Range %d (allowed Range %ld..%ld) (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      getName(),
      value,      
      intMin_,
      intMax_,
      ERROR_ASYN_WRITE_VALUE_OUT_OF_RANGE);
      return asynError;
  }

  return writeGeneric((uint8_t*)&value, sizeof(epicsInt32),
                      asynParamInt32, &bytesWritten);
}

asynStatus ecmcAsynDataItem::readUInt32Digital(epicsUInt32 *value,
                                               epicsUInt32 mask) {
  epicsUInt32 tempValue = 0;
  size_t bytesRead = 0;
  asynStatus stat = readGeneric((uint8_t*)&tempValue, sizeof(epicsUInt32),
                                asynParamUInt32Digital, &bytesRead);
  if(stat != asynSuccess) {
    return stat;
  }

  *value = tempValue & mask;  
  return asynSuccess;
}

asynStatus  ecmcAsynDataItem::writeUInt32Digital(epicsUInt32 value,
                                                 epicsUInt32 mask) {

  if(checkIntRange_ && (value > intMax_ || value < intMin_)) {
    LOGERR(
      "%s/%s:%d: Error: %s value Out Of Range %d. Allowed Range %ld..%ld (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      getName(),
      value,
      intMin_,
      intMax_,
      ERROR_ASYN_WRITE_VALUE_OUT_OF_RANGE);
      return asynError;
  }

  epicsUInt32 tempVal1 = 0;
  asynStatus stat = readUInt32Digital(&tempVal1,~mask);
  if(stat != asynSuccess) {
    return stat;
  }
  epicsUInt32 tempVal2 = tempVal1 | (value & mask);
  size_t bytesWritten = 0;
  return writeGeneric((uint8_t*)&tempVal2, sizeof(epicsUInt32),
                      asynParamUInt32Digital, &bytesWritten);
}

asynStatus  ecmcAsynDataItem::readFloat64(epicsFloat64 *value) {

  size_t bytesRead = 0;
  return readGeneric((uint8_t*)value, sizeof(epicsFloat64),
                     asynParamFloat64, &bytesRead);
}

asynStatus  ecmcAsynDataItem::writeFloat64(epicsFloat64 value) {

  size_t bytesWritten = 0;
  return writeGeneric((uint8_t*)&value, sizeof(epicsFloat64),
                      asynParamFloat64, &bytesWritten);

}

asynStatus ecmcAsynDataItem::readInt8Array(epicsInt8 *value,                   
                                           size_t nElements,
                                           size_t *nIn) {
  
  return readGeneric((uint8_t*)value, nElements * sizeof(epicsInt8),
                     asynParamInt8Array, nIn);
}

asynStatus ecmcAsynDataItem::writeInt8Array(epicsInt8 *value,
                                            size_t nElements) {

  size_t bytesWritten = 0;
  return writeGeneric((uint8_t*)&value, nElements * sizeof(epicsInt8),
                      asynParamInt8Array, &bytesWritten);
}

asynStatus ecmcAsynDataItem::readInt16Array(epicsInt16 *value,
                                       size_t nElements,
                                       size_t *nIn) {

  return readGeneric((uint8_t*)value, nElements * sizeof(epicsInt16),
                     asynParamInt16Array, nIn);
}

asynStatus ecmcAsynDataItem::writeInt16Array(epicsInt16 *value,
                                     size_t nElements) {
  size_t bytesWritten = 0;
  return writeGeneric((uint8_t*)&value, nElements * sizeof(epicsInt16),
                      asynParamInt16Array, &bytesWritten);
}

asynStatus ecmcAsynDataItem::readInt32Array(epicsInt32 *value,
                                      size_t nElements,
                                      size_t *nIn) {

 return readGeneric((uint8_t*)value, nElements * sizeof(epicsInt32),
                    asynParamInt32Array, nIn);
}

asynStatus ecmcAsynDataItem::writeInt32Array(epicsInt32 *value,
                                        size_t nElements) {
  size_t bytesWritten = 0;
  return writeGeneric((uint8_t*)&value, nElements * sizeof(epicsInt32),
                      asynParamInt32Array, &bytesWritten);
}

asynStatus ecmcAsynDataItem::readFloat32Array(epicsFloat32 *value,
                                         size_t nElements,
                                         size_t *nIn) {

  return readGeneric((uint8_t*)value, nElements * sizeof(epicsFloat32), 
                     asynParamFloat32Array, nIn);
}

asynStatus ecmcAsynDataItem::writeFloat32Array(epicsFloat32 *value,
                                          size_t nElements) {
  size_t bytesWritten = 0;
  return writeGeneric((uint8_t*)&value, nElements * sizeof(epicsFloat32),
                      asynParamFloat32Array, &bytesWritten);
}

asynStatus ecmcAsynDataItem::readFloat64Array(epicsFloat64 *value,
                                         size_t nElements,
                                         size_t *nIn) {

  return readGeneric((uint8_t*)value, nElements * sizeof(epicsFloat64), 
                     asynParamFloat64Array, nIn);
}

asynStatus ecmcAsynDataItem::writeFloat64Array(epicsFloat64 *value,
                                          size_t nElements) {

  size_t bytesWritten = 0;
  return writeGeneric((uint8_t*)&value, nElements * sizeof(epicsFloat64),
                      asynParamFloat64Array, &bytesWritten);
}

void ecmcAsynDataItem::setEcmcMaxValueInt(int64_t intMax) {
  checkIntRange_ = 1;
  intMax_ = intMax;  
}

void ecmcAsynDataItem::setEcmcMinValueInt(int64_t intMin) {
  checkIntRange_ = 1;
  intMin_ = intMin;
}

void ecmcAsynDataItem::setEcmcBitCount(size_t bits) {
   intBits_ = bits;
}

int64_t ecmcAsynDataItem::getEcmcMaxValueInt() {  
  return intMax_;
}

int64_t ecmcAsynDataItem::getEcmcMinValueInt() {
  return intMin_;
}

size_t ecmcAsynDataItem::getEcmcBitCount() {
  return intBits_;
}

void ecmcAsynDataItem:: setArrayCheckSize(bool check) {
  paramInfo_.arrayCheckSize=check;
}

bool ecmcAsynDataItem:: getArrayCheckSize() {
  return paramInfo_.arrayCheckSize;
}  

/** Validates drvInfo string
 * \param[in] drvInfo String containing information about the parameter.
 * \return asynSuccess or asynError.
 * The drvInfo string is what is after the asyn() in the "INP" or "OUT"
 * field of an record.
 */
asynStatus ecmcAsynDataItem::validateDrvInfo(const char *drvInfo)
{
  const char* functionName = "validateDrvInfo";
  asynPrint(asynPortDriver_->getTraceAsynUser(),ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName,drvInfo);

  if(strlen(drvInfo)==0){
    asynPrint(asynPortDriver_->getTraceAsynUser(),ASYN_TRACE_ERROR,"Invalid drvInfo string: Length 0 (%s).\n",drvInfo);
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

  asynPrint(asynPortDriver_->getTraceAsynUser(),ASYN_TRACE_ERROR,"Invalid drvInfo string (%s).\n", drvInfo);
  
  return asynError;
}

/** Get asyn type from record.
 * \param[in] drvInfo String containing information about the parameter.
 * \param[in/out] paramInfo Parameter information structure.
 * \return asynSuccess or asynError.
 */
asynStatus ecmcAsynDataItem::getRecordInfoFromDrvInfo(const char *drvInfo)
{
  const char* functionName = "getRecordInfoFromDrvInfo";
  asynPrint(asynPortDriver_->getTraceAsynUser(),ASYN_TRACE_FLOW, "%s:%s: drvInfo: %s\n", driverName, functionName, drvInfo);

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
    paramInfo_.recordType=strdup(dbGetRecordTypeName(pdbentry));
    status = dbFirstRecord(pdbentry);
    while(!status) {
      paramInfo_.recordName=strdup(dbGetRecordName(pdbentry));
      if(!dbIsAlias(pdbentry)){
        status=dbFindField(pdbentry,"INP");
        if(!status){
          paramInfo_.inp=strdup(dbGetString(pdbentry));

          isInput=true;
          char port[ECMC_MAX_FIELD_CHAR_LENGTH];
          int adr;
          int timeout;
          char currdrvInfo[ECMC_MAX_FIELD_CHAR_LENGTH];
          int nvals=sscanf(paramInfo_.inp,ECMC_ASYN_INP_FORMAT,port,&adr,&timeout,currdrvInfo);
          if(nvals==4){
            // Ensure correct port and drvinfo
            if(strcmp(port,asynPortDriver_->portName)==0 && strcmp(drvInfo,currdrvInfo)==0){
              recordFound=true;  // Correct port and drvinfo!\n");
            }
          } else {
            int mask=0;
            nvals=sscanf(paramInfo_.inp,ECMC_ASYN_MASK_INP_FORMAT,port,&adr,&mask,&timeout,currdrvInfo);
            if(nvals==5){
              // Ensure correct port and drvinfo
              if(strcmp(port,asynPortDriver_->portName)==0 && strcmp(drvInfo,currdrvInfo)==0){
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
          paramInfo_.out=strdup(dbGetString(pdbentry));
          isOutput=true;
          char port[ECMC_MAX_FIELD_CHAR_LENGTH];
          int adr;
          int timeout;
          char currdrvInfo[ECMC_MAX_FIELD_CHAR_LENGTH];
          int nvals=sscanf(paramInfo_.out,ECMC_ASYN_INP_FORMAT,port,&adr,&timeout,currdrvInfo);
          if(nvals==4){
            // Ensure correct port and drvinfo
            if(strcmp(port,asynPortDriver_->portName)==0 && strcmp(drvInfo,currdrvInfo)==0){
              recordFound=true;  // Correct port and drvinfo!\n");
            }
          } else {
            int mask=0;
            nvals=sscanf(paramInfo_.out,ECMC_ASYN_MASK_INP_FORMAT,port,&adr,&mask,&timeout,currdrvInfo);
            if(nvals==5){
              // Ensure correct port and drvinfo
              if(strcmp(port,asynPortDriver_->portName)==0 && strcmp(drvInfo,currdrvInfo)==0){
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
            paramInfo_.dtyp=strdup(dbGetString(pdbentry));
            paramInfo_.asynType=stringToAsynType(dbGetString(pdbentry));
          }
          else{
            paramInfo_.dtyp=0;
            paramInfo_.asynType=asynParamNotDefined;
          }

          //drvInput (not a field)
          paramInfo_.drvInfo=strdup(drvInfo);
          dbFreeEntry(pdbentry);
          // The correct record was found and the paramInfo structure is filled
          return asynSuccess;  
        }
        else{
          //Not correct record. Do cleanup.
          if(isInput){
            free(paramInfo_.inp);
            paramInfo_.inp=0;
          }
          if(isOutput){
            free(paramInfo_.out);
            paramInfo_.out=0;
          }
          paramInfo_.drvInfo=0;
          paramInfo_.scan=0;
          paramInfo_.dtyp=0;
          isInput=false;
          isOutput=false;
        }
      }
      status = dbNextRecord(pdbentry);
      free(paramInfo_.recordName);
      paramInfo_.recordName=0;
      recordFound = false;
    }
    status = dbNextRecordType(pdbentry);
    free(paramInfo_.recordType);
    paramInfo_.recordType=0;
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
asynStatus ecmcAsynDataItem::parseInfofromDrvInfo(const char* drvInfo)
{
  const char* functionName = "parseInfofromDrvInfo";
  asynPrint(asynPortDriver_->getTraceAsynUser(),ASYN_TRACE_FLOW,
            "%s:%s: drvInfo: %s\n",
            driverName,
            functionName,
            drvInfo);

  //Check if input or output
  paramInfo_.isIOIntr=false;
  const char* temp=strrchr(drvInfo,'?');
  if(temp){
    if(strlen(temp)==1){
      paramInfo_.isIOIntr=true; //All inputs will be created I/O intr
    }
  }

  asynPrint(asynPortDriver_->getTraceAsynUser(),ASYN_TRACE_FLOW, "%s:%s: drvInfo %s is %s\n",
            driverName,
            functionName,
            drvInfo,
            paramInfo_.isIOIntr ? "I/O Intr (end with ?)": "not I/O Intr (end with =)");

  //take part after last "/" if option or complete string..
  char buffer[ECMC_MAX_FIELD_CHAR_LENGTH];
  //See if option (find last '/')
  const char *drvInfoEnd=strrchr(drvInfo,'/');
  if(drvInfoEnd){ // found '/'
    int nvals=sscanf(drvInfoEnd,"/%s",buffer);
    if(nvals==1){
      paramInfo_.name=strdup(buffer);
      paramInfo_.name[strlen(paramInfo_.name)-1]=0; //Strip ? or = from end
    }
    else{
      asynPrint(asynPortDriver_->getTraceAsynUser(), ASYN_TRACE_ERROR,
                "%s:%s: Failed to parse PLC address string from drvInfo (%s)\n",
                driverName,
                functionName,
                drvInfo);
      return asynError;
    }
  }
  else{  //No options
    paramInfo_.name=strdup(drvInfo);
    paramInfo_.name[strlen(paramInfo_.name)-1]=0; //Strip ? or = from end
  }

  //Check if ECMC_OPTION_T_SAMPLE_RATE_MS option
  const char *option=ECMC_OPTION_T_SAMPLE_RATE_MS;
  paramInfo_.sampleTimeMS=asynPortDriver_->getDefaultSampleTimeMs();
  const char *isThere=strstr(drvInfo,option);
  if(isThere){
    if(strlen(isThere)<(strlen(option)+strlen("=0/"))){
      asynPrint(asynPortDriver_->getTraceAsynUser(), ASYN_TRACE_ERROR,
                "%s:%s: Failed to parse %s option from drvInfo (%s). String to short.\n",
                driverName,
                functionName,
                option,
                drvInfo);
      return asynError;
    }

    int nvals = sscanf(isThere+strlen(option),"=%lf/",&paramInfo_.sampleTimeMS);
    if(nvals==1) {
      paramInfo_.sampleTimeCycles=(int32_t)paramInfo_.sampleTimeMS/1000.0*(double)MCU_FREQUENCY;
    } 
    else {
      paramInfo_.sampleTimeMS=asynPortDriver_->getDefaultSampleTimeMs();
      asynPrint(asynPortDriver_->getTraceAsynUser(), ASYN_TRACE_ERROR,
                "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
                driverName,
                functionName,
                option,
                drvInfo);
      return asynError;
    }
  }
  
  //Check if TYPE option
  option=ECMC_OPTION_TYPE;
  paramInfo_.asynTypeStr=NULL;
  paramInfo_.asynType=asynParamNotDefined;

  isThere=strstr(drvInfo,option);
  if(isThere){
    if(strlen(isThere)<(strlen(option)+strlen("=0/"))){
      asynPrint(asynPortDriver_->getTraceAsynUser(), ASYN_TRACE_ERROR,
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
      asynPrint(asynPortDriver_->getTraceAsynUser(), ASYN_TRACE_ERROR,
                "%s:%s: Failed to parse %s option from drvInfo (%s). Wrong format.\n",
                driverName,
                functionName,
                option,
                drvInfo);
      return asynError;
    }
    paramInfo_.asynTypeStr=strdup(buffer);
    paramInfo_.asynType=stringToAsynType(paramInfo_.asynTypeStr);
  }

  return asynSuccess;
}

asynStatus ecmcAsynDataItem::setDrvInfo(const char *drvInfo) {

  const char* functionName = "setDrvInfo";

  if(validateDrvInfo(drvInfo)!=asynSuccess){
    return asynError;
  }

  // Parse options and name
  asynStatus status=parseInfofromDrvInfo(drvInfo);
  if(status!=asynSuccess){  
    return asynError;
  }

  status=getRecordInfoFromDrvInfo(drvInfo);
  if(status!=asynSuccess){
    asynPrint(asynPortDriver_->getTraceAsynUser(), ASYN_TRACE_ERROR, "%s:%s: Failed to find record with drvInfo %s.\n", driverName, functionName,drvInfo);    
    return asynError;
  }

  return asynSuccess;
}
