/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEcAsyncSDO.cpp
*
*  Created on: Dec 15, 2015
*      Author: anderssandstrom
*
\*************************************************************************/

#include <vector>
#include <exception>
#include "ecmcEcAsyncSDO.h"


/**
 * Callback function for asynWrites (control word)
 * userObj = axis object
 * 
 * */ 
asynStatus asynWriteSDOCmd(void* data, size_t bytes, asynParamType asynParType,void *userObj) {
  if (!userObj) {
    return asynError;
  }
  printf("######################asynWriteSDOCmd\n");
  return ((ecmcEcAsyncSDO*)userObj)->asynWriteSDO(data, bytes, asynParType);
}

asynStatus asynReadSDOCmd(void* data, size_t bytes, asynParamType asynParType,void *userObj) {
  if (!userObj) {
    return asynError;
  }
  printf("######################asynReadSDOCmd\n");
  return ((ecmcEcAsyncSDO*)userObj)->asynReadSDO(data, bytes, asynParType);
}

ecmcEcAsyncSDO::ecmcEcAsyncSDO(ecmcAsynPortDriver *asynDriver,                               
                               int masterId,
                               int slaveId,                               
                               ec_slave_config_t *sc, /**< Slave configuration. */
                               uint16_t sdoIndex, /**< SDO index. */
                               uint8_t sdoSubindex, /**< SDO subindex. */
                               ecmcEcDataType dt,
                               std::string alias)
{
  printf("m%ds%d, 0x%x, 0x%x, %d, %s\n",masterId, slaveId,sdoIndex,sdoSubindex,getEcDataTypeByteSize(dt),alias.c_str());
  masterId_       = masterId;
  slaveId_        = slaveId;
  asynPortDriver_ = asynDriver;
  sc_             = sc;
  sdoIndex        = sdoIndex;
  sdoSubindex     = sdoSubindex;
  dt_             = dt;
  buffer_         = 0;
  asynParamWrite_ = NULL;
  asynParamRead_  = NULL;
  asynParamValue_ = NULL;
  asynParamError_ = NULL;
  asynParamBusy_  = NULL;
  sdoError_       = 0;
  dummyReadCmd_   = 0;
  dummyWriteCmd_  = 0;
  usedSizeBytes_  = 0;
  bitLength_      = getEcDataTypeBits(dt);
  size_           = getEcDataTypeByteSize(dt);
  busy_           = 0;
  writeCmdInProcess_ = 0;
  readCmdInProcess_ = 0;
  idString_       = alias;
  idStringChar_   = strdup(idString_.c_str());


  // create object
  sdoreq_= ecrt_slave_config_create_sdo_request(
                         sc_, /**< Slave configuration. */
                         index_, /**< SDO index. */
                         subindex_, /**< SDO subindex. */
                         size_
                         );
  if(sdoreq_ == NULL) {
    LOGERR(
      "%s/%s:%d: Error: Failed creation of SDO async object.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

     throw std::bad_alloc();  
  }

  ecrt_sdo_request_timeout(sdoreq_,DEFAULT_SDO_ASYNC_TIMOUT_MS);

  if(initAsyn() != 0) {
    LOGERR(
      "%s/%s:%d: Error: initAsyn() failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

    throw std::bad_alloc();  
  }
}

ecmcEcAsyncSDO::~ecmcEcAsyncSDO()
{
  free(idStringChar_);
}

asynStatus ecmcEcAsyncSDO::asynWriteSDO(void* data,
                                        size_t bytes,
                                        asynParamType asynParType) {
  if (!sdoreq_) {
    LOGERR(
      "%s/%s:%d: Error: SDO write failed. SDO async object NULL (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,ERROR_EC_SDO_ASYNC_OBJ_NULL);
    sdoError_ = ERROR_EC_SDO_ASYNC_OBJ_NULL;
    asynParamError_->refreshParam(1);
    return asynError;
  }

  

  ec_request_state_t state=ecrt_sdo_request_state(sdoreq_);
  if(state == EC_REQUEST_BUSY) {
    LOGERR(
      "%s/%s:%d: Error: SDO write failed. SDO object busy (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_BUSY);
     sdoError_ = ERROR_EC_SDO_ASYNC_BUSY;
     asynParamError_->refreshParam(1);
    return asynError;
  }

  busy_ = 1;
  asynParamBusy_->refreshParam(1);
  writeCmdInProcess_ = true;
  //write data to sdo req obj (buffer_ data should already be updated by asyn)
  writeValue();

  //request/initiate write
  ecrt_sdo_request_write(sdoreq_);

  return asynSuccess;
}

asynStatus ecmcEcAsyncSDO::asynReadSDO(void* data,
                                       size_t bytes,
                                       asynParamType asynParType) {

  if (!sdoreq_) {
    LOGERR(
      "%s/%s:%d: Error: SDO write failed. SDO async object NULL (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,ERROR_EC_SDO_ASYNC_OBJ_NULL);
    sdoError_ = ERROR_EC_SDO_ASYNC_OBJ_NULL;
    asynParamError_->refreshParam(1);
    return asynError;
  }

  ec_request_state_t state = ecrt_sdo_request_state(sdoreq_);
  if(state == EC_REQUEST_BUSY) {
    LOGERR(
      "%s/%s:%d: Error: SDO write failed. SDO object busy (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_BUSY);
     sdoError_ = ERROR_EC_SDO_ASYNC_BUSY;
     asynParamError_->refreshParam(1);
    return asynError;
  }

  busy_ = 1;
  asynParamBusy_->refreshParam(1);
  
  ecrt_sdo_request_read(sdoreq_);
  readCmdInProcess_ = true;
  return asynSuccess;
}

int ecmcEcAsyncSDO::execute() { 
  if(!readCmdInProcess_ && !writeCmdInProcess_ ) {
    return 0;
  }

  ec_request_state_t state = ecrt_sdo_request_state(sdoreq_);

  switch(state) {
    case EC_REQUEST_UNUSED:

      break;

    case EC_REQUEST_BUSY:
      return 0; // keep waiting for finalizing request      
      break;

    case EC_REQUEST_SUCCESS:
      if(readCmdInProcess_) {
        readValue();
        asynParamValue_->refreshParamRT(1);        
      } 
      readCmdInProcess_ = 0;
      writeCmdInProcess_ = 0;
      busy_ = 0;
      asynParamBusy_->refreshParamRT(1);
      break;
    case EC_REQUEST_ERROR:
      sdoError_= ERROR_EC_SDO_ASYNC_ERROR;
      asynParamError_->refreshParamRT(1);
      readCmdInProcess_ = 0;
      writeCmdInProcess_ = 0;
      busy_ = 0;
      asynParamBusy_->refreshParamRT(1);
      break;
  }
  return 0;        
}

int ecmcEcAsyncSDO::initAsyn() {
  
  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];  
  char *name = buffer;

  // "ec%d.s%d.sdo.value"
  unsigned int charCount = snprintf(buffer,
                                    sizeof(buffer),
                                    ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d." ECMC_SDO_STR ".%s." ECMC_VALUE_STR,
                                    masterId_,
                                    slaveId_,
                                    idStringChar_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
  }
  name = buffer;
  asynParamValue_ = asynPortDriver_->addNewAvailParam(name,
                                    asynParamInt32,  //default type
                                    (uint8_t *)&(buffer_),
                                    sizeof(buffer_),
                                    dt_,
                                    0);
  if(!asynParamValue_) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MAIN_ASYN_CREATE_PARAM_FAIL);
  }

  //Add supported types
  switch(dt_) {
    case ECMC_EC_NONE:
      return 0;
      break;

    case ECMC_EC_B1:
      asynParamValue_->addSupportedAsynType(asynParamInt32);
      asynParamValue_->addSupportedAsynType(asynParamUInt32Digital);
      asynParamValue_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 1;
      break;

    case ECMC_EC_B2:
      asynParamValue_->addSupportedAsynType(asynParamInt32);
      asynParamValue_->addSupportedAsynType(asynParamUInt32Digital);
      asynParamValue_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 1;
      break;

    case ECMC_EC_B3:
      asynParamValue_->addSupportedAsynType(asynParamInt32);
      asynParamValue_->addSupportedAsynType(asynParamUInt32Digital);
      asynParamValue_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 1;
      break;

    case ECMC_EC_B4:
      asynParamValue_->addSupportedAsynType(asynParamInt32);
      asynParamValue_->addSupportedAsynType(asynParamUInt32Digital);
      asynParamValue_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 1;
      break;

    case ECMC_EC_U8:
      asynParamValue_->addSupportedAsynType(asynParamInt32);
      asynParamValue_->addSupportedAsynType(asynParamUInt32Digital);
      asynParamValue_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 1;
      break;

    case ECMC_EC_S8:
      asynParamValue_->addSupportedAsynType(asynParamInt32);
      asynParamValue_->addSupportedAsynType(asynParamUInt32Digital);
      asynParamValue_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 1;
      break;

    case ECMC_EC_U16:
      asynParamValue_->addSupportedAsynType(asynParamInt32);
      asynParamValue_->addSupportedAsynType(asynParamUInt32Digital);
      asynParamValue_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 2;
      break;

    case ECMC_EC_S16:
      asynParamValue_->addSupportedAsynType(asynParamInt32);
      asynParamValue_->addSupportedAsynType(asynParamUInt32Digital);
      asynParamValue_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 2;
      break;

    case ECMC_EC_U32:
      asynParamValue_->addSupportedAsynType(asynParamInt32);
      asynParamValue_->addSupportedAsynType(asynParamUInt32Digital);
      asynParamValue_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 4;
      break;

    case ECMC_EC_S32:
      asynParamValue_->addSupportedAsynType(asynParamInt32);
      asynParamValue_->addSupportedAsynType(asynParamUInt32Digital);
      asynParamValue_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 4;
      break;

    case ECMC_EC_U64:
      asynParamValue_->addSupportedAsynType(asynParamInt32);
      asynParamValue_->addSupportedAsynType(asynParamUInt32Digital);
      asynParamValue_->addSupportedAsynType(asynParamFloat64);

#ifdef ECMC_ASYN_ASYNPARAMINT64
      asynParamValue_->addSupportedAsynType(asynParamInt64);
#endif //ECMC_ASYN_ASYNPARAMINT64

      usedSizeBytes_ = 8;
      break;

    case ECMC_EC_S64:
      asynParamValue_->addSupportedAsynType(asynParamInt32);
      asynParamValue_->addSupportedAsynType(asynParamUInt32Digital);      
      asynParamValue_->addSupportedAsynType(asynParamFloat64);

#ifdef ECMC_ASYN_ASYNPARAMINT64
      asynParamValue_->addSupportedAsynType(asynParamInt64);
#endif //ECMC_ASYN_ASYNPARAMINT64

      usedSizeBytes_ = 8;
      break;

    case ECMC_EC_F32:
      asynParamValue_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 4;
      break;

    case ECMC_EC_F64:
      asynParamValue_->addSupportedAsynType(asynParamFloat64);
      usedSizeBytes_ = 8;      
      break;
  }
  asynParamValue_->setEcmcDataSize(usedSizeBytes_);

  asynParamValue_->setAllowWriteToEcmc(true);
  asynParamValue_->setEcmcBitCount(bitLength_);
  asynParamValue_->setEcmcMinValueInt(getEcDataTypeMinVal(dt_));
  asynParamValue_->setEcmcMaxValueInt(getEcDataTypeMaxVal(dt_));
  
  asynParamValue_->refreshParam(1);

  // "ec%d.s%d.sdo.error"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d." ECMC_SDO_STR ".%s." ECMC_ERROR_STR,
                       masterId_,
                       slaveId_,
                       idStringChar_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
  }
  name = buffer;
  asynParamError_ = asynPortDriver_->addNewAvailParam(name,
                                    asynParamInt32,  //default type
                                    (uint8_t *)&(sdoError_),
                                    sizeof(sdoError_),
                                    ECMC_EC_S32,
                                    0);
  if(!asynParamError_) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MAIN_ASYN_CREATE_PARAM_FAIL);
  }  
  asynParamError_->refreshParam(1);

  // "ec%d.s%d.sdo.readcmd"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d." ECMC_SDO_STR ".%s." ECMC_READCMD_STR,
                       masterId_,
                       slaveId_,
                       idStringChar_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
  }
  name = buffer;
  asynParamRead_ = asynPortDriver_->addNewAvailParam(name,
                                    asynParamInt32,  //default type
                                    (uint8_t *)&(dummyReadCmd_),
                                    sizeof(dummyReadCmd_),
                                    ECMC_EC_S32,
                                    0);
  if(!asynParamRead_) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MAIN_ASYN_CREATE_PARAM_FAIL);
  }
  asynParamRead_->setAllowWriteToEcmc(true);
  asynParamRead_->setExeCmdFunctPtr(asynReadSDOCmd,this); // Access to this axis 
  asynParamRead_->refreshParam(1);

  // "ec%d.s%d.sdo.writecmd"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d." ECMC_SDO_STR ".%s." ECMC_WRITECMD_STR,
                       masterId_,
                       slaveId_,
                       idStringChar_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
  }
  name = buffer;
  asynParamWrite_ = asynPortDriver_->addNewAvailParam(name,
                                    asynParamInt32,  //default type
                                    (uint8_t *)&(dummyWriteCmd_),
                                    sizeof(dummyWriteCmd_),
                                    ECMC_EC_S32,
                                    0);
  if(!asynParamWrite_) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MAIN_ASYN_CREATE_PARAM_FAIL);
  }
  asynParamWrite_->setAllowWriteToEcmc(true);
  asynParamWrite_->setExeCmdFunctPtr(asynWriteSDOCmd,this); // Access to this axis 
  asynParamWrite_->refreshParam(1);

  // "ec%d.s%d.sdo.busy"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d." ECMC_SDO_STR ".%s." ECMC_BUSY_STR,
                       masterId_,
                       slaveId_,
                       idStringChar_);

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: Error: Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
  }
  name = buffer;
  asynParamBusy_ = asynPortDriver_->addNewAvailParam(name,
                                    asynParamInt32,  //default type
                                    (uint8_t *)&(busy_),
                                    sizeof(busy_),
                                    ECMC_EC_S32,
                                    0);
  if(!asynParamBusy_) {
    LOGERR(
      "%s/%s:%d: ERROR: Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      name);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_MAIN_ASYN_CREATE_PARAM_FAIL);
  }
  asynParamBusy_->refreshParam(1);
  
  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST, ECMC_ASYN_DEFAULT_ADDR);

  return 0;
}

int ecmcEcAsyncSDO::readValue() {

  switch(dt_) {
    case ECMC_EC_NONE:
      buffer_ = 0;
      break;

    case ECMC_EC_B1:
      buffer_ = 0;
      break;

    case ECMC_EC_B2:
      buffer_ = 0;
      break;

    case ECMC_EC_B3:
      buffer_ = 0;
      break;

    case ECMC_EC_B4:
      buffer_ = 0;
      break;

    case ECMC_EC_U8:
      buffer_ = (uint64_t)EC_READ_U8(ecrt_sdo_request_data(sdoreq_));
      break;

    case ECMC_EC_S8:
      buffer_ = (uint64_t)EC_READ_S8(ecrt_sdo_request_data(sdoreq_));;
      break;

    case ECMC_EC_U16:
      buffer_ = (uint64_t)EC_READ_U16(ecrt_sdo_request_data(sdoreq_));
      break;

    case ECMC_EC_S16:
      buffer_ = (uint64_t)EC_READ_S16(ecrt_sdo_request_data(sdoreq_));
      break;

    case ECMC_EC_U32:
      buffer_ = (uint64_t)EC_READ_U32(ecrt_sdo_request_data(sdoreq_));
      break;

    case ECMC_EC_S32:
      buffer_ = (uint64_t)EC_READ_S32(ecrt_sdo_request_data(sdoreq_));
      break;
#ifdef EC_READ_U64
    case ECMC_EC_U64:
      buffer_ = (uint64_t)EC_READ_U64(ecrt_sdo_request_data(sdoreq_));
      break;
#endif

#ifdef EC_READ_S64
    case ECMC_EC_S64:
      buffer_ = (uint64_t)EC_READ_S64(ecrt_sdo_request_data(sdoreq_));
      break;
#endif

#ifdef EC_READ_REAL
    case ECMC_EC_F32:
      *float32Ptr_ = EC_READ_REAL(ecrt_sdo_request_data(sdoreq_));      
      break;
#endif

#ifdef EC_READ_LREAL
    case ECMC_EC_F64:
      *float64Ptr_ = EC_READ_LREAL(ecrt_sdo_request_data(sdoreq_));
      break;
#endif      
    default:
      buffer_ = 0;      
      break;
  }

  return 0;
}

int ecmcEcAsyncSDO::writeValue() {

  switch(dt_) {
    case ECMC_EC_NONE:
      buffer_ = 0;
      break;

    case ECMC_EC_B1:
      buffer_ = 0;
      break;

    case ECMC_EC_B2:
      buffer_ = 0;
      break;

    case ECMC_EC_B3:
      buffer_ = 0;
      break;

    case ECMC_EC_B4:
      buffer_ = 0;
      break;

    case ECMC_EC_U8:
      EC_WRITE_U8(ecrt_sdo_request_data(sdoreq_), buffer_);
      break;

    case ECMC_EC_S8:
      EC_WRITE_S8(ecrt_sdo_request_data(sdoreq_), buffer_);
      break;

    case ECMC_EC_U16:
      EC_WRITE_U16(ecrt_sdo_request_data(sdoreq_), buffer_);
      break;

    case ECMC_EC_S16:
      EC_WRITE_S16(ecrt_sdo_request_data(sdoreq_), buffer_);
      break;

    case ECMC_EC_U32:
      EC_WRITE_U32(ecrt_sdo_request_data(sdoreq_), buffer_);
      break;

    case ECMC_EC_S32:
      EC_WRITE_S32(ecrt_sdo_request_data(sdoreq_), buffer_);
      break;

#ifdef EC_WRITE_U64
    case ECMC_EC_U64:
      EC_WRITE_U64(ecrt_sdo_request_data(sdoreq_), buffer_);
      break;
#endif

#ifdef EC_WRITE_S64
    case ECMC_EC_S64:
      EC_WRITE_S64(ecrt_sdo_request_data(sdoreq_), buffer_);
      break;
#endif

#ifdef EC_WRITE_REAL
    case ECMC_EC_F32:
      EC_WRITE_REAL(ecrt_sdo_request_data(sdoreq_), *float32Ptr_);
      break;
#endif

#ifdef EC_WRITE_LREAL
    case ECMC_EC_F64:
      EC_WRITE_LREAL(ecrt_sdo_request_data(sdoreq_), *float64Ptr_);
      break;
#endif

    default:
      buffer_ = 0;      
      break;
  }

  return 0;
}
