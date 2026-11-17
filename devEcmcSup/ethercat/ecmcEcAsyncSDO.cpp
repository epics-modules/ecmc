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
#include <stdexcept>
#include <cmath>
#include <limits>
#include <algorithm>
#include "ecmcEcAsyncSDO.h"
#include "ecmcRtLogger.h"


namespace {
template<class T> T sdoNativeScalar(uint64_t storage) {
  T value; memcpy(&value,&storage,sizeof(value)); return value;
}
asynStatus asynWriteSdoScalar(void *data, size_t bytes,
                              asynParamType type, void *object) {
  return object ? static_cast<ecmcEcAsyncSDO *>(object)->asynWriteScalar(data, bytes, type) : asynError;
}
// Pack native scalar storage as little-endian wire bits, including floats.
void sdoEncodeScalar(uint64_t native, size_t bytes, uint8_t *wire) {
  uint64_t bits = 0;
  switch (bytes) {
    case 1: { uint8_t value; memcpy(&value,&native,1); bits=value; break; }
    case 2: { uint16_t value; memcpy(&value,&native,2); bits=value; break; }
    case 4: { uint32_t value; memcpy(&value,&native,4); bits=value; break; }
    case 8: bits=native; break;
  }
  for (size_t i=0;i<bytes;++i) wire[i]=static_cast<uint8_t>(bits>>(8*i));
}
uint64_t sdoDecodeScalar(const uint8_t *wire, size_t bytes) {
  uint64_t bits=0, native=0;
  for (size_t i=0;i<bytes;++i) bits|=uint64_t(wire[i])<<(8*i);
  switch (bytes) {
    case 1: { uint8_t value=static_cast<uint8_t>(bits); memcpy(&native,&value,1); break; }
    case 2: { uint16_t value=static_cast<uint16_t>(bits); memcpy(&native,&value,2); break; }
    case 4: { uint32_t value=static_cast<uint32_t>(bits); memcpy(&native,&value,4); break; }
    case 8: native=bits; break;
  }
  return native;
}
}

/**
 * Callback function for asynWrites (control word)
 * userObj = axis object
 *
 * */
asynStatus asynWriteSDOCmd(void         *data,
                           size_t        bytes,
                           asynParamType asynParType,
                           void         *userObj) {
  if (!userObj) {
    return asynError;
  }
  return ((ecmcEcAsyncSDO *)userObj)->asynWriteSDO(data, bytes, asynParType);
}

asynStatus asynReadSDOCmd(void         *data,
                          size_t        bytes,
                          asynParamType asynParType,
                          void         *userObj) {
  if (!userObj) {
    return asynError;
  }
  return ((ecmcEcAsyncSDO *)userObj)->asynReadSDO(data, bytes, asynParType);
}

static asynStatus asynWriteSDOArray(void *data, size_t bytes,
                                    asynParamType type, void *userObj) {
  return static_cast<ecmcEcAsyncSDO *>(userObj)->asynWriteArray(data, bytes, type);
}

ecmcEcAsyncSDO::ecmcEcAsyncSDO(ecmcAsynPortDriver *asynDriver,
                               int                 masterId,
                               int                 slaveId,
                               ec_slave_config_t  *sc, /**< Slave configuration. */
                               uint16_t            sdoIndex, /**< SDO index. */
                               uint8_t             sdoSubindex, /**< SDO subindex. */
                               ecmcEcDataType      dt,
                               std::string         alias,
                               size_t              arrayBytes) {
  if (arrayBytes > ECMC_SDO_ASYNC_MAX_BYTES) {
    throw std::invalid_argument("Async SDO array exceeds maximum size");
  }
  arrayReadBuffer_.resize(arrayBytes, 0);
  arrayWriteBuffer_.resize(arrayBytes, 0);
  arrayWriteReady_ = false;
  arrayWriteInitialized_.resize(arrayBytes, 0);
  arrayInitializedCount_ = 0;
  arrayValidBytes_ = 0;
  arrayReadBytes_ = 0;
  asynParamReadBytes_ = NULL;
  asynParamWriteValue_ = NULL;
  masterId_          = masterId;
  slaveId_           = slaveId;
  asynPortDriver_    = asynDriver;
  sc_                = sc;
  index_             = sdoIndex;
  subindex_          = sdoSubindex;
  dt_                = dt;
  buffer_            = 0;
  asynParamWrite_    = NULL;
  asynParamRead_     = NULL;
  asynParamValue_    = NULL;
  asynParamError_    = NULL;
  asynParamBusy_     = NULL;
  sdoError_          = 0;
  dummyReadCmd_      = 0;
  dummyWriteCmd_     = 0;
  usedSizeBytes_     = 0;
  readTrigg_         = 0;
  writeTrigg_        = 0;
  bitLength_         = getEcDataTypeBits(dt);
  size_              = arrayBytes ? arrayBytes : getEcDataTypeByteSize(dt);
  busy_              = 0;
  writeCmdInProcess_ = 0;
  readCmdInProcess_  = 0;
  idString_          = alias;
  idStringChar_      = strdup(idString_.c_str());
  stateOld_          = EC_REQUEST_UNUSED;
  int8Ptr_           = (int8_t *)&buffer_;
  uint8Ptr_          = (uint8_t *)&buffer_;
  int16Ptr_          = (int16_t *)&buffer_;
  uint16Ptr_         = (uint16_t *)&buffer_;
  int32Ptr_          = (int32_t *)&buffer_;
  uint32Ptr_         = (uint32_t *)&buffer_;
  int64Ptr_          = (int64_t *)&buffer_;
  uint64Ptr_         = (uint64_t *)&buffer_;
  float32Ptr_        = (float *)&buffer_;
  float64Ptr_        = (double *)&buffer_;

  // create object
  sdoreq_ = ecrt_slave_config_create_sdo_request(
    sc_,                      /**< Slave configuration. */
    index_,                      /**< SDO index. */
    subindex_,                      /**< SDO subindex. */
    size_
    );

  if (sdoreq_ == NULL) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Failed creation of SDO async object.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

    throw std::bad_alloc();
  }

  // Keep upload and download state independent. Queued writes are followed by
  // an upload for verification; reusing one EtherLab request can leave the
  // verification observing the preceding download request's state.
  readreq_ = ecrt_slave_config_create_sdo_request(
    sc_, index_, subindex_, size_);
  if (!readreq_) {
    throw std::bad_alloc();
  }
  ecrt_sdo_request_timeout(readreq_, DEFAULT_SDO_ASYNC_TIMOUT_MS);
  ecrt_sdo_request_timeout(sdoreq_, DEFAULT_SDO_ASYNC_TIMOUT_MS);

  if (initAsyn() != 0) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: initAsyn() failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

    throw std::bad_alloc();
  }
}

ecmcEcAsyncSDO::~ecmcEcAsyncSDO() {
  free(idStringChar_);
}

asynStatus ecmcEcAsyncSDO::asynWriteArray(void *data, size_t bytes,
                                         asynParamType type) {
  // Reject partial/oversized writes rather than downloading stale tail bytes.
  if (!data || arrayWriteBuffer_.empty() || type != asynParamInt8Array ||
      bytes != size_ || busy_) {
    sdoError_ = busy_ ? ERROR_EC_SDO_ASYNC_BUSY : ERROR_EC_SDO_ASYNC_SIZE;
    asynParamError_->refreshParam(1);
    return asynError;
  }
  memcpy(arrayWriteBuffer_.data(), data, size_);
  arrayWriteReady_ = true;
  asynParamWriteValue_->refreshParam(1);
  return asynSuccess;
}

asynStatus ecmcEcAsyncSDO::asynWriteSDO(void         *data,
                                        size_t        bytes,
                                        asynParamType asynParType) {
  if ((bytes != 4) || (asynParType != asynParamInt32)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: SDO read failed. Write cmd trigger data size or datatype mismatch.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }

  int trigg = 0;
  memcpy(&trigg, data, bytes);

  // Check if positive edge
  if (!(trigg && !writeTrigg_)) {
    writeTrigg_ = trigg;
    return asynSuccess;
  }

  writeTrigg_ = trigg;
  return requestWrite() ? asynError : asynSuccess;
}

int ecmcEcAsyncSDO::requestWrite() {
  if (queue_) {
    if (preopOnly_ && queue_->running()) return reportQueueError(ERROR_EC_SDO_PREOP_ONLY);
    const bool wasIdle=queuedPending_==0;
    const int error=queue_->submit(*this,true,buffer_);
    if (!error) {
      if (wasIdle) startupWriteFailed_=false;
      ++pendingWrites_; startupUnverified_=true;
      rememberStartupWrite(buffer_);
    }
    return error ? reportQueueError(error) : 0;
  }

  if (!sdoreq_) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: SDO write failed. SDO async object is NULL (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__, ERROR_EC_SDO_ASYNC_OBJ_NULL);
    sdoError_ = ERROR_EC_SDO_ASYNC_OBJ_NULL;
    asynParamError_->refreshParam(1);
    return sdoError_;
  }


  ec_request_state_t state = ecrt_sdo_request_state(sdoreq_);

  if (busy_ || state == EC_REQUEST_BUSY) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: SDO write failed. SDO object is busy (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_BUSY);
    sdoError_ = ERROR_EC_SDO_ASYNC_BUSY;
    asynParamError_->refreshParam(1);
    return sdoError_;
  }

  if (!arrayWriteBuffer_.empty() && !arrayWriteReady_) {
    sdoError_ = ERROR_EC_SDO_ASYNC_SIZE;
    asynParamError_->refreshParam(1);
    return sdoError_;
  }
  sdoError_ = 0;
  asynParamError_->refreshParam(1);
  busy_ = 1;
  asynParamBusy_->refreshParam(1);
  writeCmdInProcess_ = true;

  // write data to sdo req obj (buffer_ data should already be updated by asyn)
  writeValue();

  if (asynParamWriteValue_) asynParamWriteValue_->refreshParamRT(1);
  // request/initiate write
  ecrt_sdo_request_write(sdoreq_);

  return 0;
}

asynStatus ecmcEcAsyncSDO::asynReadSDO(void         *data,
                                       size_t        bytes,
                                       asynParamType asynParType) {
  if ((bytes != 4) || (asynParType != asynParamInt32)) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: SDO read failed. Read cmd trigger data size or datatype mismatch..\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }

  int trigg = 0;
  memcpy(&trigg, data, bytes);

  // Check if positive edge
  if (!(trigg && !readTrigg_)) {
    readTrigg_ = trigg;
    return asynSuccess;
  }

  readTrigg_ = trigg;
  return requestRead() ? asynError : asynSuccess;
}

int ecmcEcAsyncSDO::requestRead() {
  if (queue_) {
    const int error=queue_->submit(*this,false,0);
    return error ? reportQueueError(error) : 0;
  }

  if (!readreq_) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: SDO read failed. SDO async object is NULL (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__, ERROR_EC_SDO_ASYNC_OBJ_NULL);
    sdoError_ = ERROR_EC_SDO_ASYNC_OBJ_NULL;
    asynParamError_->refreshParam(1);
    return sdoError_;
  }

  ec_request_state_t state = ecrt_sdo_request_state(readreq_);

  if (busy_ || state == EC_REQUEST_BUSY) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: SDO read failed. SDO object is busy (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_BUSY);
    sdoError_ = ERROR_EC_SDO_ASYNC_BUSY;
    asynParamError_->refreshParam(1);
    return sdoError_;
  }

  sdoError_ = 0;
  asynParamError_->refreshParam(1);
  busy_ = 1;
  asynParamBusy_->refreshParam(1);

  ecrt_sdo_request_read(readreq_);
  readCmdInProcess_ = true;
  return 0;
}

int ecmcEcAsyncSDO::execute() {
  if (queue_) return 0; // The master's queue owns polling and verification.
  if (!readCmdInProcess_ && !writeCmdInProcess_) {
    return 0;
  }

  ec_request_state_t state = ecrt_sdo_request_state(readCmdInProcess_ ? readreq_ : sdoreq_);

  stateOld_ = state;

  switch (state) {
  case EC_REQUEST_UNUSED:
    break;

  case EC_REQUEST_BUSY:
    return 0;   // keep waiting for finalizing request

    break;

  case EC_REQUEST_SUCCESS:

    if (readCmdInProcess_) {
      const int error = readValue();
      if (error) {
        sdoError_ = error;
        asynParamError_->refreshParamRT(1);
      } else {
        asynParamValue_->refreshParamRT(1);
      }
      if (asynParamReadBytes_) {
        asynParamReadBytes_->refreshParamRT(1);
      }
    }
    readCmdInProcess_  = 0;
    writeCmdInProcess_ = 0;
    busy_              = 0;
    asynParamBusy_->refreshParamRT(1);
    break;

  case EC_REQUEST_ERROR:
    sdoError_ = ERROR_EC_SDO_ASYNC_ERROR;
    asynParamError_->refreshParamRT(1);
    readCmdInProcess_  = 0;
    writeCmdInProcess_ = 0;
    busy_              = 0;
    asynParamBusy_->refreshParamRT(1);
    break;
  }
  return 0;
}

int ecmcEcAsyncSDO::initAsyn() {
  char  buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char *name = buffer;

  // "ec%d.s%d.sdo.value"
  unsigned int charCount = snprintf(buffer,
                                    sizeof(buffer),
                                    ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d." ECMC_SDO_STR ".%s." ECMC_VALUE_STR,
                                    masterId_,
                                    slaveId_,
                                    idStringChar_);

  if (charCount >= sizeof(buffer) - 1) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Failed to generate alias. Buffer too small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
  }
  name            = buffer;
  asynParamValue_ = asynPortDriver_->addNewAvailParam(name,
                                                      arrayReadBuffer_.empty() ? asynParamInt32 : asynParamInt8Array,
                                                      arrayReadBuffer_.empty() ? (uint8_t *)&buffer_ : arrayReadBuffer_.data(),
                                                      arrayReadBuffer_.empty() ? sizeof(buffer_) : size_,
                                                      dt_,
                                                      0);

  if (!asynParamValue_) {
    ecmcRtLoggerLogError(
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

  if (!arrayReadBuffer_.empty()) {
    asynParamValue_->setAllowWriteToEcmc(false);
    asynParamValue_->refreshParam(1, size_t(0));
    const std::string prefix = "ec" + std::to_string(masterId_) + ".s" +
      std::to_string(slaveId_) + ".sdo." + idString_;
    asynParamWriteValue_ = asynPortDriver_->addNewAvailParam(
      (prefix + ".writevalue").c_str(), asynParamInt8Array,
      arrayWriteBuffer_.data(), size_, ECMC_EC_U8, 0);
    asynParamReadBytes_ = asynPortDriver_->addNewAvailParam(
      (prefix + ".bytes").c_str(), asynParamInt32,
      reinterpret_cast<uint8_t *>(&arrayReadBytes_), sizeof(arrayReadBytes_),
      ECMC_EC_S32, 0);
    if (!asynParamWriteValue_ || !asynParamReadBytes_) {
      return ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL;
    }
    asynParamWriteValue_->setAllowWriteToEcmc(true);
    asynParamWriteValue_->setExeCmdFunctPtr(asynWriteSDOArray, this);
    asynParamWriteValue_->refreshParam(1);
    asynParamReadBytes_->setAllowWriteToEcmc(false);
    asynParamReadBytes_->refreshParam(1);
  } else {
    // Add supported types
    switch (dt_) {
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

    case ECMC_EC_S8_TO_U8:
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

    case ECMC_EC_S16_TO_U16:
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

    case ECMC_EC_S32_TO_U32:
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

    case ECMC_EC_S64_TO_U64:
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

  } // scalar value setup

  // "ec%d.s%d.sdo.error"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d." ECMC_SDO_STR ".%s." ECMC_ERROR_STR,
                       masterId_,
                       slaveId_,
                       idStringChar_);

  if (charCount >= sizeof(buffer) - 1) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Failed to generate alias. Buffer too small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
  }
  name            = buffer;
  asynParamError_ = asynPortDriver_->addNewAvailParam(name,
                                                      asynParamInt32, // default type
                                                      (uint8_t *)&(sdoError_),
                                                      sizeof(sdoError_),
                                                      ECMC_EC_S32,
                                                      0);

  if (!asynParamError_) {
    ecmcRtLoggerLogError(
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
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Failed to generate alias. Buffer too small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
  }
  name           = buffer;
  asynParamRead_ = asynPortDriver_->addNewAvailParam(name,
                                                     asynParamInt32,

                                                     // default type
                                                     (uint8_t *)&(dummyReadCmd_),
                                                     sizeof(dummyReadCmd_),
                                                     ECMC_EC_S32,
                                                     0);

  if (!asynParamRead_) {
    ecmcRtLoggerLogError(
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
  asynParamRead_->setExeCmdFunctPtr(asynReadSDOCmd, this); // Access to this axis
  asynParamRead_->refreshParam(1);

  // "ec%d.s%d.sdo.writecmd"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d." ECMC_SDO_STR ".%s." ECMC_WRITECMD_STR,
                       masterId_,
                       slaveId_,
                       idStringChar_);

  if (charCount >= sizeof(buffer) - 1) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Failed to generate alias. Buffer too small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
  }
  name            = buffer;
  asynParamWrite_ = asynPortDriver_->addNewAvailParam(name,
                                                      asynParamInt32, // default type
                                                      (uint8_t *)&(
                                                        dummyWriteCmd_),
                                                      sizeof(dummyWriteCmd_),
                                                      ECMC_EC_S32,
                                                      0);

  if (!asynParamWrite_) {
    ecmcRtLoggerLogError(
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
  asynParamWrite_->setExeCmdFunctPtr(asynWriteSDOCmd, this); // Access to this axis
  asynParamWrite_->refreshParam(1);

  // "ec%d.s%d.sdo.busy"
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_EC_STR "%d." ECMC_SLAVE_CHAR "%d." ECMC_SDO_STR ".%s." ECMC_BUSY_STR,
                       masterId_,
                       slaveId_,
                       idStringChar_);

  if (charCount >= sizeof(buffer) - 1) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Failed to generate alias. Buffer too small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL);
  }
  name           = buffer;
  asynParamBusy_ = asynPortDriver_->addNewAvailParam(name,
                                                     asynParamInt32, // default type
                                                     (uint8_t *)&(busy_),
                                                     sizeof(busy_),
                                                     ECMC_EC_S32,
                                                     0);

  if (!asynParamBusy_) {
    ecmcRtLoggerLogError(
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

  asynPortDriver_->callParamCallbacks(ECMC_ASYN_DEFAULT_LIST,
                                      ECMC_ASYN_DEFAULT_ADDR);

  return 0;
}

int ecmcEcAsyncSDO::readValue() {
  const size_t bytes = ecrt_sdo_request_data_size(readreq_);
  if (!arrayReadBuffer_.empty()) {
    arrayReadBytes_ = static_cast<int32_t>(bytes <= INT32_MAX ? bytes : INT32_MAX);
    if (bytes > size_) {
      return ERROR_EC_SDO_ASYNC_SIZE; // Retain previous good waveform.
    }
    memcpy(arrayReadBuffer_.data(), ecrt_sdo_request_data(readreq_), bytes);
    arrayValidBytes_ = static_cast<int>(bytes);
    asynParamValue_->setEcmcDataSize(bytes);
    return 0;
  }
  if (bytes != size_) {
    return ERROR_EC_SDO_ASYNC_SIZE;
  }
  switch (dt_) {
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
    *uint8Ptr_ = EC_READ_U8(ecrt_sdo_request_data(readreq_));
    break;

  case ECMC_EC_S8:
    *int8Ptr_ = EC_READ_S8(ecrt_sdo_request_data(readreq_));
    break;

  case ECMC_EC_S8_TO_U8:
    *uint8Ptr_ = static_cast<uint8_t>(EC_READ_S8(ecrt_sdo_request_data(readreq_)));
    break;

  case ECMC_EC_U16:
    *uint16Ptr_ = EC_READ_U16(ecrt_sdo_request_data(readreq_));
    break;

  case ECMC_EC_S16:
    *int16Ptr_ = EC_READ_S16(ecrt_sdo_request_data(readreq_));
    break;

  case ECMC_EC_S16_TO_U16:
    *uint16Ptr_ = static_cast<uint16_t>(EC_READ_S16(ecrt_sdo_request_data(readreq_)));
    break;

  case ECMC_EC_U32:
    *uint32Ptr_ = EC_READ_U32(ecrt_sdo_request_data(readreq_));
    break;

  case ECMC_EC_S32:
    *int32Ptr_ = EC_READ_S32(ecrt_sdo_request_data(readreq_));
    break;

  case ECMC_EC_S32_TO_U32:
    *uint32Ptr_ = static_cast<uint32_t>(EC_READ_S32(ecrt_sdo_request_data(readreq_)));
    break;

#ifdef EC_READ_U64
  case ECMC_EC_U64:
    *uint64Ptr_ = EC_READ_U64(ecrt_sdo_request_data(readreq_));
    break;
#endif // ifdef EC_READ_U64

#ifdef EC_READ_S64
  case ECMC_EC_S64:
    *int64Ptr_ = EC_READ_S64(ecrt_sdo_request_data(readreq_));
    break;

  case ECMC_EC_S64_TO_U64:
    *uint64Ptr_ = static_cast<uint64_t>(EC_READ_S64(ecrt_sdo_request_data(readreq_)));
    break;
#endif // ifdef EC_READ_S64

#ifdef EC_READ_REAL
  case ECMC_EC_F32:
    *float32Ptr_ = EC_READ_REAL(ecrt_sdo_request_data(readreq_));
    break;
#endif // ifdef EC_READ_REAL

#ifdef EC_READ_LREAL
  case ECMC_EC_F64:
    *float64Ptr_ = EC_READ_LREAL(ecrt_sdo_request_data(readreq_));
    break;

#endif // ifdef EC_READ_LREAL
  default:
    buffer_ = 0;
    break;
  }

  return 0;
}

int ecmcEcAsyncSDO::writeValue() {
  if (!arrayWriteBuffer_.empty()) {
    memcpy(ecrt_sdo_request_data(sdoreq_), arrayWriteBuffer_.data(), size_);
    return 0;
  }
  switch (dt_) {
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
    EC_WRITE_U8(ecrt_sdo_request_data(sdoreq_), *uint8Ptr_);
    break;

  case ECMC_EC_S8:
    EC_WRITE_S8(ecrt_sdo_request_data(sdoreq_), *int8Ptr_);
    break;

  case ECMC_EC_S8_TO_U8:
    EC_WRITE_S8(ecrt_sdo_request_data(sdoreq_), static_cast<int8_t>(*uint8Ptr_));
    break;

  case ECMC_EC_U16:
    EC_WRITE_U16(ecrt_sdo_request_data(sdoreq_), *uint16Ptr_);
    break;

  case ECMC_EC_S16:
    EC_WRITE_S16(ecrt_sdo_request_data(sdoreq_), *int16Ptr_);
    break;

  case ECMC_EC_S16_TO_U16:
    EC_WRITE_S16(ecrt_sdo_request_data(sdoreq_), static_cast<int16_t>(*uint16Ptr_));
    break;

  case ECMC_EC_U32:
    EC_WRITE_U32(ecrt_sdo_request_data(sdoreq_), *uint32Ptr_);
    break;

  case ECMC_EC_S32:
    EC_WRITE_S32(ecrt_sdo_request_data(sdoreq_), *int32Ptr_);
    break;

  case ECMC_EC_S32_TO_U32:
    EC_WRITE_S32(ecrt_sdo_request_data(sdoreq_), static_cast<int32_t>(*uint32Ptr_));
    break;

#ifdef EC_WRITE_U64
  case ECMC_EC_U64:
    EC_WRITE_U64(ecrt_sdo_request_data(sdoreq_), *uint64Ptr_);
    break;
#endif // ifdef EC_WRITE_U64

#ifdef EC_WRITE_S64
  case ECMC_EC_S64:
    EC_WRITE_S64(ecrt_sdo_request_data(sdoreq_), *int64Ptr_);
    break;

  case ECMC_EC_S64_TO_U64:
    EC_WRITE_S64(ecrt_sdo_request_data(sdoreq_), static_cast<int64_t>(*uint64Ptr_));
    break;
#endif // ifdef EC_WRITE_S64

#ifdef EC_WRITE_REAL
  case ECMC_EC_F32:
    EC_WRITE_REAL(ecrt_sdo_request_data(sdoreq_), *float32Ptr_);
    break;
#endif // ifdef EC_WRITE_REAL

#ifdef EC_WRITE_LREAL
  case ECMC_EC_F64:
    EC_WRITE_LREAL(ecrt_sdo_request_data(sdoreq_), *float64Ptr_);
    break;
#endif // ifdef EC_WRITE_LREAL

  default:
    buffer_ = 0;
    break;
  }

  return 0;
}

namespace {
template<typename T>
int sdoAssignValue(double value, T *target) {
  if (!std::isfinite(value)) return ERROR_EC_SDO_ASYNC_VALUE;
  if (std::numeric_limits<T>::is_integer) {
    const double upper = std::ldexp(1.0, std::numeric_limits<T>::digits);
    const double lower = std::numeric_limits<T>::is_signed ? -upper : 0.0;
    if (std::trunc(value) != value || value < lower || value >= upper) {
      return ERROR_EC_SDO_ASYNC_VALUE;
    }
  } else if (std::fabs(value) > std::numeric_limits<T>::max()) {
    return ERROR_EC_SDO_ASYNC_VALUE;
  }
  *target = static_cast<T>(value);
  return 0;
}
}

int ecmcEcAsyncSDO::getValue(int element, double *value) const {
  if (!value || element < 0 || element >= getSize()) {
    return ERROR_EC_SDO_ASYNC_SIZE;
  }
  if (!arrayReadBuffer_.empty()) {
    *value = arrayReadBuffer_[element];
    return 0;
  }
  const uint64_t storage=queue_ && readValid_ ? readBuffer_ : buffer_;
  switch (dt_) {
  case ECMC_EC_U8: *value = static_cast<double>(sdoNativeScalar<uint8_t>(storage)); return 0;
  case ECMC_EC_S8: *value = static_cast<double>(sdoNativeScalar<int8_t>(storage)); return 0;
  case ECMC_EC_S8_TO_U8: *value = static_cast<double>(sdoNativeScalar<uint8_t>(storage)); return 0;
  case ECMC_EC_U16: *value = static_cast<double>(sdoNativeScalar<uint16_t>(storage)); return 0;
  case ECMC_EC_S16: *value = static_cast<double>(sdoNativeScalar<int16_t>(storage)); return 0;
  case ECMC_EC_S16_TO_U16: *value = static_cast<double>(sdoNativeScalar<uint16_t>(storage)); return 0;
  case ECMC_EC_U32: *value = static_cast<double>(sdoNativeScalar<uint32_t>(storage)); return 0;
  case ECMC_EC_S32: *value = static_cast<double>(sdoNativeScalar<int32_t>(storage)); return 0;
  case ECMC_EC_S32_TO_U32: *value = static_cast<double>(sdoNativeScalar<uint32_t>(storage)); return 0;
  case ECMC_EC_U64: *value = static_cast<double>(sdoNativeScalar<uint64_t>(storage)); return 0;
  case ECMC_EC_S64: *value = static_cast<double>(sdoNativeScalar<int64_t>(storage)); return 0;
  case ECMC_EC_S64_TO_U64: *value = static_cast<double>(sdoNativeScalar<uint64_t>(storage)); return 0;
  case ECMC_EC_F32: *value = static_cast<double>(sdoNativeScalar<float>(storage)); return 0;
  case ECMC_EC_F64: *value = static_cast<double>(sdoNativeScalar<double>(storage)); return 0;
  default: return ERROR_EC_SDO_ASYNC_VALUE;
  }
}

int ecmcEcAsyncSDO::setValue(int element, double value) {
  if (busy_ && !queue_) return ERROR_EC_SDO_ASYNC_BUSY;
  if (element < 0 || element >= getCapacity()) return ERROR_EC_SDO_ASYNC_SIZE;
  if (!arrayWriteBuffer_.empty()) {
    const int error = sdoAssignValue(value, &arrayWriteBuffer_[element]);
    if (error) return error;
    if (!arrayWriteInitialized_[element]) {
      arrayWriteInitialized_[element] = 1;
      ++arrayInitializedCount_;
    }
    if (arrayInitializedCount_ == size_) arrayWriteReady_ = true;
    // Publish staged bytes when a download is requested, not once per byte.
    return 0;
  }
  int error = 0;
  switch (dt_) {
  case ECMC_EC_U8: error = sdoAssignValue(value, uint8Ptr_); break;
  case ECMC_EC_S8: error = sdoAssignValue(value, int8Ptr_); break;
  case ECMC_EC_S8_TO_U8: error = sdoAssignValue(value, uint8Ptr_); break;
  case ECMC_EC_U16: error = sdoAssignValue(value, uint16Ptr_); break;
  case ECMC_EC_S16: error = sdoAssignValue(value, int16Ptr_); break;
  case ECMC_EC_S16_TO_U16: error = sdoAssignValue(value, uint16Ptr_); break;
  case ECMC_EC_U32: error = sdoAssignValue(value, uint32Ptr_); break;
  case ECMC_EC_S32: error = sdoAssignValue(value, int32Ptr_); break;
  case ECMC_EC_S32_TO_U32: error = sdoAssignValue(value, uint32Ptr_); break;
  case ECMC_EC_U64: error = sdoAssignValue(value, uint64Ptr_); break;
  case ECMC_EC_S64: error = sdoAssignValue(value, int64Ptr_); break;
  case ECMC_EC_S64_TO_U64: error = sdoAssignValue(value, uint64Ptr_); break;
  case ECMC_EC_F32: error = sdoAssignValue(value, float32Ptr_); break;
  case ECMC_EC_F64: error = sdoAssignValue(value, float64Ptr_); break;
  default: return ERROR_EC_SDO_ASYNC_VALUE;
  }
  if (!error) {
    asynParamValue_->refreshParamRT(1);
    if (queue_ && writeOnValue_) return requestWrite();
  }
  return error;
}

int ecmcEcAsyncSDO::reportQueueError(int error) {
  if (!sdoError_) sdoError_=error;
  asynParamError_->refreshParam(1);
  return error;
}

int ecmcEcAsyncSDO::configureQueue(ecmcSdoQueue *queue, bool writeOnValue,
                                   bool preopOnly) {
  if (!queue || queue->running() || busy_ || !arrayWriteBuffer_.empty() ||
      (usedSizeBytes_!=1 && usedSizeBytes_!=2 && usedSizeBytes_!=4 && usedSizeBytes_!=8))
    return ERROR_EC_SDO_ASYNC_VALUE;
  if (!asynParamReadValue_) {
    const std::string name="ec"+std::to_string(masterId_)+".s"+
      std::to_string(slaveId_)+".sdo."+idString_+".readvalue";
    asynParamReadValue_=asynPortDriver_->addNewAvailParam(name.c_str(),
      asynParamInt32,reinterpret_cast<uint8_t*>(&readBuffer_),sizeof(readBuffer_),dt_,0);
    if (!asynParamReadValue_) return ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL;
    // Keep enough backing-buffer capacity for the EPICS scalar interfaces
    // (asynInt32/asynFloat64), while retaining the SDO's native data size.
    asynParamReadValue_->setEcmcDataSize(usedSizeBytes_);
    asynParamReadValue_->addSupportedAsynType(asynParamFloat64);
    asynParamReadValue_->addSupportedAsynType(asynParamUInt32Digital);
#ifdef ECMC_ASYN_ASYNPARAMINT64
    asynParamReadValue_->addSupportedAsynType(asynParamInt64);
#endif
    asynParamReadValue_->setAllowWriteToEcmc(false);
    asynParamReadValue_->refreshParam(1);
  }
  queue_=queue; writeOnValue_=writeOnValue; preopOnly_=preopOnly;
  asynParamValue_->setExeCmdFunctPtr(asynWriteSdoScalar,this);
  return 0;
}

asynStatus ecmcEcAsyncSDO::asynWriteScalar(void *data, size_t bytes,
                                         asynParamType type) {
  const int error=stageScalar(data,bytes,type);
  return error ? asynError : asynSuccess;
}

int ecmcEcAsyncSDO::stageScalar(void *data, size_t bytes, asynParamType type) {
  if (!queue_ || !data) return ERROR_EC_SDO_ASYNC_VALUE;
  double value=0;
  if (type==asynParamFloat64 && bytes==sizeof(double)) {
    memcpy(&value,data,bytes);
  } else if ((type==asynParamInt32 || type==asynParamUInt32Digital) && bytes==4) {
    int32_t raw; memcpy(&raw,data,4);
    value=(dt_==ECMC_EC_U32 || dt_==ECMC_EC_S32_TO_U32) ?
      static_cast<double>(static_cast<uint32_t>(raw)) : static_cast<double>(raw);
#ifdef ECMC_ASYN_ASYNPARAMINT64
  } else if (type==asynParamInt64 && bytes==8) {
    int64_t raw; memcpy(&raw,data,8);
    if (dt_==ECMC_EC_U64 || dt_==ECMC_EC_S64 || dt_==ECMC_EC_S64_TO_U64) {
      memcpy(&buffer_,data,8);
      asynParamValue_->refreshParam(1);
      return writeOnValue_ ? requestWrite() : 0;
    }
    value=static_cast<double>(raw);
#endif
  } else return ERROR_EC_SDO_ASYNC_VALUE;
  return setValue(0,value);
}

void ecmcEcAsyncSDO::queuedRequestAccepted() {
  if (!queuedPending_) sdoError_=0;
  ++queuedPending_; busy_=1;
  asynParamError_->refreshParam(1); asynParamBusy_->refreshParam(1);
}

void ecmcEcAsyncSDO::queuedRequestComplete(int error) {
  if (error) {
    ecmcRtLoggerLogError(
      "SDO queued request failed: ec%d.s%d %s 0x%04x:%02x, phase=%d, %zu bytes, error=0x%x.\n",
      masterId_,slaveId_,idStringChar_,index_,subindex_,
      static_cast<int>(queuePhase_),size_,error);
  }
  if (activeQueuedWrite_) {
    if (pendingWrites_) --pendingWrites_;
    if (error) startupWriteFailed_=true;
    startupUnverified_=pendingWrites_ || startupWriteFailed_;
  }
  if (error && !sdoError_) sdoError_=error; // Keep the first failure across the burst.
  if (queuedPending_) --queuedPending_;
  busy_=queuedPending_!=0;
  queuePhase_=QUEUE_IDLE;
  asynParamError_->refreshParamRT(1); asynParamBusy_->refreshParamRT(1);
}

void ecmcEcAsyncSDO::publishQueuedRead(uint64_t actual) {
  readBuffer_=actual; readValid_=true;
  asynParamReadValue_->refreshParamRT(1);
}

int ecmcEcAsyncSDO::configureStartupWrite(uint64_t value) {
  if (!queue_ || queue_->running() || !sc_)
    return reportQueueError(ERROR_EC_SDO_STARTUP_PENDING);
  // Mapping/assignment objects must remain under the PDO configuration owner.
  if ((index_>=0x1600 && index_<=0x1bff) || (index_>=0x1c10 && index_<=0x1c2f))
    return reportQueueError(ERROR_EC_SDO_CONFIG_CONFLICT);
  uint8_t wire[8]={};
  sdoEncodeScalar(value,size_,wire);
  if (ecrt_slave_config_sdo(sc_,index_,subindex_,wire,size_))
    return reportQueueError(ERROR_EC_SDO_ASYNC_ERROR);
  printf("SDO startup download registered: ec%d.s%d %s 0x%04x:%02x, %zu bytes, value=0x%llx (applied by EtherLab during activation).\n",
         masterId_,slaveId_,idStringChar_,index_,subindex_,size_,
         static_cast<unsigned long long>(value));
  return 0;
}

bool ecmcEcAsyncSDO::queuedTransferReady() const {
  if (!sc_) return false;
  ec_slave_config_state_t state={};
  ecrt_slave_config_state(sc_,&state);
  return state.operational;
}

int ecmcEcAsyncSDO::beginStartupVerification(uint64_t value) {
  if (!queue_ || !queue_->running()) return ERROR_EC_SDO_STARTUP_PENDING;
  if (ecrt_sdo_request_state(readreq_)==EC_REQUEST_BUSY) return ERROR_EC_SDO_ASYNC_BUSY;
  sdoEncodeScalar(value,size_,expectedWire_);
  ecmcRtLoggerLogInfo("SDO startup verification starting: ec%d.s%d %s 0x%04x:%02x, %zu bytes, expected=0x%llx.\n",
                     masterId_,slaveId_,idStringChar_,index_,subindex_,size_,
                     static_cast<unsigned long long>(value));
  verificationRetries_=0;
  ecrt_sdo_request_read(readreq_);
  queuePhase_=QUEUE_VERIFY;
  return 0;
}

int ecmcEcAsyncSDO::beginQueuedTransfer(bool write, uint64_t value) {
  if (!queue_ || !queue_->running()) return ERROR_EC_SDO_STARTUP_PENDING;
  if (write && preopOnly_) return ERROR_EC_SDO_PREOP_ONLY;
  if (ecrt_sdo_request_state(sdoreq_)==EC_REQUEST_BUSY) return ERROR_EC_SDO_ASYNC_BUSY;
  if (write) {
    sdoEncodeScalar(value,size_,expectedWire_);
    memcpy(ecrt_sdo_request_data(sdoreq_),expectedWire_,size_);
    ecmcRtLoggerLogInfo("SDO queued write starting: ec%d.s%d %s 0x%04x:%02x, %zu bytes, value=0x%llx.\n",
                       masterId_,slaveId_,idStringChar_,index_,subindex_,size_,
                       static_cast<unsigned long long>(value));
    ecrt_sdo_request_write(sdoreq_); queuePhase_=QUEUE_WRITE;
  } else {
    ecrt_sdo_request_read(readreq_); queuePhase_=QUEUE_READ;
  }
  return 0;
}

int ecmcEcAsyncSDO::pollQueuedTransfer(bool *done) {
  *done=false;
  // Writes use the download request. Reads and write verification use the
  // separate upload request and must poll that request's state.
  ec_sdo_request_t *request = queuePhase_==QUEUE_WRITE ? sdoreq_ : readreq_;
  const ec_request_state_t state=ecrt_sdo_request_state(request);
  if (state==EC_REQUEST_BUSY || state==EC_REQUEST_UNUSED) return 0;
  if (state==EC_REQUEST_ERROR) {
    static const unsigned int MAX_VERIFICATION_RETRIES=5;
    if (queuePhase_==QUEUE_VERIFY && verificationRetries_<MAX_VERIFICATION_RETRIES) {
      ++verificationRetries_;
      ecmcRtLoggerLogInfo(
        "SDO verification upload retry %u/%u: ec%d.s%d %s 0x%04x:%02x, %zu bytes.\n",
        verificationRetries_,MAX_VERIFICATION_RETRIES,masterId_,slaveId_,
        idStringChar_,index_,subindex_,size_);
      ecrt_sdo_request_read(readreq_);
      return 0;
    }
    ecmcRtLoggerLogError(
      "SDO queued transfer failed: ec%d.s%d %s 0x%04x:%02x, phase=%d, %zu bytes (0x%x).\n",
      masterId_,slaveId_,idStringChar_,index_,subindex_,
      static_cast<int>(queuePhase_),size_,ERROR_EC_SDO_ASYNC_ERROR);
    *done=true;
    return ERROR_EC_SDO_ASYNC_ERROR;
  }
  if (queuePhase_==QUEUE_WRITE) {
    ecmcRtLoggerLogInfo("SDO queued write verification starting: ec%d.s%d %s 0x%04x:%02x, %zu bytes.\n",
                       masterId_,slaveId_,idStringChar_,index_,subindex_,size_);
    verificationRetries_=0;
    ecrt_sdo_request_read(readreq_); queuePhase_=QUEUE_VERIFY; return 0;
  }
  *done=true;
  if (ecrt_sdo_request_data_size(readreq_)!=size_) return ERROR_EC_SDO_ASYNC_SIZE;
  const uint8_t *wire=ecrt_sdo_request_data(readreq_);
  publishQueuedRead(sdoDecodeScalar(wire,size_));
  return queuePhase_==QUEUE_VERIFY && memcmp(wire,expectedWire_,size_) ? ERROR_EC_SDO_VERIFY : 0;
}

int ecmcEcAsyncSDO::transferBeforeActivation(ec_master_t *master, bool write,
    uint64_t desired, uint64_t *actual, bool *actualValid) const {
  *actualValid=false;
  if (!queue_ || queue_->running()) return ERROR_EC_SDO_PREOP_ONLY;
  ec_slave_info_t slave;
  if (ecrt_master_get_slave(master,slaveId_,&slave) || slave.al_state!=EC_AL_STATE_PREOP)
    return ERROR_EC_SDO_PREOP_ONLY;
  uint8_t expected[8]={}, received[8]={};
  uint32_t abort=0; size_t receivedSize=0;
  sdoEncodeScalar(desired,size_,expected);
  if (write)
    printf("SDO PREOP write starting: ec%d.s%d %s 0x%04x:%02x, %zu bytes, value=0x%llx.\n",
           masterId_,slaveId_,idStringChar_,index_,subindex_,size_,
           static_cast<unsigned long long>(desired));
  if (write && (ecrt_master_sdo_download(master,slaveId_,index_,subindex_,expected,size_,&abort) || abort))
    return ERROR_EC_SDO_ASYNC_ERROR;
  if (ecrt_master_get_slave(master,slaveId_,&slave) || slave.al_state!=EC_AL_STATE_PREOP)
    return ERROR_EC_SDO_PREOP_ONLY;
  if (write)
    printf("SDO PREOP write verification starting: ec%d.s%d %s 0x%04x:%02x, %zu bytes.\n",
           masterId_,slaveId_,idStringChar_,index_,subindex_,size_);
  if (ecrt_master_sdo_upload(master,slaveId_,index_,subindex_,received,size_,&receivedSize,&abort) || abort)
    return ERROR_EC_SDO_ASYNC_ERROR;
  if (receivedSize!=size_) return ERROR_EC_SDO_ASYNC_SIZE;
  *actual=sdoDecodeScalar(received,size_); *actualValid=true;
  return write && memcmp(expected,received,size_) ? ERROR_EC_SDO_VERIFY : 0;
}

namespace {
uint32_t sdoStartupChecksum(const uint8_t *data) {
  uint32_t checksum=2166136261u;
  for (size_t i=0;i<28;++i) checksum=(checksum^data[i])*16777619u;
  return checksum;
}
}

int ecmcEcAsyncSDO::enableStartupData() {
  // Startup snapshots are also used by ordinary SDOs that allow runtime writes.
  if (!queue_ || queue_->running()) return ERROR_EC_SDO_ASYNC_VALUE;
  if (asynParamStartupData_) return 0;
  const std::string name="ec"+std::to_string(masterId_)+".s"+
    std::to_string(slaveId_)+".sdo."+idString_+".startup";
  asynParamStartupData_=asynPortDriver_->addNewAvailParam(name.c_str(),asynParamInt8Array,
    startupData_,sizeof(startupData_),ECMC_EC_U8,0);
  if (!asynParamStartupData_) return ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL;
  asynParamStartupData_->setAllowWriteToEcmc(false);
  if (hasStartupWrite_) rememberStartupWrite(lastDesired_);
  else asynParamStartupData_->refreshParam(1);
  return 0;
}

void ecmcEcAsyncSDO::rememberStartupWrite(uint64_t value) {
  hasStartupWrite_=true; lastDesired_=value;
  if (!asynParamStartupData_) return;
  memset(startupData_,0,sizeof(startupData_));
  memcpy(startupData_,"ESDO",4);
  startupData_[4]=1; startupData_[5]=static_cast<uint8_t>(size_);
  startupData_[6]=static_cast<uint8_t>(dt_);
  EC_WRITE_U32(startupData_+8,static_cast<uint32_t>(masterId_));
  EC_WRITE_U16(startupData_+12,static_cast<uint16_t>(slaveId_));
  EC_WRITE_U16(startupData_+14,index_); startupData_[16]=subindex_;
  sdoEncodeScalar(value,size_,startupData_+20);
  EC_WRITE_U32(startupData_+28,sdoStartupChecksum(startupData_));
  asynParamStartupData_->refreshParam(1);
}

int ecmcEcAsyncSDO::restoreStartupData(const uint8_t *data, size_t bytes) {
  if (!queue_ || !asynParamStartupData_ || !data) return ERROR_EC_SDO_ASYNC_VALUE;
  // An actual request in this boot is authoritative over an old saved request.
  if (hasStartupWrite_) return 0;
  bool empty=true;
  for (size_t i=0;i<bytes;++i) empty &= data[i]==0;
  if (!bytes || empty) return 0; // Untouched object: no request and no default-zero write.
  if (bytes!=sizeof(startupData_) || memcmp(data,"ESDO",4) || data[4]!=1 ||
      data[5]!=size_ || data[6]!=static_cast<uint8_t>(dt_) ||
      EC_READ_U32(data+8)!=static_cast<uint32_t>(masterId_) ||
      EC_READ_U16(data+12)!=static_cast<uint16_t>(slaveId_) ||
      EC_READ_U16(data+14)!=index_ || data[16]!=subindex_ ||
      EC_READ_U32(data+28)!=sdoStartupChecksum(data))
    return reportQueueError(ERROR_EC_SDO_CONFIG_CONFLICT);
  const uint64_t restored=sdoDecodeScalar(data+20,size_);
  if ((dt_==ECMC_EC_F32 && !std::isfinite(sdoNativeScalar<float>(restored))) ||
      (dt_==ECMC_EC_F64 && !std::isfinite(sdoNativeScalar<double>(restored))))
    return reportQueueError(ERROR_EC_SDO_ASYNC_VALUE);
  buffer_=restored;
  asynParamValue_->refreshParam(1);
  return requestWrite();
}
