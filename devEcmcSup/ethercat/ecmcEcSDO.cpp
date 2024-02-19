/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcEcSDO.cpp
*
*  Created on: Dec 15, 2015
*      Author: anderssandstrom
*
\*************************************************************************/

#include <vector>
#include <exception>
#include "ecmcEcSDO.h"
#include "ecmcAsynPortDriverUtils.h"
ecmcEcSDO::ecmcEcSDO() {}

ecmcEcSDO::~ecmcEcSDO() {}

int ecmcEcSDO::write(ec_master_t *master,
                     uint16_t     slavePosition,
                     uint16_t     sdoIndex,
                     uint8_t      sdoSubIndex,
                     uint32_t     value,
                     size_t       byteSize) {
  uint32_t abortCode = 0;

  if (byteSize > 4) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Write failed, byte size to large (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      sdoSubIndex,
      slavePosition,
      ERROR_EC_SDO_SIZE_TO_LARGE);
    return ERROR_EC_SDO_SIZE_TO_LARGE;
  }
  uint8_t buffer[4];
  memset(&buffer[0], 0, 4);
  memcpy(&buffer[0], &value, 4);  // TODO ENDIANS

  int errorCode = ecrt_master_sdo_download(master,
                                           slavePosition,
                                           sdoIndex,
                                           sdoSubIndex,
                                           &buffer[0],
                                           byteSize,
                                           &abortCode);

  if (errorCode || abortCode) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Write failed with sdo error code %d, abort code 0x%x (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      sdoSubIndex,
      slavePosition,
      errorCode,
      abortCode,
      ERROR_EC_SDO_WRITE_FAILED);
  }
  return errorCode;
}

int ecmcEcSDO::addWriteComplete(ec_slave_config_t *sc,
                                uint16_t           sdoIndex,
                                const char        *dataString,
                                size_t             byteSize) {
  uint32_t abortCode = 0;
  std::vector<uint8_t> buffer;

  // Convert dataString to binary:
  char *dataPtr     = (char *)dataString;
  unsigned int data = 0;
  int nvals         = 0;
  size_t addedBytes = 0;

  while (dataPtr) {
    if (strlen(dataPtr) == 0) {
      break;
    }
    nvals = sscanf(dataPtr, "%x", &data);

    if (nvals == 1) {
      try {
        buffer.push_back((uint8_t)data);
      }
      catch (...) {
        LOGERR(
          "%s/%s:%d: ERROR: SDO data size error at sdo index 0x%x (0x%x).\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          sdoIndex,
          ERROR_EC_SDO_BUFFER_ALLOC_FAIL);
        return ERROR_EC_SDO_BUFFER_ALLOC_FAIL;
      }
      dataPtr = strchr(++dataPtr, ' ');
      addedBytes++;
    } else {
      dataPtr = NULL;
    }
  }

  if ((addedBytes == 0) || (addedBytes != byteSize)) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO data size error at sdo index 0x%x (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      ERROR_EC_SDO_DATA_SIZE_ERROR);

    return ERROR_EC_SDO_DATA_SIZE_ERROR;
  }

  int errorCode = ecrt_slave_config_complete_sdo(
    sc,     /**< Slave configuration. */
    sdoIndex,     /**< Index of the SDO to configure. */
    &buffer[0],     /**< Pointer to the data. */
    byteSize     /**< Size of the \a data. */
    );

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x: Write failed with sdo error code %d, abort code 0x%x (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      errorCode,
      abortCode,
      ERROR_EC_SDO_WRITE_FAILED);
    return errorCode;
  }

  return 0;
}

int ecmcEcSDO::addSdoConfigBuffer(ec_slave_config_t *sc,
                                  uint16_t           sdoIndex,
                                  uint8_t            sdoSubIndex,
                                  const char        *dataString,
                                  size_t             byteSize) {
  uint32_t abortCode = 0;
  std::vector<uint8_t> buffer;

  // Convert dataString to binary:
  char *dataPtr     = (char *)dataString;
  unsigned int data = 0;
  int nvals         = 0;
  size_t addedBytes = 0;

  while (dataPtr) {
    if (strlen(dataPtr) == 0) {
      break;
    }
    nvals = sscanf(dataPtr, "%x", &data);

    if (nvals == 1) {
      try {
        buffer.push_back((uint8_t)data);
      }
      catch (...) {
        LOGERR(
          "%s/%s:%d: ERROR: SDO data size error at sdo index 0x%x (0x%x).\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          sdoIndex,
          ERROR_EC_SDO_BUFFER_ALLOC_FAIL);
        return ERROR_EC_SDO_BUFFER_ALLOC_FAIL;
      }
      dataPtr = strchr(++dataPtr, ' ');
      addedBytes++;
    } else {
      dataPtr = NULL;
    }
  }

  if ((addedBytes == 0) || (addedBytes != byteSize)) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO data size error at sdo index 0x%x (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      ERROR_EC_SDO_DATA_SIZE_ERROR);

    return ERROR_EC_SDO_DATA_SIZE_ERROR;
  }

  int errorCode = ecrt_slave_config_sdo(
    sc,     /**< Slave configuration. */
    sdoIndex,     /**< Index of the SDO to configure. */
    sdoSubIndex,
    &buffer[0],     /**< Pointer to the data. */
    byteSize     /**< Size of the \a data. */
    );

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x: Write failed with sdo error code %d, abort code 0x%x (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      errorCode,
      abortCode,
      ERROR_EC_SDO_WRITE_FAILED);
    return errorCode;
  }

  return 0;
}

int ecmcEcSDO::addSdoConfigDT(ec_slave_config_t *slave,
                              uint16_t           slavePosition,
                              uint16_t           sdoIndex,
                              uint8_t            sdoSubIndex,
                              const char        *valueString,
                              ecmcEcDataType     dt) {
  size_t byteSize = getEcDataTypeByteSize(dt);

  if ((byteSize > 8) || (byteSize == 0)) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Write failed, byte size to large (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      sdoSubIndex,
      slavePosition,
      ERROR_EC_SDO_SIZE_TO_LARGE);
    return ERROR_EC_SDO_SIZE_TO_LARGE;
  }


  // Minimum 1 byte (sdos with smaller size should be written as 1 byte uint)
  if ((dt == ECMC_EC_NONE) ||
      (dt == ECMC_EC_B1) ||
      (dt == ECMC_EC_B2) ||
      (dt == ECMC_EC_B3) ||
      (dt == ECMC_EC_B4)) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Write failed, data type invalid (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      sdoSubIndex,
      slavePosition,
      ERROR_EC_SDO_DATATYPE_ERROR);
    return ERROR_EC_SDO_DATATYPE_ERROR;
  }

  // check that datatype and size matches
  // Convert string top value
  uint64_t inbuffer    = 0LL;    // Clear inbuffer
  uint8_t *inbufferPtr = (uint8_t *)&inbuffer;

  int8_t   *int8Ptr    = (int8_t *)&inbuffer;
  uint8_t  *uint8Ptr   = (uint8_t *)&inbuffer;
  int16_t  *int16Ptr   = (int16_t *)&inbuffer;
  uint16_t *uint16Ptr  = (uint16_t *)&inbuffer;
  int32_t  *int32Ptr   = (int32_t *)&inbuffer;
  uint32_t *uint32Ptr  = (uint32_t *)&inbuffer;
  int64_t  *int64Ptr   = (int64_t *)&inbuffer;
  uint64_t *uint64Ptr  = (uint64_t *)&inbuffer;
  float    *float32Ptr = (float *)&inbuffer;
  double   *float64Ptr = (double *)&inbuffer;

  bool convSuccess = false;
  int  nvals       = 0;

  switch (dt) {
  case ECMC_EC_U8:
    nvals = sscanf(valueString,
                   "%" SCNu8 "",
                   uint8Ptr);

    if (nvals == 1) {
      convSuccess = true;
    }

    break;

  case ECMC_EC_S8:
    nvals = sscanf(valueString,
                   "%" SCNd8 "",
                   int8Ptr);

    if (nvals == 1) {
      convSuccess = true;
    }

    break;

  case ECMC_EC_U16:
    nvals = sscanf(valueString,
                   "%" SCNu16 "",
                   uint16Ptr);

    if (nvals == 1) {
      convSuccess = true;
    }

    break;

  case ECMC_EC_S16:
    nvals = sscanf(valueString,
                   "%" SCNd16 "",
                   int16Ptr);

    if (nvals == 1) {
      convSuccess = true;
    }

    break;

  case ECMC_EC_U32:
    nvals = sscanf(valueString,
                   "%" SCNu32 "",
                   uint32Ptr);

    if (nvals == 1) {
      convSuccess = true;
    }

    break;

  case ECMC_EC_S32:
    nvals = sscanf(valueString,
                   "%" SCNd32 "",
                   int32Ptr);

    if (nvals == 1) {
      convSuccess = true;
    }

    break;

#ifdef EC_WRITE_U64
  case ECMC_EC_U64:
    nvals = sscanf(valueString,
                   "%" SCNu64 "",
                   uint64Ptr);

    if (nvals == 1) {
      convSuccess = true;
    }
    break;
#endif // ifdef EC_WRITE_U64

#ifdef EC_WRITE_S64
  case ECMC_EC_S64:
    nvals = sscanf(valueString,
                   "%" SCNd64 "",
                   int64Ptr);

    if (nvals == 1) {
      convSuccess = true;
    }
    break;
#endif // ifdef EC_WRITE_S64

#ifdef EC_WRITE_REAL
  case ECMC_EC_F32:
    nvals = sscanf(valueString,
                   "%f",
                   float32Ptr);

    if (nvals == 1) {
      convSuccess = true;
    }
    break;
#endif // ifdef EC_WRITE_REAL

#ifdef EC_WRITE_LREAL
  case ECMC_EC_F64:
    nvals = sscanf(valueString,
                   "%lf",
                   float64Ptr);

    if (nvals == 1) {
      convSuccess = true;
    }
    break;

#endif // ifdef EC_WRITE_LREAL
  default:
    convSuccess = false;
    break;
  }

  // Did the conversion from string to value succeed?
  // data should noe be in inbuffer
  if (!convSuccess) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Write value conversion failed (value = %s), 0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      sdoSubIndex,
      slavePosition,
      valueString,
      ERROR_EC_SDO_VALUE_CONV_ERROR);
    return ERROR_EC_SDO_VALUE_CONV_ERROR;
  }

  // Need to use the EC_WRITE* macros to keep track of endians when
  // using the generic ecrt_slave_config_sdo func

  // uint64_t  outbuffer    = 0;  // Clear outbuffer
  // uint8_t*  outbufferPtr =  (uint8_t*)&outbuffer;
  //
  // switch(dt) {
  //  case ECMC_EC_U8:
  //    EC_WRITE_U8(outbufferPtr, *uint8Ptr);
  //    break;
  //
  //  case ECMC_EC_S8:
  //    EC_WRITE_S8(outbufferPtr, *int8Ptr);
  //    break;
  //
  //  case ECMC_EC_U16:
  //    EC_WRITE_U16(outbufferPtr, *uint16Ptr);
  //    break;
  //
  //  case ECMC_EC_S16:
  //    EC_WRITE_S16(outbufferPtr, *int16Ptr);
  //    break;
  //
  //  case ECMC_EC_U32:
  //    EC_WRITE_U32(outbufferPtr, *uint32Ptr);
  //    break;
  //
  //  case ECMC_EC_S32:
  //    EC_WRITE_S32(outbufferPtr, *int32Ptr);
  //    break;
  //
  // #ifdef EC_WRITE_U64
  //  case ECMC_EC_U64:
  //    EC_WRITE_U64(outbufferPtr, *uint64Ptr);
  //    break;
  // #endif
  //
  // #ifdef EC_WRITE_S64
  //  case ECMC_EC_S64:
  //    EC_WRITE_S64(outbufferPtr, *int64Ptr);
  //    break;
  // #endif
  //
  // #ifdef EC_WRITE_REAL
  //  case ECMC_EC_F32:
  //    EC_WRITE_REAL(outbufferPtr, *float32Ptr_);
  //    break;
  // #endif
  //
  // #ifdef EC_WRITE_LREAL
  //  case ECMC_EC_F64:
  //    EC_WRITE_LREAL(outbufferPtr, *float64Ptr_);
  //    break;
  // #endif
  //  default:
  //  LOGERR(
  //    "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Write failed, data type invalid (0x%x).\n",
  //    __FILE__,
  //    __FUNCTION__,
  //    __LINE__,
  //    sdoIndex,
  //    sdoSubIndex,
  //    slavePosition,
  //    ERROR_EC_SDO_DATATYPE_ERROR);
  //  return ERROR_EC_SDO_DATATYPE_ERROR;
  // }

  int errorCode = ecrt_slave_config_sdo(slave,
                                        sdoIndex,
                                        sdoSubIndex,
                                        inbufferPtr,
                                        byteSize);

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Failed with sdo error code %d (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      sdoSubIndex,
      slavePosition,
      errorCode,
      ERROR_EC_SDO_WRITE_FAILED);
    return ERROR_EC_SDO_WRITE_FAILED;
  }

  return 0;
}

int ecmcEcSDO::addSdoConfig(ec_slave_config_t *slave,
                            uint16_t           slavePosition,
                            uint16_t           sdoIndex,
                            uint8_t            sdoSubIndex,
                            uint32_t           value,
                            size_t             byteSize) {
  if (byteSize > 4) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Write failed, byte size to large (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      sdoSubIndex,
      slavePosition,
      ERROR_EC_SDO_SIZE_TO_LARGE);
    return ERROR_EC_SDO_SIZE_TO_LARGE;
  }

  int errorCode = 0;

  switch (byteSize) {
  case 1:
    uint8_t *val8;
    val8      = reinterpret_cast<uint8_t *>(&value);
    errorCode = ecrt_slave_config_sdo8(slave, sdoIndex, sdoSubIndex, *val8);
    break;

  case 2:
    uint16_t *val16;
    val16     = reinterpret_cast<uint16_t *>(&value);
    errorCode = ecrt_slave_config_sdo16(slave, sdoIndex, sdoSubIndex, *val16);
    break;

  case 4:
    errorCode = ecrt_slave_config_sdo32(slave, sdoIndex, sdoSubIndex, value);
    break;
  }

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Write failed with sdo error code %d (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      sdoSubIndex,
      slavePosition,
      errorCode,
      ERROR_EC_SDO_WRITE_FAILED);
    return ERROR_EC_SDO_WRITE_FAILED;
  }
  return 0;
}

int ecmcEcSDO::read(ec_master_t *master,
                    uint16_t     slavePosition,
                    uint16_t     sdoIndex,
                    uint8_t      sdoSubIndex,
                    uint32_t    *readValue,
                    size_t      *readBytes) {
  uint32_t abortCode = 0;
  uint8_t  buffer[4];

  memset(buffer, 0, sizeof(buffer));
  int errorCode = ecrt_master_sdo_upload(master,
                                         slavePosition,
                                         sdoIndex,
                                         sdoSubIndex,
                                         buffer,
                                         4,
                                         readBytes,
                                         &abortCode);

  if (errorCode || abortCode) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Read failed with sdo error code %d, abort code 0x%x (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      sdoSubIndex,
      slavePosition,
      errorCode,
      abortCode,
      ERROR_EC_SDO_WRITE_FAILED);
  }

  memcpy(readValue, buffer, 4);
  return 0;
}

int ecmcEcSDO::writeAndVerify(ec_master_t *master,
                              uint16_t     slavePosition,
                              uint16_t     sdoIndex,
                              uint8_t      sdoSubIndex,
                              uint32_t     value,
                              size_t       byteSize) {
  if (byteSize > 4) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Write failed, byte size to large (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      sdoSubIndex,
      slavePosition,
      ERROR_EC_SDO_SIZE_TO_LARGE);
    return ERROR_EC_SDO_SIZE_TO_LARGE;
  }

  if (write(master, slavePosition, sdoIndex, sdoSubIndex, value, byteSize)) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Write failed (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      sdoSubIndex,
      slavePosition,
      ERROR_EC_SDO_WRITE_FAILED);
    return ERROR_EC_SDO_WRITE_FAILED;
  }
  uint32_t readValue = 0;
  size_t   readBytes = 0;

  if (read(master, slavePosition, sdoIndex, sdoSubIndex, &readValue,
           &readBytes)) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Read failed (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      sdoSubIndex,
      slavePosition,
      ERROR_EC_SDO_READ_FAILED);
    return ERROR_EC_SDO_READ_FAILED;
  }

  if (value != readValue) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x:%x at slave position %d: Verification failed (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      sdoSubIndex,
      slavePosition,
      ERROR_EC_SDO_VERIFY_FAILED);
    return ERROR_EC_SDO_VERIFY_FAILED;
  }
  return 0;
}
