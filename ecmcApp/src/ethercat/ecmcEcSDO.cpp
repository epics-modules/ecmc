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

#include "ecmcEcSDO.h"

ecmcEcSDO::ecmcEcSDO()
{}

ecmcEcSDO::~ecmcEcSDO()
{}

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

int ecmcEcSDO::writeComplete(ec_master_t *master,
                             uint16_t     slavePosition,
                             uint16_t     sdoIndex,
                             uint32_t     value,
                             size_t       byteSize) {
  uint32_t abortCode = 0;

  if (byteSize > 4) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x at slave position %d: Write failed, byte size to large (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      slavePosition,
      ERROR_EC_SDO_SIZE_TO_LARGE);
    return ERROR_EC_SDO_SIZE_TO_LARGE;
  }
  uint8_t buffer[4];
  memset(&buffer[0], 0, 4);
  memcpy(&buffer[0], &value, 4);  // TODO ENDIANS

  int errorCode = ecrt_master_sdo_download_complete(master,
                                                    slavePosition,
                                                    sdoIndex,
                                                    &buffer[0],
                                                    byteSize,
                                                    &abortCode);

  if (errorCode || abortCode) {
    LOGERR(
      "%s/%s:%d: ERROR: SDO object 0x%x at slave position %d: Write failed with sdo error code %d, abort code 0x%x (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      sdoIndex,
      slavePosition,
      errorCode,
      abortCode,
      ERROR_EC_SDO_WRITE_FAILED);
  }
  return errorCode;
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
