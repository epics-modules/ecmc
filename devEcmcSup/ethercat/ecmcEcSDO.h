/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEcSDO.h
*
*  Created on: Dec 15, 2015
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCECSDO_H_
#define ECMCECSDO_H_
#include <string.h>
#include "stdio.h"
#include "ecrt.h"
#include "ecmcError.h"
#include "ecmcDefinitions.h"

// ECSDO
#define ERROR_EC_SDO_SIZE_TO_LARGE 0x23000
#define ERROR_EC_SDO_WRITE_FAILED 0x23001
#define ERROR_EC_SDO_READ_FAILED 0x23002
#define ERROR_EC_SDO_VERIFY_FAILED 0x23003
#define ERROR_EC_SDO_DATA_SIZE_ERROR 0x23004
#define ERROR_EC_SDO_BUFFER_ALLOC_FAIL 0x23005
#define ERROR_EC_SDO_DATATYPE_ERROR 0x23006
#define ERROR_EC_SDO_VALUE_CONV_ERROR 0x23007

class ecmcEcSDO : public ecmcError {
 public:
  ecmcEcSDO();
  ~ecmcEcSDO();
  static int write(ec_master_t *master,
                   uint16_t     slavePosition,
                   uint16_t     sdoIndex,
                   uint8_t      sdoSubIndex,
                   uint32_t     value,
                   size_t       byteSize);
  static int addWriteComplete(ec_slave_config_t *sc,
                           uint16_t           sdoIndex,
                           const char*        dataString,
                           size_t             byteSize);
  static int read(ec_master_t *master,
                  uint16_t     slavePosition,
                  uint16_t     sdoIndex,
                  uint8_t      sdoSubIndex,
                  uint32_t    *readValue,
                  size_t      *readBytes);
  static int writeAndVerify(ec_master_t *master,
                            uint16_t     slavePosition,
                            uint16_t     sdoIndex,
                            uint8_t      sdoSubIndex,
                            uint32_t     value,
                            size_t       byteSize);

  // up to 4 bytes
  static int addSdoConfig(ec_slave_config_t *slave,
                          uint16_t           slavePosition,
                          uint16_t           sdoIndex,
                          uint8_t            sdoSubIndex,
                          uint32_t           value,
                          size_t             byteSize);

  // up to 8 bytes and signed
  static int addSdoConfigDT(ec_slave_config_t *slave,
                            uint16_t           slavePosition,
                            uint16_t           sdoIndex,
                            uint8_t            sdoSubIndex,
                            const char*        value,                            
                            ecmcEcDataType     dt);

  static int addSdoConfigBuffer(ec_slave_config_t *sc,
                                uint16_t           sdoIndex,
                                uint8_t            sdoSubIndex,
                                const char*        dataString,
                                size_t             byteSize);                          
};
#endif  /* ECMCECSDO_H_ */
