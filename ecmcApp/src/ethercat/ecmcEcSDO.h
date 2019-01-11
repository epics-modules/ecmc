/*
 * ecmcEcSDO.h
 *
 *  Created on: Dec 15, 2015
 *      Author: anderssandstrom
 */

#ifndef ECMCECSDO_H_
#define ECMCECSDO_H_
#include <string.h>
#include "stdio.h"
#include "ecrt.h"
#include "../main/ecmcError.h"

// ECSDO
#define ERROR_EC_SDO_SIZE_TO_LARGE 0x23000
#define ERROR_EC_SDO_WRITE_FAILED 0x23001
#define ERROR_EC_SDO_READ_FAILED 0x23002
#define ERROR_EC_SDO_VERIFY_FAILED 0x23003

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
  static int writeComplete(ec_master_t *master,
                           uint16_t     slavePosition,
                           uint16_t     sdoIndex,
                           uint32_t     value,
                           size_t       byteSize);
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
  static int addSdoConfig(ec_slave_config_t *slave,
                          uint16_t           slavePosition,
                          uint16_t           sdoIndex,
                          uint8_t            sdoSubIndex,
                          uint32_t           value,
                          size_t             byteSize);
};
#endif  /* ECMCECSDO_H_ */
