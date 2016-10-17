/*
 * ecmcEcSDO.h
 *
 *  Created on: Dec 15, 2015
 *      Author: anderssandstrom
 */

#ifndef ECMCECSDO_H_
#define ECMCECSDO_H_
#include "ecrt.h"
#include <string.h>
#include "stdio.h"
#include "ecmcError.h"

//ECSDO
#define ERROR_EC_SDO_SIZE_TO_LARGE 0x23000
#define ERROR_EC_SDO_WRITE_FAILED 0x23001
#define ERROR_EC_SDO_READ_FAILED 0x23002
#define ERROR_EC_SDO_VERIFY_FAILED 0x23003

class ecmcEcSDO : public ecmcError
{
public:
  ecmcEcSDO(ec_master_t *master,uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex, uint32_t value, int byteSize);
  ecmcEcSDO(ec_master_t *master,uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex, int byteSize);
  ~ecmcEcSDO();
  int write(uint32_t value);
  int write();
  int read();
  int writeAndVerify();
  uint32_t getReadValue();
  uint32_t getWriteValue();
  int setWriteValue(uint32_t value);
  uint16_t getSlaveBusPosition();
  uint16_t getSdoIndex();
  uint8_t getSdoSubIndex();

private:
  void initVars();
  ec_master_t *master_;
  uint16_t slavePosition_;
  uint16_t sdoIndex_;
  uint8_t sdoSubIndex_;
  uint32_t writeValue_;
  uint32_t readValue_;
  uint8_t writeBuffer_[4];
  uint8_t readBuffer_[4];
  int byteSize_;
};
#endif /* ECMCECSDO_H_ */
