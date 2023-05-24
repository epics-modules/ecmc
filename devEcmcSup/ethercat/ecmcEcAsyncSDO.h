/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEcAsyncSDO.h
*
*  Created on: Dec 15, 2015
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCECASYNCSDO_H_
#define ECMCECASYNCSDO_H_
#include <string.h>
#include "stdio.h"
#include "ecrt.h"
#include "../main/ecmcError.h"
#include "../com/ecmcAsynPortDriver.h"

#define ERROR_EC_SDO_ASYNC_BUSY 0x23500
#define ERROR_EC_SDO_ASYNC_ERROR 0x23501
#define ERROR_EC_SDO_ASYNC_OBJ_NULL 0x23502


#define DEFAULT_SDO_ASYNC_TIMOUT_MS 2000

class ecmcEcAsyncSDO : public ecmcError {
 public:
  ecmcEcAsyncSDO(int objIndex,
                 int masterId,
                 int slaveId,
                 ecmcAsynPortDriver *asynPortDriver,
                 ec_slave_config_t *sc, /**< Slave configuration. */
                 uint16_t sdoIndex, /**< SDO index. */
                 uint8_t sdoSubIndex, /**< SDO subindex. */
                 size_t size, /**< Data size to reserve. */
                 ecmcEcDataType dt,
                 std::string id);
  ~ecmcEcAsyncSDO();

  asynStatus asynWriteSDO(void* data,
                          size_t bytes,
                          asynParamType asynParType);
  asynStatus asynReadSDO(void* data,
                          size_t bytes,
                          asynParamType asynParType);

  int ecmcEcAsyncSDO::execute();

private:
  int                 initAsyn();
  int                 writeValue();
  int                 readValue();

  ec_sdo_request_t   *sdoreq_;
  ec_slave_config_t  *sc_; /**< Slave configuration. */
  uint16_t            index_; /**< SDO index. */
  uint8_t             subindex_; /**< SDO subindex. */
  size_t              size_; /**< Data size to reserve. */
  asynPortDriver      asynPortDriver_;
  ecmcAsynDataItem   *asynParamWrite_;
  ecmcAsynDataItem   *asynParamRead_;
  ecmcAsynDataItem   *asynParamValue_;
  ecmcAsynDataItem   *asynParamError_;
  ecmcAsynDataItem   *asynParamBusy_;
  int                 objIndex_;
  std::string         idString_;
  char               *idStringChar_;
  uint64_t            buffer_;
  int                 usedSizeBytes_;
  int                 bitLength_;
  int                 sdoError_;
  int                 dummyReadCmd_;
  int                 dummyWriteCmd_;
  int                 busy_;
};
#endif  /* ECMCECASYNCSDO_H_ */
