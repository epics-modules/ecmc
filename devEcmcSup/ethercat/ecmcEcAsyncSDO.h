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
#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "ecmcAsynPortDriver.h"
#include "ecmcErrorsList.h"

#define ERROR_EC_SDO_ASYNC_BUSY 0x23500
#define ERROR_EC_SDO_ASYNC_ERROR 0x23501
#define ERROR_EC_SDO_ASYNC_OBJ_NULL 0x23502
#define ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL 0x23503

#define DEFAULT_SDO_ASYNC_TIMOUT_MS 2000

class ecmcEcAsyncSDO : public ecmcError {
public:
  ecmcEcAsyncSDO(ecmcAsynPortDriver *asynDriver,
                 int                 masterId,
                 int                 slaveId,
                 ec_slave_config_t  *sc, /**< Slave configuration. */
                 uint16_t            sdoIndex, /**< SDO index. */
                 uint8_t             sdoSubIndex, /**< SDO subindex. */
                 ecmcEcDataType      dt,
                 std::string         alias);
  ~ecmcEcAsyncSDO();

  asynStatus asynWriteSDO(void         *data,
                          size_t        bytes,
                          asynParamType asynParType);
  asynStatus asynReadSDO(void         *data,
                         size_t        bytes,
                         asynParamType asynParType);

  int execute();

private:
  int initAsyn();
  int writeValue();
  int readValue();

  ec_sdo_request_t *sdoreq_;
  ec_slave_config_t *sc_;  /**< Slave configuration. */
  uint16_t index_;            /**< SDO index. */
  uint8_t subindex_;             /**< SDO subindex. */
  ecmcAsynPortDriver *asynPortDriver_;
  ecmcAsynDataItem *asynParamWrite_;
  ecmcAsynDataItem *asynParamRead_;
  ecmcAsynDataItem *asynParamValue_;
  ecmcAsynDataItem *asynParamError_;
  ecmcAsynDataItem *asynParamBusy_;
  ecmcEcDataType dt_;
  size_t size_;
  int masterId_;
  int slaveId_;
  uint64_t buffer_;
  int usedSizeBytes_;
  int bitLength_;
  int sdoError_;
  int dummyReadCmd_;
  int dummyWriteCmd_;
  int busy_;
  int8_t *int8Ptr_;
  uint8_t *uint8Ptr_;
  int16_t *int16Ptr_;
  uint16_t *uint16Ptr_;
  int32_t *int32Ptr_;
  uint32_t *uint32Ptr_;
  int64_t *int64Ptr_;
  uint64_t *uint64Ptr_;
  float *float32Ptr_;
  double *float64Ptr_;
  int writeCmdInProcess_;
  int readCmdInProcess_;
  std::string idString_;
  char *idStringChar_;
  ec_request_state_t stateOld_;
  int readTrigg_;
  int writeTrigg_;
};
#endif  /* ECMCECASYNCSDO_H_ */
