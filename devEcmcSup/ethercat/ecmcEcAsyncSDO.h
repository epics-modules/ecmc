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
#include <vector>
#include "stdio.h"
#include "ecrt.h"
#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "ecmcAsynPortDriver.h"
#include "ecmcErrorsList.h"

#include "ecmcSdoErrors.h"
#include "ecmcSdoQueue.h"
#define ECMC_SDO_ASYNC_MAX_BYTES (1024 * 1024)

#define DEFAULT_SDO_ASYNC_TIMOUT_MS 2000

class ecmcEcAsyncSDO : public ecmcError, public ecmcSdoQueueTarget {
public:
  ecmcEcAsyncSDO(ecmcAsynPortDriver *asynDriver,
                 int                 masterId,
                 int                 slaveId,
                 ec_slave_config_t  *sc, /**< Slave configuration. */
                 uint16_t            sdoIndex, /**< SDO index. */
                 uint8_t             sdoSubIndex, /**< SDO subindex. */
                 ecmcEcDataType      dt,
                 std::string         alias,
                 size_t              arrayBytes = 0);
  ~ecmcEcAsyncSDO();

  asynStatus asynWriteSDO(void         *data,
                          size_t        bytes,
                          asynParamType asynParType);
  asynStatus asynReadSDO(void         *data,
                         size_t        bytes,
                         asynParamType asynParType);

  asynStatus asynWriteArray(void *data, size_t bytes, asynParamType type);
  // Call from the serialized PLC/C++ logic cycle or under the asyn port lock.
  int requestRead();
  int requestWrite();
  const std::string& getName() const { return idString_; }
  int getBusy() const { return busy_; }
  int getSdoError() const { return sdoError_; }
  int getSize() const { return arrayReadBuffer_.empty() ? 1 : arrayValidBytes_; }
  int getCapacity() const { return arrayReadBuffer_.empty() ? 1 : size_; }
  int getValue(int element, double *value) const;
  int setValue(int element, double value);
  int execute();

  // Opt-in scalar queue. Existing manual/asyn behavior is unchanged otherwise.
  int configureQueue(ecmcSdoQueue *queue, bool writeOnValue, bool preopOnly);
  bool queued() const { return queue_ != nullptr; }
  int getSlavePosition() const { return slaveId_; }
  uint16_t getIndex() const { return index_; }
  uint8_t getSubIndex() const { return subindex_; }
  asynStatus asynWriteScalar(void *data, size_t bytes, asynParamType type);
  int stageScalar(void *data, size_t bytes, asynParamType type);
  int configureStartupWrite(uint64_t value) override;
  int beginStartupVerification(uint64_t value) override;
  bool queuedTransferReady() const override;
  uint64_t startupAddress() const override {
    return (static_cast<uint64_t>(slaveId_)<<24) |
           (static_cast<uint64_t>(index_)<<8) | subindex_;
  }
  int beginQueuedTransfer(bool write, uint64_t value) override;
  int pollQueuedTransfer(bool *done) override;
  void queuedRequestAccepted() override;
  void queuedRequestStarted(bool write) override { activeQueuedWrite_=write; }
  bool startupWriteVerified() const { return !startupUnverified_; }
  bool hasStartupWrite() const { return hasStartupWrite_; }
  int enableStartupData();
  int restoreStartupData(const uint8_t *data, size_t bytes);
  void queuedRequestComplete(int error) override;
  int transferBeforeActivation(ec_master_t *master, bool write, uint64_t desired,
                                uint64_t *actual, bool *actualValid) const;
  void publishQueuedRead(uint64_t actual);
  int reportQueueError(int error);


private:
  ecmcSdoQueue *queue_ = nullptr;
  bool writeOnValue_ = false, preopOnly_ = false;
  int queuedPending_ = 0, pendingWrites_ = 0;
  bool activeQueuedWrite_ = false, startupUnverified_ = false;
  bool startupWriteFailed_ = false, hasStartupWrite_ = false;
  uint64_t lastDesired_ = 0;
  uint8_t startupData_[32] = {};
  ecmcAsynDataItem *asynParamStartupData_ = nullptr;
  void rememberStartupWrite(uint64_t value);
  enum QueuePhase { QUEUE_IDLE, QUEUE_WRITE, QUEUE_READ, QUEUE_VERIFY };
  QueuePhase queuePhase_ = QUEUE_IDLE;
  unsigned int verificationRetries_ = 0;
  uint8_t expectedWire_[8] = {};
  uint64_t readBuffer_ = 0;
  bool readValid_ = false;
  ecmcAsynDataItem *asynParamReadValue_ = nullptr;
  int initAsyn();
  int writeValue();
  int readValue();

  ec_sdo_request_t *sdoreq_;
  ec_sdo_request_t *readreq_;
  std::vector<uint8_t> arrayReadBuffer_;
  std::vector<uint8_t> arrayWriteBuffer_;
  bool arrayWriteReady_;
  std::vector<uint8_t> arrayWriteInitialized_;
  size_t arrayInitializedCount_;
  int arrayValidBytes_;
  int32_t arrayReadBytes_;
  ecmcAsynDataItem *asynParamReadBytes_;
  ecmcAsynDataItem *asynParamWriteValue_;
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
