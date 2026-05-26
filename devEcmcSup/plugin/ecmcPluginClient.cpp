/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcPlugin.cpp
*
*  Created on: Oct 21, 2020
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcPluginClient.h"
#include "ecmcOctetIF.h"        // Log Macros
#include "ecmcRtLogger.h"
#include "ecmcDefinitions.h"
#include "ecmcErrorsList.h"
#include "ecmcMainThread.h"
#include "ecmcAxisGroup.h"
#include "ecmcMotion.h"

// TODO: REMOVE GLOBALS
#include "ecmcGlobalsExtern.h"

namespace {
int g_m2mErrorCode = 0;
int g_dataStorageErrorCode = 0;

int checkM2mIndex(int shmIndex) {
  g_m2mErrorCode = 0;

  if (shmIndex >= ECMC_SHM_ELEMENTS || shmIndex < 0) {
    g_m2mErrorCode = ERROR_SHM_INDEX_OUT_OF_RANGE;
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: SHM index out of range.\n",
                         __FILE__,
                         __FUNCTION__,
                         __LINE__);
    return g_m2mErrorCode;
  }

  if (shmObj.valid == 0) {
    g_m2mErrorCode = ERROR_SHM_NULL;
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: SHM object is NULL.\n",
                         __FILE__,
                         __FUNCTION__,
                         __LINE__);
    return g_m2mErrorCode;
  }

  return 0;
}

bool m2mMasterIndexValid(int masterIndex) {
  return shmObj.valid != 0 &&
         masterIndex < ECMC_SHM_MAX_MASTERS &&
         masterIndex > -ECMC_SHM_MAX_MASTERS;
}

uint32_t packAxisStatusWord(const ecmcAxisStatusWordType& word) {
  uint32_t packed = 0u;
  packed |= word.enable ? ECMC_CPP_AXIS_STATUS_ENABLE : 0u;
  packed |= word.enabled ? ECMC_CPP_AXIS_STATUS_ENABLED : 0u;
  packed |= word.execute ? ECMC_CPP_AXIS_STATUS_EXECUTE : 0u;
  packed |= word.busy ? ECMC_CPP_AXIS_STATUS_BUSY : 0u;
  packed |= word.attarget ? ECMC_CPP_AXIS_STATUS_AT_TARGET : 0u;
  packed |= word.moving ? ECMC_CPP_AXIS_STATUS_MOVING : 0u;
  packed |= word.limitfwd ? ECMC_CPP_AXIS_STATUS_LIMIT_FWD : 0u;
  packed |= word.limitbwd ? ECMC_CPP_AXIS_STATUS_LIMIT_BWD : 0u;
  packed |= word.homeswitch ? ECMC_CPP_AXIS_STATUS_HOME_SWITCH : 0u;
  packed |= word.homed ? ECMC_CPP_AXIS_STATUS_HOMED : 0u;
  packed |= word.inrealtime ? ECMC_CPP_AXIS_STATUS_IN_REALTIME : 0u;
  packed |= word.trajsource ? ECMC_CPP_AXIS_STATUS_TRAJ_EXTERNAL : 0u;
  packed |= word.encsource ? ECMC_CPP_AXIS_STATUS_ENC_EXTERNAL : 0u;
  packed |= word.plccmdallowed ? ECMC_CPP_AXIS_STATUS_PLC_CMD_ALLOWED : 0u;
  packed |= word.softlimfwdena ? ECMC_CPP_AXIS_STATUS_SOFT_LIMIT_FWD_EN : 0u;
  packed |= word.softlimbwdena ? ECMC_CPP_AXIS_STATUS_SOFT_LIMIT_BWD_EN : 0u;
  packed |= word.instartup ? ECMC_CPP_AXIS_STATUS_IN_STARTUP : 0u;
  packed |= word.sumilockfwd ? ECMC_CPP_AXIS_STATUS_SUM_ILOCK_FWD : 0u;
  packed |= word.sumilockbwd ? ECMC_CPP_AXIS_STATUS_SUM_ILOCK_BWD : 0u;
  packed |= word.softlimilockfwd ? ECMC_CPP_AXIS_STATUS_SOFT_ILOCK_FWD : 0u;
  packed |= word.softlimilockbwd ? ECMC_CPP_AXIS_STATUS_SOFT_ILOCK_BWD : 0u;
  packed |= word.localBusy ? ECMC_CPP_AXIS_STATUS_LOCAL_BUSY : 0u;
  packed |= word.globalBusy ? ECMC_CPP_AXIS_STATUS_GLOBAL_BUSY : 0u;
  packed |= word.blocked ? ECMC_CPP_AXIS_STATUS_BLOCKED : 0u;
  return packed;
}

int checkDataStorageIndex(int storageIndex) {
  g_dataStorageErrorCode = 0;

  if (storageIndex >= ECMC_MAX_DATA_STORAGE_OBJECTS || storageIndex < 0) {
    g_dataStorageErrorCode = ERROR_PLC_DATA_STORAGE_INDEX_OUT_OF_RANGE;
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Data storage index out of range.\n",
                         __FILE__,
                         __FUNCTION__,
                         __LINE__);
    return g_dataStorageErrorCode;
  }

  if (!dataStorages[storageIndex]) {
    g_dataStorageErrorCode = ERROR_PLC_DATA_STORAGE_OBJECT_NULL;
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Data storage object is NULL.\n",
                         __FILE__,
                         __FUNCTION__,
                         __LINE__);
    return g_dataStorageErrorCode;
  }

  return 0;
}

int checkAxisGroupIndex(int groupIndex) {
  if (groupIndex >= ECMC_MAX_AXES || groupIndex < 0) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Axis group index out of range.\n",
                         __FILE__,
                         __FUNCTION__,
                         __LINE__);
    return ERROR_PLC_AXIS_ID_OUT_OF_RANGE;
  }

  if (!axisGroups[groupIndex]) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Axis group object is NULL.\n",
                         __FILE__,
                         __FUNCTION__,
                         __LINE__);
    return ERROR_PLC_AXIS_OBJECT_NULL;
  }

  return 0;
}
}

void* getEcmcDataItem(char *idStringWP) {
  LOGINFO4("%s/%s:%d: idStringWP =%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           idStringWP);

  if (!asynPort)return NULL;

  return (void *)asynPort->findAvailDataItem(idStringWP);
}

void* getEcmcAsynDataItem(char *idStringWP) {
  LOGINFO4("%s/%s:%d: idStringWP =%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           idStringWP);

  if (!asynPort)return NULL;

  return (void *)asynPort->findAvailDataItem(idStringWP);
}

void* getEcMaster() {
  return (void *)ec;
}

int getEcmcMasterIndex() {
  if (!ec) {
    return -1;
  }
  return ec->getMasterIndex();
}

uint32_t getEcmcSlaveStateWord(int masterIndex, int slaveIndex) {
  if (!ec) {
    return 0u;
  }

  if (masterIndex >= 0 && ec->getMasterIndex() != masterIndex) {
    return 0u;
  }

  ecmcEcSlave* slave = ec->getSlave(slaveIndex);
  if (!slave) {
    return 0u;
  }

  ec_slave_config_state_t state {};
  if (slave->getSlaveState(&state) != 0) {
    return 0u;
  }

  uint32_t word = 0u;
  word |= 1u << 0;
  if (state.online) {
    word |= 1u << 1;
  }
  if (state.operational) {
    word |= 1u << 2;
  }
  word |= (static_cast<uint32_t>(state.al_state) & 0xFu) << 3;
  return word;
}

uint32_t getEcmcMasterStateWord(int masterIndex) {
  if (!ec) {
    return 0u;
  }

  if (masterIndex >= 0 && ec->getMasterIndex() != masterIndex) {
    return 0u;
  }

  ec_master_t* master = ec->getMaster();
  if (!master) {
    return 0u;
  }

  ec_master_state_t state {};
  ecrt_master_state(master, &state);

  uint32_t word = 0u;
  word |= 1u << 0;
  if (state.link_up) {
    word |= 1u << 1;
  }
  word |= (static_cast<uint32_t>(state.al_states) & 0xFu) << 2;
  word |= (static_cast<uint32_t>(state.slaves_responding) & 0xFFFFu) << 16;
  return word;
}

uint64_t getEcmcEcTimeNs() {
  return ec ? ec->getTimeNs() : 0u;
}

uint64_t getEcmcEcTimeOffsetNs() {
  return ec ? ec->getTimeOffsetNs() : 0u;
}

uint64_t getEcmcEcLastReceiveTimeNs() {
  return ec ? ec->getLastReceiveTimeNs() : 0u;
}

uint64_t getEcmcEcLastSendTimeNs() {
  return ec ? ec->getLastSendTimeNs() : 0u;
}

int getEcmcEcDomainState(int domainIndex) {
  return (ec && domainIndex >= 0) ? ec->getDomState(domainIndex) : -1;
}

int getEcmcEcStatusOK() {
  return ec ? ec->statusOK() : 0;
}

int setEcmcAxisTrajSource(int axisIndex, int source) {
  return setAxisTrajSource(axisIndex, source);
}

int setEcmcAxisEncSource(int axisIndex, int source) {
  return setAxisEncSource(axisIndex, source);
}

int setEcmcAxisExtSetPos(int axisIndex, double value) {
  return setAxisExtSetPos(axisIndex, value);
}

int setEcmcAxisExtActPos(int axisIndex, double value) {
  return setAxisExtActPos(axisIndex, value);
}

double getEcmcAxisEncoderActualPos(int axisIndex, int encoderIndex) {
  double value = 0.0;
  if (getAxisEncPosActByIndex(axisIndex, encoderIndex, &value)) {
    return 0.0;
  }
  return value;
}

int setEcmcAxisEncoderActualPos(int axisIndex, int encoderIndex, double value) {
  return setAxisEncPosActByIndex(axisIndex, encoderIndex, value);
}

int getEcmcAxisEncoderHomed(int axisIndex, int encoderIndex) {
  int value = 0;
  if (getAxisEncHomedByIndex(axisIndex, encoderIndex, &value)) {
    return -1;
  }
  return value;
}

int setEcmcAxisEncoderHomed(int axisIndex, int encoderIndex, int homed) {
  return setAxisEncHomedByIndex(axisIndex, encoderIndex, homed);
}

int getEcmcAxisEncoderReady(int axisIndex, int encoderIndex) {
  int value = 0;
  if (getAxisEncReadyByIndex(axisIndex, encoderIndex, &value)) {
    return -1;
  }
  return value;
}

int getEcmcAxisPrimaryEncoder(int axisIndex) {
  int value = -1;
  if (getAxisEncPrimaryIndex(axisIndex, &value)) {
    return -1;
  }
  return value;
}

int selectEcmcAxisPrimaryEncoder(int axisIndex, int encoderIndex) {
  return selectAxisEncPrimary(axisIndex, encoderIndex);
}

int getEcmcAxisStatus(int axisIndex, struct ecmcCppAxisStatus* status) {
  if (!status) {
    return -1;
  }

  *status = {};
  if (axisIndex >= ECMC_MAX_AXES || axisIndex < 0) {
    return ERROR_PLC_AXIS_ID_OUT_OF_RANGE;
  }

  if (!axes[axisIndex]) {
    return ERROR_PLC_AXIS_OBJECT_NULL;
  }

  const ecmcAxisDataStatus* const source = axes[axisIndex]->getAxisStatusDataPtr();
  if (!source) {
    return ERROR_PLC_AXIS_OBJECT_NULL;
  }

  status->valid = 1u;
  status->status_word = packAxisStatusWord(source->statusWord_);
  status->seq_state = source->statusWord_.seqstate;
  status->last_interlock = source->statusWord_.lastilock;
  status->traj_source = source->statusWord_.trajsource;
  status->enc_source = source->statusWord_.encsource;
  status->axis_id = source->axisId;
  status->axis_type = source->axisType;
  status->command = source->command;
  status->cmd_data = source->cmdData;
  status->encoder_count = source->encoderCount;
  status->error_code = source->errorCode;
  status->warning_code = source->warningCode;
  status->cycle_counter = source->cycleCounter;
  status->ctrl_within_deadband = source->ctrlWithinDeadband ? 1 : 0;
  status->limit_fwd_filtered = source->limitFwdFiltered ? 1 : 0;
  status->limit_bwd_filtered = source->limitBwdFiltered ? 1 : 0;
  status->home_switch_filtered = source->homeSwitchFiltered ? 1 : 0;
  status->startup_finished = source->startupFinsished ? 1 : 0;
  status->sample_time = source->sampleTime;
  status->external_trajectory_position = source->externalTrajectoryPosition;
  status->external_trajectory_velocity = source->externalTrajectoryVelocity;
  status->external_encoder_position = source->externalEncoderPosition;
  status->external_encoder_velocity = source->externalEncoderVelocity;
  status->current_position_actual = source->currentPositionActual;
  status->current_position_setpoint = source->currentPositionSetpoint;
  status->current_csp_position_setpoint_offset =
    source->currentCSPPositionSetpointOffset;
  status->current_target_position = source->currentTargetPosition;
  status->current_target_position_modulo = source->currentTargetPositionModulo;
  status->current_velocity_actual = source->currentVelocityActual;
  status->current_velocity_setpoint = source->currentVelocitySetpoint;
  status->current_velocity_setpoint_raw = source->currentVelocitySetpointRaw;
  status->current_velocity_target = source->currentVelocityTarget;
  status->current_position_setpoint_raw = source->currentPositionSetpointRaw;
  status->current_position_actual_raw = source->currentPositionActualRaw;
  status->current_velocity_ff_raw = source->currentvelocityFFRaw;
  status->control_error = source->cntrlError;
  status->control_output = source->cntrlOutput;
  status->control_output_old = source->cntrlOutputOld;
  status->current_acceleration_setpoint = source->currentAccelerationSetpoint;
  status->current_deceleration_setpoint = source->currentDecelerationSetpoint;
  status->distance_to_stop = source->distToStop;
  return 0;
}

int getEcmcAxisTrajSource(int axisIndex) {
  int value = -1;
  if (getAxisTrajSource(axisIndex, &value)) {
    return -1;
  }
  return value;
}

int getEcmcAxisEncSource(int axisIndex) {
  int value = -1;
  if (getAxisEncSource(axisIndex, &value)) {
    return -1;
  }
  return value;
}

double getEcmcAxisActualPos(int axisIndex) {
  double value = 0.0;
  if (getAxisEncPosAct(axisIndex, &value)) {
    return 0.0;
  }
  return value;
}

double getEcmcAxisSetpointPos(int axisIndex) {
  double value = 0.0;
  if (getAxisPosSet(axisIndex, &value)) {
    return 0.0;
  }
  return value;
}

double getEcmcAxisActualVel(int axisIndex) {
  double value = 0.0;
  if (getAxisEncVelAct(axisIndex, &value)) {
    return 0.0;
  }
  return value;
}

double getEcmcAxisSetpointVel(int axisIndex) {
  double value = 0.0;
  if (getAxisTrajVelo(axisIndex, &value)) {
    return 0.0;
  }
  return value;
}

int getEcmcAxisEnabled(int axisIndex) {
  int value = 0;
  if (getAxisEnabled(axisIndex, &value)) {
    return 0;
  }
  return value;
}

int getEcmcAxisBusy(int axisIndex) {
  int value = 0;
  if (getAxisBusy(axisIndex, &value)) {
    return 0;
  }
  return value;
}

int getEcmcAxisError(int axisIndex) {
  return getAxisError(axisIndex);
}

int getEcmcAxisErrorId(int axisIndex) {
  return getAxisErrorID(axisIndex);
}

int ecmcM2mWrite(int shmIndex, double value) {
  if (checkM2mIndex(shmIndex)) {
    return g_m2mErrorCode;
  }

  sem_wait(shmObj.sem);
  shmObj.dataPtr[shmIndex] = value;
  sem_post(shmObj.sem);
  return 0;
}

double ecmcM2mRead(int shmIndex) {
  if (checkM2mIndex(shmIndex)) {
    return 0.0;
  }

  sem_wait(shmObj.sem);
  const double value = shmObj.dataPtr[shmIndex];
  sem_post(shmObj.sem);
  return value;
}

int ecmcM2mStatus() {
  if (shmObj.valid == 0) {
    g_m2mErrorCode = ERROR_SHM_NULL;
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: SHM object is NULL.\n",
                         __FILE__,
                         __FUNCTION__,
                         __LINE__);
    return -ERROR_SHM_NULL;
  }

  return shmObj.valid;
}

int ecmcM2mResetError() {
  g_m2mErrorCode = 0;
  return 0;
}

int ecmcM2mGetError() {
  return g_m2mErrorCode;
}

int ecmcM2mIocRun(int masterIndex) {
  if (!m2mMasterIndexValid(masterIndex)) {
    return -1;
  }

  if (masterIndex >= 0) {
    return shmObj.mstPtr[masterIndex] > 0 ? 1 : 0;
  }

  return shmObj.simMstPtr[-masterIndex + ECMC_SHM_MAX_MASTERS] > 0 ? 1 : 0;
}

int ecmcM2mIocEcOk(int masterIndex) {
  if (!m2mMasterIndexValid(masterIndex)) {
    return -1;
  }

  if (masterIndex >= 0) {
    return shmObj.mstPtr[masterIndex] > 1 ? 1 : 0;
  }

  return 0;
}

int ecmcDataStorageClear(int storageIndex) {
  if (checkDataStorageIndex(storageIndex)) {
    return g_dataStorageErrorCode;
  }

  g_dataStorageErrorCode = dataStorages[storageIndex]->clearBuffer();
  return g_dataStorageErrorCode;
}

int ecmcDataStorageAppend(int storageIndex, double value) {
  if (checkDataStorageIndex(storageIndex)) {
    return g_dataStorageErrorCode;
  }

  g_dataStorageErrorCode = dataStorages[storageIndex]->appendData(value);
  return g_dataStorageErrorCode;
}

double ecmcDataStorageGet(int storageIndex, int dataIndex) {
  if (checkDataStorageIndex(storageIndex)) {
    return 0.0;
  }

  double value = 0.0;
  g_dataStorageErrorCode = dataStorages[storageIndex]->getDataElement(dataIndex, &value);
  return value;
}

int ecmcDataStorageSet(int storageIndex, int dataIndex, double value) {
  if (checkDataStorageIndex(storageIndex)) {
    return g_dataStorageErrorCode;
  }

  g_dataStorageErrorCode = dataStorages[storageIndex]->setDataElement(dataIndex, value);
  return g_dataStorageErrorCode;
}

int ecmcDataStorageGetIndex(int storageIndex) {
  if (checkDataStorageIndex(storageIndex)) {
    return -1;
  }

  return dataStorages[storageIndex]->getCurrentIndex();
}

int ecmcDataStorageSetIndex(int storageIndex, int dataIndex) {
  if (checkDataStorageIndex(storageIndex)) {
    return g_dataStorageErrorCode;
  }

  g_dataStorageErrorCode = dataStorages[storageIndex]->setCurrentPosition(dataIndex);
  return g_dataStorageErrorCode;
}

int ecmcDataStorageGetError() {
  return g_dataStorageErrorCode;
}

int ecmcDataStorageResetError() {
  g_dataStorageErrorCode = 0;
  return 0;
}

int ecmcDataStorageIsFull(int storageIndex) {
  if (checkDataStorageIndex(storageIndex)) {
    return -1;
  }

  g_dataStorageErrorCode = 0;
  return dataStorages[storageIndex]->isStorageFull();
}

int ecmcDataStorageGetSize(int storageIndex) {
  if (checkDataStorageIndex(storageIndex)) {
    return -1;
  }

  g_dataStorageErrorCode = 0;
  return dataStorages[storageIndex]->getSize();
}

int ecmcDataStoragePushAsyn(int storageIndex) {
  if (checkDataStorageIndex(storageIndex)) {
    return -1;
  }

  g_dataStorageErrorCode = dataStorages[storageIndex]->updateAsyn(1);
  return g_dataStorageErrorCode;
}

double ecmcDataStorageGetAvg(int storageIndex) {
  if (checkDataStorageIndex(storageIndex)) {
    return 0.0;
  }

  g_dataStorageErrorCode = 0;
  return dataStorages[storageIndex]->getAvg();
}

double ecmcDataStorageGetMin(int storageIndex) {
  if (checkDataStorageIndex(storageIndex)) {
    return 0.0;
  }

  g_dataStorageErrorCode = 0;
  return dataStorages[storageIndex]->getMin();
}

double ecmcDataStorageGetMax(int storageIndex) {
  if (checkDataStorageIndex(storageIndex)) {
    return 0.0;
  }

  g_dataStorageErrorCode = 0;
  return dataStorages[storageIndex]->getMax();
}

int ecmcDataStorageRead(int storageIndex,
                        double* data,
                        uint32_t capacity,
                        uint32_t* sizeOut) {
  if (checkDataStorageIndex(storageIndex)) {
    return g_dataStorageErrorCode;
  }

  double* source = nullptr;
  int size = 0;
  g_dataStorageErrorCode = dataStorages[storageIndex]->getData(&source, &size);
  if (g_dataStorageErrorCode) {
    return g_dataStorageErrorCode;
  }

  if (sizeOut) {
    *sizeOut = static_cast<uint32_t>(size);
  }

  if (!data || capacity == 0u) {
    return 0;
  }

  if (capacity < static_cast<uint32_t>(size)) {
    g_dataStorageErrorCode = ERROR_DATA_STORAGE_SIZE_TO_SMALL;
    return g_dataStorageErrorCode;
  }

  for (int i = 0; i < size; ++i) {
    data[i] = source[i];
  }
  return 0;
}

int ecmcDataStorageWrite(int storageIndex, const double* data, uint32_t size) {
  if (checkDataStorageIndex(storageIndex)) {
    return g_dataStorageErrorCode;
  }
  if (!data && size > 0u) {
    g_dataStorageErrorCode = ERROR_DATA_STORAGE_NULL;
    return g_dataStorageErrorCode;
  }

  g_dataStorageErrorCode =
    dataStorages[storageIndex]->setData(const_cast<double*>(data), static_cast<int>(size));
  return g_dataStorageErrorCode;
}

int ecmcDataStorageAppendArray(int storageIndex, const double* data, uint32_t size) {
  if (checkDataStorageIndex(storageIndex)) {
    return g_dataStorageErrorCode;
  }
  if (!data && size > 0u) {
    g_dataStorageErrorCode = ERROR_DATA_STORAGE_NULL;
    return g_dataStorageErrorCode;
  }

  g_dataStorageErrorCode =
    dataStorages[storageIndex]->appendData(const_cast<double*>(data), static_cast<int>(size));
  return g_dataStorageErrorCode;
}

int ecmcDataStorageAppendFromStorage(int fromStorageIndex,
                                     int fromDataIndex,
                                     uint32_t elements,
                                     int toStorageIndex) {
  if (checkDataStorageIndex(fromStorageIndex) ||
      checkDataStorageIndex(toStorageIndex)) {
    return g_dataStorageErrorCode;
  }

  const int sourceSize = dataStorages[fromStorageIndex]->getSize();
  if (fromDataIndex >= sourceSize || fromDataIndex < 0) {
    g_dataStorageErrorCode = ERROR_DATA_STORAGE_POSITION_OUT_OF_RANGE;
    return g_dataStorageErrorCode;
  }

  int elementsLocal = static_cast<int>(elements);
  if (fromDataIndex + elementsLocal >= sourceSize) {
    elementsLocal = sourceSize - fromDataIndex;
  }

  double* source = nullptr;
  g_dataStorageErrorCode =
    dataStorages[fromStorageIndex]->getDataElementPtr(fromDataIndex, &source);
  if (g_dataStorageErrorCode) {
    return g_dataStorageErrorCode;
  }

  g_dataStorageErrorCode = dataStorages[toStorageIndex]->appendData(source, elementsLocal);
  return g_dataStorageErrorCode;
}

int ecmcAxisGroupGetEnable(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getEnable() ? 1 : 0;
}

int ecmcAxisGroupGetAnyEnable(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getAnyEnable() ? 1 : 0;
}

int ecmcAxisGroupGetEnabled(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getEnabled() ? 1 : 0;
}

int ecmcAxisGroupGetAnyEnabled(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getAnyEnabled() ? 1 : 0;
}

int ecmcAxisGroupGetBusy(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getBusy() ? 1 : 0;
}

int ecmcAxisGroupGetAnyBusy(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getAnyBusy() ? 1 : 0;
}

int ecmcAxisGroupGetAnyErrorId(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getAnyErrorId();
}

int ecmcAxisGroupSetEnable(int groupIndex, int enable) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  ecmcAxisGroup* const group = axisGroups[groupIndex];
  if (enable) {
    for (int axisIndex = 0; axisIndex < ECMC_MAX_AXES; ++axisIndex) {
      if (group->inGroup(axisIndex) &&
          axes[axisIndex] &&
          axes[axisIndex]->getBlockCom()) {
        axes[axisIndex]->setExternalCommandBlockedError();
        return ERROR_MAIN_AXIS_EXTERNAL_COM_DISABLED;
      }
    }
  }

  return group->setEnable(enable != 0);
}

int ecmcAxisGroupSetTrajSource(int groupIndex, int source) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  return axisGroups[groupIndex]->setTrajSrc(static_cast<dataSource>(source));
}

int ecmcAxisGroupSetEncSource(int groupIndex, int source) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  return axisGroups[groupIndex]->setEncSrc(static_cast<dataSource>(source));
}

int ecmcAxisGroupResetError(int groupIndex) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  axisGroups[groupIndex]->setErrorReset();
  return 0;
}

int ecmcAxisGroupSetError(int groupIndex, int errorId) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  axisGroups[groupIndex]->setError(errorId);
  return 0;
}

int ecmcAxisGroupSetSlavedAxisInError(int groupIndex) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  axisGroups[groupIndex]->setSlavedAxisInError();
  return 0;
}

int ecmcAxisGroupHalt(int groupIndex) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  axisGroups[groupIndex]->halt();
  return 0;
}

int ecmcAxisGroupAxisInGroup(int groupIndex, int axisIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->inGroup(axisIndex) ? 1 : 0;
}

int ecmcAxisGroupSize(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return static_cast<int>(axisGroups[groupIndex]->size());
}

int ecmcAxisGroupGetTrajSourceExt(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getTrajSrcExt() ? 1 : 0;
}

int ecmcAxisGroupGetAnyTrajSourceExt(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getTrajSrcAnyExt() ? 1 : 0;
}

int ecmcAxisGroupSetAllowSourceChangeWhenEnabled(int groupIndex, int allow) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  axisGroups[groupIndex]->setAllowSrcChangeWhenEnabled(allow);
  return 0;
}

int ecmcAxisGroupSetMrSync(int groupIndex, int sync) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  axisGroups[groupIndex]->setMRSync(sync);
  return 0;
}

int ecmcAxisGroupSetMrStop(int groupIndex, int stop) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  axisGroups[groupIndex]->setMRStop(stop);
  return 0;
}

int ecmcAxisGroupSetMrCnen(int groupIndex, int enable) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  axisGroups[groupIndex]->setMRCnen(enable);
  return 0;
}

int ecmcAxisGroupSetAutoEnable(int groupIndex, int enable) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  axisGroups[groupIndex]->setEnableAutoEnable(enable);
  return 0;
}

int ecmcAxisGroupSetAutoDisable(int groupIndex, int enable) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  axisGroups[groupIndex]->setEnableAutoDisable(enable);
  return 0;
}

int ecmcAxisGroupSetCtrlWithinDeadband(int groupIndex, int within) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  axisGroups[groupIndex]->setAxisIsWithinCtrlDBExtTraj(within);
  return 0;
}

int ecmcAxisGroupSetIgnoreMrStatusCheckAtDisable(int groupIndex, int ignore) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  axisGroups[groupIndex]->setMRIgnoreDisableStatusCheck(ignore);
  return 0;
}

int ecmcAxisGroupGetAnyAtForwardLimit(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getAnyAtLimitFwd() ? 1 : 0;
}

int ecmcAxisGroupGetAnyAtBackwardLimit(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getAnyAtLimitBwd() ? 1 : 0;
}

int ecmcAxisGroupGetAnyAtLimit(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getAnyAtLimit() ? 1 : 0;
}

int ecmcAxisGroupGetAnyInterlocked(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getAnyIlocked() ? 1 : 0;
}

int ecmcAxisGroupSetSlavedAxisInterlocked(int groupIndex) {
  const int error = checkAxisGroupIndex(groupIndex);
  if (error) {
    return error;
  }

  axisGroups[groupIndex]->setSlavedAxisIlocked();
  return 0;
}

int ecmcAxisGroupGetCtrlWithinDeadband(int groupIndex) {
  if (checkAxisGroupIndex(groupIndex)) {
    return -1;
  }

  return axisGroups[groupIndex]->getAxisIsWithinCtrlDB() ? 1 : 0;
}

double getEcmcLutValue(int lutIndex, double index) {
  if (lutIndex < 0 || lutIndex >= ECMC_MAX_LUTS) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: LUT list index out of range.\n",
                         __FILE__,
                         __FUNCTION__,
                         __LINE__);
    return 0.0;
  }

  if (!luts[lutIndex]) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: LUT object is NULL.\n",
                         __FILE__,
                         __FUNCTION__,
                         __LINE__);
    return 0.0;
  }

  return luts[lutIndex]->getValue(index);
}

int getEcmcLutExists(int lutIndex) {
  if (lutIndex < 0 || lutIndex >= ECMC_MAX_LUTS) {
    return 0;
  }

  return luts[lutIndex] ? 1 : 0;
}

int requestEcmcIocExit(int exitCode) {
  return requestIocExitFromRt(exitCode);
}

void* getEcmcAsynPortDriver() {
  LOGINFO4("%s/%s:%d:\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);

  return (void *)asynPort;
}

double getEcmcSampleRate() {
  LOGINFO4("%s/%s:%d:\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);

  return mcuFrequency;
}

double getEcmcSampleTimeMS() {
  // mcuPeriod is in nano seconds
  return mcuPeriod / 1E6;
}

int getEcmcEpicsIOCState() {
  if (!asynPort) {
    return -1;
  }
  return asynPort->getEpicsState();
}

void publishEcmcDebugText(const char* message) {
  if (!message || !message[0]) {
    return;
  }

  LOGINFO("[ecmc cpp_logic] %s\n", message);
}

void getEcmcCppLogicHostServices(struct ecmcCppLogicHostServices* services) {
  if (!services) {
    return;
  }

  *services = {};
  services->version = ECMC_CPP_LOGIC_ABI_VERSION;
  services->get_cycle_time_s = []() -> double { return getEcmcSampleTimeMS() * 1e-3; };
  services->get_ec_time_ns = &getEcmcEcTimeNs;
  services->get_ec_time_offset_ns = &getEcmcEcTimeOffsetNs;
  services->get_ec_last_receive_time_ns = &getEcmcEcLastReceiveTimeNs;
  services->get_ec_last_send_time_ns = &getEcmcEcLastSendTimeNs;
  services->get_ec_domain_state = &getEcmcEcDomainState;
  services->get_ec_status_ok = &getEcmcEcStatusOK;
  services->get_ec_master_state_word = &getEcmcMasterStateWord;
  services->get_ec_slave_state_word = &getEcmcSlaveStateWord;
  services->get_axis_traj_source = &getEcmcAxisTrajSource;
  services->get_axis_enc_source = &getEcmcAxisEncSource;
  services->get_axis_actual_pos = &getEcmcAxisActualPos;
  services->get_axis_setpoint_pos = &getEcmcAxisSetpointPos;
  services->get_axis_actual_vel = &getEcmcAxisActualVel;
  services->get_axis_setpoint_vel = &getEcmcAxisSetpointVel;
  services->get_axis_enabled = &getEcmcAxisEnabled;
  services->get_axis_busy = &getEcmcAxisBusy;
  services->get_axis_error = &getEcmcAxisError;
  services->get_axis_error_id = &getEcmcAxisErrorId;
  services->set_axis_traj_source = &setEcmcAxisTrajSource;
  services->set_axis_enc_source = &setEcmcAxisEncSource;
  services->set_axis_ext_set_pos = &setEcmcAxisExtSetPos;
  services->set_axis_ext_act_pos = &setEcmcAxisExtActPos;
  services->get_axis_encoder_actual_pos = &getEcmcAxisEncoderActualPos;
  services->set_axis_encoder_actual_pos = &setEcmcAxisEncoderActualPos;
  services->get_axis_encoder_homed = &getEcmcAxisEncoderHomed;
  services->set_axis_encoder_homed = &setEcmcAxisEncoderHomed;
  services->get_axis_encoder_ready = &getEcmcAxisEncoderReady;
  services->get_axis_primary_encoder = &getEcmcAxisPrimaryEncoder;
  services->select_axis_primary_encoder = &selectEcmcAxisPrimaryEncoder;
  services->get_axis_status = &getEcmcAxisStatus;
  services->get_ioc_state = &getEcmcEpicsIOCState;
  services->publish_debug_text = &publishEcmcDebugText;
  services->get_lut_value = &getEcmcLutValue;
  services->lut_exists = &getEcmcLutExists;
  services->request_ioc_exit = &requestEcmcIocExit;
  services->m2m_write = &ecmcM2mWrite;
  services->m2m_read = &ecmcM2mRead;
  services->m2m_status = &ecmcM2mStatus;
  services->m2m_reset_error = &ecmcM2mResetError;
  services->m2m_get_error = &ecmcM2mGetError;
  services->m2m_ioc_run = &ecmcM2mIocRun;
  services->m2m_ioc_ec_ok = &ecmcM2mIocEcOk;
  services->ds_clear = &ecmcDataStorageClear;
  services->ds_append = &ecmcDataStorageAppend;
  services->ds_get = &ecmcDataStorageGet;
  services->ds_set = &ecmcDataStorageSet;
  services->ds_get_index = &ecmcDataStorageGetIndex;
  services->ds_set_index = &ecmcDataStorageSetIndex;
  services->ds_get_error = &ecmcDataStorageGetError;
  services->ds_reset_error = &ecmcDataStorageResetError;
  services->ds_is_full = &ecmcDataStorageIsFull;
  services->ds_get_size = &ecmcDataStorageGetSize;
  services->ds_push_asyn = &ecmcDataStoragePushAsyn;
  services->ds_get_avg = &ecmcDataStorageGetAvg;
  services->ds_get_min = &ecmcDataStorageGetMin;
  services->ds_get_max = &ecmcDataStorageGetMax;
  services->ds_read = &ecmcDataStorageRead;
  services->ds_write = &ecmcDataStorageWrite;
  services->ds_append_array = &ecmcDataStorageAppendArray;
  services->ds_append_from_storage = &ecmcDataStorageAppendFromStorage;
  services->axis_group_get_enable = &ecmcAxisGroupGetEnable;
  services->axis_group_get_any_enable = &ecmcAxisGroupGetAnyEnable;
  services->axis_group_get_enabled = &ecmcAxisGroupGetEnabled;
  services->axis_group_get_any_enabled = &ecmcAxisGroupGetAnyEnabled;
  services->axis_group_get_busy = &ecmcAxisGroupGetBusy;
  services->axis_group_get_any_busy = &ecmcAxisGroupGetAnyBusy;
  services->axis_group_get_any_error_id = &ecmcAxisGroupGetAnyErrorId;
  services->axis_group_set_enable = &ecmcAxisGroupSetEnable;
  services->axis_group_set_traj_source = &ecmcAxisGroupSetTrajSource;
  services->axis_group_set_enc_source = &ecmcAxisGroupSetEncSource;
  services->axis_group_reset_error = &ecmcAxisGroupResetError;
  services->axis_group_set_error = &ecmcAxisGroupSetError;
  services->axis_group_set_slaved_axis_in_error = &ecmcAxisGroupSetSlavedAxisInError;
  services->axis_group_halt = &ecmcAxisGroupHalt;
  services->axis_group_axis_in_group = &ecmcAxisGroupAxisInGroup;
  services->axis_group_size = &ecmcAxisGroupSize;
  services->axis_group_get_traj_source_ext = &ecmcAxisGroupGetTrajSourceExt;
  services->axis_group_get_any_traj_source_ext = &ecmcAxisGroupGetAnyTrajSourceExt;
  services->axis_group_set_allow_source_change_when_enabled =
    &ecmcAxisGroupSetAllowSourceChangeWhenEnabled;
  services->axis_group_set_mr_sync = &ecmcAxisGroupSetMrSync;
  services->axis_group_set_mr_stop = &ecmcAxisGroupSetMrStop;
  services->axis_group_set_mr_cnen = &ecmcAxisGroupSetMrCnen;
  services->axis_group_set_auto_enable = &ecmcAxisGroupSetAutoEnable;
  services->axis_group_set_auto_disable = &ecmcAxisGroupSetAutoDisable;
  services->axis_group_set_ctrl_within_deadband = &ecmcAxisGroupSetCtrlWithinDeadband;
  services->axis_group_set_ignore_mr_status_check_at_disable =
    &ecmcAxisGroupSetIgnoreMrStatusCheckAtDisable;
  services->axis_group_get_any_at_forward_limit = &ecmcAxisGroupGetAnyAtForwardLimit;
  services->axis_group_get_any_at_backward_limit = &ecmcAxisGroupGetAnyAtBackwardLimit;
  services->axis_group_get_any_at_limit = &ecmcAxisGroupGetAnyAtLimit;
  services->axis_group_get_any_interlocked = &ecmcAxisGroupGetAnyInterlocked;
  services->axis_group_set_slaved_axis_interlocked =
    &ecmcAxisGroupSetSlavedAxisInterlocked;
  services->axis_group_get_ctrl_within_deadband = &ecmcAxisGroupGetCtrlWithinDeadband;
}
