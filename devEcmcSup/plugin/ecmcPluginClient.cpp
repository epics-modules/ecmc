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

// TODO: REMOVE GLOBALS
#include "ecmcGlobalsExtern.h"

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
  services->get_ioc_state = &getEcmcEpicsIOCState;
  services->publish_debug_text = &publishEcmcDebugText;
  services->get_lut_value = &getEcmcLutValue;
  services->lut_exists = &getEcmcLutExists;
}
