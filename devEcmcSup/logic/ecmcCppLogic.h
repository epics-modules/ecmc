/*************************************************************************\
* Copyright (c) 2026 Paul Scherrer Institute
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcCppLogic.h
*
* Additive C/C++ logic interface for small cyclic logic modules.
* This does not replace the existing ecmc plugin ABI in ecmcPluginDefs.h.
*
\*************************************************************************/

#ifndef ECMC_CPP_LOGIC_H_
#define ECMC_CPP_LOGIC_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ECMC_CPP_LOGIC_ABI_VERSION 7

#define ECMC_CPP_BIND_FLAG_NONE 0u
#define ECMC_CPP_BIND_FLAG_AUTO_SIZE 1u

struct ecmcCppLogicItemBinding;

typedef int (*ecmcCppLogicPrepareItemBindingFn)(struct ecmcCppLogicItemBinding* binding,
                                                   uint32_t sourceBytes);

enum ecmcCppValueType {
  ECMC_CPP_TYPE_BOOL = 1,
  ECMC_CPP_TYPE_S8 = 2,
  ECMC_CPP_TYPE_U8 = 3,
  ECMC_CPP_TYPE_S16 = 4,
  ECMC_CPP_TYPE_U16 = 5,
  ECMC_CPP_TYPE_S32 = 6,
  ECMC_CPP_TYPE_U32 = 7,
  ECMC_CPP_TYPE_F32 = 8,
  ECMC_CPP_TYPE_F64 = 9,
  ECMC_CPP_TYPE_U64 = 10,
  ECMC_CPP_TYPE_S64 = 11,
};

struct ecmcCppLogicItemBinding {
  const char* itemName;
  void* data;
  uint32_t type;
  uint32_t bytes;
  uint32_t writable;
  uint32_t flags;
  ecmcCppLogicPrepareItemBindingFn prepare;
  void* prepareContext;
};

struct ecmcCppLogicExportedVar {
  const char* name;
  void* data;
  uint32_t type;
  uint32_t bytes;
  uint32_t writable;
};

struct ecmcCppLogicHostServices {
  uint32_t version;
  double (*get_cycle_time_s)();
  uint32_t (*get_ec_master_state_word)(int32_t master_index);
  uint32_t (*get_ec_slave_state_word)(int32_t master_index, int32_t slave_index);
  int32_t (*get_axis_traj_source)(int32_t axis_index);
  int32_t (*get_axis_enc_source)(int32_t axis_index);
  double (*get_axis_actual_pos)(int32_t axis_index);
  double (*get_axis_setpoint_pos)(int32_t axis_index);
  double (*get_axis_actual_vel)(int32_t axis_index);
  double (*get_axis_setpoint_vel)(int32_t axis_index);
  int32_t (*get_axis_enabled)(int32_t axis_index);
  int32_t (*get_axis_busy)(int32_t axis_index);
  int32_t (*get_axis_error)(int32_t axis_index);
  int32_t (*get_axis_error_id)(int32_t axis_index);
  int32_t (*set_axis_traj_source)(int32_t axis_index, int32_t source);
  int32_t (*set_axis_enc_source)(int32_t axis_index, int32_t source);
  int32_t (*set_axis_ext_set_pos)(int32_t axis_index, double value);
  int32_t (*set_axis_ext_act_pos)(int32_t axis_index, double value);
  int32_t (*set_enable_dbg)(int32_t enable);
  int32_t (*get_ioc_state)();
  const char* (*get_macros_text)();
  void (*publish_debug_text)(const char* message);
  double (*get_lut_value)(int32_t lut_index, double index);
  int32_t (*lut_exists)(int32_t lut_index);
  int32_t (*request_ioc_exit)(int32_t exit_code);
};

struct ecmcCppLogicApi {
  uint32_t abiVersion;
  const char* name;
  void (*setHostServices)(const struct ecmcCppLogicHostServices* services);
  void* (*createInstance)();
  void (*enterRealtime)(void* instance);
  void (*exitRealtime)(void* instance);
  void (*destroyInstance)(void* instance);
  void (*runCycle)(void* instance);
  const struct ecmcCppLogicItemBinding* (*getItemBindings)(void* instance);
  uint32_t (*getItemBindingCount)(void* instance);
  const struct ecmcCppLogicExportedVar* (*getExportedVars)(void* instance);
  uint32_t (*getExportedVarCount)(void* instance);
};

const struct ecmcCppLogicApi* ecmc_cpp_logic_get_api(void);

#ifdef __cplusplus
}
#endif

#endif
