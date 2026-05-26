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

#define ECMC_CPP_LOGIC_ABI_VERSION 16

#define ECMC_CPP_LOGIC_CREATE_INSTANCE_FAIL 0x2006D

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

#define ECMC_CPP_AXIS_STATUS_ENABLE            (1u << 0)
#define ECMC_CPP_AXIS_STATUS_ENABLED           (1u << 1)
#define ECMC_CPP_AXIS_STATUS_EXECUTE           (1u << 2)
#define ECMC_CPP_AXIS_STATUS_BUSY              (1u << 3)
#define ECMC_CPP_AXIS_STATUS_AT_TARGET         (1u << 4)
#define ECMC_CPP_AXIS_STATUS_MOVING            (1u << 5)
#define ECMC_CPP_AXIS_STATUS_LIMIT_FWD         (1u << 6)
#define ECMC_CPP_AXIS_STATUS_LIMIT_BWD         (1u << 7)
#define ECMC_CPP_AXIS_STATUS_HOME_SWITCH       (1u << 8)
#define ECMC_CPP_AXIS_STATUS_HOMED             (1u << 9)
#define ECMC_CPP_AXIS_STATUS_IN_REALTIME       (1u << 10)
#define ECMC_CPP_AXIS_STATUS_TRAJ_EXTERNAL     (1u << 11)
#define ECMC_CPP_AXIS_STATUS_ENC_EXTERNAL      (1u << 12)
#define ECMC_CPP_AXIS_STATUS_PLC_CMD_ALLOWED   (1u << 13)
#define ECMC_CPP_AXIS_STATUS_SOFT_LIMIT_FWD_EN (1u << 14)
#define ECMC_CPP_AXIS_STATUS_SOFT_LIMIT_BWD_EN (1u << 15)
#define ECMC_CPP_AXIS_STATUS_IN_STARTUP        (1u << 16)
#define ECMC_CPP_AXIS_STATUS_SUM_ILOCK_FWD     (1u << 17)
#define ECMC_CPP_AXIS_STATUS_SUM_ILOCK_BWD     (1u << 18)
#define ECMC_CPP_AXIS_STATUS_SOFT_ILOCK_FWD    (1u << 19)
#define ECMC_CPP_AXIS_STATUS_SOFT_ILOCK_BWD    (1u << 20)
#define ECMC_CPP_AXIS_STATUS_LOCAL_BUSY        (1u << 21)
#define ECMC_CPP_AXIS_STATUS_GLOBAL_BUSY       (1u << 22)
#define ECMC_CPP_AXIS_STATUS_BLOCKED           (1u << 23)

struct ecmcCppAxisStatus {
  uint32_t valid;
  uint32_t status_word;
  int32_t seq_state;
  int32_t last_interlock;
  int32_t traj_source;
  int32_t enc_source;
  int32_t axis_id;
  int32_t axis_type;
  int32_t command;
  int32_t cmd_data;
  int32_t encoder_count;
  int32_t error_code;
  int32_t warning_code;
  int32_t cycle_counter;
  int32_t ctrl_within_deadband;
  int32_t limit_fwd_filtered;
  int32_t limit_bwd_filtered;
  int32_t home_switch_filtered;
  int32_t startup_finished;
  double sample_time;
  double external_trajectory_position;
  double external_trajectory_velocity;
  double external_encoder_position;
  double external_encoder_velocity;
  double current_position_actual;
  double current_position_setpoint;
  double current_csp_position_setpoint_offset;
  double current_target_position;
  double current_target_position_modulo;
  double current_velocity_actual;
  double current_velocity_setpoint;
  int64_t current_velocity_setpoint_raw;
  double current_velocity_target;
  int64_t current_position_setpoint_raw;
  int64_t current_position_actual_raw;
  double current_velocity_ff_raw;
  double control_error;
  double control_output;
  double control_output_old;
  double current_acceleration_setpoint;
  double current_deceleration_setpoint;
  double distance_to_stop;
};

struct ecmcCppLogicHostServices {
  uint32_t version;
  double (*get_cycle_time_s)();
  uint64_t (*get_ec_time_ns)();
  uint64_t (*get_ec_time_offset_ns)();
  uint64_t (*get_ec_last_receive_time_ns)();
  uint64_t (*get_ec_last_send_time_ns)();
  int32_t (*get_ec_domain_state)(int32_t domain_index);
  int32_t (*get_ec_status_ok)();
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
  double (*get_axis_encoder_actual_pos)(int32_t axis_index, int32_t encoder_index);
  int32_t (*set_axis_encoder_actual_pos)(int32_t axis_index,
                                         int32_t encoder_index,
                                         double value);
  int32_t (*get_axis_encoder_homed)(int32_t axis_index, int32_t encoder_index);
  int32_t (*set_axis_encoder_homed)(int32_t axis_index,
                                    int32_t encoder_index,
                                    int32_t homed);
  int32_t (*get_axis_encoder_ready)(int32_t axis_index, int32_t encoder_index);
  int32_t (*get_axis_primary_encoder)(int32_t axis_index);
  int32_t (*select_axis_primary_encoder)(int32_t axis_index, int32_t encoder_index);
  int32_t (*get_axis_status)(int32_t axis_index, struct ecmcCppAxisStatus* status);
  int32_t (*set_enable_dbg)(int32_t enable);
  int32_t (*get_ioc_state)();
  const char* (*get_macros_text)();
  void (*publish_debug_text)(const char* message);
  double (*get_lut_value)(int32_t lut_index, double index);
  int32_t (*lut_exists)(int32_t lut_index);
  int32_t (*request_ioc_exit)(int32_t exit_code);
  int32_t (*set_create_error_message)(const char* message);
  int32_t (*m2m_write)(int32_t shm_index, double value);
  double (*m2m_read)(int32_t shm_index);
  int32_t (*m2m_status)();
  int32_t (*m2m_reset_error)();
  int32_t (*m2m_get_error)();
  int32_t (*m2m_ioc_run)(int32_t master_index);
  int32_t (*m2m_ioc_ec_ok)(int32_t master_index);
  int32_t (*ds_clear)(int32_t storage_index);
  int32_t (*ds_append)(int32_t storage_index, double value);
  double (*ds_get)(int32_t storage_index, int32_t data_index);
  int32_t (*ds_set)(int32_t storage_index, int32_t data_index, double value);
  int32_t (*ds_get_index)(int32_t storage_index);
  int32_t (*ds_set_index)(int32_t storage_index, int32_t data_index);
  int32_t (*ds_get_error)();
  int32_t (*ds_reset_error)();
  int32_t (*ds_is_full)(int32_t storage_index);
  int32_t (*ds_get_size)(int32_t storage_index);
  int32_t (*ds_push_asyn)(int32_t storage_index);
  double (*ds_get_avg)(int32_t storage_index);
  double (*ds_get_min)(int32_t storage_index);
  double (*ds_get_max)(int32_t storage_index);
  int32_t (*ds_read)(int32_t storage_index,
                     double* data,
                     uint32_t capacity,
                     uint32_t* size_out);
  int32_t (*ds_write)(int32_t storage_index, const double* data, uint32_t size);
  int32_t (*ds_append_array)(int32_t storage_index, const double* data, uint32_t size);
  int32_t (*ds_append_from_storage)(int32_t from_storage_index,
                                    int32_t from_data_index,
                                    uint32_t elements,
                                    int32_t to_storage_index);
  int32_t (*axis_group_get_enable)(int32_t group_index);
  int32_t (*axis_group_get_any_enable)(int32_t group_index);
  int32_t (*axis_group_get_enabled)(int32_t group_index);
  int32_t (*axis_group_get_any_enabled)(int32_t group_index);
  int32_t (*axis_group_get_busy)(int32_t group_index);
  int32_t (*axis_group_get_any_busy)(int32_t group_index);
  int32_t (*axis_group_get_any_error_id)(int32_t group_index);
  int32_t (*axis_group_set_enable)(int32_t group_index, int32_t enable);
  int32_t (*axis_group_set_traj_source)(int32_t group_index, int32_t source);
  int32_t (*axis_group_set_enc_source)(int32_t group_index, int32_t source);
  int32_t (*axis_group_reset_error)(int32_t group_index);
  int32_t (*axis_group_set_error)(int32_t group_index, int32_t error_id);
  int32_t (*axis_group_set_slaved_axis_in_error)(int32_t group_index);
  int32_t (*axis_group_halt)(int32_t group_index);
  int32_t (*axis_group_axis_in_group)(int32_t group_index, int32_t axis_index);
  int32_t (*axis_group_size)(int32_t group_index);
  int32_t (*axis_group_get_traj_source_ext)(int32_t group_index);
  int32_t (*axis_group_get_any_traj_source_ext)(int32_t group_index);
  int32_t (*axis_group_set_allow_source_change_when_enabled)(int32_t group_index,
                                                             int32_t allow);
  int32_t (*axis_group_set_mr_sync)(int32_t group_index, int32_t sync);
  int32_t (*axis_group_set_mr_stop)(int32_t group_index, int32_t stop);
  int32_t (*axis_group_set_mr_cnen)(int32_t group_index, int32_t enable);
  int32_t (*axis_group_set_auto_enable)(int32_t group_index, int32_t enable);
  int32_t (*axis_group_set_auto_disable)(int32_t group_index, int32_t enable);
  int32_t (*axis_group_set_ctrl_within_deadband)(int32_t group_index,
                                                 int32_t within);
  int32_t (*axis_group_set_ignore_mr_status_check_at_disable)(int32_t group_index,
                                                              int32_t ignore);
  int32_t (*axis_group_get_any_at_forward_limit)(int32_t group_index);
  int32_t (*axis_group_get_any_at_backward_limit)(int32_t group_index);
  int32_t (*axis_group_get_any_at_limit)(int32_t group_index);
  int32_t (*axis_group_get_any_interlocked)(int32_t group_index);
  int32_t (*axis_group_set_slaved_axis_interlocked)(int32_t group_index);
  int32_t (*axis_group_get_ctrl_within_deadband)(int32_t group_index);
};

struct ecmcCppLogicApi {
  uint32_t abiVersion;
  const char* name;
  void (*setHostServices)(const struct ecmcCppLogicHostServices* services);
  int32_t (*createInstance)(void** instance);
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
