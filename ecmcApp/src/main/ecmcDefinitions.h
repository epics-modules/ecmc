/*
 * ecmcDefinitions.h
 *
 *  Created on: March 14, 2016
 *      Author: anderssandstrom
 */

#define __STDC_FORMAT_MACROS  // for printf uint_64_t
#include <stdint.h>

#ifndef ECMC_DEFINITIONS_H_
#define ECMC_DEFINITIONS_H_

#define UNPACK(...) __VA_ARGS__

#define MCU_FREQUENCY 1000.0
#define MCU_NSEC_PER_SEC 1000000000
#define MCU_PERIOD_NS (int)(MCU_NSEC_PER_SEC / MCU_FREQUENCY)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * MCU_NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)

#define ECMC_MAX_AXES 64
#define ECMC_MAX_PLCS 8

// Test new conversion
#define TIMESPEC2NS(T) ((uint64_t)(((T).tv_sec - 946684800ULL) * \
                                   1000000000ULL) + (T).tv_nsec)

// #define MSG_TICK 0
#define MAX_MESSAGE 10000

// Buffer size
#define EC_MAX_OBJECT_PATH_CHAR_LENGTH 256

// Transforms
#define MAX_TRANSFORM_INPUTS ECMC_MAX_AXES *2
#define TRANSFORM_EXPR_LINE_END_CHAR '#'
#define TRANSFORM_EXPR_OUTPUT_VAR_NAME "out"
#define TRANSFORM_EXPR_COMMAND_EXECUTE_PREFIX "ex"
#define TRANSFORM_EXPR_COMMAND_ENABLE_PREFIX "en"
#define TRANSFORM_EXPR_VARIABLE_TRAJ_PREFIX "setPos"
#define TRANSFORM_EXPR_VARIABLE_ENC_PREFIX "actPos"
#define TRANSFORM_EXPR_INTERLOCK_PREFIX "ilock"

// EtherCAT
#define EC_MAX_PDOS 1024
#define EC_MAX_ENTRIES 8192
#define EC_MAX_MEM_MAPS 16
#define EC_MAX_SLAVES 512

// EC entry links
#define ECMC_EC_ENTRY_INDEX_HEALTH 0

// Axis entry links
#define ECMC_AXIS_ENTRY_INDEX_HEALTH 0

// Drive entry links
#define ECMC_DRIVEBASE_ENTRY_INDEX_CONTROL_WORD 0
#define ECMC_DRIVEBASE_ENTRY_INDEX_VELOCITY_SETPOINT 1
#define ECMC_DRIVEBASE_ENTRY_INDEX_STATUS_WORD 2
#define ECMC_DRIVEBASE_ENTRY_INDEX_BRAKE_OUTPUT 3
#define ECMC_DRIVEBASE_ENTRY_INDEX_REDUCE_TORQUE_OUTPUT 4

// Encoder drive entries
#define ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION 0
#define ECMC_ENCODER_ENTRY_INDEX_LATCH_STATUS 1
#define ECMC_ENCODER_ENTRY_INDEX_LATCH_VALUE 2
#define ECMC_ENCODER_ENTRY_INDEX_LATCH_CONTROL 3

// Monitor drive entries
#define ECMC_MON_ENTRY_INDEX_LOWLIM 0
#define ECMC_MON_ENTRY_INDEX_HIGHLIM 1
#define ECMC_MON_ENTRY_INDEX_HOMESENSOR 2
#define ECMC_MON_ENTRY_INDEX_EXTINTERLOCK 3

// Data recording
#define ECMC_MAX_DATA_RECORDERS_OBJECTS 10
#define ECMC_MAX_EVENT_OBJECTS 10
#define ECMC_MAX_DATA_STORAGE_OBJECTS 10
#define ECMC_DEFAULT_DATA_STORAGE_SIZE 1000
#define ECMC_MAX_EVENT_CONSUMERS 10
#define ECMC_MAX_COMMANDS_LISTS 10
#define ECMC_MAX_COMMANDS_IN_COMMANDS_LISTS 100

// ECMC iocsh config command
#define ECMC_IOCSH_CFG_CMD "ecmcConfigOrDie"

// Asyn  parameters in main
#define ECMC_ASYN_MAIN_PAR_LATENCY_MIN_ID 0
#define ECMC_ASYN_MAIN_PAR_LATENCY_MIN_NAME "ecmc.thread.latency.min"
#define ECMC_ASYN_MAIN_PAR_LATENCY_MAX_ID 1
#define ECMC_ASYN_MAIN_PAR_LATENCY_MAX_NAME "ecmc.thread.latency.max"
#define ECMC_ASYN_MAIN_PAR_PERIOD_MIN_ID 2
#define ECMC_ASYN_MAIN_PAR_PERIOD_MIN_NAME "ecmc.thread.period.min"
#define ECMC_ASYN_MAIN_PAR_PERIOD_MAX_ID 3
#define ECMC_ASYN_MAIN_PAR_PERIOD_MAX_NAME "ecmc.thread.period.max"
#define ECMC_ASYN_MAIN_PAR_EXECUTE_MIN_ID 4
#define ECMC_ASYN_MAIN_PAR_EXECUTE_MIN_NAME "ecmc.thread.execute.min"
#define ECMC_ASYN_MAIN_PAR_EXECUTE_MAX_ID 5
#define ECMC_ASYN_MAIN_PAR_EXECUTE_MAX_NAME "ecmc.thread.execute.max"
#define ECMC_ASYN_MAIN_PAR_SEND_MIN_ID 6
#define ECMC_ASYN_MAIN_PAR_SEND_MIN_NAME "ecmc.thread.send.min"
#define ECMC_ASYN_MAIN_PAR_SEND_MAX_ID 7
#define ECMC_ASYN_MAIN_PAR_SEND_MAX_NAME "ecmc.thread.send.max"
#define ECMC_ASYN_MAIN_PAR_APP_MODE_ID 8
#define ECMC_ASYN_MAIN_PAR_APP_MODE_NAME "ecmc.appmode"
#define ECMC_ASYN_MAIN_PAR_ERROR_ID_ID 9
#define ECMC_ASYN_MAIN_PAR_ERROR_ID_NAME "ecmc.error.id"
#define ECMC_ASYN_MAIN_PAR_ERROR_MSG_ID 10
#define ECMC_ASYN_MAIN_PAR_ERROR_MSG_NAME "ecmc.error.msg"
#define ECMC_ASYN_MAIN_PAR_RESET_ID 11
#define ECMC_ASYN_MAIN_PAR_RESET_NAME "ecmc.error.reset"
#define ECMC_ASYN_MAIN_PAR_COUNT 12

// Asyn  parameters in ec
#define ECMC_ASYN_EC_PAR_AL_STATE_ID 0
#define ECMC_ASYN_EC_PAR_AL_STATE_NAME "alstates"
#define ECMC_ASYN_EC_PAR_LINK_ID 1
#define ECMC_ASYN_EC_PAR_LINK_NAME "link"
#define ECMC_ASYN_EC_PAR_SLAVE_COUNT_ID 2
#define ECMC_ASYN_EC_PAR_SLAVE_COUNT_NAME "slavecounter"
#define ECMC_ASYN_EC_PAR_SLAVE_STAT_ID 3
#define ECMC_ASYN_EC_PAR_SLAVE_STAT_NAME "slavesstatus"
#define ECMC_ASYN_EC_PAR_MEMMAP_COUNT_ID 4
#define ECMC_ASYN_EC_PAR_MEMMAP_COUNT_NAME "memmapcounter"
#define ECMC_ASYN_EC_PAR_DOMAIN_STAT_ID 5
#define ECMC_ASYN_EC_PAR_DOMAIN_STAT_NAME "domainstatus"
#define ECMC_ASYN_EC_PAR_DOMAIN_FAIL_COUNTER_ID 6
#define ECMC_ASYN_EC_PAR_DOMAIN_FAIL_COUNTER_NAME "domainfailcounter"
#define ECMC_ASYN_EC_PAR_DOMAIN_FAIL_COUNTER__TOT_ID 7
#define ECMC_ASYN_EC_PAR_DOMAIN_FAIL_COUNTER_TOT_NAME "domainfailcountertotal"
#define ECMC_ASYN_EC_PAR_ENTRY_COUNT_ID 8
#define ECMC_ASYN_EC_PAR_ENTRY_COUNT_NAME "entrycounter"
#define ECMC_ASYN_EC_PAR_COUNT 9


// Asyn  parameters in ec slave
#define ECMC_ASYN_EC_SLAVE_PAR_ONLINE_ID 0
#define ECMC_ASYN_EC_SLAVE_PAR_ONLINE_NAME "online"
#define ECMC_ASYN_EC_SLAVE_PAR_AL_STATE_ID 1
#define ECMC_ASYN_EC_SLAVE_PAR_AL_STATE_NAME "alstates"
#define ECMC_ASYN_EC_SLAVE_PAR_OPERA_ID 2
#define ECMC_ASYN_EC_SLAVE_PAR_OPER_NAME "operational"
#define ECMC_ASYN_EC_SLAVE_PAR_ENTRY_COUNT_ID 3
#define ECMC_ASYN_EC_SLAVE_PAR_ENTRY_COUNT_NAME "entrycounter"

#define ECMC_ASYN_EC_SLAVE_PAR_COUNT 9

// Motion
enum app_mode_type {
  ECMC_MODE_CONFIG  = 0,
  ECMC_MODE_RUNTIME = 1,
  ECMC_MODE_STARTUP = 2
};

enum {
  ECMC_PRIO_LOW  = 0,
  ECMC_PRIO_HIGH = 60
};

enum axisType {
  ECMC_AXIS_TYPE_BASE       = 0,
  ECMC_AXIS_TYPE_REAL       = 1,
  ECMC_AXIS_TYPE_VIRTUAL    = 2,
  ECMC_AXIS_TYPE_TRAJECTORY = 3,
  ECMC_AXIS_TYPE_ENCODER    = 4,
};

enum operationMode {
  ECMC_MODE_OP_AUTO = 0,
  ECMC_MODE_OP_MAN  = 1,
};

enum motionCommandTypes {
  ECMC_CMD_NOCMD      = -1,
  ECMC_CMD_JOG        = 0,
  ECMC_CMD_MOVEVEL    = 1,
  ECMC_CMD_MOVEREL    = 2,
  ECMC_CMD_MOVEABS    = 3,
  ECMC_CMD_MOVEMODULO = 4,   // NOT IMPLEMENTED
  ECMC_CMD_HOMING     = 10,  // PARTLY IMPLEMENTED
  // NOT IMPLEMENTED (implemented in another way..)
  ECMC_CMD_SUPERIMP   = 20,  // NOT IMPLEMENTED
  // NOT IMPLEMENTED (implemented in another way..)
  ECMC_CMD_GEAR       = 30,
};

enum motionDirection {
  ECMC_DIR_FORWARD    = 0,
  ECMC_DIR_BACKWARD   = 1,
  ECMC_DIR_STANDSTILL = 2,
};

enum motionMode {
  ECMC_MOVE_MODE_POS = 0,
  ECMC_MOVE_MODE_VEL = 1,
};

enum dataSource {
  ECMC_DATA_SOURCE_INTERNAL           = 0,
  ECMC_DATA_SOURCE_EXTERNALENCODER    = 1,
  ECMC_DATA_SOURCE_EXTERNALTRAJECTORY = 2
};

enum coordSystMode {
  ECMC_COORD_ABS = 0,
  ECMC_COORD_REL = 1,
};

enum stopMode {
  ECMC_STOP_MODE_EMERGENCY = 0,
  ECMC_STOP_MODE_NORMAL    = 1,
  ECMC_STOP_MODE_RUN       = 2,
};

enum interlockTypes {
  ECMC_INTERLOCK_NONE                              = 0,
  ECMC_INTERLOCK_SOFT_BWD                          = 1,
  ECMC_INTERLOCK_SOFT_FWD                          = 2,
  ECMC_INTERLOCK_HARD_BWD                          = 3,
  ECMC_INTERLOCK_HARD_FWD                          = 4,
  ECMC_INTERLOCK_NO_EXECUTE                        = 5,
  ECMC_INTERLOCK_POSITION_LAG                      = 6,
  ECMC_INTERLOCK_BOTH_LIMITS                       = 7,
  ECMC_INTERLOCK_EXTERNAL                          = 8,
  ECMC_INTERLOCK_TRANSFORM                         = 9,
  ECMC_INTERLOCK_MAX_SPEED                         = 10,
  ECMC_INTERLOCK_CONT_HIGH_LIMIT                   = 11,
  ECMC_INTERLOCK_CONT_OUT_INCREASE_AT_LIMIT_SWITCH = 12,
  ECMC_INTERLOCK_AXIS_ERROR_STATE                  = 13,
  ECMC_INTERLOCK_UNEXPECTED_LIMIT_SWITCH_BEHAVIOUR = 14,
  ECMC_INTERLOCK_VELOCITY_DIFF                     = 15,
  ECMC_INTERLOCK_ETHERCAT_MASTER_NOT_OK            = 16,
  ECMC_INTERLOCK_PLC_NORMAL                        = 17,
};

enum encoderType {
  ECMC_ENCODER_TYPE_INCREMENTAL = 0,
  ECMC_ENCODER_TYPE_ABSOLUTE    = 1,
};

enum commandType {
  ECMC_CMD_TYPE_EXECUTE = 0,
  ECMC_CMD_TYPE_ENABLE  = 1,
};

enum transformVariableType {
  ECMC_TRANSFORM_VAR_TYPE_TRAJ = 0,
  ECMC_TRANSFORM_VAR_TYPE_ENC  = 1,
  ECMC_TRANSFORM_VAR_TYPE_IL   = 2,
};

enum eventType {
  ECMC_SAMPLED        = 0,
  ECMC_EDGE_TRIGGERED = 1,
};

enum triggerEdgeType {
  ECMC_POSITIVE_EDGE = 0,
  ECMC_NEGATIVE_EDGE = 1,
  ECMC_ON_CHANGE     = 2,
};

// Object types
enum mainObjectType {
  ECMC_OBJ_INVALID    = 0,
  ECMC_OBJ_AXIS       = 1,
  ECMC_OBJ_EC         = 2,
  ECMC_OBJ_DS         = 3,
  ECMC_OBJ_MAIN       = 4,
  ECMC_OBJ_THREAD     = 5
};

// Object types
enum axisSubObjectType {
  ECMC_AX_SUB_OBJ_INVALID    = 0,
  ECMC_AX_SUB_OBJ_DRIVE      = 1,
  ECMC_AX_SUB_OBJ_ENCODER    = 2,
  ECMC_AX_SUB_OBJ_MONITOR    = 3,
  ECMC_AX_SUB_OBJ_CONTROLLER = 4,
  ECMC_AX_SUB_OBJ_TRAJECTORY = 5,
  ECMC_AX_SUB_OBJ_MAIN       = 6,
};

#define ECMC_MAIN_STR "main"
#define ECMC_THREAD_STR "thread"

#define ECMC_AX_PATH_BUFFER_SIZE 256
#define ECMC_EC_PATH_BUFFER_SIZE 256

#define ECMC_PLC_FILE_COMMENT_CHAR "#"

#define ECMC_PLC_DATA_STR "plc"
#define ECMC_PLC_ENABLE_DATA_STR "enable"
#define ECMC_PLC_ERROR_DATA_STR "error"
#define ECMC_PLC_SCAN_TIME_DATA_STR "scantime"
#define ECMC_PLC_FIRST_SCAN_STR "firstscan"

#define ECMC_PLC_DATA_STORAGE_STR "ds"
#define ECMC_DATA_STORAGE_DATA_APPEND_STR "append"
#define ECMC_DATA_STORAGE_DATA_INDEX_STR "index"
#define ECMC_DATA_STORAGE_DATA_ERROR_STR "error"
#define ECMC_DATA_STORAGE_DATA_SIZE_STR "size"
#define ECMC_DATA_STORAGE_DATA_DATA_STR "data"
#define ECMC_DATA_STORAGE_DATA_CLEAR_STR "clear"
#define ECMC_DATA_STORAGE_DATA_FULL_STR "full"

#define ECMC_STATIC_VAR "static."
#define ECMC_GLOBAL_VAR "global."

#define ECMC_EC_STR "ec"
#define ECMC_SLAVE_CHAR "s"
#define ECMC_DUMMY_SLAVE_STR "ds"
#define ECMC_EC_HEALTH_STR "health"
#define ECMC_AX_STR "ax"
#define ECMC_AX_HEALTH_STR "health"
#define ECMC_DRV_STR "drv"
#define ECMC_DRV_ENABLE_STR "control"
#define ECMC_DRV_VELOCITY_STR "velocity"
#define ECMC_DRV_ENABLED_STR "status"
#define ECMC_DRV_BREAK_STR "brake"
#define ECMC_DRV_REDUCETORQUE_STR "reducetorque"

#define ECMC_PLC_VAR_FORMAT "%[0-9a-zA-Z._]"
#define ECMC_PLC_EC_ALIAS_FORMAT "%[0-9a-zA-Z_]"

#define ECMC_ENC_STR "enc"
#define ECMC_ENC_ACTPOS_STR "actpos"
#define ECMC_ENC_LATCHPOS_STR "latchpos"
#define ECMC_ENC_LATCH_STATUS_STR "latchstatus"
#define ECMC_ENC_LATCH_CONTROL_STR "latchcontrol"

#define ECMC_MON_STR "mon"
#define ECMC_MON_LOWLIM_STR "lowlim"
#define ECMC_MON_HIGHLIM_STR "highlim"
#define ECMC_MON_HOMESENSOR_STR "homesensor"
#define ECMC_MON_EXTINTERLOCK_STR "extinterlock"
#define ECMC_TRAJ_STR "traj"
#define ECMC_CNTRL_STR "cntrl"

#define ECMC_AXIS_DATA_STR_AXIS_ID "id"
#define ECMC_AXIS_DATA_STR_POS_SET "traj.setpos"
#define ECMC_AXIS_DATA_STR_POS_ACT "enc.actpos"
#define ECMC_AXIS_DATA_STR_CNTRL_ERROR "cntrl.error"
#define ECMC_AXIS_DATA_STR_POS_TARGET "traj.targetpos"
#define ECMC_AXIS_DATA_STR_POS_ERROR "cntrl.poserror"
#define ECMC_AXIS_DATA_STR_POS_RAW "enc.rawpos"
#define ECMC_AXIS_DATA_STR_CNTRL_OUT "cntrl.output"
#define ECMC_AXIS_DATA_STR_VEL_SET "traj.setvel"
#define ECMC_AXIS_DATA_STR_VEL_TARGET_SET "traj.targetvel"
#define ECMC_AXIS_DATA_STR_ACC_TARGET_SET "traj.targetacc"
#define ECMC_AXIS_DATA_STR_DEC_TARGET_SET "traj.targetdec"
#define ECMC_AXIS_DATA_STR_VEL_ACT "enc.actvel"
#define ECMC_AXIS_DATA_STR_VEL_SET_FF_RAW "traj.setvelffraw"
#define ECMC_AXIS_DATA_STR_VEL_SET_RAW "drv.setvelraw"
#define ECMC_AXIS_DATA_STR_CYCLE_COUNTER "counter"
#define ECMC_AXIS_DATA_STR_ERROR "error"
#define ECMC_AXIS_DATA_STR_COMMAND "traj.command"
#define ECMC_AXIS_DATA_STR_CMD_DATA "traj.cmddata"
#define ECMC_AXIS_DATA_STR_SEQ_STATE "seq.state"
#define ECMC_AXIS_DATA_STR_INTERLOCK_TYPE "mon.ilock"
#define ECMC_AXIS_DATA_STR_TRAJ_SOURCE "traj.source"
#define ECMC_AXIS_DATA_STR_ENC_SOURCE "enc.source"
#define ECMC_AXIS_DATA_STR_ENABLE "drv.enable"
#define ECMC_AXIS_DATA_STR_ENABLED "drv.enabled"
#define ECMC_AXIS_DATA_STR_EXECUTE "traj.execute"
#define ECMC_AXIS_DATA_STR_BUSY "traj.busy"
#define ECMC_AXIS_DATA_STR_AT_TARGET "mon.attarget"
#define ECMC_AXIS_DATA_STR_HOMED "enc.homed"
#define ECMC_AXIS_DATA_STR_LIMIT_BWD "mon.lowlim"
#define ECMC_AXIS_DATA_STR_LIMIT_FWD "mon.highlim"
#define ECMC_AXIS_DATA_STR_SOFT_LIMIT_BWD "mon.lowsoftlim"
#define ECMC_AXIS_DATA_STR_SOFT_LIMIT_FWD "mon.highsoftlim"
#define ECMC_AXIS_DATA_STR_SOFT_LIMIT_BWD_ENABLE "mon.lowsoftlimenable"
#define ECMC_AXIS_DATA_STR_SOFT_LIMIT_FWD_ENABLE "mon.highsoftlimenable"
#define ECMC_AXIS_DATA_STR_HOME_SWITCH "mon.homesensor"
#define ECMC_AXIS_DATA_STR_RESET "reset"
#define ECMC_AXIS_DATA_STR_TRAJ_DIRECTION "traj.dir"
#define ECMC_AXIS_DATA_STR_ENC_HOMEPOS "enc.homepos"
#define ECMC_AXIS_DATA_STR_BLOCK_COM "blockcom"

enum ecmcAxisDataType {
  ECMC_AXIS_DATA_NONE                  = 0,
  ECMC_AXIS_DATA_AXIS_ID               = 1,
  ECMC_AXIS_DATA_POS_SET               = 2,
  ECMC_AXIS_DATA_POS_ACT               = 3,
  ECMC_AXIS_DATA_CNTRL_ERROR           = 4,
  ECMC_AXIS_DATA_POS_TARGET            = 5,
  ECMC_AXIS_DATA_POS_ERROR             = 6,
  ECMC_AXIS_DATA_POS_RAW               = 7,
  ECMC_AXIS_DATA_CNTRL_OUT             = 8,
  ECMC_AXIS_DATA_VEL_SET               = 9,
  ECMC_AXIS_DATA_VEL_ACT               = 10,
  ECMC_AXIS_DATA_VEL_SET_FF_RAW        = 11,
  ECMC_AXIS_DATA_VEL_SET_RAW           = 12,
  ECMC_AXIS_DATA_CYCLE_COUNTER         = 13,
  ECMC_AXIS_DATA_ERROR                 = 14,
  ECMC_AXIS_DATA_COMMAND               = 15,
  ECMC_AXIS_DATA_CMD_DATA              = 16,
  ECMC_AXIS_DATA_SEQ_STATE             = 17,
  ECMC_AXIS_DATA_INTERLOCK_TYPE        = 18,
  ECMC_AXIS_DATA_TRAJ_SOURCE           = 19,
  ECMC_AXIS_DATA_ENC_SOURCE            = 20,
  ECMC_AXIS_DATA_ENABLE                = 21,
  ECMC_AXIS_DATA_ENABLED               = 22,
  ECMC_AXIS_DATA_EXECUTE               = 23,
  ECMC_AXIS_DATA_BUSY                  = 24,
  ECMC_AXIS_DATA_AT_TARGET             = 25,
  ECMC_AXIS_DATA_HOMED                 = 26,
  ECMC_AXIS_DATA_LIMIT_BWD             = 27,
  ECMC_AXIS_DATA_LIMIT_FWD             = 28,
  ECMC_AXIS_DATA_HOME_SWITCH           = 29,
  ECMC_AXIS_DATA_RESET                 = 30,
  ECMC_AXIS_DATA_VEL_TARGET_SET        = 31,
  ECMC_AXIS_DATA_ACC_TARGET_SET        = 32,
  ECMC_AXIS_DATA_DEC_TARGET_SET        = 33,
  ECMC_AXIS_DATA_SOFT_LIMIT_BWD        = 34,
  ECMC_AXIS_DATA_SOFT_LIMIT_FWD        = 35,
  ECMC_AXIS_DATA_SOFT_LIMIT_BWD_ENABLE = 36,
  ECMC_AXIS_DATA_SOFT_LIMIT_FWD_ENABLE = 37,
  ECMC_AXIS_DATA_TRAJ_DIRECTION        = 38,
  ECMC_AXIS_DATA_ENC_HOMEPOS           = 39,
  ECMC_AXIS_DATA_BLOCK_COM             = 40,
};

enum ecmcDataStorageType {
  ECMC_DATA_STORAGE_DATA_NONE   = 0,
  ECMC_DATA_STORAGE_DATA_APPEND = 1,
  ECMC_DATA_STORAGE_DATA_SIZE   = 2,
  ECMC_DATA_STORAGE_DATA_INDEX  = 3,
  ECMC_DATA_STORAGE_DATA_ERROR  = 4,
  ECMC_DATA_STORAGE_DATA_DATA   = 5,
  ECMC_DATA_STORAGE_DATA_CLEAR  = 6,
  ECMC_DATA_STORAGE_DATA_FULL   = 7,
};

enum ecmcDataSourceType {
  ECMC_RECORDER_SOURCE_NONE         = 0,
  ECMC_RECORDER_SOURCE_ETHERCAT     = 1,
  ECMC_RECORDER_SOURCE_AXIS         = 2,
  ECMC_RECORDER_SOURCE_STATIC_VAR   = 3,
  ECMC_RECORDER_SOURCE_GLOBAL_VAR   = 4,
  ECMC_RECORDER_SOURCE_DATA_STORAGE = 5,
};

typedef struct ecmcMainThreadDiag{
  uint32_t period_ns;
  uint32_t exec_ns;
  uint32_t latency_ns;
  uint32_t sendperiod_ns;
  uint32_t latency_min_ns;
  uint32_t latency_max_ns;
  uint32_t period_min_ns;
  uint32_t period_max_ns;
  uint32_t exec_min_ns;
  uint32_t exec_max_ns;
  uint32_t send_min_ns;
  uint32_t send_max_ns;
}ecmcMainThreadDiag;

  
#endif  /* ECMC_DEFINITIONS_H_ */
