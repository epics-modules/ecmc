/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcDefinitions.h
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

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

#define TIMESPEC2NS(T) ((uint64_t)(((T).tv_sec - 946684800ULL) * \
                                   1000000000ULL) + (T).tv_nsec)

// #define MSG_TICK 0
#define MAX_MESSAGE 10000

// Memory allocation
#define ECMC_STACK_SIZE (100*1024)  /*RT thread stack size*/
#define ECMC_PRE_ALLOCATION_SIZE (10*1024*1024) /* 1MB pagefault free buffer */

#define ECMC_RT_THREAD_NAME "ecmc_rt" 

// Buffer size
#define EC_MAX_OBJECT_PATH_CHAR_LENGTH 256
#define AX_MAX_DIAG_STRING_CHAR_LENGTH 1024

// Transforms
#define MAX_TRANSFORM_INPUTS ECMC_MAX_AXES *2
#define TRANSFORM_EXPR_LINE_END_CHAR '|'

// EtherCAT
#define EC_MAX_PDOS 1024
#define EC_MAX_ENTRIES 8192
#define EC_MAX_MEM_MAPS 16
#define EC_MAX_SLAVES 512
#define EC_START_TIMEOUT_S 30

#define ECMC_OVER_UNDER_FLOW_FACTOR (0.7)

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
#define ECMC_ASYN_MAIN_PAR_UPDATE_READY_ID 11
#define ECMC_ASYN_MAIN_PAR_UPDATE_READY_NAME "ecmc.updated"
#define ECMC_ASYN_MAIN_PAR_COUNT 13

// Asyn  parameters in ec
#define ECMC_ASYN_EC_PAR_MASTER_STAT_ID 0
#define ECMC_ASYN_EC_PAR_MASTER_STAT_NAME "masterstatus"
#define ECMC_ASYN_EC_PAR_DOMAIN_STAT_ID 1
#define ECMC_ASYN_EC_PAR_DOMAIN_STAT_NAME "domainstatus"
#define ECMC_ASYN_EC_PAR_MEMMAP_COUNT_ID 2
#define ECMC_ASYN_EC_PAR_MEMMAP_COUNT_NAME "memmapcounter"
#define ECMC_ASYN_EC_PAR_DOMAIN_FAIL_COUNTER_TOT_ID 3
#define ECMC_ASYN_EC_PAR_DOMAIN_FAIL_COUNTER_TOT_NAME "domainfailcountertotal"
#define ECMC_ASYN_EC_PAR_ENTRY_COUNT_ID 4
#define ECMC_ASYN_EC_PAR_ENTRY_COUNT_NAME "entrycounter"
#define ECMC_ASYN_EC_PAR_SLAVE_COUNT_ID 5
#define ECMC_ASYN_EC_PAR_SLAVE_COUNT_NAME "slavecounter"
#define ECMC_ASYN_EC_PAR_COUNT 6

// Asyn  parameters in ec slave
#define ECMC_ASYN_EC_SLAVE_PAR_STATUS_ID 0
#define ECMC_ASYN_EC_SLAVE_PAR_STATUS_NAME "slavestatus"
#define ECMC_ASYN_EC_SLAVE_PAR_COUNT 1

// Asyn  parameters in axis
#define ECMC_ASYN_AX_ACT_POS_ID 0
#define ECMC_ASYN_AX_ACT_POS_NAME "actpos"
#define ECMC_ASYN_AX_SET_POS_ID 1
#define ECMC_ASYN_AX_SET_POS_NAME "setpos"
#define ECMC_ASYN_AX_POS_ERR_ID 2
#define ECMC_ASYN_AX_POS_ERR_NAME "poserr"
#define ECMC_ASYN_AX_DIAG_ID 3
#define ECMC_ASYN_AX_DIAG_NAME "diagnostic"
#define ECMC_ASYN_AX_STATUS_ID 4
#define ECMC_ASYN_AX_STATUS_NAME "status"
#define ECMC_ASYN_AX_PAR_COUNT 5

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
  ECMC_INTERLOCK_PLC_BWD                           = 18,
  ECMC_INTERLOCK_PLC_FWD                           = 19,
};

enum plcInterlockTypes {
  ECMC_PLC_INTERLOCK_DIR_BOTH                      = 0,
  ECMC_PLC_INTERLOCK_DIR_FWD                       = 1,
  ECMC_PLC_INTERLOCK_DIR_BWD                       = 2,
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
  ECMC_TRANSFORM_VAR_TYPE_IL_BWD  = 3,
  ECMC_TRANSFORM_VAR_TYPE_IL_FWD  = 4,
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


#define AXIS_PLC_ID_TO_PLC_ID(axisId) (axisId) + ECMC_MAX_PLCS
                                   

#define ECMC_MAIN_STR "main"
#define ECMC_THREAD_STR "thread"

#define ECMC_AX_PATH_BUFFER_SIZE 256
#define ECMC_EC_PATH_BUFFER_SIZE 256

#define ECMC_PLC_FILE_COMMENT_CHAR "#"

#define ECMC_PLC_DATA_STR "plc"
#define ECMC_PLCS_DATA_STR "plcs"
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
#define ECMC_DATA_STORAGE_STATUS_STR "status"

#define ECMC_STATIC_VAR "static."
#define ECMC_GLOBAL_VAR "global."

#define ECMC_EC_STR "ec"
#define ECMC_MEMMAP_STR "mm"
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
#define ECMC_AXIS_DATA_STR_INTERLOCK_FWD_TYPE "mon.ilockfwd"
#define ECMC_AXIS_DATA_STR_INTERLOCK_BWD_TYPE "mon.ilockbwd"
#define ECMC_AXIS_DATA_STR_ALLOW_PLC_CMD "allowplccmd"

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
  ECMC_AXIS_DATA_INTERLOCK_FWD_TYPE    = 41,
  ECMC_AXIS_DATA_INTERLOCK_BWD_TYPE    = 42,
  ECMC_AXIS_DATA_ALLOW_PLC_WRITE       = 43,
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
  ECMC_RECORDER_SOURCE_NONE            = 0,
  ECMC_RECORDER_SOURCE_ETHERCAT        = 1,
  ECMC_RECORDER_SOURCE_AXIS            = 2,
  ECMC_RECORDER_SOURCE_STATIC_VAR      = 3,
  ECMC_RECORDER_SOURCE_GLOBAL_VAR      = 4,
  ECMC_RECORDER_SOURCE_DATA_STORAGE    = 5
};

enum ecmcMotionModType {
  ECMC_MOD_MOTION_NORMAL = 0,
  ECMC_MOD_MOTION_FWD    = 1,
  ECMC_MOD_MOTION_BWD    = 2,
  ECMC_MOD_MOTION_CLOSEST= 3,
  ECMC_MOD_MOTION_MAX    = 4
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

#define BIT_SET(a, b) ((a) |= (1 << (b)))
#define BIT_CLEAR(a, b) ((a) &= ~(1 << (b)))
#define BIT_FLIP(a, b) ((a) ^= (1 << (b)))
#define BIT_CHECK(a, b) ((a) & (1 << (b)))

#define EC_DT_BIT1 "B1"
#define EC_DT_BIT2 "B2"
#define EC_DT_BIT3 "B3"
#define EC_DT_BIT4 "B4"
#define EC_DT_U8   "U8"
#define EC_DT_S8   "S8"
#define EC_DT_U16  "U16"
#define EC_DT_S16  "S16"
#define EC_DT_U32  "U32"
#define EC_DT_S32  "S32"
#define EC_DT_U64  "U64"
#define EC_DT_S64  "S64"
#define EC_DT_F32  "F32"
#define EC_DT_F64  "F64"

enum ecmcEcDataType {
  ECMC_EC_NONE  = 0,
  ECMC_EC_B1    = 1,
  ECMC_EC_B2    = 2,
  ECMC_EC_B3    = 3,
  ECMC_EC_B4    = 4,  
  ECMC_EC_U8    = 5,
  ECMC_EC_S8    = 6,
  ECMC_EC_U16   = 7,
  ECMC_EC_S16   = 8,
  ECMC_EC_U32   = 9,
  ECMC_EC_S32   = 10,
  ECMC_EC_U64   = 11,
  ECMC_EC_S64   = 12,
  ECMC_EC_F32   = 13,
  ECMC_EC_F64   = 14
};


#endif  /* ECMC_DEFINITIONS_H_ */
