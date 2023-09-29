/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcErrorsList.h
*
*  Created on: Jan 29, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCERRORSLIST_H_
#define ECMCERRORSLIST_H_

// HW_MOTOR
#define ERROR_MAIN_APP_MODE_NOT_SUPPORTED 0x20000
#define ERROR_MAIN_DEMO_EC_ACITVATE_FAILED 0x20001
#define ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE 0x20002
#define ERROR_MAIN_AXIS_OBJECT_NULL 0x20004
#define ERROR_MAIN_ENCODER_INDEX_OUT_OF_RANGE 0x20005
#define ERROR_MAIN_ENCODER_OBJECT_NULL 0x20006
#define ERROR_MAIN_CONTROLLER_INDEX_OUT_OF_RANGE 0x20007
#define ERROR_MAIN_CONTROLLER_OBJECT_NULL 0x20008
#define ERROR_MAIN_TRAJECTORY_INDEX_OUT_OF_RANGE 0x20009
#define ERROR_MAIN_TRAJECTORY_OBJECT_NULL 0x2000A
#define ERROR_MAIN_DRIVE_INDEX_OUT_OF_RANGE 0x2000B
#define ERROR_MAIN_DRIVE_OBJECT_NULL 0x2000C
#define ERROR_MAIN_MONITOR_INDEX_OUT_OF_RANGE 0x2000D
#define ERROR_MAIN_MONITOR_OBJECT_NULL 0x2000E
#define ERROR_MAIN_AXIS_TRAJ_OBJECT_NULL 0x2000F
#define ERROR_MAIN_EC_NOT_INITIALIZED 0x20010
#define ERROR_MAIN_EC_SLAVE_NULL 0x20011
#define ERROR_MAIN_EC_SM_NULL 0x20012
#define ERROR_MAIN_EC_PDO_NULL 0x20013
#define ERROR_MAIN_EC_ENTRY_NULL 0x20014
#define ERROR_MAIN_ENCODER_ENTRY_INDEX_OUT_OF_RANGE 0x20015
#define ERROR_MAIN_DRIVE_ENTRY_INDEX_OUT_OF_RANGE 0x20016
#define ERROR_MAIN_MONITOR_ENTRY_INDEX_OUT_OF_RANGE 0x20017
#define ERROR_MAIN_DIAG_AXIS_INDEX_OUT_OF_RANGE 0x20018
#define ERROR_MAIN_DIAG_AXIS_FREQ_OUT_OF_RANGE 0x20019
#define ERROR_MAIN_APP_MODE_ALREADY_RUNTIME 0x2001A
#define ERROR_MAIN_EC_ACTIVATE_FAILED 0x2001B
#define ERROR_MAIN_ENC_SET_SCALE_FAIL_DRV_ENABLED 0x2001C
#define ERROR_MAIN_AXIS_ERROR_EXECUTE_INTERLOCKED 0x2001D
#define ERROR_MAIN_VIRT_AXIS_FUNCTION_NOT_SUPPORTED 0x2001E
#define ERROR_MAIN_AXIS_TYPE_UNKNOWN 0x2001F
#define ERROR_MAIN_SEQUENCE_OBJECT_NULL 0x20020
#define ERROR_MAIN_TRAJ_TRANSFORM_OBJECT_NULL 0x20021
#define ERROR_MAIN_ENC_TRANSFORM_OBJECT_NULL 0x20022
#define ERROR_MAIN_EXTERNAL_DATA_SOURCE_INVALID 0x20023
#define ERROR_MAIN_TRANSFORM_OUTPUT_VAR_MISSING 0x20024
#define ERROR_MAIN_DATA_RECORDER_INDEX_OUT_OF_RANGE 0x20025
#define ERROR_MAIN_DATA_RECORDER_NULL 0x20026
#define ERROR_MAIN_DATA_STORAGE_INDEX_OUT_OF_RANGE 0x20027
#define ERROR_MAIN_DATA_STORAGE_INVALID_SIZE 0x20028
#define ERROR_MAIN_DATA_STORAGE_NULL 0x20029
#define ERROR_MAIN_EVENT_INDEX_OUT_OF_RANGE 0x2002A
#define ERROR_MAIN_EVENT_NULL 0x2002B
#define ERROR_MAIN_AXIS_NOT_ENABLED 0x2002C
#define ERROR_MAIN_MASTER_SLAVE_IF_NULL 0x2002D
#define ERROR_MAIN_PRINT_TO_BUFFER_FAIL 0x2002E
#define ERROR_MAIN_MEM_MAP_NULL 0x2002F
#define ERROR_MAIN_AXIS_ASYN_PAR_FORMAT_ERROR 0x20030
#define ERROR_MAIN_ASYN_PORT_DRIVER_NULL 0x20031
#define ERROR_MAIN_EC_MASTER_NULL 0x20032
#define ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR 0x20033
#define ERROR_MAIN_ECMC_LINK_INVALID 0x20034
#define ERROR_MAIN_PLC_INDEX_OUT_OF_RANGE 0x20035
#define ERROR_MAIN_PLC_OBJECT_NULL 0x20036
#define ERROR_MAIN_PLCS_NULL 0x20037
#define ERROR_MAIN_PLCS_SKIPCYCLES_INVALID 0x20038
#define ERROR_MAIN_AXIS_EXTERNAL_COM_DISABLED 0x20039
#define ERROR_MAIN_EXCEPTION 0x2003A
#define ERROR_MAIN_PARSER_UNKOWN_CMD 0x2003B
#define ERROR_MAIN_PARSER_BUFFER_NULL 0x2003C
#define ERROR_MAIN_PARSER_CMD_TO_LONG 0x2003D
#define ERROR_MAIN_PARSER_INVALID_ADS_FORMAT 0x2003E
#define ERROR_MAIN_PARSER_INVALID_ADS_PORT 0x2003F
#define ERROR_MAIN_PARSER_INVALID_ADS_LEN 0x20040
#define ERROR_MAIN_PARSER_UNKNOWN_ADS_CMD 0x20041
#define ERROR_MAIN_PARSER_INVALID_ADS_OFFSET 0x20042
#define ERROR_MAIN_PARSER_INVALID_ADS_TYPE 0x20043
#define ERROR_MAIN_ASYN_CREATE_PARAM_FAIL 0x20044
#define ERROR_MAIN_MLOCKALL_FAIL 0x20045
#define ERROR_MAIN_EC_NULL 0x20046
#define ERROR_MAIN_EC_SDO_VERIFICATION_FAIL 0x20047
#define ERROR_MAIN_OBSOLETE_COMMAND 0x20048
#define ERROR_MAIN_FILTER_INVALID_SIZE 0x20049
#define ERROR_MAIN_EC_INDEX_OUT_OF_RANGE 0x2004A
#define ERROR_MAIN_RT_MUTEX_NULL 0x2004B
#define ERROR_MAIN_EC_TIMEOUT_OUT_OF_RANGE 0x2004C
#define ERROR_MAIN_SAMPLE_RATE_OUT_OF_RANGE 0x2004D
#define ERROR_MAIN_SAMPLE_RATE_CHANGE_NOT_ALLOWED 0x2004E
#define ERROR_MAIN_PLUGIN_LIST_FULL 0x2004F
#define ERROR_MAIN_PLUGIN_OBJECT_NULL 0x20050
#define ERROR_MAIN_PLUGIN_INDEX_OUT_OF_RANGE 0x20051
#define ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL 0x20052
#define ERROR_MAIN_AXIS_COM_BLOCKED 0x20053
#define ERROR_MAIN_EC_SCAN_TIMEOUT 0x20054
#define ERROR_MAIN_AXIS_ALREADY_CREATED 0x20055

// Other errors moved heer since used in many objects
#define ERROR_EC_MAIN_ENTRY_NULL 0x26006

#define ERROR_EC_MAIN_DOMAIN_DATA_FAILED 0x2600C
#define ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW 0x26024
#define ERROR_EC_ASYN_PORT_OBJ_NULL 0x26015
#define ERROR_EC_ASYN_PORT_CREATE_PARAM_FAIL 0x26016
#define ERROR_EC_ASYN_SKIP_CYCLES_INVALID 0x26017


#endif  /* ECMCERRORSLIST_H_ */
