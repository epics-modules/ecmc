/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcError.cpp
*
*  Created on: Mar 16, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcError.h"

ecmcError::ecmcError() {
  initVars();
}

ecmcError::ecmcError(int *errorPtr, int *warningPtr) {
  initVars();
  errorPtr_   = errorPtr;
  warningPtr_ = warningPtr;
}

ecmcError::~ecmcError() {}

void ecmcError::initVars() {
  errorId_        = 0;
  error_          = 0;
  warningId_      = 0;
  warning_        = 0;
  errorPathValid_ = false;
  currSeverity_   = ECMC_SEVERITY_NONE;
  memset(&errorPath_, 0, sizeof(errorPath_));
  warningPtr_ = NULL;
  errorPtr_   = NULL;
}

int ecmcError::setErrorID(const char *fileName,
                          const char *functionName,
                          int         lineNumber,
                          int         errorID) {
  if (errorID != errorId_) {
    if (errorPathValid_) {
      LOGERR("%s/%s:%d: %s=%s;\n",
             fileName,
             functionName,
             lineNumber,
             errorPath_,
             convertErrorIdToString(errorID));
    } else {
      LOGERR("%s/%s:%d: %s (0x%x).\n",
             fileName,
             functionName,
             lineNumber,
             convertErrorIdToString(errorID),
             errorID);
    }
  }

  return setErrorID(errorID);
}

int ecmcError::setErrorID(const char       *fileName,
                          const char       *functionName,
                          int               lineNumber,
                          int               errorID,
                          ecmcAlarmSeverity severity) {
  if ((errorID != errorId_) && (severity > currSeverity_)) {
    LOGERR("%s/%s:%d: %s (0x%x).\n",
           fileName,
           functionName,
           lineNumber,
           convertErrorIdToString(errorID),
           errorID);
  }

  return setErrorID(errorID, severity);
}

int ecmcError::setErrorID(int errorID) {
  if (errorID) {
    error_ = true;
  } else {
    error_ = false;
  }
  errorId_ = errorID;
  
  // Also write to "external" pointer
  if (errorPtr_) {
    *errorPtr_ = errorID;
  }

  return errorId_;
}

int ecmcError::setErrorID(int errorID, ecmcAlarmSeverity severity) {
  if (severity <= currSeverity_) {
    return errorId_;
  }
  currSeverity_ = severity;

  if (errorID) {
    error_ = true;
  } else {
    error_ = false;
  }
  errorId_ = errorID;
  
  // Also write to "external" pointer
  if (errorPtr_) {
    *errorPtr_ = errorID;
  }

  return errorId_;
}

void ecmcError::setError(bool error) {
  error_ = error;
}

void ecmcError::errorReset() {
  error_ = false;
  setErrorID(__FILE__, __FUNCTION__, __LINE__, 0);
  currSeverity_ = ECMC_SEVERITY_NONE;
  setWarningID(0);

  if (warningPtr_) {
    *warningPtr_ = 0;
  }

  if (errorPtr_) {
    *errorPtr_ = 0;
  }
}

bool ecmcError::getError() {
  return error_;
}

int ecmcError::getErrorID() {
  return errorId_;
}

ecmcAlarmSeverity ecmcError::getSeverity() {
  return currSeverity_;
}

int ecmcError::setWarningID(int warningId) {
  if (warningId != warningId_) {
    LOGINFO12("%s (0x%x).\n",
              convertWarningIdToString(warningId),
              warningId);
  }

  if (warningId) {
    warning_ = true;
  } else {
    warning_ = false;
  }
  warningId_ = warningId;

  // Also write to "external" pointer

  if (warningPtr_) {
    *warningPtr_ = warningId;
  }

  return warningId_;
}

int ecmcError::getWarningID() {
  return warningId_;
}

void ecmcError::setExternalPtrs(int *errorPtr, int *warningPtr) {
  warningPtr_ = warningPtr;
  errorPtr_   = errorPtr;
}

const char * ecmcError::convertWarningIdToString(int warningId) {
  switch (warningId) {
  case 0:   // GENERAL
    return "NO_WARNING";

    break;

  case 0x114300: // Axis
    return "WARNING_AXIS_ASYN_CMD_WHILE_BUSY";

    break;

  case 0x114301: // Axis
    return "WARNING_AXIS_ASYN_CMD_DATA_ERROR";

    break;

  case 0x114417: // Encoder
    return "WARNING_ENC_NOT_READY";

    break;

  case 0x114C00: // Monitor
    return "WARNING_MON_SOFT_LIMIT_FWD_INTERLOCK";

    break;

  case 0x114C01: // Monitor
    return "WARNING_MON_SOFT_LIMIT_BWD_INTERLOCK";

    break;

  case 0x114C02: // Monitor
    return "WARNING_MON_HARD_LIMIT_FWD_INTERLOCK";

    break;

  case 0x114C03: // Monitor
    return "WARNING_MON_HARD_LIMIT_BWD_INTERLOCK";

    break;

  case 0x114600: // Drive
    return "WARNING_DRV_WARNING_BIT_HIGH";

    break;

  case 0x114601: // Drive
    return "WARNING_DRV_ENABLED_LOST";

    break;

  case 0x114D00: // Seq
    return "WARNING_SEQ_SETPOINT_SOFTLIM_FWD_VILOATION";

    break;

  case 0x114D01: // Seq
    return "WARNING_SEQ_SETPOINT_SOFTLIM_BWD_VILOATION";

    break;
  }

  return "NO_WARNING";
}

const char * ecmcError::convertErrorIdToString(int errorId) {
  switch (errorId) {
  case 0:   // GENERAL
    return "NO_ERROR";

    break;

  case 0x14300:  // AXIS
    return "ERROR_AXIS_OBJECTS_NULL_OR_EC_INIT_FAIL";

    break;

  case 0x14301:
    return "ERROR_AXIS_DRV_OBJECT_NULL";

    break;

  case 0x14302:
    return "ERROR_AXIS_ENC_OBJECT_NULL";

    break;

  case 0x14303:
    return "ERROR_AXIS_MON_OBJECT_NULL";

    break;

  case 0x14304:
    return "ERROR_AXIS_TRAJ_OBJECT_NULL";

    break;

  case 0x14305:
    return "ERROR_AXIS_CNTRL_OBJECT_NULL";

    break;

  case 0x14306:
    return "ERROR_AXIS_SEQ_ERROR_WRONG_SENSOR_EDGE";

    break;

  case 0x14307:
    return "ERROR_AXIS_UNDEFINED_TYPE";

    break;

  case 0x14308:
    return "ERROR_AXIS_FORWARD_TRANSFORM_NULL";

    break;

  case 0x14309:
    return "ERROR_AXIS_INVERSE_TRANSFORM_NULL";

    break;

  case 0x1430A:
    return "ERROR_AXIS_TRANSFORM_ERROR_OR_NOT_COMPILED";

    break;

  case 0x1430B:
    return "ERROR_AXIS_FUNCTION_NOT_SUPPRTED";

    break;

  case 0x1430C:
    return "ERROR_AXIS_MASTER_AXIS_OBJECT_NULL";

    break;

  case 0x1430D:
    return "ERROR_AXIS_MASTER_AXIS_ENCODER_NULL";

    break;

  case 0x1430E:
    return "ERROR_AXIS_MASTER_AXIS_TRAJECTORY_NULL";

    break;

  case 0x1430F:
    return "ERROR_AXIS_MASTER_AXIS_TRANSFORM_NULL";

    break;

  case 0x14310:
    return "ERROR_AXIS_SOURCE_TYPE_NOT_DEFINED";

    break;

  case 0x14311:
    return "ERROR_AXIS_CMD_NOT_ALLOWED_WHEN_ENABLED";

    break;

  case 0x14312:
    return "ERROR_AXIS_CONFIGURED_COUNT_ZERO";

    break;

  case 0x14313:
    return "ERROR_AXIS_CASCADED_AXIS_INDEX_OUT_OF_RANGE";

    break;

  case 0x14314:
    return "ERROR_AXIS_INDEX_OUT_OF_RANGE";

    break;

  case 0x14315:
    return "ERROR_AXIS_HARDWARE_STATUS_NOT_OK";

    break;

  case 0x14316:
    return "ERROR_AXIS_NOT_ENABLED";

    break;

  case 0x14317:
    return "ERROR_AXIS_AMPLIFIER_ENABLED_LOST";

    break;

  case 0x14318:
    return "ERROR_AXIS_SEQ_OBJECT_NULL";

    break;

  case 0x14319:
    return "ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED";

    break;

  case 0x1431A:
    return "ERROR_AXIS_ASSIGN_EXT_INTERFACE_TO_SEQ_FAILED";

    break;

  case 0x1431B:
    return "ERROR_AXIS_DATA_POINTER_NULL";

    break;

  case 0x1431C:
    return "ERROR_AXIS_BUSY";

    break;

  case 0x1431D:
    return "ERROR_AXIS_TRAJ_MASTER_SLAVE_IF_NULL";

    break;

  case 0x1431E:
    return "ERROR_AXIS_ENC_MASTER_SLAVE_IF_NULL";

    break;

  case 0x1431F:
    return "ERROR_AXIS_ASYN_PORT_OBJ_NULL";

    break;

  case 0x14320:
    return "ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL";

    break;

  case 0x14321:
    return "ERROR_AXIS_PRINT_TO_BUFFER_FAIL";

    break;

  case 0x14322:
    return "ERROR_AXIS_MODULO_OUT_OF_RANGE";

    break;

  case 0x14323:
    return "ERROR_AXIS_MODULO_TYPE_OUT_OF_RANGE";

    break;

  case 0x14324:
    return "ERROR_AXIS_FILTER_OBJECT_NULL";

    break;

  case 0x14325:
    return "ERROR_AXIS_PLC_OBJECT_NULL";

    break;

  case 0x14326:
    return "ERROR_AXIS_ENC_COUNT_OUT_OF_RANGE";

    break;

  case 0x14327:
    return "ERROR_AXIS_PRIMARY_ENC_ID_OUT_OF_RANGE";

    break;

  case 0x14328:
    return "ERROR_AXIS_SWITCH_PRIMARY_ENC_NOT_ALLOWED_WHEN_BUSY";

    break;

  case 0x14600:   // DRIVE
    return "ERROR_DRV_DRIVE_INTERLOCKED";

    break;

  case 0x14601:
    return "ERROR_DRV_ASSIGN_ENTRY_FAILED";

    break;

  case 0x14602:
    return "ERROR_DRV_SCALE_DENOM_ZERO";

    break;

  case 0x14603:
    return "ERROR_DRV_ENABLE_ENTRY_NULL";

    break;

  case 0x14604:
    return "ERROR_DRV_VEL_SET_ENTRY_NULL";

    break;

  case 0x14605:
    return "ERROR_DRV_ENABLED_ENTRY_NULL";

    break;

  case 0x14606:
    return "ERROR_DRV_ENABLED_READ_ENTRY_FAIL";

    break;

  case 0x14607:
    return "ERROR_DRV_BRAKE_ENTRY_NULL";

    break;

  case 0x14608:
    return "ERROR_DRV_REDUCE_TORQUE_ENTRY_NULL";

    break;

  case 0x14609:
    return "ERROR_DRV_COMMAND_NOT_ALLOWED_IN_AUTO_MODE";

    break;

  case 0x1460A:
    return "ERROR_DRV_BRAKE_OPEN_DELAY_TIME_INVALID";

    break;

  case 0x1460B:
    return "ERROR_DRV_BRAKE_CLOSE_AHEAD_TIME_INVALID";

    break;

  case 0x1460C:
    return "ERROR_DRV_ASYN_PORT_OBJ_NULL";

    break;

  case 0x1460D:
    return "ERROR_DRV_ASYN_PRINT_TO_BUFFER_FAIL";

    break;

  case 0x1460E:
    return "ERROR_DRV_HW_ALARM_0";

    break;

  case 0x1460F:
    return "ERROR_DRV_HW_ALARM_1";

    break;

  case 0x14610:
    return "ERROR_DRV_HW_ALARM_2";

    break;

  case 0x14611:
    return "ERROR_DRV_WARNING_READ_ENTRY_FAIL";

    break;

  case 0x14612:
    return "ERROR_DRV_ALARM_READ_ENTRY_FAIL";

    break;

  case 0x14613:
    return "ERROR_DRV_STATE_MACHINE_TIME_OUT";

    break;

  case 0x14650:
    return "ERROR_DRV_DS402_CONTROL_WORD_BIT_COUNT_ERROR";

    break;

  case 0x14651:
    return "ERROR_DRV_DS402_STATUS_WORD_BIT_COUNT_ERROR";

    break;

  case 0x14652:
    return "ERROR_DRV_DS402_STATE_MACHINE_TIME_OUT";

    break;

  case 0x14653:
    return "ERROR_DRV_DS402_CONTROL_WORD_START_BIT_ERROR";

    break;

  case 0x14654:
    return "ERROR_DRV_DS402_STATUS_WORD_START_BIT_ERROR";

    break;

  case 0x14655:
    return "ERROR_DRV_DS402_FAULT_STATE";

    break;

  case 0x14400:    // ENCODER
    return "ERROR_ENC_ASSIGN_ENTRY_FAILED";

    break;

  case 0x14401:
    return "ERROR_ENC_TYPE_NOT_SUPPORTED";

    break;

  case 0x14402:
    return "ERROR_ENC_SCALE_DENOM_ZERO";

    break;

  case 0x14403:
    return "ERROR_ENC_INVALID_RATE";

    break;

  case 0x14404:
    return "ERROR_ENC_ENTRY_NULL";

    break;

  case 0x14405:
    return "ERROR_ENC_ENTRY_READ_FAIL";

    break;

  case 0x14406:
    return "ERROR_ENC_TRANSFORM_NULL";

    break;

  case 0x14407:
    return "ERROR_ENC_EXT_MASTER_SOURCE_NULL";

    break;

  case 0x14408:
    return "ERROR_ENC_EXT_MASTER_SOURCE_COUNT_ZERO";

    break;

  case 0x14409:
    return "ERROR_ENC_INVALID_SAMPLE_TIME";

    break;

  case 0x1440A:
    return "ERROR_ENC_TRANSFORM_VALIDATION_ERROR";

    break;

  case 0x1440B:
    return "ERROR_ENC_SLAVE_INTERFACE_NULL";

    break;

  case 0x1440C:
    return "ERROR_ENC_VELOCITY_FILTER_NULL";

    break;

  case 0x1440D:
    return "ERROR_ENC_RAW_MASK_INVALID";

    break;

  case 0x1440E:
    return "ERROR_ENC_ABS_MASK_INVALID";

    break;

  case 0x1440F:
    return "ERROR_ENC_ABS_BIT_COUNT_INVALID";

    break;

  case 0x14410:
    return "ERROR_ENC_HW_ALARM_0";

    break;

  case 0x14411:
    return "ERROR_ENC_HW_ALARM_1";

    break;

  case 0x14412:
    return "ERROR_ENC_HW_ALARM_2";

    break;

  case 0x14413:
    return "ERROR_ENC_WARNING_READ_ENTRY_FAIL";

    break;

  case 0x14414:
    return "ERROR_ENC_ALARM_READ_ENTRY_FAIL";

    break;

  case 0x14415:
    return "ERROR_ENC_ASYN_PARAM_NULL";

    break;

  case 0x14416:
    return "ERROR_ENC_READY_READ_ENTRY_FAIL";

    break;

  case 0x14417:
    return "ERROR_ENC_NOT_READY";

    break;

  case 0x14418:
    return "ERROR_ENC_HOME_TRIGG_LINKS_INVALID";

    break;

  case 0x14C00:  // MONITOR
    return "ERROR_MON_ASSIGN_ENTRY_FAILED";

    break;

  case 0x14C01:
    return "ERROR_MON_ENTRY_READ_FAIL";

    break;

  case 0x14C02:
    return "ERROR_MON_ENTRY_HARD_BWD_NULL";

    break;

  case 0x14C03:
    return "ERROR_MON_ENTRY_HARD_FWD_NULL";

    break;

  case 0x14C04:
    return "ERROR_MON_ENTRY_HOME_NULL";

    break;

  case 0x14C05:
    return "ERROR_MON_ENTRY_HARDWARE_INTERLOCK_NULL";

    break;

  case 0x14C06:
    return "ERROR_MON_MAX_CONTROLLER_OUTPUT_EXCEEDED";

    break;

  case 0x14C07:
    return "ERROR_MON_MAX_VELOCITY_EXCEEDED";

    break;

  case 0x14C08:
    return "ERROR_MON_MAX_POSITION_LAG_EXCEEDED";

    break;

  case 0x14C09:
    return "ERROR_MON_EXTERNAL_HARDWARE_INTERLOCK";

    break;

  case 0x14C0A:
    return "ERROR_MON_CNTRL_OUTPUT_INCREASE_AT_LIMIT";

    break;

  case 0x14C0B:
    return "ERROR_MON_SOFT_LIMIT_FWD_INTERLOCK";

    break;

  case 0x14C0C:
    return "ERROR_MON_SOFT_LIMIT_BWD_INTERLOCK";

    break;

  case 0x14C0D:
    return "ERROR_MON_HARD_LIMIT_FWD_INTERLOCK";

    break;

  case 0x14C0E:
    return "ERROR_MON_HARD_LIMIT_BWD_INTERLOCK";

    break;

  case 0x14C0F:
    return "ERROR_MON_POS_LAG_INTERLOCK";

    break;

  case 0x14C10:
    return "ERROR_MON_BOTH_LIMIT_INTERLOCK";

    break;

  case 0x14C11:
    return "ERROR_MON_DISTANCE_TO_STOP_ZERO";

    break;

  case 0x14C12:
    return "ERROR_MON_ENTRY_EXT_INTERLOCK_NULL";

    break;

  case 0x14C13:
    return "ERROR_MON_UNEXPECTED_LIMIT_SWITCH_BEHAVIOUR_INTERLOCK";

    break;

  case 0x14C14:
    return "ERROR_MON_VELOCITY_DIFFERENCE_EXCEEDED";

    break;

  case 0x14C15:
    return "ERROR_MON_TOL_OUT_OF_RANGE";

    break;

  case 0x14C16:
    return "ERROR_MON_TIME_OUT_OF_RANGE";

    break;

  case 0x14C17:
    return "ERROR_MON_POLARITY_OUT_OF_RANGE";

    break;

  case 0x14D00:    // SEQUENCER
    return "ERROR_SEQ_TRAJ_NULL";

    break;

  case 0x14D01:
    return "ERROR_SEQ_MON_NULL";

    break;

  case 0x14D02:
    return "ERROR_MON_ENTRY_HOME_NULL";

    break;

  case 0x14D03:
    return "ERROR_SEQ_CNTRL_NULL";

    break;

  case 0x14D04:
    return "ERROR_SEQ_SEQ_FAILED";

    break;

  case 0x14D05:
    return "ERROR_SEQ_COMMAND_NOT_SUPPORTED";

    break;

  case 0x14D06:
    return "ERROR_SEQ_SOFT_LIMIT_FWD";

    break;

  case 0x14D07:
    return "ERROR_SEQ_SOFT_LIMIT_BWD";

    break;

  case 0x14D08:
    return "ERROR_SEQ_TIMEOUT";

    break;

  case 0x14D09:
    return "ERROR_SEQ_CMD_OUT_OF_RANGE";

    break;

  case 0x14D0A:
    return "ERROR_SEQ_CMD_DATA_UNDEFINED";

    break;

  case 0x14D0B:
    return "ERROR_SEQ_EXTERNAL_DATA_INTERFACE_NULL";

    break;

  case 0x14D0C:
    return "ERROR_SEQ_NO_HOME_SWITCH_FLANK";

    break;

  case 0x14D0D:
    return "ERROR_SEQ_NO_SECOND_HOME_SWITCH_FLANK";

    break;

  case 0x14D0E:
    return "ERROR_SEQ_ERROR_ABS_BIT_OUT_OF_RANGE";

    break;

  case 0x14D0F:
    return "ERROR_SEQ_ERROR_POSITION_SANITY_CHECK_FAILED";

    break;

  case 0x14D10:
    return "ERROR_SEQ_ERROR_ACCELERATION_ZERO";

    break;

  case 0x14D11:
    return "ERROR_SEQ_ERROR_DECELERATION_ZERO";

    break;

  case 0x14D12:
    return "ERROR_SEQ_ERROR_VELOCITY_ZERO";

    break;

  case 0x14D13:
    return "ERROR_SEQ_ABS_OVER_UNDER_FLOW_ERROR";

    break;

  case 0x14D14:
    return "ERROR_SEQ_LATCH_COUNT_OUT_OF_RANGE";

    break;

  case 0x14D15:
    return "ERROR_SEQ_TARGET_POS_OUT_OF_RANGE";

    break;

  case 0x14D16:
    return "ERROR_SEQ_MOTION_CMD_NOT_ENABLED";

    break;

  case 0x14D17:
    return "ERROR_SEQ_HOME_POST_MOVE_FAILED";

    break;

  case 0x14D18:
    return "ERROR_SEQ_HOME_ENC_SOURCE_NOT_INTERNAL";

    break;

  case 0x14D19:
    return "ERROR_SEQ_HOME_SEQ_NOT_SUPPORTED";

    break;

  case 0x14D1A:
    return "ERROR_SEQ_HOME_NOT_ALLOWED";

    break;

  case 0x14E00:    // TRAJECTORY
    return "ERROR_TRAJ_EXT_ENC_NULL";

    break;

  case 0x14E01:
    return "ERROR_TRAJ_EXT_TRAJ_NULL";

    break;

  case 0x14E02:
    return "ERROR_TRAJ_NOT_ENABLED";

    break;

  case 0x14E03:
    return "ERROR_TRAJ_GEAR_RATIO_DENOM_ZERO";

    break;

  case 0x14E04:
    return "ERROR_TRAJ_SOFT_LIMIT_FWD_INTERLOCK";

    break;

  case 0x14E05:
    return "ERROR_TRAJ_SOFT_LIMIT_BWD_INTERLOCK";

    break;

  case 0x14E06:
    return "ERROR_TRAJ_HARD_LIMIT_FWD_INTERLOCK";

    break;

  case 0x14E07:
    return "ERROR_TRAJ_HARD_LIMIT_BWD_INTERLOCK";

    break;

  case 0x14E08:
    return "ERROR_TRAJ_POS_LAG_INTERLOCK";

    break;

  case 0x14E09:
    return "ERROR_TRAJ_BOTH_LIMIT_INTERLOCK";

    break;

  case 0x14E0A:
    return "ERROR_TRAJ_EXTERNAL_INTERLOCK";

    break;

  case 0x14E0B:
    return "ERROR_TRAJ_EXECUTE_BUT_NO_ENABLE";

    break;

  case 0x14E0C:
    return "ERROR_TRAJ_EXT_TRANSFORM_NULL";

    break;

  case 0x14E0D:
    return "ERROR_TRAJ_TRANSFORM_NOT_COMPILED";

    break;

  case 0x14E0E:
    return "ERROR_TRAJ_TRANSFORM_INTERLOCK_ERROR";

    break;

  case 0x14E0F:
    return "ERROR_TRAJ_TRANSFORM_NULL";

    break;

  case 0x14E10:
    return "ERROR_TRAJ_EXT_MASTER_SOURCE_NULL";

    break;

  case 0x14E11:
    return "ERROR_TRAJ_EXT_MASTER_SOURCE_COUNT_ZERO";

    break;

  case 0x14E12:
    return "ERROR_TRAJ_INVALID_SAMPLE_TIME";

    break;

  case 0x14E13:
    return "ERROR_TRAJ_TRANSFORM_VALIDATION_ERROR";

    break;

  case 0x14E14:
    return "ERROR_TRAJ_SLAVE_INTERFACE_NULL";

    break;

  case 0x14E15:
    return "ERROR_TRAJ_MAX_SPEED_INTERLOCK";

    break;

  case 0x14E16:
    return "ERROR_TRAJ_MOD_FACTOR_OUT_OF_RANGE";

    break;

  case 0x14E17:
    return "ERROR_TRAJ_MOD_TYPE_OUT_OF_RANGE";

    break;

  case 0x14E18:
    return "ERROR_TRAJ_MOD_POS_CHANGE_WHILE_BUSY_NOT_ALLOWED";

    break;

  case 0x14E80:    // TRAJ-S (ruckig)
    return "ERROR_TRAJ_RUCKIG_ERROR";

    break;

  case 0x14E81:
    return "ERROR_TRAJ_RUCKIG_INVALID_INPUT";

    break;

  case 0x14E82:
    return "ERROR_TRAJ_RUCKIG_TRAJ_DURATION";

    break;

  case 0x14E83:
    return "ERROR_TRAJ_RUCKIG_POS_LIMITS";

    break;

  case 0x14E84:
    return "ERROR_TRAJ_RUCKIG_NO_PHASE_SYNC";

    break;

  case 0x14E85:
    return "ERROR_TRAJ_RUCKIG_EXE_TIME_CALC";

    break;

  case 0x14E86:
    return "ERROR_TRAJ_RUCKIG_SYNC_CALC";

    break;

  case 0x14E87:
    return "ERROR_TRAJ_RUCKIG_JERK_ZERO";

    break;

  case 0x14F00:    // VIRTUAL AXIS
    return "ERROR_VIRT_AXIS_TRAJ_NULL";

    break;

  case 0x14F01:
    return "ERROR_VIRT_AXIS_CREATE_TRANSFORM_INDEX_OUT_OF_RANGE";

    break;

  case 0x14F02:
    return "ERROR_VIRT_AXIS_INDEX_OUT_OF_RANGE";

    break;

  case 0x14F03:
    return "ERROR_VIRT_AXIS_LINKED_AXIS_TRAJ_NULL";

    break;

  case 0x14F04:
    return "ERROR_VIRT_AXIS_INDEX_NULL";

    break;

  case 0x15000:    // CONTROLLER
    return "ERROR_CNTRL_INVALID_SAMPLE_TIME";

    break;

  case 0x30000:    // TRANSFORM
    return "ERROR_TRANSFORM_EXPR_NOT_COMPILED";

    break;

  case 0x30001:
    return "ERROR_TRANSFORM_INPUT_INDEX_OUT_OF_RANGE";

    break;

  case 0x30002:
    return "ERROR_TRANSFORM_COMPILE_ERROR";

    break;

  case 0x30003:
    return "ERROR_TRANSFORM_INPUT_DATA_SOURCE_NULL";

    break;

  case 0x30004:
    return "ERROR_TRANSFORM_ERROR_ADD_VARIABLE";

    break;

  case 0x30005:
    return "ERROR_TRANSFORM_INPUT_DATA_SOURCE_COUNT_ZERO";

    break;

  case 0x30006:
    return "ERROR_TRANSFORM_NOT_ENABLED";

    break;

  case 0x30007:
    return "ERROR_TRANSFORM_VECTOR_ALLOCATION_FAILED";

    break;

  case 0x30008:
    return "ERROR_TRANSFORM_EXPRTK_ALLOCATION_FAILED";

    break;

  case 0x30100:    // MASTERDATA INTERFACE
    return "ERROR_MASTER_DATA_IF_INDEX_OUT_OF_RANGE";

    break;

  case 0x30101:
    return "ERROR_MASTER_DATA_IF_GEAR_RATIO_DENOM_ZERO";

    break;

  case 0x30102:
    return "ERROR_MASTER_DATA_IF_EXPRESSION_VAR_TRAJ_MISSING";

    break;

  case 0x30103:
    return "ERROR_MASTER_DATA_IF_EXPRESSION_VAR_ENC_MISSING";

    break;

  case 0x30104:
    return "ERROR_MASTER_DATA_IF_VALIDATE_CHAR_BUFFER_OVERFLOW";

    break;

  case 0x21000:    // ECENTRY
    return "ERROR_EC_ENTRY_DATA_POINTER_NULL";

    break;

  case 0x21001:
    return "ERROR_EC_ENTRY_INVALID_OFFSET";

    break;

  case 0x21002:
    return "ERROR_EC_ENTRY_INVALID_DOMAIN_ADR";

    break;

  case 0x21003:
    return "ERROR_EC_ENTRY_INVALID_BIT_LENGTH";

    break;

  case 0x21004:
    return "ERROR_EC_ENTRY_LINK_FAILED";

    break;

  case 0x21005:
    return "ERROR_EC_ENTRY_INDEX_OUT_OF_RANGE";

    break;

  case 0x21006:
    return "ERROR_EC_ENTRY_INVALID_BIT_INDEX";

    break;

  case 0x21007:
    return "ERROR_EC_ENTRY_READ_FAIL";

    break;

  case 0x21008:
    return "ERROR_EC_ENTRY_WRITE_FAIL";

    break;

  case 0x21009:
    return "ERROR_EC_ENTRY_ASYN_TYPE_NOT_SUPPORTED";

    break;

  case 0x2100A:
    return "ERROR_EC_ENTRY_ASSIGN_ADD_FAIL";

    break;

  case 0x2100B:
    return "ERROR_EC_ENTRY_REGISTER_FAIL";

    break;

  case 0x2100C:
    return "ERROR_EC_ENTRY_VALUE_OUT_OF_RANGE";

    break;

  case 0x2100D:
    return "ERROR_EC_ENTRY_SET_ALARM_STATE_FAIL";

    break;

  case 0x2100E:
    return "ERROR_EC_ENTRY_EC_DOMAIN_ERROR";

    break;

  case 0x2100F:
    return "ERROR_EC_ENTRY_DATATYPE_INVALID";

    break;

  case 0x21010:
    return "ERROR_EC_ENTRY_SIZE_OUT_OF_RANGE";

    break;

  case 0x22000:    // ECPDO
    return "ERROR_EC_PDO_ENTRY_ARRAY_FULL";

    break;

  case 0x22001:
    return "ERROR_EC_PDO_ADD_FAIL";

    break;

  case 0x22002:
    return "ERROR_EC_PDO_CLEAR_ENTRIES_FAIL";

    break;

  case 0x23000:    // ECSDO
    return "ERROR_EC_SDO_SIZE_TO_LARGE";

    break;

  case 0x23001:
    return "ERROR_EC_SDO_WRITE_FAILED";

    break;

  case 0x23002:
    return "ERROR_EC_SDO_READ_FAILED";

    break;

  case 0x23003:
    return "ERROR_EC_SDO_VERIFY_FAILED";

    break;

  case 0x23004:
    return "ERROR_EC_SDO_DATA_SIZE_ERROR";

    break;

  case 0x23005:
    return "ERROR_EC_SDO_BUFFER_ALLOC_FAIL";

    break;

  case 0x23006:
    return "ERROR_EC_SDO_DATATYPE_ERROR";

    break;

  case 0x23007:
    return "ERROR_EC_SDO_VALUE_CONV_ERROR";

    break;

  case 0x23500: // ECSDOASYNC
    return "ERROR_EC_SDO_ASYNC_BUSY";

    break;

  case 0x23501: // ECSDOASYNC
    return "ERROR_EC_SDO_ASYNC_ERROR";

    break;

  case 0x23502: // ECSDOASYNC
    return "ERROR_EC_SDO_ASYNC_OBJ_NULL";

    break;

  case 0x23503: // ECSDOASYNC
    return "ERROR_EC_SDO_ASYNC_ASYN_OBJ_FAIL";

    break;

  case 0x24000:  // ECSLAVE
    return "ERROR_EC_SLAVE_CONFIG_FAILED";

    break;

  case 0x24001:
    return "ERROR_EC_SLAVE_CALL_NOT_ALLOWED_IN_SIM_MODE";

    break;

  case 0x24002:
    return "ERROR_EC_SLAVE_SM_ARRAY_FULL";

    break;

  case 0x24003:
    return "ERROR_EC_SLAVE_SM_INDEX_OUT_OF_RANGE";

    break;

  case 0x24004:
    return "ERROR_EC_SLAVE_ENTRY_INFO_STRUCT_NULL";

    break;

  case 0x24005:
    return "ERROR_EC_SLAVE_ENTRY_INDEX_OUT_OF_RANGE";

    break;

  case 0x24006:
    return "ERROR_EC_SLAVE_SLAVE_INFO_STRUCT_NULL";

    break;

  case 0x24007:
    return "ERROR_EC_SLAVE_CONFIG_PDOS_FAILED";

    break;

  case 0x24008:
    return "ERROR_EC_SLAVE_ENTRY_NULL";

    break;

  case 0x24009:
    return "ERROR_EC_SLAVE_STATE_CHANGED";

    break;

  case 0x2400A:
    return "ERROR_EC_SLAVE_ONLINE_OFFLINE_CHANGED";

    break;

  case 0x2400B:
    return "ERROR_EC_SLAVE_OPERATIONAL_CHANGED";

    break;

  case 0x2400C:
    return "ERROR_EC_SLAVE_CONFIG_NULL";

    break;

  case 0x2400D:
    return "ERROR_EC_SLAVE_STATE_INIT";

    break;

  case 0x2400E:
    return "ERROR_EC_SLAVE_STATE_PREOP";

    break;

  case 0x2400F:
    return "ERROR_EC_SLAVE_STATE_SAFEOP";

    break;

  case 0x24010:
    return "ERROR_EC_SLAVE_STATE_UNDEFINED";

    break;

  case 0x24011:
    return "ERROR_EC_SLAVE_NOT_OPERATIONAL";

    break;

  case 0x24012:
    return "ERROR_EC_SLAVE_NOT_ONLINE";

    break;

  case 0x24013:
    return "ERROR_EC_SLAVE_REG_ASYN_PAR_BUFFER_OVERFLOW";

    break;

  case 0x24014:
    return "ERROR_EC_SLAVE_SDO_ASYNC_CREATE_FAIL";

    break;

  case 0x24015:
    return "ERROR_EC_SLAVE_ADD_DATA_ITEM_FAIL";

    break;

  case 0x25000:  // ECSYNCMANAGER
    return "ERROR_EC_SM_PDO_ARRAY_FULL";

    break;

  case 0x25001:
    return "ERROR_EC_SM_PDO_INDEX_OUT_OF_RANGE";

    break;

  case 0x25002:
    return "ERROR_EC_SM_ENTRY_INFO_STRUCT_NULL";

    break;

  case 0x25003:
    return "ERROR_EC_SM_CONFIG_FAIL";

    break;

  case 0x25004:
    return "ERROR_EC_SM_CLEAR_PDO_FAIL";

    break;

  case 0x26000:   // EC
    return "ERROR_EC_MAIN_REQUEST_FAILED";

    break;

  case 0x26001:
    return "ERROR_EC_MAIN_CREATE_DOMAIN_FAILED";

    break;

  case 0x26002:
    return "ERROR_EC_MAIN_INVALID_SLAVE_INDEX";

    break;

  case 0x26003:
    return "ERROR_EC_MAIN_MASTER_ACTIVATE_FAILED";

    break;

  case 0x26004:
    return "ERROR_EC_MAIN_SLAVE_NULL";

    break;

  case 0x26005:
    return "ERROR_EC_MAIN_GET_SLAVE_INFO_FAILED";

    break;

  case 0x26006:
    return "ERROR_EC_MAIN_ENTRY_NULL";

    break;

  case 0x26007:
    return "ERROR_EC_MAIN_GET_ENTRY_INFO_FAILED";

    break;

  case 0x26008:
    return "ERROR_EC_MAIN_DOM_REG_PDO_ENTRY_LIST_FAILED";

    break;

  case 0x26009:
    return "ERROR_EC_MAIN_SDO_ARRAY_FULL";

    break;

  case 0x2600A:
    return "ERROR_EC_MAIN_SDO_ENTRY_NULL";

    break;

  case 0x2600B:
    return "ERROR_EC_MAIN_SDO_READ_FAILED";

    break;

  case 0x2600C:
    return "ERROR_EC_MAIN_DOMAIN_DATA_FAILED";

    break;

  case 0x2600D:
    return "ERROR_EC_MAIN_SLAVE_ARRAY_FULL";

    break;

  case 0x2600E:
    return "ERROR_EC_AL_STATE_INIT";

    break;

  case 0x2600F:
    return "ERROR_EC_AL_STATE_PREOP";

    break;

  case 0x26010:
    return "ERROR_EC_AL_STATE_SAFEOP";

    break;

  case 0x26011:
    return "ERROR_EC_LINK_DOWN";

    break;

  case 0x26012:
    return "ERROR_EC_RESPOND_VS_CONFIG_SLAVES_MISSMATCH";

    break;

  case 0x26013:
    return "ERROR_EC_STATUS_NOT_OK";

    break;

  case 0x26014:
    return "ERROR_EC_ALIAS_TO_LONG";

    break;

  case 0x26015:
    return "ERROR_EC_ASYN_PORT_OBJ_NULL";

    break;

  case 0x26016:
    return "ERROR_EC_ASYN_PORT_CREATE_PARAM_FAIL";

    break;

  case 0x26017:
    return "ERROR_EC_ASYN_SKIP_CYCLES_INVALID";

    break;

  case 0x26018:
    return "ERROR_EC_MEM_MAP_INDEX_OUT_OF_RANGE";

    break;

  case 0x26019:
    return "ERROR_EC_MEM_MAP_START_ENTRY_NULL";

    break;

  case 0x2601A:
    return "ERROR_EC_MEM_MAP_NULL";

    break;

  case 0x2601B:
    return "ERROR_EC_ASYN_ALIAS_NOT_VALID";

    break;

  case 0x2601C:
    return "ERROR_EC_AUTO_CONFIG_MASTER_INFO_FAIL";

    break;

  case 0x2601D:
    return "ERROR_EC_AUTO_CONFIG_SLAVE_INFO_FAIL";

    break;

  case 0x2601E:
    return "ERROR_EC_AUTO_CONFIG_SM_INFO_FAIL";

    break;

  case 0x2601F:
    return "ERROR_EC_AUTO_CONFIG_PDO_INFO_FAIL";

    break;

  case 0x26020:
    return "ERROR_EC_AUTO_CONFIG_ENTRY_INFO_FAIL";

    break;

  case 0x26021:
    return "ERROR_EC_AUTO_CONFIG_MASTER_NOT_SELECTED_FAIL";

    break;

  case 0x26022:
    return "ERROR_EC_AUTO_CONFIG_SLAVE_INDEX_OUT_OF_RANGE";

    break;

  case 0x26023:
    return "ERROR_EC_AUTO_CONFIG_DIRECTION_INVALID";

    break;

  case 0x26024:
    return "ERROR_EC_REG_ASYN_PAR_BUFFER_OVERFLOW";

    break;

  case 0x26025:
    return "ERROR_EC_MASTER_NULL";

    break;

  case 0x26026:
    return "ERROR_EC_SLAVE_VERIFICATION_FAIL";

    break;

  case 0x26027:
    return "ERROR_EC_NO_VALID_CONFIG";

    break;

  case 0x26028:
    return "ERROR_EC_DATATYPE_NOT_VALID";

    break;

  case 0x20000:
    return "ERROR_MAIN_DEMO_EC_ACITVATE_FAILED";

    break;

  case 0x20001:
    return "ERROR_MAIN_DEMO_EC_ACITVATE_FAILED";

    break;

  case 0x20002:
    return "ERROR_MAIN_AXIS_INDEX_OUT_OF_RANGE";

    break;

  case 0x20004:
    return "ERROR_MAIN_AXIS_OBJECT_NULL";

    break;

  case 0x20005:
    return "ERROR_MAIN_ENCODER_INDEX_OUT_OF_RANGE";

    break;

  case 0x20006:
    return "ERROR_MAIN_ENCODER_OBJECT_NULL";

    break;

  case 0x20007:
    return "ERROR_MAIN_CONTROLLER_INDEX_OUT_OF_RANGE";

    break;

  case 0x20008:
    return "ERROR_MAIN_CONTROLLER_OBJECT_NULL";

    break;

  case 0x20009:
    return "ERROR_MAIN_TRAJECTORY_INDEX_OUT_OF_RANGE";

    break;

  case 0x2000A:
    return "ERROR_MAIN_TRAJECTORY_OBJECT_NULL";

    break;

  case 0x2000B:
    return "ERROR_MAIN_DRIVE_INDEX_OUT_OF_RANGE";

    break;

  case 0x2000C:
    return "ERROR_MAIN_DRIVE_OBJECT_NULL";

    break;

  case 0x2000D:
    return "ERROR_MAIN_MONITOR_INDEX_OUT_OF_RANGE";

    break;

  case 0x2000E:
    return "ERROR_MAIN_MONITOR_OBJECT_NULL";

    break;

  case 0x2000F:
    return "ERROR_MAIN_AXIS_TRAJ_OBJECT_NULL";

    break;

  case 0x20010:
    return "ERROR_MAIN_EC_NOT_INITIALIZED";

    break;

  case 0x20011:
    return "ERROR_MAIN_EC_SLAVE_NULL";

    break;

  case 0x20012:
    return "ERROR_MAIN_EC_SM_NULL";

    break;

  case 0x20013:
    return "ERROR_MAIN_EC_PDO_NULL";

    break;

  case 0x20014:
    return "ERROR_MAIN_EC_ENTRY_NULL";

    break;

  case 0x20015:
    return "ERROR_MAIN_ENCODER_ENTRY_INDEX_OUT_OF_RANGE";

    break;

  case 0x20016:
    return "ERROR_MAIN_DRIVE_ENTRY_INDEX_OUT_OF_RANGE";

    break;

  case 0x20017:
    return "ERROR_MAIN_MONITOR_ENTRY_INDEX_OUT_OF_RANGE";

    break;

  case 0x20018:
    return "ERROR_MAIN_DIAG_AXIS_INDEX_OUT_OF_RANGE";

    break;

  case 0x20019:
    return "ERROR_MAIN_DIAG_AXIS_FREQ_OUT_OF_RANGE";

    break;

  case 0x2001A:
    return "ERROR_MAIN_APP_MODE_ALREADY_RUNTIME";

    break;

  case 0x2001B:
    return "ERROR_MAIN_EC_ACTIVATE_FAILED";

    break;

  case 0x2001C:
    return "ERROR_MAIN_ENC_SET_SCALE_FAIL_DRV_ENABLED";

    break;

  case 0x2001D:
    return "ERROR_MAIN_AXIS_ERROR_EXECUTE_INTERLOCKED";

    break;

  case 0x2001E:
    return "ERROR_MAIN_VIRT_AXIS_FUNCTION_NOT_SUPPORTED";

    break;

  case 0x2001F:
    return "ERROR_MAIN_AXIS_TYPE_UNKNOWN";

    break;

  case 0x20020:
    return "ERROR_MAIN_SEQUENCE_OBJECT_NULL";

    break;

  case 0x20021:
    return "ERROR_MAIN_TRAJ_TRANSFORM_OBJECT_NULL";

    break;

  case 0x20022:
    return "ERROR_MAIN_ENC_TRANSFORM_OBJECT_NULL";

    break;

  case 0x20023:
    return "ERROR_MAIN_EXTERNAL_DATA_SOURCE_INVALID";

    break;

  case 0x20024:
    return "ERROR_MAIN_TRANSFORM_OUTPUT_VAR_MISSING";

    break;

  case 0x20025:
    return "ERROR_MAIN_DATA_RECORDER_INDEX_OUT_OF_RANGE";

    break;

  case 0x20026:
    return "ERROR_MAIN_DATA_RECORDER_NULL";

    break;

  case 0x20027:
    return "ERROR_MAIN_DATA_STORAGE_INDEX_OUT_OF_RANGE";

    break;

  case 0x20028:
    return "ERROR_MAIN_DATA_STORAGE_INVALID_SIZE";

    break;

  case 0x20029:
    return "ERROR_MAIN_DATA_STORAGE_NULL";

    break;

  case 0x2002A:
    return "ERROR_MAIN_EVENT_INDEX_OUT_OF_RANGE";

    break;

  case 0x2002B:
    return "ERROR_MAIN_EVENT_NULL";

    break;

  case 0x2002C:
    return "ERROR_MAIN_AXIS_NOT_ENABLED";

    break;

  case 0x2002D:
    return "ERROR_MAIN_MASTER_SLAVE_IF_NULL";

    break;

  case 0x2002E:
    return "ERROR_MAIN_PRINT_TO_BUFFER_FAIL";

    break;

  case 0x2002F:
    return "ERROR_MAIN_MEM_MAP_NULL";

    break;

  case 0x20030:
    return "ERROR_MAIN_AXIS_ASYN_PAR_FORMAT_ERROR";

    break;

  case 0x20031:
    return "ERROR_MAIN_AXIS_ASYN_PORT_DRIVER_NULL";

    break;

  case 0x20032:
    return "ERROR_MAIN_EC_MASTER_NULL";

    break;

  case 0x20033:
    return "ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR";

    break;

  case 0x20034:
    return "ERROR_MAIN_ECMC_LINK_INVALID";

    break;

  case 0x20035:
    return "ERROR_MAIN_PLC_INDEX_OUT_OF_RANGE";

    break;

  case 0x20036:
    return "ERROR_MAIN_PLC_OBJECT_NULL";

    break;

  case 0x20037:
    return "ERROR_MAIN_PLCS_NULL";

    break;

  case 0x20038:
    return "ERROR_MAIN_PLCS_SKIPCYCLES_INVALID";

    break;

  case 0x20039:
    return "ERROR_MAIN_AXIS_EXTERNAL_COM_DISABLED";

    break;

  case 0x2003A:
    return "ERROR_MAIN_EXCEPTION";

    break;

  case 0x2003B:
    return "ERROR_MAIN_PARSER_UNKOWN_CMD";

    break;

  case 0x2003C:
    return "ERROR_MAIN_PARSER_BUFFER_NULL";

    break;

  case 0x2003D:
    return "ERROR_MAIN_PARSER_CMD_TO_LONG";

    break;

  case 0x2003E:
    return "ERROR_MAIN_PARSER_INVALID_ADS_FORMAT";

    break;

  case 0x2003F:
    return "ERROR_MAIN_PARSER_INVALID_ADS_PORT";

    break;

  case 0x20040:
    return "ERROR_MAIN_PARSER_INVALID_ADS_LEN";

    break;

  case 0x20041:
    return "ERROR_MAIN_PARSER_UNKNOWN_ADS_CMD";

    break;

  case 0x20042:
    return "ERROR_MAIN_PARSER_INVALID_ADS_OFFSET";

    break;

  case 0x20043:
    return "ERROR_MAIN_PARSER_INVALID_ADS_TYPE";

    break;

  case 0x20044:
    return "ERROR_MAIN_ASYN_CREATE_PARAM_FAIL";

    break;

  case 0x20045:
    return "ERROR_MAIN_MLOCKALL_FAIL";

    break;

  case 0x20046:
    return "ERROR_MAIN_EC_NULL";

    break;

  case 0x20047:
    return "ERROR_MAIN_EC_SDO_VERIFICATION_FAIL";

    break;

  case 0x20048:
    return "ERROR_MAIN_OBSOLETE_COMMAND";

    break;

  case 0x20049:
    return "ERROR_MAIN_FILTER_INVALID_SIZE";

    break;

  case 0x2004A:
    return "ERROR_MAIN_EC_INDEX_OUT_OF_RANGE";

    break;

  case 0x2004B:
    return "ERROR_MAIN_RT_MUTEX_NULL";

    break;

  case 0x2004C:
    return "ERROR_MAIN_EC_TIMEOUT_OUT_OF_RANGE";

    break;

  case 0x2004D:
    return "ERROR_MAIN_SAMPLE_RATE_OUT_OF_RANGE";

    break;

  case 0x2004E:
    return "ERROR_MAIN_SAMPLE_RATE_CHANGE_NOT_ALLOWED";

    break;

  case 0x2004F:
    return "ERROR_MAIN_PLUGIN_LIST_FULL";

    break;

  case 0x20050:
    return "ERROR_MAIN_PLUGIN_OBJECT_NULL";

    break;

  case 0x20051:
    return "ERROR_MAIN_PLUGIN_INDEX_OUT_OF_RANGE";

    break;

  case 0x20052:
    return "ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL";

    break;

  case 0x20053:
    return "ERROR_MAIN_AXIS_COM_BLOCKED";

    break;

  case 0x20054:
    return "ERROR_MAIN_EC_SCAN_TIMEOUT";

    break;

  case 0x20055:
    return "ERROR_MAIN_AXIS_ALREADY_CREATED";

    break;

  case 0x20056:
    return "ERROR_EC_MAIN_DOMAIN_NULL";

    break;

  case 0x20057:
    return "ERROR_SHMGET_ERROR";

    break;

  case 0x20058:
    return "ERROR_SHMMAT_ERROR";

    break;

  case 0x20059:
    return "ERROR_SHM_INDEX_OUT_OF_RANGE";

    break;

  case 0x2005A:
    return "ERROR_SHM_NULL";

    break;

    break;

  case 0x2005B:
    return "ERROR_SEM_NULL";

    break;

  case 0x20100:   // Data Recorder
    return "ERROR_DATA_RECORDER_BUFFER_NULL";

    break;

  case 0x20101:
    return "ERROR_DATA_RECORDER_DATA_ECENTRY_NULL";

    break;

  case 0x20102:
    return "ERROR_DATA_RECORDER_ECENTRY_READ_FAIL";

    break;

  case 0x20103:
    return "ERROR_DATA_RECORDER_EVENT_NULL";

    break;

  case 0x20104:
    return "ERROR_DATA_RECORDER_HARDWARE_STATUS_NOT_OK";

    break;

  case 0x20105:
    return "ERROR_DATA_RECORDER_NO_DATA_SOURCE_CHOOSEN";

    break;

  case 0x20106:
    return "ERROR_DATA_RECORDER_AXIS_DATA_NULL";

    break;

  case 0x20107:
    return "ERROR_DATA_RECORDER_AXIS_DATA_TYPE_NOT_CHOOSEN";

    break;

  case 0x20200:   // Data storage
    return "ERROR_DATA_STORAGE_FULL";

    break;

  case 0x20201:
    return "ERROR_DATA_STORAGE_NULL";

    break;

  case 0x20202:
    return "ERROR_DATA_STORAGE_SIZE_TO_SMALL";

    break;

  case 0x20203:
    return "ERROR_DATA_STORAGE_POSITION_OUT_OF_RANGE";

    break;

  case 0x20300:   // Event
    return "ERROR_EVENT_DATA_ECENTRY_NULL";

    break;

  case 0x20301:
    return "ERROR_EVENT_TRIGGER_ECENTRY_NULL";

    break;

  case 0x20302:
    return "ERROR_EVENT_INVALID_SAMPLE_TIME";

    break;

  case 0x20303:
    return "ERROR_EVENT_ECENTRY_READ_FAIL";

    break;

  case 0x20304:
    return "ERROR_EVENT_ARM_ECENTRY_NULL";

    break;

  case 0x20305:
    return "ERROR_EVENT_CONSUMER_INDEX_OUT_OF_RANGE";

    break;

  case 0x20306:
    return "ERROR_EVENT_HARDWARE_STATUS_NOT_OK";

    break;

  case 0x20307:
    return "ERROR_EVENT_ARM_NOT_ENABLED";

    break;

  case 0x20400:  // Command List
    return "ERROR_COMMAND_LIST_NULL";

    break;

  case 0x20401:
    return "ERROR_COMMAND_LIST_INDEX_OUT_OF_RANGE";

    break;

  case 0x20402:
    return "ERROR_COMMAND_LIST_COMMAND_WRITE_FAILED";

    break;

  case 0x20403:
    return "ERROR_COMMAND_LIST_COMMAND_READ_FAILED";

    break;

  case 0x20404:
    return "ERROR_COMMAND_LIST_COMMAND_READ_BACK_NOT_OK";

    break;

  case 0x20405:
    return "ERROR_COMMAND_LIST_VECTOR_ALLOCATION_FAILED";

    break;

  case 0x20406:
    return "ERROR_COMMAND_LIST_VECTOR_FULL";

    break;

  case 0x20407:
    return "ERROR_COMMAND_LIST_RESULT_BUFFER_OVERFLOW";

    break;

  case 0x20500:   // ecmcPLC
    return "ERROR_PLC_EXPRTK_ALLOCATION_FAILED";

    break;

  case 0x20501:
    return "ERROR_PLC_COMPILE_ERROR";

    break;

  case 0x20502:
    return "ERROR_PLC_AXIS_ID_OUT_OF_RANGE";

    break;

  case 0x20503:
    return "ERROR_PLC_ADD_EXPR_LINE_ERROR";

    break;

  case 0x20504:
    return "ERROR_PLC_EXPR_LINE_TO_LONG";

    break;

  case 0x20505:
    return "ERROR_PLC_PLC_DATA_IF_NULL";

    break;

  case 0x20506:
    return "ERROR_PLC_DATA_IF_ALLOCATION_FAILED";

    break;

  case 0x20507:
    return "ERROR_PLC_VARIABLE_COUNT_EXCEEDED";

    break;

  case 0x20508:
    return "ERROR_PLC_AXIS_OBJECT_NULL";

    break;

  case 0x20509:
    return "ERROR_PLC_DATA_STORAGE_INDEX_OUT_OF_RANGE";

    break;

  case 0x2050A:
    return "ERROR_PLC_DATA_STORAGE_OBJECT_NULL";

    break;

  case 0x2050B:
    return "ERROR_PLC_LIB_CMD_COUNT_MISS_MATCH";

    break;

  case 0x2050C:
    return "ERROR_PLC_VARIABLE_NOT_FOUND";

    break;

  case 0x2050D:
    return "ERROR_PLC_ADD_VARIABLE_FAIL";

    break;

  case 0x2050E:
    return "ERROR_PLC_VARIABLE_NAME_TO_LONG";

    break;

  case 0x2050F:
    return "ERROR_PLC_PLUGIN_INDEX_OUT_OF_RANGE";

    break;

  case 0x20600:   // ecmcPLCDataIF
    return "ERROR_PLC_AXIS_DATA_TYPE_ERROR";

    break;

  case 0x20601:
    return "ERROR_PLC_AXIS_NULL";

    break;

  case 0x20602:
    return "ERROR_PLC_EC_NULL";

    break;

  case 0x20603:
    return "ERROR_PLC_SOURCE_INVALID";

    break;

  case 0x20604:
    return "ERROR_PLC_EC_MASTER_INVALID";

    break;

  case 0x20605:
    return "ERROR_PLC_EC_SLAVE_NULL";

    break;

  case 0x20606:
    return "ERROR_PLC_EC_ENTRY_NULL";

    break;

  case 0x20607:
    return "ERROR_PLC_EC_VAR_NAME_INVALID";

    break;

  case 0x20608:
    return "ERROR_PLC_TRAJ_NULL";

    break;

  case 0x20609:
    return "ERROR_PLC_MON_NULL";

    break;

  case 0x2060A:
    return "ERROR_PLC_DATA_STORGAE_DATA_TYPE_ERROR";

    break;

  case 0x2060B:
    return "ERROR_PLC_DATA_STORAGE_NULL";

    break;

  case 0x20700:
    return "ERROR_PLCS_INDEX_OUT_OF_RANGE";

    break;

  case 0x20701:
    return "ERROR_PLCS_AXIS_INDEX_OUT_OF_RANGE";

    break;

  case 0x20702:
    return "ERROR_PLCS_EC_NOT_INITIALIZED";

    break;

  case 0x20703:
    return "ERROR_PLCS_VARIABLE_NAME_TO_LONG";

    break;

  case 0x20704:
    return "ERROR_PLCS_DATA_STORAGE_INDEX_OUT_OF_RANGE";

    break;

  case 0x20705:
    return "ERROR_PLCS_FILE_NOT_FOUND";

    break;

  case 0x20706:
    return "ERROR_PLCS_INVALID_VAR_NAME";

    break;

  case 0x20707:
    return "ERROR_PLCS_PLC_NULL";

    break;

  case 0x20708:
    return "ERROR_PLCS_EC_VAR_BIT_ACCESS_NOT_ALLOWED";

    break;

  case 0x20709:
    return "ERROR_PLCS_PLUGIN_INDEX_OUT_OF_RANGE";

    break;

  case 0x20800:
    return "ERROR_PLC_EC_LIB_BITS_OUT_OF_RANGE";

    break;

  case 0x200000:
    return "ECMC_PARSER_READ_STORAGE_BUFFER_DATA_NULL";

    break;

  case 0x210000:
    return "ECMC_COMMAND_FORMAT_ERROR";

    break;

  // Mem Map
  case 0x211000:
    return "ERROR_MEM_MAP_SIZE_OUT_OF_RANGE";

    break;

  case 0x211001:
    return "ERROR_MEM_ASYN_VAR_BUFFER_OUT_OF_RANGE";

    break;

  case 0x211002:
    return "ERROR_MEM_INDEX_OUT_OF_RANGE";

    break;

  case 0x211003:
    return "ERROR_MEM_INVALID_DATA_TYPE";

    break;

  case 0x212000:
    return "ERROR_ECDATAITEM_MAP_SIZE_OUT_OF_RANGE";

    break;

  case 0x212001:
    return "ERROR_ECDATAITEM_ASYN_VAR_BUFFER_OUT_OF_RANGE";

    break;

  case 0x212002:
    return "ERROR_ECDATAITEM_INDEX_OUT_OF_RANGE";

    break;

  case 0x212003:
    return "ERROR_ECDATAITEM_INVALID_DATA_TYPE";

    break;

  // asynDataItem
  case 0x220000:
    return "ERROR_ASYN_PORT_NULL";

    break;

  case 0x220001:
    return "ERROR_ASYN_DATA_NULL";

    break;

  case 0x220002:
    return "ERROR_ASYN_DATA_TYPE_NOT_SUPPORTED";

    break;

  case 0x220003:
    return "ERROR_ASYN_CREATE_PARAM_FAIL";

    break;

  case 0x220004:
    return "ERROR_ASYN_PARAM_NOT_VALIDATED";

    break;

  case 0x220005:
    return "ERROR_ASYN_SUPPORTED_TYPES_ARRAY_FULL";

    break;

  case 0x220006:
    return "ERROR_ASYN_DATA_BUFFER_TO_SMALL";

    break;

  case 0x220007:
    return "ERROR_ASYN_WRITE_VALUE_OUT_OF_RANGE";

    break;

  case 0x220008:
    return "ERROR_ASYN_REFRESH_FAIL";

    break;

  case 0x220009:
    return "ERROR_ASYN_CMD_FAIL";

    break;

  case 0x230000:
    return "ERROR_AXIS_FILTER_ALLOC_FAIL";

    break;

  case 0x231000:
    return "ERROR_PLUGIN_FLIE_NOT_FOUND";

    break;

  case 0x231001:
    return "ERROR_PLUGIN_OPEN_FAIL";

    break;

  case 0x231002:
    return "ERROR_PLUGIN_GET_DATA_FUNC_FAIL";

    break;

  case 0x231003:
    return "ERROR_PLUGIN_GET_DATA_FAIL";

    break;

  case 0x231004:
    return "ERROR_PLUGIN_VERSION_MISSMATCH";

    break;

  case 0x231005:
    return "ERROR_PLUGIN_LIB_NAME_UNDEFINED";

    break;

  case 0x231006:
    return "ERROR_PLUGIN_DATA_NULL";

    break;

  case 0x231007:
    return "ERROR_PLUGIN_DATA_ARG_VS_FUNC_MISSMATCH";

    break;
  }

  return "NO_MESSAGE_STRING_DEFINED_FOR_ERROR_ID";
}
