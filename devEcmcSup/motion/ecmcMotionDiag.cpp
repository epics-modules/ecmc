/*************************************************************************\
* Copyright (c) 2026 Paul Scherrer Institut
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcMotionDiag.cpp
*
\*************************************************************************/

#include "ecmcMotionDiag.h"
#include "ecmcAxisReal.h"
#include "ecmcGeneral.h"
#include "ecmcGlobalsExtern.h"
#include "ecmcRtLogger.h"
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unistd.h>

namespace {

constexpr int kDiagFileLen = 512;

struct DiagRequest {
  int level;
  char file[kDiagFileLen];
};

const char *boolText(bool value) {
  return value ? "true" : "false";
}

const char *commandText(int command) {
  switch (command) {
  case ECMC_CMD_NOCMD:
    return "NO_CMD";
  case ECMC_CMD_JOG:
    return "JOG";
  case ECMC_CMD_MOVEVEL:
    return "MOVE_VEL";
  case ECMC_CMD_MOVEREL:
    return "MOVE_REL";
  case ECMC_CMD_MOVEABS:
    return "MOVE_ABS";
  case ECMC_CMD_MOVEMODULO:
    return "MOVE_MODULO";
  case ECMC_CMD_MOVEPVTABS:
    return "MOVE_PVT_ABS";
  case ECMC_CMD_HOMING:
    return "HOMING";
  case ECMC_CMD_SUPERIMP:
    return "SUPERIMP";
  case ECMC_CMD_GEAR:
    return "GEAR";
  default:
    return "UNKNOWN";
  }
}

const char *dataSourceText(int source) {
  switch (source) {
  case ECMC_DATA_SOURCE_INTERNAL:
    return "INTERNAL";
  case ECMC_DATA_SOURCE_EXTERNAL:
    return "EXTERNAL";
  default:
    return "UNKNOWN";
  }
}

const char *axisTypeText(axisTypes type) {
  switch (type) {
  case ECMC_AXIS_TYPE_REAL:
    return "REAL";
  case ECMC_AXIS_TYPE_VIRTUAL:
    return "VIRTUAL";
  default:
    return "UNKNOWN";
  }
}

const char *masterSlaveStateText(int state) {
  switch (state) {
  case ECMC_MST_SLV_STATE_IDLE:
    return "IDLE";
  case ECMC_MST_SLV_STATE_SLAVES:
    return "SLAVES";
  case ECMC_MST_SLV_STATE_MASTERS:
    return "MASTERS";
  case ECMC_MST_SLV_STATE_RESET:
    return "RESET";
  default:
    return "UNKNOWN";
  }
}

const char *masterSlaveStatusText(int status) {
  switch (status) {
  case ECMC_MST_SLV_STATUS_IDLE:
    return "IDLE";
  case ECMC_MST_SLV_STATUS_SLAVE_ACTIVE:
    return "SLAVE_ACTIVE";
  case ECMC_MST_SLV_STATUS_PREPARING_MASTER:
    return "PREPARING_MASTER";
  case ECMC_MST_SLV_STATUS_WAIT_SLAVE_EXTERNAL:
    return "WAIT_SLAVE_EXTERNAL";
  case ECMC_MST_SLV_STATUS_MASTER_MOVING:
    return "MASTER_MOVING";
  case ECMC_MST_SLV_STATUS_WAIT_MASTER_AT_TARGET:
    return "WAIT_MASTER_AT_TARGET";
  case ECMC_MST_SLV_STATUS_WAIT_MASTER_DISABLE:
    return "WAIT_MASTER_DISABLE";
  case ECMC_MST_SLV_STATUS_FORCED_TIMEOUT_RECOVERY:
    return "FORCED_TIMEOUT_RECOVERY";
  default:
    return "UNKNOWN";
  }
}

const char *masterSlaveTransitionReasonText(int reason) {
  switch (reason) {
  case ECMC_MST_SLV_TRANSITION_NONE:
    return "NONE";
  case ECMC_MST_SLV_TRANSITION_EXTERNAL_STATE_WRITE:
    return "EXTERNAL_STATE_WRITE";
  case ECMC_MST_SLV_TRANSITION_CONTROL_DISABLED:
    return "CONTROL_DISABLED";
  case ECMC_MST_SLV_TRANSITION_SLAVE_COMMAND:
    return "SLAVE_COMMAND";
  case ECMC_MST_SLV_TRANSITION_MASTER_COMMAND:
    return "MASTER_COMMAND";
  case ECMC_MST_SLV_TRANSITION_SLAVE_COMPLETE:
    return "SLAVE_COMPLETE";
  case ECMC_MST_SLV_TRANSITION_MASTER_COMPLETE:
    return "MASTER_COMPLETE";
  case ECMC_MST_SLV_TRANSITION_LOST_ENABLE_OR_ERROR:
    return "LOST_ENABLE_OR_ERROR";
  case ECMC_MST_SLV_TRANSITION_SLAVE_TRAJ_SOURCE_FAILED:
    return "SLAVE_TRAJ_SOURCE_FAILED";
  case ECMC_MST_SLV_TRANSITION_RESET_COMPLETE:
    return "RESET_COMPLETE";
  case ECMC_MST_SLV_TRANSITION_PREPARE_TIMEOUT:
    return "PREPARE_TIMEOUT";
  case ECMC_MST_SLV_TRANSITION_MASTER_DISABLE_TIMEOUT:
    return "MASTER_DISABLE_TIMEOUT";
  default:
    return "UNKNOWN";
  }
}

void jsonString(FILE *fp, const char *text) {
  fputc('"', fp);
  if (text) {
    for (const char *p = text; *p; ++p) {
      switch (*p) {
      case '\\':
        fputs("\\\\", fp);
        break;
      case '"':
        fputs("\\\"", fp);
        break;
      case '\n':
        fputs("\\n", fp);
        break;
      case '\r':
        fputs("\\r", fp);
        break;
      case '\t':
        fputs("\\t", fp);
        break;
      default:
        if (static_cast<unsigned char>(*p) < 0x20) {
          fprintf(fp, "\\u%04x", static_cast<unsigned char>(*p));
        } else {
          fputc(*p, fp);
        }
        break;
      }
    }
  }
  fputc('"', fp);
}

void writeName(FILE *fp, int indent, const char *name) {
  fprintf(fp, "%*s", indent, "");
  jsonString(fp, name);
  fputs(": ", fp);
}

void writeJsonInt(FILE *fp, int indent, const char *name, long long value, bool comma = true) {
  writeName(fp, indent, name);
  fprintf(fp, "%lld%s\n", value, comma ? "," : "");
}

void writeJsonDouble(FILE *fp, int indent, const char *name, double value, bool comma = true) {
  writeName(fp, indent, name);
  fprintf(fp, "%.17g%s\n", value, comma ? "," : "");
}

void writeJsonBool(FILE *fp, int indent, const char *name, bool value, bool comma = true) {
  writeName(fp, indent, name);
  fprintf(fp, "%s%s\n", boolText(value), comma ? "," : "");
}

void writeJsonString(FILE *fp, int indent, const char *name, const char *value, bool comma = true) {
  writeName(fp, indent, name);
  jsonString(fp, value);
  fprintf(fp, "%s\n", comma ? "," : "");
}

void writeTimestamp(FILE *fp) {
  time_t now = time(NULL);
  struct tm localTime;
  localtime_r(&now, &localTime);
  char buffer[64] = {0};
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S%z", &localTime);
  writeJsonString(fp, 2, "timestamp", buffer);
}

void makeDumpFileName(char *buffer, size_t bufferSize) {
  time_t now = time(NULL);
  struct tm localTime;
  localtime_r(&now, &localTime);
  char timeBuffer[32] = {0};
  strftime(timeBuffer, sizeof(timeBuffer), "%Y%m%d_%H%M%S", &localTime);
  snprintf(buffer,
           bufferSize,
           "/tmp/ecmc_motion_diag_%ld_%s.yaml",
           static_cast<long>(getpid()),
           timeBuffer);
}

int collectAxisBlockers(ecmcAxisBase *axis,
                        const ecmcAxisDataStatus &status,
                        const char **blockers,
                        int maxBlockers) {
  int count = 0;
  auto add = [&](const char *text) {
    if (count < maxBlockers) {
      blockers[count] = text;
    }
    ++count;
  };
  const int command = axis->getCommand();
  if (axis->getError()) {
    add("axis_error");
  }
  if (!axis->getRealTimeStarted()) {
    add("realtime_not_started");
  }
  if (axis->getInStartupPhase()) {
    add("axis_in_startup");
  }
  if (axis->getBlockCom()) {
    add("command_interface_blocked");
  }
  if (axis->getBlocked() || status.statusWord_.blocked) {
    add("blocked_by_axis_group_or_master_slave");
  }
  if (!axis->getHwReady()) {
    add("hardware_not_ready");
  }
  if (status.statusWord_.sumilockfwd) {
    add("forward_interlock");
  }
  if (status.statusWord_.sumilockbwd) {
    add("backward_interlock");
  }
  if (status.statusWord_.limitfwd) {
    add("forward_limit_active");
  }
  if (status.statusWord_.limitbwd) {
    add("backward_limit_active");
  }
  if ((command == ECMC_CMD_MOVEABS ||
       command == ECMC_CMD_MOVEREL ||
       command == ECMC_CMD_MOVEPVTABS) &&
      !axis->getAllowPos()) {
    add("position_motion_disabled");
  }
  if ((command == ECMC_CMD_MOVEVEL ||
       command == ECMC_CMD_JOG) &&
      !axis->getAllowConstVelo()) {
    add("velocity_motion_disabled");
  }
  if (command == ECMC_CMD_HOMING && !axis->getAllowHome()) {
    add("homing_disabled");
  }
  if (command != ECMC_CMD_NOCMD && !axis->getExecute()) {
    add("command_loaded_but_execute_false");
  }
  if (!axis->getEnable()) {
    add("enable_command_false");
  }
  if (axis->getEnable() && !axis->getEnabledOnly()) {
    add("enable_command_true_but_drive_not_enabled");
  }
  return count;
}

void writeBlockerArray(FILE *fp, ecmcAxisBase *axis, const ecmcAxisDataStatus &status) {
  const char *blockers[32] = {0};
  const int blockerCount = collectAxisBlockers(axis, status, blockers, 32);
  fprintf(fp, "      \"motion_blockers\": [");
  for (int i = 0; i < blockerCount && i < 32; ++i) {
    fprintf(fp, "%s", i == 0 ? "" : ", ");
    jsonString(fp, blockers[i]);
  }
  fprintf(fp, "],\n");
}

const char *blockerSeverity(const char *blocker) {
  if (!blocker) {
    return "info";
  }
  if (strcmp(blocker, "axis_error") == 0 ||
      strcmp(blocker, "realtime_not_started") == 0 ||
      strcmp(blocker, "forward_interlock") == 0 ||
      strcmp(blocker, "backward_interlock") == 0 ||
      strcmp(blocker, "hardware_not_ready") == 0) {
    return "error";
  }
  return "warning";
}

const char *blockerSuggestion(const char *blocker) {
  if (!blocker) {
    return "";
  }
  if (strcmp(blocker, "axis_error") == 0) {
    return "Read the axis error ID and reset only after the root cause is removed.";
  }
  if (strcmp(blocker, "realtime_not_started") == 0) {
    return "Check app mode and realtime startup state.";
  }
  if (strcmp(blocker, "axis_in_startup") == 0) {
    return "Wait until startup is finished or check startup sequencing.";
  }
  if (strcmp(blocker, "command_interface_blocked") == 0) {
    return "The axis command interface is blocked; check blockCom and higher-level ownership.";
  }
  if (strcmp(blocker, "blocked_by_axis_group_or_master_slave") == 0) {
    return "Check axis groups and master/slave state machine state.";
  }
  if (strcmp(blocker, "hardware_not_ready") == 0) {
    return "Check drive/encoder hardware ready signals and EtherCAT state.";
  }
  if (strcmp(blocker, "forward_interlock") == 0) {
    return "Decode the forward interlock and check limits, PLC, safety, and trajectory interlocks.";
  }
  if (strcmp(blocker, "backward_interlock") == 0) {
    return "Decode the backward interlock and check limits, PLC, safety, and trajectory interlocks.";
  }
  if (strcmp(blocker, "forward_limit_active") == 0) {
    return "The forward limit is active; verify direction and limit wiring/state.";
  }
  if (strcmp(blocker, "backward_limit_active") == 0) {
    return "The backward limit is active; verify direction and limit wiring/state.";
  }
  if (strcmp(blocker, "position_motion_disabled") == 0) {
    return "Position motion is disabled for this axis.";
  }
  if (strcmp(blocker, "velocity_motion_disabled") == 0) {
    return "Velocity motion is disabled for this axis.";
  }
  if (strcmp(blocker, "homing_disabled") == 0) {
    return "Homing is disabled for this axis.";
  }
  if (strcmp(blocker, "command_loaded_but_execute_false") == 0) {
    return "A motion command is selected but execute is false; check command trigger path.";
  }
  if (strcmp(blocker, "enable_command_false") == 0) {
    return "Enable command is false; enable the axis or check auto-enable settings.";
  }
  if (strcmp(blocker, "enable_command_true_but_drive_not_enabled") == 0) {
    return "Enable command is true but enabled readback is false; check drive enable path.";
  }
  return "Inspect the matching object state in this dump.";
}

void buildAxisReport(int axisIndex, char *buffer, size_t bufferSize) {
  if (!buffer || bufferSize == 0) {
    return;
  }
  if (axisIndex < 0 || axisIndex >= ECMC_MAX_AXES || !axes[axisIndex]) {
    snprintf(buffer,
             bufferSize,
             "Axis %d is not configured.",
             axisIndex);
    return;
  }

  ecmcAxisBase *axis = axes[axisIndex];
  ecmcAxisDataStatus status = {};
  ecmcAxisDataStatus *statusPtr = axis->getAxisStatusDataPtr();
  if (statusPtr) {
    status = *statusPtr;
  }

  const char *blockers[32] = {0};
  const int blockerCount = collectAxisBlockers(axis, status, blockers, 32);
  const int written = snprintf(buffer,
                               bufferSize,
                               "Axis %d id=%d cmd=%s mrreqcnt=%u reqcnt=%u execcnt=%u exec=%d enable=%d enabled=%d busy=%d error=0x%x: ",
                               axisIndex,
                               axis->getAxisID(),
                               commandText(axis->getCommand()),
                               axis->getMotionCommandMotorRecordRequestCounter(),
                               axis->getMotionCommandRequestCounter(),
                               axis->getMotionCommandExecuteCounter(),
                               axis->getExecute() ? 1 : 0,
                               axis->getEnable() ? 1 : 0,
                               axis->getEnabledOnly() ? 1 : 0,
                               axis->getBusy() ? 1 : 0,
                               axis->getErrorID());
  size_t offset = written > 0 ? static_cast<size_t>(written) : 0;
  if (offset >= bufferSize) {
    buffer[bufferSize - 1] = '\0';
    return;
  }

  if (blockerCount <= 0) {
    snprintf(buffer + offset,
             bufferSize - offset,
             "no obvious software blocker.");
    return;
  }

  snprintf(buffer + offset,
           bufferSize - offset,
           "blockers=");
  offset = strlen(buffer);
  for (int i = 0; i < blockerCount && i < 32; ++i) {
    snprintf(buffer + offset,
             bufferSize - offset,
             "%s%s",
             i == 0 ? "" : ",",
             blockers[i]);
    offset = strlen(buffer);
    if (offset >= bufferSize - 1) {
      break;
    }
  }
  if (axis->getBlocked() || status.statusWord_.blocked) {
    offset = strlen(buffer);
    snprintf(buffer + offset,
             bufferSize - offset,
             "; active_sms=");
    offset = strlen(buffer);
    bool firstSms = true;
    for (int i = 0; i < ECMC_MAX_MST_SLVS_SMS; ++i) {
      if (!masterSlaveSMs[i]) {
        continue;
      }
      if (masterSlaveSMs[i]->getState() == ECMC_MST_SLV_STATE_IDLE &&
          masterSlaveSMs[i]->getStatus() == 0) {
        continue;
      }
      snprintf(buffer + offset,
               bufferSize - offset,
               "%s%d:%s,status=%d",
               firstSms ? "" : "|",
               i,
               masterSlaveStateText(masterSlaveSMs[i]->getState()),
               masterSlaveSMs[i]->getStatus());
      firstSms = false;
      offset = strlen(buffer);
      if (offset >= bufferSize - 1) {
        break;
      }
    }
    if (firstSms) {
      offset = strlen(buffer);
      snprintf(buffer + offset,
               bufferSize - offset,
               "none_active");
    }
  }
}

void writeAxisStatusWord(FILE *fp, const ecmcAxisDataStatus &status) {
  fputs("      \"status_word\": {\n", fp);
  writeJsonBool(fp, 8, "enable", status.statusWord_.enable);
  writeJsonBool(fp, 8, "enabled", status.statusWord_.enabled);
  writeJsonBool(fp, 8, "execute", status.statusWord_.execute);
  writeJsonBool(fp, 8, "busy", status.statusWord_.busy);
  writeJsonBool(fp, 8, "at_target", status.statusWord_.attarget);
  writeJsonBool(fp, 8, "moving", status.statusWord_.moving);
  writeJsonBool(fp, 8, "limit_fwd", status.statusWord_.limitfwd);
  writeJsonBool(fp, 8, "limit_bwd", status.statusWord_.limitbwd);
  writeJsonBool(fp, 8, "home_switch", status.statusWord_.homeswitch);
  writeJsonBool(fp, 8, "homed", status.statusWord_.homed);
  writeJsonBool(fp, 8, "in_realtime", status.statusWord_.inrealtime);
  writeJsonBool(fp, 8, "traj_source_external", status.statusWord_.trajsource);
  writeJsonBool(fp, 8, "enc_source_external", status.statusWord_.encsource);
  writeJsonBool(fp, 8, "plc_command_allowed", status.statusWord_.plccmdallowed);
  writeJsonBool(fp, 8, "soft_limit_fwd_enabled", status.statusWord_.softlimfwdena);
  writeJsonBool(fp, 8, "soft_limit_bwd_enabled", status.statusWord_.softlimbwdena);
  writeJsonBool(fp, 8, "in_startup", status.statusWord_.instartup);
  writeJsonBool(fp, 8, "sum_interlock_fwd", status.statusWord_.sumilockfwd);
  writeJsonBool(fp, 8, "sum_interlock_bwd", status.statusWord_.sumilockbwd);
  writeJsonBool(fp, 8, "soft_limit_interlock_fwd", status.statusWord_.softlimilockfwd);
  writeJsonBool(fp, 8, "soft_limit_interlock_bwd", status.statusWord_.softlimilockbwd);
  writeJsonBool(fp, 8, "local_busy", status.statusWord_.localBusy);
  writeJsonBool(fp, 8, "global_busy", status.statusWord_.globalBusy);
  writeJsonBool(fp, 8, "blocked", status.statusWord_.blocked);
  writeJsonInt(fp, 8, "seq_state", status.statusWord_.seqstate);
  writeJsonInt(fp, 8, "last_interlock", status.statusWord_.lastilock, false);
  fputs("      },\n", fp);
}

void writeAxis(FILE *fp, int axisIndex, ecmcAxisBase *axis, bool comma) {
  ecmcAxisDataStatus status = {};
  ecmcAxisDataStatus *statusPtr = axis->getAxisStatusDataPtr();
  if (statusPtr) {
    status = *statusPtr;
  }

  fputs("    {\n", fp);
  writeJsonInt(fp, 6, "index", axisIndex);
  writeJsonInt(fp, 6, "axis_id", axis->getAxisID());
  writeJsonString(fp, 6, "axis_type", axisTypeText(axis->getAxisType()));
  writeJsonString(fp, 6, "command_text", commandText(axis->getCommand()));
  writeJsonInt(fp, 6, "command", axis->getCommand());
  writeJsonInt(fp, 6, "cmd_data", axis->getCmdData());
  writeJsonInt(fp, 6, "motion_command_motor_record_request_counter", axis->getMotionCommandMotorRecordRequestCounter());
  writeJsonInt(fp, 6, "motion_command_request_counter", axis->getMotionCommandRequestCounter());
  writeJsonInt(fp, 6, "motion_command_execute_counter", axis->getMotionCommandExecuteCounter());
  writeJsonBool(fp, 6, "execute", axis->getExecute());
  writeJsonBool(fp, 6, "enable_command", axis->getEnable());
  writeJsonBool(fp, 6, "enabled", axis->getEnabledOnly());
  writeJsonBool(fp, 6, "busy", axis->getBusy());
  writeJsonBool(fp, 6, "local_busy", axis->getLocalBusy());
  writeJsonBool(fp, 6, "global_busy", axis->getGlobalBusy());
  writeJsonBool(fp, 6, "traj_busy", axis->getTrajBusy());
  writeJsonBool(fp, 6, "error", axis->getError());
  writeJsonInt(fp, 6, "error_id", axis->getErrorID());
  writeJsonBool(fp, 6, "blocked", axis->getBlocked());
  writeJsonBool(fp, 6, "block_com", axis->getBlockCom());
  writeJsonBool(fp, 6, "hardware_ready", axis->getHwReady());
  writeJsonBool(fp, 6, "realtime_started", axis->getRealTimeStarted());
  writeJsonBool(fp, 6, "startup_phase", axis->getInStartupPhase());
  writeJsonBool(fp, 6, "allow_position_motion", axis->getAllowPos());
  writeJsonBool(fp, 6, "allow_velocity_motion", axis->getAllowConstVelo());
  writeJsonBool(fp, 6, "allow_home", axis->getAllowHome());
  writeJsonBool(fp, 6, "allow_plc_commands", axis->getAllowCmdFromPLC());
  writeJsonString(fp, 6, "traj_source", dataSourceText(axis->getTrajDataSourceType()));
  writeJsonString(fp, 6, "enc_source", dataSourceText(axis->getEncDataSourceType()));
  writeJsonDouble(fp, 6, "actual_position", status.currentPositionActual);
  writeJsonDouble(fp, 6, "setpoint_position", status.currentPositionSetpoint);
  writeJsonDouble(fp, 6, "target_position", status.currentTargetPosition);
  writeJsonDouble(fp, 6, "actual_velocity", status.currentVelocityActual);
  writeJsonDouble(fp, 6, "setpoint_velocity", status.currentVelocitySetpoint);
  writeJsonDouble(fp, 6, "target_velocity", status.currentVelocityTarget);
  writeJsonDouble(fp, 6, "control_error", status.cntrlError);
  writeJsonDouble(fp, 6, "control_output", status.cntrlOutput);
  writeJsonDouble(fp, 6, "distance_to_stop", status.distToStop);
  writeJsonInt(fp, 6, "cycle_counter", status.cycleCounter);
  writeBlockerArray(fp, axis, status);
  writeAxisStatusWord(fp, status);
  writeJsonInt(fp, 6, "status_error_code", status.errorCode);
  writeJsonInt(fp, 6, "status_warning_code", status.warningCode, false);
  fprintf(fp, "    }%s\n", comma ? "," : "");
}

int countAxes() {
  int count = 0;
  for (int i = 0; i < ECMC_MAX_AXES; ++i) {
    if (axes[i]) {
      ++count;
    }
  }
  return count;
}

int countAxisFindings() {
  int count = 0;
  for (int i = 0; i < ECMC_MAX_AXES; ++i) {
    if (!axes[i]) {
      continue;
    }
    ecmcAxisDataStatus status = {};
    ecmcAxisDataStatus *statusPtr = axes[i]->getAxisStatusDataPtr();
    if (statusPtr) {
      status = *statusPtr;
    }
    const char *blockers[32] = {0};
    const int blockerCount = collectAxisBlockers(axes[i], status, blockers, 32);
    count += blockerCount < 32 ? blockerCount : 32;
  }
  return count;
}

void writeAxes(FILE *fp) {
  fputs("  \"axes\": [\n", fp);
  const int total = countAxes();
  int written = 0;
  for (int i = 0; i < ECMC_MAX_AXES; ++i) {
    if (!axes[i]) {
      continue;
    }
    ++written;
    writeAxis(fp, i, axes[i], written < total);
  }
  fputs("  ],\n", fp);
}

void writeAxisGroup(FILE *fp, int groupIndex, ecmcAxisGroup *group, bool comma) {
  const ecmcAxisGroupStatusSummary summary = group->getStatusSummary();
  fputs("    {\n", fp);
  writeJsonInt(fp, 6, "index", groupIndex);
  writeJsonString(fp, 6, "name", group->getName());
  writeJsonInt(fp, 6, "axis_count", static_cast<long long>(group->size()));
  writeJsonBool(fp, 6, "blocked", group->getBlocked());
  fputs("      \"axes\": [", fp);
  bool first = true;
  for (int i = 0; i < ECMC_MAX_AXES; ++i) {
    if (group->inGroup(i)) {
      fprintf(fp, "%s%d", first ? "" : ", ", i);
      first = false;
    }
  }
  fputs("],\n", fp);
  fputs("      \"summary\": {\n", fp);
  writeJsonBool(fp, 8, "all_enable_cmd", summary.allEnableCmd);
  writeJsonBool(fp, 8, "any_enable_cmd", summary.anyEnableCmd);
  writeJsonBool(fp, 8, "all_enabled", summary.allEnabled);
  writeJsonBool(fp, 8, "any_enabled", summary.anyEnabled);
  writeJsonBool(fp, 8, "all_busy", summary.allBusy);
  writeJsonBool(fp, 8, "any_busy", summary.anyBusy);
  writeJsonBool(fp, 8, "any_interlocked", summary.anyIlocked);
  writeJsonBool(fp, 8, "all_at_target", summary.allAtTarget);
  writeJsonBool(fp, 8, "all_within_control_deadband", summary.allWithinCtrlDb);
  writeJsonBool(fp, 8, "all_within_slave_control_deadband", summary.allWithinSlvCtrlDb);
  writeJsonBool(fp, 8, "all_traj_external", summary.allTrajExternal);
  writeJsonBool(fp, 8, "any_traj_external", summary.anyTrajExternal);
  writeJsonInt(fp, 8, "first_error_id", summary.firstErrorId, false);
  fputs("      }\n", fp);
  fprintf(fp, "    }%s\n", comma ? "," : "");
}

int countAxisGroups() {
  int count = 0;
  for (int i = 0; i < ECMC_MAX_AXES; ++i) {
    if (axisGroups[i]) {
      ++count;
    }
  }
  return count;
}

int countAxisGroupFindings() {
  int count = 0;
  for (int i = 0; i < ECMC_MAX_AXES; ++i) {
    if (!axisGroups[i]) {
      continue;
    }
    const ecmcAxisGroupStatusSummary summary = axisGroups[i]->getStatusSummary();
    if (axisGroups[i]->getBlocked()) {
      ++count;
    }
    if (summary.anyIlocked) {
      ++count;
    }
    if (summary.firstErrorId) {
      ++count;
    }
  }
  return count;
}

void writeAxisGroups(FILE *fp) {
  fputs("  \"axis_groups\": [\n", fp);
  const int total = countAxisGroups();
  int written = 0;
  for (int i = 0; i < ECMC_MAX_AXES; ++i) {
    if (!axisGroups[i]) {
      continue;
    }
    ++written;
    writeAxisGroup(fp, i, axisGroups[i], written < total);
  }
  fputs("  ],\n", fp);
}

int countMasterSlaveSMs() {
  int count = 0;
  for (int i = 0; i < ECMC_MAX_MST_SLVS_SMS; ++i) {
    if (masterSlaveSMs[i]) {
      ++count;
    }
  }
  return count;
}

int countMasterSlaveSMFindings() {
  int count = 0;
  for (int i = 0; i < ECMC_MAX_MST_SLVS_SMS; ++i) {
    if (!masterSlaveSMs[i]) {
      continue;
    }
    if (!masterSlaveSMs[i]->getEnabled()) {
      ++count;
    }
    if (masterSlaveSMs[i]->getState() != ECMC_MST_SLV_STATE_IDLE) {
      ++count;
    }
    if (masterSlaveSMs[i]->getStatus()) {
      ++count;
    }
    if (masterSlaveSMs[i]->getLastFaultCode()) {
      ++count;
    }
  }
  return count;
}

void writeMasterSlaveSM(FILE *fp, int index, ecmcMasterSlaveStateMachine *sm, bool comma) {
  fputs("    {\n", fp);
  writeJsonInt(fp, 6, "index", index);
  writeJsonString(fp, 6, "name", sm->getName());
  writeJsonString(fp, 6, "state_text", masterSlaveStateText(sm->getState()));
  writeJsonInt(fp, 6, "state", sm->getState());
  writeJsonString(fp, 6, "status_text", masterSlaveStatusText(sm->getStatus()));
  writeJsonInt(fp, 6, "status", sm->getStatus());
  writeJsonInt(fp, 6, "status_word", sm->getStatusWord());
  writeJsonString(fp, 6, "previous_state_text",
                  masterSlaveStateText(sm->getPreviousState()));
  writeJsonInt(fp, 6, "previous_state", sm->getPreviousState());
  writeJsonString(fp, 6, "last_transition_reason_text",
                  masterSlaveTransitionReasonText(sm->getLastTransitionReason()));
  writeJsonInt(fp, 6, "last_transition_reason", sm->getLastTransitionReason());
  writeJsonInt(fp, 6, "execute_cycle_count",
               static_cast<long long>(sm->getExecuteCycleCount()));
  writeJsonInt(fp, 6, "transition_count",
               static_cast<long long>(sm->getTransitionCount()));
  writeJsonInt(fp, 6, "last_transition_execute_cycle",
               static_cast<long long>(sm->getLastTransitionCycle()));
  writeJsonInt(fp, 6, "last_fault_code", sm->getLastFaultCode());
  writeJsonString(fp, 6, "last_fault_text",
                  sm->getLastFaultCode() ? getErrorString(sm->getLastFaultCode()) : "NONE");
  writeJsonString(fp, 6, "last_fault_status_text",
                  masterSlaveStatusText(sm->getLastFaultStatus()));
  writeJsonInt(fp, 6, "last_fault_status", sm->getLastFaultStatus());
  writeJsonInt(fp, 6, "last_fault_execute_cycle",
               static_cast<long long>(sm->getLastFaultCycle()));
  writeJsonBool(fp, 6, "enabled", sm->getEnabled());
  writeJsonBool(fp, 6, "auto_disable_masters", sm->getAutoDisableMasters());
  writeJsonBool(fp, 6, "auto_disable_slaves", sm->getAutoDisableSlaves());
  writeJsonDouble(fp, 6, "master_at_target_timeout_s", sm->getMasterAtTargetTimeout(), false);
  fprintf(fp, "    }%s\n", comma ? "," : "");
}

void writeMasterSlaveSMs(FILE *fp) {
  fputs("  \"master_slave_state_machines\": [\n", fp);
  const int total = countMasterSlaveSMs();
  int written = 0;
  for (int i = 0; i < ECMC_MAX_MST_SLVS_SMS; ++i) {
    if (!masterSlaveSMs[i]) {
      continue;
    }
    ++written;
    writeMasterSlaveSM(fp, i, masterSlaveSMs[i], written < total);
  }
  fputs("  ],\n", fp);
}

void writeFinding(FILE *fp,
                  const char *severity,
                  const char *objectType,
                  int index,
                  const char *code,
                  const char *message,
                  const char *suggestion,
                  bool comma) {
  fputs("      {\n", fp);
  writeJsonString(fp, 8, "severity", severity);
  writeJsonString(fp, 8, "object_type", objectType);
  writeJsonInt(fp, 8, "index", index);
  writeJsonString(fp, 8, "code", code);
  writeJsonString(fp, 8, "message", message);
  writeJsonString(fp, 8, "suggestion", suggestion, false);
  fprintf(fp, "      }%s\n", comma ? "," : "");
}

void writeAxisFindings(FILE *fp, int *written, int totalFindings) {
  for (int i = 0; i < ECMC_MAX_AXES; ++i) {
    if (!axes[i]) {
      continue;
    }
    ecmcAxisDataStatus status = {};
    ecmcAxisDataStatus *statusPtr = axes[i]->getAxisStatusDataPtr();
    if (statusPtr) {
      status = *statusPtr;
    }
    const char *blockers[32] = {0};
    const int blockerCount = collectAxisBlockers(axes[i], status, blockers, 32);
    const int limitedCount = blockerCount < 32 ? blockerCount : 32;
    for (int j = 0; j < limitedCount; ++j) {
      char message[192] = {0};
      snprintf(message,
               sizeof(message),
               "Axis %d may not trigger motion: %s.",
               i,
               blockers[j]);
      ++(*written);
      writeFinding(fp,
                   blockerSeverity(blockers[j]),
                   "axis",
                   i,
                   blockers[j],
                   message,
                   blockerSuggestion(blockers[j]),
                   *written < totalFindings);
    }
  }
}

void writeAxisGroupFindings(FILE *fp, int *written, int totalFindings) {
  for (int i = 0; i < ECMC_MAX_AXES; ++i) {
    if (!axisGroups[i]) {
      continue;
    }
    const ecmcAxisGroupStatusSummary summary = axisGroups[i]->getStatusSummary();
    if (axisGroups[i]->getBlocked()) {
      ++(*written);
      writeFinding(fp,
                   "warning",
                   "axis_group",
                   i,
                   "group_blocked",
                   "Axis group is blocked and may reject or suppress motion commands.",
                   "Check master/slave state and group ownership.",
                   *written < totalFindings);
    }
    if (summary.anyIlocked) {
      ++(*written);
      writeFinding(fp,
                   "error",
                   "axis_group",
                   i,
                   "group_interlocked",
                   "At least one axis in the group is interlocked.",
                   "Inspect the group axes and decode the active axis interlock.",
                   *written < totalFindings);
    }
    if (summary.firstErrorId) {
      ++(*written);
      writeFinding(fp,
                   "error",
                   "axis_group",
                   i,
                   "group_axis_error",
                   "At least one axis in the group is in error.",
                   "Inspect the first_error_id and individual axis errors.",
                   *written < totalFindings);
    }
  }
}

void writeMasterSlaveSMFindings(FILE *fp, int *written, int totalFindings) {
  for (int i = 0; i < ECMC_MAX_MST_SLVS_SMS; ++i) {
    if (!masterSlaveSMs[i]) {
      continue;
    }
    if (!masterSlaveSMs[i]->getEnabled()) {
      ++(*written);
      writeFinding(fp,
                   "warning",
                   "master_slave_state_machine",
                   i,
                   "state_machine_disabled",
                   "Master/slave state machine is disabled.",
                   "If this state machine should arbitrate motion, enable it and verify configuration.",
                   *written < totalFindings);
    }
    if (masterSlaveSMs[i]->getState() != ECMC_MST_SLV_STATE_IDLE) {
      ++(*written);
      writeFinding(fp,
                   "info",
                   "master_slave_state_machine",
                   i,
                   "state_machine_active",
                   "Master/slave state machine is not idle and may be blocking one side.",
                   "Check state_text and whether the requested axis belongs to a blocked group.",
                   *written < totalFindings);
    }
    if (masterSlaveSMs[i]->getStatus()) {
      ++(*written);
      const int status = masterSlaveSMs[i]->getStatus();
      writeFinding(fp,
                   status == ECMC_MST_SLV_STATUS_FORCED_TIMEOUT_RECOVERY ?
                   "error" : "info",
                   "master_slave_state_machine",
                   i,
                   "state_machine_status",
                   "Master/slave state machine reports an active internal phase.",
                   "Inspect status_text, state_text, and whether the requested axis belongs to a blocked group.",
                   *written < totalFindings);
    }
    if (masterSlaveSMs[i]->getLastFaultCode()) {
      ++(*written);
      writeFinding(fp,
                   "warning",
                   "master_slave_state_machine",
                   i,
                   "state_machine_historical_fault",
                   "Master/slave state machine has retained fault history.",
                   "Inspect last_fault_text, last_fault_status_text, and last_fault_execute_cycle.",
                   *written < totalFindings);
    }
  }
}

void writeDiagnosis(FILE *fp) {
  const int totalFindings = countAxisFindings() +
                            countAxisGroupFindings() +
                            countMasterSlaveSMFindings();
  fputs("  \"diagnosis\": {\n", fp);
  writeJsonInt(fp, 4, "finding_count", totalFindings);
  writeJsonString(fp,
                  4,
                  "summary",
                  totalFindings ? "Potential motion blockers found." : "No obvious motion blockers found.");
  fputs("    \"findings\": [\n", fp);
  int written = 0;
  writeAxisFindings(fp, &written, totalFindings);
  writeAxisGroupFindings(fp, &written, totalFindings);
  writeMasterSlaveSMFindings(fp, &written, totalFindings);
  fputs("    ]\n", fp);
  fputs("  },\n", fp);
}

int writeDumpFile(const DiagRequest &request) {
  FILE *fp = fopen(request.file, "w");
  if (!fp) {
    return -1;
  }

  fputs("{\n", fp);
  writeJsonInt(fp, 2, "ecmc_motion_diag_version", 4);
  writeTimestamp(fp);
  writeJsonInt(fp, 2, "level", request.level);
  writeJsonInt(fp, 2, "axis_count", countAxes());
  writeJsonInt(fp, 2, "axis_group_count", countAxisGroups());
  writeJsonInt(fp, 2, "master_slave_state_machine_count", countMasterSlaveSMs());
  writeJsonInt(fp, 2, "app_mode_command", appModeCmd);
  writeJsonInt(fp, 2, "app_mode_status", appModeStat);
  writeJsonInt(fp, 2, "controller_error", controllerError);
  writeJsonString(fp, 2, "controller_error_text", controllerErrorMsg ? controllerErrorMsg : "");
  writeJsonInt(fp,
               2,
               "rt_cycle_counter",
               static_cast<long long>(ecmcUpdatedCounter),
               request.level >= 1);
  if (request.level >= 1) {
    writeDiagnosis(fp);
    writeAxes(fp);
    writeAxisGroups(fp);
    writeMasterSlaveSMs(fp);
    ecmcLogBuffersWriteJson(fp);
  }
  fputs("}\n", fp);

  return fclose(fp);
}

}  // namespace

/*
 * Raw motion-object dump.  This intentionally lives outside the anonymous
 * namespace: the motion classes declare this writer as a friend so every
 * private/protected data member can be included without adding diagnostic
 * getters to the realtime API.
 */
class ecmcMotionStateYamlWriter {
public:
  explicit ecmcMotionStateYamlWriter(std::ostream& out) : out_(out) {}

  void dumpAll() {
    value("format", "ecmc-motion-object-state-v1");
    value("globals.appModeCmd", static_cast<long long>(appModeCmd));
    value("globals.appModeStat", static_cast<long long>(appModeStat));
    value("globals.controllerError", controllerError);
    value("globals.ecmcUpdatedCounter", ecmcUpdatedCounter);
    value("globals.currentMasterSlaveSMIndex", currentMasterSlaveSMIndex);

    for (int i = 0; i < ECMC_MAX_AXES; ++i) {
      if (axes[i]) {
        std::ostringstream prefix;
        prefix << "axes[" << i << "]";
        dumpAxis(*axes[i], prefix.str());
      }
      if (axisGroups[i]) {
        std::ostringstream prefix;
        prefix << "axisGroups[" << i << "]";
        dumpAxisGroup(*axisGroups[i], prefix.str());
      }
    }
    for (int i = 0; i < ECMC_MAX_MST_SLVS_SMS; ++i) {
      if (masterSlaveSMs[i]) {
        std::ostringstream prefix;
        prefix << "masterSlaveSMs[" << i << "]";
        dumpMasterSlave(*masterSlaveSMs[i], prefix.str());
      }
    }
    pointer("pvtController", pvtCtrl_);
    if (pvtCtrl_) dumpPvtController(*pvtCtrl_, "pvtController");
    pointer("motorRecordController", asynPortMotorRecord);
    if (asynPortMotorRecord) {
      dumpMotorRecordController(*asynPortMotorRecord, "motorRecordController");
      for (int i = 0; i < ECMC_MAX_AXES; ++i) {
        ecmcMotorRecordAxis *mrAxis = asynPortMotorRecord->getAxis(i);
        if (mrAxis) {
          std::ostringstream prefix;
          prefix << "motorRecordAxes[" << i << "]";
          dumpMotorRecordAxis(*mrAxis, prefix.str());
        }
      }
    }
  }

private:
  std::ostream& out_;

  void key(const std::string& name) { out_ << name << ": "; }
  void value(const std::string& name, bool v) {
    key(name); out_ << (v ? "true" : "false") << '\n';
  }
  void value(const std::string& name, const char *v) {
    key(name); out_ << '"';
    if (v) {
      for (const char *p = v; *p; ++p) {
        if (*p == '\\' || *p == '"') out_ << '\\';
        if (*p == '\n') out_ << "\\n";
        else if (*p == '\r') out_ << "\\r";
        else out_ << *p;
      }
    }
    out_ << "\"\n";
  }
  void value(const std::string& name, const std::string& v) {
    value(name, v.c_str());
  }
  template<size_t N>
  void value(const std::string& name, const char (&v)[N]) {
    key(name); out_ << '"';
    for (size_t i = 0; i < N && v[i]; ++i) {
      if (v[i] == '\\' || v[i] == '"') out_ << '\\';
      if (v[i] == '\n') out_ << "\\n";
      else if (v[i] == '\r') out_ << "\\r";
      else out_ << v[i];
    }
    out_ << "\"\n";
  }
  template<typename T>
  void value(const std::string& name, const T& v) {
    key(name); out_ << v << '\n';
  }
  template<typename T>
  void pointer(const std::string& name, T *v) {
    key(name);
    if (!v) out_ << "null\n";
    else out_ << "\"0x" << std::hex << reinterpret_cast<uintptr_t>(v)
              << std::dec << "\"\n";
  }
  static std::string child(const std::string& prefix, const char *name) {
    return prefix + "." + name;
  }

  void dumpErrorBase(const ecmcError& o, const std::string& p) {
    value(child(p, "errorPath_"), o.errorPath_);
    value(child(p, "errorPathValid_"), o.errorPathValid_);
    value(child(p, "error_"), o.error_); value(child(p, "errorId_"), o.errorId_);
    value(child(p, "warningId_"), o.warningId_); value(child(p, "warning_"), o.warning_);
    value(child(p, "currSeverity_"), static_cast<long long>(o.currSeverity_));
    pointer(child(p, "warningPtr_"), o.warningPtr_); pointer(child(p, "errorPtr_"), o.errorPtr_);
    value(child(p, "errorsInBuffer_"), o.errorsInBuffer_);
    value(child(p, "bufferIndex_"), o.bufferIndex_);
    for (size_t i = 0; i < o.buffer_.size(); ++i) {
      std::ostringstream n; n << p << ".buffer_[" << i << "]";
      value(n.str(), o.buffer_[i]);
    }
  }

  void dumpEcEntryLink(const ecmcEcEntryLink& o, const std::string& p) {
    dumpErrorBase(static_cast<const ecmcError&>(o), child(p, "errorBase"));
    for (int i = 0; i < ECMC_EC_ENTRY_LINKS_MAX; ++i) {
      std::ostringstream ep, bp;
      ep << p << ".entryInfoArray_[" << i << "].entry";
      bp << p << ".entryInfoArray_[" << i << "].bitNumber";
      pointer(ep.str(), o.entryInfoArray_[i].entry);
      value(bp.str(), o.entryInfoArray_[i].bitNumber);
    }
  }

  void dumpCommand(const ecmcAxisDataCommand& o, const std::string& p) {
#define D(name) value(child(p, #name), o.name)
#define E(name) value(child(p, #name), static_cast<long long>(o.name))
#define P(name) pointer(child(p, #name), o.name)
    D(positionTarget); D(velocityTarget); D(tweakValue); D(softLimitBwd);
    D(softLimitFwd); D(moduloRange); E(moduloType); E(command); D(cmdData);
    D(primaryEncIndex); D(cspDrvEncIndex); D(cfgEncIndex); D(drvModeSet);
    D(accelerationTarget); D(decelerationTarget); D(allowSourceChangeWhenEnabled);
    D(enableAtStartup); E(drvMode); P(asynDataItemCtrlWord);
    const ecmcAsynAxisControlType& w = o.controlWord_;
    const std::string wp = child(p, "controlWord_");
    value(child(wp, "enableCmd"), static_cast<bool>(w.enableCmd));
    value(child(wp, "executeCmd"), static_cast<bool>(w.executeCmd));
    value(child(wp, "stopCmd"), static_cast<bool>(w.stopCmd));
    value(child(wp, "resetCmd"), static_cast<bool>(w.resetCmd));
    value(child(wp, "encSourceCmd"), static_cast<bool>(w.encSourceCmd));
    value(child(wp, "trajSourceCmd"), static_cast<bool>(w.trajSourceCmd));
    value(child(wp, "plcEnableCmd"), static_cast<bool>(w.plcEnableCmd));
    value(child(wp, "plcCmdsAllowCmd"), static_cast<bool>(w.plcCmdsAllowCmd));
    value(child(wp, "enableSoftLimitBwd"), static_cast<bool>(w.enableSoftLimitBwd));
    value(child(wp, "enableSoftLimitFwd"), static_cast<bool>(w.enableSoftLimitFwd));
    value(child(wp, "enableDbgPrintout"), static_cast<bool>(w.enableDbgPrintout));
    value(child(wp, "tweakBwdCmd"), static_cast<bool>(w.tweakBwdCmd));
    value(child(wp, "tweakFwdCmd"), static_cast<bool>(w.tweakFwdCmd));
    value(child(wp, "blockCom"), static_cast<bool>(w.blockCom));
    value(child(wp, "spareBitsCmd"), static_cast<long long>(w.spareBitsCmd));
#undef P
#undef E
#undef D
  }

  void dumpStatus(const ecmcAxisDataStatus& o, const std::string& p) {
#define D(name) value(child(p, #name), o.name)
#define E(name) value(child(p, #name), static_cast<long long>(o.name))
    D(externalTrajectoryPosition); D(externalTrajectoryVelocity);
    D(externalEncoderPosition); D(externalEncoderVelocity);
    D(currentPositionActual); D(currentPositionSetpoint);
    D(currentCSPPositionSetpointOffset); D(currentTargetPosition);
    D(currentTargetPositionModulo); D(currentVelocityActual);
    D(currentVelocitySetpoint); D(currentVelocitySetpointRaw);
    D(currentVelocityTarget); D(currentPositionSetpointRaw);
    D(currentPositionActualRaw); D(currentvelocityFFRaw); D(cntrlError);
    D(cntrlOutput); D(cntrlOutputOld); D(currentAccelerationSetpoint);
    D(currentDecelerationSetpoint); D(command); D(cmdData); D(encoderCount);
    D(ctrlWithinDeadband); D(limitFwdFiltered); D(limitBwdFiltered);
    D(homeSwitchFiltered); D(startupFinsished); D(distToStop); D(errorCode);
    D(warningCode); D(axisId); D(cycleCounter); E(axisType); D(sampleTime);
    const ecmcAxisStatusWordType& w = o.statusWord_;
    const std::string wp = child(p, "statusWord_");
#define B(name) value(child(wp, #name), static_cast<bool>(w.name))
    B(enable); B(enabled); B(execute); B(busy); B(attarget); B(moving);
    B(limitfwd); B(limitbwd); B(homeswitch); B(homed); B(inrealtime);
    B(trajsource); B(encsource); B(plccmdallowed); B(softlimfwdena);
    B(softlimbwdena); B(instartup); B(sumilockfwd); B(sumilockbwd);
    B(softlimilockfwd); B(softlimilockbwd); B(localBusy); B(globalBusy);
    B(blocked);
    value(child(wp, "seqstate"), static_cast<long long>(w.seqstate));
    value(child(wp, "lastilock"), static_cast<long long>(w.lastilock));
#undef B
#undef E
#undef D
  }

  void dumpInterlocks(const ecmcAxisDataInterlocks& o, const std::string& p) {
#define D(name) value(child(p, #name), o.name)
#define E(name) value(child(p, #name), static_cast<long long>(o.name))
    D(hardwareInterlock); D(bwdLimitInterlock); D(fwdLimitInterlock);
    D(fwdSoftLimitInterlock); D(bwdSoftLimitInterlock);
    D(cntrlOutputHLTrajInterlock); D(cntrlOutputHLDriveInterlock);
    D(lagTrajInterlock); D(lagDriveInterlock); D(bothLimitsLowInterlock);
    D(maxVelocityTrajInterlock); D(maxVelocityDriveInterlock);
    D(velocityDiffTrajInterlock); D(velocityDiffDriveInterlock);
    D(axisErrorStateInterlock); D(noExecuteInterlock); D(driveSummaryInterlock);
    D(trajSummaryInterlockFWD); D(trajSummaryInterlockBWD);
    D(trajSummaryInterlockFWDEpics); D(trajSummaryInterlockBWDEpics);
    D(etherCatMasterInterlock); D(plcInterlock); D(plcInterlockFWD);
    D(plcInterlockBWD); D(encDiffInterlock); D(safetyInterlock);
    D(analogInterlock); D(stallInterlock); E(lastActiveInterlock);
    E(interlockStatus); E(currStopMode);
#undef E
#undef D
  }

  void dumpAxisData(const ecmcAxisData& o, const std::string& p) {
    dumpCommand(o.control_, child(p, "control_"));
    dumpCommand(o.controlOld_, child(p, "controlOld_"));
    dumpStatus(o.status_, child(p, "status_"));
    dumpStatus(o.statusOld_, child(p, "statusOld_"));
    dumpInterlocks(o.interlocks_, child(p, "interlocks_"));
    dumpInterlocks(o.interlocksOld_, child(p, "interlocksOld_"));
    value(child(p, "encoderDiffConfigChanged_"), o.encoderDiffConfigChanged_);
    for (int i = 0; i < ECMC_ASYN_AX_PAR_COUNT; ++i) {
      std::ostringstream n; n << p << ".axAsynParams_[" << i << "]";
      pointer(n.str(), o.axAsynParams_[i]);
    }
  }

  void dumpFilter(const ecmcFilter& o, const std::string& p) {
    dumpErrorBase(static_cast<const ecmcError&>(o), child(p, "errorBase"));
    pointer(child(p, "bufferVel_"), o.bufferVel_);
    value(child(p, "my_"), o.my_); value(child(p, "last_"), o.last_);
    value(child(p, "lastOutput_"), o.lastOutput_);
    value(child(p, "posSum_"), o.posSum_);
    value(child(p, "posPrevWrapped_"), o.posPrevWrapped_);
    value(child(p, "posUnwrapped_"), o.posUnwrapped_);
    value(child(p, "sampleTime_"), o.sampleTime_);
    value(child(p, "indexVel_"), static_cast<unsigned long long>(o.indexVel_));
    value(child(p, "filterSize_"), static_cast<unsigned long long>(o.filterSize_));
    value(child(p, "veloSumValid_"), o.veloSumValid_);
    value(child(p, "posStateValid_"), o.posStateValid_);
    if (o.bufferVel_) {
      for (size_t i = 0; i < o.filterSize_; ++i) {
        std::ostringstream n; n << p << ".bufferVel_[" << i << "]";
        value(n.str(), o.bufferVel_[i]);
      }
    }
  }

  void dumpEncoder(const ecmcEncoder& o, const std::string& p) {
    dumpEcEntryLink(static_cast<const ecmcEcEntryLink&>(o), child(p, "ecEntryLinkBase"));
#define D(name) value(child(p, #name), o.name)
#define E(name) value(child(p, #name), static_cast<long long>(o.name))
#define P(name) pointer(child(p, #name), o.name)
    E(encType_); P(velocityFilter_); P(positionFilter_); P(data_); D(turns_);
    D(rawPosUint_); D(rawPosUintOld_); D(rawAbsPosUint_); D(rawAbsPosUintOld_);
    D(totalRawMask_); D(totalRawRegShift_); D(rawLimit_); D(rawAbsLimit_);
    D(rawPosDouble_); D(rawPosDoubleOld_); D(rawPosMultiTurn_); D(rawPosOffset_);
    D(rawRange_); D(rawAbsRange_); D(rawTurns_); D(rawTurnsOld_); D(bits_);
    D(absBits_); D(scaleNum_); D(scaleDenom_); D(scale_); D(invScale_);
    D(engOffset_); D(actPos_); D(actPosLocal_); D(actPosUncompensated_);
    D(actPosOld_); D(actPosDelayBaseOld_); D(sampleTimeMs_); D(invSampleTime_);
    D(actVel_); D(actVelLocal_); D(homed_); D(encLatchFunctEnabled_);
    D(encLatchStatus_); D(encLatchStatusOld_); D(rawAbsPosDouble_);
    D(rawAbsPosDoubleOld_); D(rawEncLatchPos_); D(rawEncLatchPosMultiTurn_);
    D(encLatchControlWordArm_); D(encLatchControlWordIdle_); D(encLatchControlBits_);
    D(encLatchControlEnabled_); D(encLatchControlDisablePending_); D(encLatchArm_);
    D(latchIdleReadPending_); D(touchProbeAutoRearm_); D(touchProbeRearmState_);
    D(actEncLatchPos_); D(encLatchSequence_); D(encLatchTimestampRaw_);
    D(encLatchTimestampBits_); D(encLatchTimestampValid_); D(touchProbeFunctEnabled_);
    D(touchProbeStatus_); D(touchProbeStatusOld_); D(rawTouchProbePos_);
    D(rawTouchProbePosMultiTurn_); D(touchProbeControlWordArm_);
    D(touchProbeControlWordIdle_); D(touchProbeControlBits_);
    D(touchProbeControlEnabled_); D(touchProbeControlDisablePending_);
    D(touchProbeArm_); D(touchProbeIdleReadPending_); D(actTouchProbePos_);
    D(touchProbeSequence_); D(touchProbeTimestampRaw_); D(touchProbeTimestampBits_);
    D(touchProbeTimestampValid_); D(enablePositionFilter_); D(enableVelocityFilter_);
    D(hwReset_); D(hwErrorAlarm0_); D(hwErrorAlarm0Old_); D(hwErrorAlarm1_);
    D(hwErrorAlarm1Old_); D(hwErrorAlarm2_); D(hwErrorAlarm2Old_); D(hwWarning_);
    D(hwWarningOld_); D(hwReady_); D(hwReadyOld_); D(hwActPosDefined_);
    D(hwResetDefined_); D(hwErrorAlarm0Defined_); D(hwErrorAlarm1Defined_);
    D(hwErrorAlarm2Defined_); D(hwWarningDefined_); D(hwReadyBitDefined_);
    D(masterOKOld_); D(refEncIndex_); D(refDuringHoming_); D(homeLatchCountOffset_);
    D(maxPosDiffToPrimEnc_); D(encInitilized_); D(hwSumAlarm_); D(hwSumAlarmOld_);
    D(hwTriggedHomingEnabled_); D(encLocalErrorId_); D(encLocalErrorIdOld_);
    P(asynPortDriver_); P(encPosAct_); P(encVelAct_); P(encErrId_);
    D(touchProbeAsynParamsCreated_); D(touchProbeAsynValid_);
    D(touchProbeAsynSequence_); D(touchProbeAsynPosition_);
    D(touchProbeAsynTimestampValid_); D(touchProbeAsynTimestampRaw_);
    D(touchProbeAsynEventTimeNs_); D(touchProbeAsynArmed_); D(touchProbeAsynArmCmd_);
    P(touchProbeAsynValidParam_); P(touchProbeAsynSequenceParam_);
    P(touchProbeAsynPositionParam_); P(touchProbeAsynTimestampValidParam_);
    P(touchProbeAsynTimestampRawParam_); P(touchProbeAsynEventTimeParam_);
    P(touchProbeAsynArmedParam_); P(touchProbeAsynArmCmdParam_);
    D(hwReadyInvert_); D(index_); D(homeParamsValid_); D(homeVelTowardsCam_);
    D(homeVelOffCam_); D(homePosition_); D(homeSeqId_); D(homeEnablePostMove_);
    D(homePostMoveTargetPos_); D(homeAcc_); D(homeDec_); D(domainOK_);
    D(lookupTableEnable_); P(lookupTable_); D(lookupTableRange_);
    D(lookupTableScale_); D(delayTimeS_); D(enableDelayTime_);
    D(delayCompStateValid_); D(delayCompAbsScale_); D(delayCompMinTrustedVel_);
    D(delayCompMaxDistance_); D(allowOverUnderFlow_);
    if (o.velocityFilter_) dumpFilter(*o.velocityFilter_, child(p, "velocityFilter"));
    if (o.positionFilter_) dumpFilter(*o.positionFilter_, child(p, "positionFilter"));
#undef P
#undef E
#undef D
  }

  void dumpMonitor(const ecmcMonitor& o, const std::string& p) {
    dumpEcEntryLink(static_cast<const ecmcEcEntryLink&>(o), child(p, "ecEntryLinkBase"));
#define D(name) value(child(p, #name), o.name)
#define E(name) value(child(p, #name), static_cast<long long>(o.name))
#define P(name) pointer(child(p, #name), o.name)
    D(enable_); D(atTargetTol_); D(atTargetTime_); D(enableAtTargetMon_);
    D(posLagTol_); D(posLagTime_); D(enableLagMon_); D(atTargetCounter_);
    D(lagMonCounter_); D(maxVel_); D(enableMaxVelMon_); D(maxVelCounterDrive_);
    D(maxVelCounterTraj_); D(maxVelDriveILDelay_); D(maxVelTrajILDelay_);
    D(enableHardwareInterlock_); D(cntrlOutputHL_); D(enableCntrlHLMon_);
    D(enableVelocityDiffMon_); D(velocityDiffCounter_); D(velDiffTimeTraj_);
    D(velDiffTimeDrive_); D(velDiffMaxDiff_); D(enableAlarmAtHardlimitBwd_);
    D(enableAlarmAtHardlimitFwd_); P(data_); D(switchFilterCounter_);
    D(limitFwdFilterSum_); D(limitBwdFilterSum_); D(homeFilterSum_);
    for (int i = 0; i < ECMC_MON_SWITCHES_FILTER_CYCLES; ++i) {
      std::ostringstream a, b, c;
      a << p << ".limitFwdFilterBuffer_[" << i << "]";
      b << p << ".limitBwdFilterBuffer_[" << i << "]";
      c << p << ".homeFilterBuffer_[" << i << "]";
      value(a.str(), o.limitFwdFilterBuffer_[i]);
      value(b.str(), o.limitBwdFilterBuffer_[i]);
      value(c.str(), o.homeFilterBuffer_[i]);
    }
    D(latchOnLimit_); D(enableAlarmOnSofLimits_); E(hardwareInterlockPolarity_);
    E(lowLimPolarity_); E(highLimPolarity_); E(homePolarity_); P(encArray_);
    for (int i = 0; i < ECMC_MAX_ENCODERS; ++i) {
      std::ostringstream n; n << p << ".diffEncArray_[" << i << "]";
      pointer(n.str(), o.diffEncArray_[i]);
    }
    D(diffEncCount_); D(diffEncPrimaryIndex_); D(diffEncConfiguredCount_);
    D(enableDiffEncsMon_); D(ctrlDeadbandTol_); D(ctrlDeadbandCounter_);
    D(ctrlDeadbandTime_); D(slvCtrlDeadbandTol_); D(slvCtrlDeadbandCounter_);
    D(slvCtrlDeadbandTime_); D(axisIsWithinSlvCtrlDB_); D(analogRawLimit_);
    D(analogRawValue_); D(enableAnalogInterlock_); E(analogPolarity_);
    D(stallTimeFactor_); D(enableStallMon_); D(maxStallCounter_);
    D(stallLastMotionCmdCycles_); D(stallCheckAtTargetAtCycle_);
    D(stallMinTimeoutCycles_); D(limitSwitchFwdPLCOverride_);
    D(limitSwitchBwdPLCOverride_); D(limitSwitchFwdPLCOverrideValue_);
    D(limitSwitchBwdPLCOverrideValue_); D(homeSwitchPLCOverride_);
    D(homeSwitchPLCOverrideValue_); D(enableHomeSensor_);
    D(axisIsWithinCtrlDBExtTraj_); D(stopAtAnyLimit_); D(softLimitReenablePending_);
#undef P
#undef E
#undef D
  }

  void dumpTrajectory(const ecmcTrajectoryBase& o, const std::string& p) {
    dumpErrorBase(static_cast<const ecmcError&>(o), child(p, "errorBase"));
#define D(name) value(child(p, #name), o.name)
#define E(name) value(child(p, #name), static_cast<long long>(o.name))
#define P(name) pointer(child(p, #name), o.name)
    D(targetAcceleration_); D(targetDeceleration_); D(targetDecelerationEmerg_);
    D(targetVelocity_); D(targetPosition_); D(targetJerk_); D(sampleTime_);
    D(invSampleTime_); D(posSetMinus1_); D(currentPositionSetpoint_);
    D(currentVelocitySetpoint_); D(currentAccelerationSetpoint_); D(busy_);
    D(index_); D(execute_); D(executeOld_); D(startPosition_); D(enable_);
    D(enableOld_); D(internalStopCmd_); D(distToStop_); E(motionMode_);
    E(interlockStatus_); P(data_); E(latchedStopMode_); D(externalVeloLimit_);
    D(externalVeloLimitActive_);
    const ecmcTrajectoryTrapetz *trap = dynamic_cast<const ecmcTrajectoryTrapetz *>(&o);
    if (trap) {
      const std::string q = child(p, "trapetz");
      value(child(q, "stepACC_"), trap->stepACC_);
      value(child(q, "stepDEC_"), trap->stepDEC_);
      value(child(q, "stepNOM_"), trap->stepNOM_);
      value(child(q, "stepDECEmerg_"), trap->stepDECEmerg_);
      value(child(q, "prevStepSize_"), trap->prevStepSize_);
      value(child(q, "thisStepSize_"), trap->thisStepSize_);
      value(child(q, "stepStableTol_"), trap->stepStableTol_);
      value(child(q, "localCurrentPositionSetpoint_"), trap->localCurrentPositionSetpoint_);
      value(child(q, "targetPositionLocal_"), trap->targetPositionLocal_);
      value(child(q, "localBusy_"), trap->localBusy_);
    }
    const ecmcTrajectoryS *s = dynamic_cast<const ecmcTrajectoryS *>(&o);
    if (s) {
      const std::string q = child(p, "s_curve");
      pointer(child(q, "otg_"), s->otg_); pointer(child(q, "input_"), s->input_);
      pointer(child(q, "output_"), s->output_); value(child(q, "stepNOM_"), s->stepNOM_);
      value(child(q, "localCurrentPositionSetpoint_"), s->localCurrentPositionSetpoint_);
      value(child(q, "targetPositionLocal_"), s->targetPositionLocal_);
      value(child(q, "targetVelocityLocal_"), s->targetVelocityLocal_);
      value(child(q, "localBusy_"), s->localBusy_);
      value(child(q, "trajMaxVelo_"), s->trajMaxVelo_);
    }
#undef P
#undef E
#undef D
  }

  void dumpPid(const ecmcPIDController& o, const std::string& p) {
    dumpErrorBase(static_cast<const ecmcError&>(o), child(p, "errorBase"));
#define D(name) value(child(p, #name), o.name)
#define P(name) pointer(child(p, #name), o.name)
    D(kp_); D(ki_); D(kd_); D(kff_); D(kp_inner_); D(ki_inner_);
    D(kd_inner_); D(innerTol_); D(kp_use_); D(ki_use_); D(kd_use_);
    D(outputP_); D(outputI_); D(outputD_); D(outputIMax_); D(outputIMin_);
    D(outputMax_); D(outputMin_); D(ff_); D(controllerErrorOld_);
    D(sampleTime_); P(data_); D(settingMade_); D(resetIAtTrajBusy_);
    D(freezeIAtTrajBusy_); P(asynPortDriver_); P(asynKp_); P(asynKi_);
    P(asynKd_); P(asynKff_);
#undef P
#undef D
  }

  void dumpSequencer(const ecmcAxisSequencer& o, const std::string& p) {
    dumpErrorBase(static_cast<const ecmcError&>(o), child(p, "errorBase"));
#define D(name) value(child(p, #name), o.name)
#define E(name) value(child(p, #name), static_cast<long long>(o.name))
#define P(name) pointer(child(p, #name), o.name)
    D(seqState_); D(seqStateOld_); D(seqTimeout_); D(seqTimeCounter_);
    D(seqPosHomeState_); D(hwLimitSwitchFwd_); D(hwLimitSwitchFwdOld_);
    D(hwLimitSwitchBwd_); D(hwLimitSwitchBwdOld_); D(homeSensor_);
    D(homeSensorOld_); D(seqInProgress_); D(seqInProgressOld_); D(jogFwd_);
    D(jogBwd_); D(executeOld_); D(localSeqBusy_); D(homeEnablePostMove_);
    D(homePostMoveTargetPos_); D(jogVel_); D(homeVelTowardsCam_);
    D(homeVelOffCam_); D(homePosition_); D(homePosLatch1_); D(homePosLatch2_);
    D(homeAcc_); D(homeDec_); E(currSeqDirection_); P(traj_); P(encArray_);
    P(mon_); P(cntrl_); P(drv_); P(data_); P(pvt_); D(oldencRawAbsPosReg_);
    D(encRawAbsPosReg_); E(overUnderFlowLatch_); D(homeLatchCountOffset_);
    D(homeLatchCountAct_); D(enablePos_); D(enableConstVel_); D(enableHome_);
    P(modeSetEntry_); P(modeActEntry_); D(modeAct_); D(modeMotionCmd_);
    D(modeHomingCmd_); D(modeMotionCmdSet_); D(modeHomingCmdSet_); D(pvtOk_);
    D(trajLock_); D(trajLockOld_); D(pvtStopping_); D(pvtmode_); D(posSource_);
    D(homeTrigStatOld_); D(monPosLagEnaStatePriorHome_);
    D(monPosLagRestoreNeeded_); D(globalBusy_);
#undef P
#undef E
#undef D
  }

  void dumpPvtSequence(const ecmcAxisPVTSequence& o, const std::string& p) {
#define D(name) value(child(p, #name), o.name)
#define E(name) value(child(p, #name), static_cast<long long>(o.name))
#define P(name) pointer(child(p, #name), o.name)
    D(segmentCount_); D(pointCount_); D(currSegIndex_); D(currSegIndexOld_);
    D(totalTime_); D(sampleTime_); D(halfSampleTime_); D(currTime_);
    D(firstSegTime_); D(nextTime_); D(busy_); D(positionOffset_); D(execute_);
    D(executeOld_); P(data_); D(relativeMode_); E(trgMode_);
    for (size_t i = 0; i < o.points_.size(); ++i) {
      std::ostringstream q; q << p << ".points_[" << i << "]";
      pointer(q.str(), o.points_[i]);
      if (o.points_[i]) {
        value(child(q.str(), "position_"), o.points_[i]->position_);
        value(child(q.str(), "velocity_"), o.points_[i]->velocity_);
        value(child(q.str(), "time_"), o.points_[i]->time_);
      }
    }
    for (size_t i = 0; i < o.segments_.size(); ++i) {
      std::ostringstream q; q << p << ".segments_[" << i << "]";
      ecmcPvtSegment *s = o.segments_[i]; pointer(q.str(), s);
      if (s) {
        pointer(child(q.str(), "startPnt_"), s->startPnt_);
        pointer(child(q.str(), "endPnt_"), s->endPnt_);
        value(child(q.str(), "k0_"), s->k0_); value(child(q.str(), "k1_"), s->k1_);
        value(child(q.str(), "k2_"), s->k2_); value(child(q.str(), "k3_"), s->k3_);
        value(child(q.str(), "timeSpan_"), s->timeSpan_);
        value(child(q.str(), "range_"), s->range_);
        value(child(q.str(), "timeInSeg_"), s->timeInSeg_);
        value(child(q.str(), "timeInSegPow2_"), s->timeInSegPow2_);
      }
    }
    for (size_t i = 0; i < o.resultPosActArray_.size(); ++i) {
      std::ostringstream n; n << p << ".resultPosActArray_[" << i << "]";
      value(n.str(), o.resultPosActArray_[i]);
    }
    for (size_t i = 0; i < o.resultPosErrArray_.size(); ++i) {
      std::ostringstream n; n << p << ".resultPosErrArray_[" << i << "]";
      value(n.str(), o.resultPosErrArray_[i]);
    }
#undef P
#undef E
#undef D
  }

  void dumpPvtController(const ecmcPVTController& o, const std::string& p) {
    dumpEcEntryLink(static_cast<const ecmcEcEntryLink&>(o), child(p, "ecEntryLinkBase"));
#define D(name) value(child(p, #name), o.name)
#define E(name) value(child(p, #name), static_cast<long long>(o.name))
#define P(name) pointer(child(p, #name), o.name)
    D(sampleTime_); D(nextTime_); D(endTime_); D(executeOld_); D(execute_);
    E(state_); E(stateOld_); D(busy_); D(triggerDefined_); D(triggerValidatedOK_);
    D(triggerEcEntryIndex_); D(triggerCount_); D(triggerStartTime_);
    D(triggerEndTime_); D(triggerTimeBetween_); D(triggerDuration_);
    D(triggerCurrentId_); D(newTrg_); D(triggerOutputHigh_); D(axesBusyState_);
    D(axesBusyStateValid_); D(halfSampleTime_); P(asynPortDriver_);
    P(asynSoftTrigger_); D(softTrigger_);
    for (size_t i = 0; i < o.startPositions_.size(); ++i) {
      std::ostringstream n; n << p << ".startPositions_[" << i << "]";
      value(n.str(), o.startPositions_[i]);
    }
    for (size_t i = 0; i < o.axes_.size(); ++i) {
      std::ostringstream n; n << p << ".axes_[" << i << "]";
      pointer(n.str(), o.axes_[i]);
    }
    for (size_t i = 0; i < o.pvtObjs_.size(); ++i) {
      std::ostringstream n; n << p << ".pvtObjs_[" << i << "]";
      pointer(n.str(), o.pvtObjs_[i]);
      if (o.pvtObjs_[i]) dumpPvtSequence(*o.pvtObjs_[i], n.str());
    }
#undef P
#undef E
#undef D
  }

  void dumpDrive(const ecmcDriveBase& o, const std::string& p) {
    dumpEcEntryLink(static_cast<const ecmcEcEntryLink&>(o), child(p, "ecEntryLinkBase"));
#define D(name) value(child(p, #name), o.name)
#define E(name) value(child(p, #name), static_cast<long long>(o.name))
#define P(name) pointer(child(p, #name), o.name)
    D(enableAmpCmd_); D(enableAmpCmdOld_); D(stateMachineTimeoutCycles_);
    D(scale_); D(invScale_); D(scaleNum_); D(scaleDenom_); D(velSet_);
    D(cspPosSet_); D(controlWord_); D(statusWord_); P(data_); D(masterOK_);
    D(localEnabledOld_); D(brakeOpenDelayTime_); D(brakeCloseAheadTime_);
    D(brakeOutputCmdOld_); D(reduceTorqueOutputCmdOld_); D(enableReduceTorque_);
    D(enableBrake_); D(brakeOutputCmd_); D(reduceTorqueOutputCmd_);
    E(brakeState_); D(brakeCounter_); D(enableCmdOld_); P(asynPortDriver_);
    P(asynControlWd_); P(asynStatusWd_); D(cspRawActPos_); D(cspActPos_);
    D(cspRawPosOffset_); D(hwReset_); D(hwErrorAlarm0_); D(hwErrorAlarm0Old_);
    D(hwErrorAlarm1_); D(hwErrorAlarm1Old_); D(hwErrorAlarm2_);
    D(hwErrorAlarm2Old_); D(hwWarning_); D(hwWarningOld_); D(hwResetDefined_);
    D(hwErrorAlarm0Defined_); D(hwErrorAlarm1Defined_); D(hwErrorAlarm2Defined_);
    D(hwWarningDefined_); D(cycleCounterBase_); D(minVeloOutput_);
    D(maxVeloOutput_); D(veloPosOutput_); D(veloRawOffset_); P(cspEnc_);
    const ecmcDriveDS402 *ds402 = dynamic_cast<const ecmcDriveDS402 *>(&o);
    if (ds402) {
      const std::string q = child(p, "ds402");
      value(child(q, "enableStateMachine_"),
            static_cast<long long>(ds402->enableStateMachine_));
      value(child(q, "enableStateMachineOld_"),
            static_cast<long long>(ds402->enableStateMachineOld_));
      value(child(q, "cycleCounter_"), ds402->cycleCounter_);
      value(child(q, "ds402WarningOld_"), ds402->ds402WarningOld_);
      value(child(q, "localEnabled_"), ds402->localEnabled_);
      value(child(q, "localEnableAmpCmdOld_"), ds402->localEnableAmpCmdOld_);
      value(child(q, "startupFaultCleared_"), ds402->startupFaultCleared_);
    }
    const ecmcDriveStepper *stepper = dynamic_cast<const ecmcDriveStepper *>(&o);
    if (stepper) {
      value(child(p, "stepper.localEnabled_"), stepper->localEnabled_);
    }
#undef P
#undef E
#undef D
  }

  void dumpPositionCompare(const ecmcPositionCompare& o, const std::string& p) {
    dumpEcEntryLink(static_cast<const ecmcEcEntryLink&>(o), child(p, "ecEntryLinkBase"));
#define D(name) value(child(p, #name), o.name)
#define P(name) pointer(child(p, #name), o.name)
    D(minLeadTimeNs_); D(maxLeadTimeNs_); D(activateIdle_); D(activateSchedule_);
    D(pulseWidthNs_); D(resetValue_); D(linked_); D(activateIdlePending_);
    D(asynParamsCreated_); D(asynState_); D(asynReason_); D(asynDirection_);
    D(asynTargetCmd_); D(asynDirectionCmd_); D(asynOutputCmd_); D(asynArmCmd_);
    D(asynCancelCmd_); P(asynStateParam_); P(asynReasonParam_);
    P(asynSequenceParam_); P(asynTargetParam_); P(asynPositionParam_);
    P(asynVelocityParam_); P(asynAccelerationParam_); P(asynDirectionParam_);
    P(asynScheduledTimeParam_); P(asynLeadTimeParam_); P(asynSampleAgeParam_);
    P(asynPulseWidthParam_); P(asynResetTimeParam_); P(asynLastActivateParam_);
    P(asynLastOutputParam_); P(asynTargetCmdParam_); P(asynDirectionCmdParam_);
    P(asynOutputCmdParam_); P(asynArmCmdParam_); P(asynCancelCmdParam_);
    const ecmcPositionCompareStatus& s = o.status_;
    const std::string sp = child(p, "status_");
    value(child(sp, "state"), static_cast<long long>(s.state));
    value(child(sp, "reason"), static_cast<long long>(s.reason));
    value(child(sp, "sequence"), s.sequence); value(child(sp, "target"), s.target);
    value(child(sp, "position"), s.position); value(child(sp, "velocity"), s.velocity);
    value(child(sp, "acceleration"), s.acceleration); value(child(sp, "direction"), s.direction);
    value(child(sp, "scheduledTimeNs"), s.scheduledTimeNs);
    value(child(sp, "leadTimeNs"), s.leadTimeNs); value(child(sp, "sampleAgeNs"), s.sampleAgeNs);
    value(child(sp, "outputValue"), s.outputValue); value(child(sp, "resetValue"), s.resetValue);
    value(child(sp, "pulseWidthNs"), s.pulseWidthNs); value(child(sp, "resetTimeNs"), s.resetTimeNs);
    value(child(sp, "sampleTimeNs"), s.sampleTimeNs);
    value(child(sp, "controllerTimeNs"), s.controllerTimeNs);
    value(child(sp, "eventTimeNs"), s.eventTimeNs);
    value(child(sp, "lastActivateValue"), s.lastActivateValue);
    value(child(sp, "lastOutputValue"), s.lastOutputValue);
#undef P
#undef D
  }

  void dumpAxis(const ecmcAxisBase& o, const std::string& p) {
    dumpErrorBase(static_cast<const ecmcError&>(o), child(p, "errorBase"));
#define D(name) value(child(p, #name), o.name)
#define E(name) value(child(p, #name), static_cast<long long>(o.name))
#define P(name) pointer(child(p, #name), o.name)
    P(traj_); P(mon_);
    for (int i = 0; i < ECMC_MAX_ENCODERS; ++i) {
      std::ostringstream n; n << p << ".encArray_[" << i << "]";
      pointer(n.str(), o.encArray_[i]);
    }
    E(axisState_); P(asynPortDriver_); P(statusOutputEntry_);
    P(extTrajVeloFilter_); P(extEncVeloFilter_); D(allowCmdFromOtherPLC_);
    D(executeCmdOld_); D(enableExtTrajVeloFilter_); D(enableExtEncVeloFilter_);
    D(disableAxisAtErrorReset_); D(firstEnableDone_);
    value(child(p, "diagBuffer_"), o.diagBuffer_); D(printHeaderCounter_);
    D(setEncoderPos_); E(currentTrajType_); D(encPrimIndexAsyn_); D(hwReadyOld_);
    D(hwReady_); D(globalBusy_); D(ignoreMRDisableStatusCheck_);
    D(autoEnableTimoutS_); D(autoDisableAfterS_); D(autoEnableRequest_);
    D(autoEnableTimeCounter_); D(autoDisbleTimeCounter_); D(enableAutoEnable_);
    D(enableAutoDisable_); D(autoDisableAtTgtLtchEnable_);
    D(autoDisableAtTargetLatched_); D(positionTargetAsyn_); D(invSampleTime_);
    D(masterSlaveBlocked_); D(enableAutoResetError_);
    D(motionCommandMotorRecordRequestCounter_); D(motionCommandRequestCounter_);
    D(motionCommandExecuteCounter_);
    value(child(p, "mrCmds_.stopMRCmdTgl"), static_cast<bool>(o.mrCmds_.stopMRCmdTgl));
    value(child(p, "mrCmds_.stopMRVal"), static_cast<bool>(o.mrCmds_.stopMRVal));
    value(child(p, "mrCmds_.syncMRCmdTgl"), static_cast<bool>(o.mrCmds_.syncMRCmdTgl));
    value(child(p, "mrCmds_.syncMRVal"), static_cast<bool>(o.mrCmds_.syncMRVal));
    value(child(p, "mrCmds_.cnenMRCmdTgl"), static_cast<bool>(o.mrCmds_.cnenMRCmdTgl));
    value(child(p, "mrCmds_.cnenMRVal"), static_cast<bool>(o.mrCmds_.cnenMRVal));
    value(child(p, "mrCmds_.dummy"), static_cast<unsigned long long>(o.mrCmds_.dummy));
    value(child(p, "mrCmdsOld_.stopMRCmdTgl"), static_cast<bool>(o.mrCmdsOld_.stopMRCmdTgl));
    value(child(p, "mrCmdsOld_.stopMRVal"), static_cast<bool>(o.mrCmdsOld_.stopMRVal));
    value(child(p, "mrCmdsOld_.syncMRCmdTgl"), static_cast<bool>(o.mrCmdsOld_.syncMRCmdTgl));
    value(child(p, "mrCmdsOld_.syncMRVal"), static_cast<bool>(o.mrCmdsOld_.syncMRVal));
    value(child(p, "mrCmdsOld_.cnenMRCmdTgl"), static_cast<bool>(o.mrCmdsOld_.cnenMRCmdTgl));
    value(child(p, "mrCmdsOld_.cnenMRVal"), static_cast<bool>(o.mrCmdsOld_.cnenMRVal));
    value(child(p, "mrCmdsOld_.dummy"), static_cast<unsigned long long>(o.mrCmdsOld_.dummy));
    dumpAxisData(o.data_, child(p, "data_"));
    dumpPositionCompare(o.positionCompare_, child(p, "positionCompare_"));
    dumpSequencer(o.seq_, child(p, "seq_"));
    if (o.traj_) dumpTrajectory(*o.traj_, child(p, "traj"));
    if (o.mon_) dumpMonitor(*o.mon_, child(p, "monitor"));
    if (o.extTrajVeloFilter_)
      dumpFilter(*o.extTrajVeloFilter_, child(p, "extTrajVeloFilter"));
    if (o.extEncVeloFilter_)
      dumpFilter(*o.extEncVeloFilter_, child(p, "extEncVeloFilter"));
    for (int i = 0; i < ECMC_MAX_ENCODERS; ++i) {
      if (o.encArray_[i]) {
        std::ostringstream n; n << p << ".encoders[" << i << "]";
        dumpEncoder(*o.encArray_[i], n.str());
      }
    }
    const ecmcAxisReal *real = dynamic_cast<const ecmcAxisReal *>(&o);
    if (real) {
      pointer(child(p, "real.drv_"), real->drv_);
      pointer(child(p, "real.cntrl_"), real->cntrl_);
      value(child(p, "real.currentDriveType_"),
            static_cast<long long>(real->currentDriveType_));
      if (real->cntrl_) dumpPid(*real->cntrl_, child(p, "controller"));
      if (real->drv_) dumpDrive(*real->drv_, child(p, "drive"));
    }
#undef P
#undef E
#undef D
  }

  void dumpAxisGroup(const ecmcAxisGroup& o, const std::string& p) {
    dumpErrorBase(static_cast<const ecmcError&>(o), child(p, "errorBase"));
    value(child(p, "name_"), o.name_); value(child(p, "index_"), o.index_);
    value(child(p, "axesCounter_"), static_cast<unsigned long long>(o.axesCounter_));
    value(child(p, "blocked_"), o.blocked_);
    for (size_t i = 0; i < o.axes_.size(); ++i) {
      std::ostringstream n; n << p << ".axes_[" << i << "]";
      pointer(n.str(), o.axes_[i]);
    }
    for (size_t i = 0; i < o.axesIds_.size(); ++i) {
      std::ostringstream n; n << p << ".axesIds_[" << i << "]";
      value(n.str(), o.axesIds_[i]);
    }
    for (size_t i = 0; i < o.axisInGroup_.size(); ++i) {
      std::ostringstream n; n << p << ".axisInGroup_[" << i << "]";
      value(n.str(), static_cast<bool>(o.axisInGroup_[i]));
    }
  }

  void dumpMotorRecordAxis(const ecmcMotorRecordAxis& o, const std::string& p) {
#define D(name) value(child(p, #name), o.name)
#define P(name) pointer(child(p, #name), o.name)
    P(pC_);
    dumpStatus(o.drvlocal.statusOld_, child(p, "drvlocal.statusOld_"));
    dumpStatus(o.drvlocal.status_, child(p, "drvlocal.status_"));
    pointer(child(p, "drvlocal.ecmcAxis"), o.drvlocal.ecmcAxis);
    value(child(p, "drvlocal.manualVelocSlow"), o.drvlocal.manualVelocSlow);
    value(child(p, "drvlocal.manualVelocFast"), o.drvlocal.manualVelocFast);
    value(child(p, "drvlocal.axisFlags"), o.drvlocal.axisFlags);
    value(child(p, "drvlocal.nErrorIdMcu"), o.drvlocal.nErrorIdMcu);
    value(child(p, "drvlocal.nErrorIdMcuOld"), o.drvlocal.nErrorIdMcuOld);
    value(child(p, "drvlocal.nErrorIdEpicsOld"), o.drvlocal.nErrorIdEpicsOld);
    value(child(p, "drvlocal.bErrorOld"), o.drvlocal.bErrorOld);
    value(child(p, "drvlocal.nCommandActive"), o.drvlocal.nCommandActive);
    value(child(p, "drvlocal.nCommandActiveOld"), o.drvlocal.nCommandActiveOld);
    value(child(p, "drvlocal.homed"), o.drvlocal.homed);
    value(child(p, "drvlocal.axisId"), o.drvlocal.axisId);
    value(child(p, "drvlocal.waitNumPollsBeforeReady"), o.drvlocal.waitNumPollsBeforeReady);
    value(child(p, "drvlocal.illegalInTargetWindow"),
          static_cast<bool>(o.drvlocal.illegalInTargetWindow));
    value(child(p, "drvlocal.old_eeAxisWarning"),
          static_cast<long long>(o.drvlocal.old_eeAxisWarning));
    value(child(p, "drvlocal.eeAxisWarning"),
          static_cast<long long>(o.drvlocal.eeAxisWarning));
    value(child(p, "drvlocal.old_eeAxisError"),
          static_cast<long long>(o.drvlocal.old_eeAxisError));
    value(child(p, "drvlocal.eeAxisError"),
          static_cast<long long>(o.drvlocal.eeAxisError));
    value(child(p, "drvlocal.eeAxisPollNow"),
          static_cast<long long>(o.drvlocal.eeAxisPollNow));
    value(child(p, "drvlocal.dirty.statusDisconnectedOld"),
          static_cast<bool>(o.drvlocal.dirty.statusDisconnectedOld));
    value(child(p, "drvlocal.dirty.sErrorMessage"),
          static_cast<bool>(o.drvlocal.dirty.sErrorMessage));
    value(child(p, "drvlocal.dirty.initialPollNeeded"),
          static_cast<bool>(o.drvlocal.dirty.initialPollNeeded));
    value(child(p, "drvlocal.axisPrintDbg"), o.drvlocal.axisPrintDbg);
    value(child(p, "drvlocal.moveReady"), o.drvlocal.moveReady);
    value(child(p, "drvlocal.moveReadyOld"), o.drvlocal.moveReadyOld);
    value(child(p, "drvlocal.cmdErrorMessage"), o.drvlocal.cmdErrorMessage);
    value(child(p, "drvlocal.sErrorMessage"), o.drvlocal.sErrorMessage);
    value(child(p, "drvlocal.ecmcBusy"), o.drvlocal.ecmcBusy);
    value(child(p, "drvlocal.ecmcSafetyInterlock"), o.drvlocal.ecmcSafetyInterlock);
    value(child(p, "drvlocal.ecmcSummaryInterlock"), o.drvlocal.ecmcSummaryInterlock);
    value(child(p, "drvlocal.ecmcSoftLimitInterlock"), o.drvlocal.ecmcSoftLimitInterlock);
    value(child(p, "drvlocal.ecmcTrjSrc"), o.drvlocal.ecmcTrjSrc);
    value(child(p, "drvlocal.ecmcAtTarget"), o.drvlocal.ecmcAtTarget);
    value(child(p, "drvlocal.ecmcAtTargetMonEnable"), o.drvlocal.ecmcAtTargetMonEnable);
    value(child(p, "drvlocal.axisInStartup"), o.drvlocal.axisInStartup);
    value(child(p, "drvlocal.ecmcIgnoreDisableAxisStatus"), o.drvlocal.ecmcIgnoreDisableAxisStatus);
    value(child(p, "drvlocal.syncEcmcMrSoftlimits"), o.drvlocal.syncEcmcMrSoftlimits);
    value(child(p, "drvlocal.restoreMotorSoftlimits"), o.drvlocal.restoreMotorSoftlimits);
    D(triggstop_); D(triggsync_); P(pvtRunning_); P(pvtPrepare_);
    D(profileCurrentDefinedPoints_); D(profileLastBuildOk_); D(profileLastInitOk_);
    D(profileLastDefineOk_); value(child(p, "profileMessage_"), o.profileMessage_);
    D(profileInProgress_); D(profileSwitchPVTObject_); D(pvtEnabled_);
    D(profileMaxPoints_); D(updateFirstPollDone_); D(interlockStopActive_);
    D(ecmcCycleCounterAtNewCmd_);
    if (o.pvtRunning_) dumpPvtSequence(*o.pvtRunning_, child(p, "pvtRunning"));
    if (o.pvtPrepare_ && o.pvtPrepare_ != o.pvtRunning_)
      dumpPvtSequence(*o.pvtPrepare_, child(p, "pvtPrepare"));
#undef P
#undef D
  }

  void dumpMotorRecordController(const ecmcMotorRecordController& o,
                                 const std::string& p) {
#define D(name) value(child(p, #name), o.name)
#define P(name) pointer(child(p, #name), o.name)
    P(pAxes_); D(features_); value(child(p, "ctrlLocal.oldStatus"),
                                  static_cast<long long>(o.ctrlLocal.oldStatus));
    value(child(p, "ctrlLocal.initialPollDone"), o.ctrlLocal.initialPollDone);
    value(child(p, "ctrlLocal.movingPollPeriod"), o.ctrlLocal.movingPollPeriod);
    value(child(p, "ctrlLocal.idlePollPeriod"), o.ctrlLocal.idlePollPeriod);
    value(child(p, "ctrlLocal.errorId"), o.ctrlLocal.errorId);
    value(child(p, "ctrlLocal.pvtErrorId"), o.ctrlLocal.pvtErrorId);
    value(child(p, "ctrlLocal.pvtCurrentTriggerId"), o.ctrlLocal.pvtCurrentTriggerId);
#define IP(name) D(name)
    IP(ecmcMotorRecordErr_); IP(ecmcMotorRecordStatusCode_); IP(ecmcMotorRecordStatusBits_);
    IP(ecmcMotorRecordaux0_); IP(ecmcMotorRecordaux1_); IP(ecmcMotorRecordaux2_);
    IP(ecmcMotorRecordaux3_); IP(ecmcMotorRecordaux4_); IP(ecmcMotorRecordaux5_);
    IP(ecmcMotorRecordaux6_); IP(ecmcMotorRecordaux7_); IP(ecmcMotorRecordreason24_);
    IP(ecmcMotorRecordreason25_); IP(ecmcMotorRecordreason26_); IP(ecmcMotorRecordreason27_);
    IP(ecmcMotorRecordHomProc_RB_); IP(ecmcMotorRecordHomPos_RB_);
    IP(ecmcMotorRecordHomProc_); IP(ecmcMotorRecordHomPos_); IP(ecmcMotorRecordVelToHom_);
    IP(ecmcMotorRecordVelFrmHom_); IP(ecmcMotorRecordAccHom_); IP(ecmcMotorRecordHomUseHVEL_);
    IP(ecmcMotorRecordEncAct_);
#ifdef CREATE_MOTOR_REC_RESOLUTION
    IP(motorRecResolution_); IP(motorRecDirection_); IP(motorRecOffset_);
#endif
    IP(ecmcMotorRecordErrRst_); IP(ecmcMotorRecordMCUErrMsg_);
    IP(ecmcMotorRecordIlockMsg_); IP(ecmcMotorRecordIlockShortMsg_);
    IP(ecmcMotorRecordDbgStrToMcu_); IP(ecmcMotorRecordDbgStrToLog_);
    IP(ecmcMotorRecordVelAct_); IP(ecmcMotorRecordVel_RB_); IP(ecmcMotorRecordAcc_RB_);
    IP(ecmcMotorRecordCfgVELO_); IP(ecmcMotorRecordCfgVMAX_); IP(ecmcMotorRecordCfgJVEL_);
    IP(ecmcMotorRecordCfgACCS_); IP(ecmcMotorRecordCfgSREV_RB_);
    IP(ecmcMotorRecordCfgUREV_RB_); IP(ecmcMotorRecordCfgPMIN_RB_);
    IP(ecmcMotorRecordCfgPMAX_RB_); IP(ecmcMotorRecordCfgSPDB_RB_);
    IP(ecmcMotorRecordCfgRDBD_RB_); IP(ecmcMotorRecordCfgRDBD_Tim_RB_);
    IP(ecmcMotorRecordCfgRDBD_En_RB_); IP(ecmcMotorRecordCfgPOSLAG_RB_);
    IP(ecmcMotorRecordCfgPOSLAG_Tim_RB_); IP(ecmcMotorRecordCfgPOSLAG_En_RB_);
    IP(ecmcMotorRecordCfgDHLM_); IP(ecmcMotorRecordCfgDLLM_);
    IP(ecmcMotorRecordCfgDHLM_En_); IP(ecmcMotorRecordCfgDLLM_En_);
    IP(ecmcMotorRecordCfgDESC_RB_); IP(ecmcMotorRecordCfgEGU_RB_);
    IP(ecmcMotorRecordTRIGG_STOPP_); IP(ecmcMotorRecordTRIGG_DISABLE_);
    IP(ecmcMotorRecordTRIGG_SYNC_); IP(ecmcMotorRecordErrId_);
    IP(profileInitialized_); IP(profileBuilt_);
#undef IP
    value(child(p, "profileMessage_"), o.profileMessage_); P(pvtController_);
    D(profileInProgress_); D(profileTimeArraySize_);
#undef P
#undef D
  }

  void dumpMasterSlave(const ecmcMasterSlaveStateMachine& o, const std::string& p) {
    dumpErrorBase(static_cast<const ecmcError&>(o), child(p, "errorBase"));
#define D(name) value(child(p, #name), o.name)
#define E(name) value(child(p, #name), static_cast<long long>(o.name))
#define P(name) pointer(child(p, #name), o.name)
    E(state_); D(name_); D(sampleTimeS_); D(timeCounter_); D(index_);
    D(asynInitOk_); D(validationOK_); D(optionAutoDisableMasters_);
    P(masterGrp_); P(slaveGrp_); D(status_); D(statusWord_); P(asynPortDriver_);
    P(asynControl_); P(asynState_); P(asynStatus_);
    value(child(p, "control_.enable"), static_cast<bool>(o.control_.enable));
    value(child(p, "control_.autoDisableMasters"), static_cast<bool>(o.control_.autoDisableMasters));
    value(child(p, "control_.autoDisableSlaves"), static_cast<bool>(o.control_.autoDisableSlaves));
    value(child(p, "control_.enableDbgPrintouts"), static_cast<bool>(o.control_.enableDbgPrintouts));
    value(child(p, "control_.dummy"), static_cast<unsigned long long>(o.control_.dummy));
    value(child(p, "controlOld_.enable"), static_cast<bool>(o.controlOld_.enable));
    value(child(p, "controlOld_.autoDisableMasters"), static_cast<bool>(o.controlOld_.autoDisableMasters));
    value(child(p, "controlOld_.autoDisableSlaves"), static_cast<bool>(o.controlOld_.autoDisableSlaves));
    value(child(p, "controlOld_.enableDbgPrintouts"), static_cast<bool>(o.controlOld_.enableDbgPrintouts));
    value(child(p, "controlOld_.dummy"), static_cast<unsigned long long>(o.controlOld_.dummy));
    D(idleCounter_); D(masterGroupWasBusy_); D(masterGroupReachedTarget_);
    D(masterDisableInProgress_); D(slaveTrajSourceExternalWaitCycles_);
    D(masterGroupBusyCycles_); D(masterAtTargetTimeoutS_); D(masterAtTargetTimeS_);
    D(masterPrepareTimeoutS_); D(masterPrepareTimeS_); D(executeCycleCounter_);
    D(transitionCount_); D(lastTransitionCycle_); D(lastFaultCycle_);
    E(previousState_); E(trackedState_); E(lastTransitionReason_);
    D(lastFaultCode_); D(lastFaultStatus_);
#undef P
#undef E
#undef D
  }
};

void ecmcMotionDiagMakeDumpFileName(char *buffer, size_t bufferSize) {
  makeDumpFileName(buffer, bufferSize);
}

int ecmcMotionStateWriteYaml(const char *fileName) {
  if (!fileName || fileName[0] == '\0') {
    return -1;
  }

  std::ostringstream snapshot;
  if (ecmcRTMutex) {
    epicsMutexLock(ecmcRTMutex);
  }
  ecmcMotionStateYamlWriter writer(snapshot);
  writer.dumpAll();
  if (ecmcRTMutex) {
    epicsMutexUnlock(ecmcRTMutex);
  }

  std::ofstream file(fileName, std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    return -1;
  }
  file << snapshot.str();
  file.close();
  return file.good() ? 0 : -1;
}

int ecmcMotionDiagWriteDumpFile(const char *fileName, int level) {
  if (!fileName || fileName[0] == '\0') {
    return -1;
  }
  DiagRequest request = {};
  request.level = level;
  snprintf(request.file, sizeof(request.file), "%s", fileName);
  const int status = writeDumpFile(request);
  if (status || level < 2) {
    return status;
  }

  std::ostringstream snapshot;
  if (ecmcRTMutex) {
    epicsMutexLock(ecmcRTMutex);
  }
  ecmcMotionStateYamlWriter writer(snapshot);
  writer.dumpAll();
  if (ecmcRTMutex) {
    epicsMutexUnlock(ecmcRTMutex);
  }

  std::ofstream file(fileName, std::ios::out | std::ios::app);
  if (!file.is_open()) {
    return -1;
  }
  file << "---\n";
  file << "# Complete raw motion-object state (diagnostic level 2)\n";
  file << snapshot.str();
  file.close();
  return file.good() ? 0 : -1;
}

void ecmcMotionDiagBuildAxisReport(int axisIndex, char *buffer, size_t bufferSize) {
  buildAxisReport(axisIndex, buffer, bufferSize);
}
