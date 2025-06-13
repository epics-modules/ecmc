/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcCmdParser.c
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

/**\file
 * \ingroup ecmc
 * Command parser for ECMC motion control
 */

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include "ecmcCmdParser.h"
#include "ecmcOctetIF.h"
#include "ecmcMainThread.h"
#include "ecmcErrorsList.h"
#include "ecmcMotion.h"
#include "ecmcEthercat.h"
#include "ecmcMisc.h"
#include "ecmcGeneral.h"
#include "ecmcCom.h"
#include "ecmcPLC.h"
#include "ecmcPlugin.h"
#include <iocsh.h>

typedef struct
{
  int      command_no;
  unsigned nCmdData;
  int      bEnabled;
  int      bExecute;
  double   position;
  double   velocity;
  double   acceleration;
  double   manualVelocitySlow;
  double   defaultAcceleration;
  double   manualVelocityFast;
} cmd_Motor_cmd_type;

static cmd_Motor_cmd_type cmd_Motor_cmd[ECMC_MAX_AXES];
static int ecmcInitDone = 0;

extern double mcuFrequency;

// Buffers
static char cExprBuffer[ECMC_CMD_MAX_SINGLE_CMD_LENGTH];
static char cIdBuffer[ECMC_CMD_MAX_SINGLE_CMD_LENGTH];
static char cIdBuffer2[ECMC_CMD_MAX_SINGLE_CMD_LENGTH];
static char cIdBuffer3[ECMC_CMD_MAX_SINGLE_CMD_LENGTH];
static char cPlcExprBuffer[ECMC_CMD_MAX_SINGLE_CMD_LENGTH];
static char cOneCommand[ECMC_CMD_MAX_SINGLE_CMD_LENGTH];

// TODO: Cleanup macros.. should not need different for different types
#define SEND_OK_OR_ERROR_AND_RETURN(function)\
        do {\
          int iRet = function;\
          if (iRet) {\
            cmd_buf_printf(buffer, "Error: %d", iRet);\
            return 0;\
          }\
          cmd_buf_printf(buffer, "OK");\
          return 0;\
        }\
        while (0)

#define SEND_RESULT_OR_ERROR_AND_RETURN_INT(function)\
        do {\
          int iRet = function;\
          if (iRet) {\
            cmd_buf_printf(buffer, "Error: %d", iRet);\
            return 0;\
          }\
          cmd_buf_printf(buffer, "%d", iValue);\
          return 0;\
        }\
        while (0)

#define SEND_RESULT_OR_ERROR_AND_RETURN_UINT(function)\
        do {\
          int iRet = function;\
          if (iRet) {\
            cmd_buf_printf(buffer, "Error: %d", iRet);\
            return 0;\
          }\
          cmd_buf_printf(buffer, "%u", u32Value);\
          return 0;\
        }\
        while (0)

#define SEND_RESULT_OR_ERROR_AND_RETURN_UINT64(function)\
        do {\
          int iRet = function;\
          if (iRet) {\
            cmd_buf_printf(buffer, "Error: %d", iRet);\
            return 0;\
          }\
          cmd_buf_printf(buffer, "%u", u64Value);\
          return 0;\
        }\
        while (0)

#define SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(function)\
        do {\
          int iRet = function;\
          if (iRet) {\
            cmd_buf_printf(buffer, "Error: %d", iRet);\
            return 0;\
          }\
          cmd_buf_printf(buffer, "%lf", fValue);\
          return 0;\
        }\
        while (0)

#define SEND_RESULT_OR_ERROR_AND_RETURN_INT64(function)\
        do {\
          int iRet = function;\
          if (iRet) {\
            cmd_buf_printf(buffer, "Error: %d", iRet);\
            return 0;\
          }\
          cmd_buf_printf(buffer, "%" PRIu64 "", i64Value);\
          return 0;\
        }\
        while (0)

#define IF_ERROR_SEND_ERROR_AND_RETURN(function)\
        do {\
          int iRet = function;\
          if (iRet) {\
            cmd_buf_printf(buffer, "Error: %d", iRet);\
            return 0;\
          }\
        }\
        while (0)

#define ECMC_COMMAND_FORMAT_ERROR 0x210000;

void       init_axis(int axis_no) {
}

static int appendAsciiDataToStorageBuffer(int         storageIndex,
                                          const char *asciiData) {
  char *currentStringPos = strchr(asciiData, '=');

  if (currentStringPos) {
    double data = 0;

    while (currentStringPos) {
      currentStringPos++;
      int nvals = sscanf(currentStringPos, "%lf", &data);

      if (nvals) {
        int error = appendStorageBuffer(storageIndex, &data, 1);

        if (error) {
          return error;
        }
        currentStringPos = strchr(currentStringPos + 1, ',');
      } else {
        return ECMC_COMMAND_FORMAT_ERROR;
      }
    }
  } else {
    return ECMC_COMMAND_FORMAT_ERROR;
  }
  return 0;
}

static const char *const sFeaturesQ_str     = ".THIS.sFeatures?";
static const char *const ADSPORT_equals_str = "ADSPORT=";
static const char *const Main_dot_str       = "Main.";
static const char *const Cfg_dot_str        =  "Cfg.";


static int motorHandleADS_ADR_getInt(ecmcOutputBufferType *buffer,
                                     unsigned              adsport,
                                     unsigned              group_no,
                                     unsigned              offset_in_group,
                                     int                  *iValue) {
  if ((group_no >= 0x4000) && (group_no < 0x5000)) {
    int motor_axis_no = (int)group_no - 0x4000;

    switch (offset_in_group)
      case 0x15:
        return getAxisMonEnableAtTargetMon(motor_axis_no, iValue);
  } else if ((group_no >= 0x5000) && (group_no < 0x6000)) {
    double tmpValue;
    int    motor_axis_no = (int)group_no - 0x5000;
    int    ret;

    switch (offset_in_group) {
    case 0x6:

      /* Motor direction */
      ret     = getAxisEncScaleDenom(motor_axis_no, &tmpValue);
      *iValue = tmpValue < 0 ? 1 : 0;
      return ret;

    case 0x8:

      /* Encoder direction */
      ret     = getAxisEncScaleNum(motor_axis_no, &tmpValue);
      *iValue = tmpValue < 0 ? 1 : 0;
      return ret;

    case 0xB:

      // ADSPORT=501/.ADR.16#5001,16#B,2,2?; #low Softlimit enabled
      getAxisEnableSoftLimitBwd(motor_axis_no, iValue);
      return 0;

    case 0xC:

      // ADSPORT=501/.ADR.16#5001,16#C,2,2?; #high Softlimit enabled
      getAxisEnableSoftLimitFwd(motor_axis_no, iValue);
      return 0;

    default:
      return ERROR_MAIN_PARSER_INVALID_ADS_OFFSET;
    }
  } else if ((group_no >= 0x6000) && (group_no < 0x7000)) {
    // group 6000
    int ret;
    int motor_axis_no = (int)group_no - 0x6000;

    switch (offset_in_group) {
    case 0x10:
      ret = getAxisMonEnableLagMon(motor_axis_no, iValue);
      return ret;

    default:
      return ERROR_MAIN_PARSER_INVALID_ADS_OFFSET;
    }
  } else if ((group_no >= 0x7000) && (group_no < 0x8000)) {
    double fValue;
    int    ret;
    int    motor_axis_no = (int)group_no - 0x7000;

    switch (offset_in_group) {
    case 0x6:

      /* Motor direction */
      ret     = getAxisDrvScaleNum(motor_axis_no, &fValue);
      *iValue = fValue < 0 ? 1 : 0;
      return ret;

    default:
      return ERROR_MAIN_PARSER_INVALID_ADS_OFFSET;
    }
  }

  return ERROR_MAIN_PARSER_UNKOWN_CMD;
}

static int motorHandleADS_ADR_putInt(ecmcOutputBufferType *buffer,
                                     unsigned              adsport,
                                     unsigned              group_no,
                                     unsigned              offset_in_group,
                                     int                   iValue) {
  if ((group_no >= 0x5000) && (group_no < 0x6000)) {
    int motor_axis_no = (int)group_no - 0x5000;

    // ADSPORT=501/.ADR.16#5001,16#B,2,2=1; #enable low Softlimit
    if (offset_in_group == 0xB) {
      return setAxisEnableSoftLimitBwd(motor_axis_no, iValue);
    }

    // ADSPORT=501/.ADR.16#5001,16#C,2,2=1; #enable high Softlimit
    if (offset_in_group == 0xC) {
      return setAxisEnableSoftLimitFwd(motor_axis_no, iValue);
    }
  }

  if ((group_no >= 0x4000) && (group_no < 0x5000)) {
    // int motor_axis_no = (int)indexGroup - 0x4000;
    if (offset_in_group == 0x15) {
      return 0;  /* Monitor */ /* TODO */
    }
  }

  return ERROR_MAIN_PARSER_UNKOWN_CMD;
}

static int motorHandleADS_ADR_getFloat(ecmcOutputBufferType *buffer,
                                       unsigned              adsport,
                                       unsigned              group_no,
                                       unsigned              offset_in_group,
                                       double               *fValue) {
  if (adsport == 501) {
    // group 4000
    if ((group_no >= 0x4000) && (group_no < 0x5000)) {
      int motor_axis_no = (int)group_no - 0x4000;
      int iValue;
      int ret;

      switch (offset_in_group) {
      case 0x6:

        // ADSPORT=501/.ADR.16#4001,16#6,8,5?; #Homing velocity towards cam
        return getAxisHomeVelTowardsCam(motor_axis_no, fValue);

      case 0x7:

        // ADSPORT=501/.ADR.16#4001,16#7,8,5?; #Homing velocity off cam
        return getAxisHomeVelOffCam(motor_axis_no, fValue);

      case 0x8:
        *fValue = cmd_Motor_cmd[motor_axis_no].manualVelocitySlow;
        return 0;   // ERROR_MAIN_PARSER_INVALID_ADS_OFFSET;

      case 0x9:
        *fValue = cmd_Motor_cmd[motor_axis_no].manualVelocityFast;
        return 0;   // ERROR_MAIN_PARSER_INVALID_ADS_OFFSET;

      case 0x16:
        return getAxisMonAtTargetTol(motor_axis_no, fValue);

      case 0x17:
        ret     = getAxisMonAtTargetTime(motor_axis_no, &iValue);
        *fValue = iValue * 1 / mcuFrequency;
        return ret;

      case 0x27:
        ret = getAxisMonEnableMaxVel(motor_axis_no, &iValue);

        if (!ret && iValue) {   /* No error and enabled */
          return getAxisMonMaxVel(motor_axis_no, fValue);
        } else {
          *fValue = 0.0;
          return 0;
        }

      case 0x101:

        // *fValue = cmd_Motor_cmd[motor_axis_no].defaultAcceleration;
        return getAxisAcceleration(motor_axis_no, fValue);

      default:
        return ERROR_MAIN_PARSER_INVALID_ADS_OFFSET;
      }
    }

    // group 5000
    if ((group_no >= 0x5000) && (group_no < 0x6000)) {
      double tmpValue;
      int    ret;
      int    motor_axis_no = (int)group_no - 0x5000;

      switch (offset_in_group) {
      case 0xD:
        return getAxisSoftLimitPosBwd(motor_axis_no, fValue);

      case 0xE:
        return getAxisSoftLimitPosFwd(motor_axis_no, fValue);

      case 0x23:
        ret     = getAxisEncScaleNum(motor_axis_no, &tmpValue);
        *fValue = fabs(tmpValue);
        return ret;

      case 0x24:
        ret     = getAxisEncScaleDenom(motor_axis_no, &tmpValue);
        *fValue = fabs(tmpValue);
        return ret;

      default:
        return ERROR_MAIN_PARSER_INVALID_ADS_OFFSET;
      }
    } else if ((group_no >= 0x6000) && (group_no < 0x7000)) {
      // group 6000
      double tmpValue;
      int    iValue;
      int    ret;
      int    motor_axis_no = (int)group_no - 0x6000;

      switch (offset_in_group) {
      case 0x12:
        ret     = getAxisMonPosLagTol(motor_axis_no, &tmpValue);
        *fValue = tmpValue;
        return ret;

      case 0x13:
        ret     = getAxisMonPosLagTime(motor_axis_no, &iValue);
        *fValue = iValue * 1 / mcuFrequency;
        return ret;

      default:
        return ERROR_MAIN_PARSER_INVALID_ADS_OFFSET;
      }
    } else if ((group_no >= 0x7000) && (group_no < 0x8000)) {
      // group 7000
      double tmpValue;
      int    ret;
      int    motor_axis_no = (int)group_no - 0x7000;

      switch (offset_in_group) {
      case 0x101:
        ret     = getAxisDrvScaleNum(motor_axis_no, &tmpValue);
        *fValue = fabs(tmpValue);
        return ret;

      default:
        return ERROR_MAIN_PARSER_INVALID_ADS_OFFSET;
      }
    }
  }

  return ERROR_MAIN_PARSER_UNKOWN_CMD;
}

/*
  ADSPORT=501/.ADR.16#5001,16#D,8,5=-13.5; #low Softlimit
  ADSPORT=501/.ADR.16#5001,16#E,8,5=140.0; #high Softlimit
*/
static int motorHandleADS_ADR_putFloat(ecmcOutputBufferType *buffer,
                                       unsigned              adsport,
                                       unsigned              group_no,
                                       unsigned              offset_in_group,
                                       double                fValue) {
  if (adsport == 501) {
    // group 4000
    if ((group_no >= 0x4000) && (group_no < 0x5000)) {
      int motor_axis_no = (int)group_no - 0x4000;
      int ret;

      // ADSPORT=501/.ADR.16#4001,16#6,8,5=200; #Homing velocity towards cam
      switch (offset_in_group) {
      case 0x6:
        return setAxisHomeVelTowardsCam(motor_axis_no, fValue);

      // ADSPORT=501/.ADR.16#4001,16#7,8,5=100; #Homing velocity off cam
      case 0x7:
        return setAxisHomeVelOffCam(motor_axis_no, fValue);

      case 0x8:
        cmd_Motor_cmd[motor_axis_no].manualVelocitySlow = fValue;
        return 0;   // ERROR_MAIN_PARSER_INVALID_ADS_OFFSET;

      case 0x9:
        cmd_Motor_cmd[motor_axis_no].manualVelocityFast = fValue;
        return 0;   // ERROR_MAIN_PARSER_INVALID_ADS_OFFSET;

      case 0x27:
        ret = setAxisMonEnableMaxVel(motor_axis_no, fValue != 0.0);

        if (ret) {   /* error */
          return ret;
        }
        return setAxisMonMaxVel(motor_axis_no, fValue);

      default:
        return ERROR_MAIN_PARSER_INVALID_ADS_OFFSET;
      }
    }

    // group 5000
    if ((group_no >= 0x5000) && (group_no < 0x6000)) {
      int motor_axis_no = (int)group_no - 0x5000;

      // ADSPORT=501/.ADR.16#5001,16#D,8,5=-13.5; #low Softlimit
      if (offset_in_group == 0xD) {
        return setAxisSoftLimitPosBwd(motor_axis_no, fValue);
      }

      // ADSPORT=501/.ADR.16#5001,16#E,8,5=140.0; #high Softlimit
      if (offset_in_group == 0xE) {
        return setAxisSoftLimitPosFwd(motor_axis_no, fValue);
      }

      // ADSPORT=501/.ADR.16#5001,16#23,8,5=-13.5; #Encoder scale num for axis
      if (offset_in_group == 0x23) {
        return setAxisEncScaleNum(motor_axis_no, fValue);
      }

      // ADSPORT=501/.ADR.16#5001,16#24,8,5=140.0; #Encoder scale denom for axis
      if (offset_in_group == 0x24) {
        return setAxisEncScaleDenom(motor_axis_no, fValue);
      }
    }
  }

  return ERROR_MAIN_PARSER_UNKOWN_CMD;
}

/*
  ADSPORT=501/.ADR.16#5001,16#B,2,2=1;
*/
static int motorHandleADS_ADR(const char *arg, ecmcOutputBufferType *buffer) {
  const char *myarg_1         = NULL;
  unsigned    adsport         = 0;
  unsigned    group_no        = 0;
  unsigned    offset_in_group = 0;
  unsigned    len_in_PLC      = 0;
  unsigned    type_in_PLC     = 0;
  int nvals;

  nvals = sscanf(arg, "%u/.ADR.16#%x,16#%x,%u,%u=",
                 &adsport,
                 &group_no,
                 &offset_in_group,
                 &len_in_PLC,
                 &type_in_PLC);
  LOGINFO6("%s/%s:%d "
           "nvals=%d adsport=%u group_no=0x%x offset_in_group=0x%x len_in_PLC=%u type_in_PLC=%u\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           nvals,
           adsport,
           group_no,
           offset_in_group,
           len_in_PLC,
           type_in_PLC);

  if (nvals != 5) return ERROR_MAIN_PARSER_INVALID_ADS_FORMAT;

  if (adsport != 501) return ERROR_MAIN_PARSER_INVALID_ADS_PORT;

  myarg_1 = strchr(arg, '=');

  if (myarg_1) {
    myarg_1++;  /* Jump over '=' */

    switch (type_in_PLC) {
    case 5: {
      double fValue;

      if (len_in_PLC != 8) return ERROR_MAIN_PARSER_INVALID_ADS_LEN;

      nvals = sscanf(myarg_1, "%lf", &fValue);

      if (nvals != 1) return ERROR_MAIN_PARSER_INVALID_ADS_FORMAT;

      return motorHandleADS_ADR_putFloat(buffer,
                                         adsport,
                                         group_no,
                                         offset_in_group,
                                         fValue);

      break;
    }

    case 2: {
      int iValue;

      if (len_in_PLC != 2) return ERROR_MAIN_PARSER_INVALID_ADS_LEN;

      nvals = sscanf(myarg_1, "%d", &iValue);

      if (nvals != 1) return ERROR_MAIN_PARSER_INVALID_ADS_FORMAT;

      return motorHandleADS_ADR_putInt(buffer,
                                       adsport,
                                       group_no,
                                       offset_in_group,
                                       iValue);

      break;
    }

    default:
      return ERROR_MAIN_PARSER_INVALID_ADS_FORMAT;
    }
  }
  myarg_1 = strchr(arg, '?');

  if (myarg_1) {
    int res;
    myarg_1++;  /* Jump over '?' */

    switch (type_in_PLC) {
    case 5: {
      double fValue;

      if (len_in_PLC != 8) return ERROR_MAIN_PARSER_INVALID_ADS_LEN;

      res = motorHandleADS_ADR_getFloat(buffer,
                                        adsport,
                                        group_no,
                                        offset_in_group,
                                        &fValue);

      if (res) return res;

      int errorCode = cmd_buf_printf(buffer, "%g", fValue);

      if (errorCode) {
        return errorCode;
      }
      return -1;

      break;
    }

    case 2: {
      int res;
      int iValue = -1;

      if (len_in_PLC != 2) return ERROR_MAIN_PARSER_INVALID_ADS_LEN;

      res = motorHandleADS_ADR_getInt(buffer,
                                      adsport,
                                      group_no,
                                      offset_in_group,
                                      &iValue);

      if (res) return res;

      int errorCode = cmd_buf_printf(buffer, "%d", iValue);

      if (errorCode) {
        return errorCode;
      }

      return -1;

      break;
    }

    default:
      return ERROR_MAIN_PARSER_INVALID_ADS_TYPE;
    }
  }
  return ERROR_MAIN_PARSER_UNKNOWN_ADS_CMD;
}

/**
 * \brief Handles all the configuration commands"
*/
static int handleCfgCommand(const char *myarg_1) {
  int iValue        = 0;
  int iValue2       = 0;
  int iValue3       = 0;
  int iValue4       = 0;
  int iValue5       = 0;
  int iValue6       = 0;
  int iValue7       = 0;
  int iValue8       = 0;
  int iValue9       = 0;
  int iValue10      = 0;
  int nvals         = 0;
  uint64_t u64Value = 0;
  double   dValue   = 0;
  double   dValue2  = 0;
  double   dValue3  = 0;
  double   dValue4  = 0;

  /// "Cfg.SetAppMode(mode)"
  nvals = sscanf(myarg_1, "SetAppMode(%d)", &iValue);

  if (nvals == 1) {
    return setAppMode(iValue);
  }

  /// "Cfg.ValidateConfig()"
  nvals = strcmp(myarg_1, "ValidateConfig()");

  if (nvals == 0) {
    return validateConfig();
  }

  /// "Cfg.SetEcStartupTimeout(timeSeconds)"
  nvals = sscanf(myarg_1, "SetEcStartupTimeout(%d)", &iValue);

  if (nvals == 1) {
    return setEcStartupTimeout(iValue);
  }

  /// "Cfg.SetSampleRate(double sampleRate)"
  nvals = sscanf(myarg_1, "SetSampleRate(%lf)", &dValue);

  if (nvals == 1) {
    return setSampleRate(dValue);
  }

  /// "Cfg.SetSamplePeriodMs(double samplePeriodMs)"
  nvals = sscanf(myarg_1, "SetSamplePeriodMs(%lf)", &dValue);

  if (nvals == 1) {
    return setSamplePeriodMs(dValue);
  }

  /// "Cfg.SetPVTControllerTrgDurMs(double durationMs)"
  nvals = sscanf(myarg_1, "SetPVTControllerTrgDurMs(%lf)", &dValue);

  if (nvals == 1) {
    return setPVTControllerTrgDurMs(dValue);
  }

  /// "Cfg.CreateAxis(axisIndex, axisType, drvType,trajType)"
  nvals = sscanf(myarg_1,
                 "CreateAxis(%d,%d,%d,%d)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4);

  if (nvals == 4) {
    return createAxis(iValue, iValue2, iValue3, iValue4);
  }


  /// "Cfg.CreateAxis(axisIndex, axisType, drvType)"
  nvals = sscanf(myarg_1, "CreateAxis(%d,%d,%d)", &iValue, &iValue2, &iValue3);

  if (nvals == 3) {
    return createAxis(iValue, iValue2, iValue3, 0);
  }

  /// "Cfg.CreateAxis(axisIndex, axisType)"
  // Defaults as stepper drive
  nvals = sscanf(myarg_1, "CreateAxis(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return createAxis(iValue, iValue2, 0, 0);
  }

  /// "Cfg.CreateAxis(axisIndex)"
  // Defaults as real axis with stepper drive
  nvals = sscanf(myarg_1, "CreateAxis(%d)", &iValue);

  if (nvals == 1) {
    return createAxis(iValue, 1, 0, 0);
  }

  /// "Cfg.CreateDefaultAxis(axisIndex)"
  // Defaults as real axis with stepper drive and trapetzoidal traj generator
  nvals = sscanf(myarg_1, "CreateDefaultAxis(%d)", &iValue);

  if (nvals == 1) {
    return createAxis(iValue, 1, 0, 0);
  }

  /// "Cfg.AddAxisGroup(groupName)"
  cIdBuffer[0]  = '\0';
  nvals = sscanf(myarg_1, "AddAxisGroup(%[^)])",                
                 cIdBuffer);
  if (nvals == 1) {
    return addAxisGroup(cIdBuffer);
  }

  /// "Cfg.AddAxisToGroupByName(axisIndex,groupName,createGroup)"
  cIdBuffer[0]  = '\0';
  nvals = sscanf(myarg_1, "AddAxisToGroupByName(%d,%[^,],%d)",
                &iValue,
                 cIdBuffer,
                &iValue2);
  if (nvals == 3) {
    return addAxisToGroupByNameCreate(iValue,cIdBuffer,iValue2);
  }

  /// "Cfg.AddAxisToGroupByName(axisIndex,groupName)"
  cIdBuffer[0]  = '\0';
  nvals = sscanf(myarg_1, "AddAxisToGroupByName(%d,%[^)])",
                &iValue,
                 cIdBuffer);
  if (nvals == 2) {
    return addAxisToGroupByName(iValue,cIdBuffer);
  }

  /// "Cfg.AddAxisToGroupByIndex(axisIndex,groupIndex)"
  cIdBuffer[0]  = '\0';
  nvals = sscanf(myarg_1, "AddAxisToGroupByIndex(%d,%d)",
                &iValue,
                &iValue2);
  if (nvals == 2) {
    return addAxisToGroupByIndex(iValue,iValue2);
  }

  /// "Cfg.CreatePLC(int index, double cycleTimeMs)"
  nvals = sscanf(myarg_1, "CreatePLC(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return createPLC(iValue, dValue, 0);
  }

  /// "Cfg.CreatePLC(int index)"
  nvals = sscanf(myarg_1, "CreatePLC(%d)", &iValue);

  if (nvals == 1) {
    return createPLC(iValue, 1, 0);
  }

  /// "Cfg.DeletePLC(int index)"
  nvals = sscanf(myarg_1, "DeletePLC(%d)", &iValue);

  if (nvals == 1) {
    return deletePLC(iValue);
  }

  /// "Cfg.SetPLCEnable(int index,int enable)"
  nvals = sscanf(myarg_1, "SetPLCEnable(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setPLCEnable(iValue, iValue2);
  }

  /// "Cfg.LinkEcEntryToObject(ecEntryPathString,objPathString)"
  // ec0.s1.POSITION.-1
  // ax1.enc.actpos
  cIdBuffer[0]  = '\0';
  cIdBuffer2[0] = '\0';
  nvals         = sscanf(myarg_1,
                         "LinkEcEntryToObject(%[^,],%[^)])",
                         cIdBuffer,
                         cIdBuffer2);

  if (nvals == 2) {
    return linkEcEntryToObject(cIdBuffer, cIdBuffer2);
  }

  /// "Cfg.LinkEcEntryToObject(ecEntryPathString,objPathString)"
  // Allow empty entryIdString (no action will be taken)
  cIdBuffer[0]  = '\0';
  cIdBuffer2[0] = '\0';
  nvals         = sscanf(myarg_1, "LinkEcEntryToObject(,%[^)])", cIdBuffer2);

  if (nvals == 1) {
    return 0;
  }

  /// "Cfg.LinkEcEntryToObject(ecEntryPathString,objPathString)"
  // Allow empty
  nvals = strcmp(myarg_1, "LinkEcEntryToObject(,)");

  if (nvals == 0) {
    return 0;
  }

  /// "Cfg.LinkEcEntryToAxisEncoder(slaveBusPosition,entryIdString,
  /// axisIndex,encoderEntryIndex,entrybitIndex)"
  cIdBuffer[0] = '\0';
  nvals        = sscanf(myarg_1,
                        "LinkEcEntryToAxisEncoder(%d,%[^,],%d,%d,%d)",
                        &iValue,
                        cIdBuffer,
                        &iValue3,
                        &iValue4,
                        &iValue5);

  if (nvals == 5) {
    return linkEcEntryToAxisEnc(iValue, cIdBuffer, iValue3, iValue4, iValue5);
  }

  /// "Cfg.LinkEcEntryToAxisDrive(slaveBusPosition,entryIdString,
  /// axisIndex,driveEntryIndex,entrybitIndex)"
  cIdBuffer[0] = '\0';
  iValue5      = -10;
  nvals        = sscanf(myarg_1,
                        "LinkEcEntryToAxisDrive(%d,%[^,],%d,%d,%d)",
                        &iValue,
                        cIdBuffer,
                        &iValue3,
                        &iValue4,
                        &iValue5);

  // Allow empty entryIdString and/or entrybitIndex
  if (nvals == 5) {
    return linkEcEntryToAxisDrv(iValue, cIdBuffer, iValue3, iValue4, iValue5);
  }

  // Allow empty entryIdString
  cIdBuffer[0] = '\0';
  nvals        = sscanf(myarg_1,
                        "LinkEcEntryToAxisDrive(%d,,%d,%d,%d)",
                        &iValue,
                        &iValue3,
                        &iValue4,
                        &iValue5);

  if (nvals == 4) {
    return linkEcEntryToAxisDrv(iValue, cIdBuffer, iValue3, iValue4, iValue5);
  }

  /// "Cfg.LinkEcEntryToAxisMonitor(slaveBusPosition,entryIdString,
  /// axisIndex,monitorEntryIndex,entrybitIndex)"
  nvals = sscanf(myarg_1,
                 "LinkEcEntryToAxisMonitor(%d,%[^,],%d,%d,%d)",
                 &iValue,
                 cIdBuffer,
                 &iValue3,
                 &iValue4,
                 &iValue5);

  if (nvals == 5) {
    return linkEcEntryToAxisMon(iValue, cIdBuffer, iValue3, iValue4, iValue5);
  }

  /// "Cfg.LinkEcEntryToEcStatusOutput(slaveBusPosition,entryIdString)"
  nvals = sscanf(myarg_1,
                 "LinkEcEntryToEcStatusOutput(%d,%[^)])",
                 &iValue,
                 cIdBuffer);

  if (nvals == 2) {
    return linkEcEntryToEcStatusOutput(iValue, cIdBuffer);
  }

  /// "Cfg.LinkEcEntryToAxisStatusOutput(slaveBusPosition,entryIdString)"
  nvals = sscanf(myarg_1,
                 "LinkEcEntryToAxisStatusOutput(%d,%[^,],%d)",
                 &iValue,
                 cIdBuffer,
                 &iValue2);

  if (nvals == 3) {
    return linkEcEntryToAxisStatusOutput(iValue, cIdBuffer, iValue2);
  }

  /// "Cfg.WriteEcEntryIDString(slaveBusPosition,entryIdString,value)"
  nvals = sscanf(myarg_1,
                 "WriteEcEntryIDString(%d,%[^,],%" SCNu64 ")",
                 &iValue,
                 cIdBuffer,
                 &u64Value);

  if (nvals == 3) {
    return writeEcEntryIDString(iValue, cIdBuffer, u64Value);
  }

  /// "Cfg.WriteEcEntryEcPath(ecPath,value)"
  nvals = sscanf(myarg_1,
                 "WriteEcEntryEcPath(%[^,],%" SCNu64 ")",
                 cIdBuffer,
                 &u64Value);

  if (nvals == 2) {
    return writeEcEntryEcPath(cIdBuffer, u64Value);
  }

  /// "Cfg.EcSetMaster(masterIndex)"
  nvals = sscanf(myarg_1, "EcSetMaster(%d)", &iValue);

  if (nvals == 1) {
    return ecSetMaster(iValue);
  }

  /// "Cfg.EcResetMaster(masterIndex)"
  nvals = sscanf(myarg_1, "EcResetMaster(%d)", &iValue);

  if (nvals == 1) {
    return ecResetMaster(iValue);
  }

  /// "Cfg.EcAddSlave(alias,slaveBusPosition,vendorId,productCode)"
  nvals = sscanf(myarg_1,
                 "EcAddSlave(%d,%d,0x%x,0x%x)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4);

  if (nvals == 4) {
    return ecAddSlave(iValue, iValue2, iValue3, iValue4);
  }

  /// "Cfg.EcSlaveConfigWatchDog(slaveBusposition,watchdogDivider,
  /// watchdogIntervals)"
  nvals = sscanf(myarg_1,
                 "EcSlaveConfigWatchDog(%d,%d,%d)",
                 &iValue,
                 &iValue2,
                 &iValue3);

  if (nvals == 3) {
    return ecSlaveConfigWatchDog(iValue, iValue2, iValue3);
  }

  /// "Cfg.EcSlaveVerify(alias,slaveBusPosition,vendorId,productCode,revisionNum)"
  nvals = sscanf(myarg_1,
                 "EcSlaveVerify(%d,%d,0x%x,0x%x,0x%x)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5);

  if (nvals == 5) {
    // Also check revsionNum
    return ecVerifySlave(iValue, iValue2, iValue3, iValue4, iValue5);
  }

  /// "Cfg.EcSlaveVerify(alias,slaveBusPosition,vendorId,productCode)"
  nvals = sscanf(myarg_1,
                 "EcSlaveVerify(%d,%d,0x%x,0x%x)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4);

  if (nvals == 4) {
    // Do not check revsion number (use revsionNum=0)
    return ecVerifySlave(iValue, iValue2, iValue3, iValue4, 0);
  }

  /*Cfg.EcAddPdo(int nSlave,int nSyncManager,uint16_t nPdoIndex) wrong*/
  nvals = sscanf(myarg_1, "EcAddPdo(%d,%d,0x%x)", &iValue, &iValue2, &iValue3);

  if (nvals == 3) {
    return ecAddPdo(iValue, iValue2, iValue3);
  }

  /*Cfg.EcAddEntryComplete(
    uint16_t position,
    uint32_t vendor_id,
    uint32_t product_code,
    int      nDirection,
    uint8_t  nSyncMangerIndex,
    uint16_t nPdoIndex,
    uint16_t nEntryIndex,
    uint8_t  nEntrySubIndex,
    uint8_t  nBits,
    int      signed,
    char    *cID)*/
  nvals = sscanf(myarg_1,
                 "EcAddEntryComplete(%d,0x%x,0x%x,%d,%d,0x%x,0x%x,0x%x,%d,%d,%[^)])",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5,
                 &iValue6,
                 &iValue7,
                 &iValue8,
                 &iValue9,
                 &iValue10,
                 cIdBuffer);

  if (nvals == 11) {
    return ecAddEntryComplete(iValue,
                              iValue2,
                              iValue3,
                              iValue4,
                              iValue5,
                              iValue6,
                              iValue7,
                              iValue8,
                              iValue9,
                              cIdBuffer,
                              iValue10);
  }

  /*Cfg.EcAddEntryComplete(
      uint16_t position,
      uint32_t vendor_id,
      uint32_t product_code,
      int nDirection,
      uint8_t nSyncMangerIndex,
      uint16_t nPdoIndex,
      uint16_t nEntryIndex,
      uint8_t  nEntrySubIndex,
      uint8_t nBits,
      char *cID,
      int updateInRealtime
      )*/
  nvals = sscanf(myarg_1,
                 "EcAddEntryComplete(%d,0x%x,0x%x,%d,%d,0x%x,0x%x,0x%x,%d,%[^,],%d)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5,
                 &iValue6,
                 &iValue7,
                 &iValue8,
                 &iValue9,
                 cIdBuffer,
                 &iValue10);

  if (nvals == 11) {
    int ret = ecAddEntryComplete(iValue,
                                 iValue2,
                                 iValue3,
                                 iValue4,
                                 iValue5,
                                 iValue6,
                                 iValue7,
                                 iValue8,
                                 iValue9,
                                 cIdBuffer,
                                 0);

    if (ret) {
      return ret;
    }
    return ecSetEntryUpdateInRealtime(iValue, cIdBuffer, iValue10);
  }

  /*Cfg.EcAddEntryComplete(
    uint16_t position,
    uint32_t vendor_id,
    uint32_t product_code,
    int nDirection,
    uint8_t nSyncMangerIndex,
    uint16_t nPdoIndex,
    uint16_t nEntryIndex,
    uint8_t  nEntrySubIndex,
    uint8_t nBits,
    char *cID)*/
  nvals = sscanf(myarg_1,
                 "EcAddEntryComplete(%d,0x%x,0x%x,%d,%d,0x%x,0x%x,0x%x,%d,%[^)])",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5,
                 &iValue6,
                 &iValue7,
                 &iValue8,
                 &iValue9,
                 cIdBuffer);

  if (nvals == 10) {
    return ecAddEntryComplete(iValue,
                              iValue2,
                              iValue3,
                              iValue4,
                              iValue5,
                              iValue6,
                              iValue7,
                              iValue8,
                              iValue9,
                              cIdBuffer,
                              0);
  }

  // New syntax

  /*Cfg.EcAddEntryDT(
    uint16_t position,
    uint32_t vendor_id,
    uint32_t product_code,
    int      nDirection,
    uint8_t  nSyncMangerIndex,
    uint16_t nPdoIndex,
    uint16_t nEntryIndex,
    uint8_t  nEntrySubIndex,
    char    *dataType,
    char    *cID,
    int      updateInRealtime)*/
  cIdBuffer[0]  = '\0';
  cIdBuffer2[0] = '\0';
  cIdBuffer3[0] = '\0';
  nvals         = sscanf(myarg_1,
                         "EcAddEntryDT(%d,0x%x,0x%x,%d,%d,0x%x,0x%x,0x%x,%[^,],%[^,],%d)",
                         &iValue,
                         &iValue2,
                         &iValue3,
                         &iValue4,
                         &iValue5,
                         &iValue6,
                         &iValue7,
                         &iValue8,
                         cIdBuffer,
                         cIdBuffer2,
                         &iValue9);

  if (nvals == 11) {
    return ecAddEntry(iValue,
                      iValue2,
                      iValue3,
                      iValue4,
                      iValue5,
                      iValue6,
                      iValue7,
                      iValue8,
                      cIdBuffer,
                      cIdBuffer2,
                      iValue9);
  }

  // New syntax (default update in Real time)

  /*Cfg.EcAddEntryDT(
    uint16_t position,
    uint32_t vendor_id,
    uint32_t product_code,
    int      nDirection,
    uint8_t  nSyncMangerIndex,
    uint16_t nPdoIndex,
    uint16_t nEntryIndex,
    uint8_t  nEntrySubIndex,
    char    *dataType,
    char    *cID)*/
  cIdBuffer[0]  = '\0';
  cIdBuffer2[0] = '\0';
  cIdBuffer3[0] = '\0';
  nvals         = sscanf(myarg_1,
                         "EcAddEntryDT(%d,0x%x,0x%x,%d,%d,0x%x,0x%x,0x%x,%[^,],%[^)])",
                         &iValue,
                         &iValue2,
                         &iValue3,
                         &iValue4,
                         &iValue5,
                         &iValue6,
                         &iValue7,
                         &iValue8,
                         cIdBuffer,
                         cIdBuffer2);

  if (nvals == 10) {
    return ecAddEntry(iValue,
                      iValue2,
                      iValue3,
                      iValue4,
                      iValue5,
                      iValue6,
                      iValue7,
                      iValue8,
                      cIdBuffer,
                      cIdBuffer2,
                      1);
  }

  /*Cfg.EcAddSdoAsync(
    uint16_t position,
    uint16_t nIndex,
    uint8_t  nSubIndex,
    char    *dataType,
    char    *cID)*/
  cIdBuffer[0]  = '\0';
  cIdBuffer2[0] = '\0';
  nvals         = sscanf(myarg_1,
                         "EcAddSdoAsync(%d,0x%x,0x%x,%[^,],%[^)])",
                         &iValue,
                         &iValue2,
                         &iValue3,
                         cIdBuffer,
                         cIdBuffer2);

  if (nvals == 5) {
    return ecAddSdoAsync(iValue,
                         iValue2,
                         iValue3,
                         cIdBuffer,
                         cIdBuffer2);
  }

  /*Cfg.EcAddMemMapDT(
      char *startEntryIDString,  (ec0.s1.AI1)
      size_t byteSize,
      int direction,
      char *dataType,            (S32)
      char *memMapIDString       (ec0.s1.CH1_ARRAY)
      )*/

  cIdBuffer[0]  = '\0';
  cIdBuffer2[0] = '\0';
  cIdBuffer3[0] = '\0';
  nvals         = sscanf(myarg_1,
                         "EcAddMemMapDT(%[^,],%d,%d,%[^,],%[^)])",
                         cIdBuffer,
                         &iValue2,
                         &iValue3,
                         cIdBuffer2,
                         cIdBuffer3);

  if (nvals == 5) {
    return ecAddMemMapDT(cIdBuffer, (size_t)iValue2, iValue3,
                         cIdBuffer2, cIdBuffer3);
  }

  /*Cfg.EcAddMemMap(
      uint16_t startEntryBusPosition,
      char *startEntryIDString,
      size_t byteSize,
      int direction,
      char *memMapIDString
      )*/
  cIdBuffer[0]  = '\0';
  cIdBuffer2[0] = '\0';
  cIdBuffer3[0] = '\0';
  nvals         = sscanf(myarg_1,
                         "EcAddMemMap(%d,%[^,],%d,%d,%[^)])",
                         &iValue,
                         cIdBuffer,
                         &iValue2,
                         &iValue3,
                         cIdBuffer2);

  if (nvals == 5) {
    return ecAddMemMap(iValue, cIdBuffer, (size_t)iValue2, iValue3,
                       cIdBuffer2);
  }

  /*Cfg.EcAddDataDT(
        char *startEntryIDString,  (ec0.s1.AI1)
        size_t   entryByteOffset,   byte offset from startEntry
        size_t   entryBitOffset,    bit offset
        int direction,
        char *dataType,            (S32)
        char *memMapIDString       (ec0.s1.CH1_ARRAY)
        )*/
  cIdBuffer[0]  = '\0';
  cIdBuffer2[0] = '\0';
  cIdBuffer3[0] = '\0';
  nvals         = sscanf(myarg_1,
                         "EcAddDataDT(%[^,],%d,%d,%d,%[^,],%[^)])",
                         cIdBuffer,
                         &iValue2,
                         &iValue3,
                         &iValue4,
                         cIdBuffer2,
                         cIdBuffer3);

  if (nvals == 6) {
    return ecAddDataDT(cIdBuffer, (size_t)iValue2,
                       (size_t)iValue3, iValue4,
                       cIdBuffer2, cIdBuffer3);
  }

  /*Cfg.EcSlaveConfigDC(
      int slave_bus_position,
      uint16_t assign_activate,
      uint32_t sync0_cycle,
      int32_t sync0_shift,
      uint32_t sync1_cycle,
      int32_t sync1_shift )*/
  nvals = sscanf(myarg_1,
                 "EcSlaveConfigDC(%d,0x%x,%d,%d,%d,%d)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5,
                 &iValue6);

  if (nvals == 6) {
    return ecSlaveConfigDC(iValue, iValue2, iValue3, iValue4, iValue5,
                           iValue6);
  }

  /*Cfg.EcSelectReferenceDC(
      int master_index,
      int slave_bus_position)
      */
  nvals = sscanf(myarg_1, "EcSelectReferenceDC(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return ecSelectReferenceDC(iValue, iValue2);
  }

  /*Cfg.EcUseClockRealtime(int useClcRT)*/
  nvals = sscanf(myarg_1, "EcUseClockRealtime(%d)", &iValue);

  if (nvals == 1) {
    return ecUseClockRealtime(iValue);
  }

  /*Cfg.EcSetEntryUpdateInRealtime(
      uint16_t slavePosition,
      char *entryIDString,
      int updateInRealtime
      );
      */
  nvals = sscanf(myarg_1,
                 "EcSetEntryUpdateInRealtime(%d,%[^,],%d)",
                 &iValue,
                 cIdBuffer,
                 &iValue2);

  if (nvals == 3) {
    return ecSetEntryUpdateInRealtime(iValue, cIdBuffer, iValue2);
  }

  /*Cfg.EcResetError()*/
  if (0 == strcmp(myarg_1, "EcResetError()")) {
    return ecResetError();
  }

  /*Cfg.EcAddSyncManager(int nSlave,ec_direction_t nDirection,
  uint8_t nSyncMangerIndex)*/
  nvals = sscanf(myarg_1,
                 "EcAddSyncManager(%d,%d,%d)",
                 &iValue,
                 &iValue2,
                 &iValue3);

  if (nvals == 3) {
    return ecAddSyncManager(iValue, iValue2, iValue3);
  }

  /*Cfg.EcAddSdo(uint16_t slave_position,uint16_t sdo_index,
  uint8_t sdo_subindex,uint32_t value,int byteSize)*/
  nvals = sscanf(myarg_1,
                 "EcAddSdo(%d,0x%x,0x%x,%d,%d)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5);

  if (nvals == 5) {
    return ecAddSdo(iValue, iValue2, iValue3, iValue4, iValue5);
  }

  /*Cfg.EcAddSdo(uint16_t slave_position,uint16_t sdo_index,
  uint8_t sdo_subindex,uint32_t value,int byteSize)*/
  nvals = sscanf(myarg_1,
                 "EcAddSdo(%d,0x%x,0x%x,0x%x,%d)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5);

  if (nvals == 5) {
    return ecAddSdo(iValue, iValue2, iValue3, iValue4, iValue5);
  }

  cIdBuffer[0]  = '\0';
  cIdBuffer2[0] = '\0';

  /*Cfg.EcAddSdoDT(uint16_t slave_position,uint16_t sdo_index,
  uint8_t sdo_subindex,char* value, char* datatype)*/
  nvals = sscanf(myarg_1,
                 "EcAddSdoDT(%d,0x%x,0x%x,%[^,],%[^)])",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 cIdBuffer,
                 cIdBuffer2);

  if (nvals == 5) {
    return ecAddSdoDT(iValue, iValue2, iValue3, cIdBuffer, cIdBuffer2);
  }

  /*Cfg.EcAddSdoComplete(uint16_t slave_position,uint16_t sdo_index,
  ,const char* values,int byteSize)*/
  nvals = sscanf(myarg_1,
                 "EcAddSdoComplete(%d,0x%x,%[^,],%d)",
                 &iValue,
                 &iValue2,
                 &cIdBuffer[0],
                 &iValue3);

  if (nvals == 4) {
    return ecAddSdoComplete(iValue, iValue2, cIdBuffer, iValue3);
  }


  /*Cfg.EcAddSdoBuffer(uint16_t slave_position,uint16_t sdo_index,
  ,const char* values,int byteSize)*/
  nvals = sscanf(myarg_1,
                 "EcAddSdoBuffer(%d,0x%x,0x%x,%[^,],%d)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &cIdBuffer[0],
                 &iValue4);

  if (nvals == 5) {
    return ecAddSdoBuffer(iValue, iValue2, iValue3, cIdBuffer, iValue4);
  }

  /*Cfg.EcWriteSdo(uint16_t slave_position,uint16_t sdo_index,
  uint8_t sdo_subindex,uint32_t value,int byteSize)*/
  nvals = sscanf(myarg_1,
                 "EcWriteSdo(%d,0x%x,0x%x,%d,%d)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5);

  if (nvals == 5) {
    return ecWriteSdo(iValue, iValue2, iValue3, iValue4, iValue5);
  }

  /*Cfg.EcWriteSdo(uint16_t slave_position,uint16_t sdo_index,
  uint8_t sdo_subindex,uint32_t value,int byteSize)*/
  nvals = sscanf(myarg_1,
                 "EcWriteSdo(%d,0x%x,0x%x,0x%x,%d)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5);

  if (nvals == 5) {
    return ecWriteSdo(iValue, iValue2, iValue3, iValue4, iValue5);
  }

  /*Cfg.EcVerifySdo(uint16_t slave_position,uint16_t sdo_index,
  uint8_t sdo_subindex,uint32_t value,int byteSize)*/
  nvals = sscanf(myarg_1,
                 "EcVerifySdo(%d,0x%x,0x%x,%d,%d)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5);

  if (nvals == 5) {
    return ecVerifySdo(iValue, iValue2, iValue3, iValue4, iValue5);
  }

  /*Cfg.EcVerifySdo(uint16_t slave_position,uint16_t sdo_index,
  uint8_t sdo_subindex,uint32_t value,int byteSize)*/
  nvals = sscanf(myarg_1,
                 "EcVerifySdo(%d,0x%x,0x%x,0x%x,%d)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5);

  if (nvals == 5) {
    return ecVerifySdo(iValue, iValue2, iValue3, iValue4, iValue5);
  }

  uint64_t uint64Value = 0;

  /*EcWriteSoE(uint16_t  slavePosition,
              uint8_t   driveNo,
              uint16_t  idn,
              size_t    byteSize,
              uint64_t  value)*/
  nvals = sscanf(myarg_1,
                 "EcWriteSoE(%d,%d,%d,%d,%" SCNu64 ")",
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5,
                 &uint64Value);

  if (nvals == 5) {
    return ecWriteSoE(iValue2, iValue3, iValue4, iValue5,
                      (uint8_t *)(&uint64Value));
  }

  /*Cfg.EcWriteSdoComplete(uint16_t slave_position,uint16_t sdo_index,
  uint8_t sdo_subindex,uint32_t value,int byteSize)*/
  /*nvals = sscanf(myarg_1,
                 "EcWriteSdoComplete(%d,0x%x,%d,%d)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4);

  if (nvals == 4) {
    return ecWriteSdoComplete(iValue, iValue2, iValue3, iValue4);
  }*/

  /*Cfg.EcWriteSdoComplete(uint16_t slave_position,uint16_t sdo_index,
  uint8_t sdo_subindex,uint32_t value,int byteSize)*/
  /*nvals = sscanf(myarg_1,
                 "EcWriteSdoComplete(%d,0x%x,0x%x,%d)",
                 &iValue,
                 &iValue2,
                 &iValue3,
                 &iValue4);

  if (nvals == 4) {
    return ecWriteSdoComplete(iValue, iValue2, iValue3, iValue4);
  }*/

  /*Cfg.EcApplyConfig(int nMasterIndex)*/
  nvals = sscanf(myarg_1, "EcApplyConfig(%d)", &iValue);

  if (nvals == 1) {
    return ecApplyConfig(iValue);
  }

  /*Cfg.EcApplyConfig()*/
  if (0 == strcmp(myarg_1, "EcApplyConfig()")) {
    return ecApplyConfig(-1);
  }

  /*Cfg.EcSetDiagnostics(int nDiagnostics)*/
  nvals = sscanf(myarg_1, "EcSetDiagnostics(%d)", &iValue);

  if (nvals == 1) {
    return ecSetDiagnostics(iValue);
  }

  /*Cfg.EcEnablePrintouts(int enable)*/
  nvals = sscanf(myarg_1, "EcEnablePrintouts(%d)", &iValue);

  if (nvals == 1) {
    return ecEnablePrintouts(iValue);
  }

  /*Cfg.EcSetDomainFailedCyclesLimit(int nCycles)*/
  nvals = sscanf(myarg_1, "EcSetDomainFailedCyclesLimit(%d)", &iValue);

  if (nvals == 1) {
    return ecSetDomainFailedCyclesLimit(iValue);
  }

  /*Cfg.EcAddDomain(int nCycles,int offset)*/
  nvals = sscanf(myarg_1, "EcAddDomain(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return ecAddDomain(iValue, iValue2);
  }

  /*Cfg.EcSetDomainAllowOffline(int allow)*/
  nvals = sscanf(myarg_1,
                 "EcSetDomainAllowOffline(%d)",
                 &iValue);

  if (nvals == 1) {
    return ecSetDomAllowOffline(iValue);
  }

  /*Cfg.EcSetAllowOffline(int allow)*/
  nvals = sscanf(myarg_1,
                 "EcSetAllowOffline(%d)",
                 &iValue);

  if (nvals == 1) {
    return ecSetEcAllowOffline(iValue);
  }

  /*Cfg.EcSetDelayECOkAtStartup(int nCycles)*/
  nvals = sscanf(myarg_1, "EcSetDelayECOkAtStartup(%d)", &iValue);

  if (nvals == 1) {
    return ecSetDelayECOkAtStartup(iValue);
  }

  /*int Cfg.SetAxisJogVel(int traj_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisJogVel(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisJogVel(iValue, dValue);
  }

  /*int Cfg.SetAxisEnableAlarmAtHardLimits(int axis_no, int nEnable);*/
  nvals = sscanf(myarg_1,
                 "SetAxisEnableAlarmAtHardLimits(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisEnableAlarmAtHardLimits(iValue, iValue2);
  }

  /*int Cfg.SetAxisAutoEnableTimeout(int axis_no, double timeS);*/
  nvals = sscanf(myarg_1,
                 "SetAxisAutoEnableTimeout(%d,%lf)",
                 &iValue,
                 &dValue);

  if (nvals == 2) {
    return setAxisAutoEnableTimeout(iValue, dValue);
  }

  /*int Cfg.SetAxisAutoDisableAfterTime(int axis_no, double timeS);*/
  nvals = sscanf(myarg_1,
                 "SetAxisAutoDisableAfterTime(%d,%lf)",
                 &iValue,
                 &dValue);

  if (nvals == 2) {
    return setAxisAutoDisableAfterTime(iValue, dValue);
  }

  /*int Cfg.SetAxisEmergDeceleration(int traj_no, double value);*/
  nvals =
    sscanf(myarg_1, "SetAxisEmergDeceleration(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisEmergDeceleration(iValue, dValue);
  }

  /*int Cfg.SetAxisTrajSourceType(int axis_no, int nValue);*/
  nvals = sscanf(myarg_1, "SetAxisTrajSourceType(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisTrajSource(iValue, iValue2);
  }

  /*int Cfg.SetAxisEncSourceType(int axis_no, int nValue);*/
  nvals = sscanf(myarg_1, "SetAxisEncSourceType(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisEncSource(iValue, iValue2);
  }

  /*int Cfg.SetAxisEncScaleNum(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisEncScaleNum(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisEncScaleNum(iValue, dValue);
  }

  /*int Cfg.SetAxisEncScaleDenom(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisEncScaleDenom(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisEncScaleDenom(iValue, dValue);
  }

  /*int Cfg.SetAxisEncBits(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisEncBits(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisEncBits(iValue, iValue2);
  }

  /*int Cfg.SetAxisEncAbsBits(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisEncAbsBits(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisEncAbsBits(iValue, iValue2);
  }

  /*int Cfg.SetAxisEncRawMask(int axis_no, int rawMask);*/
  nvals = sscanf(myarg_1,
                 "SetAxisEncRawMask(%d,%" PRIx64 ")",
                 &iValue,
                 &u64Value);

  if (nvals == 2) {
    return setAxisEncRawMask(iValue, u64Value);
  }

  /*int Cfg.SetAxisEncType(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisEncType(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisEncType(iValue, iValue2);
  }

  /*int Cfg.SetAxisEncOffset(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisEncOffset(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisEncOffset(iValue, dValue);
  }

  /*int Cfg.SetAxisEncRefToOtherEncAtStartup(int axis_no, enc_ref);*/
  nvals = sscanf(myarg_1,
                 "SetAxisEncRefToOtherEncAtStartup(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisEncRefToOtherEncAtStartup(iValue, iValue2);
  }

  /*int Cfg.SetAxisEncDelayCyclesAndEnable(int axis_no, double cycles, int enable);*/
  nvals = sscanf(myarg_1,
                 "SetAxisEncDelayCyclesAndEnable(%d,%lf,%d)",
                 &iValue,
                 &dValue,
                 &iValue2);

  if (nvals == 3) {
    return setAxisEncDelayCyclesAndEnable(iValue,dValue,iValue2);
  }

  /*int Cfg.SetAxisEncEnableRefAtHome(int axis_no, int enbale);*/
  nvals =
    sscanf(myarg_1, "SetAxisEncEnableRefAtHome(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisEncEnableRefAtHome(iValue, iValue2);
  }

  /*int Cfg.AddAxisEnc(int axis_no);*/
  nvals = sscanf(myarg_1, "AddAxisEnc(%d)", &iValue);

  if (nvals == 1) {
    return addAxisEnc(iValue);
  }

  /*int Cfg.SelectAxisEncPrimary(int axis_no, int encIndex);*/
  nvals = sscanf(myarg_1, "SelectAxisEncPrimary(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return selectAxisEncPrimary(iValue, iValue2);
  }

  /*int Cfg.SelectAxisEncCSPDrv(int axis_no, int encIndex);*/
  nvals = sscanf(myarg_1, "SelectAxisEncCSPDrv(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return selectAxisEncCSPDrv(iValue, iValue2);
  }

  /*int Cfg.SelectAxisEncConfig(int axis_no, int encIndex);*/
  nvals = sscanf(myarg_1, "SelectAxisEncConfig(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return selectAxisEncConfig(iValue, iValue2);
  }

  /*int Cfg.SetAxisEncMaxDiffToPrimEnc(int axis_no, double  max_diff);*/
  nvals = sscanf(myarg_1,
                 "SetAxisEncMaxDiffToPrimEnc(%d,%lf)",
                 &iValue,
                 &dValue);

  if (nvals == 2) {
    return setAxisEncMaxDiffToPrimEnc(iValue, dValue);
  }


  /*int createMasterSlaveSM(int index, const char *name, const char *masterGrpName, const char* slaveGrpName);*/

  cIdBuffer[0] =  '\0';
  cIdBuffer2[0] = '\0';
  cIdBuffer3[0] = '\0';

  nvals = sscanf(myarg_1, "CreateMasterSlaveSM(%d,%[^,],%[^,],%[^)])", &iValue, cIdBuffer,cIdBuffer2,cIdBuffer3);

  if (nvals == 4) {
    return createMasterSlaveSM(iValue , cIdBuffer,cIdBuffer2,cIdBuffer3);
  }

  /*int Cfg.LoadAxisEncLookupTable(int axis_no, char *filename); */
  cExprBuffer[0] = '\0';
  nvals = sscanf(myarg_1, "LoadAxisEncLookupTable(%d,%[^)])", &iValue, cExprBuffer);

  if (nvals == 2) {
    return loadAxisEncLookupTable(iValue , cExprBuffer);
  }

  /*int Cfg.SetAxisEncLookupTableEnable(int axis_no, int enable);*/
  nvals = sscanf(myarg_1, "SetAxisEncLookupTableEnable(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisEncLookupTableEnable(iValue, iValue2);
  }

  /*int Cfg.SetAxisEncLookupTableRange(int axis_no, double range);*/
  nvals = sscanf(myarg_1,
                 "SetAxisEncLookupTableRange(%d,%lf)",
                 &iValue,
                 &dValue);

  if (nvals == 2) {
    return setAxisEncLookupTableRange(iValue, dValue);
  }

  /*int Cfg.SetAxisEncLookupTableScale(int axis_no, double scale);*/
  nvals = sscanf(myarg_1,
                 "SetAxisEncLookupTableScale(%d,%lf)",
                 &iValue,
                 &dValue);

  if (nvals == 2) {
    return setAxisEncLookupTableScale(iValue, dValue);
  }

  /*int Cfg.SetAxisCntrlKp(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlKp(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisCntrlKp(iValue, dValue);
  }

  /*int Cfg.SetAxisCntrlKi(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlKi(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisCntrlKi(iValue, dValue);
  }

  /*int Cfg.SetAxisCntrlKd(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlKd(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisCntrlKd(iValue, dValue);
  }

  /*int Cfg.SetAxisCntrlKff(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlKff(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisCntrlKff(iValue, dValue);
  }

  /*int Cfg.SetAxisCntrlDeadband(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlDeadband(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisCntrlDeadband(iValue, dValue);
  }

  /*int Cfg.SetAxisCntrlDeadbandTime(int axis_no, double value);*/
  nvals =
    sscanf(myarg_1, "SetAxisCntrlDeadbandTime(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisCntrlDeadbandTime(iValue, iValue2);
  }

  /*int Cfg.SetAxisCntrlInnerParams(axis_no, kp, ki, kd, tol);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlInnerParams(%d,%lf,%lf,%lf,%lf)",
                 &iValue,
                 &dValue,
                 &dValue2,
                 &dValue3,
                 &dValue4);

  if (nvals == 5) {
    return setAxisCntrlInnerParams(iValue, dValue, dValue2, dValue3, dValue4);
  }

  /*int Cfg.SetAxisCntrlOutHL(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlOutHL(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisCntrlOutHL(iValue, dValue);
  }

  /*int Cfg.SetAxisCntrlOutLL(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlOutLL(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisCntrlOutLL(iValue, dValue);
  }

  /*int Cfg.SetAxisCntrlIPartHL(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlIPartHL(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisCntrlIpartHL(iValue, dValue);
  }

  /*int Cfg.SetAxisCntrlIPartLL(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlIPartLL(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisCntrlIpartLL(iValue, dValue);
  }

  /*int Cfg.SetAxisSoftLimitPosBwd(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisSoftLimitPosBwd(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisSoftLimitPosBwd(iValue, dValue);
  }

  /*int Cfg.SetAxisEnableSoftLimitBwd(int axis_no, double value);*/
  nvals =
    sscanf(myarg_1, "SetAxisEnableSoftLimitBwd(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisEnableSoftLimitBwd(iValue, iValue2);
  }

  /*int Cfg.SetAxisSoftLimitPosFwd(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisSoftLimitPosFwd(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisSoftLimitPosFwd(iValue, dValue);
  }

  /*int Cfg.SetAxisEnableSoftLimitFwd(int axis_no, int value);*/
  nvals =
    sscanf(myarg_1, "SetAxisEnableSoftLimitFwd(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisEnableSoftLimitFwd(iValue, iValue2);
  }

  /*int Cfg.SetAxisEnableAlarmAtSoftLimit(int axis_no, int value);*/
  nvals =
    sscanf(myarg_1, "SetAxisEnableAlarmAtSoftLimit(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisEnableAlarmAtSoftLimit(iValue, iValue2);
  }

  /*int Cfg.SetAxisEnableMotionFunctions(int axis_no,
                                         int enablePos,
                                         int enableConstVelo,
                                         int enableHome);*/
  nvals =
    sscanf(myarg_1, "SetAxisEnableMotionFunctions(%d,%d,%d,%d)",
           &iValue, &iValue2, &iValue3, &iValue4);

  if (nvals == 4) {
    return setAxisEnableMotionFunctions(iValue, iValue2, iValue3, iValue4);
  }

  /*int Cfg.SetAxisMonEnableEncsDiff(int axis_no, int enable);*/
  nvals =
    sscanf(myarg_1, "SetAxisMonEnableEncsDiff(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisEnableCheckEncsDiff(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonAtTargetTol(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisMonAtTargetTol(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisMonAtTargetTol(iValue, dValue);
  }

  /*int Cfg.SetAxisMonAtTargetTime(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisMonAtTargetTime(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisMonAtTargetTime(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonEnableAtTargetMon(int axis_no, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonEnableAtTargetMon(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonEnableAtTargetMon(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonEnableStallMon(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisMonEnableStallMon(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisMonEnableStallMon(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonStallTimeFactor(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisMonStallTimeFactor(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisMonStallTimeFactor(iValue, dValue);
  }

  /*int Cfg.SetAxisMonStallMinTimeOut(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisMonStallMinTimeOut(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisMonStallMinTimeOut(iValue, dValue);
  }

  /*int Cfg.SetAxisMonPosLagTol(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisMonPosLagTol(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisMonPosLagTol(iValue, dValue);
  }

  /*int Cfg.SetAxisMonPosLagTime(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisMonPosLagTime(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisMonPosLagTime(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonEnableLagMon(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisMonEnableLagMon(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisMonEnableLagMon(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonMaxVel(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisMonMaxVel(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisMonMaxVel(iValue, dValue);
  }

  /*int Cfg.SetAxisMonEnableMaxVel(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisMonEnableMaxVel(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisMonEnableMaxVel(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonMaxVelDriveILDelay(int axis_no, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonMaxVelDriveILDelay(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonMaxVelDriveILDelay(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonMaxVelTrajILDelay(int axis_no, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonMaxVelTrajILDelay(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonMaxVelTrajILDelay(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonEnableExtHWInterlock(int axis_no, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonEnableExtHWInterlock(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonEnableExternalInterlock(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonExtHWInterlockPolarity(int axisIndex, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonExtHWInterlockPolarity(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonExtHWInterlockPolarity(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonEnableAnalogInterlock(int axis_no, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonEnableAnalogInterlock(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonEnableAnalogInterlock(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonAnalogInterlockPolarity(int axisIndex, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonAnalogInterlockPolarity(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonAnalogInterlockPolarity(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonAnalogInterlockRawLimit(int axisIndex, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonAnalogInterlockRawLimit(%d,%lf)",
                 &iValue,
                 &dValue);

  if (nvals == 2) {
    return setAxisMonAnalogInterlockRawLimit(iValue, dValue);
  }

  /*int Cfg.SetAxisMonLimitBwdPolarity(int axisIndex, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonLimitBwdPolarity(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonLimitBwdPolarity(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonLimitFwdPolarity(int axisIndex, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonLimitFwdPolarity(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonLimitFwdPolarity(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonHomeSwitchPolarity(int axisIndex, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonHomeSwitchPolarity(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonHomeSwitchPolarity(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonEnableCntrlOutHLMon(int axis_no, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonEnableCntrlOutHLMon(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonEnableCntrlOutHLMon(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonEnableVelocityDiff(int axis_no, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonEnableVelocityDiff(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonEnableVelocityDiff(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonVelDiffTrajILDelay(int axis_no, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonVelDiffTrajILDelay(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonVelDiffTrajILDelay(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonVelDiffDriveILDelay(int axis_no, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisMonVelDiffDriveILDelay(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisMonVelDiffDriveILDelay(iValue, iValue2);
  }

  /*int Cfg.SetAxisMonVelDiffTol(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisMonVelDiffTol(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisMonVelDiffTol(iValue, dValue);
  }

  /*int Cfg.SetAxisMonCntrlOutHL(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisMonCntrlOutHL(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisMonCntrlOutHL(iValue, dValue);
  }

  /*int Cfg.SetAxisMonLatchLimit(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisMonLatchLimit(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisMonLatchLimit(iValue, iValue2);
  }

  /*int Cfg.SetAxisLimitSwitchBwdPLCOverride(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisLimitSwitchBwdPLCOverride(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisLimitSwitchBwdPLCOverride(iValue, iValue2);
  }

  /*int Cfg.SetAxisLimitSwitchFwdPLCOverride(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisLimitSwitchFwdPLCOverride(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisLimitSwitchFwdPLCOverride(iValue, iValue2);
  }

  /*int Cfg.SetAxisHomeSwitchPLCOverride(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisHomeSwitchPLCOverride(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisHomeSwitchPLCOverride(iValue, iValue2);
  }

  /*int Cfg.SetAxisHomeSwitchEnable(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisHomeSwitchEnable(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisHomeSwitchEnable(iValue, iValue2);
  }
  
  /*int Cfg.SetAxisDrvScaleNum(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisDrvScaleNum(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisDrvScaleNum(iValue, dValue);
  }

  /*int Cfg.SetAxisDrvScaleDenom(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisDrvScaleDenom(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisDrvScaleDenom(iValue, dValue);
  }

  /*int Cfg.SetAxisDrvVelSetOffsetRaw(int axis_no, double value);*/
  nvals =
    sscanf(myarg_1, "SetAxisDrvVelSetOffsetRaw(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisDrvVelSetOffsetRaw(iValue, dValue);
  }

  /*int Cfg.SetAxisDrvBrakeEnable(int axis_no, int enable);*/
  nvals = sscanf(myarg_1, "SetAxisDrvBrakeEnable(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisDrvBrakeEnable(iValue, iValue2);
  }

  /*int Cfg.SetAxisDrvBrakeOpenDelayTime(int axis_no, int delayTime);*/
  nvals = sscanf(myarg_1,
                 "SetAxisDrvBrakeOpenDelayTime(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisDrvBrakeOpenDelayTime(iValue, iValue2);
  }

  /*int Cfg.SetAxisDrvBrakeCloseAheadTime(int axis_no, int aheadTime);*/
  nvals = sscanf(myarg_1,
                 "SetAxisDrvBrakeCloseAheadTime(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisDrvBrakeCloseAheadTime(iValue, iValue2);
  }

  /*int Cfg.SetAxisDrvStateMachineTimeout(int axis_no, double seconds);*/
  nvals = sscanf(myarg_1,
                 "SetAxisDrvStateMachineTimeout(%d,%lf)",
                 &iValue,
                 &dValue);

  if (nvals == 2) {
    return setAxisDrvStateMachineTimeout(iValue, dValue);
  }

  /*int Cfg.SetAxisDrvReduceTorqueEnable(int axis_no, int enable);*/
  nvals = sscanf(myarg_1,
                 "SetAxisDrvReduceTorqueEnable(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisDrvReduceTorqueEnable(iValue, iValue2);
  }

  /*int Cfg.SetAxisDrvType(int axis_no, int type);*/
  nvals = sscanf(myarg_1, "SetAxisDrvType(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    LOGERR(
      "%s/%s:%d: Command obsolete. Use Cfg.CreateAxis(<id>,<type>,<drvType>) instead  (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      ERROR_MAIN_OBSOLETE_COMMAND);

    // return setAxisDrvType(iValue, iValue2);
  }

  /*int Cfg.SetAxisAutoModeCmdHoming(int axis_no, int cmd);*/
  nvals =
    sscanf(myarg_1, "SetAxisAutoModeCmdHoming(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisAutoModeCmdHoming(iValue, iValue2);
  }

  /*int Cfg.SetAxisAutoModeCmdMotion(int axis_no, int cmd);*/
  nvals =
    sscanf(myarg_1, "SetAxisAutoModeCmdMotion(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisAutoModeCmdMotion(iValue, iValue2);
  }

  /*int Cfg.SetAxisModRange(int axis_no, double range);*/
  nvals = sscanf(myarg_1, "SetAxisModRange(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisModRange(iValue, dValue);
  }

  /*int Cfg.SetAxisModType(int axis_no, int type);*/
  nvals = sscanf(myarg_1, "SetAxisModType(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisModType(iValue, iValue2);
  }

  /*int Cfg.SetAxisDisableAtErrorReset(int axis_no, int disable);*/
  nvals = sscanf(myarg_1,
                 "SetAxisDisableAtErrorReset(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisDisableAtErrorReset(iValue, iValue2);
  }

  /*int Cfg.SetAxisAllowSourceChangeWhenEnabled(int axis_no, int allow);*/
  nvals = sscanf(myarg_1,
                 "SetAxisAllowSourceChangeWhenEnabled(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisAllowSourceChangeWhenEnabled(iValue, iValue2);
  }

  /*int Cfg.SetDiagAxisIndex(int axis_no);*/
  nvals = sscanf(myarg_1, "SetDiagAxisIndex(%d)", &iValue);

  if (nvals == 1) {
    return setDiagAxisIndex(iValue);
  }

  /*int Cfg.SetDiagAxisFreq(int nFreq);*/
  nvals = sscanf(myarg_1, "SetDiagAxisFreq(%d)", &iValue);

  if (nvals == 1) {
    return setDiagAxisFreq(iValue);
  }

  /*int Cfg.SetDiagAxisEnable(int nDiag);*/
  nvals = sscanf(myarg_1, "SetDiagAxisEnable(%d)", &iValue);

  if (nvals == 1) {
    return setDiagAxisEnable(iValue);
  }

  /*int Cfg.SetAxisHomePosition(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisHomePosition(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisHomePos(iValue, dValue);
  }

  /*int Cfg.SetAxisHomeVelTowardsCam(int axis_no, double value);*/
  nvals =
    sscanf(myarg_1, "SetAxisHomeVelTowardsCam(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisHomeVelTowardsCam(iValue, dValue);
  }

  /*int Cfg.SetAxisHomeVelOffCam(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisHomeVelOffCam(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisHomeVelOffCam(iValue, dValue);
  }

  /*int Cfg.SetAxisHomeLatchCountOffset(int axis_no, int count);*/
  nvals = sscanf(myarg_1,
                 "SetAxisHomeLatchCountOffset(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    printf("WARNING: The command Cfg.SetAxisHomeLatchCountOffset() will be obsolete in newer versions."
           "Please use Cfg.SetAxisEncHomeLatchCountOffset() instead.\n");
    return setAxisEncHomeLatchCountOffset(iValue, iValue2);
  }

  // new better name.. still support old name.

  /*int Cfg.SetAxisEncHomeLatchCountOffset(int axis_no, int count);*/
  nvals = sscanf(myarg_1,
                 "SetAxisEncHomeLatchCountOffset(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisEncHomeLatchCountOffset(iValue, iValue2);
  }

  /*int Cfg.SetAxisEncInvHwReady(int axis_no, int count);*/
  nvals = sscanf(myarg_1,
                 "SetAxisEncInvHwReady(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisEncInvHwReady(iValue, iValue2);
  }

  /*int Cfg.SetAxisErrorId(int axis_no, int errorid);*/
  nvals = sscanf(myarg_1,
                 "SetAxisErrorId(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisErrorId(iValue, iValue2);
  }

  /*int Cfg.SetEnableFuncCallDiag(int nEnable);*/
  nvals = sscanf(myarg_1, "SetEnableFuncCallDiag(%d)", &iValue);

  if (nvals == 1) {
    return setEnableFunctionCallDiag(iValue);
  }

  /*int Cfg.SetTraceMask(int mask);*/
  nvals = sscanf(myarg_1, "SetTraceMask(%d)", &iValue);

  if (nvals == 1) {
    debug_print_flags = iValue;
    return 0;
  }

  /*int Cfg.SetTraceMaskBit(int bitToSet, int value);*/
  nvals = sscanf(myarg_1, "SetTraceMaskBit(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    WRITE_DIAG_BIT(iValue, iValue2);
    return 0;
  }

  /*int Cfg.SetEnableTimeDiag(int nEnable);*/
  nvals = sscanf(myarg_1, "SetEnableTimeDiag(%d)", &iValue);

  if (nvals == 1) {
    return setEnableTimeDiag(iValue);
  }

  /*int Cfg.SetAxisBlockCom(int axis_no, int block);*/
  nvals = sscanf(myarg_1, "SetAxisBlockCom(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisBlockCom(iValue, iValue2);
  }

  /*int Cfg.SetAxisTrajStartPos(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisTrajStartPos(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisTrajStartPos(iValue, dValue);
  }

  /*int Cfg.SetAxisJerk(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisJerk(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisJerk(iValue, dValue);
  }

  /*int Cfg.SetAxisAcc(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisAcc(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisAcceleration(iValue, dValue);
  }

  /*int Cfg.SetAxisDec(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisDec(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisDeceleration(iValue, dValue);
  }

  /*int Cfg.SetAxisVel(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisVel(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisTargetVel(iValue, dValue);
  }

  /*int Cfg.SetAxisVelAccDecTime(int axis_no, double vel,double timeToVel);*/

  /* Set Velcoity acceleration and deceleration
   * Acceleration and deceleration is defined by time to reach velocity.
   * (because motor record uses this concept)
  */
  nvals = sscanf(myarg_1,
                 "SetAxisVelAccDecTime(%d,%lf,%lf)",
                 &iValue,
                 &dValue,
                 &dValue2);

  if (nvals == 3) {
    double acc = 0;

    if (dValue2 != 0) {
      acc = fabs(dValue / dValue2);
    }

    // Set velocity
    int errorCode = setAxisTargetVel(iValue, dValue);

    if (errorCode) {
      return errorCode;
    }

    // Set acceleration
    errorCode = setAxisAcceleration(iValue, acc);

    if (errorCode) {
      return errorCode;
    }

    // Set deceleration
    return setAxisDeceleration(iValue, acc);
  }

  /*int Cfg.SetAxisPLCTrajVelFilterEnable(int axis_no, int enable);*/
  nvals = sscanf(myarg_1,
                 "SetAxisPLCTrajVelFilterEnable(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisPLCTrajVelFilterEnable(iValue, iValue2);
  }

  /*int Cfg.SetAxisPLCTrajVelFilterSize(int axis_no, int size);*/
  nvals = sscanf(myarg_1,
                 "SetAxisPLCTrajVelFilterSize(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisPLCTrajVelFilterSize(iValue, iValue2);
  }

  /*int Cfg.SetAxisPLCEncVelFilterEnable(int axis_no, int enable);*/
  nvals = sscanf(myarg_1,
                 "SetAxisPLCEncVelFilterEnable(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisPLCEncVelFilterEnable(iValue, iValue2);
  }

  /*int Cfg.SetAxisPLCEncVelFilterSize(int axis_no, int size);*/
  nvals = sscanf(myarg_1,
                 "SetAxisPLCEncVelFilterSize(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisPLCEncVelFilterSize(iValue, iValue2);
  }

  /*int Cfg.SetAxisEncVelFilterSize(int axis_no, int size);*/
  nvals = sscanf(myarg_1,
                 "SetAxisEncVelFilterSize(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisEncVelFilterSize(iValue, iValue2);
  }

  /*int Cfg.SetAxisEncVelFilterEnable(int axis_no, int size);*/
  nvals = sscanf(myarg_1,
                 "SetAxisEncVelFilterEnable(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisEncVelFilterEnable(iValue, iValue2);
  }

  /*int Cfg.SetAxisEncPosFilterSize(int axis_no, int size);*/
  nvals = sscanf(myarg_1,
                 "SetAxisEncPosFilterSize(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisEncPosFilterSize(iValue, iValue2);
  }

  /*int Cfg.SetAxisEncPosFilterEnable(int axis_no, int size);*/
  nvals = sscanf(myarg_1,
                 "SetAxisEncPosFilterEnable(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisEncPosFilterEnable(iValue, iValue2);
  }

  /*int Cfg.AppendAxisPLCExpr(int axis_no,char *cExpr); */
  nvals = sscanf(myarg_1,
                 "AppendAxisPLCExpr(%d)=%[^\n]",
                 &iValue,
                 cExprBuffer);

  if (nvals == 1) {
    cExprBuffer[0] = '\0';
  }

  if (nvals >= 1) {  // allow empty expression
    // Change all # to ; (since ; is used as command delimiter
    // in tcpip communication)
    size_t str_len = strlen(cExprBuffer);

    int i = 0;

    for (i = 0; i < str_len; i++) {
      if (cExprBuffer[i] == TRANSFORM_EXPR_LINE_END_CHAR) {
        cExprBuffer[i] = ';';
      }
    }
    return appendAxisPLCExpr(iValue, cExprBuffer);
  }

  // nvals = sscanf(myarg_1, "SetPLCExpr(%d,\"%[^\"])",&iValue,cExprBuffer);
  nvals = sscanf(myarg_1, "SetPLCExpr(%d)=%[^\n]", &iValue, cExprBuffer);

  if (nvals == 1) {
    cExprBuffer[0] = '\0';
  }

  if (nvals >= 1) {  // allow empty expression
    // Change all # to ; (since ; is used as command
    // delimiter in tcpip communication)
    size_t str_len = strlen(cExprBuffer);

    int i = 0;

    for (i = 0; i < str_len; i++) {
      if (cExprBuffer[i] == TRANSFORM_EXPR_LINE_END_CHAR) {
        cExprBuffer[i] = ';';
      }
    }
    return setPLCExpr(iValue, cExprBuffer);
  }

  /*int Cfg.AppendPLCExpr(int index,char *cExpr); */
  nvals = sscanf(myarg_1, "AppendPLCExpr(%d)=%[^\n]", &iValue, cExprBuffer);

  if (nvals == 1) {
    cExprBuffer[0] = '\0';
  }

  if (nvals >= 1) {  // allow empty expression
    // Change all # to ; (since ; is used as command
    // delimiter in tcpip communication)
    size_t str_len = strlen(cExprBuffer);

    int i = 0;

    for (i = 0; i < str_len; i++) {
      if (cExprBuffer[i] == TRANSFORM_EXPR_LINE_END_CHAR) {
        cExprBuffer[i] = ';';
      }
    }
    return appendPLCExpr(iValue, cExprBuffer);
  }

  /*int Cfg.AppendAxisPLCExpr(int index,char *cExpr); */
  nvals =
    sscanf(myarg_1, "AppendAxisPLCExpr(%d)=%[^\n]", &iValue, cExprBuffer);

  if (nvals == 1) {
    cExprBuffer[0] = '\0';
  }

  if (nvals >= 1) {  // allow empty expression
    // Change all # to ; (since ; is used as command
    // delimiter in tcpip communication)
    size_t str_len = strlen(cExprBuffer);

    int i = 0;

    for (i = 0; i < str_len; i++) {
      if (cExprBuffer[i] == TRANSFORM_EXPR_LINE_END_CHAR) {
        cExprBuffer[i] = ';';
      }
    }

    // Axis plcs is indexed "above" normal PLCs in the PLC array
    return appendPLCExpr(iValue + ECMC_MAX_PLCS, cExprBuffer);
  }

  /*int Cfg.LoadAxisPLCFile(int index,char *cExpr); */
  nvals = sscanf(myarg_1, "LoadAxisPLCFile(%d,%[^)])", &iValue, cExprBuffer);

  if (nvals == 2) {
    // Axis plcs is indexed "above" normal PLCs in the PLC array
    return loadPLCFile(iValue + ECMC_MAX_PLCS, cExprBuffer);
  }

  /*int Cfg.LoadPLCFile(int index,char *cExpr); */
  nvals = sscanf(myarg_1, "LoadPLCFile(%d,%[^)])", &iValue, cExprBuffer);

  if (nvals == 2) {
    return loadPLCFile(iValue, cExprBuffer);
  }

  /*int Cfg.LoadPLCLibFile(int index,char *cExpr); */
  nvals = sscanf(myarg_1, "LoadPLCLibFile(%d,%[^)])", &iValue, cExprBuffer);

  if (nvals == 2) {
    return loadPLCLibFile(iValue, cExprBuffer);
  }

  /*int Cfg.ClearPLCExpr(int plcIndex);*/
  nvals = sscanf(myarg_1, "ClearPLCExpr(%d)", &iValue);

  if (nvals == 1) {
    return clearPLCExpr(iValue);
  }

  /*int Cfg.CompilePLC(int plcIndex);*/
  nvals = sscanf(myarg_1, "CompilePLC(%d)", &iValue);

  if (nvals == 1) {
    return compilePLCExpr(iValue);
  }

  /*int Cfg.CompileAxisPLC(int plcIndex);*/
  nvals = sscanf(myarg_1, "CompileAxisPLC(%d)", &iValue);

  if (nvals == 1) {
    return compileAxisPLCExpr(iValue);
  }

  /*int Cfg.SetAxisAllowCommandsFromPLC(int master_axis_no,
    int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisAllowCommandsFromPLC(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisAllowCommandsFromPLC(iValue, iValue2);
  }

  /*int Cfg.SetAxisPLCEnable(int master_axis_no, int value);*/
  nvals = sscanf(myarg_1,
                 "SetAxisPLCEnable(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setAxisPLCEnable(iValue, iValue2);
  }

  /*int Cfg.LoadPlugin(int pluginId, char *cFilename, char *configString); */
  nvals = sscanf(myarg_1,
                 "LoadPlugin(%d,%[^,],%[^)])",
                 &iValue,
                 cIdBuffer,
                 cIdBuffer2);

  if (nvals == 3) {
    return loadPlugin(iValue, cIdBuffer, cIdBuffer2);
  }

  /*int Cfg.LoadSafetyPlugin(char *cFilename, char *configString); */
  nvals = sscanf(myarg_1,
                 "LoadSafetyPlugin(%[^,],%[^)])",
                 cIdBuffer,
                 cIdBuffer2);

  if (nvals == 2) {
    return loadSafetyPlugin(cIdBuffer, cIdBuffer2);
  }

  /*int Cfg.LoadPlugin(int pluginId, char *cFilename); */
  nvals = sscanf(myarg_1, "LoadPlugin(%d,%[^)])", &iValue, cIdBuffer);

  if (nvals == 2) {
    return loadPlugin(iValue, cIdBuffer, "");
  }

  /*int Cfg.ReportPlugin(int pluginId); */
  nvals = sscanf(myarg_1, "ReportPlugin(%d)", &iValue);

  if (nvals == 1) {
    return reportPlugin(iValue);
  }

  /*int Cfg.SetAxisSeqTimeout(int axis_no, int value);  IN seconds!!*/
  nvals = sscanf(myarg_1, "SetAxisSeqTimeout(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisSeqTimeout(iValue, iValue2);
  }

  /*int Cfg.SetAxisHomeAcc(int axis_no, int value);  IN seconds!!*/
  nvals = sscanf(myarg_1, "SetAxisHomeAcc(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisHomeAcc(iValue, dValue);
  }

  /*int Cfg.SetAxisHomeDec(int axis_no, int value);  IN seconds!!*/
  nvals = sscanf(myarg_1, "SetAxisHomeDec(%d,%lf)", &iValue, &dValue);

  if (nvals == 2) {
    return setAxisHomeDec(iValue, dValue);
  }

  /*int Cfg.SetAxisHomeSeqId(int axis_no, int value);  IN seconds!!*/
  nvals = sscanf(myarg_1, "SetAxisHomeSeqId(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisHomeSeqId(iValue, iValue2);
  }

  /*int Cfg.SetAxisHomePostMoveEnable(int axis_no, int value); */
  nvals =
    sscanf(myarg_1, "SetAxisHomePostMoveEnable(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setAxisHomePostMoveEnable(iValue, iValue2);
  }

  /*int Cfg.SetAxisHomePostMoveTargetPosition(int axis_no, int value); */
  nvals = sscanf(myarg_1,
                 "SetAxisHomePostMoveTargetPosition(%d,%lf)",
                 &iValue,
                 &dValue);

  if (nvals == 2) {
    return setAxisHomePostMoveTargetPosition(iValue, dValue);
  }

  /*int Cfg.CreateStorage(int index, int elements, int bufferType);*/
  nvals = sscanf(myarg_1,
                 "CreateStorage(%d,%d,%d)",
                 &iValue,
                 &iValue2,
                 &iValue3);

  if (nvals == 3) {
    return createDataStorage(iValue, iValue2, iValue3);
  }

  /*int Cfg.SetStorageEnablePrintouts(int indexStorage,int enable);*/
  nvals =
    sscanf(myarg_1, "SetStorageEnablePrintouts(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    return setStorageEnablePrintouts(iValue, iValue2);
  }

  /*int Cfg.PrintDataStorage(int indexStorage);*/
  nvals = sscanf(myarg_1, "PrintDataStorage(%d)", &iValue);

  if (nvals == 1) {
    return printStorageBuffer(iValue);
  }

  /*int Cfg.SetDataStorageCurrentDataIndex(0,10)"*/
  nvals = sscanf(myarg_1,
                 "SetDataStorageCurrentDataIndex(%d,%d)",
                 &iValue,
                 &iValue2);

  if (nvals == 2) {
    return setDataStorageCurrentDataIndex(iValue, iValue2);
  }

  /*int Cfg.ClearStorage(int indexStorage);*/
  nvals = sscanf(myarg_1, "ClearStorage(%d)", &iValue);

  if (nvals == 1) {
    return clearStorage(iValue);
  }

  cExprBuffer[0] = '\0';
  /*int Cfg.LoadLUTFile(int index,char *cExpr); */
  nvals = sscanf(myarg_1, "LoadLUTFile(%d,%[^)])", &iValue, cExprBuffer);

  if (nvals == 2) {
    return loadLUT(iValue, cExprBuffer);
  }

  /*int Cfg.IocshCmd=<command string>*/
  nvals = sscanf(myarg_1, "IocshCmd=%[^\n]", cExprBuffer);

  if (nvals == 1) {
    return iocshCmd(cExprBuffer);
  }

  return ERROR_MAIN_PARSER_UNKOWN_CMD;
}

static int handleTwincatSyntax(const char           *myarg_1,
                               ecmcOutputBufferType *buffer) {
  int motor_axis_no = 0;
  int iValue        = 0;
  double fValue     = 0;

  /* From here on, only M1. commands */
  /* e.g. M1.nCommand=3 */
  int nvals = sscanf(myarg_1, "M%d.", &motor_axis_no);

  if (nvals != 1) {
    SEND_OK_OR_ERROR_AND_RETURN(ERROR_MAIN_PARSER_UNKOWN_CMD);
  }
  AXIS_CHECK_RETURN_USED_BUFFER(motor_axis_no);
  myarg_1 = strchr(myarg_1, '.');

  if (!myarg_1) {
    SEND_OK_OR_ERROR_AND_RETURN(ERROR_MAIN_PARSER_UNKOWN_CMD);
  }
  myarg_1++;  /* Jump over '.' */

  /* Main.Mx.stAxisStatusV2? */
  if (0 == strcmp(myarg_1, "stAxisStatusV2?")) {
    char tempBuffer[1024];  // TODO consider more efficient implementations
    int  error =
      getAxisStatusStructV2(motor_axis_no, &tempBuffer[0], sizeof(tempBuffer));

    if (error) {
      cmd_buf_printf(buffer, "Error: %d", error);
      return 0;
    }
    cmd_buf_printf(buffer, "%s", tempBuffer);
    return 0;
  }

  /* sErrorMessage?  */
  if (!strcmp(myarg_1, "sErrorMessage?")) {
    cmd_buf_printf(buffer, "%s",
                   getErrorString(getAxisErrorID(motor_axis_no)));
    return 0;
  }

  /* nCommand=3 */
  nvals = sscanf(myarg_1, "nCommand=%d", &iValue);

  if (nvals == 1) {
    SEND_OK_OR_ERROR_AND_RETURN(setAxisCommand(motor_axis_no, iValue));
  }

  /* nCmdData=1 */
  nvals = sscanf(myarg_1, "nCmdData=%d", &iValue);

  if (nvals == 1) {
    SEND_OK_OR_ERROR_AND_RETURN(setAxisCmdData(motor_axis_no, iValue));
  }

  /* bEnable= */
  nvals = sscanf(myarg_1, "bEnable=%d", &iValue);

  if (nvals == 1) {
    SEND_OK_OR_ERROR_AND_RETURN(setAxisEnable(motor_axis_no, iValue));
  }

  /* bExecute= */
  nvals = sscanf(myarg_1, "bExecute=%d", &iValue);

  if (nvals == 1) {
    SEND_OK_OR_ERROR_AND_RETURN(setAxisExecute(motor_axis_no, iValue));
  }

  /* bReset= */
  nvals = sscanf(myarg_1, "bReset=%d", &iValue);

  if (nvals == 1) {
    SEND_OK_OR_ERROR_AND_RETURN(axisErrorReset(motor_axis_no, iValue));
  }

  /* fPosition=100 */
  nvals = sscanf(myarg_1, "fPosition=%lf", &fValue);

  if (nvals == 1) {
    SEND_OK_OR_ERROR_AND_RETURN(setAxisTargetPos(motor_axis_no, fValue));
  }

  /* fVelocity=20 */
  nvals = sscanf(myarg_1, "fVelocity=%lf", &fValue);

  if (nvals == 1) {
    SEND_OK_OR_ERROR_AND_RETURN(setAxisTargetVel(motor_axis_no, fValue));
  }

  /* fAcceleration=1000 */
  nvals = sscanf(myarg_1, "fAcceleration=%lf", &fValue);

  if (nvals == 1) {
    SEND_OK_OR_ERROR_AND_RETURN(setAxisAcceleration(motor_axis_no, fValue));
  }

  /* fDeceleration=1000 */
  nvals = sscanf(myarg_1, "fDeceleration=%lf", &fValue);

  if (nvals == 1) {
    SEND_OK_OR_ERROR_AND_RETURN(setAxisDeceleration(motor_axis_no, fValue));
  }

  /* fHomePosition=100 */
  nvals = sscanf(myarg_1, "fHomePosition=%lf", &fValue);

  if (nvals == 1) {
    SEND_OK_OR_ERROR_AND_RETURN(setAxisHomePos(motor_axis_no, fValue));
  }

  if (0 == strcmp(myarg_1, "stAxisStatus?")) {
    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisEnabled(motor_axis_no, &iValue));
    int bEnable = iValue;

    int bReset = 0;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisExecute(motor_axis_no, &iValue));
    int bExecute = iValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisCommand(motor_axis_no, &iValue));
    unsigned nCommand = iValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisCmdData(motor_axis_no, &iValue));
    unsigned nCmdData = iValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisTargetVel(motor_axis_no, &fValue));
    double fVelocity = fValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisTargetPos(motor_axis_no, &fValue));
    double fPosition = fValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisAcceleration(motor_axis_no,
                                                       &fValue));
    double fAcceleration = fValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisDeceleration(motor_axis_no,
                                                       &fValue));
    double fDecceleration = fValue;

    int bJogFwd = 0;
    int bJogBwd = 0;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisAtHardFwd(motor_axis_no, &iValue));
    int bLimitFwd = iValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisAtHardBwd(motor_axis_no, &iValue));
    int bLimitBwd = iValue;


    double fOverride = 100;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisAtHome(motor_axis_no, &iValue));
    int bHomeSensor = iValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisEnabled(motor_axis_no, &iValue));
    int bEnabled = iValue;

    int bError        = getAxisError(motor_axis_no);
    unsigned nErrorId = getAxisErrorID(motor_axis_no);

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisEncVelAct(motor_axis_no, &fValue));
    double fActVelocity = fValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisEncPosAct(motor_axis_no, &fValue));
    double fActPostion = fValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisCntrlError(motor_axis_no, &fValue));
    double fActDiff = fValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisEncHomed(motor_axis_no, &iValue));
    int bHomed = iValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisBusy(motor_axis_no, &iValue));
    int bBusy = iValue;

    //    cmd_buf_printf("Main.M%d.stAxisStatus="
    //                   "%d,%d,%d,%u,%u,%g,%g,%g,%g,%d,"
    //                   "%d,%d,%d,%g,%d,%d,%d,%u,%g,%g,%g,%d,%d",
    //                   motor_axis_no,
    //                   bEnable,        /*  1 */
    //                   bReset,         /*  2 */
    //                   bExecute,       /*  3 */
    //                   nCommand,       /*  4 */
    //                   nCmdData,       /*  5 */
    //                   fVelocity,      /*  6 */
    //                   fPosition,      /*  7 */
    //                   fAcceleration,  /*  8 */
    //                   fDecceleration, /*  9 */
    //                   bJogFwd,        /* 10 */
    //                   bJogBwd,        /* 11 */
    //                   bLimitFwd,      /* 12 */
    //                   bLimitBwd,      /* 13 */
    //                   fOverride,      /* 14 */
    //                   bHomeSensor,    /* 15 */
    //                   bEnabled,       /* 16 */
    //                   bError,         /* 17 */
    //                   nErrorId,       /* 18 */
    //                   fActVelocity,   /* 19 */
    //                   fActPostion,    /* 20 */
    //                   fActDiff,       /* 21 */
    //                   bHomed,         /* 22 */
    //                   bBusy           /* 23 */
    //                   );
    cmd_buf_printf(buffer, "Main.M%d.stAxisStatus="
                           "%d,%d,%d,%u,%u,%g,%g,%g,%g,%d,"
                           "%d,%d,%d,%g,%d,%d,%d,%u,%g,%g,%g,%d,%d",
                   motor_axis_no, /*  0 */
                   bEnable, /*  1 */
                   bReset, /*  2 */
                   bExecute, /*  3 */
                   nCommand, /*  4 */
                   nCmdData, /*  5 */
                   fVelocity, /*  6 */
                   fPosition, /*  7 */
                   fAcceleration, /*  8 */
                   fDecceleration, /*  9 */
                   bJogFwd, /* 10 */
                   bJogBwd, /* 11 */
                   bLimitFwd, /* 12 */
                   bLimitBwd, /* 13 */
                   fOverride, /* 14 */
                   bHomeSensor, /* 15 */
                   bEnabled, /* 16 */
                   bError, /* 17 */
                   nErrorId, /* 18 */
                   fActVelocity, /* 19 */
                   fActPostion, /* 20 */
                   fActDiff, /* 21 */
                   bHomed, /* 22 */
                   bBusy); /* 23 */
    return 0;
  }

  if (0 == strcmp(myarg_1, "bBusy?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisBusy(motor_axis_no, &iValue));
  }

  /* bError?  */
  if (!strcmp(myarg_1, "bError?")) {
    iValue = getAxisError(motor_axis_no);
    cmd_buf_printf(buffer, "%d", iValue);
    return 0;
  }

  /* nErrorId? */
  if (!strcmp(myarg_1, "nErrorId?")) {
    iValue = getAxisErrorID(motor_axis_no);
    cmd_buf_printf(buffer, "%d", iValue);
    return 0;
  }

  /* bEnable? */
  if (!strcmp(myarg_1, "bEnable?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEnable(motor_axis_no, &iValue));
  }

  /* bEnabled? */
  if (!strcmp(myarg_1, "bEnabled?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEnabled(motor_axis_no,
                                                       &iValue));
  }

  /* bExecute? */
  if (!strcmp(myarg_1, "bExecute?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisExecute(motor_axis_no,
                                                       &iValue));
  }

  /* bReset? */
  if (!strcmp(myarg_1, "bReset?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisReset(motor_axis_no, &iValue));
  }

  /* bHomeSensor? */
  if (0 == strcmp(myarg_1, "bHomeSensor?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisAtHome(motor_axis_no, &iValue));
  }

  /* bLimitBwd? */
  if (0 == strcmp(myarg_1, "bLimitBwd?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisAtHardBwd(motor_axis_no,
                                                         &iValue));
  }

  /* bLimitFwd? */
  if (0 == strcmp(myarg_1, "bLimitFwd?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisAtHardFwd(motor_axis_no,
                                                         &iValue));
  }

  /* bHomed? */
  if (0 == strcmp(myarg_1, "bHomed?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEncHomed(motor_axis_no,
                                                        &iValue));
  }

  /* bDone? */
  if (0 == strcmp(myarg_1, "bDone?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisDone(motor_axis_no,
                                                    &iValue));
  }

  /* fActPosition? */
  if (0 == strcmp(myarg_1, "fActPosition?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisEncPosAct(motor_axis_no,
                                                            &fValue));
  }

  /* fActVelocity? */
  if (0 == strcmp(myarg_1, "fActVelocity?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisEncVelAct(motor_axis_no,
                                                            &fValue));
  }

  /* fVelocity? */
  if (0 == strcmp(myarg_1, "fVelocity?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisTargetVel(motor_axis_no,
                                                            &fValue));
  }

  /* fPosition? */
  if (0 == strcmp(myarg_1, "fPosition?")) {
    /* The "set" value */
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisTargetPos(motor_axis_no,
                                                            &fValue));
  }

  /* nCommand? */
  if (0 == strcmp(myarg_1, "nCommand?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisCommand(motor_axis_no,
                                                       &iValue));
  }

  /* nCmdData? */
  if (0 == strcmp(myarg_1, "nCmdData?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisCmdData(motor_axis_no,
                                                       &iValue));
  }

  /*nMotionAxisID? */
  if (0 == strcmp(myarg_1, "nMotionAxisID?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisID(motor_axis_no, &iValue));
  }

  /*fAcceleration? */
  if (0 == strcmp(myarg_1, "fAcceleration?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisAcceleration(motor_axis_no,
                                                               &fValue));
  }

  /*fDeceleration? */
  if (0 == strcmp(myarg_1, "fDeceleration?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisDeceleration(motor_axis_no,
                                                               &fValue));
  }

  return 0;
}

int motorHandleOneArg(const char *myarg_1, ecmcOutputBufferType *buffer) {
  int iValue        = 0;
  int iValue2       = 0;
  int iValue3       = 0;
  int iValue4       = 0;
  int iValue5       = 0;
  uint32_t u32Value = 0;
  uint64_t u64Value = 0;
  uint64_t i64Value = 0;
  double   fValue   = 0;
  int motor_axis_no = 0;
  int nvals         = 0;
  double dValue1, dValue2, dValue3, dValue4;

  //if (buffer->buffer == NULL) {
  //  return ERROR_MAIN_PARSER_BUFFER_NULL;
  //}

  if (!ecmcInitDone) {
    ecmcInitThread();
    ecmcInitDone = 1;
  }

  // Check Command length
  if (strlen(myarg_1) >= ECMC_CMD_MAX_SINGLE_CMD_LENGTH - 1) {
    LOGERR("%s/%s:%d: Command to long. Command buffer size %d :(0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ECMC_CMD_MAX_SINGLE_CMD_LENGTH,
           ERROR_MAIN_PARSER_CMD_TO_LONG);
    return ERROR_MAIN_PARSER_CMD_TO_LONG;
  }

  /* Main.*/
  if (!strncmp(myarg_1, Main_dot_str, strlen(Main_dot_str))) {
    myarg_1 += strlen(Main_dot_str);
    return handleTwincatSyntax(myarg_1, buffer);
  }

  /* ADSPORT= */
  if (!strncmp(myarg_1, ADSPORT_equals_str, strlen(ADSPORT_equals_str))) {
    int err_code;
    int nvals;
    unsigned int adsport;
    const char  *myarg_tmp;
    char dot_tmp = 0;

    myarg_1 += strlen(ADSPORT_equals_str);
    nvals    = sscanf(myarg_1, "%u/.ADR%c", &adsport, &dot_tmp);

    if ((nvals == 2) && (dot_tmp == '.')) {
      /* .ADR commands are handled here */
      err_code = motorHandleADS_ADR(myarg_1, buffer);

      if (err_code == -1)return 0;

      SEND_OK_OR_ERROR_AND_RETURN(err_code);
      return 0;
    }
    nvals = sscanf(myarg_1, "%u/", &adsport);

    if (nvals != 1) {
      return 0;
    }
    myarg_tmp = strchr(myarg_1, '/');

    if (!myarg_tmp) {
      return 0;
    }

    /* Jump over digits and '/' */
    myarg_1 = myarg_tmp + 1;
  }

  // Check if configuration command
  if (0 == strncmp(myarg_1, Cfg_dot_str, strlen(Cfg_dot_str))) {
    myarg_1 += strlen(Cfg_dot_str);
    SEND_OK_OR_ERROR_AND_RETURN(handleCfgCommand(myarg_1));
  }

  /* .THIS.sFeatures? */
  if (0 == strcmp(myarg_1, sFeaturesQ_str)) {
    cmd_buf_printf(buffer, "%s", "ecmc;stv2");
    return 0;
  }

  /* GetControllerError()*/
  if (!strcmp(myarg_1, "GetControllerError()")) {
    cmd_buf_printf(buffer, "%d", getControllerError());
    return 0;
  }

  /* ControllerErrorReset()*/
  if (!strcmp(myarg_1, "ControllerErrorReset()")) {
    SEND_OK_OR_ERROR_AND_RETURN(controllerErrorReset());
  }

  /* GetControllerErrorMessage() */
  if (!strcmp(myarg_1, "GetControllerErrorMessage()")) {
    cmd_buf_printf(buffer, "%s", getErrorString(getControllerError()));
    return 0;
  }

  /*ReadEcEntry(int nSlave, int nEntry)*/
  nvals = sscanf(myarg_1, "ReadEcEntry(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT64(readEcEntry(iValue, iValue2,
                                                      &i64Value));
  }

  /*ReadEcEntryIDString(int nSlavePosition,char *cEntryID*/
  nvals = sscanf(myarg_1, "ReadEcEntryIDString(%d,%[^)])", &iValue, cIdBuffer);

  if (nvals == 2) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT64(readEcEntryIDString(iValue,
                                                              cIdBuffer,
                                                              &i64Value));
  }

  /*ReadEcEntryIndexIDString(int nSlavePosition,char *cEntryID)*/
  nvals = sscanf(myarg_1,
                 "ReadEcEntryIndexIDString(%d,%[^)])",
                 &iValue2,
                 cIdBuffer);

  if (nvals == 2) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(readEcEntryIndexIDString(iValue2,
                                                                 cIdBuffer,
                                                                 &iValue));
  }

  /*ReadEcSlaveIndex(int nSlavePosition,int *nValue)*/
  nvals = sscanf(myarg_1, "ReadEcSlaveIndex(%d)", &iValue2);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(readEcSlaveIndex(iValue2, &iValue));
  }

  /*WriteEcEntry(int nSlave, int nEntry,int iValue)*/
  nvals =
    sscanf(myarg_1, "WriteEcEntry(%d,%d,%d)", &iValue, &iValue2, &iValue3);

  if (nvals == 3) {
    SEND_OK_OR_ERROR_AND_RETURN(writeEcEntry(iValue, iValue2, iValue3));
  }

  /*WritePLCVar(int plcIndex,const char* varName,double value)*/
  nvals = sscanf(myarg_1,
                 "WritePLCVar(%d,%[^,],%lf)",
                 &iValue,
                 cIdBuffer,
                 &dValue1);

  if (nvals == 3) {
    SEND_OK_OR_ERROR_AND_RETURN(writePLCVar(iValue, cIdBuffer, dValue1));
  }

  /*ReadPLCVar(int plcIndex,const char* varName,double *value)*/
  nvals = sscanf(myarg_1, "ReadPLCVar(%d,%[^)])", &iValue, cIdBuffer);

  if (nvals == 2) {
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(readPLCVar(iValue, cIdBuffer,
                                                      &fValue));
  }


  /*EcReadSdo(uint16_t slave_position,uint16_t sdo_index,uint8_t sdo_subindex,
  int byteSize)*/
  nvals = sscanf(myarg_1,
                 "EcReadSdo(%d,0x%x,0x%x,%d)",
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5);

  if (nvals == 4) {
    SEND_RESULT_OR_ERROR_AND_RETURN_UINT(ecReadSdo(iValue2, iValue3, iValue4,
                                                   iValue5, &u32Value));
  }

  /*EcReadSoE(uint16_t  slavePosition,
              uint8_t   driveNo,
              uint16_t  idn,
              size_t    byteSize)*/
  nvals = sscanf(myarg_1,
                 "EcReadSoE(%d,%d,%d,%d)",
                 &iValue2,
                 &iValue3,
                 &iValue4,
                 &iValue5);

  if (nvals == 4) {
    SEND_RESULT_OR_ERROR_AND_RETURN_UINT64(ecReadSoE(iValue2, iValue3, iValue4,
                                                     iValue5,
                                                     (uint8_t *)(&u64Value)));
  }

  /*EcGetSlaveVendorId(int nSlavePosition,int *nValue)*/
  nvals = sscanf(myarg_1, "EcGetSlaveVendorId(%d)", &iValue2);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_UINT(ecGetSlaveVendorId(0, iValue2,
                                                            &u32Value));
  }

  /*EcGetSlaveProductCode(int nSlavePosition,int *nValue)*/
  nvals = sscanf(myarg_1, "EcGetSlaveProductCode(%d)", &iValue2);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_UINT(ecGetSlaveProductCode(0, iValue2,
                                                               &u32Value));
  }

  /*EcGetSlaveRevisionNum(int nSlavePosition,int *nValue)*/
  nvals = sscanf(myarg_1, "EcGetSlaveRevisionNum(%d)", &iValue2);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_UINT(ecGetSlaveRevisionNum(0, iValue2,
                                                               &u32Value));
  }

  /*EcGetSlaveSerialNum(int nSlavePosition,int *nValue)*/
  nvals = sscanf(myarg_1, "EcGetSlaveSerialNum(%d)", &iValue2);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_UINT(ecGetSlaveSerialNum(0, iValue2,
                                                             &u32Value));
  }

  /*EcGetMemMapId(char *strName)*/
  nvals = sscanf(myarg_1, "EcGetMemMapId(%[^)])", cIdBuffer);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(ecGetMemMapId(cIdBuffer, &iValue));
  }

  /*GetAxisGroupIndexByName(char *groupName)*/
  nvals = sscanf(myarg_1, "GetAxisGroupIndexByName(%[^)])", cIdBuffer);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisGroupIndexByName(cIdBuffer, &iValue));
  }

  /*GetAxisBlockCom(int nAxis)*/
  nvals = sscanf(myarg_1, "GetAxisBlockCom(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisBlockCom(motor_axis_no,
                                                        &iValue));
  }

  /*GetAxisCycleCounter(int nAxis)*/
  nvals = sscanf(myarg_1, "GetAxisCycleCounter(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisCycleCounter(motor_axis_no,
                                                            &iValue));
  }

  /*GetAxisType(int nAxis)*/
  nvals = sscanf(myarg_1, "GetAxisType(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisType(motor_axis_no, &iValue));
  }

  /*GetAxisModRange(int nAxis)*/
  nvals = sscanf(myarg_1, "GetAxisModRange(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisModRange(motor_axis_no,
                                                           &fValue));
  }

  /*GetAxisModType(int nAxis)*/
  nvals = sscanf(myarg_1, "GetAxisModType(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisModType(motor_axis_no,
                                                       &iValue));
  }

  /*GetAxisMonLimitFwdPolarity(int nAxis)*/
  nvals = sscanf(myarg_1, "GetAxisMonLimitFwdPolarity(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisMonLimitFwdPolarity(
                                          motor_axis_no, &iValue));
  }

  /*GetAxisMonLimitBwdPolarity(int nAxis)*/
  nvals = sscanf(myarg_1, "GetAxisMonLimitBwdPolarity(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisMonLimitBwdPolarity(
                                          motor_axis_no, &iValue));
  }

  /*GetAxisMonHomeSwitchPolarity(int nAxis)*/
  nvals = sscanf(myarg_1, "GetAxisMonHomeSwitchPolarity(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisMonHomeSwitchPolarity(
                                          motor_axis_no, &iValue));
  }

  /*GetAxisMonExtHWInterlockPolarity(int nAxis)*/
  nvals = sscanf(myarg_1,
                 "GetAxisMonExtHWInterlockPolarity(%d)",
                 &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisMonExtHWInterlockPolarity(
                                          motor_axis_no, &iValue));
  }

  /*GetAxisMonLatchLimit(int nAxis)*/
  nvals = sscanf(myarg_1, "GetAxisMonLatchLimit(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisMonLatchLimit(motor_axis_no,
                                                             &iValue));
  }

  /*int GetAxisEnableAlarmAtHardLimits(int axis_no);*/
  nvals =
    sscanf(myarg_1, "GetAxisEnableAlarmAtHardLimits(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEnableAlarmAtHardLimits(
                                          motor_axis_no, &iValue));
  }

  /*int GetAxisEncSourceType(int axis_no);*/
  nvals = sscanf(myarg_1, "GetAxisEncSourceType(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEncSource(motor_axis_no,
                                                         &iValue));
  }

  /*int GetAxisEncPrimaryIndex(int axis_no);*/
  nvals = sscanf(myarg_1, "GetAxisEncPrimaryIndex(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEncPrimaryIndex(motor_axis_no,
                                                               &iValue));
  }

  /*int GetAxisEncConfigIndex(int axis_no);*/
  nvals = sscanf(myarg_1, "GetAxisEncConfigIndex(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEncConfigIndex(motor_axis_no,
                                                              &iValue));
  }

  ///*int GetAxisEncHomeIndex(int axis_no);*/
  // nvals = sscanf(myarg_1, "GetAxisEncHomeIndex(%d)", &motor_axis_no);
  //
  // if (nvals == 1) {
  //  SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEncHomeIndex(motor_axis_no,
  //                                                          &iValue));
  // }

  /*int GetAxisTrajSourceType(int axis_no);*/
  nvals = sscanf(myarg_1, "GetAxisTrajSourceType(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisTrajSource(motor_axis_no,
                                                          &iValue));
  }

  /*int GetAxisAllowCommandsFromPLC(int axis_no);*/
  nvals = sscanf(myarg_1,
                 "GetAxisAllowCommandsFromPLC(%d)",
                 &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisAllowCommandsFromPLC(
                                          motor_axis_no, &iValue));
  }

  /*int GetAxisPLCEnable(int axis_no);*/
  nvals =
    sscanf(myarg_1, "GetAxisPLCEnable(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisPLCEnable(
                                          motor_axis_no, &iValue));
  }

  /*GetPLCEnable(int plcIndex)*/
  nvals = sscanf(myarg_1, "GetPLCEnable(%d)", &iValue2);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getPLCEnable(iValue2, &iValue));
  }

  /*int GetAxisPLCExpr(int axis_no);   */
  nvals = sscanf(myarg_1, "GetAxisPLCExpr(%d)", &iValue);

  if (nvals == 1) {
    char *retBuf = cPlcExprBuffer;
    int   error  = 0;
    char *expr   = (char *)getAxisPLCExpr(iValue, &error);

    if (error) {
      cmd_buf_printf(buffer, "Error: %d", error);
      return 0;
    }

    int length = strlen(expr);

    if (length >= ECMC_CMD_MAX_SINGLE_CMD_LENGTH) {
      cmd_buf_printf(buffer, "Error: %d", ERROR_MAIN_PARSER_CMD_TO_LONG);
      return 0;
    }

    memcpy(retBuf, expr, length + 1);

    // Change all | to ; (since ; is used as command
    // delimiter in communication)
    size_t strLen = strlen(retBuf);
    size_t i      = 0;

    for (i = 0; i < strLen; i++) {
      if (retBuf[i] == ';') {
        retBuf[i] = TRANSFORM_EXPR_LINE_END_CHAR;
      }
    }

    cmd_buf_printf(buffer, "%s", retBuf);
    return 0;
  }

  /*int GetPLCExpr(int axis_no);   */
  nvals = sscanf(myarg_1, "GetPLCExpr(%d)", &iValue);

  if (nvals == 1) {
    char *retBuf = cPlcExprBuffer;
    int   error  = 0;
    char *expr   = (char *)getPLCExpr(iValue, &error);

    if (error) {
      cmd_buf_printf(buffer, "Error: %d", error);
      return 0;
    }

    if (strlen(expr) >= ECMC_CMD_MAX_SINGLE_CMD_LENGTH) {
      cmd_buf_printf(buffer, "Error: %d", ERROR_MAIN_PARSER_CMD_TO_LONG);
      return 0;
    }

    strcpy(retBuf, expr);

    // Change all | to ; (since ; is used as command
    // delimiter in communication)
    size_t strLen = strlen(retBuf);
    size_t i      = 0;

    for (i = 0; i < strLen; i++) {
      if (retBuf[i] == ';') {
        retBuf[i] = TRANSFORM_EXPR_LINE_END_CHAR;
      }
    }

    cmd_buf_printf(buffer, "%s", retBuf);
    return 0;
  }

  /* stAxisStatus? */  /* GetAxisDebugInfoData(in axisIndex) */
  nvals = sscanf(myarg_1, "GetAxisDebugInfoData(%d)", &iValue);

  if (nvals == 1) {
    int error = getAxisDebugInfoData(iValue,
                                     &cIdBuffer[0],
                                     sizeof(cIdBuffer));

    if (error) {
      cmd_buf_printf(buffer, "Error: %d", error);
      return 0;
    }
    cmd_buf_printf(buffer, "%s", cIdBuffer);
    return 0;
  }

  /*GetAxisEncPosRaw(int axisIndex)*/
  nvals = sscanf(myarg_1, "GetAxisEncPosRaw(%d)", &iValue);

  if (nvals == 1) {
    int64_t iTemp;
    int     error = getAxisEncPosRaw(iValue, &iTemp);

    if (error) {
      cmd_buf_printf(buffer, "Error: %d", error);
      return 0;
    }
    cmd_buf_printf(buffer, "%" PRId64, iTemp);
    return 0;
  }

  /* GetAxisDebugInfoData(in axisIndex) */
  nvals = sscanf(myarg_1, "GetAxisDebugInfoData(%d)", &iValue);

  if (nvals == 1) {
    int error = getAxisDebugInfoData(iValue,
                                     &cIdBuffer[0],
                                     sizeof(cIdBuffer));

    if (error) {
      cmd_buf_printf(buffer, "Error: %d", error);
      return 0;
    }
    cmd_buf_printf(buffer, "%s", cIdBuffer);
    return 0;
  }

  /*int MoveAbsolutePosition(int axisIndex,double positionSet,
  double velocitySet, double accelerationSet, double decelerationSet);*/
  nvals = sscanf(myarg_1,
                 "MoveAbsolutePosition(%d,%lf,%lf,%lf,%lf)",
                 &iValue,
                 &dValue1,
                 &dValue2,
                 &dValue3,
                 &dValue4);

  if (nvals == 5) {
    SEND_OK_OR_ERROR_AND_RETURN(moveAbsolutePosition(iValue, dValue1, dValue2,
                                                     dValue3, dValue4));
  }

  /*int MoveRelativePosition(int axisIndex,double positionSet,
  double velocitySet, double accelerationSet, double decelerationSet);*/
  nvals = sscanf(myarg_1,
                 "MoveRelativePosition(%d,%lf,%lf,%lf,%lf)",
                 &iValue,
                 &dValue1,
                 &dValue2,
                 &dValue3,
                 &dValue4);

  if (nvals == 5) {
    SEND_OK_OR_ERROR_AND_RETURN(moveRelativePosition(iValue, dValue1, dValue2,
                                                     dValue3, dValue4));
  }

  /*int MoveVelocity(int axisIndex,double velocitySet,
  double accelerationSet, double decelerationSet);*/
  nvals = sscanf(myarg_1,
                 "MoveVelocity(%d,%lf,%lf,%lf)",
                 &iValue,
                 &dValue1,
                 &dValue2,
                 &dValue3);

  if (nvals == 4) {
    SEND_OK_OR_ERROR_AND_RETURN(moveVelocity(iValue, dValue1, dValue2,
                                             dValue3));
  }

  /*int StopMotion(int axisIndex, int killAmplifier);*/
  nvals = sscanf(myarg_1, "StopMotion(%d,%d)", &iValue, &iValue2);

  if (nvals == 2) {
    SEND_OK_OR_ERROR_AND_RETURN(stopMotion(iValue, iValue2));
  }

  /*int StopMotion(int axisIndex);*/
  nvals = sscanf(myarg_1, "StopMotion(%d)", &iValue);

  if (nvals == 1) {
    SEND_OK_OR_ERROR_AND_RETURN(stopMotion(iValue, 0));
  }

  /*int GetStorageDataIndex(int axis_no)*/
  nvals = sscanf(myarg_1, "GetStorageDataIndex(%d)", &motor_axis_no);

  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getStorageDataIndex(motor_axis_no,
                                                            &iValue));
  }

  /*int ReadDataStorage(int storageIndex);*/
  nvals = sscanf(myarg_1, "ReadDataStorage(%d)", &iValue);

  if (nvals == 1) {
    double *bufferdata = NULL;
    int     size       = 0;
    int     error      = readStorageBuffer(iValue, &bufferdata, &size);

    if (error) {
      cmd_buf_printf(buffer, "Error: %d", error);
      return 0;
    }

    if (!bufferdata) {
      cmd_buf_printf(buffer, "Error: %d",
                     ECMC_PARSER_READ_STORAGE_BUFFER_DATA_NULL);
      return 0;
    }

    // Write ascii array delimited with ','
    cmd_buf_printf(buffer, "ReadDataStorage(%d)=", iValue);
    int i = 0;

    for (i = 0; i < size; i++) {
      if (i < size - 1) {
        cmd_buf_printf(buffer, "%lf,", bufferdata[i]);  // Data and comma
      } else {
        cmd_buf_printf(buffer, "%lf", bufferdata[i]);  // No comma for last entry
      }
    }

    return 0;
  }

  /*int WriteStorageBuffer(int axisIndex)=0,0,0,0*/
  nvals = sscanf(myarg_1, "WriteDataStorage(%d)=", &iValue);

  if (nvals == 1) {
    setDataStorageCurrentDataIndex(iValue, 0);  // Start to fill from first index in buffer
    SEND_OK_OR_ERROR_AND_RETURN(appendAsciiDataToStorageBuffer(iValue,
                                                               myarg_1));
  }

  /*int AppendDataStorage(int axisIndex)=0,0,0,0*/
  nvals = sscanf(myarg_1, "AppendDataStorage(%d)=", &iValue);

  if (nvals == 1) {
    SEND_OK_OR_ERROR_AND_RETURN(appendAsciiDataToStorageBuffer(iValue,
                                                               myarg_1));
  }

  if (!strcmp(myarg_1, "EcPrintAllHardware()")) {
    SEND_OK_OR_ERROR_AND_RETURN(ecPrintAllHardware());
  }

  nvals = sscanf(myarg_1, "EcPrintSlaveConfig(%d)=", &iValue);

  if (nvals == 1) {
    SEND_OK_OR_ERROR_AND_RETURN(ecPrintSlaveConfig(iValue));
  }

  /* if we come here, we do not understand the command */
  SEND_OK_OR_ERROR_AND_RETURN(ERROR_MAIN_PARSER_UNKOWN_CMD);
}

/*int ecmcCmdParser(int             argc,
            const char           *argv[],
            const char           *sepv[],
            ecmcOutputBufferType *buffer) {*/
int ecmcCmdParser(const char           *cmdline,
                  int                   inLen,
                  ecmcOutputBufferType *buffer) {
  int   cmdCounter = 0;
  int   multiCmd   = 0;
  int   done       = 0;
  char *nextStart  = (char *)cmdline;
  char *nextEnd    = strchr(nextStart, ';'); // check if multline

  if (nextEnd) {
    multiCmd = 1;
  }
  char *nextCmd = (char *)cmdline;

  while (!done) {
    if (nextEnd) {
      memcpy(cOneCommand, nextStart, nextEnd - nextStart);
      cOneCommand[nextEnd - nextStart] = '\0';
      nextCmd                          = cOneCommand; // Use local buffer
    } else {   // Only one cmd
      nextCmd  = nextStart;
      multiCmd = 0;
      done     = 1;
    }
    int errorCode = motorHandleOneArg(nextCmd, buffer);
    cmdCounter++;

    if (errorCode) {
      RETURN_ERROR_OR_DIE(buffer,
                          errorCode,
                          "%s/%s:%d motorHandleOneArg returned errorcode: %d\n",
                          __FILE__,
                          __FUNCTION__,
                          __LINE__,
                          errorCode);
    }

    if (multiCmd) {
      cmd_buf_printf(buffer, "%s", ";");

      if (strlen(nextEnd) > 1) {
        nextStart = nextEnd + 1;
        nextEnd   = strchr(nextStart, ';'); // check if multline
      } else {
        done = 1;
      }
    } else {
      done = 1;
    }
  }

  return 0;
}

/******************************************************************************/
