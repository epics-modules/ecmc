#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include "cmd.h"
#include "hw_motor.h"
#include "cmd_EAT.h"
#include <inttypes.h>
#include <string.h>

typedef struct
{
  int    command_no;
  unsigned nCmdData;
  int    bEnabled;
  int    bExecute;
  double position;
  double velocity;
  double acceleration;
} cmd_Motor_cmd_type;

//TODO: update marcos.. should not need different for different types
#define SEND_OK_OR_ERROR_AND_RETURN(function) {int iRet=function;if(iRet){sprintf(errorBuffer, "Error: %d", iRet);cmd_buf_printf(errorBuffer);return;}cmd_buf_printf("OK");return;}
#define SEND_RESULT_OR_ERROR_AND_RETURN_INT(function) {int iRet=function;if(iRet){sprintf(errorBuffer, "Error: %d", iRet);cmd_buf_printf(errorBuffer);return;}cmd_buf_printf("%d", iValue);return;}
#define SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(function) {int iRet=function;if(iRet){sprintf(errorBuffer, "Error: %d", iRet);cmd_buf_printf(errorBuffer);return;}cmd_buf_printf("%lf", fValue);return;}
#define SEND_RESULT_OR_ERROR_AND_RETURN_UINT64(function) {int iRet=function;if(iRet){sprintf(errorBuffer, "Error: %d", iRet);cmd_buf_printf(errorBuffer);return;}cmd_buf_printf("%"PRIu64"", i64Value);return;}
#define IF_ERROR_SEND_ERROR_AND_RETURN(function) {int iRet=function;if(iRet){sprintf(errorBuffer, "Error: %d", iRet);cmd_buf_printf(errorBuffer);return;}}

void init_axis(int axis_no)
{
  ;
}

static const char * const ADSPORT_equals_str = "ADSPORT=";
static const char * const Main_dot_str = "Main.";
static const char * const Cfg_dot_str =  "Cfg.";


static int motorHandleADS_ADR_getInt(unsigned adsport,
                                     unsigned group_no,
                                     unsigned offset_in_group,
                                     int *iValue)
{
  //This command needs to be removed
  if (group_no == 0x3040010 && offset_in_group == 0x80000049) {
    int64_t iTemp=0;
    int iRet=getAxisEncPosRaw(1,&iTemp); //Why hardcoded 1??
    *iValue=(int)iTemp;
    return iRet;
  }

  if (group_no >= 0x5000 && group_no < 0x6000) {
    int motor_axis_no = (int)group_no - 0x5000;

    //ADSPORT=501/.ADR.16#5001,16#B,2,2?; #low Softlimit enabled
    if (offset_in_group == 0xB) {
      getAxisEnableSoftLimitBwd(motor_axis_no, iValue);
      return 0;
    }
    //ADSPORT=501/.ADR.16#5001,16#C,2,2?; #high Softlimit enabled
    if (offset_in_group == 0xC) {
      getAxisEnableSoftLimitFwd(motor_axis_no, iValue);
      return 0;
    }
  }


  RETURN_ERROR_OR_DIE(__LINE__, "%s/%s:%d group_no=0x%x offset_in_group=0x%x",
                      __FILE__, __FUNCTION__, __LINE__,
                      group_no,
                      offset_in_group);
}

static int motorHandleADS_ADR_putInt(unsigned adsport,
                                     unsigned group_no,
                                     unsigned offset_in_group,
                                     int iValue)
{
  if (group_no >= 0x5000 && group_no < 0x6000) {
    int motor_axis_no = (int)group_no - 0x5000;

    //ADSPORT=501/.ADR.16#5001,16#B,2,2=1; #enable low Softlimit
    if (offset_in_group == 0xB) {
      setAxisEnableSoftLimitBwd(motor_axis_no, iValue);
      return 0;
    }
    //ADSPORT=501/.ADR.16#5001,16#C,2,2=1; #enable high Softlimit
    if (offset_in_group == 0xC) {
      setAxisEnableSoftLimitFwd(motor_axis_no, iValue);
      return 0;
    }
  }

  if (group_no >= 0x4000 && group_no < 0x5000) {
    //int motor_axis_no = (int)indexGroup - 0x4000;
    if (offset_in_group == 0x15) {
      return 0; /* Monitor */ /* TODO */
    }
  }

  //2015-02-10: Removed this command because it will not work with beckhoff or Open Source
  //if (group_no == 0x4001 && offset_in_group == 0x15) {
  //  return 0; /* Monitor What is this*/
  //}

  RETURN_ERROR_OR_DIE(__LINE__, "%s/%s:%d group_no=0x%x offset_in_group=0x%x",
               __FILE__, __FUNCTION__, __LINE__,
               group_no,
               offset_in_group);
}


static int motorHandleADS_ADR_getFloat(unsigned adsport,
                                       unsigned group_no,
                                       unsigned offset_in_group,
                                       double *fValue)
{
  if(adsport==501){

    //group 4000
    if (group_no >= 0x4000 && group_no < 0x5000) {
      int motor_axis_no = (int)group_no - 0x4000;

      //ADSPORT=501/.ADR.16#4001,16#6,8,5?; #Homing velocity towards cam
      if (offset_in_group == 0x6) {
        return getAxisHomeVelTwordsCam(motor_axis_no, fValue);
      }
      //ADSPORT=501/.ADR.16#4001,16#7,8,5?; #Homing velocity off cam
      if (offset_in_group == 0x7) {
        return getAxisHomeVelOffCam(motor_axis_no,fValue);
      }
    }

    //group 5000
    if (group_no >= 0x5000 && group_no < 0x6000) {
      int motor_axis_no = (int)group_no - 0x5000;

      //ADSPORT=501/.ADR.16#5001,16#D,8,5?; #low Softlimit
      if (offset_in_group == 0xD) {
        return getAxisSoftLimitPosBwd(motor_axis_no, fValue);
      }
      //ADSPORT=501/.ADR.16#5001,16#E,8,5?; #high Softlimit
      if (offset_in_group == 0xE) {
        return getAxisSoftLimitPosFwd(motor_axis_no, fValue);
      }

      //ADSPORT=501/.ADR.16#5001,16#23,8,5?; #Encoder scale num for axis
      if (offset_in_group == 0x23) {
        return getAxisEncScaleNum(motor_axis_no, fValue);
      }
      //ADSPORT=501/.ADR.16#5001,16#24,8,5?; #Encoder scale denom for axis
      if (offset_in_group == 0x24) {
        return getAxisEncScaleDenom(motor_axis_no, fValue);
      }
    }
  }

  return __LINE__;
}

/*
  ADSPORT=501/.ADR.16#5001,16#D,8,5=-13.5; #low Softlimit
  ADSPORT=501/.ADR.16#5001,16#E,8,5=140.0; #high Softlimit
*/
static int motorHandleADS_ADR_putFloat(unsigned adsport,
                                       unsigned group_no,
                                       unsigned offset_in_group,
                                       double fValue)
{

  if(adsport==501){

    //group 4000
    if (group_no >= 0x4000 && group_no < 0x5000) {
      int motor_axis_no = (int)group_no - 0x4000;

      //ADSPORT=501/.ADR.16#4001,16#6,8,5=200; #Homing velocity towards cam
      if (offset_in_group == 0x6) {
        return setAxisHomeVelTwordsCam(motor_axis_no, fValue);
      }
      //ADSPORT=501/.ADR.16#4001,16#7,8,5=100; #Homing velocity off cam
      if (offset_in_group == 0x7) {
        return setAxisHomeVelOffCam(motor_axis_no,fValue);
      }
    }

    //group 5000
    if (group_no >= 0x5000 && group_no < 0x6000) {
      int motor_axis_no = (int)group_no - 0x5000;

      //ADSPORT=501/.ADR.16#5001,16#D,8,5=-13.5; #low Softlimit
      if (offset_in_group == 0xD) {
        setAxisSoftLimitPosBwd(motor_axis_no, fValue);
        return 0;
      }
      //ADSPORT=501/.ADR.16#5001,16#E,8,5=140.0; #high Softlimit
      if (offset_in_group == 0xE) {
        setAxisSoftLimitPosFwd(motor_axis_no, fValue);
        return 0;
      }

      //ADSPORT=501/.ADR.16#5001,16#23,8,5=-13.5; #Encoder scale num for axis
      if (offset_in_group == 0x23) {
        return setAxisEncScaleNum(motor_axis_no, fValue);
      }
      //ADSPORT=501/.ADR.16#5001,16#24,8,5=140.0; #Encoder scale denom for axis
      if (offset_in_group == 0x24) {
        return setAxisEncScaleDenom(motor_axis_no, fValue);
      }
    }
  }
  return __LINE__;
}

/*
  ADSPORT=501/.ADR.16#5001,16#B,2,2=1;
*/
static int motorHandleADS_ADR(const char *arg)
{
  const char *myarg_1 = NULL;
  unsigned adsport = 0;
  unsigned group_no = 0;
  unsigned offset_in_group = 0;
  unsigned len_in_PLC = 0;
  unsigned type_in_PLC = 0;
  int nvals;
  nvals = sscanf(arg, "%u/.ADR.16#%x,16#%x,%u,%u=",
                 &adsport,
                 &group_no,
                 &offset_in_group,
                 &len_in_PLC,
                 &type_in_PLC);
  LOGINFO6("%s/%s:%d "
           "nvals=%d adsport=%u group_no=0x%x offset_in_group=0x%x len_in_PLC=%u type_in_PLC=%u\n",
           __FILE__, __FUNCTION__, __LINE__,
           nvals,
           adsport,
           group_no,
           offset_in_group,
           len_in_PLC,
           type_in_PLC);

  if (nvals != 5) return __LINE__;
  if (adsport != 501) return __LINE__;

  myarg_1 = strchr(arg, '=');
  if (myarg_1) {
    myarg_1++; /* Jump over '=' */
    switch (type_in_PLC) {
      case 5: {
        double fValue;
        if (len_in_PLC != 8) return __LINE__;
        nvals = sscanf(myarg_1, "%lf", &fValue);
        if (nvals != 1) return __LINE__;
        return motorHandleADS_ADR_putFloat(adsport,
                                        group_no,
                                        offset_in_group,
                                        fValue);
      }
        break;
      case 2: {
        int iValue;
        if (len_in_PLC != 2) return __LINE__;
        nvals = sscanf(myarg_1, "%d", &iValue);
        if (nvals != 1) return __LINE__;
        return motorHandleADS_ADR_putInt(adsport,
                                      group_no,
                                      offset_in_group,
                                      iValue);
      }
        break;
      default:
        return __LINE__;
    }
  }
  myarg_1 = strchr(arg, '?');
  if (myarg_1) {
    int res;
    myarg_1++; /* Jump over '?' */
    switch (type_in_PLC) {
      case 5: {
        double fValue;
        if (len_in_PLC != 8) return __LINE__;
        res = motorHandleADS_ADR_getFloat(adsport,
                                           group_no,
                                           offset_in_group,
                                           &fValue);
        if (res) return res;
        cmd_buf_printf("%g", fValue);
        return -1;
      }
        break;
      case 2: {
        int res;
        int iValue = -1;
        if (len_in_PLC != 2) return __LINE__;
        res = motorHandleADS_ADR_getInt(adsport,
                                        group_no,
                                        offset_in_group,
                                        &iValue);
        if (res) return res;
        cmd_buf_printf("%d", iValue);
        return -1;
      }
        break;
      default:
        return __LINE__;
    }
  }
  return __LINE__;
}
static int handleCfgCommand(const char *myarg_1){
  int iValue = 0;
  int iValue2 = 0;
  int iValue3 = 0;
  int iValue4 = 0;
  int iValue5 = 0;
  int iValue6 = 0;
  int iValue7 = 0;
  int iValue8 = 0;
  int iValue9 = 0;

  int nvals = 0;
  double dValue=0;
  double dValue2=0;
  char cIdBuffer[4096];

  /*Cfg.SetAppMode(x)*/
  nvals = sscanf(myarg_1, "SetAppMode(%d)", &iValue);
  if (nvals == 1) {
    return setAppMode(iValue);
  }

  /*Cfg.ValidateConfig()*/
  nvals=strcmp(myarg_1,"ValidateConfig()");
  if (nvals == 0) {
    return validateConfig();
  }

  /*Cfg.CreateAxis(nAxis, nType)*/
  nvals = sscanf(myarg_1, "CreateAxis(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return createAxis(iValue,iValue2);
  }

  /*Cfg.CreateDefaultAxis(nAxis)*/
  int nRet=0;
  nvals = sscanf(myarg_1, "CreateDefaultAxis(%d)", &iValue);
  if (nvals == 1) {
    nRet=createAxis(iValue,1);  //Default REAL axis
    if(nRet)
      return nRet;
    return 0;
  }

  /*Cfg.LinkEcEntryToAxisEncoder(int nSlave, int nEntry,int nAxis,nEncEntryIndex)*/
  nvals = sscanf(myarg_1, "LinkEcEntryToAxisEncoder(%d,%[^,],%d,%d)", &iValue,cIdBuffer,&iValue3,&iValue4);
  if (nvals == 4) {
    return linkEcEntryToAxisEnc(iValue,cIdBuffer,iValue3,iValue4);
  }

  /*Cfg.LinkEcEntryToAxisDrive(int nSlave, int nEntry,int nAxis, int nDrvEntryIndex)*/
  nvals = sscanf(myarg_1, "LinkEcEntryToAxisDrive(%d,%[^,],%d,%d)", &iValue,cIdBuffer,&iValue3,&iValue4);
  if (nvals == 4) {
    return linkEcEntryToAxisDrv(iValue,cIdBuffer,iValue3,iValue4);
  }

  /*Cfg.LinkEcEntryToAxisMonitor(int nSlave, int nEntry,int nAxis,int nMonEntryIndex)*/
  nvals = sscanf(myarg_1, "LinkEcEntryToAxisMonitor(%d,%[^,],%d,%d)", &iValue,cIdBuffer,&iValue3,&iValue4);
  if (nvals == 4) {
    return linkEcEntryToAxisMon(iValue,cIdBuffer,iValue3,iValue4);
  }

  /*Cfg.WriteEcEntryIDString(int nSlavePosition,char *cEntryID,uint64_t nValue)*/
  nvals = sscanf(myarg_1, "WriteEcEntryIDString(%d,%[^,],%d)", &iValue,cIdBuffer,&iValue3);
  if (nvals == 3) {
    return writeEcEntryIDString(iValue,cIdBuffer,iValue3);
  }

  /*Cfg.EcSetMaster(int master_index)*/
  nvals = sscanf(myarg_1, "EcSetMaster(%d)", &iValue);
  if (nvals == 1) {
    return ecSetMaster(iValue);
  }

  /*Cfg.EcAddSlave(uint16_t alias, uint16_t position, uint32_t vendor_id,uint32_t product_code)*/
  nvals = sscanf(myarg_1, "EcAddSlave(%d,%d,%x,%x)", &iValue,&iValue2,&iValue3,&iValue4);
  if (nvals == 4) {
    return ecAddSlave(iValue,iValue2,iValue3, iValue4);
  }

  /*Cfg.EcAddPdo(int nSlave,int nSyncManager,uint16_t nPdoIndex) wrong*/
  nvals = sscanf(myarg_1, "EcAddPdo(%d,%d,%x)", &iValue,&iValue2,&iValue3);
  if (nvals == 3) {
    return ecAddPdo(iValue,iValue2,iValue3);
  }

  /*Cfg.EcAddEntry(int nSlave,uint16_t nEntryIndex,uint8_t  nEntrySubIndex, uint8_t nBits) wrong comment*/
  /*nvals = sscanf(myarg_1, "EcAddEntry(%d,%d,%d,%x,%x,%d)", &iValue,&iValue2,&iValue3,&iValue4,&iValue5,&iValue6);
  if (nvals == 6) {
    return ecAddEntry(iValue,iValue2,iValue3,iValue4,iValue5,iValue6);
  }*/

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
  nvals = sscanf(myarg_1, "EcAddEntryComplete(%d,%x,%x,%d,%d,%x,%x,%x,%d,%[^)])", &iValue,&iValue2,&iValue3,&iValue4,&iValue5,&iValue6,&iValue7,&iValue8,&iValue9,cIdBuffer);
  if (nvals == 10) {
    return ecAddEntryComplete(iValue,iValue2,iValue3,iValue4,iValue5,iValue6,iValue7,iValue8,iValue9,cIdBuffer);
  }

  /*Cfg.EcSlaveConfigDC(
      int slave_bus_position,
      uint16_t assign_activate,
      uint32_t sync0_cycle,
      int32_t sync0_shift,
      uint32_t sync1_cycle,
      int32_t sync1_shift )*/
  nvals = sscanf(myarg_1, "EcSlaveConfigDC(%d,%x,%d,%d,%d,%d)", &iValue,&iValue2,&iValue3,&iValue4,&iValue5,&iValue6);
  if (nvals == 6) {
    return ecSlaveConfigDC(iValue,iValue2,iValue3,iValue4,iValue5,iValue6);
  }

  /*Cfg.EcAddSyncManager(int nSlave,ec_direction_t nDirection,uint8_t nSyncMangerIndex)*/
  nvals = sscanf(myarg_1, "EcAddSyncManager(%d,%d,%d)", &iValue,&iValue2,&iValue3);
  if (nvals == 3) {
    return ecAddSyncManager(iValue,iValue2,iValue3);
  }

  /*Cfg.EcAddSdo(uint16_t slave_position,uint16_t sdo_index,uint8_t sdo_subindex,uint32_t value,int byteSize)*/
  nvals = sscanf(myarg_1, "EcAddSdo(%d,%x,%x,%d,%d)", &iValue,&iValue2,&iValue3,&iValue4,&iValue5);
  if (nvals == 5) {
    return ecAddSdo(iValue,iValue2,iValue3,iValue4,iValue5);
  }

  /*Cfg.EcWriteSdo(uint16_t slave_position,uint16_t sdo_index,uint8_t sdo_subindex,uint32_t value,int byteSize)*/
  nvals = sscanf(myarg_1, "EcWriteSdo(%d,%x,%x,%d,%d)", &iValue,&iValue2,&iValue3,&iValue4,&iValue5);
  if (nvals == 5) {
    return ecWriteSdo(iValue,iValue2,iValue3,iValue4,iValue5);
  }

  /*Cfg.EcReadSdo(uint16_t slave_position,uint16_t sdo_index,uint8_t sdo_subindex,int byteSize)*/
  nvals = sscanf(myarg_1, "EcReadSdo(%d,%x,%x,%d)", &iValue,&iValue2,&iValue3,&iValue4);
  if (nvals == 4) {
    cmd_buf_printf("%d",ecReadSdo(iValue,iValue2,iValue3,iValue4));
    return -1; //Read command TODO move to read section
  }

  /*Cfg.EcApplyConfig(int nMasterIndex)*/
  nvals = sscanf(myarg_1, "EcApplyConfig(%d)",&iValue);
  if (nvals == 1) {
    return ecApplyConfig(iValue);
  }

  /*Cfg.EcSetDiagnostics(int nDiagnostics)*/
  nvals = sscanf(myarg_1, "EcSetDiagnostics(%d)",&iValue);
  if (nvals == 1) {
    return ecSetDiagnostics(iValue);
  }

  /*int Cfg.SetAxisJogVel(int traj_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisJogVel(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    return setAxisJogVel(iValue,dValue);
  }

  /*int Cfg.SetAxisEnableAlarmAtHardLimits(int axis_no, int nEnable);*/
  nvals = sscanf(myarg_1, "SetAxisEnableAlarmAtHardLimits(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisEnableAlarmAtHardLimits(iValue,iValue2);
  }

  /*int Cfg.SetAxisEmergDeceleration(int traj_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisEmergDeceleration(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    return setAxisEmergDeceleration(iValue,dValue);
  }

  /*int Cfg.SetAxisTrajSourceType(int axis_no, int nValue);*/
  nvals = sscanf(myarg_1, "SetAxisTrajSourceType(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisTrajSource(iValue,iValue2);
  }

  /*int Cfg.SetAxisEncSourceType(int axis_no, int nValue);*/
  nvals = sscanf(myarg_1, "SetAxisEncSourceType(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisEncSource(iValue,iValue2);
  }

  /*int Cfg.SetAxisEncScaleNum(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisEncScaleNum(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    return setAxisEncScaleNum(iValue,dValue);
  }

  /*int Cfg.SetAxisEncScaleDenom(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisEncScaleDenom(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    return setAxisEncScaleDenom(iValue,dValue);
  }

  /*int Cfg.SetAxisEncBits(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisEncBits(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisEncBits(iValue,iValue2);
  }

  /*int Cfg.SetAxisEncType(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisEncType(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisEncType(iValue,iValue2);
  }

  /*int Cfg.SetAxisEncOffset(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisEncOffset(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    return setAxisEncOffset(iValue,dValue);
  }

  /*int Cfg.SetAxisCntrlKp(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlKp(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    setAxisCntrlKp(iValue,dValue);
    return 0;
  }

  /*int Cfg.SetAxisCntrlKi(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlKi(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    setAxisCntrlKi(iValue,dValue);
    return 0;
  }

  /*int Cfg.SetAxisCntrlKd(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlKd(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    setAxisCntrlKd(iValue,dValue);
    return 0;
  }

  /*int Cfg.SetAxisCntrlKff(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlKff(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    setAxisCntrlKff(iValue,dValue);
    return 0;
  }

  /*int Cfg.SetAxisCntrlOutHL(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlOutHL(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    setAxisCntrlOutHL(iValue,dValue);
    return 0;
  }

  /*int Cfg.SetAxisCntrlOutLL(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlOutLL(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    setAxisCntrlOutLL(iValue,dValue);
    return 0;
  }

  /*int Cfg.SetAxisCntrlIPartHL(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlIPartHL(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    setAxisCntrlIpartHL(iValue,dValue);
    return 0;
  }

  /*int Cfg.SetAxisCntrlIPartLL(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisCntrlIPartLL(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    setAxisCntrlIpartLL(iValue,dValue);
    return 0;
  }

  /*int Cfg.SetAxisMonAtTargetTol(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisMonAtTargetTol(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    setAxisMonAtTargetTol(iValue,dValue);
    return 0;
  }

  /*int Cfg.SetAxisMonAtTargetTime(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisMonAtTargetTime(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    setAxisMonAtTargetTime(iValue,iValue2);
    return 0;
  }

  /*int Cfg.SetAxisMonEnableAtTargetMon(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisMonEnableAtTargetMon(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    setAxisMonEnableAtTargetMon(iValue,iValue2);
    return 0;
  }

  /*int Cfg.SetAxisMonPosLagTol(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisMonPosLagTol(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    setAxisMonPosLagTol(iValue,dValue);
    return 0;
  }

  /*int Cfg.SetAxisMonPosLagTime(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisMonPosLagTime(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    setAxisMonPosLagTime(iValue,iValue2);
    return 0;
  }

  /*int Cfg.SetAxisMonEnableLagMon(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisMonEnableLagMon(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    setAxisMonEnableLagMon(iValue,iValue2);
    return 0;
  }

  /*int Cfg.SetAxisMonMaxVel(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisMonMaxVel(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    return setAxisMonMaxVel(iValue,dValue);
  }

  /*int Cfg.SetAxisMonEnableMaxVel(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisMonEnableMaxVel(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisMonEnableMaxVel(iValue,iValue2);
  }

  /*int Cfg.SetAxisMonMaxVelDriveILDelay(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisMonMaxVelDriveILDelay(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisMonMaxVelDriveILDelay(iValue,iValue2);
  }

  /*int Cfg.SetAxisMonMaxVelTrajILDelay(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisMonMaxVelTrajILDelay(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisMonMaxVelTrajILDelay(iValue,iValue2);
  }

  /*int Cfg.SetAxisDrvScaleNum(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisDrvScaleNum(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    return setAxisDrvScaleNum(iValue,dValue);
  }

  /*int Cfg.SetAxisDrvScaleDenom(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisDrvScaleDenom(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    return setAxisDrvScaleDenom(iValue,dValue);
  }

  /*int Cfg.SetAxisDrvVelSet(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisDrvVelSet(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    return setAxisDrvVelSet(iValue,dValue);
  }

  /*int Cfg.SetAxisDrvVelSetRaw(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisDrvVelSetRaw(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisDrvVelSet(iValue,iValue2);
  }

  /*int Cfg.SetAxisDrvEnable(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisDrvEnable(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisDrvEnable(iValue,iValue2);
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

  /*int Cfg.SetAxisHomeVelTwordsCam(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisHomeVelTwordsCam(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    return setAxisHomeVelTwordsCam(iValue,dValue);
  }

  /*int Cfg.SetAxisHomeVelOffCam(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisHomeVelOffCam(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    return setAxisHomeVelOffCam(iValue,dValue);
  }

  /*int Cfg.SetAxisHomeDirection(int axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisHomeDirection(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisHomeDir(iValue,iValue2);
  }

  /*int Cfg.SetAxisGearRatio(int axis_no, double dGearNum, double dGearDenom);*/
  nvals = sscanf(myarg_1, "SetAxisGearRatio(%d,%lf,%lf)", &iValue,&dValue,&dValue2);
  if (nvals == 3) {
    return setAxisGearRatio(iValue,dValue,dValue2);
  }

  /*int Cfg.SetAxisOpMode(int axis_no, int nMode);*/
  nvals = sscanf(myarg_1, "SetAxisOpMode(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisOpMode(iValue,iValue2);
  }

  /*int Cfg.SetEnableFuncCallDiag(int nEnable);*/
  nvals = sscanf(myarg_1, "SetEnableFuncCallDiag(%d)",&iValue);
  if (nvals == 1) {
    return setEnableFunctionCallDiag(iValue);
  }

  /*int Cfg.SetEnableTimeDiag(int nEnable);*/
  nvals = sscanf(myarg_1, "SetEnableTimeDiag(%d)",&iValue);
  if (nvals == 1) {
    return setEnableTimeDiag(iValue);
  }

  /*int Cfg.SetAxisTrajStartPos(int axis_no, double value);*/
  nvals = sscanf(myarg_1, "SetAxisTrajStartPos(%d,%lf)", &iValue,&dValue);
  if (nvals == 2) {
    return setAxisTrajStartPos(iValue,dValue);;
  }

  char cExprBuffer[4096];
  /*int Cfg.SetAxisTrajTransExpr(int axis_no, char* cExpr);   */
  //nvals = sscanf(myarg_1, "SetAxisTrajTransExpr(%d,\"%[^\"])",&iValue,cExprBuffer);
  nvals = sscanf(myarg_1, "SetAxisTrajTransExpr(%d)=%[^\n]",&iValue,cExprBuffer);
  if (nvals == 2){
    //Change all # to ; (since ; is used as command delimiter in tcpip communication)
    size_t str_len=strlen(cExprBuffer);

    int i=0;
    for(i=0;i<str_len;i++){
      if(cExprBuffer[i]==TRANSFORM_EXPR_LINE_END_CHAR){
        cExprBuffer[i]=';';
      }
    }
    return  setAxisTrajTransExpr(iValue,cExprBuffer);
  }

  /*int Cfg.SetAxisEncTransExpr(int axis_no, char* cExpr);   */
  //nvals = sscanf(myarg_1, "SetAxisEncTransExpr(%d,\"%[^\"])",&iValue,cExprBuffer);
  nvals = sscanf(myarg_1, "SetAxisEncTransExpr(%d)=%[^\n]",&iValue,cExprBuffer);
  if (nvals == 2){
    //Change all # to ; (since ; is used as command delimiter in tcpip communication)
    size_t str_len=strlen(cExprBuffer);

    int i=0;
    for(i=0;i<str_len;i++){
      if(cExprBuffer[i]==TRANSFORM_EXPR_LINE_END_CHAR){
        cExprBuffer[i]=';';
      }
    }
    return  setAxisEncTransExpr(iValue,cExprBuffer);
  }

  /*int Cfg.SetAxisTransformCommandExpr(int axis_no,char *cExpr); */
  //nvals = sscanf(myarg_1, "SetAxisTransformCommandExpr(%d,\"%[^\"])",&iValue,cExprBuffer);
  nvals = sscanf(myarg_1, "SetAxisTransformCommandExpr(%d)=%[^\n]",&iValue,cExprBuffer);
  if (nvals == 2){
    //Change all # to ; (since ; is used as command delimiter in tcpip communication)
    size_t str_len=strlen(cExprBuffer);

    int i=0;
    for(i=0;i<str_len;i++){
      if(cExprBuffer[i]==TRANSFORM_EXPR_LINE_END_CHAR){
        cExprBuffer[i]=';';
      }
    }
    return  setAxisTransformCommandExpr(iValue,cExprBuffer);
  }

  /*int Cfg.SetAxisEnableCommandsFromOtherAxis(int master_axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisEnableCommandsFromOtherAxis(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisEnableCommandsFromOtherAxis(iValue,iValue2);
  }

  /*int Cfg.SetAxisEnableCommandsTransform(int master_axis_no, int value);*/
  nvals = sscanf(myarg_1, "SetAxisEnableCommandsTransform(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisEnableCommandsTransform(iValue,iValue2);
  }


  /*int Cfg.SetAxisSeqTimeout(int axis_no, int value);  IN seconds!!*/
  nvals = sscanf(myarg_1, "SetAxisSeqTimeout(%d,%d)", &iValue,&iValue2);
  if (nvals == 2) {
    return setAxisSeqTimeout(iValue,iValue2);
  }

  return 1;
}

static void motorHandleOneArg(const char *myarg_1)
{
  const char *myarg = myarg_1;
  char errorBuffer[1024];
  int iValue = 0;
  uint64_t i64Value=0;
  int iValue2=0;
  int iValue3=0;
  double fValue = 0;
  int motor_axis_no = 0;
  int nvals = 0;

  //Check if configuration command
  if (0 == strncmp(myarg_1, Cfg_dot_str,strlen(Cfg_dot_str))) {
    myarg_1 += strlen(Cfg_dot_str);
    SEND_OK_OR_ERROR_AND_RETURN(handleCfgCommand(myarg_1))
  }

  /* ADSPORT= */
  if (!strncmp(myarg_1, ADSPORT_equals_str, strlen(ADSPORT_equals_str))) {
    int err_code;
    myarg_1 += strlen(ADSPORT_equals_str);
    err_code = motorHandleADS_ADR(myarg_1);
    if (err_code == -1) return;
    if (err_code == 0) {
      cmd_buf_printf("OK");
      return;
    }
    RETURN_OR_DIE("%s/%s:%d myarg_1=%s err_code=%d",
                  __FILE__, __FUNCTION__, __LINE__,
                  myarg_1,
                  err_code);
  }

  /*ReadEcEntry(int nSlave, int nEntry)*/
  nvals = sscanf(myarg_1, "ReadEcEntry(%d,%d)",&iValue,&iValue2);
  if (nvals == 2) {
    SEND_RESULT_OR_ERROR_AND_RETURN_UINT64(readEcEntry(iValue,iValue2,&i64Value))
  }

  char cIdBuffer[4096];
  /*ReadEcEntryIDString(int nSlavePosition,char *cEntryID,uint64_t nValue)*/
  nvals = sscanf(myarg_1, "ReadEcEntryIDString(%d,%[^)])", &iValue,cIdBuffer);
  if (nvals == 2) {
    SEND_RESULT_OR_ERROR_AND_RETURN_UINT64(readEcEntryIDString(iValue,cIdBuffer,&i64Value))
  }

  /*ReadEcEntryIndexIDString(int nSlavePosition,char *cEntryID,int *nValue)*/
  nvals = sscanf(myarg_1, "ReadEcEntryIndexIDString(%d,%[^)])",&iValue2,cIdBuffer);
  if (nvals == 2) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(readEcEntryIndexIDString(iValue2,cIdBuffer,&iValue))
  }

  /*ReadEcSlaveIndex(int nSlavePosition,int *nValue)*/
  nvals = sscanf(myarg_1, "ReadEcSlaveIndex(%d)",&iValue2);
  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(readEcSlaveIndex(iValue2,&iValue))
  }

  /*WriteEcEntry(int nSlave, int nEntry,int iValue)*/
  nvals = sscanf(myarg_1, "WriteEcEntry(%d,%d,%d)", &iValue,&iValue2,&iValue3);
  if (nvals == 3) {
    SEND_OK_OR_ERROR_AND_RETURN(writeEcEntry(iValue,iValue2,iValue3));
  }

  /*ReadAxisGearRatio(int nAxis)*/
  nvals = sscanf(myarg_1, "ReadAxisGearRatio(%d)",&motor_axis_no);
  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisGearRatio(motor_axis_no,&fValue))
  }

  /*GetAxisOpMode(int nAxis)*/
  nvals = sscanf(myarg_1, "GetAxisOpMode(%d)",&motor_axis_no);
  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisOpMode(motor_axis_no,&iValue))
  }

  /*GetAxisType(int nAxis)*/
  nvals = sscanf(myarg_1, "GetAxisType(%d)",&motor_axis_no);
  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisType(motor_axis_no,&iValue))
  }

  /*int GetAxisEnableAlarmAtHardLimits(int axis_no);*/
  nvals = sscanf(myarg_1, "SetAxisEnableAlarmAtHardLimits(%d)", &motor_axis_no);
  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEnableAlarmAtHardLimits(motor_axis_no,&iValue))
  }

  /*int GetAxisEncSourceType(int axis_no);*/
  nvals = sscanf(myarg_1, "GetAxisEncSourceType(%d)", &motor_axis_no);
  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEncSource(motor_axis_no,&iValue))
  }

  /*int GetAxisTrajSourceType(int axis_no);*/
  nvals = sscanf(myarg_1, "GetAxisTrajSourceType(%d)", &motor_axis_no);
  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisTrajSource(motor_axis_no,&iValue))
  }

  /*int GetAxisEnableCommandsFromOtherAxis(int axis_no);*/
  nvals = sscanf(myarg_1, "GetAxisEnableCommandsFromOtherAxis(%d)", &motor_axis_no);
  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEnableCommandsFromOtherAxis(motor_axis_no,&iValue))
  }

  /*int GetAxisEnableCommandsTransform(int axis_no);*/
  nvals = sscanf(myarg_1, "GetAxisEnableCommandsTransform(%d)", &motor_axis_no);
  if (nvals == 1) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEnableCommandsTransform(motor_axis_no,&iValue))
  }

  /*int GetAxisTrajTransExpr(int axis_no, char* cExpr);   */
  nvals = sscanf(myarg_1, "GetAxisTrajTransExpr(%d)",&iValue);
  if (nvals == 1){
    char *retBuf;
    int error=0;

    retBuf=strdup(getAxisTrajTransExpr(iValue, &error));
    if(error){
      sprintf(errorBuffer, "Error: %d", error);
      free(retBuf);
      return;
    }

    //Change all # to ; (since ; is used as command delimiter in tcpip communication)
    size_t strLen=strlen(retBuf);
    size_t i=0;
    for(i=0;i<strLen;i++){
      if(retBuf[i]==';'){
        retBuf[i]=TRANSFORM_EXPR_LINE_END_CHAR;
      }
    }
    cmd_buf_printf("%s",retBuf);
    free(retBuf);
    return;
  }

  /*int GetAxisEncTransExpr(int axis_no, char* cExpr);   */
  nvals = sscanf(myarg_1, "GetAxisEncTransExpr(%d)",&iValue);
  if (nvals == 1){
    char *retBuf;
    int error=0;

    retBuf=strdup(getAxisEncTransExpr(iValue, &error));
    if(error){
      sprintf(errorBuffer, "Error: %d", error);
      free(retBuf);
      return;
    }

    //Change all # to ; (since ; is used as command delimiter in tcpip communication)
    size_t strLen=strlen(retBuf);
    size_t i=0;
    for(i=0;i<strLen;i++){
      if(retBuf[i]==';'){
        retBuf[i]=TRANSFORM_EXPR_LINE_END_CHAR;
      }
    }
    cmd_buf_printf("%s",retBuf);
    free(retBuf);
    return;
  }

  /*int GetAxisEncTransExpr(int axis_no, char* cExpr);   */
  nvals = sscanf(myarg_1, "GetAxisTransformCommandExpr(%d)",&iValue);
  if (nvals == 1){
    char *retBuf;
    int error=0;

    retBuf=strdup(getAxisTransformCommandExpr(iValue, &error));
    if(error){
      sprintf(errorBuffer, "Error: %d", error);
      free(retBuf);
      return;
    }

    //Change all # to ; (since ; is used as command delimiter in tcpip communication)
    size_t strLen=strlen(retBuf);
    size_t i=0;
    for(i=0;i<strLen;i++){
      if(retBuf[i]==';'){
        retBuf[i]=TRANSFORM_EXPR_LINE_END_CHAR;
      }
    }
    cmd_buf_printf("%s",retBuf);
    free(retBuf);
    return;
  }

  /* Main.*/
  if (!strncmp(myarg_1, Main_dot_str, strlen(Main_dot_str))) {
    myarg_1 += strlen(Main_dot_str);
  }

  /* From here on, only M1. commands */
  /* e.g. M1.nCommand=3 */
  nvals = sscanf(myarg_1, "M%d.", &motor_axis_no);
  if (nvals != 1) {
    RETURN_OR_DIE("%s/%s:%d line=\"%s\" nvals=%d",
                  __FILE__, __FUNCTION__, __LINE__,
                  myarg, nvals);
  }
  AXIS_CHECK_RETURN(motor_axis_no);
  myarg_1 = strchr(myarg_1, '.');
  if (!myarg_1) {
    RETURN_OR_DIE("%s/%s:%d line=%s missing '.'",
                  __FILE__, __FUNCTION__, __LINE__,
                  myarg);
  }
  myarg_1++; /* Jump over '.' */

  if (0 == strcmp(myarg_1, "bBusy?")){
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisBusy(motor_axis_no,&iValue))
  }
  /* bError?  */
  if (!strcmp(myarg_1, "bError?")) {
    iValue=getAxisError(motor_axis_no);
    cmd_buf_printf("%d", iValue);
    return;
  }

  /* bEnable? */
  if (!strcmp(myarg_1, "bEnable?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEnable(motor_axis_no,&iValue))
  }

  /* bEnabled? */
  if (!strcmp(myarg_1, "bEnabled?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEnabled(motor_axis_no,&iValue))
  }
  /* bExecute? */
  if (!strcmp(myarg_1, "bExecute?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisExecute(motor_axis_no,&iValue))
  }

  /* bReset? */
    if (!strcmp(myarg_1, "bReset?")) {
      SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisReset(motor_axis_no,&iValue))
    }

  /* bHomeSensor? */
  if (0 == strcmp(myarg_1, "bHomeSensor?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisAtHome(motor_axis_no,&iValue))
  }

  /* bLimitBwd? */
  if (0 == strcmp(myarg_1, "bLimitBwd?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisAtHardBwd(motor_axis_no,&iValue))
  }
  /* bLimitFwd? */
  if (0 == strcmp(myarg_1, "bLimitFwd?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisAtHardFwd(motor_axis_no,&iValue))
  }
  /* bHomed? */
  if (0 == strcmp(myarg_1, "bHomed?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisEncHomed(motor_axis_no,&iValue))
  }
  /* fActPosition? */
  if (0 == strcmp(myarg_1, "fActPosition?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisEncPosAct(motor_axis_no,&fValue))
  }
  /* fActVelocity? */
  if (0 == strcmp(myarg_1, "fActVelocity?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisEncVelAct(motor_axis_no,&fValue))
  }

  /* fVelocity? */
  if (0 == strcmp(myarg_1, "fVelocity?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisTargetVel(motor_axis_no,&fValue))
  }

  /* fPosition? */
  if (0 == strcmp(myarg_1, "fPosition?")) {
    /* The "set" value */
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisTargetPos(motor_axis_no,&fValue))
  }

  /* nCommand? */
  if (0 == strcmp(myarg_1, "nCommand?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisCommand(motor_axis_no,&iValue))
  }

  /* nCmdData? */
  if (0 == strcmp(myarg_1, "nCmdData?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisCmdData(motor_axis_no,&iValue))
  }

  /*nMotionAxisID? */
  if (0 == strcmp(myarg_1, "nMotionAxisID?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_INT(getAxisID(motor_axis_no,&iValue))
  }

  /*fAcceleration? */
  if (0 == strcmp(myarg_1, "fAcceleration?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisAcceleration(motor_axis_no,&fValue))
  }

  /*fDeceleration? */
  if (0 == strcmp(myarg_1, "fDeceleration?")) {
    SEND_RESULT_OR_ERROR_AND_RETURN_DOUBLE(getAxisDeceleration(motor_axis_no,&fValue))
  }

  /* stAxisStatus? */
  if (0 == strcmp(myarg_1, "stAxisStatus?")) {
    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisEnabled(motor_axis_no,&iValue))
    int bEnable = iValue;

    int bReset = 0;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisExecute(motor_axis_no,&iValue))
    int bExecute =iValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisCommand(motor_axis_no,&iValue))
    unsigned nCommand = iValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisCmdData(motor_axis_no,&iValue))
    unsigned nCmdData = iValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisTargetVel(motor_axis_no,&fValue))
    double fVelocity = fValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisTargetPos(motor_axis_no,&fValue))
    double fPosition = fValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisAcceleration(motor_axis_no,&fValue))
    double fAcceleration = fValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisDeceleration(motor_axis_no,&fValue))
    double fDecceleration = fValue;

    int bJogFwd = 0;
    int bJogBwd = 0;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisAtHardFwd(motor_axis_no,&iValue))
    int bLimitFwd = iValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisAtHardBwd(motor_axis_no,&iValue))
    int bLimitBwd = iValue;;


    double fOverride = 100;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisAtHome(motor_axis_no,&iValue))
    int bHomeSensor = iValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisEnabled(motor_axis_no,&iValue))
    int bEnabled = iValue;

    int bError = getAxisError(motor_axis_no);
    unsigned nErrorId = getAxisErrorID(motor_axis_no);

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisEncVelAct(motor_axis_no,&fValue))
    double fActVelocity = fValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisEncPosAct(motor_axis_no,&fValue))
    double fActPostion = fValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisCntrlError(motor_axis_no,&fValue))
    double fActDiff = fValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisEncHomed(motor_axis_no,&iValue))
    int bHomed = iValue;

    IF_ERROR_SEND_ERROR_AND_RETURN(getAxisBusy(motor_axis_no,&iValue))
    int bBusy = iValue;

    cmd_buf_printf("Main.M%d.stAxisStatus="
                   "%d,%d,%d,%u,%u,%g,%g,%g,%g,%d,"
                   "%d,%d,%d,%g,%d,%d,%d,%u,%g,%g,%g,%d,%d",
                   motor_axis_no,
                   bEnable,        /*  1 */
                   bReset,         /*  2 */
                   bExecute,       /*  3 */
                   nCommand,       /*  4 */
                   nCmdData,       /*  5 */
                   fVelocity,      /*  6 */
                   fPosition,      /*  7 */
                   fAcceleration,  /*  8 */
                   fDecceleration, /*  9 */
                   bJogFwd,        /* 10 */
                   bJogBwd,        /* 11 */
                   bLimitFwd,      /* 12 */
                   bLimitBwd,      /* 13 */
                   fOverride,      /* 14 */
                   bHomeSensor,    /* 15 */
                   bEnabled,       /* 16 */
                   bError,         /* 17 */
                   nErrorId,       /* 18 */
                   fActVelocity,   /* 19 */
                   fActPostion,    /* 20 */
                   fActDiff,       /* 21 */
                   bHomed,         /* 22 */
                   bBusy           /* 23 */
                   );
    return;
  }
  /* sErrorMessage?  */
  if (!strcmp(myarg_1, "sErrorMessage?")) {
    cmd_buf_printf("%s", getErrorString(getAxisErrorID(motor_axis_no)));
    return;
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
  /* End of "get" commands, from here, set commands */

   /* fHomePosition=100 */
   nvals = sscanf(myarg_1, "fHomePosition=%lf", &fValue);
   if (nvals == 1) {
     SEND_OK_OR_ERROR_AND_RETURN(setAxisHomePos(motor_axis_no, fValue));
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

  /* if we come here, we do not understand the command */
  RETURN_OR_DIE("%s/%s:%d line=%s",
                __FILE__, __FUNCTION__, __LINE__,
                myarg);
}

void cmd_EAT(int argc, const char *argv[], const char *sepv[])
{
  const char *myargline = (argc > 0) ? argv[0] : "";
  int i;
  if (PRINT_STDOUT_BIT6())
  {
    const char *myarg[6];
    myarg[0] = myargline;
    myarg[1] = (argc >= 1) ? argv[1] : "";
    myarg[2] = (argc >= 2) ? argv[2] : "";
    myarg[3] = (argc >= 3) ? argv[3] : "";
    myarg[4] = (argc >= 4) ? argv[4] : "";
    myarg[5] = (argc >= 5) ? argv[5] : "";
    LOGINFO6("%s/%s:%d argc=%d "
             "myargline=\"%s\" myarg[1]=\"%s\" myarg[2]=\"%s\" myarg[3]=\"%s\" myarg[4]=\"%s\" myarg[5]=\"%s\"\n",
             __FILE__, __FUNCTION__, __LINE__,
             argc,  myargline,
             myarg[1], myarg[2], myarg[3], myarg[4], myarg[5]);
  }

  for (i = 1; i <= argc; i++) {
    motorHandleOneArg(argv[i]);
    cmd_buf_printf("%s", sepv[i]);
    if (PRINT_STDOUT_BIT6()) {
      LOGINFO6("%s/%s:%d i=%d "
               "argv[%d]=%s, sepv[%d]=\"",
               __FILE__, __FUNCTION__, __LINE__,
               argc, i, argv[i], i);
      cmd_dump_to_std(sepv[i], strlen(sepv[i]));
      LOGINFO6("\"\n");
    }
  } /* while argc > 0 */
}
/******************************************************************************/
