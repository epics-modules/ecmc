/*
FILENAME...   ecmcMotorRecordController.h
*/

#ifndef ECMC_MOTOR_RECORD_CONTROLLER_H
#define ECMC_MOTOR_RECORD_CONTROLLER_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "ecmcMotorRecordAxis.h"


#ifndef motorRecResolutionString
#define CREATE_MOTOR_REC_RESOLUTION
#define motorRecDirectionString         "MOTOR_REC_DIRECTION"
#define motorRecOffsetString            "MOTOR_REC_OFFSET"
#define motorRecResolutionString        "MOTOR_REC_RESOLUTION"
#endif

#define ecmcMotorRecordErrString                  "MCUErr"
#define ecmcMotorRecordErrIdString                "ErrId"
#define ecmcMotorRecordStatusCodeString           "StatusCode"
#define ecmcMotorRecordStatusBitsString           "StatusBits"
#define ecmcMotorRecordaux0_String                "AuxBit0"
#define ecmcMotorRecordaux1_String                "AuxBit1"
#define ecmcMotorRecordaux2_String                "AuxBit2"
#define ecmcMotorRecordaux3_String                "AuxBit3"
#define ecmcMotorRecordaux4_String                "AuxBit4"
#define ecmcMotorRecordaux5_String                "AuxBit5"
#define ecmcMotorRecordaux6_String                "AuxBit6"
#define ecmcMotorRecordaux7_String                "AuxBit7"
#define ecmcMotorRecordreason24_String            "ReasonBit24"
#define ecmcMotorRecordreason25_String            "ReasonBit25"
#define ecmcMotorRecordreason26_String            "ReasonBit26"
#define ecmcMotorRecordreason27_String            "ReasonBit27"
#define ecmcMotorRecordHomProc_RBString           "HomProc-RB"
#define ecmcMotorRecordHomPos_RBString            "HomPos-RB"
#define ecmcMotorRecordHomProcString              "HomProc"
#define ecmcMotorRecordHomPosString               "HomPos"
#define ecmcMotorRecordVelToHomString             "VelToHom"
#define ecmcMotorRecordVelFrmHomString            "VelFrmHom"
#define ecmcMotorRecordAccHomString               "AccHom"
#define ecmcMotorRecordEnc_ActString              "EncAct"
#define ecmcMotorRecordErrRstString               "ErrRst"
#define ecmcMotorRecordVelActString               "VelAct"
#define ecmcMotorRecordVel_RBString               "Vel-RB"
#define ecmcMotorRecordAcc_RBString               "Acc-RB"
#define ecmcMotorRecordCfgVELO_String             "CfgVELO"
#define ecmcMotorRecordCfgVMAX_String             "CfgVMAX"
#define ecmcMotorRecordCfgJVEL_String             "CfgJVEL"
#define ecmcMotorRecordCfgACCS_String             "CfgACCS"
#define ecmcMotorRecordCfgDHLMString              "CfgDHLM"
#define ecmcMotorRecordCfgDLLMString              "CfgDLLM"
#define ecmcMotorRecordCfgDHLM_EnString           "CfgDHLM-En"
#define ecmcMotorRecordCfgDLLM_EnString           "CfgDLLM-En"
#define ecmcMotorRecordCfgSREV_RBString           "CfgSREV-RB"
#define ecmcMotorRecordCfgUREV_RBString           "CfgUREV-RB"
#define ecmcMotorRecordCfgPMIN_RBString           "CfgPMIN-RB"
#define ecmcMotorRecordCfgPMAX_RBString           "CfgPMAX-RB"
#define ecmcMotorRecordCfgSPDB_RBString           "CfgSPDB-RB"
#define ecmcMotorRecordCfgRDBD_RBString           "CfgRDBD-RB"
#define ecmcMotorRecordCfgRDBD_Tim_RBString       "CfgRDBD-Tim-RB"
#define ecmcMotorRecordCfgRDBD_En_RBString        "CfgRDBD-En-RB"
#define ecmcMotorRecordCfgPOSLAG_RBString         "CfgPOSLAG-RB"
#define ecmcMotorRecordCfgPOSLAG_Tim_RBString     "CfgPOSLAG-Tim-RB"
#define ecmcMotorRecordCfgPOSLAG_En_RBString      "CfgPOSLAG-En-RB"
#define ecmcMotorRecordCfgDESC_RBString           "CfgDESC-RB"
#define ecmcMotorRecordCfgEGU_RBString            "CfgEGU-RB"


#define ecmcMotorRecordMCUErrMsgString            "MCUErrMsg"
#define ecmcMotorRecordDbgStrToMcuString          "StrToMCU"
#define ecmcMotorRecordDbgStrToLogString          "StrToLOG"

#define HOMPROC_MANUAL_SETPOS    15

extern const char *modNamEMC;

extern "C" {
  unsigned   netToUint(const void *data, size_t lenInPlc);
  double     netToDouble(const void *data, size_t lenInPlc);
  void       doubleToNet(const double value, void *data, size_t lenInPlc);
  void       uintToNet(const unsigned value, void *data, size_t lenInPlc);
  int ecmcMotorRecordCreateAxis(const char *ecmcMotorRecordName, int axisNo,
                           int axisFlags, const char *axisOptionsStr);

  asynStatus disconnect_C(asynUser *pasynUser);
  asynStatus checkACK(const char *outdata, size_t outlen, const char *indata);
  const char *plcUnitTxtFromUnitCode(unsigned unitCode);
  const char *ecmcMotorRecordstrStatus(asynStatus status);
  const char *errStringFromErrId(int nErrorId);
}
#define NETTOUINT(n)       netToUint((const void*)&n, sizeof(n))
#define NETTODOUBLE(n)     netToDouble((const void*)&n, sizeof(n))
#define UINTTONET(val,n)   uintToNet((val), (&n), sizeof(n))
#define DOUBLETONET(val,n) doubleToNet((val), (&n), sizeof(n))

class epicsShareClass ecmcMotorRecordController : public asynMotorController {
public:
#define PARAM_IDX_OPMODE_AUTO_UINT32            1
#define PARAM_IDX_MICROSTEPS_UINT32             2
#define PARAM_IDX_ABS_MIN_FLOAT32              30
#define PARAM_IDX_ABS_MAX_FLOAT32              31
#define PARAM_IDX_USR_MIN_FLOAT32              32
#define PARAM_IDX_USR_MAX_FLOAT32              33
#define PARAM_IDX_WRN_MIN_FLOAT32              34
#define PARAM_IDX_WRN_MAX_FLOAT32              35
#define PARAM_IDX_FOLLOWING_ERR_WIN_FLOAT32    55
#define PARAM_IDX_HYTERESIS_FLOAT32            56
#define PARAM_IDX_REFSPEED_FLOAT32             58
#define PARAM_IDX_VBAS_FLOAT32                 59
#define PARAM_IDX_SPEED_FLOAT32                60
#define PARAM_IDX_ACCEL_FLOAT32                61
#define PARAM_IDX_IDLE_CURRENT_FLOAT32         62
#define PARAM_IDX_MOVE_CURRENT_FLOAT32         64
#define PARAM_IDX_MICROSTEPS_FLOAT32           67
#define PARAM_IDX_STEPS_PER_UNIT_FLOAT32       68
#define PARAM_IDX_HOME_POSITION_FLOAT32        69
#define PARAM_IDX_FUN_REFERENCE               133
#define PARAM_IDX_FUN_SET_POSITION            137
#define PARAM_IDX_FUN_MOVE_VELOCITY           142


#define FEATURE_BITS_V1               (1)
#define FEATURE_BITS_V2               (1<<1)
#define FEATURE_BITS_V3               (1<<2)
#define FEATURE_BITS_V4               (1<<3)

#define FEATURE_BITS_ECMC             (1<<5)


  ecmcMotorRecordController(const char *portName, const char *ecmcMotorRecordPortName,
                       int numAxes, double movingPollPeriod,
                       double idlePollPeriod,
                       const char *optionStr);

  ~ecmcMotorRecordController();
  void report(FILE *fp, int level);
  asynStatus setMCUErrMsg(const char *value);
  asynStatus configController(int needOk, const char *value);
  //asynStatus writeReadOnErrorDisconnect(void);
  ecmcMotorRecordAxis* getAxis(asynUser *pasynUser);
  ecmcMotorRecordAxis* getAxis(int axisNo);
  int features_;

  protected:
  void udateMotorLimitsRO(int axisNo);
  void udateMotorLimitsRO(int axisNo, int enabledHighAndLow,
                          double fValueHigh, double fValueLow);
  void handleStatusChange(asynStatus status);
  asynStatus writeReadBinaryOnErrorDisconnect(asynUser *pasynUser,
                                              const char *outdata,
                                              size_t outlen,
                                              char *indata, size_t inlen,
                                              size_t *pnread);

  asynStatus writeReadControllerPrint(int traceMask);
  asynStatus writeReadACK(int traceMask);
  int getFeatures(void);
  asynStatus poll();

    struct {
    asynStatus   oldStatus;
    unsigned int local_no_ASYN_;
    unsigned int hasConfigError;
    unsigned int initialPollDone;
    unsigned int indexerOffset;
    unsigned int specialDbgStrToMcuDeviceLength;
    unsigned int specialDbgStrToMcuDeviceOffset;
    unsigned adsport;
    int useADSbinary;
    struct {
      unsigned int stAxisStatus_V1  :1;
      unsigned int stAxisStatus_V2  :1;
      unsigned int bSIM             :1;
      unsigned int bECMC            :1;
      unsigned int bADS             :1;
    } supported;
  } ctrlLocal;

  /* First parameter */
  int ecmcMotorRecordErr_;
  int ecmcMotorRecordStatusCode_;
  int ecmcMotorRecordStatusBits_;
  int ecmcMotorRecordaux0_;
  int ecmcMotorRecordaux1_;
  int ecmcMotorRecordaux2_;
  int ecmcMotorRecordaux3_;
  int ecmcMotorRecordaux4_;
  int ecmcMotorRecordaux5_;
  int ecmcMotorRecordaux6_;
  int ecmcMotorRecordaux7_;
  int ecmcMotorRecordreason24_;
  int ecmcMotorRecordreason25_;
  int ecmcMotorRecordreason26_;
  int ecmcMotorRecordreason27_;
  int ecmcMotorRecordHomProc_RB_;
  int ecmcMotorRecordHomPos_RB_;
  int ecmcMotorRecordHomProc_;
  int ecmcMotorRecordHomPos_;
  int ecmcMotorRecordVelToHom_;
  int ecmcMotorRecordVelFrmHom_;
  int ecmcMotorRecordAccHom_;
  int ecmcMotorRecordEncAct_;

#ifdef CREATE_MOTOR_REC_RESOLUTION
  int motorRecResolution_;
  int motorRecDirection_;
  int motorRecOffset_;
#endif

  /* Add parameters here */
  int ecmcMotorRecordErrRst_;
  int ecmcMotorRecordMCUErrMsg_;
  int ecmcMotorRecordDbgStrToMcu_;
  int ecmcMotorRecordDbgStrToLog_;
  int ecmcMotorRecordVelAct_;
  int ecmcMotorRecordVel_RB_;
  int ecmcMotorRecordAcc_RB_;
  int ecmcMotorRecordCfgVELO_;
  int ecmcMotorRecordCfgVMAX_;
  int ecmcMotorRecordCfgJVEL_;
  int ecmcMotorRecordCfgACCS_;
  int ecmcMotorRecordCfgSREV_RB_;
  int ecmcMotorRecordCfgUREV_RB_;
  int ecmcMotorRecordCfgPMIN_RB_;
  int ecmcMotorRecordCfgPMAX_RB_;
  int ecmcMotorRecordCfgSPDB_RB_;
  int ecmcMotorRecordCfgRDBD_RB_;
  int ecmcMotorRecordCfgRDBD_Tim_RB_;
  int ecmcMotorRecordCfgRDBD_En_RB_;
  int ecmcMotorRecordCfgPOSLAG_RB_;
  int ecmcMotorRecordCfgPOSLAG_Tim_RB_;
  int ecmcMotorRecordCfgPOSLAG_En_RB_;
  int ecmcMotorRecordCfgDHLM_;
  int ecmcMotorRecordCfgDLLM_;
  int ecmcMotorRecordCfgDHLM_En_;
  int ecmcMotorRecordCfgDLLM_En_;
  int ecmcMotorRecordCfgDESC_RB_;
  int ecmcMotorRecordCfgEGU_RB_;

  int ecmcMotorRecordErrId_;
  /* Last parameter */

  // ECMC
  char  *mcuPortName_;
  double movingPollPeriod_;
  double idlePollPeriod_;
  // ECMC

  #define FIRST_VIRTUAL_PARAM ecmcMotorRecordErr_
  #define LAST_VIRTUAL_PARAM ecmcMotorRecordErrId_
  #define NUM_VIRTUAL_MOTOR_PARAMS ((int) (&LAST_VIRTUAL_PARAM - &FIRST_VIRTUAL_PARAM + 1))
  friend class ecmcMotorRecordAxis;  
};

#endif
