/*
FILENAME...   ecmcMotorRecordController.h
*/

#ifndef ECMC_MOTOR_RECORD_CONTROLLER_H
#define ECMC_MOTOR_RECORD_CONTROLLER_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "ecmcMotorRecordAxis.h"
#include "ecmcPVTController.h"

extern ecmcAsynPortDriver *asynPort;

#ifndef motorRecResolutionString
#define CREATE_MOTOR_REC_RESOLUTION
#define motorRecDirectionString         "MOTOR_REC_DIRECTION"
#define motorRecOffsetString            "MOTOR_REC_OFFSET"
#define motorRecResolutionString        "MOTOR_REC_RESOLUTION"
#endif // ifndef motorRecResolutionString

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
#define ecmcMotorRecordHomUseHVELString           "HomeUseHVEL"
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
#define ecmcMotorRecordTRIGG_STOPString           "TRIGG_STOPP"
#define ecmcMotorRecordTRIGGDISABLEString         "TRIGG_DISABLE"
#define ecmcMotorRecordTRIGGSYNCString            "TRIGG_SYNC"

#define ecmcMotorRecordMCUErrMsgString            "MCUErrMsg"
#define ecmcMotorRecordDbgStrToMcuString          "StrToMCU"
#define ecmcMotorRecordDbgStrToLogString          "StrToLOG"

#define HOMPROC_MANUAL_SETPOS    15
#define MAX_MESSAGE_LEN   256
#define ECMC_MR_CNTRL_ADDR 0

extern const char *modNamEMC;

extern "C" {
int ecmcMotorRecordCreateAxis(const char *ecmcMotorRecordName,
                              int         axisNo,
                              int         axisFlags,
                              const char *axisOptionsStr);
const char* ecmcMotorRecordstrStatus(asynStatus status);
const char* errStringFromErrId(int nErrorId);
}

class epicsShareClass ecmcMotorRecordController : public asynMotorController {
public:
#define FEATURE_BITS_V2               (1 << 1)
#define FEATURE_BITS_ECMC             (1 << 5)

  ecmcMotorRecordController(const char *portName,
                            const char *ecmcMotorRecordPortName,
                            int         numAxes,
                            double      movingPollPeriod,
                            double      idlePollPeriod,
                            const char *optionStr);

  ~ecmcMotorRecordController();
  void                 report(FILE *fp,
                              int   level);
  asynStatus           setMCUErrMsg(const char *value);

  // asynStatus configController(int needOk, const char *value);
  ecmcMotorRecordAxis* getAxis(asynUser *pasynUser);
  ecmcMotorRecordAxis* getAxis(int axisNo);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value,size_t nElements);
  // Have to override the profile* since basecalse does not include error and status handling
  asynStatus initializeProfile(size_t maxProfilePoints);
  asynStatus buildProfile();
  asynStatus executeProfile();
  asynStatus abortProfile();
  asynStatus readbackProfile();
  asynStatus enableAxisPVTFunc(int axisNo, int enable);
  int features_;

protected:
  ecmcMotorRecordAxis **pAxes_;       /**< Array of pointers to axis objects */
  //void udateMotorLimitsRO(int axisNo);
  //void udateMotorLimitsRO(int    axisNo,
  //                        int    enabledHighAndLow,
  //                        double fValueHigh,
  //                        double fValueLow);
  void       handleStatusChange(asynStatus status);
  int        getFeatures(void);
  asynStatus poll();
  void       profilePoll();
  ecmcPVTController *getPVTController();
  void readEcmcControllerStatus();

  struct {
    asynStatus   oldStatus;
    unsigned int initialPollDone;
    double       movingPollPeriod;
    double       idlePollPeriod;
    int          errorId;
    int          pvtErrorId;
    int          pvtCurrentTriggerId;
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
  int ecmcMotorRecordHomUseHVEL_;
  int ecmcMotorRecordEncAct_;

#ifdef CREATE_MOTOR_REC_RESOLUTION
  int motorRecResolution_;
  int motorRecDirection_;
  int motorRecOffset_;
#endif // ifdef CREATE_MOTOR_REC_RESOLUTION

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
  int ecmcMotorRecordTRIGG_STOPP_;
  int ecmcMotorRecordTRIGG_DISABLE_;
  int ecmcMotorRecordTRIGG_SYNC_;

  int ecmcMotorRecordErrId_;
  
  // Custom profile 

  int profileInitialized_;
  int profileBuilt_;
  
  /* Last parameter */

  #define FIRST_VIRTUAL_PARAM ecmcMotorRecordErr_
  #define LAST_VIRTUAL_PARAM ecmcMotorRecordErrId_
  #define NUM_VIRTUAL_MOTOR_PARAMS ((int)(&LAST_VIRTUAL_PARAM -\
                                          &FIRST_VIRTUAL_PARAM + 1))
  
  asynStatus profileValidateTime();
  void setProfileInProgress(bool progress);
  char profileMessage_[MAX_MESSAGE_LEN];
  ecmcPVTController *pvtController_;
  bool profileInProgress_;
  size_t profileTimeArraySize_;

  friend class ecmcMotorRecordAxis;
};

#endif // ifndef ECMC_MOTOR_RECORD_CONTROLLER_H
