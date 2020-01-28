/*
FILENAME...   ecmcMotorRecordAxis.h
*/

#ifndef ECMC_MOTOR_RECORD_AXIS_H
#define ECMC_MOTOR_RECORD_AXIS_H

#include "asynMotorAxis.h"
#include <stdint.h>
#include <asynInt32SyncIO.h>
#include <asynFloat64SyncIO.h>
#include <asynInt8ArraySyncIO.h>

#include "../main/ecmcGlobalsExtern.h"

#define AMPLIFIER_ON_FLAG_CREATE_AXIS  (1)
#define AMPLIFIER_ON_FLAG_AUTO_ON      (1<<1)
#define AMPLIFIER_ON_FLAG_USING_CNEN   (1<<2)

// ECMC ###########################################################################################
#define ECMC_MAX_ASYN_DIAG_STR_LEN    256
#define ECMC_MAX_ASYN_DRVINFO_STR_LEN 128
// Statuses
#define ECMC_ASYN_AXIS_STAT_STRING          "T_SMP_MS=%d/TYPE=asynInt32/ax%d.status?"
#define ECMC_ASYN_AXIS_DIAG_STRING          "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnostic?"
#define ECMC_ASYN_AXIS_STAT_BIN_STRING      "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.statusbin?"
// Control
#define ECMC_ASYN_AXIS_CONT_STRING          "T_SMP_MS=%d/TYPE=asynInt8ArrayOut/ax%d.controlbin?"

/*
typedef struct {
  unsigned char              enable:1;
  unsigned char              execute:1;
  unsigned char              reset:1;
  unsigned char              softlimfwdena:1;
  unsigned char              softlimbwdena:1;
  unsigned char              trigg:4;
  unsigned char              unused_2:8;
  unsigned char              cmd:8;
  unsigned int               cmddata:8;
}ecmcAxisControlWordType;

enum motionCommandTypes {             // Data order for motor record communications
  ECMC_CMD_NOCMD              = -1,   
  ECMC_CMD_JOG                = 0,    
  ECMC_CMD_MOVEVEL            = 1,    // cmd, vel, acc
  ECMC_CMD_MOVEREL            = 2,    // cmd, pos, vel, acc
  ECMC_CMD_MOVEABS            = 3,    // cmd, pos, vel, acc
  ECMC_CMD_MOVEMODULO         = 4,    
  ECMC_CMD_HOMING             = 10,   // cmd, seqnbr,homepos,velhigh,vellow,acc
  ECMC_CMD_SUPERIMP           = 20,   
  ECMC_CMD_GEAR               = 30,   
  ECMC_CMD_STOP               = 100,  // cmd, (Should have been 0 instead of 100)
  ECMC_CMD_SET_ENABLE         = 101,  // cmd, enable
  ECMC_CMD_SET_SOFTLIMBWD     = 102,  // cmd, soflimbwd
  ECMC_CMD_SET_SOFTLIMFWD     = 103,  // cmd, soflimfwd
  ECMC_CMD_SET_SOFTLIMBWD_ENA = 104,  // cmd, soflimbwdena
  ECMC_CMD_SET_SOFTLIMFWD_ENA = 105,  // cmd, soflimfwdena
  ECMC_CMD_SET_RESET          = 106,  // cmd
};

typedef struct {
  motionCommandTypes         cmd;  
  double                     val0;
  double                     val1;
  double                     val2;
  double                     val3;
  double                     val4;
  double                     val5;
  double                     val6;
  double                     val7;
  double                     val8;
  double                     val9;
}ecmcAsynClinetCmdType;

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

enum dataSource {
  ECMC_DATA_SOURCE_INTERNAL           = 0,
  ECMC_DATA_SOURCE_EXTERNALENCODER    = 1,
  ECMC_DATA_SOURCE_EXTERNALTRAJECTORY = 2
};
typedef struct {
  unsigned char              enable        : 1;
  unsigned char              enabled       : 1;
  unsigned char              execute       : 1;
  unsigned char              busy          : 1;
  unsigned char              attarget      : 1;
  unsigned char              moving        : 1;
  unsigned char              limitfwd      : 1;
  unsigned char              limitbwd      : 1;
  unsigned char              homeswitch    : 1;
  unsigned char              instartup     : 1;
  unsigned char              inrealtime    : 1;
  unsigned char              trajsource    : 1;
  unsigned char              encsource     : 1;
  unsigned char              plccmdallowed : 1;
  unsigned char              softlimfwdena : 1;
  unsigned char              softlimbwdena : 1;
  unsigned char              homed         : 1;
  unsigned char              sumilockfwd   : 1;
  unsigned char              sumilockbwd   : 1;
  unsigned char              unused        : 1;
  unsigned char              seqstate      : 4;
  unsigned char              lastilock     : 8;
} ecmcAxisStatusWordType;

typedef struct {
  double             positionSetpoint;
  double             positionActual;
  double             positionError;
  double             positionTarget;
  double             cntrlError;
  double             cntrlOutput;
  double             velocityActual;
  double             velocitySetpoint;
  double             velocityFFRaw;
  int64_t            positionRaw;
  double             homeposition;
  int                error;
  int                velocitySetpointRaw;
  int                seqState;
  int                cmdData;
  motionCommandTypes command;
  interlockTypes     trajInterlock;
  ecmcAxisStatusWordType statusWd;
} ecmcAxisStatusOnChangeType;

typedef struct {
  int                        axisID;
  int                        cycleCounter;
  double                     acceleration;
  double                     deceleration;
  double                     soflimFwd;
  double                     soflimBwd;
  bool                       reset  : 1;
  bool                       moving : 1;
  bool                       stall  : 1;
  ecmcAxisStatusOnChangeType onChangeData;
} ecmcAxisStatusType;
*/
//ECMC End

extern const char *modNamEMC;

extern "C" {
  int ecmcMotorRecordCreateAxis(const char *ecmcMotorRecordName, int axisNo,
                      int axisFlags, const char *axisOptionsStr);
  asynStatus writeReadOnErrorDisconnect_C(asynUser *pasynUser,
                                          const char *outdata, size_t outlen,
                                          char *indata, size_t inlen);
  asynStatus checkACK(const char *outdata, size_t outlen, const char *indata);
  double ecmcMotorRecordgetNowTimeSecs(void);
}

typedef struct {
  /* V1 members */
  int bEnable;           /*  1 */
  int bReset;            /*  2 */
  int bExecute;          /*  3 */
  int nCommand;          /*  4 */
  int nCmdData;          /*  5 */
  double fVelocity;      /*  6 */
  double fPosition;      /*  7 */
  double fAcceleration;  /*  8 */
  double fDecceleration; /*  9 */
  int bJogFwd;           /* 10 */
  int bJogBwd;           /* 11 */
  int bLimitFwd;         /* 12 */
  int bLimitBwd;         /* 13 */
  double fOverride;      /* 14 */
  int bHomeSensor;       /* 15 */
  int bEnabled;          /* 16 */
  int bError;            /* 17 */
  int nErrorId;          /* 18 */
  double fActVelocity;   /* 19 */
  double fActPosition;   /* 20 */
  double fActDiff;       /* 21 */
  int bHomed;            /* 22 */
  int bBusy;             /* 23 */
  /* V2 members */
  double encoderRaw;
  int atTarget;
  /* neither V1 nor V2, but calculated here */
  int mvnNRdyNex; /* Not in struct. Calculated in poll() */
  int motorStatusDirection; /* Not in struct. Calculated in pollAll() */
  int motorDiffPostion;     /* Not in struct. Calculated in poll() */
} st_axis_status_type;

class epicsShareClass ecmcMotorRecordAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  ecmcMotorRecordAxis(class ecmcMotorRecordController *pC,
                      int axisNo,
                      ecmcAxisBase *ecmcAxisRef,
                      int axisFlags, 
                      const char *axisOptionsStr);

  void report(FILE *fp, int level);
  void callParamCallbacksUpdateError();

  // done ECMC
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus stop(double acceleration);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus setPosition(double value);
  // partly ECMC
  asynStatus poll(bool *moving);

  // ECMC  added##############
  //asynStatus statBinDataCallback(epicsInt8 *data);
  //asynStatus contBinDataCallback(epicsInt8 *data);

  // ECMC End ##############
private:
  typedef enum
  {
    eeAxisErrorIOCcomError = -1,
    eeAxisErrorNoError,
    eeAxisErrorMCUError,
    eeAxisErrorCmdError,
    eeAxisErrorNotFound,
    eeAxisErrorNotHomed,
    eeAxisIllegalInTargetWindow
  } eeAxisErrorType;

  typedef enum
  {
    eeAxisWarningNoWarning,
    eeAxisWarningCfgZero,
    eeAxisWarningVeloZero,
    eeAxisWarningSpeedLimit
  } eeAxisWarningType;

  typedef enum
  {
    pollNowReadScaling,
    pollNowReadMonitoring,
    pollNowReadBackSoftLimits,
    pollNowReadBackVelocities
  } eeAxisPollNowType;

  ecmcMotorRecordController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  struct {
    st_axis_status_type old_st_axis_status;
    double scaleFactor;
    const char *externalEncoderStr;
    const char *cfgfileStr;
    // const char *cfgDebug_str;
    int axisFlags;
    int MCU_nErrorId;     /* nErrorID from MCU */
    int old_MCU_nErrorId; /* old nErrorID from MCU */
    int old_EPICS_nErrorId; /* old nErrorID from MCU */

    int old_bError;   /* copy of bError */

    unsigned int waitNumPollsBeforeReady;

    int nCommandActive;
    int old_nCommandActive;
    int homed;
    unsigned int illegalInTargetWindow :1;
    eeAxisWarningType old_eeAxisWarning;
    eeAxisWarningType eeAxisWarning;
    eeAxisErrorType old_eeAxisError;
    eeAxisErrorType eeAxisError;
    eeAxisPollNowType eeAxisPollNow;
    /* Which values have changed in the EPICS IOC, but are not updated in the
       motion controller */
    struct {
      unsigned int oldStatusDisconnected : 1;
      unsigned int sErrorMessage    :1; /* From MCU */
      unsigned int initialPollNeeded :1;
    }  dirty;

    struct {
      int          statusVer;           /* 0==V1, busy old style 1==V1, new style*/
    }  supported;

    /* Error texts when we talk to the controller, there is not an "OK"
       Or, failure in setValueOnAxisVerify() */
    char cmdErrorMessage[80]; /* From driver */
    char sErrorMessage[80]; /* From controller */
    char adsport_str[15]; /* "ADSPORT=12345/" */ /* 14 should be enough, */
    unsigned int adsPort;
  } drvlocal;

  // asynStatus readConfigLine(const char *line, const char **errorTxt_p);
  // asynStatus readConfigFile(void);
  asynStatus readBackAllConfig(int axisID);
  asynStatus updateCfgValue(int function, int    newValue, const char *name);
  asynStatus updateCfgValue(int function, double newValue, const char *name);
  asynStatus readBackSoftLimits(void);
  // asynStatus readBackHoming(void);
  asynStatus readScaling(int axisID);
  asynStatus readMonitoring(int axisID);
  asynStatus readBackVelocities(int axisID);
  // asynStatus readBackEncoders(int axisID);
  asynStatus initialPoll(void);
  asynStatus initialPollInternal(void);
  // asynStatus setValueOnAxis(const char* var, int value);
  // asynStatus setValueOnAxisVerify(const char *var, const char *rbvar,
  //                                 int value, unsigned int retryCount);
  // asynStatus setValueOnAxis(const char* var, double value);
  // asynStatus setValuesOnAxis(const char* var1, double value1, const char* var2, double value2);
  // asynStatus setSAFValueOnAxis(unsigned indexGroup,
  //                              unsigned indexOffset,
  //                              int value);
  // asynStatus setSAFValueOnAxisVerify(unsigned indexGroup,
  //                                    unsigned indexOffset,
  //                                    int value,
  //                                    unsigned int retryCount);
  // asynStatus setSAFValueOnAxis(unsigned indexGroup,
  //                              unsigned indexOffset,
  //                              double value);
  // asynStatus setSAFValueOnAxisVerify(unsigned indexGroup,
  //                                    unsigned indexOffset,
  //                                    double value,
  //                                    unsigned int retryCount);
  // asynStatus getSAFValueFromAxisPrint(unsigned indexGroup,
  //                                     unsigned indexOffset,
  //                                     const char *name,
  //                                     int *value);
  // asynStatus getSAFValuesFromAxisPrint(unsigned iIndexGroup,
  //                                      unsigned iIndexOffset,
  //                                      const char *iname,
  //                                      int *iValue,
  //                                      unsigned fIndexGroup,
  //                                      unsigned fIndexOffset,
  //                                      const char *fname,
  //                                      double *fValue);
  // asynStatus getSAFValueFromAxisPrint(unsigned indexGroup,
  //                                     unsigned indexOffset,
  //                                     const char *name,
  //                                     double *value);
  // asynStatus getValueFromAxis(const char* var, int *value);
  // asynStatus getValueFromAxis(const char* var, double *value);
  // asynStatus getStringFromAxis(const char* var, char *value, size_t maxlen);
  // asynStatus getValueFromController(const char* var, double *value);  
  asynStatus setIntegerParam(int function, int value);
  asynStatus setDoubleParam(int function, double value);
  // asynStatus setStringParamDbgStrToMcu(const char *value);
  // asynStatus setStringParam(int function, const char *value);
#ifndef motorMessageTextString
  void updateMsgTxtFromDriver(const char *value);
#endif

  //  Done
  asynStatus enableAmplifier(int);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus stopAxisInternal(const char *function_name, double acceleration);
  asynStatus resetAxis(void);
  // ECMC specific
  //asynStatus connectEcmcAxis();
  //asynStatus readStatusBin();        // Read binary status data over int8array interface
  //asynStatus readAll();
  //asynStatus readControlBin();       // Read binary control data over int8array interface
  //asynStatus writeControlBin(ecmcAsynClinetCmdType controlWd);
  asynStatus printDiagBinData();
  asynStatus setEnable(int on);
  // Temporary convert betwwen differnt structure types.. Remove later
  asynStatus uglyConvertFunc(ecmcAxisStatusType*in ,st_axis_status_type *out);
/*  asynUser *asynUserStatBin_;      // "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnosticbin?"  
  asynUser *asynUserStatBinIntr_;  // "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnosticbin?"  
  asynUser *asynUserCntrlBin_;     // "T_SMP_MS=%d/TYPE=asynInt32/ax%d.controlstruct="
  asynUser *asynUserCntrlBinIntr_; // "T_SMP_MS=%d/TYPE=asynInt32/ax%d.controlstruct?"
  
  ecmcAsynClinetCmdType   controlBinData_;
  ecmcAsynClinetCmdType   controlBinDataRB_;*/
  int       axisId_;
  ecmcAxisStatusType      statusBinData_;
  
  /*asynInterface          *pasynIFStatBinIntr_;
  asynInt8Array          *pIFStatBinIntr_;
  void                   *interruptStatBinPvt_;
  
  asynInterface          *pasynIFContBinIntr_;
  asynInt8Array          *pIFContBinIntr_;
  void                   *interruptContBinPvt_;*/
  ecmcAxisBase           *ecmcAxis_;
  double                  oldPositionAct_;  // needed for uglyConvertFunc().. 
  double                  manualVelocSlow_;
  double                  manualVelocFast_;
  // ECMC end
  friend class ecmcMotorRecordController;
};

#endif
