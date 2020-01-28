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
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus stop(double acceleration);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus setPosition(double value);
  asynStatus poll(bool *moving);
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

  ecmcMotorRecordController *pC_; /**< Pointer to the asynMotorController to which this axis belongs.
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

  asynStatus readBackAllConfig(int axisID);
  asynStatus updateCfgValue(int function, int    newValue, const char *name);
  asynStatus updateCfgValue(int function, double newValue, const char *name);
  asynStatus readBackSoftLimits(void);
  asynStatus readScaling(int axisID);
  asynStatus readMonitoring(int axisID);
  asynStatus readBackVelocities(int axisID);
  asynStatus initialPoll(void);
  asynStatus initialPollInternal(void);
  asynStatus setIntegerParam(int function, int value);
  asynStatus setDoubleParam(int function, double value);
#ifndef motorMessageTextString
  void updateMsgTxtFromDriver(const char *value);
#endif
  asynStatus enableAmplifier(int);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus stopAxisInternal(const char *function_name, double acceleration);
  asynStatus resetAxis(void);
  asynStatus printDiagBinData();
  asynStatus setEnable(int on);
  // Temporary convert betwwen differnt structure types.. Remove later
  asynStatus uglyConvertFunc(ecmcAxisStatusType*in ,st_axis_status_type *out);
  int       axisId_;
  ecmcAxisStatusType      statusBinData_;
  ecmcAxisBase           *ecmcAxis_;
  double                  oldPositionAct_;  // needed for uglyConvertFunc().. 
  double                  manualVelocSlow_;
  double                  manualVelocFast_;
  // ECMC end
  friend class ecmcMotorRecordController;
};

#endif
