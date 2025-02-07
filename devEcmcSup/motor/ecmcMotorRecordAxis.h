/*
FILENAME...   ecmcMotorRecordAxis.h
*/

#ifndef ECMC_MOTOR_RECORD_AXIS_H
#define ECMC_MOTOR_RECORD_AXIS_H

#include "asynMotorAxis.h"
#include <stdint.h>
#include "ecmcAxisBase.h"


#define AMPLIFIER_ON_FLAG_CREATE_AXIS  (1)
#define AMPLIFIER_ON_FLAG_AUTO_ON      (1 << 1)
#define AMPLIFIER_ON_FLAG_USING_CNEN   (1 << 2)

#define ECMCAMPLIFIER_ON_FLAG_USING_CNEN   (1 << 2)

extern const char *modNamEMC;

extern "C" {
int ecmcMotorRecordCreateAxis(const char *ecmcMotorRecordName,
                              int         axisNo,
                              int         axisFlags,
                              const char *axisOptionsStr);
double ecmcMotorRecordgetNowTimeSecs(void);
}

class epicsShareClass ecmcMotorRecordAxis : public asynMotorAxis {
public:
  /* These are the methods we override from the base class */
  ecmcMotorRecordAxis(class ecmcMotorRecordController *pC,
                      int                              axisNo,
                      ecmcAxisBase                    *ecmcAxisRef,
                      int                              axisFlags,
                      const char                      *axisOptionsStr);

  void       report(FILE *fp,
                    int   level);
  void       callParamCallbacksUpdateError();
  asynStatus moveVelocity(double min_velocity,
                          double max_velocity,
                          double acceleration);
  asynStatus stop(double acceleration);
  asynStatus move(double position,
                  int    relative,
                  double min_velocity,
                  double max_velocity,
                  double acceleration);
  asynStatus home(double min_velocity,
                  double max_velocity,
                  double acceleration,
                  int    forwards);
  asynStatus setPosition(double value);
  asynStatus setHighLimit(double highLimit);
  asynStatus setLowLimit(double lowLimit);

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
    ecmcAxisStatusType statusBinDataOld;
    ecmcAxisStatusType statusBinData;
    ecmcAxisBase      *ecmcAxis;
    double             manualVelocSlow;
    double             manualVelocFast;
    int                axisFlags;
    int                nErrorIdMcu;     /* nErrorID from MCU */
    int                nErrorIdMcuOld; /* old nErrorID from MCU */
    int                nErrorIdEpicsOld; /* old nErrorID from MCU */
    int                bErrorOld;   /* copy of bError */
    int                nCommandActive;
    int                nCommandActiveOld;
    int                homed;
    int                axisId;
    unsigned int       waitNumPollsBeforeReady;
    unsigned int       illegalInTargetWindow : 1;
    eeAxisWarningType  old_eeAxisWarning;
    eeAxisWarningType  eeAxisWarning;
    eeAxisErrorType    old_eeAxisError;
    eeAxisErrorType    eeAxisError;
    eeAxisPollNowType  eeAxisPollNow;

    /* Which values have changed in the EPICS IOC, but are not updated in the
       motion controller */
    struct {
      unsigned int statusDisconnectedOld : 1;
      unsigned int sErrorMessage         : 1;     /* From MCU */
      unsigned int initialPollNeeded     : 1;
    }    dirty;
    int  moveReady;
    int  moveReadyOld;
    char cmdErrorMessage[80];               /* From driver */
    char sErrorMessage[80];               /* From controller */
    bool ecmcBusy;
    bool ecmcSafetyInterlock;
    bool ecmcSummaryInterlock;
    int  ecmcTrjSrc;
    bool ecmcAtTarget;
    bool ecmcAtTargetMonEnable;
  } drvlocal;

  //int       restorePowerOnOffNeeded_;
  int        triggstop_;
  int        triggsync_;
  asynStatus readBackAllConfig(int axisID);
  asynStatus updateCfgValue(int         function,
                            int         newValue,
                            const char *name);
  asynStatus updateCfgValue(int         function,
                            double      newValue,
                            const char *name);
  asynStatus readBackSoftLimits(void);
  asynStatus readScaling(int axisID);
  asynStatus readMonitoring(int axisID);
  asynStatus readBackVelocities(int axisID);
  asynStatus initialPoll(void);
  asynStatus initialPollInternal(void);
  asynStatus setIntegerParam(int function,
                             int value);
  asynStatus setDoubleParam(int    function,
                            double value);
  asynStatus enableAmplifier(int);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus stopAxisInternal(const char *function_name,
                              double      acceleration);
  asynStatus resetAxis(void);
  asynStatus printDiagBinData();
  asynStatus setEnable(int on);
  asynStatus readEcmcAxisStatusData();
  bool       pollPowerIsOn(void);
#ifndef motorMessageTextString
  void       updateMsgTxtFromDriver(const char *value);
#endif // ifndef motorMessageTextString

  friend class ecmcMotorRecordController;
};

#endif // ifndef ECMC_MOTOR_RECORD_AXIS_H
