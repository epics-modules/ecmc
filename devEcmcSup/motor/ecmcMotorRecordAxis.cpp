/*
  FILENAME... ecmcMotorRecordAxis.cpp
*/
/*
st_axis_status.bEnable              = statusBinData_.onChangeData.statusWd.enable;
  st_axis_status.bExecute             = statusBinData_.onChangeData.statusWd.execute;
  st_axis_status.nCommand             = statusBinData_.onChangeData.command;
  st_axis_status.nCmdData             = statusBinData_.onChangeData.cmdData;
  st_axis_status.fVelocity            = statusBinData_.onChangeData.velocitySetpoint;
  st_axis_status.fPosition            = statusBinData_.onChangeData.positionTarget;
  st_axis_status.fAcceleration        = statusBinData_.acceleration;
  st_axis_status.fDecceleration       = statusBinData_.deceleration;  
  st_axis_status.bHomeSensor          = statusBinData_.onChangeData.statusWd.homeswitch;
  st_axis_status.bEnabled             = statusBinData_.onChangeData.statusWd.enabled;
  st_axis_status.bError               = statusBinData_.onChangeData.error>0;
  st_axis_status.nErrorId             = statusBinData_.onChangeData.error;
  st_axis_status.fActVelocity         = statusBinData_.onChangeData.velocityActual;
  st_axis_status.fActPosition         = statusBinData_.onChangeData.positionActual;
  st_axis_status.fActDiff             = statusBinData_.onChangeData.positionError;
  st_axis_status.bHomed               = statusBinData_.onChangeData.statusWd.homed;
  st_axis_status.bBusy                = statusBinData_.onChangeData.statusWd.busy;
  st_axis_status.encoderRaw           = statusBinData_.onChangeData.positionRaw;
  st_axis_status.atTarget             = statusBinData_.onChangeData.statusWd.attarget;
  st_axis_status.bLimitBwd            = statusBinData_.onChangeData.statusWd.limitbwd;
  st_axis_status.bLimitFwd            = statusBinData_.onChangeData.statusWd.limitfwd;
  
  // Data derveid from struct
  st_axis_status.mvnNRdyNex           = statusBinData_.onChangeData.statusWd.busy || !statusBinData_.onChangeData.statusWd.attarget;
  st_axis_status.motorStatusDirection = statusBinData_.onChangeData.positionActual > 
                                              oldPositionAct_ ? 1:0;
  st_axis_status.motorDiffPostion     = 1; //Always set to 1?? why 
  
  //TODO not accesible for ecmc.. neeeded? Delete later
  st_axis_status.bJogFwd              = 0;
  st_axis_status.bJogBwd              = 0;
  st_axis_status.bReset               = 0;
  st_axis_status.fOverride            = 100;
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>

#include <epicsThread.h>

#include "motor.h"
#include "ecmcMotorRecordAxis.h"
#include "ecmcMotorRecordController.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

#define HOMPROC_MANUAL_SETPOS    15

/* The maximum number of polls we wait for the motor
   to "start" (report moving after a new move command */
#define WAITNUMPOLLSBEFOREREADY 3

static ecmcMotorRecordController *pC;

/** Creates a new ecmcMotorRecordAxis object.
 * \param[in] pC Pointer to the ecmcMotorRecordController to which this axis belongs.
 * \param[in] axisNo Index number of this axis, range 1 to pC->numAxes_. (0 is not used)
 *
 *
 * Initializes register numbers, etc.
 */
ecmcMotorRecordAxis::ecmcMotorRecordAxis(ecmcMotorRecordController *pC,
                                         int axisNo,
                                         ecmcAxisBase *ecmcAxisRef,
                                         int axisFlags,
                                         const char *axisOptionsStr)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  ecmcAxis_ = ecmcAxisRef;
  if(!ecmcAxis_) {
        LOGERR(
      "%s/%s:%d: ERROR: Axis ref NULL. Application exits...\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

    exit(EXIT_FAILURE);
  }

  manualVelocSlow_ = 0;
  manualVelocFast_ = 0;
  oldPositionAct_  = 0;
  axisId_          = axisNo;
  //ECMC End
  
  int powerAutoOnOff = -1; /* undefined */
  /* Some parameters are only defined in the ESS fork of the motor module.
     So they have the ifdef */
#ifdef motorFlagsDriverUsesEGUString
  setIntegerParam(pC_->motorFlagsDriverUsesEGU_,1);
#endif
#ifdef motorFlagsAdjAfterHomedString
  setIntegerParam(pC_->motorFlagsAdjAfterHomed_, 1);
#endif
#ifdef motorWaitPollsBeforeReadyString
  setIntegerParam(pC_->motorWaitPollsBeforeReady_ , WAITNUMPOLLSBEFOREREADY);
#endif
  memset(&drvlocal, 0, sizeof(drvlocal));
  memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
  /* Force a printout of this 3 variables, once the controller is connected,
     they have 0 or 1 (but never -1) */
  drvlocal.old_st_axis_status.bHomed = -1;
  drvlocal.old_st_axis_status.bLimitBwd = -1;
  drvlocal.old_st_axis_status.bLimitFwd = -1;

  drvlocal.old_eeAxisError = eeAxisErrorIOCcomError;

  /* We pretend to have an encoder (fActPosition) */
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);
#ifdef motorFlagsNoStopProblemString
  setIntegerParam(pC_->motorFlagsNoStopProblem_, 1);
#endif
#ifdef motorFlagsNoStopOnLsString
  setIntegerParam(pC_->motorFlagsNoStopOnLS_, 1);
#endif
#ifdef motorFlagsLSrampDownString
  setIntegerParam(pC_->motorFlagsLSrampDown_, 1);
#endif
#ifdef motorFlagsPwrWaitForOnString
  setIntegerParam(pC_->motorFlagsPwrWaitForOn_, 1);
#endif
#ifdef motorShowPowerOffString
    setIntegerParam(pC_->motorShowPowerOff_, 1);
#endif
#ifdef  motorNotHomedProblemString
    setIntegerParam(pC_->motorNotHomedProblem_, MOTORNOTHOMEDPROBLEM_ERROR);
#endif

  drvlocal.scaleFactor = 1.0;
  if (axisOptionsStr && axisOptionsStr[0]) {
    const char * const encoder_is_str      = "encoder=";    
#ifndef motorFlagsDriverUsesEGUString
    /* The non-ESS motor needs a dummy "stepm-size" to compensate for MRES */
    const char * const stepSize_str        = "stepSize=";
#endif
    const char * const homProc_str 	       = "HomProc=";
    const char * const homPos_str  	       = "HomPos=";
    const char * const adsPort_str         = "adsPort=";
    const char * const axisFlags_str       = "axisFlags=";
    const char * const powerAutoOnOff_str  = "powerAutoOnOff=";
    const char * const powerOffDelay_str   = "powerOffDelay=";
    const char * const powerOnDelay_str    = "powerOnDelay=";
    const char * const scaleFactor_str     = "scaleFactor=";

    char *pOptions = strdup(axisOptionsStr);
    char *pThisOption = pOptions;
    char *pNextOption = pOptions;

    while (pNextOption && pNextOption[0]) {
      pNextOption = strchr(pNextOption, ';');
      if (pNextOption) {
        *pNextOption = '\0'; /* Terminate */
        pNextOption++;       /* Jump to (possible) next */
      }
      if (!strncmp(pThisOption, encoder_is_str, strlen(encoder_is_str))) {
        pThisOption += strlen(encoder_is_str);
        drvlocal.externalEncoderStr = strdup(pThisOption);
#ifndef motorFlagsDriverUsesEGUString
      } else if (!strncmp(pThisOption, stepSize_str, strlen(stepSize_str))) {
        pThisOption += strlen(stepSize_str);
        /* This option is obsolete, depending on motor */
        drvlocal.scaleFactor = atof(pThisOption);
#endif
      } else if (!strncmp(pThisOption, adsPort_str, strlen(adsPort_str))) {
        pThisOption += strlen(adsPort_str);
        int adsPort = atoi(pThisOption);
        if (adsPort > 0) {
          drvlocal.adsPort = (unsigned)adsPort;
        }
      } else if (!strncmp(pThisOption, axisFlags_str, strlen(axisFlags_str))) {
        pThisOption += strlen(axisFlags_str);
        int myAxisFlags = atoi(pThisOption);
        if (myAxisFlags > 0) {
          axisFlags = myAxisFlags;
        }
      } else if (!strncmp(pThisOption, powerAutoOnOff_str, strlen(powerAutoOnOff_str))) {
        pThisOption += strlen(powerAutoOnOff_str);
	      powerAutoOnOff = atoi(pThisOption);
      } else if (!strncmp(pThisOption, homProc_str, strlen(homProc_str))) {
        pThisOption += strlen(homProc_str);
        int homProc = atoi(pThisOption);
        setIntegerParam(pC_->ecmcMotorRecordHomProc_, homProc);
      } else if (!strncmp(pThisOption, homPos_str, strlen(homPos_str))) {
        pThisOption += strlen(homPos_str);
        double homPos = atof(pThisOption);
        setDoubleParam(pC_->ecmcMotorRecordHomPos_, homPos);
      } else if (!strncmp(pThisOption, scaleFactor_str, strlen(scaleFactor_str))) {
        pThisOption += strlen(scaleFactor_str);
        drvlocal.scaleFactor = atof(pThisOption);
      } else if (!strncmp(pThisOption, powerOffDelay_str, strlen(powerOffDelay_str))) {
        double powerOffDelay;
        pThisOption += strlen(powerOffDelay_str);
        powerOffDelay = atof(pThisOption);
        updateCfgValue(pC_->motorPowerOffDelay_, powerOffDelay, "powerOffDelay");
      } else if (!strncmp(pThisOption, powerOnDelay_str, strlen(powerOnDelay_str))) {
        double powerOnDelay;
        pThisOption += strlen(powerOnDelay_str);
        powerOnDelay = atof(pThisOption);
        updateCfgValue(pC_->motorPowerOnDelay_, powerOnDelay, "powerOnDelay");
      }
      pThisOption = pNextOption;
    }
    free(pOptions);
  }
  drvlocal.axisFlags = axisFlags;
  if (axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    setIntegerParam(pC->motorStatusGainSupport_, 1);
  }
  if (powerAutoOnOff >= 0) {
    /* The new handling using options */
    setIntegerParam(pC_->motorPowerAutoOnOff_, powerAutoOnOff);
    /* the delays had been set up above */
  } else if (axisFlags & AMPLIFIER_ON_FLAG_AUTO_ON) {
    /* old, legacy, to support old start scripts where flags == 6 are used */
#ifdef POWERAUTOONOFFMODE2
    setIntegerParam(pC_->motorPowerAutoOnOff_, POWERAUTOONOFFMODE2);
    setDoubleParam(pC_->motorPowerOnDelay_,   6.0);
    setDoubleParam(pC_->motorPowerOffDelay_, -1.0);
#endif
  }
  /* Set the module name to "" if we have FILE/LINE enabled by asyn */
  if (pasynTrace->getTraceInfoMask(pC_->pasynUserController_) & ASYN_TRACEINFO_SOURCE) modNamEMC = "";
  
  initialPoll();
}

extern "C" int ecmcMotorRecordCreateAxis(const char *ecmcMotorRecordName, 
                                         int axisNo,
                                         int axisFlags,
                                         const char *axisOptionsStr)
{
  pC = (ecmcMotorRecordController*) findAsynPortDriver(ecmcMotorRecordName);
  if (!pC) {
    printf("Error port %s not found\n", ecmcMotorRecordName);
    return asynError;
  }

  if (axisNo >= ECMC_MAX_AXES || axisNo <= 0) {
    printf("ERROR: Axis index out of range.\n");
    return asynError;
  }
  if (axes[axisNo] == NULL) {
    printf("ERROR: Axis object NULL\n");
    return asynError;
  }

  pC->lock();
  new ecmcMotorRecordAxis(pC, axisNo, axes[axisNo], axisFlags, axisOptionsStr);
  pC->unlock();
  return asynSuccess;
  
}

asynStatus ecmcMotorRecordAxis::updateCfgValue(int function,
                                          double newValue,
                                          const char *name)
{
  double oldValue;
  asynStatus status = pC_->getDoubleParam(axisNo_, function, &oldValue);
  if (status) {
    /* First time that we write the value after IOC restart
       ECMC configures everything from the iocshell, no need to
       do a print here */
    if (!(pC_->features_ & FEATURE_BITS_ECMC)) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%supdateCfgValue(%d) %s=%f\n",
                modNamEMC, axisNo_, name, newValue);
    }
  } else if (newValue != oldValue) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%supdateCfgValue(%d) old%s=%f new%s=%f\n",
              modNamEMC, axisNo_, name, oldValue, name, newValue);
  }
  return pC_->setDoubleParam(axisNo_, function, newValue);
}

asynStatus ecmcMotorRecordAxis::updateCfgValue(int function,
                                          int newValue,
                                          const char *name)
{
  int oldValue;
  asynStatus status = pC_->getIntegerParam(axisNo_, function, &oldValue);
  if (status) {
    /* First time that we write the value after IOC restart
       ECMC configures everything from the iocshell, no need to
       do a print here */
    if (!((pC_->features_ & FEATURE_BITS_ECMC))) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%supdateCfgValue(%d) %s=%d\n",
                modNamEMC, axisNo_, name, newValue);
    }
  } else if (newValue != oldValue) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%supdateCfgValue(%d) old%s=%d new%s=%d\n",
              modNamEMC, axisNo_, name, oldValue, name, newValue);
  }
  return pC_->setIntegerParam(axisNo_, function, newValue);
}

asynStatus ecmcMotorRecordAxis::readBackSoftLimits(void)
{
  printf("Dbg################# %s\n",__FUNCTION__);

  int    enabledFwd = 0,  enabledBwd = 0;
  double fValueFwd = 0.0, fValueBwd  = 0.0;
  double scaleFactor = drvlocal.scaleFactor;

  fValueBwd  = ecmcAxis_->getMon()->getSoftLimitBwd();
  fValueFwd  = ecmcAxis_->getMon()->getSoftLimitFwd();
  enabledBwd = ecmcAxis_->getMon()->getEnableSoftLimitBwd();
  enabledFwd = ecmcAxis_->getMon()->getEnableSoftLimitFwd();

  pC_->setIntegerParam(axisNo_, pC_->ecmcMotorRecordCfgDLLM_En_, enabledBwd);
  pC_->setDoubleParam(axisNo_, pC_->ecmcMotorRecordCfgDLLM_, fValueBwd);
  pC_->setIntegerParam(axisNo_, pC_->ecmcMotorRecordCfgDHLM_En_, enabledFwd);
  pC_->setDoubleParam(axisNo_, pC_->ecmcMotorRecordCfgDHLM_, fValueFwd);
  
  if (scaleFactor) {
    pC_->udateMotorLimitsRO(axisNo_, enabledBwd && enabledFwd,
                            fValueFwd / scaleFactor, 
                            fValueBwd / scaleFactor);
  }
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::readScaling(int axisID)
{

  int errorCode =0;
  double num = 0, denom = 0;

  errorCode=ecmcAxis_->getEncScaleNum(&num);
  if(errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: function getEncScaleNum() returned error (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode);
    return asynError;
  }
  errorCode=ecmcAxis_->getEncScaleDenom(&denom);
  if(errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: function getEncScaleDenom() returned error (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode);
    return asynError;
  }

  if(!denom) {
    LOGERR(
      "%s/%s:%d: ERROR: Encoder denominator scale zero.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

    return asynError;
  }

  if(!num) {
    LOGERR(
      "%s/%s:%d: ERROR: Encoder numerator scale zero.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

    return asynError;
  }

  // Why is not this scale needed/updated? Always 1?
  // drvlocal.scaleFactor = num / denom;

  updateCfgValue(pC_->ecmcMotorRecordCfgSREV_RB_, denom, "srev");
  updateCfgValue(pC_->ecmcMotorRecordCfgUREV_RB_, num, "urev");
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::readMonitoring(int axisID)
{
  double poslag_tol, attarget_tol;
  int    poslag_time, attarget_time, poslag_enable, attarget_enable;

  // Position lag monitoring (following error)
  poslag_tol = ecmcAxis_->getMon()->getPosLagTol();
  poslag_time = ecmcAxis_->getMon()->getPosLagTime() * 1 / (double)MCU_FREQUENCY;
  poslag_enable = ecmcAxis_->getMon()->getEnableLagMon();

  // At target monitoring (must be enabled)
  attarget_tol = ecmcAxis_->getMon()->getAtTargetTol();
  attarget_time = ecmcAxis_->getMon()->getAtTargetTime() * 1 / (double)MCU_FREQUENCY;
  attarget_enable = ecmcAxis_->getMon()->getEnableAtTargetMon();
   
  // At target monitoring must be enabled
  drvlocal.illegalInTargetWindow = (!attarget_enable || !attarget_tol);

  // (Target position monitoring value)
  updateCfgValue(pC_->ecmcMotorRecordCfgSPDB_RB_,  attarget_tol, "spbd"); 
  // (Target position monitoring value)
  updateCfgValue(pC_->ecmcMotorRecordCfgRDBD_RB_, attarget_tol, "rdbd");
  // (Target position monitoring time)
  updateCfgValue(pC_->ecmcMotorRecordCfgRDBD_Tim_RB_, attarget_time , "rdbd_time");
  // (Target position monitoring enable)
  updateCfgValue(pC_->ecmcMotorRecordCfgRDBD_En_RB_, attarget_enable, "rdbd_en");
  // (Maximum position lag value)
  updateCfgValue(pC_->ecmcMotorRecordCfgPOSLAG_RB_, poslag_tol, "poslag");
  // (Maximum position lag time)
  updateCfgValue(pC_->ecmcMotorRecordCfgPOSLAG_Tim_RB_, poslag_time, "poslag_tim");
  // (Maximum position lag enable)
  updateCfgValue(pC_->ecmcMotorRecordCfgPOSLAG_En_RB_, poslag_enable, "poslag_en");
  
  return asynSuccess;
}


asynStatus ecmcMotorRecordAxis::readBackVelocities(int axisID)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  
  double scaleFactor = drvlocal.scaleFactor;
  double vel_max, acceleration;

  vel_max = ecmcAxis_->getMon()->getMaxVel();
  acceleration = ecmcAxis_->getTraj()->getAcc();

  if (manualVelocFast_ > 0.0) {
    updateCfgValue(pC_->ecmcMotorRecordCfgVELO_, manualVelocFast_ / scaleFactor, "velo");
  }
  if (vel_max > 0.0) {
    updateCfgValue(pC_->ecmcMotorRecordCfgVMAX_, vel_max / scaleFactor, "vmax");
  }
  if (manualVelocSlow_ > 0.0) {
    updateCfgValue(pC_->ecmcMotorRecordCfgJVEL_, manualVelocSlow_ / scaleFactor, "jvel");
  }
  if (acceleration > 0.0) {
    updateCfgValue(pC_->ecmcMotorRecordCfgACCS_, acceleration / scaleFactor, "accs");
  }
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::initialPoll(void)
{
  asynStatus status;

  if (!drvlocal.dirty.initialPollNeeded)
    return asynSuccess;

  status = initialPollInternal();
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sinitialPoll(%d) status=%d\n",
            modNamEMC, axisNo_, status);
  if (status == asynSuccess) drvlocal.dirty.initialPollNeeded = 0;
  return status;
}


asynStatus ecmcMotorRecordAxis::readBackAllConfig(int axisID)
{
  asynStatus status = asynSuccess;
  if (status == asynSuccess) status = readScaling(axisID);
  if (status == asynSuccess) status = readMonitoring(axisID);
  if (status == asynSuccess) status = readBackSoftLimits();
  if (status == asynSuccess) status = readBackVelocities(axisID);
  return status;
}

/** Connection status is changed, the dirty bits must be set and
 *  the values in the controller must be updated
 * \param[in] AsynStatus status
 *
 * Sets the dirty bits
 */
asynStatus ecmcMotorRecordAxis::initialPollInternal(void)
{
  asynStatus status = asynSuccess;

  if (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_CREATE_AXIS) {
    /* Enable the amplifier when the axis is created,
       but wait until we have a connection to the controller.
       After we lost the connection, Re-enable the amplifier
       See AMPLIFIER_ON_FLAG */
    status = enableAmplifier(1);
  }
  if (status == asynSuccess) status = readBackAllConfig(axisId_);
  if (status == asynSuccess && drvlocal.dirty.oldStatusDisconnected) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%sconnected(%d)\n", modNamEMC, axisNo_);
    drvlocal.dirty.oldStatusDisconnected = 0;
  }
  return status;
}

/** Reports on status of the axis
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * After printing device-specific information calls asynMotorAxis::report()
 */
void ecmcMotorRecordAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

/** Move the axis to a position, either absolute or relative
 * \param[in] position in steps
 * \param[in] relative (0=absolute, otherwise relative)
 * \param[in] minimum velocity, steps/sec
 * \param[in] maximum velocity, steps/sec
 * \param[in] acceleration,  steps/sec/sec
 *
 */
asynStatus ecmcMotorRecordAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
            "%smove(%d) position=%f relative=%d maxVelocity=%f acceleration=%f\n",
            modNamEMC, axisNo_,
            position, relative, maxVelocity, acceleration);

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;

  /* Do range check */
  if (!drvlocal.scaleFactor) {
    drvlocal.eeAxisWarning = eeAxisWarningCfgZero;
    return asynSuccess;
  } else if (!maxVelocity) {
    drvlocal.eeAxisWarning = eeAxisWarningVeloZero;
    return asynSuccess;
  }
  
  int errorCode = 0;
  if(relative) {
    errorCode = ecmcAxis_->moveRelativePosition(position * drvlocal.scaleFactor,
                                                maxVelocity * drvlocal.scaleFactor,
                                                acceleration * drvlocal.scaleFactor,
                                                acceleration * drvlocal.scaleFactor);
  }
  else
  {
    errorCode = ecmcAxis_->moveAbsolutePosition(position * drvlocal.scaleFactor,
                                                maxVelocity * drvlocal.scaleFactor,
                                                acceleration * drvlocal.scaleFactor,
                                                acceleration * drvlocal.scaleFactor);
    
  }  
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return errorCode == 0 ? asynSuccess:asynError;
}


/** Home the motor, search the home position
 * \param[in] minimum velocity, mm/sec
 * \param[in] maximum velocity, mm/sec
 * \param[in] acceleration, seconds to maximum velocity
 * \param[in] forwards (0=backwards, otherwise forwards)
 *
 */
asynStatus ecmcMotorRecordAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  // cmd, nCmddata,homepos,velhigh,vellow,acc
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
            "%shome(%d) forwards=%d maxVelocity=%f acceleration=%f\n",
            modNamEMC, axisNo_,
            forwards, maxVelocity, acceleration);

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;

  // Read from records / params
  double homPos    = 0.0; /* The homPos may be undefined, then use 0.0 */
  int cmdData      = -1;
  double velToCam  = 0;
  double velOffCam = 0;
  double accHom    = 0;
   
  // nCmdData (sequence number)
  asynStatus status = pC_->getIntegerParam(axisNo_, pC_->ecmcMotorRecordHomProc_,&cmdData);
  if (cmdData == HOMPROC_MANUAL_SETPOS || status != asynSuccess) {
    return asynError;
  }
  
  // Home position
  (void)pC_->getDoubleParam(axisNo_, pC_->ecmcMotorRecordHomPos_, &homPos);
  
  // Velocity to cam (high velo)
  status = pC_->getDoubleParam(axisNo_,
                               pC_->ecmcMotorRecordVelToHom_,
                               &velToCam);
  if (status != asynSuccess) {
    return asynError;
  }

  // Velocity off cam (low velo)
  status = pC_->getDoubleParam(axisNo_,
                               pC_->ecmcMotorRecordVelFrmHom_,
                               &velOffCam);
  if (status != asynSuccess) {
    return asynError;
  }
  
  // Acceleration
  status = pC_->getDoubleParam(axisNo_,
                               pC_->ecmcMotorRecordAccHom_,
                               &accHom);
  if (status != asynSuccess) {
    return asynError;
  }
  
  int errorCode =  ecmcAxis_->moveHome(cmdData,homPos,velToCam,velOffCam,accHom,accHom);
  
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return errorCode == 0 ? asynSuccess:asynError;
}


/** jog the the motor, search the home position
 * \param[in] minimum velocity, mm/sec (not used)
 * \param[in] maximum velocity, mm/sec (positive or negative)
 * \param[in] acceleration, seconds to maximum velocity
 *
 */
asynStatus ecmcMotorRecordAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  /* Do range check */
  if (!drvlocal.scaleFactor) {
    drvlocal.eeAxisWarning = eeAxisWarningCfgZero;
    return asynSuccess;
  } else if (!maxVelocity) {
    drvlocal.eeAxisWarning = eeAxisWarningVeloZero;
    return asynSuccess;
  }
  
  if(acceleration==0) {
    LOGERR(
      "%s/%s:%d: ERROR: Acceleration setpoint 0.0. Command aborted..\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }

  // Velocity
  double velo = maxVelocity * drvlocal.scaleFactor;

  // Acc m/sec2
  double acc = 0.0;

  if (acceleration != 0) {
    double acc_in_seconds = maxVelocity / acceleration;
    acc = velo / acc_in_seconds;
  }

  // check sign
  if (acc  < 0) {
    acc = -acc;
  }

  int errorCode = ecmcAxis_->moveVelocity(velo,
                                          acc,
                                          acc);

  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return errorCode == 0 ? asynSuccess:asynError;
}

/**
 * See asynMotorAxis::setPosition
 */
asynStatus ecmcMotorRecordAxis::setPosition(double value)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
            "%ssetPosition(%d) value=%lf\n",
            modNamEMC, axisNo_, value);

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  
  // Read from records / params
  double homPos    = 0.0; /* The homPos may be undefined, then use 0.0 */
  
  //int    cmdData   = -1; 
  // nCmdData (sequence number) Must be "manual cmddata (=15)"
  // asynStatus status = pC_->getIntegerParam(axisNo_, pC_->ecmcMotorRecordHomProc_,&cmdData);
  // if (cmdData != HOMPROC_MANUAL_SETPOS || status != asynSuccess) {
  //   return asynError;
  // }

  // Home position
  (void)pC_->getDoubleParam(axisNo_, pC_->ecmcMotorRecordHomPos_, &homPos);
  
  // always use home sequence 15 for setPosition
  int errorCode =  ecmcAxis_->moveHome(HOMPROC_MANUAL_SETPOS,
                                       homPos,0,0,0,0);
  
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return errorCode == 0 ? asynSuccess:asynError;
}

asynStatus ecmcMotorRecordAxis::resetAxis(void)
{
  ecmcAxis_->errorReset();
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::setEnable(int on) {
  int errorCode = ecmcAxis_->setEnable(on);
  if(errorCode){
    LOGERR(
      "%s/%s:%d: ERROR: Function setEnable(%d) returned errorCode (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      on,
      errorCode);

    return asynError;
  }
  
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return asynSuccess;
}

/** 
 *  Enable the amplifier on an axis
 */
asynStatus ecmcMotorRecordAxis::enableAmplifier(int on)
{
  asynStatus status = asynSuccess;
  unsigned counter = 10;

//  const char *enableEnabledReadback = "bEnabled";
// #ifdef POWERAUTOONOFFMODE2
//   {
//     int autoPower;
//     pC_->getIntegerParam(axisNo_, pC_->motorPowerAutoOnOff_, &autoPower);
//     if (autoPower) {
//       /* The record/driver will check for enabled - don't do that here */
//       enableEnabledReadback = "bEnable";
//     }
//   }
// #endif
  
  on = on ? 1 : 0; /* either 0 or 1 */
  
  if(statusBinData_.onChangeData.statusWd.enabled == on && 
     statusBinData_.onChangeData.statusWd.enable == on) {
    return status;  // status OK
  }

  if (!on) {
    /* Amplifier is on and should be turned off.*/
    status = stopAxisInternal(__FUNCTION__, 0);
    if (status) return status;
  }

  status = setEnable(on);
  if (status) return status;

  while (counter) {
    epicsThreadSleep(.1);
    if (statusBinData_.onChangeData.statusWd.enabled == on && 
        statusBinData_.onChangeData.statusWd.enable == on &&
       !statusBinData_.onChangeData.statusWd.busy) {
      return asynSuccess;
    }
    counter = counter -1;
  }

  /* if we come here, it went wrong */
  if (!drvlocal.cmdErrorMessage[0]) {
    snprintf(drvlocal.cmdErrorMessage, sizeof(drvlocal.cmdErrorMessage)-1,
             "E: enableAmplifier(%d) failed. out=%s in=%s\n",
             axisNo_, pC_->outString_, pC_->inString_);
    /* The poller co-ordinates the writing into the parameter library */
  }
  return asynError;
}

/** 
 * Stop the axis
 */
asynStatus ecmcMotorRecordAxis::stopAxisInternal(const char *function_name, double acceleration)
{ 
  int errorCode = ecmcAxis_->setExecute(0);
  if(errorCode){
    LOGERR(
      "%s/%s:%d: ERROR: Function setExecute(0) returned errorCode (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode);

    return asynError;
  }

  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  
  return asynSuccess;
}

/** 
 * Stop the axis, called by motor Record
 */
asynStatus ecmcMotorRecordAxis::stop(double acceleration )
{
  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  return stopAxisInternal(__FUNCTION__, acceleration);
}

void ecmcMotorRecordAxis::callParamCallbacksUpdateError()
{
  const char *msgTxtFromDriver = NULL;
  int EPICS_nErrorId = drvlocal.MCU_nErrorId;
  drvlocal.eeAxisError = eeAxisErrorNoError;
  if (drvlocal.supported.statusVer == -1) {
    drvlocal.eeAxisError = eeAxisErrorNotFound;
    msgTxtFromDriver = "Not found";
  } else if (EPICS_nErrorId) {
    /* Error from MCU */
    drvlocal.eeAxisError = eeAxisErrorMCUError;
    msgTxtFromDriver = &drvlocal.sErrorMessage[0];
  } else if (drvlocal.dirty.sErrorMessage) {
    /* print error below */
    drvlocal.eeAxisError = eeAxisErrorIOCcomError;
  } else if (drvlocal.cmdErrorMessage[0]) {
    drvlocal.eeAxisError = eeAxisErrorCmdError;
    msgTxtFromDriver = &drvlocal.cmdErrorMessage[0];
  } else if (!drvlocal.homed &&
             (drvlocal.nCommandActive != ECMC_CMD_HOMING)) {
    drvlocal.eeAxisError = eeAxisErrorNotHomed;
  } else if (drvlocal.illegalInTargetWindow) {
    drvlocal.eeAxisError = eeAxisIllegalInTargetWindow;
    msgTxtFromDriver = "E: InTargetPosWin";
  }
  if (drvlocal.eeAxisError != drvlocal.old_eeAxisError ||
      drvlocal.eeAxisWarning != drvlocal.old_eeAxisWarning ||
      drvlocal.old_EPICS_nErrorId != EPICS_nErrorId ||
      drvlocal.old_nCommandActive != drvlocal.nCommandActive) {

    if (!msgTxtFromDriver && drvlocal.eeAxisWarning) {
      /* No error to show yet */
      switch(drvlocal.eeAxisWarning) {
      case eeAxisWarningCfgZero:
        msgTxtFromDriver = "E: scaleFactor is 0.0";
        break;
      case eeAxisWarningVeloZero:
        msgTxtFromDriver = "E: velo is 0.0";
        break;
      case eeAxisWarningSpeedLimit:
        msgTxtFromDriver = "Speed Limit";
        break;
      case eeAxisWarningNoWarning:
        break;
      }
    }
    /* No warning to show yet */
    if (!msgTxtFromDriver) {
      switch(drvlocal.nCommandActive) {
#ifdef motorLatestCommandString
      case ECMC_CMD_MOVEVEL:
        setIntegerParam(pC_->motorLatestCommand_, LATEST_COMMAND_MOVE_VEL);
        break;
      case ECMC_CMD_MOVEREL:
        setIntegerParam(pC_->motorLatestCommand_, LATEST_COMMAND_MOVE_REL);
        break;
      case ECMC_CMD_MOVEABS:
        setIntegerParam(pC_->motorLatestCommand_, LATEST_COMMAND_MOVE_ABS);
        break;
      case ECMC_CMD_HOMING:
        setIntegerParam(pC_->motorLatestCommand_, LATEST_COMMAND_HOMING);
        break;
#endif
      case 0:
        break;
      default:
        msgTxtFromDriver = "Moving";
      }
    }
    /* End of error/warning text messages */

    /* Axis has a problem: Report to motor record */
    /*
     * Site note: Some versions of the motor module needed and
     *  #ifdef motorFlagsNoStopProblemString
     * here. Today these versions are history, and the
     * motorFlagsNoStopProblemString is no longer defined in the
     * motor module. So we need to remove the #ifdef here.
     */
    setIntegerParam(pC_->motorStatusProblem_,
                    drvlocal.eeAxisError != eeAxisErrorNoError);
    /* MCU has a problem: set the red light in CSS */
    setIntegerParam(pC_->ecmcMotorRecordErr_,
                    drvlocal.eeAxisError == eeAxisErrorMCUError);
    setIntegerParam(pC_->ecmcMotorRecordErrId_, EPICS_nErrorId);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) callParamCallbacksUpdateError"
              " Error=%d old=%d ErrID=0x%x old=0x%x Warn=%d nCmd=%d old=%d txt=%s\n",
              modNamEMC, axisNo_, drvlocal.eeAxisError, drvlocal.old_eeAxisError,
              EPICS_nErrorId, drvlocal.old_EPICS_nErrorId,
              drvlocal.eeAxisWarning,
              drvlocal.nCommandActive, drvlocal.old_nCommandActive,
              msgTxtFromDriver ? msgTxtFromDriver : "NULL");

    updateMsgTxtFromDriver(msgTxtFromDriver);
    
    drvlocal.old_eeAxisError = drvlocal.eeAxisError;
    drvlocal.old_eeAxisWarning = drvlocal.eeAxisWarning;
    drvlocal.old_EPICS_nErrorId = EPICS_nErrorId;
    drvlocal.old_nCommandActive = drvlocal.nCommandActive;
  }

  callParamCallbacks();
}

/** Polls the axis.
 * This function reads the motor position, the limit status, the home status, the moving status,
 * and the drive power-on status.
 * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
 * and then calls callParamCallbacks() at the end.
 * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus ecmcMotorRecordAxis::poll(bool *moving)
{
  //printf("Dbg################# 1 %s\n",__FUNCTION__);
  asynStatus comStatus = asynSuccess;
  st_axis_status_type st_axis_status;

  double timeBefore = ecmcMotorRecordgetNowTimeSecs();
  int waitNumPollsBeforeReady_ = drvlocal.waitNumPollsBeforeReady;

  /* Driver not yet initialized, do nothing */
  if (!ecmcAxis_->getRealTimeStarted()) 
    return comStatus;

  // Get values from ecmc
  ecmcAxisStatusType *tempAxisStat = ecmcAxis_->getDebugInfoDataPointer();
  if(!tempAxisStat) {
    LOGERR(
      "%s/%s:%d: ERROR: function getAxisDebugInfoDataPointer() returned NULL.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

    return asynError;
  }
  memcpy(&statusBinData_,tempAxisStat,sizeof(ecmcAxisStatusType));
  memset(&st_axis_status, 0, sizeof(st_axis_status));
  uglyConvertFunc(&statusBinData_,&st_axis_status);
  //printDiagBinData();

  setIntegerParam(pC_->motorStatusHomed_, st_axis_status.bHomed);
  drvlocal.homed = st_axis_status.bHomed;
  setIntegerParam(pC_->motorStatusCommsError_, 0);
  setIntegerParam(pC_->motorStatusAtHome_, st_axis_status.bHomeSensor);
  setIntegerParam(pC_->motorStatusLowLimit_, !st_axis_status.bLimitBwd);
  setIntegerParam(pC_->motorStatusHighLimit_, !st_axis_status.bLimitFwd);
  setIntegerParam(pC_->motorStatusPowerOn_, st_axis_status.bEnabled);
  setDoubleParam(pC_->ecmcMotorRecordVelAct_, st_axis_status.fActVelocity);
  setDoubleParam(pC_->ecmcMotorRecordAcc_RB_, st_axis_status.fAcceleration);

#ifndef motorWaitPollsBeforeReadyString
  if (drvlocal.waitNumPollsBeforeReady) {
    *moving = true;
  }
  else
#endif
    {
      *moving = st_axis_status.mvnNRdyNex ? true : false;
      if (!st_axis_status.mvnNRdyNex &&
          !(pC_->features_ & FEATURE_BITS_ECMC)) {
        /* not moving: poll the parameters for this axis */        
        switch (drvlocal.eeAxisPollNow) {          
        case pollNowReadScaling:
          readScaling(axisId_);          
          break;
        case pollNowReadMonitoring:
          readMonitoring(axisId_);
          drvlocal.eeAxisPollNow = pollNowReadBackSoftLimits;
          break;
        case pollNowReadBackSoftLimits:
          readBackSoftLimits();
          drvlocal.eeAxisPollNow = pollNowReadBackVelocities;
          break;
        case pollNowReadBackVelocities:
        default:          
          readBackVelocities(axisId_);
          drvlocal.eeAxisPollNow = pollNowReadScaling;
          break;
        }
      }
    }

  if (st_axis_status.mvnNRdyNex){
    drvlocal.nCommandActive = st_axis_status.nCommand;
  }
  else {
    drvlocal.nCommandActive = 0;
    if (drvlocal.eeAxisWarning == eeAxisWarningSpeedLimit) {
      drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
    }
  }
  
  if (drvlocal.nCommandActive != ECMC_CMD_HOMING) {
  
    setDoubleParam(pC_->motorPosition_,
                   st_axis_status.fActPosition / drvlocal.scaleFactor);
    setDoubleParam(pC_->motorEncoderPosition_,
                   st_axis_status.fActPosition  / drvlocal.scaleFactor);
    drvlocal.old_st_axis_status.fActPosition = st_axis_status.fActPosition;
    setDoubleParam(pC_->ecmcMotorRecordVel_RB_, st_axis_status.fVelocity);
  }
  setDoubleParam(pC_->ecmcMotorRecordEncAct_, st_axis_status.encoderRaw);

  if (drvlocal.old_st_axis_status.bHomed != st_axis_status.bHomed) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) homed=%d\n",
              modNamEMC, axisNo_, st_axis_status.bHomed);
    drvlocal.old_st_axis_status.bHomed =  st_axis_status.bHomed;
  }
  if (drvlocal.old_st_axis_status.bLimitBwd != st_axis_status.bLimitBwd) {    
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) LLS=%d\n",
              modNamEMC, axisNo_, !st_axis_status.bLimitBwd);
    drvlocal.old_st_axis_status.bLimitBwd =  st_axis_status.bLimitBwd;
  }
  if (drvlocal.old_st_axis_status.bLimitFwd != st_axis_status.bLimitFwd) {    
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) HLS=%d\n",
              modNamEMC, axisNo_,!st_axis_status.bLimitFwd);
    drvlocal.old_st_axis_status.bLimitFwd = st_axis_status.bLimitFwd;
  }

#ifndef motorWaitPollsBeforeReadyString
  if (drvlocal.waitNumPollsBeforeReady) {    
    /* Don't update moving, done, motorStatusProblem_ */
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) mvnNRdyNexAt=%d Ver=%d bBusy=%d bExecute=%d bEnabled=%d atTarget=%d waitNumPollsBeforeReady=%d\n",
              modNamEMC,
              axisNo_, st_axis_status.mvnNRdyNex,
              drvlocal.supported.statusVer,
              st_axis_status.bBusy, st_axis_status.bExecute,
              st_axis_status.bEnabled, st_axis_status.atTarget,
              drvlocal.waitNumPollsBeforeReady);
    drvlocal.waitNumPollsBeforeReady--;
    callParamCallbacks();
  }
  else
#endif
    {
      if (drvlocal.old_st_axis_status.mvnNRdyNex != st_axis_status.mvnNRdyNex ||
          drvlocal.old_st_axis_status.bBusy      != st_axis_status.bBusy ||
          drvlocal.old_st_axis_status.bEnabled   != st_axis_status.bEnabled ||
          drvlocal.old_st_axis_status.bExecute   != st_axis_status.bExecute ||
          drvlocal.old_st_axis_status.atTarget   != st_axis_status.atTarget) {
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "%spoll(%d) mvnNRdy=%d Ver=%d bBusy=%d bExe=%d bEnabled=%d atTarget=%d wf=%d ENC=%g fPos=%g fActPosition=%g time=%f\n",
                  modNamEMC, axisNo_, st_axis_status.mvnNRdyNex,
                  drvlocal.supported.statusVer,
                  st_axis_status.bBusy, st_axis_status.bExecute,
                  st_axis_status.bEnabled, st_axis_status.atTarget,
                  waitNumPollsBeforeReady_,
                  st_axis_status.encoderRaw, st_axis_status.fPosition,
                  st_axis_status.fActPosition,
                  ecmcMotorRecordgetNowTimeSecs() - timeBefore);
      }
    }
  setIntegerParam(pC_->motorStatusDirection_, st_axis_status.motorStatusDirection);
  setIntegerParam(pC_->motorStatusMoving_, st_axis_status.mvnNRdyNex);
  setIntegerParam(pC_->motorStatusDone_, !st_axis_status.mvnNRdyNex);

  drvlocal.MCU_nErrorId = st_axis_status.nErrorId;

  if (drvlocal.old_bError != st_axis_status.bError ||
             drvlocal.old_MCU_nErrorId != drvlocal.MCU_nErrorId ||
             drvlocal.dirty.sErrorMessage) {    
    char sErrorMessage[256];
    int nErrorId = st_axis_status.nErrorId;
    const char *errIdString = errStringFromErrId(nErrorId);
    sErrorMessage[0] = '\0';
    drvlocal.sErrorMessage[0] = '\0';
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) bError=%d st_axis_status.nErrorId=0x%x\n",
              modNamEMC, axisNo_, st_axis_status.bError,
              nErrorId);
    drvlocal.old_bError = st_axis_status.bError;
    drvlocal.old_MCU_nErrorId = nErrorId;
    drvlocal.dirty.sErrorMessage = 0;
    
    if (nErrorId) {
      /* Get the ErrorMessage to have it in the log file */
      strcpy(&sErrorMessage[0],ecmcError::convertErrorIdToString(nErrorId));  
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%ssErrorMessage(%d)=\"%s\"\n",
                modNamEMC, axisNo_, sErrorMessage);
    }
    
    // Write error string to drvlocal.sErrorMessage
    /* First choice: "well known" ErrorIds */
    if (errIdString[0]) {
      snprintf(drvlocal.sErrorMessage, sizeof(drvlocal.sErrorMessage)-1, "E: %s (0x%x)",
               errIdString, nErrorId);
    } else {
      /* ecmc has error messages */
      snprintf(drvlocal.sErrorMessage, sizeof(drvlocal.sErrorMessage)-1, "E: %s (0x%x)",
               sErrorMessage, nErrorId);
    }
    /* The poller will update the MsgTxt field */
    updateMsgTxtFromDriver(drvlocal.sErrorMessage);
  }
  callParamCallbacksUpdateError();

  memcpy(&drvlocal.old_st_axis_status, &st_axis_status,
         sizeof(drvlocal.old_st_axis_status));
  return asynSuccess;
}

/** Set the motor closed loop status
 * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus ecmcMotorRecordAxis::setClosedLoop(bool closedLoop)
{
  int value = closedLoop ? 1 : 0;
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%ssetClosedLoop(%d)=%d\n",  modNamEMC, axisNo_, value);
  if (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    return enableAmplifier(value);
  }
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::setIntegerParam(int function, int value)
{
  asynStatus status;
  int errorCode = 0;
  //unsigned indexGroup5000 = 0x5000;

  if (function == pC_->motorUpdateStatus_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d motorUpdateStatus_)=%d\n", modNamEMC, axisNo_, value);
  } 
  else if (function == pC_->motorStatusCommsError_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
              "%ssetIntegerParam(%d pC_->motorStatusCommsError_)=%d\n",
              modNamEMC, axisNo_, value);
    if (value && !drvlocal.dirty.oldStatusDisconnected) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s Communication error(%d)\n", modNamEMC, axisNo_);
      memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
      drvlocal.MCU_nErrorId = 0;
    }
#ifdef motorPowerAutoOnOffString
  } 
  else if (function == pC_->motorPowerAutoOnOff_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d motorPowerAutoOnOff_)=%d\n", modNamEMC, axisNo_, value);
#endif
  } 
  else if (function == pC_->ecmcMotorRecordHomProc_) {
    int motorNotHomedProblem = 0;
    setIntegerParam(pC_->ecmcMotorRecordHomProc_RB_, value);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d HomProc_)=%d motorNotHomedProblem=%d\n",
              modNamEMC, axisNo_, value, motorNotHomedProblem);
  } 
  else if (function == pC_->ecmcMotorRecordErrRst_) {
    if (value) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%ssetIntegerParam(%d ErrRst_)=%d\n",
                modNamEMC, axisNo_, value);
      /*  We do not want to call the base class */
      return resetAxis();
    }
    /* If someone writes 0 to the field, just ignore it */
    return asynSuccess;

  }
  else if (function == pC_->ecmcMotorRecordCfgDHLM_En_) {
    // Set enable soft limit fwd

    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d ecmcMotorRecordCfgDHLM_En)=%d\n",
              modNamEMC, axisNo_, value);    
    errorCode = ecmcAxis_->getMon()->setEnableSoftLimitFwd(value);
    readBackSoftLimits();
    return errorCode == 0 ? asynSuccess : asynError;

  }
  else if (function == pC_->ecmcMotorRecordCfgDLLM_En_) {
    // Set enable soft limit bwd
    
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d ecmcMotorRecordCfgDLLM_En)=%d\n",
              modNamEMC, axisNo_, value);
    
    errorCode = ecmcAxis_->getMon()->setEnableSoftLimitBwd(value);
    readBackSoftLimits();
    return errorCode==0 ? asynSuccess : asynError;
  }

  //Call base class method
  status = asynMotorAxis::setIntegerParam(function, value);
  return status;
}

/** Set a floating point parameter on the axis
 * \param[in] function, which parameter is updated
 * \param[in] value, the new value
 *
 * When the IOC starts, we will send the soft limits to the controller.
 * When a soft limit is changed, and update is send them to the controller.
 */
asynStatus ecmcMotorRecordAxis::setDoubleParam(int function, double value)
{
  asynStatus status;
  int errorCode = 0;
  //unsigned indexGroup5000 = 0x5000;

  if (function == pC_->motorMoveRel_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorMoveRel_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorMoveAbs_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorMoveAbs_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorMoveVel_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorMoveVel_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorHome_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorHome__)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorStop_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorStop_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorVelocity_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorVelocity_=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorVelBase_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorVelBase_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorAccel_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorAccel_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorDeferMoves_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motmotorDeferMoves_=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorMoveToHome_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motmotorMoveToHome_=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorResolution_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorResolution_=%g\n",  modNamEMC, axisNo_, value);
    /* Limits handled above */

#ifdef motorPowerOnDelayString
  } else if (function == pC_->motorPowerOnDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPowerOnDelay_)=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef motorPowerOffDelayString
  } else if (function == pC_->motorPowerOffDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPowerOffDelay_=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef motorPowerOffFractionString
  } else if (function == pC_->motorPowerOffFraction_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motomotorPowerOffFraction_=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef motorPostMoveDelayString
  } else if (function == pC_->motorPostMoveDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPostMoveDelay_=%g\n", modNamEMC, axisNo_, value);
#endif
  } else if (function == pC_->motorStatus_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorStatus_)=%g\n", modNamEMC, axisNo_, value);
#ifdef ecmcMotorRecordHVELFRMString
  } else if (function == pC_->ecmcMotorRecordHVELfrm_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d HVELfrm_)=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef ecmcMotorRecordHomPosString
  } else if (function == pC_->ecmcMotorRecordHomPos_) {
    pC_->setDoubleParam(axisNo_, pC_->ecmcMotorRecordHomPos_RB_, value);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d HomPos_)=%f\n", modNamEMC, axisNo_, value);
#endif

  } // Set soft limit fwd
  else if (function == pC_->ecmcMotorRecordCfgDHLM_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgDHLM_)=%f\n", modNamEMC, axisNo_, value);
    
    errorCode = ecmcAxis_->getMon()->setSoftLimitFwd(value);
    readBackSoftLimits();
    return errorCode==0 ? asynSuccess : asynError;

  } // Set soft limit bwd
  else if (function == pC_->ecmcMotorRecordCfgDLLM_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgDLLM_)=%f\n", modNamEMC, axisNo_, value);
    
    errorCode = ecmcAxis_->getMon()->setSoftLimitBwd(value);
    readBackSoftLimits();
    return errorCode==0 ? asynSuccess : asynError;

  } // manual velo fast.. Just store here in "motor record" driver
  else if (function == pC_->ecmcMotorRecordCfgVELO_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgVELO_)=%f\n", modNamEMC, axisNo_, value);
  
    manualVelocFast_ = value;
    return asynSuccess;

  } // Set monitor max velocity
  else if (function == pC_->ecmcMotorRecordCfgVMAX_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgVMAX_)=%f\n", modNamEMC, axisNo_, value);
    errorCode = ecmcAxis_->getMon()->setMaxVel(value);
    return errorCode==0 ? asynSuccess : asynError;

  } // manual velo fast.. Just store here in "motor record" driver
  else if (function == pC_->ecmcMotorRecordCfgJVEL_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgJVEL_)=%f\n", modNamEMC, axisNo_, value);    
    manualVelocSlow_ = value;
    return asynSuccess;

  } // Set acceleration
  else if (function == pC_->ecmcMotorRecordCfgACCS_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgACCS_)=%f\n", modNamEMC, axisNo_, value);
    ecmcAxis_->getTraj()->setAcc(value);
    return asynSuccess;

  }
  // Call the base class method
  status = asynMotorAxis::setDoubleParam(function, value);
  return status;
}

#ifndef motorMessageTextString
void ecmcMotorRecordAxis::updateMsgTxtFromDriver(const char *value)
{
  if (value && value[0]) {
    setStringParam(pC_->ecmcMotorRecordMCUErrMsg_,value);
  } else {
    setStringParam(pC_->ecmcMotorRecordMCUErrMsg_, "");
  }
}
#endif

//Just for debug
asynStatus ecmcMotorRecordAxis::printDiagBinData() {
  int asynLevel=ASYN_TRACE_ERROR;
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.axisID = %d\n",statusBinData_.axisID);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.cycleCounter = %d\n",statusBinData_.cycleCounter);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.acceleration = %lf\n",statusBinData_.acceleration);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.deceleration = %lf\n",statusBinData_.deceleration);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.reset = %d\n",statusBinData_.reset);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.moving = %d\n",statusBinData_.moving);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.stall = %d\n",statusBinData_.stall);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.positionSetpoint = %lf\n",statusBinData_.onChangeData.positionSetpoint);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.positionActual = %lf\n",statusBinData_.onChangeData.positionActual);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.positionError = %lf\n",statusBinData_.onChangeData.positionError);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.positionTarget = %lf\n",statusBinData_.onChangeData.positionTarget);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.cntrlError = %lf\n",statusBinData_.onChangeData.cntrlError);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.cntrlOutput = %lf\n",statusBinData_.onChangeData.cntrlOutput);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.velocityActual = %lf\n",statusBinData_.onChangeData.velocityActual);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.velocitySetpoint = %lf\n",statusBinData_.onChangeData.velocitySetpoint);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.velocityFFRaw = %lf\n",statusBinData_.onChangeData.velocityFFRaw);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.positionRaw = %ld\n",statusBinData_.onChangeData.positionRaw);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.error = %d\n",statusBinData_.onChangeData.error);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.velocitySetpointRaw = %d\n",statusBinData_.onChangeData.velocitySetpointRaw);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.seqState = %d\n",statusBinData_.onChangeData.statusWd.seqstate);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.cmdData = %d\n",statusBinData_.onChangeData.cmdData);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.command = %d\n",(int)statusBinData_.onChangeData.command);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.trajInterlock = %d\n",(int)statusBinData_.onChangeData.trajInterlock);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.lastActiveInterlock = %d\n",(int)statusBinData_.onChangeData.statusWd.lastilock);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.trajSource = %d\n",(int)statusBinData_.onChangeData.statusWd.trajsource);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.encSource = %d\n",(int)statusBinData_.onChangeData.statusWd.encsource);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.enable = %d\n",statusBinData_.onChangeData.statusWd.enable);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.enabled = %d\n",statusBinData_.onChangeData.statusWd.enabled);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.execute = %d\n",statusBinData_.onChangeData.statusWd.execute);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.busy = %d\n",statusBinData_.onChangeData.statusWd.busy);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.atTarget = %d\n",statusBinData_.onChangeData.statusWd.attarget);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.homed = %d\n",statusBinData_.onChangeData.statusWd.homed);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.limitFwd = %d\n",statusBinData_.onChangeData.statusWd.limitfwd);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.limitBwd = %d\n",statusBinData_.onChangeData.statusWd.limitbwd);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.homeSwitch = %d\n",statusBinData_.onChangeData.statusWd.homeswitch);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.sumIlockFwd = %d\n",statusBinData_.onChangeData.statusWd.sumilockfwd);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  statusBinData_.onChangeData.sumIlockBwd = %d\n",statusBinData_.onChangeData.statusWd.sumilockbwd);
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::uglyConvertFunc(ecmcAxisStatusType*in ,st_axis_status_type *out) {
  //just to test new interface.. Get all data from statusBinData_ instead    
  out->bEnable              = in->onChangeData.statusWd.enable;
  out->bExecute             = in->onChangeData.statusWd.execute;
  out->nCommand             = in->onChangeData.command;
  out->nCmdData             = in->onChangeData.cmdData;
  out->fVelocity            = in->onChangeData.velocitySetpoint;
  out->fPosition            = in->onChangeData.positionTarget;
  out->fAcceleration        = in->acceleration;
  out->fDecceleration       = in->deceleration;  
  out->bHomeSensor          = in->onChangeData.statusWd.homeswitch;
  out->bEnabled             = in->onChangeData.statusWd.enabled;
  out->bError               = in->onChangeData.error>0;
  out->nErrorId             = in->onChangeData.error;
  out->fActVelocity         = in->onChangeData.velocityActual;
  out->fActPosition         = in->onChangeData.positionActual;
  out->fActDiff             = in->onChangeData.positionError;
  out->bHomed               = in->onChangeData.statusWd.homed;
  out->bBusy                = in->onChangeData.statusWd.busy;
  out->encoderRaw           = in->onChangeData.positionRaw;
  out->atTarget             = in->onChangeData.statusWd.attarget;
  out->bLimitBwd            = in->onChangeData.statusWd.limitbwd;
  out->bLimitFwd            = in->onChangeData.statusWd.limitfwd;
  
  // Data derveid from struct
  out->mvnNRdyNex           = in->onChangeData.statusWd.busy || !in->onChangeData.statusWd.attarget;
  out->motorStatusDirection = in->onChangeData.positionActual > 
                                              oldPositionAct_ ? 1:0;
  out->motorDiffPostion     = 1; /*Always set to 1?? why */
  
  //TODO not accesible for ecmc.. neeeded? Delete later
  out->bJogFwd              = 0;
  out->bJogBwd              = 0;
  out->bReset               = 0;
  out->fOverride            = 100;

  oldPositionAct_ =  in->onChangeData.positionActual;

  return asynSuccess;
}
