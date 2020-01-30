/*
  FILENAME... ecmcMotorRecordAxis.cpp
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

  // initialize 
  memset(&drvlocal, 0, sizeof(drvlocal));
  memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));

  drvlocal.ecmcAxis = ecmcAxisRef;
  if(!drvlocal.ecmcAxis) {
        LOGERR(
      "%s/%s:%d: ERROR: Axis ref NULL. Application exits...\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

    exit(EXIT_FAILURE);
  }

  drvlocal.axisId          = axisNo;
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
#ifndef motorFlagsDriverUsesEGUString
    /* The non-ESS motor needs a dummy "stepm-size" to compensate for MRES */
    const char * const stepSize_str        = "stepSize=";
#endif
    const char * const homProc_str 	       = "HomProc=";
    const char * const homPos_str  	       = "HomPos=";
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
      
      if (!strncmp(pThisOption, axisFlags_str, strlen(axisFlags_str))) {
        pThisOption += strlen(axisFlags_str);
        int myAxisFlags = atoi(pThisOption);
        if (myAxisFlags > 0) {
          axisFlags = myAxisFlags;
        }      
#ifndef motorFlagsDriverUsesEGUString
      } else if (!strncmp(pThisOption, stepSize_str, strlen(stepSize_str))) {
        pThisOption += strlen(stepSize_str);
        /* This option is obsolete, depending on motor */
        drvlocal.scaleFactor = atof(pThisOption);
#endif
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
  if (pasynTrace->getTraceInfoMask(pC_->pasynUserSelf) & ASYN_TRACEINFO_SOURCE) modNamEMC = "";
  
  initialPoll();
}

#define AMPLIFIER_ON_FLAG_CREATE_AXIS  (1)
#define AMPLIFIER_ON_FLAG_AUTO_ON      (1<<1)
#define AMPLIFIER_ON_FLAG_USING_CNEN   (1<<2)

extern "C" int ecmcMotorRecordCreateAxis(const char *ecmcMotorRecordName, 
                                         int axisNo,
                                         int axisFlags,
                                         const char *axisOptionsStr)
{
  if (!ecmcMotorRecordName || (axisNo < 0) || (axisFlags < 0)) {
    printf("\n");
    printf("Iocsh command to create a model 3 asyn motor record driver axis for use with ECMC.\n");
    printf("Creates an ecmcMotorRecordCreateAxis object (derived from asynMotorAxis).\n");
    printf("\n");
    printf("ecmcMotorRecordCreateAxis(\n");
    printf("    MotorRecordName : Obsolete. Not used. Kept to keep syntax same as EthercatMC module.  : \"NOT_USED\"\n");
    printf("    axisNo          : Maximum number of axes (asyn parameters will be created for all).   : \"10\"\n");
    printf("    axisFlags       : Axis options (defaults to 6 in ecmccfg=> bit 1 and bit 2 set)       : \"6\"\n");
    printf("                          bit 0 : AMPLIFIER_ON_FLAG_CREATE_AXIS\n");
    printf("                          bit 1 : AMPLIFIER_ON_FLAG_AUTO_ON\n");
    printf("                          bit 2 : AMPLIFIER_ON_FLAG_USING_CNEN\n");
    printf("    axisOptionsStr  : Currently Not used in ECMC. Optional options string.                : \"\" \n");
    printf(")\n");    
    printf("Example:\n");
    printf("ecmcMotorRecordCreateAxis(\"NOT_USED\",10,6,\"\")\n");
    printf("\n");
    return asynError;
  }

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
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
                "%supdateCfgValue(%d) %s=%f\n",
                modNamEMC, axisNo_, name, newValue);
    }
  } else if (newValue != oldValue) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
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
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
                "%supdateCfgValue(%d) %s=%d\n",
                modNamEMC, axisNo_, name, newValue);
    }
  } else if (newValue != oldValue) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%supdateCfgValue(%d) old%s=%d new%s=%d\n",
              modNamEMC, axisNo_, name, oldValue, name, newValue);
  }
  return pC_->setIntegerParam(axisNo_, function, newValue);
}

asynStatus ecmcMotorRecordAxis::readBackSoftLimits(void)
{
  int    enabledFwd = 0,  enabledBwd = 0;
  double fValueFwd = 0.0, fValueBwd  = 0.0;
  double scaleFactor = drvlocal.scaleFactor;

  fValueBwd  = drvlocal.ecmcAxis->getMon()->getSoftLimitBwd();
  fValueFwd  = drvlocal.ecmcAxis->getMon()->getSoftLimitFwd();
  enabledBwd = drvlocal.ecmcAxis->getMon()->getEnableSoftLimitBwd();
  enabledFwd = drvlocal.ecmcAxis->getMon()->getEnableSoftLimitFwd();

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

  errorCode=drvlocal.ecmcAxis->getEncScaleNum(&num);
  if(errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: function getEncScaleNum() returned error (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode);
    return asynError;
  }
  errorCode=drvlocal.ecmcAxis->getEncScaleDenom(&denom);
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

  // Why is not the "drvlocal.scaleFactor" needed/updated? Always 1..
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
  poslag_tol = drvlocal.ecmcAxis->getMon()->getPosLagTol();
  poslag_time = drvlocal.ecmcAxis->getMon()->getPosLagTime() * 1 / (double)MCU_FREQUENCY;
  poslag_enable = drvlocal.ecmcAxis->getMon()->getEnableLagMon();

  // At target monitoring (must be enabled)
  attarget_tol = drvlocal.ecmcAxis->getMon()->getAtTargetTol();
  attarget_time = drvlocal.ecmcAxis->getMon()->getAtTargetTime() * 1 / (double)MCU_FREQUENCY;
  attarget_enable = drvlocal.ecmcAxis->getMon()->getEnableAtTargetMon();
   
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
  double scaleFactor = drvlocal.scaleFactor;
  double vel_max, acceleration;

  vel_max = drvlocal.ecmcAxis->getMon()->getMaxVel();
  acceleration = drvlocal.ecmcAxis->getTraj()->getAcc();

  if (drvlocal.manualVelocFast > 0.0) {
    updateCfgValue(pC_->ecmcMotorRecordCfgVELO_, drvlocal.manualVelocFast / scaleFactor, "velo");
  }
  if (vel_max > 0.0) {
    updateCfgValue(pC_->ecmcMotorRecordCfgVMAX_, vel_max / scaleFactor, "vmax");
  }
  if (drvlocal.manualVelocSlow > 0.0) {
    updateCfgValue(pC_->ecmcMotorRecordCfgJVEL_, drvlocal.manualVelocSlow / scaleFactor, "jvel");
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
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
            "%sinitialPoll(%d) status=%d\n",
            modNamEMC, axisNo_, status);
  if (status == asynSuccess) {
    drvlocal.dirty.initialPollNeeded = 0;
  }

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
  if (status == asynSuccess) status = readBackAllConfig(drvlocal.axisId);
  if (status == asynSuccess && drvlocal.dirty.statusDisconnectedOld) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%sconnected(%d)\n", modNamEMC, axisNo_);
    drvlocal.dirty.statusDisconnectedOld = 0;
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
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
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
    errorCode = drvlocal.ecmcAxis->moveRelativePosition(position * drvlocal.scaleFactor,
                                                maxVelocity * drvlocal.scaleFactor,
                                                acceleration * drvlocal.scaleFactor,
                                                acceleration * drvlocal.scaleFactor);
  }
  else
  {
    errorCode = drvlocal.ecmcAxis->moveAbsolutePosition(position * drvlocal.scaleFactor,
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
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
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
  
  int errorCode =  drvlocal.ecmcAxis->moveHome(cmdData,homPos,velToCam,velOffCam,accHom,accHom);
  
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

  int errorCode = drvlocal.ecmcAxis->moveVelocity(velo,
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
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
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
  int errorCode =  drvlocal.ecmcAxis->moveHome(HOMPROC_MANUAL_SETPOS,
                                       homPos,0,0,0,0);
  
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return errorCode == 0 ? asynSuccess:asynError;
}

asynStatus ecmcMotorRecordAxis::resetAxis(void)
{
  drvlocal.ecmcAxis->errorReset();
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::setEnable(int on) {
  int errorCode = drvlocal.ecmcAxis->setEnable(on);
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
  
  if(drvlocal.statusBinData.onChangeData.statusWd.enabled == on && 
     drvlocal.statusBinData.onChangeData.statusWd.enable == on) {
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
    if (drvlocal.statusBinData.onChangeData.statusWd.enabled == on && 
        drvlocal.statusBinData.onChangeData.statusWd.enable == on &&
       !drvlocal.statusBinData.onChangeData.statusWd.busy) {
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
  int errorCode = drvlocal.ecmcAxis->setExecute(0);
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
  int EPICS_nErrorId = drvlocal.nErrorIdMcu;
  drvlocal.eeAxisError = eeAxisErrorNoError;
  if (EPICS_nErrorId) {
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
      drvlocal.nErrorIdEpicsOld != EPICS_nErrorId ||
      drvlocal.nCommandActiveOld != drvlocal.nCommandActive) {

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
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%spoll(%d) callParamCallbacksUpdateError"
              " Error=%d old=%d ErrID=0x%x old=0x%x Warn=%d nCmd=%d old=%d txt=%s\n",
              modNamEMC, axisNo_, drvlocal.eeAxisError, drvlocal.old_eeAxisError,
              EPICS_nErrorId, drvlocal.nErrorIdEpicsOld,
              drvlocal.eeAxisWarning,
              drvlocal.nCommandActive, drvlocal.nCommandActiveOld,
              msgTxtFromDriver ? msgTxtFromDriver : "NULL");

    updateMsgTxtFromDriver(msgTxtFromDriver);
    
    drvlocal.old_eeAxisError = drvlocal.eeAxisError;
    drvlocal.old_eeAxisWarning = drvlocal.eeAxisWarning;
    drvlocal.nErrorIdEpicsOld = EPICS_nErrorId;
    drvlocal.nCommandActiveOld = drvlocal.nCommandActive;
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
  asynStatus comStatus = asynSuccess;

  double timeBefore = ecmcMotorRecordgetNowTimeSecs();
  int waitNumPollsBeforeReady_ = drvlocal.waitNumPollsBeforeReady;

  /* Driver not yet initialized, do nothing */
  if (!drvlocal.ecmcAxis->getRealTimeStarted()) 
    return comStatus;

  // Get values from ecmc
  ecmcAxisStatusType *tempAxisStat = drvlocal.ecmcAxis->getDebugInfoDataPointer();
  if(!tempAxisStat) {
    LOGERR(
      "%s/%s:%d: ERROR: function getAxisDebugInfoDataPointer() returned NULL.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

    return asynError;
  }
  // copy data locally
  memcpy(&drvlocal.statusBinData,tempAxisStat,sizeof(ecmcAxisStatusType));
  drvlocal.moveNotReadyNext = drvlocal.statusBinData.onChangeData.statusWd.busy || !drvlocal.statusBinData.onChangeData.statusWd.attarget;

  setIntegerParam(pC_->motorStatusHomed_, drvlocal.statusBinData.onChangeData.statusWd.homed);
  drvlocal.homed = drvlocal.statusBinData.onChangeData.statusWd.homed;
  setIntegerParam(pC_->motorStatusCommsError_, 0);
  setIntegerParam(pC_->motorStatusAtHome_, drvlocal.statusBinData.onChangeData.statusWd.homeswitch);
  setIntegerParam(pC_->motorStatusLowLimit_, !drvlocal.statusBinData.onChangeData.statusWd.limitbwd);
  setIntegerParam(pC_->motorStatusHighLimit_, !drvlocal.statusBinData.onChangeData.statusWd.limitfwd);
  setIntegerParam(pC_->motorStatusPowerOn_, drvlocal.statusBinData.onChangeData.statusWd.enabled);
  setDoubleParam(pC_->ecmcMotorRecordVelAct_, drvlocal.statusBinData.onChangeData.velocityActual);
  setDoubleParam(pC_->ecmcMotorRecordAcc_RB_, drvlocal.statusBinData.acceleration);

#ifndef motorWaitPollsBeforeReadyString
  if (drvlocal.waitNumPollsBeforeReady) {
    *moving = true;
  }
  else
#endif
    {
      *moving = drvlocal.moveNotReadyNext ? true : false;
      if (!drvlocal.moveNotReadyNext &&
          !(pC_->features_ & FEATURE_BITS_ECMC)) {
        /* not moving: poll the parameters for this axis */        
        switch (drvlocal.eeAxisPollNow) {          
        case pollNowReadScaling:
          readScaling(drvlocal.axisId);          
          break;
        case pollNowReadMonitoring:
          readMonitoring(drvlocal.axisId);
          drvlocal.eeAxisPollNow = pollNowReadBackSoftLimits;
          break;
        case pollNowReadBackSoftLimits:
          readBackSoftLimits();
          drvlocal.eeAxisPollNow = pollNowReadBackVelocities;
          break;
        case pollNowReadBackVelocities:
        default:          
          readBackVelocities(drvlocal.axisId);
          drvlocal.eeAxisPollNow = pollNowReadScaling;
          break;
        }
      }
    }

  if (drvlocal.moveNotReadyNext){
    drvlocal.nCommandActive = drvlocal.statusBinData.onChangeData.command;
  }
  else {
    drvlocal.nCommandActive = 0;
    if (drvlocal.eeAxisWarning == eeAxisWarningSpeedLimit) {
      drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
    }
  }
  
  if (drvlocal.nCommandActive != ECMC_CMD_HOMING) {
  
    setDoubleParam(pC_->motorPosition_,
                   drvlocal.statusBinData.onChangeData.positionActual / drvlocal.scaleFactor);
    setDoubleParam(pC_->motorEncoderPosition_,
                   drvlocal.statusBinData.onChangeData.positionActual  / drvlocal.scaleFactor);
    drvlocal.statusBinDataOld.onChangeData.positionActual = drvlocal.statusBinData.onChangeData.positionActual;
    setDoubleParam(pC_->ecmcMotorRecordVel_RB_, drvlocal.statusBinData.onChangeData.velocitySetpoint);
  }
  setDoubleParam(pC_->ecmcMotorRecordEncAct_, (double)drvlocal.statusBinData.onChangeData.positionRaw);

  if (drvlocal.statusBinDataOld.onChangeData.statusWd.homed != drvlocal.statusBinData.onChangeData.statusWd.homed) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%spoll(%d) homed=%d\n",
              modNamEMC, axisNo_, drvlocal.statusBinData.onChangeData.statusWd.homed);
    drvlocal.statusBinDataOld.onChangeData.statusWd.homed =  drvlocal.statusBinData.onChangeData.statusWd.homed;
  }
  if (drvlocal.statusBinDataOld.onChangeData.statusWd.limitbwd != drvlocal.statusBinData.onChangeData.statusWd.limitbwd) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%spoll(%d) LLS=%d\n",
              modNamEMC, axisNo_, !drvlocal.statusBinData.onChangeData.statusWd.limitbwd);
    drvlocal.statusBinDataOld.onChangeData.statusWd.limitbwd =  drvlocal.statusBinData.onChangeData.statusWd.limitbwd;
  }
  if (drvlocal.statusBinDataOld.onChangeData.statusWd.limitfwd != drvlocal.statusBinData.onChangeData.statusWd.limitfwd) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%spoll(%d) HLS=%d\n",
              modNamEMC, axisNo_,!drvlocal.statusBinData.onChangeData.statusWd.limitfwd);
    drvlocal.statusBinDataOld.onChangeData.statusWd.limitfwd = drvlocal.statusBinData.onChangeData.statusWd.limitfwd;
  }

#ifndef motorWaitPollsBeforeReadyString
  if (drvlocal.waitNumPollsBeforeReady) {    
    /* Don't update moving, done, motorStatusProblem_ */
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%spoll(%d) mvnNRdyNexAt=%d bBusy=%d bExecute=%d bEnabled=%d atTarget=%d waitNumPollsBeforeReady=%d\n",
              modNamEMC,
              axisNo_, drvlocal.moveNotReadyNext,
              drvlocal.statusBinData.onChangeData.statusWd.busy, drvlocal.statusBinData.onChangeData.statusWd.execute,
              drvlocal.statusBinData.onChangeData.statusWd.enabled, drvlocal.statusBinData.onChangeData.statusWd.attarget,
              drvlocal.waitNumPollsBeforeReady);
    drvlocal.waitNumPollsBeforeReady--;
    callParamCallbacks();
  }
  else
#endif
    {
      if (drvlocal.moveNotReadyNext != drvlocal.moveNotReadyNext ||
          drvlocal.statusBinDataOld.onChangeData.statusWd.busy     != drvlocal.statusBinData.onChangeData.statusWd.busy ||
          drvlocal.statusBinDataOld.onChangeData.statusWd.enabled  != drvlocal.statusBinData.onChangeData.statusWd.enabled ||
          drvlocal.statusBinDataOld.onChangeData.statusWd.execute  != drvlocal.statusBinData.onChangeData.statusWd.execute ||
          drvlocal.statusBinDataOld.onChangeData.statusWd.attarget != drvlocal.statusBinData.onChangeData.statusWd.attarget) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
                  "%spoll(%d) mvnNRdy=%d bBusy=%d bExecute=%d bEnabled=%d atTarget=%d wf=%d ENC=%g fPos=%g fActPosition=%g time=%f\n",
                  modNamEMC, axisNo_, drvlocal.moveNotReadyNext,                  
                  drvlocal.statusBinData.onChangeData.statusWd.busy, drvlocal.statusBinData.onChangeData.statusWd.execute,
                  drvlocal.statusBinData.onChangeData.statusWd.enabled, drvlocal.statusBinData.onChangeData.statusWd.attarget,
                  waitNumPollsBeforeReady_,
                  (double)drvlocal.statusBinData.onChangeData.positionRaw, drvlocal.statusBinData.onChangeData.positionTarget,
                  drvlocal.statusBinData.onChangeData.positionActual,
                  ecmcMotorRecordgetNowTimeSecs() - timeBefore);
      }
    }
  setIntegerParam(pC_->motorStatusDirection_, 
                  drvlocal.statusBinData.onChangeData.positionActual > 
                  drvlocal.statusBinDataOld.onChangeData.positionActual ? 1:0);
  setIntegerParam(pC_->motorStatusMoving_, drvlocal.moveNotReadyNext);
  setIntegerParam(pC_->motorStatusDone_, !drvlocal.moveNotReadyNext);

  drvlocal.nErrorIdMcu = drvlocal.statusBinData.onChangeData.error;

  if (drvlocal.bErrorOld != (drvlocal.statusBinData.onChangeData.error > 0) ||
             drvlocal.nErrorIdMcuOld != drvlocal.nErrorIdMcu ||
             drvlocal.dirty.sErrorMessage) {    
    char sErrorMessage[256];
    int nErrorId = drvlocal.statusBinData.onChangeData.error;
    const char *errIdString = errStringFromErrId(nErrorId);
    sErrorMessage[0] = '\0';
    drvlocal.sErrorMessage[0] = '\0';
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%spoll(%d) bError=%d drvlocal.statusBinData.onChangeData.error=0x%x\n",
              modNamEMC, axisNo_, drvlocal.statusBinData.onChangeData.error>0,
              nErrorId);
    drvlocal.bErrorOld = drvlocal.statusBinData.onChangeData.error > 0;
    drvlocal.nErrorIdMcuOld = nErrorId;
    drvlocal.dirty.sErrorMessage = 0;
    
    if (nErrorId) {
      /* Get the ErrorMessage to have it in the log file */
      strcpy(&sErrorMessage[0],ecmcError::convertErrorIdToString(nErrorId));  
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
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

  memcpy(&drvlocal.statusBinDataOld, &drvlocal.statusBinData,
         sizeof(drvlocal.statusBinDataOld));
  return asynSuccess;
}

/** Set the motor closed loop status
 * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus ecmcMotorRecordAxis::setClosedLoop(bool closedLoop)
{
  int value = closedLoop ? 1 : 0;
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
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
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d motorUpdateStatus_)=%d\n", modNamEMC, axisNo_, value);
  } 
  else if (function == pC_->motorStatusCommsError_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
              "%ssetIntegerParam(%d pC_->motorStatusCommsError_)=%d\n",
              modNamEMC, axisNo_, value);
    if (value && !drvlocal.dirty.statusDisconnectedOld) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s Communication error(%d)\n", modNamEMC, axisNo_);
      memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
      drvlocal.nErrorIdMcu = 0;
    }
#ifdef motorPowerAutoOnOffString
  } 
  else if (function == pC_->motorPowerAutoOnOff_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d motorPowerAutoOnOff_)=%d\n", modNamEMC, axisNo_, value);
#endif
  } 
  else if (function == pC_->ecmcMotorRecordHomProc_) {
    int motorNotHomedProblem = 0;
    setIntegerParam(pC_->ecmcMotorRecordHomProc_RB_, value);
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d HomProc_)=%d motorNotHomedProblem=%d\n",
              modNamEMC, axisNo_, value, motorNotHomedProblem);
  } 
  else if (function == pC_->ecmcMotorRecordErrRst_) {
    if (value) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
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

    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d ecmcMotorRecordCfgDHLM_En)=%d\n",
              modNamEMC, axisNo_, value);    
    errorCode = drvlocal.ecmcAxis->getMon()->setEnableSoftLimitFwd(value);
    readBackSoftLimits();
    return errorCode == 0 ? asynSuccess : asynError;

  }
  else if (function == pC_->ecmcMotorRecordCfgDLLM_En_) {
    // Set enable soft limit bwd
    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d ecmcMotorRecordCfgDLLM_En)=%d\n",
              modNamEMC, axisNo_, value);
    
    errorCode = drvlocal.ecmcAxis->getMon()->setEnableSoftLimitBwd(value);
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
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorMoveRel_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorMoveAbs_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorMoveAbs_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorMoveVel_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorMoveVel_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorHome_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorHome__)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorStop_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorStop_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorVelocity_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorVelocity_=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorVelBase_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorVelBase_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorAccel_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorAccel_)=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorDeferMoves_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motmotorDeferMoves_=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorMoveToHome_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motmotorMoveToHome_=%g\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorResolution_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorResolution_=%g\n",  modNamEMC, axisNo_, value);
    /* Limits handled above */

#ifdef motorPowerOnDelayString
  } else if (function == pC_->motorPowerOnDelay_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPowerOnDelay_)=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef motorPowerOffDelayString
  } else if (function == pC_->motorPowerOffDelay_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPowerOffDelay_=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef motorPowerOffFractionString
  } else if (function == pC_->motorPowerOffFraction_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motomotorPowerOffFraction_=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef motorPostMoveDelayString
  } else if (function == pC_->motorPostMoveDelay_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPostMoveDelay_=%g\n", modNamEMC, axisNo_, value);
#endif
  } else if (function == pC_->motorStatus_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorStatus_)=%g\n", modNamEMC, axisNo_, value);
#ifdef ecmcMotorRecordHVELFRMString
  } else if (function == pC_->ecmcMotorRecordHVELfrm_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d HVELfrm_)=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef ecmcMotorRecordHomPosString
  } else if (function == pC_->ecmcMotorRecordHomPos_) {
    pC_->setDoubleParam(axisNo_, pC_->ecmcMotorRecordHomPos_RB_, value);
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d HomPos_)=%f\n", modNamEMC, axisNo_, value);
#endif

  } // Set soft limit fwd
  else if (function == pC_->ecmcMotorRecordCfgDHLM_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgDHLM_)=%f\n", modNamEMC, axisNo_, value);
    
    errorCode = drvlocal.ecmcAxis->getMon()->setSoftLimitFwd(value);
    readBackSoftLimits();
    return errorCode==0 ? asynSuccess : asynError;

  } // Set soft limit bwd
  else if (function == pC_->ecmcMotorRecordCfgDLLM_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgDLLM_)=%f\n", modNamEMC, axisNo_, value);
    
    errorCode = drvlocal.ecmcAxis->getMon()->setSoftLimitBwd(value);
    readBackSoftLimits();
    return errorCode==0 ? asynSuccess : asynError;

  } // manual velo fast.. Just store here in "motor record" driver
  else if (function == pC_->ecmcMotorRecordCfgVELO_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgVELO_)=%f\n", modNamEMC, axisNo_, value);
  
    drvlocal.manualVelocFast = value;
    return asynSuccess;

  } // Set monitor max velocity
  else if (function == pC_->ecmcMotorRecordCfgVMAX_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgVMAX_)=%f\n", modNamEMC, axisNo_, value);
    errorCode = drvlocal.ecmcAxis->getMon()->setMaxVel(value);
    return errorCode==0 ? asynSuccess : asynError;

  } // manual velo fast.. Just store here in "motor record" driver
  else if (function == pC_->ecmcMotorRecordCfgJVEL_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgJVEL_)=%f\n", modNamEMC, axisNo_, value);    
    drvlocal.manualVelocSlow = value;
    return asynSuccess;

  } // Set acceleration
  else if (function == pC_->ecmcMotorRecordCfgACCS_) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgACCS_)=%f\n", modNamEMC, axisNo_, value);
    drvlocal.ecmcAxis->getTraj()->setAcc(value);
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

/**
 * Printout entire drvlocal.statusBinData. Just for debug purpose..
 * 
*/
asynStatus ecmcMotorRecordAxis::printDiagBinData() {
  int asynLevel=ASYN_TRACE_ERROR;
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.axisID = %d\n",drvlocal.statusBinData.axisID);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.cycleCounter = %d\n",drvlocal.statusBinData.cycleCounter);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.acceleration = %lf\n",drvlocal.statusBinData.acceleration);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.deceleration = %lf\n",drvlocal.statusBinData.deceleration);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.reset = %d\n",drvlocal.statusBinData.reset);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.moving = %d\n",drvlocal.statusBinData.moving);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.stall = %d\n",drvlocal.statusBinData.stall);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.positionSetpoint = %lf\n",drvlocal.statusBinData.onChangeData.positionSetpoint);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.positionActual = %lf\n",drvlocal.statusBinData.onChangeData.positionActual);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.positionError = %lf\n",drvlocal.statusBinData.onChangeData.positionError);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.positionTarget = %lf\n",drvlocal.statusBinData.onChangeData.positionTarget);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.cntrlError = %lf\n",drvlocal.statusBinData.onChangeData.cntrlError);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.cntrlOutput = %lf\n",drvlocal.statusBinData.onChangeData.cntrlOutput);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.velocityActual = %lf\n",drvlocal.statusBinData.onChangeData.velocityActual);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.velocitySetpoint = %lf\n",drvlocal.statusBinData.onChangeData.velocitySetpoint);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.velocityFFRaw = %lf\n",drvlocal.statusBinData.onChangeData.velocityFFRaw);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.positionRaw = %ld\n",drvlocal.statusBinData.onChangeData.positionRaw);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.error = %d\n",drvlocal.statusBinData.onChangeData.error);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.velocitySetpointRaw = %d\n",drvlocal.statusBinData.onChangeData.velocitySetpointRaw);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.seqState = %d\n",drvlocal.statusBinData.onChangeData.statusWd.seqstate);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.cmdData = %d\n",drvlocal.statusBinData.onChangeData.cmdData);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.command = %d\n",(int)drvlocal.statusBinData.onChangeData.command);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.trajInterlock = %d\n",(int)drvlocal.statusBinData.onChangeData.trajInterlock);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.lastActiveInterlock = %d\n",(int)drvlocal.statusBinData.onChangeData.statusWd.lastilock);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.trajSource = %d\n",(int)drvlocal.statusBinData.onChangeData.statusWd.trajsource);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.encSource = %d\n",(int)drvlocal.statusBinData.onChangeData.statusWd.encsource);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.enable = %d\n",drvlocal.statusBinData.onChangeData.statusWd.enable);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.enabled = %d\n",drvlocal.statusBinData.onChangeData.statusWd.enabled);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.execute = %d\n",drvlocal.statusBinData.onChangeData.statusWd.execute);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.busy = %d\n",drvlocal.statusBinData.onChangeData.statusWd.busy);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.atTarget = %d\n",drvlocal.statusBinData.onChangeData.statusWd.attarget);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.homed = %d\n",drvlocal.statusBinData.onChangeData.statusWd.homed);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.limitFwd = %d\n",drvlocal.statusBinData.onChangeData.statusWd.limitfwd);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.limitBwd = %d\n",drvlocal.statusBinData.onChangeData.statusWd.limitbwd);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.homeSwitch = %d\n",drvlocal.statusBinData.onChangeData.statusWd.homeswitch);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.sumIlockFwd = %d\n",drvlocal.statusBinData.onChangeData.statusWd.sumilockfwd);
  asynPrint(pC_->pasynUserSelf, asynLevel,"  drvlocal.statusBinData.onChangeData.sumIlockBwd = %d\n",drvlocal.statusBinData.onChangeData.statusWd.sumilockbwd);
  return asynSuccess;
}
