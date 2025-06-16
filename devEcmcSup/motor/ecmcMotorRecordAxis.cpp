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
#include "ecmcGlobalsExtern.h"
#include "ecmcPluginClient.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif // ifndef ASYN_TRACE_INFO

/* The maximum number of polls we wait for the motor
   to "start" (report moving after a new move command */
#define WAITNUMPOLLSBEFOREREADY 3

static ecmcMotorRecordController *pC;
extern asynUser *pPrintOutAsynUser;

/**
 * Option strings
*/
#define ECMC_AXIS_OPT_HOME_PROC         "HomProc="
#define ECMC_AXIS_OPT_HOME_POS          "HomPos="
#define ECMC_AXIS_OPT_FLAGS             "axisFlags="
#define ECMC_AXIS_OPT_POWER_AUTO_ON_OFF "powerAutoOnOff="
#define ECMC_AXIS_OPT_POWER_OFF_DELAY   "powerOffDelay="
#define ECMC_AXIS_OPT_POWER_ON_DELAY    "powerOnDelay="
#define ECMC_AXIS_OPT_STR_LEN 15

#define ECMC_AXIS_ENABLE_SLEEP_PERIOD 0.1
#define ECMC_AXIS_ENABLE_MAX_SLEEP_TIME 3.0


/** Creates a new ecmcMotorRecordAxis object.
 * \param[in] pC Pointer to the ecmcMotorRecordController to which this axis belongs.
 * \param[in] axisNo Index number of this axis, range 1 to pC->numAxes_. (0 is not used)
 *
 *
 * Initializes register numbers, etc.
 */
ecmcMotorRecordAxis::ecmcMotorRecordAxis(ecmcMotorRecordController *pC,
                                         int                        axisNo,
                                         ecmcAxisBase              *ecmcAxisRef,
                                         int                        axisFlags,
                                         const char                *axisOptionsStr)
  : asynMotorAxis(pC, axisNo),
  pC_(pC) {  

  /* Some parameters are only defined in the ESS fork of the motor module.
     So they have the ifdef */
  setIntegerParam(pC_->motorFlagsDriverUsesEGU_,   1);
#ifdef motorFlagsAdjAfterHomedString
  setIntegerParam(pC_->motorFlagsAdjAfterHomed_,   1);
#endif // ifdef motorFlagsAdjAfterHomedString
#ifdef motorWaitPollsBeforeReadyString
  setIntegerParam(pC_->motorWaitPollsBeforeReady_, WAITNUMPOLLSBEFOREREADY);
#endif // ifdef motorWaitPollsBeforeReadyString
#ifdef motorFlagsNtmUpdateString
  setIntegerParam(pC_->motorFlagsNtmUpdate_,       1);
#endif // ifdef motorFlagsNtmUpdateString
  profileMaxPoints_ = 0;
  // initialize
  memset(&drvlocal,       0,    sizeof(drvlocal));
  memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
  strcpy(profileMessage_, "");

  //restorePowerOnOffNeeded_ = 0;
  drvlocal.ecmcAxis    = ecmcAxisRef;
  profileLastBuildOk_  = false;
  profileLastInitOk_   = false;
  profileLastDefineOk_ = false;
  profileInProgress_   = false;
  profileSwitchPVTObject_ = false;
  pvtEnabled_             = 0;
  
  pvtPrepare_ = NULL;
  pvtRunning_ = NULL;

  if (!drvlocal.ecmcAxis) {
    LOGERR(
      "%s/%s:%d: ERROR: Axis ref NULL. Application exits...\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

    exit(EXIT_FAILURE);
  }
  drvlocal.axisId          = axisNo;
  drvlocal.old_eeAxisError = eeAxisErrorIOCcomError;
  drvlocal.axisFlags       = axisFlags;
  triggstop_ = 1;
  triggsync_ = 1;

  /* We pretend to have an encoder (fActPosition) */
  setIntegerParam(pC_->motorStatusHasEncoder_,   1);
#ifdef motorFlagsNoStopProblemString
  setIntegerParam(pC_->motorFlagsNoStopProblem_, 1);
#endif // ifdef motorFlagsNoStopProblemString
#ifdef motorFlagsNoStopOnLsString
  setIntegerParam(pC_->motorFlagsNoStopOnLS_,    1);
#endif // ifdef motorFlagsNoStopOnLsString
#ifdef motorFlagsLSrampDownString
  setIntegerParam(pC_->motorFlagsLSrampDown_,    1);
#endif // ifdef motorFlagsLSrampDownString
#ifdef motorFlagsPwrWaitForOnString
  setIntegerParam(pC_->motorFlagsPwrWaitForOn_,  1);
#endif // ifdef motorFlagsPwrWaitForOnString

  if (axisFlags & AMPLIFIER_ON_FLAG_AUTO_ON) {
#ifdef POWERAUTOONOFFMODE2
    setIntegerParam(pC_->motorPowerAutoOnOff_, POWERAUTOONOFFMODE2);
    setDoubleParam(pC_->motorPowerOnDelay_,  6.0);
    setDoubleParam(pC_->motorPowerOffDelay_, -1.0);
#endif // ifdef POWERAUTOONOFFMODE2
#ifdef motorShowPowerOffString
    setIntegerParam(pC_->motorShowPowerOff_,    1);
#endif // ifdef motorShowPowerOffString
#ifdef  motorNotHomedProblemString
    setIntegerParam(pC_->motorNotHomedProblem_, MOTORNOTHOMEDPROBLEM_ERROR);
#endif // ifdef  motorNotHomedProblemString
  }

  if (axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    setIntegerParam(pC->motorStatusGainSupport_, 1);
  }

  if (axisOptionsStr && axisOptionsStr[0]) {
    char *pOptions    = strdup(axisOptionsStr);
    char *pThisOption = pOptions;
    char *pNextOption = pOptions;

    while (pNextOption && pNextOption[0]) {
      pNextOption = strchr(pNextOption, ';');

      if (pNextOption) {
        *pNextOption = '\0'; /* Terminate */
        pNextOption++;       /* Jump to (possible) next */
      }

      if (!strncmp(pThisOption, ECMC_AXIS_OPT_FLAGS,
                          strlen(ECMC_AXIS_OPT_FLAGS))) {
        pThisOption += strlen(ECMC_AXIS_OPT_FLAGS);
        int myAxisFlags = atoi(pThisOption);

        if (myAxisFlags > 0) {
          axisFlags = myAxisFlags;
        }
      } else if (!strncmp(pThisOption, ECMC_AXIS_OPT_POWER_AUTO_ON_OFF,
                          strlen(ECMC_AXIS_OPT_POWER_AUTO_ON_OFF))) {
        pThisOption += strlen(ECMC_AXIS_OPT_POWER_AUTO_ON_OFF);
        int powerAutoOnOff = -1;       /* undefined */
        powerAutoOnOff = atoi(pThisOption);
        setIntegerParam(pC_->motorPowerAutoOnOff_, powerAutoOnOff);
      } else if (!strncmp(pThisOption, ECMC_AXIS_OPT_HOME_PROC,
                          strlen(ECMC_AXIS_OPT_HOME_PROC))) {
        pThisOption += strlen(ECMC_AXIS_OPT_HOME_PROC);
        int homProc = atoi(pThisOption);
        setIntegerParam(pC_->ecmcMotorRecordHomProc_, homProc);
      } else if (!strncmp(pThisOption, ECMC_AXIS_OPT_HOME_POS,
                          strlen(ECMC_AXIS_OPT_HOME_POS))) {
        pThisOption += strlen(ECMC_AXIS_OPT_HOME_POS);
        double homPos = atof(pThisOption);
        setDoubleParam(pC_->ecmcMotorRecordHomPos_, homPos);
      } else if (!strncmp(pThisOption, ECMC_AXIS_OPT_POWER_OFF_DELAY,
                          strlen(ECMC_AXIS_OPT_POWER_OFF_DELAY))) {
        double powerOffDelay;
        pThisOption  += strlen(ECMC_AXIS_OPT_POWER_OFF_DELAY);
        powerOffDelay = atof(pThisOption);
        updateCfgValue(pC_->motorPowerOffDelay_, powerOffDelay,
                       "powerOffDelay");
      } else if (!strncmp(pThisOption, ECMC_AXIS_OPT_POWER_ON_DELAY,
                          strlen(ECMC_AXIS_OPT_POWER_ON_DELAY))) {
        double powerOnDelay;
        pThisOption += strlen(ECMC_AXIS_OPT_POWER_ON_DELAY);
        powerOnDelay = atof(pThisOption);
        updateCfgValue(pC_->motorPowerOnDelay_, powerOnDelay, "powerOnDelay");
      }
      pThisOption = pNextOption;
    }
    free(pOptions);
  }

  /* Set the module name to "" if we have FILE/LINE enabled by asyn */
  if (pasynTrace->getTraceInfoMask(pPrintOutAsynUser) &
      ASYN_TRACEINFO_SOURCE) modNamEMC = "";
  

  initialPoll();
  
}

extern "C" int ecmcMotorRecordCreateAxis(const char *controllerPortName,
                                         int         axisNo,
                                         int         axisFlags,
                                         const char *axisOptionsStr) {
  if ((axisNo >= ECMC_MAX_AXES) || (axisNo < 0)) {
    printf("ERROR: axisNo out of range. Allowed values 0..%d\n",
           ECMC_MAX_AXES - 1);
    return asynError;
  }

  if (!controllerPortName || (axisNo < 0) || (axisFlags < 0)) {
    printf("\n");
    printf(
      "Iocsh command to create a model 3 asyn motor record driver axis for use with ECMC.\n");
    printf(
      "Creates an ecmcMotorRecordAxis object (derived from asynMotorAxis).\n");
    printf("\n");
    printf("ecmcMotorRecordCreateAxis(\n");
    printf(
      "    controllerPortName : Asyn port name for the motor controller.                          : \"ECMC_ASYN_MOTOR_PORT\"\n");
    printf(
      "    axisNo             : Maximum number of axes (asyn parameters will be created for all). : \"10\"\n");
    printf(
      "    axisFlags          : Axis options (defaults to 6 in ecmccfg => 0x110):                 : \"6\"\n");
    printf(
      "                             -bit 0 : AMPLIFIER_ON_FLAG_CREATE_AXIS\n");
    printf("                             -bit 1 : AMPLIFIER_ON_FLAG_AUTO_ON\n");
    printf(
      "                             -bit 2 : AMPLIFIER_ON_FLAG_USING_CNEN\n");
    printf(
      "    axisOptionsStr     : Optional options strings:                                         : \"\" \n");
    printf(
      "                             -%-*s : Set homing sequence type (over-rides/writes def in record/param)\n",
      ECMC_AXIS_OPT_STR_LEN,
      ECMC_AXIS_OPT_HOME_PROC);
    printf(
      "                             -%-*s : Set homing position (over-rides/writes def in record/param)\n",
      ECMC_AXIS_OPT_STR_LEN,
      ECMC_AXIS_OPT_HOME_POS);
    printf(
      "                             -%-*s : Set axisFlags (over-rides/writes axisFlags in this call)\n",
      ECMC_AXIS_OPT_STR_LEN,
      ECMC_AXIS_OPT_FLAGS);
    printf(
      "                             -%-*s : Set powerAutoOnOff (over-rides/writes def in record/param)\n",
      ECMC_AXIS_OPT_STR_LEN,
      ECMC_AXIS_OPT_POWER_AUTO_ON_OFF);
    printf(
      "                             -%-*s : Set powerOffDelay (over-rides/writes def in record/param)\n",
      ECMC_AXIS_OPT_STR_LEN,
      ECMC_AXIS_OPT_POWER_OFF_DELAY);
    printf(
      "                             -%-*s : Set powerOnDelay (over-rides/writes def in record/param)\n",
      ECMC_AXIS_OPT_STR_LEN,
      ECMC_AXIS_OPT_POWER_ON_DELAY);
    printf(")\n");
    printf("Example:\n");
    printf("ecmcMotorRecordCreateAxis(\"ECMC_ASYN_MOTOR_PORT\",10,6,\"\")\n");
    printf("\n");
    return asynError;
  }

  pC = (ecmcMotorRecordController *)findAsynPortDriver(controllerPortName);

  if (!pC) {
    printf("ERROR: Asyn port %s not found.\n", controllerPortName);
    return asynError;
  }

  if ((axisNo >= ECMC_MAX_AXES) || (axisNo <= 0)) {
    printf("ERROR: Axis index out of range.\n");
    return asynError;
  }

  if (axes[axisNo] == NULL) {
    printf("ERROR: Axis object NULL.\n");
    return asynError;
  }

  pC->lock();
  new ecmcMotorRecordAxis(pC, axisNo, axes[axisNo], axisFlags, axisOptionsStr);
  pC->unlock();
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::updateCfgValue(int         function,
                                               double      newValue,
                                               const char *name) {
  double oldValue;
  asynStatus status = pC_->getDoubleParam(axisNo_, function, &oldValue);

  if (status) {
    /* First time that we write the value after IOC restart
       ECMC configures everything from the iocshell, no need to
       do a print here */
    if (!(pC_->features_ & FEATURE_BITS_ECMC)) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_INFO,
                "%s/%s:%d: Axis[%d] Update cfg %s=%f\n",
                __FILE__,
                __FUNCTION__,
                __LINE__,
                axisNo_,
                name,
                newValue);
    }
  } else if (newValue != oldValue) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%s/%s:%d: Axis[%d] Update cfg %s=%f (old %f)\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              axisNo_,
              name,
              newValue,
              oldValue);
  }
  return pC_->setDoubleParam(axisNo_, function, newValue);
}

asynStatus ecmcMotorRecordAxis::updateCfgValue(int         function,
                                               int         newValue,
                                               const char *name) {
  int oldValue;
  asynStatus status = pC_->getIntegerParam(axisNo_, function, &oldValue);

  if (status) {
    /* First time that we write the value after IOC restart
       ECMC configures everything from the iocshell, no need to
       do a print here */
    if (!((pC_->features_ & FEATURE_BITS_ECMC))) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_INFO,
                "%s/%s:%d: Axis[%d] Update cfg %s=%d\n",
                __FILE__,
                __FUNCTION__,
                __LINE__,
                axisNo_,
                name,
                newValue);
    }
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%s/%s:%d: Axis[%d] Update cfg %s=%d (old %d)\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              axisNo_,
              name,
              newValue,
              oldValue);
  } else if (newValue != oldValue) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%s/%s:%d: Axis[%d] Update cfg %s=%d (old %d)\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              axisNo_,
              name,
              newValue,
              oldValue);
  }
  return pC_->setIntegerParam(axisNo_, function, newValue);
}

asynStatus ecmcMotorRecordAxis::readBackSoftLimits(void) {
  int enabledFwd = 0,  enabledBwd = 0;
  double fValueFwd = 0.0, fValueBwd  = 0.0;

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  fValueBwd  = drvlocal.ecmcAxis->getMon()->getSoftLimitBwd();
  fValueFwd  = drvlocal.ecmcAxis->getMon()->getSoftLimitFwd();
  enabledBwd = drvlocal.ecmcAxis->getMon()->getEnableSoftLimitBwd();
  enabledFwd = drvlocal.ecmcAxis->getMon()->getEnableSoftLimitFwd();

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  pC_->setIntegerParam(axisNo_, pC_->ecmcMotorRecordCfgDLLM_En_, enabledBwd);
  pC_->setDoubleParam(axisNo_, pC_->ecmcMotorRecordCfgDLLM_, fValueBwd);
  pC_->setIntegerParam(axisNo_, pC_->ecmcMotorRecordCfgDHLM_En_, enabledFwd);
  pC_->setDoubleParam(axisNo_, pC_->ecmcMotorRecordCfgDHLM_, fValueFwd);


  pC_->udateMotorLimitsRO(axisNo_,
                          enabledBwd && enabledFwd,
                          fValueFwd,
                          fValueBwd);
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::readScaling(int axisID) {
  int errorCode = 0;
  double num = 0, denom = 0;

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  errorCode = drvlocal.ecmcAxis->getEncScaleNum(&num);

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: function getEncScaleNum() returned error (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode);
    return asynError;
  }

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  errorCode = drvlocal.ecmcAxis->getEncScaleDenom(&denom);

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: function getEncScaleDenom() returned error (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode);
    return asynError;
  }

  if (!denom) {
    LOGERR(
      "%s/%s:%d: ERROR: Encoder denominator scale zero.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

    return asynError;
  }

  if (!num) {
    LOGERR(
      "%s/%s:%d: ERROR: Encoder numerator scale zero.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

    return asynError;
  }
  updateCfgValue(pC_->ecmcMotorRecordCfgSREV_RB_, denom, "srev");
  updateCfgValue(pC_->ecmcMotorRecordCfgUREV_RB_, num,   "urev");

  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::readMonitoring(int axisID) {
  double poslag_tol, attarget_tol, attarget_time, poslag_time;
  int    poslag_enable, attarget_enable;

  // Position lag monitoring (following error)
  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  poslag_tol  = drvlocal.ecmcAxis->getMon()->getPosLagTol();
  poslag_time = drvlocal.ecmcAxis->getMon()->getPosLagTime() * 1 /
                mcuFrequency;
  poslag_enable = drvlocal.ecmcAxis->getMon()->getEnableLagMon();

  // At target monitoring (must be enabled)
  attarget_tol  = drvlocal.ecmcAxis->getMon()->getAtTargetTol();
  attarget_time = drvlocal.ecmcAxis->getMon()->getAtTargetTime() * 1 /
                  mcuFrequency;
  attarget_enable = drvlocal.ecmcAxis->getMon()->getEnableAtTargetMon();

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  // At target monitoring must be enabled
  if(!attarget_enable || !attarget_tol) {
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR,
            "%sinitialPoll(%d): Warning: Attarget monitoring disabled\n",
            modNamEMC, axisNo_);
  }
  drvlocal.illegalInTargetWindow = (!attarget_enable || !attarget_tol);

  // (Target position monitoring value)
  updateCfgValue(pC_->ecmcMotorRecordCfgSPDB_RB_, attarget_tol,
                 "spbd");

  // (Target position monitoring value)
  updateCfgValue(pC_->ecmcMotorRecordCfgRDBD_RB_, attarget_tol,
                 "rdbd");

  // (Target position monitoring time)
  updateCfgValue(pC_->ecmcMotorRecordCfgRDBD_Tim_RB_,
                 attarget_time,
                 "rdbd_time");

  // (Target position monitoring enable)
  updateCfgValue(pC_->ecmcMotorRecordCfgRDBD_En_RB_,
                 attarget_enable,
                 "rdbd_en");

  // (Maximum position lag value)
  updateCfgValue(pC_->ecmcMotorRecordCfgPOSLAG_RB_,
                 poslag_tol,
                 "poslag");

  // (Maximum position lag time)
  updateCfgValue(pC_->ecmcMotorRecordCfgPOSLAG_Tim_RB_,
                 poslag_time,
                 "poslag_tim");

  // (Maximum position lag enable)
  updateCfgValue(pC_->ecmcMotorRecordCfgPOSLAG_En_RB_,
                 poslag_enable,
                 "poslag_en");

  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::readBackVelocities(int axisID) {
  double vel_max, acceleration;

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  vel_max      = drvlocal.ecmcAxis->getMon()->getMaxVel();
  acceleration = drvlocal.ecmcAxis->getTraj()->getAcc();

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  if (drvlocal.manualVelocFast > 0.0) {
    updateCfgValue(pC_->ecmcMotorRecordCfgVELO_,
                   drvlocal.manualVelocFast,
                   "velo");
  }

  if (vel_max > 0.0) {
    updateCfgValue(pC_->ecmcMotorRecordCfgVMAX_, vel_max, "vmax");
  }

  if (drvlocal.manualVelocSlow > 0.0) {
    updateCfgValue(pC_->ecmcMotorRecordCfgJVEL_,
                   drvlocal.manualVelocSlow,
                   "jvel");
  }

  if (acceleration > 0.0) {
    updateCfgValue(pC_->ecmcMotorRecordCfgACCS_, acceleration, "accs");
  }
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::initialPoll(void) {
  asynStatus status;

  if (!drvlocal.dirty.initialPollNeeded) return asynSuccess;

  status = initialPollInternal();
  asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
            "%sinitialPoll(%d) status=%d\n",
            modNamEMC, axisNo_, status);

  if (status == asynSuccess) {
    drvlocal.dirty.initialPollNeeded = 0;
  }

  return status;
}

asynStatus ecmcMotorRecordAxis::readBackAllConfig(int axisID) {
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
asynStatus ecmcMotorRecordAxis::initialPollInternal(void) {
  asynStatus status = asynSuccess;

  if (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_CREATE_AXIS) {
    /* Enable the amplifier when the axis is created,
       but wait until we have a connection to the controller.
       After we lost the connection, Re-enable the amplifier
       See AMPLIFIER_ON_FLAG */
    status = enableAmplifier(1);
  }

  if (status == asynSuccess) status = readBackAllConfig(drvlocal.axisId);

  if ((status == asynSuccess) && drvlocal.dirty.statusDisconnectedOld) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%s/%s:%d: Axis[%d] Connected\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              axisNo_);
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
void ecmcMotorRecordAxis::report(FILE *fp, int level) {
  fprintf(fp, "  axis %d\n", axisNo_);

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
asynStatus ecmcMotorRecordAxis::move(double position,
                                     int    relative,
                                     double minVelocity,
                                     double maxVelocity,
                                     double acceleration) {
  asynPrint(pPrintOutAsynUser,
            ASYN_TRACE_INFO,
            "%s/%s:%d: Axis[%d] Move cmd: trgpos=%lf, rel=%d, velo = %lf..%lf, acc=%lf\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            axisNo_,
            position,
            relative,
            minVelocity,
            maxVelocity,
            acceleration);

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;

  /* Do range check */
  if (!maxVelocity) {
    drvlocal.eeAxisWarning = eeAxisWarningVeloZero;
    return asynSuccess;
  }

  int errorCode = 0;

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);

  if (drvlocal.ecmcAxis->getBlockExtCom()) {
    if (ecmcRTMutex) epicsMutexUnlock(ecmcRTMutex);
    LOGERR(
      "%s/%s:%d: ERROR: Communication to ECMC blocked, motion commands not allowed..\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }

  if (relative) {
    errorCode = drvlocal.ecmcAxis->moveRelativePosition(position,
                                                        maxVelocity,
                                                        acceleration,
                                                        acceleration);
  } else {
    errorCode = drvlocal.ecmcAxis->moveAbsolutePosition(position,
                                                        maxVelocity,
                                                        acceleration,
                                                        acceleration);
  }

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

#ifndef motorWaitPollsBeforeReadyString
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
#endif // ifndef motorWaitPollsBeforeReadyString
  return errorCode == 0 ? asynSuccess : asynError;
}

/** Home the motor, search the home position
 * \param[in] minimum velocity, mm/sec
 * \param[in] maximum velocity, mm/sec
 * \param[in] acceleration, seconds to maximum velocity
 * \param[in] forwards (0=backwards, otherwise forwards)
 *
 */
asynStatus ecmcMotorRecordAxis::home(double minVelocity,
                                     double maxVelocity,
                                     double acceleration,
                                     int    forwards) {
  // cmd, nCmddata,homepos,velhigh,vellow,acc

  asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
            "%s/%s:%d: Axis[%d] Home cmd:  = %lf..%lf, acc=%lf, fwd=%d\n",
            __FILE__, __FUNCTION__, __LINE__,
            axisNo_, minVelocity, maxVelocity, acceleration, forwards);

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;

  // Read from records / params
  double homPos    = 0.0; /* The homPos may be undefined, then use 0.0 */
  int    cmdData   = -1;
  double velToCam  = 0;
  double velOffCam = 0;
  double accHom    = 0;

  // nCmdData (sequence number)
  asynStatus status = pC_->getIntegerParam(axisNo_,
                                           pC_->ecmcMotorRecordHomProc_,
                                           &cmdData);

  if (status != asynSuccess) {
    return asynError;
  }

  /* ECMC_SEQ_HOME_SET_POS is blocked by motor record.
   * The new sequence ECMC_SEQ_HOME_SET_POS_2 (25) is the same but not blocked.
   * by motor record.
   *
   * Handle auto restore and set position in the same way
   */
  if ((cmdData == ECMC_SEQ_HOME_SET_POS_2) ||
      (cmdData == ECMC_SEQ_HOME_SET_POS)) {
    // Use ECMC_SEQ_HOME_SET_POS internally in ecmc
    cmdData = ECMC_SEQ_HOME_SET_POS;
  }

  // Home position
  (void)pC_->getDoubleParam(axisNo_,
                            pC_->ecmcMotorRecordHomPos_,
                            &homPos);

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

  int errorCode = 0;

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);

  if (drvlocal.ecmcAxis->getBlockExtCom()) {
    if (ecmcRTMutex) epicsMutexUnlock(ecmcRTMutex);

    LOGERR(
      "%s/%s:%d: ERROR: Communication to ECMC blocked, motion commands not allowed..\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }

  printf("cmdData=%d\n",cmdData);
  
  // "-1" will lead to not overwriting anything that is set in ecmc
  errorCode =  drvlocal.ecmcAxis->moveHome(cmdData,
                                           homPos,
                                           velToCam,
                                           velOffCam,
                                           accHom,
                                           accHom);

  if (ecmcRTMutex) epicsMutexUnlock(ecmcRTMutex);

#ifndef motorWaitPollsBeforeReadyString
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
#endif // ifndef motorWaitPollsBeforeReadyString
  return errorCode == 0 ? asynSuccess : asynError;
}

/** jog the the motor, search the home position
 * \param[in] minimum velocity, mm/sec (not used)
 * \param[in] maximum velocity, mm/sec (positive or negative)
 * \param[in] acceleration, seconds to maximum velocity
 *
 */
asynStatus ecmcMotorRecordAxis::moveVelocity(double minVelocity,
                                             double maxVelocity,
                                             double acceleration) {
  asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
            "%s/%s:%d: Axis[%d] Move vel cmd:  = %lf..%lf, acc=%lf\n",
            __FILE__, __FUNCTION__, __LINE__,
            axisNo_, minVelocity, maxVelocity, acceleration);

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;

  /* Do range check */
  if (!maxVelocity) {
    drvlocal.eeAxisWarning = eeAxisWarningVeloZero;
    return asynSuccess;
  }

  if (acceleration == 0) {
    LOGERR(
      "%s/%s:%d: ERROR: Acceleration setpoint 0.0. Command aborted..\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }

  // Velocity
  double velo = maxVelocity;

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

  int errorCode = 0;

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);

  if (drvlocal.ecmcAxis->getBlockExtCom()) {
    if (ecmcRTMutex) epicsMutexUnlock(ecmcRTMutex);
    LOGERR(
      "%s/%s:%d: ERROR: Communication to ECMC blocked, motion commands not allowed..\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }

  // if(drvlocal.ecmcAxis->getAllowConstVelo()) {
  errorCode = drvlocal.ecmcAxis->moveVelocity(velo,
                                              acc,
                                              acc);

  // } else
  // {
  //  LOGERR(
  //    "%s/%s:%d: ERROR: Constant velo disabled and therefore not allowed.\n",
  //    __FILE__,
  //    __FUNCTION__,
  //    __LINE__);
  // }

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

#ifndef motorWaitPollsBeforeReadyString
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
#endif // ifndef motorWaitPollsBeforeReadyString
  return errorCode == 0 ? asynSuccess : asynError;
}

/**
 * See asynMotorAxis::setPosition
 */
asynStatus ecmcMotorRecordAxis::setPosition(double value) {
  asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
            "%s/%s:%d: Axis[%d] Set pos cmd:  = %lf\n",
            __FILE__, __FUNCTION__, __LINE__,
            axisNo_, value);

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  int errorCode =  drvlocal.ecmcAxis->setPosition(value);

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  return errorCode == 0 ? asynSuccess : asynError;
}

asynStatus ecmcMotorRecordAxis::resetAxis(void) {
  asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
            "%s/%s:%d: Axis[%d] Reset cmd\n",
            __FILE__, __FUNCTION__, __LINE__,
            axisNo_);

  drvlocal.eeAxisWarning      = eeAxisWarningNoWarning;
  drvlocal.cmdErrorMessage[0] = 0;

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  drvlocal.ecmcAxis->errorReset();

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  // Refresh
  bool moving;
  poll(&moving);
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::setEnable(int on) {
  asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
            "%s/%s:%d: Axis[%d] Set enable cmd %d\n",
            __FILE__, __FUNCTION__, __LINE__,
            axisNo_, on);

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);

  if (drvlocal.ecmcAxis->getBlockExtCom()) {
    if (ecmcRTMutex) epicsMutexUnlock(ecmcRTMutex);
    LOGERR(
      "%s/%s:%d: ERROR: Communication to ECMC blocked, motion commands not allowed..\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }

  int errorCode = drvlocal.ecmcAxis->setEnable(on);
  
  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: Function setEnable(%d) returned errorCode (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      on,
      errorCode);

    return asynError;
  }

  
  return asynSuccess;
}

/**
 *  Used by auto power on functionality
 *  (used  if POWERAUTOONOFFMODE2, motorPowerAutoOnOff_ = 2);
 *  Method returns enabled state of axis.
 *  Method is called cyclic from asynMotorController::autoPowerOn()
 */
bool ecmcMotorRecordAxis::pollPowerIsOn(void) {
  int enabled = 0;

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  enabled = drvlocal.ecmcAxis->getEnabled() && drvlocal.ecmcAxis->getEnable();
  
  if(drvlocal.ecmcAxis->getMon()->getSumInterlock()) {
    triggstop_++;
    if(triggstop_ == 0) {
      triggstop_++;
    }
    asynMotorAxis::setIntegerParam(pC_->motorStop_, triggstop_);  // Stop also triggered in ecmc, try to sync motor record and ecmc    
    asynMotorAxis::setIntegerParam(pC_->ecmcMotorRecordTRIGG_STOPP_,triggstop_);  // Stop also triggered in ecmc, try to sync motor record and ecmc
    asynMotorAxis::setIntegerParam(pC_->motorStatusDone_, 1);    
    callParamCallbacks();
  }

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
            "%s/%s:%d: Axis[%d] Poll power is on %d\n",
            __FILE__, __FUNCTION__, __LINE__,
            axisNo_, enabled > 0);

  return enabled > 0;
}

/**
 *  Enable the amplifier on an axis
 */
asynStatus ecmcMotorRecordAxis::enableAmplifier(int on) {
  asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
            "%s/%s:%d: Axis[%d] Enable amp cmd %d\n",
            __FILE__, __FUNCTION__, __LINE__,
            axisNo_, on);

  if(drvlocal.ecmcSafetyInterlock && on) {
    printf("Safety[%d]: enableAmplifier(%d) failed. Ecmc safety interlock active\n",axisNo_,on);
    asynMotorAxis::setIntegerParam(pC_->motorClosedLoop_, 0);
    callParamCallbacks();
    return asynError;
  }

  asynStatus status  = asynSuccess;
  unsigned   counter = (int)(ECMC_AXIS_ENABLE_MAX_SLEEP_TIME /
                             ECMC_AXIS_ENABLE_SLEEP_PERIOD);
  int  justCheckForEnable = 0;
  bool moving             = 0;

  #ifdef POWERAUTOONOFFMODE2
  {
    int autoPower;
    pC_->getIntegerParam(axisNo_, pC_->motorPowerAutoOnOff_, &autoPower);

    if (autoPower) {
      /* The record/driver will check for enabled (pollPowerIsOn)- only check enable */
      justCheckForEnable = 1;
    }
  }
   #endif // ifdef POWERAUTOONOFFMODE2

  on = on ? 1 : 0; /* either 0 or 1 */

  if ((drvlocal.status_.statusWord_.enabled == on) &&
      (drvlocal.status_.statusWord_.enable == on)) {
    return status;  // status OK
  }

  if (!on) {
    /* Amplifier is on and should be turned off.*/
    status = stopAxisInternal(__FUNCTION__, 0);

    if (status) return status;
  }

  status = setEnable(on);

  /* 
    Ignore disable error.
    For use with master axes groups when plc code control disabling of axes.   
  */

  if(drvlocal.ecmcIgnoreDisableAxisStatus && !on) {
    poll(&moving);
    return asynSuccess;
  }

  if (status) return status;

  while (counter) {
    epicsThreadSleep(ECMC_AXIS_ENABLE_SLEEP_PERIOD);
    asynStatus status = readEcmcAxisStatusData();
    if (status) {
      return status;
    }    
    if (((drvlocal.status_.statusWord_.enabled == on) ||
         justCheckForEnable) &&
        (drvlocal.status_.statusWord_.enable == on) &&
        !drvlocal.status_.statusWord_.busy) {
      /* The poller co-ordinates the writing into the parameter library */
      poll(&moving);
      return asynSuccess;
    }
    counter = counter - 1;
  }

  /* if we come here, it went wrong */
  if (!drvlocal.cmdErrorMessage[0]) {
    snprintf(drvlocal.cmdErrorMessage, sizeof(drvlocal.cmdErrorMessage) - 1,
             "E: enableAmplifier(%d) failed.",
             axisNo_);
  }

  return asynError;
}

/**
 * Stop the axis
 */
asynStatus ecmcMotorRecordAxis::stopAxisInternal(const char *function_name,
                                                 double      acceleration) {
  asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
            "%s/%s:%d: Axis[%d] Stop cmd:  name=%s, acc=%lf\n",
            __FILE__, __FUNCTION__, __LINE__,
            axisNo_, function_name, acceleration);

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  int errorCode = drvlocal.ecmcAxis->setExecute(0);
  if(profileInProgress_) {
    if(pC_->pvtController_){
      if(pC_->pvtController_->getBusy()){
        pC_->abortProfile();
        printf("Axis[%d]: stopAxisInternal\n",axisNo_);
      }
    }
    profileInProgress_ = false;
  }
  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  if (errorCode) {
    LOGERR(
      "%s/%s:%d: ERROR: Function setExecute(0) returned errorCode (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      errorCode);

    return asynError;
  }

  return asynSuccess;
}

/**
 * Stop the axis, called by motor Record
 */
asynStatus ecmcMotorRecordAxis::stop(double acceleration) {
  asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
            "%s/%s:%d: Axis[%d] Stop cmd:  acc=%lf\n",
            __FILE__, __FUNCTION__, __LINE__,
            axisNo_, acceleration);

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  return stopAxisInternal(__FUNCTION__, acceleration);
}

void ecmcMotorRecordAxis::callParamCallbacksUpdateError() {
  const char *msgTxtFromDriver = NULL;
  int EPICS_nErrorId           = drvlocal.nErrorIdMcu;

  drvlocal.eeAxisError = eeAxisErrorNoError;

  if (EPICS_nErrorId) {
    /* Error from MCU */
    drvlocal.eeAxisError = eeAxisErrorMCUError;
    msgTxtFromDriver     = &drvlocal.sErrorMessage[0];
  } else if (drvlocal.dirty.sErrorMessage) {
    /* print error below */
    drvlocal.eeAxisError = eeAxisErrorIOCcomError;
  } else if (drvlocal.cmdErrorMessage[0]) {
    drvlocal.eeAxisError = eeAxisErrorCmdError;
    msgTxtFromDriver     = &drvlocal.cmdErrorMessage[0];
  } else if (drvlocal.illegalInTargetWindow) {
    drvlocal.eeAxisError = eeAxisIllegalInTargetWindow;
    msgTxtFromDriver     = "E: InTargetPosWin";
  }

  if ((drvlocal.eeAxisError != drvlocal.old_eeAxisError) ||
      (drvlocal.eeAxisWarning != drvlocal.old_eeAxisWarning) ||
      (drvlocal.nErrorIdEpicsOld != EPICS_nErrorId) ||
      (drvlocal.nCommandActiveOld != drvlocal.nCommandActive)) {
    if (!msgTxtFromDriver && drvlocal.eeAxisWarning) {
      /* No error to show yet */
      switch (drvlocal.eeAxisWarning) {
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
      switch (drvlocal.nCommandActive) {
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

#endif // ifdef motorLatestCommandString
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

    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%spoll(%d) callParamCallbacksUpdateError"
              " Error=%d old=%d ErrID=0x%x old=0x%x Warn=%d nCmd=%d old=%d txt=%s\n",
              modNamEMC,
              axisNo_,
              drvlocal.eeAxisError,
              drvlocal.old_eeAxisError,
              EPICS_nErrorId,
              drvlocal.nErrorIdEpicsOld,
              drvlocal.eeAxisWarning,
              drvlocal.nCommandActive,
              drvlocal.nCommandActiveOld,
              msgTxtFromDriver ? msgTxtFromDriver : "NULL");

    updateMsgTxtFromDriver(msgTxtFromDriver);

    drvlocal.old_eeAxisError   = drvlocal.eeAxisError;
    drvlocal.old_eeAxisWarning = drvlocal.eeAxisWarning;
    drvlocal.nErrorIdEpicsOld  = EPICS_nErrorId;
    drvlocal.nCommandActiveOld = drvlocal.nCommandActive;
  }

  callParamCallbacks();
}

asynStatus ecmcMotorRecordAxis::readEcmcAxisStatusData() {

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  /* Driver not yet initialized, do nothing */
  if (!drvlocal.ecmcAxis->getRealTimeStarted()) {
    if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);
    return asynSuccess;
  }

  drvlocal.ecmcSafetyInterlock = drvlocal.ecmcAxis->getMon()->getSafetyInterlock();
  drvlocal.ecmcBusy = drvlocal.ecmcAxis->getBusy();
  drvlocal.ecmcAtTarget = drvlocal.ecmcAxis->getMon()->getAtTarget();
  drvlocal.ecmcAtTargetMonEnable = drvlocal.ecmcAxis->getMon()->getEnableAtTargetMon();
  drvlocal.axisPrintDbg = drvlocal.ecmcAxis->getPrintDbg();
  drvlocal.axisInStartup = drvlocal.ecmcAxis->getInStartupPhase();
  
  drvlocal.ecmcSummaryInterlock = drvlocal.ecmcAxis->getMon()->getSumInterlock();
  drvlocal.ecmcTrjSrc = drvlocal.ecmcAxis->getTrajDataSourceType() == 
                        ECMC_DATA_SOURCE_EXTERNAL;

  drvlocal.ecmcIgnoreDisableAxisStatus = drvlocal.ecmcAxis->getMRIgnoreDisableStatusCheck();

  // Get values from ecmc
  ecmcAxisDataStatus *tempAxisStat =
    drvlocal.ecmcAxis->getAxisStatusDataPtr();
  if (!tempAxisStat) {
    if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);
    LOGERR(
      "%s/%s:%d: ERROR: function getAxisDebugInfoDataPointer() returned NULL.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }

  // copy data locally
  memcpy(&drvlocal.status_, tempAxisStat, sizeof(ecmcAxisDataStatus));

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  return asynSuccess;
}

/** Polls the axis.
 * This function reads the motor position, the limit status, the home status, the moving status,
 * and the drive power-on status.
 * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
 * and then calls callParamCallbacks() at the end.
 * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus ecmcMotorRecordAxis::poll(bool *moving) {
  double timeBefore = ecmcMotorRecordgetNowTimeSecs();

#ifndef motorWaitPollsBeforeReadyString
  int waitNumPollsBeforeReady_ = drvlocal.waitNumPollsBeforeReady;
#endif // ifndef motorWaitPollsBeforeReadyString

  asynStatus status = readEcmcAxisStatusData();

  if(drvlocal.axisInStartup) {
    return asynError;  
  }

  if(drvlocal.ecmcSummaryInterlock) {

    triggstop_++;
    if(triggstop_ == 0) {
      triggstop_++;
    }
    asynMotorAxis::setIntegerParam(pC_->motorStop_, triggstop_);  // Stop also triggered in ecmc, try to sync motor record and ecmc    
    
    if (!drvlocal.ecmcBusy && drvlocal.ecmcSafetyInterlock) {
      asynMotorAxis::setIntegerParam(pC_->motorClosedLoop_, 0);
    }
    callParamCallbacks();
  }

  //#endif // ifdef POWERAUTOONOFFMODE2

  if (status) {
    return status;
  }

  if (drvlocal.ecmcAxis) {
    if(drvlocal.ecmcAtTargetMonEnable && drvlocal.status_.statusWord_.enabled) {
      drvlocal.moveReady = !drvlocal.ecmcBusy && drvlocal.status_.statusWord_.attarget; //&& !drvlocal.status_.statusWord_.enabled;
    } else {
      drvlocal.moveReady = !drvlocal.ecmcBusy;// && !drvlocal.status_.statusWord_.enabled;
    }
  } else {
    drvlocal.moveReady = false;
  }

  // Axis in external mode and busy, then trigg SYNC
  if((drvlocal.ecmcTrjSrc && drvlocal.ecmcBusy)) {
    triggsync_++;
    asynMotorAxis::setIntegerParam(pC_->ecmcMotorRecordTRIGG_SYNC_,triggsync_);
  }

  setIntegerParam(pC_->motorStatusHomed_,
                  (drvlocal.status_.
                   statusWord_.homed > 0));
  drvlocal.homed = drvlocal.status_.statusWord_.homed > 0;
  setIntegerParam(pC_->motorStatusCommsError_, 0);
  setIntegerParam(pC_->motorStatusAtHome_,
                  (drvlocal.status_.statusWord_.homeswitch >
                   0));
  setIntegerParam(pC_->motorStatusLowLimit_,
                  ((!drvlocal.status_.statusWord_.limitbwd) >
                   0));
  setIntegerParam(pC_->motorStatusHighLimit_,
                  ((!drvlocal.status_.statusWord_.limitfwd) >
                   0));
  setIntegerParam(pC_->motorStatusPowerOn_,
                  (drvlocal.status_.statusWord_.enabled > 0));
  setDoubleParam(pC_->ecmcMotorRecordVelAct_,
                 drvlocal.status_.currentVelocityActual);
  setDoubleParam(pC_->ecmcMotorRecordAcc_RB_,
                 drvlocal.status_.currentAccelerationSetpoint);

  callParamCallbacks();
#ifndef motorWaitPollsBeforeReadyString

  if (drvlocal.waitNumPollsBeforeReady) {
    *moving = true;
  } else
#endif // ifndef motorWaitPollsBeforeReadyString
  {
    *moving = !drvlocal.moveReady;
  }

  if (!drvlocal.moveReady) {
    drvlocal.nCommandActive = drvlocal.status_.command;
  } else {
    drvlocal.nCommandActive = 0;

    if (drvlocal.eeAxisWarning == eeAxisWarningSpeedLimit) {
      drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
    }
  }

  // if (drvlocal.nCommandActive != ECMC_CMD_HOMING) {

  setDoubleParam(pC_->motorPosition_,
                 drvlocal.status_.currentPositionActual);
  setDoubleParam(pC_->motorEncoderPosition_,
                 drvlocal.status_.currentPositionActual);
  setDoubleParam(pC_->ecmcMotorRecordVel_RB_,
                 drvlocal.status_.currentVelocitySetpoint);

  // }
  setDoubleParam(pC_->ecmcMotorRecordEncAct_,
                 (double)drvlocal.statusOld_.currentPositionActualRaw);

  if (drvlocal.statusOld_.statusWord_.homed !=
      drvlocal.status_.statusWord_.homed) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%spoll(%d) homed=%d\n",
              modNamEMC,
              axisNo_,
              drvlocal.status_.statusWord_.homed);
  }

  if (drvlocal.statusOld_.statusWord_.limitbwd !=
      drvlocal.status_.statusWord_.limitbwd) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%spoll(%d) LLS=%d\n",
              modNamEMC,
              axisNo_,
              !drvlocal.status_.statusWord_.limitbwd);
  }

  if (drvlocal.statusOld_.statusWord_.limitfwd !=
      drvlocal.status_.statusWord_.limitfwd) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%spoll(%d) HLS=%d\n",
              modNamEMC,
              axisNo_,
              !drvlocal.status_.statusWord_.limitfwd);
  }

#ifndef motorWaitPollsBeforeReadyString

  if (drvlocal.waitNumPollsBeforeReady) {
    /* Don't update moving, done, motorStatusProblem_ */
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%spoll(%d) mvnNRdyNexAt=%d bBusy=%d bExecute=%d bEnabled=%d atTarget=%d waitNumPollsBeforeReady=%d\n",
              modNamEMC,
              axisNo_,
              !drvlocal.moveReady,
              drvlocal.status_.statusWord_.busy,
              drvlocal.status_.statusWord_.execute,
              drvlocal.status_.statusWord_.enabled,
              drvlocal.status_.statusWord_.attarget,
              drvlocal.waitNumPollsBeforeReady);
    drvlocal.waitNumPollsBeforeReady--;
    callParamCallbacks();
  } else
#endif // ifndef motorWaitPollsBeforeReadyString
  {
    if ((drvlocal.moveReadyOld != drvlocal.moveReady) ||
        (drvlocal.statusOld_.statusWord_.busy     !=
         drvlocal.status_.statusWord_.busy) ||
        (drvlocal.statusOld_.statusWord_.enabled  !=
         drvlocal.status_.statusWord_.enabled) ||
        (drvlocal.statusOld_.statusWord_.execute  !=
         drvlocal.status_.statusWord_.execute) ||
        (drvlocal.statusOld_.statusWord_.attarget !=
         drvlocal.status_.statusWord_.attarget)) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_INFO,
                "%spoll(%d) mvnNRdy=%d bBusy=%d bExecute=%d bEnabled=%d atTarget=%d wf=%d ENC=%g fPos=%g fActPosition=%g time=%f\n",
                modNamEMC,
                axisNo_,
                !drvlocal.moveReady,
                drvlocal.status_.statusWord_.busy,
                drvlocal.status_.statusWord_.execute,
                drvlocal.status_.statusWord_.enabled,
                drvlocal.status_.statusWord_.attarget,
                waitNumPollsBeforeReady_,
                (double)drvlocal.statusOld_.currentPositionActualRaw,
                drvlocal.status_.currentTargetPosition,
                drvlocal.status_.currentPositionActual,
                ecmcMotorRecordgetNowTimeSecs() - timeBefore);
    }
  }
  setIntegerParam(pC_->motorStatusDirection_,
                  drvlocal.status_.currentPositionActual >
                  drvlocal.statusOld_.currentPositionActual ? 1 : 0);

  setIntegerParam(pC_->motorStatusMoving_, !drvlocal.moveReady);
  setIntegerParam(pC_->motorStatusDone_,   drvlocal.moveReady);

  drvlocal.nErrorIdMcu = drvlocal.status_.errorCode;

  if ((drvlocal.bErrorOld !=
       (drvlocal.status_.errorCode > 0)) ||
      (drvlocal.nErrorIdMcuOld != drvlocal.nErrorIdMcu) ||
      drvlocal.dirty.sErrorMessage) {
    char sErrorMessage[256];
    int  nErrorId           = drvlocal.status_.errorCode;
    const char *errIdString = errStringFromErrId(nErrorId);
    sErrorMessage[0]          = '\0';
    drvlocal.sErrorMessage[0] = '\0';
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%spoll(%d) bError=%d drvlocal.status_.errorCode=0x%x\n",
              modNamEMC,
              axisNo_,
              drvlocal.status_.errorCode > 0,
              nErrorId);
    drvlocal.bErrorOld = drvlocal.status_.errorCode >
                         0;
    drvlocal.nErrorIdMcuOld      = nErrorId;
    drvlocal.dirty.sErrorMessage = 0;

    if (nErrorId) {
      /* Get the ErrorMessage to have it in the log file */
      strcpy(&sErrorMessage[0], ecmcError::convertErrorIdToString(nErrorId));
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
                "%ssErrorMessage(%d)=\"%s\"\n",
                modNamEMC, axisNo_, sErrorMessage);
    }

    // Write error string to drvlocal.sErrorMessage

    /* First choice: "well known" ErrorIds */
    size_t bytes = 0;

    if (errIdString[0]) {
      bytes = snprintf(drvlocal.sErrorMessage,
                       sizeof(drvlocal.sErrorMessage) - 1,
                       "E: %s (0x%x)",
                       errIdString,
                       nErrorId);
    } else {
      /* ecmc has error messages */
      bytes = snprintf(drvlocal.sErrorMessage,
                       sizeof(drvlocal.sErrorMessage) - 1,
                       "E: %s (0x%x)",
                       sErrorMessage,
                       nErrorId);
    }

    if (bytes >= sizeof(drvlocal.sErrorMessage) - 1) {
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
                "Warning: Error message trucated (%s)\n", sErrorMessage);
    }

    /* The poller will update the MsgTxt field */

    // updateMsgTxtFromDriver(drvlocal.sErrorMessage);
  }
  #ifdef motorFlagsHomeOnLsString
  setIntegerParam(pC_->motorFlagsHomeOnLs_, 1);
  #endif // ifdef motorFlagsHomeOnLsString

  callParamCallbacksUpdateError();

  drvlocal.moveReadyOld = drvlocal.moveReady;
  memcpy(&drvlocal.statusOld_, &drvlocal.status_,
         sizeof(drvlocal.statusOld_));
  return asynSuccess;
}

/** Set the motor closed loop status
 * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus ecmcMotorRecordAxis::setClosedLoop(bool closedLoop) {
  int value = closedLoop ? 1 : 0;

  asynPrint(pPrintOutAsynUser, ASYN_TRACE_FLOW,
            "%ssetClosedLoop(%d)=%d\n",  modNamEMC, axisNo_, value);

  if (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    return enableAmplifier(value);
  }
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::setIntegerParam(int function, int value) {
  asynStatus status;
  int errorCode = 0;

  if (function == pC_->motorUpdateStatus_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d motorUpdateStatus_)=%d\n",
              modNamEMC,
              axisNo_,
              value);
  } else if (function == pC_->motorStatusCommsError_) {
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_FLOW,
              "%ssetIntegerParam(%d pC_->motorStatusCommsError_)=%d\n",
              modNamEMC, axisNo_, value);

    if (value && !drvlocal.dirty.statusDisconnectedOld) {
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
                "%s Communication error(%d)\n", modNamEMC, axisNo_);
      memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
      drvlocal.nErrorIdMcu = 0;
    }
#ifdef motorPowerAutoOnOffString
  } else if (function == pC_->motorPowerAutoOnOff_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d motorPowerAutoOnOff_)=%d\n",
              modNamEMC,
              axisNo_,
              value);
#endif // ifdef motorPowerAutoOnOffString
  } else if (function == pC_->ecmcMotorRecordHomProc_) {
    int motorNotHomedProblem = 0;
    setIntegerParam(pC_->ecmcMotorRecordHomProc_RB_, value);
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d HomProc_)=%d motorNotHomedProblem=%d\n",
              modNamEMC, axisNo_, value, motorNotHomedProblem);
  } else if (function == pC_->ecmcMotorRecordErrRst_) {
    if (value) {
      asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
                "%ssetIntegerParam(%d ErrRst_)=%d\n",
                modNamEMC, axisNo_, value);

      /*  We do not want to call the base class */
      return resetAxis();
    }

    /* If someone writes 0 to the field, just ignore it */
    return asynSuccess;
  } else if (function == pC_->ecmcMotorRecordCfgDHLM_En_) {
    // Set enable soft limit fwd

    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d ecmcMotorRecordCfgDHLM_En)=%d\n",
              modNamEMC, axisNo_, value);

    if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
    errorCode = drvlocal.ecmcAxis->getMon()->setEnableSoftLimitFwd(value);

    if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

    readBackSoftLimits();
    return errorCode == 0 ? asynSuccess : asynError;
  } else if (function == pC_->ecmcMotorRecordCfgDLLM_En_) {
    // Set enable soft limit bwd

    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d ecmcMotorRecordCfgDLLM_En)=%d\n",
              modNamEMC, axisNo_, value);

    if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
    errorCode = drvlocal.ecmcAxis->getMon()->setEnableSoftLimitBwd(value);

    if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

    readBackSoftLimits();
    return errorCode == 0 ? asynSuccess : asynError;
  }

  // Call base class method
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
asynStatus ecmcMotorRecordAxis::setDoubleParam(int function, double value) {
  asynStatus status;
  int errorCode = 0;

  if (function == pC_->motorMoveRel_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorMoveRel_)=%g\n",
              modNamEMC,
              axisNo_,
              value);
  } else if (function == pC_->motorMoveAbs_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorMoveAbs_)=%g\n",
              modNamEMC,
              axisNo_,
              value);
  } else if (function == pC_->motorMoveVel_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorMoveVel_)=%g\n",
              modNamEMC,
              axisNo_,
              value);
  } else if (function == pC_->motorHome_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorHome__)=%g\n",
              modNamEMC,
              axisNo_,
              value);
  } else if (function == pC_->motorStop_) {
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorStop_)=%g\n", modNamEMC, axisNo_,
              value);
  } else if (function == pC_->motorVelocity_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorVelocity_=%g\n",
              modNamEMC,
              axisNo_,
              value);
  } else if (function == pC_->motorVelBase_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorVelBase_)=%g\n",
              modNamEMC,
              axisNo_,
              value);
  } else if (function == pC_->motorAccel_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorAccel_)=%g\n",
              modNamEMC,
              axisNo_,
              value);
  } else if (function == pC_->motorDeferMoves_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motmotorDeferMoves_=%g\n",
              modNamEMC,
              axisNo_,
              value);
  } else if (function == pC_->motorMoveToHome_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motmotorMoveToHome_=%g\n",
              modNamEMC,
              axisNo_,
              value);
  } else if (function == pC_->motorResolution_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorResolution_=%g\n",
              modNamEMC,
              axisNo_,
              value);

    /* Limits handled above */

#ifdef motorPowerOnDelayString
  } else if (function == pC_->motorPowerOnDelay_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPowerOnDelay_)=%g\n",
              modNamEMC,
              axisNo_,
              value);
#endif // ifdef motorPowerOnDelayString
#ifdef motorPowerOffDelayString
  } else if (function == pC_->motorPowerOffDelay_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPowerOffDelay_=%g\n",
              modNamEMC,
              axisNo_,
              value);
#endif // ifdef motorPowerOffDelayString
#ifdef motorPowerOffFractionString
  } else if (function == pC_->motorPowerOffFraction_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motomotorPowerOffFraction_=%g\n",
              modNamEMC,
              axisNo_,
              value);
#endif // ifdef motorPowerOffFractionString
#ifdef motorPostMoveDelayString
  } else if (function == pC_->motorPostMoveDelay_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPostMoveDelay_=%g\n",
              modNamEMC,
              axisNo_,
              value);
#endif // ifdef motorPostMoveDelayString
  } else if (function == pC_->motorStatus_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorStatus_)=%g\n",
              modNamEMC,
              axisNo_,
              value);
#ifdef ecmcMotorRecordHVELFRMString
  } else if (function == pC_->ecmcMotorRecordHVELfrm_) {
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d HVELfrm_)=%g\n", modNamEMC, axisNo_, value);
#endif // ifdef ecmcMotorRecordHVELFRMString
#ifdef ecmcMotorRecordHomPosString
  } else if (function == pC_->ecmcMotorRecordHomPos_) {
    pC_->setDoubleParam(axisNo_, pC_->ecmcMotorRecordHomPos_RB_, value);
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d HomPos_)=%f\n", modNamEMC, axisNo_, value);
#endif // ifdef ecmcMotorRecordHomPosString
  } // Set soft limit fwd
  else if (function == pC_->ecmcMotorRecordCfgDHLM_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgDHLM_)=%f\n",
              modNamEMC,
              axisNo_,
              value);

    if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
    errorCode = drvlocal.ecmcAxis->getMon()->setSoftLimitFwd(value);

    if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

    readBackSoftLimits();
    return errorCode == 0 ? asynSuccess : asynError;
  } // Set soft limit bwd
  else if (function == pC_->ecmcMotorRecordCfgDLLM_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgDLLM_)=%f\n",
              modNamEMC,
              axisNo_,
              value);

    if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
    errorCode = drvlocal.ecmcAxis->getMon()->setSoftLimitBwd(value);

    if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

    readBackSoftLimits();
    return errorCode == 0 ? asynSuccess : asynError;
  } // manual velo fast.. Just store here in "motor record" driver
  else if (function == pC_->ecmcMotorRecordCfgVELO_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgVELO_)=%f\n",
              modNamEMC,
              axisNo_,
              value);

    drvlocal.manualVelocFast = value;
    return asynSuccess;
  } // Set monitor max velocity
  else if (function == pC_->ecmcMotorRecordCfgVMAX_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgVMAX_)=%f\n",
              modNamEMC,
              axisNo_,
              value);

    if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
    errorCode = drvlocal.ecmcAxis->getMon()->setMaxVel(value);

    if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

    return errorCode == 0 ? asynSuccess : asynError;
  } // manual velo fast.. Just store here in "motor record" driver
  else if (function == pC_->ecmcMotorRecordCfgJVEL_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgJVEL_)=%f\n",
              modNamEMC,
              axisNo_,
              value);
    drvlocal.manualVelocSlow = value;
    return asynSuccess;
  } // Set acceleration
  else if (function == pC_->ecmcMotorRecordCfgACCS_) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgACCS_)=%f\n",
              modNamEMC,
              axisNo_,
              value);

    if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
    drvlocal.ecmcAxis->getSeq()->setAcc(value);

    if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

    return asynSuccess;
  }

  // Call the base class method
  status = asynMotorAxis::setDoubleParam(function, value);
  return status;
}

#ifndef motorMessageTextString
void ecmcMotorRecordAxis::updateMsgTxtFromDriver(const char *value) {
  if (value && value[0]) {
    setStringParam(pC_->ecmcMotorRecordMCUErrMsg_, value);
  } else {
    setStringParam(pC_->ecmcMotorRecordMCUErrMsg_, "");
  }
}

#endif // ifndef motorMessageTextString

/** Set the high limit position of the motor.
  * \param[in] highLimit The new high limit position that should be set in the hardware. Units=steps.*/
asynStatus ecmcMotorRecordAxis::setHighLimit(double highLimit) {
  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  int errorCode = drvlocal.ecmcAxis->getMon()->setSoftLimitFwd(highLimit);

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  if (errorCode) {
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
              "%ssetHighLimit(%d)=%lf\n", modNamEMC, axisNo_, highLimit);

    return asynError;
  }
  return asynMotorAxis::setHighLimit(highLimit);
}

/** Set the low limit position of the motor.
  * \param[in] lowLimit The new low limit position that should be set in the hardware. Units=steps.*/
asynStatus ecmcMotorRecordAxis::setLowLimit(double lowLimit) {
  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  int errorCode = drvlocal.ecmcAxis->getMon()->setSoftLimitBwd(lowLimit);

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  if (errorCode) {
    asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO,
              "%ssetLowLimit(%d)=%lf\n", modNamEMC, axisNo_, lowLimit);

    return asynError;
  }
  return asynMotorAxis::setLowLimit(lowLimit);
}

asynStatus ecmcMotorRecordAxis::initializeProfile(size_t maxProfilePoints)
{
  printf("ecmcMotorRecordAxis::initializeProfile()\n");
  if(!pvtEnabled_) {
    printf("ecmcMotorRecordAxis::initializeProfile(): INFO axis[%d]: PVT not enabled\n",axisNo_);
    return asynSuccess;
  }
  profileMaxPoints_ = maxProfilePoints;
  profileLastInitOk_= false;
  asynStatus status = asynMotorAxis::initializeProfile(maxProfilePoints);
  
  profileLastInitOk_ = status == asynSuccess;
  
  // Add axis ref to PVT controller
  ecmcPVTController * pvtCtrl = pC_->getPVTController();
  if(!pvtCtrl) {
    printf("ecmcMotorRecordAxis::initializeProfile(): Error axis[%d]: ecmcPVTController == NULL\n", axisNo_);
    return asynError;
  }
  
  // Add axes to PVT (since !pvtEnabled_)
  //pvtCtrl->addPVTAxis(drvlocal.ecmcAxis);
  
  return status;
}

asynStatus ecmcMotorRecordAxis::defineProfile(double *positions, size_t numPoints)
{
  printf("ecmcMotorRecordAxis::defineProfile()\n");
  if(!pvtEnabled_) {
    printf("ecmcMotorRecordAxis::defineProfile(): INFO axis[%d]: PVT not enabled\n",axisNo_);
    return asynSuccess;
  }

  profileLastDefineOk_= false;
  pC_->setIntegerParam(pC_->profileBuildState_, PROFILE_BUILD_DONE);
  pC_->setIntegerParam(pC_->profileBuildStatus_, PROFILE_STATUS_UNDEFINED);
  sprintf(profileMessage_, "Build not executed.\n");
  pC_->setStringParam(pC_->profileBuildMessage_, profileMessage_);
  pC_->callParamCallbacks();

  size_t i;
  asynStatus status;

  // Call the base class function
  status = asynMotorAxis::defineProfile(positions, numPoints);
  if (status) {
    printf("ecmcMotorRecordAxis::defineProfile(): ERROR!!!\n");
    return status;    
  }

  for (i = 0; i < numPoints; i++) {    
    profilePositions_[i] = positions[i];
  }

  profileCurrentDefinedPoints_ = numPoints;

  profileLastDefineOk_ = status == asynSuccess;
  for (size_t i = 0; i < (profileCurrentDefinedPoints_); i++) {
    if(drvlocal.axisPrintDbg) {
      printf("ecmcMotorRecordAxis::defineProfile: profilePositions_[%ld] = %lf\n",i,profilePositions_[i]);
    }
  }

  return asynSuccess;
}

/** Function to build a coordinated move of multiple axes. */
asynStatus ecmcMotorRecordAxis::buildProfile()
{
  printf("ecmcMotorRecordAxis::buildProfile()\n");
  if(!pvtEnabled_) {
    printf("ecmcMotorRecordAxis::buildProfile(): INFO axis[%d]: PVT not enabled\n",axisNo_);
    return asynSuccess;
  }

  if(profileInProgress_) {
    printf("ecmcMotorRecordAxis::buildProfile(): INFO axis[%d]: Profile in progress\n",axisNo_);
    return asynError;
  }

  profileLastBuildOk_ = false;
  asynMotorAxis::buildProfile();

  if (!profileLastInitOk_ || !profileLastDefineOk_) {
     LOGERR(
      "%s/%s:%d: ERROR: Define or Init not performed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);

    return asynError;
  }
  
  if(profileCurrentDefinedPoints_<2) {
     LOGERR(
      "%s/%s:%d: ERROR: Defined profile position count invalid (<=1).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }

  double accTime = 0;
  asynStatus status = pC_->getDoubleParam(pC_->profileAcceleration_, &accTime);
  
  if(status != asynSuccess) {
     LOGERR(
      "%s/%s:%d: ERROR: Failed read profileAcceleration_.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }

  int pointsCount = 0;
  status = pC_->getIntegerParam(pC_->profileNumPoints_, &pointsCount);

  if(status != asynSuccess) {
     LOGERR(
      "%s/%s:%d: ERROR: Failed read profileNumPoints_.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }

  if((size_t)pointsCount > profileCurrentDefinedPoints_) {
     LOGERR(
      "%s/%s:%d: ERROR: Point count larger than defined point count (%d > %zu).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      pointsCount,
      profileCurrentDefinedPoints_);
    return asynError;
  }

  if((size_t)pointsCount <= 0 || (size_t)pointsCount > profileMaxPoints_) {
     LOGERR(
      "%s/%s:%d: ERROR: Invalid point count, must be 1..%zu.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      profileMaxPoints_);
    return asynError;
  }

  if(!pvtPrepare_) {       
    pvtPrepare_ = new ecmcAxisPVTSequence(getEcmcSampleTimeMS()/1000, profileMaxPoints_);
  }

  if(!pvtPrepare_ ) {
     LOGERR(
      "%s/%s:%d: ERROR: Allocation of PVT object failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__);
    return asynError;
  }
  
  // Clear prepared pvt object
  pvtPrepare_->clear();

  double time;
  int timeMode;
  
  pC_->getIntegerParam(ECMC_MR_CNTRL_ADDR, pC_->profileTimeMode_, &timeMode);
  pC_->getDoubleParam(ECMC_MR_CNTRL_ADDR, pC_->profileFixedTime_, &time);

  if(drvlocal.axisPrintDbg) {
    if(timeMode==PROFILE_TIME_MODE_FIXED) {
      printf("TIME_MODE=PROFILE_TIME_MODE_FIXED, time %lf\n",time);
    } else {
      printf("TIME_MODE=PROFILE_TIME_MODE_ARRAY\n");
    }
  
    for (int i = 0; i < pointsCount; i++) {
      if(timeMode == PROFILE_TIME_MODE_FIXED) {
      printf("time[%d] = %lf \n",i, time);
      } else { // Time array
        printf("pC->profileTimes_[%d] = %lf \n",i, pC->profileTimes_[i]);
      }
      printf("profilePositions_[%d] = %lf \n",i, profilePositions_[i]);
    }
  }

  double preVelo = 0;
  double postVelo = 0;
  double currTime = accTime;  // Start at this time since first segment takes the acceleration

  if(timeMode == PROFILE_TIME_MODE_FIXED) {      
    preVelo = (profilePositions_[1]-profilePositions_[0]) / time;
  } else { // Time array
    preVelo = (profilePositions_[1]-profilePositions_[0]) / pC->profileTimes_[0];
  }

  // Add pre-point for acceleration
  double distAcc = preVelo * accTime / 2; 
  pvtPrepare_->addPoint(new ecmcPvtPoint(profilePositions_[0] - distAcc,0,0));

  if(drvlocal.axisPrintDbg) {
    printf("Added pre-point for acc (%lf,%lf,%lf)\n",profilePositions_[0]- distAcc, preVelo, 0.0);
  }

  // First point
  pvtPrepare_->addPoint(new ecmcPvtPoint(profilePositions_[0], preVelo, currTime));
  
  if(drvlocal.axisPrintDbg) {
    printf("Added point (%lf,%lf,%lf)\n",profilePositions_[0], preVelo, currTime);
  }

  if(timeMode == PROFILE_TIME_MODE_FIXED) {
    currTime += time;
  } else { // Time array
    currTime += pC->profileTimes_[0];
  }

  // Add center points
  double velo = preVelo;
  for (int i = 1; i < (pointsCount-1); i++) {
    if(timeMode == PROFILE_TIME_MODE_FIXED) {      
      postVelo = (profilePositions_[i+1]-profilePositions_[i]) / time;
    } else { // Time array
      postVelo = (profilePositions_[i+1]-profilePositions_[i]) / pC->profileTimes_[i];
    }

    velo = (preVelo + postVelo)/2;
    pvtPrepare_->addPoint(new ecmcPvtPoint(profilePositions_[i],velo, currTime));
    if(drvlocal.axisPrintDbg) {
      printf("Added point (%lf,%lf,%lf)",profilePositions_[i], velo, currTime);
    }
    if(timeMode == PROFILE_TIME_MODE_FIXED) {
      currTime += time;
    } else { // Time array
      currTime += pC->profileTimes_[i];
    }
    preVelo   = postVelo;
  }
  
  // Add last point. same velo as prev point    
  pvtPrepare_->addPoint(new ecmcPvtPoint(profilePositions_[pointsCount-1], postVelo, currTime));  
  if(drvlocal.axisPrintDbg) {
    printf("Added point (%lf,%lf,%lf)",profilePositions_[pointsCount-1], postVelo, currTime);
  }

  currTime +=accTime;

  // Add post-point for deceleration
  distAcc = postVelo * accTime / 2;
  pvtPrepare_->addPoint(new ecmcPvtPoint(profilePositions_[pointsCount-1] + distAcc, 0.0, currTime));
 
  if(drvlocal.axisPrintDbg) {
    printf("Added post-point for dec (%lf,%lf,%lf)\n",profilePositions_[pointsCount-1] + distAcc, preVelo, currTime);
  }

  // Dump what we have
  pvtPrepare_->print();

  if(!drvlocal.ecmcAxis || !pC_->getPVTController()) {
    return asynError;
  }
  
  profileLastBuildOk_ = true;
  profileSwitchPVTObject_ = true;

  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::executeProfile() {
  if(drvlocal.axisPrintDbg) {
    printf("ecmcMotorRecordAxis::executeProfile()\n");
  }
  if(!pvtEnabled_) {
    printf("ecmcMotorRecordAxis::executeProfile(): INFO axis[%d]: PVT not enabled\n",axisNo_);
    return asynSuccess;
  }
  int useAxis = 0;
  asynStatus status = pC_->getIntegerParam(axisNo_, pC_->profileUseAxis_, &useAxis);  
  
  if(useAxis == 0) {
    printf("ecmcMotorRecordAxis::executeProfile(): Info axis[%d]: Axis not in use (ignoring execute command).\n", axisNo_);
    return asynSuccess;
  }

  if(!profileLastBuildOk_) {
    printf("ecmcMotorRecordAxis::executeProfile(): Error axis[%d]: Last build did not complete successfully\n", axisNo_);
    return asynError;
  }
  
  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);

  if(pvtRunning_) {
    if(pvtRunning_->getBusy()) {
      printf("ecmcMotorRecordAxis::executeProfile(): Error axis[%d]: Profile busy..\n",axisNo_);
      if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);
      return asynError;
    }
  }

  //Add axis to pvt controller
  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  pC_->getPVTController()->addAxis(drvlocal.ecmcAxis);
  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);
  
  if(profileSwitchPVTObject_) {
    // switch pvt objects
    ecmcAxisPVTSequence *pvtTempSwitch = NULL;
    pvtTempSwitch = pvtRunning_;
    pvtRunning_ = pvtPrepare_;
    
    // Add pvt object to axis
    drvlocal.ecmcAxis->getSeq()->setPVTObject(pvtRunning_);
    pvtPrepare_ = pvtTempSwitch;
    profileSwitchPVTObject_ = false;
  }
  
  // Add pvt object to controller
  //pC_->getPVTController()->addAxis(pvtRunning_);

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  if(drvlocal.axisPrintDbg) {
    printf("ecmcMotorRecordAxis::executeProfile()\n");
  }
  status = asynMotorAxis::executeProfile();
  if(status != asynSuccess) {
    return status;
  }

  if(!profileLastBuildOk_) {
    printf("ecmcMotorRecordAxis::executeProfile(): Error axis[%d]: Last build did not complete successfully\n", axisNo_);
    return asynError;
  }

  int mode = 0;
  status = pC_->getIntegerParam(pC_->profileMoveMode_, &mode);

  // Check for interlock
  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  int ilock=drvlocal.ecmcAxis->getSumInterlock();
  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);
  if(ilock) {
    printf("ecmcMotorRecordAxis::executeProfile(): Error Axis[%d]: Axis interlocked, aborting profile...\n",
    drvlocal.axisId);
    abortProfile();
    return asynError;
  }

  // The actual execute is handled from ecmcPVTController in ecmcMotorRecordController::executeProfile
  //int errorCode = 0;
  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);

  pvtRunning_->setRelMode(mode==PROFILE_MOVE_MODE_RELATIVE) ;

  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  profileInProgress_ = true;
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::abortProfile() {
  printf("ecmcMotorRecordAxis::abortProfile()\n");
  profileInProgress_ = false;

  // Stop is called from ecmcMotorRecordController()::abortProfile()

  asynStatus status = asynMotorAxis::abortProfile();
  if(status != asynSuccess) {
    return status;
  }
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::readbackProfile() {
  int status = 0;

  if(!pvtRunning_) {
    printf("!pvtRunning_\n");
    return asynError;
  }

  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);

  // Get pointers to data of ecmc vectors
  double *dataPosAct =  pvtRunning_->getResultPosActDataPrt();
  double *dataPosErr =  pvtRunning_->getResultPosErrDataPrt();
  size_t elements = pvtRunning_->getResultBufferSize();

  if(dataPosAct == NULL || dataPosAct == NULL) {
    if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);
    printf("!pointer NULL_\n");
    return asynError;
  }

  if(elements == 0) {
    if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);
    printf("!elements 0_\n");
    return asynError;
  }

  if(elements > profileMaxPoints_) {
    elements = profileMaxPoints_;
  }

  memcpy(profileReadbacks_,       dataPosAct, elements * sizeof(double));
  memcpy(profileFollowingErrors_, dataPosErr, elements * sizeof(double));
  
  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  status  = pC_->doCallbacksFloat64Array(profileReadbacks_,       elements, pC_->profileReadbacks_, axisNo_);
  status |= pC_->doCallbacksFloat64Array(profileFollowingErrors_, elements, pC_->profileFollowingErrors_, axisNo_);
  return asynSuccess;
}


bool ecmcMotorRecordAxis::getProfileLastBuildSuccess() {
  return profileLastBuildOk_;
}

asynStatus ecmcMotorRecordAxis::checkProfileStatus() {

  // Check for errors
  if(drvlocal.nErrorIdMcu) {
    printf("ecmcMotorRecordController::checkProfileStatus(): Error Axis[%d]: Axis in error state, aborting profile... 0x%x\n",
           drvlocal.axisId,drvlocal.nErrorIdMcu);
    //abortProfile();  // handle in controller
    return asynError;
  }
  
  // Check for interlock
  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  int ilock=drvlocal.ecmcAxis->getSumInterlock();
  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);
  if(ilock) {
    printf("ecmcMotorRecordController::checkProfileStatus(): Error Axis[%d]: Axis interlocked, aborting profile...\n",
           drvlocal.axisId);
    //abortProfile();  // handle in controller
    return asynError;
  }
  return asynSuccess;
}

int ecmcMotorRecordAxis::getProfileCurrentSegementID() {
  int id = -1;
  if(!pvtRunning_ || !profileInProgress_ ) {
    return id; //Invalid..
  }
  
  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
    id = pvtRunning_->getCurrentSegementId();
  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  return id;
}

int ecmcMotorRecordAxis::getProfileBusy() {
  int busy = -1;
  if(!pvtRunning_ || !profileInProgress_ ) {
    return busy;  //Invalid..
  }
  
  if (ecmcRTMutex)epicsMutexLock(ecmcRTMutex);
  busy = pvtRunning_->getBusy();
  if (ecmcRTMutex)epicsMutexUnlock(ecmcRTMutex);

  return busy;
}

void ecmcMotorRecordAxis::setEnablePVTFunc(int enable) {
  pvtEnabled_ = enable;
}

void ecmcMotorRecordAxis::invalidatePVTBuild() {
  profileLastBuildOk_ = false;
}

bool ecmcMotorRecordAxis::getPVTEnabled() {
  return pvtEnabled_;
}

size_t ecmcMotorRecordAxis::getProfilePointCount() {
  return profileCurrentDefinedPoints_;
}
