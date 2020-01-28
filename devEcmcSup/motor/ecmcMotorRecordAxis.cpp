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

/* Callback for axis status data struct (binary)
*/
/*void statBinCallback(void *userPvt, asynUser *pasynUser, epicsInt8 *value, size_t nelements) {
//  printf("diagBinCallback()\n");
  ecmcMotorRecordAxis * axis = (ecmcMotorRecordAxis*)userPvt;
  if(!axis) {
    printf("%s/%s:%d: Error: Axis not found.\n", 
           __FILE__,
           __FUNCTION__,
           __LINE__);
    return;
  }
  if(nelements!=sizeof(ecmcAxisStatusType)) {
    printf("%s/%s:%d: Error: Wrong byte count (sizeof(value)!=(sizeof(ecmcAxisStatusType)).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    return;
  }
  axis->statBinDataCallback(value);  
}*/

/* Callback for axis control data struct (binary)
*/
/*void contBinCallback(void *userPvt, asynUser *pasynUser, epicsInt8 *value, size_t nelements) {
//  printf("diagBinCallback()\n");
  ecmcMotorRecordAxis * axis = (ecmcMotorRecordAxis*)userPvt;
  if(!axis) {
    printf("%s/%s:%d: Error: Axis not found.\n", 
           __FILE__,
           __FUNCTION__,
           __LINE__);
    return;
  }
  if(nelements!=sizeof(ecmcAsynClinetCmdType)) {
    printf("%s/%s:%d: Error: Wrong byte count (sizeof(value)!=(sizeof(ecmcAxisStatusType)).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    return;
  }
  axis->contBinDataCallback(value);  
}*/

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
  
  // ECMC  
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
  oldPositionAct_ = 0;
  
  /*asynUserStatBin_      = NULL; // "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnosticbin?"
  asynUserStatBinIntr_  = NULL; // "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnosticbin?"  
  asynUserCntrlBin_     = NULL; // "T_SMP_MS=%d/TYPE=asynInt32/ax%d.controlstruct="
  asynUserCntrlBinIntr_ = NULL; // "T_SMP_MS=%d/TYPE=asynInt32/ax%d.controlstruct?"

  pasynIFStatBinIntr_   = NULL;
  pIFStatBinIntr_       = NULL;
  interruptStatBinPvt_  = NULL;


  pasynIFContBinIntr_   = NULL;
  pIFContBinIntr_       = NULL;
  interruptContBinPvt_  = NULL;

  memset(&statusBinData_,0, sizeof(statusBinData_));
  memset(&controlBinData_,0, sizeof(ecmcAsynClinetCmdType));
  memset(&controlBinDataRB_,0, sizeof(ecmcAsynClinetCmdType));*/
  
  axisId_ = axisNo;
  
  
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
    const char * const encoder_is_str = "encoder=";
    const char * const cfgfile_str = "cfgFile=";
    //const char * const cfgDebug_str = "getDebugText=";
#ifndef motorFlagsDriverUsesEGUString
    /* The non-ESS motor needs a dummy "stepm-size" to compensate for MRES */
    const char * const stepSize_str = "stepSize=";
#endif
    const char * const homProc_str 	   = "HomProc=";
    const char * const homPos_str  	   = "HomPos=";
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
      }  else if (!strncmp(pThisOption, cfgfile_str, strlen(cfgfile_str))) {
        pThisOption += strlen(cfgfile_str);
        drvlocal.cfgfileStr = strdup(pThisOption);
      // } else if (!strncmp(pThisOption, cfgDebug_str, strlen(cfgDebug_str))) {
      //   pThisOption += strlen(cfgDebug_str);
      //   drvlocal.cfgDebug_str = strdup(pThisOption);
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

  /*asynStatus status = connectEcmcAxis();
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "connectEcmcAxis() failed on port %s\n",pC_->mcuPortName_);
  }*/
  
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

// asynStatus ecmcMotorRecordAxis::readBackHoming(void)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);

//   asynStatus status;
//   int    homProc = 0;
//   double homPos  = 0.0;

//   /* Don't read it, when the driver has been configured with HomProc= */
//   status = pC_->getIntegerParam(axisNo_, pC_->ecmcMotorRecordHomProc_,&homProc);

//   if (status != asynSuccess) {
//         LOGERR(
//       "%s/%s:%d: ERROR: No valid homing procedure defined.\n",
//       __FILE__,
//       __FUNCTION__,
//       __LINE__);
//      return status;     
//   }

//   status = pC_->getIntegerParam(axisNo_, pC_->ecmcMotorRecordHomPos_,&homPos);
//   if (status != asynSuccess) {
//         LOGERR(
//       "%s/%s:%d: ERROR: No valid homing position defined.\n",
//       __FILE__,
//       __FUNCTION__,
//       __LINE__);
//      return status;     
//   }

//   return asynSuccess;
// }

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

  // Why not needed?
  // drvlocal.scaleFactor = num / denom;

  updateCfgValue(pC_->ecmcMotorRecordCfgSREV_RB_, denom, "srev");
  updateCfgValue(pC_->ecmcMotorRecordCfgUREV_RB_, num, "urev");
  return asynSuccess;
}

asynStatus ecmcMotorRecordAxis::readMonitoring(int axisID)
{
  printf("Dbg################# %s\n",__FUNCTION__);

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

// asynStatus ecmcMotorRecordAxis::readBackEncoders(int axisID)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);

//   /* https://infosys.beckhoff.com/english.php?content=../content/1033/tcadsdevicenc2/index.html&id= */
//   asynStatus status;
//   uint32_t numEncoders = 0;
//   int nvals;



//   snprintf(pC_->outString_, sizeof(pC_->outString_),
//            "ADSPORT=501/.ADR.16#%X,16#%X,8,19?;",
//            0x4000 + axisID, 0x57
//            );
//   status = pC_->writeReadOnErrorDisconnect();
//   if (status) return status;
//   nvals = sscanf(pC_->inString_, "%u",
//                  &numEncoders);
//   if (nvals != 1) {
//     asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//               "%snvals=%d command=\"%s\" response=\"%s\"\n",
//               modNamEMC, nvals, pC_->outString_, pC_->inString_);
//     return asynError;
//   }
//   asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
//             "%sreadBackEncoders(%d) numEncoders=%u\n",
//             modNamEMC, axisNo_, numEncoders);
//   return asynSuccess;
// }

asynStatus ecmcMotorRecordAxis::initialPoll(void)
{
  printf("Dbg################# %s\n",__FUNCTION__);

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
  // /* for ECMC homing is configured from EPICS, do NOT do the readback */
  // if (!(pC_->features_ & FEATURE_BITS_ECMC)) {
  //   if (!drvlocal.scaleFactor) {
  //     asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
  //               "%sreadBackAllConfig(%d) drvlocal.scaleFactor=0.0\n",
  //               modNamEMC, axisNo_);
  //     return asynError;
  //   }
  //   if (status == asynSuccess) status = readBackHoming();
  // }
  if (status == asynSuccess) status = readScaling(axisID);
  if (status == asynSuccess) status = readMonitoring(axisID);
  if (status == asynSuccess) status = readBackSoftLimits();
  if (status == asynSuccess) status = readBackVelocities(axisID);
  // if (!(pC_->features_ & FEATURE_BITS_ECMC)) {
  //   if (status == asynSuccess) status = readBackEncoders(axisID);
  // }
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

  // status = readConfigFile();
  // if (status) {
  //   asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
  //             "%s(%d) readConfigFile() failed\n",
  //             modNamEMC, axisNo_);
  //   updateMsgTxtFromDriver("ConfigError Config File");
  //   return status;
  // }
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

  // if (drvlocal.supported.statusVer == -1) {
  //   callParamCallbacksUpdateError();
  //   return asynSuccess;
  // }
  
  // ---------------------ECMC

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

  // ---------------------ECMC

  /*if (drvlocal.cfgDebug_str) {
    printf("Dbg################# 2 %s\n",__FUNCTION__);
    asynStatus comStatus;
    snprintf(pC_->outString_, sizeof(pC_->outString_), "%s", drvlocal.cfgDebug_str);
    comStatus = pC_->writeReadOnErrorDisconnect();
    if (!comStatus) {
      printf("Dbg################# 3 %s\n",__FUNCTION__);
      updateMsgTxtFromDriver(pC_->inString_);
    }
  }*/

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

  /*if (drvlocal.externalEncoderStr) {
    comStatus = getValueFromController(drvlocal.externalEncoderStr,
                                       &st_axis_status.encoderRaw);
    if (!comStatus) setDoubleParam(pC_->ecmcMotorRecordEncAct_,
                                   st_axis_status.encoderRaw);
  } else if (pC_->features_ & FEATURE_BITS_V2) {*/
  setDoubleParam(pC_->ecmcMotorRecordEncAct_, st_axis_status.encoderRaw);
  //}

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

//  if (drvlocal.cfgDebug_str) {
//    ; /* Do not do the following */
 /* } else*/
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
      
      // (void)getStringFromAxis("sErrorMessage", (char *)&sErrorMessage[0],
      //                         sizeof(sErrorMessage));
      
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
    //status = setSAFValueOnAxis(indexGroup5000, 0xC, value);
    readBackSoftLimits();
    return errorCode == 0 ? asynSuccess : asynError;

  }
  else if (function == pC_->ecmcMotorRecordCfgDLLM_En_) {
    // Set enable soft limit bwd
    
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d ecmcMotorRecordCfgDLLM_En)=%d\n",
              modNamEMC, axisNo_, value);
    
    errorCode = ecmcAxis_->getMon()->setEnableSoftLimitBwd(value);
    // status = setSAFValueOnAxis(indexGroup5000, 0xB, value);
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
    //status = setSAFValueOnAxis(indexGroup5000, 0xE, value);
    readBackSoftLimits();
    return errorCode==0 ? asynSuccess : asynError;

  } // Set soft limit bwd
  else if (function == pC_->ecmcMotorRecordCfgDLLM_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgDLLM_)=%f\n", modNamEMC, axisNo_, value);
    
    errorCode = ecmcAxis_->getMon()->setSoftLimitBwd(value);
    //status = setSAFValueOnAxis(indexGroup5000, 0xD, value);
    readBackSoftLimits();
    return errorCode==0 ? asynSuccess : asynError;

  } // manual velo fast.. Just store here in "motor record" driver
  else if (function == pC_->ecmcMotorRecordCfgVELO_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgVELO_)=%f\n", modNamEMC, axisNo_, value);
  
    manualVelocFast_ = value;
    //status = setSAFValueOnAxis(0x4000, 0x9, value);
    return asynSuccess;

  } // Set monitor max velocity
  else if (function == pC_->ecmcMotorRecordCfgVMAX_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgVMAX_)=%f\n", modNamEMC, axisNo_, value);
    errorCode = ecmcAxis_->getMon()->setMaxVel(value);
    //status = setSAFValueOnAxis(0x4000, 0x27, value);
    return errorCode==0 ? asynSuccess : asynError;

  } // manual velo fast.. Just store here in "motor record" driver
  else if (function == pC_->ecmcMotorRecordCfgJVEL_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgJVEL_)=%f\n", modNamEMC, axisNo_, value);    
    manualVelocSlow_ = value;
    //status = setSAFValueOnAxis(0x4000, 0x8, value);
    return asynSuccess;

  } // Set acceleration
  else if (function == pC_->ecmcMotorRecordCfgACCS_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ecmcMotorRecordCfgACCS_)=%f\n", modNamEMC, axisNo_, value);
    ecmcAxis_->getTraj()->setAcc(value);
    //status = setSAFValueOnAxis(0x4000, 0x101, value);
    return asynSuccess;

  }
  // Call the base class method
  status = asynMotorAxis::setDoubleParam(function, value);
  return status;
}

// asynStatus ecmcMotorRecordAxis::setStringParamDbgStrToMcu(const char *value)
// {
//   const char * const Main_this_str = "Main.this.";

//   asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
//             "%ssetStringParamDbgStrToMcu(%d)=\"%s\"\n",
//             modNamEMC, axisNo_, value);
//   /* empty strings are not send to the controller */
//   if (!value[0]) return asynSuccess;

//   /* Check the string. E.g. Main.this. and Sim.this. are passed
//      as Main.M1 or Sim.M1
//      ADR commands are handled below */
//   if (!strncmp(value, Main_this_str, strlen(Main_this_str))) {
//     snprintf(pC_->outString_, sizeof(pC_->outString_), "%sMain.M%d.%s",
//              drvlocal.adsport_str, axisNo_, value + strlen(Main_this_str));
//     return pC_->writeReadACK(ASYN_TRACE_INFO);
//   }
//   /* If we come here, the command was not understood */
//   return asynError;
// }

// asynStatus ecmcMotorRecordAxis::setStringParam(int function, const char *value)
// {
//   if (function == pC_->ecmcMotorRecordDbgStrToLog_) {
//     asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
//               "%ssetStringParamDbgStrToLog(%d)=\"%s\"\n",
//               modNamEMC, axisNo_, value);
//   } else if (function == pC_->ecmcMotorRecordDbgStrToMcu_) {
//     return setStringParamDbgStrToMcu(value);
//   }
//   /* Call base class method */
//   return asynMotorAxis::setStringParam(function, value);
// }

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


// asynStatus ecmcMotorRecordAxis::readConfigLine(const char *line, const char **errorTxt_p)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   const char *setRaw_str = "setRaw "; /* Raw is Raw */
//   const char *setValue_str = "setValue "; /* prefixed with ADSPORT */
//   const char *setADRinteger_str = "setADRinteger ";
//   const char *setADRdouble_str  = "setADRdouble ";
//   const char *setSim_str = "setSim ";

//   asynStatus status = asynError;
//   const char *errorTxt = NULL;

//   while (*line == ' ') line++;
//   if (line[0] == '#') {
//     /*  Comment line */
//     return asynSuccess;
//   }

//   if (!strncmp(setRaw_str, line, strlen(setRaw_str))) {
//     const char *cfg_txt_p = &line[strlen(setRaw_str)];
//     while (*cfg_txt_p == ' ') cfg_txt_p++;

//     snprintf(pC_->outString_, sizeof(pC_->outString_), "%s", cfg_txt_p);
//     status = pC_->writeReadACK(ASYN_TRACE_INFO);
//   } else if (!strncmp(setValue_str, line, strlen(setValue_str))) {
//     const char *cfg_txt_p = &line[strlen(setValue_str)];
//     while (*cfg_txt_p == ' ') cfg_txt_p++;

//     snprintf(pC_->outString_, sizeof(pC_->outString_), "%s%s",
//              drvlocal.adsport_str, cfg_txt_p);
//     status = pC_->writeReadACK(ASYN_TRACE_INFO);
//   } else if (!strncmp(setSim_str, line, strlen(setSim_str))) {
//     if (pC_->features_ & FEATURE_BITS_SIM) {
//       const char *cfg_txt_p = &line[strlen(setRaw_str)];
//       while (*cfg_txt_p == ' ') cfg_txt_p++;

//       snprintf(pC_->outString_, sizeof(pC_->outString_),
//                "Sim.M%d.%s", axisNo_, cfg_txt_p);
//       status = pC_->writeReadACK(ASYN_TRACE_INFO);
//     } else {
//       status = asynSuccess;
//     }
//   } else if (!strncmp(setADRinteger_str, line, strlen(setADRinteger_str))) {
//     unsigned indexGroup;
//     unsigned indexOffset;
//     int value;
//     int nvals = 0;
//     const char *cfg_txt_p = &line[strlen(setADRinteger_str)];
//     while (*cfg_txt_p == ' ') cfg_txt_p++;
//     nvals = sscanf(cfg_txt_p, "%x %x %d",
//                    &indexGroup, &indexOffset, &value);
//     if (nvals == 3) {
//       status = setSAFValueOnAxisVerify(indexGroup, indexOffset, value, 1);
//     } else {
//       errorTxt = "Need 4 values";
//     }
//   } else if (!strncmp(setADRdouble_str, line, strlen(setADRdouble_str))) {
//     unsigned indexGroup;
//     unsigned indexOffset;
//     double value;
//     int nvals = 0;
//     const char *cfg_txt_p = &line[strlen(setADRdouble_str)];
//     while (*cfg_txt_p == ' ') cfg_txt_p++;
//     nvals = sscanf(cfg_txt_p, "%x %x %lf",
//                    &indexGroup, &indexOffset, &value);
//     if (nvals == 3) {
//       status = setSAFValueOnAxisVerify(indexGroup, indexOffset,
//                                        value, 1);
//     } else {
//       errorTxt = "Need 4 values";
//     }
//   } else {
//     errorTxt = "Illegal command";
//   }
//   if (errorTxt_p && errorTxt) {
//     *errorTxt_p = errorTxt;
//   }
//   return status;
// }


// asynStatus ecmcMotorRecordAxis::readConfigFile(void)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
  
//   const char *simOnly_str = "simOnly ";
//   FILE *fp;
//   char *ret = &pC_->outString_[0];
//   int line_no = 0;
//   asynStatus status = asynSuccess;
//   const char *errorTxt = NULL;
//   /* no config file, or successfully uploaded : return */
//   if (!drvlocal.cfgfileStr) {
//     return asynSuccess;
//   }
//   fp = fopen(drvlocal.cfgfileStr, "r");
//   if (!fp) {
//     int saved_errno = errno;
//     char cwdbuf[4096];
//     char errbuf[4196];

//     char *mypwd = getcwd(cwdbuf, sizeof(cwdbuf));
//     snprintf(errbuf, sizeof(errbuf)-1,
//              "E: readConfigFile: %s\n%s/%s",
//              strerror(saved_errno),
//              mypwd ? mypwd : "",
//              drvlocal.cfgfileStr);
//     updateMsgTxtFromDriver(errbuf);
//     asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//               "%s(%d)%s\n", modNamEMC, axisNo_, errbuf);
//     return asynError;
//   }
//   while (ret && !status && !errorTxt) {
//     char rdbuf[256];
//     size_t i;
//     size_t len;

//     line_no++;
//     ret = fgets(rdbuf, sizeof(rdbuf), fp);
//     if (!ret) break;    /* end of file or error */
//     len = strlen(ret);
//     if (!len) continue; /* empty line, no LF */
//     for (i=0; i < len; i++) {
//       /* No LF, no CR , no ctrl characters, */
//       if (rdbuf[i] < 32) rdbuf[i] = 0;
//     }
//     len = strlen(ret);
//     if (!len) continue; /* empty line with LF */
//     asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//               "%sreadConfigFile(%d) %s:%u %s\n",
//               modNamEMC, axisNo_,
//               drvlocal.cfgfileStr, line_no, rdbuf);

//     if (!strncmp(simOnly_str, rdbuf, strlen(simOnly_str))) {
//       /* "simOnly " Only for the simulator */
//       if (pC_->features_ & FEATURE_BITS_SIM) {
//         status = readConfigLine(&rdbuf[strlen(simOnly_str)], &errorTxt);
//       }
//     } else {
//       status = readConfigLine(rdbuf, &errorTxt);
//     }

//     if (status || errorTxt) {
//       char errbuf[256];
//       errbuf[sizeof(errbuf)-1] = 0;
//       if (status) {
//         snprintf(errbuf, sizeof(errbuf)-1,
//                  "E: %s:%d out=%s\nin=%s",
//                  drvlocal.cfgfileStr, line_no, pC_->outString_, pC_->inString_);
//       } else {
//         snprintf(errbuf, sizeof(errbuf)-1,
//                  "E: %s:%d \"%s\"\n%s",
//                  drvlocal.cfgfileStr, line_no, rdbuf, errorTxt);
//       }

//       asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//                 "%sreadConfigFile %s\n", modNamEMC, errbuf);
//       updateMsgTxtFromDriver(errbuf);
//     }
//   } /* while */

//   if (ferror(fp) || status || errorTxt) {
//     asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//               "%sreadConfigFile %sstatus=%d errorTxt=%s (%s)\n",
//               modNamEMC,
//               ferror(fp) ? "ferror " : "",
//               (int)status,
//               errorTxt ? errorTxt : "",
//               drvlocal.cfgfileStr);
//     fclose(fp);
//     return asynError;
//   }
//   return asynSuccess;
// }

// asynStatus ecmcMotorRecordAxis::setSAFValueOnAxisVerify(unsigned indexGroup,
//                                                    unsigned indexOffset,
//                                                    int value,
//                                                    unsigned int retryCount)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   asynStatus status = asynSuccess;
//   unsigned int counter = 0;
//   int rbvalue = 0 - value;
//   while (counter < retryCount) {
//     status = getSAFValueFromAxisPrint(indexGroup, indexOffset, "value=", &rbvalue);
//     if (status) break;
//     if (rbvalue == value) break;
//     status = setSAFValueOnAxis(indexGroup, indexOffset, value);
//     counter++;
//     if (status) break;
//     epicsThreadSleep(.1);
//   }
//   return status;
// }

/** Sets a floating point value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (floating point) variable to be updated
 *
 */
// asynStatus ecmcMotorRecordAxis::setValueOnAxis(const char* var, double value)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   snprintf(pC_->outString_, sizeof(pC_->outString_),
//            "%sMain.M%d.%s=%g",
//            drvlocal.adsport_str, axisNo_, var, value);
//   return pC_->writeReadACK(ASYN_TRACE_INFO);
// }

/** Sets 2 floating point value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (floating point) variable to be updated
 *
 */
// asynStatus ecmcMotorRecordAxis::setValuesOnAxis(const char* var1, double value1,
//                                            const char* var2, double value2)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   snprintf(pC_->outString_, sizeof(pC_->outString_),
//            "%sMain.M%d.%s=%g;%sMain.M%d.%s=%g",
//            drvlocal.adsport_str, axisNo_, var1, value1,
//            drvlocal.adsport_str, axisNo_, var2, value2);
//   return pC_->writeReadACK(ASYN_TRACE_INFO);
// }

// asynStatus ecmcMotorRecordAxis::getSAFValueFromAxisPrint(unsigned indexGroup,
//                                                     unsigned indexOffset,
//                                                     const char *name,
//                                                     int *value)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   int res;
//   int nvals;
//   asynStatus status;
//   if (axisId_ <= 0) return asynError;
//   snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,2,2?",
//           501, indexGroup + axisId_, indexOffset);
//   status = pC_->writeReadOnErrorDisconnect();
//   if (status)
//     return status;
//   nvals = sscanf(pC_->inString_, "%d", &res);
//   if (nvals != 1) {
//     asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//               "%snvals=%d command=\"%s\" response=\"%s\"\n",
//               modNamEMC, nvals, pC_->outString_, pC_->inString_);
//     return asynError;
//   }
//   asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
//             "%sout=%s in=%s %s=%d\n",
//             modNamEMC, pC_->outString_, pC_->inString_,name, res);
//   *value = res;
//   return asynSuccess;
// }

// asynStatus ecmcMotorRecordAxis::getSAFValueFromAxisPrint(unsigned indexGroup,
//                                                     unsigned indexOffset,
//                                                     const char *name,
//                                                     double *value)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   double res;
//   int nvals;
//   asynStatus status;
//   if (axisId_ <= 0) return asynError;
//   snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,8,5?",
//           501, indexGroup + axisId_, indexOffset);
//   status = pC_->writeReadOnErrorDisconnect();
//   if (status)
//     return status;
//   nvals = sscanf(pC_->inString_, "%lf", &res);
//   if (nvals != 1) {
//     asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//               "%snvals=%d command=\"%s\" response=\"%s\"\n",
//                modNamEMC, nvals, pC_->outString_, pC_->inString_);
//     return asynError;
//   }
//   asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
//             "%sout=%s in=%s %s=%g\n",
//             modNamEMC, pC_->outString_, pC_->inString_,name, res);
//   *value = res;
//   return asynSuccess;
// }


/** Gets an integer or boolean value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the integer result
 *
 */
// asynStatus ecmcMotorRecordAxis::getValueFromAxis(const char* var, int *value)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   asynStatus status;
//   int res;
//   snprintf(pC_->outString_, sizeof(pC_->outString_),
//            "%sMain.M%d%s?",
//            drvlocal.adsport_str, axisNo_, var);
//   status = pC_->writeReadOnErrorDisconnect();
//   if (status)
//     return status;
//   if (var[0] == 'b') {
//     if (!strcmp(pC_->inString_, "0")) {
//       res = 0;
//     } else if (!strcmp(pC_->inString_, "1")) {
//       res = 1;
//     } else {
//       asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//                 "%scommand=\"%s\" response=\"%s\"\n",
//                 modNamEMC, pC_->outString_, pC_->inString_);
//       return asynError;
//     }
//   } else {
//     int nvals = sscanf(pC_->inString_, "%d", &res);
//     if (nvals != 1) {
//       asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//                 "%snvals=%d command=\"%s\" response=\"%s\"\n",
//                  modNamEMC, nvals, pC_->outString_, pC_->inString_);
//       return asynError;
//     }
//   }
//   asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
//             "%sout=%s in=%s status=%s (%d) iValue=%d\n",
//             modNamEMC,
//             pC_->outString_, pC_->inString_,
//             ecmcMotorRecordstrStatus(status), (int)status, res);

//   *value = res;
//   return asynSuccess;
// }

/** Gets an integer (or boolean) and a double value from an axis and print
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the integer result
 *
 */
// asynStatus ecmcMotorRecordAxis::getSAFValuesFromAxisPrint(unsigned iIndexGroup,
//                                                      unsigned iIndexOffset,
//                                                      const char *iname,
//                                                      int *iValue,
//                                                      unsigned fIndexGroup,
//                                                      unsigned fIndexOffset,
//                                                      const char *fname,
//                                                      double *fValue)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   int iRes;
//   int nvals;
//   double fRes;
//   asynStatus status;
//   if (axisId_ <= 0) return asynError;
//   snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,2,2?;ADSPORT=%u/.ADR.16#%X,16#%X,8,5?",
//           501, iIndexGroup + axisId_, iIndexOffset,
//           501, fIndexGroup + axisId_, fIndexOffset);
//   status = pC_->writeReadOnErrorDisconnect();
//   if (status)
//     return status;
//   nvals = sscanf(pC_->inString_, "%d;%lf", &iRes, &fRes);
//   if (nvals != 2) {
//     asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//               "%snvals=%d command=\"%s\" response=\"%s\"\n",
//                modNamEMC, nvals, pC_->outString_, pC_->inString_);
//     return asynError;
//   }
//   asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
//             "%sout=%s in=%s %s=%d %s=%g\n",
//             modNamEMC, pC_->outString_, pC_->inString_, iname, iRes, fname, fRes);

//   *iValue = iRes;
//   *fValue = fRes;
//   return asynSuccess;

// }

/** Gets a floating point value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the double result
 *
 */
// asynStatus ecmcMotorRecordAxis::getValueFromAxis(const char* var, double *value)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   asynStatus status;
//   int nvals;
//   double res;
//   snprintf(pC_->outString_, sizeof(pC_->outString_),
//            "%sMain.M%d%s?",
//            drvlocal.adsport_str, axisNo_, var);
//   status = pC_->writeReadOnErrorDisconnect();
//   if (status)
//     return status;
//   nvals = sscanf(pC_->inString_, "%lf", &res);
//   if (nvals != 1) {
//     asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//               "%snvals=%d command=\"%s\" response=\"%s\"\n",
//                modNamEMC, nvals, pC_->outString_, pC_->inString_);
//     return asynError;
//   }
//   *value = res;
//   return asynSuccess;
// }

/** Gets a string value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the string result
 *
 */
// asynStatus ecmcMotorRecordAxis::getStringFromAxis(const char *var, char *value, size_t maxlen)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   value[0] = '\0'; /* Always have a valid string */
//     asynStatus status;
//     snprintf(pC_->outString_, sizeof(pC_->outString_),
//              "%sMain.M%d.%s?", drvlocal.adsport_str, axisNo_, var);
//     status = pC_->writeReadOnErrorDisconnect();
//     if (status) return status;
//     memcpy(value, pC_->inString_, maxlen);
//   return asynSuccess;
// }

// asynStatus ecmcMotorRecordAxis::getValueFromController(const char* var, double *value)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   asynStatus status;
//   int nvals;
//   double res;
//   snprintf(pC_->outString_, sizeof(pC_->outString_), "%s?", var);
//   status = pC_->writeReadOnErrorDisconnect();
//   if (status)
//     return status;
//   nvals = sscanf(pC_->inString_, "%lf", &res);
//   if (nvals != 1) {
//     asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//               "%snvals=%d command=\"%s\" response=\"%s\"\n",
//                modNamEMC, nvals, pC_->outString_, pC_->inString_);
//     return asynError;
//   }
//   *value = res;
//   return asynSuccess;
// }

// asynStatus ecmcMotorRecordAxis::setSAFValueOnAxis(unsigned indexGroup,
//                                              unsigned indexOffset,
//                                              int value)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   if (axisId_ <= 0) return asynError;
//   snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,2,2=%d",
//           501, indexGroup + axisId_, indexOffset, value);
//   return pC_->writeReadACK(ASYN_TRACE_INFO);
// }

// asynStatus ecmcMotorRecordAxis::setSAFValueOnAxis(unsigned indexGroup,
//                                              unsigned indexOffset,
//                                              double value)
// {
//   if (axisId_ <= 0) return asynError;
//   snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,8,5=%g",
//           501, indexGroup + axisId_, indexOffset, value);
//   return pC_->writeReadACK(ASYN_TRACE_INFO);
// }

/** Sets an integer or boolean value on an axis, read it back and retry if needed
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] name of the variable where we can read back
 * \param[in] value the (integer) variable to be updated
 * \param[in] number of retries
 */
// asynStatus ecmcMotorRecordAxis::setValueOnAxisVerify(const char *var, const char *rbvar,
//                                                 int value, unsigned int retryCount)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   asynStatus status = asynSuccess;
//   unsigned int counter = 0;
//   int rbvalue = 0 - value;
//   while (counter <= retryCount) {
//     snprintf(pC_->outString_, sizeof(pC_->outString_),
//              "%sMain.M%d.%s=%d;%sMain.M%d.%s?",
//              drvlocal.adsport_str, axisNo_, var, value,
//              drvlocal.adsport_str, axisNo_, rbvar);
//     status = pC_->writeReadOnErrorDisconnect();
//     asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
//               "%ssetValueOnAxisVerify(%d) out=%s in=%s status=%s (%d)\n",
//               modNamEMC, axisNo_,pC_->outString_, pC_->inString_,
//               ecmcMotorRecordstrStatus(status), (int)status);
//     if (status) {
//       return status;
//     } else {
//       int nvals = sscanf(pC_->inString_, "OK;%d", &rbvalue);
//       if (nvals != 1) {
//         asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//                   "%snvals=%d command=\"%s\" response=\"%s\"\n",
//                   modNamEMC, nvals, pC_->outString_, pC_->inString_);
//         return asynError;
//       }
//       if (status) break;
//       if (rbvalue == value) break;
//       counter++;
//       epicsThreadSleep(.1);
//     }
//   }
//   /* Verification failed.
//      Store the error (unless there was an error before) */
//   if ((rbvalue != value) && !drvlocal.cmdErrorMessage[0]) {
//       asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
//                 "%ssetValueOnAxisV(%d) var=%s value=%d rbvalue=%d",
//                 modNamEMC, axisNo_,var, value, rbvalue);
//       snprintf(drvlocal.cmdErrorMessage, sizeof(drvlocal.cmdErrorMessage)-1,
//                "E: setValueOnAxisV(%s) value=%d rbvalue=%d",
//                var, value, rbvalue);

//       /* The poller co-ordinates the writing into the parameter library */
//   }
//   return status;
// }

// asynStatus ecmcMotorRecordAxis::setSAFValueOnAxisVerify(unsigned indexGroup,
//                                                    unsigned indexOffset,
//                                                    double value,
//                                                    unsigned int retryCount)
// {
//   printf("Dbg################# %s\n",__FUNCTION__);
//   asynStatus status = asynSuccess;
//   unsigned int counter = 0;
//   double rbvalue = 0 - value;
//   while (counter < retryCount) {
//     status = getSAFValueFromAxisPrint(indexGroup, indexOffset, "value", &rbvalue);
//     if (status) break;
//     if (rbvalue == value) break;
//     status = setSAFValueOnAxis(indexGroup, indexOffset, value);
//     counter++;
//     if (status) break;
//     epicsThreadSleep(.1);
//   }
//   return status;
// }

/** Sets an integer or boolean value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (integer) variable to be updated
 *
 */
// asynStatus ecmcMotorRecordAxis::setValueOnAxis(const char* var, int value)
// {
//   snprintf(pC_->outString_, sizeof(pC_->outString_),
//            "%sMain.M%d.%s=%d",
//            drvlocal.adsport_str, axisNo_, var, value);
//   return pC_->writeReadACK(ASYN_TRACE_INFO);
// }

/** 
 * Connect to ECMC axis over SyncIO interface
 * 
 *  All read / write parameters are connected:
 *  1. "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.statusbin?"
 *  2. "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.controlbin?"
 * 
*/
/*asynStatus ecmcMotorRecordAxis::connectEcmcAxis() {

  char buffer[ECMC_MAX_ASYN_DRVINFO_STR_LEN];
  int movingPollPeriodMs = (int)(pC_->movingPollPeriod_*1000);
  char *name = &buffer[0];
  unsigned int charCount = 0;
  asynStatus status;

  asynInterface *pinterface;
  asynDrvUser *pDrvUser;

  
  // Control Data structure (sync io and interrupt)
  charCount = snprintf(name,
                       sizeof(buffer),
                       ECMC_ASYN_AXIS_CONT_STRING,
                       movingPollPeriodMs,
                       axisId_);
  if (charCount >= sizeof(buffer) - 1) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to generate drvInfo for %s on asynport %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      ECMC_ASYN_AXIS_CONT_STRING,
      pC_->mcuPortName_);
    return asynError;
  }

  status = pasynInt8ArraySyncIO->connect(pC_->mcuPortName_, 0, &asynUserCntrlBin_, name);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to connect drvInfo to ECMC for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);
  }
  
  asynUserCntrlBinIntr_ = pasynManager->createAsynUser(0,0);
  status = pasynManager->connectDevice(asynUserCntrlBinIntr_, pC_->mcuPortName_, 0);
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): pasynManager->connectDevice() failed for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);    
    return asynError;
  }
  
  pasynIFContBinIntr_ = pasynManager->findInterface(asynUserCntrlBinIntr_, "asynInt8Array", 1);
  if (!pasynIFContBinIntr_) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): pasynManager->findInterface() failed for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);    
    return asynError;
  }

  pinterface = pasynManager->findInterface(asynUserCntrlBinIntr_, asynDrvUserType, 1);
  if (!pinterface) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): pasynManager->findInterface() failed for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);    
    return asynError;
  }
  
  pDrvUser = (asynDrvUser *)pinterface->pinterface;

  status = pDrvUser->create(pinterface->drvPvt, asynUserCntrlBinIntr_, name, 0, 0);
  if (status) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): pDrvUser->create() failed for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);        
    return asynError;
  }

  pIFContBinIntr_ = (asynInt8Array *)pasynIFContBinIntr_->pinterface;

  if(interruptContBinPvt_!=NULL) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): interruptContBinPvt_->pinterface != NULL for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);        
    return asynError;
  }
  
  status = pIFContBinIntr_->registerInterruptUser(pasynIFContBinIntr_->drvPvt, asynUserCntrlBinIntr_,
                                                    contBinCallback, this, &interruptContBinPvt_);
  if (status) {      
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): pIFContBinIntr_->registerInterruptUser() failed for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);        

    return asynError;
  }  

  // Status binary struct (interrupt and sync)
  charCount = snprintf(name,
                       sizeof(buffer),
                       ECMC_ASYN_AXIS_STAT_BIN_STRING,
                       movingPollPeriodMs,
                       axisId_);
  if (charCount >= sizeof(buffer) - 1) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to generate drvInfo for %s on asynport %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      ECMC_ASYN_AXIS_STAT_BIN_STRING,
      pC_->mcuPortName_);
    return asynError;
  }
           
  status = pasynInt8ArraySyncIO->connect(pC_->mcuPortName_, 0, &asynUserStatBin_, name);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to connect drvInfo to ECMC for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);
  }

  asynUserStatBinIntr_ = pasynManager->createAsynUser(0,0);
  status = pasynManager->connectDevice(asynUserStatBinIntr_, pC_->mcuPortName_, 0);
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): pasynManager->connectDevice() failed for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);    
    return asynError;
  }
  
  pasynIFStatBinIntr_ = pasynManager->findInterface(asynUserStatBinIntr_, "asynInt8Array", 1);
  if (!pasynIFStatBinIntr_) {    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): pasynManager->findInterface() failed for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);    
    return asynError;
  }

  pinterface = pasynManager->findInterface(asynUserStatBinIntr_, asynDrvUserType, 1);
  if (!pinterface) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): pasynManager->findInterface() failed for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);    
    return asynError;
  }
  
  pDrvUser = (asynDrvUser *)pinterface->pinterface;

  status = pDrvUser->create(pinterface->drvPvt, asynUserStatBinIntr_, name, 0, 0);
  if (status) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): pDrvUser->create() failed for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);        
    return asynError;
  }

  pIFStatBinIntr_ = (asynInt8Array *)pasynIFStatBinIntr_->pinterface;

  if(interruptStatBinPvt_!=NULL) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): pasynIFStatBinIntr_->pinterface == NULL for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);        
    return asynError;
  }
  
  status = pIFStatBinIntr_->registerInterruptUser(pasynIFStatBinIntr_->drvPvt, asynUserStatBinIntr_,
                                                    statBinCallback, this, &interruptStatBinPvt_);
  if (status) {      
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): pIFStatBinIntr_->registerInterruptUser() failed for parameter %s.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_,
      name);        

    return asynError;
  }  

  readAll();
  return asynSuccess;
}*/

/*asynStatus ecmcMotorRecordAxis::readStatusBin() {
  
  size_t inBytes = 0;
  ecmcAxisStatusType diagBinDataTemp;
  asynStatus status = pasynInt8ArraySyncIO->read(asynUserStatBin_,
                                            (epicsInt8*)&diagBinDataTemp,
                                            sizeof(ecmcAxisStatusType),
                                            &inBytes,
                                            DEFAULT_CONTROLLER_TIMEOUT);    

  if (status!=asynSuccess || inBytes != sizeof(ecmcAxisStatusType)) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to read diag binary data from ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }

  statusBinData_ = diagBinDataTemp;
  return asynSuccess;
}*/

/*asynStatus ecmcMotorRecordAxis::readControlBin() {

  size_t inBytes = 0;
  ecmcAsynClinetCmdType controlBinTemp;
  asynStatus status = pasynInt8ArraySyncIO->read(asynUserCntrlBin_,(epicsInt8*)&controlBinTemp,sizeof(ecmcAsynClinetCmdType),&inBytes,DEFAULT_CONTROLLER_TIMEOUT);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to read control data structure from ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }
  controlBinDataRB_ = controlBinTemp;

  return asynSuccess;
}*/

/*asynStatus ecmcMotorRecordAxis::writeControlBin(ecmcAsynClinetCmdType controlStruct) {

  epicsInt8 *pCntData = (epicsInt8 *)&controlStruct;
  asynStatus status = pasynInt8ArraySyncIO->write(asynUserCntrlBin_,pCntData,sizeof(ecmcAsynClinetCmdType),DEFAULT_CONTROLLER_TIMEOUT);
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s/%s:%d: ERROR (axis %d): Failed to write control word to ecmc.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      axisId_);
    return asynError;
  }  
  return asynSuccess;
}*/

/*asynStatus ecmcMotorRecordAxis::readAll() {
  
  asynStatus status;

  status =readStatusBin();
  if(status) {
    return status;
  }

  return readControlBin();
}*/

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

  //printDiagBinData();
  return asynSuccess;
}
