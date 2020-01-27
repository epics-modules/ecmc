/*
  FILENAME... EthercatMCAxis.cpp
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

static EthercatMCController *pC;

/* Callback for axis status data struct (binary)
*/
void statBinCallback(void *userPvt, asynUser *pasynUser, epicsInt8 *value, size_t nelements) {
//  printf("diagBinCallback()\n");
  EthercatMCAxisEcmc * axis = (EthercatMCAxisEcmc*)userPvt;
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
}

/* Callback for axis control data struct (binary)
*/
void contBinCallback(void *userPvt, asynUser *pasynUser, epicsInt8 *value, size_t nelements) {
//  printf("diagBinCallback()\n");
  EthercatMCAxisEcmc * axis = (EthercatMCAxisEcmc*)userPvt;
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
}

/** Creates a new EthercatMCAxis object.
 * \param[in] pC Pointer to the EthercatMCController to which this axis belongs.
 * \param[in] axisNo Index number of this axis, range 1 to pC->numAxes_. (0 is not used)
 *
 *
 * Initializes register numbers, etc.
 */
EthercatMCAxisEcmc::EthercatMCAxisEcmc(EthercatMCController *pC, int axisNo,
                               int axisFlags, const char *axisOptionsStr)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  // ECMC  
  asynUserStatBin_      = NULL; // "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.diagnosticbin?"
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
  memset(&controlBinDataRB_,0, sizeof(ecmcAsynClinetCmdType));

  axisId_ = axisNo;

  oldPositionAct_ = 0;
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
    const char * const cfgDebug_str = "getDebugText=";
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
      } else if (!strncmp(pThisOption, cfgDebug_str, strlen(cfgDebug_str))) {
        pThisOption += strlen(cfgDebug_str);
        drvlocal.cfgDebug_str = strdup(pThisOption);
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
        setIntegerParam(pC_->EthercatMCHomProc_, homProc);
      } else if (!strncmp(pThisOption, homPos_str, strlen(homPos_str))) {
        pThisOption += strlen(homPos_str);
        double homPos = atof(pThisOption);
        setDoubleParam(pC_->EthercatMCHomPos_, homPos);
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

  asynStatus status = connectEcmcAxis();
  if (status!=asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "connectEcmcAxis() failed on port %s\n",pC_->mcuPortName_);
  }
  
  initialPoll();
}


extern "C" int EthercatMCCreateAxisEcmc(const char *EthercatMCName, int axisNo,
                                    int axisFlags, const char *axisOptionsStr)
{
  
  pC = (EthercatMCController*) findAsynPortDriver(EthercatMCName);
  if (!pC) {
    printf("Error port %s not found\n", EthercatMCName);
    return asynError;
  }
  pC->lock();
  new EthercatMCAxisEcmc(pC, axisNo, axisFlags, axisOptionsStr);
  pC->unlock();
  return asynSuccess;
  
}

asynStatus EthercatMCAxisEcmc::updateCfgValue(int function,
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

asynStatus EthercatMCAxisEcmc::updateCfgValue(int function,
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

asynStatus EthercatMCAxisEcmc::readBackSoftLimits(void)
{
  printf("Dbg################# %s\n",__FUNCTION__);

  asynStatus status;
  int nvals;
  int enabledHigh = 0, enabledLow = 0;
  double fValueHigh = 0.0, fValueLow  = 0.0;
  double scaleFactor = drvlocal.scaleFactor;

  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=501/.ADR.16#%X,16#%X,2,2?;ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,2,2?;ADSPORT=501/.ADR.16#%X,16#%X,8,5?",
           0x5000 + axisId_, 0xC,
           0x5000 + axisId_, 0xE,
           0x5000 + axisId_, 0xB,
           0x5000 + axisId_, 0xD);

  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%d;%lf;%d;%lf",
                 &enabledHigh, &fValueHigh, &enabledLow, &fValueLow);
  if (nvals != 4) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
              modNamEMC, nvals, pC_->outString_, pC_->inString_);
    enabledHigh = enabledLow = 0;
    fValueHigh = fValueLow = 0.0;
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
            "%sout=%s in=%s CHLM_En=%d CHLM=%f CLLM_En=%d CLLM=%f\n",
            modNamEMC, pC_->outString_, pC_->inString_,
            enabledHigh, fValueHigh, enabledLow, fValueLow);
  /* EthercatMCCHLMXX are info(asyn:READBACK,"1"),
     so we must use pC_->setXXX(axisNo_..)  here */



  pC_->setIntegerParam(axisNo_, pC_->EthercatMCCfgDHLM_En_, enabledHigh);
  pC_->setDoubleParam(axisNo_, pC_->EthercatMCCfgDHLM_, fValueHigh);
  pC_->setIntegerParam(axisNo_, pC_->EthercatMCCfgDLLM_En_, enabledLow);
  pC_->setDoubleParam(axisNo_, pC_->EthercatMCCfgDLLM_, fValueLow);

  if (scaleFactor) {
    pC_->udateMotorLimitsRO(axisNo_, enabledHigh && enabledLow,
                            fValueHigh / scaleFactor, fValueLow / scaleFactor);
  }
  return status;
}

asynStatus EthercatMCAxisEcmc::readBackHoming(void)
{
  printf("Dbg################# %s\n",__FUNCTION__);

  asynStatus status;
  int    homProc = 0;
  double homPos  = 0.0;

  /* Don't read it, when the driver has been configured with HomProc= */
  status = pC_->getIntegerParam(axisNo_, pC_->EthercatMCHomProc_,&homProc);
  if (status == asynSuccess) return status;

  status = getValueFromAxis("_EPICS_HOMPROC", &homProc);
  if (!status) setIntegerParam(pC_->EthercatMCHomProc_, homProc);
  status = getValueFromAxis("_EPICS_HOMPOS", &homPos);
  if (status) {
    /* fall back */
    status = getSAFValueFromAxisPrint(0x5000, 0x103, "homPos", &homPos);
  }
  if (!status) setDoubleParam(pC_->EthercatMCHomPos_, homPos);
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::readScaling(int axisID)
{
  printf("Dbg################# %s\n",__FUNCTION__);

  int nvals;
  asynStatus status;
  double srev = 0, urev = 0;

  double scaleFactor = drvlocal.scaleFactor;

  if (!scaleFactor) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;",
           0x5000 + axisID, 0x24,  // SREV
           0x5000 + axisID, 0x23   // UREV
           );
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  nvals = sscanf(pC_->inString_, "%lf;%lf",
                 &srev, &urev);

  if (nvals != 2) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d\n", modNamEMC, nvals);
    return asynError;
  }
  updateCfgValue(pC_->EthercatMCCfgSREV_RB_, srev, "srev");
  updateCfgValue(pC_->EthercatMCCfgUREV_RB_, urev, "urev");
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::readMonitoring(int axisID)
{
  printf("Dbg################# %s\n",__FUNCTION__);

  int nvals;
  asynStatus status;
  double rdbd, rdbd_tim, poslag = -1, poslag_tim = -1;
  int rdbd_en, poslag_en = 0;
  double scaleFactor = drvlocal.scaleFactor;

  if (!scaleFactor) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,2,2?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,2,2?",
           0x4000 + axisID, 0x16,  // RDBD_RB
           0x4000 + axisID, 0x17,  // RDBD_Tim
           0x4000 + axisID, 0x15,  // RDBD_En
           0x6000 + axisID, 0x12,  // PosLag
           0x6000 + axisID, 0x13,  // PosLag_Tim
           0x6000 + axisID, 0x10); // Poslag_En
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  nvals = sscanf(pC_->inString_, "%lf;%lf;%d;%lf;%lf;%d",
                 &rdbd, &rdbd_tim, &rdbd_en, &poslag, &poslag_tim, &poslag_en
                 );
  if ((nvals != 6) && (nvals != 3)) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%sreadMonitoring(%d) nvals=%d out=%s in=%s \n",
              modNamEMC, axisNo_, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  updateCfgValue(pC_->EthercatMCCfgSPDB_RB_, rdbd, "spbd");
  updateCfgValue(pC_->EthercatMCCfgRDBD_RB_, rdbd, "rdbd");
  updateCfgValue(pC_->EthercatMCCfgRDBD_Tim_RB_, rdbd_tim , "rdbd_time");
  updateCfgValue(pC_->EthercatMCCfgRDBD_En_RB_, rdbd_en, "rdbd_en");

  drvlocal.illegalInTargetWindow = (!rdbd_en || !rdbd);

  if (nvals == 6) {
    updateCfgValue(pC_->EthercatMCCfgPOSLAG_RB_, poslag, "poslag");
    updateCfgValue(pC_->EthercatMCCfgPOSLAG_Tim_RB_, poslag_tim, "poslag_tim");
    updateCfgValue(pC_->EthercatMCCfgPOSLAG_En_RB_, poslag_en, "poslag_en");
  }
  return asynSuccess;
}


asynStatus EthercatMCAxisEcmc::readBackVelocities(int axisID)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  asynStatus status;
  int nvals;
  double scaleFactor = drvlocal.scaleFactor;
  double velo, vmax, jvel, accs;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;"
           "ADSPORT=501/.ADR.16#%X,16#%X,8,5?;",
           0x4000 + axisID, 0x9,   // VELO"
           0x4000 + axisID, 0x27,  // VMAX"
           0x4000 + axisID, 0x8,   // JVEL
           0x4000 + axisID, 0x101  // ACCS"
           );
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  nvals = sscanf(pC_->inString_, "%lf;%lf;%lf;%lf",
                 &velo, &vmax, &jvel, &accs);
  if (nvals != 4) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
              modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
            "%svelo=%f vmax=%f jvel=%f accs=%f\n",
            modNamEMC, velo, vmax, jvel, accs);
  if (velo > 0.0) {
    updateCfgValue(pC_->EthercatMCCfgVELO_, velo / scaleFactor, "velo");
  }
  if (vmax > 0.0) {
    updateCfgValue(pC_->EthercatMCCfgVMAX_, vmax / scaleFactor, "vmax");
  }
  if (jvel > 0.0) {
    updateCfgValue(pC_->EthercatMCCfgJVEL_, jvel / scaleFactor, "jvel");
  }
  if (accs > 0.0) {
    updateCfgValue(pC_->EthercatMCCfgACCS_, accs / scaleFactor, "accs");
  }
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::readBackEncoders(int axisID)
{
  printf("Dbg################# %s\n",__FUNCTION__);

  /* https://infosys.beckhoff.com/english.php?content=../content/1033/tcadsdevicenc2/index.html&id= */
  asynStatus status;
  uint32_t numEncoders = 0;
  int nvals;

  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=501/.ADR.16#%X,16#%X,8,19?;",
           0x4000 + axisID, 0x57
           );
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  nvals = sscanf(pC_->inString_, "%u",
                 &numEncoders);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
              modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sreadBackEncoders(%d) numEncoders=%u\n",
            modNamEMC, axisNo_, numEncoders);
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::initialPoll(void)
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


asynStatus EthercatMCAxisEcmc::readBackAllConfig(int axisID)
{
  asynStatus status = asynSuccess;
  /* for ECMC homing is configured from EPICS, do NOT do the readback */
  if (!(pC_->features_ & FEATURE_BITS_ECMC)) {
    if (!drvlocal.scaleFactor) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%sreadBackAllConfig(%d) drvlocal.scaleFactor=0.0\n",
                modNamEMC, axisNo_);
      return asynError;
    }
    if (status == asynSuccess) status = readBackHoming();
  }
  if (status == asynSuccess) status = readScaling(axisID);
  if (status == asynSuccess) status = readMonitoring(axisID);
  if (status == asynSuccess) status = readBackSoftLimits();
  if (status == asynSuccess) status = readBackVelocities(axisID);
  if (!(pC_->features_ & FEATURE_BITS_ECMC)) {
    if (status == asynSuccess) status = readBackEncoders(axisID);
  }
  return status;
}

/** Connection status is changed, the dirty bits must be set and
 *  the values in the controller must be updated
 * \param[in] AsynStatus status
 *
 * Sets the dirty bits
 */
asynStatus EthercatMCAxisEcmc::initialPollInternal(void)
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
void EthercatMCAxisEcmc::report(FILE *fp, int level)
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
asynStatus EthercatMCAxisEcmc::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
            "%smove(%d) position=%f relative=%d maxVelocity=%f acceleration=%f\n",
            modNamEMC, axisNo_,
            position, relative, maxVelocity, acceleration);

  // cmd, pos, vel, acc
  controlBinData_.cmd = relative ? ECMC_CMD_MOVEREL : ECMC_CMD_MOVEABS;

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  /* Do range check */
  if (!drvlocal.scaleFactor) {
    drvlocal.eeAxisWarning = eeAxisWarningCfgZero;
    return asynSuccess;
  } else if (!maxVelocity) {
    drvlocal.eeAxisWarning = eeAxisWarningVeloZero;
    return asynSuccess;
  }

  // Position
  controlBinData_.val0 = position * drvlocal.scaleFactor;
  // Velocity
  controlBinData_.val1 = maxVelocity * drvlocal.scaleFactor;
  // Acc m/sec2
  controlBinData_.val2 = acceleration * drvlocal.scaleFactor;

  asynStatus status = writeControlBin(controlBinData_);
  if(status != asynSuccess){
    return status;
  }
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return status;
}


/** Home the motor, search the home position
 * \param[in] minimum velocity, mm/sec
 * \param[in] maximum velocity, mm/sec
 * \param[in] acceleration, seconds to maximum velocity
 * \param[in] forwards (0=backwards, otherwise forwards)
 *
 */
asynStatus EthercatMCAxisEcmc::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  // cmd, nCmddata,homepos,velhigh,vellow,acc
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
            "%shome(%d) forwards=%d maxVelocity=%f acceleration=%f\n",
            modNamEMC, axisNo_,
            forwards, maxVelocity, acceleration);

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;

  controlBinData_.cmd = ECMC_CMD_HOMING;

  // Read from records / params
  double homPos    = 0.0; /* The homPos may be undefined, then use 0.0 */
  int cmdData      = -1;
  double velToCam  = 0;
  double velOffCam = 0;
  double accHom    = 0;
   
  // nCmdData (sequence number)
  asynStatus status = pC_->getIntegerParam(axisNo_, pC_->EthercatMCHomProc_,&cmdData);
  if (cmdData == HOMPROC_MANUAL_SETPOS || status != asynSuccess) {
    return asynError;
  }
  controlBinData_.val0 = (double)cmdData;

  // Home position
  (void)pC_->getDoubleParam(axisNo_, pC_->EthercatMCHomPos_, &homPos);
  controlBinData_.val1 = homPos;

  // Velocity to cam (high velo)
  status = pC_->getDoubleParam(axisNo_,
                               pC_->EthercatMCVelToHom_,
                               &velToCam);
  if (status != asynSuccess) {
    return asynError;
  }
  controlBinData_.val2 = velToCam;

  // Velocity off cam (low velo)
  status = pC_->getDoubleParam(axisNo_,
                               pC_->EthercatMCVelFrmHom_,
                               &velOffCam);
  if (status != asynSuccess) {
    return asynError;
  }
  controlBinData_.val3 = velOffCam;

  status = pC_->getDoubleParam(axisNo_,
                               pC_->EthercatMCAccHom_,
                               &accHom);
  if (status != asynSuccess) {
    return asynError;
  }
  controlBinData_.val4 = accHom;

  status = writeControlBin(controlBinData_);
  if(status != asynSuccess){
    return status;
  }

  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return status;
}


/** jog the the motor, search the home position
 * \param[in] minimum velocity, mm/sec (not used)
 * \param[in] maximum velocity, mm/sec (positive or negative)
 * \param[in] acceleration, seconds to maximum velocity
 *
 */
asynStatus EthercatMCAxisEcmc::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  // cmd, vel, acc
  controlBinData_.cmd = ECMC_CMD_MOVEVEL;

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  /* Do range check */
  if (!drvlocal.scaleFactor) {
    drvlocal.eeAxisWarning = eeAxisWarningCfgZero;
    return asynSuccess;
  } else if (!maxVelocity) {
    drvlocal.eeAxisWarning = eeAxisWarningVeloZero;
    return asynSuccess;
  }
  
  // Velocity
  controlBinData_.val0 = maxVelocity * drvlocal.scaleFactor;
  // Acc m/sec2
  controlBinData_.val1 = 0.0;

  if (acceleration > 0.0001) {
    double acc_in_seconds = maxVelocity / acceleration;
    controlBinData_.val1 = controlBinData_.val0 / acc_in_seconds;
  }

  if (controlBinData_.val1  < 0) {
    controlBinData_.val1 = 0 - controlBinData_.val1;
  }

  asynStatus status = writeControlBin(controlBinData_);
  if(status != asynSuccess){
    return status;
  }
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return status;
}

/**
 * See asynMotorAxis::setPosition
 */
asynStatus EthercatMCAxisEcmc::setPosition(double value)
{
  // cmd, nCmddata,homepos
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
            "%ssetPosition(%d) value=%lf\n",
            modNamEMC, axisNo_, value);

  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;

  controlBinData_.cmd = ECMC_CMD_HOMING;

  // Read from records / params
  double homPos    = 0.0; /* The homPos may be undefined, then use 0.0 */
  int cmdData      = -1;
   
  // nCmdData (sequence number) Must be "manual cmddata (=15)"
  asynStatus status = pC_->getIntegerParam(axisNo_, pC_->EthercatMCHomProc_,&cmdData);
  if (cmdData != HOMPROC_MANUAL_SETPOS || status != asynSuccess) {
    return asynError;
  }
  controlBinData_.val0 = (double)cmdData;

  // Home position
  (void)pC_->getDoubleParam(axisNo_, pC_->EthercatMCHomPos_, &homPos);
  controlBinData_.val1 = homPos;

  status = writeControlBin(controlBinData_);
  if(status != asynSuccess){
    return status;
  }

  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return status;
}

asynStatus EthercatMCAxisEcmc::resetAxis(void)
{
  controlBinData_.cmd = ECMC_CMD_SET_RESET;
  asynStatus status = writeControlBin(controlBinData_);
  if(status != asynSuccess){
    return status;
  }

  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return status;
}

asynStatus EthercatMCAxisEcmc::setEnable(int on) {
  controlBinData_.cmd = ECMC_CMD_SET_ENABLE; 
  controlBinData_.val0 = on ? 1.0 : 0.0; 
  return writeControlBin(controlBinData_);
}

/** Enable the amplifier on an axis
 *
 */
asynStatus EthercatMCAxisEcmc::enableAmplifier(int on)
{
  asynStatus status = asynSuccess;
  unsigned counter = 10;

//   const char *enableEnabledReadback = "bEnabled";
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
asynStatus EthercatMCAxisEcmc::stopAxisInternal(const char *function_name, double acceleration)
{ 
  controlBinData_.cmd = ECMC_CMD_STOP;
  asynStatus status = writeControlBin(controlBinData_);
  if(status != asynSuccess){
    return status;
  }
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
  return status;
}

/** 
 * Stop the axis, called by motor Record
 */
asynStatus EthercatMCAxisEcmc::stop(double acceleration )
{
  drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  return stopAxisInternal(__FUNCTION__, acceleration);
}

void EthercatMCAxisEcmc::callParamCallbacksUpdateError()
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
    setIntegerParam(pC_->EthercatMCErr_,
                    drvlocal.eeAxisError == eeAxisErrorMCUError);
    setIntegerParam(pC_->EthercatMCErrId_, EPICS_nErrorId);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) callParamCallbacksUpdateError"
              " Error=%d old=%d ErrID=0x%x old=0x%x Warn=%d nCmd=%d old=%d txt=%s\n",
              modNamEMC, axisNo_, drvlocal.eeAxisError, drvlocal.old_eeAxisError,
              EPICS_nErrorId, drvlocal.old_EPICS_nErrorId,
              drvlocal.eeAxisWarning,
              drvlocal.nCommandActive, drvlocal.old_nCommandActive,
              msgTxtFromDriver ? msgTxtFromDriver : "NULL");

    if (!drvlocal.cfgDebug_str) {
      updateMsgTxtFromDriver(msgTxtFromDriver);
    }
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
asynStatus EthercatMCAxisEcmc::poll(bool *moving)
{
  //printf("Dbg################# 1 %s\n",__FUNCTION__);
  asynStatus comStatus = asynSuccess;
  st_axis_status_type st_axis_status;

  double timeBefore = EthercatMCgetNowTimeSecs();
  int waitNumPollsBeforeReady_ = drvlocal.waitNumPollsBeforeReady;

  /* Driver not yet initialized, do nothing */
  if (!drvlocal.scaleFactor) return comStatus;

  if (drvlocal.supported.statusVer == -1) {
    callParamCallbacksUpdateError();
    return asynSuccess;
  }

  memset(&st_axis_status, 0, sizeof(st_axis_status));

  // ---------------------ECMC
   
  uglyConvertFunc(&statusBinData_,&st_axis_status);
  
  // ---------------------ECMC

  if (drvlocal.cfgDebug_str) {
    printf("Dbg################# 2 %s\n",__FUNCTION__);
    asynStatus comStatus;
    snprintf(pC_->outString_, sizeof(pC_->outString_), "%s", drvlocal.cfgDebug_str);
    comStatus = pC_->writeReadOnErrorDisconnect();
    if (!comStatus) {
      printf("Dbg################# 3 %s\n",__FUNCTION__);
      updateMsgTxtFromDriver(pC_->inString_);
    }
  }

  setIntegerParam(pC_->motorStatusHomed_, st_axis_status.bHomed);
  drvlocal.homed = st_axis_status.bHomed;
  setIntegerParam(pC_->motorStatusCommsError_, 0);
  setIntegerParam(pC_->motorStatusAtHome_, st_axis_status.bHomeSensor);
  setIntegerParam(pC_->motorStatusLowLimit_, !st_axis_status.bLimitBwd);
  setIntegerParam(pC_->motorStatusHighLimit_, !st_axis_status.bLimitFwd);
  setIntegerParam(pC_->motorStatusPowerOn_, st_axis_status.bEnabled);
  setDoubleParam(pC_->EthercatMCVelAct_, st_axis_status.fActVelocity);
  setDoubleParam(pC_->EthercatMCAcc_RB_, st_axis_status.fAcceleration);

#ifndef motorWaitPollsBeforeReadyString
  if (drvlocal.waitNumPollsBeforeReady) {
    printf("Dbg################# 4 %s\n",__FUNCTION__);
    *moving = true;
  }
  else
#endif
    {
      *moving = st_axis_status.mvnNRdyNex ? true : false;
      if (!st_axis_status.mvnNRdyNex &&
          !(pC_->features_ & FEATURE_BITS_ECMC)) {
        printf("Dbg################# 5 %s\n",__FUNCTION__);
        /* not moving: poll the parameters for this axis */        
        switch (drvlocal.eeAxisPollNow) {          
        case pollNowReadScaling:
          printf("Dbg################# 6 %s\n",__FUNCTION__);    
          readScaling(axisId_);          
          drvlocal.eeAxisPollNow = pollNowReadMonitoring;
          break;
        case pollNowReadMonitoring:
          printf("Dbg################# 7 %s\n",__FUNCTION__);
          readMonitoring(axisId_);
          drvlocal.eeAxisPollNow = pollNowReadBackSoftLimits;
          break;
        case pollNowReadBackSoftLimits:
          printf("Dbg################# 8 %s\n",__FUNCTION__);
          readBackSoftLimits();
          drvlocal.eeAxisPollNow = pollNowReadBackVelocities;
          break;
        case pollNowReadBackVelocities:
        default:
          printf("Dbg################# 9 %s\n",__FUNCTION__);
          readBackVelocities(axisId_);
          drvlocal.eeAxisPollNow = pollNowReadScaling;
          break;
        }
      }
    }

  if (st_axis_status.mvnNRdyNex){
    //printf("Dbg################# 10 %s\n",__FUNCTION__);
    drvlocal.nCommandActive = st_axis_status.nCommand;
  }
  else {
    //printf("Dbg################# 11 %s\n",__FUNCTION__);
    drvlocal.nCommandActive = 0;
    if (drvlocal.eeAxisWarning == eeAxisWarningSpeedLimit) {
      drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
    }
  }

  if (drvlocal.nCommandActive != ECMC_CMD_HOMING) {
    //printf("Dbg################# 12 %s\n",__FUNCTION__);
    setDoubleParam(pC_->motorPosition_,
                   st_axis_status.fActPosition / drvlocal.scaleFactor);
    setDoubleParam(pC_->motorEncoderPosition_,
                   st_axis_status.fActPosition / drvlocal.scaleFactor);
    drvlocal.old_st_axis_status.fActPosition = st_axis_status.fActPosition;
    setDoubleParam(pC_->EthercatMCVel_RB_, st_axis_status.fVelocity);
  }

  if (drvlocal.externalEncoderStr) {
    //printf("Dbg################# 13 %s\n",__FUNCTION__);
    comStatus = getValueFromController(drvlocal.externalEncoderStr,
                                       &st_axis_status.encoderRaw);
    if (!comStatus) setDoubleParam(pC_->EthercatMCEncAct_,
                                   st_axis_status.encoderRaw);
  } else if (pC_->features_ & FEATURE_BITS_V2) {
    setDoubleParam(pC_->EthercatMCEncAct_, st_axis_status.encoderRaw);
  }

  if (drvlocal.old_st_axis_status.bHomed != st_axis_status.bHomed) {
    //printf("Dbg################# 14 %s\n",__FUNCTION__);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) homed=%d\n",
              modNamEMC, axisNo_, st_axis_status.bHomed);
    drvlocal.old_st_axis_status.bHomed =  st_axis_status.bHomed;
  }
  if (drvlocal.old_st_axis_status.bLimitBwd != st_axis_status.bLimitBwd) {
    //printf("Dbg################# 15 %s\n",__FUNCTION__);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) LLS=%d\n",
              modNamEMC, axisNo_, !st_axis_status.bLimitBwd);
    drvlocal.old_st_axis_status.bLimitBwd =  st_axis_status.bLimitBwd;
  }
  if (drvlocal.old_st_axis_status.bLimitFwd != st_axis_status.bLimitFwd) {
    //printf("Dbg################# 16 %s\n",__FUNCTION__);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) HLS=%d\n",
              modNamEMC, axisNo_,!st_axis_status.bLimitFwd);
    drvlocal.old_st_axis_status.bLimitFwd = st_axis_status.bLimitFwd;
  }

#ifndef motorWaitPollsBeforeReadyString
  if (drvlocal.waitNumPollsBeforeReady) {
    //printf("Dbg################# 17 %s\n",__FUNCTION__);
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
      //printf("Dbg################# 18 %s\n",__FUNCTION__);
      if (drvlocal.old_st_axis_status.mvnNRdyNex != st_axis_status.mvnNRdyNex ||
          drvlocal.old_st_axis_status.bBusy      != st_axis_status.bBusy ||
          drvlocal.old_st_axis_status.bEnabled   != st_axis_status.bEnabled ||
          drvlocal.old_st_axis_status.bExecute   != st_axis_status.bExecute ||
          drvlocal.old_st_axis_status.atTarget   != st_axis_status.atTarget) {
          //printf("Dbg################# 19 %s\n",__FUNCTION__);
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "%spoll(%d) mvnNRdy=%d Ver=%d bBusy=%d bExe=%d bEnabled=%d atTarget=%d wf=%d ENC=%g fPos=%g fActPosition=%g time=%f\n",
                  modNamEMC, axisNo_, st_axis_status.mvnNRdyNex,
                  drvlocal.supported.statusVer,
                  st_axis_status.bBusy, st_axis_status.bExecute,
                  st_axis_status.bEnabled, st_axis_status.atTarget,
                  waitNumPollsBeforeReady_,
                  st_axis_status.encoderRaw, st_axis_status.fPosition,
                  st_axis_status.fActPosition,
                  EthercatMCgetNowTimeSecs() - timeBefore);
      }
    }
  setIntegerParam(pC_->motorStatusDirection_, st_axis_status.motorStatusDirection);
  setIntegerParam(pC_->motorStatusMoving_, st_axis_status.mvnNRdyNex);
  setIntegerParam(pC_->motorStatusDone_, !st_axis_status.mvnNRdyNex);

  drvlocal.MCU_nErrorId = st_axis_status.nErrorId;
  //printf("Dbg################# 20 %s\n",__FUNCTION__);
  if (drvlocal.cfgDebug_str) {
    //printf("Dbg################# 21 %s\n",__FUNCTION__);
    ; /* Do not do the following */
  } else if (drvlocal.old_bError != st_axis_status.bError ||
             drvlocal.old_MCU_nErrorId != drvlocal.MCU_nErrorId ||
             drvlocal.dirty.sErrorMessage) {
    //printf("Dbg################# 23 %s\n",__FUNCTION__);
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
      //printf("Dbg################# 24 %s\n",__FUNCTION__);
      /* Get the ErrorMessage to have it in the log file */
      (void)getStringFromAxis("sErrorMessage", (char *)&sErrorMessage[0],
                              sizeof(sErrorMessage));
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%ssErrorMessage(%d)=\"%s\"\n",
                modNamEMC, axisNo_, sErrorMessage);
    }
    /* First choice: "well known" ErrorIds */
    if (errIdString[0]) {
      snprintf(drvlocal.sErrorMessage, sizeof(drvlocal.sErrorMessage)-1, "E: %s %x",
               errIdString, nErrorId);
    } else if ((pC_->features_ & FEATURE_BITS_ECMC) && nErrorId) {
      /* emcmc has error messages */
      snprintf(drvlocal.sErrorMessage, sizeof(drvlocal.sErrorMessage)-1, "E: %s",
               sErrorMessage);
    } else if (nErrorId) {
      snprintf(drvlocal.sErrorMessage, sizeof(drvlocal.sErrorMessage)-1, "E: Cntrl Error %x", nErrorId);
    }
    /* The poller will update the MsgTxt field */
    // updateMsgTxtFromDriver(drvlocal.sErrorMessage);
  }
  callParamCallbacksUpdateError();

  memcpy(&drvlocal.old_st_axis_status, &st_axis_status,
         sizeof(drvlocal.old_st_axis_status));
  return asynSuccess;

//skip:
  return asynError;
}

/** Set the motor closed loop status
 * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus EthercatMCAxisEcmc::setClosedLoop(bool closedLoop)
{
  int value = closedLoop ? 1 : 0;
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%ssetClosedLoop(%d)=%d\n",  modNamEMC, axisNo_, value);
  if (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    return enableAmplifier(value);
  }
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::setIntegerParam(int function, int value)
{
  asynStatus status;
  unsigned indexGroup5000 = 0x5000;
  if (function == pC_->motorUpdateStatus_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d motorUpdateStatus_)=%d\n", modNamEMC, axisNo_, value);
  } else if (function == pC_->motorStatusCommsError_) {
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
  } else if (function == pC_->motorPowerAutoOnOff_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d motorPowerAutoOnOff_)=%d\n", modNamEMC, axisNo_, value);
#endif
  } else if (function == pC_->EthercatMCHomProc_) {
    int motorNotHomedProblem = 0;
    setIntegerParam(pC_->EthercatMCHomProc_RB_, value);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d HomProc_)=%d motorNotHomedProblem=%d\n",
              modNamEMC, axisNo_, value, motorNotHomedProblem);
  } else if (function == pC_->EthercatMCErrRst_) {
    if (value) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%ssetIntegerParam(%d ErrRst_)=%d\n",
                modNamEMC, axisNo_, value);
      /*  We do not want to call the base class */
      return resetAxis();
    }
    /* If someone writes 0 to the field, just ignore it */
    return asynSuccess;
  } else if (function == pC_->EthercatMCCfgDHLM_En_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d EthercatMCCfgDHLM_En)=%d\n",
              modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xC, value);
    readBackSoftLimits();
    return status;
  } else if (function == pC_->EthercatMCCfgDLLM_En_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d EthercatMCCfgDLLM_En)=%d\n",
              modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xB, value);
    readBackSoftLimits();
    return status;
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
asynStatus EthercatMCAxisEcmc::setDoubleParam(int function, double value)
{
  asynStatus status;
  unsigned indexGroup5000 = 0x5000;

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
#ifdef EthercatMCHVELFRMString
  } else if (function == pC_->EthercatMCHVELfrm_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d HVELfrm_)=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef EthercatMCHomPosString
  } else if (function == pC_->EthercatMCHomPos_) {
    pC_->setDoubleParam(axisNo_, pC_->EthercatMCHomPos_RB_, value);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d HomPos_)=%f\n", modNamEMC, axisNo_, value);
#endif
  } else if (function == pC_->EthercatMCCfgDHLM_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d EthercatMCCfgDHLM_)=%f\n", modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xE, value);
    readBackSoftLimits();
    return status;
  } else if (function == pC_->EthercatMCCfgDLLM_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d EthercatMCCfgDLLM_)=%f\n", modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xD, value);
    readBackSoftLimits();
    return status;
  } else if (function == pC_->EthercatMCCfgVELO_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d EthercatMCCfgVELO_)=%f\n", modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(0x4000, 0x9, value);
    return status;
  } else if (function == pC_->EthercatMCCfgVMAX_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d EthercatMCCfgVMAX_)=%f\n", modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(0x4000, 0x27, value);
    return status;
  } else if (function == pC_->EthercatMCCfgJVEL_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d EthercatMCCfgJVEL_)=%f\n", modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(0x4000, 0x8, value);
    return status;
  } else if (function == pC_->EthercatMCCfgACCS_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d EthercatMCCfgACCS_)=%f\n", modNamEMC, axisNo_, value);
    status = setSAFValueOnAxis(0x4000, 0x101, value);
    return status;
  }
  // Call the base class method
  status = asynMotorAxis::setDoubleParam(function, value);
  return status;
}

asynStatus EthercatMCAxisEcmc::setStringParamDbgStrToMcu(const char *value)
{
  const char * const Main_this_str = "Main.this.";
  const char * const Sim_this_str = "Sim.this.";

  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%ssetStringParamDbgStrToMcu(%d)=\"%s\"\n",
            modNamEMC, axisNo_, value);
  /* empty strings are not send to the controller */
  if (!value[0]) return asynSuccess;

  /* Check the string. E.g. Main.this. and Sim.this. are passed
     as Main.M1 or Sim.M1
     ADR commands are handled below */
  if (!strncmp(value, Main_this_str, strlen(Main_this_str))) {
    snprintf(pC_->outString_, sizeof(pC_->outString_), "%sMain.M%d.%s",
             drvlocal.adsport_str, axisNo_, value + strlen(Main_this_str));
    return pC_->writeReadACK(ASYN_TRACE_INFO);
  }
  /* caput IOC:m1-DbgStrToMCU Sim.this.log=M1.log */
  if (!strncmp(value, Sim_this_str, strlen(Sim_this_str))) {
    snprintf(pC_->outString_, sizeof(pC_->outString_), "Sim.M%d.%s",
             axisNo_, value + strlen(Sim_this_str));
    return pC_->writeReadACK(ASYN_TRACE_INFO);
  }
  /* If we come here, the command was not understood */
  return asynError;
}

asynStatus EthercatMCAxisEcmc::setStringParam(int function, const char *value)
{
  if (function == pC_->EthercatMCDbgStrToLog_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetStringParamDbgStrToLog(%d)=\"%s\"\n",
              modNamEMC, axisNo_, value);
  } else if (function == pC_->EthercatMCDbgStrToMcu_) {
    return setStringParamDbgStrToMcu(value);
  }
  /* Call base class method */
  return asynMotorAxis::setStringParam(function, value);
}

#ifndef motorMessageTextString
void EthercatMCAxisEcmc::updateMsgTxtFromDriver(const char *value)
{
  if (value && value[0]) {
    setStringParam(pC_->EthercatMCMCUErrMsg_,value);
  } else {
    setStringParam(pC_->EthercatMCMCUErrMsg_, "");
  }
}
#endif


// asynStatus EthercatMCAxisEcmc::readConfigLine(const char *line, const char **errorTxt_p)
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


// asynStatus EthercatMCAxisEcmc::readConfigFile(void)
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

asynStatus EthercatMCAxisEcmc::setSAFValueOnAxisVerify(unsigned indexGroup,
                                                   unsigned indexOffset,
                                                   int value,
                                                   unsigned int retryCount)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  int rbvalue = 0 - value;
  while (counter < retryCount) {
    status = getSAFValueFromAxisPrint(indexGroup, indexOffset, "value=", &rbvalue);
    if (status) break;
    if (rbvalue == value) break;
    status = setSAFValueOnAxis(indexGroup, indexOffset, value);
    counter++;
    if (status) break;
    epicsThreadSleep(.1);
  }
  return status;
}

/** Sets a floating point value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (floating point) variable to be updated
 *
 */
asynStatus EthercatMCAxisEcmc::setValueOnAxis(const char* var, double value)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "%sMain.M%d.%s=%g",
           drvlocal.adsport_str, axisNo_, var, value);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

/** Sets 2 floating point value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (floating point) variable to be updated
 *
 */
asynStatus EthercatMCAxisEcmc::setValuesOnAxis(const char* var1, double value1,
                                           const char* var2, double value2)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "%sMain.M%d.%s=%g;%sMain.M%d.%s=%g",
           drvlocal.adsport_str, axisNo_, var1, value1,
           drvlocal.adsport_str, axisNo_, var2, value2);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

asynStatus EthercatMCAxisEcmc::getSAFValueFromAxisPrint(unsigned indexGroup,
                                                    unsigned indexOffset,
                                                    const char *name,
                                                    int *value)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  int res;
  int nvals;
  asynStatus status;
  if (axisId_ <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,2,2?",
          501, indexGroup + axisId_, indexOffset);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%d", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
              modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sout=%s in=%s %s=%d\n",
            modNamEMC, pC_->outString_, pC_->inString_,name, res);
  *value = res;
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::getSAFValueFromAxisPrint(unsigned indexGroup,
                                                    unsigned indexOffset,
                                                    const char *name,
                                                    double *value)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  double res;
  int nvals;
  asynStatus status;
  if (axisId_ <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,8,5?",
          501, indexGroup + axisId_, indexOffset);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
               modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sout=%s in=%s %s=%g\n",
            modNamEMC, pC_->outString_, pC_->inString_,name, res);
  *value = res;
  return asynSuccess;
}


/** Gets an integer or boolean value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the integer result
 *
 */
asynStatus EthercatMCAxisEcmc::getValueFromAxis(const char* var, int *value)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  asynStatus status;
  int res;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "%sMain.M%d%s?",
           drvlocal.adsport_str, axisNo_, var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  if (var[0] == 'b') {
    if (!strcmp(pC_->inString_, "0")) {
      res = 0;
    } else if (!strcmp(pC_->inString_, "1")) {
      res = 1;
    } else {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%scommand=\"%s\" response=\"%s\"\n",
                modNamEMC, pC_->outString_, pC_->inString_);
      return asynError;
    }
  } else {
    int nvals = sscanf(pC_->inString_, "%d", &res);
    if (nvals != 1) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%snvals=%d command=\"%s\" response=\"%s\"\n",
                 modNamEMC, nvals, pC_->outString_, pC_->inString_);
      return asynError;
    }
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sout=%s in=%s status=%s (%d) iValue=%d\n",
            modNamEMC,
            pC_->outString_, pC_->inString_,
            EthercatMCstrStatus(status), (int)status, res);

  *value = res;
  return asynSuccess;
}

/** Gets an integer (or boolean) and a double value from an axis and print
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the integer result
 *
 */
asynStatus EthercatMCAxisEcmc::getSAFValuesFromAxisPrint(unsigned iIndexGroup,
                                                     unsigned iIndexOffset,
                                                     const char *iname,
                                                     int *iValue,
                                                     unsigned fIndexGroup,
                                                     unsigned fIndexOffset,
                                                     const char *fname,
                                                     double *fValue)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  int iRes;
  int nvals;
  double fRes;
  asynStatus status;
  if (axisId_ <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,2,2?;ADSPORT=%u/.ADR.16#%X,16#%X,8,5?",
          501, iIndexGroup + axisId_, iIndexOffset,
          501, fIndexGroup + axisId_, fIndexOffset);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%d;%lf", &iRes, &fRes);
  if (nvals != 2) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
               modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sout=%s in=%s %s=%d %s=%g\n",
            modNamEMC, pC_->outString_, pC_->inString_, iname, iRes, fname, fRes);

  *iValue = iRes;
  *fValue = fRes;
  return asynSuccess;

}

/** Gets a floating point value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the double result
 *
 */
asynStatus EthercatMCAxisEcmc::getValueFromAxis(const char* var, double *value)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  asynStatus status;
  int nvals;
  double res;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "%sMain.M%d%s?",
           drvlocal.adsport_str, axisNo_, var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
               modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}

/** Gets a string value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the string result
 *
 */
asynStatus EthercatMCAxisEcmc::getStringFromAxis(const char *var, char *value, size_t maxlen)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  value[0] = '\0'; /* Always have a valid string */
    asynStatus status;
    snprintf(pC_->outString_, sizeof(pC_->outString_),
             "%sMain.M%d.%s?", drvlocal.adsport_str, axisNo_, var);
    status = pC_->writeReadOnErrorDisconnect();
    if (status) return status;
    memcpy(value, pC_->inString_, maxlen);
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::getValueFromController(const char* var, double *value)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  asynStatus status;
  int nvals;
  double res;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "%s?", var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n",
               modNamEMC, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}


asynStatus EthercatMCAxisEcmc::setSAFValueOnAxis(unsigned indexGroup,
                                             unsigned indexOffset,
                                             int value)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  if (axisId_ <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,2,2=%d",
          501, indexGroup + axisId_, indexOffset, value);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

asynStatus EthercatMCAxisEcmc::setSAFValueOnAxis(unsigned indexGroup,
                                             unsigned indexOffset,
                                             double value)
{
  if (axisId_ <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "ADSPORT=%u/.ADR.16#%X,16#%X,8,5=%g",
          501, indexGroup + axisId_, indexOffset, value);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

/** Sets an integer or boolean value on an axis, read it back and retry if needed
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] name of the variable where we can read back
 * \param[in] value the (integer) variable to be updated
 * \param[in] number of retries
 */
asynStatus EthercatMCAxisEcmc::setValueOnAxisVerify(const char *var, const char *rbvar,
                                                int value, unsigned int retryCount)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  int rbvalue = 0 - value;
  while (counter <= retryCount) {
    snprintf(pC_->outString_, sizeof(pC_->outString_),
             "%sMain.M%d.%s=%d;%sMain.M%d.%s?",
             drvlocal.adsport_str, axisNo_, var, value,
             drvlocal.adsport_str, axisNo_, rbvar);
    status = pC_->writeReadOnErrorDisconnect();
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetValueOnAxisVerify(%d) out=%s in=%s status=%s (%d)\n",
              modNamEMC, axisNo_,pC_->outString_, pC_->inString_,
              EthercatMCstrStatus(status), (int)status);
    if (status) {
      return status;
    } else {
      int nvals = sscanf(pC_->inString_, "OK;%d", &rbvalue);
      if (nvals != 1) {
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "%snvals=%d command=\"%s\" response=\"%s\"\n",
                  modNamEMC, nvals, pC_->outString_, pC_->inString_);
        return asynError;
      }
      if (status) break;
      if (rbvalue == value) break;
      counter++;
      epicsThreadSleep(.1);
    }
  }
  /* Verification failed.
     Store the error (unless there was an error before) */
  if ((rbvalue != value) && !drvlocal.cmdErrorMessage[0]) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%ssetValueOnAxisV(%d) var=%s value=%d rbvalue=%d",
                modNamEMC, axisNo_,var, value, rbvalue);
      snprintf(drvlocal.cmdErrorMessage, sizeof(drvlocal.cmdErrorMessage)-1,
               "E: setValueOnAxisV(%s) value=%d rbvalue=%d",
               var, value, rbvalue);

      /* The poller co-ordinates the writing into the parameter library */
  }
  return status;
}

asynStatus EthercatMCAxisEcmc::setSAFValueOnAxisVerify(unsigned indexGroup,
                                                   unsigned indexOffset,
                                                   double value,
                                                   unsigned int retryCount)
{
  printf("Dbg################# %s\n",__FUNCTION__);
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  double rbvalue = 0 - value;
  while (counter < retryCount) {
    status = getSAFValueFromAxisPrint(indexGroup, indexOffset, "value", &rbvalue);
    if (status) break;
    if (rbvalue == value) break;
    status = setSAFValueOnAxis(indexGroup, indexOffset, value);
    counter++;
    if (status) break;
    epicsThreadSleep(.1);
  }
  return status;
}

/** Sets an integer or boolean value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (integer) variable to be updated
 *
 */
asynStatus EthercatMCAxisEcmc::setValueOnAxis(const char* var, int value)
{
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "%sMain.M%d.%s=%d",
           drvlocal.adsport_str, axisNo_, var, value);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

/** 
 * Connect to ECMC axis over SyncIO interface
 * 
 *  All read / write parameters are connected:
 *  1. "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.statusbin?"
 *  2. "T_SMP_MS=%d/TYPE=asynInt8ArrayIn/ax%d.controlbin?"
 * 
*/
asynStatus EthercatMCAxisEcmc::connectEcmcAxis() {

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
}

asynStatus EthercatMCAxisEcmc::readStatusBin() {
  
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
}

asynStatus EthercatMCAxisEcmc::readControlBin() {

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
}

asynStatus EthercatMCAxisEcmc::writeControlBin(ecmcAsynClinetCmdType controlStruct) {

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
}

asynStatus EthercatMCAxisEcmc::readAll() {
  
  asynStatus status;

  status =readStatusBin();
  if(status) {
    return status;
  }

  return readControlBin();
}

//Just for debug
asynStatus EthercatMCAxisEcmc::printDiagBinData() {
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

asynStatus EthercatMCAxisEcmc::uglyConvertFunc(ecmcAxisStatusType*in ,st_axis_status_type *out) {
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

asynStatus EthercatMCAxisEcmc::statBinDataCallback(epicsInt8 *data){
  memcpy(&statusBinData_,data,sizeof(ecmcAxisStatusType));
  return asynSuccess;
}

asynStatus EthercatMCAxisEcmc::contBinDataCallback(epicsInt8 *data){
  memcpy(&controlBinDataRB_,data,sizeof(ecmcAsynClinetCmdType));
  return asynSuccess;
}
