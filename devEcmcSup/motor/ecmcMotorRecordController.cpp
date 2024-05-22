/*
FILENAME... ecmcMotorRecordController.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include <iocsh.h>
#include <epicsExit.h>
#include <epicsThread.h>

#include <epicsExport.h>
#include "ecmcMotorRecordAxis.h"
#include "ecmcMotorRecordController.h"
#include "ecmcGlobalsExtern.h"


static const char *driverName = "ecmcMotorController";

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif // ifndef ASYN_TRACE_INFO

#define ECMC_MR_CNTRL_ADDR 0

extern asynUser *pPrintOutAsynUser;

const char *modNamEMC = "ecmcMotorRecord:: ";

const static char *const strEcmcCreateControllerDef =
  "ecmcMotorRecordCreateController";
const static char *const strEcmcCreateAxisDef =
  "ecmcMotorRecordCreateAxis";

const static unsigned reportedFeatureBits =
  FEATURE_BITS_ECMC |
  FEATURE_BITS_V2;

extern "C"
double ecmcMotorRecordgetNowTimeSecs(void) {
  epicsTimeStamp nowTime;

  epicsTimeGetCurrent(&nowTime);
  return nowTime.secPastEpoch + (nowTime.nsec * 0.000000001);
}

extern "C" const char* errStringFromErrId(int nErrorId) {
  switch (nErrorId) {
  case 0x4221:
    return "Velo not allowed";

  case 0x4223:
    return "Axis positioning enable";

  case 0x4450:
  case 0x4451:
    return "Follow error";

  case 0x4260:
    return "Amplifier off";

  case 0x4263:
    return "Is still proc";

  case 0x42A0:
    return "Consequ Err";

  case 0x4460:
    return "Low soft limit";

  case 0x4461:
    return "High soft limit";

  case 0x4462:
    return "Min position";

  case 0x4463:
    return "Max position";

  case 0x4464:
    return "HW fault";

  case 0x4550:
    return "Follow err pos";

  case 0x4551:
    return "Follow err vel";

  case 0x4650:
    return "Drv HW not rdy";

  case 0x4655:
    return "Inv IO data";

  case 0x4B09:
    return "Axis not ready";

  case 0x4B0A:
    return "Homing failed";

  default:
    return "";
  }
}

extern "C" const char* ecmcMotorRecordstrStatus(asynStatus status) {
  switch ((int)status) {
  case asynSuccess:             return "asynSuccess";

  case asynTimeout:             return "asynTimeout";

  case asynOverflow:            return "asynOverflow";

  case asynError:               return "asynError";

  case asynDisconnected:        return "asynDisconnected";

  case asynDisabled:            return "asynDisabled";

  case asynParamAlreadyExists:  return "asynParamAlreadyExists";

  case asynParamNotFound:       return "asynParamNotFound";

  case asynParamWrongType:      return "asynParamWrongType";

  case asynParamBadIndex:       return "asynParamBadIndex";

  case asynParamUndefined:      return "asynParamUndefined";

  default: return "??";
  }
}

/** Creates a new ecmcMotorRecordController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MotorPortName     The name of the drvAsynSerialPort that was created previously to connect to the ecmcMotorRecord controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
ecmcMotorRecordController::ecmcMotorRecordController(const char *portName,
                                                     const char *MotorPortName,

                                                     // Not needed..
                                                     int         numAxes,
                                                     double      movingPollPeriod,
                                                     double      idlePollPeriod,
                                                     const char *optionStr)
  :  asynMotorController(portName, ECMC_MAX_AXES, NUM_VIRTUAL_MOTOR_PARAMS,
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0) { // Default priority and stack size
  /* Controller */
  memset(&ctrlLocal, 0, sizeof(ctrlLocal));
  pAxes_ = (ecmcMotorRecordAxis **)(asynMotorController::pAxes_);
  ctrlLocal.movingPollPeriod = movingPollPeriod;
  ctrlLocal.idlePollPeriod   = idlePollPeriod;
  ctrlLocal.oldStatus        = asynDisconnected;
  features_                  = FEATURE_BITS_V2 | FEATURE_BITS_ECMC;
  profileInitialized_        = 0;
  profileBuilt_              = 0;
#ifndef motorMessageTextString
  createParam("MOTOR_MESSAGE_TEXT",
              asynParamOctet,
              &ecmcMotorRecordMCUErrMsg_);
#else // ifndef motorMessageTextString
  createParam(ecmcMotorRecordMCUErrMsgString,
              asynParamOctet,
              &ecmcMotorRecordMCUErrMsg_);
#endif // ifndef motorMessageTextString
  createParam(ecmcMotorRecordDbgStrToMcuString,
              asynParamOctet,
              &ecmcMotorRecordDbgStrToMcu_);
  createParam(ecmcMotorRecordDbgStrToLogString,
              asynParamOctet,
              &ecmcMotorRecordDbgStrToLog_);

  /* Per axis */
  createParam(ecmcMotorRecordErrString,
              asynParamInt32,
              &ecmcMotorRecordErr_);
  createParam(ecmcMotorRecordErrIdString,
              asynParamInt32,
              &ecmcMotorRecordErrId_);

  createParam(ecmcMotorRecordEnc_ActString,
              asynParamFloat64,
              &ecmcMotorRecordEncAct_);
  createParam(ecmcMotorRecordHomProcString,
              asynParamInt32,
              &ecmcMotorRecordHomProc_);
  createParam(ecmcMotorRecordHomPosString,
              asynParamFloat64,
              &ecmcMotorRecordHomPos_);
  createParam(ecmcMotorRecordStatusCodeString,
              asynParamInt32,
              &ecmcMotorRecordStatusCode_);
  createParam(ecmcMotorRecordStatusBitsString,
              asynParamInt32,
              &ecmcMotorRecordStatusBits_);
  createParam(ecmcMotorRecordHomProc_RBString,
              asynParamInt32,
              &ecmcMotorRecordHomProc_RB_);
  createParam(ecmcMotorRecordHomPos_RBString,
              asynParamFloat64,
              &ecmcMotorRecordHomPos_RB_);
  createParam(ecmcMotorRecordVelToHomString,
              asynParamFloat64,
              &ecmcMotorRecordVelToHom_);
  createParam(ecmcMotorRecordVelFrmHomString,
              asynParamFloat64,
              &ecmcMotorRecordVelFrmHom_);
  createParam(ecmcMotorRecordAccHomString,
              asynParamFloat64,
              &ecmcMotorRecordAccHom_);
  createParam(ecmcMotorRecordErrRstString,
              asynParamInt32,
              &ecmcMotorRecordErrRst_);
  createParam(ecmcMotorRecordVelActString,
              asynParamFloat64,
              &ecmcMotorRecordVelAct_);
  createParam(ecmcMotorRecordVel_RBString,
              asynParamFloat64,
              &ecmcMotorRecordVel_RB_);
  createParam(ecmcMotorRecordAcc_RBString,
              asynParamFloat64,
              &ecmcMotorRecordAcc_RB_);
  createParam(ecmcMotorRecordaux0_String,
              asynParamOctet,
              &ecmcMotorRecordaux0_);
  createParam(ecmcMotorRecordaux1_String,
              asynParamOctet,
              &ecmcMotorRecordaux1_);
  createParam(ecmcMotorRecordaux2_String,
              asynParamOctet,
              &ecmcMotorRecordaux2_);
  createParam(ecmcMotorRecordaux3_String,
              asynParamOctet,
              &ecmcMotorRecordaux3_);
  createParam(ecmcMotorRecordaux4_String,
              asynParamOctet,
              &ecmcMotorRecordaux4_);
  createParam(ecmcMotorRecordaux5_String,
              asynParamOctet,
              &ecmcMotorRecordaux5_);
  createParam(ecmcMotorRecordaux6_String,
              asynParamOctet,
              &ecmcMotorRecordaux6_);
  createParam(ecmcMotorRecordaux7_String,
              asynParamOctet,
              &ecmcMotorRecordaux7_);
  createParam(ecmcMotorRecordreason24_String,
              asynParamOctet,
              &ecmcMotorRecordreason24_);
  createParam(ecmcMotorRecordreason25_String,
              asynParamOctet,
              &ecmcMotorRecordreason25_);
  createParam(ecmcMotorRecordreason26_String,
              asynParamOctet,
              &ecmcMotorRecordreason26_);
  createParam(ecmcMotorRecordreason27_String,
              asynParamOctet,
              &ecmcMotorRecordreason27_);
  createParam(ecmcMotorRecordCfgVELO_String,
              asynParamFloat64,
              &ecmcMotorRecordCfgVELO_);
  createParam(ecmcMotorRecordCfgVMAX_String,
              asynParamFloat64,
              &ecmcMotorRecordCfgVMAX_);
  createParam(ecmcMotorRecordCfgJVEL_String,
              asynParamFloat64,
              &ecmcMotorRecordCfgJVEL_);
  createParam(ecmcMotorRecordCfgACCS_String,
              asynParamFloat64,
              &ecmcMotorRecordCfgACCS_);
  createParam(ecmcMotorRecordCfgDHLMString,
              asynParamFloat64,
              &ecmcMotorRecordCfgDHLM_);
  createParam(ecmcMotorRecordCfgDLLMString,
              asynParamFloat64,
              &ecmcMotorRecordCfgDLLM_);
  createParam(ecmcMotorRecordCfgDHLM_EnString,
              asynParamInt32,
              &ecmcMotorRecordCfgDHLM_En_);
  createParam(ecmcMotorRecordCfgDLLM_EnString,
              asynParamInt32,
              &ecmcMotorRecordCfgDLLM_En_);

  createParam(ecmcMotorRecordCfgSREV_RBString,
              asynParamFloat64,
              &ecmcMotorRecordCfgSREV_RB_);
  createParam(ecmcMotorRecordCfgUREV_RBString,
              asynParamFloat64,
              &ecmcMotorRecordCfgUREV_RB_);
  createParam(ecmcMotorRecordCfgPMIN_RBString,
              asynParamFloat64,
              &ecmcMotorRecordCfgPMIN_RB_);
  createParam(ecmcMotorRecordCfgPMAX_RBString,
              asynParamFloat64,
              &ecmcMotorRecordCfgPMAX_RB_);
  createParam(ecmcMotorRecordCfgSPDB_RBString,
              asynParamFloat64,
              &ecmcMotorRecordCfgSPDB_RB_);
  createParam(ecmcMotorRecordCfgRDBD_RBString,
              asynParamFloat64,
              &ecmcMotorRecordCfgRDBD_RB_);
  createParam(ecmcMotorRecordCfgRDBD_Tim_RBString,
              asynParamFloat64,
              &ecmcMotorRecordCfgRDBD_Tim_RB_);
  createParam(ecmcMotorRecordCfgRDBD_En_RBString,
              asynParamInt32,
              &ecmcMotorRecordCfgRDBD_En_RB_);
  createParam(ecmcMotorRecordCfgPOSLAG_RBString,
              asynParamFloat64,
              &ecmcMotorRecordCfgPOSLAG_RB_);
  createParam(ecmcMotorRecordCfgPOSLAG_Tim_RBString,
              asynParamFloat64,
              &ecmcMotorRecordCfgPOSLAG_Tim_RB_);
  createParam(ecmcMotorRecordCfgPOSLAG_En_RBString,
              asynParamInt32,
              &ecmcMotorRecordCfgPOSLAG_En_RB_);

  createParam(ecmcMotorRecordCfgDESC_RBString,
              asynParamOctet,
              &ecmcMotorRecordCfgDESC_RB_);
  createParam(ecmcMotorRecordCfgEGU_RBString,
              asynParamOctet,
              &ecmcMotorRecordCfgEGU_RB_);

  createParam(ecmcMotorRecordTRIGG_STOPString,
              asynParamInt32,
              &ecmcMotorRecordTRIGG_STOPP_);

  createParam(ecmcMotorRecordTRIGGDISABLEString,
              asynParamInt32,
              &ecmcMotorRecordTRIGG_DISABLE_);

#ifdef CREATE_MOTOR_REC_RESOLUTION

  /* Latest asynMotorController does this, but not the version in 6.81 (or 6.9x) */
  createParam(motorRecResolutionString, asynParamFloat64,
              &motorRecResolution_);
  createParam(motorRecDirectionString,  asynParamInt32,   &motorRecDirection_);
  createParam(motorRecOffsetString,     asynParamFloat64, &motorRecOffset_);
#endif // ifdef CREATE_MOTOR_REC_RESOLUTION

  setIntegerParam(profileCurrentPoint_, 0);
  setIntegerParam(profileActualPulses_, 0);
  setIntegerParam(profileNumReadbacks_, 0);
  
  callParamCallbacks();
  startPoller(movingPollPeriod, idlePollPeriod, 2);

}

ecmcMotorRecordController::~ecmcMotorRecordController() {}

/** Creates a new ecmcMotorRecordController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MotorPortName     Obsolete.. Not used kept to keep same syntax. The name of the drvAsynIPPPort that was created previously to connect to the ecmcMotorRecord controller
  * \param[in] numAxes           The number of axes that this controller supports (0 is not used)
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int ecmcMotorRecordCreateController(const char *portName,
                                               const char *MotorPortName,
                                               int         numAxes,
                                               int         movingPollPeriod,
                                               int         idlePollPeriod,
                                               const char *optionStr) {
  if (!portName || !MotorPortName || !movingPollPeriod || !idlePollPeriod) {
    printf("\n");
    printf(
      "Iocsh command to create a model 3 asyn motor record driver for use with ECMC.\n");
    printf(
      "Creates an ecmcMotorRecordController object (derived from asynMotorController).\n");
    printf("\n");
    printf("ecmcMotorRecordCreateController(\n");
    printf(
      "    portName         : Asyn port name for this motor record driver.                        : \"ECMC_ASYN_MOTOR_PORT\"\n");
    printf(
      "    MotorPortName    : Obsolete. Not used. Kept to keep syntax same as EthercatMC module.  : \"NOT_USED\"\n");
    printf(
      "    numAxes          : Obsolete. Not used. Kept to keep syntax same as EthercatMC module.  : \"0\"\n");
    printf("                       numAxes already defined in ecmc to %d\n",
           ECMC_MAX_AXES);
    printf(
      "    movingPollPeriod : Scan period rate of motor record status update when axis is moving. : \"200\" (unit [ms])\n");
    printf(
      "    idlePollPeriod   : Scan rate of motor record status update when moving.                : \"1000\" (unit [ms])\n");
    printf(
      "    optionStr        : Currently Not used. Optional options string.                        : \"\" \n");
    printf(")\n");
    printf("Example:\n");
    printf(
      "ecmcMotorRecordCreateController(\"ECMC_ASYN_MOTOR_PORT\",\"NOT_USED\",0,0.2,1.0,\"\")\n");
    printf("\n");
    return asynError;
  }

  if (asynPortMotorRecord) {
    printf(
      "Error: ecmcMotorRecordController already initialized. ECMC only supports one ecmcMotorRecordController\n");
    return asynError;
  }

  asynPortMotorRecord = new ecmcMotorRecordController(portName,
                                                      MotorPortName,
                                                      numAxes + 1,
                                                      movingPollPeriod / 1000.0,
                                                      idlePollPeriod / 1000.0,
                                                      optionStr);
  return asynSuccess;
}

asynStatus ecmcMotorRecordController::setMCUErrMsg(const char *value) {
  asynStatus status = setStringParam(ecmcMotorRecordMCUErrMsg_, value);

  if (!status) status = callParamCallbacks();
  return status;
}

void ecmcMotorRecordController::udateMotorLimitsRO(int axisNo) {
  double fValueHigh = 0.0, fValueLow = 0.0;
  int    enabledHigh = 0, enabledLow = 0;

  /* When the integer parameter is undefined, 0 is returned,
     same as not enabled */
  getIntegerParam(axisNo, ecmcMotorRecordCfgDHLM_En_, &enabledHigh);
  getIntegerParam(axisNo, ecmcMotorRecordCfgDLLM_En_, &enabledLow);

  if (enabledHigh && enabledLow) {
    asynStatus status1, status2;
    status1 = getDoubleParam(axisNo, ecmcMotorRecordCfgDHLM_, &fValueHigh);
    status2 = getDoubleParam(axisNo, ecmcMotorRecordCfgDLLM_, &fValueLow);

    if (status1 || status2) {
      udateMotorLimitsRO(axisNo, 0, 0.0, 0.0);
      return;
    }
  }
  udateMotorLimitsRO(axisNo, enabledHigh && enabledLow, fValueHigh, fValueLow);
}

void ecmcMotorRecordController::udateMotorLimitsRO(int    axisNo,
                                                   int    enabledHighAndLow,
                                                   double fValueHigh,
                                                   double fValueLow) {
#ifdef motorHighLimitROString
  static const double fABSMIN = -3.0e+38;
  static const double fABSMAX =  3.0e+38;
  int valid                   = 1;

  if ((fValueLow >= fValueHigh) ||
      (fValueLow <= fABSMIN) || (fValueHigh >= fABSMAX)) {
    /* Any limit not active or out of range: set everything to 0 */
    valid = 0;
  }

  if (!enabledHighAndLow || !valid) {
    /* Any limit not active or out of range: set everything to 0 */
    fValueHigh = fValueLow  = 0.0;
  }
  asynMotorAxis *pAxis = getAxis(axisNo);

  if (pAxis) {
    double oldValueHigh, oldValueLow;
    getDoubleParam(axisNo, motorHighLimitRO_, &oldValueHigh);
    getDoubleParam(axisNo, motorLowLimitRO_,  &oldValueLow);

    if ((fValueHigh != oldValueHigh) || (fValueLow != oldValueLow)) {
      asynPrint(pPrintOutAsynUser,
                ASYN_TRACE_INFO,
                "%sudateMotorLimitsRO(%d) enabledHighAndLow=%d valid=%d fValueHigh=%g fValueLow=%g\n",
                modNamEMC,
                axisNo,
                enabledHighAndLow,
                valid,
                fValueHigh,
                fValueLow);
    }


    /* We need the overload function from asynMotorAxis to
       let the values ripple into the motorRecord */
    pAxis->setDoubleParam(motorHighLimitRO_, fValueHigh);
    pAxis->setDoubleParam(motorLowLimitRO_,  fValueLow);
  }
#else // ifdef motorHighLimitROString
  (void)axisNo;
  (void)enabledHighAndLow;
  (void)fValueHigh;
  (void)fValueLow;
#endif // ifdef motorHighLimitROString
}

void ecmcMotorRecordController::handleStatusChange(asynStatus status) {
  if (status != ctrlLocal.oldStatus) {
    asynPrint(pPrintOutAsynUser,
              ASYN_TRACE_INFO,
              "%soldStatus=%s (%d) status=%s (%d)\n",
              modNamEMC,
              ecmcMotorRecordstrStatus(ctrlLocal.oldStatus),
              (int)ctrlLocal.oldStatus,
              ecmcMotorRecordstrStatus(status),
              (int)status);

    if (status) {
      /* Connected -> Disconnected */
      int i;
      ctrlLocal.initialPollDone = 0;

      /* Keep bits that are specified via options,
         clear bits that are fetched from the controller */
      features_ &= ~reportedFeatureBits;
      setMCUErrMsg("MCU Disconnected");

      for (i = 0; i < numAxes_; i++) {
        asynMotorAxis *pAxis = getAxis(i);

        if (!pAxis) continue;
        pAxis->setIntegerParam(motorStatusCommsError_, 1);
        pAxis->callParamCallbacks();
      }
    } else {
      /* Disconnected -> Connected */
      setMCUErrMsg("MCU Cconnected");
    }
    ctrlLocal.oldStatus = status;
  }
}

asynStatus ecmcMotorRecordController::poll(void) {
  asynStatus status = asynSuccess;

  asynPrint(pPrintOutAsynUser, ASYN_TRACE_FLOW,
            "%spoll ctrlLocal.initialPollDone=%d\n",
            modNamEMC, ctrlLocal.initialPollDone);

  if (!features_) {
    features_ = FEATURE_BITS_V2 | FEATURE_BITS_ECMC; // getFeatures();

    ctrlLocal.initialPollDone = 1;
  }

  //status = setIntegerParam(1,profileCurrentPoint_, 0);
  //printf("Setting profileCurrentPoint_ to 0 (status=%d)\n",(int)status );
  //callParamCallbacks(1);
  return status;
}

// Handle all controller related writes here. All controller params are stored in address/axisNo 0
// All controller related info is stored in axis no 0 and getAxis retruns NULL (axis[0] = NULL).
// Not nice but probablbly mosty generic way todo it
asynStatus ecmcMotorRecordController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  asynMotorAxis *pAxis;
  int axisNo = -1;
  asynStatus status = asynError;
  static const char *functionName = "writeFloat64";

  pAxis = getAxis(pasynUser);

  // If axis is defined then all can be handled in asynMotorController (both axes and controller related)
  if (pAxis) {
    printf("ecmcMotorRecordController::writeFloat64: pAxis is valid\n");
    return asynMotorController::writeFloat64(pasynUser, value);
  }

  // can be controller related (axisNo == 0)
  status = getAddress(pasynUser, &axisNo);
  if(status != asynSuccess || axisNo!=ECMC_MR_CNTRL_ADDR) {
    printf("ecmcMotorRecordController::writeFloat64: getAddress returned error\n");
    return status;
  }

  printf("ecmcMotorRecordController::writeFloat64: Controller related parameter.\n");

  // Must be controller related
  // write to lib
  status = setDoubleParam(ECMC_MR_CNTRL_ADDR,function, value);
  callParamCallbacks();
  return status;
}

// Handle all controller related writes here. All controller params are stored in address/axisNo 0
// All controller related info is stored in axis no 0 and getAxis retruns NULL (axis[0] = NULL).
// Not nice but probablbly mosty generic way todo it
asynStatus ecmcMotorRecordController::writeInt32(asynUser *pasynUser, epicsInt32 value) {

  int function = pasynUser->reason;
  asynStatus status = asynError;
  asynMotorAxis *pAxis;
  int axisNo = -1;
  static const char *functionName = "writeInt32";

  pAxis = getAxis(pasynUser);


  // If axis is defined then all can be handled in asynMotorController (both axes and controller related)
  if (pAxis) {
    printf("ecmcMotorRecordController::writeInt32: pAxis is valid\n");
    return asynMotorController::writeInt32(pasynUser, value);
  }

  // can be controller related (axisNo == 0)
  status = getAddress(pasynUser, &axisNo);
  if(status != asynSuccess || axisNo!=ECMC_MR_CNTRL_ADDR) {
    printf("ecmcMotorRecordController::writeInt32: getAddress returned error\n");
    return status;
  }

  printf("ecmcMotorRecordController::writeInt32: Controller related parameter.\n");

  // Must be controller related
  if (function == motorDeferMoves_) {
    printf("setDeferredMoves\n");
    status = asynMotorController::setDeferredMoves(value);     
  } else if (function == profileBuild_) {
    printf("buildProfile\n");
    status = buildProfile();
  } else if (function == profileExecute_) {
    printf("executeProfile\n");
    status = asynMotorController::executeProfile();
  } else if (function == profileAbort_) {
    printf("abortProfile\n");
    status = asynMotorController::abortProfile();
  } else if (function == profileReadback_) {
    printf("readbackProfile\n");
    status = asynMotorController::readbackProfile();
  }

  if(status != asynSuccess) {
    return status;  
  }

  // write to lib
  status = setIntegerParam(ECMC_MR_CNTRL_ADDR,function, value);
  callParamCallbacks();

  return status;
}

asynStatus ecmcMotorRecordController::buildProfile() {
  printf("ecmcMotorRecordController::buildProfile()\n");  
  if(!profileInitialized_) {
    printf("ecmcMotorRecordController: Error: Profile not initialized...\n");
    return asynError;
  }
  setIntegerParam(ECMC_MR_CNTRL_ADDR,profileBuildState_, PROFILE_BUILD_BUSY);
  
  asynStatus status = asynMotorController::buildProfile();
  
  profileBuilt_ = status == asynSuccess; 

  setIntegerParam(ECMC_MR_CNTRL_ADDR,profileBuildState_, PROFILE_BUILD_DONE);

  if(status != asynSuccess) {
    setIntegerParam(ECMC_MR_CNTRL_ADDR,profileBuildStatus_, PROFILE_STATUS_FAILURE);

    return status;
  }
  setIntegerParam(ECMC_MR_CNTRL_ADDR,profileBuildStatus_, PROFILE_STATUS_SUCCESS);
  
  return status;
}

asynStatus ecmcMotorRecordController::initializeProfile(size_t maxProfilePoints)
{
  printf("ecmcMotorRecordController::initializeProfile(%d)\n",maxProfilePoints);
  asynStatus status = asynMotorController::initializeProfile(maxProfilePoints);
  profileInitialized_ = status == asynSuccess;
  return status;
}

/** Returns a pointer to an ecmcMotorRecordAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
ecmcMotorRecordAxis* ecmcMotorRecordController::getAxis(asynUser *pasynUser)
{
    int axisNo;
    
    getAddress(pasynUser, &axisNo);
    return getAxis(axisNo);
}

/** Returns a pointer to an ecmcMotorRecordAxis object.
  * Returns NULL if the axis number is invalid.
  * \param[in] axisNo Axis index number. */
ecmcMotorRecordAxis* ecmcMotorRecordController::getAxis(int axisNo)
{
    if ((axisNo < 0) || (axisNo >= numAxes_)) return NULL;
    return pAxes_[axisNo];
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void ecmcMotorRecordController::report(FILE *fp, int level) {
  fprintf(fp,
          "ECMC motor record driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
          this->portName,
          numAxes_,
          ctrlLocal.movingPollPeriod,
          ctrlLocal.idlePollPeriod);

  if (level > 0) {
    ecmcMotorRecordAxis *tempAxis = NULL;

    for (int i = 0; i < numAxes_; i++) {
      tempAxis = getAxis(i);

      if (tempAxis) {
        tempAxis->report(fp, level);
      }
    }
  }

  // Call the base class method
  asynMotorController::report(fp,
                              level);
}

/** Code for iocsh registration */
static const iocshArg ecmcMotorRecordCreateControllerArg0 =
{ "Port name", iocshArgString };
static const iocshArg ecmcMotorRecordCreateControllerArg1 =
{ "NOT_USED", iocshArgString };
static const iocshArg ecmcMotorRecordCreateControllerArg2 =
{ "NOT_USED", iocshArgInt };
static const iocshArg ecmcMotorRecordCreateControllerArg3 =
{ "Moving poll period (ms)", iocshArgInt };
static const iocshArg ecmcMotorRecordCreateControllerArg4 =
{ "Idle poll period (ms)", iocshArgInt };
static const iocshArg ecmcMotorRecordCreateControllerArg5 =
{ "options", iocshArgString };
static const iocshArg *const ecmcMotorRecordCreateControllerArgs[] = {
  &ecmcMotorRecordCreateControllerArg0,
  &ecmcMotorRecordCreateControllerArg1,
  &ecmcMotorRecordCreateControllerArg2,
  &ecmcMotorRecordCreateControllerArg3,
  &ecmcMotorRecordCreateControllerArg4,
  &ecmcMotorRecordCreateControllerArg5 };
static const iocshFuncDef    ecmcMotorRecordCreateControllerDef =
{ strEcmcCreateControllerDef, 6,
  ecmcMotorRecordCreateControllerArgs };
static void ecmcMotorRecordCreateContollerCallFunc(const iocshArgBuf *args) {
  ecmcMotorRecordCreateController(args[0].sval, args[1].sval, args[2].ival,
                                  args[3].ival, args[4].ival, args[5].sval);
}

/* ecmcMotorRecordCreateAxis */
static const iocshArg ecmcMotorRecordCreateAxisArg0 =
{ "Controller port name", iocshArgString };
static const iocshArg ecmcMotorRecordCreateAxisArg1 =
{ "Axis number", iocshArgInt };
static const iocshArg ecmcMotorRecordCreateAxisArg2 =
{ "axisFlags", iocshArgInt };
static const iocshArg ecmcMotorRecordCreateAxisArg3 =
{ "axisOptionsStr", iocshArgString };
static const iocshArg * const ecmcMotorRecordCreateAxisArgs[] =
{ &ecmcMotorRecordCreateAxisArg0,
  &
  ecmcMotorRecordCreateAxisArg1,
  &
  ecmcMotorRecordCreateAxisArg2,
  &
  ecmcMotorRecordCreateAxisArg3 };
static const iocshFuncDef ecmcMotorRecordCreateAxisDef =
{ strEcmcCreateAxisDef, 4,
  ecmcMotorRecordCreateAxisArgs };

static void ecmcMotorRecordCreateAxisCallFunc(const iocshArgBuf *args) {
  ecmcMotorRecordCreateAxis(args[0].sval,
                            args[1].ival,
                            args[2].ival,
                            args[3].sval);
}

asynStatus ecmcCreateProfile(const char *asynPort,         /* specify which controller by port name */
                            int maxPoints)               /* maximum number of profile points */
{
  ecmcMotorRecordController *pC;
  static const char *functionName = "ecmcCreateProfile";

  pC = (ecmcMotorRecordController*) findAsynPortDriver(asynPort);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, asynPort);
    return asynError;
  }
  pC->lock();
  pC->initializeProfile(maxPoints);
  pC->unlock();
  return asynSuccess;
}

/* ecmcCreateProfile */
static const iocshArg ecmcCreateProfileArg0 = {"Controller port name", iocshArgString};
static const iocshArg ecmcCreateProfileArg1 = {"Max points", iocshArgInt};
static const iocshArg * const ecmcCreateProfileArgs[] = {&ecmcCreateProfileArg0,
                                                        &ecmcCreateProfileArg1};
static const iocshFuncDef ecmcCreateProfileCallFuncDef = {"ecmcCreateProfile", 2, ecmcCreateProfileArgs};

static void ecmcCreateProfileCallFunc(const iocshArgBuf *args)
{
  ecmcCreateProfile(args[0].sval, args[1].ival);
}

static void ecmcMotorRecordControllerRegister(void) {
  iocshRegister(&ecmcMotorRecordCreateControllerDef,
                ecmcMotorRecordCreateContollerCallFunc);
  iocshRegister(&ecmcMotorRecordCreateAxisDef,
                ecmcMotorRecordCreateAxisCallFunc);
  iocshRegister(&ecmcCreateProfileCallFuncDef,
                ecmcCreateProfileCallFunc);
}

extern "C" {
epicsExportRegistrar(ecmcMotorRecordControllerRegister);
}
