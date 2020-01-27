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

#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#include "ecmcMotorRecordAxis.h"
#include "ecmcMotorRecordController.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

const char *modNamEMC = "ecmcMotorRecord:: ";

const static char *const strEcmcCreateControllerDef  = "ecmcMotorRecordCreateController";
const static char *const strEcmcCreateAxisDef = "ecmcMotorRecordCreateAxis";

const static char *const strCtrlReset = ".ctrl.ErrRst";

const static char *const modulName = "ecmcMotorRecordAxis::";

const static unsigned reportedFeatureBits =
   FEATURE_BITS_ECMC |
   FEATURE_BITS_V2;

extern "C"
double ecmcMotorRecordgetNowTimeSecs(void)
{
    epicsTimeStamp nowTime;
    epicsTimeGetCurrent(&nowTime);
    return nowTime.secPastEpoch + (nowTime.nsec * 0.000000001);
}

extern "C" const char *errStringFromErrId(int nErrorId)
{
  switch(nErrorId) {
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

extern "C" const char *ecmcMotorRecordstrStatus(asynStatus status)
{
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
                                           const char *MotorPortName, int numAxes,
                                           double movingPollPeriod,
                                           double idlePollPeriod,
                                           const char *optionStr)
  :  asynMotorController(portName, numAxes, NUM_VIRTUAL_MOTOR_PARAMS,
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;
  // ECMC
  mcuPortName_      = strdup(MotorPortName);
  movingPollPeriod_ = movingPollPeriod;
  idlePollPeriod_   = idlePollPeriod;
  // ECMC

  /* Controller */
  memset(&ctrlLocal, 0, sizeof(ctrlLocal));
  ctrlLocal.oldStatus = asynDisconnected;  
  features_ = FEATURE_BITS_V2 | FEATURE_BITS_ECMC;
#ifndef motorMessageTextString
  createParam("MOTOR_MESSAGE_TEXT",          asynParamOctet,       &ecmcMotorRecordMCUErrMsg_);
#else
  createParam(ecmcMotorRecordMCUErrMsgString,     asynParamOctet,       &ecmcMotorRecordMCUErrMsg_);
#endif
  createParam(ecmcMotorRecordDbgStrToMcuString,   asynParamOctet,       &ecmcMotorRecordDbgStrToMcu_);
  createParam(ecmcMotorRecordDbgStrToLogString,   asynParamOctet,       &ecmcMotorRecordDbgStrToLog_);

  /* Per axis */
  createParam(ecmcMotorRecordErrString,           asynParamInt32,       &ecmcMotorRecordErr_);
  createParam(ecmcMotorRecordErrIdString,         asynParamInt32,       &ecmcMotorRecordErrId_);

  createParam(ecmcMotorRecordEnc_ActString,       asynParamFloat64,     &ecmcMotorRecordEncAct_);
  createParam(ecmcMotorRecordHomProcString,       asynParamInt32,       &ecmcMotorRecordHomProc_);
  createParam(ecmcMotorRecordHomPosString,        asynParamFloat64,     &ecmcMotorRecordHomPos_);
  createParam(ecmcMotorRecordStatusCodeString,    asynParamInt32,       &ecmcMotorRecordStatusCode_);
  createParam(ecmcMotorRecordStatusBitsString,    asynParamInt32,       &ecmcMotorRecordStatusBits_);
  createParam(ecmcMotorRecordHomProc_RBString,    asynParamInt32,       &ecmcMotorRecordHomProc_RB_);
  createParam(ecmcMotorRecordHomPos_RBString,     asynParamFloat64,     &ecmcMotorRecordHomPos_RB_);
  createParam(ecmcMotorRecordVelToHomString,      asynParamFloat64,     &ecmcMotorRecordVelToHom_);
  createParam(ecmcMotorRecordVelFrmHomString,     asynParamFloat64,     &ecmcMotorRecordVelFrmHom_);
  createParam(ecmcMotorRecordAccHomString,        asynParamFloat64,     &ecmcMotorRecordAccHom_);
  createParam(ecmcMotorRecordErrRstString,        asynParamInt32,       &ecmcMotorRecordErrRst_);
  createParam(ecmcMotorRecordVelActString,        asynParamFloat64,     &ecmcMotorRecordVelAct_);
  createParam(ecmcMotorRecordVel_RBString,        asynParamFloat64,     &ecmcMotorRecordVel_RB_);
  createParam(ecmcMotorRecordAcc_RBString,        asynParamFloat64,     &ecmcMotorRecordAcc_RB_);
  createParam(ecmcMotorRecordaux0_String,         asynParamOctet,       &ecmcMotorRecordaux0_);
  createParam(ecmcMotorRecordaux1_String,         asynParamOctet,       &ecmcMotorRecordaux1_);
  createParam(ecmcMotorRecordaux2_String,         asynParamOctet,       &ecmcMotorRecordaux2_);
  createParam(ecmcMotorRecordaux3_String,         asynParamOctet,       &ecmcMotorRecordaux3_);
  createParam(ecmcMotorRecordaux4_String,         asynParamOctet,       &ecmcMotorRecordaux4_);
  createParam(ecmcMotorRecordaux5_String,         asynParamOctet,       &ecmcMotorRecordaux5_);
  createParam(ecmcMotorRecordaux6_String,         asynParamOctet,       &ecmcMotorRecordaux6_);
  createParam(ecmcMotorRecordaux7_String,         asynParamOctet,       &ecmcMotorRecordaux7_);
  createParam(ecmcMotorRecordreason24_String,     asynParamOctet,       &ecmcMotorRecordreason24_);
  createParam(ecmcMotorRecordreason25_String,     asynParamOctet,       &ecmcMotorRecordreason25_);
  createParam(ecmcMotorRecordreason26_String,     asynParamOctet,       &ecmcMotorRecordreason26_);
  createParam(ecmcMotorRecordreason27_String,     asynParamOctet,       &ecmcMotorRecordreason27_);
  createParam(ecmcMotorRecordCfgVELO_String,      asynParamFloat64,     &ecmcMotorRecordCfgVELO_);
  createParam(ecmcMotorRecordCfgVMAX_String,      asynParamFloat64,     &ecmcMotorRecordCfgVMAX_);
  createParam(ecmcMotorRecordCfgJVEL_String,      asynParamFloat64,     &ecmcMotorRecordCfgJVEL_);
  createParam(ecmcMotorRecordCfgACCS_String,      asynParamFloat64,     &ecmcMotorRecordCfgACCS_);
  createParam(ecmcMotorRecordCfgDHLMString,       asynParamFloat64,     &ecmcMotorRecordCfgDHLM_);
  createParam(ecmcMotorRecordCfgDLLMString,       asynParamFloat64,     &ecmcMotorRecordCfgDLLM_);
  createParam(ecmcMotorRecordCfgDHLM_EnString,    asynParamInt32,       &ecmcMotorRecordCfgDHLM_En_);
  createParam(ecmcMotorRecordCfgDLLM_EnString,    asynParamInt32,       &ecmcMotorRecordCfgDLLM_En_);

  createParam(ecmcMotorRecordCfgSREV_RBString,    asynParamFloat64,     &ecmcMotorRecordCfgSREV_RB_);
  createParam(ecmcMotorRecordCfgUREV_RBString,    asynParamFloat64,     &ecmcMotorRecordCfgUREV_RB_);
  createParam(ecmcMotorRecordCfgPMIN_RBString,    asynParamFloat64,     &ecmcMotorRecordCfgPMIN_RB_);
  createParam(ecmcMotorRecordCfgPMAX_RBString,    asynParamFloat64,     &ecmcMotorRecordCfgPMAX_RB_);
  createParam(ecmcMotorRecordCfgSPDB_RBString,    asynParamFloat64,     &ecmcMotorRecordCfgSPDB_RB_);
  createParam(ecmcMotorRecordCfgRDBD_RBString,    asynParamFloat64,     &ecmcMotorRecordCfgRDBD_RB_);
  createParam(ecmcMotorRecordCfgRDBD_Tim_RBString,asynParamFloat64,     &ecmcMotorRecordCfgRDBD_Tim_RB_);
  createParam(ecmcMotorRecordCfgRDBD_En_RBString, asynParamInt32,       &ecmcMotorRecordCfgRDBD_En_RB_);
  createParam(ecmcMotorRecordCfgPOSLAG_RBString,  asynParamFloat64,     &ecmcMotorRecordCfgPOSLAG_RB_);
  createParam(ecmcMotorRecordCfgPOSLAG_Tim_RBString,asynParamFloat64,   &ecmcMotorRecordCfgPOSLAG_Tim_RB_);
  createParam(ecmcMotorRecordCfgPOSLAG_En_RBString, asynParamInt32,     &ecmcMotorRecordCfgPOSLAG_En_RB_);

  createParam(ecmcMotorRecordCfgDESC_RBString,    asynParamOctet,       &ecmcMotorRecordCfgDESC_RB_);
  createParam(ecmcMotorRecordCfgEGU_RBString,     asynParamOctet,       &ecmcMotorRecordCfgEGU_RB_);

#ifdef CREATE_MOTOR_REC_RESOLUTION
  /* Latest asynMotorController does this, but not the version in 6.81 (or 6.9x) */
  createParam(motorRecResolutionString,        asynParamFloat64,      &motorRecResolution_);
  createParam(motorRecDirectionString,           asynParamInt32,      &motorRecDirection_);
  createParam(motorRecOffsetString,            asynParamFloat64,      &motorRecOffset_);
#endif

  /* Connect to ecmcMotorRecord controller */
  status = pasynOctetSyncIO->connect(MotorPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s cannot connect to motor controller\n", modulName);              
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);

}

ecmcMotorRecordController::~ecmcMotorRecordController() {
  // ECMC
  free(mcuPortName_);
  // ECMC
}

/** Creates a new ecmcMotorRecordController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MotorPortName  The name of the drvAsynIPPPort that was created previously to connect to the ecmcMotorRecord controller
  * \param[in] numAxes           The number of axes that this controller supports (0 is not used)
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int ecmcMotorRecordCreateController(const char *portName,
                                          const char *MotorPortName,
                                          int numAxes,
                                          int movingPollPeriod,
                                          int idlePollPeriod,
                                          const char *optionStr)
{
  new ecmcMotorRecordController(portName, MotorPortName, 1+numAxes,
                           movingPollPeriod/1000., idlePollPeriod/1000.,
                           optionStr);
  return(asynSuccess);
}

extern "C" asynStatus disconnect_C(asynUser *pasynUser)
{
  asynStatus status = asynError;
  asynInterface *pasynInterface = NULL;
  asynCommon     *pasynCommon = NULL;
  pasynInterface = pasynManager->findInterface(pasynUser,
                                               asynCommonType,
                                               0 /* FALSE */);
  if (pasynInterface) {
    pasynCommon = (asynCommon *)pasynInterface->pinterface;
    status = pasynCommon->disconnect(pasynInterface->drvPvt,
                                       pasynUser);
    if (status != asynSuccess) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s status=%s (%d)\n",
                modulName, ecmcMotorRecordstrStatus(status), (int)status);
    }
  } else {
    asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s pasynInterface=%p pasynCommon=%p\n",
              modulName, pasynInterface, pasynCommon);
  }
  return status;
}

extern "C"
asynStatus writeReadOnErrorDisconnect_C(asynUser *pasynUser,
                                        const char *outdata, size_t outlen,
                                        char *indata, size_t inlen)
{
  size_t nwrite;
  asynStatus status = asynError;
  int eomReason = 0;
  size_t nread;
  status = pasynOctetSyncIO->writeRead(pasynUser, outdata, outlen,
                                       indata, inlen,
                                       DEFAULT_CONTROLLER_TIMEOUT,
                                       &nwrite, &nread, &eomReason);
  if ((status == asynTimeout) ||
      (!nread && (eomReason & ASYN_EOM_END))) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s out=%s nread=%lu eomReason=%x (%s%s%s) status=%s (%d)\n",
              modulName, outdata,(unsigned long)nread,
              eomReason,
              eomReason & ASYN_EOM_CNT ? "CNT" : "",
              eomReason & ASYN_EOM_EOS ? "EOS" : "",
              eomReason & ASYN_EOM_END ? "END" : "",
              ecmcMotorRecordstrStatus(status), status);
    disconnect_C(pasynUser);
    return asynError; /* TimeOut -> Error */
  }
  return status;
}

/** Writes a string to the controller and reads a response.
  * Disconnects in case of error
  */
asynStatus ecmcMotorRecordController::writeReadOnErrorDisconnect(void)
{
  asynStatus status = asynError;
  size_t outlen = strlen(outString_);
  double timeBefore = ecmcMotorRecordgetNowTimeSecs();
  inString_[0] = '\0';
  status = writeReadOnErrorDisconnect_C(pasynUserController_, outString_, outlen,
                                        inString_, sizeof(inString_));
  if (!status) {
    if (strstr(inString_, "State timout") ||
        strstr(inString_, "To long time in one state")) {
      double timeDelta = ecmcMotorRecordgetNowTimeSecs() - timeBefore;

      asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%sout=%s in=%s timeDelta=%f\n",
                modNamEMC, outString_, inString_, timeDelta);
      /* Try again */
      timeBefore = ecmcMotorRecordgetNowTimeSecs();
      status = writeReadOnErrorDisconnect_C(pasynUserController_,
                                            outString_, outlen,
                                            inString_, sizeof(inString_));
      timeDelta = ecmcMotorRecordgetNowTimeSecs() - timeBefore;
      asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%sout=%s in=%s timeDelta=%f status=%s (%d)\n",
                modNamEMC, outString_, inString_, timeDelta,
                ecmcMotorRecordstrStatus(status), (int)status);
    }
  }
  handleStatusChange(status);
  if (status)
  {
    return asynError;
  }
  return status;
}

extern "C"
asynStatus checkACK(const char *outdata, size_t outlen,
                    const char *indata)
{
  size_t i;
  unsigned int numOK = 1;
  int res = 1;
  for( i = 0; i < outlen; i++) {
    if (outdata[i] == ';') numOK++;
  }
  switch(numOK) {
    case 1: res = strcmp(indata, "OK");  break;
    case 2: res = strcmp(indata, "OK;OK");  break;
    case 3: res = strcmp(indata, "OK:OK;OK");  break;
    case 4: res = strcmp(indata, "OK;OK;OK;OK");  break;
    default:
    ;
  }
  return res ? asynError : asynSuccess;
}

asynStatus ecmcMotorRecordController::writeReadControllerPrint(int traceMask)
{
  asynStatus status = writeReadOnErrorDisconnect();
  if (status && !ctrlLocal.oldStatus) traceMask |= ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER;
  asynPrint(pasynUserController_, traceMask,
            "%sout=%s in=%s status=%s (%d)\n",
            modNamEMC, outString_, inString_,
            ecmcMotorRecordstrStatus(status), (int)status);
  return status;
}

asynStatus ecmcMotorRecordController::writeReadACK(int traceMask)
{
  asynStatus status = writeReadOnErrorDisconnect();
  switch (status) {
    case asynError:
      return status;
    case asynSuccess:
    {
      const char *semicolon = &outString_[0];
      unsigned int numOK = 1;
      int res = 1;
      while (semicolon && semicolon[0]) {
        semicolon = strchr(semicolon, ';');
        if (semicolon) {
          numOK++;
          semicolon++;
        }
      }
      switch(numOK) {
        case 1: res = strcmp(inString_, "OK");  break;
        case 2: res = strcmp(inString_, "OK;OK");  break;
        case 3: res = strcmp(inString_, "OK:OK;OK");  break;
        case 4: res = strcmp(inString_, "OK;OK;OK;OK");  break;
        case 5: res = strcmp(inString_, "OK;OK;OK;OK;OK");  break;
        case 6: res = strcmp(inString_, "OK;OK;OK;OK;OK;OK");  break;
        case 7: res = strcmp(inString_, "OK;OK;OK;OK;OK;OK;OK");  break;
        case 8: res = strcmp(inString_, "OK;OK;OK;OK;OK;OK;OK;OK");  break;
        default:
          ;
      }
      if (res) {
        status = asynError;
        asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "%sout=%s in=%s return=%s (%d)\n",
                  modNamEMC, outString_, inString_,
                  ecmcMotorRecordstrStatus(status), (int)status);
        return status;
      }
    }
    default:
      break;
  }
  asynPrint(pasynUserController_, traceMask,
            "%sout=%s in=%s status=%s (%d)\n",
            modNamEMC, outString_, inString_,
            ecmcMotorRecordstrStatus(status), (int)status);
  return status;
}

asynStatus ecmcMotorRecordController::setMCUErrMsg(const char *value)
{
  asynStatus status = setStringParam(ecmcMotorRecordMCUErrMsg_, value);
  if (!status) status = callParamCallbacks();
  return status;
}

void ecmcMotorRecordController::udateMotorLimitsRO(int axisNo)
{
  double fValueHigh = 0.0, fValueLow = 0.0;
  int enabledHigh = 0, enabledLow = 0;

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

void ecmcMotorRecordController::udateMotorLimitsRO(int axisNo, int enabledHighAndLow,
                                              double fValueHigh, double fValueLow)
{
#ifdef motorHighLimitROString
  static const double fABSMIN = -3.0e+38;
  static const double fABSMAX =  3.0e+38;
  int valid = 1;
  if (fValueLow >= fValueHigh ||
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
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%sudateMotorLimitsRO(%d) enabledHighAndLow=%d valid=%d fValueHigh=%g fValueLow=%g\n",
                modNamEMC, axisNo,
                enabledHighAndLow, valid, fValueHigh, fValueLow);
    }


    /* We need the overload function from asynMotorAxis to
       let the values ripple into the motorRecord */
    pAxis->setDoubleParam(motorHighLimitRO_, fValueHigh);
    pAxis->setDoubleParam(motorLowLimitRO_,  fValueLow);
  }
#else
  (void)axisNo;
  (void)enabledHighAndLow;
  (void)fValueHigh;
  (void)fValueLow;
#endif

}

void ecmcMotorRecordController::handleStatusChange(asynStatus status)
{
  if (status != ctrlLocal.oldStatus) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%soldStatus=%s (%d) status=%s (%d)\n",
              modNamEMC,
              ecmcMotorRecordstrStatus(ctrlLocal.oldStatus), (int)ctrlLocal.oldStatus,
              ecmcMotorRecordstrStatus(status), (int)status);
    if (status) {
      /* Connected -> Disconnected */
      int i;
      ctrlLocal.initialPollDone = 0;
      /* Keep bits that are specified via options,
         clear bits that are fetched from the controller */
      features_ &= ~reportedFeatureBits;
      setMCUErrMsg("MCU Disconnected");
      for (i=0; i<numAxes_; i++) {
        asynMotorAxis *pAxis=getAxis(i);
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

asynStatus ecmcMotorRecordController::poll(void)
{
  asynStatus status = asynSuccess;

  asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
            "%spoll ctrlLocal.initialPollDone=%d\n",
            modNamEMC, ctrlLocal.initialPollDone);
  if (!features_) {
    features_ = FEATURE_BITS_V2 | FEATURE_BITS_ECMC; //getFeatures();

    ctrlLocal.initialPollDone = 1;
  }
  return status;
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void ecmcMotorRecordController::report(FILE *fp, int level)
{
  fprintf(fp, "Twincat motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an ecmcMotorRecordAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
ecmcMotorRecordAxis* ecmcMotorRecordController::getAxis(asynUser *pasynUser)
{
  return static_cast<ecmcMotorRecordAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an ecmcMotorRecordAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
ecmcMotorRecordAxis* ecmcMotorRecordController::getAxis(int axisNo)
{
  return static_cast<ecmcMotorRecordAxis*>(asynMotorController::getAxis(axisNo));
}


/** Code for iocsh registration */
static const iocshArg ecmcMotorRecordCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg ecmcMotorRecordCreateControllerArg1 = {"EPICS ASYN TCP motor port name", iocshArgString};
static const iocshArg ecmcMotorRecordCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg ecmcMotorRecordCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg ecmcMotorRecordCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg ecmcMotorRecordCreateControllerArg5 = {"options", iocshArgString};
static const iocshArg *const ecmcMotorRecordCreateControllerArgs[] = {
                                                            &ecmcMotorRecordCreateControllerArg0,
                                                            &ecmcMotorRecordCreateControllerArg1,
                                                            &ecmcMotorRecordCreateControllerArg2,
                                                            &ecmcMotorRecordCreateControllerArg3,
                                                            &ecmcMotorRecordCreateControllerArg4,
                                                            &ecmcMotorRecordCreateControllerArg5};
static const iocshFuncDef ecmcMotorRecordCreateControllerDef = {strEcmcCreateControllerDef, 6,
                                                           ecmcMotorRecordCreateControllerArgs};
static void ecmcMotorRecordCreateContollerCallFunc(const iocshArgBuf *args)
{
  ecmcMotorRecordCreateController(args[0].sval, args[1].sval, args[2].ival,
                             args[3].ival, args[4].ival, args[5].sval);
}

/* ecmcMotorRecordCreateAxis */
static const iocshArg ecmcMotorRecordCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg ecmcMotorRecordCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg ecmcMotorRecordCreateAxisArg2 = {"axisFlags", iocshArgInt};
static const iocshArg ecmcMotorRecordCreateAxisArg3 = {"axisOptionsStr", iocshArgString};
static const iocshArg * const ecmcMotorRecordCreateAxisArgs[] = {&ecmcMotorRecordCreateAxisArg0,
                                                            &ecmcMotorRecordCreateAxisArg1,
                                                            &ecmcMotorRecordCreateAxisArg2,
                                                            &ecmcMotorRecordCreateAxisArg3};
static const iocshFuncDef ecmcMotorRecordCreateAxisDef = {strEcmcCreateAxisDef, 4,
                                                         ecmcMotorRecordCreateAxisArgs};

static void ecmcMotorRecordCreateAxisCallFunc(const iocshArgBuf *args)
{
  ecmcMotorRecordCreateAxis(args[0].sval, args[1].ival, args[2].ival, args[3].sval);
}

static void ecmcMotorRecordControllerRegister(void)
{
  iocshRegister(&ecmcMotorRecordCreateControllerDef, ecmcMotorRecordCreateContollerCallFunc);
  iocshRegister(&ecmcMotorRecordCreateAxisDef,   ecmcMotorRecordCreateAxisCallFunc);
}

extern "C" {
  epicsExportRegistrar(ecmcMotorRecordControllerRegister);
}
