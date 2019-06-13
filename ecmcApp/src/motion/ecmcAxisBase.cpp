/*
 * cMcuAxisBase.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: anderssandstrom
 */

#include "ecmcAxisBase.h"
#include "../plc/ecmcPLCTask.h"
#include <inttypes.h>
#include <stdint.h>
#include <string>
#include <new>
#include <iostream>

ecmcAxisBase::ecmcAxisBase(ecmcAsynPortDriver *asynPortDriver,
                           int axisID, 
                           double sampleTime) {

  PRINT_ERROR_PATH("axis[%d].error", axisID);
  initVars();
  asynPortDriver_                 = asynPortDriver;
  data_.axisId_                   = axisID;
  data_.sampleTime_               = sampleTime;
  data_.command_.operationModeCmd = ECMC_MODE_OP_AUTO;

  try {
    enc_  = new ecmcEncoder(&data_, data_.sampleTime_);
    traj_ = new ecmcTrajectoryTrapetz(&data_,
                                    data_.sampleTime_);
    mon_ = new ecmcMonitor(&data_);

    extTrajVeloFilter_ = new ecmcFilter(data_.sampleTime_);    
    extEncVeloFilter_ = new ecmcFilter(data_.sampleTime_);    

  } catch(std::bad_alloc& ex) {
      LOGERR(
        "%s/%s:%d: ERROR (axis %d): Mem alloc error.\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        data_.axisId_);

        exit(1);
  }

  seq_.setAxisDataRef(&data_);
  seq_.setTraj(traj_);
  seq_.setMon(mon_);
  seq_.setEnc(enc_);
/*  int error = seq_.setExtTrajIF(externalInputTrajectoryIF_);

  if (error) {
    setErrorID(__FILE__,
               __FUNCTION__,
               __LINE__,
               ERROR_AXIS_ASSIGN_EXT_INTERFACE_TO_SEQ_FAILED);
  }*/

  initAsyn();
}

ecmcAxisBase::~ecmcAxisBase() {
  delete enc_;
  enc_ = NULL;
  delete traj_;
  traj_ = NULL;
  delete mon_;
  mon_ = NULL;
  delete extTrajVeloFilter_;
  extTrajVeloFilter_ = NULL;
  delete extEncVeloFilter_;
  extEncVeloFilter_ = NULL;
}

void ecmcAxisBase::initVars() {
  // errorReset();  //THIS IS NONO..
  data_.axisType_              = ECMC_AXIS_TYPE_BASE;
  data_.command_.reset         = false;
  allowCmdFromOtherPLC_        = false;
  plcEnable_                   = false;
  data_.status_.inStartupPhase = false;

  /*for (int i = 0; i < ECMC_MAX_AXES; i++) {
    axes_[i] = NULL;
  }*/

  data_.status_.inRealtime = false;

  data_.status_.externalTrajectoryPosition  = 0;
  data_.status_.externalTrajectoryVelocity  = 0;
  //data_.status_.externalTrajectoryInterlock = ECMC_INTERLOCK_EXTERNAL;

  data_.status_.externalEncoderPosition  = 0;
  data_.status_.externalEncoderVelocity  = 0;
  //data_.status_.externalEncoderInterlock = ECMC_INTERLOCK_EXTERNAL;

  data_.status_.currentPositionActual   = 0;
  data_.status_.currentPositionSetpoint = 0;
  data_.status_.currentVelocityActual   = 0;
  data_.status_.currentVelocitySetpoint = 0;

  data_.sampleTime_ = 1 / 1000;
  memset(&statusData_,    0, sizeof(statusData_));
  memset(&statusDataOld_, 0, sizeof(statusDataOld_));
  printHeaderCounter_      = 0;
  data_.status_.enabledOld = false;
  data_.status_.enableOld  = false;
  data_.status_.executeOld = false;
  cycleCounter_            = 0;
  axisState_               = ECMC_AXIS_STATE_STARTUP;
  oldPositionAct_          = 0;
  oldPositionSet_          = 0;
  asynPortDriver_          = NULL;
  for (int i = 0; i < ECMC_ASYN_AX_PAR_COUNT; i++) {
    axAsynParams_[i] = NULL;
  }
  statusOutputEntry_          = 0;
  blockExtCom_                = 0;
  statusWord_                 = 0;
  memset(diagBuffer_,0,AX_MAX_DIAG_STRING_CHAR_LENGTH);
  plc_ = NULL;
  extTrajVeloFilter_ = NULL;
  extEncVeloFilter_ =NULL;
  enableExtTrajVeloFilter_ = false;
  enableExtEncVeloFilter_ = false;
}

void ecmcAxisBase::printAxisState() {
  switch (axisState_) {
  case ECMC_AXIS_STATE_STARTUP:
    LOGINFO15("%s/%s:%d: axis[%d].state=ECMC_AXIS_STATE_STARTUP;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_);
    break;

  case ECMC_AXIS_STATE_DISABLED:
    LOGINFO15("%s/%s:%d: axis[%d].state=ECMC_AXIS_STATE_DISABLED;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_);
    break;

  case ECMC_AXIS_STATE_ENABLED:
    LOGINFO15("%s/%s:%d: axis[%d].state=ECMC_AXIS_STATE_ENABLED;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_);
    break;

  default:
    LOGINFO15("%s/%s:%d: axis[%d].state=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              axisState_);
    break;
  }
}

void ecmcAxisBase::printCurrentState() {
  // called by derived classes
  printAxisState();
  LOGINFO15("%s/%s:%d: axis[%d].reset=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            data_.command_.reset > 0);
  LOGINFO15("%s/%s:%d: axis[%d].allowCmdFromOtherPLC=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            allowCmdFromOtherPLC_ > 0);
  LOGINFO15("%s/%s:%d: axis[%d].plcEnable=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            plcEnable_ > 0);
  LOGINFO15("%s/%s:%d: axis[%d].inStartupPhase=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            data_.status_.inStartupPhase > 0);
  LOGINFO15("%s/%s:%d: axis[%d].inRealtime=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            data_.status_.inRealtime > 0);
  LOGINFO15("%s/%s:%d: axis[%d].enable=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            data_.command_.enable > 0);
  LOGINFO15("%s/%s:%d: axis[%d].enabled=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            data_.status_.enabled > 0);
  LOGINFO15("%s/%s:%d: axis[%d].moving=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            data_.status_.moving > 0);
}

void ecmcAxisBase::preExecute(bool masterOK) {

  data_.interlocks_.etherCatMasterInterlock = !masterOK;
  data_.refreshInterlocks();

  statusData_.onChangeData.trajSource = data_.command_.trajSource;    
  statusData_.onChangeData.encSource = data_.command_.encSource;
  data_.status_.moving = std::abs(
    data_.status_.currentVelocityActual) > 0;

  if (data_.command_.encSource == ECMC_DATA_SOURCE_INTERNAL) {
    enc_->readEntries();
  }

  // Axis state machine
  switch (axisState_) {
  case ECMC_AXIS_STATE_STARTUP:
    setEnable(false);
    data_.status_.busy       = false;
    data_.status_.distToStop = 0;

    if (masterOK) {
      // Auto reset hardware error if starting up
      if ((getErrorID() == ERROR_AXIS_HARDWARE_STATUS_NOT_OK) &&
          data_.status_.inStartupPhase) {
        errorReset();
        setInStartupPhase(false);
        enc_->setToZeroIfRelative();
      }
      LOGINFO15("%s/%s:%d: axis[%d].state=ECMC_AXIS_STATE_DISABLED;\n",
                __FILE__,
                __FUNCTION__,
                __LINE__,
                data_.axisId_);
      axisState_ = ECMC_AXIS_STATE_DISABLED;
    }
    break;

  case ECMC_AXIS_STATE_DISABLED:
    data_.status_.busy       = false;
    data_.status_.distToStop = 0;

    if (data_.status_.enabled) {
      LOGINFO15("%s/%s:%d: axis[%d].state=ECMC_AXIS_STATE_ENABLED;\n",
                __FILE__,
                __FUNCTION__,
                __LINE__,
                data_.axisId_);
      axisState_ = ECMC_AXIS_STATE_ENABLED;
    }

    if (!masterOK) {
      LOGERR(
        "%s/%s:%d: ERROR (axis %d): State change (ECMC_AXIS_STATE_DISABLED->ECMC_AXIS_STATE_STARTUP).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        data_.axisId_);
      axisState_ = ECMC_AXIS_STATE_STARTUP;
    }
    break;

  case ECMC_AXIS_STATE_ENABLED:
    data_.status_.distToStop = traj_->distToStop(
      data_.status_.currentVelocitySetpoint);

    if (data_.command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
      data_.status_.currentTargetPosition = traj_->getTargetPos();
    } else {  // Synchronized to other axis
      data_.status_.currentTargetPosition =
        data_.status_.currentPositionSetpoint;
    }

    if (!data_.status_.enabled) {
      LOGINFO15("%s/%s:%d: axis[%d].state=ECMC_AXIS_STATE_DISABLED;\n",
                __FILE__,
                __FUNCTION__,
                __LINE__,
                data_.axisId_);
      axisState_ = ECMC_AXIS_STATE_DISABLED;
    }

    if (!masterOK) {
      LOGERR(
        "%s/%s:%d: ERROR (axis %d): State change (ECMC_AXIS_STATE_ENABLED->ECMC_AXIS_STATE_STARTUP).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        data_.axisId_);
      LOGINFO15("%s/%s:%d: axis[%d].state=ECMC_AXIS_STATE_STARTUP;\n",
                __FILE__,
                __FUNCTION__,
                __LINE__,
                data_.axisId_);
      axisState_ = ECMC_AXIS_STATE_STARTUP;
    }

    break;
  }
  mon_->readEntries();

  //Exceute sync PLC always (if plcEnable_)
  if(plc_ && plcEnable_ && 
     /*(data_.command_.trajSource != ECMC_DATA_SOURCE_INTERNAL ||
      data_.command_.encSource  != ECMC_DATA_SOURCE_INTERNAL) &&*/
      axisState_ != ECMC_AXIS_STATE_STARTUP) {
    
    data_.status_.externalTrajectoryPositionOld = 
            data_.status_.externalTrajectoryPosition;
    data_.status_.externalEncoderPositionOld = 
            data_.status_.externalEncoderPosition;

    /* plc writes directlly to data_.status_.externalEncoderPosition
     and data_.status_.externalTrajectoryPosition*/
    plc_->execute(masterOK);
    
    // Filter velocities from PLC source
    // Traj
    if(enableExtTrajVeloFilter_ && extTrajVeloFilter_) {
      data_.status_.externalTrajectoryVelocity = extTrajVeloFilter_->getFiltVelo(
        data_.status_.externalTrajectoryPosition -
        data_.status_.externalTrajectoryPositionOld);
    }
    else {
      data_.status_.externalTrajectoryVelocity = data_.status_.externalTrajectoryPosition -
        data_.status_.externalTrajectoryPositionOld / data_.sampleTime_;
    }
    // Enc
    if(enableExtEncVeloFilter_ && extEncVeloFilter_) {
      data_.status_.externalEncoderVelocity = extEncVeloFilter_->getFiltVelo(
        data_.status_.externalEncoderPosition -
        data_.status_.externalEncoderPositionOld);
    } else {
      data_.status_.externalTrajectoryVelocity = data_.status_.externalEncoderPosition -
        data_.status_.externalEncoderPositionOld / data_.sampleTime_;
    }
  }
  //refreshExternalInputSources();
}

void ecmcAxisBase::postExecute(bool masterOK) {  
  // Write encoder entries
  enc_->writeEntries();

  if (data_.status_.busyOld != data_.status_.busy) {
    LOGINFO15("%s/%s:%d: axis[%d].busy=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              data_.status_.busy > 0);
  }
  data_.status_.busyOld = data_.status_.busy;

  if (data_.status_.enabledOld != data_.status_.enabled) {
    LOGINFO15("%s/%s:%d: axis[%d].enabled=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              data_.status_.enabled > 0);
  }
  data_.status_.enabledOld = data_.status_.enabled;

  data_.status_.executeOld                 = getExecute();
  data_.status_.currentPositionSetpointOld =
    data_.status_.currentPositionSetpoint;
  data_.status_.cntrlOutputOld = data_.status_.cntrlOutput;

  cycleCounter_++;
  refreshDebugInfoStruct();

  statusWord_ = 0;
  // bit 0 enabled
  statusWord_ = statusWord_ + (getEnabled()>0);
  // bit 1 execute  
  statusWord_ = statusWord_ + ((seq_.getExecute()>0) << 1);
  // bit 2 busy
  statusWord_ = statusWord_ + ((data_.status_.busy>0) << 2);
  // bit 3 at target
  statusWord_ = statusWord_ + ((data_.status_.atTarget>0) << 3);
  // bit 4 moving
  statusWord_ = statusWord_ + ((data_.status_.moving>0) << 4);
  // bit 5 limit fwd
  statusWord_ = statusWord_ + ((data_.status_.limitFwd>0) << 5);
  // bit 6 limit bwd
  statusWord_ = statusWord_ + ((data_.status_.limitBwd>0) << 6);
  // bit 7 homeswitch
  statusWord_ = statusWord_ + ((data_.status_.homeSwitch>0) << 7);
  // bit 8 inStartupPhase
  statusWord_ = statusWord_ + ((data_.status_.inStartupPhase>0) << 8);
  // bit 9 inRealtime
  statusWord_ = statusWord_ + ((data_.status_.inRealtime>0) << 9);
  // bit 16..23 seq state
  statusWord_ = statusWord_ + (((uint8_t)data_.status_.seqState) << 16);
  // bit 24..31 lastActiveInterlock type
  statusWord_ = statusWord_ + (((uint8_t)data_.interlocks_.lastActiveInterlock) << 24);

  // Update asyn parameters  
  axAsynParams_[ECMC_ASYN_AX_ACT_POS_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_SET_POS_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_POS_ERR_ID]->refreshParamRT(0);
  axAsynParams_[ECMC_ASYN_AX_STATUS_ID]->refreshParamRT(0);
  
  if(axAsynParams_[ECMC_ASYN_AX_DIAG_ID]->willRefreshNext() && axAsynParams_[ECMC_ASYN_AX_DIAG_ID]->initialized() ) {    
    int  bytesUsed = 0;
    int  error = getAxisDebugInfoData(&diagBuffer_[0],
                                      AX_MAX_DIAG_STRING_CHAR_LENGTH,
                                      &bytesUsed);   

    if (error) {
      LOGERR(
        "%s/%s:%d: ERROR (axis %d): Fail to update asyn par axis<id>.diag. Buffer to small.\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        data_.axisId_);
    } else {
      axAsynParams_[ECMC_ASYN_AX_DIAG_ID]->refreshParamRT(1,(uint8_t*)diagBuffer_,bytesUsed);
    }
  }
  else {
    //Just to count up for correct freq
    axAsynParams_[ECMC_ASYN_AX_DIAG_ID]->refreshParamRT(0);  
  }

  // Update status entry if linked
  if (statusOutputEntry_) {
    statusOutputEntry_->writeValue(getErrorID() == 0);
  }
}

axisType ecmcAxisBase::getAxisType() {
  return data_.axisType_;
}

int ecmcAxisBase::getAxisID() {
  return data_.axisId_;
}

void ecmcAxisBase::setReset(bool reset) {
  if (data_.command_.reset != reset) {
    LOGINFO15("%s/%s:%d: axis[%d].reset=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              reset);
  }

  data_.command_.reset = reset;

  if (reset) {
    errorReset();

    if (getMon() != NULL) {
      getMon()->errorReset();
    }

    if (getEnc() != NULL) {
      getEnc()->errorReset();
    }

    if (getTraj() != NULL) {
      getTraj()->errorReset();
    }

    if (getSeq() != NULL) {
      getSeq()->errorReset();
    }

    if (getDrv() != NULL) {
      getDrv()->errorReset();
    }

    if (getCntrl() != NULL) {
      getCntrl()->errorReset();
    }
  }
}

bool ecmcAxisBase::getReset() {
  return data_.command_.reset;
}

int ecmcAxisBase::setAllowCmdFromPLC(bool enable) {
  if (allowCmdFromOtherPLC_ != enable) {
    LOGINFO15("%s/%s:%d: axis[%d].enableCascadedCommands=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              enable);
  }
  allowCmdFromOtherPLC_ = enable;
  return 0;
}

bool ecmcAxisBase::getAllowCmdFromPLC() {
  return allowCmdFromOtherPLC_;
}

/*int ecmcAxisBase::setAxisArrayPointer(ecmcAxisBase *axis, int index) {
  if ((index >= ECMC_MAX_AXES) || (index < 0)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_INDEX_OUT_OF_RANGE);
  }
  axes_[index] = axis;
  
  //Set axis ref to plc
  return plc_->setAxisArrayPointer(axis,index);
}*/

//bool ecmcAxisBase::checkAxesForEnabledTransfromCommands(commandType type) {
//  for (int i = 0; i < ECMC_MAX_AXES; i++) {
//    if (axes_[i] != NULL) {
//      if (axes_[i]->getCascadedCommandsEnabled()) {
//        return true;
//      }
//    }
//  }
//  return false;
//}

int ecmcAxisBase::setEnablePLC(bool enable) {
  if (data_.status_.inRealtime) {
    if (enable) {
      int error = plc_->validate();

      if (error) {
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
      }
    }
  }

  if (plcEnable_ != enable) {
    LOGINFO15("%s/%s:%d: axis[%d].enableCommandsTransform=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              enable > 0);
  }
  plcEnable_ = enable;

  return 0;
}

bool ecmcAxisBase::getEnablePLC() {
  return plcEnable_;
}

/*int ecmcAxisBase::fillCommandsTransformData() {
  int error = 0;

  // Execute
  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    if (axes_[i] != NULL) {
      error = commandTransform_->setData(axes_[i]->getExecute(),
                                         ECMC_CMD_TYPE_EXECUTE,
                                         i);

      if (error) {
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
      }
    } else {
      error = commandTransform_->setData(0, ECMC_CMD_TYPE_EXECUTE, i);

      if (error) {
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
      }
    }
  }

  // Enable
  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    if (axes_[i] != NULL) {
      error = commandTransform_->setData(axes_[i]->getEnable(),
                                         ECMC_CMD_TYPE_ENABLE,
                                         i);

      if (error) {
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
      }
    } else {
      error = commandTransform_->setData(0, ECMC_CMD_TYPE_ENABLE, i);

      if (error) {
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
      }
    }
  }
  return 0;
}*/

/*int ecmcAxisBase::setCommandsTransformExpression(std::string expression) {
  LOGINFO15("%s/%s:%d: axis[%d].commandTransformExpression=%s;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            expression.c_str());
  return commandTransform_->setExpression(expression);
}*/

/*int ecmcAxisBase::setEnable_Transform() {
  // Atleast one axis have enabled getting execute from transform
  if (checkAxesForEnabledTransfromCommands(ECMC_CMD_TYPE_ENABLE) &&
      enableCommandTransform_) {
    if (!commandTransform_->getCompiled()) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_AXIS_TRANSFORM_ERROR_OR_NOT_COMPILED);
    }
    int error = fillCommandsTransformData();

    if (error) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }

    // Execute transform
    commandTransform_->refresh();

    // write changes to axes
    for (int i = 0; i < ECMC_MAX_AXES; i++) {
      if (commandTransform_ == NULL) {
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_AXIS_INVERSE_TRANSFORM_NULL);
      }

      if (axes_[i] != NULL) {
        if (axes_[i]->getCascadedCommandsEnabled() &&
            commandTransform_->getDataChanged(ECMC_CMD_TYPE_ENABLE,
                                              i) && (i != data_.axisId_)) {
          int error =
            axes_[i]->setEnable(commandTransform_->getData(ECMC_CMD_TYPE_ENABLE,
                                                           i));

          if (error) {
            return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
          }
        }
      }
    }
  }

  return 0;
}*/

/*int ecmcAxisBase::setExecute_Transform() {
  // Atleast one axis have enabled getting execute from transform
  if (checkAxesForEnabledTransfromCommands(ECMC_CMD_TYPE_EXECUTE) &&
      enableCommandTransform_) {
    if (!commandTransform_->getCompiled()) {
      LOGINFO7(
        "%s/%s:%d: Error: Command transform not compiled for axis %d.\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        data_.axisId_);
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_AXIS_TRANSFORM_ERROR_OR_NOT_COMPILED);
    }

    int error = fillCommandsTransformData();

    if (error) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }

    // Execute transform
    commandTransform_->refresh();

    // write changes to axes
    for (int i = 0; i < ECMC_MAX_AXES; i++) {
      if (axes_[i] != NULL) {
        if (axes_[i]->getCascadedCommandsEnabled() &&
            commandTransform_->getDataChanged(ECMC_CMD_TYPE_EXECUTE,
                                              i) && (i != data_.axisId_)) {
          int error =
            axes_[i]->setExecute(commandTransform_->getData(
                                   ECMC_CMD_TYPE_EXECUTE,
                                   i));
          if (error) {
            return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
          }
        }
      }
    }
  }
  return 0;
}*/

/*ecmcCommandTransform * ecmcAxisBase::getCommandTransform() {
  return commandTransform_;
}*/

void ecmcAxisBase::setInStartupPhase(bool startup) {
  if (data_.status_.inStartupPhase != startup) {
    LOGINFO15("%s/%s:%d: axis[%d].inStartupPhase=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              startup);
  }
  data_.status_.inStartupPhase = startup;
}

int ecmcAxisBase::setDriveType(ecmcDriveTypes driveType) {
  return setErrorID(__FILE__,
                    __FUNCTION__,
                    __LINE__,
                    ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

/*int ecmcAxisBase::setTrajTransformExpression(std::string expressionString) {
  ecmcCommandTransform *transform =
    externalInputTrajectoryIF_->getExtInputTransform();

  if (!transform) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
  }

  LOGINFO15("%s/%s:%d: axis[%d].trajTransformExpression=%s;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            expressionString.c_str());

  int error = transform->setExpression(expressionString);

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  error = externalInputTrajectoryIF_->validate();

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  return 0;
}*/

/*int ecmcAxisBase::setEncTransformExpression(std::string expressionString) {
  ecmcCommandTransform *transform =
    externalInputEncoderIF_->getExtInputTransform();

  if (!transform) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
  }

  LOGINFO15("%s/%s:%d: axis[%d].encTransformExpression=%s;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_.axisId_,
            expressionString.c_str());

  int error = transform->setExpression(expressionString);

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  error = externalInputEncoderIF_->validate();

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  return 0;
}*/

int ecmcAxisBase::setTrajDataSourceType(dataSource refSource) {
  if (getEnable() && (refSource != ECMC_DATA_SOURCE_INTERNAL)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED);
  }

  // If realtime: Ensure that transform object is compiled and ready to go
  if ((refSource != ECMC_DATA_SOURCE_INTERNAL) && data_.status_.inRealtime && !plc_->getCompiled()) {
    return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_TRAJ_TRANSFORM_NULL);
  }
   
  // Check if object is ok to go to refSource
  int error = plc_->validate();
  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  if (refSource != ECMC_DATA_SOURCE_INTERNAL) {
    data_.interlocks_.noExecuteInterlock = false;
    data_.refreshInterlocks();
    data_.status_.busy = true;
  }

  if (data_.command_.trajSource != refSource) {
    LOGINFO15("%s/%s:%d: axis[%d].trajDataSourceType=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              refSource);
  }
  data_.command_.trajSource = refSource;
  return 0;
}

int ecmcAxisBase::setEncDataSourceType(dataSource refSource) {
  int error = 0;
  if (getEnable()) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED);
  }

  // If realtime: Ensure that ethercat enty for actual position is linked
  if ((refSource == ECMC_DATA_SOURCE_INTERNAL) && data_.status_.inRealtime) {
    
    error = getEnc()->validateEntry(
      ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION);

    if (error) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }
  }

  // If realtime: Ensure that transform object is compiled and ready to go
  if ((refSource != ECMC_DATA_SOURCE_INTERNAL) && data_.status_.inRealtime && !plc_->getCompiled()) {    
    
    error = plc_->compile();

    if (error) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }

    error = plc_->validate();

    if (error) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }
  }
  
  if (data_.command_.encSource != refSource) {
    LOGINFO15("%s/%s:%d: axis[%d].encDataSourceType=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              refSource);
  }

  data_.command_.encSource = refSource;
  return 0;
}

int ecmcAxisBase::setRealTimeStarted(bool realtime) {
  if (data_.status_.inRealtime != realtime) {
    LOGINFO15("%s/%s:%d: axis[%d].inRealtime=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              realtime);
  }
  data_.status_.inRealtime = realtime;
  return 0;
}

bool ecmcAxisBase::getError() {
  int error = ecmcAxisBase::getErrorID();

  if (error) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }
  return ecmcError::getError();
}

int ecmcAxisBase::getErrorID() {
  // GeneralsetErrorID
  if (ecmcError::getError()) {
    return ecmcError::getErrorID();
  }

  // Monitor
  ecmcMonitor *mon = getMon();

  if (mon) {
    if (mon->getError()) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, mon->getErrorID());
    }
  }

  // Encoder
  ecmcEncoder *enc = getEnc();

  if (enc) {
    if (enc->getError()) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, enc->getErrorID());
    }
  }

  // Drive
  ecmcDriveBase *drv = getDrv();

  if (drv) {
    if (drv->getError()) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, drv->getErrorID());
    }
  }

  // Trajectory
  ecmcTrajectoryTrapetz *traj = getTraj();

  if (traj) {
    if (traj->getError()) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, traj->getErrorID());
    }
  }

  // Controller
  ecmcPIDController *cntrl = getCntrl();

  if (cntrl) {
    if (cntrl->getError()) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, cntrl->getErrorID());
    }
  }

  // Sequencer
  ecmcAxisSequencer *seq = getSeq();

  if (seq) {
    if (seq->getErrorID()) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, seq->getErrorID());
    }
  }

  return ecmcError::getErrorID();
}

int ecmcAxisBase::setEnableLocal(bool enable) {
  if (enable && !data_.command_.enable) {
    traj_->setStartPos(data_.status_.currentPositionActual);
    traj_->setCurrentPosSet(data_.status_.currentPositionActual);
    traj_->setTargetPos(data_.status_.currentPositionActual);
    data_.status_.currentTargetPosition = data_.status_.currentPositionActual;
  }
  traj_->setEnable(enable);

  if (data_.command_.enable != enable) {
    LOGINFO15("%s/%s:%d: axis[%d].enable=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              enable);
  }

  data_.status_.enableOld = data_.command_.enable;
  data_.command_.enable   = enable;
  return 0;
}

void ecmcAxisBase::errorReset() {
  // Monitor
  ecmcMonitor *mon = getMon();

  if (mon) {
    mon->errorReset();
  }

  // Encoder
  ecmcEncoder *enc = getEnc();

  if (enc) {
    enc->errorReset();
  }

  // Drive
  ecmcDriveBase *drv = getDrv();

  if (drv) {
    drv->errorReset();
  }

  // Trajectory
  ecmcTrajectoryTrapetz *traj = getTraj();

  if (traj) {
    traj->errorReset();
  }

  // Controller
  ecmcPIDController *cntrl = getCntrl();

  if (cntrl) {
    cntrl->errorReset();
  }

  // Sequencer
  ecmcAxisSequencer *seq = getSeq();

  if (seq) {
    seq->errorReset();
  }

  ecmcError::errorReset();
}

/*int ecmcAxisBase::refreshExternalInputSources() {
  
  // Trajectory
  int error = externalInputTrajectoryIF_->refreshInputs();

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }
  data_.status_.externalTrajectoryPosition =
    externalInputTrajectoryIF_->getInputPos();
  data_.status_.externalTrajectoryVelocity =
    externalInputTrajectoryIF_->getInputVel();
  data_.interlocks_.trajTransformInterlock =
    externalInputTrajectoryIF_->getInputIlock();
  data_.refreshInterlocks();

  // Encoder
  error = externalInputEncoderIF_->refreshInputs();

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }
  data_.status_.externalEncoderPosition =
    externalInputEncoderIF_->getInputPos();
  data_.status_.externalEncoderVelocity =
    externalInputEncoderIF_->getInputVel();
  data_.interlocks_.encTransformInterlock =
    externalInputEncoderIF_->getInputIlock();
  data_.refreshInterlocks();
  return 0;
}*/

/*int ecmcAxisBase::refreshExternalOutputSources() {
  externalInputTrajectoryIF_->getOutputDataInterface()->setPosition(
    data_.status_.currentPositionSetpoint);
  externalInputTrajectoryIF_->getOutputDataInterface()->setVelocity(
    data_.status_.currentVelocitySetpoint);

  externalInputEncoderIF_->getOutputDataInterface()->setPosition(
    data_.status_.currentPositionActual);
  externalInputEncoderIF_->getOutputDataInterface()->setVelocity(
    data_.status_.currentVelocityActual);

  if (getMon()) {
    externalInputEncoderIF_->getOutputDataInterface()->setInterlock(
      data_.interlocks_.trajSummaryInterlockFWD ||
      data_.interlocks_.trajSummaryInterlockBWD);
    externalInputTrajectoryIF_->getOutputDataInterface()->setInterlock(
      data_.interlocks_.trajSummaryInterlockFWD ||
      data_.interlocks_.trajSummaryInterlockBWD);
  }
  return 0;
}*/

/*ecmcMasterSlaveIF * ecmcAxisBase::getExternalTrajIF() {
  return externalInputTrajectoryIF_;
}*/

/*ecmcMasterSlaveIF * ecmcAxisBase::getExternalEncIF() {
  return externalInputEncoderIF_;
}*/

int ecmcAxisBase::validateBase() {
  /*int error = externalInputEncoderIF_->validate();

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  error = externalInputTrajectoryIF_->validate();

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }*/

  if(plc_ && plcEnable_) {
    return plc_->validate();
  }
  
  return 0;
}

int ecmcAxisBase::getPosAct(double *pos) {
  *pos = data_.status_.currentPositionActual;
  return 0;
}

int ecmcAxisBase::getVelAct(double *vel) {
  *vel = data_.status_.currentVelocityActual;
  return 0;
}

int ecmcAxisBase::getPosSet(double *pos) {
  if ((data_.command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) && getSeq()) {
    *pos = data_.command_.positionTarget;
  } else {
    *pos = data_.status_.currentPositionSetpoint;
  }

  return 0;
}

ecmcEncoder * ecmcAxisBase::getEnc() {
  return enc_;
}

ecmcTrajectoryTrapetz * ecmcAxisBase::getTraj() {
  return traj_;
}

ecmcMonitor * ecmcAxisBase::getMon() {
  return mon_;
}

ecmcAxisSequencer * ecmcAxisBase::getSeq() {
  return &seq_;
}

int ecmcAxisBase::getAxisHomed(bool *homed) {
  *homed = enc_->getHomed();
  return 0;
}

int ecmcAxisBase::setAxisHomed(bool homed) {
  enc_->setHomed(homed);
  return 0;
}

int ecmcAxisBase::getEncScaleNum(double *scale) {
  *scale = enc_->getScaleNum();
  return 0;
}

int ecmcAxisBase::setEncScaleNum(double scale) {
  enc_->setScaleNum(scale);
  return 0;
}

int ecmcAxisBase::getEncScaleDenom(double *scale) {
  *scale = enc_->getScaleDenom();
  return 0;
}

int ecmcAxisBase::setEncScaleDenom(double scale) {
  enc_->setScaleDenom(scale);
  return 0;
}

int ecmcAxisBase::getEncPosRaw(int64_t *rawPos) {
  *rawPos = enc_->getRawPosMultiTurn();
  return 0;
}

int ecmcAxisBase::setCommand(motionCommandTypes command) {
  seq_.setCommand(command);

  return 0;
}

int ecmcAxisBase::setCmdData(int cmdData) {
  seq_.setCmdData(cmdData);
  return 0;
}

motionCommandTypes ecmcAxisBase::getCommand() {
  return seq_.getCommand();
}

int ecmcAxisBase::getCmdData() {
  return seq_.getCmdData();
}

int ecmcAxisBase::slowExecute() {
  if (oldPositionAct_ != data_.status_.currentPositionActual) {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.actPos=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              data_.status_.currentPositionActual);
  }
  oldPositionAct_ = data_.status_.currentPositionActual;

  if (oldPositionSet_ != data_.status_.currentPositionSetpoint) {
    LOGINFO15("%s/%s:%d: axis[%d].trajectory.currentPositionSetpoint=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              data_.status_.currentPositionSetpoint);
  }
  oldPositionSet_ = data_.status_.currentPositionSetpoint;

  if (data_.status_.movingOld != data_.status_.moving) {
    LOGINFO15("%s/%s:%d: axis[%d].moving=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_.axisId_,
              data_.status_.moving > 0);
  }
  data_.status_.movingOld = data_.status_.moving;

  return 0;
}

void ecmcAxisBase::printAxisStatus() {
  if (memcmp(&statusDataOld_.onChangeData, &statusData_.onChangeData,
             sizeof(statusData_.onChangeData)) == 0) {
    return;  // Printout on change
  }

  statusDataOld_ = statusData_;

  // Only print header once per 25 status lines
  if (printHeaderCounter_ <= 0) {
    LOGINFO(
      "ecmc::  Ax     PosSet     PosAct     PosErr    PosTarg   DistLeft    CntrOut   VelFFSet     VelAct   VelFFRaw VelRaw  Error Co CD St IL LI TS ES En Ex Bu Ta Hd L- L+ Ho\n");
    printHeaderCounter_ = 25;
  }
  printHeaderCounter_--;

  LOGINFO(
    "ecmc:: %3d %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %10.3lf %6i %6x %2d %2d %2d %2d %2d %2d %2d %1d%1d %2d %2d %2d %2d %2d %2d %2d\n",
    statusData_.axisID,
    statusData_.onChangeData.positionSetpoint,
    statusData_.onChangeData.positionActual,
    statusData_.onChangeData.cntrlError,
    statusData_.onChangeData.positionTarget,
    statusData_.onChangeData.positionError,
    statusData_.onChangeData.cntrlOutput,
    statusData_.onChangeData.velocitySetpoint,
    statusData_.onChangeData.velocityActual,
    statusData_.onChangeData.velocityFFRaw,
    statusData_.onChangeData.velocitySetpointRaw,
    statusData_.onChangeData.error,
    statusData_.onChangeData.command,
    statusData_.onChangeData.cmdData,
    statusData_.onChangeData.seqState,
    statusData_.onChangeData.trajInterlock,
    statusData_.onChangeData.lastActiveInterlock,
    statusData_.onChangeData.trajSource,
    statusData_.onChangeData.encSource,
    statusData_.onChangeData.enable,
    statusData_.onChangeData.enabled,
    statusData_.onChangeData.execute,
    statusData_.onChangeData.busy,
    statusData_.onChangeData.atTarget,
    statusData_.onChangeData.homed,
    statusData_.onChangeData.limitBwd,
    statusData_.onChangeData.limitFwd,
    statusData_.onChangeData.homeSwitch);
}

int ecmcAxisBase::setExecute(bool execute) {
  // Internal trajectory source
  if (data_.command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
    // Allow direct homing without enable
    if (execute && !getEnable() &&
        !((data_.command_.cmdData == 15) && (data_.command_.command == 10))) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_AXIS_NOT_ENABLED);
    }

    if (execute && !data_.status_.executeOld && data_.status_.busy) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_AXIS_BUSY);
    }

    int error = seq_.setExecute(execute);

    if (error) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }
  }

  return 0; //setExecute_Transform();
}

bool ecmcAxisBase::getExecute() {
  if (data_.command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
    return seq_.getExecute();
  } else {
    return true;
  }
}

bool ecmcAxisBase::getBusy() {
  return data_.status_.busy;
}

int ecmcAxisBase::getDebugInfoData(ecmcAxisStatusType *data) {
  if (data == NULL) {
    return ERROR_AXIS_DATA_POINTER_NULL;
  }

  memcpy(data, &statusData_, sizeof(*data));
  return 0;
}

ecmcAxisStatusType * ecmcAxisBase::getDebugInfoDataPointer() {
  return &statusData_;
}

int ecmcAxisBase::getCycleCounter() {
  /// Use for watchdog purpose (will overflow)
  return cycleCounter_;
}

bool ecmcAxisBase::getEnable() {
  return data_.command_.enable;
}

bool ecmcAxisBase::getEnabled() {
  return data_.status_.enabled && data_.command_.enable;
}

int ecmcAxisBase::initAsyn() {
  if (asynPortDriver_ == NULL) {
    LOGERR("%s/%s:%d: ERROR (axis %d): AsynPortDriver object NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_.axisId_,
           ERROR_AXIS_ASYN_PORT_OBJ_NULL);
    return ERROR_AXIS_ASYN_PORT_OBJ_NULL;
  }

  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char *name = NULL;
  unsigned int charCount = 0;
  ecmcAsynDataItem *paramTemp = NULL;
  // Act pos
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_AX_STR "%d." ECMC_ASYN_AX_ACT_POS_NAME,
                       getAxisID());
  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamFloat64,
                                         (uint8_t *)&(statusData_.onChangeData.positionActual),
                                         sizeof(statusData_.onChangeData.positionActual),
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_ACT_POS_ID] = paramTemp;

  // Set pos
  charCount = snprintf(buffer,
                      sizeof(buffer),
                      ECMC_AX_STR "%d." ECMC_ASYN_AX_SET_POS_NAME,
                      getAxisID());
  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamFloat64,
                                         (uint8_t *)&(statusData_.onChangeData.positionSetpoint),
                                         sizeof(statusData_.onChangeData.positionSetpoint),
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_SET_POS_ID] = paramTemp;

  // Pos error (following error)
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_AX_STR "%d." ECMC_ASYN_AX_POS_ERR_NAME,
                       getAxisID());
  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamFloat64,
                                         (uint8_t *)&(statusData_.onChangeData.cntrlError),
                                         sizeof(statusData_.onChangeData.cntrlError),
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_POS_ERR_ID] = paramTemp;

  // Diagnostic string (array)
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_AX_STR "%d." ECMC_ASYN_AX_DIAG_NAME,
                       getAxisID());
  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }
  name = buffer;  
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamInt8Array,
                                         (uint8_t *)diagBuffer_,
                                         AX_MAX_DIAG_STRING_CHAR_LENGTH,
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }

  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_DIAG_ID] = paramTemp;

  // Status word
  charCount = snprintf(buffer,
                       sizeof(buffer),
                       ECMC_AX_STR "%d." ECMC_ASYN_AX_STATUS_NAME,
                       getAxisID());
  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate alias. Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                         asynParamUInt32Digital,
                                         (uint8_t *)&(statusWord_),
                                         sizeof(statusWord_),
                                         0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_.axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->addSupportedAsynType(asynParamInt32);
  paramTemp->addSupportedAsynType(asynParamUInt32Digital);    
  paramTemp->allowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  axAsynParams_[ECMC_ASYN_AX_STATUS_ID] = paramTemp;

  asynPortDriver_->callParamCallbacks();
  return 0;
}

int ecmcAxisBase::getAxisDebugInfoData(char *buffer,
                                       int   bufferByteSize,
                                       int  *bytesUsed) {
  ecmcAxisStatusType data;
  int error = getDebugInfoData(&data);

  if (error) {
    return error;
  }

  // (Ax,PosSet,PosAct,PosErr,PosTarg,DistLeft,CntrOut,VelFFSet,VelAct,VelFFRaw,VelRaw,CycleCounter,Error,Co,CD,St,IL,TS,ES,En,Ena,Ex,Bu,Ta,L-,L+,Ho");
  int ret = snprintf(buffer,
                     bufferByteSize,
                     "%d,%lf,%lf,%lf,%lf,%lf,%" PRId64 ",%lf,%lf,%lf,%lf,%d,%d,%x,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                     data.axisID,
                     data.onChangeData.positionSetpoint,
                     data.onChangeData.positionActual,
                     data.onChangeData.cntrlError,
                     data.onChangeData.positionTarget,
                     data.onChangeData.positionError,
                     data.onChangeData.positionRaw,
                     data.onChangeData.cntrlOutput,
                     data.onChangeData.velocitySetpoint,
                     data.onChangeData.velocityActual,
                     data.onChangeData.velocityFFRaw,
                     data.onChangeData.velocitySetpointRaw,
                     data.cycleCounter,
                     data.onChangeData.error,
                     data.onChangeData.command,
                     data.onChangeData.cmdData,
                     data.onChangeData.seqState,
                     data.onChangeData.trajInterlock,
                     data.onChangeData.lastActiveInterlock,
                     data.onChangeData.trajSource,
                     data.onChangeData.encSource,
                     data.onChangeData.enable,
                     data.onChangeData.enabled,
                     data.onChangeData.execute,
                     data.onChangeData.busy,
                     data.onChangeData.atTarget,
                     data.onChangeData.homed,
                     data.onChangeData.limitBwd,
                     data.onChangeData.limitFwd,
                     data.onChangeData.homeSwitch);

  if ((ret >= bufferByteSize) || (ret <= 0)) {
    *bytesUsed = 0;
    return ERROR_AXIS_PRINT_TO_BUFFER_FAIL;
  }
  *bytesUsed = ret;
  return 0;
}

int ecmcAxisBase::setEcStatusOutputEntry(ecmcEcEntry *entry) {
  statusOutputEntry_ = entry;
  return 0;
}

motionDirection ecmcAxisBase::getAxisSetDirection() {
  if (!data_.status_.enabled) {
    return ECMC_DIR_STANDSTILL;
  }

  // Transform or internal trajectory
  if (data_.command_.trajSource == ECMC_DATA_SOURCE_INTERNAL) {
    return getTraj()->getCurrSetDir();
  } else {
    if (data_.status_.currentPositionSetpoint <
        data_.status_.currentPositionSetpointOld) {
      return ECMC_DIR_BACKWARD;
    } else if (data_.status_.currentPositionSetpoint >
               data_.status_.currentPositionSetpointOld) {
      return ECMC_DIR_FORWARD;
    } else {
      return ECMC_DIR_STANDSTILL;
    }
  }
  return ECMC_DIR_STANDSTILL;
}

int ecmcAxisBase::getBlockExtCom() {
  return blockExtCom_;
}

int ecmcAxisBase::setBlockExtCom(int block) {
  blockExtCom_ = block;
  return 0;
}

int ecmcAxisBase::setModRange(double mod) {
  //Must be same mod factor in traj and enc
  if(mod<0) {
    LOGERR(
       "ERROR (axis %d): Modulo factor out of range. Must be a positive value (0x%x).\n",
        data_.axisId_,ERROR_AXIS_MODULO_OUT_OF_RANGE);

    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_MODULO_OUT_OF_RANGE);

  }
  data_.command_.moduloRange = mod;
  return 0;
}

double ecmcAxisBase::getModRange() {
  return data_.command_.moduloRange;
}

int ecmcAxisBase::setModType(int type) {
  
  if(type < 0 || type >= ECMC_MOD_MOTION_MAX) {
    LOGERR(
       "ERROR (axis %d): Modulo type out of range (0x%x).\n",
        data_.axisId_,ERROR_AXIS_MODULO_TYPE_OUT_OF_RANGE);

    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_AXIS_MODULO_TYPE_OUT_OF_RANGE);
  }
  
  data_.command_.moduloType = (ecmcMotionModType) type;
  return 0;
}

int ecmcAxisBase::getModType() {
  return (int)data_.command_.moduloType;
}

double ecmcAxisBase::getPosErrorMod() {

  double normalCaseError = data_.status_.currentPositionSetpoint-data_.status_.currentPositionActual;
  if(data_.command_.moduloRange==0) {
    return normalCaseError;
  }
  
  // Modulo
  if(std::abs(normalCaseError)<data_.command_.moduloRange*ECMC_OVER_UNDER_FLOW_FACTOR) {
    // No overflows 
    return normalCaseError;
  } else {
    //Overflow has happended in either encoder or setpoint
    double overUnderFlowError = data_.command_.moduloRange-std::abs(normalCaseError);
    double  setDiff = data_.status_.currentPositionSetpoint - data_.status_.currentPositionSetpointOld;
    //Moving forward (overflow)
    if(setDiff>0 || setDiff < -data_.command_.moduloRange*ECMC_OVER_UNDER_FLOW_FACTOR) {
      //Actual lagging setpoint  ACT SET
      if(data_.status_.currentPositionActual>data_.status_.currentPositionSetpoint) {         
        return overUnderFlowError;
      } 
      else { //Actual before setpoint SET ACT
        return -overUnderFlowError;
      }
    }
    else {
      //Moving backward (underflow)    
      //Actual lagging setpoint SET ACT
      if(data_.status_.currentPositionActual<data_.status_.currentPositionSetpoint) {             
        return -overUnderFlowError;
      } 
      else { //Actual before setpoint ACT SET
        return overUnderFlowError;
      }       
    }         
  }
  LOGERR(
        "%s/%s:%d: WARNING (axis %d): Modulo controller error calc failed...\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        data_.axisId_);

  return 0;
}

int ecmcAxisBase::getCntrlError(double *error) {
  *error = data_.status_.cntrlError;
  return 0;
}

void ecmcAxisBase::refreshDebugInfoStruct() {
  statusData_.onChangeData.atTarget       = data_.status_.atTarget;
  statusData_.axisID                      = data_.axisId_;
  statusData_.cycleCounter                = cycleCounter_;
  statusData_.onChangeData.busy           = data_.status_.busy;
  statusData_.onChangeData.cntrlError     = data_.status_.cntrlError;
  statusData_.onChangeData.cntrlOutput    = data_.status_.cntrlOutput;
  statusData_.onChangeData.enable         = data_.command_.enable;
  statusData_.onChangeData.enabled        = getEnabled();
  statusData_.onChangeData.error          = getErrorID();
  statusData_.onChangeData.execute        = getExecute();
  statusData_.onChangeData.homeSwitch     = data_.status_.homeSwitch;
  statusData_.onChangeData.limitBwd       = data_.status_.limitBwd;
  statusData_.onChangeData.limitFwd       = data_.status_.limitFwd;  
  statusData_.onChangeData.positionActual =
    data_.status_.currentPositionActual;
  statusData_.onChangeData.positionError = getPosErrorMod();    
  statusData_.onChangeData.positionSetpoint =
    data_.status_.currentPositionSetpoint;
  statusData_.onChangeData.positionTarget =
    data_.status_.currentTargetPosition;
  statusData_.onChangeData.seqState      = seq_.getSeqState();
  statusData_.onChangeData.trajInterlock =
    data_.interlocks_.interlockStatus;
  statusData_.onChangeData.lastActiveInterlock =
    data_.interlocks_.lastActiveInterlock;
  statusData_.onChangeData.sumIlockBwd = 
    data_.interlocks_.trajSummaryInterlockBWD;
  statusData_.onChangeData.sumIlockFwd = 
    data_.interlocks_.trajSummaryInterlockFWD;
  statusData_.onChangeData.velocityActual =
    data_.status_.currentVelocityActual;
  statusData_.onChangeData.velocitySetpoint =
    data_.status_.currentVelocitySetpoint;
  statusData_.onChangeData.velocitySetpointRaw =
    data_.status_.currentVelocitySetpointRaw;
  statusData_.onChangeData.velocityFFRaw =
    data_.status_.currentvelocityFFRaw;
  statusData_.onChangeData.cmdData     = data_.command_.cmdData;
  statusData_.onChangeData.command     = data_.command_.command;
  statusData_.onChangeData.positionRaw = enc_->getRawPosRegister();
  statusData_.onChangeData.homed       = enc_->getHomed();
  statusData_.acceleration             = traj_->getAcc();
  statusData_.deceleration             = traj_->getDec();
  statusData_.reset                    = data_.command_.reset;
  statusData_.moving                   = data_.status_.moving;
  statusData_.stall                    =
    data_.interlocks_.lagTrajInterlock
    || data_.interlocks_.
    lagDriveInterlock
    || data_.interlocks_.
    velocityDiffTrajInterlock
    || data_.interlocks_.
    velocityDiffDriveInterlock;
}

int ecmcAxisBase::setEnable(bool enable) {
  if (!enable) {  // Remove execute if enable is going down
    setExecute(false);
  }

  if (enable && validate()) {
    setExecute(false);
    return getErrorID();
  }

  int error = setEnableLocal(enable);

  if (error) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
  }

  // Cascade commands via command transformation
  return 0; //setEnable_Transform();
}

int ecmcAxisBase::setPLC(ecmcPLCTask* plc) {
  plc_ = plc;
  return 0;
}

int ecmcAxisBase::setExtSetPos(double pos) {
  data_.status_.externalTrajectoryPosition = pos;
  return 0;
}

int ecmcAxisBase::setExtActPos(double pos) {
  data_.status_.externalEncoderPosition = pos;
  return 0;
}

int ecmcAxisBase::setEnableExtTrajVeloFilter(bool enable) {
  if (enable && !enableExtTrajVeloFilter_) {
    extTrajVeloFilter_->reset();
  }

  enableExtTrajVeloFilter_ = enable;

  return 0;
}

int ecmcAxisBase::setEnableExtEncVeloFilter(bool enable) {
  if (enable && !enableExtEncVeloFilter_) {
    extEncVeloFilter_->reset();
  }

  enableExtEncVeloFilter_ = enable;

  return 0;
}

dataSource ecmcAxisBase::getTrajDataSourceType() {
  return data_.command_.trajSource;
}

dataSource ecmcAxisBase::getEncDataSourceType() {
  return data_.command_.encSource;
}

bool ecmcAxisBase::getEnableExtTrajVeloFilter() {
  return enableExtTrajVeloFilter_;
}
  
bool ecmcAxisBase::getEnableExtEncVeloFilter() {
  return enableExtEncVeloFilter_;
}

