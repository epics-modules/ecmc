/*
 * ecmcMasterSlave.cpp
 *
 *  Created on: Mar 17, 2016
 *      Author: anderssandstrom
 */

#include "ecmcMasterSlaveIF.h"

ecmcMasterSlaveIF::ecmcMasterSlaveIF(int           defaultAxisId,
                                     interfaceType ifType,
                                     double        sampleTime) {
  PRINT_ERROR_PATH("axis[%d].masterSlaveIF.error", defaultAxisId);
  initVars();
  sampleTime_    = sampleTime;
  defaultAxisId_ = defaultAxisId;
  interfaceType_ = ifType;
  LOGINFO15("%s/%s:%d: axis[%d].masterSlaveIF=new;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            defaultAxisId_);

  transform_ = new ecmcCommandTransform(3, ECMC_MAX_AXES);  // currently three commands

  if (!transform_) {
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR TRANSFORM OBJECT.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    exit(EXIT_FAILURE);
  }
  transform_->addCmdPrefix(TRANSFORM_EXPR_VARIABLE_TRAJ_PREFIX,
                           ECMC_TRANSFORM_VAR_TYPE_TRAJ);
  transform_->addCmdPrefix(TRANSFORM_EXPR_VARIABLE_ENC_PREFIX,
                           ECMC_TRANSFORM_VAR_TYPE_ENC);
  transform_->addCmdPrefix(TRANSFORM_EXPR_INTERLOCK_PREFIX,
                           ECMC_TRANSFORM_VAR_TYPE_IL);

  velocityFilter_ = new ecmcFilter(sampleTime);

  if (!velocityFilter_) {
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR VELOCITY-FILTER OBJECT.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    exit(EXIT_FAILURE);
  }
  velocityFilter_->setSampleTime(sampleTime_);
  printCurrentState();
}

ecmcMasterSlaveIF::~ecmcMasterSlaveIF() {
  delete transform_;
  delete velocityFilter_;
}

void ecmcMasterSlaveIF::printCurrentState() {
  switch (interfaceType_) {
  case ECMC_ENCODER_INTERFACE:
    LOGINFO15("%s/%s:%d: axis[%d].masterSlaveIF.interfaceType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              defaultAxisId_,
              "ECMC_ENCODER_INTERFACE");
    break;

  case ECMC_TRAJECTORY_INTERFACE:
    LOGINFO15("%s/%s:%d: axis[%d].masterSlaveIF.interfaceType=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              defaultAxisId_,
              "ECMC_TRAJECTORY_INTERFACE");
    break;

  default:
    LOGINFO15("%s/%s:%d: axis[%d].masterSlaveIF.interfaceType=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              defaultAxisId_,
              interfaceType_);
    break;
  }
}

void ecmcMasterSlaveIF::initVars() {
  for (int i = 0; i < MAX_TRANSFORM_INPUTS; i++) {
    inputDataInterface_[i] = NULL;
  }
  dataSource_             = ECMC_DATA_SOURCE_INTERNAL;
  numInputSources_        = 0;
  gearRatio_              = 1;
  defaultAxisId_          = 0;
  interfaceType_          = ECMC_ENCODER_INTERFACE;
  interlockDefiendinExpr_ = false;
  externalPosition_       = 0;
  externalPositionOld_    = 0;
  externalVelocity_       = 0;
  externalInterlock_      = ECMC_INTERLOCK_TRANSFORM;
  sampleTime_             = 0;
  velocityFilterEnable_   = false;
}

ecmcMasterSlaveData * ecmcMasterSlaveIF::getOutputDataInterface() {
  return &outputDataInterface_;
}

int ecmcMasterSlaveIF::addInputDataInterface(ecmcMasterSlaveData *masterData,
                                             int                  index) {
  if ((index >= MAX_TRANSFORM_INPUTS) || (index < 0)) {
    return ERROR_MASTER_DATA_IF_INDEX_OUT_OF_RANGE;
  }
  inputDataInterface_[index] = masterData;
  numInputSources_++;
  return 0;
}

ecmcMasterSlaveData * ecmcMasterSlaveIF::getExtInputDataInterface(int index) {
  if ((index >= ECMC_MAX_AXES) || (index < 0)) {
    return NULL;
  }
  return inputDataInterface_[index];
}

int ecmcMasterSlaveIF::setDataSourceType(dataSource refSource) {
  if (refSource != ECMC_DATA_SOURCE_INTERNAL) {
    int error = transform_->getErrorID();

    if (error) {
      return error;
    }
  }
  dataSource_ = refSource;
  return 0;
}

dataSource ecmcMasterSlaveIF::getDataSourceType() {
  return dataSource_;
}

int ecmcMasterSlaveIF::getNumExtInputSources() {
  return numInputSources_;
}

ecmcCommandTransform * ecmcMasterSlaveIF::getExtInputTransform() {
  return transform_;
}

int ecmcMasterSlaveIF::getExtInputPos(int axisId, int commandIndex,
                                      double *val) {
  *val = transform_->getData(commandIndex, axisId) * gearRatio_;
  return 0;
}

int ecmcMasterSlaveIF::getExtInputPos(int commandIndex, double *val) {
  *val = transform_->getData(commandIndex, defaultAxisId_) * gearRatio_;
  return 0;
}

bool ecmcMasterSlaveIF::getExtInputInterlock(int axisId, int commandIndex) {
  return static_cast<bool>(transform_->getData(commandIndex, axisId));
}

bool ecmcMasterSlaveIF::getExtInputInterlock(int commandIndex) {
  return static_cast<bool>(transform_->getData(commandIndex, defaultAxisId_));
}

int ecmcMasterSlaveIF::transformRefresh() {
  int error = 0;

  // Trajectory
  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    if (inputDataInterface_[i] != NULL) {
      error = transform_->setData(inputDataInterface_[i]->getPosition(),
                                  ECMC_TRANSFORM_VAR_TYPE_TRAJ,
                                  i);

      if (error) {
        return error;
      }
    } else {
      error = transform_->setData(0, ECMC_TRANSFORM_VAR_TYPE_TRAJ, i);

      if (error) {
        return error;
      }
    }
  }

  // Encoder
  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    if (inputDataInterface_[i + ECMC_MAX_AXES] != NULL) {
      error = transform_->setData(
        inputDataInterface_[i + ECMC_MAX_AXES]->getPosition(),
        ECMC_TRANSFORM_VAR_TYPE_ENC,
        i);

      if (error) {
        return error;
      }
    } else {
      error = transform_->setData(0, ECMC_TRANSFORM_VAR_TYPE_ENC, i);

      if (error) {
        return error;
      }
    }
  }


  // Interlocks (for both encoder and Traj so AND operation)
  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    if ((inputDataInterface_[i] != NULL) &&
        (inputDataInterface_[i + ECMC_MAX_AXES] != NULL)) {
      // Trajectory and Encoder
      error = transform_->setData(
        inputDataInterface_[i]->getInterlock() &&
        inputDataInterface_[i + ECMC_MAX_AXES]->getInterlock(),
        ECMC_TRANSFORM_VAR_TYPE_IL,
        i);

      if (error) {
        return error;
      }
    } else {
      error = transform_->setData(0, ECMC_TRANSFORM_VAR_TYPE_IL, i);

      if (error) {
        return error;
      }
    }
  }
  return transform_->refresh();
}

int ecmcMasterSlaveIF::validate() {
  return validate(dataSource_);
}

int ecmcMasterSlaveIF::validate(dataSource nextDataSource) {
  char axisIdStr[12];
  int ret = snprintf(axisIdStr, sizeof(axisIdStr), "%d", defaultAxisId_);
  if ((ret >= static_cast<int>(sizeof(axisIdStr))) || (ret <= 0)) {
    return ERROR_MASTER_DATA_IF_VALIDATE_CHAR_BUFFER_OVERFLOW;
  }
  std::string strToFind = "";
  bool found            = false;

  if (nextDataSource != ECMC_DATA_SOURCE_INTERNAL) {
    // Ensure that setpoint is defined in expression
    switch (interfaceType_) {
    case  ECMC_ENCODER_INTERFACE:
      strToFind = TRANSFORM_EXPR_VARIABLE_ENC_PREFIX;
      strToFind.append(axisIdStr);
      strToFind.append(":=");
      found = transform_->getExpression()->find(strToFind) !=
              std::string::npos;

      if (!found) {
        return ERROR_MASTER_DATA_IF_EXPRESSION_VAR_ENC_MISSING;
      }
      break;

    case ECMC_TRAJECTORY_INTERFACE:
      strToFind = TRANSFORM_EXPR_VARIABLE_TRAJ_PREFIX;
      strToFind.append(axisIdStr);
      strToFind.append(":=");
      found = transform_->getExpression()->find(strToFind) !=
              std::string::npos;

      if (!found) {
        return ERROR_MASTER_DATA_IF_EXPRESSION_VAR_TRAJ_MISSING;
      }
      break;
    }
    return transform_->validate();
  }

  // See if interlock is defined then transform needs to be executed
  strToFind = TRANSFORM_EXPR_INTERLOCK_PREFIX;
  strToFind.append(axisIdStr);
  interlockDefiendinExpr_ = transform_->getExpression()->find(strToFind) !=
                            std::string::npos;

  return 0;
}

int ecmcMasterSlaveIF::setGearRatio(double ratioNum, double ratioDenom) {
  if (ratioDenom == 0) {
    return ERROR_MASTER_DATA_IF_GEAR_RATIO_DENOM_ZERO;
  }

  gearRatio_ = ratioNum / ratioDenom;
  return 0;
}

int ecmcMasterSlaveIF::getGearRatio(double *ratio) {
  *ratio = gearRatio_;
  return 0;
}

bool ecmcMasterSlaveIF::getInterlockDefined() {
  return interlockDefiendinExpr_;
}

int ecmcMasterSlaveIF::refreshInputs() {
  int error = 0;

  if ((dataSource_ != ECMC_DATA_SOURCE_INTERNAL) || interlockDefiendinExpr_) {
    error = transformRefresh();

    if (error) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }

    // Setpoint or actual position depends on type of interface
    int commandIndex = 0;

    switch (interfaceType_) {
    case  ECMC_ENCODER_INTERFACE:
      commandIndex = ECMC_TRANSFORM_VAR_TYPE_ENC;
      break;

    case ECMC_TRAJECTORY_INTERFACE:
      commandIndex = ECMC_TRANSFORM_VAR_TYPE_TRAJ;
      break;
    }

    error = getExtInputPos(defaultAxisId_, commandIndex, &externalPosition_);

    if (error) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, error);
    }

    if (velocityFilterEnable_) {
      externalVelocity_ = velocityFilter_->positionBasedVelAveraging(
        externalPosition_);
    } else {
      externalVelocity_ = (externalPosition_ - externalPositionOld_) /
                          sampleTime_;
      externalPositionOld_ = externalPosition_;
    }
    
    // 1=OK, 0=STOP
    if (!getExtInputInterlock(defaultAxisId_, ECMC_TRANSFORM_VAR_TYPE_IL)) {
      externalInterlock_ = ECMC_INTERLOCK_TRANSFORM;
    } else {
      externalInterlock_ = ECMC_INTERLOCK_NONE;
    }
  } else {
    externalPosition_  = 0;
    externalVelocity_  = 0;
    externalInterlock_ = ECMC_INTERLOCK_NONE;
  }
  return 0;
}

double ecmcMasterSlaveIF::getInputPos() {
  return externalPosition_;
}

double ecmcMasterSlaveIF::getInputVel() {
  return externalVelocity_;
}

interlockTypes ecmcMasterSlaveIF::getInputIlock() {
  return externalInterlock_;
}

int ecmcMasterSlaveIF::setEnableVelFilter(bool enable) {
  if (enable && !velocityFilterEnable_) {
    velocityFilter_->reset();
  }

  velocityFilterEnable_ = enable;

  return 0;
}
