/*************************************************************************\
* Copyright (c) 2024 Paul Scherrer Institut
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAxisGroup.cpp
*
*  Created on: Mar 28, 2024
*      Author: anderssandstrom
*
\*************************************************************************/
#include "ecmcAxisGroup.h"
#include "ecmcErrorsList.h"

ecmcAxisGroup::ecmcAxisGroup(int index, const char *name){
  name_ = name;
  axesCounter_ = 0;
  index_ = index;
  blocked_ = false;
  axisInGroup_.assign(ECMC_MAX_AXES, false);
  LOGINFO("%s/%s:%d: INFO: Axis group[%d] %s created.\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          index_,
          name_.c_str());
};

ecmcAxisGroup::~ecmcAxisGroup(){

};

const char* ecmcAxisGroup::getName(){
  return name_.c_str();
};

// Add axis to group
void ecmcAxisGroup::addAxis(ecmcAxisBase *axis){
  if(!axis) {
    throw std::runtime_error( "Axis NULL");
  }
  // Keep this container non-null so hot RT loops can avoid per-element null checks.
  axes_.push_back(axis);
  const int axisId = axis->getAxisID();
  axesIds_.push_back(axisId);
  if ((axisId >= 0) && (axisId < ECMC_MAX_AXES)) {
    axisInGroup_[axisId] = true;
  }
  axesCounter_++;
  LOGINFO("%s/%s:%d: INFO: Axis[%d]: Added to axis group[%d] %s.\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          axisId,
          index_,
          name_.c_str());
};

// Check if all axes in group are enable
bool ecmcAxisGroup::getEnable(){
  if (axes_.empty()) {
    return false;
  }
  for (auto *axis : axes_) {
    if (!axis->getEnable()) {
      return false;
    }
  }
  return true;
};

// Check if at least one axis in group are enable
bool ecmcAxisGroup::getAnyEnable(){ 
  for (auto *axis : axes_) {
    if (axis->getEnable()) {
      return true;
    }
  }
  return false;
};

// Check if all axes in group are enabled
bool ecmcAxisGroup::getEnabled(){
  if (axes_.empty()) {
    return false;
  }
  for (auto *axis : axes_) {
    if (!axis->getEnabled()) {
      return false;
    }
  }
  return true;
};

// Check if at least one axis in group are enabled
bool ecmcAxisGroup::getAnyEnabled(){
  for (auto *axis : axes_) {
    // Only check enabled flag, not enable cmd
    if (axis->getEnabledOnly()) {
      return true;
    }
  }
  return false;
};

// Check if all axes in group are busy
bool ecmcAxisGroup::getBusy(){
  if (axes_.empty()) {
    return false;
  }
  for (auto *axis : axes_) {
    if (!axis->getBusy()) {
      return false;
    }
  }
  return true;
};

// Check if at least one axis in group are busy
bool ecmcAxisGroup::getAnyBusy(){
  for (auto *axis : axes_) {
    if (axis->getBusy()) {
      return true;
    }
  }
  return false;
};

// Check if at least one axis in group are in error state
int ecmcAxisGroup::getAnyErrorId(){
  for (auto *axis : axes_) {
    const int errorId = axis->getErrorID();
    if (errorId) {
      return errorId;
    }
  }
  return 0;
};

// Set enable of all axes in group
int ecmcAxisGroup::setEnable(bool enable){
  for (auto *axis : axes_) {
    const int error = axis->setEnable(enable);
    if (error) {
      return error;
    }
  }
  return 0;
}

// set traj source of all axes in group
int ecmcAxisGroup::setTrajSrc(dataSource trajSource){
  for (auto *axis : axes_) {
    const int error = axis->getSeq()->setTrajDataSourceType(trajSource);
    if (error) {
      return error;
    }
  }
  return 0;
}

// set enc source of all axes in group
int ecmcAxisGroup::setEncSrc(dataSource encSource){
  for (auto *axis : axes_) {
    const int error = axis->setEncDataSourceType(encSource);
    if (error) {
      return error;
    }
  }
  return 0;
}

void ecmcAxisGroup::setErrorReset(){
  for (auto *axis : axes_) {
    axis->errorReset();
  }
}

// Set errors all axes
void ecmcAxisGroup::setError(int error){
  for (auto *axis : axes_) {
    axis->setErrorID(error);
  }
}

// Set slaved axis error all axes
void ecmcAxisGroup::setSlavedAxisInError(){
  for (auto *axis : axes_) {
    axis->setSlavedAxisInError();
  }
};

// Set slaved axis error all axes
void ecmcAxisGroup::halt(){
  for (auto *axis : axes_) {
    axis->stopMotion(0);
  }
};

// Check if axis is in group
bool ecmcAxisGroup::inGroup(int axisIndex){
  if ((axisIndex >= 0) && (axisIndex < ECMC_MAX_AXES)) {
    return axisInGroup_[axisIndex];
  }

  // Out-of-range lookup fallback keeps behavior for non-standard ids.
  for(const int& i : axesIds_) {
    if(i == axisIndex)  {
      return 1;
    }   
  }
  return 0;
}

// Axis count in group
size_t ecmcAxisGroup::size(){
  return axesCounter_;
}

ecmcAxisGroupStatusSummary ecmcAxisGroup::getStatusSummary(bool includeMonFields) {
  ecmcAxisGroupStatusSummary summary;
  const bool hasAxes = !axes_.empty();
  summary.allEnableCmd    = hasAxes;
  summary.anyEnableCmd    = false;
  summary.allEnabled      = hasAxes;
  summary.anyEnabled      = false;
  summary.allBusy         = hasAxes;
  summary.anyBusy         = false;
  summary.anyIlocked      = false;
  summary.allAtTarget     = hasAxes;
  summary.allWithinCtrlDb = hasAxes;
  summary.allTrajExternal = hasAxes;
  summary.anyTrajExternal = false;
  summary.firstErrorId    = 0;

  for (auto *axis : axes_) {
    const bool enableCmd = axis->getEnable();
    const bool enabledOnly = axis->getEnabledOnly();
    const bool enabled = enableCmd && enabledOnly;
    const bool busy = axis->getBusy();
    const bool ilocked = axis->getSumInterlock();
    const dataSource trajSource = axis->getTrajDataSourceType();
    const int errorId = axis->getErrorID();
    summary.allEnableCmd = summary.allEnableCmd && enableCmd;
    summary.anyEnableCmd = summary.anyEnableCmd || enableCmd;
    summary.allEnabled = summary.allEnabled && enabled;
    summary.anyEnabled = summary.anyEnabled || enabledOnly;
    summary.allBusy = summary.allBusy && busy;
    summary.anyBusy = summary.anyBusy || busy;
    summary.anyIlocked = summary.anyIlocked || ilocked;
    summary.allTrajExternal = summary.allTrajExternal &&
                              (trajSource != ECMC_DATA_SOURCE_INTERNAL);
    summary.anyTrajExternal = summary.anyTrajExternal ||
                              (trajSource == ECMC_DATA_SOURCE_EXTERNAL);
    if ((summary.firstErrorId == 0) && errorId) {
      summary.firstErrorId = errorId;
    }

    if (includeMonFields && (summary.allAtTarget || summary.allWithinCtrlDb)) {
      auto * const mon = axis->getMon();

      if (summary.allAtTarget) {
        const bool axisAtTarget = mon->getEnableAtTargetMon() ? mon->getAtTarget() : !busy;
        summary.allAtTarget = axisAtTarget;
      }
      if (summary.allWithinCtrlDb) {
        summary.allWithinCtrlDb = mon->getAxisIsWithinCtrlDB();
      }
    }
  }
  return summary;
}

// get all traj src in extern
bool ecmcAxisGroup::getTrajSrcExt(){
  if (axes_.empty()) {
    return false;
  }
  for (auto *axis : axes_) {
    if (axis->getTrajDataSourceType() == ECMC_DATA_SOURCE_INTERNAL) {
      return false;
    }
  }
  return true;
};

// get any traj src in extern
bool ecmcAxisGroup::getTrajSrcAnyExt(){
  for (auto *axis : axes_) {
    if (axis->getTrajDataSourceType() == ECMC_DATA_SOURCE_EXTERNAL) {
      return true;
    }
  }
  return false;
};

// set allow traj/enc source change when enabled
void ecmcAxisGroup::setAllowSrcChangeWhenEnabled(int allow){
  for (auto *axis : axes_) {
    axis->setAllowSourceChangeWhenEnabled(allow);
  }
}

void ecmcAxisGroup::setMRSync(bool sync) {
  for (auto *axis : axes_) {
    axis->setMRSync(sync);
  }
}

void ecmcAxisGroup::setMRStop(bool stop) {
  for (auto *axis : axes_) {
    axis->setMRStop(stop);
  }
}

void ecmcAxisGroup::setMRCnen(bool cnen) {
  for (auto *axis : axes_) {
    axis->setMRCnen(cnen);
  }
}

void ecmcAxisGroup::setMRIgnoreDisableStatusCheck(bool ignore) {
  for (auto *axis : axes_) {
    axis->setMRIgnoreDisableStatusCheck(ignore);
  }
}

void ecmcAxisGroup::setEnableAutoEnable(bool enable) {
  for (auto *axis : axes_) {
    axis->setEnableAutoEnable(enable);
  }
}

void ecmcAxisGroup::setEnableAutoDisable(bool enable) {
  for (auto *axis : axes_) {
    axis->setEnableAutoDisable(enable);
  }
}

// Check if any axes in group is at fwd limit switch
bool ecmcAxisGroup::getAnyAtLimitFwd() {
  for (auto *axis : axes_) {
    if (!axis->getLimitFwd()) {
      return true;
    }
  }
  return false;
}

// Check if any axes in group is at bwd limit switch
bool ecmcAxisGroup::getAnyAtLimitBwd() {
  for (auto *axis : axes_) {
    if (!axis->getLimitBwd()) {
      return true;
    }
  }
  return false;
}

// Check if any axes in group is at a limit switch
bool ecmcAxisGroup::getAnyAtLimit() {
  for (auto *axis : axes_) {
    if (!axis->getLimitFwd() || !axis->getLimitBwd()) {
      return true;
    }
  }
  return false;
}

void ecmcAxisGroup::setSlavedAxisIlocked() {
  for (auto *axis : axes_) {
    axis->setSlavedAxisInterlock();
  }
}

void ecmcAxisGroup::setAxisIsWithinCtrlDBExtTraj(bool within) {
  for (auto *axis : axes_) {
    axis->getMon()->setAxisIsWithinCtrlDBExtTraj(within);
  }
}

bool ecmcAxisGroup::getAxisIsWithinCtrlDB() {
  if (axes_.empty()) {
    return false;
  }
  for (auto *axis : axes_) {
    if (!axis->getMon()->getAxisIsWithinCtrlDB()) {
      return false;
    }
  }
  return true;
}

bool ecmcAxisGroup::getAnyIlocked() {
  for (auto *axis : axes_) {
    if (axis->getSumInterlock()) {
      return true;
    }
  }
  return false;
}

bool ecmcAxisGroup::getAtTarget() {
  if (axes_.empty()) {
    return false;
  }
  for (auto *axis : axes_) {
    auto * const mon = axis->getMon();
    if(mon->getEnableAtTargetMon()) {
      if (!mon->getAtTarget()) {
        return false;
      }
    } else {
      if (axis->getBusy()) {
        return false;
      }
    }
  }
  return true;
}

// set Group blocked
void ecmcAxisGroup::setBlocked(bool blocked) {
  if (blocked_ == blocked) {
    return;
  }
  blocked_ = blocked;
  for (auto *axis : axes_) {
    axis->setBlocked(blocked);
  }

}

// get Group blocked
bool ecmcAxisGroup::getBlocked() {
  return blocked_;
}

bool ecmcAxisGroup::getAxisAutoDisableEnabled() {
  if (axes_.empty()) {
    return false;
  }
  bool autoDisable = true;
  for (auto *axis : axes_) {
    autoDisable = autoDisable && axis->getEnableAutoDisable();
    if (!autoDisable) {
      return false;
    }
  }
  return autoDisable;
}
