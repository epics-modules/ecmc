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

ecmcAxisGroup::ecmcAxisGroup(int index, const char *name){
  name_ = name;
  axesCounter_ = 0;
  index_ = index;
  printf("ecmcAxisGroup: Created axis group[%d] %s.\n", index_, name_.c_str());
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
  axes_.push_back(axis);
  axesIds_.push_back(axis->getAxisID());
  axesCounter_++;
  printf("ecmcAxisGroup: Added axis %d to group[%d] %s.\n", axis->getAxisID(),index_,name_.c_str());
};

// Check if all axes in group are enable
bool ecmcAxisGroup::getEnable(){
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      if (!(*axis)->getEnable()) {
        return 0;
      }
    }
  }
  return 1;
};

// Check if at least one axis in group are enable
bool ecmcAxisGroup::getAnyEnable(){ 
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      if ((*axis)->getEnable()) {
        return 1;
      }
    }
  }
  return 0;
};

// Check if all axes in group are enabled
bool ecmcAxisGroup::getEnabled(){
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      if (!(*axis)->getEnabled()) {
        return 0;
      }
    }
  }
  return 1;
};

// Check if at least one axis in group are enabled
bool ecmcAxisGroup::getAnyEnabled(){
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      if ((*axis)->getEnabled()) {
        return 1;
      }
    }
  }
  return 0;
};

// Check if all axes in group are busy
bool ecmcAxisGroup::getBusy(){
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      if (!(*axis)->getBusy()) {
        return 0;
      }
    }
  }
  return 1;
};

// Check if at least one axis in group are busy
bool ecmcAxisGroup::getAnyBusy(){
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      if ((*axis)->getBusy()) {
        return 1;
      }
    }
  }
  return 0;
};

// Check if at least one axis in group are in error state
int ecmcAxisGroup::getAnyErrorId(){
  int errorId = 0;
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      errorId = (*axis)->getErrorID();
      if (errorId) {
        return errorId;
      }
    }
  }
  return 0;
};

// Set enable of all axes in group
int ecmcAxisGroup::setEnable(bool enable){
  int error = 0;
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      error = (*axis)->setEnable(enable);
      if(error) {
        return error;
      }
    }
  }
  return error;
}

// set traj source of all axes in group
int ecmcAxisGroup::setTrajSrc(dataSource trajSource){
  int error = 0;
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      error = (*axis)->setTrajDataSourceType(trajSource);
      if(error) {
        return error;
      }
    }
  }
  return error;
}

// set enc source of all axes in group
int ecmcAxisGroup::setEncSrc(dataSource encSource){
  int error = 0;
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      error = (*axis)->setEncDataSourceType(encSource);
      if(error) {
        return error;
      }
    }
  }
  return error;
}

void ecmcAxisGroup::setErrorReset(){
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      (*axis)->errorReset();
    }
  }
}

// Set errors all axes
void ecmcAxisGroup::setError(int error){
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      (*axis)->setErrorID(error);
    }
  }
}

// Set slaved axis error all axes
void ecmcAxisGroup::setSlavedAxisInError(){
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      (*axis)->setSlavedAxisInError();
    }
  }
};

// Set slaved axis error all axes
void ecmcAxisGroup::halt(){
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      (*axis)->stopMotion(0);
    }
  }
};

// Check if axis is in group
bool ecmcAxisGroup::inGroup(int axisIndex){
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

// get all traj src in extern
bool ecmcAxisGroup::getTrajSrcExt(){
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      if ( ((*axis)->getTrajDataSourceType()) == ECMC_DATA_SOURCE_INTERNAL) {
        return 0;
      }
    }
  }
  return 1;
};

// get any traj src in intern
bool ecmcAxisGroup::getTrajSrcAnyExt(){
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      if ( ((*axis)->getTrajDataSourceType()) == ECMC_DATA_SOURCE_EXTERNAL) {
        return 1;
      }
    }
  }
  return 0;
};

// set allow traj/enc source change when enabled
void ecmcAxisGroup::setAllowSrcChangeWhenEnabled(int allow){
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      (*axis)->setAllowSourceChangeWhenEnabled(allow);
    }
  }
}

void ecmcAxisGroup::setSyncNextPoll(bool sync) {
  for(std::vector<ecmcAxisBase*>::iterator axis = axes_.begin(); axis != axes_.end(); ++axis) {
    if((*axis)) {
      (*axis)->setSyncActSet(sync);
    }
  }
}
