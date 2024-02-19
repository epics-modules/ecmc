/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcFilter.cpp
*
*  Created on: Jan 12, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcFilter.h"

ecmcFilter::ecmcFilter(double sampleTime) {
  initVars();
  sampleTime_ = sampleTime;
  filterSize_ = FILTER_BUFFER_SIZE_DEF;
  bufferVel_  = new double[filterSize_];

  for (int i = 0; i < (int)filterSize_; i++) {
    bufferVel_[i] = 0;
  }
}

ecmcFilter::ecmcFilter(double sampleTime, size_t size) {
  initVars();
  sampleTime_ = sampleTime;
  filterSize_ = size;
  bufferVel_  = new double[filterSize_];

  for (int i = 0; i < (int)filterSize_; i++) {
    bufferVel_[i] = 0;
  }
}

ecmcFilter::~ecmcFilter() {}

void ecmcFilter::initVars() {
  errorReset();
  indexVel_ = 0;
}

double ecmcFilter::getFiltVelo(double distSinceLastScan) {
  double sum = 0;

  bufferVel_[indexVel_] = distSinceLastScan;
  indexVel_++;

  if (indexVel_ >= filterSize_) {
    indexVel_ = 0;
  }

  for (int i = 0; i < (int)filterSize_; i++) {
    sum = sum + bufferVel_[i];
  }

  lastOutput_ = sum / (static_cast<double>(filterSize_)) / sampleTime_;

  return lastOutput_;
}

double ecmcFilter::getFiltPos(double pos, double modRange) {
  double sum = 0;

  bufferVel_[indexVel_] = pos;
  indexVel_++;

  if (indexVel_ >= filterSize_) {
    indexVel_ = 0;
  }

  double modThreshold = FILTER_POS_MODULO_OVER_UNDER_FLOW_LIMIT * modRange;

  for (int i = 0; i < (int)filterSize_; i++) {
    // check if value over/under flow mod range compared to latest value
    if ((pos - bufferVel_[i]) > modThreshold) {
      sum = sum + bufferVel_[i] + modRange;
    } else if ((pos - bufferVel_[i]) < -modThreshold) {
      sum = sum + bufferVel_[i] - modRange;
    } else {
      sum = sum + bufferVel_[i];
    }
  }

  lastOutput_ = sum / (static_cast<double>(filterSize_));

  // Ensure result is within modrange
  if (modRange > 0) {
    if (lastOutput_ >= modRange) {
      lastOutput_ = lastOutput_ - modRange;
    } else if (lastOutput_ < 0) {
      lastOutput_ = lastOutput_ + modRange;
    }
  }

  return lastOutput_;
}

int ecmcFilter::reset() {
  initVars();
  return 0;
}

int ecmcFilter::initFilter(double pos) {
  for (int i = 0; i < (int)filterSize_; i++) {
    bufferVel_[i] = 0;
  }

  indexVel_ = 0;
  return 0;
}

void ecmcFilter::setSampleTime(double sampleTime) {
  sampleTime_ = sampleTime;
}

int ecmcFilter::setFilterSize(size_t size) {
  double *tempBuffer = NULL;

  try {
    tempBuffer = new double[size];
  } catch (std::bad_alloc& ex) {
    LOGERR(
      "%s/%s:%d: ERROR: Filter Mem Alloc Error. Old filter settings still valid (size=%zu)\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      filterSize_
      );
    return ERROR_AXIS_FILTER_ALLOC_FAIL;
  }
  delete bufferVel_;
  bufferVel_  = tempBuffer;
  filterSize_ = size;

  for (int i = 0; i < (int)filterSize_; i++) {
    bufferVel_[i] = 0;
  }
  indexVel_ = 0;
  return 0;
}
