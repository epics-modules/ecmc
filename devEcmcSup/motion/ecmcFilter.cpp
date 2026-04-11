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
#include "ecmcErrorsList.h"
#include "ecmcRtLogger.h"

ecmcFilter::ecmcFilter(double sampleTime) {
  initVars();
  sampleTime_ = sampleTime;
  filterSize_ = FILTER_BUFFER_SIZE_DEF;
  bufferVel_  = new double[filterSize_];

  for (int i = 0; i < (int)filterSize_; i++) {
    bufferVel_[i] = 0;
  }
  my_ = 0;
  veloSumValid_ = true;
}

ecmcFilter::ecmcFilter(double sampleTime, size_t size) {
  initVars();
  sampleTime_ = sampleTime;
  filterSize_ = size;
  bufferVel_  = new double[filterSize_];

  for (int i = 0; i < (int)filterSize_; i++) {
    bufferVel_[i] = 0;
  }
  my_ = 0;
  veloSumValid_ = true;
}

ecmcFilter::~ecmcFilter() {
  delete[] bufferVel_;
  bufferVel_ = NULL;
}

void ecmcFilter::initVars() {
  errorReset();
  indexVel_       = 0;
  my_             = 0;
  last_           = 0;
  lastOutput_     = 0;
  posSum_         = 0;
  posPrevWrapped_ = 0;
  posUnwrapped_   = 0;
  veloSumValid_   = false;
  posStateValid_  = false;
}

double ecmcFilter::getFiltVelo(double distSinceLastScan) {
  if (!veloSumValid_) {
    my_ = 0;
    for (int i = 0; i < (int)filterSize_; i++) {
      my_ += bufferVel_[i];
    }
    veloSumValid_ = true;
  }

  // Running sum for O(1) moving average update.
  my_ += distSinceLastScan - bufferVel_[indexVel_];
  bufferVel_[indexVel_] = distSinceLastScan;
  indexVel_++;

  if (indexVel_ >= filterSize_) {
    indexVel_ = 0;
  }

  posStateValid_ = false;
  lastOutput_ = my_ / (static_cast<double>(filterSize_)) / sampleTime_;

  return lastOutput_;
}

double ecmcFilter::getFiltPos(double pos, double modRange) {
  double modThreshold = FILTER_POS_MODULO_OVER_UNDER_FLOW_LIMIT * modRange;

  if (!posStateValid_) {
    // Build an unwrapped ring buffer once, preserving the original wrap-compensated
    // startup behavior relative to the latest sample.
    bufferVel_[indexVel_] = pos;
    posSum_               = 0;
    posPrevWrapped_       = pos;
    posUnwrapped_         = pos;

    for (int i = 0; i < (int)filterSize_; i++) {
      double sample = bufferVel_[i];

      if (modRange > 0) {
        if ((pos - sample) > modThreshold) {
          sample += modRange;
        } else if ((pos - sample) < -modThreshold) {
          sample -= modRange;
        }
      }
      bufferVel_[i] = sample;
      posSum_ += sample;
    }
    posStateValid_ = true;
  } else {
    double delta = pos - posPrevWrapped_;

    if (modRange > 0) {
      if (delta > modThreshold) {
        delta -= modRange;
      } else if (delta < -modThreshold) {
        delta += modRange;
      }
    }

    posPrevWrapped_ = pos;
    posUnwrapped_ += delta;
    posSum_ += posUnwrapped_ - bufferVel_[indexVel_];
    bufferVel_[indexVel_] = posUnwrapped_;
  }

  indexVel_++;

  if (indexVel_ >= filterSize_) {
    indexVel_ = 0;
  }

  veloSumValid_ = false;
  lastOutput_ = posSum_ / (static_cast<double>(filterSize_));

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
  return initFilter(0);
}

int ecmcFilter::initFilter(double pos) {
  for (int i = 0; i < (int)filterSize_; i++) {
    bufferVel_[i] = 0;
  }

  my_ = 0;
  posSum_ = 0;
  posPrevWrapped_ = 0;
  posUnwrapped_ = 0;
  veloSumValid_ = true;
  posStateValid_ = false;
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
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Filter Mem Alloc Error. Old filter settings still valid (size=%zu)\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      filterSize_
      );
    return ERROR_AXIS_FILTER_ALLOC_FAIL;
  }
  double *oldBuffer   = bufferVel_;
  size_t  oldSize     = filterSize_;
  size_t  oldIndex    = indexVel_;
  bool    posWasValid = posStateValid_;
  bool    velWasValid = veloSumValid_;

  // Preserve output continuity on grow by padding with the current average
  // (velocity and position use different internal sums/units).
  double padValue = 0;
  if (oldSize > 0) {
    if (posWasValid) {
      padValue = posSum_ / static_cast<double>(oldSize);
    } else if (velWasValid) {
      padValue = my_ / static_cast<double>(oldSize);
    } else if (oldBuffer) {
      size_t lastIndex = (oldIndex + oldSize - 1) % oldSize;
      padValue = oldBuffer[lastIndex];
    }
  }

  for (size_t i = 0; i < size; ++i) {
    tempBuffer[i] = padValue;
  }

  if (oldBuffer && oldSize > 0 && size > 0) {
    size_t keep = (oldSize < size) ? oldSize : size;
    // Keep the most recent samples in chronological order.
    size_t start = (oldIndex + oldSize - keep) % oldSize;
    size_t dst   = size - keep;
    for (size_t i = 0; i < keep; ++i) {
      tempBuffer[dst + i] = oldBuffer[(start + i) % oldSize];
    }
  }

  delete[] oldBuffer;
  bufferVel_  = tempBuffer;
  filterSize_ = size;
  indexVel_   = 0;  // oldest sample position after linearized repack

  my_ = 0;
  posSum_ = 0;
  for (size_t i = 0; i < filterSize_; ++i) {
    my_ += bufferVel_[i];
    posSum_ += bufferVel_[i];
  }

  veloSumValid_ = true;
  posStateValid_ = posWasValid;
  if (posStateValid_ && filterSize_ > 0) {
    posUnwrapped_ = bufferVel_[filterSize_ - 1];
    // Keep posPrevWrapped_ from last update to preserve unwrap continuity.
  } else if (!posStateValid_) {
    posUnwrapped_ = 0;
  }
  return 0;
}

size_t ecmcFilter::getFilterSize() {
  return filterSize_;
}
