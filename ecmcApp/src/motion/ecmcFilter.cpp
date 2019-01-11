/*
 * cMcuFilter.cpp
 *
 *  Created on: Jan 12, 2016
 *      Author: anderssandstrom
 */

#include "ecmcFilter.h"

ecmcFilter::ecmcFilter(double sampleTime) {
  LOGINFO15("%s/%s:%d: filter[x]=new;\n", __FILE__, __FUNCTION__, __LINE__);
  initVars();
  sampleTime_ = sampleTime;
}

ecmcFilter::~ecmcFilter() {}

void ecmcFilter::initVars() {
  errorReset();

  for (int i = 0; i < FILTER_BUFFER_SIZE_POS; i++) {
    bufferPos_[i] = 0;
  }

  for (int i = 0; i < FILTER_BUFFER_SIZE_VEL; i++) {
    bufferVel_[i] = 0;
  }
  indexPos_ = 0;
  indexVel_ = 0;
}

double ecmcFilter::lowPassAveraging(double input) {
  double sum = 0;

  bufferVel_[indexVel_] = input;
  indexVel_++;

  if (indexVel_ >= FILTER_BUFFER_SIZE_VEL) {
    indexVel_ = 0;
  }

  for (int i = 0; i < (FILTER_BUFFER_SIZE_VEL); i++) {
    sum = sum + bufferVel_[i];
  }
  return sum / (static_cast<double>(FILTER_BUFFER_SIZE_VEL));
}

// returns velocity
double ecmcFilter::positionBasedVelAveraging(double actPosition) {
  if (indexPos_ >= FILTER_BUFFER_SIZE_POS) {
    indexPos_ = 0;
  }
  bufferPos_[indexPos_] = actPosition;
  int nextIndex = indexPos_ + 1;

  if (nextIndex >= FILTER_BUFFER_SIZE_POS) {
    nextIndex = 0;
  }

  double vel = lowPassAveraging(
    (bufferPos_[indexPos_] - bufferPos_[nextIndex]) /
    (sampleTime_ * static_cast<double>(FILTER_BUFFER_SIZE_POS)));
  indexPos_ = nextIndex;
  return vel;
}

int ecmcFilter::reset() {
  initVars();
  return 0;
}

int ecmcFilter::initFilter(double pos) {
  for (int i = 0; i < FILTER_BUFFER_SIZE_POS; i++) {
    bufferPos_[i] = pos;
  }

  for (int i = 0; i < FILTER_BUFFER_SIZE_VEL; i++) {
    bufferVel_[i] = 0;
  }
  indexPos_ = 0;
  indexVel_ = 0;
  return 0;
}

void ecmcFilter::setSampleTime(double sampleTime) {
  sampleTime_ = sampleTime;
}
