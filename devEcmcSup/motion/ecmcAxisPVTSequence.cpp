/*************************************************************************\
* Copyright (c) 2024 Paul Scherrer Institut
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAxisPVTSequence.cpp
*
*  Created on: April 25, 2024
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcAxisPVTSequence.h"

ecmcAxisPVTSequence::ecmcAxisPVTSequence(double sampleTimeNs,size_t maxPoints) {
  segmentCount_ = 0;
  pointCount_   = 0;
  totalTimeNs_    = 0;
  currTimeNs_   = 0;
  sampleTimeNs_ = round(sampleTimeNs);
  busy_         = false;
  currSegIndex_ = 0;
  currSegIndexOld_ = 0;
  positionOffset_ = 0.0;
  data_           = NULL;
  firstSegTimeNs_ = 0;
  segments_.reserve(maxPoints + 3);
  points_.reserve(maxPoints + 3);
  resultPosActArray_.reserve(maxPoints);  
  resultPosErrArray_.reserve(maxPoints);
}

void   ecmcAxisPVTSequence::setSampleTime(double sampleTimeNs) {
  sampleTimeNs_   = round(sampleTimeNs) ;
}

void ecmcAxisPVTSequence::addSegment(ecmcPvtPoint *start, ecmcPvtPoint *end ) {
  printf("ecmcAxisPVTSequence::addSegment()\n");
  segments_.push_back(new ecmcPvtSegment(start, end));
  segmentCount_++;
}

void ecmcAxisPVTSequence::addPoint(ecmcPvtPoint *pnt) {
  try {
    points_.push_back(pnt);
    pointCount_++;
    if(pointCount_ > 1) {
      addSegment(points_[pointCount_-2], points_[pointCount_-1]);
    };
  } catch (std::bad_alloc& ex) {
    printf("ecmcAxisPVTSequence::addPoint() : Exception\n");
  }
}

double ecmcAxisPVTSequence::startTime(){
  if(segmentCount_ <= 0) {
      return -1;
  }
  return segments_[0]->getStartPoint()->timeNs_;
}

double ecmcAxisPVTSequence::endTime(){
  if(segmentCount_ <= 0) {
    return -1;
  }
  return segments_[segmentCount_-1]->getEndPoint()->timeNs_;
}

// Call before starting a seq
void ecmcAxisPVTSequence::initSeq() {
  busy_ = true;
  currTimeNs_   = round(startTime());
  currSegIndex_ = 0;
  currSegIndexOld_ = 0;
  resultPosActArray_.clear();
  resultPosErrArray_.clear();
  if(segmentCount_ > 0) {
    firstSegTimeNs_ = segments_[0]->getEndPoint()->timeNs_;
  }
  printf("ecmcAxisPVTSequence::initSeq()\n");
}

bool ecmcAxisPVTSequence::validate() {
  return /*built_ && */ segmentCount_ > 0;
}

ecmcPvtSegment* ecmcAxisPVTSequence::getSeqmentAtTime(double timeNs) {
  if(segmentCount_ <= 0) {
    return NULL;
  };
  if(timeNs < startTime() || timeNs > endTime() ) {
    return NULL;
  };
  for(size_t i=0; i < segmentCount_; i++) {
    if(timeNs >= segments_[i]->getStartPoint()->timeNs_  && 
       timeNs < segments_[i]->getEndPoint()->timeNs_ ) {
      return segments_[i];
    }
  }

  // Check if last seq endtime (equals)
  if( timeNs == segments_[segmentCount_-1]->getEndPoint()->timeNs_) {
    return segments_[segmentCount_-1];
  }
  
  return NULL;
}

bool ecmcAxisPVTSequence::isLastSample() {
  return isLastSample(currTimeNs_);
}

bool ecmcAxisPVTSequence::isLastSample(double timeNs) {
  return timeNs >= endTime();
}

bool ecmcAxisPVTSequence::isTimeValid(double timeNs) {
  if(segmentCount_ <= 0) {
    return false;
  }

  if(timeNs < startTime() || timeNs > endTime() ) {
    return false;
  }
  return true;
}

// Go to next sample in time
// return true as long not exceeding endtime
bool ecmcAxisPVTSequence::nextSampleStep(){
  // If  traj rampdown..
  //Do not increase time if not executeing
  if(!execute_) {
    return currTimeNs_ < endTime();
  }

  // Increase time
  currTimeNs_ = round(currTimeNs_ + sampleTimeNs_);

  // Switch segment?
  //if(currTime_ >= segments_[currSegIndex_]->getEndPoint()->time_) {
  if(currTimeNs_ > segments_[currSegIndex_]->getEndPoint()->timeNs_) {
    if(currSegIndex_ < segmentCount_-1) {
      // the time must be in the next segment
      currSegIndex_++;
    } else {  // last segment and last sample, set to curr time to end-time
      busy_ = false;
      currTimeNs_ = endTime();
    }
  }
  return currTimeNs_ < endTime();
}

// For RT sequential access
double ecmcAxisPVTSequence::getCurrPosition(){
  // Check if values should be latched to the result* vectors
  if(currSegIndexOld_ != currSegIndex_ ) {
    currSegIndexOld_ = currSegIndex_;
    // skip first and last segment
    if(currSegIndex_ < segmentCount_) {
      printf("pvt[%lu]: time %lf, posact %lf , posset %lf, poserr %lf\n",currSegIndex_ - 1,
              (currTimeNs_ - firstSegTimeNs_) / 1E9,
              data_->status_.currentPositionActual - positionOffset_,
              data_->status_.currentPositionSetpoint - positionOffset_,
              data_->status_.cntrlError);
      resultPosActArray_.push_back(data_->status_.currentPositionActual - positionOffset_);
      resultPosErrArray_.push_back(data_->status_.cntrlError);
    }
  }

  printf("xpvt[%lu]: time %lf, posact %lf , posset %lf, poserr %lf\n",currSegIndex_ - 1,
              currTimeNs_,
              data_->status_.currentPositionActual - positionOffset_,
              data_->status_.currentPositionSetpoint - positionOffset_,
              data_->status_.cntrlError);
  
  return segments_[currSegIndex_]->position(currTimeNs_) + positionOffset_;
}

// For RT sequential access
double ecmcAxisPVTSequence::getCurrVelocity(){
  return segments_[currSegIndex_]->velocity(currTimeNs_);
}

// For RT sequential access
double ecmcAxisPVTSequence::getCurrAcceleration(){
  return segments_[currSegIndex_]->acceleration(currTimeNs_);
}

double ecmcAxisPVTSequence::getCurrTime(){
  return currTimeNs_;
}

// For non RT access
double ecmcAxisPVTSequence::position(double timeNs, int *valid) {
  ecmcPvtSegment* temp = getSeqmentAtTime(timeNs);
  if(!temp) {
      *valid = 0;
      return 0;
  }
  *valid = 1;
  return temp->position(timeNs);
}

// For non RT access
double ecmcAxisPVTSequence::velocity(double timeNs, int *valid) {
  ecmcPvtSegment* temp = getSeqmentAtTime(timeNs);
  if(!temp) {
      *valid = 0;
      return 0;
  }
  *valid = 1;
  return temp->velocity(timeNs);
}

// For non RT access
double ecmcAxisPVTSequence::acceleration(double timeNs, int *valid) {
  ecmcPvtSegment* temp = getSeqmentAtTime(timeNs);
  if(!temp) {
      *valid = 0;
      return 0;
  }
  *valid = 1;
  return temp->acceleration(timeNs);
}

void   ecmcAxisPVTSequence::print() {
  if(segmentCount_<=0) {
    printf("PVT object empty\n");
    return;
  }

  printf("time [ns], pos[egu], vel[egu]\n");
  for(size_t i=0; i < pointCount_; i++) {
    points_[i]->print();
  }
}

void   ecmcAxisPVTSequence::printRT() {
  if(segmentCount_<=0) {
    printf("PVT object empty\n");
    return;
  }
  printf("RT traj:\n");
  printf("time [s], pos[egu], vel[egu/s], acc [egu/s/s]\n");

  do {
    printf("%lf, %lf, %lf, %lf\n",getCurrTime(),
                                  getCurrPosition(),
                                  getCurrVelocity(),
                                  getCurrAcceleration());
    nextSampleStep();
  }

  while(!isLastSample());
  // also print last sample
  printf("%lf, %lf, %lf, %lf\n",getCurrTime(),
                                getCurrPosition(),
                                getCurrVelocity(),
                                getCurrAcceleration());
}

bool ecmcAxisPVTSequence::getBusy() { 
  return busy_;
}

void ecmcAxisPVTSequence::setBusy(bool busy) { 
  busy_ = busy;
}

void ecmcAxisPVTSequence::clear() { 
  segments_.clear();
  points_.clear();
  resultPosActArray_.clear();
  resultPosErrArray_.clear();
  segmentCount_ = 0;
  pointCount_   = 0;
  totalTimeNs_  = 0;
  currTimeNs_   = 0;
  busy_         = false;
  currSegIndex_ = 0;
  currSegIndexOld_ = 0;
}

int ecmcAxisPVTSequence::validateRT() {
  if(segmentCount_==0 || data_ == NULL) {
    printf(" ecmcAxisPVTSequence::validateRT(): Error: Segment count 0\n");
    return ERROR_SEQ_PVT_CFG_INVALID;
  }
  for(uint i = 0; i < segmentCount_; ++i) {
    printf("seg[%u]: starttime %lf, stoptime %lf\n",i,
    segments_[i]->getStartPoint()->timeNs_,
    segments_[i]->getEndPoint()->timeNs_);
  }
  return 0;
}

int ecmcAxisPVTSequence::setPositionOffset(double offset) {
  positionOffset_ = offset;
  return 0;
}

int ecmcAxisPVTSequence::setExecute(bool execute) {
  execute_ = execute;
  printf("ecmcAxisPVTSequence::setExecute(%d)\n",execute);
  if (!executeOld_ && execute_) {    
    initSeq();
    busy_ = true;
  }

  if(!execute) {
    busy_ = false;
    //initSeq();
  }

  executeOld_ = execute_;
  return 0;
}

bool ecmcAxisPVTSequence::getExecute() {
  return execute_;
}

int ecmcAxisPVTSequence::getCurrentSegementId() {
  return currSegIndex_;
}

int ecmcAxisPVTSequence::setAxisDataRef(ecmcAxisData *data) {
  data_ = data;
  return 0;
}
