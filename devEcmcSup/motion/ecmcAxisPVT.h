/*************************************************************************\
* Copyright (c) 2024 Paul Scherrer Institut
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAxisPVT.h
*
*  Created on: April 25, 2024
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCAXISPVT_H_
#define ECMCAXISPVT_H_

#include <vector>
#include <cstdio>

class ecmcPvtPoint {
  public:
    double position_;
    double velocity_;
    double time_;
    ecmcPvtPoint(double position, double velocity, double timeS) {
      position_ = position;
      velocity_ = velocity;
      time_ = timeS;
    };
};

// Third order polynom between 2 ecmcPvtPoints
class ecmcPvtSegment {
  private:
    ecmcPvtPoint *startPnt_;
    ecmcPvtPoint *endPnt_;
    double k0_,k1_,k2_,k3_;
    double timeSpan_,range_;
    // Allocate all variables in object
    double timeInSeg_;
    double timeInSegPow2_;
    // Calc coeffs
    bool build() {
      timeSpan_ = endPnt_->time_ - startPnt_->time_;
      if (timeSpan_ < 0) {
        return false;
      };
      range_    = endPnt_->position_ - startPnt_->position_;
      k0_ = startPnt_->position_;
      k1_ = startPnt_->velocity_;
      k2_ = 3 * range_ / (timeSpan_ * timeSpan_) - (2 * startPnt_->velocity_ + endPnt_->velocity_) / timeSpan_;
      k3_ = -2 * range_ / (timeSpan_ * timeSpan_ * timeSpan_) + (startPnt_->velocity_ + endPnt_->velocity_) / (timeSpan_ * timeSpan_);
      return true;
    };

  public:
    ecmcPvtSegment(ecmcPvtPoint *start, ecmcPvtPoint *end) {
      startPnt_      = start;
      endPnt_        = end;
      k0_            = 0;
      k1_            = 0;
      k2_            = 0;
      k3_            = 0;
      timeSpan_      = 0;
      timeInSeg_     = 0;
      timeInSegPow2_ = 0;
      range_         = 0;
      // Calc poly coeffs
      build();
    };

    ecmcPvtPoint *getStartPoint() {
     return startPnt_;
    };
    
    ecmcPvtPoint *getEndPoint() {
     return endPnt_;
    };

    bool isTimeValid(double time) {
      return time >= startPnt_->time_ && time <= endPnt_->time_;
    };
    
    double position(double time) {
      if(!isTimeValid(time)) {
        // Exception
        printf("ERROR: TIME INVALID, NOT WITHIN SEGMENT");
        return 0;
      };
      timeInSeg_ = time - startPnt_->time_;
      timeInSegPow2_ = timeInSeg_ * timeInSeg_;
      return k0_ + k1_ * timeInSeg_ + k2_ * timeInSegPow2_ + k3_ * timeInSegPow2_ * timeInSeg_;
    };

    double velocity(double time) {
      if(!isTimeValid(time)) {
        // Exception
        printf("ERROR: TIME INVALID, NOT WITHIN SEGMENT");
        return 0;
      };
      timeInSeg_ = time - startPnt_->time_;
      return k1_ + 2 * k2_ * timeInSeg_ + 3 * k3_ * timeInSeg_  * timeInSeg_;
    };

    double acceleration(double time) {
      if(!isTimeValid(time)) {
        // Exception
        printf("ERROR: TIME INVALID, NOT WITHIN SEGMENT");
        return 0;
      };
      return 2 * k2_ + 6 * k3_ * (time - startPnt_->time_);
    };
};

class ecmcPvtSequence {
  private:
    std::vector<ecmcPvtSegment*> segments_;
    std::vector<ecmcPvtPoint*> points_;
    size_t segmentCount_, pointCount_, currSegIndex_;
    double totalTime_, sampleTime_, currTime_;
    bool busy_;
    bool built_;

    void addSegment(ecmcPvtPoint *start, ecmcPvtPoint *end ) {
      segments_.push_back(new ecmcPvtSegment(start, end));
      segmentCount_++;
    };

  public:
    ecmcPvtSequence(double sampleTime) {
      segmentCount_ = 0;
      pointCount_   = 0;
      totalTime_    = 0;
      sampleTime_   = sampleTime;
      currTime_     = 0;
      busy_         = false;
      currSegIndex_ = 0;
      //built_        = false;
    };

    void addPoint(ecmcPvtPoint *pnt) {
      points_.push_back(pnt);
      pointCount_++;
      if(pointCount_ > 1) {
         addSegment(points_[pointCount_-2], points_[pointCount_-1]);
      };
      built_ = false;
    };
    
    double startTime(){
      if(segmentCount_ <= 0) {
          return -1;
      }
      return segments_[0]->getStartPoint()->time_;
    };

    double endTime(){
      if(segmentCount_ <= 0) {
          return -1;
      }
      return segments_[segmentCount_-1]->getEndPoint()->time_;
    };

    // Call before starting a seq
    void initSeq() {
      busy_ = true;
      currTime_ = startTime();
      currSegIndex_ = 0;
    };

    // void build() {
    //   for(size_t i = 0; i < segmentCount_; i++) {
    //     segments_[i]->build();
    //   };
    //   currTime_ = startTime();
    //   busy_ = false;
    //   built_ = true;
    // };

    bool validate() {
      return /*built_ && */ segmentCount_ > 0;
    };

    ecmcPvtSegment* getSeqmentAtTime(double time) {
      if(segmentCount_ <= 0) {
        return NULL;
      };

      if(time < startTime() || time > endTime() ) {
        return NULL;
      };

      for(size_t i=0; i < segmentCount_; i++) {
        if(time >= segments_[i]->getStartPoint()->time_  && 
           time < segments_[i]->getEndPoint()->time_ ) {
          return segments_[i];
        }
      }
    
      // Check if last seq endtime (equals)
      if( time == segments_[segmentCount_-1]->getEndPoint()->time_) {
        return segments_[segmentCount_-1];
      }
      
      return NULL;
    }

    bool isLastSample() {
      return isLastSample(currTime_);
    };

    bool isLastSample(double time) {
      return time >= endTime();
    };

    bool isTimeValid(double time) {
      if(segmentCount_ <= 0) {
        return false;
      };

      if(time < startTime() || time > endTime() ) {
        return false;
      };
      return true;
    }

    // Go to next sample in time
    // return true as long not exceeding endtime
    bool nextSampleStep(){
      // Increase time
      currTime_ = currTime_ + sampleTime_;
      
      // Switch segment?
      if(currTime_ > segments_[currSegIndex_]->getEndPoint()->time_) {
        if(currSegIndex_ < segmentCount_) {
          // the time must be in the next segment
          currSegIndex_++;
        } else {  // last segment and last sample, set to curr time to end-time
          currTime_ = endTime();
        }
      }
      return currTime_ < endTime();
    }
    
    // For RT sequential access
    double getCurrPosition(){
      return segments_[currSegIndex_]->position(currTime_);
    };
    
    // For RT sequential access
    double getCurrVelocity(){
      return segments_[currSegIndex_]->velocity(currTime_);
    };
    
    // For RT sequential access
    double getCurrAcceleration(){
      return segments_[currSegIndex_]->acceleration(currTime_);
    };

    double getCurrTime(){
      return currTime_;
    };

    // For non RT access
    double position(double time, int *valid) {
      ecmcPvtSegment* temp = getSeqmentAtTime(time);
      if(!temp) {
          *valid = 0;
          return 0;
      }
      *valid = 1;
      return temp->position(time);
    }

    // For non RT access
    double velocity(double time, int *valid) {
      ecmcPvtSegment* temp = getSeqmentAtTime(time);
      if(!temp) {
          *valid = 0;
          return 0;
      }
      *valid = 1;
      return temp->velocity(time);
    }

    // For non RT access
    double acceleration(double time, int *valid) {
      ecmcPvtSegment* temp = getSeqmentAtTime(time);
      if(!temp) {
          *valid = 0;
          return 0;
      }
      *valid = 1;
      return temp->acceleration(time);
    }
};
#endif  /* ECMCAXISPVT_H_ */
