/*************************************************************************\
* Copyright (c) 2024 Paul Scherrer Institut
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAxisPVTSequence.h
*
*  Created on: April 25, 2024
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCAXISPVT_H_
#define ECMCAXISPVT_H_

#define ERROR_SEQ_PVT_CFG_INVALID 0x14D1D

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
    }

    void print() {
       printf("%lf,%lf,%lf\n",time_, position_, velocity_);  
    } 
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
      }
      range_    = endPnt_->position_ - startPnt_->position_;
      k0_ = startPnt_->position_;
      k1_ = startPnt_->velocity_;
      k2_ = 3 * range_ / (timeSpan_ * timeSpan_) - (2 * startPnt_->velocity_ + endPnt_->velocity_) / timeSpan_;
      k3_ = -2 * range_ / (timeSpan_ * timeSpan_ * timeSpan_) + (startPnt_->velocity_ + endPnt_->velocity_) / (timeSpan_ * timeSpan_);
      return true;
    }

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
    }

    ecmcPvtPoint *getStartPoint() {
     return startPnt_;
    }
    
    ecmcPvtPoint *getEndPoint() {
     return endPnt_;
    }

    bool isTimeValid(double time) {
      return time >= startPnt_->time_ && time <= endPnt_->time_;
    }
    
    double position(double time) {
      if(!isTimeValid(time)) {
        // Exception
        printf("ERROR: TIME INVALID, NOT WITHIN SEGMENT");
        return 0;
      }
      timeInSeg_ = time - startPnt_->time_;
      timeInSegPow2_ = timeInSeg_ * timeInSeg_;
      return k0_ + k1_ * timeInSeg_ + k2_ * timeInSegPow2_ + k3_ * timeInSegPow2_ * timeInSeg_;
    }

    double velocity(double time) {
      if(!isTimeValid(time)) {
        // Exception
        printf("ERROR: TIME INVALID, NOT WITHIN SEGMENT");
        return 0;
      }
      timeInSeg_ = time - startPnt_->time_;
      return k1_ + 2 * k2_ * timeInSeg_ + 3 * k3_ * timeInSeg_  * timeInSeg_;
    }

    double acceleration(double time) {
      if(!isTimeValid(time)) {
        // Exception
        printf("ERROR: TIME INVALID, NOT WITHIN SEGMENT");
        return 0;
      }
      return 2 * k2_ + 6 * k3_ * (time - startPnt_->time_);
    }
};

class ecmcAxisPVTSequence {
  public:
    ecmcAxisPVTSequence(double sampleTime);
    void   setSampleTime(double sampleTime);
    void   addPoint(ecmcPvtPoint *pnt);
    double startTime();
    double endTime();    
    void   initSeq(); // Call before starting a seq
    bool   validate();
    bool   isLastSample();
    bool   isLastSample(double time);
    bool   isTimeValid(double time);
    bool   nextSampleStep();        // Go to next sample in time, return true as long not exceeding endtime
    double getCurrPosition();       // For RT sequential access
    double getCurrVelocity();       // For RT sequential access
    double getCurrAcceleration();   // For RT sequential access
    double getCurrTime();
    int    getCurrentSegementId();
    double position(double time, int *valid);     // For non RT access
    double velocity(double time, int *valid);     // For non RT access
    double acceleration(double time, int *valid); // For non RT access
    bool   getBusy();
    void   setBusy(bool busy);
    void   print();
    void   printRT();
    void   clear();
    int    validateRT();
    int    setPositionOffset(double offset);  // For running relative
    int    setExecute(bool execute);
    bool   getExecute();
    

  private:
    void            addSegment(ecmcPvtPoint *start, ecmcPvtPoint *end );
    ecmcPvtSegment* getSeqmentAtTime(double time);
    std::vector<ecmcPvtSegment*> segments_;
    std::vector<ecmcPvtPoint*> points_;
    size_t segmentCount_, pointCount_, currSegIndex_;
    double totalTime_, sampleTime_, currTime_;
    bool busy_;
    double positionOffset_;  // For relative motion
    bool execute_;
    bool executeOld_;
};
#endif  /* ECMCAXISPVT_H_ */
