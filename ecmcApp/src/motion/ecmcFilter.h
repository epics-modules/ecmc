/*
 * cMcuFilter.h
 *
 *  Created on: Jan 12, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCFILTER_H_
#define ECMCFILTER_H_
#include <stdio.h>
#include <iomanip>
#include <iostream>
#include <cstring>
#include "../main/ecmcError.h"
#include "../com/cmd.h"

#define CUTOFF 100
#define FILTER_BUFFER_SIZE_POS 150
#define FILTER_BUFFER_SIZE_VEL 100

class ecmcFilter : public ecmcError {
 public:
  explicit ecmcFilter(double sampleTime);
  ~ecmcFilter();
  // returns velocity
  double positionBasedVelAveraging(double actPosition);
  void   setSampleTime(double sampleTime);
  int    reset();
  // Init filter to certain position
  int    initFilter(double pos);

 private:
  void   initVars();
  double lowPassAveraging(double input);
  double bufferVel_[FILTER_BUFFER_SIZE_VEL];
  double bufferPos_[FILTER_BUFFER_SIZE_POS];
  double my_;
  double last_;
  double lastOuput_;
  double sampleTime_;
  int indexPos_;
  int indexVel_;
};

#endif  /* ECMCFILTER_H_ */
