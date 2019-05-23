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
#include "../com/ecmcOctetIF.h"

#define CUTOFF 100
#define FILTER_BUFFER_SIZE_POS 150
#define FILTER_BUFFER_SIZE_VEL 100

class ecmcFilter : public ecmcError {
 public:
  explicit ecmcFilter(double sampleTime);
  ~ecmcFilter();
  void   setSampleTime(double sampleTime);
  int    reset();
  // Init filter to certain position
  int    initFilter(double pos);
  double getFiltVelo(double input);

 private:
  void   initVars();
  double bufferVel_[FILTER_BUFFER_SIZE_VEL];
  double my_;
  double last_;
  double lastOutput_;
  double sampleTime_;
  int indexVel_;
};

#endif  /* ECMCFILTER_H_ */
