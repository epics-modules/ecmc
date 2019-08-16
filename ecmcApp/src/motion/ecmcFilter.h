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

#define ERROR_AXIS_FILTER_ALLOC_FAIL 0x230000

#define FILTER_BUFFER_SIZE_VEL 100

class ecmcFilter : public ecmcError {
 public:
  explicit ecmcFilter(double sampleTime);
  ~ecmcFilter();
  void   setSampleTime(double sampleTime);
  int    setFilterSize(size_t size);
  int    reset();
  // Init filter to certain position
  int    initFilter(double pos);
  double getFiltVelo(double input);

 private:
  void   initVars();
  double *bufferVel_;
  double my_;
  double last_;
  double lastOutput_;
  double sampleTime_;
  size_t indexVel_;
  size_t filterSize_;
};

#endif  /* ECMCFILTER_H_ */
