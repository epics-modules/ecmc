/*
 * cMcuFilter.h
 *
 *  Created on: Jan 12, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCFILTER_H_
#define ECMCFILTER_H_
#include <iomanip>
#include <iostream>
#include <cstring>
#include <stdio.h>
#include "ecmcError.h"
#include "cmd.h"

#define CUTOFF 100
#define FILTER_BUFFER_SIZE_POS 150
#define FILTER_BUFFER_SIZE_VEL 100

class ecmcFilter : public ecmcError
{
public:
  ecmcFilter(double sampleTime);
  ~ecmcFilter();
  //double Update(double input);

  double positionBasedVelAveraging(double actPosition);  //returns velocity
  //double lowPassExponential(double input, double average, double factor);
  //double lowPassFrequency(double input);
  void setSampleTime(double sampleTime);
  int reset();

private:
  void initVars();
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

#endif /* ECMCFILTER_H_ */
