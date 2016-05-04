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

#include "ecmcError.h"
#include "ecmcErrorsList.h"

#define CUTOFF 100
#define FILTER_BUFFER_SIZE 15

class ecmcFilter : public ecmcError
{
public:
  ecmcFilter(double t);
  ~ecmcFilter();
  double Update(double input);
  double lowPassAveraging(double input);
  double lowPassExponential(double input, double average, double factor);
  double lowPassFrequency(double input);
  void setSampleTime(double sampleTime);
private:
  void initVars();
  double buffer_[FILTER_BUFFER_SIZE];
  double my_;
  double last_;
  double lastOuput_;
  double sampleTime_;
};

#endif /* ECMCFILTER_H_ */
