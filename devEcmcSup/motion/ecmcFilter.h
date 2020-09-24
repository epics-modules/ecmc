/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcFilter.h
*
*  Created on: Jan 12, 2016
*      Author: anderssandstrom
*
\*************************************************************************/
#ifndef ECMCFILTER_H_
#define ECMCFILTER_H_
#include <stdio.h>
#include <iomanip>
#include <iostream>
#include <cstring>
#include "../main/ecmcError.h"
#include "../com/ecmcOctetIF.h"

#define ERROR_AXIS_FILTER_ALLOC_FAIL 0x230000

#define FILTER_BUFFER_SIZE_DEF 100
#define FILTER_POS_MODULO_OVER_UNDER_FLOW_LIMIT 0.7

class ecmcFilter : public ecmcError {
 public:
  explicit ecmcFilter(double sampleTime); 
  explicit ecmcFilter(double sampleTime, size_t size);
  ~ecmcFilter();
  void   setSampleTime(double sampleTime);
  int    setFilterSize(size_t size);
  int    reset();
  // Init filter to certain position
  int    initFilter(double pos);
  double getFiltVelo(double input);
  // Need to compensate for modulo range in postion filter
  double getFiltPos(double pos, double modRange);

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
