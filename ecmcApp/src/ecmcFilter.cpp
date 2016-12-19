/*
 * cMcuFilter.cpp
 *
 *  Created on: Jan 12, 2016
 *      Author: anderssandstrom
 */

#include "ecmcFilter.h"

ecmcFilter::ecmcFilter(double t)
{
  initVars();
  double constant=10;  //TODO move
  my_ = t / (t + constant);
}

ecmcFilter::~ecmcFilter() {
  ;
}

void ecmcFilter::initVars(){
  errorReset();
  my_ = 0;
  last_ = 0;
  lastOuput_=0;
  sampleTime_=1;
  for(int i=0;i<FILTER_BUFFER_SIZE;i++){
    buffer_[i]=0;
  }
  index_=0;
}

/*double ecmcFilter::Update(double input)
{
  last_ = my_ * (input - last_)+ last_;
  return last_;
}*/

double ecmcFilter::lowPassAveraging(double input)
{
  double sum = 0;
  buffer_[index_] = input;
  index_++;
  if(index_>=FILTER_BUFFER_SIZE){
    index_=0;
  }
  for(int i = 0; i<(FILTER_BUFFER_SIZE); i++){
    sum =sum +buffer_[i];
  }
  return sum/((double)FILTER_BUFFER_SIZE);
}

/*double ecmcFilter::lowPassExponential(double input, double average, double factor)
{
   return input*factor + (1-factor)*average;  // ensure factor belongs to  [0,1]
}*/

/*double ecmcFilter::lowPassFrequency(double input)
{
  double RC = 1.0/(CUTOFF*2*3.14);
  double dt = 1.0/sampleTime_;
  double alpha = dt/(RC+dt);
  return lastOuput_ = lastOuput_ + (alpha*(input - lastOuput_));
}*/

void ecmcFilter:: setSampleTime(double sampleTime)
{
  sampleTime_=sampleTime;
}
