/*
 * cMcuFilter.cpp
 *
 *  Created on: Jan 12, 2016
 *      Author: anderssandstrom
 */

#include "ecmcFilter.h"

ecmcFilter::ecmcFilter(double sampleTime)
{
  initVars();
  sampleTime_=sampleTime;
//  double constant=10;  //TODO move
//  my_ = t / (t + constant);
}

ecmcFilter::~ecmcFilter() {
  ;
}

void ecmcFilter::initVars(){
  errorReset();
//  my_ = 0;
//  last_ = 0;
//  lastOuput_=0;
//  sampleTime_=1;
  for(int i=0;i<FILTER_BUFFER_SIZE_POS;i++){
    bufferPos_[i]=0;
  }
  for(int i=0;i<FILTER_BUFFER_SIZE_VEL;i++){
    bufferVel_[i]=0;
  }
  indexPos_=0;
  indexVel_=0;
}

/*double ecmcFilter::Update(double input)
{
  last_ = my_ * (input - last_)+ last_;
  return last_;
}*/

double ecmcFilter::lowPassAveraging(double input)
{
  double sum = 0;
  bufferVel_[indexVel_] = input;
  indexVel_++;
  if(indexVel_>=FILTER_BUFFER_SIZE_VEL){
      indexVel_=0;
  }
  for(int i = 0; i<(FILTER_BUFFER_SIZE_VEL); i++){
    sum =sum +bufferVel_[i];
  }
  return sum/((double)FILTER_BUFFER_SIZE_VEL);
}

double ecmcFilter::positionBasedVelAveraging(double actPosition)  //returns velocity
{
  if(indexPos_>=FILTER_BUFFER_SIZE_POS){
      indexPos_=0;
  }
  bufferPos_[indexPos_] = actPosition;
  int nextIndex=indexPos_+1;

  if(nextIndex>=FILTER_BUFFER_SIZE_POS){
    nextIndex=0;
  }

  double vel=lowPassAveraging((bufferPos_[indexPos_]-bufferPos_[nextIndex])/(sampleTime_*(double)FILTER_BUFFER_SIZE_POS));
  indexPos_=nextIndex;
  return vel;
}

int ecmcFilter::reset()
{
  initVars();
  return 0;
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
