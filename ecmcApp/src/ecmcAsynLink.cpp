/*
 * ecmcAsynLink.cpp
 *
 *  Created on: Dec 2, 2015
 *      Author: anderssandstrom
 */

#include "ecmcAsynLink.h"

ecmcAsynLink::ecmcAsynLink()
{
  initVars();
}

void ecmcAsynLink::initVars()
{
  asynParameterIndex_=-1;
  asynParameterType_=asynParamFloat64;
  asynPortDriver_=NULL;
  asynUpdateCycles_=0;
  asynUpdateCycleCounter_=0;
}

ecmcAsynLink::~ecmcAsynLink()
{
  ;
}

int ecmcAsynLink::setAsynParameterIndex(int index)
{
  asynParameterIndex_=index;
  return 0;
}

int ecmcAsynLink::getAsynParameterIndex()
{
  return asynParameterIndex_;
}

int ecmcAsynLink::setAsynParameterType(asynParamType parType)
{
  asynParameterType_=parType;
  return 0;
}

int ecmcAsynLink::getAsynParameterType()
{
  return asynParameterType_;
}

int ecmcAsynLink::setAsynPortDriver(ecmcAsynPortDriver *asynPortDriver)
{
  asynPortDriver_=asynPortDriver;
  return 0;
}

int ecmcAsynLink::setAsynParameterSkipCycles(int skipCycles)
{
  asynUpdateCycles_=skipCycles;
  return 0;
}
