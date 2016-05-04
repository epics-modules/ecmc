/*
 * ecmcMasterSlave.cpp
 *
 *  Created on: Mar 17, 2016
 *      Author: anderssandstrom
 */

#include "ecmcMasterSlaveIF.h"

ecmcMasterSlaveIF::ecmcMasterSlaveIF(double sampleTime)
{
  initVars();
  transform_=new ecmcTransform();
  setSampleTime(sampleTime);
}

ecmcMasterSlaveIF::~ecmcMasterSlaveIF()
{
  delete transform_;
  transform_=NULL;
}

void ecmcMasterSlaveIF::initVars()
{
  for(int i=0;i<MAX_TRANSFORM_INPUTS;i++){
    inputDataInterface_[i]=NULL;
  }
  dataSource_=ECMC_DATA_SOURCE_INTERNAL;
  numInputSources_=0;
  sampleTime_=0;
}

ecmcMasterSlaveData *ecmcMasterSlaveIF::getOutputDataInterface()
{
  return &outputDataInterface_;
}

int ecmcMasterSlaveIF::addInputDataInterface(ecmcMasterSlaveData *masterData, int index)
{
  if(index>=MAX_TRANSFORM_INPUTS || index<0){
    return ERROR_MASTER_DATA_IF_INDEX_OUT_OF_RANGE;
  }
  //_inputDataInterface[_nNumInputSources]=masterData;
  inputDataInterface_[index]=masterData;
  transform_->addInputDataObject(inputDataInterface_[index],index);
  numInputSources_++;
  return 0;
}

ecmcMasterSlaveData *ecmcMasterSlaveIF::getExtInputDataInterface(int index)
{
  if(index>=MAX_AXES || index<0){
    return NULL;
  }
  return inputDataInterface_[index];
}

int ecmcMasterSlaveIF::setDataSourceType(dataSource refSource)
{
  dataSource_=refSource;
  transform_->setDataSource(dataSource_);
  return 0;
}

dataSource ecmcMasterSlaveIF::getDataSourceType()
{
  return dataSource_;
}

int ecmcMasterSlaveIF::getNumExtInputSources()
{
  return numInputSources_;
}

int ecmcMasterSlaveIF::setSampleTime(double sampleTime)
{
  sampleTime_=sampleTime;
  transform_->setSampleTime(sampleTime_);
  return 0;
}

ecmcTransform *ecmcMasterSlaveIF::getExtInputTransform()
{
  return transform_;
}

int ecmcMasterSlaveIF::getExtInputPos(double *val)
{
  double temp=0;
  int errorCode=transform_->getOutput(&temp);
  if(errorCode){
    return errorCode;
  }
  *val=temp;
  return 0;
}

int ecmcMasterSlaveIF::getExtInputVel(double *val)
{
  double temp=0;
  int errorCode=transform_->getDiffOutput(&temp);
  if(errorCode){
    return errorCode;
  }
  *val=temp;
  return 0;
}

bool ecmcMasterSlaveIF::getExtInputInterlock()
{
  return transform_->getInterlock();
}

int ecmcMasterSlaveIF::transformRefresh()
{
  return transform_->refresh();
}
