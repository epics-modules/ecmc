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
  gearRatio_=1;
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
  if(index>=ECMC_MAX_AXES || index<0){
    return NULL;
  }
  return inputDataInterface_[index];
}

int ecmcMasterSlaveIF::setDataSourceType(dataSource refSource)
{
  if(refSource!=ECMC_DATA_SOURCE_INTERNAL){
    int error=transform_->getErrorID();
    if(error){
      return error;
    }
  }
  dataSource_=refSource;
  return transform_->setDataSource(dataSource_);
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
  *val=temp* gearRatio_;
  return 0;
}

int ecmcMasterSlaveIF::getExtInputVel(double *val)
{
  double temp=0;
  int errorCode=transform_->getDiffOutput(&temp);
  if(errorCode){
    return errorCode;
  }
  *val=temp*gearRatio_;
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

int ecmcMasterSlaveIF::validate()
{
  if(dataSource_!=ECMC_DATA_SOURCE_INTERNAL){
    return transform_->validate();
  }
  return 0;
}

int ecmcMasterSlaveIF::setGearRatio(double ratioNum, double ratioDenom)
{
  if( ratioDenom ==0){
    return ERROR_MASTER_DATA_IF_GEAR_RATIO_DENOM_ZERO;
  }

  gearRatio_=ratioNum/ratioDenom;
  return 0;
}

int ecmcMasterSlaveIF::getGearRatio(double *ratio)
{
  *ratio=gearRatio_;
  return 0;
}
