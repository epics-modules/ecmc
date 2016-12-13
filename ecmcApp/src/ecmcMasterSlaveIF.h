/*
 * ecmcMasterSlave.h
 *
 *  Created on: Mar 17, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCMASTERSLAVEIF_H_
#define ECMCMASTERSLAVEIF_H_

#include "ecmcDefinitions.h"
#include "ecmcMasterSlaveData.h"
#include "ecmcTransform.h"

//MASTERDATA INTERFACE
#define ERROR_MASTER_DATA_IF_INDEX_OUT_OF_RANGE 0x30100
#define ERROR_MASTER_DATA_IF_GEAR_RATIO_DENOM_ZERO 0x30101

class ecmcMasterSlaveIF
{
public:
  ecmcMasterSlaveIF(double dSampleTime);
  ~ecmcMasterSlaveIF();
  ecmcMasterSlaveData *getOutputDataInterface();
  int addInputDataInterface(ecmcMasterSlaveData *masterData,int index);
  ecmcMasterSlaveData *getExtInputDataInterface(int index);
  int setDataSourceType(dataSource refSource);
  dataSource getDataSourceType();
  int getNumExtInputSources();
  int setSampleTime(double sampleTime);
  ecmcTransform *getExtInputTransform();
  int getExtInputPos(double *val);
  int getExtInputVel(double *val);
  bool getExtInputInterlock();
  int transformRefresh();
  int validate();
  int setGearRatio(double ratioNum, double ratioDenom);
  int getGearRatio(double *ratio);

private:
  void initVars();
  ecmcMasterSlaveData *inputDataInterface_[MAX_TRANSFORM_INPUTS];
  ecmcMasterSlaveData outputDataInterface_;
  dataSource dataSource_;
  ecmcTransform *transform_;
  int numInputSources_;
  double sampleTime_;
  double gearRatio_;
};

#endif /* ECMCMASTERSLAVEIF_H_ */
