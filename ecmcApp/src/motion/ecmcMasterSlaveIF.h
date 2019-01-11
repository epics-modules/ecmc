/*
 * ecmcMasterSlave.h
 *
 *  Created on: Mar 17, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCMASTERSLAVEIF_H_
#define ECMCMASTERSLAVEIF_H_

#include <stdio.h>
#include <string>
#include "../main/ecmcDefinitions.h"
#include "../main/ecmcError.h"
#include "ecmcMasterSlaveData.h"
#include "ecmcCommandTransform.h"
#include "ecmcFilter.h"

// MASTERDATA INTERFACE
#define ERROR_MASTER_DATA_IF_INDEX_OUT_OF_RANGE 0x30100
#define ERROR_MASTER_DATA_IF_GEAR_RATIO_DENOM_ZERO 0x30101
#define ERROR_MASTER_DATA_IF_EXPRESSION_VAR_TRAJ_MISSING 0x30102
#define ERROR_MASTER_DATA_IF_EXPRESSION_VAR_ENC_MISSING 0x30103
#define ERROR_MASTER_DATA_IF_VALIDATE_CHAR_BUFFER_OVERFLOW 0x30104

enum interfaceType {
  ECMC_ENCODER_INTERFACE    = 0,
  ECMC_TRAJECTORY_INTERFACE = 1,
};


class ecmcMasterSlaveIF : public ecmcError {
 public:
  ecmcMasterSlaveIF(int           defaultAxisId,
                    interfaceType ifType,
                    double        sampleTime);
  ~ecmcMasterSlaveIF();
  ecmcMasterSlaveData * getOutputDataInterface();
  int                   addInputDataInterface(ecmcMasterSlaveData *masterData,
                                              int                  index);
  ecmcMasterSlaveData * getExtInputDataInterface(int index);
  int                   setDataSourceType(dataSource refSource);
  dataSource            getDataSourceType();
  int                   getNumExtInputSources();
  ecmcCommandTransform* getExtInputTransform();
  int                   validate();
  int                   validate(dataSource nextDataSource);
  int                   setGearRatio(double ratioNum,
                                     double ratioDenom);
  int                   getGearRatio(double *ratio);
  bool                  getInterlockDefined();
  int                   refreshInputs();
  double                getInputPos();
  double                getInputVel();
  interlockTypes        getInputIlock();
  int                   getExtInputPos(int     axisId,
                                       int     commandIndex,
                                       double *val);
  // For default axis number
  int                   getExtInputPos(int     commandIndex,
                                       double *val);
  bool                  getExtInputInterlock(int axisId,
                                             int commandIndex);
  // For default axis number
  bool                  getExtInputInterlock(int commandIndex);
  int                   setEnableVelFilter(bool enable);
  void                  printCurrentState();

 private:
  int                   transformRefresh();
  void                  initVars();
  ecmcMasterSlaveData *inputDataInterface_[MAX_TRANSFORM_INPUTS];
  ecmcMasterSlaveData outputDataInterface_;
  dataSource dataSource_;
  int numInputSources_;
  double sampleTime_;
  double gearRatio_;
  double oldPos_;
  ecmcCommandTransform *transform_;
  int defaultAxisId_;
  interfaceType interfaceType_;
  bool interlockDefiendinExpr_;
  double externalPosition_;
  double externalPositionOld_;
  double externalVelocity_;
  interlockTypes externalInterlock_;
  ecmcFilter *velocityFilter_;
  bool velocityFilterEnable_;
};

#endif  /* ECMCMASTERSLAVEIF_H_ */
