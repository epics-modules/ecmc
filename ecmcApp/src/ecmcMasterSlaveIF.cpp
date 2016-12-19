/*
 * ecmcMasterSlave.cpp
 *
 *  Created on: Mar 17, 2016
 *      Author: anderssandstrom
 */

#include "ecmcMasterSlaveIF.h"

ecmcMasterSlaveIF::ecmcMasterSlaveIF(int defaultAxisId,interfaceType ifType)
{
  initVars();
  defaultAxisId_=defaultAxisId;
  interfaceType_=ifType;
  transform_=new ecmcCommandTransform(3,ECMC_MAX_AXES);  //currently two commands
  transform_->addCmdPrefix(TRANSFORM_EXPR_VARIABLE_TRAJ_PREFIX,ECMC_TRANSFORM_VAR_TYPE_TRAJ);
  transform_->addCmdPrefix(TRANSFORM_EXPR_VARIABLE_ENC_PREFIX,ECMC_TRANSFORM_VAR_TYPE_ENC);
  transform_->addCmdPrefix(TRANSFORM_EXPR_INTERLOCK_PREFIX,ECMC_TRANSFORM_VAR_TYPE_IL);
}

ecmcMasterSlaveIF::~ecmcMasterSlaveIF()
{
  //delete transform_;
  delete transform_;
}

void ecmcMasterSlaveIF::initVars()
{
  for(int i=0;i<MAX_TRANSFORM_INPUTS;i++){
    inputDataInterface_[i]=NULL;
  }
  dataSource_=ECMC_DATA_SOURCE_INTERNAL;
  numInputSources_=0;
  gearRatio_=1;
  defaultAxisId_=0;
  interfaceType_=ECMC_ENCODER_INTERFACE;
  interlockDefiendinExpr_=false;
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
  inputDataInterface_[index]=masterData;
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

ecmcCommandTransform *ecmcMasterSlaveIF::getExtInputTransform()
{
  return transform_;
}

int ecmcMasterSlaveIF::getExtInputPos(int axisId,int commandIndex,double *val)
{
  *val=transform_->getData(commandIndex,axisId)*gearRatio_;
  return 0;
}

int ecmcMasterSlaveIF::getExtInputPos(int commandIndex,double *val)
{
  *val=transform_->getData(commandIndex,defaultAxisId_)*gearRatio_;
  return 0;
}

bool ecmcMasterSlaveIF::getExtInputInterlock(int axisId,int commandIndex)
{
  return (bool)transform_->getData(commandIndex,axisId);;
}

bool ecmcMasterSlaveIF::getExtInputInterlock(int commandIndex)
{
  return (bool)transform_->getData(commandIndex,defaultAxisId_);
}

int ecmcMasterSlaveIF::transformRefresh()
{
  int error=0;
  //Trajectory
  for(int i=0;i<ECMC_MAX_AXES;i++){
    if(inputDataInterface_[i]!=NULL){
      error=transform_->setData(inputDataInterface_[i]->getPosition(),ECMC_TRANSFORM_VAR_TYPE_TRAJ,i);
      if(error){
        return error;
      }
    }
    else{
      error=transform_->setData(0,ECMC_TRANSFORM_VAR_TYPE_TRAJ,i);
      if(error){
        return error;
      }
    }
  }

  //Encoder
  for(int i=0;i<ECMC_MAX_AXES;i++){
    if(inputDataInterface_[i+ECMC_MAX_AXES]!=NULL){
      error=transform_->setData(inputDataInterface_[i+ECMC_MAX_AXES]->getPosition(),ECMC_TRANSFORM_VAR_TYPE_ENC,i);
      if(error){
        return error;
      }
    }
    else{
      error=transform_->setData(0,ECMC_TRANSFORM_VAR_TYPE_ENC,i);
      if(error){
        return error;
      }
    }
  }


  //Interlocks (for both encoder and Traj so AND operation)
  for(int i=0;i<ECMC_MAX_AXES;i++){
    if(inputDataInterface_[i]!=NULL && inputDataInterface_[i+ECMC_MAX_AXES]!=NULL){
      //Trajectory and Encoder
      error=transform_->setData(inputDataInterface_[i]->getInterlock() && inputDataInterface_[i+ECMC_MAX_AXES]->getInterlock(),ECMC_TRANSFORM_VAR_TYPE_IL,i);
      if(error){
        return error;
      }
    }
    else{
      error=transform_->setData(0,ECMC_TRANSFORM_VAR_TYPE_IL,i);
      if(error){
        return error;
      }
    }
  }
  return transform_->refresh();
}

int ecmcMasterSlaveIF::validate()
{
  return validate(dataSource_);
}


int ecmcMasterSlaveIF::validate(dataSource nextDataSource)
{
  char axisIdStr[12];
  sprintf(axisIdStr, "%d", defaultAxisId_);
  std::string strToFind="";
  bool found=false;
  if(nextDataSource!=ECMC_DATA_SOURCE_INTERNAL){
    //Ensure that setpoint is defined in expression
    switch(interfaceType_){
      case  ECMC_ENCODER_INTERFACE:
        strToFind=TRANSFORM_EXPR_VARIABLE_ENC_PREFIX;
	strToFind.append(axisIdStr);
	strToFind.append(":=");
	found=transform_->getExpression()->find(strToFind)!=std::string::npos;
	if(!found){
	  return ERROR_MASTER_DATA_IF_EXPRESSION_VAR_ENC_MISSING;
	}
        break;
      case ECMC_TRAJECTORY_INTERFACE:
        strToFind=TRANSFORM_EXPR_VARIABLE_TRAJ_PREFIX;
	strToFind.append(axisIdStr);
	strToFind.append(":=");
	found=transform_->getExpression()->find(strToFind)!=std::string::npos;
	if(!found){
	  return ERROR_MASTER_DATA_IF_EXPRESSION_VAR_TRAJ_MISSING;
	}
        break;
    }
    return transform_->validate();
  }

  //See if interlock is defined then transform needs to be executed
  strToFind=TRANSFORM_EXPR_INTERLOCK_PREFIX;
  strToFind.append(axisIdStr);
  interlockDefiendinExpr_=transform_->getExpression()->find(strToFind)!=std::string::npos;

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

bool ecmcMasterSlaveIF::getInterlockDefined()
{
  return interlockDefiendinExpr_;
}
