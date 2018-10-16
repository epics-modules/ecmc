/*
 *
 *  Created on: Oct 15, 2018
 *      Author: anderssandstrom
 */

#include "ecmcPLCs.h"

ecmcPLCs::ecmcPLCs(ecmcEc *ec)
{
  initVars();
  ec_=ec;
}

ecmcPLCs::~ecmcPLCs()
{
  for(int i=0;i<ECMC_MAX_PLCS;i++){
    delete plcs_[i];
  }
}

void ecmcPLCs::initVars()
{
  for(int i=0; i<ECMC_MAX_PLCS;i++){
    plcs_[i]=NULL;
    plcEnable_[i]=NULL;
    plcError_[i]=NULL;
    plcScanTime_[i]=NULL;
  }
  
  for(int i=0; i<ECMC_MAX_AXES;i++){
    axes_[i]=NULL;
  }

  ec_=NULL;
}

int ecmcPLCs::createPLC(int plcIndex, int skipCycles)
{
  if(plcIndex<0 || plcIndex>=ECMC_MAX_PLCS){
    return ERROR_PLCS_INDEX_OUT_OF_RANGE;
  }

  if(!ec_->getInitDone())
    return ERROR_PLCS_EC_NOT_INITIALIZED;

  if(plcs_[plcIndex]){
    delete plcs_[plcIndex];
    plcs_[plcIndex]=NULL;
  }

  plcs_[plcIndex]=new ecmcPLC(ec_,skipCycles);

  //Set axes pointers (for the already configuered axes)
  for(int i=0; i<ECMC_MAX_AXES;i++){
    plcs_[plcIndex]->setAxisArrayPointer(axes_[i],i);
  }

  return plcs_[plcIndex]->getErrorID();
}

int ecmcPLCs::setAxisArrayPointer(ecmcAxisBase *axis,int index)
{
  if(index>=ECMC_MAX_AXES || index<0){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLCS_AXIS_INDEX_OUT_OF_RANGE);
  }
  axes_[index]=axis;
  return 0;
}

int ecmcPLCs::validate()
{
  for(int i=0; i<ECMC_MAX_PLCS;i++){
    if(plcs_[i]!=NULL){
      int errorCode=plcs_[i]->validate();
      if(errorCode){
        return errorCode;
      }
    }
  }
  return 0;
}

int ecmcPLCs::execute(bool ecOK)
{
  for(int i=0;i<ECMC_MAX_PLCS;i++){
    if(plcs_[i]!=NULL){
	    plcs_[i]->execute(ecOK);
    }
  }  
  return 0;
}

int ecmcPLCs::getExpr(int plcIndex, std::string *expr)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  expr=plcs_[plcIndex]->getExpr();
  return 0;
}

int ecmcPLCs::setExpr(int plcIndex,char *expr)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  return plcs_[plcIndex]->setExpr(expr);
}

int ecmcPLCs::addExprLine(int plcIndex,char *expr)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  return plcs_[plcIndex]->addExprLine(expr);
}

int ecmcPLCs::clearExpr(int plcIndex)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  return plcs_[plcIndex]->clearExpr();
}

int ecmcPLCs::compileExpr(int plcIndex)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  return plcs_[plcIndex]->compile();
}

int ecmcPLCs::setEnable(int plcIndex,int enable)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  return plcs_[plcIndex]->setEnable(enable);
}

int ecmcPLCs::getEnable(int plcIndex,int *enabled)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  *enabled=plcs_[plcIndex]->getEnable();
  return 0;
}

int ecmcPLCs::deletePLC(int plcIndex)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  delete plcs_[plcIndex];
  return 0;
}

int ecmcPLCs::getCompiled(int plcIndex,int *compiled)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  *compiled=plcs_[plcIndex]->getCompiled();
  return 0;
}