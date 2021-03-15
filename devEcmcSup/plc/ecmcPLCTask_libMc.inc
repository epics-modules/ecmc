/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPLCTask_libMc.inc
*
*  Created on: Nov 19, 2018
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ecmcPLC_libMc_inc_
#define ecmcPLC_libMc_inc_

#include "../main/ecmcErrorsList.h"

#define CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndex) {             \
  if(axIndex>=ECMC_MAX_AXES || axIndex<=0){                   \
    mc_errorCode=0;                                           \
    LOGERR("ERROR: Axis index out of range.\n");              \
    return (double)ERROR_PLC_AXIS_ID_OUT_OF_RANGE;}           \
    if(ecmcPLCTask::statAxes_[axIndex]==NULL){                \
      LOGERR("ERROR: Axis object NULL\n");                    \
      return (double)ERROR_PLC_AXIS_OBJECT_NULL;              \
    }                                                         \
  }                                                           \

const char* mcLibCmdList[] = { "mc_move_abs(",
                               "mc_move_rel(",
                               "mc_move_vel(",
                               "mc_home(",
                               "mc_halt(",
                               "mc_power(",    
                               "mc_get_err(",
                               "mc_reset(",
                               "mc_get_busy(",
                               "mc_get_homed(",
                               "mc_get_axis_err(",
                               "mc_set_enable_motion_funcs(",
                               "mc_move_ext_pos(",
                              };

ecmcAxisBase *ecmcPLCTask::statAxes_[ECMC_MAX_AXES]={};
ecmcDataStorage *ecmcPLCTask::statDs_[ECMC_MAX_DATA_STORAGE_OBJECTS]={};
static int statLastAxesExecuteAbs_[ECMC_MAX_AXES]={};
static int statLastAxesExecuteRel_[ECMC_MAX_AXES]={};
static int statLastAxesExecuteVel_[ECMC_MAX_AXES]={};
static int statLastAxesExecuteHalt_[ECMC_MAX_AXES]={};
static int statLastAxesExecuteHome_[ECMC_MAX_AXES]={};
static int mc_errorCode=0;
static int mc_cmd_count = 13;

// Note cannot use ecmcAxisBase::move* since the execution in plc with execute needs to be correct
inline double mc_move_abs(double axIndex,double execute,double pos, double vel, double acc,double dec)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);  
  
  // check traj source, must be internal
  if(ecmcPLCTask::statAxes_[index]->getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    mc_errorCode=ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  int trigg=!statLastAxesExecuteAbs_[index] && (bool)execute;
  statLastAxesExecuteAbs_[index]=execute;

  if (!(bool)execute){
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute((bool)execute);
    return mc_errorCode;
  }
  
  if(trigg){    
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute(0);
    if(mc_errorCode){
      return mc_errorCode;
    }
    ecmcPLCTask::statAxes_[index]->getSeq()->setTargetPos(pos);
    ecmcPLCTask::statAxes_[index]->getSeq()->setTargetVel(vel);
    ecmcPLCTask::statAxes_[index]->getTraj()->setDec(dec);
    ecmcPLCTask::statAxes_[index]->getTraj()->setAcc(acc);

    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setCommand(ECMC_CMD_MOVEABS);    
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setCmdData(0);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute((bool)execute);
    if(mc_errorCode){
      return mc_errorCode;
    }
  }

  return 0.0;
}

// Note cannot use ecmcAxisBase::move* since the execution in plc with execute needs to be correct
inline double mc_move_rel(double axIndex,double execute,double pos, double vel, double acc,double dec)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);  
  
  // check traj source, must be internal
  if(ecmcPLCTask::statAxes_[index]->getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    mc_errorCode=ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  int trigg=!statLastAxesExecuteRel_[index] && (bool)execute;
  statLastAxesExecuteRel_[index]=execute;
  
  if (!(bool)execute){
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute((bool)execute);
    return mc_errorCode;
  }

  if(trigg){    
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute(0);
    if(mc_errorCode){
      return mc_errorCode;
    }
    ecmcPLCTask::statAxes_[index]->getSeq()->setTargetPos(pos);
    ecmcPLCTask::statAxes_[index]->getSeq()->setTargetVel(vel);
    ecmcPLCTask::statAxes_[index]->getTraj()->setDec(dec);
    ecmcPLCTask::statAxes_[index]->getTraj()->setAcc(acc);

    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setCommand(ECMC_CMD_MOVEREL);    
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setCmdData(0);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute((bool)execute);
    if(mc_errorCode){
      return mc_errorCode;
    }
  }

  return 0.0;
}

// Note cannot use ecmcAxisBase::move* since the execution in plc with execute needs to be correct
inline double mc_move_vel(double axIndex,double execute, double vel, double acc,double dec)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);  

  // check traj source, must be internal
  if(ecmcPLCTask::statAxes_[index]->getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    mc_errorCode=ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  int trigg=!statLastAxesExecuteVel_[index] && (bool)execute;
  statLastAxesExecuteVel_[index]=execute;

  if (!(bool)execute){
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute((bool)execute);
    return mc_errorCode;
  }

  if(trigg){    
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute(0);
    if(mc_errorCode){
      return mc_errorCode;
    }    
    ecmcPLCTask::statAxes_[index]->getSeq()->setTargetVel(vel);
    ecmcPLCTask::statAxes_[index]->getTraj()->setDec(dec);
    ecmcPLCTask::statAxes_[index]->getTraj()->setAcc(acc);

    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setCommand(ECMC_CMD_MOVEVEL);    
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setCmdData(0);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute((bool)execute);
    if(mc_errorCode){
      return mc_errorCode;
    }
  }

  return 0.0;
}

// Note cannot use ecmcAxisBase::move* since the execution in plc with execute needs to be correct
inline double mc_home(double axIndex,double execute,double seqId, double velTwoardsCam,double velOffCam)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);  
  
  // check traj source, must be internal
  if(ecmcPLCTask::statAxes_[index]->getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    mc_errorCode=ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  int trigg=!statLastAxesExecuteHome_[index] && (bool)execute;
  statLastAxesExecuteHome_[index]=execute;
  
  if (!(bool)execute){
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute((bool)execute);
    return mc_errorCode;
  }

  if(trigg){    
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute(0);
    if(mc_errorCode){
      return mc_errorCode;
    }    
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->getSeq()->setHomeVelTwordsCam(velTwoardsCam);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->getSeq()->setHomeVelOffCam(velOffCam);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setCommand(ECMC_CMD_HOMING);    
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setCmdData((int)seqId);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute((bool)execute);
    if(mc_errorCode){
      return mc_errorCode;
    }
  }

  return 0.0;
}

inline double mc_halt(double axIndex,double execute)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);  
  
  // check traj source, must be internal
  if(ecmcPLCTask::statAxes_[index]->getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    mc_errorCode=ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  if (!(bool)execute){
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute((bool)execute);
    return mc_errorCode;
  }

  int trigg=!statLastAxesExecuteHalt_[index] && (bool)execute;
  statLastAxesExecuteHalt_[index]=execute;

  if(trigg){    
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute(0);
    if(mc_errorCode){
      return mc_errorCode;
    }
  }
  return 0.0;
}

inline double mc_power(double axIndex,double enable)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);  
  if(ecmcPLCTask::statAxes_[index]->getEnable() != (bool)enable){
    mc_errorCode=ecmcPLCTask::statAxes_[index]->setEnable((bool)enable);    
    return (double)mc_errorCode;
  }
  return 0;
}

inline double mc_reset(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);  
  ecmcPLCTask::statAxes_[index]->setReset(1);
  mc_errorCode = 0;
  return 0;
}

inline double mc_get_busy(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);  
  return (double) ecmcPLCTask::statAxes_[index]->getBusy();
}

inline double mc_get_homed(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  bool homed=0;
  mc_errorCode=ecmcPLCTask::statAxes_[index]->getAxisHomed(&homed);
  return (double) homed;
}

inline double mc_get_axis_err(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);  
  return (double) ecmcPLCTask::statAxes_[index]->getErrorID();
}

inline double mc_set_enable_motion_funcs(double axIndex, 
                                  double enablePos,
                                  double enableVel,
                                  double enableHome)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);  
  return (double) ecmcPLCTask::statAxes_[index]->setAllowMotionFunctions(
                    (int)enablePos,(int)enableVel,(int)enableHome);
}

inline double mc_get_err()
{
  return (double)mc_errorCode;
}

inline double mc_move_ext_pos(double axIndex,double execute, double vel, double acc, double dec)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);  

  // check traj source, must be internal
  if(ecmcPLCTask::statAxes_[index]->getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    mc_errorCode=ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  int trigg=!statLastAxesExecuteAbs_[index] && (bool)execute;
  statLastAxesExecuteAbs_[index]=execute;

  if (!(bool)execute){
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute((bool)execute);
    return mc_errorCode;
  }
  
  if(trigg){    
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute(0);
    if(mc_errorCode){
      return mc_errorCode;
    }
    double pos = ecmcPLCTask::statAxes_[index]->getExtSetPos();
    ecmcPLCTask::statAxes_[index]->getSeq()->setTargetPos(pos);
    ecmcPLCTask::statAxes_[index]->getSeq()->setTargetVel(vel);
    ecmcPLCTask::statAxes_[index]->getTraj()->setDec(dec);
    ecmcPLCTask::statAxes_[index]->getTraj()->setAcc(acc);

    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setCommand(ECMC_CMD_MOVEABS);    
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setCmdData(1);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute((bool)execute);
    if(mc_errorCode){
      return mc_errorCode;
    }
  }
  return 0.0;
}

#endif /* ecmcPLC_libMc_inc_ */
