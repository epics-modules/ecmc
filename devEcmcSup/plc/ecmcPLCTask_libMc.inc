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

#include "ecmcErrorsList.h"
#include "ecmcAxisGroup.h"

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

#define CHECK_PLC_AXISGRP_RETURN_IF_ERROR(grpIndex) {         \
  if(intGrpIndex >= ECMC_MAX_AXES || intGrpIndex < 0){        \
    mc_errorCode=0;                                           \
    LOGERR("ERROR: Group index out of range.\n");             \
    return (double)ERROR_PLC_AXIS_ID_OUT_OF_RANGE;}           \
    if(ecmcPLCTask::statAxisGrp_[intGrpIndex]==NULL){         \
      LOGERR("ERROR: Group object NULL\n");                   \
      return (double)ERROR_PLC_AXIS_OBJECT_NULL;              \
    }                                                         \
  }                                                           \

#define CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(grpIndex) {       \
 if(intGrpIndex >= ECMC_MAX_AXES || intGrpIndex < 0){         \
    mc_errorCode=ERROR_PLC_AXIS_ID_OUT_OF_RANGE;              \
    LOGERR("ERROR: Group index out of range.\n");             \
    return 0;                                                 \
  }                                                           \
  if(ecmcPLCTask::statAxisGrp_[intGrpIndex] == NULL){         \
    mc_errorCode=ERROR_PLC_AXIS_OBJECT_NULL;                  \
    LOGERR("ERROR: Group object NULL\n");                     \
    return 0;                                                 \
  }                                                           \
}                                                             \

#define CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(axIndex) { \
  if(!ecmcPLCTask::statAxes_[index]->getAllowCmdFromPLC()) {  \
    mc_errorCode=ERROR_MAIN_AXIS_COM_BLOCKED;                 \
    return ERROR_MAIN_AXIS_COM_BLOCKED;                       \
  }                                                           \
}                                                             \

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
                               "mc_home_pos(",
                               "mc_get_act_pos(",
                               "mc_set_prim_enc(",
                               "mc_get_prim_enc(",
                               "mc_set_axis_error(",
                               "mc_set_slaved_axis_in_error(",
                               "mc_grp_get_enable(",
                               "mc_grp_get_any_enable(",
                               "mc_grp_get_enabled(",
                               "mc_grp_get_any_enabled(",
                               "mc_grp_get_busy(",
                               "mc_grp_get_any_busy(",
                               "mc_grp_get_any_error_id(",
                               "mc_grp_set_enable(",
                               "mc_grp_set_traj_src(",
                               "mc_grp_set_enc_src(",
                               "mc_grp_reset_error(",
                               "mc_grp_set_error(",
                               "mc_grp_set_slaved_axis_in_error(",
                               "mc_grp_halt(",
                               "mc_grp_axis_in_grp(",
                               "mc_grp_size(",
                               "mc_grp_get_traj_src_ext(",
                               "mc_grp_get_any_traj_src_ext(",
                               "mc_grp_set_allow_src_change_when_enabled(",
                               "mc_grp_mr_set_sync(",
                               "mc_grp_mr_set_stop(",
                               "mc_grp_mr_set_cnen(",
                               "mc_mr_set_sync(",
                               "mc_mr_set_stop(",
                               "mc_mr_set_cnen(",
                               "mc_get_enc_ready(",
                               "mc_set_act_pos(",
                              };

ecmcAxisGroup   *ecmcPLCTask::statAxisGrp_[ECMC_MAX_AXES]={};
ecmcAxisBase    *ecmcPLCTask::statAxes_[ECMC_MAX_AXES]={};
ecmcDataStorage *ecmcPLCTask::statDs_[ECMC_MAX_DATA_STORAGE_OBJECTS]={};

static int statLastAxesExecuteAbs_[ECMC_MAX_AXES]={};
static int statLastAxesExecuteRel_[ECMC_MAX_AXES]={};
static int statLastAxesExecuteVel_[ECMC_MAX_AXES]={};
static int statLastAxesExecuteHalt_[ECMC_MAX_AXES]={};
static int statLastAxesExecuteHome_[ECMC_MAX_AXES]={};
static int mc_errorCode=0;
static int mc_cmd_count = 46;

// Note cannot use ecmcAxisBase::move* since the execution in plc with execute needs to be correct
inline double mc_move_abs(double axIndex,double execute,double pos, double vel, double acc,double dec)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);

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
    ecmcPLCTask::statAxes_[index]->getSeq()->setDec(dec);
    ecmcPLCTask::statAxes_[index]->getSeq()->setAcc(acc);

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
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);

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
    ecmcPLCTask::statAxes_[index]->getSeq()->setDec(dec);
    ecmcPLCTask::statAxes_[index]->getSeq()->setAcc(acc);

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
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);

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
    ecmcPLCTask::statAxes_[index]->getSeq()->setDec(dec);
    ecmcPLCTask::statAxes_[index]->getSeq()->setAcc(acc);

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
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);

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
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->getSeq()->setHomeVelTowardsCam(velTwoardsCam);
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

inline double mc_home_pos(double axIndex,double execute,double seqId, double velTwoardsCam,double velOffCam, double homePos)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);  

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
    ecmcPLCTask::statAxes_[index]->getSeq()->setHomePosition(homePos);
    
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->setExecute(0);
    if(mc_errorCode){
      return mc_errorCode;
    }    
    mc_errorCode=(double)ecmcPLCTask::statAxes_[index]->getSeq()->setHomeVelTowardsCam(velTwoardsCam);
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
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);

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
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);

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
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);

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

inline double mc_get_act_pos(double axIndex, double encIndex)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex - 1;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  int error = 0;
  // Index starts at 0.. messy.. thats why -1 above
  ecmcEncoder *enc=ecmcPLCTask::statAxes_[axIndexInt]->getEnc(encIndexInt,&error);
  if(error){
     mc_errorCode = error;
     return 0.0;
  }

  return enc->getActPos();
}

inline double mc_set_prim_enc(double axIndex, double encIndex)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);

  // Index starts at 1.. messy..
  int error = ecmcPLCTask::statAxes_[axIndexInt]->selectPrimaryEncoder(encIndexInt) ;
  if(error){
     mc_errorCode = error;
  }

  return mc_errorCode;
}

inline double mc_get_prim_enc(double axIndex)
{
  int axIndexInt=(int)axIndex;  

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  // Index starts at 1.. messy..
  return (double)ecmcPLCTask::statAxes_[axIndexInt]->getPrimaryEncoderIndex();
}

inline double mc_set_axis_error(double axIndex, double errorId)
{
  int axIndexInt=(int)axIndex;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  return (double)ecmcPLCTask::statAxes_[axIndexInt]->setErrorID((int)errorId);
}

inline double mc_set_slaved_axis_in_error(double axIndex)
{
  int axIndexInt=(int)axIndex;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  return (double)ecmcPLCTask::statAxes_[axIndexInt]->setSlavedAxisInError();
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
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);
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
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);

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
    ecmcPLCTask::statAxes_[index]->getSeq()->setDec(dec);
    ecmcPLCTask::statAxes_[index]->getSeq()->setAcc(acc);

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

// Group functions
inline double mc_grp_get_enable(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->getEnable();
}

inline double mc_grp_get_any_enable(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->getAnyEnable();
}

inline double mc_grp_get_enabled(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->getEnabled();
}

inline double mc_grp_get_any_enabled(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->getAnyEnabled();
}

inline double mc_grp_get_busy(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->getBusy();
}

inline double mc_grp_get_any_busy(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->getAnyBusy();
}

inline double mc_grp_get_any_error_id(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->getAnyErrorId();
}

inline double mc_grp_set_enable(double grpIndex, double enable){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->setEnable((int)enable);
}

inline double mc_grp_set_traj_src(double grpIndex, double source){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->setTrajSrc((dataSource)(int)source);
}

inline double mc_grp_set_enc_src(double grpIndex, double source){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->setEncSrc((dataSource)(int)source);
}

inline double mc_grp_reset_error(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  ecmcPLCTask::statAxisGrp_[intGrpIndex]->setErrorReset();
  return 0;
}

inline double mc_grp_set_error(double grpIndex, double errorId){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  ecmcPLCTask::statAxisGrp_[intGrpIndex]->setError((int)errorId);
  return 0;
}

inline double mc_grp_set_slaved_axis_in_error(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  ecmcPLCTask::statAxisGrp_[intGrpIndex]->setSlavedAxisInError();
  return 0;
}

inline double mc_grp_halt(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  ecmcPLCTask::statAxisGrp_[intGrpIndex]->halt();
  return 0;
}

inline double mc_grp_axis_in_grp(double grpIndex, double axisId){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->inGroup(axisId);
}

inline double mc_grp_size(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->size();
}

inline double mc_grp_get_traj_src_ext(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->getTrajSrcExt();
}

inline double mc_grp_get_any_traj_src_ext(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  return (double)ecmcPLCTask::statAxisGrp_[intGrpIndex]->getTrajSrcAnyExt();
}

inline double mc_grp_set_allow_src_change_when_enabled(double grpIndex, double allow){
  int intGrpIndex=(int)grpIndex;
  int allow_int=(int)allow;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  ecmcPLCTask::statAxisGrp_[intGrpIndex]->setAllowSrcChangeWhenEnabled(allow_int);
  return 0;
}

inline double mc_grp_mr_set_sync(double grpIndex, double sync){
  int intGrpIndex=(int)grpIndex;
  int intSync=(int)sync;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  ecmcPLCTask::statAxisGrp_[intGrpIndex]->setMRSync(intSync);
  return 0;
}

inline double mc_grp_mr_set_stop(double grpIndex, double stop){
  int intGrpIndex=(int)grpIndex;
  int intStop=(int)stop;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  ecmcPLCTask::statAxisGrp_[intGrpIndex]->setMRStop(intStop);
  return 0;
}

inline double mc_grp_mr_set_cnen(double grpIndex, double enable){
  int intGrpIndex=(int)grpIndex;
  int intEnable=(int)enable;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  ecmcPLCTask::statAxisGrp_[intGrpIndex]->setMRCnen(intEnable);
  return 0;
}

inline double mc_mr_set_sync(double axIndex, double sync){
  int index=(int)axIndex;
  int intSync=(int)sync;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);
  ecmcPLCTask::statAxes_[index]->setMRSync(intSync);
  return 0;
}

inline double mc_mr_set_stop(double axIndex, double stop){
  int index=(int)axIndex;
  int intStop=(int)stop;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);
  ecmcPLCTask::statAxes_[index]->setMRStop(intStop);
  return 0;
}

inline double mc_mr_set_cnen(double axIndex, double enable){
  int index=(int)axIndex;
  int intEnable=(int)enable;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);
  ecmcPLCTask::statAxes_[index]->setMRCnen(intEnable);
  return 0;
}

inline double mc_set_act_pos(double axIndex, double encIndex, double actpos)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex - 1;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  int error = 0;
  // getEnc() index starts at 0.. messy.. thats why -1 above
  ecmcEncoder *enc=ecmcPLCTask::statAxes_[axIndexInt]->getEnc(encIndexInt,&error);
  if(error){
     mc_errorCode = error;
     return error;
  }

  enc->setActPos(actpos);
  
  return 0.0;
}

inline double mc_get_enc_ready(double axIndex, double encIndex)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex - 1;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  // getEnc() index starts at 0.. messy.. thats why -1 above
  int error = 0;
  ecmcEncoder *enc=ecmcPLCTask::statAxes_[axIndexInt]->getEnc(encIndexInt,&error);
  if(error){
     mc_errorCode = error;
     return 0.0;
  }

  return (double) enc->hwReady();
}

#endif /* ecmcPLC_libMc_inc_ */
