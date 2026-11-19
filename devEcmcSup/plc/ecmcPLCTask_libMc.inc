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
#include "ecmcMotionSequence.h"
#include "ecmcGlobalsExtern.h"

#define CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndex) {             \
  if(axIndex>=ECMC_MAX_AXES || axIndex<0){                    \
    mc_errorCode=0;                                           \
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Axis index out of range.\n", __FILE__, __FUNCTION__, __LINE__);              \
    return (double)ERROR_PLC_AXIS_ID_OUT_OF_RANGE;}           \
    if(axes[axIndex]==NULL){                                  \
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Axis object is NULL.\n", __FILE__, __FUNCTION__, __LINE__);                    \
      return (double)ERROR_PLC_AXIS_OBJECT_NULL;              \
    }                                                         \
  }                                                           \

#define CHECK_PLC_AXISGRP_RETURN_IF_ERROR(grpIndex) {         \
  if(intGrpIndex >= ECMC_MAX_AXES || intGrpIndex < 0){        \
    mc_errorCode=0;                                           \
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Group index out of range.\n", __FILE__, __FUNCTION__, __LINE__);             \
    return (double)ERROR_PLC_AXIS_ID_OUT_OF_RANGE;}           \
    if(axisGroups[intGrpIndex]==NULL){                        \
      ecmcRtLoggerLogError("%s/%s:%d: ERROR: Group object is NULL.\n", __FILE__, __FUNCTION__, __LINE__);                   \
      return (double)ERROR_PLC_AXIS_OBJECT_NULL;              \
    }                                                         \
  }                                                           \

#define CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(grpIndex) {       \
 if(intGrpIndex >= ECMC_MAX_AXES || intGrpIndex < 0){         \
    mc_errorCode=ERROR_PLC_AXIS_ID_OUT_OF_RANGE;              \
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Group index out of range.\n", __FILE__, __FUNCTION__, __LINE__);             \
    return 0;                                                 \
  }                                                           \
  if(axisGroups[intGrpIndex] == NULL){                        \
    mc_errorCode=ERROR_PLC_AXIS_OBJECT_NULL;                  \
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Group object is NULL.\n", __FILE__, __FUNCTION__, __LINE__);                     \
    return 0;                                                 \
  }                                                           \
}                                                             \

#define CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(axIndex) { \
  if(axes[index]->getBlockCom()) {                            \
    mc_errorCode=axes[index]->setExternalCommandBlockedError(); \
    return ERROR_MAIN_AXIS_EXTERNAL_COM_DISABLED;             \
  }                                                           \
  if(!axes[index]->getAllowCmdFromPLC()) {  \
    mc_errorCode=ERROR_MAIN_AXIS_COM_BLOCKED;                 \
    return ERROR_MAIN_AXIS_COM_BLOCKED;                       \
  }                                                           \
}                                                             \

#define CHECK_PLC_SEQ_RETURN_IF_ERROR(seqIndex) {             \
  if(seqIndex>=ECMC_MAX_MOTION_SEQUENCES || seqIndex<0){      \
    mc_errorCode=ERROR_MAIN_DATA_STORAGE_INDEX_OUT_OF_RANGE;  \
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Sequence index out of range.\n", __FILE__, __FUNCTION__, __LINE__); \
    return (double)mc_errorCode;}                             \
  if(motionSeqs[seqIndex]==NULL){                             \
    mc_errorCode=ERROR_MAIN_SEQUENCE_OBJECT_NULL;             \
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Sequence object is NULL.\n", __FILE__, __FUNCTION__, __LINE__); \
    return (double)mc_errorCode;                              \
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
                               "mc_set_ignore_disabled_motion_cmds(",
                               "mc_set_enable_motion_funcs(",
                               "mc_move_ext_pos(",
                               "mc_home_pos(",
                               "mc_get_act_pos(",
                               "mc_touch_probe_arm(",
                               "mc_touch_probe_get_valid(",
                               "mc_get_touch_probe_pos(",
                               "mc_get_touch_probe_sequence(",
                               "mc_touch_probe_get_time(",
                               "mc_touch_probe_get_timestamp_raw(",
                               "mc_touch_probe_get_timestamp_bits(",
                               "mc_pos_compare_arm(",
                               "mc_pos_compare_cancel(",
                               "mc_pos_compare_get_state(",
                               "mc_pos_compare_get_sequence(",
                               "mc_pos_compare_get_scheduled_time(",
                               "mc_pos_compare_get_lead_time(",
                               "mc_pos_compare_get_sample_age(",
                               "mc_pos_compare_get_position(",
                               "mc_pos_compare_get_velocity(",
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
                               "mc_set_enc_homed(",
                               "mc_get_enc_homed(",
                               "mc_set_traj_vel(",
                               "mc_set_traj_acc(",
                               "mc_set_traj_dec(",
                               "mc_set_traj_jerk(",
                               "mc_grp_set_ignore_mr_status_check_at_disable(",
                               "mc_grp_get_any_at_fwd_limit(",
                               "mc_grp_get_any_at_bwd_limit(",
                               "mc_grp_get_any_at_limit(",
                               "mc_grp_set_slaved_axis_ilocked(",
                               "mc_grp_set_autoenable_enable(,"
                               "mc_grp_set_autodisable_enable(,"
                               "mc_set_autoenable_enable(,"
                               "mc_set_autodisable_enable(,"
                               "mc_grp_set_ctrl_within_db(,"
                               "mc_grp_get_ctrl_within_db(,"
                               "mc_grp_get_any_ilocked(",
                               "mc_get_hw_ready(",
                               "mc_ctrl_i_reset(",
                               "seq_arm(",
                               "seq_set_step(",
                               "seq_get_state(",
                               "seq_get_valid(",
                               "seq_get_armed(",
                               "seq_get_running(",
                               "seq_get_compile_busy(",
                               "seq_get_step(",
                               "seq_get_action(",
                               "seq_get_error(",
                               "seq_get_step_count(",
                               "seq_get_elapsed_ms(",
                              };

extern ecmcAxisGroup   *axisGroups[ECMC_MAX_AXES];
extern ecmcAxisBase    *axes[ECMC_MAX_AXES];

static int statLastAxesExecuteAbs_[ECMC_MAX_AXES]={};
static int statLastAxesExecuteRel_[ECMC_MAX_AXES]={};
static int statLastAxesExecuteVel_[ECMC_MAX_AXES]={};
static int statLastAxesExecuteHalt_[ECMC_MAX_AXES]={};
static int statLastAxesExecuteHome_[ECMC_MAX_AXES]={};
static int statLastSeqExecuteArm_[ECMC_MAX_MOTION_SEQUENCES]={};
static int statLastSeqExecuteSetStep_[ECMC_MAX_MOTION_SEQUENCES]={};
static int mc_errorCode = 0;
static int mc_cmd_count = 95;

// Note cannot use ecmcAxisBase::move* since the execution in plc with execute needs to be correct
inline double mc_move_abs(double axIndex,double execute,double pos, double vel, double acc,double dec)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);

  // check traj source, must be internal
  if(axes[index]->getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    mc_errorCode=ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  int trigg=!statLastAxesExecuteAbs_[index] && (bool)execute;
  statLastAxesExecuteAbs_[index]=execute;
  if (axes[index]->motionCommandDisabled(ECMC_CMD_MOVEABS)) {
    if (trigg) axes[index]->ignoreMotionCommand(ECMC_CMD_MOVEABS);
    return 0;
  }

  if (!(bool)execute){
    mc_errorCode=(double)axes[index]->setExecute((bool)execute);
    return mc_errorCode;
  }
  
  if(trigg){    
    mc_errorCode=(double)axes[index]->setExecute(0);
    if(mc_errorCode){
      return mc_errorCode;
    }
    axes[index]->setTargetVel(vel);
    axes[index]->setDec(dec);
    axes[index]->setAcc(acc);

    mc_errorCode=(double)axes[index]->setCommand(ECMC_CMD_MOVEABS);    
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axes[index]->setCmdData(0);
    if(mc_errorCode){
      return mc_errorCode;
    }
    // The sequencer interprets pos using the active command type.
    axes[index]->setTargetPosAndCmd(pos);
    mc_errorCode=(double)axes[index]->setExecute((bool)execute);
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
  if(axes[index]->getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    mc_errorCode=ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  int trigg=!statLastAxesExecuteRel_[index] && (bool)execute;
  statLastAxesExecuteRel_[index]=execute;
  if (axes[index]->motionCommandDisabled(ECMC_CMD_MOVEREL)) {
    if (trigg) axes[index]->ignoreMotionCommand(ECMC_CMD_MOVEREL);
    return 0;
  }
  
  if (!(bool)execute){
    mc_errorCode=(double)axes[index]->setExecute((bool)execute);
    return mc_errorCode;
  }

  if(trigg){    
    mc_errorCode=(double)axes[index]->setExecute(0);
    if(mc_errorCode){
      return mc_errorCode;
    }
    axes[index]->setTargetVel(vel);
    axes[index]->setDec(dec);
    axes[index]->setAcc(acc);

    mc_errorCode=(double)axes[index]->setCommand(ECMC_CMD_MOVEREL);    
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axes[index]->setCmdData(0);
    if(mc_errorCode){
      return mc_errorCode;
    }
    // The sequencer interprets pos using the active command type.
    axes[index]->setTargetPosAndCmd(pos);
    mc_errorCode=(double)axes[index]->setExecute((bool)execute);
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
  if(axes[index]->getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    mc_errorCode=ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  int trigg=!statLastAxesExecuteVel_[index] && (bool)execute;
  statLastAxesExecuteVel_[index]=execute;
  if (axes[index]->motionCommandDisabled(ECMC_CMD_MOVEVEL)) {
    if (trigg) axes[index]->ignoreMotionCommand(ECMC_CMD_MOVEVEL);
    return 0;
  }

  if (!(bool)execute){
    mc_errorCode=(double)axes[index]->setExecute((bool)execute);
    return mc_errorCode;
  }

  if(trigg){    
    mc_errorCode=(double)axes[index]->setExecute(0);
    if(mc_errorCode){
      return mc_errorCode;
    }    
    axes[index]->setTargetVel(vel);
    axes[index]->setDec(dec);
    axes[index]->setAcc(acc);

    mc_errorCode=(double)axes[index]->setCommand(ECMC_CMD_MOVEVEL);    
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axes[index]->setCmdData(0);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axes[index]->setExecute((bool)execute);
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
  if(axes[index]->getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    mc_errorCode=ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  int trigg=!statLastAxesExecuteHome_[index] && (bool)execute;
  statLastAxesExecuteHome_[index]=execute;
  if (axes[index]->motionCommandDisabled(ECMC_CMD_HOMING)) {
    if (trigg) axes[index]->ignoreMotionCommand(ECMC_CMD_HOMING);
    return 0;
  }
  
  if (!(bool)execute){
    mc_errorCode=(double)axes[index]->setExecute((bool)execute);
    return mc_errorCode;
  }

  if(trigg){    
    mc_errorCode=(double)axes[index]->setExecute(0);
    if(mc_errorCode){
      return mc_errorCode;
    }    
    mc_errorCode=(double)axes[index]->getSeq()->setHomeVelTowardsCam(velTwoardsCam);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axes[index]->getSeq()->setHomeVelOffCam(velOffCam);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axes[index]->setCommand(ECMC_CMD_HOMING);    
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axes[index]->setCmdData((int)seqId);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axes[index]->setExecute((bool)execute);
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
  if(axes[index]->getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    mc_errorCode=ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  int trigg=!statLastAxesExecuteHome_[index] && (bool)execute;
  statLastAxesExecuteHome_[index]=execute;
  if (axes[index]->motionCommandDisabled(ECMC_CMD_HOMING)) {
    if (trigg) axes[index]->ignoreMotionCommand(ECMC_CMD_HOMING);
    return 0;
  }
  
  if (!(bool)execute){
    mc_errorCode=(double)axes[index]->setExecute((bool)execute);
    return mc_errorCode;
  }

  if(trigg){    
    axes[index]->getSeq()->setHomePosition(homePos);
    
    mc_errorCode=(double)axes[index]->setExecute(0);
    if(mc_errorCode){
      return mc_errorCode;
    }    
    mc_errorCode=(double)axes[index]->getSeq()->setHomeVelTowardsCam(velTwoardsCam);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axes[index]->getSeq()->setHomeVelOffCam(velOffCam);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axes[index]->setCommand(ECMC_CMD_HOMING);    
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axes[index]->setCmdData((int)seqId);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axes[index]->setExecute((bool)execute);
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
  if(axes[index]->getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    mc_errorCode=ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  if (!(bool)execute){
    mc_errorCode=(double)axes[index]->setExecute((bool)execute);
    return mc_errorCode;
  }

  int trigg=!statLastAxesExecuteHalt_[index] && (bool)execute;
  statLastAxesExecuteHalt_[index]=execute;

  if(trigg){    
    mc_errorCode=(double)axes[index]->setExecute(0);
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
  if((bool)enable) {
    CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);
  }

  if(axes[index]->getEnable() != (bool)enable){
    mc_errorCode=axes[index]->setEnable((bool)enable);    
    return (double)mc_errorCode;
  }
  return 0;
}

inline double mc_reset(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);

  axes[index]->setReset(1);
  mc_errorCode = 0;
  return 0;
}

inline double mc_ctrl_i_reset(double axIndex, double reset)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);

  if ((bool)reset) {
    ecmcPIDController *cntrl = axes[index]->getCntrl();
    if (!cntrl) {
      mc_errorCode = ERROR_AXIS_CNTRL_OBJECT_NULL;
      return (double)mc_errorCode;
    }
    cntrl->resetI();
  }

  mc_errorCode = 0;
  return 0;
}

inline double mc_get_busy(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  return (double) axes[index]->getBusy();
}

inline double mc_get_act_pos(double axIndex, double encIndex)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex - 1;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  auto * const axis = axes[axIndexInt];
  int error = 0;
  // Index starts at 0.. messy.. thats why -1 above
  ecmcEncoder *enc=axis->getEnc(encIndexInt,&error);
  if(error){
     mc_errorCode = error;
     return 0.0;
  }

  return enc->getActPos();
}

inline double mc_get_touch_probe_pos(double axIndex, double encIndex)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex - 1;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  int error = 0;
  ecmcEncoder *enc=axes[axIndexInt]->getEnc(encIndexInt,&error);
  if(error){
    mc_errorCode = error;
    return 0.0;
  }

  mc_errorCode = 0;
  return enc->getTouchProbePosEng();
}

inline double mc_touch_probe_arm(double axIndex, double encIndex, double arm)
{
  int index=(int)axIndex;
  int encIndexInt=(int)encIndex - 1;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);
  if (encIndexInt < 0 || encIndexInt >= ECMC_MAX_ENCODERS) {
    mc_errorCode = ERROR_AXIS_ENC_COUNT_OUT_OF_RANGE;
    return (double)mc_errorCode;
  }

  mc_errorCode =
    axes[index]->setTouchProbeArm(encIndexInt, (bool)arm);
  return (double)mc_errorCode;
}

inline double mc_touch_probe_get_valid(double axIndex, double encIndex)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex - 1;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  if (encIndexInt < 0 || encIndexInt >= ECMC_MAX_ENCODERS) {
    mc_errorCode = ERROR_AXIS_ENC_COUNT_OUT_OF_RANGE;
    return 0.0;
  }

  int error = 0;
  const uint64_t nearbyDcTimeNs = ec ? ec->getLastSendTimeNs() : 0;
  const ecmcEcTimedValue<double> result =
    axes[axIndexInt]->getTouchProbeResult(encIndexInt,
                                          nearbyDcTimeNs,
                                          &error);
  if(error){
    mc_errorCode = error;
    return 0.0;
  }

  mc_errorCode = 0;
  return result.valid ? 1.0 : 0.0;
}

inline double mc_get_touch_probe_sequence(double axIndex, double encIndex)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex - 1;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  int error = 0;
  ecmcEncoder *enc=axes[axIndexInt]->getEnc(encIndexInt,&error);
  if(error){
    mc_errorCode = error;
    return 0.0;
  }

  mc_errorCode = 0;
  return (double)enc->getTouchProbeSequence();
}

inline double mc_touch_probe_get_time(double axIndex, double encIndex)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex - 1;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  if (encIndexInt < 0 || encIndexInt >= ECMC_MAX_ENCODERS) {
    mc_errorCode = ERROR_AXIS_ENC_COUNT_OUT_OF_RANGE;
    return 0.0;
  }

  int error = 0;
  const uint64_t nearbyDcTimeNs = ec ? ec->getLastSendTimeNs() : 0;
  const ecmcEcTimedValue<double> result =
    axes[axIndexInt]->getTouchProbeResult(encIndexInt,
                                          nearbyDcTimeNs,
                                          &error);
  if(error){
    mc_errorCode = error;
    return 0.0;
  }

  mc_errorCode = 0;
  return (double)result.eventTimeNs;
}

inline double mc_touch_probe_get_timestamp_raw(double axIndex,
                                               double encIndex)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex - 1;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  int error = 0;
  ecmcEncoder *enc=axes[axIndexInt]->getEnc(encIndexInt,&error);
  if(error){
    mc_errorCode = error;
    return 0.0;
  }

  mc_errorCode = 0;
  return (double)enc->getTouchProbeTimestampRaw();
}

inline double mc_touch_probe_get_timestamp_bits(double axIndex,
                                                double encIndex)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex - 1;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  int error = 0;
  ecmcEncoder *enc=axes[axIndexInt]->getEnc(encIndexInt,&error);
  if(error){
    mc_errorCode = error;
    return 0.0;
  }

  mc_errorCode = 0;
  return (double)enc->getTouchProbeTimestampBits();
}

inline double mc_pos_compare_arm(double axIndex,
                                 double target,
                                 double direction,
                                 double outputValue)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);

  mc_errorCode = axes[index]->armPositionCompare(target,
                                                 (int)direction,
                                                 (uint64_t)outputValue);
  return (double)mc_errorCode;
}

inline double mc_pos_compare_cancel(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);

  mc_errorCode = axes[index]->cancelPositionCompare();
  return (double)mc_errorCode;
}

inline ecmcPositionCompareStatus mc_pos_compare_status(int index)
{
  return axes[index]->getPositionCompareStatus();
}

inline double mc_pos_compare_get_state(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  mc_errorCode = 0;
  return (double)mc_pos_compare_status(index).state;
}

inline double mc_pos_compare_get_sequence(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  mc_errorCode = 0;
  return (double)mc_pos_compare_status(index).sequence;
}

inline double mc_pos_compare_get_scheduled_time(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  mc_errorCode = 0;
  return (double)mc_pos_compare_status(index).scheduledTimeNs;
}

inline double mc_pos_compare_get_lead_time(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  mc_errorCode = 0;
  return (double)mc_pos_compare_status(index).leadTimeNs;
}

inline double mc_pos_compare_get_sample_age(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  mc_errorCode = 0;
  return (double)mc_pos_compare_status(index).sampleAgeNs;
}

inline double mc_pos_compare_get_position(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  mc_errorCode = 0;
  return mc_pos_compare_status(index).position;
}

inline double mc_pos_compare_get_velocity(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  mc_errorCode = 0;
  return mc_pos_compare_status(index).velocity;
}

inline double mc_set_prim_enc(double axIndex, double encIndex)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);

  // Index starts at 1.. messy..
  int error = axes[axIndexInt]->selectPrimaryEncoder(encIndexInt) ;
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
  return (double)axes[axIndexInt]->getPrimaryEncoderIndex();
}

inline double mc_set_axis_error(double axIndex, double errorId)
{
  int axIndexInt=(int)axIndex;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  return (double)axes[axIndexInt]->setErrorID((int)errorId);
}

inline double mc_set_slaved_axis_in_error(double axIndex)
{
  int axIndexInt=(int)axIndex;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  return (double)axes[axIndexInt]->setSlavedAxisInError();
}

inline double mc_get_homed(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  bool homed=0;
  mc_errorCode=axes[index]->getAxisHomed(&homed);
  return (double) homed;
}

inline double mc_set_enc_homed(double axIndex, double encIndex, double homed)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex -1;
  // getEnc() index starts at 0.. messy.. thats why -1 above
  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  auto * const axis = axes[axIndexInt];

  int error = 0;
  ecmcEncoder *enc=axis->getEnc(encIndexInt,&error);
  if(error){
     mc_errorCode = error;
     return error;
  }

  enc->setHomed(homed > 0);

  return 0.0;
}

inline double mc_get_enc_homed(double axIndex, double encIndex)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex -1;
  // getEnc() index starts at 0.. messy.. thats why -1 above
  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  auto * const axis = axes[axIndexInt];

  int error = 0;
  ecmcEncoder *enc=axis->getEnc(encIndexInt,&error);
  if(error){
     mc_errorCode = error;
     return -error;
  }

  return (double)enc->getHomed();
}

inline double mc_get_hw_ready(double axIndex)
{
  int axIndexInt=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);

  return (double)axes[axIndexInt]->getHwReady();
}

inline double seq_arm(double seqIndex, double execute)
{
  int index=(int)seqIndex;
  int executeInt=(int)execute;
  CHECK_PLC_SEQ_RETURN_IF_ERROR(index);

  int trigg = !statLastSeqExecuteArm_[index] && executeInt;
  statLastSeqExecuteArm_[index] = executeInt;

  if(!executeInt || !trigg) {
    return 0.0;
  }

  mc_errorCode = requestMotionSeqArmRTSafe(index);
  return (double)mc_errorCode;
}

inline double seq_set_step(double seqIndex, double execute, double stepId)
{
  int index=(int)seqIndex;
  int executeInt=(int)execute;
  CHECK_PLC_SEQ_RETURN_IF_ERROR(index);

  int trigg = !statLastSeqExecuteSetStep_[index] && executeInt;
  statLastSeqExecuteSetStep_[index] = executeInt;

  if(!executeInt || !trigg) {
    return 0.0;
  }

  mc_errorCode = requestMotionSeqCurrentStepRTSafe(index, (int)stepId);
  return (double)mc_errorCode;
}

inline double seq_get_state(double seqIndex)
{
  int index=(int)seqIndex;
  CHECK_PLC_SEQ_RETURN_IF_ERROR(index);
  return (double)getMotionSeqState(index);
}

inline double seq_get_valid(double seqIndex)
{
  int index=(int)seqIndex;
  CHECK_PLC_SEQ_RETURN_IF_ERROR(index);
  return (double)getMotionSeqValid(index);
}

inline double seq_get_armed(double seqIndex)
{
  int index=(int)seqIndex;
  CHECK_PLC_SEQ_RETURN_IF_ERROR(index);
  return (double)getMotionSeqArmed(index);
}

inline double seq_get_running(double seqIndex)
{
  int index=(int)seqIndex;
  CHECK_PLC_SEQ_RETURN_IF_ERROR(index);
  return (double)getMotionSeqRunning(index);
}

inline double seq_get_compile_busy(double seqIndex)
{
  int index=(int)seqIndex;
  CHECK_PLC_SEQ_RETURN_IF_ERROR(index);
  return (double)getMotionSeqCompileBusy(index);
}

inline double seq_get_step(double seqIndex)
{
  int index=(int)seqIndex;
  CHECK_PLC_SEQ_RETURN_IF_ERROR(index);
  return (double)getMotionSeqStepId(index);
}

inline double seq_get_action(double seqIndex)
{
  int index=(int)seqIndex;
  CHECK_PLC_SEQ_RETURN_IF_ERROR(index);
  return (double)getMotionSeqAction(index);
}

inline double seq_get_error(double seqIndex)
{
  int index=(int)seqIndex;
  CHECK_PLC_SEQ_RETURN_IF_ERROR(index);
  return (double)getMotionSeqErrorId(index);
}

inline double seq_get_step_count(double seqIndex)
{
  int index=(int)seqIndex;
  CHECK_PLC_SEQ_RETURN_IF_ERROR(index);
  return (double)getMotionSeqStepCount(index);
}

inline double seq_get_elapsed_ms(double seqIndex)
{
  int index=(int)seqIndex;
  CHECK_PLC_SEQ_RETURN_IF_ERROR(index);
  return getMotionSeqElapsedMs(index);
}

inline double mc_get_axis_err(double axIndex)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  return (double) axes[index]->getErrorID();
}

inline double mc_set_ignore_disabled_motion_cmds(double axIndex, double enable)
{
  int index = (int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);
  return axes[index]->setIgnoreDisabledMotionCommands((int)enable);
}

inline double mc_set_enable_motion_funcs(double axIndex, 
                                         double enablePos,
                                         double enableVel,
                                         double enableHome)
{
  int index=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);
  return (double) axes[index]->setAllowMotionFunctions(
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
  auto * const axis = axes[index];

  // check traj source, must be internal
  if(axis->getTrajDataSourceType() != ECMC_DATA_SOURCE_INTERNAL) {
    mc_errorCode=ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
    return ERROR_MAIN_TRAJ_SOURCE_NOT_INTERNAL;
  }

  int trigg=!statLastAxesExecuteAbs_[index] && (bool)execute;
  statLastAxesExecuteAbs_[index]=execute;
  if (axes[index]->motionCommandDisabled(ECMC_CMD_MOVEABS)) {
    if (trigg) axes[index]->ignoreMotionCommand(ECMC_CMD_MOVEABS);
    return 0;
  }

  if (!(bool)execute){
    mc_errorCode=(double)axis->setExecute((bool)execute);
    return mc_errorCode;
  }
  
  if(trigg){    
    mc_errorCode=(double)axis->setExecute(0);
    if(mc_errorCode){
      return mc_errorCode;
    }
    double pos = axis->getExtSetPos();
    axis->setTargetPosAndCmd(pos);
    axis->setTargetVel(vel);
    axis->setDec(dec);
    axis->setAcc(acc);

    mc_errorCode=(double)axis->setCommand(ECMC_CMD_MOVEABS);    
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axis->setCmdData(1);
    if(mc_errorCode){
      return mc_errorCode;
    }
    mc_errorCode=(double)axis->setExecute((bool)execute);
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
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->getEnable();
}

inline double mc_grp_get_any_enable(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->getAnyEnable();
}

inline double mc_grp_get_enabled(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->getEnabled();
}

inline double mc_grp_get_any_enabled(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->getAnyEnabled();
}

inline double mc_grp_get_busy(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->getBusy();
}

inline double mc_grp_get_any_busy(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->getAnyBusy();
}

inline double mc_grp_get_any_error_id(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->getAnyErrorId();
}

inline double mc_grp_set_enable(double grpIndex, double enable){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  if ((int)enable) {
    for (int axisIndex = 0; axisIndex < ECMC_MAX_AXES; ++axisIndex) {
      if (grp->inGroup(axisIndex) &&
          axes[axisIndex] &&
          axes[axisIndex]->getBlockCom()) {
        mc_errorCode = axes[axisIndex]->setExternalCommandBlockedError();
        return ERROR_MAIN_AXIS_EXTERNAL_COM_DISABLED;
      }
    }
  }
  return (double)grp->setEnable((int)enable);
}

inline double mc_grp_set_traj_src(double grpIndex, double source){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->setTrajSrc((dataSource)(int)source);
}

inline double mc_grp_set_enc_src(double grpIndex, double source){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->setEncSrc((dataSource)(int)source);
}

inline double mc_grp_reset_error(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  grp->setErrorReset();
  return 0;
}

inline double mc_grp_set_error(double grpIndex, double errorId){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  grp->setError((int)errorId);
  return 0;
}

inline double mc_grp_set_slaved_axis_in_error(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  grp->setSlavedAxisInError();
  return 0;
}

inline double mc_grp_halt(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  grp->halt();
  return 0;
}

inline double mc_grp_axis_in_grp(double grpIndex, double axisId){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->inGroup(axisId);
}

inline double mc_grp_size(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->size();
}

inline double mc_grp_get_traj_src_ext(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->getTrajSrcExt();
}

inline double mc_grp_get_any_traj_src_ext(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->getTrajSrcAnyExt();
}

inline double mc_grp_set_allow_src_change_when_enabled(double grpIndex, double allow){
  int intGrpIndex=(int)grpIndex;
  int allow_int=(int)allow;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  grp->setAllowSrcChangeWhenEnabled(allow_int);
  return 0;
}

inline double mc_grp_mr_set_sync(double grpIndex, double sync){
  int intGrpIndex=(int)grpIndex;
  int intSync=(int)sync;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  grp->setMRSync(intSync);
  return 0;
}

inline double mc_grp_mr_set_stop(double grpIndex, double stop){
  int intGrpIndex=(int)grpIndex;
  int intStop=(int)stop;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  grp->setMRStop(intStop);
  return 0;
}

inline double mc_grp_mr_set_cnen(double grpIndex, double enable){
  int intGrpIndex=(int)grpIndex;
  int intEnable=(int)enable;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  grp->setMRCnen(intEnable);
  return 0;
}

inline double mc_grp_set_autoenable_enable(double grpIndex, double enable){
  int intGrpIndex=(int)grpIndex;
  int intEnable=(int)enable;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  grp->setEnableAutoEnable(intEnable);
  return 0;
}

inline double mc_grp_set_autodisable_enable(double grpIndex, double enable){
  int intGrpIndex=(int)grpIndex;
  int intEnable=(int)enable;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  grp->setEnableAutoDisable(intEnable);
  return 0;
}

inline double mc_grp_set_ctrl_within_db(double grpIndex, double within){
  int intGrpIndex=(int)grpIndex;
  int intWithin=(int)within;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  grp->setAxisIsWithinCtrlDBExtTraj(intWithin);
  return 0;
}

inline double mc_set_autoenable_enable(double axIndex, double enable)
{
  int axIndexInt=(int)axIndex;
  int intEnable=(int)enable;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  
  axes[axIndexInt]->setEnableAutoEnable(intEnable);
  
  return 0.0;
}

inline double mc_set_autodisable_enable(double axIndex, double enable)
{
  int axIndexInt=(int)axIndex;
  int intEnable=(int)enable;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  
  axes[axIndexInt]->setEnableAutoDisable(intEnable);
  
  return 0.0;
}

inline double mc_mr_set_sync(double axIndex, double sync){
  int index=(int)axIndex;
  int intSync=(int)sync;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);
  axes[index]->setMRSync(intSync);
  return 0;
}

inline double mc_mr_set_stop(double axIndex, double stop){
  int index=(int)axIndex;
  int intStop=(int)stop;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);
  axes[index]->setMRStop(intStop);
  return 0;
}

inline double mc_mr_set_cnen(double axIndex, double enable){
  int index=(int)axIndex;
  int intEnable=(int)enable;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(index);
  CHECK_PLC_AXIS_COM_ALLOWED_RETURN_IF_ERROR(index);
  axes[index]->setMRCnen(intEnable);
  return 0;
}

inline double mc_set_act_pos(double axIndex, double encIndex, double actpos)
{
  int axIndexInt=(int)axIndex;
  int encIndexInt=(int)encIndex - 1;

  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  auto * const axis = axes[axIndexInt];
  int error = 0;
  // getEnc() index starts at 0.. messy.. thats why -1 above
  ecmcEncoder *enc=axis->getEnc(encIndexInt,&error);
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
  auto * const axis = axes[axIndexInt];
  // getEnc() index starts at 0.. messy.. thats why -1 above
  int error = 0;
  ecmcEncoder *enc=axis->getEnc(encIndexInt,&error);
  if(error){
     mc_errorCode = error;
     return 0.0;
  }

  return (double) enc->hwReady();
}

inline double mc_set_traj_vel(double axIndex, double vel)
{
  int axIndexInt=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  auto * const axis = axes[axIndexInt];
  axis->getTraj()->setTargetVel(vel);
  return 0.0;
}

inline double mc_set_traj_acc(double axIndex, double acc)
{
  int axIndexInt=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  auto * const axis = axes[axIndexInt];
  axis->getTraj()->setAcc(acc);
  return 0.0;
}

inline double mc_set_traj_dec(double axIndex, double dec)
{
  int axIndexInt=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  auto * const axis = axes[axIndexInt];
  axis->getTraj()->setDec(dec);
  return 0.0;
}

inline double mc_set_traj_jerk(double axIndex, double jerk)
{
  int axIndexInt=(int)axIndex;
  CHECK_PLC_AXIS_RETURN_IF_ERROR(axIndexInt);
  auto * const axis = axes[axIndexInt];
  axis->getTraj()->setJerk(jerk);
  return 0.0;
}

inline double mc_grp_set_ignore_mr_status_check_at_disable(double grpIndex, double ignore){
  int intGrpIndex=(int)grpIndex;
  int ignore_int=(int)ignore;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  grp->setMRIgnoreDisableStatusCheck(ignore_int);
  return 0;
}

inline double mc_grp_get_any_at_fwd_limit(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double) grp->getAnyAtLimitFwd();
}

inline double mc_grp_get_any_at_bwd_limit(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double) grp->getAnyAtLimitBwd();
}

inline double mc_grp_get_any_at_limit(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double) grp->getAnyAtLimit();
}

inline double mc_grp_get_any_ilocked(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double) grp->getAnyIlocked();
}

inline double mc_grp_set_slaved_axis_ilocked(double grpIndex){
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  grp->setSlavedAxisIlocked();
  return 0.0;
}

inline double mc_grp_get_ctrl_within_db(double grpIndex)
{
  int intGrpIndex=(int)grpIndex;
  CHECK_PLC_AXISGRP_RETURN_0_IF_ERROR(intGrpIndex);
  auto * const grp = axisGroups[intGrpIndex];
  return (double)grp->getAxisIsWithinCtrlDB();
}

#endif /* ecmcPLC_libMc_inc_ */
