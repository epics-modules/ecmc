/*************************************************************************\
* Copyright (c) 2024 Paul Scherrer Institut
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcMasterSlaveStateMachine.cpp
*
*  Created on: Jun 09, 2025
*      Author: anderssandstrom
*
\*************************************************************************/
#include "ecmcMasterSlaveStateMachine.h"

ecmcMasterSlaveStateMachine::ecmcMasterSlaveStateMachine(int index,
                                                         const char *name,
                                                         ecmcAxisGroup *masterGrp,
                                                         ecmcAxisGroup *slaveGrp){
  index_ = index;
  name_ = name;
  masterGrp_ = masterGrp;
  slaveGrp_ = slaveGrp;
  std::cout "ecmcMasterSlaveStateMachine: Created master slave state machine [" << index_ << "]: " << name_;
};

ecmcMasterSlaveStateMachine::~ecmcMasterSlaveStateMachine(){
};

const char* ecmcMasterSlaveStateMachine::getName(){
  return name_.c_str();
};

void ecmcMasterSlaveStateMachine::execute(){



};

int ecmcMasterSlaveStateMachine::validate(){
  if( masterGrp_ == NULL || slaveGrp_ == NULL){
    return ERROR_MST_SLV_SM_GRP_NULL;
  };
  return 0;
};

// PLC code to be converted to c++
# Special version of state machine:
#  Master axes (normally the virtual axes) will not be automatically disabled when in state 1 (can be handled by motor record instead)
#  Slaved axes, (normally the physical axes), will however be automatically disabled when in state -1.
#

#- Initial PLC values
if(${SELF}.firstscan) {
  static.counter :=  0;
  static.VMState := -2;
  #- ignore error at disable from master axes motor record. Disable is controlled by plc code.
  mc_grp_set_ignore_mr_status_check_at_disable(${GRP_ID_MA},1);
};

#- this is needed because initially upon IocInit, motor busy flags are briefly enabled, and the first motorRecord poll at 100ms might cause issues
if ( static.counter < ${MR_DLY=0.105} ) {
  static.counter += ${SELF}.scantime;

  if ( static.counter >= ${MR_DLY=0.105}) { static.VMState:=0 };

  #- VMs in Internal mode, ready to listen
  mc_grp_set_traj_src(${GRP_ID_MA},0);
};

# STATES
# State -1: at least 1 RM is in motion, RM in charge.
# State  0: all RM and VM are off.
# State  1: at least 1 VM is in motion, VM in charge.

#- State -1: at least 1 RM is in motion, RM in charge.
if(static.VMState==-1) {

  #- RM are no longer in motion
  #- -1 -> 0
  #- Trigger Condition:
  #-  - all RM are not busy
  if( mc_grp_get_any_busy(${GRP_ID_SA}) == 0 ) {

    #- disable motors
    mc_grp_set_enable(${GRP_ID_SA},0);
    mc_grp_mr_set_cnen(${GRP_ID_SA},0);
    mc_grp_set_enable(${GRP_ID_MA},0);
    mc_grp_mr_set_cnen(${GRP_ID_MA},0);

    #- VMs in Internal mode, back to ground state
    mc_grp_set_traj_src(${GRP_ID_MA},0);

    #- state change
    ${DBG=#}println('-1 -> 0');
    static.VMState:=0;

    #- sync virtual axes
    mc_grp_mr_set_sync(${GRP_ID_MA},1);
    mc_grp_mr_set_stop(${GRP_ID_MA},1);
  };
  
  #- If RM Motion is occuring, and user commands VM motion: simply disable the VMs to void the command.
  if( mc_grp_get_any_enabled(${GRP_ID_MA})==1 and mc_grp_get_any_enable(${GRP_ID_MA})==1 ) {
    mc_grp_set_enable(${GRP_ID_MA},0);
  };
}

#- State 0: all RM and VM are off.
else if(static.VMState == 0) {

  #- 0 -> -1
  #- Trigger Condition:
  #-  - at least 1 RM is running
  #-  - all RM are in internal mode
  #-  - all VM are disabled

  if( mc_grp_get_any_busy(${GRP_ID_SA}) == 1 and mc_grp_get_any_traj_src_ext(${GRP_ID_SA}) == 0 and mc_grp_get_any_enabled(${GRP_ID_MA}) == 0 ) {
    #- VMs in PLC mode, so no following errors occur
    mc_grp_set_traj_src(${GRP_ID_MA},0);
    #- state change
    ${DBG=#}println('0 -> -1');
    static.VMState:=-1;
  }

  #- 0 -> 1
  #- Trigger Condition:
  #-  - at least 1 VM is enabled
  else if( mc_grp_get_any_enabled(${GRP_ID_MA})==1 and mc_grp_get_any_error_id(${GRP_ID_SA})==0) {
    #- Exit Condition: all axes are on
    if( mc_grp_get_enabled(${GRP_ID_SA}) == 1 and mc_grp_get_enabled(${GRP_ID_MA}) == 1 and mc_grp_get_any_busy(${GRP_ID_SA}) == 0 ) {
      if( mc_grp_get_any_busy(${GRP_ID_MA}) == 1 ) {
        mc_grp_set_traj_src(${GRP_ID_SA},1);
        #- state change
        ${DBG=#}println('0 -> 1');
        static.VMState:=1;
      };    
    }
    #- Actions: If Exit Conditions not met
    else {
      #- enable motors      
      var master:= mc_grp_set_enable(${GRP_ID_MA},1);
      var slave := mc_grp_set_enable(${GRP_ID_SA},1);
      if(slave or master) {
        mc_grp_set_enable(${GRP_ID_MA},0);
        mc_grp_set_enable(${GRP_ID_SA},0);
        mc_grp_mr_set_cnen(${GRP_ID_MA},0);
        mc_grp_mr_set_cnen(${GRP_ID_SA},0);
        mc_grp_mr_set_sync(${GRP_ID_MA},1);
        mc_grp_mr_set_sync(${GRP_ID_SA},1);
        if(slave) {
          mc_grp_set_slaved_axis_in_error(${GRP_ID_MA});
        }
        print('Error enabling axes...');
        static.VMState:=0;
        ${DBG=#}println('0 -> 0');
      };
      #- halt real motors
      mc_grp_halt(${GRP_ID_SA});
    };
  } else if(mc_grp_get_any_error_id(${GRP_ID_SA}) > 0) {
    mc_grp_set_slaved_axis_in_error(${GRP_ID_MA});
  };
}

#- State 1: At least 1 VM is in motion, VM in charge.
else if(static.VMState == 1) {

  #- Check if any slaved axis is in error state and forward to master axes
  if(mc_grp_get_any_error_id(${GRP_ID_SA})) {
    mc_grp_set_slaved_axis_in_error(${GRP_ID_MA});
  };

  # Keep enable if atleast one master axis is busy..
  #if(mc_grp_get_any_busy(${GRP_ID_MA}) == 1) {
  #    mc_grp_set_enable(${GRP_ID_MA},1);
  #};
  # else
  if( mc_grp_get_any_busy(${GRP_ID_MA}) == 0 and mc_grp_get_any_enabled(${GRP_ID_MA}) == 1 ) {
    if(${AUTO_DISABLE_VMS=1}) {
      mc_grp_set_enable(${GRP_ID_MA},0);
      mc_grp_set_enable(${GRP_ID_SA},0);
    };    
  };

  #- autodisable only enabled when the master axes are not busy (only valid for axis.autoEnable.disableTime > 0)
  mc_grp_set_autodisable_enable(${GRP_ID_MA},mc_grp_get_any_busy(${GRP_ID_MA}) == 0);

  #- Set "atTarget/inCtrlDb" of slaved axes if all master axes are atTarget (to reduce trq on slaved axes)
  #- Only valid in external traj mode
  mc_grp_set_ctrl_within_db(${GRP_ID_SA},mc_grp_get_attraget(${GRP_ID_MA}))

  #- Sync slaved axes enable with master axis enabled
  mc_grp_set_enable(${GRP_ID_SA},mc_grp_get_enable(${GRP_ID_MA}));

  # If any of the slaved axes at a limit then switch state -1
  if(mc_grp_get_any_at_limit(${GRP_ID_SA})) {
    ${DBG=#}println('Physical axis at limit. Force state change to -1')
    ${DBG=#}println('1 -> -1');
    static.VMState:=-1;

    mc_grp_set_traj_src(${GRP_ID_SA},0);
    mc_grp_mr_set_sync(${GRP_ID_SA},1);
    mc_grp_mr_set_stop(${GRP_ID_SA},1);
    mc_grp_halt(${GRP_ID_SA});

    #- sync real axes
    mc_grp_mr_set_sync(${GRP_ID_SA},1);
    mc_grp_mr_set_stop(${GRP_ID_SA},1);

    # Set master axis in error state
    mc_grp_set_slaved_axis_ilocked(${GRP_ID_MA});
  };

  #- basically a done test
  #- 1 -> 0
  if( mc_grp_get_any_enabled(${GRP_ID_MA}) == 0 and mc_grp_get_any_enabled(${GRP_ID_SA}) == 0) {
    
    mc_grp_set_traj_src(${GRP_ID_SA},0);

    mc_grp_reset_error(${GRP_ID_SA});
    #- state change
    ${DBG=#}println('1 -> 0');
    static.VMState:= 0;

    #- sync real axes
    mc_grp_mr_set_sync(${GRP_ID_SA},1);
    mc_grp_mr_set_stop(${GRP_ID_SA},1);
  };
};
