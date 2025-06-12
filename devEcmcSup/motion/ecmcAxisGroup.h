/*************************************************************************\
* Copyright (c) 2024 Paul Scherrer Institut
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAxisGroup.h
*
*  Created on: Mar 28, 2024
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCAXISGROUP_H_
#define ECMCAXISGROUP_H_

#define ERROR_AXISGRP_ADD_GROUP_FAIL 0x16000
#define ERROR_AXISGRP_NOT_FOUND 0x16001

#include "ecmcError.h"
#include "ecmcAxisBase.h"
#include "ecmcDefinitions.h"
#include <stdexcept>
#include <vector>
#include <string>

class ecmcAxisGroup : public ecmcError {
  public:
    ecmcAxisGroup(int index, const char *name);
    ~ecmcAxisGroup();
    // Get group name
    const char* getName();
    // Add axis to group
    void addAxis(ecmcAxisBase *axis);
    // Check if all axes in group are enable
    bool getEnable();
    // Check if at least one axis in group are enable
    bool getAnyEnable();
    // Check if all axes in group are enabled
    bool getEnabled();
    // Check if any axes in group are enabled
    bool getAnyEnabled();
    // Check if any axes in group is at a limit switch
    bool getAnyAtLimit();
    // Check if any axes in group is at fwd limit switch
    bool getAnyAtLimitFwd();
    // Check if any axes in group is at bwd limit switch
    bool getAnyAtLimitBwd();
    // Check if any axes in group is interlocked
    bool getAnyIlocked();
    // Check if all axes in group are busy
    bool getBusy();
    // Check if at least one axis in group are busy
    bool getAnyBusy();
    // Check if at least one axis in group are in error state
    int getAnyErrorId();
    // Set enable of all axes in group
    int setEnable(bool enable);
    // set traj source of all axes in group
    int setTrajSrc(dataSource trajSource);
    // set allow traj source change when enabled
    void setAllowSrcChangeWhenEnabled(int allow);
    // get all traj src in extern
    bool getTrajSrcExt();
    // get all traj src in intern
    bool getTrajSrcAnyExt();
    // set enc source of all axes in group
    int setEncSrc(dataSource encSource);
    // Reset errors all axes
    void setErrorReset();
    // Set errors all axes
    void setError(int error);
    // Set slaved axis error all axes
    void setSlavedAxisInError();
    // Set slaved axis interlock
    void setSlavedAxisIlocked();
    // SYNC motor record
    void setMRSync(bool sync);
    // STOP motor record
    void setMRStop(bool stop);
    // CNEN motor record
    void setMRCnen(bool cnen);
    // Ignore MR disabel status check
    void setMRIgnoreDisableStatusCheck(bool ignore);
    // Stop motion
    void halt();
    // Check if axis is in group
    bool inGroup(int axisIndex);
    // Axis count in group
    size_t size();
    // Enable auto-enable functionality (if configured)
    void setEnableAutoEnable(bool enable);
    // Enable auto-disable functionality (if configured)
    void setEnableAutoDisable(bool enable);
    // Axis is within ctrl deadband for reduce trq when in external traj mode 
    void setAxisIsWithinCtrlDBExtTraj(bool within);
    // get "atTarget"
    bool getAxisIsWithinCtrlDB();

  private:
    std::string name_;  
    int index_;  
    std::vector<ecmcAxisBase*>  axes_;
    size_t axesCounter_;
    std::vector<int> axesIds_;
};

#endif  /* ECMCAXISGROUP_H_ */