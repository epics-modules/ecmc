/*************************************************************************\
* Copyright (c) 2024 Paul Scherrer Institut
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcMasterSlaveStateMachine.h
*
*  Created on: Jun 09, 2025
*      Author: anderssandstrom 
*
* Credits: From the beginning this state machine code was executed as ecmc-PLC code and was mainly developed by Alvin Acerbo at PSI.
*
* This class implements a state machine to be able to synchronize execution of motions on different groups of axes,
* master group and slave group. The slave group normally consists of physical axes and the master group of virtual 
* (but could be exceptions).
* Example, for a slit system the physical axes controlling the slits are linked to a the slave group and the gap and 
* center is linked to the master group.
* The state machin starts in "ECMC_MST_SLV_STATE_IDLE" state. If an motion command is executed on a physical/slaved axis then the state 
* machine state is set to "SLAVED_AXES_IN_CHARGE" and commands for the master axes are blocked. All axes in the slaved 
* group can the be moved. When all motion commands are executed and no axes in the slaved group are busy then all 
* slaved axes are disabled and the state is set to ECMC_MST_SLV_STATE_SLAVES. If a axis in the master axes group gets a command then the state
* is set to "ECMC_MST_SLV_STATE_MASTERS", then all master axis can receive commands, in addition to that, all axes in both 
* groups are enabled (if auto enable is configured) and the trajectory source for the slaved axes are set to external.
* Any of the master axes can now receive commands but commands to the slaved axes are discarded. Again,
* after all motion commands in the master group have been finalized the all axes disable and state is back to idle.
* Basically the state machine allows the user to execute motion commands on any axes in both groups without the need of 
* chaning anything.
*
\*************************************************************************/

#ifndef ecmcMasterSlaveStateMachine_H_
#define ecmcMasterSlaveStateMachine_H_

#define MST_SLV_START_DELAY_S 0.105

#define ERROR_MST_SLV_SM_GRP_NULL 0x17000
#define ERROR_MST_SLV_SM_GRP_INIT_ASYN_FAILED 0x17001

#include "ecmcError.h"
#include "ecmcAxisGroup.h"
#include "ecmcDefinitions.h"
#include "ecmcAsynPortDriver.h"
#include <string>
#include <iostream>

enum masterSlaveStates {
  ECMC_MST_SLV_STATE_IDLE    = 0,
  ECMC_MST_SLV_STATE_SLAVES  = 1,
  ECMC_MST_SLV_STATE_MASTERS = 2,
  ECMC_MST_SLV_STATE_RESET   = 3,
};

class ecmcMasterSlaveStateMachine : public ecmcError {
  public:
    ecmcMasterSlaveStateMachine(ecmcAsynPortDriver *asynPortDriver,
                                int index,
                                const char *name,
                                double sampleTimeS,
                                ecmcAxisGroup *masterGrp,
                                ecmcAxisGroup *slaveGrp);
    ~ecmcMasterSlaveStateMachine();
    const char* getName();
    void execute();
    int validate();

  private:
    int stateIdle();
    int stateSlave();
    int stateMaster();
    int stateReset();
    int initAsyn();
    void refreshAsyn();
    int createAsynParam(const char        *nameFormat,
                        asynParamType      asynType,
                        ecmcEcDataType     ecmcType,
                        uint8_t           *data,
                        size_t             bytes,
                        ecmcAsynDataItem **asynParamOut);

    masterSlaveStates state_;
    std::string name_;
    double sampleTimeS_, timeCounter_;
    int index_;
    bool asynInitOk_;
    bool validationOK_;
    bool optionAutoDisableMasters_;
    ecmcAxisGroup *masterGrp_;
    ecmcAxisGroup *slaveGrp_;
    int enable_;
    int status_;
    ecmcAsynPortDriver *asynPortDriver_;
    ecmcAsynDataItem *asynEnable_;
    ecmcAsynDataItem *asynState_;
    ecmcAsynDataItem *asynStatus_;
};

#endif  /* ecmcMasterSlaveStateMachine_H_ */