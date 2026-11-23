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
#define MST_SLV_MASTER_AT_TARGET_TIMEOUT_DEFAULT_S 10.0
#define MST_SLV_TRAJ_SRC_CHANGE_WAIT_CYCLES 5

#include "ecmcError.h"
#include "ecmcErrorsList.h"
#include "ecmcAxisGroup.h"
#include "ecmcDefinitions.h"
#include "ecmcAsynPortDriver.h"
#include <cstdint>
#include <string>
#include <iostream>

typedef struct {
  unsigned char enable : 1;
  unsigned char autoDisableMasters :1;
  unsigned char autoDisableSlaves  :1;
  unsigned char enableDbgPrintouts :1;
  unsigned int  dummy        : 28;
} ecmcMasterSlaveControlWord;

enum masterSlaveStates {
  ECMC_MST_SLV_STATE_IDLE    = 0,
  ECMC_MST_SLV_STATE_SLAVES  = 1,
  ECMC_MST_SLV_STATE_MASTERS = 2,
  ECMC_MST_SLV_STATE_RESET   = 3,
};

enum masterSlaveStatus {
  ECMC_MST_SLV_STATUS_IDLE                    = 0,
  ECMC_MST_SLV_STATUS_SLAVE_ACTIVE            = 0x0001,
  ECMC_MST_SLV_STATUS_PREPARING_MASTER        = 0x0002,
  ECMC_MST_SLV_STATUS_WAIT_SLAVE_EXTERNAL     = 0x0004,
  ECMC_MST_SLV_STATUS_MASTER_MOVING           = 0x0008,
  ECMC_MST_SLV_STATUS_WAIT_MASTER_AT_TARGET   = 0x0010,
  ECMC_MST_SLV_STATUS_WAIT_MASTER_DISABLE     = 0x0020,
  ECMC_MST_SLV_STATUS_FORCED_TIMEOUT_RECOVERY = 0x8000,
};

enum masterSlaveTransitionReason {
  ECMC_MST_SLV_TRANSITION_NONE = 0,
  ECMC_MST_SLV_TRANSITION_EXTERNAL_STATE_WRITE,
  ECMC_MST_SLV_TRANSITION_CONTROL_DISABLED,
  ECMC_MST_SLV_TRANSITION_SLAVE_COMMAND,
  ECMC_MST_SLV_TRANSITION_MASTER_COMMAND,
  ECMC_MST_SLV_TRANSITION_SLAVE_COMPLETE,
  ECMC_MST_SLV_TRANSITION_MASTER_COMPLETE,
  ECMC_MST_SLV_TRANSITION_LOST_ENABLE_OR_ERROR,
  ECMC_MST_SLV_TRANSITION_SLAVE_TRAJ_SOURCE_FAILED,
  ECMC_MST_SLV_TRANSITION_RESET_COMPLETE,
  ECMC_MST_SLV_TRANSITION_PREPARE_TIMEOUT,
  ECMC_MST_SLV_TRANSITION_MASTER_DISABLE_TIMEOUT,
};

enum masterSlaveStatusWordLayout : uint32_t {
  ECMC_MST_SLV_STATUS_WORD_PHASE_MASK            = 0x0000FFFFu,
  ECMC_MST_SLV_STATUS_WORD_PREVIOUS_STATE_MASK   = 0x00030000u,
  ECMC_MST_SLV_STATUS_WORD_PREVIOUS_STATE_SHIFT  = 16,
  ECMC_MST_SLV_STATUS_WORD_TRANSITION_REASON_MASK = 0x003C0000u,
  ECMC_MST_SLV_STATUS_WORD_TRANSITION_REASON_SHIFT = 18,
  ECMC_MST_SLV_STATUS_WORD_HISTORICAL_FAULT      = 0x00400000u,
};

class ecmcMotionStateYamlWriter;

class ecmcMasterSlaveStateMachine : public ecmcError {
  friend class ecmcMotionStateYamlWriter;
  public:
    ecmcMasterSlaveStateMachine(ecmcAsynPortDriver *asynPortDriver,
                                int index,
                                const char *name,
                                double sampleTimeS,
                                ecmcAxisGroup *masterGrp,
                                ecmcAxisGroup *slaveGrp,
                                int autoDisbleMasters,
                                int autoDisbleSlaves);
    ~ecmcMasterSlaveStateMachine();
    const char* getName();
    int getIndex() const;
    int getState() const;
    int getStatus() const;
    uint32_t getStatusWord() const;
    int getEnabled() const;
    int getAutoDisableMasters() const;
    int getAutoDisableSlaves() const;
    int getPreviousState() const;
    int getLastTransitionReason() const;
    uint64_t getExecuteCycleCount() const;
    uint64_t getTransitionCount() const;
    uint64_t getLastTransitionCycle() const;
    int getLastFaultCode() const;
    int getLastFaultStatus() const;
    uint64_t getLastFaultCycle() const;
    bool getStartupReady() const;
    int getStartupBlockingAxis() const;
    bool isMasterAxis(int axisIndex) const;
    void execute();
    int validate();
    int validateAxisOwnership(ecmcMasterSlaveStateMachine *other);
    int setMasterAtTargetTimeout(double timeoutS);
    double getMasterAtTargetTimeout() const;

  private:
    int stateIdle();
    int stateSlave();
    int stateMaster();
    int stateReset();
    void resetMasterRuntimeState();
    void transitionTo(masterSlaveStates newState,
                      masterSlaveTransitionReason reason);
    void latchFault(int errorCode, int status);
    uint32_t buildStatusWord() const;
    void enterIdleFromMaster();
    void abortMasterToIdle(int errorCode, const char *reason);
    int initAsyn();
    void refreshAsyn();
    void setMrIgnoreEnableAlarm();
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
    bool masterTrajectoryStartAllowed_;
    bool optionAutoDisableMasters_;
    ecmcAxisGroup *masterGrp_;
    ecmcAxisGroup *slaveGrp_;    
    int status_;
    uint32_t statusWord_;
    ecmcAsynPortDriver *asynPortDriver_;
    ecmcAsynDataItem *asynControl_;
    ecmcAsynDataItem *asynState_;
    ecmcAsynDataItem *asynStatus_;
    ecmcMasterSlaveControlWord control_;
    ecmcMasterSlaveControlWord controlOld_;
    int idleCounter_;
    bool masterGroupWasBusy_;
    bool masterGroupReachedTarget_;
    bool masterDisableInProgress_;
    int slaveTrajSourceExternalWaitCycles_;
    uint64_t masterGroupBusyCycles_;
    double masterAtTargetTimeoutS_;
    double masterAtTargetTimeS_;
    double masterPrepareTimeoutS_;
    double masterPrepareTimeS_;
    uint64_t executeCycleCounter_;
    uint64_t transitionCount_;
    uint64_t lastTransitionCycle_;
    uint64_t lastFaultCycle_;
    masterSlaveStates previousState_;
    masterSlaveStates trackedState_;
    masterSlaveTransitionReason lastTransitionReason_;
    int lastFaultCode_;
    int lastFaultStatus_;
};

#endif  /* ecmcMasterSlaveStateMachine_H_ */
