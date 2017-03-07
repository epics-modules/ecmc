/*
 * cMcuAxisBase.h
 *
 *  Created on: Mar 10, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCAXISBASE_H_
#define ECMCAXISBASE_H_

#include <stdint.h>
#include <string.h>
#include "ecmcCommandTransform.h"
#include "ecmcDefinitions.h"
#include "ecmcDriveBase.hpp"
#include "ecmcDriveStepper.hpp"
#include "ecmcDriveDS402.hpp"
#include "ecmcEncoder.h"
#include "ecmcError.h"
#include "ecmcMonitor.hpp"
#include "ecmcPIDController.hpp"
#include "ecmcSequencer.hpp"
#include "ecmcTrajectoryTrapetz.hpp"
#include "ecmcMasterSlaveIF.h"
#include "ecmcAxisData.h"

//AXIS ERRORS
#define ERROR_AXIS_OBJECTS_NULL_OR_EC_INIT_FAIL 0x14300
#define ERROR_AXIS_DRV_OBJECT_NULL 0x14301
#define ERROR_AXIS_ENC_OBJECT_NULL 0x14302
#define ERROR_AXIS_MON_OBJECT_NULL 0x14303
#define ERROR_AXIS_TRAJ_OBJECT_NULL 0x14304
#define ERROR_AXIS_CNTRL_OBJECT_NULL 0x14305
#define ERROR_AXIS_SEQ_ERROR_WRONG_SENSOR_EDGE 0x14306
#define ERROR_AXIS_UNDEFINED_TYPE 0x14307
#define ERROR_AXIS_FORWARD_TRANSFORM_NULL 0x14308
#define ERROR_AXIS_INVERSE_TRANSFORM_NULL 0x14309
#define ERROR_AXIS_TRANSFORM_ERROR_OR_NOT_COMPILED 0x1430A
#define ERROR_AXIS_FUNCTION_NOT_SUPPRTED 0x1430B
#define ERROR_AXIS_MASTER_AXIS_OBJECT_NULL 0x1430C
#define ERROR_AXIS_MASTER_AXIS_ENCODER_NULL 0x1430D
#define ERROR_AXIS_MASTER_AXIS_TRAJECTORY_NULL 0x1430E
#define ERROR_AXIS_MASTER_AXIS_TRANSFORM_NULL 0x1430F
#define ERROR_AXIS_SOURCE_TYPE_NOT_DEFINED 0x14310
#define ERROR_AXIS_CMD_NOT_ALLOWED_WHEN_ENABLED 0x14311
#define ERROR_AXIS_CONFIGURED_COUNT_ZERO 0x14312
#define ERROR_AXIS_CASCADED_AXIS_INDEX_OUT_OF_RANGE 0x14313
#define ERROR_AXIS_INDEX_OUT_OF_RANGE 0x14314
#define ERROR_AXIS_HARDWARE_STATUS_NOT_OK 0x14315
#define ERROR_AXIS_NOT_ENABLED 0x14316
#define ERROR_AXIS_AMPLIFIER_ENABLED_LOST 0x14317
#define ERROR_AXIS_SEQ_OBJECT_NULL 0x14318
#define ERROR_AXIS_COMMAND_NOT_ALLOWED_WHEN_ENABLED 0x14319
#define ERROR_AXIS_ASSIGN_EXT_INTERFACE_TO_SEQ_FAILED 0x1431A
#define ERROR_AXIS_DATA_POINTER_NULL 0x1431B
#define ERROR_AXIS_BUSY 0x1431C
#define ERROR_AXIS_TRAJ_MASTER_SLAVE_IF_NULL 0x1431D
#define ERROR_AXIS_ENC_MASTER_SLAVE_IF_NULL 0x1431E

enum axisState{
  ECMC_AXIS_STATE_STARTUP=0,
  ECMC_AXIS_STATE_DISABLED=1,
  ECMC_AXIS_STATE_ENABLED=2,
};

typedef struct {
    double positionSetpoint;
    double positionActual;
    double positionError;
    double positionTarget;
    double cntrlError;
    double cntrlOutput;
    double velocityActual;
    double velocitySetpoint;
    int velocitySetpointRaw;
    double velocityFFRaw;
    int   error;
    bool enable;
    bool enabled;
    bool execute;
    bool busy;
    int seqState;
    bool atTarget;
    interlockTypes trajInterlock;
    bool limitFwd;
    bool limitBwd;
    bool homeSwitch;
    motionCommandTypes command;
    int cmdData;
    dataSource trajSource;
    dataSource encSource;
} ecmcAxisStatusOnChangeType;


typedef struct {
    int axisID;
    int cycleCounter;
    ecmcAxisStatusOnChangeType onChangeData;
} ecmcAxisStatusType;

class ecmcAxisBase : public ecmcError
{
public:
  ecmcAxisBase(int axisID, double sampleTime);
  virtual ~ecmcAxisBase();
  virtual int setOpMode(operationMode nMode)=0;
  virtual operationMode getOpMode()=0;
  virtual int getCntrlError(double* error)=0;
  virtual int setEnable(bool enable)=0;
  virtual int setDriveType(ecmcDriveTypes driveType);
  virtual ecmcDriveBase *getDrv()=0;
  virtual ecmcPIDController *getCntrl()=0;
  virtual int validate()=0;
  virtual void execute(bool masterOK)=0;
  bool getEnable();
  bool getEnabled();
  void preExecute(bool masterOK);
  void postExecute(bool masterOK);
  int setExecute(bool execute);
  bool getExecute();
  int getAxisHomed(bool *homed);
  int getEncScaleNum(double *scale);
  int setEncScaleNum(double scale);
  int getEncScaleDenom(double *scale);
  int setEncScaleDenom(double scale);
  int getEncPosRaw(int64_t *rawPos);
  int setCommand(motionCommandTypes command);
  int setCmdData(int cmdData);
  motionCommandTypes getCommand();
  int getCmdData();

  ecmcTrajectoryTrapetz *getTraj();
  ecmcMonitor *getMon();
  ecmcEncoder *getEnc();
  ecmcSequencer * getSeq();
  int getPosAct(double *pos);
  int getPosSet(double *pos);
  int getVelAct(double *vel);
  axisType getAxisType();
  int getAxisID();
  void setReset(bool reset);
  bool getReset();
  int setEnableCascadedCommands(bool  enable); //Allow enable and execute commands from master axis to this axis
  bool getCascadedCommandsEnabled();
  int setEnableCommandsTransform(bool  enable);
  bool getEnableCommandsTransform();
  int setAxisArrayPointer(ecmcAxisBase *axes,int index);
  int setCommandsTransformExpression(std::string expression);
  ecmcCommandTransform *getCommandTransform();
  void setInStartupPhase(bool startup);
  int setTrajTransformExpression(std::string expressionString);
  int setEncTransformExpression(std::string expressionString);
  int setTrajDataSourceType(dataSource refSource);
  int setEncDataSourceType(dataSource refSource);
  int setRealTimeStarted(bool realtime);
  bool getError();
  int getErrorID();
  void errorReset();
  int setEnableLocal(bool enable);
  int validateBase();
  ecmcMasterSlaveIF *getExternalTrajIF();
  ecmcMasterSlaveIF *getExternalEncIF();
  bool getBusy();
  int getDebugInfoData(ecmcAxisStatusType *data);
  int getCycleCounter();
  void printAxisStatus();
protected:
  void initVars();
  int fillCommandsTransformData();
  bool checkAxesForEnabledTransfromCommands(commandType type);
  int setEnable_Transform();
  int setExecute_Transform();
  int refreshExternalInputSources();
  int refreshExternalOutputSources();
  virtual void refreshDebugInfoStruct()=0;
  bool cascadedCommandsEnable_;  // Allow other axis to enable and execute this axis
  bool enableCommandTransform_;  // Allow other axis to enable and execute this axis
  ecmcCommandTransform *commandTransform_;
  ecmcAxisBase *axes_[ECMC_MAX_AXES];
  ecmcMasterSlaveIF *externalInputTrajectoryIF_;
  ecmcMasterSlaveIF *externalInputEncoderIF_;
  ecmcTrajectoryTrapetz *traj_;
  ecmcMonitor *mon_;
  ecmcEncoder *enc_;
  ecmcSequencer seq_;
  ecmcAxisStatusType statusData_;
  ecmcAxisStatusType statusDataOld_;
  int printHeaderCounter_;
  ecmcAxisData data_;
  bool executeCmdOld_;
  bool trajInterlockOld;
  int cycleCounter_;
  axisState axisState_;

};

#endif /* ECMCAXISBASE_H_ */
