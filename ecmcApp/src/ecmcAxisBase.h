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



class ecmcAxisBase : public ecmcError
{
public:
  ecmcAxisBase(double sampleTime);
  virtual ~ecmcAxisBase();
  virtual void execute(bool masterOK)=0;
  virtual int setOpMode(operationMode nMode)=0;
  virtual operationMode getOpMode()=0;
  virtual int getActPos(double *pos)=0;
  virtual int getActVel(double *vel)=0;
  virtual int getAxisHomed(bool *homed)=0;
  virtual int getEncScaleNum(double *scale)=0;
  virtual int setEncScaleNum(double scale)=0;
  virtual int getEncScaleDenom(double *scale)=0;
  virtual int setEncScaleDenom(double scale)=0;
  virtual int getCntrlError(double* error)=0;
  virtual int getEncPosRaw(int64_t *rawPos)=0;
  virtual int setExecute(bool execute)=0;
  virtual bool getExecute()=0;
  virtual int setEnable(bool enable)=0;
  virtual bool getEnable()=0;
  virtual int setCommand(motionCommandTypes command)=0;
  virtual int setCmdData(int cmdData)=0;
  virtual motionCommandTypes getCommand()=0;
  virtual int getCmdData()=0;
  virtual int setDriveType(ecmcDriveTypes driveType);
  virtual ecmcDriveBase *getDrv()=0;
  virtual ecmcTrajectoryTrapetz *getTraj()=0;
  virtual ecmcMonitor *getMon()=0;
  virtual ecmcEncoder *getEnc()=0;
  virtual ecmcPIDController *getCntrl()=0;
  virtual ecmcSequencer * getSeq()=0;
  virtual void printStatus()=0;
  virtual int validate()=0;
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
  int setExternalExecute(bool execute);
  int validateBase();

  ecmcMasterSlaveIF * getExternalTrajIF();
  ecmcMasterSlaveIF * getExternalEncIF();

protected:
  void initVars();
  int fillCommandsTransformData();
  bool checkAxesForEnabledTransfromCommands(commandType type);
  int setEnable_Transform();
  int setExecute_Transform();
  int refreshExternalInputSources();
  int refreshExternalOutputSources();

  int axisID_;
  bool reset_;
  axisType axisType_;
  bool cascadedCommandsEnable_;  // Allow other axis to enable and execute this axis
  bool enableCommandTransform_;  // Allow other axis to enable and execute this axis
  ecmcCommandTransform *commandTransform_;
  ecmcAxisBase *axes_[ECMC_MAX_AXES];
  bool inStartupPhase_;
  bool realtime_;
  bool enable_;
  bool externalExecute_;


  ecmcMasterSlaveIF *externalInputTrajectoryIF_;
  ecmcMasterSlaveIF *externalInputEncoderIF_;
  //dataSource trajectoryInputSource_;
  //dataSource encoderInputSource_;

  double externalTrajectoryPosition_;
  double externalTrajectoryVelocity_;
  interlockTypes externalTrajectoryInterlock_;

  double externalEncoderPosition_;
  double externalEncoderVelocity_;
  interlockTypes externalEncoderInterlock_;

  double currentPositionActual_;
  double currentPositionSetpoint_;

  double currentVelocityActual_;
  double currentVelocitySetpoint_;
};

#endif /* ECMCAXISBASE_H_ */
