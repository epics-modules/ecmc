/*
 * cMcuAxisBase.h
 *
 *  Created on: Mar 10, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCAXISBASE_H_
#define ECMCAXISBASE_H_

#define __STDC_FORMAT_MACROS  // To "reinclude" inttypes
#include "../main/ecmcDefinitions.h"
#include "../main/ecmcError.h"
#include "../com/ecmcAsynPortDriver.h"
#include "ecmcCommandTransform.h"
#include "ecmcDriveBase.h"
#include "ecmcDriveStepper.h"
#include "ecmcDriveDS402.h"
#include "ecmcEncoder.h"
#include "ecmcMonitor.h"
#include "ecmcPIDController.h"
#include "ecmcAxisSequencer.h"
#include "ecmcTrajectoryTrapetz.h"
#include "ecmcMasterSlaveIF.h"
#include "ecmcAxisData.h"

// AXIS ERRORS
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
#define ERROR_AXIS_ASYN_PORT_OBJ_NULL 0x1431F
#define ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL 0x14320
#define ERROR_AXIS_PRINT_TO_BUFFER_FAIL 0x14321
#define ERROR_AXIS_MODULO_OUT_OF_RANGE 0x14322
#define ERROR_AXIS_MODULO_TYPE_OUT_OF_RANGE 0x14323

enum axisState {
  ECMC_AXIS_STATE_STARTUP  = 0,
  ECMC_AXIS_STATE_DISABLED = 1,
  ECMC_AXIS_STATE_ENABLED  = 2,
};

typedef struct {
  double             positionSetpoint;
  double             positionActual;
  double             positionError;
  double             positionTarget;
  double             cntrlError;
  double             cntrlOutput;
  double             velocityActual;
  double             velocitySetpoint;
  double             velocityFFRaw;
  int64_t            positionRaw;
  int                error;
  int                velocitySetpointRaw;
  int                seqState;
  int                cmdData;
  motionCommandTypes command;
  interlockTypes     trajInterlock;
  interlockTypes     lastActiveInterlock;
  dataSource         trajSource;
  dataSource         encSource;
  bool               enable;
  bool               enabled;
  bool               execute;
  bool               busy;
  bool               atTarget;
  bool               homed;
  bool               limitFwd;
  bool               limitBwd;
  bool               homeSwitch;
} ecmcAxisStatusOnChangeType;

typedef struct {
  int                        axisID;
  int                        cycleCounter;
  double                     acceleration;
  double                     deceleration;
  bool                       reset;
  bool                       moving;
  bool                       stall;
  ecmcAxisStatusOnChangeType onChangeData;
} ecmcAxisStatusType;

class ecmcAxisBase : public ecmcError {
 public:
  ecmcAxisBase(ecmcAsynPortDriver *asynPortDriver,
               int    axisID,
               double sampleTime);
  virtual ~ecmcAxisBase();
  virtual int                setOpMode(operationMode nMode) = 0;
  virtual operationMode      getOpMode()                    = 0;
  virtual int                getCntrlError(double *error)   = 0;
  virtual int                setEnable(bool enable)         = 0;
  virtual int                setDriveType(ecmcDriveTypes driveType);
  virtual ecmcDriveBase    * getDrv()               = 0;
  virtual ecmcPIDController* getCntrl()             = 0;
  virtual int                validate()             = 0;
  virtual void               execute(bool masterOK) = 0;
  virtual void               printCurrentState();
  bool                       getEnable();
  bool                       getEnabled();
  void                       preExecute(bool masterOK);
  void                       postExecute(bool masterOK);
  int                        setExecute(bool execute);
  bool                       getExecute();
  int                        getAxisHomed(bool *homed);
  int                        setAxisHomed(bool homed);
  int                        getEncScaleNum(double *scale);
  int                        setEncScaleNum(double scale);
  int                        getEncScaleDenom(double *scale);
  int                        setEncScaleDenom(double scale);
  int                        getEncPosRaw(int64_t *rawPos);
  int                        setCommand(motionCommandTypes command);
  int                        setCmdData(int cmdData);
  motionCommandTypes         getCommand();
  int                        getCmdData();
  int                        slowExecute();
  ecmcTrajectoryTrapetz    * getTraj();
  ecmcMonitor              * getMon();
  ecmcEncoder              * getEnc();
  ecmcAxisSequencer        * getSeq();
  int                        getPosAct(double *pos);
  int                        getPosSet(double *pos);
  int                        getVelAct(double *vel);
  axisType                   getAxisType();
  int                        getAxisID();
  void                       setReset(bool reset);
  bool                       getReset();
  int                        setEnableCascadedCommands(bool enable);
  bool                  getCascadedCommandsEnabled();
  int                   setEnableCommandsTransform(bool enable);
  bool                  getEnableCommandsTransform();
  int                   setAxisArrayPointer(ecmcAxisBase *axes,
                                            int           index);
  int                   setCommandsTransformExpression(
    std::string expression);
  ecmcCommandTransform* getCommandTransform();
  void                  setInStartupPhase(bool startup);
  int                   setTrajTransformExpression(
    std::string expressionString);
  int                   setEncTransformExpression(
    std::string expressionString);
  int                   setTrajDataSourceType(dataSource refSource);
  int                   setEncDataSourceType(dataSource refSource);
  int                   setRealTimeStarted(bool realtime);
  bool                  getError();
  int                   getErrorID();
  void                  errorReset();
  int                   setEnableLocal(bool enable);
  int                   validateBase();
  ecmcMasterSlaveIF   * getExternalTrajIF();
  ecmcMasterSlaveIF   * getExternalEncIF();
  bool                  getBusy();
  int                   getBlockExtCom();
  int                   setBlockExtCom(int block);
  int                   getDebugInfoData(ecmcAxisStatusType *data);      // memcpy
  int                   getAxisDebugInfoData(char *buffer,
                                             int   bufferByteSize,
                                             int  *bytesUsed);
  ecmcAxisStatusType  * getDebugInfoDataPointer();
  int                   getCycleCounter();
  void                  printAxisStatus();
  int                   initAsyn();
  int                   setEcStatusOutputEntry(ecmcEcEntry *entry);
  motionDirection       getAxisSetDirection();
  int                   setModRange(double mod);
  double                getModRange();
  int                   setModType(int type);
  int                   getModType();

 protected:
  void         printAxisState();
  void         initVars();
  int          fillCommandsTransformData();
  bool         checkAxesForEnabledTransfromCommands(commandType type);
  int          setEnable_Transform();
  int          setExecute_Transform();
  int          refreshExternalInputSources();
  int          refreshExternalOutputSources();
  virtual void refreshDebugInfoStruct() = 0;
  double       getPosErrorMod();
  bool cascadedCommandsEnable_;  // Allow other axis to enable and execute this
                                 // axis
  bool enableCommandTransform_;  // Allow other axis to enable and execute this
                                 // axis
  ecmcCommandTransform *commandTransform_;
  ecmcAxisBase *axes_[ECMC_MAX_AXES];
  ecmcMasterSlaveIF *externalInputTrajectoryIF_;
  ecmcMasterSlaveIF *externalInputEncoderIF_;
  ecmcTrajectoryTrapetz *traj_;
  ecmcMonitor *mon_;
  ecmcEncoder *enc_;
  ecmcAxisSequencer seq_;
  ecmcAxisStatusType statusData_;
  ecmcAxisStatusType statusDataOld_;
  int printHeaderCounter_;
  ecmcAxisData data_;
  bool executeCmdOld_;
  int cycleCounter_;
  axisState axisState_;
  double oldPositionAct_;
  double oldPositionSet_;

  // Axis default parameters over asyn I/O intr
  ecmcAsynPortDriver *asynPortDriver_;
  ecmcAsynDataItem  *axAsynParams_[ECMC_ASYN_AX_PAR_COUNT];
  ecmcEcEntry *statusOutputEntry_;
  int blockExtCom_;
  char diagBuffer_[AX_MAX_DIAG_STRING_CHAR_LENGTH];
  uint32_t statusWord_;
};

#endif  /* ECMCAXISBASE_H_ */
