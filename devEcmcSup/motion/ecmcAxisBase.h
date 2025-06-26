/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcAxisBase.h
*
*  Created on: Mar 10, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCAXISBASE_H_
#define ECMCAXISBASE_H_

#define __STDC_FORMAT_MACROS  // To "reinclude" inttypes
#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "ecmcAsynPortDriver.h"
#include "ecmcDriveBase.h"
#include "ecmcDriveStepper.h"
#include "ecmcDriveDS402.h"
#include "ecmcEncoder.h"
#include "ecmcMonitor.h"
#include "ecmcPIDController.h"
#include "ecmcAxisSequencer.h"
#include "ecmcTrajectoryBase.h"
#include "ecmcTrajectoryS.h"
#include "ecmcTrajectoryTrapetz.h"
#include "ecmcAxisData.h"
#include "ecmcFilter.h"
#include "ecmcMotionUtils.h"

enum axisState {
  ECMC_AXIS_STATE_STARTUP  = 0,
  ECMC_AXIS_STATE_DISABLED = 1,
  ECMC_AXIS_STATE_ENABLED  = 2,
};

typedef struct {
  unsigned char stopMRCmdTgl : 1;  // Trigger command toggle
  unsigned char stopMRVal    : 1;  // STOP value
  unsigned char syncMRCmdTgl : 1;  // Trigger command toggle
  unsigned char syncMRVal    : 1;  // SYNC value
  unsigned char cnenMRCmdTgl : 1;  // Trigger command toggle
  unsigned char cnenMRVal    : 1;  // CNEN value
  unsigned int  dummy        : 26;
} ecmcMRCmds;

class ecmcAxisBase : public ecmcError {
public:
  ecmcAxisBase(ecmcAsynPortDriver *asynPortDriver,
               int                 axisID,
               double              sampleTime,
               ecmcTrajTypes       trajType);
  virtual ~ecmcAxisBase();
  virtual ecmcDriveBase*     getDrv()               = 0;
  virtual ecmcPIDController* getCntrl()             = 0;
  virtual int                validate()             = 0;
  virtual void               execute(bool masterOK) = 0;
  int                        getCntrlError(double *error);
  int                        setEnable(bool enable);
  bool                       getEnable();
  bool                       getEnabled();  // Enable and Enabled
  bool                       getEnabledOnly();  // Only Enabled
  void                       preExecute(bool masterOK);
  void                       postExecute(bool masterOK);
  int                        setExecute(bool execute);
  int                        setExecute(bool execute, bool ignoreBusy);
  bool                       getExecute();
  int                        getAxisHomed(bool *homed);
  int                        setAxisHomed(bool homed);
  int                        getEncScaleNum(double *scale);
  int                        setEncScaleNum(double scale);
  int                        getEncScaleDenom(double *scale);
  int                        setEncScaleDenom(double scale);
  int                        getEncPosRaw(int64_t *rawPos);
  bool                       getLimitBwd();
  bool                       getLimitFwd();
  int                        setEncInvHwReady(int invert);
  int                        loadEncLookupTable(const char* filename);
  int                        setEncLookupTableEnable(int enable);
  int                        setCommand(motionCommandTypes command);
  int                        setCmdData(int cmdData);
  motionCommandTypes         getCommand();
  int                        getCmdData();
  int                        slowExecute();
  int                        setGlobalBusy(bool busy);  // Allow for sequences
  ecmcAxisDataStatus*        getAxisStatusDataPtr();
  int                        getAxisDebugInfoData(char *buffer,
                                       int   bufferByteSize,
                                       int  *bytesUsed);
  ecmcTrajectoryBase*        getTraj();
  ecmcMonitor*               getMon();
  ecmcEncoder*               getEnc();
  ecmcEncoder*               getEnc(int  encIndex,
                                    int *error);
  ecmcEncoder*               getConfigEnc();  // get current encoder being configured
  ecmcEncoder*               getPrimEnc();
  ecmcEncoder*               getCSPEnc();
  ecmcAxisSequencer*         getSeq();
  int                        getPosAct(double *pos);
  int                        getPosSet(double *pos);
  int                        getVelSet(double *vel);
  int                        getVelAct(double *vel);
  axisTypes                  getAxisType();
  int                        getAxisID();
  void                       setReset(bool reset);
  bool                       getReset();
  int                        setAllowCmdFromPLC(bool enable);
  bool                       getAllowCmdFromPLC();
  void                       setInStartupPhase(bool startup);
  bool                       getInStartupPhase();
  int                        setEncDataSourceType(dataSource refSource);
  dataSource                 getTrajDataSourceType();
  dataSource                 getEncDataSourceType();
  int                        setRealTimeStarted(bool realtime);
  int                        getRealTimeStarted();
  bool                       getError();
  int                        getErrorID();
  void                       errorReset();
  int                        setSlavedAxisInError();
  int                        setSlavedAxisInterlock();
  int                        setEnableLocal(bool enable);
  int                        validateBase();
  bool                       getBusy();
  bool                       getTrajBusy();
  int                        getBlockExtCom();
  int                        setBlockExtCom(int block);
  int                        getCycleCounter();
  void                       printAxisStatus();
  int                        initAsyn();
  int                        setEcStatusOutputEntry(ecmcEcEntry *entry);
  motionDirection            getAxisSetDirection();
  int                        setModRange(double mod);
  double                     getModRange();
  int                        setModType(int type);
  int                        getModType();
  int                        setExtSetPos(double pos);
  int                        setExtActPos(double pos);
  int                        setEnableExtEncVeloFilter(bool enable);
  int                        setEnableExtTrajVeloFilter(bool enable);
  bool                       getEnableExtEncVeloFilter();
  bool                       getEnableExtTrajVeloFilter();
  int                        setExtTrajVeloFiltSize(size_t size);
  int                        setExtEncVeloFiltSize(size_t size);
  int                        setEncVeloFiltSize(size_t size);
  int                        setEncVeloFiltEnable(bool enable);
  int                        setEncPosFiltSize(size_t size);
  int                        setEncPosFiltEnable(bool enable);
  int                        setDisableAxisAtErrorReset(bool disable);
  int                        moveAbsolutePosition(double positionSet,
                                                  double velocitySet,
                                                  double accelerationSet,
                                                  double decelerationSet);
  int                        moveAbsolutePosition(double positionSet);
  int moveRelativePosition(double positionSet,
                           double velocitySet,
                           double accelerationSet,
                           double decelerationSet);
  int moveVelocity(double velocitySet,
                   double accelerationSet,
                   double decelerationSet);
  int moveHome(int    nCmdData,
               double homePositionSet,
               double velocityTowardsCamSet,
               double velocityOffCamSet,
               double accelerationSet,
               double decelerationSet);
  int        movePVTAbs();
  int        movePVTAbs(bool ignoreBusy);  // Ignores busy (for triggering from PVT controller)
  int        moveHome(); // Use configs from encoder object
  int        setPosition(double homePositionSet);                  // Autosave
  int        stopMotion(int killAmplifier);
  int        setAutoEnableTimeout(double timeS);
  int        setAutoDisableAfterTime(double timeS);
  int        setEnableAutoEnable(bool enable);
  int        setEnableAutoDisable(bool enable);
  asynStatus axisAsynWriteCmd(void         *data,
                              size_t        bytes,
                              asynParamType asynParType);
  asynStatus axisAsynWriteTargetVelo(void         *data,
                                     size_t        bytes,
                                     asynParamType asynParType);
  asynStatus axisAsynWriteTargetPos(void         *data,
                                    size_t        bytes,
                                    asynParamType asynParType);
  asynStatus axisAsynWriteCommand(void         *data,
                                  size_t        bytes,
                                  asynParamType asynParType);
  asynStatus axisAsynWriteCmdData(void         *data,
                                  size_t        bytes,
                                  asynParamType asynParType);
  asynStatus axisAsynWriteSetEncPos(void         *data,
                                    size_t        bytes,
                                    asynParamType asynParType);
  asynStatus axisAsynWritePrimEncCtrlId(void         *data,
                                        size_t        bytes,
                                        asynParamType asynParType);
  asynStatus axisAsynWriteAcc(void         *data,
                              size_t        bytes,
                              asynParamType asynParType);
  asynStatus axisAsynWriteDec(void         *data,
                              size_t        bytes,
                              asynParamType asynParType);
  asynStatus axisAsynWriteSetTweakValue(void         *data,
                                        size_t        bytes,
                                        asynParamType asynParType);
  int        setAllowMotionFunctions(bool enablePos,
                                     bool enableConstVel,
                                     bool enableHome);
  int        getAllowPos();
  int        getAllowConstVelo();
  int        getAllowHome();
  int        addEncoder();
  int        selectPrimaryEncoder(int index,
                                  int overrideError);
  int        selectPrimaryEncoder(int index);
  int        selectCSPDriveEncoder(int index);
  int        selectConfigEncoder(int index);
  int        getPrimaryEncoderIndex();                  // Control (PID)
  int        getConfigEncoderIndex();                   // Config
  double     getExtSetPos();
  double     getExtActPos();
  int        setAllowSourceChangeWhenEnabled(bool allow);
  void       setTargetVel(double velTarget);
  void       setTargetPos(double posTarget);
  void       setTweakDist(double dist);
  void       setAcc(double acc);
  void       setDec(double dec);
  void       setEmergencyStopInterlock(int stop);
  void       setExternalMaxVelo(double veloLimit,int active);
  double     getTrajVelo();
  double     getEncVelo();
  void       setMRSync(bool sync);    // SYNC motor record
  void       setMRStop(bool stop);    // STOP motor record
  void       setMRCnen(bool cnen);    // CNEN motor record
  int        setCntrlKp(double kp);
  int        setCntrlKi(double ki);
  int        setCntrlKd(double kd);
  int        setCntrlKff(double kff);

  /*
     Ignore status when motor record tries to disable.
     Only to be used when axis groups and PSI PLC state machine is in use.
     This PLC code overrides the enable commands since indiviual virtual axes 
     are not allowed to disable during motion.
     NOTE: the disable bit will be set and ecmcAxisMotorRecord:setClosedLoop(0) will always return asynSuccess.
  */
  void       setMRIgnoreDisableStatusCheck(bool ignore);
  bool       getMRIgnoreDisableStatusCheck();  // Ignore status when

  int        getSumInterlock();
  int        getPrintDbg();
  ecmcAxisPVTSequence* getPVTObject();
  double     getCurrentPositionSetpoint();
  

protected:
  void       initVars();
  void       refreshDebugInfoStruct();

  int        createAsynParam(const char        *nameFormat,
                             asynParamType      asynType,
                             ecmcEcDataType     ecmcType,
                             uint8_t           *data,
                             size_t             bytes,
                             ecmcAsynDataItem **asynParamOut);
  void refreshStatusWd();
  void initControlWord();
  void initEncoders();
  bool getHwReady();
  void autoEnableSM();
  void autoDisableSM();
  void refreshAsynTargetValue();
  bool commandValid(motionCommandTypes command);

  ecmcTrajectoryBase *traj_;
  ecmcMonitor *mon_;
  ecmcEncoder *encArray_[ECMC_MAX_ENCODERS];
  ecmcAxisSequencer seq_;
  ecmcAxisData data_;
  axisState axisState_;

  // Axis default parameters over asyn I/O intr
  ecmcAsynPortDriver *asynPortDriver_;
  ecmcEcEntry *statusOutputEntry_;
  ecmcFilter *extTrajVeloFilter_;
  ecmcFilter *extEncVeloFilter_;
  bool allowCmdFromOtherPLC_;
  bool executeCmdOld_;
  bool enableExtTrajVeloFilter_;
  bool enableExtEncVeloFilter_;
  bool disableAxisAtErrorReset_;
  bool firstEnableDone_;
  char diagBuffer_[AX_MAX_DIAG_STRING_CHAR_LENGTH];
  int printHeaderCounter_;
  int blockExtCom_;
  double setEncoderPos_;
  ecmcTrajTypes currentTrajType_;
  int encPrimIndexAsyn_;
  int hwReadyOld_;
  int hwReady_;
  // Commads to trigger MR record actions like SYNC and STOP from ecmc over PVs
  ecmcMRCmds mrCmds_;
  ecmcMRCmds mrCmdsOld_;
  bool globalBusy_;
  bool ignoreMRDisableStatusCheck_;
  double autoEnableTimoutS_;
  double autoDisableAfterS_;
  bool autoEnableRequest_;
  double autoEnableTimeCounter_;
  double autoDisbleTimeCounter_;
  bool enableAutoEnable_;
  bool enableAutoDisable_;
  double positionTargetAsyn_;
};

#endif  /* ECMCAXISBASE_H_ */
