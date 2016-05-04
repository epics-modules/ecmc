#ifndef MOTOR_H
#define MOTOR_H

#include <inttypes.h>
#include <string.h>

#include "ecmcDefinitions.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AXIS_CHECK_RETURN(_axis) {init_axis(_axis); if (((_axis) <= 0) || ((_axis) >=MAX_AXES)) return;}
#define AXIS_CHECK_RETURN_ZERO(_axis) {init_axis(_axis); if (((_axis) <= 0) || ((_axis) >=MAX_AXES)) return 0;}

void hw_motor_init(int);
int hw_motor_global_init(void);


/**************** Set and Get interface***********************/
int getAxisError(int axisIndex);
int getAxisErrorID(int axisIndex);
const char *getErrorString(int errorNumber);
int getAxisExecute(int axisIndex,int *value);
int getAxisCommand(int axisIndex,int *value);
int getAxisCmdData(int axisIndex,int *value);
int getAxisReset(int axisIndex,int *value);
int getAxisEnabled(int cntrl_no,int *value);
int getAxisEnable(int axisIndex,int *value);
int getAxisBusy(int axisIndex,int*value);
int getAxisID(int axisIndex,int*value);
int getAxisEnableAlarmAtHardLimits(int axisIndex,int *value);
int getAxisSoftLimitPosBwd(int axisIndex,double *value);
int getAxisSoftLimitPosFwd(int axisIndex,double *value);
int getAxisEnableSoftLimitBwd(int axisIndex,int *value);
int getAxisEnableSoftLimitFwd(int axisIndex,int *value);
int getAxisOpMode(int axisIndex,int *value);
int getAxisAcceleration(int axisIndex,double *value);
int getAxisDeceleration(int axisIndex,double *value);
int getAxisTargetPos(int axisIndex,double *value);
int getAxisTargetVel(int axisIndex,double *value);
int getAxisDone(int axisIndex,double *value);
int getAxisGearRatio(int axisIndex,double *value);
int getAxisAtHardFwd(int axisIndex,int *value);
int getAxisAtHardBwd(int axisIndex,int *value);
int getAxisEncHomed(int axisIndex,int *value);
int getAxisEncPosAct(int axisIndex,double *value);
int getAxisEncVelAct(int axisIndex,double *value);
int getAxisAtHome(int axisIndex,int *value);
int getAxisCntrlError(int axisIndex,double *value);
int getAxisHomeVelOffCam(int axisIndex,double *value);
int getAxisHomeVelTwordsCam(int axisIndex,double *value);
int getAxisEncScaleNum(int axisIndex,double *value);
int getAxisEncScaleDenom(int axisIndex,double *value);
int getAxisEncPosRaw(int axisIndex,int64_t *value);  //TODO change data type
int getAxisCntrlEnable(int axisIndex,int *value);
int getAxisCntrlTargetPos(int axisIndex,double *value);
int getAxisCntrlEnabled(int axisIndex,int *value);
int getAxisCntrlOutPpart(int axisIndex,double *value);
int getAxisCntrlOutIpart(int axisIndex,double *value);
int getAxisCntrlOutDpart(int axisIndex,double *value);
int getAxisCntrlOutFFpart(int axisIndex,double *value);
int getAxisCntrlOutput(int axisIndex,double *value);
int getAxisCntrlVelSet(int axisIndex,double *value);
int getAxisCntrlRate(int axisIndex,double *value);
int getAxisDrvScale(int axisIndex,double *value);
int getAxisDrvEnable(int axisIndex,int *value);
int getAxisMonAtTarget(int axisIndex,int *value);
int getAxisMonAtTargetCounter(int axisIndex,int *value);
int getAxisMonLagError(int axisIndex,double *value);
int getAxisMonLagMonCounter(int axisIndex,double *value);
int getAxisMonAtHome(int axisIndex,int *value);
int getAxisTrajTransEnable(int axisIndex, int *value);
int getAxisEncTransEnable(int axisIndex, int *value);
int getAxisType(int axisIndex, int *value);
const char* getAxisTrajTransExpr(int axisIndex, int *error);
const char* getAxisEncTransExpr(int axisIndex, int *error);
const char* getAxisTransformCommandExpr(int axisIndex, int *error);
int getAxisTrajSource(int axisIndex, int *value);
int getAxisEncSource(int axisIndex, int *value);
int getAxisEnableCommandsFromOtherAxis(int axisIndex, int *value);
int getAxisEnableCommandsTransform(int axisIndex, int *value);

int setAxisExecute(int axisIndex, int value);
int setAxisCommand(int axisIndex, int value);
int setAxisCmdData(int axisIndex, int value);
int setAxisEnable(int axisIndex, int value);
int setAxisEnableAlarmAtHardLimits(int axisIndex,int value);
int setAxisEnableSoftLimitBwd(int axisIndex, int value);
int setAxisEnableSoftLimitFwd(int axisIndex, int value);
int setAxisSoftLimitPosBwd(int axisIndex, double value);
int setAxisSoftLimitPosFwd(int axisIndex, double value);
int setAxisAcceleration(int axisIndex, double value);
int setAxisDeceleration(int axisIndex, double value);
int setAxisEmergDeceleration(int axisIndex, double value);
int setAxisJerk(int axisIndex, double value);
int setAxisTargetPos(int axisIndex, double value);
int setAxisTargetVel(int axisIndex, double value);
int setAxisJogVel(int axisIndex, double value);
int setAxisOpMode(int axisIndex, int value);
int setAxisEncScaleDenom(int axisIndex, double value);
int setAxisEncScaleNum(int axisIndex, double value);
int setAxisHomePos(int axisIndex, double value);
int setAxisHomeVelTwordsCam(int axisIndex,double dVel);
int setAxisHomeVelOffCam(int axisIndex,double dVel);
int setAxisHomeDir(int axisIndex,int nDir);
int setAxisGearRatio(int axisIndex,double ratioNum,double ratioDenom);
int axisErrorReset(int axisIndex, int value);
int setAxisTrajTransExpr(int axisIndex, char *expr);
int setAxisEncTransExpr(int axisIndex, char *expr);
int setAxisTrajSource(int axisIndex, int value);
int setAxisEncSource(int axisIndex, int value);
int setAxisTrajStartPos(int axisIndex,double value);
int setAxisEncOffset(int axisIndex, double value);
int setAxisEncType(int axisIndex, int value);
int setAxisEncBits(int axisIndex, int value);
int setAxisCntrlKp(int axisIndex, double value);
int setAxisCntrlKi(int axisIndex, double value);
int setAxisCntrlKd(int axisIndex, double value);
int setAxisCntrlKff(int axisIndex, double value);
int setAxisCntrlOutHL(int axisIndex, double value);
int setAxisCntrlOutLL(int axisIndex, double value);
int setAxisCntrlIpartLL(int axisIndex, double value);
int setAxisCntrlIpartHL(int axisIndex, double value);
int setAxisDrvScaleNum(int axisIndex, double value);
int setAxisDrvScaleDenom(int axisIndex, double value);
int setAxisDrvEnable(int axisIndex, int value);
int setAxisDrvVelSet(int axisIndex, double value);
int setAxisDrvVelSetRaw(int axisIndex, int value);
int setAxisMonAtTargetTol(int axisIndex, double value);
int setAxisMonAtTargetTime(int axisIndex, int value);
int setAxisMonEnableAtTargetMon(int axisIndex, int value);
int setAxisMonPosLagTol(int axisIndex, double value);
int setAxisMonPosLagTime(int axisIndex, int value);
int setAxisMonEnableLagMon(int axisIndex, int value);
int setAxisMonMaxVel(int axisIndex, double value);
int setAxisMonEnableMaxVel(int axisIndex, int value);
int setAxisMonMaxVelDriveILDelay(int axisIndex, int value);
int setAxisMonMaxVelTrajILDelay(int axisIndex, int value);
int setAxisSeqTimeout(int axisIndex, int value);
int setAxisEnableCommandsFromOtherAxis(int axisIndex, int value);
int setAxisEnableCommandsTransform(int axisIndex, int value);
int setAxisTransformCommandExpr(int axisIndex,char *expr);

//Application
int setAppMode(int mode);
int createAxis(int index, int type);
int validateConfig();

//Links bewteen MCU and hardware
int linkEcEntryToAxisEnc(int slaveIndex,char *entryIDString,int axisIndex,int encoderEntryIndex);
int linkEcEntryToAxisDrv(int slaveIndex,char *entryIDString,int axisIndex,int driveEntryIndex);
int linkEcEntryToAxisMon(int slaveIndex,char *entryIDString,int axisIndex,int monitorEntryIndex);

//hardware
int ecSetMaster(int masterIndex);
int ecAddSlave(uint16_t alias, uint16_t position, uint32_t vendorId,uint32_t productCode);
int ecAddSyncManager(int slaveIndex,int direction,uint8_t syncMangerIndex);
int ecAddPdo(int slaveIndex,int syncManager,uint16_t pdoIndex);
//int ecAddEntry(int slaveIndex,int nSyncManager,int nPdo,uint16_t nEntryIndex,uint8_t  nEntrySubIndex, uint8_t nBits);
int ecAddEntryComplete(
    uint16_t position,
    uint32_t vendorId,
    uint32_t productCode,
    int direction,
    uint8_t syncMangerIndex,
    uint16_t pdoIndex,
    uint16_t entryIndex,
    uint8_t  entrySubIndex,
    uint8_t nits,
    char *entryIDString);
int ecSlaveConfigDC(
    int slaveBusPosition,
    uint16_t assignActivate, /**< AssignActivate word. */
    uint32_t sync0Cycle, /**< SYNC0 cycle time [ns]. */
    int32_t sync0Shift, /**< SYNC0 shift time [ns]. */
    uint32_t sync1Cycle, /**< SYNC1 cycle time [ns]. */
    int32_t sync1Shift /**< SYNC1 shift time [ns]. */);
int ecAddSdo(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t value, int byteSize); //size in bytes
int ecWriteSdo(uint16_t slavePposition,uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t value,int byteSize);
uint32_t ecReadSdo(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,int byteSize);
int ecApplyConfig(int masterIndex);
int writeEcEntry(int slaveIndex,int entryIndex,uint64_t value);
int writeEcEntryIDString(int slavePosition,char *entryIDString,uint64_t value);
int readEcEntry(int slaveIndex,int entryIndex,uint64_t *value);
int readEcEntryIDString(int slavePosition,char *entryIDString,uint64_t *value);
int readEcEntryIndexIDString(int slavePosition,char *entryIDString,int *value);
int readEcSlaveIndex(int slavePosition,int *value);
int ecSetDiagnostics(int value);

//Diagnostics
int setDiagAxisIndex(int axisIndex);
int setDiagAxisFreq(int value);
int setDiagAxisEnable(int value);
int setEnableTimeDiag(int value);
int setEnableFunctionCallDiag(int value);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
