/*
 * cMcuAxisEncoder.h
 *
 *  Created on: Mar 15, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCAXISENCODER_H_
#define ECMCAXISENCODER_H_

#include "ecmcAxisBase.h"

class ecmcAxisEncoder: public ecmcAxisBase
{
public:
  ecmcAxisEncoder(int axisID, double sampleTime);
  ~ecmcAxisEncoder();
  void execute(bool masterOK);
  int setOpMode(operationMode mode);
  operationMode getOpMode();
  int getActPos(double *pos);
  int getActVel(double *vel);
  int getAxisHomed(bool *homed);
  int getEncScaleNum(double *scale);
  int setEncScaleNum(double scale);
  int getEncScaleDenom(double *scale);
  int setEncScaleDenom(double scale);
  int getEncPosRaw(int64_t *rawPos);
  int getCntrlError(double* error);
  int setExecute(bool execute);
  bool getExecute();
  int setEnable(bool enable);
  bool getEnable();
  int setCommand(motionCommandTypes command);
  int setCmdData(int cmdData);
  motionCommandTypes getCommand();
  int getCmdData();

  ecmcDriveBase *getDrv();
  ecmcTrajectoryTrapetz *getTraj();
  ecmcMonitor *getMon();
  ecmcEncoder *getEnc();
  ecmcPIDController *getCntrl();
  ecmcSequencer *getSeq();
  void printStatus();

  int validate();

 private:
  void initVars();
  bool initDone_;
  operationMode operationMode_;
  ecmcEncoder *enc_;
  double sampleTime_;
  bool execute_;
  ecmcMasterSlaveData slaveDataInterface_;
};
#endif /* ECMCAXISENCODER_H_ */
