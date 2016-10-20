/*
 * cMcuAxisTraj.h
 *
 *  Created on: Mar 17, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCAXISTRAJ_H_
#define ECMCAXISTRAJ_H_

#include "ecmcAxisBase.h"

class ecmcAxisTraj : public ecmcAxisBase
{
public:
ecmcAxisTraj(int nAxisID, double sampleTime);
  ~ecmcAxisTraj();
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
  void errorReset();
  int getErrorID();
  bool getError();
  int setCommand(motionCommandTypes command);
  int setCmdData(int cmdData);
  motionCommandTypes getCommand();
  int getCmdData();
  void printStatus();

  ecmcDriveBase *getDrv();
  ecmcTrajectory *getTraj();
  ecmcMonitor *getMon();
  ecmcEncoder *getEnc();
  ecmcPIDController *getCntrl();
  ecmcSequencer *getSeq();
  int validate();

private:
  void initVars();
  bool initDone_;
  ecmcTrajectory *traj_;
  ecmcMonitor *mon_;
  ecmcSequencer seq_;
  double sampleTime_;
};

#endif /* ECMCAXISTRAJ_H_ */
