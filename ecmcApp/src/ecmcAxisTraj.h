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
  ~ecmcAxisTraj() override;
  void execute(bool masterOK) override;
  int setOpMode(operationMode mode) override;
  operationMode getOpMode() override;

  int getActPos(double *pos) override;
  int getActVel(double *vel) override;
  int getAxisHomed(bool *homed) override;

  int getEncScaleNum(double *scale) override;
  int setEncScaleNum(double scale) override;
  int getEncScaleDenom(double *scale) override;
  int setEncScaleDenom(double scale) override;
  int getEncPosRaw(int64_t *rawPos) override;
  int getCntrlError(double* error) override;

  int setExecute(bool execute) override;
  bool getExecute() override;
  int setEnable(bool enable) override;
  bool getEnable() override;
  void errorReset() override;
  int getErrorID() override;
  bool getError() override;
  int setCommand(motionCommandTypes command) override;
  int setCmdData(int cmdData) override;
  motionCommandTypes getCommand() override;
  int getCmdData() override;
  void printStatus() override;

  ecmcDrive *getDrv() override;
  ecmcTrajectory *getTraj() override;
  ecmcMonitor *getMon() override;
  ecmcEncoder *getEnc() override;
  ecmcPIDController *getCntrl() override;
  ecmcSequencer *getSeq() override;
  int validate() override;

private:
  void initVars();
  bool initDone_;
  ecmcTrajectory *traj_;
  ecmcMonitor *mon_;
  ecmcSequencer seq_;
  double sampleTime_;
};

#endif /* ECMCAXISTRAJ_H_ */
