/*
 * cMcuAxisReal.h
 *
 *  Created on: Mar 10, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCAXISREAL_H_
#define ECMCAXISREAL_H_

#include <stdint.h>
#include "ecmcAxisBase.h"
#include "ecmcEncoder.h"


class ecmcAxisReal: public ecmcAxisBase
{
public:
  ecmcAxisReal(int axisID, double sampleTime);
  ~ecmcAxisReal() override;
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

  ecmcDrive *getDrv() override;
  ecmcTrajectory *getTraj() override;
  ecmcMonitor *getMon() override;
  ecmcEncoder *getEnc() override;
  ecmcPIDController *getCntrl() override;
  ecmcSequencer *getSeq() override;
  void printStatus() override;
  int validate() override;
private:
  void initVars();
  bool _bInitDone;
  operationMode operationMode_;
  ecmcTrajectory *traj_;
  ecmcMonitor *mon_;
  ecmcEncoder *enc_;
  ecmcDrive *drv_;
  ecmcPIDController *cntrl_;
  ecmcSequencer seq_;
  double sampleTime_;
};

#endif /* ECMCAXISREAL_H_ */
