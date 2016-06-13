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
  ~ecmcAxisReal();
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

  ecmcDrive *getDrv();
  ecmcTrajectory *getTraj();
  ecmcMonitor *getMon();
  ecmcEncoder *getEnc();
  ecmcPIDController *getCntrl();
  ecmcSequencer *getSeq();
  void printStatus();
  int validate();
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
