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
  ~ecmcAxisEncoder() override;
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
  bool initDone_;
  operationMode operationMode_;
  ecmcEncoder *enc_;
  double sampleTime_;
  bool execute_;
  bool enable_;
  ecmcMasterSlaveData slaveDataInterface_;
};
#endif /* ECMCAXISENCODER_H_ */
