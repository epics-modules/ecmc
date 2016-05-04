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
#include "ecmcDrive.hpp"
#include "ecmcEncoder.h"
#include "ecmcError.h"
#include "ecmcErrorsList.h"
#include "ecmcMonitor.hpp"
#include "ecmcPIDController.hpp"
#include "ecmcSequencer.hpp"
#include "ecmcTrajectory.hpp"

class ecmcAxisBase : public ecmcError
{
public:
  ecmcAxisBase();
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

  virtual ecmcDrive *getDrv()=0;
  virtual ecmcTrajectory *getTraj()=0;
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
protected:
  void initVars();
  int fillCommandsTransformData();
  bool checkAxesForEnabledTransfromCommands(commandType type);
  int setEnable_Transform();
  int setExecute_Transform();
  int axisID_;
  bool reset_;
  axisType axisType_;
  bool cascadedCommandsEnable_;  // Allow other axis to enable and execute this axis
  bool enableCommandTransform_;  // Allow other axis to enable and execute this axis
  ecmcCommandTransform *commandTransform_;
  ecmcAxisBase *axes_[MAX_AXES];
};

#endif /* ECMCAXISBASE_H_ */
