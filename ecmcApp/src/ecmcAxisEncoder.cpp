/*
 * cMcuAxisEncoder.cpp
 *
 *  Created on: Mar 15, 2016
 *      Author: anderssandstrom
 */

#include "ecmcAxisEncoder.h"

ecmcAxisEncoder::ecmcAxisEncoder(int axisID, double sampleTime)
{
  initVars();
  axisID_=axisID;
  axisType_=ECMC_AXIS_TYPE_ENCODER;
  sampleTime_=sampleTime;
  enc_=new ecmcEncoder(sampleTime_);
}

ecmcAxisEncoder::~ecmcAxisEncoder()
{
  delete enc_;
}

void ecmcAxisEncoder::initVars()
{
  initDone_=false;
  operationMode_=ECMC_MODE_OP_AUTO;
  sampleTime_=1;
  execute_=false;
  enable_=false;
}

void ecmcAxisEncoder::execute(bool masterOK)
{
  if(masterOK){

    if(inStartupPhase_){
      //Auto reset hardware error
      if(getErrorID()==ERROR_AXIS_HARDWARE_STATUS_NOT_OK){
        errorReset();
      }
      setInStartupPhase(false);
    }

    //Read from hardware
    enc_->readEntries();
  }
  else{
	if(getEnable()){
	  setEnable(false);
	}
    setErrorID(ERROR_AXIS_HARDWARE_STATUS_NOT_OK);
  }
}

int ecmcAxisEncoder::setExecute(bool execute)
{
  execute_=execute;

  //Cascade commands via command transformation
  return setExecute_Transform();
}

bool ecmcAxisEncoder::getExecute()
{
  return execute_;
}

int ecmcAxisEncoder::setEnable(bool enable)
{
  enable_=enable;

  //Cascade commands via command transformation
  return setEnable_Transform();
}

bool ecmcAxisEncoder::getEnable()
{
  return enable_;
}

int ecmcAxisEncoder::getErrorID()
{
  if(enc_->getError()){
    return setErrorID(enc_->getErrorID());
  }
  return ecmcError::getErrorID();
}

bool ecmcAxisEncoder::getError()
{
  bool bErr=enc_->getError();
  if(bErr){
    setError(bErr);
  }
  return ecmcError::getError();
}

int ecmcAxisEncoder::setOpMode(operationMode mode)
{
  operationMode_=mode; //TODO not used
  return 0;
}

operationMode ecmcAxisEncoder::getOpMode()
{
  return operationMode_;  //TODO not used
}

int ecmcAxisEncoder::getActPos(double *pos)
{
  *pos=enc_->getActPos();
  return 0;
}

int ecmcAxisEncoder::getActVel(double *vel)
{

  *vel=enc_->getActVel();
  return 0;
}

int ecmcAxisEncoder::getAxisHomed(bool *homed)
{
  *homed=enc_->getHomed();
  return 0;
}

int ecmcAxisEncoder::getEncScaleNum(double *scale)
{
  *scale=enc_->getScaleNum();
  return 0;
}

int ecmcAxisEncoder::setEncScaleNum(double scale)
{
  enc_->setScaleNum(scale);
  return 0;
}

int ecmcAxisEncoder::getEncScaleDenom(double *scale)
{
  *scale=enc_->getScaleDenom();
  return 0;
}

int ecmcAxisEncoder::setEncScaleDenom(double scale)
{
  enc_->setScaleDenom(scale);
  return 0;
}

int ecmcAxisEncoder::getCntrlError(double* error)
{
  *error=0;
  return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

int ecmcAxisEncoder::getEncPosRaw(int64_t *rawPos)
{
  *rawPos=enc_->getRawPos();
  return 0;
}

int ecmcAxisEncoder::setCommand(motionCommandTypes command)
{
  return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

int ecmcAxisEncoder::setCmdData(int cmdData)
{
  return setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
}

void ecmcAxisEncoder::errorReset()
{
  enc_->errorReset();
  ecmcError::errorReset();
}

motionCommandTypes ecmcAxisEncoder::getCommand()
{
  setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
  return ECMC_CMD_NOCMD;
}

int ecmcAxisEncoder::getCmdData()
{
  setErrorID(ERROR_AXIS_FUNCTION_NOT_SUPPRTED);
  return -1;
}


ecmcEncoder *ecmcAxisEncoder::getEnc()
{
  return enc_;
}

ecmcPIDController *ecmcAxisEncoder::getCntrl()
{
  return NULL;
}

ecmcDrive *ecmcAxisEncoder::getDrv()
{
  return NULL;
}

ecmcTrajectory *ecmcAxisEncoder::getTraj()
{
  return NULL;
}

ecmcMonitor *ecmcAxisEncoder::getMon()
{
  return NULL;
}

ecmcSequencer *ecmcAxisEncoder::getSeq()
{
  return NULL;
}

void ecmcAxisEncoder::printStatus()
{

  printf("%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%i\t%x",
      axisID_,
      0.0,
      enc_->getActPos(),
      0.0,
      0.0,
      0.0,
      enc_->getActVel(),
      0.0,
      0.0,
      0,
      getErrorID());

  printf("\t%d  %d  %d  %d  %d  %d  %d  %d  %d\n",
      getEnable(),
      getExecute(),
      0,
      0,
      0,
      0,
      0,
      0,
      0);
}

int ecmcAxisEncoder::validate()
{
  int errorRet=0;
  if(enc_==NULL){
    return setErrorID(ERROR_AXIS_ENC_OBJECT_NULL);
  }

  errorRet=enc_->validate();
  if(errorRet){
    return setErrorID(errorRet);
  }

  return 0;
}
