/*
 *
 *  Created on: Oct 4: 2018
 *      Author: anderssandstrom
 */

#include "ecmPLCDataIF.h"

ecmPLCDataIF::ecmPLCDataIF(ecmcAxisBase *axis,std::string axisDataSource)
{
  errorReset();
  initVars();
  axis_=axis;
  source_=ECMC_RECORDER_SOURCE_AXIS;
  dataSourceAxis_=parseAxisDataSource(axisDataSource);
  if(dataSourceAxis_==ECMC_AXIS_DATA_NONE){
    LOGERR("%s/%s:%d: ERROR: Axis data Source %s Undefined  (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,axisDataSource,ERROR_PLC_AXIS_DATA_TYPE_ERROR);
  }
}

ecmPLCDataIF(ecmcEc *ec,std::string ecDataSource)
{
  errorReset();
  initVars();
  ec_=ec;
  source_=ECMC_RECORDER_SOURCE_ETHERCAT;
  int errorCode=parseAndLinkEcDataSource(ecDataSource);
  if(errorCode){
    LOGERR("%s/%s:%d: ERROR: Ec data Source %s Undefined  (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ecDataSource,errorCode);
  }
}

ecmPLCDataIF::~ecmPLCDataIF()
{

}

void ecmPLCDataIF::initVars()
{
  axis_=0;
  ec_=0;
  data_=0;
  dataSourceAxis_=ECMC_AXIS_DATA_NONE;
  source_=ECMC_RECORDER_SOURCE_NONE;
  entry_=0;
}

double& ecmPLCDataIF::getDataRef(){
  return data_;
}

int ecmPLCDataIF::read()
{
 int =0;
 switch(source_){
   ECMC_RECORDER_SOURCE_NONE:
     errorCode=ERROR_PLC_SOURCE_INVALID;
     break;
   ECMC_RECORDER_SOURCE_ETHERCAT:
     errorCode=readEc();
     break;
   ECMC_RECORDER_SOURCE_AXIS:
     errorCode=readAxis();
     break;
 }
 dataOld_=data_;
 return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
}

int ecmPLCDataIF::write()
{
  //Only write if data changed between read and write
  if(data_==dataOld_){
    return 0;
  }

  switch(source_){
    ECMC_RECORDER_SOURCE_NONE:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_SOURCE_INVALID);
      break;
    ECMC_RECORDER_SOURCE_ETHERCAT:
      return writeEc();
      break;
    ECMC_RECORDER_SOURCE_AXIS:
      return writeAxis();
      break;
  }
  return ERROR_PLC_SOURCE_INVALID;
}

int ecmPLCDataIF::readEc()
{
  uint64_t value;
  int errorCode=readEcEntryValue(ECMC_PLC_EC_ENTRY_INDEX,&value);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  data_=(double)value; //Risk of data loss
  return 0;
}

int ecmPLCDataIF::writeEc()
{
  uint64_t value=(uint64_t)data_;
  int errorCode=writeEcEntryValue(int entryIndex,uint64_t value);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  return 0;
}

int ecmPLCDataIF::readAxis()
{
  if(axis_==NULL){
    return ERROR_PLC_AXIS_NULL;
  }

  ecmcAxisStatusType *axisData=axis_->getDebugInfoDataPointer();

  switch(dataSourceAxis_){
    case ECMC_AXIS_DATA_NONE:
      data_=0;
      break;
    case ECMC_AXIS_DATA_AXIS_ID:
      data_=(double)axisData->axisID;
      break;
    case ECMC_AXIS_DATA_POS_SET:
      data_=axisData->onChangeData->positionSetpoint;
      break;
    case ECMC_AXIS_DATA_POS_ACT:
      data_=axisData->onChangeData->positionActual;
      break;
    case ECMC_AXIS_DATA_CNTRL_ERROR:
      data_=axisData->onChangeData->cntrlError;
      break;
    case ECMC_AXIS_DATA_POS_TARGET:
      data_=axisData->onChangeData->positionTarget;
      break;
    case ECMC_AXIS_DATA_POS_ERROR:
      data_=axisData->onChangeData->positionError;
      break;
    case ECMC_AXIS_DATA_POS_RAW:
      data_=(double)axisData->onChangeData->positionRaw;  //Risc of data loss
      break;
    case ECMC_AXIS_DATA_CNTRL_OUT:
      data_=axisData->onChangeData->cntrlOutput;
      break;
    case ECMC_AXIS_DATA_VEL_SET:
      data_=axisData->onChangeData->velocitySetpoint;
      break;
    case ECMC_AXIS_DATA_VEL_ACT:
      data_=axisData->onChangeData->velocityActual;
      break;
    case ECMC_AXIS_DATA_VEL_SET_FF_RAW:
      data_=axisData->onChangeData->velocityFFRaw;
      break;
    case ECMC_AXIS_DATA_VEL_SET_RAW:
      data_=(double)axisData->onChangeData->velocitySetpointRaw;
      break;
    case ECMC_AXIS_DATA_CYCLE_COUNTER:
      data_=(double)axisData->cycleCounter;
      break;
    case ECMC_AXIS_DATA_ERROR:
      data_=(double)axisData->onChangeData->error;
      break;
    case ECMC_AXIS_DATA_COMMAND:
      data_=(double)axisData->onChangeData->command;
      break;
    case ECMC_AXIS_DATA_CMD_DATA:
      data_=(double)axisData->onChangeData->cmdData;
      break;
    case ECMC_AXIS_DATA_SEQ_STATE:
      data_=(double)axisData->onChangeData->seqState;
      break;
    case ECMC_AXIS_DATA_INTERLOCK_TYPE:
      data_=(double)axisData->onChangeData->trajInterlock;
      break;
    case ECMC_AXIS_DATA_TRAJ_SOURCE:
      data_=(double)axisData->onChangeData->trajSource;
      break;
    case ECMC_AXIS_DATA_ENC_SOURCE:
      data_=(double)axisData->onChangeData->encSource;
      break;
    case ECMC_AXIS_DATA_ENABLE:
      data_=(double)axisData->onChangeData->enable;
      break;
    case ECMC_AXIS_DATA_ENABLED:
      data_=(double)axisData->onChangeData->enabled;
      break;
    case ECMC_AXIS_DATA_EXECUTE:
      data_=(double)axisData->onChangeData->execute;
      break;
    case ECMC_AXIS_DATA_BUSY:
      data_=(double)axisData->onChangeData->busy;
      break;
    case ECMC_AXIS_DATA_AT_TARGET:
      data_=(double)axisData->onChangeData->atTarget;
      break;
    case ECMC_AXIS_DATA_HOMED:
      data_=(double)axisData->onChangeData->homed;
      break;
    case ECMC_AXIS_DATA_LIMIT_BWD:
      data_=(double)axisData->onChangeData->limitBwd;
      break;
    case ECMC_AXIS_DATA_LIMIT_FWD:
      data_=(double)axisData->onChangeData->limitFwd;
      break;
    case ECMC_AXIS_DATA_HOME_SWITCH:
      data_=(double)axisData->onChangeData->homeSwitch;
      break;
    case ECMC_AXIS_DATA_RESET:
      data_=(double )axis_->getReset();
      break;
    default:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_AXIS_DATA_TYPE_ERROR);
      break;
  }

  return 0;
}

int ecmPLCDataIF::writeAxis()
{
  if(axis_==NULL){
    return ERROR_PLC_AXIS_AXIS_NULL;
  }

  //Only write commands if changed
  switch(dataSourceAxis_){
    case ECMC_AXIS_DATA_NONE:
      return 0;
      break;
    case ECMC_AXIS_DATA_AXIS_ID:
      return 0;
      break;
    case ECMC_AXIS_DATA_POS_SET:
      return 0;
      break;
    case ECMC_AXIS_DATA_POS_ACT:
      return 0;
      break;
    case ECMC_AXIS_DATA_CNTRL_ERROR:
      return 0;
      break;
    case ECMC_AXIS_DATA_POS_TARGET:
      return axis_->getSeq()->setTargetPos(data_);
      break;
    case ECMC_AXIS_DATA_POS_ERROR:
      return 0;
      break;
    case ECMC_AXIS_DATA_POS_RAW:
      return 0;
      break;
    case ECMC_AXIS_DATA_CNTRL_OUT:
      return 0;
      break;
    case ECMC_AXIS_DATA_VEL_SET:
      return axis_->getSeq()->setTargetVel(data_);
      break;
    case ECMC_AXIS_DATA_VEL_ACT:
      return 0;
      break;
    case ECMC_AXIS_DATA_VEL_SET_FF_RAW:
      return 0;
      break;
    case ECMC_AXIS_DATA_VEL_SET_RAW:
      return 0;
      break;
    case ECMC_AXIS_DATA_CYCLE_COUNTER:
      return 0;
      break;
    case ECMC_AXIS_DATA_ERROR:
      return 0;
      break;
    case ECMC_AXIS_DATA_COMMAND:
      return axis_->setCommand((motionCommandTypes)data_);
      break;
    case ECMC_AXIS_DATA_CMD_DATA:
      return axis_->setCmdData((int)data_);
      break;
    case ECMC_AXIS_DATA_SEQ_STATE:
      return 0;
      break;
    case ECMC_AXIS_DATA_INTERLOCK_TYPE:
      return 0;
      break;
    case ECMC_AXIS_DATA_TRAJ_SOURCE:
      return 0;
      break;
    case ECMC_AXIS_DATA_ENC_SOURCE:
      return 0;
      break;
    case ECMC_AXIS_DATA_ENABLE:
      return axis_->setEnable((bool)data_);
      break;
    case ECMC_AXIS_DATA_ENABLED:
      return 0;
      break;
    case ECMC_AXIS_DATA_EXECUTE:
      return axis_->setExecute((bool)data_);
      break;
    case ECMC_AXIS_DATA_BUSY:
      return 0;
      break;
    case ECMC_AXIS_DATA_AT_TARGET:
      return 0;
      break;
    case ECMC_AXIS_DATA_HOMED:
      return 0;
      break;
    case ECMC_AXIS_DATA_LIMIT_BWD:
      return 0;
      break;
    case ECMC_AXIS_DATA_LIMIT_FWD:
      return 0;
      break;
    case ECMC_AXIS_DATA_HOME_SWITCH:
      return 0;
      break;
    case ECMC_AXIS_DATA_RESET:
      return axis_->setReset((bool)data_);
      break;
    default:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_AXIS_DATA_TYPE_ERROR);
      break;
  }
  return 0;
}

ecmcAxisDataType ecmPLCDataIF::parseAxisDataSource(std::string axisDataSource)
{
  int npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_AXIS_ID);
  if(npos==0){
    return ECMC_AXIS_DATA_AXIS_ID;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_POS_SET);
  if(npos==0){
   return ECMC_AXIS_DATA_POS_SET;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_POS_ACT);
  if(npos==0){
    return ECMC_AXIS_DATA_POS_ACT;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_POS_TARGET);
  if(npos==0){
    return ECMC_AXIS_DATA_POS_TARGET;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_POS_ERROR);
  if(npos==0){
    return ECMC_AXIS_DATA_POS_ERROR;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_POS_RAW);
  if(npos==0){
    return ECMC_AXIS_DATA_POS_RAW;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_CNTRL_OUT);
  if(npos==0){
    return ECMC_AXIS_DATA_CNTRL_OUT;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_VEL_SET);
  if(npos==0){
    return ECMC_AXIS_DATA_VEL_SET;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_VEL_ACT);
  if(npos==0){
    return ECMC_AXIS_DATA_VEL_ACT;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_VEL_SET_FF_RAW);
  if(npos==0){
    return ECMC_AXIS_DATA_VEL_SET_FF_RAW;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_VEL_SET_RAW);
  if(npos==0){
    return ECMC_AXIS_DATA_VEL_SET_RAW;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_CYCLE_COUNTER);
  if(npos==0){
    return ECMC_AXIS_DATA_CYCLE_COUNTER;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_ERROR);
  if(npos==0){
    return ECMC_AXIS_DATA_ERROR;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_COMMAND);
  if(npos==0){
    return ECMC_AXIS_DATA_COMMAND;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_CMD_DATA);
  if(npos==0){
    return ECMC_AXIS_DATA_CMD_DATA;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_SEQ_STATE);
  if(npos==0){
    return ECMC_AXIS_DATA_SEQ_STATE;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_INTERLOCK_TYPE);
  if(npos==0){
    return ECMC_AXIS_DATA_INTERLOCK_TYPE;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_TRAJ_SOURCE);
  if(npos==0){
    return ECMC_AXIS_DATA_TRAJ_SOURCE;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_ENC_SOURCE);
  if(npos==0){
    return ECMC_AXIS_DATA_ENC_SOURCE;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_ENABLE);
  if(npos==0){
    return ECMC_AXIS_DATA_ENABLE;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_ENABLED);
  if(npos==0){
    return ECMC_AXIS_DATA_ENABLED;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_EXECUTE);
  if(npos==0){
    return ECMC_AXIS_DATA_EXECUTE;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_BUSY);
  if(npos==0){
    return ECMC_AXIS_DATA_BUSY;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_AT_TARGET);
  if(npos==0){
    return ECMC_AXIS_DATA_AT_TARGET ;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_HOMED);
  if(npos==0){
    return ECMC_AXIS_DATA_HOMED  ;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_LIMIT_BWD);
  if(npos==0){
    return ECMC_AXIS_DATA_LIMIT_BWD;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_LIMIT_FWD);
  if(npos==0){
    return ECMC_AXIS_DATA_LIMIT_FWD;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_HOME_SWITCH);
  if(npos==0){
    return ECMC_AXIS_DATA_HOME_SWITCH;
  }

  npos=strcmp(axisDataSource,ECMC_AXIS_DATA_STR_RESET);
  if(npos==0){
    return ECMC_AXIS_DATA_RESET;
  }

  return ECMC_AXIS_DATA_NONE;
}

int ecmPLCDataIF::parseAndLinkEcDataSource(std::string ecDataSource)
{
  int masterId=0;
  int slaveId=0;
  int bitId=0;
  int nvals=0;
  char alias[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int errorCode=parseEcPath(ecDataSource.c_str(), &masterId, &slaveId, alias,&bitId);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  if(ec_->getMasterIndex()!=masterId){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_EC_MASTER_INVALID);
  }

  ecmcEcSlave *slave=NULL;
  if(slaveIndex>=0){
    slave=ec.findSlave(slaveId);
  }
  else{
    slave=ec.getSlave(slaveId);
  }

  if(slave==NULL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_EC_SLAVE_NULL);
  }

  std::string sEntryID=alias;

  ecmcEcEntry *entry=slave->findEntry(sEntryID);

  if(entry==NULL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_EC_ENTRY_NULL);
  }

  errorCode=setEntryAtIndex(entry,ECMC_PLC_EC_ENTRY_INDEX,bitId);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  return 0;
}

int ecmPLCDataIF::parseEcPath(char* ecPath, int *master,int *slave, char*alias,int *bit)
{
  int masterId=0;
  int slaveId=0;
  int bitId=0;
  int nvals=0;

  nvals = sscanf(ecPath, "ec%d.s%d.%[^.].%d",&masterId,&slaveId,alias,&bitId);
  if (nvals == 4){
	*master=masterId;
	*slave=slaveId;
	*bit=bitId;
    return 0;
  }
  nvals = sscanf(ecPath, "ec%d.s%d.%s",&masterId,&slaveId,alias);
  if (nvals == 3){
	*master=masterId;
	*slave=slaveId;
	*bit=-1;
    return 0;
  }
  return ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR;
}
