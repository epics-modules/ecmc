/*
 *
 *  Created on: Oct 4: 2018
 *      Author: anderssandstrom
 */

#include "ecmcPLCDataIF.h"

ecmcPLCDataIF::ecmcPLCDataIF(ecmcAxisBase *axis,char *axisVarName)
{
  errorReset();
  initVars();
  axis_=axis;
  varName_=axisVarName;
  source_=ECMC_RECORDER_SOURCE_AXIS;
  dataSourceAxis_=parseAxisDataSource(axisVarName);
  if(dataSourceAxis_==ECMC_AXIS_DATA_NONE){
    LOGERR("%s/%s:%d: ERROR: Axis data Source Undefined  (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_PLC_AXIS_DATA_TYPE_ERROR);
  }
}

ecmcPLCDataIF::ecmcPLCDataIF(ecmcEc *ec,char *ecVarName)
{
  errorReset();
  initVars();
  ec_=ec;
  varName_=ecVarName;
  source_=ECMC_RECORDER_SOURCE_ETHERCAT;
  parseAndLinkEcDataSource(ecVarName);
}

ecmcPLCDataIF::ecmcPLCDataIF(char *varName,ecmcDataSourceType dataSource)
{
  errorReset();
  initVars();
  varName_=varName;
  source_=dataSource;
}

ecmcPLCDataIF::~ecmcPLCDataIF()
{

}

void ecmcPLCDataIF::initVars()
{
  axis_=0;
  ec_=0;
  data_=0;
  varName_="";
  dataSourceAxis_=ECMC_AXIS_DATA_NONE;
  source_=ECMC_RECORDER_SOURCE_NONE;
  readOnly_=0;
}

double& ecmcPLCDataIF::getDataRef(){
  return data_;
}

int ecmcPLCDataIF::read()
{
 int errorCode=0;
 switch(source_){
   case ECMC_RECORDER_SOURCE_NONE:
     errorCode=ERROR_PLC_SOURCE_INVALID;
     break;
   case ECMC_RECORDER_SOURCE_ETHERCAT:
     errorCode=readEc();
     break;
   case ECMC_RECORDER_SOURCE_AXIS:
     errorCode=readAxis();
     break;
   case ECMC_RECORDER_SOURCE_STATIC_VAR:
     return 0;
     break;
   case ECMC_RECORDER_SOURCE_GLOBAL_VAR:
     return 0;
     break;

 }
 dataRead_=data_;
 return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
}

int ecmcPLCDataIF::write()
{
  //Only write if data changed between read and write
  if(data_==dataRead_ || readOnly_){
    return 0;
  }
  switch(source_){
    case ECMC_RECORDER_SOURCE_NONE:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_SOURCE_INVALID);
      break;
    case ECMC_RECORDER_SOURCE_ETHERCAT:
      return writeEc();
      break;
    case ECMC_RECORDER_SOURCE_AXIS:
      return writeAxis();
      break;
    case ECMC_RECORDER_SOURCE_STATIC_VAR:
      return 0;
      break;
   case ECMC_RECORDER_SOURCE_GLOBAL_VAR:
      return 0;
      break;
  }
  return ERROR_PLC_SOURCE_INVALID;
}

int ecmcPLCDataIF::readEc()
{
  uint64_t value;
  int errorCode=readEcEntryValue(ECMC_PLC_EC_ENTRY_INDEX,&value);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  data_=(double)value; //Risk of data loss
  return 0;
}

int ecmcPLCDataIF::writeEc()
{
  uint64_t value=(uint64_t)data_;
  int errorCode=writeEcEntryValue(ECMC_PLC_EC_ENTRY_INDEX,value);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  return 0;
}

int ecmcPLCDataIF::readAxis()
{
  if(axis_==NULL){
    return ERROR_PLC_AXIS_NULL;
  }

  if(axis_->getTraj()==NULL){
    return ERROR_PLC_TRAJ_NULL;
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
      data_=axisData->onChangeData.positionSetpoint;
      break;
    case ECMC_AXIS_DATA_POS_ACT:
      data_=axisData->onChangeData.positionActual;
      break;
    case ECMC_AXIS_DATA_CNTRL_ERROR:
      data_=axisData->onChangeData.cntrlError;
      break;
    case ECMC_AXIS_DATA_POS_TARGET:
      data_=axisData->onChangeData.positionTarget;
      break;
    case ECMC_AXIS_DATA_POS_ERROR:
      data_=axisData->onChangeData.positionError;
      break;
    case ECMC_AXIS_DATA_POS_RAW:
      data_=(double)axisData->onChangeData.positionRaw;  //Risc of data loss
      break;
    case ECMC_AXIS_DATA_CNTRL_OUT:
      data_=axisData->onChangeData.cntrlOutput;
      break;
    case ECMC_AXIS_DATA_VEL_SET:
      data_=axisData->onChangeData.velocitySetpoint;
      break;
    case ECMC_AXIS_DATA_VEL_ACT:
      data_=axisData->onChangeData.velocityActual;
      break;
    case ECMC_AXIS_DATA_VEL_SET_FF_RAW:
      data_=axisData->onChangeData.velocityFFRaw;
      break;
    case ECMC_AXIS_DATA_VEL_SET_RAW:
      data_=(double)axisData->onChangeData.velocitySetpointRaw;
      break;
    case ECMC_AXIS_DATA_CYCLE_COUNTER:
      data_=(double)axisData->cycleCounter;
      break;
    case ECMC_AXIS_DATA_ERROR:
      data_=(double)axisData->onChangeData.error;
      break;
    case ECMC_AXIS_DATA_COMMAND:
      data_=(double)axisData->onChangeData.command;
      break;
    case ECMC_AXIS_DATA_CMD_DATA:
      data_=(double)axisData->onChangeData.cmdData;
      break;
    case ECMC_AXIS_DATA_SEQ_STATE:
      data_=(double)axisData->onChangeData.seqState;
      break;
    case ECMC_AXIS_DATA_INTERLOCK_TYPE:      
      data_=(double)axisData->onChangeData.trajInterlock==0;
      break;
    case ECMC_AXIS_DATA_TRAJ_SOURCE:
      data_=(double)axisData->onChangeData.trajSource;
      break;
    case ECMC_AXIS_DATA_ENC_SOURCE:
      data_=(double)axisData->onChangeData.encSource;
      break;
    case ECMC_AXIS_DATA_ENABLE:
      data_=(double)axisData->onChangeData.enable;
      break;
    case ECMC_AXIS_DATA_ENABLED:
      data_=(double)axisData->onChangeData.enabled;
      break;
    case ECMC_AXIS_DATA_EXECUTE:
      data_=(double)axisData->onChangeData.execute;
      break;
    case ECMC_AXIS_DATA_BUSY:
      data_=(double)axisData->onChangeData.busy;
      break;
    case ECMC_AXIS_DATA_AT_TARGET:
      data_=(double)axisData->onChangeData.atTarget;
      break;
    case ECMC_AXIS_DATA_HOMED:
      data_=(double)axisData->onChangeData.homed;
      break;
    case ECMC_AXIS_DATA_LIMIT_BWD:
      data_=(double)axisData->onChangeData.limitBwd;
      break;
    case ECMC_AXIS_DATA_LIMIT_FWD:
      data_=(double)axisData->onChangeData.limitFwd;
      break;
    case ECMC_AXIS_DATA_HOME_SWITCH:
      data_=(double)axisData->onChangeData.homeSwitch;
      break;
    case ECMC_AXIS_DATA_RESET:
      data_=0;
      break;
    case ECMC_AXIS_DATA_VEL_TARGET_SET:
      data_=(double )axis_->getSeq()->getTargetVel();
      break;
      break;
    case ECMC_AXIS_DATA_ACC_TARGET_SET:
      data_=(double ) axis_->getTraj()->getAcc();
      break;
    case ECMC_AXIS_DATA_DEC_TARGET_SET:
      data_=(double )axis_->getTraj()->getDec();
      break;
    case ECMC_AXIS_DATA_SOFT_LIMIT_BWD:
      data_=axis_->getMon()->getSoftLimitBwd();
      break;
    case ECMC_AXIS_DATA_SOFT_LIMIT_FWD:
      data_=axis_->getMon()->getSoftLimitFwd();
      break;
    case ECMC_AXIS_DATA_SOFT_LIMIT_BWD_ENABLE:
      data_=(bool)axis_->getMon()->getEnableSoftLimitBwd();
      break;
    case ECMC_AXIS_DATA_SOFT_LIMIT_FWD_ENABLE:
      data_=(bool)axis_->getMon()->getEnableSoftLimitFwd();
      break;
    case ECMC_AXIS_DATA_TRAJ_DIRECTION:      
      switch(axis_->getAxisSetDirection()){
        case ECMC_DIR_BACKWARD:
          data_=-1;
          break;
        case ECMC_DIR_FORWARD:
          data_=1;
          break;
        case ECMC_DIR_STANDSTILL:
          data_=0;
          break;
      }
      break;

    default:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_AXIS_DATA_TYPE_ERROR);
      break;
  }

  return 0;
}

int ecmcPLCDataIF::writeAxis()
{
  if(axis_==NULL){
    return ERROR_PLC_AXIS_NULL;
  }

  if(axis_->getTraj()==NULL){
    return ERROR_PLC_TRAJ_NULL;
  }

  if(axis_->getMon()==NULL){
    return ERROR_PLC_MON_NULL;
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
      axis_->getSeq()->setTargetPos(data_);
      return 0;
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
      return 0;
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
      return axis_->getMon()->setPLCInterlock(data_==0);
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
      if(data_){
        axis_->errorReset();
      }
      return 0;
      break;
    case ECMC_AXIS_DATA_VEL_TARGET_SET:
      axis_->getSeq()->setTargetVel(data_);
      break;
    case ECMC_AXIS_DATA_ACC_TARGET_SET:
      axis_->getTraj()->setAcc(data_);
      break;
    case ECMC_AXIS_DATA_DEC_TARGET_SET:
      axis_->getTraj()->setDec(data_);
      break;
    case ECMC_AXIS_DATA_SOFT_LIMIT_BWD:
      axis_->getMon()->setSoftLimitBwd(data_);
      break;
    case ECMC_AXIS_DATA_SOFT_LIMIT_FWD:
      axis_->getMon()->setSoftLimitFwd(data_);
      break;
    case ECMC_AXIS_DATA_SOFT_LIMIT_BWD_ENABLE:
      axis_->getMon()->setEnableSoftLimitBwd((bool)data_);
      break;
    case ECMC_AXIS_DATA_SOFT_LIMIT_FWD_ENABLE:
      axis_->getMon()->setEnableSoftLimitFwd((bool)data_);
      break;
    case ECMC_AXIS_DATA_TRAJ_DIRECTION:
      return 0;
      break;
    default:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_AXIS_DATA_TYPE_ERROR);
      break;
  }
  return 0;
}

ecmcAxisDataType ecmcPLCDataIF::parseAxisDataSource(char * axisDataSource)
{
  char *varName;

  varName=strstr(axisDataSource,ECMC_AX_STR);
  if(!varName){
    return ECMC_AXIS_DATA_NONE;
  }
  varName=strstr(axisDataSource,".");
  if(!varName){
    return ECMC_AXIS_DATA_NONE;
  }
  varName++;

  int npos=strcmp(varName,ECMC_AXIS_DATA_STR_AXIS_ID);
  if(npos==0){
    return ECMC_AXIS_DATA_AXIS_ID;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_POS_SET);
  if(npos==0){
   return ECMC_AXIS_DATA_POS_SET;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_POS_ACT);
  if(npos==0){
    return ECMC_AXIS_DATA_POS_ACT;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_POS_TARGET);
  if(npos==0){
    return ECMC_AXIS_DATA_POS_TARGET;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_POS_ERROR);
  if(npos==0){
    return ECMC_AXIS_DATA_POS_ERROR;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_POS_RAW);
  if(npos==0){
    return ECMC_AXIS_DATA_POS_RAW;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_CNTRL_OUT);
  if(npos==0){
    return ECMC_AXIS_DATA_CNTRL_OUT;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_VEL_SET);
  if(npos==0){
    return ECMC_AXIS_DATA_VEL_SET;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_VEL_ACT);
  if(npos==0){
    return ECMC_AXIS_DATA_VEL_ACT;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_VEL_SET_FF_RAW);
  if(npos==0){
    return ECMC_AXIS_DATA_VEL_SET_FF_RAW;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_VEL_SET_RAW);
  if(npos==0){
    return ECMC_AXIS_DATA_VEL_SET_RAW;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_CYCLE_COUNTER);
  if(npos==0){
    return ECMC_AXIS_DATA_CYCLE_COUNTER;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_ERROR);
  if(npos==0){
    return ECMC_AXIS_DATA_ERROR;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_COMMAND);
  if(npos==0){
    return ECMC_AXIS_DATA_COMMAND;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_CMD_DATA);
  if(npos==0){
    return ECMC_AXIS_DATA_CMD_DATA;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_SEQ_STATE);
  if(npos==0){
    return ECMC_AXIS_DATA_SEQ_STATE;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_INTERLOCK_TYPE);
  if(npos==0){
    return ECMC_AXIS_DATA_INTERLOCK_TYPE;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_TRAJ_SOURCE);
  if(npos==0){
    return ECMC_AXIS_DATA_TRAJ_SOURCE;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_ENC_SOURCE);
  if(npos==0){
    return ECMC_AXIS_DATA_ENC_SOURCE;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_ENABLE);
  if(npos==0){
    return ECMC_AXIS_DATA_ENABLE;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_ENABLED);
  if(npos==0){
    return ECMC_AXIS_DATA_ENABLED;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_EXECUTE);
  if(npos==0){
    return ECMC_AXIS_DATA_EXECUTE;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_BUSY);
  if(npos==0){
    return ECMC_AXIS_DATA_BUSY;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_AT_TARGET);
  if(npos==0){
    return ECMC_AXIS_DATA_AT_TARGET ;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_HOMED);
  if(npos==0){
    return ECMC_AXIS_DATA_HOMED  ;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_LIMIT_BWD);
  if(npos==0){
    return ECMC_AXIS_DATA_LIMIT_BWD;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_LIMIT_FWD);
  if(npos==0){
    return ECMC_AXIS_DATA_LIMIT_FWD;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_HOME_SWITCH);
  if(npos==0){
    return ECMC_AXIS_DATA_HOME_SWITCH;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_RESET);
  if(npos==0){
    return ECMC_AXIS_DATA_RESET;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_VEL_TARGET_SET);
  if(npos==0){
    return ECMC_AXIS_DATA_VEL_TARGET_SET;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_ACC_TARGET_SET);
  if(npos==0){
    return ECMC_AXIS_DATA_ACC_TARGET_SET;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_DEC_TARGET_SET);
  if(npos==0){
    return ECMC_AXIS_DATA_DEC_TARGET_SET;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_SOFT_LIMIT_BWD);
  if(npos==0){
    return ECMC_AXIS_DATA_SOFT_LIMIT_BWD;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_SOFT_LIMIT_FWD);
  if(npos==0){
    return ECMC_AXIS_DATA_SOFT_LIMIT_FWD;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_SOFT_LIMIT_BWD_ENABLE);
  if(npos==0){
    return ECMC_AXIS_DATA_SOFT_LIMIT_BWD_ENABLE;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_SOFT_LIMIT_FWD_ENABLE);
  if(npos==0){
    return ECMC_AXIS_DATA_SOFT_LIMIT_FWD_ENABLE;
  }

  npos=strcmp(varName,ECMC_AXIS_DATA_STR_TRAJ_DIRECTION);
  if(npos==0){
    return ECMC_AXIS_DATA_TRAJ_DIRECTION;
  }

  return ECMC_AXIS_DATA_NONE;
}

int ecmcPLCDataIF::parseAndLinkEcDataSource(char* ecDataSource)
{
  int masterId=0;
  int slaveId=0;
  int bitId=0;
  char alias[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int errorCode=parseEcPath(ecDataSource, &masterId, &slaveId, alias,&bitId);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  if(ec_->getMasterIndex()!=masterId){
    LOGERR("%s/%s:%d: ERROR: Master %s not configured (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ecDataSource,ERROR_PLC_EC_MASTER_INVALID);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_EC_MASTER_INVALID);
  }

  ecmcEcSlave *slave=NULL;
  if(slaveId>=0){
    slave=ec_->findSlave(slaveId);
  }
  else{
    slave=ec_->getSlave(slaveId);
  }

  if(slave==NULL){
    LOGERR("%s/%s:%d: ERROR: Slave %s not configured (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ecDataSource,ERROR_PLC_EC_SLAVE_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_EC_SLAVE_NULL);
  }

  std::string sEntryID=alias;

  ecmcEcEntry *entry=slave->findEntry(sEntryID);

  if(entry==NULL){
    LOGERR("%s/%s:%d: ERROR: Entry %s not configured (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ecDataSource,ERROR_PLC_EC_ENTRY_NULL);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_EC_ENTRY_NULL);
  }

  errorCode=setEntryAtIndex(entry,ECMC_PLC_EC_ENTRY_INDEX,bitId);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  return validateEntry(ECMC_PLC_EC_ENTRY_INDEX);
}

int ecmcPLCDataIF::parseEcPath(char* ecPath, int *master,int *slave, char*alias,int *bit)
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
  return ERROR_PLC_EC_VAR_NAME_INVALID;
}

const char *ecmcPLCDataIF::getVarName()
{
  return varName_.c_str();
}

int ecmcPLCDataIF::validate()
{
  if(getErrorID()){
    return getErrorID();
  }

  switch(source_){
    case ECMC_RECORDER_SOURCE_NONE:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_SOURCE_INVALID);
      break;
    case ECMC_RECORDER_SOURCE_ETHERCAT:
      if(!ec_){
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_EC_NULL);
      }
      else{
	      return 0;
      }
      break;
    case ECMC_RECORDER_SOURCE_AXIS:
      if(!axis_){
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_AXIS_NULL);
      }
      else{
	      return 0;
      }
      break;
    case ECMC_RECORDER_SOURCE_STATIC_VAR:
      return 0;
      break;
    case ECMC_RECORDER_SOURCE_GLOBAL_VAR:
      return 0;
      break;
  }
  return ERROR_PLC_SOURCE_INVALID;
}

double ecmcPLCDataIF::getData()
{
  return data_;
}

void ecmcPLCDataIF::setData(double data)
{
  data_=data;
}

int ecmcPLCDataIF::setReadOnly(int readOnly)
{
  readOnly_=readOnly;
  return 0;
}