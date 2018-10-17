/*
 *
 *  Created on: Oct 15, 2018
 *      Author: anderssandstrom
 */

#include "ecmcPLCs.h"

ecmcPLCs::ecmcPLCs(ecmcEc *ec)
{
  initVars();
  ec_=ec;
}

ecmcPLCs::~ecmcPLCs()
{
  for(int i=0;i<ECMC_MAX_PLCS;i++){
    delete plcs_[i];
  }
}

void ecmcPLCs::initVars()
{
  globalVariableCount_=0;
  for(int i=0; i<ECMC_MAX_PLCS;i++){
    plcs_[i]=NULL;
    plcEnable_[i]=NULL;
    plcError_[i]=NULL;    
  }
  
  for(int i=0; i<ECMC_MAX_AXES;i++){
    axes_[i]=NULL;
  }

  for(int i=0; i<ECMC_MAX_PLC_VARIABLES;i++){
    globalDataArray_[i]=0;    
  }

  ec_=NULL;
}

int ecmcPLCs::createPLC(int plcIndex, int skipCycles)
{
  if(plcIndex<0 || plcIndex>=ECMC_MAX_PLCS){
    return ERROR_PLCS_INDEX_OUT_OF_RANGE;
  }

  if(!ec_->getInitDone())
    return ERROR_PLCS_EC_NOT_INITIALIZED;

  if(plcs_[plcIndex]){
    delete plcs_[plcIndex];
    plcs_[plcIndex]=NULL;
  }

  plcs_[plcIndex]=new ecmcPLC(skipCycles);
  int errorCode=plcs_[plcIndex]->getErrorID();
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  errorCode=addPLCDefaultVariables(plcIndex,skipCycles);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  return 0;
}

int ecmcPLCs::setAxisArrayPointer(ecmcAxisBase *axis,int index)
{
  if(index>=ECMC_MAX_AXES || index<0){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLCS_AXIS_INDEX_OUT_OF_RANGE);
  }
  axes_[index]=axis;
  return 0;
}

int ecmcPLCs::validate()
{
  //Set scantime
  int errorCode=updateAllScanTimeVars();
  if(errorCode){
    return errorCode;
  }

  //Validate
  for(int i=0; i<ECMC_MAX_PLCS;i++){
    if(plcs_[i]!=NULL){
      int errorCode=plcs_[i]->validate();
      if(errorCode){
        return errorCode;
      }
    }
  }
  return 0;
}

int ecmcPLCs::execute(bool ecOK)
{
  for(int plcIndex=0;plcIndex<ECMC_MAX_PLCS;plcIndex++){
    if(plcs_[plcIndex]!=NULL){

      // Update data from global sources
      for(int i=0; i<globalVariableCount_;i++){
        if(globalDataArray_[i]){
          globalDataArray_[i]->read();
        }
      }

      if(plcEnable_[plcIndex]){
        if(plcEnable_[plcIndex]->getData()){
          plcs_[plcIndex]->execute(ecOK);
        }
      }
      
      // Update changed global data
      for(int i=0; i<globalVariableCount_;i++){
        if(globalDataArray_[i]){
          globalDataArray_[i]->write();
        }
      }
    }  
  }
  return 0;
}

int ecmcPLCs::getExpr(int plcIndex, std::string *expr)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  expr=plcs_[plcIndex]->getExpr();
  return 0;
}

int ecmcPLCs::setExpr(int plcIndex,char *expr)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  plcs_[plcIndex]->clearExpr();
  int errorCode=addExprLine(plcIndex,expr);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  return compileExpr(plcIndex);
}

int ecmcPLCs::addExprLine(int plcIndex,char *expr)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  int errorCode=parseExpr(plcIndex,expr);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  return plcs_[plcIndex]->addExprLine(expr);
}

int ecmcPLCs::clearExpr(int plcIndex)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  return plcs_[plcIndex]->clearExpr();
}

int ecmcPLCs::compileExpr(int plcIndex)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex)
  return plcs_[plcIndex]->compile();
}

int ecmcPLCs::setEnable(int plcIndex,int enable)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  plcEnable_[plcIndex]->setData((double)enable);
  return 0;
}

int ecmcPLCs::getEnable(int plcIndex,int *enabled)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  *enabled=(int)plcEnable_[plcIndex]->getData();
  return 0;
}

int ecmcPLCs::deletePLC(int plcIndex)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  delete plcs_[plcIndex];
  return 0;
}

int ecmcPLCs::getCompiled(int plcIndex,int *compiled)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);
  *compiled=plcs_[plcIndex]->getCompiled();
  return 0;
}

int ecmcPLCs::parseExpr(int plcIndex,char * exprStr)
{
  CHECK_PLC_RETURN_IF_ERROR(plcIndex);

  if(strlen(exprStr)>=EC_MAX_OBJECT_PATH_CHAR_LENGTH-1){
    LOGERR("%s/%s:%d: ERROR: Expression to long (0x%x).\n",__FILE__, __FUNCTION__, __LINE__,ERROR_PLC_EXPR_LINE_TO_LONG);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_EXPR_LINE_TO_LONG);
  }

  //Axis (global)
  int errorCode=parseAxis(plcIndex,exprStr);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  //EtherCAT (global)
  errorCode=parseEC(plcIndex,exprStr);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  // Static  (local)
  errorCode=parseStatic(plcIndex,exprStr);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  // Global
  errorCode=parseGlobal(plcIndex,exprStr);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  // PLC
  errorCode=parsePLC(plcIndex,exprStr);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  return 0;
}

int ecmcPLCs::findGlobalDataIF(char * varName, ecmcPLCDataIF **outDataIF)
{
  for(int i=0; i<globalVariableCount_;i++){
    if(globalDataArray_[i]){
      int n =strcmp(varName,globalDataArray_[i]->getVarName());
      if(n==0){
        *outDataIF=globalDataArray_[i];
        return 0;
      }
    }
  }
  return 0;
}

int ecmcPLCs::createNewGlobalDataIF(char * varName,ecmcDataSourceType dataSource,ecmcPLCDataIF **outDataIF)
{
  //Axes data
  //Ec data
  //Global data 
  if(globalVariableCount_>=ECMC_MAX_PLC_VARIABLES-1 || globalVariableCount_<0){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_VARIABLE_COUNT_EXCEEDED);
  }

  int axisId=-1;
  switch(dataSource){
    case ECMC_RECORDER_SOURCE_NONE:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_SOURCE_INVALID);
      break;

    case ECMC_RECORDER_SOURCE_ETHERCAT:
      if(!ec_){
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_EC_NULL);
      }
	    globalDataArray_[globalVariableCount_]=new ecmcPLCDataIF(ec_,varName); 
      *outDataIF=globalDataArray_[globalVariableCount_];
      globalVariableCount_++;      
      break;

    case ECMC_RECORDER_SOURCE_AXIS:
      axisId=getAxisIndex(varName);
      if(axisId>=ECMC_MAX_AXES || axisId<0){
	      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_AXIS_ID_OUT_OF_RANGE);
      }
      if(!axes_[axisId]){
	      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_AXIS_ID_OUT_OF_RANGE);
      }
	    globalDataArray_[globalVariableCount_]=new ecmcPLCDataIF(axes_[axisId],varName); 
      *outDataIF=globalDataArray_[globalVariableCount_];
      globalVariableCount_++;      
      break;

    case ECMC_RECORDER_SOURCE_STATIC_VAR:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_SOURCE_INVALID);    
      break;
    
    case ECMC_RECORDER_SOURCE_GLOBAL_VAR:
      globalDataArray_[globalVariableCount_]=new ecmcPLCDataIF(varName,ECMC_RECORDER_SOURCE_GLOBAL_VAR); 
      *outDataIF=globalDataArray_[globalVariableCount_];
      globalVariableCount_++;      
      break;
  }
  return 0;
}

int ecmcPLCs::getAxisIndex(char *varName)
{
  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int axisId=0;
  int nvals = sscanf(varName,ECMC_AX_STR"%d.%[0-9a-zA-Z._]",&axisId,buffer);
  if (nvals == 2){
     return axisId;
  }
  return -1;
}

/*
 * find and register all axis variables in string
 */
int ecmcPLCs::parseAxis(int plcIndex,char * exprStr)
{
  //find  and register all axis variables in string
  int nvals=0;
  int axisId;
  int errorCode=0;
  char *strAxis=exprStr;
  char varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  while((strAxis=strstr(strAxis,ECMC_AX_STR)) && strlen(strAxis)>0){
    // Sanity check
    nvals = sscanf(strAxis,ECMC_AX_STR"%d.%[0-9a-zA-Z._]",&axisId,varName);
    if (nvals == 2){
      if(axisId>=ECMC_MAX_AXES || axisId<0){
	      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_AXIS_ID_OUT_OF_RANGE);
      }
      if(!axes_[axisId]){
	      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_AXIS_ID_OUT_OF_RANGE);
      }
      varName[0]='\0';
      nvals = sscanf(strAxis,"%[0-9a-zA-Z._]",varName);
      if (nvals == 1){
        errorCode=createAndRegisterNewDataIF(plcIndex,varName,ECMC_RECORDER_SOURCE_AXIS);
        if(errorCode){
          return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
        }
      }
    }
    strAxis++;
  }
  return 0;
}

/*
 * find and register all ec variables in string
 */
int ecmcPLCs::parseEC(int plcIndex,char * exprStr)
{
  int ecId;
  char *strEc=exprStr;
  char varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  varName[0]='\0';
  while((strEc=strstr(strEc,ECMC_EC_STR)) && strlen(strEc)>0){
    //Sanity check
    int nvals = sscanf(strEc, ECMC_EC_STR"%d.%[0-9a-zA-Z._]",&ecId,varName);
    if (nvals == 2){
      varName[0]='\0';
      nvals = sscanf(strEc,"%[0-9a-zA-Z._]",varName);
      if (nvals == 1){        
        int errorCode=createAndRegisterNewDataIF(plcIndex,varName,ECMC_RECORDER_SOURCE_ETHERCAT);
        if(errorCode){
          return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
        }
      }
    }
    strEc++;
  }
  return 0;
}

int ecmcPLCs::parseStatic(int plcIndex,char * exprStr)
{
  //find static variable
  char *strStatic=exprStr;
  char varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  varName[0]='\0';
  while((strStatic=strstr(strStatic,ECMC_STATIC_VAR)) && strlen(strStatic)>0){
    //Sanity check
    int nvals = sscanf(strStatic,"%[0-9a-zA-Z._]",varName);
    if (nvals == 1){
      int errorCode=plcs_[plcIndex]->addAndRegisterLocalVar(varName);
      if(errorCode){
	      return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
      }
    }
    strStatic++;
  }
  return 0;
}

int ecmcPLCs::parseGlobal(int plcIndex,char * exprStr)
{
  //find global variable
  char *strGlobal=exprStr;
  char varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  varName[0]='\0';
  while((strGlobal=strstr(strGlobal,ECMC_GLOBAL_VAR)) && strlen(strGlobal)>0){
    //Sanity check
    int nvals = sscanf(strGlobal,"%[0-9a-zA-Z._]",varName);
    if (nvals == 1){
      int errorCode=createAndRegisterNewDataIF(plcIndex,varName,ECMC_RECORDER_SOURCE_GLOBAL_VAR);
      if(errorCode){
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
      }
    }
    strGlobal++;
  }
  return 0;
}

int ecmcPLCs::parsePLC(int plcIndex,char * exprStr)
{
  //find plc variable
  char *strPLC=exprStr;
  char varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  
  varName[0]='\0';
  while((strPLC=strstr(strPLC,ECMC_PLC_DATA_STR)) && strlen(strPLC)>0){
    //Sanity check 1
    int tempInt=0;
    int nvals = sscanf(strPLC,ECMC_PLC_DATA_STR"%d.%[0-9a-zA-Z._]",&tempInt,varName);
    if (nvals == 2){                  
      varName[0]='\0';
      //Sanity check 2
      int nvals = sscanf(strPLC,"%[0-9a-zA-Z._]",varName);
      if (nvals == 1){
        int errorCode=createAndRegisterNewDataIF(plcIndex,varName,ECMC_RECORDER_SOURCE_GLOBAL_VAR);
        if(errorCode){
          return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
        }
      }
    }
    strPLC++;
  }
  return 0;
}

/*
 * Create new dataIF if needed and register in PLC
 */
int ecmcPLCs::createAndRegisterNewDataIF(int plcIndex,char * varName,ecmcDataSourceType dataSource)
{ 
  ecmcPLCDataIF *dataIF=NULL;
  int errorCode=findGlobalDataIF(varName, &dataIF);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  if(!dataIF){
    //create new
    errorCode=createNewGlobalDataIF(varName,dataSource,&dataIF);
    if(errorCode){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
    }
  }

  //Register in dedicated PLC if needed
  errorCode=plcs_[plcIndex]->addAndReisterGlobalVar(dataIF);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }

  return 0;
}

int ecmcPLCs::addPLCDefaultVariables(int plcIndex,int skipCycles)
 {
  //Add plc<index>.enable
  char varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  int chars = snprintf (varName,EC_MAX_OBJECT_PATH_CHAR_LENGTH-1,ECMC_PLC_DATA_STR"%d."ECMC_PLC_ENABLE_DATA_STR,plcIndex);
  if(chars>=EC_MAX_OBJECT_PATH_CHAR_LENGTH-1){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLCS_VARIABLE_NAME_TO_LONG);
  } 
  int errorCode=createAndRegisterNewDataIF(plcIndex,varName,ECMC_RECORDER_SOURCE_GLOBAL_VAR);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  ecmcPLCDataIF *dataIF=NULL;
  errorCode=findGlobalDataIF(varName,&dataIF);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  if(!dataIF){
    LOGERR("%s/%s:%d: Failed allocation of ecmcPLCDataIF plcEnable object %s (0x%x).\n",__FILE__,__FUNCTION__,__LINE__,varName,ERROR_PLC_DATA_IF_ALLOCATION_FAILED);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_DATA_IF_ALLOCATION_FAILED);
  }
  plcEnable_[plcIndex]=dataIF;
  
  //Add plc<index>.error
  chars = snprintf (varName,EC_MAX_OBJECT_PATH_CHAR_LENGTH-1,ECMC_PLC_DATA_STR"%d."ECMC_PLC_ERROR_DATA_STR,plcIndex);
  if(chars>=EC_MAX_OBJECT_PATH_CHAR_LENGTH-1){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLCS_VARIABLE_NAME_TO_LONG);
  }
  errorCode=createAndRegisterNewDataIF(plcIndex,varName,ECMC_RECORDER_SOURCE_GLOBAL_VAR);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  dataIF=NULL;
  errorCode=findGlobalDataIF(varName,&dataIF);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  if(!dataIF){
    LOGERR("%s/%s:%d: Failed allocation of ecmcPLCDataIF plcError_ object %s (0x%x).\n",__FILE__,__FUNCTION__,__LINE__,varName,ERROR_PLC_DATA_IF_ALLOCATION_FAILED);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_DATA_IF_ALLOCATION_FAILED);
  }
  plcError_[plcIndex]=dataIF;
  
  //Add plc<index>.scantime
  chars = snprintf (varName,EC_MAX_OBJECT_PATH_CHAR_LENGTH-1,ECMC_PLC_DATA_STR"%d."ECMC_PLC_SCAN_TIME_DATA_STR,plcIndex);
  if(chars>=EC_MAX_OBJECT_PATH_CHAR_LENGTH-1){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLCS_VARIABLE_NAME_TO_LONG);
  }
  errorCode=createAndRegisterNewDataIF(plcIndex,varName,ECMC_RECORDER_SOURCE_GLOBAL_VAR);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  dataIF=NULL;
  errorCode=findGlobalDataIF(varName,&dataIF);
  if(errorCode){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
  }
  if(!dataIF){
    LOGERR("%s/%s:%d: Failed allocation of ecmcPLCDataIF plcScanTime_ object %s (0x%x).\n",__FILE__,__FUNCTION__,__LINE__,varName,ERROR_PLC_DATA_IF_ALLOCATION_FAILED);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLC_DATA_IF_ALLOCATION_FAILED);
  }
  dataIF->setReadOnly(1);
  dataIF->setData(1/MCU_FREQUENCY*(skipCycles+1));  

  return 0;
}

int ecmcPLCs::getPLCErrorID()
{
  for(int i=0;i<ECMC_MAX_PLCS;i++){
    if(plcError_[i]){
      if(plcError_[i]->getData()){
        return (int)plcError_[i]->getData();
      }
    } 
  }
  return 0;
}

bool ecmcPLCs::getError()
{
  return getPLCErrorID() || ecmcError::getError();
}

int ecmcPLCs::getErrorID()
{
  if(getPLCErrorID()){
    return getPLCErrorID();
  }
  return ecmcError::getErrorID();
}

int ecmcPLCs::updateAllScanTimeVars(){

//Update all scantime variables
char varName[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  for(int i=0; i<ECMC_MAX_PLCS;i++){
    if(plcs_[i]){ 
      int chars = snprintf (varName,EC_MAX_OBJECT_PATH_CHAR_LENGTH-1,ECMC_PLC_DATA_STR"%d."ECMC_PLC_ENABLE_DATA_STR,i);
      if(chars>=EC_MAX_OBJECT_PATH_CHAR_LENGTH-1){
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_PLCS_VARIABLE_NAME_TO_LONG);
      } 
      ecmcPLCDataIF *dataIF=NULL;
      int errorCode=findGlobalDataIF(varName,&dataIF);
      if(errorCode){
        return setErrorID(__FILE__,__FUNCTION__,__LINE__,errorCode);
      }
      if(dataIF){
        dataIF->setReadOnly(1);
        dataIF->setData(plcs_[i]->getSampleTime());  
      }
    }
  }
  return 0;
}