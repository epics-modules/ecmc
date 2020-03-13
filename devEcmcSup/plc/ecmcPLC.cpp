/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPLC.cpp
*
*  Created on: Oct 5, 2018
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcPLC.h"

#include "../motion/ecmcMotion.h"
#include "../motion/ecmcAxisBase.h"
#include "../ethercat/ecmcEc.h"
#include "../misc/ecmcDataStorage.h"
#include "../com/ecmcAsynPortDriver.h"
#include "ecmcPLCMain.h"
#include "ecmcPLCTask.h"

extern ecmcAxisBase       *axes[ECMC_MAX_AXES];
extern ecmcEc             *ec;
extern ecmcDataStorage    *dataStorages[ECMC_MAX_DATA_STORAGE_OBJECTS];
extern ecmcPLCMain        *plcs;
extern ecmcAsynPortDriver *asynPort;
extern double              mcuFrequency;
extern int                 sampleRateChangeAllowed;

int createPLC(int index,  double cycleTimeMs, int axisPLC) {
  LOGINFO4("%s/%s:%d index=%d, cycleTimeMs=%lf, axisPLC?=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index,
           cycleTimeMs,
           axisPLC);

  if(cycleTimeMs / 1000 < (1 / mcuFrequency)) {
    LOGERR(
      "%s/%s:%d: PLC cycletime out of range."
      " Cycle time must be higher than realtime thread (time >= %lf ms)."
      " (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      (1 / mcuFrequency * 1000),
      ERROR_MAIN_SAMPLE_RATE_OUT_OF_RANGE);
    return ERROR_MAIN_SAMPLE_RATE_OUT_OF_RANGE;
  }

  // Sample rate fixed
  sampleRateChangeAllowed = 0;

  if (!plcs) {
    plcs = new ecmcPLCMain(ec,mcuFrequency,asynPort);
  }

  if (axisPLC) {
    if ((index < 0) || (index >= ECMC_MAX_PLCS + ECMC_MAX_AXES)) {
      return ERROR_MAIN_PLC_INDEX_OUT_OF_RANGE;
    }
  } else {
    if ((index < 0) || (index >= ECMC_MAX_PLCS)) {
      return ERROR_MAIN_PLC_INDEX_OUT_OF_RANGE;
    }
  }

  //if (!ec->getInitDone()) return ERROR_MAIN_EC_NOT_INITIALIZED;

  // Set axes pointers (for the already configuered axes)
  for (int i = 0; i < ECMC_MAX_AXES; i++) {
    plcs->setAxisArrayPointer(axes[i], i);
  }

  // Set data storage pointers
  for (int i = 0; i < ECMC_MAX_DATA_STORAGE_OBJECTS; i++) {
    plcs->setDataStoragePointer(dataStorages[i], i);
  }

  int skipCycles = cycleTimeMs * mcuFrequency / 1000-1;
  if (skipCycles < 0) {
    return ERROR_MAIN_PLCS_SKIPCYCLES_INVALID;
  }

  return plcs->createPLC(index, skipCycles);
}

int deletePLC(int index) {
  LOGINFO4("%s/%s:%d index=%d\n", __FILE__, __FUNCTION__, __LINE__, index);
  CHECK_PLCS_RETURN_IF_ERROR();
  return plcs->deletePLC(index);
}

int setPLCExpr(int index, char *expr) {
  LOGINFO4("%s/%s:%d index=%d value=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index,
           expr);
  CHECK_PLCS_RETURN_IF_ERROR();
  return plcs->setExpr(index, expr);
}

int appendPLCExpr(int index, char *expr) {
  LOGINFO4("%s/%s:%d index=%d value=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index,
           expr);
  CHECK_PLCS_RETURN_IF_ERROR();
  return plcs->appendExprLine(index, expr);
}

int loadPLCFile(int index, char *fileName) {
  LOGINFO4("%s/%s:%d index=%d value=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index,
           fileName);
  CHECK_PLCS_RETURN_IF_ERROR();
  return plcs->loadPLCFile(index, fileName);
}

int clearPLCExpr(int index) {
  LOGINFO4("%s/%s:%d index=%d\n", __FILE__, __FUNCTION__, __LINE__, index);
  CHECK_PLCS_RETURN_IF_ERROR();
  return plcs->clearExpr(index);
}

int compilePLCExpr(int index) {
  LOGINFO4("%s/%s:%d index=%d\n", __FILE__, __FUNCTION__, __LINE__, index);
  CHECK_PLCS_RETURN_IF_ERROR();
  return plcs->compileExpr(index);
}

int writePLCVar(int index, const char *varName, double value) {
  LOGINFO4("%s/%s:%d index=%d, varName=%s, value=%lf\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index,
           varName,
           value);
  CHECK_PLCS_RETURN_IF_ERROR();
  return plcs->writeStaticPLCVar(index, varName, value);
}

int readPLCVar(int index, const char *varName, double *value) {
  LOGINFO4("%s/%s:%d index=%d, varName=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index,
           varName);
  CHECK_PLCS_RETURN_IF_ERROR();
  return plcs->readStaticPLCVar(index, varName, value);
}

int setPLCEnable(int index, int enable) {
  LOGINFO4("%s/%s:%d index=%d\n", __FILE__, __FUNCTION__, __LINE__, index);
  CHECK_PLCS_RETURN_IF_ERROR();
  return plcs->setEnable(index, enable);
}

int getPLCEnable(int index, int *enabled) {
  LOGINFO4("%s/%s:%d index=%d\n", __FILE__, __FUNCTION__, __LINE__, index);
  CHECK_PLCS_RETURN_IF_ERROR();
  return plcs->getEnable(index, enabled);
}

const char* getPLCExpr(int plcIndex, int *error) {
  LOGINFO4("%s/%s:%d plcIndex=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           plcIndex);

  if (plcIndex >= ECMC_MAX_PLCS + ECMC_MAX_AXES || plcIndex < 0) {
    LOGERR("ERROR: PLC index out of range.\n");
    *error=ERROR_PLCS_INDEX_OUT_OF_RANGE;
    return "";
  }
    
  int errorLocal = 0;
  std::string *expr = plcs->getExpr(plcIndex,&errorLocal);
  if(errorLocal) {
    *error = errorLocal;
    return "";
  }

  return expr->c_str();
}