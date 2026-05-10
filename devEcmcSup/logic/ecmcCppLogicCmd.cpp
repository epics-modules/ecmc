/*************************************************************************\
* Copyright (c) 2026 Paul Scherrer Institute
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcCppLogicCmd.cpp
*
\*************************************************************************/

#include "ecmcCppLogicCmd.h"

#include "ecmcCppLogicLib.h"
#include "ecmcDefinitions.h"
#include "ecmcError.h"
#include "ecmcErrorsList.h"
#include "ecmcGlobalsExtern.h"
#include "ecmcOctetIF.h"

#include <string>

namespace {
std::string g_lastCppLogicErrorMessage;
}

const char* getLastCppLogicErrorMessage(void) {
  return g_lastCppLogicErrorMessage.empty() ? "" : g_lastCppLogicErrorMessage.c_str();
}

int loadCppLogic(int logicId, const char* filenameWP, const char* configStr) {
  LOGINFO4("%s/%s:%d logicId=%d filenameWP=%s configStr=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           logicId,
           filenameWP ? filenameWP : "(null)",
           configStr ? configStr : "(null)");

  g_lastCppLogicErrorMessage.clear();

  if ((logicId < 0) || (logicId >= ECMC_MAX_PLUGINS)) {
    return ERROR_MAIN_CPP_LOGIC_INDEX_OUT_OF_RANGE;
  }

  if (cppLogics[logicId]) {
    delete cppLogics[logicId];
    cppLogics[logicId] = NULL;
  }

  cppLogics[logicId] = new ecmcCppLogicLib(logicId);

  if (!cppLogics[logicId]) {
    return ERROR_MAIN_CPP_LOGIC_OBJECT_NULL;
  }

  int errorCode = cppLogics[logicId]->load(filenameWP, configStr ? configStr : "");
  if (errorCode) {
    g_lastCppLogicErrorMessage = cppLogics[logicId]->getLastErrorMessage();
    LOGERR("%s/%s:%d: LoadCppLogic(%d, %s) failed: %s (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           logicId,
           filenameWP ? filenameWP : "(null)",
           ecmcError::convertErrorIdToString(errorCode),
           errorCode);
    delete cppLogics[logicId];
    cppLogics[logicId] = NULL;
    return errorCode;
  }

  return 0;
}

int appendCppLogicMacros(int logicId, const char* macrosText) {
  LOGINFO4("%s/%s:%d logicId=%d macrosText=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           logicId,
           macrosText ? macrosText : "(null)");

  if ((logicId < 0) || (logicId >= ECMC_MAX_PLUGINS)) {
    return ERROR_MAIN_CPP_LOGIC_INDEX_OUT_OF_RANGE;
  }

  if (!cppLogics[logicId]) {
    return ERROR_MAIN_CPP_LOGIC_OBJECT_NULL;
  }

  return cppLogics[logicId]->appendMacros(macrosText ? macrosText : "");
}

int reportCppLogic(int logicId) {
  LOGINFO4("%s/%s:%d logicId=%d\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           logicId);

  if ((logicId < 0) || (logicId >= ECMC_MAX_PLUGINS)) {
    return ERROR_MAIN_CPP_LOGIC_INDEX_OUT_OF_RANGE;
  }

  if (!cppLogics[logicId]) {
    return ERROR_MAIN_CPP_LOGIC_OBJECT_NULL;
  }

  cppLogics[logicId]->report();
  return 0;
}
