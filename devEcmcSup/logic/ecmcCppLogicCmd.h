/*************************************************************************\
* Copyright (c) 2026 Paul Scherrer Institute
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcCppLogicCmd.h
*
\*************************************************************************/

#ifndef ECMC_CPP_LOGIC_CMD_H_
#define ECMC_CPP_LOGIC_CMD_H_

#ifdef __cplusplus
extern "C" {
#endif

int loadCppLogic(int logicId, const char* filenameWP, const char* configStr);
int appendCppLogicMacros(int logicId, const char* macrosText);
int reportCppLogic(int logicId);
const char* getLastCppLogicErrorMessage(void);

#ifdef __cplusplus
}
#endif

#endif
