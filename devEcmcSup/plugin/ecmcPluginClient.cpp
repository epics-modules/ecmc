/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcPlugin.cpp
*
*  Created on: Oct 21, 2020
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcPluginClient.h"
#include "ecmcOctetIF.h"        // Log Macros
#include "ecmcDefinitions.h"
#include "ecmcErrorsList.h"

// TODO: REMOVE GLOBALS
#include "ecmcGlobalsExtern.h"

void* getEcmcDataItem(char *idStringWP) {
  LOGINFO4("%s/%s:%d: idStringWP =%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           idStringWP);

  if (!asynPort)return NULL;

  return (void *)asynPort->findAvailDataItem(idStringWP);
}

void* getEcmcAsynDataItem(char *idStringWP) {
  LOGINFO4("%s/%s:%d: idStringWP =%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           idStringWP);

  if (!asynPort)return NULL;

  return (void *)asynPort->findAvailDataItem(idStringWP);
}

void* getEcMaster() {
  return (void *)ec;
}

void* getEcmcAsynPortDriver() {
  LOGINFO4("%s/%s:%d:\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);

  return (void *)asynPort;
}

double getEcmcSampleRate() {
  LOGINFO4("%s/%s:%d:\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);

  return mcuFrequency;
}

double getEcmcSampleTimeMS() {
  // mcuPeriod is in nano seconds
  return mcuPeriod / 1E6;
}

int getEcmcEpicsIOCState() {
  if (!asynPort) {
    return -1;
  }
  return asynPort->getEpicsState();
}
