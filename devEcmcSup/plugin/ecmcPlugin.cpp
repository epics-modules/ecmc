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

#include "ecmcPlugin.h"

// TODO: REMOVE GLOBALS
#include "../main/ecmcGlobalsExtern.h"

int loadModule(const char* moduleFilenameWP) {
  LOGINFO4("%s/%s:%d moduleFilenameWP=%s\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           moduleFilenameWP);

  return 0;
}
