/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcRtLoggerPortDriver.h
*
*  Created on: Apr 11, 2026
*
\*************************************************************************/

#ifndef ECMC_RT_LOGGER_PORT_DRIVER_H_
#define ECMC_RT_LOGGER_PORT_DRIVER_H_

int         ecmcRtLoggerPortDriverStart();
void        ecmcRtLoggerPortDriverPublishMessage(int level, const char *message);
void        ecmcRtLoggerPortDriverPublishDropped(unsigned int dropped);
const char *ecmcRtLoggerPortDriverGetPortName();

#endif  /* ECMC_RT_LOGGER_PORT_DRIVER_H_ */
