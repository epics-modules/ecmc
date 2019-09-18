/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcCmdParser.h
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom, torstenbögershausen
*
\*************************************************************************/

#ifndef ECMC_CMD_EAT_H
# define ECMC_CMD_EAT_H

# include "ecmcOctetIF.h"

# ifdef __cplusplus
extern "C" {
# endif /* ifdef __cplusplus */

int ecmcCmdParser(int                   argc,
                  const char           *argv[],
                  const char           *seperator[],
                ecmcOutputBufferType *buffer);
int motorHandleOneArg(const char           *myarg_1,
                      ecmcOutputBufferType *buffer);

# ifdef __cplusplus
}
# endif /* ifdef __cplusplus */

#endif  /* ECMC_CMD_EAT_H */
