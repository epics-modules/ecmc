/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPLCTask_libFileIO.inc
*
*  Created on: Nov 19, 2018
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ecmcPLC_libFileIO_inc_
#define ecmcPLC_libFileIO_inc_

//Used to find functions in exprtk fileIO package.
const char* fileIOLibCmdList[] = { 
                              "println(",
                              "print(",
                              "open(",
                              "close(",
                              "write(",
                              "read(",    
                              "getline(",    
                              "eof",    
                              };

static int fileIO_cmd_count=sizeof(fileIOLibCmdList)/sizeof(fileIOLibCmdList[0]);

#endif /* ecmcPLC_libFileIO_inc_ */
