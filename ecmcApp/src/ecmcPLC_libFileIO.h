/*
 * ecmcPLC_libMc.h
 *
 *  Created on: Nov 19, 2018
 *      Author: anderssandstrom
 */

#ifndef ecmcPLC_libFileIO_H_
#define ecmcPLC_libFileIO_H_

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

#endif /* ecmcPLC_libFileIO_H_ */
