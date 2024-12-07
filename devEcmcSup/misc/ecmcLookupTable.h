/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcLookupTable.h
*
*  Created on: Dec 06, 2024
*      Author: anderssandstrom
*
\*************************************************************************/
#ifndef ECMCLOOKUPTABLE_H_
#define ECMCLOOKUPTABLE_H_
#include <stdio.h>
#include <cstring>
#include <vector>
#include "ecmcError.h"

#define ERROR_LOOKUP_TABLE_ERROR 0x1441A
#define ERROR_LOOKUP_TABLE_NOT_SORTED 0x1441B
#define ERROR_LOOKUP_TABLE_OPEN_FILE_FAILED 0x1441C
#define ERROR_LOOKUP_TABLE_FILE_FORMAT_INVALID 0x1441D

/* 
    Use as an correction table for an encoder:
    * indexTable_: should represent the raw encoder positions UINT64_T
    * valueTable_: represents the error that should be corrected INT32_T
    * For raw positions in between the Error is interpolated.
    * For raw values outside the range indexTable_ compensation 
    is made with a static value of the first or last value of the correction table.
*/
template <typename T1, typename T2> class ecmcLookupTable : public ecmcError {
public:
  ecmcLookupTable(const std::string& filename);
  ~ecmcLookupTable();
  int getValidatedOK();
  T2 getValue(T1 inputIndex);

private:
  int loadCorrFile(const std::string& filename);
  int validate();
  std::vector<T1> indexTable_;
  std::vector<T2> valueTable_;
  bool validatedOK_;
};

#endif  /* ECMCLOOKUPTABLE_H_ */
