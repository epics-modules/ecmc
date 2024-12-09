/*************************************************************************\
* Copyright (c) 2024 Paul Scherrer Institute
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
#include <iterator>
#include <algorithm>
#include <iostream>
#include <fstream>
#include "ecmcError.h"

#define ERROR_LOOKUP_TABLE_ERROR 0x1441A
#define ERROR_LOOKUP_TABLE_NOT_SORTED 0x1441B
#define ERROR_LOOKUP_TABLE_OPEN_FILE_FAILED 0x1441C
#define ERROR_LOOKUP_TABLE_FILE_FORMAT_INVALID 0x1441D

/* 
    Use as correction table for an encoder:
    * indexTable_: should represent the raw encoder positions UINT64_T
    * valueTable_: represents the error that should be added INT32_T
    * For raw positions in between the Error is interpolated.
    * For raw values outside the range indexTable_ compensation 
    is made with a static value of the first or last value of the correction table.
*/
template <typename T1, typename T2> 
class ecmcLookupTable : public ecmcError {
public:
  
  ecmcLookupTable(const std::string& filename) {
     validatedOK_ = false;
     indexTable_.clear();
     valueTable_.clear();
     int error = loadTable(filename);
     if(error  || !getValidatedOK()) {
       throw error; 
    }
  }
  ~ecmcLookupTable(){};

  // Function to perform linear interpolation
  T2 getValue(T1 inputIndex) {
   
    // Handle cases where inputIndex is out of bounds (no interpolation)
    if (inputIndex <= indexTable_.front()) {
      return inputIndex + valueTable_.front();
    }
    if (inputIndex >= indexTable_.back()) {
      return inputIndex + valueTable_.back();
    }
   
    // Binary search to find the interval
    auto it = std::lower_bound(indexTable_.begin(), indexTable_.end(), inputIndex);
    // Determine the indices for interpolation
    size_t idx = std::distance(indexTable_.begin(), it);
    size_t i1 = idx - 1;
    size_t i2 = idx;
   
    // Linear interpolation.. Maybe need to look into the casts again
    return (T2) (valueTable_[i1] + (static_cast<double>(valueTable_[i2] - valueTable_[i1]) *
                      (inputIndex - indexTable_[i1])) / (indexTable_[i2] - indexTable_[i1]));
  }

  int getValidatedOK() {
    return validatedOK_;
  }  

private:
  std::vector<T1> indexTable_;
  std::vector<T2> valueTable_;
  bool validatedOK_;

  int loadTable(const std::string& filename) {
  
    // Open the file
    std::ifstream inputFile(filename);
    if (!inputFile.is_open()) {
      LOGERR(
        "%s/%s:%d: ERROR: Opening correction file %s failed (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        filename.c_str(),
        ERROR_LOOKUP_TABLE_OPEN_FILE_FAILED);
        return setErrorID(__FILE__,
                          __FUNCTION__,
                          __LINE__,
                          ERROR_LOOKUP_TABLE_OPEN_FILE_FAILED);
    }
  
    // Clear the vectors to ensure they're empty before loading new data
    indexTable_.clear();
    valueTable_.clear();
    std::string line;
    
    printf("INFO: Loading correction table:\n");
    int index = 1;
    while (std::getline(inputFile, line)) {
      std::istringstream lineStream(line);
      T1 indexValue;  // Example the encoder raw count
      T2 value;            // Example the error at indexValue
      
      printf("%03d: %s\n",index,line.c_str());
      index++;
  
      // Skip commented lines
      if( line[0] == '#') {
        continue;
      }
      // Read two values from the current line
      if (lineStream >> indexValue >> value) {
          indexTable_.push_back(indexValue);
          valueTable_.push_back(value);
      } else {
        LOGERR(
          "%s/%s:%d: ERROR: Correction file format invalid (0x%x).\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          ERROR_LOOKUP_TABLE_FILE_FORMAT_INVALID);
          return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_LOOKUP_TABLE_FILE_FORMAT_INVALID);
      }
    }
  
    inputFile.close();
    return validate();
  }


  int validate() {
    validatedOK_ = false;
    // Check that both vectors are the same size and non-empty and size bigger than 2 
    if (indexTable_.size() != valueTable_.size() || indexTable_.empty() || indexTable_.size() < 3) {
      LOGERR(
        "%s/%s:%d: ERROR: Encoder correction table size miss-match (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_LOOKUP_TABLE_ERROR);
  
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_LOOKUP_TABLE_ERROR);
    }
  
     // Ensure the indexTable_ vector is sorted
    for (size_t i = 1; i < indexTable_.size(); ++i) {
      if (indexTable_[i] < indexTable_[i - 1]) {
         LOGERR(
        "%s/%s:%d: ERROR: Encoder correction table not sorted (0x%x).\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        ERROR_LOOKUP_TABLE_NOT_SORTED);
  
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_LOOKUP_TABLE_NOT_SORTED);
      }
    }
    validatedOK_ = true;
    return 0;
  }
};

#endif  /* ECMCLOOKUPTABLE_H_ */
