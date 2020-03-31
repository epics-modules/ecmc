/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcDataItem.h
*
*  Created on: May 23, 2016
*      Author: anderssandstrom
*  
*  Class to access a generic ecmc information data item by it's identification string
*
\*************************************************************************/

#ifndef ECMCDATAITEM_H_
#define ECMCDATAITEM_H_

#include "stdlib.h"
#include "ecmcDefinitions.h"

typedef enum {
    ECMC_DATA_DIR_INVALID, /**< Invalid direction. Do not use this value. */
    ECMC_DATA_DIR_WRITE,   /**< Values written to ecmc (example: ao,bo,.. records). */
    ECMC_DATA_DIR_READ,    /**< Values read by from ecmc (example:  ai,bi,.. records) */
    ECMC_DATA_DIR_RW,      /**< RW. */
    ECMC_DATA_DIR_COUNT    /**< Number of directions. For internal use only. */
} ecmcDataDir;

class ecmcDataItem {
 public:
  ecmcDataItem(char*          idStringWP, // Unique id string    
               uint8_t        *data,      // Pointer to data
               ecmcEcDataType dataType,   // Element data type
               size_t         dataSize,   // Total bytes
               size_t         elements,   // Number elements
               ecmcDataDir    dataDir)    // Input or output
);
  ~ecmcDataItem();
  char*          ecmcIdStringWP_;   // Unique id string    
  uint8_t        *ecmcData_;        // Pointer to data
  ecmcEcDataType emcmDataType_;     // Element data type
  size_t         ecmcDataSize_;     // Total bytes
  size_t         ecmcElements_;     // Number elements
  ecmcDataDir    ecmcDataDir_;      // Input or output
};

#endif  /* ECMCDATAITEM_H_ */
