/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcDataItem.cpp
*
*  Created on: Mar 31, 2020
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcDataItem.h"

ecmcDataItem::ecmcDataItem(char*          idStringWP, // Unique id string    
                           uint8_t        *data,      // Pointer to data
                           ecmcEcDataType dataType,   // Element data type
                           size_t         dataSize,   // Total bytes
                           size_t         elements,   // Number elements
                           int            dataDir) {  // Read,Write or both

}

ecmcDataItem::~ecmcDataItem() {

}




  ecmcDataItem(char*          idStringWP, // Unique id string    
               uint8_t        *data,      // Pointer to data
               ecmcEcDataType dataType,   // Element data type
               size_t         dataSize,   // Total bytes
               size_t         elements,   // Number elements
               int            dataDir)    // Input or output
);
  ~ecmcDataItem();
  char*          ecmcIdStringWP_;   // Unique id string    
  uint8_t        *ecmcData_;        // Pointer to data
  ecmcEcDataType emcmDataType_;     // Element data type
  size_t         ecmcDataSize_;     // Total bytes
  size_t         ecmcElements_;     // Number elements
  int            ecmcDataDir_;      // Input or output
