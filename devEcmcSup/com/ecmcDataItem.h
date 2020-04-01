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
#include "inttypes.h"
#include "ecmcDefinitions.h"

typedef enum {
    ECMC_DIR_INVALID, /**< Invalid direction. Do not use this value. */
    ECMC_DIR_WRITE,   /**< Values written to ecmc. */
    ECMC_DIR_READ,    /**< Values read from ecmc. */    
    ECMC_DIR_COUNT    /**< Number of directions. For internal use only. */
} ecmcDataDir;

struct ecmcDataItemData{
  uint8_t       *data;  
  size_t         dataSize;
  size_t         dataBitCount;
  ecmcEcDataType dataType;
  ecmcDataDir    dataDirection;
  double         dataUpdateRateMs;
  int            dataPointerValid;
};

/**
*  Class for generic access to all registered data items in ecmc (extension to asynDataItem).
*/
class ecmcDataItem {
 public:
  ecmcDataItem();
  virtual ~ecmcDataItem();
  void    setEcmcMaxValueInt(int64_t intMax);
  int64_t getEcmcMinValueInt();
  void    setEcmcMinValueInt(int64_t intMin);
  int64_t getEcmcMaxValueInt();
  void    setEcmcBitCount(size_t bits);
  size_t  getEcmcBitCount();
  void    setAllowWriteToEcmc(bool allowWrite);
  bool    getAllowWriteToEcmc();
  void    setArrayCheckSize(bool check);
  bool    getArrayCheckSize();
  void    setEcmcDataType(ecmcEcDataType dt);
  ecmcEcDataType getEcmcDataType();
  int     setEcmcDataPointer(uint8_t *data,size_t bytes);
  int     getEcmcDataPointerValid();
  void    setEcmcDataSize(size_t bytes);
  size_t  getEcmcDataSize();
  void    setEcmcDataMaxSize(size_t bytes);
  size_t  getEcmcDataMaxSize();

  ecmcDataItemData *getDataItemData();
  virtual ecmcDataItemData* getDataItemDataIfMe(char* idStringWP) = 0;

 protected:
  ecmcDataItemData dataItem_;
  int      checkIntRange_;
  int      arrayCheckSize_;
  int64_t  intMax_;
  int64_t  intMin_;
  size_t   ecmcMaxSize_;
};

#endif  /* ECMCDATAITEM_H_ */
