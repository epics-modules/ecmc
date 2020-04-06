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

#define ECMC_DATA_ITEM_MAX_CALLBACK_FUNCS 8
typedef enum {
    ECMC_DIR_INVALID, /**< Invalid direction. Do not use this value. */
    ECMC_DIR_WRITE,   /**< Values written to ecmc. */
    ECMC_DIR_READ,    /**< Values read from ecmc. */    
    ECMC_DIR_COUNT    /**< Number of directions. For internal use only. */
} ecmcDataDir;

struct ecmcDataItemInfo{
  char          *name;
  uint8_t       *data;  
  size_t         dataSize;
  size_t         dataElementSize;
  size_t         dataBitCount;
  ecmcEcDataType dataType;
  ecmcDataDir    dataDirection;
  double         dataUpdateRateMs;
  int            dataPointerValid;  
};

/**  
*  Callback prototype to clients that subscribe to data (non asyn clients)
*  (last arg void* should the object supplied when registered)
*/
typedef void(*ecmcDataUpdatedCallback)(uint8_t*,size_t,ecmcEcDataType,void*);

/**
*  Class for generic access to all registered data items in ecmc (base class to asynDataItem).
*  All ecmc related information is handled in tgis class. All asyn related 
*  is handled in class ecmcAsynDataItem
*/
class ecmcDataItem {
 public:
  ecmcDataItem(const char *name);
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
  void    setEcmcDataElementSize(size_t bytes);
  size_t  getEcmcDataElementSize();
  void    setEcmcDataMaxSize(size_t bytes);
  size_t  getEcmcDataMaxSize();
  char *  getName();

  ecmcDataItemInfo *getDataItemInfo();

  /** general write function 
     asyn param callback will not be made if this function is used */
  int     write(uint8_t *data,
                size_t   bytes);
  /** general read function */
  int     read(uint8_t *data,
               size_t   bytes);

  /** Register data updated callback
  *   Return handle if success (to be used if deregister) otherwise -1 */
  int  regDataUpdatedCallback(ecmcDataUpdatedCallback func, void* callingObj);
  
  /** Deregister data updated callback by handle 
  *   (retuned by regDataUpdatedCallback()) */
  void deregDataUpdatedCallback(int handle);

 protected:
  virtual void refresh();

  ecmcDataItemInfo dataItem_;
  int      checkIntRange_;
  int      arrayCheckSize_;
  int64_t  intMax_;
  int64_t  intMin_;
  size_t   ecmcMaxSize_;
  
  ecmcDataUpdatedCallback callbackFuncs_[ECMC_DATA_ITEM_MAX_CALLBACK_FUNCS];
  void* callbackObjs_[ECMC_DATA_ITEM_MAX_CALLBACK_FUNCS];
  int callbackFuncsMaxIndex_;
};

#endif  /* ECMCDATAITEM_H_ */
