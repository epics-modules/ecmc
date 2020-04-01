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
#include "string.h"

ecmcDataItem::ecmcDataItem(const char *name) {
  memset(&dataItem_,0,sizeof(dataItem_));
  checkIntRange_  = 0;
  intMax_         = 0;
  intMin_         = 0;
  arrayCheckSize_ = 0;
  ecmcMaxSize_    = 0;
  dataItem_.name  = strdup(name);
  dataItem_.dataUpdateRateMs = -1;
}

ecmcDataItem::~ecmcDataItem() {
  free(dataItem_.name);
}

void ecmcDataItem::setEcmcMaxValueInt(int64_t intMax) {
  checkIntRange_ = 1;
  intMax_        = intMax;  
}

void ecmcDataItem::setEcmcMinValueInt(int64_t intMin) {
  checkIntRange_ = 1;
  intMin_        = intMin;
}

void ecmcDataItem::setEcmcBitCount(size_t bits) {
   dataItem_.dataBitCount = bits;
}

int64_t ecmcDataItem::getEcmcMaxValueInt() {  
  return intMax_;
}

int64_t ecmcDataItem::getEcmcMinValueInt() {
  return intMin_;
}

size_t ecmcDataItem::getEcmcBitCount() {
  return dataItem_.dataBitCount;
}

void ecmcDataItem::setAllowWriteToEcmc(bool allowWrite) {
  dataItem_.dataDirection = allowWrite ? ECMC_DIR_WRITE : ECMC_DIR_READ;
}

bool ecmcDataItem::getAllowWriteToEcmc() {
  return dataItem_.dataDirection == ECMC_DIR_WRITE;
}

void ecmcDataItem::setArrayCheckSize(bool check) {
  arrayCheckSize_ = check;
}

bool ecmcDataItem::getArrayCheckSize() {
  return arrayCheckSize_;
}  

void ecmcDataItem::setEcmcDataType(ecmcEcDataType dt) {
  dataItem_.dataType = dt;
}

ecmcEcDataType ecmcDataItem::getEcmcDataType() {
  return dataItem_.dataType;
}

int ecmcDataItem::setEcmcDataPointer(uint8_t *data,size_t bytes)
{
  dataItem_.data     = data;
  dataItem_.dataSize = bytes;
  ecmcMaxSize_       = bytes;
  dataItem_.dataPointerValid = 1;
  return 0;
}

int ecmcDataItem::getEcmcDataPointerValid() {
  return dataItem_.dataPointerValid;
}

ecmcDataItemInfo *ecmcDataItem::getDataItemData() {
  return &dataItem_;
}

void ecmcDataItem::setEcmcDataSize(size_t bytes) {
  dataItem_.dataSize = bytes;
}
  
size_t ecmcDataItem::getEcmcDataSize() {
  return dataItem_.dataSize;
}

void ecmcDataItem::setEcmcDataMaxSize(size_t bytes) {
  ecmcMaxSize_ = bytes;
}

size_t ecmcDataItem::getEcmcDataMaxSize() {
  return ecmcMaxSize_;
}

void ecmcDataItem::refresh() {
   //handle callbacks to subscribers here
}

char *ecmcDataItem::getName() {  
  return dataItem_.name;
}
