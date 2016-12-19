/*
 * ecmcEcEntryLink.cpp
 *
 *  Created on: May 23, 2016
 *      Author: anderssandstrom
 */

#include "ecmcEcEntryLink.h"

ecmcEcEntryLink::ecmcEcEntryLink()
{
  for(int i=0;i<MaxEcEntryLinks;i++){
    entryInfoArray_[i].entry=NULL;
    entryInfoArray_[i].bitNumber=-1;
  }
}

ecmcEcEntryLink::~ecmcEcEntryLink()
{
  ;
}

int ecmcEcEntryLink::setEntryAtIndex(ecmcEcEntry *entry,int index,int bitIndex)
{
  if(entry !=NULL && index< MaxEcEntryLinks && index>=0){
    entryInfoArray_[index].entry=entry;
    entryInfoArray_[index].bitNumber=bitIndex;
    return 0;
  }
  else{
    LOGINFO5("%s/%s:%d: ERROR: Assigning entry to object entry list at index %d failed.(0x%x).\n",__FILE__, __FUNCTION__, __LINE__,index,ERROR_EC_ENTRY_LINK_FAILED);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ENTRY_LINK_FAILED);
  }
}

int ecmcEcEntryLink::validateEntry(int index)
{
  if(index>=MaxEcEntryLinks || index<0){
    return ERROR_EC_ENTRY_INDEX_OUT_OF_RANGE;
  }

  if(entryInfoArray_[index].bitNumber>=0){
    return validateEntryBit(index);
  }

  if(entryInfoArray_[index].entry ==NULL){
    return ERROR_EC_ENTRY_DATA_POINTER_NULL;
  }

  return 0;
}

int ecmcEcEntryLink::validateEntryBit(int index)
{
  if(index>=MaxEcEntryLinks || index<0){
    return ERROR_EC_ENTRY_INDEX_OUT_OF_RANGE;
  }

  if(entryInfoArray_[index].entry ==NULL){
    return ERROR_EC_ENTRY_DATA_POINTER_NULL;
  }

  if(entryInfoArray_[index].bitNumber<0 || entryInfoArray_[index].bitNumber>entryInfoArray_[index].entry->getBits()){
    return ERROR_EC_ENTRY_INVALID_BIT_INDEX;
  }

  return 0;
}

int ecmcEcEntryLink::readEcEntryValue(int entryIndex,uint64_t *value)
{
  uint64_t tempRaw=0;
  if(entryInfoArray_[entryIndex].bitNumber<0){
    if(entryInfoArray_[entryIndex].entry->readValue(&tempRaw)){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ENTRY_READ_FAIL);
    }
  }
  else{
    if(entryInfoArray_[entryIndex].entry->readBit(entryInfoArray_[entryIndex].bitNumber,&tempRaw)){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ENTRY_READ_FAIL);
    }
  }
  *value=tempRaw;
  return 0;
}

int ecmcEcEntryLink::writeEcEntryValue(int entryIndex,uint64_t value)
{
  if(entryInfoArray_[entryIndex].bitNumber<0){
    if(entryInfoArray_[entryIndex].entry->writeValue(value)){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ENTRY_WRITE_FAIL);
    }
  }
  else{
    if(entryInfoArray_[entryIndex].entry->writeBit(entryInfoArray_[entryIndex].bitNumber,value)){
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_EC_ENTRY_WRITE_FAIL);
    }
  }
  return 0;
}

int ecmcEcEntryLink::getEntryBitCount(int index, int *bitCount)
{
  if(entryInfoArray_[index].entry ==NULL){
    return ERROR_EC_ENTRY_DATA_POINTER_NULL;
  }

  *bitCount=entryInfoArray_[index].entry->getBits();
  return 0;
}

int ecmcEcEntryLink::getEntryStartBit(int index, int *startBit)
{
  if(entryInfoArray_[index].entry ==NULL){
    return ERROR_EC_ENTRY_DATA_POINTER_NULL;
  }

  *startBit=entryInfoArray_[index].bitNumber;
  return 0;

}
