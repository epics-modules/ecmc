/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcEcEntryLink.cpp
*
*  Created on: May 23, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcEcEntryLink.h"
#include "ecmcRtLogger.h"

extern app_mode_type appModeStat;

int ecmcEcEntryLink::setDomainError(const char *fileName,
                                     const char *functionName,
                                     int         lineNumber) {
  if (appModeStat == ECMC_MODE_STARTUP) {
    return ERROR_EC_ENTRY_EC_DOMAIN_ERROR;
  }

  return setErrorID(fileName,
                    functionName,
                    lineNumber,
                    ERROR_EC_ENTRY_EC_DOMAIN_ERROR);
}

ecmcEcEntryLink::ecmcEcEntryLink(int *errorPtr, int *warningPtr) : ecmcError(
    errorPtr,
    warningPtr) {
  initVars();
}

ecmcEcEntryLink::ecmcEcEntryLink() {
  initVars();
}

void ecmcEcEntryLink::initVars() {
  for (int i = 0; i < ECMC_EC_ENTRY_LINKS_MAX; i++) {
    entryInfoArray_[i].entry     = NULL;
    entryInfoArray_[i].bitNumber = -1;
  }
}

ecmcEcEntryLink::~ecmcEcEntryLink() {}

int ecmcEcEntryLink::setEntryAtIndex(ecmcEcEntry *entry,
                                     int          index,
                                     int          bitIndex) {
  if ((entry != NULL) && (index < ECMC_EC_ENTRY_LINKS_MAX) && (index >= 0)) {
    entryInfoArray_[index].entry     = entry;
    entryInfoArray_[index].bitNumber = bitIndex;
    return 0;
  } else {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Assigning entry to object entry list at index %d failed.(0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index,
      ERROR_EC_ENTRY_LINK_FAILED);
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_LINK_FAILED);
  }
}

int ecmcEcEntryLink::validateEntry(int index) {
  if ((index >= ECMC_EC_ENTRY_LINKS_MAX) || (index < 0)) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Entry list index %d out of range.(0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index,
           ERROR_EC_ENTRY_INDEX_OUT_OF_RANGE);
    return ERROR_EC_ENTRY_INDEX_OUT_OF_RANGE;
  }

  if (entryInfoArray_[index].bitNumber >= 0) {
    return validateEntryBit(index);
  }

  if (entryInfoArray_[index].entry == NULL) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Entry ethercat data pointer NULL at index %d (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index,
      ERROR_EC_ENTRY_DATA_POINTER_NULL);
    return ERROR_EC_ENTRY_DATA_POINTER_NULL;
  }

  return 0;
}

int ecmcEcEntryLink::validateEntryBit(int index) {
  if ((index >= ECMC_EC_ENTRY_LINKS_MAX) || (index < 0)) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Entry list index %d out of range.(0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           index,
           ERROR_EC_ENTRY_INDEX_OUT_OF_RANGE);
    return ERROR_EC_ENTRY_INDEX_OUT_OF_RANGE;
  }

  if (entryInfoArray_[index].entry == NULL) {
    ecmcRtLoggerLogError(
      "%s/%s:%d: ERROR: Entry ethercat data pointer NULL at index %d (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      index,
      ERROR_EC_ENTRY_DATA_POINTER_NULL);
    return ERROR_EC_ENTRY_DATA_POINTER_NULL;
  }

  if (entryInfoArray_[index].bitNumber<0 ||
                                       entryInfoArray_[index].bitNumber>
      entryInfoArray_[index].entry->getBits() -
      1) {
    ecmcRtLoggerLogError("%s/%s:%d: ERROR: Invalid bit index.(0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           ERROR_EC_ENTRY_INVALID_BIT_INDEX);
    return ERROR_EC_ENTRY_INVALID_BIT_INDEX;
  }

  return 0;
}

int ecmcEcEntryLink::readEcEntryValue(int entryIndex, uint64_t *value) {
  ecmcEcEntry * const entry = entryInfoArray_[entryIndex].entry;
  const int bitNumber       = entryInfoArray_[entryIndex].bitNumber;

  if (!checkDomainOK(entryIndex)) {
    return setDomainError(__FILE__, __FUNCTION__, __LINE__);
  }

  uint64_t tempRaw = 0;

  if (bitNumber < 0) {
    if (entry->readValue(&tempRaw)) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_EC_ENTRY_READ_FAIL);
    }
  } else {
    if (entry->readBit(bitNumber, &tempRaw)) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_EC_ENTRY_READ_FAIL);
    }
  }
  *value = tempRaw;
  return 0;
}

int ecmcEcEntryLink::readEcEntryValueDouble(int entryIndex, double *value) {
  ecmcEcEntry * const entry = entryInfoArray_[entryIndex].entry;

  if (!checkDomainOK(entryIndex)) {
    return setDomainError(__FILE__, __FUNCTION__, __LINE__);
  }

  double tempDouble = 0;

  if (entry->readDouble(&tempDouble)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_READ_FAIL);
  }
  *value = tempDouble;
  return 0;
}

int ecmcEcEntryLink::writeEcEntryBits(int entryIndex, int bits, uint64_t value) {
  ecmcEcEntry * const entry = entryInfoArray_[entryIndex].entry;
  const int bitNumber       = entryInfoArray_[entryIndex].bitNumber;

  if (entry->writeBits(bitNumber, bits, value)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_WRITE_FAIL);
  }
  
  // Still write value to entry (above) even if domain error to keep value up to date
  if (!checkDomainOK(entryIndex)) {
    return setDomainError(__FILE__, __FUNCTION__, __LINE__);
  }
  return 0;
}


int ecmcEcEntryLink::readEcEntryBits(int entryIndex, int bits, uint64_t *value) {
  ecmcEcEntry * const entry = entryInfoArray_[entryIndex].entry;
  const int bitNumber       = entryInfoArray_[entryIndex].bitNumber;

  if (!checkDomainOK(entryIndex)) {
    return setDomainError(__FILE__, __FUNCTION__, __LINE__);
  }

  uint64_t tempRaw = 0;

 if (entry->readBits(bitNumber, bits, &tempRaw)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_READ_FAIL);
  }

  *value = tempRaw;
  return 0;
}

int ecmcEcEntryLink::writeEcEntryValue(int entryIndex, uint64_t value) {
  ecmcEcEntry * const entry = entryInfoArray_[entryIndex].entry;
  const int bitNumber       = entryInfoArray_[entryIndex].bitNumber;

  if (bitNumber < 0) {
    if (entry->writeValue(value)) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_EC_ENTRY_WRITE_FAIL);
    }
  } else {
    if (entry->writeBit(bitNumber, value)) {
      return setErrorID(__FILE__,
                        __FUNCTION__,
                        __LINE__,
                        ERROR_EC_ENTRY_WRITE_FAIL);
    }
  }

  // Still write value to entry (above) even if domain error to keep value up to date
  if (!checkDomainOK(entryIndex)) {
    return setDomainError(__FILE__, __FUNCTION__, __LINE__);
  }

  return 0;
}

int ecmcEcEntryLink::writeEcEntryValueDouble(int entryIndex, double value) {
  ecmcEcEntry * const entry = entryInfoArray_[entryIndex].entry;

  if (entry->writeDouble(value)) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_EC_ENTRY_WRITE_FAIL);
  }

  // Still write value to entry (above) even if domain error to keep value up to date
  if (!checkDomainOK(entryIndex)) {
    return setDomainError(__FILE__, __FUNCTION__, __LINE__);
  }

  return 0;
}

int ecmcEcEntryLink::getEntryBitCount(int index, int *bitCount) {
  if (entryInfoArray_[index].entry == NULL) {
    return ERROR_EC_ENTRY_DATA_POINTER_NULL;
  }

  *bitCount = entryInfoArray_[index].entry->getBits();
  return 0;
}

int ecmcEcEntryLink::getEntryStartBit(int index, int *startBit) {
  if (entryInfoArray_[index].entry == NULL) {
    return ERROR_EC_ENTRY_DATA_POINTER_NULL;
  }

  *startBit = entryInfoArray_[index].bitNumber;
  return 0;
}

bool ecmcEcEntryLink::checkEntryExist(int entryIndex) {
  if ((entryIndex >= 0) && (entryIndex < ECMC_EC_ENTRY_LINKS_MAX)) {
    return entryInfoArray_[entryIndex].entry != NULL;
  }
  return false;
}

ecmcEcDataType ecmcEcEntryLink::getEntryDataType(int index) {
  if ((index < 0) || (index >= ECMC_EC_ENTRY_LINKS_MAX)) {
    return ECMC_EC_NONE;
  }

  if (entryInfoArray_[index].entry == NULL) {
    return ECMC_EC_NONE;
  }

  return entryInfoArray_[index].entry->getDataType();
}

int ecmcEcEntryLink::getSlaveId(int index) {
  if ((index < 0) || (index >= ECMC_EC_ENTRY_LINKS_MAX)) {
    return -1000;
  }

  if (entryInfoArray_[index].entry == NULL) {
    return -1000;
  }

  return entryInfoArray_[index].entry->getSlaveId();
}

bool ecmcEcEntryLink::checkDomainOK(int entryIndex) {
  if ((entryIndex < 0) || (entryIndex >= ECMC_EC_ENTRY_LINKS_MAX)) {
    return false;
  }

  if (entryInfoArray_[entryIndex].entry == NULL) {
    return false;
  }

  return entryInfoArray_[entryIndex].entry->getDomainOK();
}

bool ecmcEcEntryLink::checkDomainOKAllEntries() {
  for (int i = 0; i < ECMC_EC_ENTRY_LINKS_MAX; i++) {
    ecmcEcEntry * const entry = entryInfoArray_[i].entry;
    if (entry) {
      if (!entry->getDomainOK()) {
        return false;
      }
    } else {  // no more entries
      break;
    }
  }
  return true;
}
