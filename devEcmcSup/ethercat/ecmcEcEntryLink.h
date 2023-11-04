/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcEcEntryLink.h
*
*  Created on: May 23, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCECENTRYLINK_H_
#define ECMCECENTRYLINK_H_

#include "ecmcEcEntry.h"

#define ECMC_EC_ENTRY_LINKS_MAX 20

struct entryInfo {
  ecmcEcEntry *entry;
  int          bitNumber;
};

class ecmcEcEntryLink : public ecmcError {
public:
  ecmcEcEntryLink();
  ecmcEcEntryLink(int *errorPtr,
                  int *warningPtr);
  ~ecmcEcEntryLink();
  void           initVars();

  int            setEntryAtIndex(ecmcEcEntry *entry,
                                 int          index,
                                 int          bitIndex);
  int            validateEntry(int index);
  int            readEcEntryValue(int       entryIndex,
                                  uint64_t *value);
  int            readEcEntryValueDouble(int     entryIndex,
                                        double *value);
  int            writeEcEntryValue(int      entryIndex,
                                   uint64_t value);

  int            writeEcEntryValueDouble(int    entryIndex,
                                         double value);

  bool           checkEntryExist(int entryIndex);
  bool           checkDomainOK(int entryIndex);
  bool           checkDomainOKAllEntries();

protected:
  int            validateEntryBit(int index);
  int            getEntryBitCount(int  index,
                                  int *bitCount);
  int            getEntryStartBit(int  index,
                                  int *startBit);
  ecmcEcDataType getEntryDataType(int index);
  int            getSlaveId(int index);

private:
  entryInfo entryInfoArray_[ECMC_EC_ENTRY_LINKS_MAX];
};

#endif  /* ECMCECENTRYLINK_H_ */
