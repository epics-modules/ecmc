#ifndef CMCUDRIVE_H_
#define CMCUDRIVE_H_
#include <stdio.h>
#include <cmath>

#include "ecmcDefinitions.h"
#include "ecmcEcEntry.h"
#include "ecmcEcPdo.h"
#include "ecmcError.h"
#include "ecmcErrorsList.h"

#define MaxMcuDriveEntries 10

class ecmcDrive : public ecmcError
{
public:
  ecmcDrive();
  ecmcDrive(double scale);
  ~ecmcDrive();
  void initVars() ;
  void setScaleNum(double scaleNum);
  int setScaleDenom(double scaleDenom);
  double getScale();
  int setVelSet(double vel);
  double getVelSet();
  int setVelSetRaw(int rawVel);
  int getVelSetRaw();
  bool getEnable();
  int setEnable(bool enable);
  int setEntryAtIndex(ecmcEcEntry *entry,int index);
  void setInterlock(bool interlock);
  bool getInterlock();
  void writeEntries();
  int validate();
private:
  double scale_;
  double scaleNum_;
  double scaleDenom_;
  int velSetRaw_;
  double velSet_;
  bool enable_;
  bool interlock_;
  operationMode opeationMode_;
  ecmcEcEntry *entryArray_[MaxMcuDriveEntries];  //CH 0 Enable, CH1 VelSet
};
#endif
