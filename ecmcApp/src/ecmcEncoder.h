/*
 * Encoder.h
 *
 *  Created on: Mar 14, 2016
 *      Author: anderssandstrom
 */

#ifndef ECMCENCODER_H_
#define ECMCENCODER_H_

#include <stdint.h>  //TODO different include in hw_motor for uint8_t ???
#include <stdio.h>
#include <ecrt.h>
#include <string.h>
#include <cmath>

#include "ecmcDefinitions.h"
#include "ecmcEcEntry.h"
#include "ecmcEcPdo.h"
#include "ecmcError.h"
#include "ecmcErrorsList.h"
#include "ecmcFilter.h"
#include "ecmcMasterSlaveData.h"
#include "ecmcMasterSlaveIF.h"
#include "ecmcTransform.h"

#define MaxMcuEncoderEntries 10

class ecmcEncoder : public ecmcError , public ecmcMasterSlaveIF
{
public:
  ecmcEncoder(double sampleTime);
  ~ecmcEncoder();
  int setBits(int bits);
  int64_t getRawPos();
  int setScaleNum(double scaleNum);
  int setScaleDenom(double scaleDenom);
  double getScaleNum();
  double getScaleDenom();
  double  getScale();
  double getActPos();
  void setActPos(double pos);
  void setSampleTime(double sampleTime);
  double getSampleTime();
  double getActVel();
  void setHomed(bool homed);
  bool getHomed();
  encoderType getType();
  int setType(encoderType encType);
  int setEntryAtIndex(ecmcEcEntry *entry,int index);
  double readEntries();
  void setOffset(double offset);
  int validate();

 protected:
  void initVars();
  int32_t turns_;
  uint64_t rawPosUint_;
  uint64_t rawPosUintOld_;
  int64_t rawPos_;
  double scaleNum_;
  double scaleDenom_;
  double scale_;
  double offset_;
  double actPos_;
  double actPosOld_;
  double sampleTime_;
  double actVel_;
  encoderType encType_;
  bool homed_;
  int64_t range_;
  uint64_t limit_;
  int bits_;
  ecmcEcEntry *entryArray_[MaxMcuEncoderEntries];  //CH 0 ActPos
  int64_t handleOverUnderFlow(uint64_t newValue,int bits);
  ecmcFilter *velocityFilter_;
};

#endif /* ECMCENCODER_H_ */
