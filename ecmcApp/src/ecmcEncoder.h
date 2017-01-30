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
#include "ecmcEcEntryLink.h"
#include "ecmcEcPdo.h"
#include "ecmcError.h"
#include "ecmcFilter.h"
#include "ecmcAxisData.h"

//ENCODER
#define ERROR_ENC_ASSIGN_ENTRY_FAILED 0x14400
#define ERROR_ENC_TYPE_NOT_SUPPORTED 0x14401
#define ERROR_ENC_SCALE_DENOM_ZERO 0x14402
#define ERROR_ENC_INVALID_RATE 0x14403
#define ERROR_ENC_ENTRY_NULL 0x14404
#define ERROR_ENC_ENTRY_READ_FAIL 0x14405
#define ERROR_ENC_TRANSFORM_NULL 0x14406
#define ERROR_ENC_EXT_MASTER_SOURCE_NULL 0x14407
#define ERROR_ENC_EXT_MASTER_SOURCE_COUNT_ZERO 0x14408
#define ERROR_ENC_INVALID_SAMPLE_TIME 0x14409
#define ERROR_ENC_TRANSFORM_VALIDATION_ERROR 0x1440A
#define ERROR_ENC_SLAVE_INTERFACE_NULL 0x1440B

#define ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION 0

class ecmcEncoder : public ecmcEcEntryLink
{
public:
  ecmcEncoder(ecmcAxisData *axisData,double sampleTime);
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
  double getSampleTime();
  double getActVel();
  void setHomed(bool homed);
  bool getHomed();
  encoderType getType();
  int setType(encoderType encType);
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
  int64_t handleOverUnderFlow(uint64_t newValue,int bits);
  ecmcFilter *velocityFilter_;
  ecmcAxisData* data_;
};

#endif /* ECMCENCODER_H_ */
