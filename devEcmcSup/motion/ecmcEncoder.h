/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEncoder.h
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMCENCODER_H_
#define ECMCENCODER_H_

#define __STDC_FORMAT_MACROS  // To have PRIx64
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <ecrt.h>
#include <string.h>
#include <cmath>
#include "../main/ecmcDefinitions.h"
#include "../main/ecmcError.h"
#include "../ethercat/ecmcEcEntry.h"
#include "../ethercat/ecmcEcEntryLink.h"
#include "../ethercat/ecmcEcPdo.h"
#include "ecmcFilter.h"
#include "ecmcAxisData.h"

// ENCODER ERRORS
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
#define ERROR_ENC_VELOCITY_FILTER_NULL 0x1440C
#define ERROR_ENC_RAW_MASK_INVALID 0x1440D
#define ERROR_ENC_ABS_MASK_INVALID 0x1440E
#define ERROR_ENC_ABS_BIT_COUNT_INVALID 0x1440F
#define ERROR_ENC_HW_ALARM_0 0x14410
#define ERROR_ENC_HW_ALARM_1 0x14411
#define ERROR_ENC_HW_ALARM_2 0x14412
#define ERROR_ENC_WARNING_READ_ENTRY_FAIL 0x14413
#define ERROR_ENC_ALARM_READ_ENTRY_FAIL 0x14414

#define ECMC_FILTER_VELO_DEF_SIZE 100
#define ECMC_FILTER_POS_DEF_SIZE 10

// Constants
#define ECMC_ENCODER_MAX_VALUE_64_BIT ((uint64_t)(0xFFFFFFFFFFFFFFFFULL))

// Do not allow absolute encoders with less than 4 bits
#define ECMC_ENCODER_ABS_BIT_MIN 4

enum ecmcOverUnderFlowType {
  ECMC_ENC_NORMAL    = 0,
  ECMC_ENC_UNDERFLOW = 1,
  ECMC_ENC_OVERFLOW  = 2,
};

class ecmcEncoder : public ecmcEcEntryLink {
 public:
  ecmcEncoder(ecmcAxisData *axisData,
              double        sampleTime);
  ~ecmcEncoder();
  virtual void          errorReset();
  int                   setBits(int bits);
  int                   getBits();
  // Used for homing of partly absolute encoders (applied after raw mask)
  int                   setAbsBits(int absBits);
  int                   getAbsBits();
  int64_t               getRawPosMultiTurn();
  uint64_t              getRawPosRegister();
  uint64_t              getRawAbsPosRegister();  // Only absolute bits
  int                   setScaleNum(double scaleNum);
  int                   setScaleDenom(double scaleDenom);
  double                getScaleNum();
  double                getScaleDenom();
  double                getScale();
  double                getActPos();
  double                getAbsRangeEng();
  int64_t               getAbsRangeRaw();
  void                  setActPos(double pos);
  double                getSampleTime();
  double                getActVel();
  void                  setHomed(bool homed);
  bool                  getHomed();
  encoderType           getType();
  int                   setType(encoderType encType);
  double                readEntries(bool masterOK);
  int                   writeEntries();
  int                   setOffset(double offset);
  int                   validate();
  int                   setToZeroIfRelative();
  int                   setRawMask(uint64_t mask);
  bool                  getLatchFuncEnabled();
  void                  setArmLatch(bool arm);
  bool                  getArmLatch();
  bool                  getNewValueLatched();
  double                getLatchPosEng();
  ecmcOverUnderFlowType getOverUnderflow();
  int                   setVeloFilterSize(size_t size);
  int                   setPosFilterSize(size_t size);
  int                   setPosFilterEnable(bool enable);
  
 protected:
  void                  initVars();
  int                   countTrailingZerosInMask(uint64_t mask);
  int                   countBitWidthOfMask(uint64_t mask,
                                            int      trailZeros);
  int64_t               handleOverUnderFlow(uint64_t rawPosOld,
                                            uint64_t rawPos,
                                            int64_t  rawTurns,
                                            uint64_t rawLimit,
                                            int      bits);
  encoderType encType_;
  ecmcFilter *velocityFilter_;
  ecmcFilter *positionFilter_;
  ecmcAxisData *data_;
  int64_t turns_;
  uint64_t rawPosUint_;          // Raw position register (masked and shifted)
  uint64_t rawPosUintOld_;       // Raw position register (masked and shifted)
  uint64_t rawAbsPosUint_;       // Only absolute bits
  uint64_t rawAbsPosUintOld_;    // Only absolute bits
  uint64_t totalRawMask_;
  uint64_t totalRawRegShift_;
  uint64_t rawLimit_;
  uint64_t rawAbsLimit_;
  int64_t rawPosMultiTurn_;
  int64_t rawPosOffset_;
  int64_t rawRange_;
  int64_t rawAbsRange_;
  int64_t rawTurns_;
  int64_t rawTurnsOld_;
  int bits_;
  int absBits_;
  double scaleNum_;
  double scaleDenom_;
  double scale_;
  double engOffset_;
  double actPos_;
  double actPosOld_;
  double sampleTime_;
  double actVel_;
  bool homed_;
  bool encLatchFunctEnabled_;
  bool encLatchStatus_;
  bool encLatchStatusOld_;
  int64_t rawEncLatchPos_;
  int64_t rawEncLatchPosMultiTurn_;
  bool encLatchControl_;
  double actEncLatchPos_;
  bool enablePositionFilter_;
  uint64_t hwReset_;
  uint64_t hwErrorAlarm0_;
  uint64_t hwErrorAlarm0Old_;
  uint64_t hwErrorAlarm1_;
  uint64_t hwErrorAlarm1Old_;
  uint64_t hwErrorAlarm2_;
  uint64_t hwErrorAlarm2Old_;
  uint64_t hwWarning_;
  uint64_t hwWarningOld_;
  bool hwResetDefined_;
  bool hwErrorAlarm0Defined_;
  bool hwErrorAlarm1Defined_;
  bool hwErrorAlarm2Defined_;
  bool hwWarningDefined_;
  bool masterOKOld_;

};

#endif  /* ECMCENCODER_H_ */
