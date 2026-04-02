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
#include <limits>
#include <vector>
#include "ecmcDefinitions.h"
#include "ecmcErrorsList.h"
#include "ecmcError.h"
#include "ecmcEcEntry.h"
#include "ecmcEcEntryLink.h"
#include "ecmcEcPdo.h"
#include "ecmcFilter.h"
#include "ecmcAxisData.h"
#include "ecmcMotionUtils.h"
#include "ecmcLookupTable.h"

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
  ecmcEncoder(ecmcAsynPortDriver *asynPortDriver,
              ecmcAxisData       *axisData,
              double              sampleTime,
              int                 index);
  ~ecmcEncoder();
  virtual void          errorReset();
  int                   setBits(int bits);
  int                   getBits();

  // Used for homing of partly absolute encoders (applied after raw mask)
  int                   setAbsBits(int absBits);
  int                   getAbsBits();
  int64_t               getRawPosMultiTurn();
  uint64_t              getRawPosRegister();
  double                getRawPosRegisterDouble();
  uint64_t              getRawAbsPosRegister();  // Only absolute bits
  ecmcEcDataType        getActPosEntryDataType();
  bool                  actPosEntryUsesFloatingPoint();
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
  double                getOffset();
  int                   validate();
  int                   setToZeroIfRelative();
  int                   setRawMask(uint64_t mask);
  uint64_t              getRawMask();
  bool                  getLatchFuncEnabled();
  int                   setHomeLatchArmControlWord(uint64_t control, int bits);
  void                  setArmLatch(bool arm);
  bool                  getArmLatch();
  bool                  getNewValueLatched();
  double                getLatchPosEng();
  ecmcOverUnderFlowType getOverUnderflow();
  int                   setVeloFilterSize(size_t size);
  int                   getVeloFilterSize();
  int                   setVelFilterEnable(bool enable);
  int                   getVelFilterEnable();
  int                   setPosFilterSize(size_t size);
  int                   getPosFilterSize();
  int                   setPosFilterEnable(bool enable);
  int                   getPosFilterEnable();

  // Ref this encoder to other encoder at startup (i.e ref relative encoder to abs at startup)
  int                   setRefToOtherEncAtStartup(int encIndex);
  int                   getRefToOtherEncAtStartup();
  int                   setRefAtHoming(int refEnable);
  bool                  getRefAtHoming();
  void                  setMaxPosDiffToPrimEnc(double distance);
  double                getMaxPosDiffToPrimEnc();
  int                   hwReady();
  int                   setInvHwReady(int invert);

  // For homing (just storing data)
  int                   getHomeParamsValid();
  void                  setHomeVelTowardsCam(double vel);
  double                getHomeVelTowardsCam();
  void                  setHomeVelOffCam(double vel);
  double                getHomeVelOffCam();
  void                  setHomePosition(double pos);
  double                getHomePosition();
  void                  setHomePostMoveTargetPosition(double targetPos);
  double                getHomePostMoveTargetPosition();
  void                  setHomePostMoveEnable(int enable);
  int                   getHomePostMoveEnable();
  void                  setHomeLatchCountOffset(int count);
  int                   getHomeLatchCountOffset();
  void                  setHomeSeqId(int seqid);
  int                   getHomeSeqId();
  void                  setHomeAcc(double acc);
  double                getHomeAcc();
  void                  setHomeDec(double dec);
  double                getHomeDec();
  int                   setHomeExtTrigg(bool val);
  int                   getHomeExtTriggStat();
  int                   getHomeExtTriggEnabled();
  // Load index table from file (will be default enabled)
  int                   loadLookupTable(const std::string& filename);
  // Enable use of lookup table
  int                   setLookupTableEnable(bool enable);
  int                   getLookupTableEnable();
  int                   setLookupTableRange(double range);
  double                getLookupTableRange();
  int                   setLookupTableScale(double scale);
  double                getLookupTableScale();
  int                   setDelayCyclesAndEnable(double cycles, bool enable); 
  double                getDelayCycles();
  int                   getDelayCompEnable();
  // Only propagate encoder errors from the primary encoder to the axis
  int                   setErrorID(int errorID) override;
  int                   setErrorID(int               errorID,
                                   ecmcAlarmSeverity severity) override;
  int                   setErrorID(const char *fileName,
                                   const char *functionName,
                                   int         lineNumber,
                                   int         errorID) override;
  int                   setErrorID(const char       *fileName,
                                   const char       *functionName,
                                   int               lineNumber,
                                   int               errorID,
                                   ecmcAlarmSeverity severity) override;
  int                   setAllowOverUnderFlow(bool allow);
protected:
  void                  initVars();
  bool                  entryTypeIsFloat(ecmcEcDataType type) const;
  static int64_t        clampDoubleToInt64(double value);
  static uint64_t       clampDoubleToUInt64(double value);
  int                   countTrailingZerosInMask(uint64_t mask);
  int                   countBitWidthOfMask(uint64_t mask,
                                            int      trailZeros);
  int64_t               handleOverUnderFlow(uint64_t rawPosOld,
                                            uint64_t rawPos,
                                            int64_t  rawTurns,
                                            uint64_t rawLimit,
                                            int      bits);
  bool                  isPrimary() const;
  uint8_t* getActPosPtr();
  uint8_t* getActVelPtr();
  int      initAsyn();
  int      readHwActPos(bool masterOK,
                        bool domainOK);
  int      readHwWarningError(bool domainOK);
  int      readHwLatch(bool domainOK);
  int      readHwReady(bool domainOK);
  
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
  double rawPosDouble_;
  double rawPosDoubleOld_;
  double rawPosMultiTurn_;
  double rawPosOffset_;
  int64_t rawRange_;
  int64_t rawAbsRange_;
  int64_t rawTurns_;
  int64_t rawTurnsOld_;
  int bits_;
  int absBits_;
  double scaleNum_;
  double scaleDenom_;
  double scale_;
  double invScale_;
  double engOffset_;
  double actPos_;
  double actPosLocal_;
  double actPosOld_;
  double sampleTimeMs_;
  double invSampleTime_;
  double actVel_;
  double actVelLocal_;
  bool homed_;
  bool encLatchFunctEnabled_;
  bool encLatchStatus_;
  bool encLatchStatusOld_;
  double rawAbsPosDouble_;
  double rawAbsPosDoubleOld_;
  double rawEncLatchPos_;
  double rawEncLatchPosMultiTurn_;
  uint64_t encLatchControlWordArm_;
  uint64_t encLatchControlWordIdle_;
  int      encLatchControlBits_;
  bool     encLatchArm_;
  double actEncLatchPos_;
  bool enablePositionFilter_;
  bool enableVelocityFilter_;
  uint64_t hwReset_;
  uint64_t hwErrorAlarm0_;
  uint64_t hwErrorAlarm0Old_;
  uint64_t hwErrorAlarm1_;
  uint64_t hwErrorAlarm1Old_;
  uint64_t hwErrorAlarm2_;
  uint64_t hwErrorAlarm2Old_;
  uint64_t hwWarning_;
  uint64_t hwWarningOld_;
  uint64_t hwReady_;
  uint64_t hwReadyOld_;
  bool hwActPosDefined_;
  bool hwResetDefined_;
  bool hwErrorAlarm0Defined_;
  bool hwErrorAlarm1Defined_;
  bool hwErrorAlarm2Defined_;
  bool hwWarningDefined_;
  bool hwReadyBitDefined_;
  bool masterOKOld_;
  int refEncIndex_;
  bool refDuringHoming_;
  int homeLatchCountOffset_;
  double maxPosDiffToPrimEnc_;
  bool encInitilized_;
  bool hwSumAlarm_;
  bool hwSumAlarmOld_;
  bool hwTriggedHomingEnabled_;
  int encLocalErrorId_;
  int encLocalErrorIdOld_;

  // Asyn
  ecmcAsynPortDriver *asynPortDriver_;
  ecmcAsynDataItem *encPosAct_;
  ecmcAsynDataItem *encVelAct_;
  ecmcAsynDataItem *encErrId_;

  int hwReadyInvert_;
  int index_; // Index of this encoder (im axis object)

  // Homing
  int homeParamsValid_;
  double homeVelTowardsCam_;
  double homeVelOffCam_;
  double homePosition_;
  int homeSeqId_;
  bool homeEnablePostMove_;
  double homePostMoveTargetPos_;
  double homeAcc_;
  double homeDec_;
  int domainOK_;
  bool lookupTableEnable_;
  ecmcLookupTable<double, double>  *lookupTable_; 
  double lookupTableRange_;
  double lookupTableScale_;
  
  double delayTimeS_; // Compensate for delay between setpoint and actual value (should default to 2 cycles)
  bool enableDelayTime_;
  bool allowOverUnderFlow_;
};

#endif  /* ECMCENCODER_H_ */
