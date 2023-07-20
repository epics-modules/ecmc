/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEncoder.cpp
*
*  Created on: Mar 14, 2016
*      Author: anderssandstrom
*
\*************************************************************************/

#include "ecmcEncoder.h"

ecmcEncoder::ecmcEncoder(ecmcAsynPortDriver *asynPortDriver,
                         ecmcAxisData *axisData,
                         double        sampleTime,
                         int index)
                         : ecmcEcEntryLink(&(axisData->status_.errorCode),&(axisData->status_.warningCode)) {
  initVars();

  asynPortDriver_ = asynPortDriver;
  data_           = axisData;
  sampleTime_     = sampleTime;
  index_          = index;

  initAsyn();
  
  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
  
  velocityFilter_ = new ecmcFilter(sampleTime,ECMC_FILTER_VELO_DEF_SIZE);
  positionFilter_ = new ecmcFilter(sampleTime,ECMC_FILTER_POS_DEF_SIZE);

  if (!velocityFilter_) {
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR VELOCITY-FILTER OBJECT.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    setErrorID(__FILE__, __FUNCTION__, __LINE__,
               ERROR_ENC_VELOCITY_FILTER_NULL);
    exit(EXIT_FAILURE);
  }
}

ecmcEncoder::~ecmcEncoder() {
  delete velocityFilter_;
  velocityFilter_ = NULL;

  delete positionFilter_;
  positionFilter_ = NULL;
}

void ecmcEncoder::initVars() {
  errorReset();
  index_                = 0;
  encType_              = ECMC_ENCODER_TYPE_INCREMENTAL;
  rawPosMultiTurn_      = 0;
  rawRange_             = 0;
  rawLimit_             = 0;
  rawAbsLimit_          = 0;
  bits_                 = 0;
  rawPosUintOld_        = 0;
  rawPosUint_           = 0;
  scale_                = 0;
  engOffset_            = 0;
  actPos_               = 0;
  actPosOld_            = 0;
  sampleTime_           = 1;
  actVel_               = 0;
  actPosLocal_          = 0;
  actVelLocal_          = 0;
  homed_                = false;
  enablePositionFilter_ = false;
  scaleNum_             = 0;
  scaleDenom_           = 1;
  absBits_              = 0;
  totalRawMask_         = ECMC_ENCODER_MAX_VALUE_64_BIT;
  totalRawRegShift_     = 0;
  rawPosOffset_         = 0;
  encLatchFunctEnabled_ = 0;
  encLatchStatus_       = 0;
  encLatchStatusOld_    = 0;
  rawEncLatchPos_       = 0;
  encLatchControl_      = 0;
  rawTurns_             = 0;
  rawTurnsOld_          = 0;
  actEncLatchPos_       = 0;
  rawAbsPosUint_        = 0;
  rawAbsPosUintOld_     = 0;
  hwReset_              = 0;
  hwErrorAlarm0_        = 0;
  hwErrorAlarm0Old_     = 0;
  hwErrorAlarm1_        = 0;
  hwErrorAlarm1Old_     = 0;
  hwErrorAlarm2_        = 0;
  hwErrorAlarm2Old_     = 0;
  hwWarning_            = 0;
  hwWarningOld_         = false;
  hwResetDefined_       = false;
  hwErrorAlarm0Defined_ = false;
  hwErrorAlarm1Defined_ = false;
  hwErrorAlarm2Defined_ = false;
  hwWarningDefined_     = false;
  hwReadyBitDefined_    = false;
  masterOKOld_          = false;
  hwActPosDefined_      = false;
  refEncIndex_          = -1;
  refDuringHoming_      = true;
  homeLatchCountOffset_ = 0;
  encPosAct_            = NULL;
  encVelAct_            = NULL;
  asynPortDriver_       = NULL;
  maxPosDiffToPrimEnc_  = 0;  
  encInitilized_        = 0;
  hwReady_              = 0;
}

int64_t ecmcEncoder::getRawPosMultiTurn() {
  // Overflow compensated raw value
  return rawPosMultiTurn_;
}

uint64_t ecmcEncoder::getRawPosRegister() {
  // Non overflow compensated raw value
  return rawPosUint_;
}

uint64_t ecmcEncoder::getRawAbsPosRegister() {
  // Non overflow compensated raw value of abs bits only
  return rawAbsPosUint_;
}

int ecmcEncoder::setScaleNum(double scaleNum) {
  scaleNum_ = scaleNum;

  if (std::abs(scaleDenom_) > 0) {
    scale_ = scaleNum_ / scaleDenom_;
  }
  return 0;
}

int ecmcEncoder::setScaleDenom(double scaleDenom) {
  scaleDenom_ = scaleDenom;

  if (scaleDenom_ == 0) {
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_ENC_SCALE_DENOM_ZERO);
  }
  scale_ = scaleNum_ / scaleDenom_;
  return 0;
}

double ecmcEncoder::getScale() {
  return scale_;
}

double ecmcEncoder::getActPos() {
  return actPos_;
}

void ecmcEncoder::setActPos(double pos) {
  // reset overflow counter
  engOffset_       = 0;
  rawTurnsOld_     = 0;
  rawTurns_        = 0;
  rawPosOffset_    = pos / scale_ - rawPosUint_;
  rawPosMultiTurn_ = rawPosUint_ + rawPosOffset_;

  actPosOld_ = pos;
  actPosLocal_    = pos;

  // Must clear velocity filter
  velocityFilter_->initFilter(pos);
  positionFilter_->initFilter(pos);
  actPos_ = pos;
}

int ecmcEncoder::setOffset(double offset) {
  engOffset_ = offset;
  return 0;
}

double ecmcEncoder::getSampleTime() {
  return sampleTime_;
}

double ecmcEncoder::getActVel() {
  return actVel_;
}

void ecmcEncoder::setHomed(bool homed) {
  homed_ = homed;
}

bool ecmcEncoder::getHomed() {
  return homed_;
}

int ecmcEncoder::setType(encoderType encType) {
  switch (encType) {
  case ECMC_ENCODER_TYPE_ABSOLUTE:
    encType_ = encType;
    homed_   = true;
    break;

  case ECMC_ENCODER_TYPE_INCREMENTAL:
    encType_ = encType;
    break;

  default:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_ENC_TYPE_NOT_SUPPORTED);

    break;
  }
  
  return 0;
}

encoderType ecmcEncoder::getType() {
  return encType_;
}

/*
* Return updated turns (based on over/underflow)
*/
int64_t ecmcEncoder::handleOverUnderFlow(uint64_t rawPosOld,
                                         uint64_t rawPos,
                                         int64_t  rawTurns,
                                         uint64_t rawLimit,
                                         int      bits) {
  int64_t turns = rawTurns;
  // Only support for over/under flow of datatypes less than 64 bit
  if (bits < 64) {
    // Overflow
    if ((rawPosOld > rawPos) && (rawPosOld - rawPos > rawLimit)) {
      turns++;
    } else {
      // Underflow
      if ((rawPosOld < rawPos) && (rawPos - rawPosOld > rawLimit)) {
        turns--;
      }
    }
  } else {
    turns = 0;
  }
  return turns;
}

int ecmcEncoder::setBits(int bits) {
  if (bits == 0) {
    // Special case.. Need to support this since otherwise axis
    // with external source will lead to config error
    bits_             = 0;
    rawRange_         = 0;
    rawLimit_         = 0;
    totalRawRegShift_ = 0;
    totalRawMask_     = 0;
    return 0;
  }

  int errorCode = setRawMask((uint64_t)(pow(2, bits) - 1));

  if (errorCode) {
    return errorCode;
  }

  return 0;
}

int ecmcEncoder::getBits() {
  return bits_;
}

int ecmcEncoder::setAbsBits(int absBits) {

  if(absBits>bits_) {
    LOGERR("%s/%s:%d: Encoder abs. bit count invalid. (abs. bits > total bits) (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, ERROR_ENC_ABS_BIT_COUNT_INVALID);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_ENC_ABS_BIT_COUNT_INVALID);
  }
  // if bits_ is not set
  if (bits_ == 0) {
    bits_ = absBits;
  }

  absBits_     = absBits;
  rawAbsRange_ = pow(2, absBits_);
  rawAbsLimit_ = rawAbsRange_ * ECMC_OVER_UNDER_FLOW_FACTOR;  // Limit for over/under-flow
  return 0;
}

int ecmcEncoder::getAbsBits() {
  return absBits_;
}

int ecmcEncoder::setRawMask(uint64_t mask) {
  int trailingZeros = countTrailingZerosInMask(mask);

  if (trailingZeros < 0) {
    LOGERR("%s/%s:%d: Encoder Raw Mask Invalid, mask==0 (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, ERROR_ENC_RAW_MASK_INVALID);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_ENC_RAW_MASK_INVALID);
  }
  int bitWidth = countBitWidthOfMask(mask, trailingZeros);

  if (bitWidth < 0) {
    LOGERR("%s/%s:%d: Encoder Raw Mask Invalid. Mask not continuous (0x%x).\n",
           __FILE__, __FUNCTION__, __LINE__, ERROR_ENC_RAW_MASK_INVALID);
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_ENC_RAW_MASK_INVALID);
  }

  bits_             = bitWidth;
  rawRange_         = pow(2, bits_) - 1;
  // Limit for over/under-flow
  rawLimit_         = rawRange_ * ECMC_OVER_UNDER_FLOW_FACTOR;
  totalRawRegShift_ = pow(2, trailingZeros) - 1;
  totalRawMask_     = mask;
  return 0;
}

double ecmcEncoder::getScaleNum() {
  return scaleNum_;
}

double ecmcEncoder::getScaleDenom() {
  return scaleDenom_;
}


int  ecmcEncoder::readHwActPos(bool masterOK) {

  if(!hwActPosDefined_) {
    return 0;
  }

  uint64_t tempRaw = 0;
  
  // Actual position entry
  // Act position
  if (readEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION, &tempRaw)!=0) {
    return ERROR_ENC_ENTRY_READ_FAIL;
  }  
  
  rawPosUintOld_ = rawPosUint_;
  // Filter value with mask
  rawPosUint_    = (totalRawMask_ & tempRaw) - totalRawRegShift_;
  
  if(!encInitilized_ && masterOK) {
    // if ready bit defined
    if(hwReadyBitDefined_) {
      if(hwReady_ > 0) {
        rawPosUintOld_ = rawPosUint_;
        encInitilized_ = 1;

        LOGERR("%s/%s:%d: INFO (axis %d): Encoder initialized (readybit==OK).\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          data_->axisId_);

      }
    } else {  // else latch value at positive edge of masterOK
      // If first valid value (at first hw ok),
      // then store the same position in last cycle value.
      // This to avoid over/underflow since rawPosUintOld_ is initiated to 0.
      if(!masterOKOld_) {
        rawPosUintOld_ = rawPosUint_;
        encInitilized_ = 1;
        LOGERR("%s/%s:%d: INFO (axis %d): Encoder initialized (masterOK==true ).\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          data_->axisId_);
      }
    }
  }
  // Check over/underflow (update turns counter)
  rawTurnsOld_ = rawTurns_;
  rawTurns_    = handleOverUnderFlow(rawPosUintOld_,
                                     rawPosUint_,
                                     rawTurns_,
                                     rawLimit_,
                                     bits_);
  rawPosMultiTurn_ = rawTurns_ * rawRange_ + rawPosUint_ + rawPosOffset_;
   // Calculate absolute encoder data
  if (absBits_ > 0) {
    rawAbsPosUintOld_ = rawAbsPosUint_;
    rawAbsPosUint_    = (rawAbsRange_ - 1) & rawPosUint_;  // filter abs bits
  } else {
    rawAbsPosUintOld_ = 0;
    rawAbsPosUint_    = 0;
  }  
  
  actPosLocal_    = scale_ * rawPosMultiTurn_ + engOffset_;
   // If first valid value (at first hw ok),
  // then store the same position in last cycle value.
  // This to avoid over/underflow since actPosOld_ is initiated to 0.
   if(!masterOKOld_ && masterOK) {
    actPosOld_ = actPosLocal_;
  }
   // Check modulo
  if(data_->command_.moduloRange != 0) {    
    if(actPosLocal_ >= data_->command_.moduloRange){      
      actPosLocal_ = actPosLocal_-data_->command_.moduloRange;      
      // Reset stuff to be able to run forever
      engOffset_       = 0;
      rawTurnsOld_     = 0;
      rawTurns_        = 0;
      rawPosOffset_    = actPosLocal_ / scale_ - rawPosUint_;
      rawPosMultiTurn_ = rawPosUint_ + rawPosOffset_;
    }
    if(actPosLocal_ < 0){
      actPosLocal_ = data_->command_.moduloRange + actPosLocal_;
      // Reset stuff to be able to run forever
      engOffset_       = 0;
      rawTurnsOld_     = 0;
      rawTurns_        = 0;
      rawPosOffset_    = actPosLocal_ / scale_ - rawPosUint_;
      rawPosMultiTurn_ = rawPosUint_ + rawPosOffset_;
    }
  }
   if(enablePositionFilter_)  {
    actPosLocal_ = positionFilter_->getFiltPos(actPosLocal_, data_->command_.moduloRange);
  }
   double distTraveled =  actPosLocal_ - actPosOld_;
  if(data_->command_.moduloRange != 0) {    
    double modThreshold = FILTER_POS_MODULO_OVER_UNDER_FLOW_LIMIT * data_->command_.moduloRange;
    if(actPosLocal_ - actPosOld_ > modThreshold) {
      distTraveled = actPosLocal_ - actPosOld_ - data_->command_.moduloRange;
    }
    else if (actPosLocal_ - actPosOld_ < -modThreshold){
      distTraveled = actPosLocal_ - actPosOld_ + data_->command_.moduloRange;
    }
  }
  actVelLocal_    = velocityFilter_->getFiltVelo(distTraveled);
  return 0;
}

int ecmcEncoder::readHwLatch() {
  // Encoder latch entries (status and position)
  if (!encLatchFunctEnabled_) {
    return 0;
  }

  uint64_t tempRaw = 0;

  if (readEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_LATCH_STATUS, &tempRaw)) {
    return ERROR_ENC_ENTRY_READ_FAIL;
  }
  encLatchStatusOld_ = encLatchStatus_;
  encLatchStatus_    = tempRaw > 0;
   if (readEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_LATCH_VALUE, &tempRaw)) {
    return ERROR_ENC_ENTRY_READ_FAIL;
  }
   // Also treat latched position as actual position (same mask and shift)
  rawEncLatchPos_ = (totalRawMask_ & tempRaw) - totalRawRegShift_;
   // if new latched value then calculate latched value in engineering units
  if (encLatchStatus_ > encLatchStatusOld_) {
    // Calculate multiturn latch value position (raw)
    // Use rawTurnsOld_ since over/under flow might have occured after
    // value was latched in hardware
    int64_t turns = handleOverUnderFlow(rawPosUintOld_,
                                        rawEncLatchPos_,
                                        rawTurnsOld_,
                                        rawLimit_,
                                        bits_);
    rawEncLatchPosMultiTurn_ = turns * rawRange_ + rawEncLatchPos_ +
                               rawPosOffset_;
    actEncLatchPos_ = scale_ * rawEncLatchPosMultiTurn_ +
                      engOffset_;
  }

  return 0;
}

int ecmcEncoder::readHwWarningError() {
  int errorLocal = 0;
  // Check warning link. Think about forwarding warning info to motor record somehow
  if (hwWarningDefined_) {
    if (readEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_WARNING, &hwWarning_)) {
      hwWarning_ = 0;
      hwWarningOld_ = 0;
      errorLocal = ERROR_ENC_WARNING_READ_ENTRY_FAIL;
    }
    if(hwWarning_ > 0 && hwWarningOld_ == 0) {
      LOGERR("%s/%s:%d: WARNING (axis %d): Encoder hardware in warning state.\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          data_->axisId_);
    }
    if(hwWarning_ == 0 && hwWarningOld_ > 0) {
      LOGERR("%s/%s:%d: INFO (axis %d): Encoder hardware warning state cleared.\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          data_->axisId_);
    }
     hwWarningOld_ = hwWarning_;
  }
  
  // check alarm 0
  if (hwErrorAlarm0Defined_) {
    if (readEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_ALARM_0, &hwErrorAlarm0_)) {
      hwErrorAlarm0_ = 0;
      hwErrorAlarm0Old_ = 0;
      errorLocal = ERROR_ENC_ALARM_READ_ENTRY_FAIL;
     }
    
    // Set Alarm
    if(hwErrorAlarm0_) {
      data_->command_.enable = 0;
      errorLocal = ERROR_ENC_HW_ALARM_0;
    }    
    hwErrorAlarm0Old_ = hwErrorAlarm0_;
  }
  
  // check alarm 1
  if (hwErrorAlarm1Defined_) {
    if (readEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_ALARM_1, &hwErrorAlarm1_)) {
      hwErrorAlarm1_ = 0;
      hwErrorAlarm1Old_ = 0;        
      errorLocal = ERROR_ENC_ALARM_READ_ENTRY_FAIL;
    }
    
    // Set Alarm
    if(hwErrorAlarm1_) {
      data_->command_.enable = 0;
      errorLocal = ERROR_ENC_HW_ALARM_1;
    }    
    hwErrorAlarm1Old_ = hwErrorAlarm1_;
  }
  
  // check alarm 2
  if (hwErrorAlarm2Defined_) {
    if (readEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_ALARM_2, &hwErrorAlarm2_)) {
      hwErrorAlarm2_ = 0;
      hwErrorAlarm2Old_ = 0;
      errorLocal = ERROR_ENC_ALARM_READ_ENTRY_FAIL;

    }
    
    // Set Alarm
    if(hwErrorAlarm2_) {
      data_->command_.enable = 0;
      errorLocal = ERROR_ENC_HW_ALARM_2;
    }    
    hwErrorAlarm2Old_ = hwErrorAlarm2_;
  }
  return errorLocal;
}

// Check that encoder is ready during runtime (and enabled)
int ecmcEncoder::readHwReady() {  
  // Check warning link. Think about forwarding warning info to motor record somehow
  if (hwReadyBitDefined_) {
    if (readEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_READY, &hwReady_)) {
      hwReady_ = 0;
      return ERROR_ENC_READY_READ_ENTRY_FAIL;
    }

    if( hwReady_ == 0) {      
      if(data_->status_.enabled) {
        // Error when enabled, this is serious, remove power
        data_->command_.enable = 0;
        return ERROR_ENC_NOT_READY;
      } else {
        // just set warning when not enabled
        setWarningID(WARNING_ENC_NOT_READY);
      }
    }
  }

  return 0;
}

int ecmcEncoder::hwReady() {
  if(getErrorID()) {
    return 0;
  }
  if(data_->command_.encSource == ECMC_DATA_SOURCE_INTERNAL) {
    if (!encInitilized_ || (hwReadyBitDefined_ && hwReady_ == 0)) {
      return 0;    
    } 
  }
  // Ok 
  return 1;
}

double ecmcEncoder::readEntries(bool masterOK) {
  int errorLocal = 0;
  actPosOld_ = actPos_;

  if (getError() || !masterOK) {

    if(data_->command_.encSource == ECMC_DATA_SOURCE_EXTERNAL) {
      actPos_ = data_->status_.externalEncoderPosition;      
    }

    return actPos_;
  }
    
  errorLocal = readHwActPos(masterOK);
  if(errorLocal) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, errorLocal);
  }

  errorLocal = readHwLatch();
  if(errorLocal) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, errorLocal);
  }

  errorLocal = readHwWarningError();
  if(errorLocal) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, errorLocal);
  }

  errorLocal = readHwReady();
  if(errorLocal) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, errorLocal);
  }

  // Local source
  if (data_->command_.encSource == ECMC_DATA_SOURCE_INTERNAL) {
    actPos_ = actPosLocal_;
    actVel_ = actVelLocal_;
  } else if (data_->command_.encSource == ECMC_DATA_SOURCE_EXTERNAL && 
      data_->command_.primaryEncIndex == index_) { // External source
    actPos_ = data_->status_.externalEncoderPosition;
    actVel_ = data_->status_.externalEncoderVelocity;
  }

  // Update Asyn
  encPosAct_->refreshParamRT(0);
  encVelAct_->refreshParamRT(0);

  masterOKOld_ = masterOK;
  return actPos_;
}

int ecmcEncoder::writeEntries() {
  if (encLatchFunctEnabled_) {
    if (writeEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_LATCH_CONTROL,
                          (encLatchControl_ > 0))) {
      setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_ENC_ENTRY_READ_FAIL);
    }
  }
  
  int errorCode = 0;
  // write reset
  if (hwResetDefined_) {
    errorCode =
      writeEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_RESET,
                        (uint64_t)hwReset_);
    hwReset_ = 0;
    if (errorCode) {
      setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
  }
 
  return 0;
}

int ecmcEncoder::validate() {
  
  int errorCode = 0;
  if (sampleTime_ <= 0) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_ENC_INVALID_SAMPLE_TIME);
  }

  if (scaleDenom_ == 0) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_ENC_SCALE_DENOM_ZERO);
  }

  if ((encType_ != ECMC_ENCODER_TYPE_ABSOLUTE) &&
      (encType_ != ECMC_ENCODER_TYPE_INCREMENTAL)) {
    return setErrorID(__FILE__, __FUNCTION__, __LINE__,
                      ERROR_ENC_TYPE_NOT_SUPPORTED);
  }

  hwActPosDefined_ = false;
  if(data_->command_.encSource == ECMC_DATA_SOURCE_INTERNAL) {
    if(checkEntryExist(ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION)) {
      errorCode = validateEntry(ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION);
  
      if (errorCode) {  // Act position
        return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_ENC_ENTRY_NULL);
      }
      hwActPosDefined_ = true;
    }
  }

  // Check if latch entries are linked then "enable" latch funct
  if (checkEntryExist(ECMC_ENCODER_ENTRY_INDEX_LATCH_STATUS) &&
      checkEntryExist(ECMC_ENCODER_ENTRY_INDEX_LATCH_VALUE)  &&
      checkEntryExist(ECMC_ENCODER_ENTRY_INDEX_LATCH_CONTROL)) {
    encLatchFunctEnabled_ = !validateEntry(
      ECMC_ENCODER_ENTRY_INDEX_LATCH_STATUS) &&
                            !validateEntry(ECMC_ENCODER_ENTRY_INDEX_LATCH_VALUE)
                            &&
                            !validateEntry(
      ECMC_ENCODER_ENTRY_INDEX_LATCH_CONTROL);
  } else {
    encLatchFunctEnabled_ = false;
  }

  // Check reset link
  if (checkEntryExist(ECMC_ENCODER_ENTRY_INDEX_RESET)) {
    errorCode = validateEntry(ECMC_ENCODER_ENTRY_INDEX_RESET);
    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
    hwResetDefined_ = true;
  }

  // Check warning link
  if (checkEntryExist(ECMC_ENCODER_ENTRY_INDEX_WARNING)) {
    errorCode = validateEntry(ECMC_ENCODER_ENTRY_INDEX_WARNING);
    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
    hwWarningDefined_ = true;
  }

  // Check alarm link 0
  if (checkEntryExist(ECMC_ENCODER_ENTRY_INDEX_ALARM_0)) {
    errorCode = validateEntry(ECMC_ENCODER_ENTRY_INDEX_ALARM_0);
    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
    hwErrorAlarm0Defined_ = true;
  }

  // Check alarm link 1
  if (checkEntryExist(ECMC_ENCODER_ENTRY_INDEX_ALARM_1)) {
    errorCode = validateEntry(ECMC_ENCODER_ENTRY_INDEX_ALARM_1);
    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
    hwErrorAlarm1Defined_ = true;
  }

  // Check alarm link 2
  if (checkEntryExist(ECMC_ENCODER_ENTRY_INDEX_ALARM_2)) {
    errorCode = validateEntry(ECMC_ENCODER_ENTRY_INDEX_ALARM_2);
    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
    hwErrorAlarm2Defined_ = true;
  }

  // Check ready link
  if (checkEntryExist(ECMC_ENCODER_ENTRY_INDEX_READY)) {
    errorCode = validateEntry(ECMC_ENCODER_ENTRY_INDEX_READY);
    if (errorCode) {
      return setErrorID(__FILE__, __FUNCTION__, __LINE__, errorCode);
    }
    hwReadyBitDefined_ = true;
  }

  if(encPosAct_ == NULL) {
    LOGERR("%s/%s:%d: ERROR (axis %d): Encoder asyn param object NULL (encPosAct_) for encoder %d (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_->axisId_,
           index_,
           ERROR_ENC_ASYN_PARAM_NULL);

    return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_ENC_ASYN_PARAM_NULL );
  }

  if(encVelAct_ == NULL) {
    LOGERR("%s/%s:%d: ERROR (axis %d): Encoder asyn param object NULL (encVelAct_) for encoder %d (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_->axisId_,
           index_,
           ERROR_ENC_ASYN_PARAM_NULL);

    return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_ENC_ASYN_PARAM_NULL);
  }

  return 0;
}

// Set encoder value to zero at startup if incremental
int ecmcEncoder::setToZeroIfRelative() {
  if (encType_ == ECMC_ENCODER_TYPE_INCREMENTAL) {
    setActPos(0);
  }
  return 0;
}

int ecmcEncoder::countTrailingZerosInMask(uint64_t mask) {
  if (mask == 0) {
    return -1;
  }
  int zeros = 0;

  while (mask % 2 == 0) {
    zeros++;
    mask >>= 1;
  }
  return zeros;
}

// Count bit width of mask
int ecmcEncoder::countBitWidthOfMask(uint64_t mask, int trailZeros) {
  // Shift away trailing zeros
  mask = mask >> trailZeros;
  uint64_t maskNoTrailingZeros = mask;

  // Count all ones in a "row" (without zeros)
  int ones = 0;

  while (mask & 1) {
    ones += 1;
    mask  = mask >> 1;
  }

  // ensure no more ones in more significant part (must be a cont. ones)
  if (maskNoTrailingZeros > (pow(2, ones) - 1)) {
    return -1;
  }
  return ones;
}

/*
* Return if encoder latch entries are linked and valid
*/
bool ecmcEncoder::getLatchFuncEnabled() {
  return encLatchFunctEnabled_;
}

/*
* Arm encoder hardware latch
*/
void ecmcEncoder::setArmLatch(bool arm) {
  encLatchControl_ = arm;
}

/*
* Return arm state of encoder hardware latch
*/
bool ecmcEncoder::getArmLatch() {
  return encLatchControl_;
}

/*
* New value latched (only high during one cycle)
*/
bool ecmcEncoder::getNewValueLatched() {
  return encLatchStatus_ > encLatchStatusOld_;
}

/*
* Return last latched encoder value in engineering units
*/
double ecmcEncoder::getLatchPosEng() {
  return actEncLatchPos_;
}

/*
* Over or underflow occured
*/
ecmcOverUnderFlowType ecmcEncoder::getOverUnderflow() {
  if (rawTurnsOld_ > rawTurns_) {
    return ECMC_ENC_UNDERFLOW;
  } else if (rawTurnsOld_ < rawTurns_) {
    return ECMC_ENC_OVERFLOW;
  }
  return ECMC_ENC_NORMAL;
}

/*
* absolute range in engineering units
*/
double ecmcEncoder::getAbsRangeEng() {
  return std::abs(rawAbsRange_ * scale_);
}

/*
* absolute range in counts
*/
int64_t ecmcEncoder::getAbsRangeRaw() {
  return rawAbsRange_;
}

/*
* Set velocity filter size (to get stable velocity 
  if resolution is poor compared to sample rate) 
*/
int ecmcEncoder::setVeloFilterSize(size_t size) {
  if(size<1) {
    size=1;
  }
  return velocityFilter_->setFilterSize(size);
}

/*
* Set position filter size (to get stable position
  if resolution is poor compared to sample rate)
*/
int ecmcEncoder::setPosFilterSize(size_t size) {
  
  if(size<=1) {
    enablePositionFilter_ = false;
  }
  else {
    enablePositionFilter_ = true;
  }

  return positionFilter_->setFilterSize(size);
}

/*
* Set position filter enable (to get stable position
  if resolution is poor compared to sample rate)
*/
int ecmcEncoder::setPosFilterEnable(bool enable) {
  enablePositionFilter_ = enable;
  return 0;
}

void ecmcEncoder::errorReset() {
  
  // Reset hardware if needed
  if(hwResetDefined_) {
    hwReset_ = 1;
  }

  ecmcError::errorReset();
}

int ecmcEncoder::setRefToOtherEncAtStartup(int encIndex) {
  refEncIndex_ = encIndex;
  return 0;
}

int ecmcEncoder::getRefToOtherEncAtStartup() {
  return refEncIndex_;
}

uint8_t* ecmcEncoder::getActPosPtr() {
  return (uint8_t*)&actPos_;
}

uint8_t* ecmcEncoder::getActVelPtr() {
  return (uint8_t*)&actVel_;
}

int ecmcEncoder::setRefAtHoming(int refEnable) {
  
  // do not change if -1. Allow ecmccfg to set default value
  if(refEnable < 0) {
    return 0;
  }

  refDuringHoming_ = refEnable;
  return 0;
}

bool ecmcEncoder::getRefAtHoming() {
  return refDuringHoming_;
}

void ecmcEncoder::setHomeLatchCountOffset(int count) {
  homeLatchCountOffset_ = count;
}

int ecmcEncoder::getHomeLatchCountOffset() {
  return homeLatchCountOffset_;
}

int ecmcEncoder::initAsyn() {

  // Add Asynparms for new encoder
  if (asynPortDriver_ == NULL) {
    LOGERR("%s/%s:%d: ERROR (axis %d): AsynPortDriver object NULL (0x%x).\n",
           __FILE__,
           __FUNCTION__,
           __LINE__,
           data_->axisId_,
           ERROR_AXIS_ASYN_PORT_OBJ_NULL);
    return ERROR_AXIS_ASYN_PORT_OBJ_NULL;
  }

  char buffer[EC_MAX_OBJECT_PATH_CHAR_LENGTH];
  char *name = NULL;
  unsigned int charCount = 0;
  ecmcAsynDataItem *paramTemp = NULL;
  
  // Actpos
  if(index_==0) {  // first encoder will be called actpos
    charCount = snprintf(buffer,
                         sizeof(buffer),
                         ECMC_AX_STR "%d." ECMC_ASYN_ENC_ACT_POS_NAME,
                         data_->axisId_);
   
  } else { // encoder 1..7 will be called actpos1 actpos7
    charCount = snprintf(buffer,
                         sizeof(buffer),
                         ECMC_AX_STR "%d." ECMC_ASYN_ENC_ACT_POS_NAME"%d",
                         data_->axisId_,
                         index_);
  }
  
  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate (%s). Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      ECMC_AX_STR "%d." ECMC_ASYN_ENC_ACT_POS_NAME"%d",
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }

  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                                asynParamFloat64,
                                                getActPosPtr(),
                                                8,
                                                ECMC_EC_F64,
                                                0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  encPosAct_ = paramTemp;

  // Actvel
  if(index_ == 0) {  // first encoder will be called actvel
    charCount = snprintf(buffer,
                         sizeof(buffer),
                         ECMC_AX_STR "%d." ECMC_ASYN_ENC_ACT_VEL_NAME,
                         data_->axisId_);
  } else { // encoder 1..7 will be called actvel1 actvel7
    charCount = snprintf(buffer,
                         sizeof(buffer),
                         ECMC_AX_STR "%d." ECMC_ASYN_ENC_ACT_VEL_NAME"%d",
                         data_->axisId_,
                         index_);
  }

  if (charCount >= sizeof(buffer) - 1) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Failed to generate (%s). Buffer to small (0x%x).\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      ECMC_AX_STR "%d." ECMC_ASYN_ENC_ACT_POS_NAME"%d",
      ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL);
    return ERROR_AXIS_ASYN_PRINT_TO_BUFFER_FAIL;
  }
  
  name = buffer;
  paramTemp = asynPortDriver_->addNewAvailParam(name,
                                                asynParamFloat64,
                                                getActVelPtr(),
                                                8,
                                                ECMC_EC_F64,
                                                0);
  if(!paramTemp) {
    LOGERR(
      "%s/%s:%d: ERROR (axis %d): Add create default parameter for %s failed.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__,
      data_->axisId_,
      name);
    return ERROR_MAIN_ASYN_CREATE_PARAM_FAIL;
  }
  paramTemp->setAllowWriteToEcmc(false);
  paramTemp->refreshParam(1);
  encVelAct_ = paramTemp;
  return 0;
}

void   ecmcEncoder::setMaxPosDiffToPrimEnc(double distance) {
  maxPosDiffToPrimEnc_ = std::abs(distance);
}

double ecmcEncoder::getMaxPosDiffToPrimEnc() {
  return maxPosDiffToPrimEnc_;
}
