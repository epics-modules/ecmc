/*
 * ecmcEncoder.cpp
 *
 *  Created on: Mar 14, 2016
 *      Author: anderssandstrom
 */

#include "ecmcEncoder.h"

ecmcEncoder::ecmcEncoder(ecmcAxisData *axisData,
                         double        sampleTime) : ecmcEcEntryLink() {
  PRINT_ERROR_PATH("axis[%d].encoder.error", axisData->axisId_);
  data_       = axisData;
  sampleTime_ = sampleTime;
  initVars();

  if (!data_) {
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n", __FILE__, __FUNCTION__, __LINE__);
    exit(EXIT_FAILURE);
  }
  velocityFilter_ = new ecmcFilter(sampleTime);

  if (!velocityFilter_) {
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR VELOCITY-FILTER OBJECT.\n",
           __FILE__,
           __FUNCTION__,
           __LINE__);
    setErrorID(__FILE__, __FUNCTION__, __LINE__,
               ERROR_ENC_VELOCITY_FILTER_NULL);
    exit(EXIT_FAILURE);
  }
  LOGINFO15("%s/%s:%d: axis[%d].encoder=new;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_);
  printCurrentState();
}

ecmcEncoder::~ecmcEncoder() {
  delete velocityFilter_;
  velocityFilter_ = NULL;
}

void ecmcEncoder::printCurrentState() {
  switch (encType_) {
  case ECMC_ENCODER_TYPE_INCREMENTAL:
    LOGINFO10("%s/%s:%d: axis[%d].encoder.type=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              "ECMC_ENCODER_TYPE_INCREMENTAL");
    break;

  case ECMC_ENCODER_TYPE_ABSOLUTE:
    LOGINFO10("%s/%s:%d: axis[%d].encoder.type=%s;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              "ECMC_ENCODER_TYPE_ABSOLUTE");
    break;

  default:
    LOGINFO10("%s/%s:%d: axis[%d].encoder.type=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              encType_);
    break;
  }
  LOGINFO15("%s/%s:%d: axis[%d].encoder.enable=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            data_->command_.enable > 0);
  LOGINFO15("%s/%s:%d: axis[%d].encoder.scaleNum=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            scaleNum_);
  LOGINFO15("%s/%s:%d: axis[%d].encoder.scaleDenom=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            scaleDenom_);
  LOGINFO15("%s/%s:%d: axis[%d].encoder.offset=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            engOffset_);
  LOGINFO15("%s/%s:%d: axis[%d].encoder.actPos=%lf;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            actPos_);
  LOGINFO15("%s/%s:%d: axis[%d].encoder.homed=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            homed_);
  LOGINFO15("%s/%s:%d: axis[%d].encoder.bits=%d;\n",
            __FILE__,
            __FUNCTION__,
            __LINE__,
            data_->axisId_,
            bits_);
}

void ecmcEncoder::initVars() {
  errorReset();
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
  homed_                = false;
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
  if (scaleNum_ != scaleNum) {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.scaleNum=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              scaleNum);
  }

  scaleNum_ = scaleNum;

  if (std::abs(scaleDenom_) > 0) {
    scale_ = scaleNum_ / scaleDenom_;
  }
  return 0;
}

int ecmcEncoder::setScaleDenom(double scaleDenom) {
  if (scaleDenom_ != scaleDenom) {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.scaleDenom=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              scaleDenom);
  }

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
  if (actPos_ != pos) {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.actPos=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              pos);
  }

  // reset overflow counter
  engOffset_       = 0;
  rawTurnsOld_     = 0;
  rawTurns_        = 0;
  rawPosOffset_    = pos / scale_ - rawPosUint_;
  rawPosMultiTurn_ = rawPosUint_ + rawPosOffset_;

  actPosOld_ = pos;
  actPos_    = pos;

  // Must clear velocity filter
  velocityFilter_->initFilter(pos);
}

int ecmcEncoder::setOffset(double offset) {
  if (engOffset_ != offset) {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.offset=%lf;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              offset);
  }

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
  if (homed_ != homed) {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.homed=%d;\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              data_->axisId_,
              homed);
  }
  homed_ = homed;
}

bool ecmcEncoder::getHomed() {
  return homed_;
}

int ecmcEncoder::setType(encoderType encType) {
  switch (encType) {
  case ECMC_ENCODER_TYPE_ABSOLUTE:

    if (encType_ != encType) {
      LOGINFO15("%s/%s:%d: axis[%d].encoder.type=%s;\n",
                __FILE__,
                __FUNCTION__,
                __LINE__,
                data_->axisId_,
                "ECMC_ENCODER_TYPE_ABSOLUTE");
    }
    encType_ = encType;
    homed_   = true;
    break;

  case ECMC_ENCODER_TYPE_INCREMENTAL:

    if (encType_ != encType) {
      LOGINFO15("%s/%s:%d: axis[%d].encoder.type=%s;\n",
                __FILE__,
                __FUNCTION__,
                __LINE__,
                data_->axisId_,
                "ECMC_ENCODER_TYPE_INCREMENTAL");
    }
    encType_ = encType;
    break;

  default:
    return setErrorID(__FILE__,
                      __FUNCTION__,
                      __LINE__,
                      ERROR_ENC_TYPE_NOT_SUPPORTED);

    break;
  }
  validate();
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

    if (bits_ != bits) {
      LOGINFO15("%s/%s:%d: axis[%d].encoder.bits=%d;\n",
                __FILE__, __FUNCTION__, __LINE__, data_->axisId_, bits);
    }
    return 0;
  }

  int errorCode = setRawMask((uint64_t)(pow(2, bits) - 1));

  if (errorCode) {
    return errorCode;
  }

  if (bits_ != bits) {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.bits=%d;\n",
              __FILE__, __FUNCTION__, __LINE__, data_->axisId_, bits);
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
    LOGINFO15("%s/%s:%d: axis[%d].encoder.bits=%d;\n",
              __FILE__, __FUNCTION__, __LINE__, data_->axisId_, absBits);
  }

  if (absBits_ != absBits) {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.absbits=%d;\n",
              __FILE__, __FUNCTION__, __LINE__, data_->axisId_, absBits);
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

  bits_     = bitWidth;
  rawRange_ = pow(2, bits_) - 1;
  rawLimit_ = rawRange_ * ECMC_OVER_UNDER_FLOW_FACTOR;  // Limit for over/under-flow

  if (totalRawRegShift_ != mask) {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.rawmask=%" PRIx64 ";\n",
              __FILE__, __FUNCTION__, __LINE__, data_->axisId_, mask);
  }
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

double ecmcEncoder::readEntries() {
  actPosOld_ = actPos_;

  if (getError()) {
    rawPosMultiTurn_ = 0;
    actPos_          = scale_ * (rawPosMultiTurn_ + rawPosOffset_) +
                       engOffset_;
    return actPos_;
  }

  int error = validateEntry(ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION);

  if (error) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_ENC_ENTRY_NULL);
    return 0;
  }

  // Act position
  uint64_t tempRaw = 0;

  if (readEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION, &tempRaw)) {
    setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_ENC_ENTRY_READ_FAIL);
    return actPos_;
  }

  // Filter value with mask
  rawPosUintOld_ = rawPosUint_;
  rawPosUint_    = (totalRawMask_ & tempRaw) - totalRawRegShift_;
  

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
  actPosOld_ = actPos_;
  actPos_    = scale_ * rawPosMultiTurn_ + engOffset_;
  //Not affected by modulo
  actVel_    = velocityFilter_->getFiltVelo(actPos_ - actPosOld_);

  // Check modulo
  if(data_->command_.moduloRange != 0) {    
    if(actPos_ >= data_->command_.moduloRange){
      actPos_ = actPos_-data_->command_.moduloRange;
      // Reset stuff to be able to run forever
      engOffset_       = 0;
      rawTurnsOld_     = 0;
      rawTurns_        = 0;
      rawPosOffset_    = actPos_ / scale_ - rawPosUint_;
      rawPosMultiTurn_ = rawPosUint_ + rawPosOffset_;
    }
    if(actPos_ < 0){
      actPos_ = data_->command_.moduloRange + actPos_;
      // Reset stuff to be able to run forever
      engOffset_       = 0;
      rawTurnsOld_     = 0;
      rawTurns_        = 0;
      rawPosOffset_    = actPos_ / scale_ - rawPosUint_;
      rawPosMultiTurn_ = rawPosUint_ + rawPosOffset_;
    }
  }

  // Encoder latch entries (status and position)
  if (encLatchFunctEnabled_) {
    if (readEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_LATCH_STATUS, &tempRaw)) {
      setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_ENC_ENTRY_READ_FAIL);
    }
    encLatchStatusOld_ = encLatchStatus_;
    encLatchStatus_    = tempRaw > 0;

    if (readEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_LATCH_VALUE, &tempRaw)) {
      setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_ENC_ENTRY_READ_FAIL);
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
  }

  return actPos_;
}

int ecmcEncoder::writeEntries() {
  if (encLatchFunctEnabled_) {
    if (writeEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_LATCH_CONTROL,
                          (encLatchControl_ > 0))) {
      setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_ENC_ENTRY_READ_FAIL);
    }
  }
  return 0;
}

int ecmcEncoder::validate() {
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

  int errorCode = validateEntry(ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION);

  if (errorCode) {  // Act position
    return setErrorID(__FILE__, __FUNCTION__, __LINE__, ERROR_ENC_ENTRY_NULL);
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
* Set volocity filter size (to get stable velocity 
  if resolution is poor compared to sample rate)
*/
int ecmcEncoder::setVeloFilterSize(size_t size) {
  return velocityFilter_->setFilterSize(size);
}