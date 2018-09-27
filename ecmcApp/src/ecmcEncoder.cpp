/*
 * ecmcEncoder.cpp
 *
 *  Created on: Mar 14, 2016
 *      Author: anderssandstrom
 */

#include "ecmcEncoder.h"

ecmcEncoder::ecmcEncoder(ecmcAxisData *axisData,double sampleTime) : ecmcEcEntryLink()
{
  PRINT_ERROR_PATH("axis[%d].encoder.error",axisData->axisId_);
  data_=axisData;
  sampleTime_=sampleTime;
  initVars();
  if(!data_){
    LOGERR("%s/%s:%d: DATA OBJECT NULL.\n",__FILE__,__FUNCTION__,__LINE__);
    exit(EXIT_FAILURE);
  }
  velocityFilter_=new ecmcFilter(sampleTime);
  if(!velocityFilter_){
    LOGERR("%s/%s:%d: FAILED TO ALLOCATE MEMORY FOR VELOCITY-FILTER OBJECT.\n",__FILE__,__FUNCTION__,__LINE__);
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_ENC_VELOCITY_FILTER_NULL);
    exit(EXIT_FAILURE);
  }
  LOGINFO15("%s/%s:%d: axis[%d].encoder=new;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_);
  printCurrentState();
}

ecmcEncoder::~ecmcEncoder()
{
  delete velocityFilter_;
}

void ecmcEncoder::printCurrentState()
{
  switch(encType_){
    case ECMC_ENCODER_TYPE_INCREMENTAL:
      LOGINFO10("%s/%s:%d: axis[%d].encoder.type=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_ENCODER_TYPE_INCREMENTAL");
      break;
    case ECMC_ENCODER_TYPE_ABSOLUTE:
      LOGINFO10("%s/%s:%d: axis[%d].encoder.type=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_ENCODER_TYPE_ABSOLUTE");
      break;
    default:
      LOGINFO10("%s/%s:%d: axis[%d].encoder.type=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,encType_);
      break;
  }
  LOGINFO15("%s/%s:%d: axis[%d].encoder.enable=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,data_->command_.enable>0);
  LOGINFO15("%s/%s:%d: axis[%d].encoder.scaleNum=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,scaleNum_);
  LOGINFO15("%s/%s:%d: axis[%d].encoder.scaleDenom=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,scaleDenom_);
  LOGINFO15("%s/%s:%d: axis[%d].encoder.offset=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,offset_);
  LOGINFO15("%s/%s:%d: axis[%d].encoder.actPos=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,actPos_);
  LOGINFO15("%s/%s:%d: axis[%d].encoder.homed=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,homed_);
  LOGINFO15("%s/%s:%d: axis[%d].encoder.bits=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,bits_);
}

void ecmcEncoder::initVars()
{
  errorReset();
  encType_=ECMC_ENCODER_TYPE_INCREMENTAL;
  rawPos_=0;
  range_=0;
  limit_=0;
  bits_=0;
  rawPosUintOld_=0;
  rawPosUint_=0;
  turns_=0;
  scale_=0;
  offset_=0;
  actPos_=0;
  actPosOld_=0;
  sampleTime_=1;
  actVel_=0;
  homed_=false;
  scaleNum_=0;
  scaleDenom_=1;
  absBits_=0;
  totalRawMask_=ECMC_ENCODER_MAX_VALUE_64_BIT;
  totalRawOffset_=0;
}

int64_t ecmcEncoder::getRawPos()
{
  return rawPos_;
}

int ecmcEncoder::setScaleNum(double scaleNum)
{
  if(scaleNum_!=scaleNum)
  {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.scaleNum=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,scaleNum);
  }

  scaleNum_=scaleNum;
  if(std::abs(scaleDenom_)>0){
    scale_=scaleNum_/scaleDenom_;
  }
  return 0;
}

int ecmcEncoder::setScaleDenom(double scaleDenom)
{
  if(scaleDenom_!=scaleDenom)
  {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.scaleDenom=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,scaleDenom);
  }

  scaleDenom_=scaleDenom;
  if(scaleDenom_==0){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_ENC_SCALE_DENOM_ZERO);
  }
  scale_=scaleNum_/scaleDenom_;
  return 0;
}

double ecmcEncoder::getScale()
{
  return scale_;
}

double ecmcEncoder::getActPos()
{
  return actPos_;
}

void ecmcEncoder::setActPos(double pos)
{
  if(actPos_!=pos)
  {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.actPos=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,pos);
  }

  //calculate new offset
  offset_=offset_+pos-actPos_;
  actPosOld_=pos;
  actPos_=pos;
  //Must clear velocity filter
  velocityFilter_->initFilter(pos);
}

int ecmcEncoder::setOffset(double offset)
{
  if(offset_!=offset)
  {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.offset=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,offset);
  }

  offset_=offset;
  return 0;
}

double ecmcEncoder::getSampleTime()
{
  return sampleTime_;
}

double ecmcEncoder::getActVel()
{
  return actVel_;
}

void ecmcEncoder::setHomed(bool homed)
{
  if(homed_!=homed)
  {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.homed=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,homed);
  }
  homed_=homed;
}

bool ecmcEncoder::getHomed()
{
  return homed_;
}

int ecmcEncoder::setType(encoderType encType)
{
  switch(encType){
    case ECMC_ENCODER_TYPE_ABSOLUTE:
      if(encType_!=encType)
      {
        LOGINFO15("%s/%s:%d: axis[%d].encoder.type=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_ENCODER_TYPE_ABSOLUTE");
      }
      encType_=encType;
      homed_=true;
      break;
    case ECMC_ENCODER_TYPE_INCREMENTAL:
      if(encType_!=encType)
      {
        LOGINFO15("%s/%s:%d: axis[%d].encoder.type=%s;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,"ECMC_ENCODER_TYPE_INCREMENTAL");
      }
      encType_=encType;
      break;
    default:
      return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_ENC_TYPE_NOT_SUPPORTED);
      break;
  }
  validate();
  return 0;
}

encoderType ecmcEncoder::getType()
{
  return encType_;
}

int64_t ecmcEncoder::handleOverUnderFlow(uint64_t newValue, int bits)
{
  rawPosUintOld_=rawPosUint_;
  rawPosUint_=newValue;
  if(bits<64){//Only support for over/under flow of datatypes less than 64 bit
    if(rawPosUintOld_>rawPosUint_ && rawPosUintOld_-rawPosUint_>limit_){//Overflow
      turns_++;
    }
    else{
      if(rawPosUintOld_<rawPosUint_ &&rawPosUint_-rawPosUintOld_ >limit_){//Underflow
        turns_--;
      }
    }
  }
  else{
    turns_=0;
  }

  return turns_*range_+rawPosUint_;
}

int ecmcEncoder::setBits(int bits)
{
  if(bits==0){
	// Special case.. Need to support this since otherwise axis with external source will lead to config error
    bits_=0;
    range_=0;
    limit_=0;
    totalRawOffset_=0;
    totalRawMask_=0;
    if(bits_!=bits){
      LOGINFO15("%s/%s:%d: axis[%d].encoder.bits=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,bits);
    }
	return 0;
  }

  int errorCode=setRawMask((uint64_t)(pow(2,bits)-1));
  if(errorCode){
    return errorCode;
  }
  if(bits_!=bits){
    LOGINFO15("%s/%s:%d: axis[%d].encoder.bits=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,bits);
  }
  return 0;
}

int ecmcEncoder::setAbsBits(int absBits)
{
  if(absBits_!=absBits){
    LOGINFO15("%s/%s:%d: axis[%d].encoder.absbits=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,absBits);
  }

  absBits_=absBits;
  return 0;
}

int ecmcEncoder::setRawMask(uint64_t mask)
{
  int trailingZeros=countTrailingZerosInMask(mask);
  if(trailingZeros<0){
    LOGERR("%s/%s:%d: Encoder Raw Mask Invalid. Mask not allowed to be 0 (0x%x).\n",__FILE__,__FUNCTION__,__LINE__,ERROR_ENC_RAW_MASK_INVALID);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_ENC_RAW_MASK_INVALID);
  }
  int bitWidth=countBitWidthOfMask(mask,trailingZeros);
  if(bitWidth<0){
    LOGERR("%s/%s:%d: Encoder Raw Mask Invalid. Mask must be continuous with ones (0x%x).\n",__FILE__,__FUNCTION__,__LINE__,ERROR_ENC_RAW_MASK_INVALID);
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_ENC_RAW_MASK_INVALID);
  }

  bits_=bitWidth;
  range_=pow(2,bits_)-1;
  limit_=range_*2/3;  //Limit for over/under-flow

  if(totalRawOffset_!=mask){
    LOGINFO15("%s/%s:%d: axis[%d].encoder.rawmask=%"PRIx64";\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,mask);
  }
  totalRawOffset_=pow(2,trailingZeros)-1;
  totalRawMask_=mask;
  return 0;
}

double ecmcEncoder::getScaleNum()
{
  return scaleNum_;
}

double ecmcEncoder::getScaleDenom()
{
  return scaleDenom_;
}

double ecmcEncoder::readEntries()
{
  actPosOld_=actPos_;

  if(getError()){
      rawPos_=0;
      actPos_=scale_*rawPos_+offset_;
      return actPos_;
  }

  int error=validateEntry(ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION);
  if(error){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_ENC_ENTRY_NULL);
    return 0;
  }

  //Act position
  uint64_t tempRaw=0;

  if(readEcEntryValue(ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION,&tempRaw)){
    setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_ENC_ENTRY_READ_FAIL);
    return actPos_;
  }
  //Apply mask
  tempRaw=(totalRawMask_ & tempRaw)-totalRawOffset_;  //Apply masks and offset
  rawPos_=handleOverUnderFlow(tempRaw,bits_) ;
  actPosOld_=actPos_;
  actPos_=scale_*rawPos_+offset_;
  actVel_=velocityFilter_->positionBasedVelAveraging(actPos_);

  return actPos_;
}

int ecmcEncoder::validate()
{

  if(sampleTime_<=0){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_ENC_INVALID_SAMPLE_TIME);
  }

  if(scaleDenom_==0){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_ENC_SCALE_DENOM_ZERO);
  }

  if(encType_!=ECMC_ENCODER_TYPE_ABSOLUTE && encType_!= ECMC_ENCODER_TYPE_INCREMENTAL){
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_ENC_TYPE_NOT_SUPPORTED);
  }

  int errorCode=validateEntry(ECMC_ENCODER_ENTRY_INDEX_ACTUAL_POSITION);
  if(errorCode){   //Act position
    return setErrorID(__FILE__,__FUNCTION__,__LINE__,ERROR_ENC_ENTRY_NULL);
  }

  return 0;
}

//Set encoder value to zero at startup if incremental
int ecmcEncoder::setToZeroIfRelative()
{
  if(encType_==ECMC_ENCODER_TYPE_INCREMENTAL){
	setActPos(0);
  }
  return 0;
}

int ecmcEncoder::countTrailingZerosInMask(uint64_t mask)
{
  if(mask==0){
	return -1;
  }
  int zeros=0;
  while(mask % 2==0){
	zeros++;
	mask>>=1;
  }
  printf("==========================countTrailingZerosInMask=%d\n",zeros);
  return zeros;
}

//Count bit width of mask
int ecmcEncoder::countBitWidthOfMask(uint64_t mask,int trailZeros)
{
  //Shift away trailing zeros
  mask = mask>> trailZeros;
  uint64_t maskNoTrailingZeros = mask;

  //Count all ones in a "row" (without zeros)
  int ones=0;
  while(mask & 1){
    ones += 1;
    mask = mask >> 1;
  }

  //ensure no more ones in more significant part (must be a cont. ones)
  if(maskNoTrailingZeros>(pow(2,ones)-1)){
	return -1;
  }
  printf("==========================countBitWidthOfMask=%d\n",ones);
  return ones;
}


