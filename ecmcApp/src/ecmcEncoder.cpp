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
  LOGINFO15("%s/%s:%d: axis[%d].encoder.sampleTime=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,sampleTime_);
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

void ecmcEncoder::setOffset(double offset)
{
  if(offset_!=offset)
  {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.offset=%lf;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,offset);
  }

  offset_=offset;
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
  if(bits_<64){//Only support for over/under flow of datatypes less than 64 bit
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
  if(bits_!=bits)
  {
    LOGINFO15("%s/%s:%d: axis[%d].encoder.bits=%d;\n",__FILE__, __FUNCTION__, __LINE__,data_->axisId_,bits);
  }

  bits_=bits;
  range_=pow(2,bits_);
  limit_=range_*2/3;  //Limit for change in value
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
