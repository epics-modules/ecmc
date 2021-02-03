/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPLCTask_libEc.inc
*
*  Created on: Nov 19, 2018
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ecmcPLC_libEc_inc_
#define ecmcPLC_libEc_inc_

#define ECMC_EC_LIB_MAX_BITS 64

#define ERROR_PLC_EC_LIB_BITS_OUT_OF_RANGE 0x20800

ecmcEc *ecmcPLCTask::statEc_=NULL;

#define CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(bits) { \
  ec_errorCode=0;                                        \
  if(bits>=ECMC_EC_LIB_MAX_BITS || bits<0){              \
    ec_errorCode=ERROR_PLC_EC_LIB_BITS_OUT_OF_RANGE;     \
    LOGERR("ERROR: Bit position out of range.\n");       \
    return value;                                        \
  }                                                      \
}                                                        \

const char* ecLibCmdList[] = { "ec_set_bit(",
                               "ec_clr_bit(",
                               "ec_flp_bit(",
                               "ec_chk_bit(",
                               "ec_print_bin(",
                               "ec_print_hex(",
                               "ec_get_err(",
                               "ec_wrt_bit(",
                               "ec_mm_cp(",
                               "ec_err_rst(",
                               "ec_wrt_bits(",
                               "ec_chk_bits(",
                               "ec_get_time(",
                               "ec_get_mm_type(",
                               "ec_get_mm_data(",
                               "ec_set_mm_data(",
                               "ec_get_mm_size(",
                               "ec_get_time_l32(",
                               "ec_get_time_u32(",
                               "ec_mm_append_to_ds(",
                               "ec_mm_append_to_ds_scale_offset"
                              };

static int ec_errorCode = 0;
static int ec_cmd_count = 21;

inline double ec_set_bit(double value, double bitIndex)
{
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(bitIndex);
  uint64_t temp=(uint64_t)value;  
  return (double)BIT_SET(temp,(int)bitIndex);
}

inline double ec_wrt_bit(double value, double wrtValue,double bitIndex)
{
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(bitIndex);
  uint64_t temp=(uint64_t)value;
  if(wrtValue>0) {
    return (double)BIT_SET(temp,(int)bitIndex);
  }
  else {
    return (double)BIT_CLEAR(temp,(int)bitIndex);
  }
}

inline double ec_wrt_bits(double value,
                          double wrtValue,
                          double startBit,
                          double stopBit)
{  
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(startBit);
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(stopBit);
  uint64_t temp=(uint64_t)value;
  uint64_t data=(uint64_t)wrtValue;
  uint64_t mask=-1;
  if((stopBit-startBit+1)<64) {
    mask=((1LU << ((int)(stopBit-startBit+1)))-1);
  }
  
  return (temp & ~(mask << (int)startBit)) | ((data & mask) << (int)startBit);
}

inline double ec_chk_bits(double value, double startBit, double stopBit)
{  
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(startBit);
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(stopBit);
  
  uint64_t temp=(uint64_t)value;
  uint64_t mask=-1 <<(int)startBit;
  if((stopBit-startBit+1)<64) {
    mask=((1LU << ((int)(stopBit-startBit+1)))-1)<<(int)startBit;
  }   
  
  return (double)((temp & mask) >> (int)startBit);
}

inline double ec_get_time() {
  ec_errorCode = 0;
  if(!ecmcPLCTask::statEc_) {
    ec_errorCode = ERROR_MAIN_EC_MASTER_NULL;
    return -(double)ec_errorCode;
  }

  return (double)ecmcPLCTask::statEc_->getTimeNs();
}

inline double ec_get_time_l32() {
  ec_errorCode = 0;
  if(!ecmcPLCTask::statEc_) {
    ec_errorCode = ERROR_MAIN_EC_MASTER_NULL;
    return -(double)ec_errorCode;
  }  
  return (double)((uint32_t)ecmcPLCTask::statEc_->getTimeNs());
}

inline double ec_get_time_u32() {
  ec_errorCode = 0;
  if(!ecmcPLCTask::statEc_) {
    ec_errorCode = ERROR_MAIN_EC_MASTER_NULL;
    return -(double)ec_errorCode;
  }  
  return (double)((uint32_t) (ecmcPLCTask::statEc_->getTimeNs() >> 32));
}

inline double ec_clr_bit(double value, double bitIndex)
{
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(bitIndex);
  uint64_t temp=(uint64_t)value;  
  return (double)BIT_CLEAR(temp,(int)bitIndex);
}

inline double ec_flp_bit(double value, double bitIndex)
{
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(bitIndex);
  uint64_t temp=(uint64_t)value;  
  return (double)BIT_FLIP(temp,(int)bitIndex);
}

inline double ec_chk_bit(double value, double bitIndex)
{
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(bitIndex);
  uint64_t temp=(uint64_t)value;  
  return (double)BIT_CHECK(temp,(int)bitIndex)>0;
}

inline double ec_print_hex(double value, double startBit,double stopBit)
{
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(startBit);
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(stopBit);
  
  uint64_t temp=(uint64_t)value;
  uint64_t mask=-1 <<(int)startBit;
  if((stopBit-startBit+1)<64) {
    mask=((1LU << ((int)(stopBit-startBit+1)))-1)<<(int)startBit;
  }   
  printf("%" PRIx64 "",(temp & mask) >> (int)startBit);  
  return 0;
}

inline double ec_print_bin(double value, double startBit,double stopBit)
{
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(startBit);
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(stopBit);
  
  uint64_t temp=(uint64_t)value;
  for(int i=stopBit;i>=startBit;i--){
    printf("%u",BIT_CHECK(temp,i) ? 1:0);    
  }
  
  return 0;
}

inline double ec_get_err()
{  
  return (double)ec_errorCode;
}

// Mem map copy
inline double ec_mm_cp(double srcId, double destId) {
  ec_errorCode = 0;
  if(!ecmcPLCTask::statEc_) {
    ec_errorCode = ERROR_MAIN_EC_MASTER_NULL;
    return (double)ec_errorCode;
  }
  
  ecmcEcMemMap *src = ecmcPLCTask::statEc_->getMemMap((int)srcId);
  ecmcEcMemMap *dest = ecmcPLCTask::statEc_->getMemMap((int)destId);
  if(!src || !dest) {
    ec_errorCode = ERROR_MAIN_MEM_MAP_NULL;
    return (double)ec_errorCode;
  }

  //Size to copy
  int sizeToCp = src->getByteSize();
  int sizeDest = dest->getByteSize();
  if(sizeDest<sizeToCp) {
    sizeToCp = sizeDest;
  }

  //Write data from src to dest 
  memcpy(dest->getBufferPointer(),src->getBufferPointer(),sizeToCp);
  return ec_errorCode;
}

// Get mm data type
inline double ec_get_mm_type(double srcId) {
  ec_errorCode = 0;
  if(!ecmcPLCTask::statEc_) {
    ec_errorCode = ERROR_MAIN_EC_MASTER_NULL;
    return -(double)ec_errorCode;
  }
  
  ecmcEcMemMap *src = ecmcPLCTask::statEc_->getMemMap((int)srcId);
  
  if(!src) {
    ec_errorCode = ERROR_MAIN_MEM_MAP_NULL;
    return -(double)ec_errorCode;
  }
  
  return (double)src->getDataType();
}

// Get mm data
inline double ec_get_mm_data(double srcId,double index) {
  ec_errorCode = 0;
  if(!ecmcPLCTask::statEc_) {
    ec_errorCode = ERROR_MAIN_EC_MASTER_NULL;
    return 0;
  }
  
  ecmcEcMemMap *src = ecmcPLCTask::statEc_->getMemMap((int)srcId);
  
  if(!src) {
    ec_errorCode = ERROR_MAIN_MEM_MAP_NULL;
    return 0;
  }

  double dataTemp = 0;

  ec_errorCode = src->getDoubleDataAtIndex((int)index,&dataTemp);
  if(ec_errorCode) {
    return 0;
  }

  return dataTemp;
}

// Set mm data type
inline double ec_set_mm_data(double srcId,double index, double data) {
  ec_errorCode = 0;
  if(!ecmcPLCTask::statEc_) {
    ec_errorCode = ERROR_MAIN_EC_MASTER_NULL;
    return 0;
  }
  
  ecmcEcMemMap *src = ecmcPLCTask::statEc_->getMemMap((int)srcId);
  
  if(!src) {
    ec_errorCode = ERROR_MAIN_MEM_MAP_NULL;
    return 0;
  }

  ec_errorCode = src->setDoubleDataAtIndex((int)index,data);

  return ec_errorCode;
}

// Get mm data size
inline double ec_get_mm_size(double srcId) {
  ec_errorCode = 0;
  if(!ecmcPLCTask::statEc_) {
    ec_errorCode = ERROR_MAIN_EC_MASTER_NULL;
    return 0;
  }
  
  ecmcEcMemMap *src = ecmcPLCTask::statEc_->getMemMap((int)srcId);
  
  if(!src) {
    ec_errorCode = ERROR_MAIN_MEM_MAP_NULL;
    return 0;
  }

  return (double)src->getElementCount();
}

// Append mm data to ds
inline double ec_mm_append_to_ds(double mmId,double dsId) {
  ec_errorCode = 0;
  if(!ecmcPLCTask::statEc_) {
    ec_errorCode = ERROR_MAIN_EC_MASTER_NULL;
    return 0;
  }
  
  ecmcEcMemMap *src = ecmcPLCTask::statEc_->getMemMap((int)mmId);
  
  if(!src) {
    ec_errorCode = ERROR_MAIN_MEM_MAP_NULL;
    return (double)ec_errorCode;
  }
  
  int index = (int)dsId;
  CHECK_PLC_DATA_STORAGE_RETURN_IF_ERROR(index);
  
  int mmSize = src->getElementCount();
  double dataTemp = 0;
  //TODO: This is slow... Better solution needed. Better to make a plugin
  for(int i=0;i<mmSize;i++) {

    ec_errorCode = src->getDoubleDataAtIndex((int)i,&dataTemp);
    if(ec_errorCode) {
      return (double)ec_errorCode;
    }
    ec_errorCode = ecmcPLCTask::statDs_[index]->appendData(dataTemp, i==mmSize-1);

  }

  return (double)ec_errorCode;
}

// Append mm data to ds and scale and offset
inline double ec_mm_append_to_ds_scale_offset(double mmId,double dsId,double dScale,double dOffset) {
  ec_errorCode = 0;
  if(!ecmcPLCTask::statEc_) {
    ec_errorCode = ERROR_MAIN_EC_MASTER_NULL;
    return 0;
  }
  
  ecmcEcMemMap *src = ecmcPLCTask::statEc_->getMemMap((int)mmId);
  
  if(!src) {
    ec_errorCode = ERROR_MAIN_MEM_MAP_NULL;
    return (double)ec_errorCode;
  }
  
  int index = (int)dsId;
  CHECK_PLC_DATA_STORAGE_RETURN_IF_ERROR(index);
  
  int mmSize = src->getElementCount();
  double dataTemp = 0;
  //TODO: This is slow... Better solution needed. Better to make a plugin
  for(int i=0;i<mmSize;i++) {

    ec_errorCode = src->getDoubleDataAtIndex((int)i,&dataTemp);
    if(ec_errorCode) {
      return (double)ec_errorCode;
    }
    // update asyn only on last element
    ec_errorCode = ecmcPLCTask::statDs_[index]->appendData(dataTemp * dScale + dOffset, i==mmSize-1);
  }

  return (double)ec_errorCode;
}

// Reset error
inline double ec_err_rst() {
  ec_errorCode = 0;
  return 0;
}

#endif /* ecmcPLC_libEc_inc_ */
