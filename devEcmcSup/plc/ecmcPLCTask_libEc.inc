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
                              };

static int ec_errorCode = 0;
static int ec_cmd_count = 13;

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
  if(!ecmcPLCTask::statEc_) {
    ec_errorCode = ERROR_MAIN_EC_MASTER_NULL;
    return -(double)ec_errorCode;
  }

  return (double)ecmcPLCTask::statEc_->getTimeNs();
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

// Reset error
inline double ec_err_rst() {
  ec_errorCode = 0;
  return 0;
}

#endif /* ecmcPLC_libEc_inc_ */


