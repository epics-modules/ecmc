/*
 * ecmcPLC_libEc.inc
 *
 *  Created on: Nov 19, 2018
 *      Author: anderssandstrom
 */

#ifndef ecmcPLC_libEc_inc_
#define ecmcPLC_libEc_inc_

#define ECMC_EC_LIB_MAX_BITS 64

#define ERROR_PLC_EC_LIB_BITS_OUT_OF_RANGE 0x20800

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
                              };

static int ec_errorCode=0;
static int ec_cmd_count=sizeof(ecLibCmdList)/sizeof(ecLibCmdList[0]);

inline double ec_set_bit(double value, double bitIndex)
{
  CHECK_PLC_BITS_AND_RETURN_VALUE_IF_ERROR(bitIndex);
  uint64_t temp=(uint64_t)value;  
  return (double)BIT_SET(temp,(int)bitIndex);
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
  temp = temp & ((1U << ((int)stopBit)) - 1); 
  temp = temp << (int)startBit;
  printf("%"PRIx64"",temp);  
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
#endif /* ecmcPLC_libEc_inc_ */


