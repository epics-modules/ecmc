/*
 * ecmcPLC_libEc.h
 *
 *  Created on: Nov 19, 2018
 *      Author: anderssandstrom
 */

#ifndef ecmcPLC_libEc_H_
#define ecmcPLC_libEc_H_

/*
* Functions available in exprssion TK
*/
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))

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

inline double ec_get_err()
{  
  return (double)ec_errorCode;
}

#endif /* ecmcPLC_libEc_H_ */

