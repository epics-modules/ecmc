/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcOctetIF.h
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom, torstenbögershausen
*
\*************************************************************************/

#ifndef ECMC_OCTET_IF_H
# define ECMC_OCTET_IF_H

# define __STDC_FORMAT_MACROS // To have PRIx64
# include <inttypes.h>

/* asynPrintf() */
# include "asynDriver.h"

# ifdef __cplusplus
extern "C" {
# endif /* ifdef __cplusplus */


# ifndef ASYN_TRACE_INFO
#  define ASYN_TRACE_INFO      0x0040
# endif /* ifndef ASYN_TRACE_INFO */

/* global to be used by all printing */

// extern  asynUser *pPrintOutAsynUser;
extern  asynUser *pPrintOutAsynUser;

/*
 * logging stuff
 */

extern unsigned int debug_print_flags;
extern unsigned int die_on_error_flags;

/* Where to print: currently to stdout.
 * May be changed in the future similat to asyn
 */

# define ECMC_CMD_BUFFER_SIZE 65536
# define ECMC_CMD_MAX_SINGLE_CMD_LENGTH 4096

# define PRINT_OUT    (1 << 1)

# define PRINT_STDOUT_BIT1() (debug_print_flags & (1 << 1))
# define PRINT_STDOUT_BIT2() (debug_print_flags & (1 << 2))

# define PRINT_STDOUT_BIT4() (debug_print_flags & (1 << 4))
# define PRINT_STDOUT_BIT5() (debug_print_flags & (1 << 5))
# define PRINT_STDOUT_BIT6() (debug_print_flags & (1 << 6))
# define PRINT_STDOUT_BIT7() (debug_print_flags & (1 << 7))
# define PRINT_STDOUT_BIT8() (debug_print_flags & (1 << 8))
# define PRINT_STDOUT_BIT9() (debug_print_flags & (1 << 9))
# define PRINT_STDOUT_BIT10() (debug_print_flags & (1 << 10))
# define PRINT_STDOUT_BIT11() (debug_print_flags & (1 << 11))
# define PRINT_STDOUT_BIT12() (debug_print_flags & (1 << 12))
# define PRINT_STDOUT_BIT13() (debug_print_flags & (1 << 13))
# define PRINT_STDOUT_BIT14() (debug_print_flags & (1 << 14))
# define PRINT_STDOUT_BIT15() (debug_print_flags & (1 << 15))

# define DIE_ON_ERROR_BIT0() (die_on_error_flags & 1)
# define DIE_ON_ERROR_BIT1() (die_on_error_flags & (1 << 1))

# define WRITE_DIAG_BIT(bitnumber, value)     \
  do {                                        \
    if (value) {                              \
      debug_print_flags |= 1 << bitnumber;    \
    } else {                                  \
      debug_print_flags &= ~(1 << bitnumber); \
    }                                         \
  } while (0)

# define FUNCTION_CALL_DIAGNOSTICS_BIT 4
# define FUNCTION_ETHERCAT_DIAGNOSTICS_BIT 5
# define FUNCTION_CMD_EAT_DIAGNOSTICS_BIT 6
# define FUNCTION_AXIS_DIAGNOSTICS_BIT 7
# define FUNCTION_COMMAND_LIST_DIAGNOSTICS_BIT 8
# define FUNCTION_DATA_STORAGE_DIAGNOSTICS_BIT 9
# define FUNCTION_EVENTS_DIAGNOSTICS_BIT 10
# define FUNCTION_DATA_RECORDER_DIAGNOSTICS_BIT 11
# define FUNCTION_HW_MOTOR_AXIS_DIAGNOSTICS_BIT 12
# define FUNCTION_TIMING_DIAGNOSTICS_BIT 13
# define FUNCTION_AXES_ON_CHANGE_DATA_BIT 15

# define LOGINFO(fmt, ...)                                                    \
  {                                                                           \
    (void)asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO, fmt, ## __VA_ARGS__); \
  }

# define LOGINFO4(fmt, ...)                                     \
  do {                                                          \
    if (PRINT_STDOUT_BIT4()) (void)asynPrint(pPrintOutAsynUser, \
                                             ASYN_TRACE_INFO,   \
                                             fmt,               \
                                             ## __VA_ARGS__);   \
  } while (0)

# define LOGINFO5(fmt, ...)                                     \
  do {                                                          \
    if (PRINT_STDOUT_BIT5()) (void)asynPrint(pPrintOutAsynUser, \
                                             ASYN_TRACE_INFO,   \
                                             fmt,               \
                                             ## __VA_ARGS__);   \
  } while (0)

# define LOGINFO6(fmt, ...)                                     \
  do {                                                          \
    if (PRINT_STDOUT_BIT6()) (void)asynPrint(pPrintOutAsynUser, \
                                             ASYN_TRACE_INFO,   \
                                             fmt,               \
                                             ## __VA_ARGS__);   \
  } while (0)

# define LOGINFO7(fmt, ...)                                     \
  do {                                                          \
    if (PRINT_STDOUT_BIT7()) (void)asynPrint(pPrintOutAsynUser, \
                                             ASYN_TRACE_INFO,   \
                                             fmt,               \
                                             ## __VA_ARGS__);   \
  } while (0)

# define LOGINFO8(fmt, ...)                                     \
  do {                                                          \
    if (PRINT_STDOUT_BIT8()) (void)asynPrint(pPrintOutAsynUser, \
                                             ASYN_TRACE_INFO,   \
                                             fmt,               \
                                             ## __VA_ARGS__);   \
  } while (0)

# define LOGINFO9(fmt, ...)                                     \
  do {                                                          \
    if (PRINT_STDOUT_BIT9()) (void)asynPrint(pPrintOutAsynUser, \
                                             ASYN_TRACE_INFO,   \
                                             fmt,               \
                                             ## __VA_ARGS__);   \
  } while (0)

# define LOGINFO10(fmt, ...)                                     \
  do {                                                           \
    if (PRINT_STDOUT_BIT10()) (void)asynPrint(pPrintOutAsynUser, \
                                              ASYN_TRACE_INFO,   \
                                              fmt,               \
                                              ## __VA_ARGS__);   \
  } while (0)

# define LOGINFO11(fmt, ...)                                     \
  do {                                                           \
    if (PRINT_STDOUT_BIT11()) (void)asynPrint(pPrintOutAsynUser, \
                                              ASYN_TRACE_INFO,   \
                                              fmt,               \
                                              ## __VA_ARGS__);   \
  } while (0)

# define LOGINFO12(fmt, ...)                                     \
  do {                                                           \
    if (PRINT_STDOUT_BIT12()) (void)asynPrint(pPrintOutAsynUser, \
                                              ASYN_TRACE_INFO,   \
                                              fmt,               \
                                              ## __VA_ARGS__);   \
  } while (0)

# define LOGINFO13(fmt, ...)                                     \
  do {                                                           \
    if (PRINT_STDOUT_BIT13()) (void)asynPrint(pPrintOutAsynUser, \
                                              ASYN_TRACE_INFO,   \
                                              fmt,               \
                                              ## __VA_ARGS__);   \
  } while (0)

# define LOGINFO14(fmt, ...)                                     \
  do {                                                           \
    if (PRINT_STDOUT_BIT14()) (void)asynPrint(pPrintOutAsynUser, \
                                              ASYN_TRACE_INFO,   \
                                              fmt,               \
                                              ## __VA_ARGS__);   \
  } while (0)

# define LOGINFO15(fmt, ...)                                                    \
  do {                                                                          \
    if (PRINT_STDOUT_BIT15()) {                                                 \
      (void)asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO, fmt, ## __VA_ARGS__); \
    }                                                                           \
  } while (0)

# define LOGERR(fmt, ...)                                                      \
  {                                                                            \
    (void)asynPrint(pPrintOutAsynUser, ASYN_TRACE_ERROR, fmt, ## __VA_ARGS__); \
  }


# define LOGERR_ERRNO(fmt, ...)                                               \
  {                                                                           \
    (void)asynPrint(pPrintOutAsynUser, ASYN_TRACE_INFO, fmt, ## __VA_ARGS__); \
  }

# define ECMC_RETURN_ERROR_STRING "Error: "
# define RETURN_OR_DIE(buffer, fmt, ...)                        \
  do {                                                          \
    cmd_buf_printf(buffer, ECMC_RETURN_ERROR_STRING);           \
    cmd_buf_printf(buffer, fmt, ## __VA_ARGS__);                \
    if (DIE_ON_ERROR_BIT0()) (void)asynPrint(pPrintOutAsynUser, \
                                             ASYN_TRACE_INFO,   \
                                             fmt,               \
                                             ## __VA_ARGS__);   \
    if (DIE_ON_ERROR_BIT0()) (void)asynPrint(pPrintOutAsynUser, \
                                             ASYN_TRACE_INFO,   \
                                             "%s",              \
                                             "\n");             \
    if (DIE_ON_ERROR_BIT1()) exit(2);                           \
    return;                                                     \
  }                                                             \
  while (0)

# define RETURN_ERROR_OR_DIE(buffer, errcode, fmt, ...)         \
  do {                                                          \
    cmd_buf_printf(buffer, "Error: ");                          \
    cmd_buf_printf(buffer, fmt, ## __VA_ARGS__);                \
    if (DIE_ON_ERROR_BIT0()) (void)asynPrint(pPrintOutAsynUser, \
                                             ASYN_TRACE_INFO,   \
                                             fmt,               \
                                             ## __VA_ARGS__);   \
    if (DIE_ON_ERROR_BIT0()) (void)asynPrint(pPrintOutAsynUser, \
                                             ASYN_TRACE_INFO,   \
                                             "%s",              \
                                             "\n");             \
    if (DIE_ON_ERROR_BIT1()) exit(2);                           \
    return errcode;                                             \
  }                                                             \
  while (0)

/*
 * Structure to hold output buffer and associated information
 */
typedef struct {
  int  bufferSize;
  int  bytesUsed;
  char buffer[ECMC_CMD_BUFFER_SIZE];
} ecmcOutputBufferType;

/*
 * Interface from EPICS to the Motion Controller:
 * Send a command in, and get the response text in outbuf
 * The function returns normally 0.
 * If not, something serious went wrong.
 */

extern int  CMDwriteIt(const char *inbuf,
                       size_t      inlen);
extern int  CMDreadIt(char  *outbuf,
                      size_t outlen);
extern void cmd_dump_to_std(const char *buf,
                            unsigned    len);
extern int  clearBuffer(ecmcOutputBufferType *buffer);
extern int  cmd_buf_printf(ecmcOutputBufferType *buffer,
                           const char           *format,
                           ...);
extern int  removeBytesFromBuffer(ecmcOutputBufferType *buffer,
                                  size_t                len);
# ifdef __cplusplus
}
# endif /* ifdef __cplusplus */

#endif  /* ECMC_OCTET_IF_H */
