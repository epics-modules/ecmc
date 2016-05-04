#ifndef EEMC_CMD_H
#define EEMC_CMD_H

#ifdef __cplusplus
extern "C" {
#endif



  /*
   * logging stuff
   */

  extern unsigned int debug_print_flags;
  extern unsigned int die_on_error_flags;
#if 0
  extern FILE *stdlog;
#else
#define stdlog (stderr)
#endif


#define PRINT_OUT    (1<<1)

#define PRINT_STDOUT_BIT1() (debug_print_flags & (1<<1))
#define PRINT_STDOUT_BIT2() (debug_print_flags & (1<<2))

#define PRINT_STDOUT_BIT4() (debug_print_flags & (1<<4))
#define PRINT_STDOUT_BIT5() (debug_print_flags & (1<<5))
#define PRINT_STDOUT_BIT6() (debug_print_flags & (1<<6))
#define PRINT_STDOUT_BIT7() (debug_print_flags & (1<<7))

#define DIE_ON_ERROR_BIT0() (die_on_error_flags & 1)
#define DIE_ON_ERROR_BIT1() (die_on_error_flags & (1<<1))




#define LOGINFO(fmt, ...)                        \
{                                                \
  (void)fprintf(stdlog, fmt, ##__VA_ARGS__);     \
}

#define LOGINFO4(fmt, ...)                       \
do {                                             \
  if (PRINT_STDOUT_BIT4()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__);   \
} while (0)

#define LOGINFO5(fmt, ...)                       \
do {                                             \
  if (PRINT_STDOUT_BIT5()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__);   \
} while (0)

#define LOGINFO6(fmt, ...)                       \
do {                                             \
  if (PRINT_STDOUT_BIT6()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__);   \
} while (0)

#define LOGINFO7(fmt, ...)                       \
do {                                             \
  if (PRINT_STDOUT_BIT7()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__);   \
} while (0)


#define LOGERR(fmt, ...)                         \
{                                                \
  (void)fprintf(stdlog, fmt, ##__VA_ARGS__);     \
}


#define LOGERR_ERRNO(fmt, ...)                   \
{                                                \
  (void)fprintf(stdlog, "%s/%s:%d errno=%d (%s) ", __FILE__,__FUNCTION__, __LINE__, errno, strerror(errno)); \
  (void)fprintf(stdlog, fmt, ##__VA_ARGS__);     \
}

#define RETURN_OR_DIE(fmt, ...)                 \
  do {                                          \
    cmd_buf_printf("Error: ");                  \
    cmd_buf_printf(fmt, ##__VA_ARGS__);         \
    if (DIE_ON_ERROR_BIT0()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__);   \
    if (DIE_ON_ERROR_BIT0()) (void)fprintf(stdlog, "%s", "\n"); \
    if (DIE_ON_ERROR_BIT1())  exit(2);          \
    return;                                     \
  }                                             \
  while(0)

#define RETURN_ERROR_OR_DIE(errcode,fmt, ...)   \
  do {                                          \
    cmd_buf_printf("Error: ");                  \
    cmd_buf_printf(fmt, ##__VA_ARGS__);         \
    if (DIE_ON_ERROR_BIT0()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__);   \
    if (DIE_ON_ERROR_BIT0()) (void)fprintf(stdlog, "%s", "\n"); \
    if (DIE_ON_ERROR_BIT1())  exit(2);          \
    return errcode;                             \
  }                                             \
  while(0)

  /*
   * Interface from EPICS to the Motion Controller:
   * Send a command in, and get the response text in outbuf
   * The function returns normally 0.
   * If not, something serious went wrong.
   */
  extern int CMDwriteIt(const char *inbuf, size_t inlen);
  extern int CMDreadIt(char *outbuf, size_t outlen);

  __attribute__((format (printf,1,2)))
  extern void cmd_buf_printf(const char *fmt, ...);
  extern void cmd_dump_to_std(const char *buf, unsigned len);
  /* "RAW" Functions to handle the input line, and later to
     get the response buffer,
     used with socket interface */
  extern int cmd_handle_input_line(const char *input_line);
  extern char *get_buf(void);
  extern void clear_buf();
#ifdef __cplusplus
}
#endif

#endif /* EEMC_CMD_H */

