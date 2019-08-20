#include <string.h>
#include <stdio.h>
#include <ctype.h>  /* isprint() */
#include <stdlib.h>
#include <stdarg.h>

#include "ecmcOctetIF.h"
#include "ecmcCmdParser.h"
#include "../main/ecmcErrorsList.h"

unsigned int debug_print_flags      = 0; // 65535;
unsigned int die_on_error_flags     = 1;
unsigned int argv0_semicolon_is_sep = 0;

static ecmcOutputBufferType outputBuffer = { ECMC_CMD_BUFFER_SIZE, 0 };

static int addToBuffer(ecmcOutputBufferType *buffer,
                       const char           *addText,
                       size_t                addLength);

/*****************************************************************************/

static int cmd_buf_vprintf(ecmcOutputBufferType *buffer,
                           const char           *format,
                           va_list               arg) {
  const static size_t len = 4096;

  char *buf = calloc(len, 1);
  int   res = vsnprintf(buf, len - 1, format, arg);

  if (res >= 0) {
    addToBuffer(buffer, buf, res);
  }
  free(buf);
  buf = NULL; 
  return res;
}

/*****************************************************************************/

int cmd_buf_printf(ecmcOutputBufferType *buffer, const char *format, ...) {
  if (buffer == NULL) {
    return ERROR_MAIN_PARSER_BUFFER_NULL;
  }
  va_list ap;
  va_start(ap, format);
  (void)cmd_buf_vprintf(buffer, format, ap);
  va_end(ap);
  return 0;
}

/*****************************************************************************/

static int addToBuffer(ecmcOutputBufferType *buffer,
                       const char           *addText,
                       size_t                addLength) {
  if (buffer == NULL) {
    return __LINE__;
  }

  if (addLength >= buffer->bufferSize - buffer->bytesUsed - 1) {
    return __LINE__;
  }

  memcpy(&buffer->buffer[buffer->bytesUsed], addText, addLength);
  buffer->bytesUsed                += addLength;
  buffer->buffer[buffer->bytesUsed] = '\0';
  return 0;
}

/*****************************************************************************/

static int removeFromBuffer(ecmcOutputBufferType *buffer, size_t len) {
  if (buffer == NULL) {
    return __LINE__;
  }

  int bytesToMove = buffer->bytesUsed - len;

  if (bytesToMove < 0) {
    return __LINE__;
  }

  memmove(&buffer->buffer[0], &buffer->buffer[len], bytesToMove);
  buffer->bytesUsed                 = bytesToMove;
  buffer->buffer[buffer->bytesUsed] = '\0';
  return 0;
}

/*****************************************************************************/

int clearBuffer(ecmcOutputBufferType *buffer) {
  if (buffer == NULL) {
    return __LINE__;
  }
  buffer->bytesUsed = 0;
  buffer->buffer[0] = '\0';
  return 0;
}

static ecmcOutputBufferType* getEpicsBuffer() {
  return &outputBuffer;
}

/*****************************************************************************/

void cmd_dump_to_std(const char *buf, unsigned len) {
  unsigned int i;

  for (i = 0; i < len; i++) {
    int ch = buf[i];

    switch (ch) {
    case '\t':
      fprintf(stdout, "\\t");
      break;

    case '\r':
      fprintf(stdout, "\\r");
      break;

    case '\n':
      fprintf(stdout, "\\n");
      break;

    default:

      if (isprint(ch)) fprintf(stdout, "%c", ch);
      else fprintf(stdout, "\\%03o", ch);
    }
  }
  fflush(stdout);
}

/*****************************************************************************/

static int create_argv_sepv(const char *line,
                            const char ***argv_p, char ***sepv_p) {
  char  *input_line                  = strdup(line);
  size_t calloc_len                  = 2 + strlen(input_line);
  char  *separator                   = NULL;
  static const size_t MAX_SEPARATORS = 4;
  int argc                           = 0;

  /* Allocate an array big enough, could be max strlen/2
     space <-> non-space transitions */
  const char **argv;  /* May be more */
  char **sepv;

  argv    = (const char **)(void *)calloc(calloc_len, sizeof(char *));
  *argv_p = argv;

  if (argv  == NULL) {
    return 0;
  }
  sepv    = (char **)(void *)calloc(calloc_len, sizeof(char *));
  *sepv_p = sepv;

  if (sepv  == NULL) {
    return 0;
  }

  /* argv[0] is the whole line */
  {
    size_t line_len = strlen(input_line);
    argv[argc] = strdup(input_line);
    sepv[argc] = calloc(1, MAX_SEPARATORS);

    if (argv0_semicolon_is_sep &&
        (line_len > 1) && (input_line[line_len - 1] == ';')) {
      /* Special: the last character is ; move it from
         input line into the separator */
      char *sep = (char *)sepv[argc];
      sep[0] = ';';
      sep    = (char *)&argv[argc][line_len - 1];
      sep[0] = '\0';
    }
  }

  if (!strlen(input_line)) {
    return argc;
  }

  if (strchr(input_line, ';') != NULL) {
    separator = ";";
  }  // else if (strchr(input_line, ' ') != NULL) {

  // separator = " ";
  // }
  if (separator) {
    argc++;

    /* Start the loop */
    char *arg_begin = input_line;
    char *next_sep  = strchr(input_line, separator[0]);
    char *arg_end   = next_sep ? next_sep : input_line + strlen(input_line);

    while (arg_begin) {
      size_t sepi    = 0;
      char  *sep     = NULL;
      size_t arg_len = arg_end - arg_begin;

      argv[argc] = calloc(1, arg_len + 1);
      memcpy((char *)argv[argc], arg_begin, arg_len);
      sepv[argc] = calloc(1, MAX_SEPARATORS);
      sep        = sepv[argc];

      if (next_sep) {
        /* There is another separator */
        sep[sepi++] = separator[0];
      }
      arg_begin = arg_end;

      if (arg_begin[0] == separator[0]) {
        arg_begin++;  /* Jump over, if any */
      }
      next_sep = strchr(arg_begin, separator[0]);
      arg_end  = next_sep ? next_sep : input_line + strlen(input_line);

      if (!strlen(arg_begin)) {
        break;
      } else {
        argc++;
      }
    }
  } else {
    /* argv[1] is the whole line */
    argc       = 1;
    argv[argc] = strdup(input_line);
    sepv[argc] = calloc(1, MAX_SEPARATORS);
  }

  free(input_line);

  if (PRINT_STDOUT_BIT2()) {
    int i;

    /****  Print what we have */
    fprintf(stdout, "%s/%s:%d argc=%d calloc_len=%u\n",
            __FILE__, __FUNCTION__, __LINE__,
            argc, (unsigned)calloc_len);

    for (i = 0; i <= argc; i++) {
      fprintf(stdout, "%s/%s:%d argv[%d]=\"%s\" sepv[%d]=\"%s\"\n",
              __FILE__, __FUNCTION__, __LINE__,
              i, argv[i] ? argv[i] : "NULL",
              i, sepv[i] ? sepv[i] : "NULL");
    }
  }

  return argc;
}

/*****************************************************************************/
int cmd_handle_input_line(const char           *input_line,
                          ecmcOutputBufferType *buffer) {
  static unsigned int counter;
  const char **my_argv = NULL;
  char **my_sepv       = NULL;
  int    argc          = create_argv_sepv(input_line,
                                          (const char ***)&my_argv,
                                          (char ***)&my_sepv);
  const char *argv1 = (argc > 1) ? my_argv[1] : "";
  int is_EAT_cmd    = 1; // strchr(input_line, ';') != NULL;

  if (is_EAT_cmd) {
    int errorCode = ecmcCmdParser(argc, my_argv, (const char **)my_sepv, buffer);

    if (errorCode) {
      RETURN_ERROR_OR_DIE(buffer,
                          __LINE__,
                          "%s/%s:%d ecmcCmdParser returned error: %x.",
                          __FILE__,
                          __FUNCTION__,
                          __LINE__,
                          errorCode);
    }
  } else if ((argc > 1) && (0 == strcmp(argv1, "bye"))) {
    fprintf(stdout, "%s/%s:%d bye\n", __FILE__, __FUNCTION__, __LINE__);
    return 1;
  } else if ((argc > 1) && (0 == strcmp(argv1, "kill"))) {
    exit(0);
  } else if ((argv1[0] == 'h') ||
             (argv1[0] == '?')) {
    cmd_buf_printf(buffer, "%s",
                   "Valid commands :\n"
                   "bye            : Bye\nkill           : exit(0)\n");
  } else if (argc > 2) {
    cmd_buf_printf(buffer, "error(%s:%d): invalid command (%s)\n",
                   __FILE__, __LINE__,  argv1);
  } else if (argc == 1) {
    /* Just a return, print a prompt */
  }
  {
    int i;

    for (i = 0; i <= argc; i++)
    {
      free((void *)my_argv[i]);
      free((void *)my_sepv[i]);
    }
    free(my_argv);
    free(my_sepv);
  }

  if (PRINT_STDOUT_BIT2()) {
    fprintf(stdout, "%s/%s:%d (%u)\n",
            __FILE__, __FUNCTION__, __LINE__,
            counter++);
  }
  return 0;
}

/* from EPICS into MCU */
int CMDwriteIt(const char *inbuf, size_t inlen) {
  int   had_cr  = 0;
  int   had_lf  = 0;
  char *new_buf = (char *)inbuf;

  if (!inbuf || !inlen) return -1;

  if (PRINT_STDOUT_BIT1() && stdout) {
    fprintf(stdout, "%s/%s:%d IN=\"", __FILE__, __FUNCTION__, __LINE__);
    cmd_dump_to_std(inbuf, inlen);
    fprintf(stdout, "\"\n");
  }

  new_buf = malloc(inlen + 1);
  memcpy(new_buf, inbuf, inlen);
  new_buf[inlen] = 0;

  // printf("************NEW COMMAND IN WRITE: %s",new_buf);

  if ((inlen > 1) && (new_buf[inlen - 1] == '\n')) {
    had_lf             = 1;
    new_buf[inlen - 1] = '\0';
    inlen--;

    if ((inlen > 1) && (new_buf[inlen - 1] == '\r')) {
      had_cr             = 1;
      new_buf[inlen - 1] = '\0';
      inlen--;
    }
  }

  // Add clear buffer here?
  int errorCode = cmd_handle_input_line(new_buf, getEpicsBuffer());

  if (errorCode) {
    RETURN_ERROR_OR_DIE(getEpicsBuffer(),
                        __LINE__,
                        "%s/%s:%d cmd_handle_input_line returned error: %x.",
                        __FILE__,
                        __FUNCTION__,
                        __LINE__,
                        errorCode);
  }

  free(new_buf);

  errorCode = cmd_buf_printf(getEpicsBuffer(),
                             "%s%s",
                             had_cr ? "\r" : "",
                             had_lf ? "\n" : "");

  if (errorCode) {
    RETURN_ERROR_OR_DIE(getEpicsBuffer(),
                        __LINE__,
                        "%s/%s:%d cmd_buf_printf returned error: %x.",
                        __FILE__,
                        __FUNCTION__,
                        __LINE__,
                        errorCode);
  }

  return 0;
}

/* from MCU into EPICS */
int CMDreadIt(char *outbuf, size_t outlen) {  
  int ret;

  if (!outbuf || !outlen) return -1;

  ret = snprintf(outbuf, outlen + 1, "%s", getEpicsBuffer()->buffer);

  if (ret < 0) {
    // printf("RET <0");
    clearBuffer(getEpicsBuffer());
    return ret;
  }

  if (PRINT_STDOUT_BIT1() && stdout) {
    fprintf(stdout, "%s/%s:%d OUT=\"", __FILE__, __FUNCTION__, __LINE__);
    cmd_dump_to_std(outbuf, strlen(outbuf));
    fprintf(stdout, "\"\n");
  }

  if (ret >= outlen + 1) {
    ret = outlen;  // snprintf max utilize buffer size minus one.
  }
  removeFromBuffer(getEpicsBuffer(), ret);

  if (PRINT_STDOUT_BIT1() && stdout) {
    fprintf(stdout, "%s/%s:%d OUT2=\"", __FILE__, __FUNCTION__, __LINE__);
    cmd_dump_to_std(outbuf, strlen(outbuf));
    fprintf(stdout, "\"\n");
  }

  return 0;
}

/*
 *
 command interface, argv argv
 inbuf                            argc  argv
 ""                                  0  argv[0] = "" // Special case; Does not happen
 "Main.M1.nMotionAxisID"             1  argv[0] = "Main.M1.nMotionAxisID?"
                                        sepv[0] = ""
                                        argv[1] = "Main.M1.nMotionAxisID?"
                                        sepv[1] = ""

 "Main.M1.nMotionAxisID?"            1  argv[0] = "Main.M1.nMotionAxisID?"
                                        sepv[0] = ""argv[1] = "Main.M1.nMotionAxisID?"
                                        sepv[1] = ""
 "Main.M1.nMotionAxisID?;"           1  argv[0] = "Main.M1.nMotionAxisID?"
                                        sepv[0] = ""
                                        argv[1] = "Main.M1.nMotionAxisID?"
                                        sepv[1] = ";"

 "X1;x2;x3;x4"                       4  argv[0] = "X1;x2;x3;x4"
                                        sepv[0] = ""
                                        argv[1] = "X2"
                                        sepv[1] = ";"
                                        argv[2] = "X3"
                                        sepv[2] = ";"
                                        argv[3] = "X4"
                                        sepv[3] = ""

 */
