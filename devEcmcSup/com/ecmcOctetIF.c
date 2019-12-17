/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcOctetIF.c
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom, torstenbögershausen
*
\*************************************************************************/

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

static ecmcOutputBufferType outputBuffer = { .bufferSize = ECMC_CMD_BUFFER_SIZE, .bytesUsed = 0};
static int initDone = 0;

static char inputBuffer[ECMC_CMD_BUFFER_SIZE] = {0};

/*****************************************************************************/

int cmd_buf_printf(ecmcOutputBufferType *buffer, const char *format, ...) {
  if (buffer == NULL) {
    return ERROR_MAIN_PARSER_BUFFER_NULL;
  }
  va_list ap;
  va_start(ap, format);
  char *buff = &buffer->buffer[buffer->bytesUsed];    
  int   res = vsnprintf(buff, buffer->bufferSize-buffer->bytesUsed, format, ap);
  buffer->bytesUsed                += res;
  va_end(ap);
  if (res < 0) {
    return res;
  }
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
  buffer->bufferSize = ECMC_CMD_BUFFER_SIZE;
  return 0;
}

/*****************************************************************************/

static ecmcOutputBufferType* getEpicsBuffer() {
  return &outputBuffer;
}

/*****************************************************************************/

void init() {
  clearBuffer(getEpicsBuffer());
  initDone = 1;
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

/* from EPICS into MCU */
int CMDwriteIt(const char *inbuf, size_t inlen) {
  int   had_cr  = 0;
  int   had_lf  = 0;

  if(!initDone) {
    init();
  }

  if (!inbuf || !inlen) return -1;  

  if (PRINT_STDOUT_BIT1() && stdout) {
    fprintf(stdout, "%s/%s:%d IN=\"", __FILE__, __FUNCTION__, __LINE__);
    cmd_dump_to_std(inbuf, inlen);
    fprintf(stdout, "\"\n");
  }

  if (inlen>=ECMC_CMD_BUFFER_SIZE) {
    LOGERR(
      "%s/%s:%d: ERROR: asynOctet command to long and will be discarded.\n",
      __FILE__,
      __FUNCTION__,
      __LINE__      
    );    
    return -1;    
  }

  int copyLength = inlen+1;

  // Work with local input buffer
  memcpy(inputBuffer, inbuf, copyLength);
  
  // Check LF and or CR
  if ((inlen > 1) && (inputBuffer[inlen - 1] == '\n')) {
    had_lf             = 1;
    inputBuffer[inlen - 1] = '\0';
    inlen--;

    if ((inlen > 1) && (inputBuffer[inlen - 1] == '\r')) {
      had_cr             = 1;
      inputBuffer[inlen - 1] = '\0';
      inlen--;
    }
  }
  
  int errorCode = ecmcCmdParser(inputBuffer, inlen, getEpicsBuffer());

  if (errorCode) {
    RETURN_ERROR_OR_DIE(getEpicsBuffer(),
                        __LINE__,
                        "%s/%s:%d ecmcCmdParser returned error: %x.",
                        __FILE__,
                        __FUNCTION__,
                        __LINE__,
                        errorCode);
  }

  // clear buffer 
  memset(inputBuffer,0,copyLength);

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
