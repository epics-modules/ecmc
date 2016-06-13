#ifndef CMD_EAT_H
#define CMD_EAT_H

#include "cmd.h"

#ifdef __cplusplus
extern "C" {
#endif

  int cmd_EAT(int argc, const char *argv[], const char *seperator[],ecmcOutputBufferType *buffer);
  int motorHandleOneArg(const char *myarg_1,ecmcOutputBufferType *buffer);

#ifdef __cplusplus
}
#endif

#endif /* CMD_EAT_H */
