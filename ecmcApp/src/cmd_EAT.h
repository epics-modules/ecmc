#ifndef ECMC_CMD_EAT_H
# define ECMC_CMD_EAT_H

# include "cmd.h"

# ifdef __cplusplus
extern "C" {
# endif /* ifdef __cplusplus */

int cmd_EAT(int                   argc,
            const char           *argv[],
            const char           *seperator[],
            ecmcOutputBufferType *buffer);
int motorHandleOneArg(const char           *myarg_1,
                      ecmcOutputBufferType *buffer);

# ifdef __cplusplus
}
# endif /* ifdef __cplusplus */

#endif  /* ECMC_CMD_EAT_H */
