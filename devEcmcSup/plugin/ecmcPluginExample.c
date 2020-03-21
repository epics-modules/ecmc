/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcPluginExample.cpp
*
*  Created on: Mar 21, 2020
*      Author: anderssandstrom
*
\*************************************************************************/

#include <stdio.h>
#include "ecmcPluginDefs.h"

/** Optional. 
 *  Will be called once at load.
 *  Return value other than 0 will be considered error.
 *  ecmcRefs will be used to pass ecmc objects to lib
 **/
int exampleConstruct(void* ecmcRefs)
{
  printf("exampleConstruct...\n");
  return 0;
}

/** Optional. 
 *  Will be called once at unload.
 **/
void exampleDestruct(void)
{
  printf("exampleDestruct...\n");
}

/** Optional. 
 *  Will be called each realtime cycle if definded
 *  Return value other than 0 will be considered error.
 **/
int exampleRealtime(void)
{
  printf("exampleRealtime...\n");
  return 0;
}

double customPlcFunc1(double arg1, double arg2)
{
  printf("customPlcFunc1 %lf, %lf.\n",arg1,arg2);
  return arg1 * arg2;
}

double customPlcFunc2(double arg1, double arg2, double arg3)
{
  printf("customPlcFunc2 %lf, %lf, %lf.\n",arg1,arg2,arg3);
  return arg1 * arg2 * arg3;
}

// Compile data for lib
struct ecmcPluginData pluginDataDef = {
  // ECMC_PLUG_VERSION_MAGIC
  .ifVer = ECMC_PLUG_VERSION_MAGIC,
  // Lib name 
  .libName = "ecmcExamplePlugin",
  // Optional construct func, called once at load. NULL if not definded.
  .constructFnc = exampleConstruct,
  // Optional destruct func, called once at unload. NULL if not definded.
  .destructFnc = exampleDestruct,
  // Optional func that will be called each rt cycle. NULL if not definded.
  .realtimeFnc = exampleRealtime,
  // Allow max ECMC_PLUGIN_MAX_FUNC_COUNT custom funcs
  .funcs =
   {  
       /* Note max ECMC_PLUGIN_MAX_PLC_FUNC_COUNT funcs allowed*/
      
      /*----customPlcFunc1----*/
      { 
        // Function name
        .funcName = "ex_customPlcFunc1",
        // Number of arguments in the function prototytpe
        .argCount = 2,
        /**
        * 7 different prototypes allowed (only doubles since reg in plc).
        * Only funcArg${argCount} func shall be assigned the rest set to NULL.
        * funcArg${argCount}. These need to match. 
        **/
        .funcArg0 = NULL,
        .funcArg1 = NULL,
        .funcArg2 = customPlcFunc1, // Func has 2 args
        .funcArg3 = NULL,
        .funcArg4 = NULL,
        .funcArg6 = NULL,
        .funcArg6 = NULL
      },

      /*----customPlcFunc2----*/
      {
        // Function name
        .funcName = "ex_customPlcFunc2",
        // Number of arguments in the function prototytpe
        .argCount = 3,
        /**
        * 7 different prototypes allowed (only doubles since reg in plc).
        * Only funcArg${argCount} func shall be assigned the rest set to NULL.
        * funcArg${argCount}. These need to match. 
        **/
        .funcArg0 = NULL,
        .funcArg1 = NULL,
        .funcArg2 = NULL,
        .funcArg3 = customPlcFunc2, // Func has 3 args
        .funcArg4 = NULL,
        .funcArg6 = NULL,
        .funcArg6 = NULL
      },
      {0} //last element set all to zero..
   }
};

// Register plugin data
ecmc_plugin_register(pluginDataDef);
