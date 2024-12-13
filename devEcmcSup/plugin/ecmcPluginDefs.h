/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcPluginDefs.h
*
*  Created on: Mar 21, 2020
*      Author: anderssandstrom
*
\*************************************************************************/
#ifndef ECMC_PLUGIN_DEFS_H_
#define ECMC_PLUGIN_DEFS_H_

#define ECMC_PLUGIN_MAX_PLC_FUNC_COUNT 64
#define ECMC_PLUGIN_MAX_PLC_CONST_COUNT 64
#define ECMC_PLUGIN_MAX_PLC_ARG_COUNT 10
#define ECMC_PLUG_VER_MAJOR 1
#define ECMC_PLUG_VER_MINOR 0
#define ECMC_PLUG_VER_PATCH 0
#define ECMC_PLUG_VERSION(a, b, c) (((a) << 16) + ((b) << 8) + (c))
#define ECMC_PLUG_VERSION_MAGIC ECMC_PLUG_VERSION(ECMC_PLUG_VER_MAJOR,\
                                                  ECMC_PLUG_VER_MINOR,\
                                                  ECMC_PLUG_VER_PATCH)

// Structure for defining one custom plc function
struct ecmcOnePlcFunc {
  // Function name (this is the name you use in ecmc plc-code)
  const char *funcName;

  // Function description
  const char *funcDesc;

  /**
   * 11 different prototypes allowed (only doubles since reg in plc).
   * Only one funcArg<argCount> func shall be assigned the rest set to NULL.
   **/
  double (*funcArg0)();
  double (*funcArg1)(double);
  double (*funcArg2)(double,
                     double);
  double (*funcArg3)(double,
                     double,
                     double);
  double (*funcArg4)(double,
                     double,
                     double,
                     double);
  double (*funcArg5)(double,
                     double,
                     double,
                     double,
                     double);
  double (*funcArg6)(double,
                     double,
                     double,
                     double,
                     double,
                     double);
  double (*funcArg7)(double,
                     double,
                     double,
                     double,
                     double,
                     double,
                     double);
  double (*funcArg8)(double,
                     double,
                     double,
                     double,
                     double,
                     double,
                     double,
                     double);
  double (*funcArg9)(double,
                     double,
                     double,
                     double,
                     double,
                     double,
                     double,
                     double,
                     double);
  double (*funcArg10)(double,
                      double,
                      double,
                      double,
                      double,
                      double,
                      double,
                      double,
                      double,
                      double);
  void *funcGenericObj;  // generic_function_t  generic Func object (allow strings)
};

// Structure for defining one custom plc constant
struct ecmcOnePlcConst {
  const char *constName;
  const char *constDesc;
  double      constValue;
};

struct ecmcPluginData {
  // ECMC_PLUG_VERSION_MAGIC
  int ifVersion;

  // Name
  const char *name;

  // Description
  const char *desc;

  // Option description
  const char *optionDesc;

  // Plugin version
  int version;

  // Optional construct func, called once at load (with config string (with options if needed))
  int (*constructFnc)(char *config);

  // Optional destruct func, called once at unload
  void (*destructFnc)(void);

  // Optional func that will be called once just before enter realtime mode
  int (*realtimeEnterFnc)(void);

  // Optional func that will be called once just before exit realtime mode
  int (*realtimeExitFnc)(void);

  // Optional func that will be called each realtime cycle
  int (*realtimeFnc)(int);

  // Allow max ECMC_PLUGIN_MAX_PLC_FUNC_COUNT custom functions
  struct ecmcOnePlcFunc funcs[ECMC_PLUGIN_MAX_PLC_FUNC_COUNT];

  // Allow max ECMC_PLUGIN_MAX_PLC_CONST_COUNT custom constants
  struct ecmcOnePlcConst consts[ECMC_PLUGIN_MAX_PLC_CONST_COUNT];
};

#ifndef ECMC_PLUGIN_MODULE_NAME

#define ECMC_PLUGIN_MODULE_NAME dummy

#endif
// Force exapansion
#define CONCAT(a,b) a##b
#define EXPAND_AND_CONCAT(a,b) CONCAT(a,b)

#define ecmc_plugin_register(pluginData)\
        struct ecmcPluginData *EXPAND_AND_CONCAT(_plugin_get_data_lib, ECMC_PLUGIN_MODULE_NAME)(void) {\
          return &pluginData;\
        }\

#endif  /* ECMC_PLUGIN_DEFS_H_ */
