/*
 * ecmcAsynPortDriverUtils.h

 * Created Feb. 25 Jan, 2019
 */
#ifndef ECMC_ASYN_PORT_DRIVER_UTILS_H_
#define ECMC_ASYN_PORT_DRIVER_UTILS_H_

#include "asynPortDriver.h"  //data types
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "../main/ecmcDefinitions.h"
#include "../main/ecmcErrorsList.h"

#define ECMC_MAX_FIELD_CHAR_LENGTH 128

#define ECMC_OPTION_T_MAX_DLY_MS "T_DLY_MS"
#define ECMC_OPTION_T_SAMPLE_RATE_MS "T_SMP_MS"
#define ECMC_OPTION_TIMEBASE "TIMEBASE"  // PLC or EPICS
#define ECMC_OPTION_TIMEBASE_EPICS "EPICS"
#define ECMC_OPTION_TIMEBASE_ECMC "ECMC"
#define ECMC_OPTION_TYPE "TYPE"
#define ECMC_OPTION_CMD "CMD"

#define ECMC_OPTION_CMD_UINT_TO_FLOAT64 "UINT64TOFLOAT64"
#define ECMC_OPTION_CMD_INT_TO_FLOAT64 "INT64TOFLOAT64"

#define ECMC_ASYN_PAR_OCTET_NAME "ecmc.asynoctet"
#define ECMC_ASYN_INP_FORMAT "@asyn(%[^,],%d,%d)%s"
#define ECMC_ASYN_MASK_INP_FORMAT "@asynMask(%[^,],%d,%x,%d)%s"
typedef enum{
  ECMC_TIME_BASE_ECMC=0,
  ECMC_TIME_BASE_EPICS=1,
  ECMC_TIME_BASE_MAX
} ECMCTIMESOURCE;

typedef enum{
  ECMC_SOURCE_UNDEFINED=0,  
  ECMC_SOURCE_EC=1,
  ECMC_SOURCE_AXIS=2,
  ECMC_SOURCE_MAIN=3,
  ECMC_SOURCE_DS=4,
  ECMC_SOURCE_MAX
} ECMC_SOURCE;

const char *asynTypeToString(long type);
const char *epicsStateToString(int state);
asynParamType stringToAsynType(char *typeStr);
int windowsToEpicsTimeStamp(uint64_t plcTime, epicsTimeStamp *ts);

/*Available strings:
 *  ec<masterId>.s<slaveId>.<alias>  (defaults complete ecentry)
 *  ec<masterId>.s<slaveId>.<alias>.<bit> (only one bit)
*/
int parseEcPath(char *ecPath,
                int  *master,
                int  *slave,
                char *alias,
                int  *bit);

/** \breif Parse main ECMC object type from string.\n
 *
 * \param[in] objPath variable name.\n
 * \param[out] objIndex Object index.\n
 * \param[out] objectType Object type.\n
 *
 * \return 0 if success or otherwise an error code.\n
 */
int getMainObjectType(char             *objPath,
                      int              *objIndex,
                      mainObjectType   *objectType);

/** \breif Parse Axis sub object type from string.\n
 *
 * \param[in] objPath variable name.\n
 * \param[out] objIndex Object index.\n
 * \param[out] objectType Object type.\n
 *
 * \return 0 if success or otherwise an error code.\n
 */
int getAxSubObjectType(char              *objPath,                       
                       axisSubObjectType *objectType);

/** \breif Parse Axis Encoder function type from string.\n
 *
 * \param[in] objPath variable name.\n
 * \param[out] objectFunction Object Function.\n
 * 
 * \return 0 if success or otherwise an error code.\n
 */
int getAxEncFuncType(char *objPath,                              
                     int  *objectFunction);

/** \breif Parse Axis Drive function type from string.\n
 *
 * \param[in] objPath variable name.\n
 * \param[out] objectFunction Object Function.\n
 * 
 * \return 0 if success or otherwise an error code.\n
 */
int getAxDriveFuncType(char *objPath,                              
                       int *objectFunction);

/** \breif Parse Axis Monitor function type from string.\n
 *
 * \param[in] objPath variable name.\n
 * \param[out] objectFunction Object Function.\n
 * 
 * \return 0 if success or otherwise an error code.\n
 */
int getAxMonFuncType(char *objPath,                              
                     int *objectFunction);

/** \breif Parse Axis Main object function type from string.\n
 *
 * \param[in] objPath variable name.\n
 * \param[out] objectFunction Object Function.\n
 * 
 * \return 0 if success or otherwise an error code.\n
 */
int getAxMainFuncType(char *objPath,
                    int *objectFunction);

/** \breif Parse Ec Main object function type from string.\n
 *
 * \param[in] objPath variable name.\n
 * \param[out] objectFunction Object Function.\n
 * 
 * \return 0 if success or otherwise an error code.\n
 */
int getEcMainFuncType(char *objPath,
                      int *objectFunction);

/*typedef enum {
    asynParamNotDefined,
    asynParamInt32,
    asynParamUInt32Digital,
    asynParamFloat64,
    asynParamOctet,
    asynParamInt8Array,
    asynParamInt16Array,
    asynParamInt32Array,
    asynParamFloat32Array,
    asynParamFloat64Array,
    asynParamGenericPointer
} asynParamType;
*/

#endif  /* ECMC_ASYN_PORT_DRIVER_UTILS_H_ */