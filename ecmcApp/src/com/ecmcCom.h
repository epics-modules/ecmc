#ifndef ECMC_MISC_H_
#define ECMC_MISC_H_

#include "../com/ecmcOctetIF.h"        // Log Macros
#include "../main/ecmcErrorsList.h"
#include "../main/ecmcDefinitions.h"

# ifdef __cplusplus
extern "C" {
# endif  // ifdef __cplusplus



/** \breif Link EtherCAT entry to ASYN parameter.
 *
 * Fast access of EtherCAT data from EPICS records is possible by linking an
 * EtherCAT entry to an ASYN parameter. Update frequency of the asyn parameter
 * can be changed with the "skipCycles" parameter. Maximum update frequency is
 * the same frequency as the EtherCAT realtime bus.\n
 * This function is called by the iocsh command
 * "ecmcAsynPortDriverAddParameter()". For more information see documentation
 * of ecmcAsynPortDriverAddParameter().\n
 *
 *  \param[in] masterIndex Index of EtherCAT master.\n
 *  \param[in] busPosition Bus position of EtherCAT slave.\n
 *  \param[in] entryIdString String for addressing ethercat entry:\n
 *             ec.s<slave number>.<ethercat entry id>
 *  \param[in] asynParType Data type to be transfered.\n*
 *             asynParType=1: asynInt32
 *             asynParType=3: asynFloat64
 *  \param[in] skipCycles Number of realtime loops in between updates of asyn-
 *  parameter.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: entryIdString for an EtherCAT entry called "INPUT_1"
 * on slave 10: "ec.s10.INPUT_1".\n
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
int linkEcEntryToAsynParameter(int         masterIndex,
                               int         busPosition,
                               const char *entryIDString,
                               int         asynParType,
                               int         skipCycles);

/** \breif Link EtherCAT memory map to ASYN parameter.
 *
 * Fast access of EtherCAT data from EPICS records is possible by linking an
 * EtherCAT memory map to an ASYN parameter. Update frequency of the asyn parameter
 * can be changed with the "skipCycles" parameter. Maximum update frequency is
 * the same frequency as the EtherCAT realtime bus.\n
 * This function is called by the iocsh command
 * "ecmcAsynPortDriverAddParameter()". For more information see documentation
 * of ecmcAsynPortDriverAddParameter(), ecAddMemMap(), readEcMemMap() and
 * ecSetEntryUpdateInRealtime().\n
 *
 *  \param[in] masterIndex Index of EtherCAT master.\n
 *  \param[in] memMapIDString String for addressing ethercat entry:\n
 *             ec.mm.<memory map id>
 *  \param[in] asynParType Data type to be transfered.\n*
 *             asynParType=5: asynParamInt8Array
 *             asynParType=6: asynParamInt16Array
 *             asynParType=7: asynParamInt32Array
 *             asynParType=8: asynParamFloat32Array
 *             asynParType=9: asynParamFloat64Array
 *  \param[in] skipCycles Number of realtime loops in between updates of asyn-
 *  parameter.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: memMapIDString for an memory map called "AI_1_ARRAY":
 * "ec.mm.AI_1_ARRAY".\n
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
int linkEcMemMapToAsynParameter(int         masterIndex,
                                const char *memMapIDString,
                                int         asynParType,
                                int         skipCycles);

/** \breif Initilize asyn for ecmc
 *
 *  \param[in] asynPortObject Asyn port object.\n
 * \return 0 if success or otherwise an error code.\n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
int initEcmcAsyn(void *asynPortObject);

/** \breif Add default asyn parameters for ecmc
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
int ecmcAddDefaultAsynParams();

/** \breif Add default asyn parameters for axis
 *
 *  \param[in] regAsynParams Register default asyn parameters.\n
 *  \param[in] axisIndex Index of axis.\n
 *  \param[in] skipCycles Number of cycles to postpone update.\n
 * \return 0 if success or otherwise an error code.\n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
int addDefaultAsynAxis(int regAsynParams,
                       int axisIndex,
                       int skipCycles);

/** \breif Add diagnostic string for axis as asyn parameter.\n
 *
 *  \param[in] regAsynParams Register default asyn parameters.\n
 *  \param[in] axisIndex Index of axis.\n
 *  \param[in] skipCycles Number of cycles to postpone update.\n
 * \return 0 if success or otherwise an error code.\n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
int addDiagAsynAxis(int regAsynParams,
                    int axisIndex,
                    int skipCycles);

/** \breif Add default asyn parameters for EtherCAT master
 *
 *  \param[in] masterIndex Index of EtherCAT master.\n
 *  \param[in] regAsynParams Register default asyn parameters.\n
 *  \param[in] skipCycles Number of cycles to postpone update.\n
 * \return 0 if success or otherwise an error code.\n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
int addDefaultAsynEc(int masterIndex,
                     int regAsynParams,
                     int skipCycles);

/** \breif Add default asyn parameters for EtherCAT slave
 *
 *  \param[in] masterIndex Index of EtherCAT master.\n
 *  \param[in] busPosition Bus position of EtherCAT slave.\n
 *  \param[in] regAsynParams Register default asyn parameters.\n
 *  \param[in] skipCycles Number of cycles to postpone update.\n
 * \return 0 if success or otherwise an error code.\n
 *
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
int addDefaultAsynEcSlave(int masterIndex,
                          int busPosition,
                          int regAsynParams,
                          int skipCycles);

# ifdef __cplusplus
}
# endif  // ifdef __cplusplus

#endif  /* ECMC_MISC_H_ */