/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcGeneral.h
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMC_GENERAL_H_
#define ECMC_GENERAL_H_

# ifdef __cplusplus
extern "C" {
# endif  // ifdef __cplusplus

/** \brief Returns the controller error code.\n
 *
 * \return current error code of controller.
 *
 * \note Example: Get current controller error code.\n
 * "GetControllerError()" //Command string to ecmcCmdParser.c\n
 */
int getControllerError();

/** \brief Resets the controller error code.\n
 *
 * \return 0
 *
 * \note Example: Reset controller error.\n
 * "ControllerErrorReset()" //Command string to ecmcCmdParser.c\n
 */
int controllerErrorReset();

/** \brief Get axis error code in string format.\n
 *
 * \param[in] errorNumber Error code.\n
 *
 * \return axis error string.\n
 *
 * \note Example: Get error code of axis 3.\n
 * "Main.M3.sErrorMessage?" //Command string to ecmcCmdParser.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
const char* getErrorString(int errorNumber);

/** \brief Validates the current configuration)"
 *
 * \return 0  if the current configuration is valid for runtime otherwise an
 * error code.\n
 *
 * \note Example: Validate configuration.\n
 * "Cfg.ValidateConfig()" //Command string to ecmcCmdParser.c\n
 */
int validateConfig();

/** \brief Links an EtherCAT entry to an object.
   *
   *
   *  \param[in] ecPath EtherCAT entry path in the following format:\n
   *                    ec<masterId>.s<slaveId>.<alias>.<bitId>
   *                    ec<masterId>.s<slaveId>.<alias>
   *  \param[in] axPath Axis parameter path in the following format:\n
   *                    ax<id>.enc.actpos\n
   *                    ax<id>.drv.enable\n
   *                    ax<id>.drv.velocity\n
   *                    ax<id>.drv.enabled\n
   *                    ax<id>.drv.break\n
   *                    ax<id>.drv.reducetorque\n
   *                    ax<id>.mon.lowlim\n
   *                    ax<id>.mon.highlim\n
   *                    ax<id>.mon.homesensor\n
   *                    ax<id>.mon.extinterlock\n
   *                    ax<id>.health\n
   *                    ec<masterId>.health\n
   *
   *  \return 0 if success or otherwise an error code.\n
   *
   *  \note Example: Link an EtherCAT entry configured as "POSITION_ACT" in slave 1
   *  as actual position for the encoder of axis 5.\n
   *  "Cfg.LinkEcEntryToObject(ec0.s1.POSITION_ACT,ax5.enc.actpos)" //Command string
   *  to ecmcCmdParser.c\n
   *     *
   */
int linkEcEntryToObject(char *ecPath,
                        char *objPath);

/** \brief Enable printouts of timing information related to the realtime
 * thread.\n
 *
 * \param[in] enable Enable printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable printout time diagnostics.\n
 *  "Cfg.SetEnableTimeDiag(1)" //Command string to ecmcCmdParser.c\n
 */
int setEnableTimeDiag(int value);

/** \brief Enable printouts of which functions in hw_motor.cpp are being
 * called.\n
 *
 * \param[in] enable Enable printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable function call diagnostics.\n
 *  "Cfg.SetEnableFuncCallDiag(1)" //Command string to ecmcCmdParser.c\n
 */
int setEnableFunctionCallDiag(int value);
                       
# ifdef __cplusplus
}
# endif  // ifdef __cplusplus

#endif  /* ECMC_GENERAL_H_ */