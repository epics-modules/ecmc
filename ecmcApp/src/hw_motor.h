
#ifndef MOTOR_H
#define MOTOR_H

/**\file
 * \defgroup ecmc
 * Main interface for ECMC motion control.
 * \author Anders Sandström
 * \contact anders.sandstrom@esss.se
 */
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <string.h>

#include "ecmcDefinitions.h"

#ifdef __cplusplus
extern "C" {
#endif

///Checks if an axis index is valid
#define AXIS_CHECK_RETURN(_axis) {init_axis(_axis); if (((_axis) <= 0) || ((_axis) >=ECMC_MAX_AXES)) return;}
#define AXIS_CHECK_RETURN_ZERO(_axis) {init_axis(_axis); if (((_axis) <= 0) || ((_axis) >=ECMC_MAX_AXES)) return 0;}
#define AXIS_CHECK_RETURN_USED_BUFFER(_axis) {init_axis(_axis); if (((_axis) <= 0) || ((_axis) >=ECMC_MAX_AXES)) return 0;}

//Error Codes
#define CMD_EAT_READ_STORAGE_BUFFER_DATA_NULL 0x200000


/** \breif Initialization routine for ecmc.\n
 */
int hw_motor_global_init(void);

/** \breif Returns the controller error code.\n
 *
 * \return current error code of controller.
 *
 * \note Example: Get current controller error code.\n
 * "GetControllerError()" //Command string to cmd_EAT.c\n
 */
int getControllerError();

/** \breif Resets the controller error code.\n
 *
 * \return 0
 *
 * \note Example: Reset controller error.\n
 * "ControllerErrorReset()" //Command string to cmd_EAT.c\n
 */
int controllerErrorReset();

/** \breif Move axis to an absolute position.\n
 *
 * \param[in] axisIndex Axis index.\n
 * \param[in] positionSet Position setpoint.\n
 * \param[in] velocitySet Velocity setpoint.\n
 * \param[in] accelerationSet Acceleration setpoint.\n
 * \param[in] decelerationSet Deceleration setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Move axis 5 to position 1234 with a velocity of 10, acceleration of 100, and deceleration of 200.\n
 * "MoveAbsolutePosition(5,1234,10,100,200)" //Command string to cmd_EAT.c\n
 */
int moveAbsolutePosition(
    int axisIndex,
    double positionSet,
    double velocitySet,
    double accelerationSet,
    double decelerationSet
    );

/** \breif Move axis to a relative position.\n
 *
 * \param[in] axisIndex Axis index.\n
 * \param[in] positionSet Relative position setpoint.\n
 * \param[in] velocitySet Velocity setpoint.\n
 * \param[in] accelerationSet Acceleration setpoint.\n
 * \param[in] decelerationSet Deceleration setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Move axis 5 to 123 relative to current position with a velocity of 10,
 * acceleration of 100, and deceleration of 200.\n
 * "MoveRelativePosition(5,1234,10,100,200)" //Command string to cmd_EAT.c\n
 */
int moveRelativePosition(
    int axisIndex,
    double positionSet,
    double velocitySet,
    double accelerationSet,
    double decelerationSet);

/** \breif Move axis in a constant velocity.\n
 *
 * \param[in] axisIndex Axis index.\n
 * \param[in] velocitySet Velocity setpoint.\n
 * \param[in] accelerationSet Acceleration setpoint.\n
 * \param[in] decelerationSet Deceleration setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Move axis 5  at a constant velocity of 10, acceleration of
 * 100, and deceleration of 200.\n
 * "MoveVelocity(5,10,100,200)" //Command string to cmd_EAT.c\n
 */
int moveVelocity(int axisIndex,double velocitySet, double accelerationSet, double decelerationSet);

/** \breif Stop axis.\n
 *
 * \param[in] axisIndex Axis index.\n
 * \param[in] killAmplifier Disable amplifier.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Stop motion and kill amplifier of axis 2.\n
 * "StopMotion(2,1)" //Command string to cmd_EAT.c\n
 */
int stopMotion(int axisIndex,int killAmplifier);

/** \breif Get axis error state.\n
 *
 * \param[in] axisIndex Axis index.\n
 *
 * \return axis error state.\n
 *
 * \note Example: Get error code of axis 3.\n
 * "Main.M3.bError?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisError(int axisIndex);

/** \breif Get axis error code.\n
 *
 * \param[in] axisIndex Axis index.\n
 *
 * \return axis error code.\n
 *
 * \note Example: Get error code of axis 3.\n
 * "Main.M3.nErrorId?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisErrorID(int axisIndex);

/** \breif Get axis error code in string format.\n
 *
 * \param[in] errorNumber Error code.\n
 *
 * \return axis error string.\n
 *
 * \note Example: Get error code of axis 3.\n
 * "Main.M3.sErrorMessage?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
const char *getErrorString(int errorNumber);

/** \breif Get axis execution cycle counter.\n
 * Can be used for checking that logic for an axis object is
 * executing.\n
 *
 * \param[in] axisIndex Axis index.\n
 * \param[out] counter Execution cycle counter.\n
 *
 * \return  error code.\n
 * \note The counter can overflow.
 * \note Example: Get cycle counter of axis 3.\n
 * "GetAxisCycleCounter(3)" //Command string to cmd_EAT.c.\n
  */
int getAxisCycleCounter(int axisIndex,int *counter);

/** \breif Get axis debug information string.\n
 *
 * \param[in] axisIndex Axis index.\n
 * \param[in,out] buffer Pointer to char output data buffer.\n
 * \param[in] bufferByteSize Size of data buffer.\n
 *
 * \return error code.\n
 *
 * \note Example: Get information of axis 3.\n
 * "GetAxisDebugInfoData(3)" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisDebugInfoData(int axisIndex,char *buffer, int bufferByteSize);

/** \breif Get axis status structure V2.\n
 *
 * \param[in] axisIndex Axis index.\n
 * \param[in,out] buffer Pointer to char output data buffer.\n
 * \param[in] bufferByteSize Size of data buffer.\n
 *
 * \return error code.\n
 *
 * \note Example: Get information of axis 3.\n
 * "Main.M3.stAxisStatusV2?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisStatusStructV2(int axisIndex,char *buffer, int bufferByteSize);

/** \breif Get axis execute bit.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  state of axis execute bit.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get execute bit for axis 3.\n
 * "Main.M3.bExecute?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisExecute(int axisIndex,int *value);

/** \breif Get axis command word.\n
 *
 * The command word defines different modes of operation for and axis. See
 * fbDriveVirtual manual for more information.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  axis command word.\n
 *  value = 0: Jog.\n
 *  value = 1: Constant Velocity.\n
 *  value = 2: Relative Positioning.\n
 *  value = 3: Absolute Positioning.\n
 *  value = 10: Homing.\n
 *  See manual for fbDriveVirtual for more information.
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get command word for axis 3.\n
 * "Main.M3.nCommand?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisCommand(int axisIndex,int *value);

/** \breif Get axis command data word.\n
 *
 * The command data word is an argument linked to the axis command. See
 * fbDriveVirtual manual for more information.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  axis command data word.\n
 *
 * Command word = 10 (homing):\n
 *   value 1..6: Different homing sequences.\n
 *
 * Command word = 4 (absolute positioning): \n
 *   value 1: Go to start position of trajectory generator external
 *   transformation expressions. This is useful to avoid jumps  during
 *   start phase when absolute synchronizing.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get command data word for axis 3.\n
 * "Main.M3.nCmdData?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisCmdData(int axisIndex,int *value);

/** \breif Get axis reset bit.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  state of axis reset bit.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get reset bit for axis 3.\n
 * "Main.M3.bReset?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisReset(int axisIndex,int *value);

/** \breif Get axis amplifier state bit.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  state of axis amplifier.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get amplifier actual state for axis 3.\n
 * "Main.M3.bEnabled?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisEnabled(int cntrl_no,int *value);

/** \breif Get axis amplifier command bit.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  state of axis amplifier command bit.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get amplifier enable command bit for axis 3.\n
 * "Main.M3.bEnable?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisEnable(int axisIndex,int *value);

/** \breif Get axis busy bit.\n
 *
 * The axis busy bit is high while an command is executed or while synchronizing to other axes.
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  state of axis busy bit.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get busy state for axis 3.\n
 * "Main.M3.bBusy?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisBusy(int axisIndex,int*value);

/** \breif Get axis index.\n
 *
 * This function is only implemented for compatibility reasons with the
 * ESS MCAG TwinCAT implementation. (An TwinCAT plc axis can be linked to
 * different NC-axis).
 *
 * If the axis at axisIndex exists, axisIndex will be "returned" in the value parameter.
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  axis index of chosen axis.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get busy state for axis 3.\n
 * "Main.M3.nMotionAxisID?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisID(int axisIndex,int*value);

/** \breif Get enable alarms at limits bit.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value state of enable alarm at limits bit.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get state of enable alarm at limits bit for axis 3.\n
 * "GetAxisEnableAlarmAtHardLimits(3)" //Command string to cmd_EAT.c.\n
 */
int getAxisEnableAlarmAtHardLimits(int axisIndex,int *value);

/** \breif Get backward soft-limit position.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value soft-limit position.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get soft-limit backward position for axis 3.\n
 * "ADSPORT=501/.ADR.16#5003,16#D,8,5?" //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisSoftLimitPosBwd(int axisIndex,double *value);

/** \breif Get forward soft-limit position.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value Soft-limit position.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get soft-limit forward position for axis 3.\n
 * "ADSPORT=501/.ADR.16#5003,16#E,8,5?" //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisSoftLimitPosFwd(int axisIndex,double *value);

/** \breif Get backward soft-limit enabled state of an axis.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value soft-limit enabled.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get soft-limit backward enabled state for axis 3.\n
 * "ADSPORT=501/.ADR.16#5003,16#B,2,2?"  //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisEnableSoftLimitBwd(int axisIndex,int *value);

/** \breif Get forward soft-limit enabled state of an axis.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value soft-limit enabled.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get soft-limit forward enabled state for axis 3.\n
 * "ADSPORT=501/.ADR.16#5001,16#C,2,2?"  //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisEnableSoftLimitFwd(int axisIndex,int *value);

/** \breif Get axis operation mode.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value current operation mode of axis.\n
 * value = 0 Axis is in automatic mode (controller enabled).\n
 * value = 1 Axis is in manual mode (controller and some "safety features
 * disabled). A direct output to the drive can be set in this mode.
 * Only useful in commissioning.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get current operation mode of axis 3.\n
 * "GetAxisOpMode(3)"  //Command string to cmd_EAT.c.\n
 */
int getAxisOpMode(int axisIndex,int *value);

/** \breif Get axis acceleration setpoint.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value acceleration setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get acceleration setpoint for axis 3.\n
 * "Main.M3.fAcceleration?"  //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisAcceleration(int axisIndex,double *value);

/** \breif Get axis deceleration setpoint.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value deceleration setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get deceleration setpoint for axis 3.\n
 * "Main.M3.fDeceleration?"  //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisDeceleration(int axisIndex,double *value);

/** \breif Get axis target position setpoint.\n
 *
 * The target position is the desired end setpoint of a motion.
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value target position setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get target position setpoint for axis 3.\n
 * "Main.M3.fPosition?"  //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisTargetPos(int axisIndex,double *value);

/** \breif Get axis target velocity setpoint.\n
 *
 * The target velocity is the desired velocity of a motion.\n
 * Note: The actual velocity can be higher than this setpoint.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value target velocity setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get target velocity setpoint for axis 3.\n
 * "Main.M3.fVelocity?"  //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisTargetVel(int axisIndex,double *value);

/** \breif Get axis done bit.\n
 *
 * The axis done bit is high when an axis is ready to take a new command
 * (inverse of busy bit).\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  state of axis done bit.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get done state for axis 3.\n
 * "Main.M3.bDone?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisDone(int axisIndex,double *value);

/** \breif Get gear ration setting.\n
 *
 * Note: The gear ratio is only valid during synchronized motions.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  gear ratio.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get gear ratio setting for axis 3.\n
 * "ReadAxisGearRatio(3)*" //Command string to cmd_EAT.c.\n
 */
int getAxisGearRatio(int axisIndex,double *value);

/** \breif Get state of forward hard limit.\n
 *
 * Checks state of forward hard limit switch.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  state of forward limit switch.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get state of forward limit switch for axis 3.\n
 * "Main.M3.bLimitFwd?*" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisAtHardFwd(int axisIndex,int *value);

/** \breif Get state of backward hard limit.\n
 *
 * Checks state of backward hard limit switch.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  state of backward limit switch.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get state of backward limit switch for axis 3.\n
 * "Main.M3.bLimitBwd?*" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisAtHardBwd(int axisIndex,int *value);

/** \breif Get encoder homed bit.\n
 *
 * Checks if encoder has been homed.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  state of encoder homed bit.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Check if encoder of axis 3 has been homed.\n
 * "Main.M3.bHomed?*" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisEncHomed(int axisIndex,int *value);

/** \breif Get actual encoder position.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  encoder actual position.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get actual encoder position for axis 3.\n
 * "Main.M3.fActPosition?*" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisEncPosAct(int axisIndex,double *value);

/** \breif Get actual encoder velocity.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  encoder actual velocity.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get actual encoder velocity for axis 3.\n
 * "Main.M3.fActVelocity?*" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisEncVelAct(int axisIndex,double *value);

/** \breif Get state of reference/home switch.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value  state of reference/home switch.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get state of reference/home switch for axis 3.\n
 * "Main.M3.stAxisStatus?" //The command returns a complete structure of
 * information including the state of the reference switch
 * //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisAtHome(int axisIndex,int *value);

/** \breif Get actual pid controller error.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value actual error of pid controller.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get pid controller error for axis 3.\n
 * "Main.M3.stAxisStatus?" //The command returns a complete structure of
 * information including the state of the pid controller error *
 * //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisCntrlError(int axisIndex,double *value);

/** \breif Get off cam referencing/homing velocity setpoint.\n
 *
 * This velocity setpoint is only used during homing sequence when the
 * reference switch already have been found. See command
 * "getAxisHomeVelTwordsCam()" for more information.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value Velocity setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get off cam referencing/homing velocity setpoint for axes 3.\n
 * "ADSPORT=501/.ADR.16#4003,16#7,8,5?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisHomeVelOffCam(int axisIndex,double *value);

/** \breif Get twords cam referencing/homing velocity setpoint.\n
 *
 * This velocity setpoint is only used during homing sequence when finding
 * the reference switch. See command "getAxisHomeVelTwordsCam()" for more
 * information.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value Velocity setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get twords cam referencing/homing velocity setpoint for axes 3.\n
 * "ADSPORT=501/.ADR.16#4003,16#6,8,5?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisHomeVelTwordsCam(int axisIndex,double *value);

/** \breif Get the numerator part of the encoder scale.\n
 *
 * The encoder scale factor is divided into one numerator and one denominator
 * part. This function reads the numerator part.
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value Encoder scale numerator part.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get encoder scale numerator for axes 3.\n
 * "ADSPORT=501/.ADR.16#5003,16#23,8,5?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisEncScaleNum(int axisIndex,double *value);

/** \breif Get the denominator part of the encoder scale.\n
 *
 * The encoder scale factor is divided into one numerator and one denominator
 * part. This function reads the denominator part.
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value Encoder scale denominator part.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get encoder scale denominator for axes 8.\n
 * "ADSPORT=501/.ADR.16#5008,16#24,8,5?" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int getAxisEncScaleDenom(int axisIndex,double *value);

/** \breif Get raw unscaled encoder value.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value Raw encoder value.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get raw encoder value for axis 3.\n
 * "GetAxisEncPosRaw(3)" //Command string to cmd_EAT.c.\n
 */
int getAxisEncPosRaw(int axisIndex,int64_t *value);

/** \breif Get enable state of PID-controller.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value PID-controller enabled.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note No command string implemented in the cmd_EAT.c parser.\n
 */
//int getAxisCntrlEnable(int axisIndex,int *value);
//int getAxisCntrlTargetPos(int axisIndex,double *value);
//int getAxisCntrlEnabled(int axisIndex,int *value);

/** \breif Get PID-controller proportional output part.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value PID-controller proportional output.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note No command string implemented in the cmd_EAT.c parser.\n
 */
int getAxisCntrlOutPpart(int axisIndex,double *value);

/** \breif Get PID-controller integral output part.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value PID-controller integral output.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note No command string implemented in the cmd_EAT.c parser.\n
 */
int getAxisCntrlOutIpart(int axisIndex,double *value);

/** \breif Get PID-controller differential output part.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value PID-controller differential output.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note No command string implemented in the cmd_EAT.c parser.\n
 */
int getAxisCntrlOutDpart(int axisIndex,double *value);

/** \breif Get PID-controller feed forward output part.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value PID-controller feed forward output.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note No command string implemented in the cmd_EAT.c parser.\n
 */
int getAxisCntrlOutFFpart(int axisIndex,double *value);

/** \breif Get PID-controllegetAxisAtHomer total output part.\n
 *
 * The current total output from the PID-controller (the sum of the P,I,D and
 * FF part).\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value PID-controller total output.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note No command string implemented in the cmd_EAT.c parser.\n
 */
int getAxisCntrlOutput(int axisIndex,double *value);

//int getAxisCntrlVelSet(int axisIndex,double *value);
//int getAxisCntrlRate(int axisIndex,double *value);

/** \breif Get the drive output scale factor.\n
 *
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value Drive output scale factor.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note No command string implemented in the cmd_EAT.c parser.\n
 */
int getAxisDrvScale(int axisIndex,double *value);

/** \breif Get enable state of drive.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value Drive enabled.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note No command string implemented in the cmd_EAT.c parser.\n
 */
int getAxisDrvEnable(int axisIndex,int *value);

/** \breif Get at target.\n
 *
 * Checks if axis is within a certain tolerance from target position.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value At target.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note No command string implemented in the cmd_EAT.c parser.\n
 */
int getAxisMonAtTarget(int axisIndex,int *value);
//int getAxisMonAtTargetCounter(int axisIndex,int *value);
//int getAxisMonLagError(int axisIndex,double *value);
//int getAxisMonLagMonCounter(int axisIndex,double *value);
//int getAxisMonAtHome(int axisIndex,int *value);
//int getAxisTrajTransEnable(int axisIndex, int *value);
//int getAxisEncTransEnable(int axisIndex, int *value);

/** \breif Get axis type.\n
 *
 * An axis can be of the following types:
 *   type = 1 : Normal axis (drive, encoder, monitor,pid-controller,trajectory).\n
 *   type = 2 : Virtual axis (encoder, monitor, trajectory).\n
 *   type = 3 : Trajectory axis (monitor, trajectory generator).\n
 *   type = 4 : Encoder axis (encoder, monitor).\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value Axis type.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get encoder scale denominator for axes 8.\n
 * "GetAxisType(8)" //Command string to cmd_EAT.c.\n
 */
int getAxisType(int axisIndex, int *value);

/** \breif Get axis trajectory transformation expression.\n
 *
 * The axis transformation expression is used for synchronization of axes. The
 * expression is a mathematical expression describing relation ship between
 * different axes.\n
 *
 * Example: "out:=sin(traj1+enc5)/500;".\n
 *   traj1 = trajectory setpoint of axis 1.\n
 *   enc5  = actual encoder position of axis 5.\n
 *   out   = the trajectory setpoint for the current axis (axes[axisIndex]).\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] error Error code.\n
 * error = 0: No error.\n
 * error > 0: Error.\n
 *
 * \return pointer to transformation expression.\n
 *
 * \note Example: Get trajectory transformation expression for axes 5.\n
 * "GetAxisTrajTransExpr(5)" //Command string to cmd_EAT.c.\n
 */
const char* getAxisTrajTransExpr(int axisIndex, int *error);

/** \breif Get axis encoder transformation expression.\n
 *
 * The axis transformation expression is used for synchronization of axes. The
 * expression is a mathematical expression describing relation ship between
 * different axes.\n
 *
 * Example: "out:=sin(traj1+enc5)/500;".\n
 *   traj1 = trajectory setpoint of axis 1.\n
 *   enc5  = actual encoder position of axis 5.\n
 *   out   = the encoder setpoint for the current axis (axes[axisIndex]).\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] error Error code.\n
 * error = 0: No error.\n
 * error > 0: Error.\n
 *
 * \return pointer to transformation expression.\n
 *
 * \note Example: Get encoder transformation expression for axes 5.\n
 * "GetAxisEncTransExpr(5)" //Command string to cmd_EAT.c.\n
 */
const char* getAxisEncTransExpr(int axisIndex, int *error);

/** \breif Get axis command transformation expression.\n
 *
 * The axis transformation expression is used for enabling and executing of
 * axes based on mathematical expressions. This is useful when synchronizing
 * axes i.e. a slave axis could be enabled and executed at the same time as
 * the master axis.\n
 *
 * Example: Enable of axis 2 is related to the enable command of axis 1 and 5.
 * The execute command is related to an expression including the
 * execute command for axis 2 and 7.\n
 * "en2:=en1 or en5; ex1:=ex2 + ex7;".\n
 *   enY = enable command for axis Y.\n
 *   exY = execute command for axis Y.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] error Error code.\n
 * error = 0: No error.\n
 * error > 0: Error.\n
 *
 * \return pointer to transformation expression.\n
 *
 * \note Example: Get command transformation expression for axes 5.\n
 * "GetAxisTransformCommandExpr(5)" //Command string to cmd_EAT.c.\n
 */
const char* getAxisTransformCommandExpr(int axisIndex, int *error);

/** \breif Get axis trajectory data source.\n
 *
 * An axis trajectory generator can get position setpoints from different
 * sources:\n
 *   source = 0 : Internal (normal trajectory generation).\n
 *   source = 1 : External from transformation expression (synchronization).\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value Source type.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get trajectory generator data source type for axis 3.\n
 * "GetAxisTrajSourceType(3)" //Command string to cmd_EAT.c.\n
 */
int getAxisTrajSource(int axisIndex, int *value);

/** \breif Get axis encoder data source.\n
 *
 * An axis encoder can get actual position from different sources:\n
 *   source = 0 : Internal (EtherCAT entry).\n
 *   source = 1 : External from transformation expression (synchronization).\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value Source type.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get encoder generator data source type for axis 3.\n
 * "GetAxisEncSourceType(3)" //Command string to cmd_EAT.c.\n
 */
int getAxisEncSource(int axisIndex, int *value);

/** \breif Get axis enable command from other axis.\n
 *
 * An axis can receive commands from an other axis command expression.
 * However, the axis must be allow to receive these commands, see
 * command getAxisTransformCommandExpr() for more information.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value enable.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get axis enable command from other axis for axis 3.\n
 * "GetAxisEnableCommandsFromOtherAxis(3)" //Command string to cmd_EAT.c.\n
 */
int getAxisEnableCommandsFromOtherAxis(int axisIndex, int *value);

/** \breif Get axis enable command transform.\n
 *
 * The command transformation expression for an axis can be enabled/disabled.
 * see command getAxisTransformCommandExpr() for more information.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value enable.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get axis enable command transform for axis 5.\n
 * "GetAxisEnableCommandsTransform(5)" //Command string to cmd_EAT.c.\n
 */
int getAxisEnableCommandsTransform(int axisIndex, int *value);

/** \breif Set axis execute bit.\n
 *
 * An positive edge of the execute bit triggers a new command.\n
 * Motion is interlocked when the execute bit is low.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Execute bit.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set execute bit for axis 3.\n
 * "Main.M3.bExecute=1" //Command string to cmd_EAT.c.\n
 *
 * Example: Stop motion on axis 3.\n
 * "Main.M3.bExecute=0" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisExecute(int axisIndex, int value);

/** \breif Set axis command word.\n
 *
 * The command word defines different modes of operation for and axis. See
 * fbDriveVirtual manual for more information.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value  axis command word.\n
 *  value = 0: Jog.\n
 *  value = 1: Constant Velocity.\n
 *  value = 2: Relative Positioning.\n
 *  value = 3: Absolute Positioning.\n
 *  value = 10: Homing.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set command word to absolute positioning for axis 4.\n
 * "Main.M4.nCommand=3" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisCommand(int axisIndex, int value);

/** \breif Set axis command data word.\n
 *
 * The command data word is an argument linked to the axis command. See
 * fbDriveVirtual manual for more information.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value  Axis command data word.\n
 *
 * The command data word have different meaning depending on the command
 * word.\n
 * Command word = 10 (homing):\n
 *   Command Data 1..6: Different homing sequences.\n
 *
 * Command word = 4 (absolute positioning): \n
 *   Command Data 1: Go to start position of trajectory generator external
 *   transformation expressions. This is useful to avoid jumps  during
 *   start phase when absolute synchronizing.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Prepare homing sequence 3 at next positive edge of execute
 * for axis 3 (two command needed).\n
 * "Main.M3.nCommand=10" //Set homing. Command string to cmd_EAT.c.\n
 * "Main.M3.nCmdData=3"  //Set homing sequence. Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisCmdData(int axisIndex, int value);

/** \breif Set axis amplifier enable command bit.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value  State of axis amplifier command bit.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set amplifier enable command bit for axis 3.\n
 * "Main.M3.bEnable=1" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisEnable(int axisIndex, int value);

/** \breif Set enable alarms at limits bit.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Enable alarm at limits bit.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable alarm at limits bit for axis 3.\n
 * "Cfg.SetAxisEnableAlarmAtHardLimits(3,1)" //Command string to cmd_EAT.c.\n
 */
int setAxisEnableAlarmAtHardLimits(int axisIndex,int value);

/** \breif Set enable backward soft-limit of an axis.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] value Soft-limit enable command.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Disable backward soft-limit for axis 3.\n
 * "ADSPORT=501/.ADR.16#5003,16#B,2,2=0"  //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisEnableSoftLimitBwd(int axisIndex, int value);

/** \breif Set enable forward soft-limit of an axis.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Soft-limit enable command.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable forward soft-limit for axis 3.\n
 * "ADSPORT=501/.ADR.16#5001,16#C,2,2=1"  //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisEnableSoftLimitFwd(int axisIndex, int value);

/** \breif Set backward soft-limit position.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Soft-limit position.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set backward soft-limit position to -100 for axis 3.\n
 * "ADSPORT=501/.ADR.16#5003,16#E,8,5=-100" //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisSoftLimitPosBwd(int axisIndex, double value);

/** \breif Set forward soft-limit position.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Soft-limit position.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set forward soft-limit position to 123 for axis 3.\n
 * "ADSPORT=501/.ADR.16#5003,16#E,8,5=123" //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisSoftLimitPosFwd(int axisIndex, double value);

/** \breif Set axis acceleration setpoint.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Acceleration setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set acceleration setpoint for axis 3 to 500.\n
 * "Main.M3.fAcceleration=500"  //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisAcceleration(int axisIndex, double value);

/** \breif Set axis deceleration setpoint.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Deceleration setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set deceleration setpoint for axis 3 to 500.\n
 * "Main.M3.fDeceleration=500"  //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisDeceleration(int axisIndex, double value);

/** \breif Set axis emergency deceleration setpoint.\n
 *
 * The emergency deceleration setpoint is used:\n
 * - limit switch is engaged.\n
 * - both limit switches are engaged.\n
 * - ...
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Emergency deceleration setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set emergency deceleration setpoint for axis 3 to 54321.\n
 * "Cfg.SetAxisEmergDeceleration(3,54321)"  //Command string to cmd_EAT.c.\n
 */
int setAxisEmergDeceleration(int axisIndex, double value);

/** \breif Set axis maximum jerk setpoint. NOT USED!\n
 *
 * \note Currently not used!!!!!.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Jerk setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: No command string implemented in the cmd_EAT.c parser.\n
 */
int setAxisJerk(int axisIndex, double value);

/** \breif Set axis target position setpoint.\n
 *
 * The target position is the desired end setpoint of a motion.
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Target position setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set target position setpoint for axis 3 to 111.\n
 * "Main.M3.fPosition=111"  //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisTargetPos(int axisIndex, double value);

/** \breif Set axis target velocity setpoint.\n
 *
 * The target velocity is the desired velocity of a motion.\n
 * Note: The actual velocity can be higher than this setpoint.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Target velocity setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set target velocity setpoint for axis 3 to 55.\n
 * "Main.M3.fVelocity=55"  //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisTargetVel(int axisIndex, double value);

/** \breif Set axis jog velocity setpoint.\n
 *
 * The jog velocity is the desired velocity of a motion when executing a jog
 * command (command=10).\n
 * Note: The actual velocity can be higher than this setpoint.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Jog velocity setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set target velocity setpoint for axis 3 to 55.\n
 * "Cfg.SetAxisJogVel(3,55)"  //Command string to cmd_EAT.c.\n
 */
int setAxisJogVel(int axisIndex, double value);

/** \breif Set axis operation mode.\n
 *
 * \note This command can disable certain safety functionality and
 * should only be used with high caution and only during commissioning.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Current operation mode of axis.\n
 * value = 0 Axis is in automatic mode (controller enabled).\n
 * value = 1 Axis is in manual mode (controller and some "safety features
 * disabled). A direct output to the drive can be set in this mode.
 * Only useful in commissioning.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set current operation mode of axis 3 to manual.\n
 * "Cfg.SetAxisOpMode(3,1)"  //Command string to cmd_EAT.c.\n
 */
int setAxisOpMode(int axisIndex, int value);

/** \breif Set the denominator part of the encoder scale.\n
 *
 * The encoder scale factor is divided into one numerator and one denominator
 * part. This function reads the denominator part.
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Encoder scale denominator part.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set encoder scale denominator for axes 8 to 4096.\n
 * "ADSPORT=501/.ADR.16#5008,16#24,8,5=4096" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisEncScaleDenom(int axisIndex, double value);

/** \breif Set the numerator part of the encoder scale.\n
 *
 * The encoder scale factor is divided into one numerator and one denominator
 * part. This function reads the numerator part.
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Encoder scale numerator part.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set encoder scale numerator for axes 3 to 360.\n
 * "ADSPORT=501/.ADR.16#5003,16#23,8,5=360" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisEncScaleNum(int axisIndex, double value);

/** \breif Set axis home reference position setpoint.\n
 *
 * The home reference position setpoint is only used during referencing/homing
 * sequence. The final referencing will be done to this position
 * (the reference switch edge will have this positon after referenceing).\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Home position setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set home position setpoint for axis 3 to 111.\n
 * "Main.M3.fHomePosition=111"  //Command string to cmd_EAT.c.\n
 *
 * \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisHomePos(int axisIndex, double value);

/** \breif Set twords cam referencing/homing velocity setpoint.\n
 *
 * This velocity setpoint is only used during homing sequence when finding
 * the reference switch. See command "getAxisHomeVelTwordsCam()" for more
 * information.\n
 * Normally the off cam velocity setpoint is lower than the twords cam setpoint
 * since the accuracy of homing is depending on how accurate the signal edge
 * of the reference switch can be identified.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Velocity setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set twords cam referencing/homing velocity setpoint for axes
 * 3 to 10.\n
 * "ADSPORT=501/.ADR.16#4003,16#6,8,5=10" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisHomeVelTwordsCam(int axisIndex,double dVel);

/** \breif Set off cam referencing/homing velocity setpoint.\n
 *
 * This velocity setpoint is only used during homing sequence when the
 * reference switch already have been found. See command
 * "getAxisHomeVelTwordsCam()" for more information.\n
 * Normally the off cam velocity setpoint is lower than the twords cam setpoint
 * since the accuracy of homing is depending on how accurate the signal edge
 * of the reference switch can be identified.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Velocity setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set off cam referencing/homing velocity setpoint for axes
 * 3 to 7.5.\n
 * "ADSPORT=501/.ADR.16#4003,16#7,8,5=7.5" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int setAxisHomeVelOffCam(int axisIndex,double dVel);

/** \breif Set start direction of referencing/homing sequence. NOTE: NOT
 * USED!!\n
 *
 * \note This command is not used. The different directions are implemented as
 * different values of command data instead.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Velocity setpoint.\n
 *    value = 0: Forward direction.\n
 *    value = 1: Backward direction.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set off referencing/homing direction for axes
 * 3 to positive.\n
 * "Cfg.SetAxisHomeDirection(3,0)" //Command string to cmd_EAT.c.\n
 */
int setAxisHomeDir(int axisIndex,int nDir);

/** \breif Set gear ration setting.\n
 *
 * Note: The gear ratio is only valid during synchronized motions.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] ratioNum  Gear ratio factor numerator.\n
 * \param[in] ratioDenom  Gear ratio factor denominator.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set gear ratio setting for axis 3 to 1/7.\n
 * "Cfg.SetAxisGearRatio(3,1,7)" //Command string to cmd_EAT.c.\n
 */
int setAxisGearRatio(int axisIndex,double ratioNum,double ratioDenom);

/** \breif Set axis reset bit.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value  State of axis reset bit.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set reset bit for axis 3.\n
 * "Main.M3.bReset=1" //Command string to cmd_EAT.c.\n
 *
 *  \todo  "TwinCAT syntax. Needs to be changed.\n
 */
int axisErrorReset(int axisIndex, int value);

/** \breif Set axis trajectory transformation expression.\n
 *
 * The axis transformation expression is used for synchronization of axes. The
 * expression is a mathematical expression describing relation ship between
 * different axes.\n
 *
 * Example: "out:=sin(traj1+enc5)/500;il1=il2 and enc3>123;".\n
 *   trajY = trajectory setpoint of axis Y.\n
 *   encY  = actual encoder position of axis Y.\n
 *   out   = the trajectory setpoint for the current axis (axes[axisIndex]).\n
 *   ilY   = interlock state for axis Y..\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] expr Expression.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set trajectory transformation expression for axes 5 to
 * "out:=sin(traj1+enc5)/500#il1=il2 and enc3>123#".\n
 * "Cfg.SetAxisTrajTransExpr(5)=out:=sin(traj1+enc5)/500#il1=il2 and enc3>123#" //Command string to cmd_EAT.c.\n
 */
int setAxisTrajTransExpr(int axisIndex, char *expr);

/** \breif Enables/disables velocity filter of external setpoint.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] enable enable filter.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable velocity filter for external setpoint position for axis 5.
 * "Cfg.SetAxisTrajExtVelFilterEnable(5,1) //Command string to cmd_EAT.c.\n
 */
int setAxisTrajExtVelFilterEnable(int axisIndex, int enable);

/** \breif Set axis encoder transformation expression.\n
 *
 * The axis transformation expression is used for synchronization of axes. The
 * expression is a mathematical expression describing relation ship between
 * different axes.\n
 *
 * Example: "out:=sin(traj1+enc5)/500;".\n
 *   trajY = trajectory setpoint of axis Y.
 *   encY  = actual encoder position of axis Y.
 *   out   = the trajectory setpoint for the current axis (axes[axisIndex]).\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] expr Expression.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set encoder transformation expression for axes 5 to
 * "out:=sin(traj1+enc5)/500#il1=il2 and enc3>123#".\n
 * "Cfg.SetAxisEncTransExpr(5)=out:=sin(traj1+enc5)/500#il1=il2 and enc3>123#" //Command string to cmd_EAT.c.\n
 */
int setAxisEncTransExpr(int axisIndex, char *expr);

/** \breif Enables/disables velocity filter of external actual value.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] enable enable filter.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable velocity filter for external actual position for axis 5.
 * "Cfg.SetAxisEncExtVelFilterEnable(5,1) //Command string to cmd_EAT.c.\n
 */
int setAxisEncExtVelFilterEnable(int axisIndex, int enable);

/** \breif Set axis trajectory data source.\n
 *
 * An axis trajectory generator can get position setpoints from different
 * sources:\n
 *   source = 0 : Internal (normal trajectory generation).\n
 *   source = 1 : External from transformation expression (synchronization).\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Source type.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set trajectory generator data source type for axis 3 to
 * internal.\n
 * "Cfg.SetAxisTrajSourceType(3,0)" //Command string to cmd_EAT.c.\n
 */
int setAxisTrajSource(int axisIndex, int value);

/** \breif Set axis encoder data source.\n
 *
 * An axis encoder can get actual position from different sources:\n
 *   source = 0 : Internal (EtherCAT entry).\n
 *   source = 1 : External from transformation expression (synchronization).\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Source type.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set encoder data source type for axis 3 to
 * external.\n
 * "Cfg.SetAxisEncSourceType(3,1)" //Command string to cmd_EAT.c.\n
 */
int setAxisEncSource(int axisIndex, int value);

/** \breif Set axis trajectory start position. NOTE: NOT USED!!.\n
 *
 * \note This function is not used currently.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Start position.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set trajectory start position to 100 for axis 3.\n
 * "Cfg.SetAxisTrajStartPos(3,100)" //Command string to cmd_EAT.c.\n
 */
int setAxisTrajStartPos(int axisIndex,double value);

/** \breif Set encoder offset value.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Offset position.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set encoder offset value to 100 for axis 3.\n
 * "Cfg.SetAxisEncOffset(3,100)" //Command string to cmd_EAT.c.\n
 */
int setAxisEncOffset(int axisIndex, double value);

/** \breif Set encoder type.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Encoder type.\n
 *  value = 0: Incremental encoder.\n
 *  value = 1: Absolute encoder.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set encoder type to absolute for axis 3.\n
 * "Cfg.SetAxisEncType(3,1)" //Command string to cmd_EAT.c.\n
 */
int setAxisEncType(int axisIndex, int value);

/** \breif Set encoder register bit count.\n
 *
 * The bit count is used to handle over/under flow.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Encoder register bit count.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set encoder bit count to 16 for axis 3.\n
 * "Cfg.SetAxisEncBits(3,16)" //Command string to cmd_EAT.c.\n
 */
int setAxisEncBits(int axisIndex, int value);

/** \breif Set PID-controller proportional gain.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Proportional gain.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set PID-controller proportional gain to 1.5 for axis 3.\n
 * "Cfg.SetAxisCntrlKp(3,1.5)" //Command string to cmd_EAT.c.\n
 */
int setAxisCntrlKp(int axisIndex, double value);

/** \breif Set PID-controller integral gain.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Integral gain.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set PID-controller integral gain to 0.1 for axis 3.\n
 * "Cfg.SetAxisCntrlKi(3,0.1)" //Command string to cmd_EAT.c.\n
 */
int setAxisCntrlKi(int axisIndex, double value);

/** \breif Set PID-controller differential gain.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Differential gain.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set PID-controller differential gain to 1.1 for axis 3.\n
 * "Cfg.SetAxisCntrlKd(3,1.1)" //Command string to cmd_EAT.c.\n
 */
int setAxisCntrlKd(int axisIndex, double value);

/** \breif Set PID-controller feed forward gain.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Feed forward gain.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set PID-controller feed forward gain to 4.1 for axis 3.\n
 * "Cfg.SetAxisCntrlKff(3,4.1)" //Command string to cmd_EAT.c.\n
 */
int setAxisCntrlKff(int axisIndex, double value);

/** \breif Set PID-controller maximum output value.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Max value.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set PID-controller max value to 1000 for axis 3.\n
 * "Cfg.SetAxisCntrlOutHL(3,1000)" //Command string to cmd_EAT.c.\n
 */
int setAxisCntrlOutHL(int axisIndex, double value);

/** \breif Set PID-controller minimum output value.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Min value.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set PID-controller min value to -1000 for axis 3.\n
 * "Cfg.SetAxisCntrlOutLL(3,-1000)" //Command string to cmd_EAT.c.\n
 */
int setAxisCntrlOutLL(int axisIndex, double value);

/** \breif Set PID-controller minimum integral part output value.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Min integral value.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set PID-controller minimum integral part output value to -700 for axis 3.\n
 * "Cfg.SetAxisCntrlIpartLL(3,-700)" //Command string to cmd_EAT.c.\n
 */
int setAxisCntrlIpartLL(int axisIndex, double value);

/** \breif Set PID-controller maximum integral part output value.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Max integral value.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set PID-controller maximum integral part output value to 700 for axis 3.\n
 * "Cfg.SetAxisCntrlIpartHL(3,700)" //Command string to cmd_EAT.c.\n
 */
int setAxisCntrlIpartHL(int axisIndex, double value);

/** \breif Get drive output scale numerator.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] pointer to return value Scale numerator.\n
 *
 * \return 0 if success or otherwise an error code.\n
 */
int getAxisDrvScaleNum(int axisIndex, double *value);

/** \breif Set drive output scale numerator.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Scale numerator.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set drive output scale numerator to 10000 for axis 3.\n
 * "Cfg.SetAxisDrvScaleNum(3,10000)" //Command string to cmd_EAT.c.\n
 */
int setAxisDrvScaleNum(int axisIndex, double value);

/** \breif Set drive output scale denominator.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Scale denominator.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set drive output scale denominator to 32000 for axis 3.\n
 * "Cfg.SetAxisDrvScaleDenom(3,32000)" //Command string to cmd_EAT.c.\n
 */
int setAxisDrvScaleDenom(int axisIndex, double value);

/** \breif Set drive amplifier enable. NOTE: Only used in manual mode!!!\n
 *
 * \note This function is only used in manual mode otherwise use
 * setAxisEnable() instead.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Enable amplifier.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable amplifier for axis 3.\n
 * "Cfg.SetAxisDrvEnable(3,1)" //Command string to cmd_EAT.c.\n
 */
int setAxisDrvEnable(int axisIndex, int value);

/** \breif Set drive velocity setpoint. NOTE: Only used in manual mode!!!\n
 *
 * \note This function is only used in manual mode otherwise use.\n
 *
 * If in manual mode and amplifier enabled the velocity setpoint will be
 * rescaled and output to the drive.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Drive velocity setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set drive velocity setpoint to 10 for axis 3.\n
 * "Cfg.SetAxisDrvVelSet(3,10)" //Command string to cmd_EAT.c.\n
 */
int setAxisDrvVelSet(int axisIndex, double value);

/** \breif Set drive raw velocity setpoint. NOTE: Only used in manual mode!!!\n
 *
 * \note This function is only used in manual mode otherwise use.\n
 *
 * If in manual mode and amplifier enabled the raw velocity setpoint will be
 * directly output to the drive (without being rescaled).\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Raw drive velocity setpoint.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set raw drive velocity setpoint to 1000 for axis 3.\n
 * "Cfg.SetAxisDrvVelSetRaw(3,1000)" //Command string to cmd_EAT.c.\n
 */
int setAxisDrvVelSetRaw(int axisIndex, int value);

/** \breif Set enable of brake.\n
 *
 *  The brake output will follow the amplifier enable state of the drive. The
 *  brake is high when amplifier enable is high and low when amplifier enable
 *  is low.\n
 *
 *  Note: Delays are can be configured with:\n
 *        -setAxisDrvBrakeOpenDelayTime() \n
 *        -setAxisDrvBrakeCloseAheadTime() \n
 *
 *  Note: A valid EtherCAT entry must be linked to the drive objects brake
 *  entry in order to work like intended.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Brake enable.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable brake for axis 3.\n
 * "Cfg.SetAxisDrvBrakeEnable(3,1)" //Command string to cmd_EAT.c.\n
 */
int setAxisDrvBrakeEnable(int axisIndex, int enable);

/** \breif Set brake open delay time .\n
 *
 *  The release of the brake will be delayed for an additional
 *  delay time when enabling the amplifier.
 *
 *  Note: A valid EtherCAT entry must be linked to the drive objects brake
 *  entry in order to work like intended.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] delayTime Delay time in cycles.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set brake open delay time for axis 3 to 100 cycles.\n
 * "Cfg.SetAxisDrvBrakeOpenDelayTime(3,100)" //Command string to cmd_EAT.c.\n
 */
int setAxisDrvBrakeOpenDelayTime(int axisIndex, int delayTime);

/** \breif Set brake close ahead time .\n
 *
 *  Activation of the brake will be made prior to disabling the amplifier.\n
 *
 *  Note: A valid EtherCAT entry must be linked to the drive objects brake
 *  entry in order to work like intended.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] aheadTime Ahead time in cycles.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set brake close ahead time for axis 3 to 100 cycles.\n
 * "Cfg.SetAxisDrvBrakeCloseAheadTime(3,100)" //Command string to cmd_EAT.c.\n
 */
int setAxisDrvBrakeCloseAheadTime(int axisIndex, int aheadTime);

/** \breif Set enable of reduce torque functionality.\n
 *
 *  The reduce torque output will go high when the axis is atTarget. The
 *  "atTarget" monitoring functionality needs to be configured in order to
 *  reduce torque at the correct time.\n
 *
 *  Note: A valid EtherCAT entry must be linked to the drive objects reduce torque
 *  entry in order to work like intended.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Reduce torque enable.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable reduce torque for axis 3.\n
 * "Cfg.SetAxisDrvReduceTorqueEnable(3,1)" //Command string to cmd_EAT.c.\n
 */
int setAxisDrvReduceTorqueEnable(int axisIndex, int enable);

/** \breif Set drive type.\n
 *
 * \note  ALL SETTINGS MADE TO THE DRIVE WILL BE OVERWRITTEN.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] type Drive type.\n
 *                 0 = Stepper drive.\n
 *                 1 = DS402 drive.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set axis 3 drive type to stepper.\n
 * "Cfg.SetAxisDrvType(3,0)" //Command string to cmd_EAT.c.\n
 */
int setAxisDrvType(int axisIndex, int type);

/** \breif Get "at target" monitoring tolerance.\n
 * \param[in] axisIndex  Axis index.\n
 * \param[out] pointer to return value At target tolerance.\n
 *
 * \return 0 if success or otherwise an error code.\n
 */
int getAxisMonAtTargetTol(int axisIndex, double *value);

/** \breif Set "at target" monitoring tolerance.\n
 *
 *  The motion will be considered to have reached the target position
 *  when the difference between the actual position and target position
 *  is within this tolerance for a certain number of cycles, see command
 *  setAxisMonAtTargetTime() for more information.\n
 *
 *  Note: The "at target" functionality needs to be enabled by the command
 *  setAxisMonEnableAtTargetMon().\n
 *
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value At target tolerance.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set at target tolerance to 0.1 for axis 7.\n
 * "Cfg.SetAxisMonAtTargetTol(7,0.1)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonAtTargetTol(int axisIndex, double value);

/** \breif Get "at target" monitoring time (cycles).\n
 * \param[in] axisIndex  Axis index.\n
 * \param[out] pointer to return value At target time (cycles) .\n
 *
 * \return 0 if success or otherwise an error code.\n
 */
int getAxisMonAtTargetTime(int axisIndex, int *value);

/** \breif Set "at target" monitoring time (cycles).\n
 *
 *  The motion will be considered to have reached the target position
 *  when the difference between the actual position and target position
 *  is within a certain tolerance for a this number of cycles, see command
 *  setAxisMonAtTargetTol() for more information.\n
 *
 *  Note: The "at target" functionality needs to be enabled by the command
 *  setAxisMonEnableAtTargetMon().\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value At target time (cycles) .\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set at target tolerance time (cycles) to 10 for axis 7.\n
 * "Cfg.SetAxisMonAtTargetTime(7,10)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonAtTargetTime(int axisIndex, int value);

/** \breif Get enable "at target" monitoring.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] pointer to return value Enable monitoring .\n
 *
 * \return 0 if success or otherwise an error code.\n
 */
int getAxisMonEnableAtTargetMon(int axisIndex, int *value);

/** \breif Enable "at target" monitoring.\n
 *
 *  The motion will be considered to have reached the target position
 *  when the difference between the actual position and target position
 *  is within a certain tolerance for a this number of cycles, see command
 *  setAxisMonAtTargetTol() and setAxisMonAtTargetTime() for more information.
 *  \n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Enable monitoring .\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable at target monitoring for axis 7.\n
 * "Cfg.SetAxisMonEnableAtTargetMon(7,1)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonEnableAtTargetMon(int axisIndex, int value);

/** \breif Get position lag maximum monitoring tolerance.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] pointer to return value Position lag maximum tolerance.\n
 *
 * \return 0 if success or otherwise an error code.\n
 */
int getAxisMonPosLagTol(int axisIndex, double *value);

/** \breif Set position lag maximum monitoring tolerance.\n
 *
 *  The position lag monitoring functionality monitors the difference between
 *  the actual position and trajectory generated setpoint position. The motion
 *  can be interlocked if the difference exceeds this tolerance for a certain
 *  number of cycles, see command setAxisMonPosLagTime() for more information.
 *  \n
 *  A big position lag error during a motion could be generated by bad tuning or
 *  mechanical issues.\n
 *
 *  Note: The position lag functionality needs to be enabled with the command
 *  setAxisMonEnableLagMon().\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Position lag maximum tolerance.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set maximum position lag tolerance to 0.2 for axis 7.\n
 * "Cfg.SetAxisMonPosLagTol(7,0.2)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonPosLagTol(int axisIndex, double value);

/** \breif Get position lag monitoring time (cycles).\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[out] pointer to return value Position lag time (cycles).\n
 *
 * \return 0 if success or otherwise an error code.\n
 */
int getAxisMonPosLagTime(int axisIndex, int *value);

/** \breif Set position lag monitoring time (cycles).\n
 *
  *  The position lag monitoring functionality monitors the difference between
 *  the actual position and trajectory generated setpoint position. The motion
 *  can be interlocked if the difference exceeds a certain tolerance for a this
 *  number of cycles, see command setAxisMonPosLagTol() for more information.
 *  \n
 *  A big position lag error during a motion could be generated by bad tuning or
 *  mechanical issues.\n
 *
 *  Note: The position lag functionality needs to be enabled with the command
 *  setAxisMonEnableLagMon().\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Position lag time (cycles).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set position lag time (cycles) to 10 for axis 7.\n
 * "Cfg.SetAxisMonPosLagTime(7,10)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonPosLagTime(int axisIndex, int value);

/** \breif Enable position lag monitoring.\n
 *
  *  The position lag monitoring functionality monitors the difference between
 *  the actual position and trajectory generated setpoint position. The motion
 *  can be interlocked if the difference exceeds a certain tolerance for a certain
 *  number of cycles, see command setAxisMonPosLagTime() and setAxisMonPosLagTol()
 *  for more information.\n
 *  A big position lag error during a motion could be generated by bad tuning or
 *  mechanical issues.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Enable monitoring.\n.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Disable position lag monitoring for axis 2.\n
 * "Cfg.SetAxisMonEnableLagMon(2,0)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonEnableLagMon(int axisIndex, int value);

/** \breif Set maximum allowed velocity.\n
 *
 * The motion will be interlocked if the actual velocity or the velocity
 * setpoint exceeds this value.\n
 *
 *  Note: The maximum velocity monitoring needs to be enabled with the command
 *  setAxisMonEnableMaxVel().\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Maximum velocity.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set maximum velocoity for axis 3 to 20.\n
 * "Cfg.SetAxisMonMaxVel(3,20)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonMaxVel(int axisIndex, double value);

/** \breif Enable maximum velocity monitoring (over speed).\n
 *
 * The motion will be interlocked if the actual velocity or the velocity
 * setpoint exceeds the limit value set by the command setAxisMonMaxVel().\n
 *
  * \param[in] axisIndex  Axis index.\n
 * \param[in] value Enable monitoring.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable over speed monitoring for axis 3.\n
 * "Cfg.SetAxisMonEnableMaxVel(3,1)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonEnableMaxVel(int axisIndex, int value);

/** \breif Set velocity monitoring interlock delay for drive.\n
 *
 * The over speed interlock for the drive can be delayed in
 * order ensure that encoder noise reulsting in a high velocity will not
 * interlock motion. If the overspeed interlock is still active after this
 * amount of cycles the drive will be interlocked and the amplifier will
 * be turned off.\n
 *
 *  Note: The maximum velocity monitoring needs to be enabled with the command
 *  setAxisMonEnableMaxVel().\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Delay cycles.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set drive over speed interlock delay to 10 cycles for axis 4.\n
 * "Cfg.SetAxisMonMaxVelDriveILDelay(4,10)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonMaxVelDriveILDelay(int axisIndex, int value);

/** \breif Set velocity monitoring interlock delay for trajectory.\n
 *
 * The over speed interlock for the trajectory generator can be delayed in
 * order ensure that encoder noise reulsting in a high velocity will not
 * interlock motion. If the overspeed interlock is still active after this
 * amount of cycles the trajectory generator will be interlocked  and velocity
 * will be ramped down to stand still.\n
 *
 *  Note: The maximum velocity monitoring needs to be enabled with the command
 *  setAxisMonEnableMaxVel().\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Delay cycles.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set trajectory generator over speed interlock delay to 10 cycles
 * for axis 4.\n
 * "Cfg.SetAxisMonMaxVelTrajILDelay(4,10)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonMaxVelTrajILDelay(int axisIndex, int value);

/** \breif Set sequence timeout time in seconds.\n
 *
 * The motion sequences (mainly related to homing) can be aborted if not
 * finalized within a certain time. This is to prevent that the motion
 * control hangs in one sequence. This could happen if the reference switch
 * for some reason is not encountered.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Timeout time.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set sequence timeout value for axis 2 to 1 minute.\n
 * "Cfg.SetAxisSeqTimeout(2,60)" //Command string to cmd_EAT.c.\n
 */
int setAxisSeqTimeout(int axisIndex, int value);

/** \breif Enable controller output high limit monitoring.\n
 *
 *  The controller output high limit monitoring functionality monitors
 *  the controller output. The motion can be interlocked if the controller
 *  output exceeds a certain tolerance, see command setAxisMonCntrlOutHL()
 *  for more information.\n
 *  A high controller output during a motion could be generated by bad tuning or
 *  mechanical issues.\n
 *
 *  \note WARNING: If an error is detected, the enable (power) will be removed
 *  from the axis. Please always ensure that the power less state is the
 *  safe state.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Enable monitoring.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Disable controller output high limit monitoring for axis 2.\n
 * "Cfg.SetAxisMonEnableCntrlOutHLMon(2,0)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonEnableCntrlOutHLMon(int axisIndex, int value);

/** \breif Set monitoring controller output high limit.\n
 *
 * Set maximum allowed controller output for an axis.\n
 *  A high controller output during a motion could be generated by bad tuning
 *  or mechanical issues, see command setAxisMonEnableCntrlOutHLMon()for more
 *  information.\n
 *
 *  \note WARNING: If an error is detected, the enable (power) will be removed
 *  from the axis. Please always ensure that the power less state is the
 *  safe state.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Maximum allowed controller output.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set maximum allowed output to 2000 for for axis 3.\n
 * "Cfg.SetAxisMonCntrlOutHL(3,2000)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonCntrlOutHL(int axisIndex, double value);

/** \breif Enable monitoring of velocity difference.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Enable monitoring.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable monitoring for axis 2.\n
 * "Cfg.SetAxisMonEnableVelocityDiff(2,1)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonEnableVelocityDiff(int axisIndex, int value);

/** \breif Set trajectory interlock filter time in cycles for velocity
 * difference monitoring.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Time in cycles.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set filter time to 100 cycles for axis 2.\n
 * "Cfg.SetAxisMonVelDiffTrajILDelay(2,100)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonVelDiffTrajILDelay(int axisIndex, int value);

/** \breif Set drive interlock filter time in cycles for velocity
 * difference monitoring.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Time in cycles.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set filter time to 500 cycles for axis 2.\n
 * "Cfg.SetAxisMonVelDiffDriveILDelay(2,500)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonVelDiffDriveILDelay(int axisIndex, int value);

/** \breif Set maximum allowed difference between setpoint speed and
 * actual speed±n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value velocity difference.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set maximum difference 0.5 for axis 2.\n
 * "Cfg.SetAxisMonVelDiffTol(2,0.5)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonVelDiffTol(int axisIndex, double value);

/** \breif Enable motion axis interlock from EtherCAT entry.\n
 *
 *Enable
 * The motion can be interlocked based on an EtherCAT entry. See command
 * linkEcEntryToAxisMon() for more information.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Enable external interlock.\n
 *
 * \note WARNING: If an error is detected, the enable (power) will be removed
 *  from the axis. Please always ensure that the power less state is the
 *  safe state.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable external interlock for axis 7.\n
 * "Cfg.SetAxisMonEnableExtHWInterlock(7,1)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonEnableExternalInterlock(int axisIndex, int value);

/** \breif Set polarity of motion axis interlock from EtherCAT entry.\n
 *
 *
 * The motion can be interlocked based on an EtherCAT entry. See command
 * linkEcEntryToAxisMon() for more information.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Polarity external interlock.\n
 *                0 = NC (High is OK) Default.\n
 *                1 = NO (Low is OK).\n
 *
 * \note WARNING: If an error is detected, the enable (power) will be removed
 *  from the axis. Please always ensure that the power less state is the
 *  safe state.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set external interlock to NC for axis 7.\n
 * "Cfg.SetAxisMonExtHWInterlockPolarity(7,0)" //Command string to cmd_EAT.c.\n
 */
int setAxisMonExtHWInterlockPolarity(int axisIndex, int value);

/** \breif Enable commands from other axis.\n
 *
 * An axis can receive commands from an other axis command expression.
 * However, the axis must be allow to receive these commands, see
 * command setAxisTransformCommandExpr() for more information.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Enable.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable command from other axis for axis 3.\n
 * "SetAxisEnableCommandsFromOtherAxis(3,1)" //Command string to cmd_EAT.c.\n
 */
int setAxisEnableCommandsFromOtherAxis(int axisIndex, int value);

/** \breif Enable command transformation expression.\n
 *
 * The command transformation expression for an axis can be enabled/disabled.
 * see command setAxisTransformCommandExpr() for more information.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] value Enable.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Disable command transformation expression axis 5.\n
 * "SetAxisEnableCommandsTransform(5,0)" //Command string to cmd_EAT.c.\n
 */
int setAxisEnableCommandsTransform(int axisIndex, int value);

/** \breif Set axis command transformation expression.\n
 *
 * The axis transformation expression is used for enabling and executing of
 * axes based on mathematical expressions. This is useful when synchronizing
 * axes i.e. a slave axis could be enabled and executed at the same time as
 * the master axis.\n
 *
 * Example: Enable of axis 2 is related to the enable command of axis 1 and 5.
 * The execute command is related to an expression including the
 * execute command for axis 2 and 7.\n
 * "en2:=en1 or en5; ex1:=ex2 + ex7;#".\n
 *   enY = enable command for axis Y.\n
 *   exY = execute command for axis Y.\n
 *
 * \param[in] axisIndex  Axis index.\n
 * \param[in] expr Command expression.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set command transformation expression for axes 5 to
 * en2:=en1 or en5# ex1:=ex2 + ex7#.\n
 * "SetAxisTransformCommandExpr(5)=en2:=en1 or en5# ex1:=ex2 + ex7#"
 * //Command string to cmd_EAT.c.\n
 */
int setAxisTransformCommandExpr(int axisIndex,char *expr);

/** \breif Sets application mode
 *
 * Before entering runtime mode a validation of both hardware and motion
 * objects will be executed. See command validateConfig().\n
 *
 * \param[in] mode    Application mode to set.\n
 *   mode = 0: Configuration mode.\n
 *   mode = 1: Runtime mode.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set the application mode to runtime.\n
 * "Cfg.SetAppMode(1)" //Command string to cmd_EAT.c
 */
int setAppMode(int mode);

/** \breif Creates an axis object at index axisIndex.
 *
 * \param[in] axisIndex Index of axis to address.\n
 * \param[in] type Type of axis.\n
 *   type = 1 : Normal axis (drive, encoder, monitor,pid-controller,trajectory).\n
 *   type = 2 : Virtual axis (encoder, monitor, trajectory).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create a virtual axis at axisIndex 1.\n
 *  "Cfg.CreateAxis(1,2)" //Command string to cmd_EAT.c\n
 */
int createAxis(int axisIndex, int type);

/** \breif Validates the current configuration)"
 *
 * \return 0  if the current configuration is valid for runtime otherwise an
 * error code.\n
 *
 * \note Example: Validate configuration.\n
 * "Cfg.ValidateConfig()" //Command string to cmd_EAT.c\n
 */
int validateConfig();

/** \breif Links an EtherCAT entry to the encoder object of the axis at axisIndex.
   *
   *
   *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
   *    slaveBusPosition = -1: Used to address the simulation slave. Only two
   *                         entries are configured, "ZERO" with default
   *                         value 0 and "ONE" with default value 1.\n
   *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves\n
   *  \param[in] entryIdString String for addressing purpose (see command
   *                      "Cfg.EcAddEntryComplete() for more information").\n
   *  \param[in] axisIndex Index of axis to link to.\n
   *  \param[in] encoderEntryIndex Index of encoder objects entry list.\n
   *    encoderEntryIndex = 0: Actual position (input).\n
   *  \param[in] entryBitIndex Bit index of EtherCAT entry to use.\n
   *    entryBitIndex = -1: All bits of the entry will be used.\n
   *    entryBitIndex = 0..64: Only the selected bit will be used.\n
   *
   *  \return 0 if success or otherwise an error code.\n
   *
   *  \note Example: Link an EtherCAT entry configured as "POSITION_ACT" in slave 1
   *  as actual position for the encoder of axis 5.\n
   *  "Cfg.LinkEcEntryToAxisEncoder(1,POSITION_ACT,5,0,-1)" //Command string
   *  to cmd_EAT.c\n
   */
int linkEcEntryToAxisEnc(int slaveBusPosition,char *entryIdString,int axisIndex,int encoderEntryIndex,int entryBitIndex);

/** \breif Links an EtherCAT entry to the drive object of the axis at axisIndex.
   *
   *
   *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
   *    slaveBusPosition = -1: Used to address the simulation slave. Only two
   *                         entries are configured, "ZERO" with default
   *                         value 0 and "ONE" with default value 1.\n
   *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves\n
   *  \param[in] entryIdString String for addressing purpose (see command
   *                      "Cfg.EcAddEntryComplete() for more information").\n
   *  \param[in] axisIndex Index of axis to link to.\n
   *  \param[in] driveEntryIndex Index of drive objects entry list.\n
   *    driveEntryIndex = 0: Amplifier enable (output).\n
   *    driveEntryIndex = 1: Velocity setpoint (output).\n
   *    driveEntryIndex = 2: Amplifier enabled (input).\n
   *    driveEntryIndex = 3: Brake (output).\n
   *    driveEntryIndex = 4: Reduce torque (output).\n
   *  \param[in] entryBitIndex Bit index of EtherCAT entry to use.\n
   *    entryBitIndex = -1: All bits of the entry will be used.\n
   *    entryBitIndex = 0..64: Only the selected bit will be used.\n
   *
   * \return 0 if success or otherwise an error code.\n
   *
   *  \note Example 1: Link an EtherCAT entry configured as "VELOCITY_SET" in slave 3
   *  as velocity setpoint entry for the drive object of axis 5.\n
   *   "Cfg.LinkEcEntryToAxisDrive(3,VELOCITY_SET,5,1,-1)" //Command string to cmd_EAT.c\n
   *
   *  Example 2: Link bit 0 of an EtherCAT entry configured as "STM_CONTROL" in
   *  slave 3 as enable amplifier output entry for the drive object of axis 5.\n
   *   "Cfg.LinkEcEntryToAxisDrive(3,STM_CONTROL,5,0,0)" //Command string to cmd_EAT.c\n
   *
   *  Example 3: If a drive have no feedback for enable, a simulation slave and
   *  entry can be used. The simulation EtherCAT slave can be addressed with a
   *  slaveBusIndex of -1.
   *  The simulation slave contains two entries, "ZERO" with default value zero
   *  and "ONE" with default value set to 1.\n
   *   "Cfg.LinkEcEntryToAxisDrive(-1,ONE,5,0,0)" //Command string to cmd_EAT.c\n
   */
int linkEcEntryToAxisDrv(int slaveBusPosition,char *entryIdString,int axisIndex,int driveEntryIndex,int entryBitIndex);

/** \breif Links an EtherCAT entry to the monitor object of the axis at axisIndex\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = -1: Used to address the simulation slave. Only two
 *                           entries are configured, "ZERO" with default
 *                           value 0 and "ONE" with default value 1.\n
 *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param[in] entryIdString String for addressing purpose (see command
 *                      "Cfg.EcAddEntryComplete() for more information").\n
 *  \param[in] axisIndex Index of axis to link to.\n
 *  \param[in] monitorEntryIndex Index of monitor objects entry list.\n
 *    monitorEntryIndex = 0: Limit switch backward direction.\n
 *    monitorEntryIndex = 1: Limit switch forward direction.\n
 *    monitorEntryIndex = 2: Reference switch (homing).\n
 *    monitorEntryIndex = 3: External interlock input (optional).\n
 *  \param[in] entryBitIndex Bit index of EtherCAT entry to use.\n
 *    entryBitIndex = -1: All bits of the entry will be used.\n
 *    entryBitIndex = 0..64: Only the selected bit will be used.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 *  \note Example 1: Link an EtherCAT entry configured as "INPUT_0" in slave 1 as
 *  forward limit switch entry for the monitor object of axis 5.\n
 *   "Cfg.LinkEcEntryToAxisMonitor(1,"INPUT_0",5,1,0)" //Command string to cmd_EAT.c\n
 *
 *  Example 2: If an axis is not equipped with limit switches the entries for
 *  limit switches needs to be linked to the simulation entries. The
 *  simulation slave contains two entries, "ZERO" with default value
 *  zero and "ONE" with default value set to 1.\n
 *   "Cfg.LinkEcEntryToAxisEncoder(-1,ONE,5,1,0)" //Command string to cmd_EAT.c\n
 */
int linkEcEntryToAxisMon(int slaveBusPosition,char *entryIdString,int axisIndex,int monitorEntryIndex,int entryBitIndex);

/** \breif Create an event object.
 *
 * An event can be trigger actions based on hardware related events (changes
 * of values on the EtherCAT bus (entries)).\n
 *
 * The events needs to be configured with atleast a subset of the below commands:\n
 * 1. linkEcEntryToEvent()
 * 2. setEventType()
 * 3. setEventSampleTime()
 * 4. setEventTriggerEdge()
 * 5. setEventEnableArmSequence()
 *
 * Currently two kinds of objects can be linked to an event:\n
 * 1. Data recorder object (records data at a triggered event), see
 * command createRecorder().\n
 * 2. Command list object (executes a series of commands at a triggered
 * event), see command  createCommandList().\n
 *
 * \param[in] index Index of event to create.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create a event object at index 1.\n
 *  "Cfg.CreateEvent(1)" //Command string to cmd_EAT.c\n
 */
int createEvent(int index);

/** \breif Links an EtherCAT entry to an event object. \n
 *
 *  \param[in] eventIndex Index of event object to link to.\n
 *  \param[in] eventEntryIndex Index of event objects entry list.\n
 *    eventEntryIndex = 0: Event trigger data input.\n
 *    eventEntryIndex = 1: Arm output (to re-arm latching hardware).\n
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = -1: Used to address the simulation slave. Only two
 *                           entries are configured, "ZERO" with default
 *                           value 0 and "ONE" with default value 1.\n
 *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param[in] entryIdString String for addressing purpose (see command
 *                      "Cfg.EcAddEntryComplete() for more information").\n
 *  \param[in] entryBitIndex Bit index of EtherCAT entry to use.\n
 *    entryBitIndex = -1: All bits of the entry will be used.\n
 *    entryBitIndex = 0..64: Only the selected bit will be used.\n
 *
 *
 * \return 0 if success or otherwise an error code.\n
 *
 *  \note Example 1: Link bit 0 of an EtherCAT entry configured as "INPUT_0"
 *  in slave 1 as "event trigger data" for event object 7.\n
 *  "Cfg.LinkEcEntryToEvent(7,0,1,"INPUT_0",0)" //Command string to cmd_EAT.c\n
 *
 *   \todo This function have not consistent parameter order with the other
 *    link functions as "linkEcEntryToAxisMon".\n
 */
int linkEcEntryToEvent(int indexEvent,int eventEntryIndex,int slaveBusPosition,char *entryIDString,int bitIndex);

/** \breif Set event type.\n
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[in] type Type of event.\n
 *   type = 0: Sample rate triggered (see setEventSampleTime()).\n
 *   type = 1: Trigger on data source edge (see setEventTriggerEdge()).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set event type to edge triggered for event 5.\n
 *  "Cfg.SetEventType(5,1)" //Command string to cmd_EAT.c\n
 */
int setEventType(int indexEvent,int type);

/** \breif Set event sampling time (cycle counts).\n
 *
 * Sets the sample rate at which events will be triggered.
 *
 * Note: This parameter is only valid for sample rate triggered event type (type = 0),
 * see setEventType().
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[in] type Type of event.\n
 *   type = 0: Sample rate triggered (see setEventSampleTime()).\n
 *   type = 1: Trigger on data source edge (see setEventTriggerEdge()).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set event sample time (cycle counts) to 10 for event 5.\n
 *  "Cfg.SetEventSampleTime(5,10)" //Command string to cmd_EAT.c\n
 */
int setEventSampleTime(int indexEvent,int sampleTime);

/** \breif Set event trigger edge.\n
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[in] triggerEdge Type of event.\n
 *   triggerEdge = 0: Trigger event on positive edge.\n
 *   triggerEdge = 1: Trigger event on negative edge.\n
 *   triggerEdge = 2: Trigger event on on change.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create a event object at index 1.\n
 *  "Cfg.CreateEvent(1)" //Command string to cmd_EAT.c\n
 */
int setEventTriggerEdge(int indexEvent,int triggerEdge);

/** \breif Enable event.\n
 *
 * Event evaluation and triggering is only active when the enable bit is
 * high.\n
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[in] enable Enable.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Start evaluation of events for event object 4 .\n
 *  "Cfg.SetEventEnable(4,1)" //Command string to cmd_EAT.c\n
 */
int setEventEnable(int indexEvent,int enable);

/** \breif Get event enabled.\n
 *
 * Event evaluation and triggering is only active when the enable bit is
 * high.\n
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[out] enabled Enabled.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get Enabled for event 2 .\n
 *  "GetEventEnabled(2)" //Command string to cmd_EAT.c\n
 */
int getEventEnabled(int indexEvent,int *enabled);

/** \breif Enable arm sequence.\n
 *
 * Some hardware require that the input card is re-armed after each value have
 * been collected (typically latched I/O). The arm sequence will, if enabled,
 * write a 0 followed by a 1 the following cycle to the arm output entry, see
 * command linkEcEntryToEvent().\n
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[in] enable Enable arm sequence.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable arm sequence for event 3 .\n
 *  "Cfg.SetEventEnableArmSequence(3,1)" //Command string to cmd_EAT.c\n
 */
int setEventEnableArmSequence(int indexEvent,int enable);

/** \breif Enable diagnostic printouts from event object.\n
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[in] enable Enable diagnostic printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable diagnostic printouts for event 3 .\n
 *  "Cfg.SetEventEnablePrintouts(3,1)" //Command string to cmd_EAT.c\n
 */
int setEventEnablePrintouts(int indexEvent,int enable);

/** \breif Force trigger event.\n
 *
 * Any subscriber functions to an event will be executed.\n
 *
 * \param[in] indexEvent Index of event to address.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Force trigger event 1.\n
 *  "Cfg.TriggerEvent(1)" //Command string to cmd_EAT.c\n
 */
int triggerEvent(int indexEvent);

/** \breif Arm event.\n
 *
 * Manually execute arm sequence, see command setEventEnableArmSequence().\n
 *
 * \param[in] indexEvent Index of event to address.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Run arm sequence for event 7.\n
 *  "Cfg.ArmEvent(7)" //Command string to cmd_EAT.c\n
 */
int armEvent(int indexEvent);

/** \breif Create a data storage object.
 *
 * The data storage object is a data buffer.
 *
 * Currently only the recorder object utilizes the data storage object.
 * However, it's foreseen that it will also be useful to store trajectory array
 * data in\n.
 *
 * \todo A method to copy the data to an EPICS waveform needs to be
 * implemented.\n
 *
 * \param[in] index Index of data storage object to create.\n
 * \param[in] elements Size of data buffer.\n
 * \param[in] bufferType Data buffer type.\n
 *  bufferType = 0: LIFO buffer.\n
 *  bufferType = 1: Ring buffer.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create a LIFO data storage object with 1000 elements at
 * index 1.\n
 *  "Cfg.CreateStorage(1,1000,0)" //Command string to cmd_EAT.c\n
 */
int createDataStorage(int index, int elements, int bufferType);

/** \breif Clear data storage buffer.
 *
 *  Erases all data within a data storage object.
 *
 * \param[in] index Index of data storage object to clear.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Clear data storage object with index 4.\n
 *  "Cfg.ClearStorage(4)" //Command string to cmd_EAT.c\n
 */
int clearStorage(int indexStorage);

/** \breif Get current index of data in storage buffer.\n
 *
 * \param[in] index Index of data storage object to clear.\n
 * \param[out] index Current data element index.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get current data element index for data storage object
 * with index 4.\n
 *  "GetStorageDataIndex(4)" //Command string to cmd_EAT.c\n
 */
int getStorageDataIndex(int indexStorage,int *index);

/** \breif Enable diagnostic printouts from data storage object.\n
 *
 * \param[in] indexStroage Index of data storage object to address.\n
 * \param[in] enable Enable diagnostic printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable diagnostic printouts for data storage object 3 .\n
 *  "Cfg.SetStorageEnablePrintouts(3,1)" //Command string to cmd_EAT.c\n
 */
int setStorageEnablePrintouts(int indexStorage,int enable);

/** \breif Print contents of buffer.\n
 *
 * \param[in] indexStroage Index of data storage object to address.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Print contents of data storage object 3 .\n
 *  "Cfg.PrintDataStorage(3)" //Command string to cmd_EAT.c\n
 */
int printStorageBuffer(int indexStorage);

/** \breif Reads contents of storage buffer.\n
 *
 * \param[in] indexStorage Index of data storage object to address.\n
 * \param[out] data Pointer to data.\n
 * \param[out] size Number of data elements.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Read contents of data storage object 3 .\n
 *  "ReadDataStorage(3)" //Command string to cmd_EAT.c\n
 */
int readStorageBuffer(int indexStorage, double **data, int* size);

/** \breif Writes data to storage buffer.\n
 *
 * \param[in] indexStorage Index of data storage object to address.\n
 * \param[in] data Pointer to data.\n
 * \param[in] size Number of data elements to write.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Write contents of data storage object 3 .\n
 *  "WriteDataStorage(3)=0,0,0,0,0,0...." //Command string to cmd_EAT.c\n
 */
int writeStorageBuffer(int indexStorage, double *data, int size);

/** \breif Appends data to the end of storage buffer.\n
 *
 * \param[in] indexStorage Index of data storage object to address.\n
 * \param[in] data Pointer to data.\n
 * \param[in] size Number of data elements to write.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Append data to data storage object 3 .\n
 *  "AppendDataStorage(3)=0,0,0,0,0,0....." //Command string to cmd_EAT.c\n
 */
int appendStorageBuffer(int indexStorage, double *data, int size);

/** \breif Set current data index of storage buffer.\n
 *
 * This function can be used to set teh current index of a data
 * storage buffer. A subsequent append of data will start at this
 * position index in the buffer array.
 *
 * \param[in] indexStorage Index of data storage object to address.\n
 * \param[in] position Position index of data.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set current index of data storage 0 to 10.\n
 *  "Cfg.SetDataStorageCurrentDataIndex(0,10)" //Command string to cmd_EAT.c\n
 */
int setDataStorageCurrentDataIndex(int indexStorage,int position);

/** \breif Create recorder object.
 *
 * The recorder object stores data, from an EtherCAT entry, in a data storage
 * object. The recording of a data point can be triggered by an event or by
 * the command triggerRecorder().\n
 *
 * An EtherCAT data entry needs to be linked to the object with the command
 * linkEcEntryToRecorder().\n
 * The data recorder object needs a data storage object to store data in. The
 * data storage object needs to be linked to the recorder object with the
 * command linkStorageToRecorder().\n
 *
 * \param[in] index Index of recorder object to create.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create a recorder object at index 1.\n
 * "Cfg.CreateRecorder(1)" //Command string to cmd_EAT.c\n
 */
int createRecorder(int indexRecorder);

/** \breif Link storage object to recorder object.
 *
 * \param[in] indexStorage Index of storage object.\n
 * \param[in] indexRecorder Index of recorder object.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Link storage object 5 to recorder object 3.\n
 * "Cfg.LinkStorageToRecorder(5,3)" //Command string to cmd_EAT.c\n
 */
int linkStorageToRecorder(int indexStorage,int indexRecorder);

/** \breif Links an EtherCAT entry to a recorder object. \n
 *
 *  \param[in] indexRecorder Index of recorder object to link to.\n
 *  \param[in] recorderEntryIndex Index of recorder objects entry list.\n
 *    recorderEntryIndex = 0: data input.\n
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = -1: Used to address the simulation slave. Only two
 *                           entries are configured, "ZERO" with default
 *                           value 0 and "ONE" with default value 1.\n
 *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param[in] entryIdString String for addressing purpose (see command
 *                      "Cfg.EcAddEntryComplete() for more information").\n
 *  \param[in] entryBitIndex Bit index of EtherCAT entry to use.\n
 *    entryBitIndex = -1: All bits of the entry will be used.\n
 *    entryBitIndex = 0..64: Only the selected bit will be used.\n
 *
 *
 * \return 0 if success or otherwise an error code.\n
 *
 *  \note Example 1: Link bit 0 of an EtherCAT entry configured as "INPUT_0"
 *  in slave 1 as data for recorder object 7.\n
 *  "Cfg.LinkEcEntryToRecorder(7,0,1,"INPUT_0",0)" //Command string to cmd_EAT.c\n
 *
 * \todo This function have not consistent parameter order with the other
 *  link functions as "linkEcEntryToAxisMon".\n
 */
int linkEcEntryToRecorder(int indexRecorder,int recorderEntryIndex,int slaveBusPosition,char *entryIDString,int bitIndex);

/** \breif Links an axis data source to a recorder object. \n
 *
 *  \param[in] indexRecorder Index of recorder object to link to.\n
 *  \param[in] axisIndex Index of axis to get data from.\n
 *  \param[in] dataToStore data to record from axis object.\n
 *    dataToStore = 0 : No data choosen.\n
 *    dataToStore = 1 : Position Setpoint (from trajectory generator).\n
 *    dataToStore = 2 : Position Actual (scaled).\n
 *    dataToStore = 3 : Position Error.\n
 *    dataToStore = 4 : Position Target.\n
 *    dataToStore = 5 : Controller Error.\n
 *    dataToStore = 6 : Controller Output.\n
 *    dataToStore = 7 : Velocity Setpoint.\n
 *    dataToStore = 8 : Velocity Actual.\n
 *    dataToStore = 9 : Velocity Setpoint Raw.\n
 *    dataToStore = 10: Velocity Setpoint Feed Forward Raw.\n
 *    dataToStore = 11: Error Code.\n
 *    dataToStore = 12: Enable (command).\n
 *    dataToStore = 13: Enabled (status).\n
 *    dataToStore = 14: Execute (command).\n
 *    dataToStore = 15: Busy (status).\n
 *    dataToStore = 16: Sequence state.\n
 *    dataToStore = 17: At Target (status).\n
 *    dataToStore = 18: Interlock type.\n
 *    dataToStore = 19: Limit Switch forward.\n
 *    dataToStore = 20: Limit Switch backward.\n
 *    dataToStore = 21: Home switch.\n
 *    dataToStore = 22: Command.\n
 *    dataToStore = 23: Command Data (cmdData).\n
 *    dataToStore = 24: Trajectory setpoint source.\n
 *          0  = Internal Source.\n
 *          >0 = External Source.\n
 *    dataToStore = 25: Encoder setpoint source.\n
 *          0  = Internal Source.\n
 *          >0 = External Source.\n
 *    dataToStore = 26: Axis Id.\n
 *    dataToStore = 27: Cycle counter.\n
 *    dataToStore = 28: Position Raw.\n
 *    dataToStore = 29: Encoder homed.\n

 * \return 0 if success or otherwise an error code.\n
 *
 *  \note Example 1: Link Actual position of axis 4 to recorder 1:.\n
 *  in slave 1 as data for recorder object 7.\n
 *  "Cfg.linkAxisDataToRecorder(1,4,2)" //Command string to cmd_EAT.c\n
 */
int linkAxisDataToRecorder(int indexRecorder,int axisIndex,int dataToStore);

/** \breif Enable recorder.\n
 *
 * Recording of data is only active when the enable bit is high.\n
 *
 * \param[in] indexRecorder Index of recorder to address.\n
 * \param[in] enable Enable.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Start data recording of recorder object 4.\n
 *  "Cfg.SetRecorderEnable(4,1)" //Command string to cmd_EAT.c\n
 */
int setRecorderEnable(int indexRecorder,int enable);

/** \breif Get recorder enabled.\n
 *
 * Recording of data is only active when the enable bit is high.\n
 *
 * \param[in] indexRecorder Index of recorder to address.\n
 * \param[out] enabled Enabled.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get recorder object 4 enabled.\n
 *  "GetRecorderEnabled(4)" //Command string to cmd_EAT.c\n
 */
int getRecorderEnabled(int indexRecorder,int *enabled);

/** \breif Enable diagnostic printouts from recorder object.\n
 *
 * \param[in] indexRecorder Index of recorder object to address.\n
 * \param[in] enable Enable diagnostic printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable diagnostic printouts for recorder object 3 .\n
 *  "Cfg.SetRecorderEnablePrintouts(3,1)" //Command string to cmd_EAT.c\n
 */
int setRecorderEnablePrintouts(int indexRecorder,int enable);

/** \breif Link recorder object to event object.\n
 *
 * \param[in] indexRecorder Index of recorder object to address.\n
 * \param[in] indexEvent Index of event object to address.\n
 * \param[in] consumerIndex Event consumer index (one event can have a
 * list with consumers).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Link recorder object 3 with event object 4, event consumer index 0.\n
 *  "Cfg.LinkRecorderToEvent(3,4,0)" //Command string to cmd_EAT.c\n
 */
int linkRecorderToEvent(int indexRecorder,int indexEvent, int consumerIndex);

/** \breif Force trigger recorder.\n
 *
 * The recorder object will store data in the linked data storage object.\n
 *
 * \param[in] indexRecorder Index of recorder to address.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Force trigger recorder 1.\n
 *  "Cfg.TriggerRecorder(1)" //Command string to cmd_EAT.c\n
 */
int triggerRecorder(int indexRecorder);

/** \breif Create a command list object.
 *
 * The command list object consists of a list of commands that can be executed.
 * Currently any command can be added to the command list. The execution of the
 * command list can be triggered by an event or by the command
 * triggerCommandList().\n
 *
 * \note If the command list is triggered by a hardware event. The command
 * list will be executed from the realtime thread. In this case special
 * consideration needs to be taken so that the jitter of realtime thread is not
 * worsen. Normal motion commands are normally no issue to use but commands
 * that creates new axis or takes long time to return should be avoided.\n
 *
 * \param[in] index Index of command list object to create.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create a command list object at index 1.\n
 * "Cfg.CreateCommandList(1)" //Command string to cmd_EAT.c\n
 */
int createCommandList(int indexCommandList);

/** \breif Link command list object to event object.\n
 *
 * \param[in] indexCommandList Index of command list object to address.\n
 * \param[in] indexEvent Index of event object to address.\n
 * \param[in] consumerIndex Event consumer index (one event can have a
 * list with consumers).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Link coommand list object 3 with event object 4, event consumer index 0.\n
 *  "Cfg.LinkCommandListToEvent(3,4,0)" //Command string to cmd_EAT.c\n
 */
int linkCommandListToEvent(int indexCommandList,int indexEvent, int consumerIndex);

/** \breif Enable command list.\n
 *
 * Command list will only be executed when the enable bit is high.\n
 *
 * \param[in] indexCommandList Index of command list to address.\n
 * \param[in] enable Enable.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: "Enable" command list execution for object 4.\n
 *  "Cfg.SetCommandListEnable(4,1)" //Command string to cmd_EAT.c\n
 */
int setCommandListEnable(int indexCommandList,int enable);

/** \breif Enable diagnostic printouts from command list object.\n
 *
 * \param[in] indexCommandList Index of command list object to address.\n
 * \param[in] enable Enable diagnostic printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable diagnostic printouts for command list object 3 .\n
 *  "Cfg.SetCommandListEnablePrintouts(3,1)" //Command string to cmd_EAT.c\n
 */
int setCommandListEnablePrintouts(int indexCommandList,int enable);

/** \breif Add command to command list.\n
 *
 * \param[in] indexCommandList Index of command list object to address.\n
 * \param[in] expr Command string.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Add "Main.M1.bExecute=1" to command list 1.\n
 *  "Cfg.AddCommandToCommandList(1)=Main.M1.bExecute=1" //Command string to
 *  cmd_EAT.c\n
 */
int addCommandListCommand(int indexCommandList,char *expr);

/** \breif Force trigger command list.\n
 *
 * All commands in the command list object will be executed.\n
 *
 * \param[in] indexCommandList Index of command list to address.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Force trigger command list 1.\n
 *  "Cfg.TriggerCommandList(1)" //Command string to cmd_EAT.c\n
 */
int triggerCommandList(int indexCommandList);

/** \breif Selects EtherCAT master to use.\n
 *
 *  \param[in] masterIndex EtherCAT master index.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Select /dev/EtherCAT0.\n
 *  "Cfg.EcSetMaster(0)" //Command string to cmd_EAT.c\n
 */
int ecSetMaster(int masterIndex);

/** \breif  Retry configuring slaves for an selected EtherCAT master.\n
 *
 * Via this method, the application can tell the master to bring all slaves to
 * OP state. In general, this is not necessary, because it is automatically
 * done by the master. But with special slaves, that can be reconfigured by
 * the vendor during runtime, it can be useful.
 *
 * \note  masterIndex is not used. Currently only one master is supported and
 * this master will be reset. The masterIndex is for future use.
 *
 *  \param[in] masterIndex EtherCAT master index.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Reset master. Master index is NOT currently supported.\n
 *  "Cfg.EcResetMaster(10)" //Command string to cmd_EAT.c\n
 */
int ecResetMaster(int masterIndex);

/** \breif Adds an EtherCAT slave to the hardware configuration.\n
 *
 * Each added slave will be assigned an additional index which will be zero for
 * the first successfully added slave and then incremented for each successful
 * call to "Cfg.EcAddSlave()".\n
 *
 * NOTE: if an complete entry needs to be configured the command
 * "Cfg.EcAddEntryComplete()" should be used. This command will add slave,
 * sync. manger, pdo and entry if needed.\n
 *
 *  \param alias Alias of slave. Set to zero to disable.\n
 *  \param slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = -1: Used to address the simulation slave. Only two
 *                           entries are configured, "ZERO" with default
 *                           value 0 and "ONE" with default value 1.\n
 *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param vendorId Identification value for slave vendor.\n
 *    vendorId = 0x2: Beckhoff.\n
 *    vendorId = 0x48554B: Kendrion Kuhnke Automation GmbH.\n
 *  \param productCode Product identification code.\n
 *    productCode=0x13ed3052: EL5101 incremental encoder input.\n
 *
 * \note All configuration data can be found in the documentaion of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Add a EL5101 Beckhoff slave at slave position 1.\n
 *  "Cfg.EcAddSlave(0,1,0x2,0x13ed3052)" //Command string to cmd_EAT.c\n
 */
int ecAddSlave(uint16_t alias, uint16_t position, uint32_t vendorId,uint32_t productCode);

///obsolete command. Use ecAddEntryComplete() command instead.\n
int ecAddSyncManager(int slaveIndex,int direction,uint8_t syncMangerIndex);

///obsolete command. Use ecAddEntryComplete() command instead.\n
int ecAddPdo(int slaveIndex,int syncManager,uint16_t pdoIndex);

//int ecAddEntry(int slaveIndex,int nSyncManager,int nPdo,uint16_t nEntryIndex,uint8_t  nEntrySubIndex, uint8_t nBits);

/** \breif Adds an EtherCAT slave to the hardware configuration.\n
 *
 * Each added slave will be assigned an additional index which will be zero for
 * the first successfully added slave and then incremented for each successful
 * call to "Cfg.EcAddSlave()".\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = -1: Used to address the simulation slave. Only two
 *                           entries are configured, "ZERO" with default
 *                           value 0 and "ONE" with default value 1.\n
 *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param[in] vendorId Identification value for slave vendor.\n
 *    vendorId = 0x2: Beckhoff.\n
 *    vendorId = 0x48554B: Kendrion Kuhnke Automation GmbH.\n
 *  \param[in] productCode Product identification code.\n
 *    productCode=0x13ed3052: EL5101 incremental encoder input.\n
 *  \param[in] direction Data transfer direction..\n
 *    direction  = 1:  Output (from master).\n
 *    direction  = 2:  Input (to master).\n
 *  \param[in] syncMangerIndex Index of sync manager.
 *  \param[in] pdoIndex Index of process data object. Needs to be entered
 *                           in hex format.\n
 *  \param[in] entryIndex Index of process data object entry. Needs to be
 *                           entered in hex format.\n
 *  \param[in] entrySubIndex Index of process data object sub entry.
 *                           Needs to be entered in hex format.\n
 *  \param[in] bits Bit count.\n
 *  \param[in] entryIDString Identification string used for addressing the
 *                           entry.\n
 *
 * \note All configuration data can be found in the documentation of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Add an EtherCAT entry for the actual position of an EL5101
 * incremental encoder card.\n
 * "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,2,3,0x1a03,0x6010,0x10,16,POSITION)"
 * //Command string to cmd_EAT.c\n
 *
 * \note Example: Add an EtherCAT entry for the velocity setpoint of an EL7037
 * stepper drive card.\n
 * "Cfg.EcAddEntryComplete(7,0x2,0x1b7d3052,1,2,0x1604,0x7010,0x21,16,
 * VELOCITY_SETPOINT)" //Command string to cmd_EAT.c\n
 */
int ecAddEntryComplete(
    uint16_t slaveBusPosition,
    uint32_t vendorId,
    uint32_t productCode,
    int direction,
    uint8_t syncMangerIndex,
    uint16_t pdoIndex,
    uint16_t entryIndex,
    uint8_t  entrySubIndex,
    uint8_t bits,
    char *entryIDString);

/** \breif Adds a memory map object to access data directly from EtherCAT
 *   domain.\n
 *
 *  The start of the memory map is addressed by a previously configured
 *  EtherCAT entry and a size.
 *
 *  \param[in] startEntryBusPosition Position of the EtherCAT slave on the bus
 *                                   where the start entry is configured.\n
 *    startEntryBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param[in] startEntryIDString I Identification string of the start EtherCAT
 *   *                           entry.\n
 *  \param[in] byteSize Size of memory map objects (size to access).\n
 *  \param[in] direction Data transfer direction..\n
 *    direction  = 1:  Output (from master).\n
 *    direction  = 2:  Input (to master).\n
 *  \param[in] entryIDString Identification string used for addressing the
 *                           memory map object.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Add an EtherCAT input memory map of size 200 bytes starting at
 * entry "AI1" on slave 10. Name the memory map WAVEFORM. Type
 * argument is excluded in\n
 * "Cfg.EcAddMemMap(10,AI1,200,2,WAVEFORM)" //Command string to cmd_EAT.c\n
 */
int ecAddMemMap(
    uint16_t startEntryBusPosition,
    char *startEntryIDString,
    size_t byteSize,
    int direction,
    char *memMapIDString
    );

/** \breif Configure slave DC clock.\n
 *
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n*
 *  \param[in] assignActivate A* Each added slave will be assigned an additional index which will be zero for
 * the first successfully added slave and then incremented for each successful
 * call to "Cfg.EcAddSlave()".\n
 ssign active word. This information can be found
 *                           in the ESI slave description file. Needs to be
 *                           entered in hex format.\n
 *  \param[in] sync0Cycle Cycle time in ns.\n
 *  \param[in] sync0Shift Phase shift in ns.\n
 *  \param[in] sync1Cycle Cycle time in ns.\n
 *  \param[in] sync1Shift Not used.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Configure the DC clock of an EL5101 incremental encoder
 * input card: slaveBusPosition=1, assignActivate=0x320, sync0Cycle=1000000
 * (1kHz), sync0Shift=10, sync1Cycle=1000000 (1kHz).\n
 * "Cfg.EcSlaveConfigDC(1,0x320,1000000,10,1000000,0)" //Command string to
 * cmd_EAT.c\n
 */
int ecSlaveConfigDC(
    int slaveBusPosition,
    uint16_t assignActivate, /**< AssignActivate word. */
    uint32_t sync0Cycle, /**< SYNC0 cycle time [ns]. */
    int32_t sync0Shift, /**< SYNC0 shift time [ns]. */
    uint32_t sync1Cycle, /**< SYNC1 cycle time [ns]. */
    int32_t sync1Shift /**< SYNC1 shift time [ns]. */);

/** \breif Select EtherCAT reference clock.\n
 *
 *  \param[in] masterIndex Index of master, see command ecSetMaster().\n
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.
 *                              Note: the slave needs to be equipped
 *                              with a DC.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n*
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Select slave 3 as reference clock for master 0.\n
 * "Cfg.EcSelectReferenceDC(0,3)" //Command string to cmd_EAT.c\n
 */
int ecSelectReferenceDC(int masterIndex,int slaveBusPosition);

/** \breif Adds a Service Data Object for writing.
 *
 * Adds a Service Data Object for writing to the sdo registers of EtherCAT
 * slaves. An sdo object will be added to the hardware configuration.
 * The writing will occur when the hardware configuration is applied.\n
 *
 * \note This command can only be used in configuration mode.\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n
 *  \param[in] sdoIndex Index of service data object. Needs to be
 *                           entered in hex format.\n
 *  \param[in] sdoSubIndex Sub index of service data object .
 *                           Needs to be entered in hex format.\n
 *  \param[in] value Value to write.\n
 *  \param[in] byteSize Byte count to write.\n
 *
 * \note All configuration data can be found in the documentation of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Write 1A (1000mA) to maximum current of the EL7037 stepper drive card
 * on slave position 2.\n
 * "Cfg.EcAddSdo(2,0x8010,0x1,1000,2)" //Command string to cmd_EAT.c\n
 */
int ecAddSdo(uint16_t slaveBusPosition,uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t value, int byteSize); //size in bytes

/** \breif Write to a Service Data Object.
 *
 * Writing will occur directly when the command is issued.\n
 *
 * \note This command can only be used in configuration mode.\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n
 *  \param[in] sdoIndex Index of service data object. Needs to be
 *                           entered in hex format.\n
 *  \param[in] sdoSubIndex Sub index of service data object .
 *                           Needs to be entered in hex format.\n
 *  \param[in] value Value to write.\n
 *  \param[in] byteSize Byte count to write.\n
 *
 * \note All configuration data can be found in the documentation of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Write 1A (1000mA) to maximum current of the EL7037 stepper drive card
 * on slave position 2.\n
 * "Cfg.EcWriteSdo(2,0x8010,0x1,1000,2)" //Command string to cmd_EAT.c\n
 */
int ecWriteSdo(uint16_t slavePposition,uint16_t sdoIndex,uint8_t sdoSubIndex,uint32_t value,int byteSize);

/** \breif Read a Service Data Object.
 *
 * Writing will occur directly when the command is issued.\n
 *
 * \note This command can only be used in configuration mode.\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n
 *  \param[in] sdoIndex Index of service data object. Needs to be
 *                           entered in hex format.\n
 *  \param[in] sdoSubIndex Sub index of service data object .
 *                           Needs to be entered in hex format.\n
 *  \param[in] byteSize Byte count to read.\n
 *
 * \note All configuration data can be found in the documentation of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return read result.\n
 *
 * \note Example: Read maximum current setting of the EL7037 stepper drive card
 * on slave position 2.\n
 * "Cfg.EcReadSdo(2,0x8010,0x1,2)" //Command string to cmd_EAT.c\n
 */
uint32_t ecReadSdo(uint16_t slavePosition,uint16_t sdoIndex,uint8_t sdoSubIndex,int byteSize);

/** \breif Configure Slave watch dog.\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.
 *                              Note: the slave needs to be equipped
 *                              with a DC.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n
 *  \param[in] watchdogDivider  Number of 40 ns intervals. Used as a
 *                              base unit for all slave watchdogs. If set
 *                              to zero, the value is not written, so the
 *                              default is used.\n
 *  \param[in] watchdogIntervals Number of base intervals for process
 *                              data watchdog. If set to zero, the value
 *                              is not written, so the default is used.\n
 *
 * \note Example: Set watchdog times to 100,100 for slave at busposition 1.\n
 * "Cfg.EcSlaveConfigWatchDog(1,100,100)" //Command string to cmd_EAT.c\n
 *
 * \return 0 if success or otherwise an error code.\n
 */
int ecSlaveConfigWatchDog(int slaveBusPosition,int watchdogDivider,int watchdogIntervals);

/** \breif Apply hardware configuration to master.\n
 *
 * This command needs to be executed before entering runtime.\n
 *
 * \note This command can only be used in configuration mode.\n
 *
 *  \param[in] masterIndex Index of master, see command ecSetMaster().\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Apply hardware configuration to master 0 (/dev/EtherCAT0).\n
 * "Cfg.EcApplyConfig(0)" //Command string to cmd_EAT.c\n
 */
int ecApplyConfig(int masterIndex);

/** \breif Writes a value to an EtherCAT entry.\n
  *
  *  \param[in] slaveIndex Index of order of added slave (not bus position),
  *                        see command EcAddSlave()).\n
  *  \param[in] entryIndex Index of order of added entry (within slave).
  *  \param[in] value Value to be written.\n
  *
  * \note the slaveIndex and entryIndex can be read with the commands
  * readEcSlaveIndex() and readEcEntryIndexIDString().\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Write a 0 to the 2:nd added entry (entryIndex=1) in the 4:th
  * added slave (slaveIndex=3).\n
  *  "Cfg.WriteEcEntry(3,1,0)" //Command string to cmd_EAT.c\n
  */
int writeEcEntry(int slaveIndex,int entryIndex,uint64_t value);

/** \breif Writes a value to an EtherCAT entry addressed by slaveBusPosition
 * and entryIdString.\n
  *
  *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
  *    slaveBusPosition = -1: Used to address the simulation slave. Only two
  *                           entries are configured, "ZERO" with default
  *                           value 0 and "ONE" with default value 1.\n
  *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
  *  \param[in] entryIdString String for addressing purpose (see command
  *                      "Cfg.EcAddEntryComplete() for more information").\n
  *  \param[in] value Value to be written.\n
  *
  * Note: This command should not be used when realtime performance is needed
  * (also see "WriteECEntry()" command).\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Write a 1 to a digital output configured as "OUTPUT_0" on slave 1\n
  *  "Cfg.WriteEcEntryIDString(1,OUTPUT_1,1)" //Command string to cmd_EAT.c\n
  */
int writeEcEntryIDString(int slaveBusPosition,char *entryIdString,uint64_t value);

/** \breif Read a value from an EtherCAT entry.\n
  *
  *  \param[in] slaveIndex Index of order of added slave (not bus position),
  *                        see command EcAddSlave()).\n
  *  \param[in] entryIndex Index of order of added entry (within slave).
  *  \param[out] value Read value (result).\n
  *
  * \note the slaveIndex and entryIndex can be read with the commands
  * readEcSlaveIndex() and readEcEntryIndexIDString().\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Read the 2:nd added entry (entryIndex=1) in the 4:th added
  * slave (slaveIndex=3).\n
  * "Cfg.ReadEcEntry(3,1)" //Command string to cmd_EAT.c\n
  */
int readEcEntry(int slaveIndex,int entryIndex,uint64_t *value);

/** \breif Read a value from an EtherCAT entry addressed by slaveBusPosition
 *   and entryIdString.\n
  *
  *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
  *    slaveBusPosition = -1: Used to address the simulation slave. Only two
  *                           entries are configured, "ZERO" with default
  *                           value 0 and "ONE" with default value 1.\n
  *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
  *  \param[in] entryIdString String for addressing purpose (see command
  *                      "Cfg.EcAddEntryComplete() for more information").\n
  *  \param[out] value Read value (result).\n
  *
  * Note: This command should not be used when realtime performance is needed
  * (also see "WriteECEntry()" command).\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Read a digital input configured as "INPUT_0" on slave 1\n
  *  "Cfg.ReadEcEntryIDString(1,INPUT_0)" //Command string to cmd_EAT.c\n
  */
int readEcEntryIDString(int slavePosition,char *entryIDString,uint64_t *value);

/** \breif Read the object Index of an entry addressed by slaveBusPosition
 *   and entryIdString.\n
  *
  * The first entry added in a slave will receive index 0, then for each added
  * entry, its index will be incremented by one. This index will be returned by
  * this function. This index is needed to address a certain slave with the
  * commands readEcEntry() and writeEcEntry() (for use in realtime, since
  * these functions address the slave and entry arrays directly via indices).\n
  *
  * \note This is not the same as the entryIndex in EcAddEntryComplete().\n
  *
  * \todo Change confusing naming of this function. entryIndex and index of entry is
  * easily confused.\n
  *
  *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
  *    slaveBusPosition = -1: Used to address the simulation slave. Only two
  *                           entries are configured, "ZERO" with default
  *                           value 0 and "ONE" with default value 1.\n
  *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
  *  \param[in] entryIdString String for addressing purpose (see command
  *                      "Cfg.EcAddEntryComplete() for more information").\n
  *  \param[out] value Entry index (the order is was added).\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Read a the index of an entry configured as "INPUT_0" on slave 1\n
  *  "Cfg.ReadEcEntryIndexIDString(1,INPUT_0)" //Command string to cmd_EAT.c\n
  */
int readEcEntryIndexIDString(int slavePosition,char *entryIDString,int *value);

/** \breif Read the object Index of an slave addressed by slaveBusPosition.\n
  *
  * The first slave added in will receive index 0, then for each added
  * slave, its index will be incremented by one. This index will be returned by
  * this function. This index is needed to address a certain slave with the
  * commands readEcEntry() and writeEcEntry() (for use in realtime, since
  * these functions address the slave and entry arrays directly via indices).\n
  *
  * \note slaveIndex and slaveBusPosition is not the same.\n
  *
  *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
  *    slaveBusPosition = -1: Used to address the simulation slave. Only two
  *                           entries are configured, "ZERO" with default
  *                           value 0 and "ONE" with default value 1.\n
  *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Read the slave index of the slave with bus position 5.\n
  *  "Cfg.ReadEcSlaveIndex(5)" //Command string to cmd_EAT.c\n
  */
int readEcSlaveIndex(int slavePosition,int *value);

/* \breif Links ethercat entry to asyn parameter
 */
int linkEcEntryToAsynParameter(void* asynPortObject, const char *entryIDString, int asynParType,int skipCycles);

/* \breif Links ethercat memory map to asyn parameter
 */
int linkEcMemMapToAsynParameter(void* asynPortObject, const char *memMapIDString, int asynParType,int skipCycles);

/* \breif reads mem map object
 */
int readEcMemMap(const char *memMapIDString,uint8_t *data,size_t bytesToRead, size_t *bytesRead);

/* \breif Set update in realtime bit for an entry
 *
 * If set to zero the entry will not be updated during realtime operation.\n
 * Useful when accessing data with memory maps instead covering many entries
 * like oversampling arrays (its the unnecessary to update each entry in
 * array).\n
 */
int ecSetEntryUpdateInRealtime(
    uint16_t slavePosition,
    char *entryIDString,
    int updateInRealtime
    );

/** \breif Enable EtherCAT bus diagnostics.\n
  *
  * Diagnostics are made at three different levels:\n
  * 1. Slave level.\n
  * 2. Domain level.\n
  * 3. Master level.\n
  *
  * All three levels of diagnostics are enabled or disabled with this command.
  * If the diagnostics are enabled the motion axes are interlocked if any of
  * the above levels report issues. If the diagnostics are disabled the motion
  * axes are not interlocked even if there's alarms on the EtherCAT bus.\n
  *
  * \param[in] enable Enable diagnostics.\n
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Enable EtherCAT diagnostics.\n
  *  "Cfg.EcSetDiagnostics(1)" //Command string to cmd_EAT.c\n
  */
int ecSetDiagnostics(int enable);

/** \breif Set allowed bus cycles in row of none complete domain
 * data transfer.\n
 *
 * Allows a certain number of bus cycles of non-complete domain data
 * transfer before alarming.\n
 *
 *  \note Normally the application is correct configured all domain data
 *  transfers should be complete. This command should therefor only be
 *  used for testing purpose when the final configuration is not yet made.\n
  *
  * \param[in] cycles Number of cycles.\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Set the allowd bus cycles of non complete domain data to
  * 10.\n
  *  "Cfg.EcSetDomainFailedCyclesLimit(10)" //Command string to cmd_EAT.c\n
  */
int ecSetDomainFailedCyclesLimit(int cycles);

/** \breif Reset error on all EtherCat objects.\n
 *
 * Resets error on the following object types:\n
 * 1. ecmcEc().\n
 * 2. ecmcEcSlave().\n
 * 3. ecmcEcSyncManager().\n
 * 4. ecmcEcPdo().\n
 * 5. ecmcEcSDO().\n
 * 6. ecmcEcEntry().\n
 *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Reset EtherCAT errors.\n
  *  "Cfg.EcResetError()" //Command string to cmd_EAT.c\n
  */
int ecResetError();

/** \breif Enable diagnostic printouts from EtherCAT objects.\n
 *
 * Enables/Disables diagnostic printouts from:
 * 1. ecmcEc().\n
 * 2. ecmcEcSlave().\n
 * 3. ecmcEcSyncManager().\n
 * 4. ecmcEcPdo().\n
 * 5. ecmcEcSDO().\n
 * 6. ecmcEcEntry().\n
 *
 * \param[in] enable Enable diagnostic printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable EtherCAT related diagnostic printouts.\n
 *  "Cfg.EcEnablePrintouts(1)" //Command string to cmd_EAT.c\n
 */
int ecEnablePrintouts(int value);

/** \breif Set axis index for detailed motion diagnostics.\n
 *
 * \param[in] axisIndex Index of axis.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Choose detailed motion diagnostics for axis 3.\n
 *  "Cfg.SetDiagAxisIndex(3)" //Command string to cmd_EAT.c\n
 */
int setDiagAxisIndex(int axisIndex);

/** \breif Set axis frequency of detailed motion diagnostics printouts.\n
 *
 * \param[in] freq Printout frequency.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set printout frequency to 10.\n
 *  "Cfg.SetDiagAxisFreq(10)" //Command string to cmd_EAT.c\n
 */
int setDiagAxisFreq(int freq);

/** \breif Enable detailed motion diagnostics printouts.\n
 *
 * The detailed motion diagnostics prints valuable motion related information
 * for one axis.\n
 * See setDiagAxisIndex() and setDiagAxisFreq() for more information.\n
 *
 * \param[in] enable Enable printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable printout of detailed motion diagnostics.\n
 *  "Cfg.SetDiagAxisEnable(1)" //Command string to cmd_EAT.c\n
 */
int setDiagAxisEnable(int enable);

/** \breif Enable printouts of timing information related to the realtime
 * thread.\n
 *
 * \param[in] enable Enable printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable printout time diagnostics.\n
 *  "Cfg.SetEnableTimeDiag(1)" //Command string to cmd_EAT.c\n
 */
int setEnableTimeDiag(int value);

/** \breif Enable printouts of which functions in hw_motor.cpp are being
 * called.\n
 *
 * \param[in] enable Enable printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable function call diagnostics.\n
 *  "Cfg.SetEnableFuncCallDiag(1)" //Command string to cmd_EAT.c\n
 */
int setEnableFunctionCallDiag(int value);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
