#ifndef ECMC_PLC_H_
#define ECMC_PLC_H_

#include "../com/ecmcOctetIF.h"        // Log Macros
#include "../main/ecmcErrorsList.h"
#include "../main/ecmcDefinitions.h"

#define CHECK_PLCS_RETURN_IF_ERROR()                                          \
{                                                                             \
  if (!plcs) {                                                                \
    return ERROR_MAIN_PLCS_NULL;                                              \
  }                                                                           \
}     

# ifdef __cplusplus
extern "C" {
# endif  // ifdef __cplusplus

/** \breif Create a PLC object
 * called.\n
 *
 * \param[in] index PLC index number
 * \param[in] cycleTime CycleTime in seconds
 * \param[in] axisPLC If PLC for axis
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create PLC at index 0 (executing every 10th cycle).\n
 *  "Cfg.CreatePLC(0,9)" //Command string to ecmcCmdParser.c\n
 *
 * \note Example: Create PLC at index 0 (executing every cycle).\n
 *  "Cfg.CreatePLC(0)" //Command string to ecmcCmdParser.c\n
 */
int createPLC(int index,
              double cycleTime,
              int axisPLC);

/** \breif Delete PLC.\n
 *
 * Delete PLC object
 * ethercat entries are available.
 *
 * \param[in] index  PLC index.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Delete PLC 4\n
 * "Cfg.DeletePLC(4);" //Command string to ecmcCmdParser.c.\n
 */
int deletePLC(int index);

/** \breif Set PLC expression.\n
 *
 * The PLC expression is used for PLC functionalities.\n
 *
 * See appendPLCExpr() for more information.\n
 *
 * \param[in] index  PLC index.\n
 * \param[in] expr Expression.\n
 *
 * \return 0 if success or otherwise an error code.\n
 
 * \note Pipe sign "|" should be used instead of ";" This because asynOctet 
 * interface uses ";" as command delimiter\n.
 * 
 * \note Example: Set expression for PLC 5 to
 * "ec0.s1.OUTPIN_1.0=ec0.s2.INPIN_3.0\n
 * "Cfg.SetPLCExpr(5)=ec0.s1.OUTPIN_1.0=ec0.s2.INPIN_3.0|" //Command string to ecmcCmdParser.c.\n
 */
int setPLCExpr(int   index,
               char *expr);

/** \breif Append Line to PLC expression.\n
 *
 * Append one line of PLC Code. Code syntax is defined on exprtk website.\n
 * Variables that can be used are described below.\n
 *
 * \param[in] index  PLC index.\n
 * \param[in] expr Expression.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 *  Accessible variables in code:
 *   1.  static.<varname>             Static variable. Initiated to 0. (rw)\n
 *                                    Access only in the PLC where defined.\n
 *                                    Will keep value between execution\n
 *                                    loops.\n
 *   2.  global.<varname>             Global variable. Initiated to 0. (rw)\n
 *                                    Access from all PLCs.\n
 *                                    Will keep value between execution\n
 *                                    loops.\n
 *   3.  var <varname>                Local variable (exprtk syntax)   (rw)\n
 *                                    Will NOT keep value between\n
 *                                    execution loops.\n
 *
 *  EtherCAT data:
 *   1.  ec<ecid>.s<sid>.<alias>.<bitid>  ethetcat data                (rw)\n
 *                                    ecid:  ethercat master index\n
 *                                    sid:   ethercat slave bus position\n
 *                                    alias: entry name as defined in\n
 *                                           "Cfg.EcAddEntryComplete()\n
 *                                    bitid: bit index (optional)\n
 *
 *  Motion variables:
 *   1.  ax<id>.id                    axis id                          (ro)\n
 *   2.  ax<id>.reset                 reset axis error                 (rw)\n
 *   3.  ax<id>.counter               execution counter                (ro)\n
 *   4.  ax<id>.error                 error                            (ro)\n
 *   5.  ax<id>.enc.actpos            actual position                  (ro)\n
 *   6.  ax<id>.enc.actvel            actual velocity                  (ro)\n
 *   7.  ax<id>.enc.rawpos            actual raw position              (ro)\n
 *   8.  ax<id>.enc.source            internal source or expressions   (ro)\n
 *                                    source = 0: internal encoder\n
 *                                    source > 0: actual pos from expr\n
 *   9.  ax<id>.enc.homed             encoder homed                    (rw)\n
 *   10. ax<id>.enc.homepos           homing position                  (rw)\n
 *   11. ax<id>.traj.setpos           curent trajectory setpoint       (ro)\n
 *   12. ax<id>.traj.targetpos        target position                  (rw)\n
 *   13. ax<id>.traj.targetvel        target velocity setpoint         (rw)\n
 *   14. ax<id>.traj.targetacc        target acceleration setpoint     (rw)\n
 *   15. ax<id>.traj.targetdec        target deceleration setpoint     (rw)\n
 *   16. ax<id>.traj.setvel           current velocity setpoint        (ro)\n
 *   17. ax<id>.traj.setvelffraw      feed forward raw velocity        (ro)\n
 *   18. ax<id>.traj.command          command                          (rw)\n
 *                                    command=1: move velocity\n
 *                                    command=2: move rel. pos\n
 *                                    command=3: move abs. pos\n
 *                                    command=10: homing\n
 *   19. ax<id>.traj.cmddata          cmddat. Homing procedure\n
 *                                    only valid if ax<id>.traj.command=10\n
 *                                    cmddata=1 : ref low limit\n
 *                                    cmddata=2 : ref high limit\n
 *                                    cmddata=3 : ref home sensor\n
 *                                                (via low limit)\n
 *                                    cmddata=4 : ref home sensor\n
 *                                                (via high limit)\n
 *                                    cmddata=5 : ref center of home sensor\n
 *                                                (via low limit)\n
 *                                    cmddata=6 : ref center of home sensor\n
 *                                                (via high limit)\n
 *                                    cmddata=15 : direct homing\n
  *                                   cmddata=21 : ref partly abs. encoder\n
 *                                                 (via low limit).\n
 *                                                 ref at abs bits.\n
 *                                                 over/under-flow.\n.
 *                                    cmddata=22 : ref partly abs. encoder\n
 *                                                 (via high limit).\n
 *                                                 ref at abs bits.\n
 *                                                 over/under-flow.\n.
 *   20. ax<id>.traj.source           internal source or expressions   (ro)\n
 *                                    source = 0: internal traj\n
 *                                    source > 0: setpoints from expr\n
 *   21. ax<id>.traj.execute          execute motion command           (rw)\n
 *   22. ax<id>.traj.busy             axis busy                        (ro)\n
 *   23. ax<id>.traj.dir              axis setpoint direction          (ro)\n
 *                                    ax<id>.traj.dir>0: forward\n
 *                                    ax<id>.traj.dir<0: backward\n
 *                                    ax<id>.traj.dir=0: standstill\n
 *   24. ax<id>.cntrl.error           actual controller error          (ro)\n
 *   25. ax<id>.cntrl.poserror        actual position error            (ro)\n
 *   26. ax<id>.cntrl.output          actual controller output         (ro)\n
 *   27. ax<id>.drv.setvelraw         actual raw velocity setpoint     (ro)\n
 *   28. ax<id>.drv.enable            enable drive command             (rw)\n
 *   29. ax<id>.drv.enabled           drive enabled                    (ro)\n
 *   30. ax<id>.seq.state             sequence state (homing)          (ro)\n
 *   31. ax<id>.mon.ilock             motion interlock  (both dir)     (rw)\n
 *                                    ax<id>.mon.ilock=1: motion allowed\n
 *                                    ax<id>.mon.ilock=0: motion not allowed\n
 *   32. ax<id>.mon.ilockbwd          motion interlock bwd dir         (rw)\n
 *                                    ax<id>.mon.ilockbwd=1: motion allowed\n
 *                                    ax<id>.mon.ilockbwd=0: motion not allowed\n
 *   33. ax<id>.mon.ilockfwd          motion interlock fwd dir         (rw)\n
 *                                    ax<id>.mon.ilockfwd=1: motion allowed\n
 *                                    ax<id>.mon.ilockfwd=0: motion not allowedn
 *   34. ax<id>.mon.attarget          axis at taget                    (ro)\n
 *   35. ax<id>.mon.lowlim            low limit switch                 (ro)\n
 *   36. ax<id>.mon.highlim           high limit switch                (ro)\n
 *   37. ax<id>.mon.homesensor        home sensor                      (ro)\n
 *   38. ax<id>.mon.lowsoftlim        low soft limit                   (rw)\n
 *   39. ax<id>.mon.highsoftlim       high soft limit                  (rw)\n
 *   40. ax<id>.mon.lowsoftlimenable  low soft limit enable            (rw)\n
 *   41. ax<id>.mon.highsoftlimenable high soft limit enable           (rw)\n
 *   42. ax<id>.blockcom              Enables/disables "set" commands   (rw)\n
 *                                    via command parser (ascii commands)\n
 *                                    Statuses can still be read.\n
 *                                    Exceptions ("set"-commands) that
 *                                    will work:
 *                                    - "StopMotion(axid)"
 *                                    - "Cfg.SetAxisBlockCom(axid,block)"
 *
 *  PLC variables:
 *   1.  plc<id>.enable               plc enable                       (rw)\n
 *                                    (end exe with "plc<id>.enable:=0#"\n
 *                                    Could be usefull for startup\n
 *                                    sequences)\n
 *   2.  plc<id>.error                plc error                        (rw)\n
 *                                    Will be forwarded to user as\n
 *                                    controller error.\n
 *   3.  plc<id>.scantime             plc sample time in seconds       (ro)\n
 *   4.  plc<id>.firstscan            true during first plc scan only  (ro)\n
 *                                    usefull for initiations of variables\n
 *
 *  Data Storage variables:
 *   1.  ds<id>.size                  Set/get size of data storage     (rw)\n
 *                                    Set will clear the data storage\n
 *   2.  ds<id>.append                Add new data at end              (rw)\n
 *                                    Current position index will be
 *                                    increased
 *   3.  ds<id>.data                  Set/get data ar current position (rw)\n
 *   4.  ds<id>.index                 Set/get current position index   (rw)\n
 *   5.  ds<id>.error                 Data storage class error         (ro)\n
 *   6.  ds<id>.clear                 Data buffer clear (set to zero)  (ro)\n
 *   7.  ds<id>.full                  True if data storage is full     (ro)\n
 *
 *  Function Lib: EtherCAT
 *   1. retvalue = ec_set_bit(
 *                           <value>,         : Value to change
 *                           <bitindex>       : Bit index
 *                           );
 *      Sets bit at bitindex position of value. Returns the new value.\n
 *
 *   2. retvalue = ec_clr_bit(
 *                           <value>,         : Value to change
 *                           <bitindex>       : Bit index
 *                           );
 *      Clears bit at bitindex position of value. Returns the new value.\n
 *
 *   3. retvalue = ec_flp_bit(
 *                           <value>,         : Value to change
 *                           <bitindex>       : Bit index
 *                           );
 *      Flips bit at bitindex position of value. Returns the new value.\n
 *
 *   4. retvalue = ec_chk_bit(
 *                           <value>,         : Value to change
 *                           <bitindex>       : Bit index
 *                           );
 *      Checks bit at bitindex position of value. Returns the value of bit.\n
 *
 *   5. retvalue=ec_get_err():
 *      Returns error code from last lib call.\n
 *
 *  Function Lib: Motion
 *   1. retvalue = mc_move_abs(
 *                           <axIndex>,       : Axis index\n
 *                           <execute>,       : Trigger\n
 *                           <pos>,           : Target position\n
 *                           <vel>,           : Target velocity\n
 *                           <acc>,           : Acceleration\n
 *                           <dec>            : Deceleration\n
 *                           ):
 *      Absolute motion of axis.\n
 *      Motion is triggerd with a positive edge on <execute> input.\n
 *      returns 0 if success or error code.\n
 *
 *   2. retvalue = mc_move_rel(
 *                           <axIndex>,       : Axis index\n
 *                           <execute>,       : Trigger\n
 *                           <pos>,           : Target position\n
 *                           <vel>,           : Target velocity\n
 *                           <acc>,           : Acceleration\n
 *                           <dec>            : Deceleration\n
 *                           );
 *
 *      Relative motion of axis <axIndex>.\n
 *      Motion is triggerd with a positive edge on <execute> input.\n
 *      returns 0 if success or error code.\n
 *
 *   3. retvalue = mc_move_vel(
 *                           <axIndex>,       : Axis index\n
 *                           <execute>,       : Trigger\n
 *                           <vel>,           : Target velocity\n
 *                           <acc>,           : Acceleration\n
 *                           <dec>            : Deceleration\n
 *                           );
 *      Constant velocity motion of axis <axIndex>.\n
 *      Motion is triggerd with a positive edge on <execute> input.\n
 *      returns 0 if success or error code.\n
 *
 *   4. retvalue = mc_home(
 *                           <axIndex>,       : Axis index\n
 *                           <execute>,       : Trigger\n
 *                           <seqId>,         : Motion sequence\n
 *                           <velTwoardsCam>, : Target Velocity twords cam\n
 *                           <velOffCam>      : Target velocity off cam\n
 *                           );
 *      Perform a homing sequence of axis <axIndex>.\n
 *      Motion is triggerd with a positive edge on <execute> input.\n
 *      returns 0 if success or error code.\n
 *
 *   5. retvalue = mc_halt(
 *                           <axIndex>,       : Axis index\n
 *                           <execute>,       : Trigger\n
 *                           );
 *      Stop motion of axis <axIndex>.\n
 *      Motion is triggerd with a positive edge on <execute> input.\n
 *      returns 0 if success or error code.\n
 *
 *   6. retvalue = mc_power(
 *                           <axIndex>,       : Axis index\n
 *                           <enable>,        : Enable power\n
 *                           );
 *      Enable power of  axis <axIndex>.\n
 *      Motion is triggerd with a positive edge on <execute> input.\n
 *      returns 0 if success or error code.\n
 *
 *   7. retvalue = mc_get_err();
 *      Returns error code for last lib call.\n
 *
 *  Function Lib: Data Storage
 *   1. retvalue = ds_append_data(
 *                           <dsIndex>,       : Data storage index\n
 *                           <data>,          : Data\n
 *                           );
 *      Append data to data storage.\n
 *      returns 0 if success or error code.\n
 *
 *   2. retvalue = ds_clear_data(
 *                           <dsIndex>,       : Data storage index\n
 *                           );
 *      Clear data to data storage.\n
 *      returns 0 if success or error code.\n
 *
 *   3. retvalue = ds_get_data(
 *                           <dsIndex>,       : Data storage index\n
 *                           <bufferIndex>,   : Buffer index\n
 *                           );
 *      Returns data from buffer.\n
 *
 *   4. retvalue = ds_set_data(
 *                           <dsIndex>,       : Data storage index\n
 *                           <bufferIndex>,   : Buffer index\n
 *                           );
 *      Sets data in data storage buffer.\n
 *      returns 0 if success or error code.\n
 *
 *   5. retvalue = ds_get_buff_id(
 *                           <dsIndex>,       : Data storage index\n
 *                           );
 *      Returns current buffer index.\n
 *
 *   6. retvalue = ds_set_buff_id(
 *                           <dsIndex>,       : Data storage index\n
 *                           <bufferIndex>,   : Buffer index\n
 *                           );
 *      Sets current buffer index in data storage buffer.\n
 *      returns 0 if success or error code.\n
 *
 *   7. retvalue = ds_is_full(
 *                           <dsIndex>,       : Data storage index\n
 *                           );
 *      Returns true if buffer is full.\n
 *
 *   8. retvalue = ds_get_size(
 *                           <dsIndex>,       : Data storage index\n
 *                           );
 *      Returns buffer size of data storage.\n
 *
 *   9. retvalue = ds_get_err()
 *      Returns error code for last lib call.\n
 * 
 *   10. retvalue = ds_push_asyn(
 *                           <dsIndex>,       : Data storage index\n
 *                           );
 *      Push data array to epics. (usefull if asyn param sample time\n
 *      is set to -1).\n
 *
 *   11. retvalue = ds_get_avg(
 *                           <dsIndex>,       : Data storage index\n
 *                           );
 *      Returns average of the values in the data storage.\n
 *
 *   12. retvalue = ds_get_min(
 *                           <dsIndex>,       : Data storage index\n
 *                           );
 *      Returns minimum of the values in the data storage.\n
 *
 *   13. retvalue = ds_get_max(
 *                           <dsIndex>,       : Data storage index\n
 *                           );
 *      Returns maximum of the values in the data storage.\n
 *
 * \note Pipe sign "|" should be used instead of ";" This because asynOctet 
 * interface uses ";" as command delimiter\n.
 * 
 * \note Example: Add one line of PLC code to PLC 5
 * "ec0.s1.OUTPIN_1.0=ec0.s2.INPIN_3.0\n
 * "Cfg.AppendPLCExpr(5,ec0.s1.OUTPIN_1.0=ec0.s2.INPIN_3.0|)" //Command string to ecmcCmdParser.c.\n
 *
 * \note Example: Add code by plc file to PLC 5 (prefered solution, the ";" can\n 
 * be used):\n
 * "Cfg.LoadPLCFile(5,<filename with path>)" //Command string to ecmcCmdParser.c.\n
 */
int appendPLCExpr(int   index,
                  char *expr);

/** \breif Load PLC file.\n
 *
 * Load file with PLC code to PLC.\n
 * For syntax look at "appendPLCExpr()".\n
 * PLC file will be compiled and
 * enabled.\n
 *
 * \param[in] index     PLC index.\n
 * \param[in] fileName  File name.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Load plc fil with code to PLC 5\n
 * "Cfg.LoadPLCFile(5,/home/iocuser/dummyPLC.plc)" //Command string to ecmcCmdParser.c.\n
 */
int loadPLCFile(int   index,
                char *fileName);

/** \breif Write to PLC variable.\n
 * \note: Only static variables are supported.\n
 *
 * \param[in] index     PLC index.\n
 * \param[in] varName   Variable name.\n
 * \param[in] value     Value to write.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Write 100.0 to variable static.test in plc 5\n
 * "WritePLCVar(5,static.test,100.0)" //Command string to ecmcCmdParser.c.\n
 */
int writePLCVar(int         index,
                const char *varName,
                double      value);

/** \breif Read PLC variable.\n
 * \note: Only static variables are supported.\n
 *
 * \param[in] index     PLC index.\n
 * \param[in] varName   Variable name.\n
 * \param[out] value    Read value.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Read variable static.test in plc 5\n
 * "ReadPLCVar(5,static.test)" //Command string to ecmcCmdParser.c.\n
 */
int readPLCVar(int         index,
               const char *varName,
               double     *value);

/** \breif Clear PLC expression.\n
 *
 * Clears all plc code of a PLC object.\n
 *
 * \param[in] index  PLC index.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Clear code of PLC 5\n
 * "Cfg.ClearPLCExpr(5)" //Command string to ecmcCmdParser.c.\n
 */
int clearPLCExpr(int index);

/** \breif Compile PLC expression.\n
 *
 * Compiles code of PLC object
 *
 * \param[in] index  PLC index.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Compile code of PLC 5\n
 * "Cfg.CompilePLC(5)" //Command string to ecmcCmdParser.c.\n
 */
int compilePLCExpr(int index);

/** \breif Set enable of PLC.\n
 *
 * Enable a PLC.\n
 *
 * \param[in] index  PLC index.\n
 * \param[in] enable Enable.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set enable for PLC 5\n
 * "Cfg.SetPLCEnable(5,1)" //Command string to ecmcCmdParser.c.\n
 */
int setPLCEnable(int index,
                 int enable);

/** \breif Set enable of PLC.\n
 *
 * Enable a PLC.\n
 *
 * \param[in] index  PLC index.\n
 * \param[out] enabled Enabled.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get enable for PLC 5\n
 * "GetPLCEnable(5)" //Command string to ecmcCmdParser.c.\n
 */
int getPLCEnable(int  index,
                 int *enabled);

# ifdef __cplusplus
}
# endif  // ifdef __cplusplus

#endif  /* ECMC_PLC_H_ */