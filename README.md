# README #

## ECMC Motion Control Module for EPICS ##

ECMC is a motion control and data acquistion module for use with EPICS system.

### Features ###

* Motion Control
    * Motor record support
        * Absolute Positioning
        * Relative Positioning
        * Constant velocity
        * Evaluation of limit and reference switches
        * Homing (several different sequences available)
            * Low limit (refId = 1)
            * High limit (refId = 2)
            * Ref. sensor via low limit (refId = 3)
            * Ref. sensor via high limit (refId = 4)
            * Ref. sensor center via low limit (refId = 5)
            * Ref. sensor center via high limit (refId = 6)
            * Encoder index via low limit (refId = 11)
            * Encoder index via high limit (refId = 12)
            * Set position (refId = 15)
            * Single turn abs encoder (resolver) ref via low limit (refId = 21)
            * Single turn abs encoder (resolver) ref via high limit (refId = 22)        
        * Interlocks
            * Following error
            * Max speed
            * Ethercat I/O
            * Ethercat bus health
    * Advanced features
        * Virtuel and normal axis
        * Modulo movements
        * Syncronization (to axis or any I/O) Handled through expressions
            * Example slaving  "ax2.traj.setpos:=ax1.enc.actpos;"
            * Example sync     "ax2.traj.setpos:=ax1.traj.setpos;"
            * Example phasing  "ax2.traj.setpos:=ax1.traj.setpos+100;"
            * Example phasing  "ax2.traj.setpos:=ax1.traj.setpos+ax3.traj.setpos;"
            * Advanced         "ax2.traj.setpos:=ax1.traj.setpos*sin(ax3.traj.setpos/100)+ec0.s2.VALUE;"
            * Enable amps      "ax2.drv.enable:=ax1.drv.enable;"
            * Interlocks       "ax2.mon.ilockfwd:=ec0.s1.INPUT_1 or ax1.enc.actpos < 100;"
    * Data acquisition
        * Supoort of many different analog and digital slaves
        * Up to 100kHz analog and 1Mhz digital (oversampling slaves)      
    * Control
        * PLC objects can be created with custom sample rate where logic (including motion) can be handled in realtime.
        (see examples in ecmctraining reposotory: https://bitbucket.org/europeanspallationsource/ecmctraining/src/master/startup/ecmcProject_HardwareTest/
        * Same syntax as for syncronization:
            * "ec0.s45.VALUE:=static.gain*77+ax2.traj.setpos/10;"                        
        * PLC varaiables can be accessed from EPICS. 
        * PLC code can be loaded and updated in realtime.
    * Hardware support: 
        * Drives: stepper, servo, pulse direction and analog
        * I/O: Several analog and digital terrminals
        * Other: bus-couplers, system terminals
        * Current list: https://bitbucket.org/europeanspallationsource/ecmctraining/src/master/startup/hardware/

### How do I get set up? ###

Fastest and easiest way to get started is by using the configuration scripts accessible in
ecmctraining repository: https://bitbucket.org/europeanspallationsource/ecmctraining/

ECMC is configured via EPICS-iocsh:

* iocsh commands:
    * Create an ECMC application:
        * ecmcAsynPortDriverConfigure <portName>,<paramTableSize>, <prio>, <disableAutoConnect>, <defaultSampleRateMS>
    * Configure ECMC application. Exit ECMC if returns error: 
        * ecmcConfigOrDie <ecmc ASCII command>
    * Configure ECMC application.
        * ecmcConfig <ecmc ASCII command> 
    * Print availabe asyn parameters and other info:
        * ecmcReport <detail> 
        * asynReport <detail>. Same as ecmcReport but also calls asynReport in other modules.

* ASCII-cmds (see doxygen documentation for more information):
    * Motion configs. Example:
        * ecmcConfigOrDie "Cfg.CreateAxis(<axisid>,1,<type>)"
        * ecmcConfigOrDie "Cfg.SetAxisCntrlKp(<axisid>,<kp>)"    
        * ...
    * Ethercat bus configs. Example:
        * ecmcConfigOrDie "Cfg.EcAddEntryComplete(<slaveid>,<vendorid>,<productid>,<dir>,<smid>,<pdoid>,<entryid>,<entrysubid>,<bits>,<name>)"  
        * ...
    * Ethercat slave configs (SDO,SoE). Example set max motor current for EL7037 to 1500mA:
        * ecmcConfigOrDie "Cfg.EcAddSdo(<slaveid>,0x8010,0x1,1500,2)"
    * PLC configuration:
        * ecmcConfigOrDie "Cfg.CreatePLC(<plcid>,<sample rate ms>)"
        * ecmcConfigOrDie "Cfg.LoadPLCFile(<plcid>,<filename>)"
        * ecmcConfigOrDie "Cfg.AppendPLCExpr(<plcid>)=ax1.enc.homepos:=25#"
        * ...

### Environment ###
ECMC runs best under certain conditions:

* E3 (ESS EPICS Environment)
* RT-patch
* Settings found in https://github.com/icshwi/e3-tools/tree/master/rt_conf

### Needed EPICS modules ###

* Open source ethercat master:      https://etherlab.org/, https://github.com/icshwi/etherlabmaster 
* ExprTK:                           https://github.com/ArashPartow/exprtk,https://github.com/icshwi/e3-exprtk
* E3 (ESS EPICS Environment):       https://github.com/icshwi/e3
* Realtime configuration:           https://github.com/icshwi/e3-tools/tree/master/rt_conf
* Motor:                            https://github.com/icshwi/e3-motor
* EthercatMC (motor record driver): https://github.com/icshwi/e3-ethercatmc
* Asyn driver:                      https://github.com/icshwi/e3-asyn
* Stream Device:                    https://github.com/icshwi/e3-StreamDevice

### Issues/bug report ###

  https://jira.esss.lu.se/projects/ECMC/issues/

### Support ###

* Motor Record:  Torsten Bögershausen, torsten.bogershausen@esss.se
* ECMC:          Anders Sandström, anders.sandstrom@esss.se

## Release Notes ##

### ECMC 6.0.0 ###

Version 6.0.0 is not compeltely backward compatible since some changes 
in command-set have been made.
Please use ecmctrainig version 6.0.0 for configuration

#### Updates ####

##### General #####
* Remove ecmcAsynPortDriverAddParameter iocsh command
  Parameters are added automatically (when ethercat entries, axis objects, or .. are created/defined)
* Redefinition of asyn drvuser string: option1/option2/optionx/<variable name><read or write>
    * Options:
        * T_SMP_MS = <sample time>
            Possbility to define sample rate. If not, the default sample rate is used (defined by ecmcAsynPortDriverConfigure command)
        * TYPE     = <type>
            * asynInt32,
            * asynUInt32Digital,
            * asynFloat64,
            * asynInt8Array,
            * asynInt16Array,
            * asynInt32Array,
            * asynFloat32Array,
            * asynFloat64Array,            
        * CMD      = <cmd>
            * INT64TOFLOAT64:  Casts a INT64 in ecmc to float (only valid for TYPE=asynFloat64)
            * UINT64TOFLOAT64  Casts a unit64 in ecmc to float (only valid for TYPE=asynFloat64)
    * Varaible name (use ecmcReport 2 or asynReport 3 to see availabe parameters):
        * ethercat data: ec<masterid>.s<slaveid>.<name> (name defined by Cfg.EcAddEntryComplete() command)
        * axis:          ax<axisid>.<name>  
        * data storage:  ds<id>.<name>
    * Read or write. Use "?" for read and "=" for write variables.
    * Examples:
        * "T_SMP_MS=100/TYPE=asynInt32/ec0.s1.CH1_VALUE?"
        * "TYPE=asynFloat64/ax1.setpos?"
        * "TYPE=asynFloat64/ax2.actpos?"
        * "TYPE=asynInt32/ec0.s2.STM_CONTROL="
        * "CMD=UINT64TOFLOAT64/T_SMP_MS=1000/TYPE=asynFloat64/ec0.s5.CH1_VALUE?"

* Add asynReport iocsh command:
    * asynReport 2: list all asyn parameters connected to records
    * asynReport 3: list all availabe parameters (that can be connected to records)
      NOTE: Also other asyn modules will display information when asynReport is executed.
      The motor record driver will output alot of information which could lead to that
      it's hard to find the information. Therefore ecmcReport command was added (which
      just print out information from ecmc).

* Add ecmcReport iocsh command:
  Same as asynReport but only for ecmc (will not for instance list asynReport for motor record driver).

* Add ONE, ZERO entries for each slave withn read and write access.
  This to allow more flexibilty when linking values:
     ec<m-id>.s<s-id>.ONE  : 0xFFFFFFFF (a 32 bit register filled with 1)
     ec<m-id>.s<s-id>.ZERO : 0x0 (a 32 bit register filled with 0)    
     example: ec0.s23.ONE can be used.
     NOTE: The "ONE" and "ZERO" entries for slave -1 is still available for backward compatibility
     (ec<m-id>.s-1.ONE/ZERO)

* Add status word for objects as asyn parameters:
    * Ec           :  ec<m-id>.masterstatus
    * EcSlave      :  ec<m-id>.s<s-id>.slavestatus : 
        * bit 0      : online
        * bit 1      : operational
        * bit 2      : alstate.init
        * bit 3      : alstate.preop
        * bit 4      : alstate.safeop
        * bit5       : alstate.op
        * bit 16..32 : entrycounter
    * Axis         :  ax<axis id>.status
    * Data Storage : ds<id>.status

* Add support for modulo movement (both jog/constant velo and positioning):
    * Cfg.SetAxisModRange(<axis id>, <mod range>)":
      <mod range>: Encoder and Trajectory range will be within 0..<mod range>
    * Cfg.SetAxisModType(<axis id>, <mod type>)" For positioning
      <mod type>:   0 = Normal
                    1 = Always Fwd
                    2 = Always Bwd
                    3 = Closets way (minimize travel)
        
* Add time stamp in ECMC layer:
    * Time stamp source can be choosen by setting record TSE-field
        * TSE =  0 EPCIS timestamp
        * TSE = -2 ECMC timestamp, 
* Update sample rates from skip cycles to rate in ms

* Remove command: Cfg.SetAxisGearRatio(). Obsolete, handled through axis plc instaed.

* ecmcConfigOrDie: Add printout of command if fail (error)

* Add asyn parameter that updates when all other parameters have been updated: ecmc.updated of type asynInt32ArrayIn

* Update to new asynPortDriver constructor (to remove warning)

* Add command Cfg.EcSlaveVerify(). Check that a slave at a certain bus-position have the correct vendor-id and product-id.
  This command is added to all hardware snippets in ecmctraining to ensure that the slave is of correct type.
  
* Add range checks for ec-entry writes (based on bit length and signed vs unsigned)

* Add memory lock (memlockall) and allocation of stack memory (good for rt performance)

* Add licence file

* Add basic implemetation of a basic comminsioning GUI (ecmccomgui). Currentlly in different repo:
  based on pyQT, pyEpics

* Add command that verifies a SDO enries value:
  Cfg.EcVerifySdo(uint16_t slave_position,
                  uint16_t sdo_index,
                  uint8_t sdo_subindex,
                  uint32_t verValue,
                  int byteSize)

* Add extra command for create an axis
  Cfg.CreateAxis(int axisIndex,
                 int axisType,  //Optional, defaults to normal axis
                 int drvType)   //optional, defaults to stepper axis 
    
* Remove Command Cfg.SetAxisDrvType(). Use Cfg.CreateAxis(<id>,<type>,<drvType>) instead

* Add command for remote access to iocsh over asynRecord: Cfg.IocshCmd=<cmd>

* Update of ds402 state machine.

* Add command to set velocity low pass filter size for encoder (actpos) and trajetory (setpos):
  Needed if resolution of encoder is low. Since the velocity is used for feed forward purpose. 
  Two cases:
    1. For internal source: 
      Cfg.SetAxisEncVelFilterSize(<axisId>,<filtersize>)"
    2. External velocity  (values from PLC code). 
       Cfg.SetAxisPLCEncVelFilterSize(<axisId>,<filtersize>)" (not used)
       Cfg.SetAxisPLCTrajVelFilterSize(<axisId>,<filtersize>)"
  Renamed commands:
    * Cfg.SetAxisEncExtVelFilterEnable()   -> Cfg.SetAxisPLCEncVelFilterEnable() (not used)
    * Cfg.SetAxisTrajExtVelFilterEnable()  -> Cfg.SetAxisPLCTrajVelFilterEnable()

* Replace axis command transform with axis dedcicated PLC object (executed in sync before axis object), 
  in order to use same syntax as in rest of ECMC.
  Many new variables and functions can now be used in the axis sync PLC (same as "normal" PLC object)
  Currentlly the following indexes apply (not needed info if the Cfg.SetAxisPLC* commands are used):
    1. normal PLCs are in range 0..ECMC_MAX_PLCS-1
    2. Axis PLCs are in range ECMC_MAX_PLCS..(ECMC_MAX_PLCS + ECMC_MAX_AXES - 1) 

    * PLCs can set
        1. the current setpoint of an axis by writing to: ax<id>.traj.setpos. The axis will use this setpoint if trajectory source is set to PLC (formally called external)
        2. the current encoder actual position by writing to: ax<id>.enc.actpos. The axis will use this setpoint if encoder source is set to PLC (formally called external)
      All other PLC variables can be used in the axis plcs (see file plcSyntax.plc in ecmctraining)

    * Remove unused files (ecmcMasterSlave*, ecmcCommandTransform*)

    * Renamed commands
        * Cfg.GetAxisEnableCommandsTransform()     ->  Cfg.GetAxisPLCEnable()          
          Check if Axis PLC is enabled.

        * Cfg.SetAxisEnableCommandsTransform()     ->  Cfg.SetAxisPLCEnable()
          Enable axis PLC (the plc will execute only if enabled

        * Cfg.GetAxisTransformCommandExpr()        ->  Cfg.GetAxisPLCExpr()  
          Read current PLC code for axis PLC.

        * Cfg.SetAxisTransformCommandExpr()        ->  Cfg.SetAxisPLCExpr()   
          Write PLC code for axis PLC.

        * Cfg.GetAxisEnableCommandsFromOtherAxis() ->  Cfg.GetAxisAllowCommandsFromPLC()
          Check if axis allows commands/writes from any PLC.

        * Cfg.SetAxisEnableCommandsFromOtherAxis() ->  Cfg.SetAxisAllowCommandsFromPLC()
          Allow commands (writes) from any PLC object (includig axis PLC)

    * New commands
        * Cfg.AppendAxisPLCExpr(<axis id>, <code string>)
          Append a line of code to a Axis PLC
  
        * Cfg.LoadAxisPLCExpr(<axis id>, <filename>) 
          Load a file with code to a Axis PLC

* Add plc functions for average, min and max of a data storage:
    * ds_get_avg(<ds id>) : Get average (usefull as lowpass filter)
    * ds_get_min(<ds id>) : Get minimum
    * ds_get_max(<ds id>) : Get maximum
  All three functions only use the data that has been written to the storage (not the empty/free space). 
  
* Add interlock variables in PLC : 
    * axis<id>.mon.ilockbwd : allow motion in backward dir (stop if set to 0) 
    * axis<id>.mon.ilockfwd : allow motion in forward dir (stop if set to 0)

### Migration  guide (5.x.x to 6.0.0) ###

Use ecmctraining 6.0.0 for all new projects

Changes:

* init snippet:
    * Added variable to select time stamp source (ECMC_TIME_SOURCE):
        * ECMC_TIME_SOURCE = -2 for time stamp in ECMC (default). 
        * ECMC_TIME_SOURCE = 0 for time stamp in EPICS
    * TODO: Set ECMC_TIME_SOURCE if timestamp source is important

* ecmc_axis snippet:
    * TODO: Modulo support. Add below lines to all axes_* files (general section):
        * epicsEnvSet("ECMC_MOD_RANGE" ,"0") # Modulo range (traj setpoints and encoder values will be in range 0..ECMC_MOD_RANGE)
        * epicsEnvSet("ECMC_MOD_TYPE","0") # For positioning and MOD_RANGE>0: 0 = Normal, 1 = Always Fwd, 2 = Always Bwd, 3 = Closest Distance

    * Cfg.SetAxisDrvType() command is obsolete:
        * "Cfg.CreateAxis(${ECMC_AXIS_NO},1,${ECMC_DRV_TYPE})" is used instead of "Cfg.CreateDefaultAxis(${ECMC_AXIS_NO})" and "Cfg.SetAxisDrvType(${ECMC_AXIS_NO},${ECMC_DRV_TYPE})".
        * TODO: No change needed (use ecmc_axis v6.0.0)    

* ecmc_sync_axis:
    * Command transform object have been replaced with a PLC for each axis. Therefore all commands and syntax related to syncronization have been changed. See above for more info.
    * TODO: Convert sync expressions to new standard. See examples https://bitbucket.org/europeanspallationsource/ecmctraining/src/master/startup/ecmcProject_MCU1021_exampleMasterSlave/axis_2_sync

* ethercat hardware snippets:
    * Have been renamed to allow easier use of macros. All snippest follow the standard ecmc<slave name>. example ecmcEL2004    
    (before ecmcEL2004-digitalOutput)
    * Each slave have a dedicated substitution file and some a dedicated template file.
    * General slave database entries are "loaded" via substitution file
    * Added "Cfg.EcSlaveVerify(0,${ECMC_EC_SLAVE_NUM},${ECMC_EC_VENDOR_ID},${ECMC_EC_PRODUCT_ID})" to ensure that correct slave is on correct bus position. If wrong type of slave an error will be returned and ECMC will die (since ecmcConfigOrDie is used).
    * ecmcAsynPortDriverAddParameter iocsh command have been removed from all "ecmc<slave name>-records" files
      Asyn links are instead automatically linked through asynportdriver::drvusercreate and adressed through the drvInfo string.
      See above in this doc for definition/syntax of drvInfo string in. Use "ecmcReport 3" or "asynReport 3" to see accessible parameters (see also ecmctraining 6.0.0). As long as you use the ecmctraining v6.0.0  snippets and database files this will not be an issue.
    * TODO: Use new hardware snippets.

* Startup files:
    * Sample time of records are defined by ECMC_SAMPLE_RATE_MS (now in milli seconds).
    * TODO: Change ECMC_ASYN_SKIP_CYCLES to ECMC_SAMPLE_RATE_MS.

* OPI: Have been updated to fit new records.
    * TODO: Nothing

## Todo ##
* Remove manual motion mode (not needed.. motors can be run manually directlly from ethercat entries)
* Add PID controller in PLC lib
* Add licence headers to all source files
* Consider change error handling system to exception based (try catch)
* Clean up in axis sub objects:
    * emcmMonitor:  Add verify setpoint function (mostly rename)
    * ecmcTrajectory: Make more independent of other code
    * ecmcTrajectory: Make Spline Version
* Implementation of robot and hexapod kinematics? How todo best possible?
* Make possible to add ethercat hardware on the fly (hard, seems EPICS do not support dynamic load of records)
* Only stop motion when the slaves used by axis are in error state or not reachable.
* Test EtherCAT redundacy
* Move to ecmcconfig instead of ecmctraining
* ......