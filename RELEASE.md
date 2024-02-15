
Release Notes
===

# ECMC v9.0.1_RC4
* Add analog interlock to motion axes (intended for use with temperature sensors):
```
ecmcConfigOrDie "Cfg.LinkEcEntryToObject(<ethercat entry>,"ax<axis id>.mon.analoginterlock")"
ecmcConfigOrDie "Cfg.SetAxisMonEnableAnalogInterlock(<axis id>,<1/0>)"
ecmcConfigOrDie "Cfg.SetAxisMonAnalogInterlockPolarity(<axis id>,<0=low is ok (high bad), 1=high is ok (low bad)>)"
ecmcConfigOrDie "Cfg.SetAxisMonAnalogInterlockRawLimit(<axis id>,<raw value limit for trip>)"        
```
* Allow asyn param names/aliases with dots '.' (mainly used for EcDataItems)
* Ensure no duplication of asyn paarmeter names
* Removed info printouts for epics state and linked parameters
* Add commands for setting controller deadband (defaults to atTargetTol and atTargetTime):
```
    ecmcConfigOrDie "Cfg.SetAxisCntrlDeadband(<axis_id>,<tol>)"
    ecmcConfigOrDie "Cfg.SetAxisCntrlDeadbandTime(<axis_id>,<time>)" 
```
  the function allows to have 0 deadband for control and at the same time have a atTarget tolerance (for motor record)
* Add printout of data rawvalue (8byte) for ecmcGrepParam and ecmcReport.
* Add asyn params to control acceleration and deceleration
* Add command to offset raw velocity setpoints (for axes). One usecase can be to offset the zero velocity setpoint output of analog outputs:
```
ecmcConfigOrDie "Cfg.SetAxisDrvVelSetOffsetRaw(<axis_id>,<offset>)"

```
* Add command that adds an SDO object with a predefined type, up to 8 bytes:

Example: Write sign 64bit int 
```
ecmcConfigOrDie "Cfg.EcAddSdoDT(<slave index>,<sdoindex>,<sdosubindex>,<valuestring>,<datatypestring>)" 
#  0x8030:08, rwrwrw, int64, 64 bit, "Calibration position" for EP7211-0034
ecmcConfigOrDie "Cfg.EcAddSdoDT(11,0x8030,0x8,-1234,S64)"

# Verification
ethercat upload -p11 -m0 0x8030 0x8
0xfffffffffffffb2e -1234
```
* Remove reset of attarget bit when error reset is executed
* Fix of brake not engaging when drive loose power
* At traj source change then set target pos to setpos

## Add iocsh command for if statements:
1. ecmcIf(\<expression\>,\<optional true macro\>,\<optional false macro\>)
2. ecmcEndIf(\<optional true macro\>,\<optional false macro\>)

### ecmcIf(\<expression\>,\<optional true macro\>,\<optional false macro\>)
ecmcIf() set two macros depending on the value of the evaluated expression. If it evaluates to true:
1. IF_TRUE=""        Allows execution of a line of code   
2. IF_FALSE= "#-"    Block execution of a line of code

If expression evaluates to false:
1. IF_TRUE="#-"      Block execution of a line of code
2. IF_FALSE= ""      Allows execution of a line of code

Note: These macros is the default names for the macros (but can be changed by assignment of the 2 last params in call to ecmcIf()):
1. IF_TRUE for true
2. IF_FALSE for false

### ecmcEndIf(\<optional true macro\>,\<optional false macro\>)
The ecmcEndIf() command unsets the last used macros (for true and false), if differnt names are passed as arguments then then these macros are unset (for nested if statements).

### Example of of syntax
Example 1:
```
ecmcIf("<expression>")
${IF_TRUE} # Code to execute if expression eval true
#- else
${IF_FALSE} # Code to execute if expression eval false
ecmcEndIf()
```
Example 2:
```
ecmcIf("$(VAL1)=$(VAL2)")
${IF_TRUE}epicsEnvSet(IS_EQUAL,"1")
#- else
${IF_FALSE}epicsEnvSet(IS_EQUAL,"0")
ecmcEndIf()
```
Note: For nested calls to ecmcIf() and ecmcEndIf() optional macros must be used.

## Update of encoder handling
* Add asyn parameter to select primary encoder (for control). This parameter will also set the index of homing encoder. 
```
ax<id>.primencid
```
* The axis actpos and actvel always shows values based on the primary encoder.
* The axis actpos01..08, actvel01..08 shows the individual encoder values.
* Remove concept with homing encoder. The homing encoder is always the primary encoder.
* Add asyn param to switch encoder

The update is done in order to make it simpler to for instance switch between open loop and closed loop for steppers.

## Allow several domains:

Add domain: All entries configured after this call will belong to th new domain:
```
ecmcConfigOrDie "Cfg.EcAddDomain(<exe_cycles>,<exe_offset>)"
```
The domain can be configured to execute at slower rates than the default ec rate and with offsets.

The domain can be configured to be allowd to be offline:
```
ecmcConfigOrDie "Cfg.EcSetDomainAllowOffline(1)"
```
Note: If the domain is offline, ecmc will start even if the slaves in the domain are not connected to the bus.

Note: Axes which are using data from a domain will be interlocked if the domain status is not OK.

The ecmccfg commands addDomain.cmd wraps this functionality:
```
addDomain.cmd "ALLOW_OFFLINE=<allow>, EXE_RATE=<rate>,EXE_OFFSET=<offset>"
```
All parameters are optional.

A new plc function is added to allow supervision of the domain states:
```
ec_get_dom_state(<dom_index>)
```
## master 2 master communication:
By default a buffer of 128 doubles can be used for communication between different masters by plc functions:
```
m2m_write(<index>,<value>)     : write a value to an index in the buffer (index must be 0..119)
m2m_read(<index>)              : read a value at an index in the buffer (index must be 0..119)
m2m_stat()                     : check that connection to memory is ok.
m2m_err_rst()                  : reset any error
m2m_get_err()                  : get error code
m2m_ioc_ec_ok(<master_index>)  : ioc/master ethercat status ok/operational. 1==op, 0==not op, -1==error
m2m_ioc_run(<master_index>)    : ioc/master running (negative master id is ioc:s without ec master)
```
## Add extra set of controller parameters
Use different controller parameters depending on distance to target. This can be usefull in for instance systemes with backlash.

The command to configure this is:
```
"Cfg.SetAxisCntrlInnerParams(axis_no, kp, ki, kd, tol);
```
If distance to target is within +-tol then these controlelr parameters will be used. 

In ecmccfg-jinja, the configuration looks like below (the inner section):
```
controller:
  Kp: 10
  Ki: 0.1
  Kd: 0.25
  inner:
    tol: 1.0
    Kp: 10
    Ki: 0.1
    Kd: 0.25
```
## Add jinja script for adding extra encoder (in ecmccfg)

```
${SCRIPTEXEC} ${ECMC_CONFIG_ROOT}loadYamlEnc.cmd, "FILE=./cfg/enc1.yaml"
```
enc1.yal:
```
encoder:
  numerator: 360
  denominator: 12800
  type: 1         # Type: 0=Incremental, 1=Absolute
  bits: 16        # Total bit count of encoder raw data
  absOffset: 82.801    # Encoder offset in eng units (for absolute encoders)
  position: ec0.s4.positionActual01  # Ethercat entry for act-pos (encoder)
```
For more info on extra encoders read futher down in this file.

# ECMC 9.0.0
* Fixes to brake control
* Add asynparameter command to set encoder position (ax.setencpos). Usefull for save restore
* Block "Cfg.CreateAxis()" for already created axis 
* Add command for delaying the OK status for the ethercat bus at startup. Could be usefull to allow dc clocks to stabilize or for slaves that not report correct data even though reporting OP:
```
"Cfg.EcSetDelayECOkAtStartup(<milliseconds>)"
```
* Add asyn param for thread status:
  * memlockOK 
  * prioOK
* Assign velo to ecmc target velo asyn param (non motor record interface)
* Add rw asyn and plc access to controller PID parameters:
```
ax<id>.ctrl.kp
ax<id>.ctrl.ki
ax<id>.ctrl.kd
ax<id>.ctrl.kff
```

* Add command for enabling/disabling alarm when at softlimit:
```
Cfg.SetAxisEnableAlarmAtSoftLimit(int axis_no, int enable)
```
* Add command "Cfg.EcApplyConfig()" without master index as parameter. ecmc only allows one master per ioc.
* Return error if cmddata is not valid for the current motion command (at execution).
* Add command to allow change of encoder and trajectory source when axis is enabled:
```
Cfg.SetAxisAllowSourceChangeWhenEnabled(int axis_no, int allow)
```
## Add support for reading and writing SDO:s in runtime

The "Cfg.EcAddSdoAsync()" can be used to add a SDO to be read or written dunring runtime:
```
ecmcConfigOrDie "Cfg.EcAddSdoAsync(<slave>,<index>,<subindex>,<datatype>,<alias>)"
```
The command generates 5 asyn parameters:
```
ec<masterid>.s<slaveid>.sdo.<alias>.writecmd
ec<masterid>.s<slaveid>.sdo.<alias>.readcmd
ec<masterid>.s<slaveid>.sdo.<alias>.value
ec<masterid>.s<slaveid>.sdo.<alias>.error
ec<masterid>.s<slaveid>.sdo.<alias>.busy
```

### Process to write data
1. Write data to PV linked to "ec<masterid>.s<slaveid>.sdo.<alias>.value"
2. Ensure there is a positive edge on the PV linked to "ec<masterid>.s<slaveid>.sdo.<alias>.writecmd"
3. Check for errors in PV linked to ec<masterid>.s<slaveid>.sdo.<alias>.error

### Process to read data
1. Ensure there is a positive edge on the PV linked to "ec<masterid>.s<slaveid>.sdo.<alias>.readcmd"
2. Check for errors in PV linked to ec<masterid>.s<slaveid>.sdo.<alias>.error
3. Read data from PV linked to "ec<masterid>.s<slaveid>.sdo.<alias>.value"

### ecmccfg command addEcSdoRT.cmd 

The "addEcSdoRT.cmd" ecmccfg comand can be used to register a async SDO and generate PVs. Example for drive voltage setting if an EL7031 drive:
```
# Add RT SDO for reading writing voltage seting in terminal
${SCRIPTEXEC} ${ecmccfg_DIR}addEcSdoRT.cmd, "SLAVE_ID=${DRV},INDEX=0x8010,SUBINDEX=0x3,DT=U16,NAME=DrvVlt"
```

This call will generate the following PVs (corresponding to the above asyn params):
```
c6025a:m0s004-SDO-DrvVlt-WrtCmd
c6025a:m0s004-SDO-DrvVlt-RdCmd
c6025a:m0s004-SDO-DrvVlt-Val
c6025a:m0s004-SDO-DrvVlt-ErrId
c6025a:m0s004-SDO-DrvVlt-Bsy
```

## Multi encoder support

### General
Support for multiple encoders for axis objects added, max 8 encoders/axis. The follwing commands added to ecmc:
```
* "Cfg.AddAxisEnc(${ECMC_AXIS_NO})"
* "Cfg.SelectAxisEncPrimary($(ECMC_AXIS_NO),${ECMC_ENC_PRIMARY_ID=-1})"
* "Cfg.SelectAxisEncHome($(ECMC_AXIS_NO),${ECMC_ENC_HOME_ID=-1})"
* "Cfg.SelectAxisEncConfig($(ECMC_AXIS_NO),${ECMC_ENC_CFG_ID=-1})"
* "GetAxisEncConfigIndex($(ECMC_AXIS_NO))"
* "Cfg.SetAxisEncRefToOtherEncAtStartup($(ECMC_AXIS_NO),${ECMC_ENC_REF_TO_ENC_AT_STARTUP_ID=-1})"
* "Cfg.SetAxisEncEnableRefAtHome($(ECMC_AXIS_NO),${ECMC_ENC_REF_AT_HOME=-1})"
* "Cfg.SetAxisEncMaxDiffToPrimEnc($(ECMC_AXIS_NO),${ECMC_ENC_MAX_DIFF_TO_PRIM_ENC=0})"
```
The Cfg.CreateAxis() command will create one encoder with index 0, backwards compatible. This encoder is the default the encoder used for control (primary encoder) and also the encoder used during the defined homing sequence, if any. If an extra encoder is needed the Cfg.AddAxisEnc() can be executed. After an execution of Cfg.AddAxisEnc() all following encoder confguration commands will be applied to the newest encoder. If configuration of another encoder is needed then the "Cfg.SelectAxisEncConfig()" command can be used to switch the encoder being configured.

### Control
The primary encoder,used for control can be changed with the command "Cfg.SelectAxisEncPrimary()", defaults to encoder index 0:

Example: Set encoder with index 2 (the third encoder) to primary encoder for axis 1
```
ecmcConfigOrDie "Cfg.SelectAxisEncPrimary(1,2)"
```

### Homing / referencing
The encoder used for homing can be defined with the command "Cfg.SelectAxisEncHome()", defaults to encoder index 0.

If the primary and homing encoder are not the same, then during homing a tempoerarty swicth of encoder will occur. This can result in a step in actual position value. After homing control will be switched back to the primnary encoder.

Example: Use encoder with index 3 for homing of axis 1
```
ecmcConfigOrDie "Cfg.SelectAxisEncHome(1,3)"
```
Encoders can be referenced to the value of antother encoder at startup. Could be usefull if one encoder is absolute and one is incremental, to refernce the incremnetal to the value of the absolute encoder. This nfeature is configured by the "Cfg.SetAxisEncRefToOtherEncAtStartup()" command.

Example: Reference encoder 2 to the value of encoder 1 for axis 3
```
# Select encoder to configure
ecmcConfigOrDie "Cfg.SelectAxisEncConfig(3,2)"
# Encoder 2 selected, reference it to encoder 1 at startup
ecmcConfigOrDie "Cfg.SetAxisEncRefToOtherEncAtStartup(3,1)"
```
Encoders can also be referenced to the value of other encoders after a successful homing sequence (of any encoder). This feature can be enabled by the "Cfg.SetAxisEncEnableRefAtHome()" command:

Example: Allow referencing of encoder 1 after successfull homing axis 3
```
# Select encoder to configure
ecmcConfigOrDie "Cfg.SelectAxisEncConfig(3,1)"
# Encoder 1 selected, enable to reference it:
ecmcConfigOrDie "Cfg.SetAxisEncEnableRefAtHome(3,1)"
```

### Monitoring
The difference between each encoder and the primary encoder can be monitored by setting the maximum allowd difference with the "Cfg.SetAxisEncMaxDiffToPrimEnc()" command. Monitoring will only be executed between encoders being homed. If the deviation is higher than defined, the axis will be interlocked and thereby stopped.

Example: Allow a maximum difference of 0.1 between encoder 2 and the primary encoder for axis 4:
```
# Select encoder to configure
ecmcConfigOrDie "Cfg.SelectAxisEncConfig(4,2)"
# Encoder 2 selected, set maximum position difference:
ecmcConfigOrDie "Cfg.SetAxisEncMaxDiffToPrimEnc(4,0.1)"
```
### Example

For complete example see: "ecmccfg/examples/test/multi_encoder/"

The simplest way to create an additional encoder is to use the ecmccfg command addEncoder.cmd directlly after configuration of the axis:
```
epicsEnvSet("DEV",      "$(IOC)")
$(SCRIPTEXEC) ($(ecmccfg_DIR)configureAxis.cmd, CONFIG=./cfg/linear_1.ax)

# Add extra encoder to axis 1 (open loop counter)
$(SCRIPTEXEC) ($(ecmccfg_DIR)addEncoder.cmd, CONFIG=./cfg/axis_1_enc_1.enc)
```
The encoder configuration file supplied to addEncoder.cmd should look like this:
```
epicsEnvSet("ECMC_ENC_SCALE_NUM"               "60")
epicsEnvSet("ECMC_ENC_SCALE_DENOM"             "12800")
epicsEnvSet("ECMC_ENC_TYPE"                    "0")                        # Type: 0=Incremental, 1=Absolute
epicsEnvSet("ECMC_ENC_BITS"                    "16")                       # Total bit count of encoder raw data
epicsEnvSet("ECMC_ENC_ABS_BITS",               "0")                        # Absolute bit count (for absolute encoders) always least significant part of ECMC_ENC_BITS
epicsEnvSet("ECMC_ENC_ABS_OFFSET"              "0")                        # Encoder offset in eng units (for absolute encoders)
epicsEnvSet("ECMC_EC_ENC_ACTPOS",              "ec0.s7.positionActual01")  # Ethercat entry for actual position input (encoder)
epicsEnvSet("ECMC_EC_ENC_RESET",               "")                         # Reset   (if no encoder reset bit then leave empty)
epicsEnvSet("ECMC_EC_ENC_ALARM_0",             "")                         # Error 0 (if no encoder error bit then leave empty)
epicsEnvSet("ECMC_EC_ENC_ALARM_1",             "")                         # Error 1 (if no encoder error bit then leave empty)
epicsEnvSet("ECMC_EC_ENC_ALARM_2",             "")                         # Error 2 (if no encoder error bit then leave empty)
epicsEnvSet("ECMC_EC_ENC_WARNING",             "")                         # Warning (if no encoder warning bit then leave empty)

# This encoder (enc 1) will be refenced to encoder 0 at startup (set to -1 to not change setting)
epicsEnvSet("ECMC_ENC_REF_TO_ENC_AT_STARTUP_ID",  "-1")

# Encoder index for closed loop control (set to -1 to not change setting)
epicsEnvSet("ECMC_ENC_PRIMARY_ID",  "-1")

# Encoder index for homing, the specified homing seq will be executed based on this enc. (set to -1 to not change setting)
epicsEnvSet("ECMC_ENC_HOME_ID",  "1")

# Reference this encoder ar homing (set to -1 to not change setting)
epicsEnvSet("ECMC_ENC_REF_AT_HOME",  "1")

# Maximum position diff between this encoder and primary (set to 0 to disable)
epicsEnvSet("ECMC_ENC_MAX_DIFF_TO_PRIM_ENC",  "0.05")
```

# ECMC 8.0.1
* Fixes to brake control

# ECMC 8.0.0

* Add support for jerk limited trajectories based on ruckig (https://github.com/pantor/ruckig).
  Trapetzoidal trajectories still supported and default in ecmccfg. Ruckig module is now a dependency.
  
  Note S-curve trajectory restrictions: On the fly velocity target changes not allowed when posiitoning (blocked).
  
* Update of trajectory generator to allow "on the fly" update of target postion and target velocity (trapetz).
  
* Ensure that master is not scanning slaves at startup. If scanning then wait until scan is finished (with timeout).  

* Remove commands Cfg.SetOpMode() and GetOpMode(). Obsolete and not used.

* Add command "Cfg.SetAxisHomePosition()", same as Main.Mx.fHomePosition (try to phase out this old syntax)

* Add possibility to run motion without motor record.

Control params:
```
ax<id>.control
ax<id>.command
ax<id>.cmddata
ax<id>.targpos
ax<id>.targvelo
```

Status params:
```
ax<id>.status
ax<id>.actpos
ax<id>.setpos
ax<id>.poserr
ax<id>.diagnostic
```

Examples and more info are available in:
ecmccfg/examples/test/motionWithoutMotorRecord/

* Brake handling: 
  - Engage brake if not enabled
  - Start counting open delay when drive enabled goes high (instead of enable cmd).
* Reset axis enable cmd if ethercat in error state (prevent re-enable when ethercat returns to OK)
* Add command to set drive timeout for enabled and DS402 state machine: "Cfg.SetAxisDrvStateMachineTimeout(int axis_no, int seconds)"
* Fix of absolute encoder reading at ioc startup

# ECMC 7.0.1
* Add homing seq 25 (same as 15 but not blocked by motor record and reserved for save/restore). The sequence will just set a new position.
* Add axisType in status word
* Fix moving bit in motor record poll

# ECMC 7.0.0
* New release to match ecmccfg 7.0.0
* Add functionality to move to a position after success full homing procedure.
  The functionality is configuerd by the follwing two commands:
  ```
  1. ecmcConfigOrDie "Cfg.SetAxisHomePostMoveEnable(<axis_id>, <enable>)"                 # default disabled
  2. ecmcConfigOrDie "Cfg.SetAxisHomePostMoveTargetPosition(<axis_id>,<target position>)" # default 0.0
  ```
  
* Add homing seq 7..10:
  - seq 7 : Ref at first rising edge of home sensor moving in backward direction. 
  - seq 8 : Ref at first rising edge of home sensor moving in forward direction.
  - seq 9 : Ref at center of home sensor rising edges:
    1. Move backward 
    2. latch encoder position at rising edge of homesensor
    3. continue move backward until falling edge of homsensor, then stop
    4. Move forward and latch encoder position at rising edge of home sensor, then stop
    5. Reference at centerpoint between the two latched positions to homeposition value.
  - seq 10: Ref at center of home sensor rising edges:
    1. Move forward 
    2. latch encoder position at rising edge of homesensor
    3. continue move forward until falling edge of homsensor, then stop
    4. Move backward and latch encoder position at rising edge of home sensor, then stop
    5. Reference at centerpoint between the two latched positions to homeposition value.
  
  Note: Polarity of home sensor can be changed with the following command:
  ```
  "Cfg.SetAxisMonHomeSwitchPolarity(int axisIndex, int polarity)";
  # polarity==0 is NC (default)
  # polarity==1 is NO
  ```

## Migration  guide (6.x.x to 7.0.0)

Version 7 of ecmc and ecmccfg is intended to be more flexible in PV naming making it possible to follow for instance the ESS naming standard. At the same time a more systematic/genric approach to naming have been implemneted making use of macros easier. Because of these changes v7 is not compatible with v6. The last version of ecmc and ecmccfg with the old naming is v6.3.3.

### What have changed
The changes are described in detail in this file: [Naming Convention](https://github.com/icshwi/ecmccfg/blob/master/namingConvention.md)

These changes will mainly affect:
1. Default ecmc PV names
2. ecmc parameter names

#### 1. PV names
Updated PV names affects:
1. Local databases accesing data from ecmc PV:s
2. OPIs accessing data from ecmc PVs
3. Archiving configuration 

Example of updated PV name:
```
Old:
IOC_TEST:ec0-s6-EL5101-Enc-PosAct

New:
IOC_TEST:m0s006-Enc01-PosAct
```

#### 2. ecmc parameter names
Updated ecmc paramter names affects:
1. Local databases accesing data directlly from ecmc parameters
2. EtherCAT variables in ecmc PLC:s
3. Motion configuration links to hardware (in both normal *.ax files and sync files. *.sax)

Example of updated ecmc parameter name:
```
Old:
ec0.s5.BI_7

New:
ec0.s5.binaryInput07
```

Example of needed update in PLC code and sync files:
```
Old:
ec0.s5.BO_7:=ec0.s1.CH1_VALUE > 500;

New:
ec0.s5.binaryOutput07:=ec0.s1.analogInput01 > 500;
```

Example of needed update of hardware links in motion configuration file:
```
Old:
epicsEnvSet("ECMC_EC_DRV_CONTROL",        "ec0.s8.STM_CONTROL.0")        # Ethercat entry for control 

New:
epicsEnvSet("ECMC_EC_DRV_CONTROL",        "ec0.s8.driveControl01.0")     # Ethercat entry for control 
```

### New Functionalities 6.3.3

Also update the motion copnfiguration files with the new functionalities in ecmc 6.3.3.

#### Drive and Encoder Alarms, Warning and reset linked to motor record

Since ecmc 6.3.3 the follwing can be linked to the motion axis:
* 3 drive hardware alarms
* 1 drive reset error bit
* 1 drive warning bit
* 3 encoder hardware alarms
* 1 encoder reset error bit
* 1 encoder warning bit
 
Alarms:

If any of the alarm bits is going high the axis will stop and and an alarm meassage will be propagated to motor record (if used).

Reset:

The reset bit is linked to the motor record reset PV (a reset from motor reset pv will result in reset toggle for 1 scan).

Warning:

The warning bit will at the moment just result in a printout in the iocsh log. Several drives, like el7037, are in warning state when not enabled.

Variables:

In order to use the new alarm and reset functionality some new varaibles need to be set in the motion configuration files.

Example EL7037 and EL5002:
```
# Add to drive section (example of EL7037 drive):
epicsEnvSet("ECMC_EC_DRV_RESET",          "ec0.s8.driveControl01.1")      # Reset
epicsEnvSet("ECMC_EC_DRV_ALARM_0",        "ec0.s8.driveStatus01.3")       # Error
epicsEnvSet("ECMC_EC_DRV_ALARM_1",        "ec0.s8.driveStatus01.7")       # Stall
epicsEnvSet("ECMC_EC_DRV_ALARM_2",        "ec0.s8.driveStatus01.14")      # Sync error
epicsEnvSet("ECMC_EC_DRV_WARNING",        "ec0.s8.driveStatus01.2")       

# Add to encoder section (example of EWL5002 SSI encoder):
epicsEnvSet("ECMC_EC_ENC_RESET",          "")                             # Reset (no reset bit)
epicsEnvSet("ECMC_EC_ENC_ALARM_0",        "ec0.s3.encoderStatus01.0")     # Error 0: SSI input error
epicsEnvSet("ECMC_EC_ENC_ALARM_1",        "ec0.s3.encoderStatus01.1")     # Error 1: SSI frame error
epicsEnvSet("ECMC_EC_ENC_ALARM_2",        "ec0.s3.encoderStatus01.2")     # Error 2: Encoder power fail
epicsEnvSet("ECMC_EC_ENC_WARNING",        "")                             # Warning (no warning bit)
```
Note: that the above examples are for EL7037 and EL5002. For other drives and encoders other hardware links need to be added.

The bits can be linked to any ethercat entries.


# ECMC 6.3.3
* Block setEnable from motor record if ax<id>.blockcom==1
* Block mc_* plc functions if ax<id>.allowplccmd==0
* Allow plc:s to write to axis as default.
* Increase max allowed datastorage count to 32
* Increase max allowed plc count to 16
* Add alarm and reset links to axis encoder and drive object (that can be assigned with the "Cfg.LinkEcEntryToObject()"-command):
  
  - ax<id>.enc.reset
  - ax<id>.enc.alarm0
  - ax<id>.enc.alarm1
  - ax<id>.enc.alarm2
  - ax<id>.enc.warning
  - ax<id>.drv.reset
  - ax<id>.drv.alarm0
  - ax<id>.drv.alarm1
  - ax<id>.drv.alarm2
  - ax<id>.drv.warning

  The intention with these links is to link drive and encoder related errors and reset commads to interlock motion and display alarms in axis error field. For an EL7037 the following bits could be mapped:
  
  - ax<id>.drv.reset   to  ec<id>.s<id>.STM_CONTROL.1   // Reset bit in drive control word.
  - ax<id>.drv.alarm0  to  ec<id>.s<id>.STM_STATUS.3    // Error bit in drive status word.
  - ax<id>.drv.alarm1  to  ec<id>.s<id>.STM_STATUS.7   // Stall bit drive status word.
  - ax<id>.drv.alarm2  to  ec<id>.s<id>.STM_STATUS.14   // Sync error bit drive status word.  
  - ax<id>.drv.warning to  ec<id>.s<id>.STM_STATUS.2   // Warning bit drive status word.
  
  If the reset link is defined then this bit will be set for one cycle when issueing an error reset command.
  Note: Any of these new links can be left unused/blank.

* Add plc function mc_home_pos():
  Reference encoder to a postion (same as mc_home but with an extra argument for position, so no need to set home position with ax<id>.enc.homepos). 

* Add plc function mc_move_ext_pos():
  Plc function will move axis to the current external plc setpoint. Inteded to be used to move a slave axis to it's start position before allowing external setpoints.

* Add variables:

  - ax<id>.enc.extactpos  (ro): Current external plc calculated actual position (ax<id>.enc.actpos can be used for writing).
  - ax<id>.traj.extsetpos (ro):  Current external plc calculated setpoint position(ax<id>.enc.setpos can be used for writing).

* Position setpoint is syncronized with actual position when axis is disabled. Setpoint is updated once at positive edge of enable command. Note: The position setpoint is synced with actual position before first enable.

# ECMC 6.3.2
* Add command to enable/disable motion functions (all are by default enabled):
```
ecmcConfigOrDie "Cfg.SetAxisEnableMotionFunctions(<axisid>,
                                                  <enable pos>,
                                                  <enable const velo>,
                                                  <enable home>)"
```
* ecmc-updated param changed to uint64 if asyn version >=4.37
* Fix printout of plugin plc functions info for funcs with more than 6 args.

# ECMC 6.3.1
* Add support for 64 bit integers (if ASYN version >= 4.37)
* Add plc function to push memmaps to epics on demand (used when T_SMP_MS=-1):
  ```
  ec_mm_push_asyn(<memmap id>)
  ```
* Add command to get memmap id from name: EcGetMemMapId(<memmap name>).
  Can be used to feed memmap ids into PLCs.
  ```
  ecmcConfig "EcGetMemMapId(ec0.s11.mm.CH1_ARRAY_IN)"
  epicsEnvSet(MM_CH_1_IN,${ECMC_CONFIG_RETURN_VAL})
  ```
* Add command to use CLOCK_REALTIME (instead of CLOCK_MONOTONIC which is default in ecmc):
```
Use CLOCK_REALTIME:
ecmcConfigOrDie "Cfg.EcUseClockRealtime(1)"
Use CLOCK_MONOTONIC:
ecmcConfigOrDie "Cfg.EcUseClockRealtime(0)"
```
* Allow up to 64 memmaps (before max 16)
* Add command to diable axis at reset:
```
Cfg.SetAxisDisableAtErrorReset(<axis id>, disable)
```

# ECMC 6.3.0
* Add axis control word and update axis status word (phasing out stream)
* Add ethercat summary status bit (asyn param ec<m-id>.ok). Forced update each cycle.
* Update domain status each cycle
* Use new motor record release (dependency on motor 7.0.4)
* Support for motor record RDBL
* Bug fix in relative motion (when using RDBL)

# ECMC 6.2.4
* Add support for drive CSP mode (Cyclic Sync. Position interface). CSP can be used by linking a ethercat entry to "ax<id>.drv.position" (and not linking an entry to "ax1.drv.velocity"):
```
# Setup CSP on axis 1:
ecmcConfigOrDie "Cfg.LinkEcEntryToObject(ec1.s3.SET_POSITION,"ax1.drv.position")"
ecmcConfigOrDie "Cfg.LinkEcEntryToObject("","ax1.drv.velocity")"
```
NOTE: The ecmc position control loop parameters will not have any effect in CSP mode (since position loop is distributed to the drive). Control parameters need to be set directlly in the drive (by SDO):
```
# These commands will not have any effect in CSP mode (set to 0, or you will get a warning meassage):
ecmcConfigOrDie "Cfg.SetAxisCntrlKp(${ECMC_AXIS_NO},${ECMC_CNTRL_KP})"
ecmcConfigOrDie "Cfg.SetAxisCntrlKi(${ECMC_AXIS_NO},${ECMC_CNTRL_KI})"
ecmcConfigOrDie "Cfg.SetAxisCntrlKd(${ECMC_AXIS_NO},${ECMC_CNTRL_KD})"
ecmcConfigOrDie "Cfg.SetAxisCntrlKff(${ECMC_AXIS_NO},${ECMC_CNTRL_KFF})"
...
```
* Update to DS402 timeout unit to seconds (before it was cycles).
* Add possabilty to verify actual ec slave revision vs config revsion with "Cfg.EcSlaveVerify()" command.
  The slave revision number must be higher or equal (>=) comapred to the config revsion number in order to be approved.
  Note: Comparsion is only made if the supplied config revsion number >= 0.
  Note: "Cfg.EcSlaveVerify()" can still be used without revsion number and then the revsion chesk will not be performed.
* Add encoder position averaging lp-filter:
  * Cfg.SetAxisEncPosFilterSize(<axis_id>,<filter_size>)
  * Cfg.SetAxisEncPosFilterEnable(<axis_id>,<enable>)
* Plugin interface: Fix sampeltime unit to ms
* Add warnings for 64bit ethercat plc variables (might not be represented properly since all plc variables are doubles). A typical example would be a 64bit dc-timestamp.
* Add plc- functions
  * ds_append_to_ds()
  * ec_mm_append_ds()
* Minor fix of float and double ec entries
* Minor fix asynUint32Digital interface

# ECMC 6.2.3
* Add support for motor auto enable/disable
* Add plc functions ec_get_time_u32(), ec_get_time_l32().
* ecmcForLoop: iocsh command: Support decreasing iterations
* Fix moving bit in ecmcMotorRecordAxis::poll()

# ECMC 6.2.2
* Allow JVEL change while jogging
* Smoother trajectory at stopping (start stop seq. one cycle earlier than before to avoid small jerk at stop)
* Update to support autosave of motor position

# ECMC 6.2.1
* Add exprtk support for plc functions with stings as args  (need ess-exprtk 1.2.1)
* Add possabilty ti use plc-functions with strings a sarguments in plugins
* Expose ecmc epics ioc state to plugins
* Add plugin for pvAccess in ecmc plc:s: https://github.com/anderssandstrom/e3-ecmcPlugin_Utils

# ECMC 6.2

### Add plugin support
ecmc plugins (shared libs) can be loaded into ecmc. Plugins can:
* Access ecmc data:
  * ethercat data
  * motion data
  * plc data
  * data storages
  * subscribe to data by callbacks
  * Add/register new asyn parameters for direct access over asyn
* Get callbacks:  
  * Each realtime loop  
  * When ecmc data updates
  * At plugin load
  * At plugin unload
  * At enter realtime
  * At exit realtine
* Implement custom ecmc plc functions
* Implement custom ecmc plc constants

A plugin can also access the ecmc api headers for direct access to most parts of ecmc.

Some plugin examples:
1. FFT plugin: https://github.com/anderssandstrom/e3-ecmcPlugin_FFT
2. Demo plugin: https://github.com/anderssandstrom/e3-ecmcPlugin_Advanced
3. WiringPi ecmc wrapper: https://github.com/anderssandstrom/e3-ecmcPlugin_RasPi

Plugins can be loaded by the ecmccfg command loadPlugin.cmd. Plugin configurations can be added in a configurations string.
Example:
```
#- Load plugin 0: Advanced
epicsEnvSet(ECMC_PLUGIN_FILNAME,"/epics/base-7.0.3.1/require/3.1.2/siteMods/ecmcPlugin_Advanced/master/lib/linux-arm/libecmcPlugin_Advanced.so")
epicsEnvSet(ECMC_PLUGIN_CONFIG,"DBG_PRINT=0;") # Only one option implemented in this plugin
${SCRIPTEXEC} ${ecmccfg_DIR}loadPlugin.cmd, "PLUGIN_ID=0,FILE=${ECMC_PLUGIN_FILNAME},CONFIG='${ECMC_PLUGIN_CONFIG}', REPORT=1"
epicsEnvUnset(ECMC_PLUGIN_FILNAME)
epicsEnvUnset(ECMC_PLUGIN_CONFIG)

```
### Allow running without EtherCAT hardware
Add possibility to run without attached ethercat hardware. Most functionalities are still available:
* PLC:s
* motion (most likely to use virtual axes, but also normal axis can be used).
* ..

See examples in ecmccfg.

Note: When running without ethercat hardware you should not execute the "Cfg.EcSetMaster()" command.

### Add iocsh command to exit
Add ecmcExit() to exit EPICS/ECMC (needed since the iocsh command "exit" just stops reading current "file"). 
Can be used for stopping execution when a configuration error occurs.

```
  Use "ecmcExit" to exit ecmc/EPICS.
  Use "ecmcExit -h | --help" to get help message.

```

### Add iocsh command to execute for loop
Usefull for:
* Large systems with many similar sub systems
* Configuring hardware with many PDOs (oversampling)

```
"ecmcForLoop(<filename>, <macros>, <loopvar>, <from>, <to>, <step>)" to loop execution of file with a changing loop variable.
             <filename> : Filename to execute in for loop.
             <macros>   : Macros to feed to execution of file.
             <loopvar   : Environment variable to use as index in for loop.
             <from>     : <loopvar> start value.
             <to>       : <loopvar> end value.
             <step>     : Step to increase <loopvar> each loop cycle.

```
Example ("ECMC_LOOP_IDX" as loop variable):

```
ecmcForLoop(./loopStep.cmd,"",ECMC_LOOP_IDX,1,5,1)
ecmcEpicsEnvSetCalc("TESTING",1*10)
epicsEnvShow("TESTING")
TESTING=10
ecmcEpicsEnvSetCalc("TESTING",2*10)
epicsEnvShow("TESTING")
TESTING=20
ecmcEpicsEnvSetCalc("TESTING",3*10)
epicsEnvShow("TESTING")
TESTING=30
ecmcEpicsEnvSetCalc("TESTING",4*10)
epicsEnvShow("TESTING")
TESTING=40
ecmcEpicsEnvSetCalc("TESTING",5*10)
epicsEnvShow("TESTING")
TESTING=50

```
where "loopStep.cmd" file looks like this (note the use of "ECMC_LOOP_IDX"):
```
#- Commands tp execute in each loop of example ecmcForLoop.script
ecmcEpicsEnvSetCalc("TESTING",${ECMC_LOOP_IDX}*10)
epicsEnvShow("TESTING")

```

### Add iocsh command to check if a file exist
Usefull for checking that configuration files really exist and then can be loaded.
```
ecmcFileExist(<filename>, <die>, <check EPICS dirs>, <dirs>)" to check if a file exists.
              <filename>          : Filename to check.
              <die>               : Exit EPICS if file not exist. Optional, defaults to 0.
              <check EPICS dirs>  : Look for files in EPICS_DB_INCLUDE_PATH dirs. Optional, defaults to 0.\n");
              <dirs>              : List of dirs to search for file in (separated with ':').
result will be stored in the EPICS environment variable "ECMC_FILE_EXIST_RETURN_VAL"
```
Example:
```
ecmcFileExist("file_exist.cfg")
epicsEnvShow(ECMC_FILE_EXIST_RETURN_VAL)
ECMC_FILE_EXIST_RETURN_VAL=1

ecmcFileExist("file_not_exist.cfg",1)
Error: File "file_not_exist.cfg" does not exist. ECMC shuts down.

ecmcFileExist("ecmcEK1100.substitutions",1,1)
epicsEnvShow(ECMC_FILE_EXIST_RETURN_VAL)
ECMC_FILE_EXIST_RETURN_VAL=1

ecmcFileExist("ecmcEK1100.substitutions",0,0,"/home/")
epicsEnvShow(ECMC_FILE_EXIST_RETURN_VAL)
ECMC_FILE_EXIST_RETURN_VAL=0

```

### Add new commands to set sample rate:

1. Preferred way. Write to the parameter "EC_RATE" of startup.cmd (unit of EC_RATE is [Hz]):

```
$(ECMCCFG_INIT)$(SCRIPTEXEC) ${ecmccfg_DIR}startup.cmd, "IOC=$(IOC),ECMC_VER=<ecmcVersion>,stream_VER=<stremVer>, EC_RATE=<rate>"
```

2. Issueing the command "ecmcConfigOrDie "Cfg.SetSampleRate(<sampleRate>)" where the unit of sampleRate is [Hz]:

```
ecmcConfigOrDie "Cfg.SetSampleRate(500)"
```
3. Issueing the command "ecmcConfigOrDie "Cfg.SetSampleTimeMs(<sampleTimeMs>)" where unit of sampleTime is [ms]:
```
ecmcConfigOrDie "Cfg.SetSampleTimeMs(2)"
```

Note: The commands "Cfg.SetSampleRate()" or "Cfg.SetSampleTimeMs()" needs to be executed before any ecmc configuration 
is performed but of cource after the "require ecmc" statement. Therefore the parameter EC_RATE is the prefereed way to handle changed sample rates.

Example and doc in: ecmccfg/examples/test/changeSampleRate/

### ecmcConfig(OrDie) result to epics environment variable

The return value from ecmcConfig(OrDie) is stored in the EPICS environment variable
"ECMC_CONFIG_RETURN_VAL". This value can be used to make som dynamic configuration.
All ASCII configuration commands for ecmcConfig(OrDie) can be used in the same way.

Example: Read firmware version of an EL7037 stepper drive
Note: SDO reads need to be before "SetAppMode(1)"
```
ecmcConfig "EcReadSdo(${ECMC_SLAVE_NUM},0x100a,0x0,2)"
epicsEnvShow(ECMC_CONFIG_RETURN_VAL)
ECMC_CONFIG_RETURN_VAL=14640

```
The variable "ECMC_CONFIG_RETURN_VAL" then can be used to set record fields, name or alias for instance.. 

Example: Read "ID" PDO from EK1101 (shown in detail in aliasRecordFromPdoData.script)
Note: PDO reads need to be after "SetAppMode(1)" since cyclic value
```
ecmcConfig "ReadEcEntryIDString(${ECMC_SLAVE_NUM},"ID")"
2020/02/28 08:58:03.771 1024
## This is the value of the EK1101 ID switch
epicsEnvShow(ECMC_CONFIG_RETURN_VAL)
ECMC_CONFIG_RETURN_VAL=1024
```

### Add "ecmcEpicsEnvSetCalcTernary()" 
Used for evaluating expressions and set EPCIS environment variables to different strings.
depending on if the expression evaluates to "true" or "false". Can be usefull for:
* Choose different files to load like plc-files, axis configurations, db-files or..
* making conditional ecmc settings
* ...
  
``` 
ecmcEpicsEnvSetCalcTernary -h

 Test iocsh function "ecmcEpicsEnvSetCalcTernary()" t

        Use "ecmcEpicsEnvSetCalcTernary(<envVarName>,  <expression>, <trueString>, <falseString>)" to evaluate the expression and        assign the variable.
          <envVarName>  : EPICS environment variable name.
          <expression>  : Calculation expression (see exprTK for available functionality). Examples:
                          Simple expression:"5.5+${TEST_SCALE}*sin(${TEST_ANGLE}/10)".                          
                          Use of "RESULT" variable: "if(${TEST_VAL}>5){RESULT:=100;}else{RESULT:=200;};".
                          Strings are used within '<str>': "'test'='test'". Note: expression result must be numeric and 
                          not string (in this case expression result is 1 => <envVarName> will be set to <trueString>).
          <trueString>  : String to set <envVarName> if expression (or "RESULT") evaluates to true.
          <falseString> : String to set <envVarName> if expression (or "RESULT") evaluates to false.

```
Examples:
```
#### Simple true false
epicsEnvSet("VALUE",10)
# ecmcEpicsEnvSetCalcTernary("test_var", "${VALUE}+2+5/10","True","False")
ecmcEpicsEnvSetCalcTernary("test_var", "10+2+5/10","True","False")
epicsEnvShow("test_var")
test_var=True

### Can be used for choosing different files
# ecmcEpicsEnvSetCalcTernary("filename", "${VALUE}>20","./plc_fast.cfg","./plc_slow.cfg")
ecmcEpicsEnvSetCalcTernary("filename", "10>20","./plc_fast.cfg","./plc_slow.cfg")
epicsEnvShow("filename")
filename=./plc_slow.cfg

### Comparing strings 1 (simple):
# ecmcEpicsEnvSetCalcTernary("result", "'$(filename)'='./plc_slow.cfg'","equal","not_equal")
ecmcEpicsEnvSetCalcTernary("result", "'./plc_slow.cfg'='./plc_slow.cfg'","equal","not_equal")
epicsEnvShow("result")
result=equal

### Comparing strings 2 (with if-else):
# ecmcEpicsEnvSetCalcTernary("result", "if('$(filename)'='test') {RESULT:=1;}else{RESULT:=0;};","use_this_file.cfg","no_use_this_file.cfg")
ecmcEpicsEnvSetCalcTernary("result", "if('./plc_slow.cfg'='test') {RESULT:=1;}else{RESULT:=0;};","use_this_file.cfg","no_use_this_file.cfg")
epicsEnvShow("result")
result=no_use_this_file.cfg

```

### Add "ecmcEpicsEnvSetCalc()"
iocsh command for evaluating expressions and setting epics environment variables. Usefull for calculate:
  * slave offsets
  * sdo/pdo adresses (also in hex)
  * scalings in motion
  * record fields
  * ...
``` 
ecmcEpicsEnvSetCalc -h

      "ecmcEpicsEnvSetCalc(<envVarName>,  <expression>, <format>)"
          <envVarName> : EPICS environment variable name.
          <expression> : Calculation expression (see exprTK for available functionality).
                         Simple expression:"5.5+${TEST_SCALE}*sin(${TEST_ANGLE}/10)".                          
                         Use of "RESULT" variable: "if(${TEST_VAL}>5){RESULT:=100;}else{RESULT:=200;};".
                         Strings are used within '<str>': "'test'='test'". Note: expression result must be numeric and 
                         not string (in this case expression result is 1 => <envVarName> will be set to "1").
          <format>     : Optional format string. Example "%lf", "%8.3lf", "%x", "%04d" or "%d", defaults to "%d".
                         Can contain text like "0x%x" or "Hex value is 0x60%x".
                         Must contain one numeric value where result of expression will be written.

       Restrictions:
         - Some flags and/or width/precision combinations might not be supported.
         - Hex numbers in the expression is not allowed (but hex as output by formating is OK).
         - Non floatingpoint values will be rounded to nearest int.
```
Examples:
```
#### Calculate addresses in HEX with specified width
# ecmcEpicsEnvSetCalc("test2", "$(test1)+1+2+3+4+5*10.1", "%03x")
ecmcEpicsEnvSetCalc("test2", "061+1+2+3+4+5*10.1", "%03x")
epicsEnvShow("test2")
test2=07a

#### Hex + text in format string
# ecmcEpicsEnvSetCalc("test2", "$(test1)+1+2+3+4+5*10.1", "This is the number: 0x%06x")
ecmcEpicsEnvSetCalc("test2", "061+1+2+3+4+5*10.1", "This is the number: 0x%06x")
epicsEnvShow("test2")
test2=This is the number: 0x00007a

#### Calculate scalings (floating point)
epicsEnvSet("IORange",32768)
# ecmcEpicsEnvSetCalc("scaling", "$(test1)/$(IORange)*10", "%lf")
ecmcEpicsEnvSetCalc("scaling", "061/32768*10", "%lf")
epicsEnvShow("scaling")
scaling=0.018616

#### Offset slave numbers (format defaults to "%d")
epicsEnvSet("ECMC_SLAVE_NUM",10)
epicsEnvSet("ECMC_SLAVE_NUM_OFFSET",25)
# ecmcEpicsEnvSetCalc("ECMC_SLAVE_NUM", "$(ECMC_SLAVE_NUM)+$(ECMC_SLAVE_NUM_OFFSET)")
ecmcEpicsEnvSetCalc("ECMC_SLAVE_NUM", "10+25")
epicsEnvShow("ECMC_SLAVE_NUM")
ECMC_SLAVE_NUM=35

#### Comparing strings 1 (use '<string>'):
epicsEnvSet("filename","ecmcEL3002.cmd")
# ecmcEpicsEnvSetCalc("result", "'$(filename)'='test.script'")
ecmcEpicsEnvSetCalc("result", "'ecmcEL3002.cmd'='test.script'")
epicsEnvShow("result")
result=0

#### Comparing strings 2 (if-else):
# ecmcEpicsEnvSetCalc("result", "if('$(filename)'='test.script') {RESULT:=1;}else{RESULT:=0;};")
ecmcEpicsEnvSetCalc("result", "if('ecmcEL3002.cmd'='test.script') {RESULT:=1;}else{RESULT:=0;};")
epicsEnvShow("result")
result=0

```
### Misc
* Add commands for SDO complete and buffer access:
 * "Cfg.EcAddSdoComplete()"
 * "Cfg.EcAddSdoBuffer()"
* Remove obsolete command "EcWriteSdoComplete()"
* Add command to set EtherCAT startup timeout: "Cfg.SetEcStartupTimeout(<time_seconds>)"

# ECMC 6.1

* Add custom motor record driver (asynMotor) in order to improve realtime performance
* New iocsh commands (same syntax as EthercatMC):

 ```sh
ecmcMotorRecordCreateController(portName,         // Asyn port name for asyn com with ecmcMotorRecord
                                 motorPortName,    // Not used. Kept to keep syntax same as EthercatMC
                                 numAxes,          // Not used. Kept to keep syntax same as EthercatMC
                                 movingPollPeriod, // Poll period when moving [ms]
                                 idlePollPeriod,   // Poll period when idle [ms]
                                 optionStr)        // Options (execute "ecmcMotorRecordCreateController" in iocsh to see available options)
```

```sh
ecmcMotorRecordCreateAxis(controllerPortName,     // Same as portName for controller
                          axisNo,                 // Axis index
                          axisFlags,              // Options mainly for powering axis.
                          axisOptionsStr)         // Options (execute "ecmcMotorRecordCreateAxis" in iocsh to see available options)

```
* Note: EthercatMC is still supported and can be used like before
    
# ECMC 6.0.2

* Add iocsh commands "ecmcGrepParam()" and "ecmcGrepRecord()"
    1. ecmcGrepParam(<pattern>):
       Lists information about all ecmc parameters which parameter name corresponds to pattern.
    2. ecmcGrepRecord(<pattern>):
       Lists information about all ecmc parameters which ecmc-record name corresponds to pattern.
* Add PLC expression as asyn param (to avoid stream device).
* Use epicsThreadCreate() for rt thread (ecmc thread now accessible through MCoreUtils module).
* Remove LOGINFO15 printouts. For printouts use PLC instead.
* Fix inconversion command FLOAT64TOINT32 cmd.
* Change naming of axis plc default parameters (Example: plcs.plc9.enable => plcs.ax1.plc.enable)
* ecmcAxisBase: Add trajsource, encsource  and plccmdallowed in axis status word.
* ecmcEcSlave: Only update ethercat entries that are in use (increase performance).
* Add functionality to latch limit switch.
  Motion will need to stop even if limit switch is just low for one cycle (bounce).
  For safety reasons, this is also the default behaviour in ecmc from now on.
  Before motion was allowed all the time when limits were OK (even after limit bounce).
    
  Two commands have been added for this functionallity:
    1. "Cfg.SetAxisMonLatchLimit(<axis_no>,<enable>)"
    2. "GetAxisMonLatchLimit(<axis_no>)"
    
  Currentlly it's a bit unclear how this will affect syncronized axes.
  If issues then the functionality can be disabled of with:
  "Cfg.SetAxisMonLatchLimit(<axis_no>, 0)"

* Make cmd parser more effichent (runtime commands eval first).
* Change ecmc_rt thread default prio to from 60 to 72.
* Add access to memmap data in PLC:s (see below for added plc functions)
* Add definitions of EC datatypes to entries and memmaps: 
* ECMC repo. moved to https://github.com/epics-modules/ecmc
* ECMC tested with macros in plc code:
    * Same syntax as other iocsh macros (code parsed with msi)
    * Handled in ecmccfg loadPLCFile.cmd. 
    * Usefull for writing generic code
* Tested with analog output oversampling cards (100kHz) (EL4732).
* Added PLC-functions:
    1. ec_wrt_bits(<val>,<val_to_write>,<start_bit>,<stop_bit>):
       Writes <val_to_write> to <start_bit>..<stop_bit> of <val>

    2. ec_chk_bits(<val>,<start_bit>,<stop_bit>):
       Returns value of bits in range <start_bit>..<stop_bit> of <val>

    3. ec_get_time():
       Returns current time in nanoseconds from (1 jan 2000)

    4. ec_mm_cp( <srcId>,<sdestId>) 
       Copy memmaps

    5. ec_get_mm_type( <srcId>);
       Return type of memmap

    6. ec_get_mm_data(<srcId>,<index>)
       Read data point from memmap

    7. ec_set_mm_data(<srcId>,<index>,<data>)
       Write data point to memmap

    8. ec_get_mm_size(<srcId>)
       Returns number of elements (of type "ec_get_mm_type()")in memmap with srcId. 
       If return value is less than zero it should be considered to be an error code.

    9. ec_get_time()
       Returns current time in nano seconds (from 1 Jan 2000, same as EtherCAT DC:s).
       If return value is less than zero it should be considered to be an error code.

    10. ec_err_rst():
        Reset ec li error code. 

    11. Fixed ec_print_hex() plc function.

# ECMC 6.0.1

* The functionalities of ecmctraining is now migrated to ecmccfg repo: 
    Main repo: https://github.com/paulscherrerinstitute/ecmccfg
    Local ESS fork: https://github.com/icshwi/ecmccfg (For E3, use https://github.com/icshwi/e3-ecmccfg)
    The preferred way to configure ecmc is by the use of ecmccfg instead of ecmctraining

* PLC: 
    1. Add ec_wrt_bit() command.
    2. Add ax<if>.allowplccmd variable to plcs.
    3. Allow write to ax<id>.traj.source and ax<id>.enc.source from plc.

* Add possibility to change polarity of switches
    1. "Cfg.SetAxisMonLimitBwdPolarity(<axis id>,<pol>)"
    2. "GetAxisMonLimitBwdPolarity(<axis id>)"
    3. "Cfg.SetAxisMonLimitFwdPolarity(<axis id>,<pol>)"
    4. "GetAxisMonLimitFwdPolarity(<axisid>)"
    5. "Cfg.SetAxisMonHomeSwitchPolarity(<axis id>,<pol>)"
    6. "GetAxisMonHomeSwitchPolarity(<axis id>)"
    7. "GetAxisMonExtHWInterlockPolarity(<axis id>)"
    
    * <axis id>   Id of axis
    * <pol>       Polarity of switch
        * 0 = NC (1=OK)
        * 1 = NO (0=OK)

    Note: Even if polarity of limit switches is set to NO, the internal representation
    for limit switch values will still be 1 = OK (which then corresponds to a 0 on the physical input).

* Add drvInfoString cmd :FLOAT64TOINT32:
    Possibility to convert ECMC float64 to int32. Useful for instance when writing to plc bit variables 
    from bo records

# ECMC 6.0.0

Version 6.0.0 is not completely backward compatible since some changes 
in command-set have been made.
Please use ecmctrainig version 6.0.0 for configuration

## Updates 

### General
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
    * Variable name (use ecmcReport 2 or asynReport 3 to see available parameters):
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
    * asynReport 3: list all available parameters (that can be connected to records)
      NOTE: Also other asyn modules will display information when asynReport is executed.
      The motor record driver will output a lot of information which could lead to that
      it's hard to find the information. Therefore ecmcReport command was added (which
      just print out information from ecmc).

* Add ecmcReport iocsh command:
  Same as asynReport but only for ecmc (will not for instance list asynReport for motor record driver).

* Add ONE, ZERO entries for each slave with read and write access.
  This to allow more flexibility when linking values:
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
    * Time stamp source can be chosen by setting record TSE-field
        * TSE =  0 EPCIS timestamp
        * TSE = -2 ECMC timestamp, 
* Update sample rates from skip cycles to rate in ms

* Remove command: Cfg.SetAxisGearRatio(). Obsolete, handled through axis plc instead.

* ecmcConfigOrDie: Add printout of command if fail (error)

* Add asyn parameter that updates when all other parameters have been updated: ecmc.updated of type asynInt32ArrayIn

* Update to new asynPortDriver constructor (to remove warning)

* Add command Cfg.EcSlaveVerify(). Check that a slave at a certain bus-position have the correct vendor-id and product-id.
  This command is added to all hardware snippets in ecmctraining to ensure that the slave is of correct type.
  
* Add range checks for ec-entry writes (based on bit length and signed vs unsigned)

* Add memory lock (memlockall) and allocation of stack memory (good for rt performance)

* Add license file

* Add basic implementation of a basic commissioning GUI (ecmccomgui). Currently in different repository:
  based on pyQT, pyEpics

* Add command that verifies a SDO entries value:
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

* Add command to set velocity low pass filter size for encoder (actpos) and trajectory (setpos):
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

* Replace axis command transform with axis dedicated PLC object (executed in sync before axis object), 
  in order to use same syntax as in rest of ECMC.
  Many new variables and functions can now be used in the axis sync PLC (same as "normal" PLC object)
  Currently the following indexes apply (not needed info if the Cfg.SetAxisPLC* commands are used):
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
    * ds_get_avg(<ds id>) : Get average (Useful as lowpass filter)
    * ds_get_min(<ds id>) : Get minimum
    * ds_get_max(<ds id>) : Get maximum
  All three functions only use the data that has been written to the storage (not the empty/free space). 
  
* Add interlock variables in PLC : 
    * axis<id>.mon.ilockbwd : allow motion in backward dir (stop if set to 0) 
    * axis<id>.mon.ilockfwd : allow motion in forward dir (stop if set to 0)

* SetAppMode(1) compiles all PLC:s. If new PLC code is added at later stage then it needs to be compiled by:
    * Cfg.CompilePLC(<plcIndex>)
    * Cfg.CompileAxisPLC(<axisIndex>)

## Migration  guide (5.x.x to 6.0.0)

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
    * Command transform object have been replaced with a PLC for each axis. Therefore all commands and syntax related to synchronization have been changed. See above for more info.
    * TODO: Convert sync expressions to new standard. See examples https://bitbucket.org/europeanspallationsource/ecmctraining/src/master/startup/ecmcProject_MCU1021_exampleMasterSlave/axis_2_sync

* ethercat hardware snippets:
    * Have been renamed to allow easier use of macros. All snippest follow the standard ecmc<slave name>. example ecmcEL2004    
    (before ecmcEL2004-digitalOutput)
    * Each slave have a dedicated substitution file and some a dedicated template file.
    * General slave database entries are "loaded" via substitution file
    * Added "Cfg.EcSlaveVerify(0,${ECMC_EC_SLAVE_NUM},${ECMC_EC_VENDOR_ID},${ECMC_EC_PRODUCT_ID})" to ensure that correct slave is on correct bus position. If wrong type of slave an error will be returned and ECMC will die (since ecmcConfigOrDie is used).
    * ecmcAsynPortDriverAddParameter iocsh command have been removed from all "ecmc<slave name>-records" files
      Asyn links are instead automatically linked through asynportdriver::drvusercreate and addressed through the drvInfo string.
      See above in this doc for definition/syntax of drvInfo string in. Use "ecmcReport 3" or "asynReport 3" to see accessible parameters (see also ecmctraining 6.0.0). As long as you use the ecmctraining v6.0.0  snippets and database files this will not be an issue.
    * TODO: Use new hardware snippets.

* Startup files:
    * Sample time of records are defined by ECMC_SAMPLE_RATE_MS (now in milli seconds).
    * TODO: Change ECMC_ASYN_SKIP_CYCLES to ECMC_SAMPLE_RATE_MS.

* OPI: Have been updated to fit new records.
    * TODO: Nothing

* PLC:
    * Remove Cfg.CompilePLC(<plcindex>). The compile is done automatically during validation at transition to runtime (SetAppMode()).
    * Allow load of new expression while "old" compiled are executing at runtime. Switch to new expression with Cfg.CompilePLC() or Cfg.CompileAxisPLC().

# Todo
* Add PID controller in PLC lib
* Consider change error handling system to exception based (try catch)
* Clean up in axis sub objects:
    * emcmMonitor:  Add verify setpoint function (mostly rename)
    * ecmcTrajectory: Make more independent of other code
* Implementation of robot and hexapod kinematics? How to do best possible?
* Make possible to add ethercat hardware on the fly (hard, seems EPICS do not support dynamic load of records)
* Only stop motion when the slaves used by axis are in error state or not reachable.
* Test EtherCAT redundancy
* Add possibility to link axis functionality to plcs (right now for instance a limit needs to be an EtherCAT entry).
