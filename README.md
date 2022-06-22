ECMC Motion Control Module for EPICS 
==

ECMC is a motion control and data acquisition module for use with EPICS system.


# Initialize

```
git submodule update --init --reference ./
```

# Features

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
            * Set position dedicated for auto restore (refId = 15)
            * Single turn abs encoder (resolver) ref via low limit (refId = 21)
            * Single turn abs encoder (resolver) ref via high limit (refId = 22)
            * Set position (refId = 25)
        * Interlocks
            * Following error
            * Max speed
            * Ethercat I/O
            * Ethercat bus health
    * Trajectories:
        * Trapetzoidal
        * Jerk limited (ruckig, see below)
        * Custom:
            * Calculated in ecmc plc
            * Buffered positions from array (data storage)
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
        (see examples in ecmccfg repository: https://gitlab.esss.lu.se/epics-modules/ecmccfg/-/tree/master/examples/test
        * Same syntax as for synchronization:
            * "ec0.s45.VALUE:=static.gain*77+ax2.traj.setpos/10;"                        
        * PLC variables can be accessed from EPICS.        
    * Hardware support: 
        * Drives: stepper, servo, pulse direction and analog
        * I/O: Several analog and digital terminals
        * Other: bus-couplers, system terminals
        * Current list: https://gitlab.esss.lu.se/epics-modules/ecmccfg/-/tree/master/hardware

# How do I get set up?

Fastest and easiest way to get started is by using the configuration scripts accessible in
ecmccfg repository: https://github.com/paulscherrerinstitute/ecmccfg (or ESS fork https://github.com/icshwi/ecmccfg)

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

# Environment
ECMC runs best under certain conditions:

* E3 (ESS EPICS Environment)
* RT-patch
* Settings found in https://github.com/icshwi/realtime-config
* configuration by ecmccfg:
  * https://github.com/paulscherrerinstitute/ecmccfg
  * https://github.com/icshwi/ecmccfg (ESS fork)


# Needed EPICS modules

* Open source ethercat master:      https://etherlab.org/, https://github.com/icshwi/etherlabmaster 
* ExprTK:                           https://github.com/icshwi/e3-exprtk (https://github.com/ArashPartow/exprtk)
* E3 (ESS EPICS Environment):       https://github.com/icshwi/e3
* Real-time configuration:          https://github.com/icshwi/realtime-config
* Motor:                            https://github.com/icshwi/e3-motor (https://github.com/EuropeanSpallationSource/motor)
* Asyn driver:                      https://github.com/icshwi/e3-asyn
* ecmccfg                           https://github.com/paulscherrerinstitute/ecmccfg or
                                    https://gitlab.esss.lu.se/epics-modules/ecmccfg
* ruckig                            https://github.com/anderssandstrom/e3-ruckig, https://github.com/anderssandstrom/ruckig
Optional modules:
* EthercatMC (motor record driver): https://github.com/icshwi/e3-ethercatmc

# Plugins
Plugins with extra functionality that can be loaded during startup:
* CNC g-code support by grbl:       https://github.com/anderssandstrom/ecmc_plugin_grbl
* SocketCAN support (inc. reduced CANOpen support): https://github.com/anderssandstrom/e3-ecmc_plugin_socketcan
* FFTs by kissfft:                  https://github.com/anderssandstrom/e3-ecmc_plugin_fft
* Raspi wiringPi support:            https://github.com/anderssandstrom/e3-ecmc_plugin_raspi
* PVA support from ecmc-plc:        https://github.com/anderssandstrom/e3-ecmc_plugin_pva
* Simple scope for ethercat dc slaves):   https://github.com/anderssandstrom/e3-ecmc_plugin_scope

# Issues/bug report

https://github.com/epics-modules/ecmc/issues

# Support

* ECMC                                 : Anders Sandström, anders.sandstrom@esss.se
* Motor Record                         : Torsten Bögershausen, torsten.bogershausen@esss.se
  * ecmcMotorRecord driver (preferred) : Anders Sandström, anders.sandstrom@esss.se
  * (EthercatMC driver                 : Torsten Bögershausen, torsten.bogershausen@esss.se)
