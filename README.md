# ecmc Motion Control Module for EPICS

ecmc is a motion control and data acquisition module for EPICS-based systems, with support for motion axes, EtherCAT I/O, real-time control logic, and high-rate data acquisition.

## Quick Start

1. Clone the repository.
2. Initialize submodules.
3. Use the `ecmccfg` repository to generate/configure a working IOC setup.

```bash
git submodule update --init --reference ./
```

Configuration scripts and examples are available in [`ecmccfg`](https://github.com/paulscherrerinstitute/ecmccfg).

## Features

### Motion Control

- Motor record support
  - Absolute positioning
  - Relative positioning
  - Constant velocity
  - Evaluation of limit and reference switches
  - Homing (several sequences)
    - Low limit (`refId = 1`)
    - High limit (`refId = 2`)
    - Reference sensor via low limit (`refId = 3`)
    - Reference sensor via high limit (`refId = 4`)
    - Reference sensor center via low limit (`refId = 5`)
    - Reference sensor center via high limit (`refId = 6`)
    - Encoder index via low limit (`refId = 11`)
    - Encoder index via high limit (`refId = 12`)
    - Set position dedicated for auto restore (`refId = 15`)
    - Single-turn absolute encoder (resolver) reference via low limit (`refId = 21`)
    - Single-turn absolute encoder (resolver) reference via high limit (`refId = 22`)
    - Set position (`refId = 25`)

- Interlocks
  - Following error
  - Max speed
  - EtherCAT I/O
  - EtherCAT bus health

- Trajectories
  - Trapezoidal
  - Jerk-limited (Ruckig)
  - Custom
    - Calculated in ecmc PLC
    - Buffered positions from array (data storage)

- Advanced features
  - Virtual and normal axes
  - Modulo movements
  - Synchronization (to axis or any I/O) handled through expressions
    - Slaving: `ax2.traj.setpos:=ax1.enc.actpos;`
    - Sync: `ax2.traj.setpos:=ax1.traj.setpos;`
    - Phasing: `ax2.traj.setpos:=ax1.traj.setpos+100;`
    - Phasing with third axis: `ax2.traj.setpos:=ax1.traj.setpos+ax3.traj.setpos;`
    - Advanced: `ax2.traj.setpos:=ax1.traj.setpos*sin(ax3.traj.setpos/100)+ec0.s2.VALUE;`
    - Enable amplifiers: `ax2.drv.enable:=ax1.drv.enable;`
    - Interlock expression: `ax2.mon.ilockfwd:=ec0.s1.INPUT_1 or ax1.enc.actpos < 100;`

### Data Acquisition

- Support for many different analog and digital slaves
- Up to 100 kHz analog and 1 MHz digital (oversampling slaves)

### Control / PLC

- PLC objects can be created with custom sample rates where logic (including motion) can be handled in real-time
- Same expression syntax can be used for control and synchronization, for example:
  - `ec0.s45.VALUE:=static.gain*77+ax2.traj.setpos/10;`
- PLC variables can be accessed from EPICS
- More info: [ecmc PLC configuration manual](https://paulscherrerinstitute.github.io/ecmccfg/manual/plc_cfg/)

### Hardware Support

- Drives: stepper, servo, pulse-direction, and analog
- I/O: several analog and digital terminals
- Other: bus couplers, system terminals
- Current hardware list: <https://github.com/paulscherrerinstitute/ecmccfg/tree/master/hardware>

## Setup and Configuration

The fastest way to get started is to use the configuration scripts in the [`ecmccfg` repository](https://github.com/paulscherrerinstitute/ecmccfg).

ecmc is configured via EPICS IOC shell (`iocsh`).

### Common `iocsh` Commands

- Create an ecmc application
  - `ecmcAsynPortDriverConfigure <portName>,<paramTableSize>,<prio>,<disableAutoConnect>,<defaultSampleRateMS>`
- Configure ecmc application and exit ecmc on error
  - `ecmcConfigOrDie <ecmc ASCII command>`
- Configure ecmc application (non-fatal on error)
  - `ecmcConfig <ecmc ASCII command>`
- Print available asyn parameters and other info
  - `ecmcReport <detail>`
  - `asynReport <detail>` (same as `ecmcReport` but also calls `asynReport` in other modules)

### ecmc ASCII Command Examples

See the doxygen documentation for full details.

- Motion configuration
  - `ecmcConfigOrDie "Cfg.CreateAxis(<axisid>,1,<type>)"`
  - `ecmcConfigOrDie "Cfg.SetAxisCntrlKp(<axisid>,<kp>)"`
- EtherCAT bus configuration
  - `ecmcConfigOrDie "Cfg.EcAddEntryComplete(<slaveid>,<vendorid>,<productid>,<dir>,<smid>,<pdoid>,<entryid>,<entrysubid>,<bits>,<name>)"`
- EtherCAT slave configuration (SDO/SoE), example: set max motor current for EL7037 to 1500 mA
  - `ecmcConfigOrDie "Cfg.EcAddSdo(<slaveid>,0x8010,0x1,1500,2)"`
- PLC configuration
  - `ecmcConfigOrDie "Cfg.CreatePLC(<plcid>,<sample rate ms>)"`
  - `ecmcConfigOrDie "Cfg.LoadPLCFile(<plcid>,<filename>)"`
  - `ecmcConfigOrDie "Cfg.AppendPLCExpr(<plcid>)=ax1.enc.homepos:=25#"`

## Documentation and Examples

- [Manual](https://paulscherrerinstitute.github.io/ecmccfg/manual/)
- [Best-practice examples](https://github.com/paulscherrerinstitute/ecmccfg/tree/master/examples/PSI/best_practice)

## Environment

ecmc runs best under these conditions:

- PSI environment (or ESS `e3` environment)
- Real-time kernel patch (RT patch)
- Configuration via [`ecmccfg`](https://github.com/paulscherrerinstitute/ecmccfg)

## Required EPICS Modules / Dependencies

- Open source EtherCAT master: <https://etherlab.org/>
- ExprTK: <https://github.com/paulscherrerinstitute/exprtk-ecmc>
- Motor: <https://github.com/EuropeanSpallationSource/motor>
- Asyn driver: <https://github.com/epics-modules/asyn>
- `ecmccfg`: <https://github.com/paulscherrerinstitute/ecmccfg>
- Ruckig: <https://github.com/anderssandstrom/ruckig>

## Optional Modules

- `ecmccomp`: <https://github.com/paulscherrerinstitute/ecmccomp>

## Plugins

Plugins with extra functionality that can be loaded during startup:

- CNC G-code support by grbl: <https://github.com/anderssandstrom/ecmc_plugin_grbl>
- SocketCAN support (including reduced CANopen support): <https://github.com/anderssandstrom/e3-ecmc_plugin_socketcan>
- FFT support via kissfft: <https://github.com/paulscherrerinstitute/ecmc_plugin_fft>
- Raspberry Pi wiringPi support: <https://github.com/anderssandstrom/e3-ecmc_plugin_raspi>
- PVA support from `ecmc-plc`: <https://github.com/anderssandstrom/e3-ecmc_plugin_pva>
- Simple scope for EtherCAT DC slaves: <https://github.com/anderssandstrom/e3-ecmc_plugin_scope>
- DAQ plugin: <https://github.com/paulscherrerinstitute/ecmc_plugin_daq>

## Issues / Bug Reports

- <https://github.com/epics-modules/ecmc/issues>

## Support

- ecmc: Anders Sandstrom (`anders.sandstroem@psi.ch`)
- Motor Record: Torsten Bogershausen (`torsten.bogershausen@esss.se`)
- `ecmcMotorRecord` model 3 driver (preferred): Anders Sandstrom (`anders.sandstroem@psi.ch`)
