# ecmc

`ecmc` is an EPICS-based motion control and data acquisition module for EtherCAT systems.
It combines axis control, realtime PLC logic, data acquisition, and runtime extensions in one IOC-oriented package.

This repository contains the core `ecmc` module, example IOC files, and the EPICS build integration needed to build it into an IOC application.

## What ecmc provides

- Motion control for stepper, servo, pulse-direction, and analog drives
- EtherCAT integration based on the open Etherlab master
- Realtime PLC tasks with expression-based logic and access to motion and I/O data
- Built-in trajectory generators, including trapezoidal and jerk-limited motion
- Custom motion through PLC expressions or buffered position arrays
- EPICS motor record integration
- Runtime plugin support for optional functionality such as CNC, DAQ, FFT, CAN, and safety-related extensions

## Main capabilities

### Motion control

- Absolute positioning
- Relative positioning
- Constant-velocity motion
- Homing with multiple reference sequences
- Virtual and physical axes
- Modulo motion
- Synchronization and phasing through expressions
- Motion interlocks, including following error, speed limits, EtherCAT I/O, and bus health

### Trajectories

- Trapezoidal trajectory generation
- Jerk-limited trajectory generation based on `ruckig`
- Custom trajectories calculated in `ecmc` PLC logic
- Buffered trajectories from data storage objects

### Realtime control and acquisition

- PLC objects running at configurable sample rates
- Access to axis, PLC, and EtherCAT data from EPICS
- High-rate acquisition support for suitable EtherCAT terminals
- Integration of custom functions and constants through plugins

## Repository layout

- `devEcmcSup/`: core `ecmc` library sources
- `ecmcExampleTop/`: example IOC application and boot files
- `configure/`: EPICS build configuration
- `documentation/`: Doxygen configuration and generated-documentation inputs
- `tools/`: helper scripts and utilities

## Requirements

`ecmc` is intended for EPICS environments with EtherCAT and realtime support.

### Runtime environment

- EPICS Base
- Linux system with realtime support recommended
- Etherlab EtherCAT master
- IOC shell-based configuration workflow

### Build dependencies

- `EPICS_BASE`
- `asyn`
- `motor`
- `exprtkSupport`
- Etherlab user library
- `ruckig`

### Configuration tooling

- `ecmccfg`

The easiest way to configure a working system is through the scripts and examples in:

https://github.com/paulscherrerinstitute/ecmccfg

### Optional modules

- `ecmccomp`

## Build

Initialize submodules first:

```sh
git submodule update --init --reference ./
```

Set external module paths in [configure/RELEASE](configure/RELEASE) or a local override file such as `configure/RELEASE.local`.
The sample files in this repository show the expected EPICS module layout.

Build the module from the repository root:

```sh
make
```

This builds the support library, example IOC application, and boot files through the standard EPICS build system.

## Quick start

For a real system, start with `ecmccfg` examples rather than building a full configuration manually.

Typical workflow:

1. Build `ecmc` and verify that your external module paths are correct.
2. Use `ecmccfg` to generate or reuse startup scripts for your hardware.
3. Create an IOC and configure an `ecmc` application with `ecmcAsynPortDriverConfigure`.
4. Load EtherCAT, axis, and PLC configuration through `ecmcConfigOrDie` or `ecmcConfig`.
5. Start the IOC and verify axis and EtherCAT status from EPICS.

Example IOC files are included under [ecmcExampleTop/](ecmcExampleTop/).

## Configuration model

`ecmc` is configured primarily through EPICS IOC shell commands and `ecmc` ASCII commands.

### IOC shell entry points

- `ecmcAsynPortDriverConfigure(<portName>,<paramTableSize>,<prio>,<disableAutoConnect>,<defaultSampleRateMS>)`
- `ecmcConfigOrDie "<ecmc ASCII command>"`
- `ecmcConfig "<ecmc ASCII command>"`
- `ecmcReport <detail>`
- `asynReport <detail>`

### Common configuration areas

- Axis creation and controller setup
- EtherCAT bus, PDO, and SDO configuration
- PLC creation and PLC file loading
- Plugin loading

Examples:

```iocsh
ecmcConfigOrDie "Cfg.CreateAxis(<axisid>,1,<type>)"
ecmcConfigOrDie "Cfg.EcAddEntryComplete(<slaveid>,<vendorid>,<productid>,<dir>,<smid>,<pdoid>,<entryid>,<entrysubid>,<bits>,<name>)"
ecmcConfigOrDie "Cfg.CreatePLC(<plcid>,<sample rate ms>)"
ecmcConfigOrDie "Cfg.LoadPLCFile(<plcid>,<filename>)"
ecmcConfigOrDie "Cfg.AppendPLCExpr(<plcid>)=ax1.enc.homepos:=25#"
```

## Plugins and extensions

Optional plugins can be loaded at startup to extend `ecmc` with extra functionality.

Current plugin examples:

- CNC G-code support by grbl: https://github.com/anderssandstrom/ecmc_plugin_grbl
- SocketCAN support, including reduced CANopen support: https://github.com/anderssandstrom/e3-ecmc_plugin_socketcan
- FFT support by kissfft: https://github.com/paulscherrerinstitute/ecmc_plugin_fft
- Raspberry Pi wiringPi support: https://github.com/anderssandstrom/e3-ecmc_plugin_raspi
- PVA support from `ecmc-plc`: https://github.com/anderssandstrom/e3-ecmc_plugin_pva
- Scope plugin for EtherCAT DC slaves: https://github.com/anderssandstrom/e3-ecmc_plugin_scope
- DAQ plugin: https://github.com/paulscherrerinstitute/ecmc_plugin_daq

## Documentation and examples

- Manual: https://paulscherrerinstitute.github.io/ecmccfg/manual/
- Configuration examples: https://github.com/paulscherrerinstitute/ecmccfg/tree/master/examples/PSI/best_practice
- Hardware definitions: https://github.com/paulscherrerinstitute/ecmccfg/tree/master/hardware

## Environment notes

`ecmc` works best in environments that already follow the PSI or ESS `e3` style of EPICS deployment and use a realtime Linux setup.
The exact external module paths and EtherCAT user-library paths depend on your site packaging and target platform.

## Support

- Issues: https://github.com/epics-modules/ecmc/issues
- `ecmc`: Anders Sandstrom, anders.sandstroem@psi.ch
- Motor record: Torsten Bogershausen, torsten.bogershausen@esss.se
- `ecmcMotorRecord` model 3 driver: Anders Sandstrom, anders.sandstroem@psi.ch
