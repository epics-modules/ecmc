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
- `devEcmcSup/logic/`: additive `cpp_logic` interface, helper headers, and examples
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
ecmcConfigOrDie "Cfg.EcAddEntryAlias(<slaveid>,<name>,<alias>)"
ecmcConfigOrDie "Cfg.EcAddEntryAlias(ec0.s<slaveid>.<name>,<alias>)"
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

`ecmc` also now contains an additive `cpp_logic` C/C++ interface under
[devEcmcSup/logic/](devEcmcSup/logic/) that is separate from the original
plugin ABI and intended for small cyclic logic modules.

The `cpp_logic` area also includes helper headers for common patterns:

- [`ecmcCppLogic.hpp`](devEcmcSup/logic/ecmcCppLogic.hpp): bindings, exports, host services, and loader glue
- [`ecmcCppMotion.hpp`](devEcmcSup/logic/ecmcCppMotion.hpp): `MC_*` style C++ wrappers on top of the existing `ecmc` motion API
- [`ecmcCppControl.hpp`](devEcmcSup/logic/ecmcCppControl.hpp): control helpers such as `ecmcCpp::Pid`
- [`ecmcCppUtils.hpp`](devEcmcSup/logic/ecmcCppUtils.hpp): utility helpers such as edge triggers, `TON`/`TOF`/`TP` style timers, latches, blink/state helpers, filtering, moving averages, min/max hold, and EtherCAT status wrappers
- [`ecmcCppTrace.hpp`](devEcmcSup/logic/ecmcCppTrace.hpp): reusable fixed-capacity triggered trace helper for waveform-style exports
- [`ecmcCppPersist.hpp`](devEcmcSup/logic/ecmcCppPersist.hpp): retained-value helper for infrequent load/save of trivially copyable values
- [`CPP_LOGIC_HELPERS.md`](devEcmcSup/logic/CPP_LOGIC_HELPERS.md): compact helper reference for the C++ logic headers
- [`devEcmcSup/logic/examples/cpp_logic_starter`](devEcmcSup/logic/examples/cpp_logic_starter): smallest practical starter example for a new `cpp_logic` shared library

`cpp_logic` logic classes derive from `ecmcCpp::LogicBase` and may override:

- `enterRealtime()` for one-time startup actions when RT begins
- `run()` for cyclic logic
- `exitRealtime()` for one-time cleanup when RT stops

That is the preferred place for non-cyclic work such as retained-value startup
restore, instead of doing file I/O from the cyclic `run()` path.

`cpp_logic` modules can be loaded directly in `ecmc` with:

```iocsh
ecmcConfigOrDie "Cfg.LoadCppLogic(0,/path/to/cpp_logic.so)"
ecmcConfigOrDie "Cfg.LoadCppLogic(0,/path/to/cpp_logic.so,asyn_port=CPP.LOGIC0;sample_rate_ms=2;update_rate_ms=20)"
```

For normal IOC usage, the recommended entry point is the companion IOC shell
wrapper in `ecmccfg`:

```iocsh
iocshLoad("$(ecmccfg_DIR)loadCppLogic.cmd",
          "LOGIC_ID=0,FILE=/path/to/cpp_logic.so,ASYN_PORT=CPP.LOGIC0")
```

That wrapper loads the built-in `cpp_logic` control/status PVs by default.
Custom `epics.*` substitutions can be enabled separately with
`LOAD_APP_PVS=1,EPICS_SUBST=...`.

Each loaded `cpp_logic` instance gets its own dedicated asyn port. Both the
built-in runtime PVs and all user-defined `epics.*` exports are published on
that port.

`cpp_logic` also supports an optional free-form `MACROS` text string from the
IOC startup path. That text is available to user code through
`ecmcCpp::getMacrosString()` and is intended for small logic-specific
configuration parsed by the constructor or other one-time lifecycle code.

That script is intended to load the built-in core substitutions from:

- `../ecmccfg/db/generic/ecmcCppLogicCore.substitutions`

For `cpp_logic` EPICS exports there is also an offline substitutions generator:

- [`tools/ecmcCppLogicSubstGen.py`](tools/ecmcCppLogicSubstGen.py)

It can inspect a compiled C++ logic shared library through
`ecmc_cpp_logic_get_api()` and generate substitutions for the variables
declared through the `cpp_logic` `epics` export builder. Load the generic core
substitutions and the generated custom substitutions on the same `cpp_logic`
asyn port.

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
